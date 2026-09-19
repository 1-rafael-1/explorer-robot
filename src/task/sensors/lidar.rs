//! Real COIN-D6 `LiDAR` task — on-demand power, warm-up, and streaming.
//!
//! Replaces the synthetic `lidar_stub` task. A single long-lived embassy task on
//! **core1** owns the [`CoinD6`] driver for the whole runtime — the UART and the
//! power GPIO are static, core-owned peripherals, so the driver cannot be rebuilt
//! per mode — and is driven by a command channel.
//!
//! # Lifecycle
//!
//! The sensor is off at boot. [`acquire`] powers it on, sends the vendor start
//! command (a failure is logged and tolerated, because the device may already be
//! streaming), and waits for the rotor to settle. The driver's only watchdogs
//! count bytes, so the warm-up wait is bounded by a wall clock here: on timeout
//! the task power-cycles and retries a bounded number of times before reporting
//! [`AcquireError::Failed`]. [`release`] stops the device, drops its power, and
//! clears the stale state — cloud, obstacle flag and cleared edge — in the same
//! step. Powering the device is what the on-robot check watches: the gate goes
//! high only while acquired.
//!
//! # Streaming
//!
//! One raw revolution is read per iteration ([`CoinD6::read_scan`]) — no
//! multi-spin aggregation on the live path, because avoidance latency matters
//! more than smoothing. Each revolution becomes a [`Cloud`] through the
//! `lidar_cloud` crate, the Front Sector test drives the lock-free obstacle flag,
//! and a change raises [`Events::ObstacleDetected`]. A read error restarts the
//! device; a persistently dead sensor ends in [`LidarStatus::Failed`] rather than
//! silent spin.

// The `acquire`/`release`/`status`/`is_acquired` API is consumed by the Room
// Scan and Coast-and-Avoid tasks in later tickets and is unused until then.
#![allow(dead_code)]

use core::sync::atomic::{AtomicU8, Ordering};

use coin_d6::{CoinD6, Config, Scan, WarmupConfig};
use defmt::{Debug2Format, info, warn};
use embassy_rp::{gpio::Output, uart::BufferedUart};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer, with_timeout};
use lidar_cloud::Cloud;
use static_cell::StaticCell;

use crate::system::{
    event::{Events, ObstacleSource, raise_event},
    state::perception::{self, ChangeDetected},
};

// ── Public types ──────────────────────────────────────────────────────────────

/// Lifecycle state of the owned `LiDAR` driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum LidarStatus {
    /// Powered down; the sensor is not in use.
    Off,
    /// Powered on, sending start, and waiting for the rotor to settle.
    Warming,
    /// Warmed up and publishing scans into perception.
    Streaming,
    /// A lifecycle attempt exhausted its retries; the sensor is powered down.
    Failed,
}

/// Why an [`acquire`] request could not be honoured.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AcquireError {
    /// The sensor is already acquired (warming or streaming).
    Busy,
    /// The lifecycle exhausted its retries; the sensor stays powered down.
    Failed,
}

// ── Tuning constants ──────────────────────────────────────────────────────────

/// Driver ingest chunk size; the driver requires at least 1 KiB.
const INGEST_LEN: usize = 1024;
/// Wall-clock bound on one warm-up attempt.
///
/// The driver has no wall clock — its watchdogs count bytes — so the call site
/// supplies the hard bound. On expiry the device is power-cycled.
const WARMUP_TIMEOUT: Duration = Duration::from_secs(5);
/// How long the power rail is held low between power-cycle attempts.
const POWER_CYCLE_OFF: Duration = Duration::from_millis(200);
/// Bounded number of power-cycle attempts per acquire.
const ACQUIRE_ATTEMPTS: u8 = 3;

// ── Status cell (lock-free) ───────────────────────────────────────────────────

/// Encoded value of [`LidarStatus::Off`] in [`LIDAR_STATUS`].
const STATUS_OFF: u8 = 0;
/// Encoded value of [`LidarStatus::Warming`] in [`LIDAR_STATUS`].
const STATUS_WARMING: u8 = 1;
/// Encoded value of [`LidarStatus::Streaming`] in [`LIDAR_STATUS`].
const STATUS_STREAMING: u8 = 2;
/// Encoded value of [`LidarStatus::Failed`] in [`LIDAR_STATUS`].
const STATUS_FAILED: u8 = 3;

/// Current lifecycle state, read lock-free from any context.
static LIDAR_STATUS: AtomicU8 = AtomicU8::new(STATUS_OFF);

// ── Command channel & replies ─────────────────────────────────────────────────

/// Commands accepted by the `LiDAR` task.
#[derive(Debug, Clone, Copy)]
enum LidarCommand {
    /// Run the power-on/warm-up lifecycle.
    Acquire,
    /// Stop, power off, and clear stale state.
    Release,
}

/// Command channel to the `LiDAR` task (capacity 4).
static COMMAND: Channel<CriticalSectionRawMutex, LidarCommand, 4> = Channel::new();

/// One reply per [`LidarCommand::Acquire`]; a channel rather than a signal so
/// concurrent callers each receive one reply without losing a wake-up.
static ACQUIRE_REPLY: Channel<CriticalSectionRawMutex, Result<(), AcquireError>, 4> = Channel::new();

/// One acknowledgement per [`LidarCommand::Release`].
static RELEASE_REPLY: Channel<CriticalSectionRawMutex, (), 4> = Channel::new();

/// Caller-allocated ingest chunk for the driver (at least 1 KiB).
static INGEST: StaticCell<[u8; INGEST_LEN]> = StaticCell::new();

// ── Driver type ───────────────────────────────────────────────────────────────

/// The concrete driver the task owns: buffered UART0 plus the GPIO26 power gate.
type LidarDriver = CoinD6<'static, BufferedUart, Output<'static>>;

// ── Public API ────────────────────────────────────────────────────────────────

/// Power on the sensor and wait until it is warmed and streaming.
///
/// Sends the start command (tolerating a write failure, since the device may
/// already be streaming) and waits for the rotor to settle behind a wall-clock
/// bound. On timeout the task power-cycles and retries a bounded number of
/// times, then reports [`AcquireError::Failed`] and leaves the status
/// `Failed`. Calling this while already acquired returns [`AcquireError::Busy`]
/// without starting a second lifecycle.
///
/// # Errors
///
/// [`AcquireError::Busy`] if already acquired, [`AcquireError::Failed`] if the
/// lifecycle exhausted its retries.
pub async fn acquire() -> Result<(), AcquireError> {
    COMMAND.send(LidarCommand::Acquire).await;
    ACQUIRE_REPLY.receive().await
}

/// Stop the device, drop its power, and clear all stale state.
///
/// Returns once the task has completed the release, so a caller can rely on the
/// cloud being absent and the obstacle flag cleared. The stop command is
/// best-effort; the power gate and the state clear always run.
pub async fn release() {
    COMMAND.send(LidarCommand::Release).await;
    RELEASE_REPLY.receive().await;
}

/// Current lifecycle state. Lock-free (an `AtomicU8`), safe from any context.
#[must_use]
pub fn status() -> LidarStatus {
    match LIDAR_STATUS.load(Ordering::Relaxed) {
        STATUS_WARMING => LidarStatus::Warming,
        STATUS_STREAMING => LidarStatus::Streaming,
        STATUS_FAILED => LidarStatus::Failed,
        _ => LidarStatus::Off,
    }
}

/// Whether the sensor is currently owned (warming or streaming).
#[must_use]
pub fn is_acquired() -> bool {
    matches!(status(), LidarStatus::Warming | LidarStatus::Streaming)
}

// ── Embassy task ──────────────────────────────────────────────────────────────

/// `LiDAR` driver embassy task.
///
/// Idles (sensor off) until an acquire command arrives, runs the lifecycle, then
/// streams revolutions until a release command arrives. Runs on **core1** so
/// `UART0_IRQ` is enabled on core1's NVIC.
#[embassy_executor::task]
#[allow(clippy::large_futures)]
pub async fn lidar_task(uart: BufferedUart, power: Output<'static>) {
    let ingest = INGEST.init([0u8; INGEST_LEN]);
    let mut driver = LidarDriver::new(uart, power, ingest, Config::default());
    let mut scratch = Scan::new();
    let mut sequence: u64 = 0;
    let mut last_obstacle: Option<bool> = None;

    info!("[lidar] task started on core1 (sensor off)");

    loop {
        // Off (or failed): block until a command arrives.
        match COMMAND.receive().await {
            LidarCommand::Acquire => {}
            LidarCommand::Release => {
                // Already off — nothing to stop or clear.
                RELEASE_REPLY.send(()).await;
                continue;
            }
        }

        info!("[lidar] acquire requested");
        if run_acquire(&mut driver, &mut scratch).await {
            set_status(LidarStatus::Streaming);
            ACQUIRE_REPLY.send(Ok(())).await;
            info!("[lidar] streaming");
            // Seed the edge tracker with the current flag so the first scan
            // only raises an event on a real change.
            last_obstacle = Some(perception::is_obstacle_detected());
            stream_scans(&mut driver, &mut scratch, &mut sequence, &mut last_obstacle).await;
            info!("[lidar] streaming stopped");
        } else {
            let _ = driver.power_off();
            set_status(LidarStatus::Failed);
            clear_stale_state(&mut last_obstacle).await;
            ACQUIRE_REPLY.send(Err(AcquireError::Failed)).await;
            warn!("[lidar] acquire failed");
        }
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

/// Store a new lifecycle state in the lock-free status cell.
fn set_status(status: LidarStatus) {
    let encoded = match status {
        LidarStatus::Off => STATUS_OFF,
        LidarStatus::Warming => STATUS_WARMING,
        LidarStatus::Streaming => STATUS_STREAMING,
        LidarStatus::Failed => STATUS_FAILED,
    };
    LIDAR_STATUS.store(encoded, Ordering::Relaxed);
}

/// Run the acquire lifecycle with bounded retries, returning success.
///
/// Sets the status to [`LidarStatus::Warming`] for the whole sequence. Each
/// failed attempt is followed by a power-cycle; the caller sets the final
/// `Streaming`/`Failed` status.
async fn run_acquire(driver: &mut LidarDriver, scratch: &mut Scan) -> bool {
    set_status(LidarStatus::Warming);
    for attempt in 1..=ACQUIRE_ATTEMPTS {
        match with_timeout(WARMUP_TIMEOUT, attempt_acquire(driver, scratch)).await {
            Ok(true) => return true,
            Ok(false) => warn!("[lidar] acquire attempt {} failed", attempt),
            Err(_) => warn!("[lidar] acquire attempt {} timed out", attempt),
        }
        // Power-cycle so the retry starts from a known-off device.
        let _ = driver.power_off();
        Timer::after(POWER_CYCLE_OFF).await;
    }
    false
}

/// One acquire attempt: power on, send start (tolerated), and warm up.
async fn attempt_acquire(driver: &mut LidarDriver, scratch: &mut Scan) -> bool {
    if let Err(error) = driver.power_on().await {
        warn!("[lidar] power-on failed: {:?}", Debug2Format(&error));
        return false;
    }
    // The device may already be streaming, so a start-write failure is not fatal
    // — warm-up is the real proof that data flows.
    if let Err(error) = driver.start().await {
        warn!("[lidar] start command failed (tolerated): {:?}", Debug2Format(&error));
    }
    match driver.warm_up(scratch, &WarmupConfig::default()).await {
        Ok(outcome) => {
            info!("[lidar] warm-up: {:?}", Debug2Format(&outcome));
            true
        }
        Err(error) => {
            warn!("[lidar] warm-up failed: {:?}", Debug2Format(&error));
            false
        }
    }
}

/// Stream revolutions until a release command arrives or the device dies.
///
/// Commands are polled between revolutions; an acquire seen while streaming gets
/// a `Busy` reply, and a release tears the device down and returns. A read error
/// power-cycles and re-runs the acquire lifecycle once; if that fails the status
/// becomes `Failed` and the task returns to idle.
async fn stream_scans(
    driver: &mut LidarDriver,
    scratch: &mut Scan,
    sequence: &mut u64,
    last_obstacle: &mut Option<bool>,
) {
    loop {
        while let Ok(command) = COMMAND.try_receive() {
            match command {
                LidarCommand::Acquire => {
                    ACQUIRE_REPLY.send(Err(AcquireError::Busy)).await;
                }
                LidarCommand::Release => {
                    stop_and_power_off(driver).await;
                    clear_stale_state(last_obstacle).await;
                    set_status(LidarStatus::Off);
                    RELEASE_REPLY.send(()).await;
                    return;
                }
            }
        }

        match driver.read_scan(scratch).await {
            Ok(()) => {
                *sequence = sequence.wrapping_add(1);
                publish_scan(scratch, *sequence, last_obstacle).await;
            }
            Err(error) => {
                // `read_scan` recovers UART glitches and resyncs in place; a
                // returned error means no revolution assembled within the byte
                // watchdog, so restart the device from a clean power cycle.
                warn!(
                    "[lidar] scan read failed, restarting device: {:?}",
                    Debug2Format(&error)
                );
                let _ = driver.power_off();
                Timer::after(POWER_CYCLE_OFF).await;
                if run_acquire(driver, scratch).await {
                    set_status(LidarStatus::Streaming);
                    info!("[lidar] device restarted");
                } else {
                    let _ = driver.power_off();
                    set_status(LidarStatus::Failed);
                    clear_stale_state(last_obstacle).await;
                    warn!("[lidar] device failed permanently");
                    return;
                }
            }
        }
    }
}

/// Map one revolution into a cloud, publish it, and drive the obstacle edge.
async fn publish_scan(scan: &Scan, sequence: u64, last_obstacle: &mut Option<bool>) {
    let cloud = Cloud::from_spin(scan, sequence);
    let detected = cloud.front_sector_obstacle();

    perception::update_lidar_points(cloud).await;

    if *last_obstacle != Some(detected) {
        perception::set_lidar_obstacle(detected).await;
        raise_event(Events::ObstacleDetected {
            source: ObstacleSource::Lidar,
            detected,
        })
        .await;
        *last_obstacle = Some(detected);
    }

    #[cfg(feature = "telemetry_logs")]
    info!("[lidar] scan {} (obstacle: {})", sequence, detected);
}

/// Send the best-effort stop command, then deassert the power gate.
async fn stop_and_power_off(driver: &mut LidarDriver) {
    if let Err(error) = driver.stop().await {
        warn!("[lidar] stop command failed (best-effort): {:?}", Debug2Format(&error));
    }
    if let Err(error) = driver.power_off() {
        warn!("[lidar] power-off failed: {:?}", Debug2Format(&error));
    }
}

/// Clear the stale cloud and obstacle flag, raising the cleared edge if needed.
///
/// Release and terminal-failure both funnel through here so a powered-down
/// sensor never leaves an obstacle flag or an old cloud behind. The cleared edge
/// is raised only when the flag had actually been set.
async fn clear_stale_state(last_obstacle: &mut Option<bool>) {
    if perception::clear_lidar_state().await == ChangeDetected::ChangedToCleared {
        raise_event(Events::ObstacleDetected {
            source: ObstacleSource::Lidar,
            detected: false,
        })
        .await;
    }
    *last_obstacle = None;
}
