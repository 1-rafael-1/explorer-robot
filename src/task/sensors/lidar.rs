//! Real COIN-D6 `LiDAR` task — on-demand power, warm-up, and streaming.
//!
//! Replaces the synthetic `lidar_stub` task. A single long-lived embassy task on
//! **core1** owns the [`CoinD6`] driver for the whole runtime — the UART and the
//! power GPIO are static, core-owned peripherals, so the driver cannot be rebuilt
//! per mode — and is driven by a one-way command channel.
//!
//! # The model
//!
//! A mode calls [`enable`] when it needs perception and [`disable`] when it is
//! done. The commands are one-way on a dedicated [`Channel`], as motor, flash and
//! encoder commands are: the task owns the power gate, the vendor start command,
//! the warm-up, the retries, streaming and the power-off, and keeps the driver for
//! the runtime. At most one mode needs perception at a time (ADR-0017) — the panel
//! is modal, and each mode slot admits one mode — so the enabled sensor serves
//! every reader.
//!
//! Readiness is observed by polling the lock-free [`status`] — `Off`, `Warming`,
//! `Streaming` or `Failed`; the task never signals. Bring-up is edge-triggered: a
//! duplicate [`enable`] is inert, and a failed attempt holds at `Failed` until a
//! [`disable`] followed by an [`enable`], so a caller cannot spin retries by
//! polling. [`disable`] stops the device, drops the power gate and clears the
//! stale cloud, obstacle flag and cleared edge (ADR-0015) in the same step.
//!
//! # Lifecycle
//!
//! The sensor is off at boot. [`enable`] powers it on, sends the vendor start
//! command (a failure is logged and tolerated, because the device may already be
//! streaming), and waits for the rotor to settle. The driver's only watchdogs
//! count bytes, so the warm-up wait is bounded by a wall clock here: on timeout
//! the task power-cycles and retries a bounded number of times before reporting
//! [`LidarStatus::Failed`]. [`disable`] stops the device, drops the power gate,
//! and clears the stale cloud, obstacle flag and cleared edge in the same step.
//!
//! # Streaming
//!
//! One raw revolution is read per iteration ([`CoinD6::read_scan`]) behind a
//! wall-clock bound ([`STREAM_READ_TIMEOUT`]) — no multi-spin aggregation on the
//! live path, because avoidance latency matters more than smoothing. Each
//! revolution becomes a [`Cloud`] through the `lidar_cloud` crate, the Front
//! Sector test drives the lock-free obstacle flag, and a change raises
//! [`Events::ObstacleDetected`]. The driver's own watchdogs count bytes, not
//! time, so a powered sensor that goes silent would otherwise stall the read
//! forever; expiring the bound power-cycles and brings the device back up,
//! exactly as a read error does. A persistently dead sensor therefore still
//! ends in [`LidarStatus::Failed`] and a queued disable is still honoured,
//! rather than silent spin.

use core::sync::atomic::{AtomicU8, Ordering};

use coin_d6::{AggregationConfig, CoinD6, Config, Scan, WarmupConfig, aggregate};
use defmt::{Debug2Format, info, warn};
use embassy_rp::{gpio::Output, uart::BufferedUart};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer, with_timeout};
use lidar_cloud::{Cloud, SLOT_WIDTH_DEG};
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
    /// A bring-up or restart failed; the sensor is powered down and holds at
    /// `Failed` until a disable then enable.
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
/// Wall-clock bound on one streaming revolution.
///
/// [`CoinD6::read_scan`] is bounded only by the driver's byte-count ring-start
/// watchdog (128 KiB, ≈5.7 s of continuous bytes at the 230 400 baud link), and
/// that watchdog counts bytes actually received: a fully silent link delivers
/// none, so it never fires and the read stalls forever. The driver is
/// executor-agnostic and has no wall clock, so only the call site can supply a
/// real bound. A healthy revolution takes ~90 ms at nominal rotor speed, so two
/// seconds gives roughly a 20× margin while still keeping a queued
/// [`LidarCommand::Disable`] reachable. Expiry is handled exactly like a read
/// error: the device is power-cycled and brought up again.
const STREAM_READ_TIMEOUT: Duration = Duration::from_secs(2);
/// How long the power rail is held low between power-cycle attempts.
const POWER_CYCLE_OFF: Duration = Duration::from_millis(200);
/// Bounded number of power-cycle attempts per enable.
const BRINGUP_ATTEMPTS: u8 = 3;

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

// ── Command channel ───────────────────────────────────────────────────────────

/// Commands accepted by the `LiDAR` task.
#[derive(Debug, Clone, Copy)]
enum LidarCommand {
    /// Power the sensor on and stream, if it is not already enabled.
    Enable,
    /// Stop, power off, and clear stale state.
    Disable,
}

/// Command channel to the `LiDAR` task (capacity 4).
static COMMAND: Channel<CriticalSectionRawMutex, LidarCommand, 4> = Channel::new();

/// Caller-allocated ingest chunk for the driver (at least 1 KiB).
static INGEST: StaticCell<[u8; INGEST_LEN]> = StaticCell::new();

// ── Driver type ───────────────────────────────────────────────────────────────

/// The concrete driver the task owns: buffered UART0 plus the GPIO26 power gate.
type LidarDriver = CoinD6<'static, BufferedUart, Output<'static>>;

// ── Public API ────────────────────────────────────────────────────────────────

/// Ask the task to enable the sensor, without waiting for a reply.
///
/// One-way: this raises this mode's need and returns as soon as the request is
/// queued. The task owns the bring-up, the warm-up, the retries and streaming, so
/// readiness is observed by polling [`status`]. A duplicate is inert — no second
/// bring-up and no extra power — so a caller cannot spin retries by polling, and a
/// failed attempt holds at [`LidarStatus::Failed`] until a [`disable`] followed by
/// an [`enable`].
pub async fn enable() {
    COMMAND.send(LidarCommand::Enable).await;
}

/// Ask the task to disable the sensor, without waiting for a reply.
///
/// One-way: clears this mode's need, and the task stops the device, drops the power
/// gate and clears the stale cloud, obstacle flag and cleared edge in the same
/// step. Disabling when not enabled is inert. The outcome is observed by polling
/// [`status`].
pub async fn disable() {
    COMMAND.send(LidarCommand::Disable).await;
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

// ── Embassy task ──────────────────────────────────────────────────────────────

/// Why the streaming loop returned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StreamEnd {
    /// A disable asked the task to power the sensor down.
    Disabled,
    /// The device failed and could not be restarted.
    Failed,
}

/// `LiDAR` driver embassy task.
///
/// Idles (sensor off) until an enable asks for the sensor, runs the bring-up, then
/// streams revolutions until a disable powers it down or the device dies. Runs on
/// **core1** so `UART0_IRQ` is enabled on core1's NVIC.
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
        // Off (or failed): block until an enable asks for the sensor. A disable
        // when the sensor is already off is inert.
        match COMMAND.receive().await {
            LidarCommand::Enable => {}
            LidarCommand::Disable => continue,
        }

        info!("[lidar] enable requested");
        if run_bring_up(&mut driver, &mut scratch).await {
            set_status(LidarStatus::Streaming);
            info!("[lidar] streaming");
            // Seed the edge tracker with the current flag so the first scan only
            // raises an event on a real change.
            last_obstacle = Some(perception::is_obstacle_detected());
            let end = stream_scans(&mut driver, &mut scratch, &mut sequence, &mut last_obstacle).await;
            info!("[lidar] streaming stopped");
            if end == StreamEnd::Disabled {
                continue;
            }
        } else {
            let _ = driver.power_off();
            set_status(LidarStatus::Failed);
            clear_stale_state(&mut last_obstacle).await;
            warn!("[lidar] enable failed");
        }

        // The device failed, either bringing up or while streaming. Hold at
        // `Failed` until a disable followed by an enable, so a duplicate enable
        // cannot spin retries.
        loop {
            match COMMAND.receive().await {
                LidarCommand::Disable => break,
                LidarCommand::Enable => {}
            }
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

/// Run the bring-up lifecycle with bounded retries, returning success.
///
/// Sets the status to [`LidarStatus::Warming`] for the whole sequence and leaves
/// the caller to set the final `Streaming`/`Failed` status. Each failed attempt is
/// followed by a power-cycle.
async fn run_bring_up(driver: &mut LidarDriver, scratch: &mut Scan) -> bool {
    set_status(LidarStatus::Warming);
    for attempt in 1..=BRINGUP_ATTEMPTS {
        match with_timeout(WARMUP_TIMEOUT, attempt_bring_up(driver, scratch)).await {
            Ok(true) => return true,
            Ok(false) => warn!("[lidar] bring-up attempt {} failed", attempt),
            Err(_) => warn!("[lidar] bring-up attempt {} timed out", attempt),
        }
        // Power-cycle so the retry starts from a known-off device.
        let _ = driver.power_off();
        Timer::after(POWER_CYCLE_OFF).await;
    }
    false
}

/// One bring-up attempt: power on, send start (tolerated), and warm up.
async fn attempt_bring_up(driver: &mut LidarDriver, scratch: &mut Scan) -> bool {
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

/// Stream revolutions until a disable powers the sensor down or the device dies.
///
/// Commands are polled between revolutions. A duplicate enable is inert — the
/// sensor is already up. A disable stops the device, drops its power, clears its
/// stale cloud and obstacle flag, and returns [`StreamEnd::Disabled`].
///
/// Each revolution is read behind [`STREAM_READ_TIMEOUT`], so the read cannot
/// block indefinitely on a silent link: a driver read error and a wall-clock
/// expiry restart identically ([`restart_stream`]). If the restart fails the
/// status becomes `Failed`, the power rail is dropped, and the caller returns to
/// idle.
async fn stream_scans(
    driver: &mut LidarDriver,
    scratch: &mut Scan,
    sequence: &mut u64,
    last_obstacle: &mut Option<bool>,
) -> StreamEnd {
    loop {
        while let Ok(command) = COMMAND.try_receive() {
            match command {
                LidarCommand::Enable => {
                    // Already enabled and streaming; a duplicate is inert.
                }
                LidarCommand::Disable => {
                    stop_and_power_off(driver).await;
                    clear_stale_state(last_obstacle).await;
                    set_status(LidarStatus::Off);
                    return StreamEnd::Disabled;
                }
            }
        }

        match with_timeout(STREAM_READ_TIMEOUT, driver.read_scan(scratch)).await {
            Ok(Ok(())) => {
                *sequence = sequence.wrapping_add(1);
                publish_scan(scratch, *sequence, last_obstacle).await;
            }
            Ok(Err(error)) => {
                // `read_scan` recovers UART glitches and resyncs in place; a
                // returned error means no revolution assembled within the byte
                // watchdog, so restart the device from a clean power cycle.
                warn!(
                    "[lidar] scan read failed, restarting device: {:?}",
                    Debug2Format(&error)
                );
                if !restart_stream(driver, scratch, last_obstacle).await {
                    return StreamEnd::Failed;
                }
            }
            Err(_) => {
                // The driver's byte watchdog only counts bytes that arrive, so
                // it never fires on a silent link. Expiry means no revolution
                // arrived within the wall-clock bound; restart like any error.
                warn!("[lidar] scan read timed out (sensor silent), restarting device");
                if !restart_stream(driver, scratch, last_obstacle).await {
                    return StreamEnd::Failed;
                }
            }
        }
    }
}

/// Power-cycle and re-bring-up after a streaming read ends badly, returning
/// whether the device is streaming again.
///
/// Shared by the two ways a streaming read can end badly — a driver read error
/// and a wall-clock silence expiry — so both restart identically. On success the
/// device streams again; on failure the power rail is dropped, the status becomes
/// [`LidarStatus::Failed`], stale state is cleared, and the caller must return to
/// idle.
async fn restart_stream(driver: &mut LidarDriver, scratch: &mut Scan, last_obstacle: &mut Option<bool>) -> bool {
    let _ = driver.power_off();
    Timer::after(POWER_CYCLE_OFF).await;
    if run_bring_up(driver, scratch).await {
        set_status(LidarStatus::Streaming);
        info!("[lidar] device restarted");
        return true;
    }
    let _ = driver.power_off();
    set_status(LidarStatus::Failed);
    clear_stale_state(last_obstacle).await;
    warn!("[lidar] device failed permanently");
    false
}

/// Map one revolution into a cloud, publish it, and drive the obstacle edge.
async fn publish_scan(scan: &Scan, sequence: u64, last_obstacle: &mut Option<bool>) {
    // Reduce onto the cloud's own slot grid rather than the driver's native 0.9°
    // default: the 1.0° width is deliberately wider than the sensor's sample
    // spacing, so every slot receives a sample and a `None` slot is always a
    // measured no-return, never an unsampled hole (ADR-0018).
    let config = AggregationConfig {
        resolution_deg: SLOT_WIDTH_DEG,
        ..AggregationConfig::default()
    };
    let reduction = aggregate::<_, { lidar_cloud::SLOTS }>(core::slice::from_ref(scan), &config);
    let cloud = Cloud::from_reduction(&reduction, sequence);
    let detected = cloud.front_sector_obstacle();

    perception::update_lidar_points(cloud).await;

    if *last_obstacle != Some(detected) {
        perception::set_lidar_obstacle(detected);
        raise_event(Events::ObstacleDetected {
            source: ObstacleSource::Lidar,
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
/// Disable and terminal-failure both funnel through here so a powered-down sensor
/// never leaves an obstacle flag or an old cloud behind. The cleared edge is
/// raised only when the flag had actually been set.
async fn clear_stale_state(last_obstacle: &mut Option<bool>) {
    if perception::clear_lidar_state().await == ChangeDetected::ChangedToCleared {
        raise_event(Events::ObstacleDetected {
            source: ObstacleSource::Lidar,
        })
        .await;
    }
    *last_obstacle = None;
}
