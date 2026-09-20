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
//! [`AcquireError::Failed`].
//!
//! Ownership is leased and ref-counted. A successful [`acquire`] yields a
//! move-only [`Lease`] and a second acquire of a streaming sensor is a
//! ref-count increment; an acquisition that races an in-flight bring-up is
//! refused with [`AcquireError::Busy`], and the task — not the caller — decides
//! that refusal. [`release`] consumes the lease, so a mode can only hand
//! back a lease it actually holds: the sensor stops, drops its power, and clears
//! the stale state — cloud, obstacle flag and cleared edge — in the same step,
//! but only when the last lease is released. Powering the device is what the
//! on-robot check watches: the gate goes high only while a lease is held.
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

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

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
    /// An acquisition is already in flight and not yet streaming. The task
    /// refuses the request without starting a second lifecycle.
    Busy,
    /// The lifecycle exhausted its retries; the sensor stays powered down.
    Failed,
}

// ── Lease ──────────────────────────────────────────────────────────────────────

/// A move-only token proving the holder acquired the `LiDAR`.
///
/// The token is neither [`Clone`] nor [`Copy`], and its only field is private, so
/// the one way to obtain one is a successful [`acquire`]. [`release`] consumes
/// it, so a mode can only hand back a lease it actually holds — releasing a
/// sensor another mode owns is not expressible. The obligation on the holder is
/// that every acquired lease is eventually handed back; a lease dropped without
/// release leaks its ref count and keeps the sensor powered.
#[must_use = "the sensor is powered down only when the lease is handed back to release"]
pub struct Lease {
    /// Private marker; keeps the constructor confined to this module.
    _private: (),
}

impl Lease {
    /// Seal the token shipped back in an [`acquire`] reply.
    const fn new() -> Self {
        Self { _private: () }
    }
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

/// Generation of the acquisition lifecycle, used to tag [`LidarCommand::Acquire`].
///
/// An [`acquire`] caller reads the generation before it enqueues its request and
/// the task compares that tag with the value in force as it serves the request.
/// The task bumps it when a bring-up begins and again when streaming is reached,
/// so a tag that no longer matches can only have been read before a transition —
/// a request made while an acquisition was already in flight. That is the
/// authority behind [`AcquireError::Busy`]; the caller-side `Warming` test is
/// only a fast path.
static ACQUIRE_GENERATION: AtomicU32 = AtomicU32::new(0);

// ── Command channel & replies ─────────────────────────────────────────────────

/// Commands accepted by the `LiDAR` task.
#[derive(Debug, Clone, Copy)]
enum LidarCommand {
    /// Run the power-on/warm-up lifecycle, if the tag is still current.
    Acquire {
        /// The [`ACQUIRE_GENERATION`] value the caller read before sending.
        generation: u32,
    },
    /// Stop, power off, and clear stale state.
    Release,
}

/// Command channel to the `LiDAR` task (capacity 4).
static COMMAND: Channel<CriticalSectionRawMutex, LidarCommand, 4> = Channel::new();

/// One reply per [`LidarCommand::Acquire`]; a channel rather than a signal so
/// concurrent callers each receive one reply without losing a wake-up. A
/// successful reply carries the [`Lease`] the caller then owns.
static ACQUIRE_REPLY: Channel<CriticalSectionRawMutex, Result<Lease, AcquireError>, 4> = Channel::new();

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
/// On success returns the move-only [`Lease`] the caller must hand back to
/// [`release`]. Sends the start command (tolerating a write failure, since the
/// device may already be streaming) and waits for the rotor to settle behind a
/// wall-clock bound. On timeout the task power-cycles and retries a bounded
/// number of times, then reports [`AcquireError::Failed`] and leaves the status
/// `Failed`.
///
/// An acquisition already in flight is refused with [`AcquireError::Busy`],
/// without starting a second lifecycle. The [`status`] test below is only a fast
/// path that avoids waiting out a multi-second warm-up; the authority is the
/// task, which refuses any request whose [`ACQUIRE_GENERATION`] tag a bring-up or
/// the streaming transition has made stale. Acquiring an already-streaming
/// sensor is a ref-count increment and yields another lease.
///
/// # Errors
///
/// [`AcquireError::Busy`] if an acquisition is already in flight and not yet
/// streaming, [`AcquireError::Failed`] if the lifecycle exhausted its retries.
pub async fn acquire() -> Result<Lease, AcquireError> {
    // Read the generation *before* the status fast path: the tag must describe
    // the phase the caller is about to act on, so a caller racing a bring-up
    // cannot pick up the generation that bring-up leaves behind at streaming.
    let generation = ACQUIRE_GENERATION.load(Ordering::Relaxed);
    // Fast path only — the task re-checks the tag and is the authority.
    if status() == LidarStatus::Warming {
        return Err(AcquireError::Busy);
    }
    COMMAND.send(LidarCommand::Acquire { generation }).await;
    ACQUIRE_REPLY.receive().await
}

/// Stop the device, drop its power, and clear all stale state.
///
/// Consumes the lease the caller holds, so only an owner can ask for a release.
/// Returns once the task has completed it, so a caller can rely on the cloud
/// being absent and the obstacle flag cleared. The sensor is torn down only when
/// this was the last outstanding lease; while another mode still holds one, the
/// device keeps streaming. The stop command is best-effort; the power gate and
/// the state clear always run on the last release.
pub async fn release(lease: Lease) {
    // Destructure the token so it is provably consumed: only an owner of a lease
    // can hand one back, and the task alone decides whether it was the last.
    let Lease { _private: () } = lease;
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

// ── Embassy task ──────────────────────────────────────────────────────────────

/// `LiDAR` driver embassy task.
///
/// Idles (sensor off) until an acquire command arrives, runs the lifecycle, then
/// streams revolutions until the last lease is released or the device dies. Runs
/// on **core1** so `UART0_IRQ` is enabled on core1's NVIC.
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
            LidarCommand::Acquire { generation } => {
                // Admission is decided here, not by the caller's fast path. A tag
                // that is not the generation in force was read before a past
                // transition, so that caller raced an acquisition already in
                // flight; refuse it without starting another lifecycle.
                if generation != ACQUIRE_GENERATION.load(Ordering::Relaxed) {
                    ACQUIRE_REPLY.send(Err(AcquireError::Busy)).await;
                    continue;
                }
            }
            LidarCommand::Release => {
                // Already off — nothing to stop or clear.
                RELEASE_REPLY.send(()).await;
                continue;
            }
        }

        info!("[lidar] acquire requested");
        if run_acquire(&mut driver, &mut scratch).await {
            // Streaming is a new generation: tags read during bring-up are now
            // stale and must not be served as leases while this run streams.
            advance_generation();
            set_status(LidarStatus::Streaming);
            // Leases outstanding for this streaming run; the sensor is up while
            // this is nonzero. It is spent by the time `stream_scans` returns —
            // either released to zero or abandoned when the device died — so the
            // next acquisition always begins a fresh count.
            let mut leases: u32 = 1;
            ACQUIRE_REPLY.send(Ok(Lease::new())).await;
            info!("[lidar] streaming");
            // Seed the edge tracker with the current flag so the first scan
            // only raises an event on a real change.
            last_obstacle = Some(perception::is_obstacle_detected());
            stream_scans(
                &mut driver,
                &mut scratch,
                &mut sequence,
                &mut last_obstacle,
                &mut leases,
            )
            .await;
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

/// Begin a new acquisition generation.
///
/// Called as a bring-up starts and again as streaming is reached, so every tag a
/// caller read before that point stops matching the value in force and its
/// request is refused as [`AcquireError::Busy`].
fn advance_generation() {
    ACQUIRE_GENERATION.fetch_add(1, Ordering::Relaxed);
}

/// Run the acquire lifecycle with bounded retries, returning success.
///
/// Advances the acquisition generation as the bring-up begins, sets the status
/// to [`LidarStatus::Warming`] for the whole sequence, and leaves the caller to
/// set the final `Streaming`/`Failed` status. Each failed attempt is followed by
/// a power-cycle.
async fn run_acquire(driver: &mut LidarDriver, scratch: &mut Scan) -> bool {
    // A bring-up beginning starts a new generation: every tag read before this
    // point describes an earlier phase and must not be served as a lease.
    advance_generation();
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

/// Stream revolutions until the last lease is released or the device dies.
///
/// Commands are polled between revolutions. An acquire tagged with the current
/// generation is a ref-count increment and yields another [`Lease`]; a stale tag
/// is refused as [`AcquireError::Busy`] rather than served as a lease. A release
/// decrements the count and tears the device down only as it reaches zero, so a
/// mode handing back its lease cannot stop a sensor another mode still holds. A
/// read error power-cycles and re-runs the acquire lifecycle once; if that fails
/// the status becomes `Failed` and the task returns to idle.
async fn stream_scans(
    driver: &mut LidarDriver,
    scratch: &mut Scan,
    sequence: &mut u64,
    last_obstacle: &mut Option<bool>,
    leases: &mut u32,
) {
    loop {
        while let Ok(command) = COMMAND.try_receive() {
            match command {
                LidarCommand::Acquire { generation } => {
                    if generation == ACQUIRE_GENERATION.load(Ordering::Relaxed) {
                        *leases = leases.saturating_add(1);
                        ACQUIRE_REPLY.send(Ok(Lease::new())).await;
                    } else {
                        // Read before a transition: this caller raced the
                        // bring-up that started this run, or the streaming
                        // transition itself, so refuse it instead of serving a
                        // second lease and powering nothing extra on.
                        ACQUIRE_REPLY.send(Err(AcquireError::Busy)).await;
                    }
                }
                LidarCommand::Release => {
                    *leases = leases.saturating_sub(1);
                    let last = *leases == 0;
                    if last {
                        stop_and_power_off(driver).await;
                        clear_stale_state(last_obstacle).await;
                        set_status(LidarStatus::Off);
                    }
                    RELEASE_REPLY.send(()).await;
                    if last {
                        return;
                    }
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
                    // A restart reaches streaming as a new generation too, so
                    // tags read before the failure are stale.
                    advance_generation();
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
