//! Real COIN-D6 `LiDAR` task — on-demand power, warm-up, and streaming.
//!
//! Replaces the synthetic `lidar_stub` task. A single long-lived embassy task on
//! **core1** owns a [`LidarSession`], which retains the UART0/GPIO28/GPIO29
//! [`Peri`] handles, the `LIDAR_PWR` enable and the static buffers. The UART
//! exists only while the switched 5 V branch is powered: [`LidarSession::power_up`]
//! asserts the enable, waits for the switch rise and constructs the buffered
//! UART; [`LidarSession::power_down`] quiesces it, drops it, releases TX and RX
//! to high-Z, clears `UARTCR.EN`/`TXE`/`RXE`, and only then deasserts
//! `LIDAR_PWR` (ADR-0023). The task is driven by a command channel.
//!
//! # Lifecycle
//!
//! The sensor is off at boot and no UART exists. [`acquire`] powers the branch
//! on, probes the device-info frame, sends the vendor start command (a failure
//! is logged and tolerated, because the device may already be streaming), and
//! waits for the rotor to settle. The driver's only watchdogs count bytes, so
//! the warm-up wait is bounded by a wall clock here: on timeout the task powers
//! the branch down and retries a bounded number of times before reporting
//! [`AcquireError::Failed`]. Every path that deasserts `LIDAR_PWR` — acquire
//! failure, retry, streaming restart, terminal failure and the last lease return
//! — goes through [`LidarSession::power_down`], so no path can switch the branch
//! off with a live UART.
//!
//! Ownership is leased and ref-counted. A successful [`acquire`] yields a
//! move-only [`Lease`] and a second acquire of a streaming sensor is a
//! ref-count increment; an acquisition that races an in-flight bring-up is
//! refused with [`AcquireError::Busy`], and the task — not the caller — decides
//! that refusal. [`return_lease`] consumes the lease, so a mode can only hand
//! back a lease it actually holds: the sensor stops, drops its power, and clears
//! the stale state — cloud, obstacle flag and cleared edge — in the same step,
//! but only when the last lease is returned. Powering the device is what the
//! on-robot check watches: the gate goes high only while a lease is held.
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
//! forever; expiring the bound powers the branch down and re-acquires it, exactly
//! as a read error does. A persistently dead sensor therefore still ends in
//! [`LidarStatus::Failed`] and a queued lease return is still honoured, rather
//! than silent spin.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use coin_d6::{AggregationConfig, CoinD6, Config, Scan, WarmupConfig, aggregate};
use defmt::{Debug2Format, info, warn};
use embassy_rp::{
    Peri,
    gpio::{Input, Level, Output, Pull},
    pac,
    peripherals::{PIN_28, PIN_29, PIN_30, UART0},
    uart::{self, BufferedUart},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer, with_timeout};
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
/// the one way to obtain one is a successful [`acquire`]. [`return_lease`]
/// consumes it, so a mode can only hand back a lease it actually holds —
/// returning a sensor another mode owns is not expressible. The obligation on the
/// holder is that every acquired lease is eventually handed back; a lease
/// dropped without returning leaks its ref count and keeps the sensor powered.
#[must_use = "the sensor is powered down only when the lease is handed back to return_lease"]
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

/// UART baud rate for the COIN-D6 (230400 8N1).
const UART_BAUD: u32 = 230_400;
/// `BufferedUart` TX ring buffer size (start/stop commands are 4 bytes).
const TX_BUF_LEN: usize = 16;
/// `BufferedUart` RX ring buffer size; absorbs stream data between reads.
const RX_BUF_LEN: usize = 4096;
/// Driver ingest chunk size; the driver requires at least 1 KiB.
const INGEST_LEN: usize = 1024;
/// Wall-clock bound on one warm-up attempt.
///
/// The driver has no wall clock — its watchdogs count bytes — so the call site
/// supplies the hard bound. This wraps probe, start and warm-up, but not power
/// assertion or UART construction: a timeout here leaves the driver alive so the
/// normal power-down can run.
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
/// [`LidarCommand::ReturnLease`] reachable. Expiry is handled exactly like a read
/// error: the branch is powered down and re-acquired.
const STREAM_READ_TIMEOUT: Duration = Duration::from_secs(2);
/// Bounded best-effort stop command during power-down.
const STOP_TIMEOUT: Duration = Duration::from_millis(100);
/// Bounded wait for the UART transmitter to drain after the stop command.
///
/// `flush()` only drains the software ring; `BUSY` covers the hardware shifter,
/// so without this wait the stop command can be truncated.
const BUSY_TIMEOUT: Duration = Duration::from_millis(10);
/// Minimum off interval between power-ups. Provisional, pending the `LiDAR`'s
/// required off time and board measurement (ADR-0023).
const POWER_CYCLE_OFF: Duration = Duration::from_millis(200);
/// TPS22917 switch rise time after asserting `LIDAR_PWR`. Provisional, pending
/// the board's `CT` value (ADR-0023).
const BRANCH_RISE: Duration = Duration::from_millis(50);
/// Bounded number of power-cycle attempts per acquire.
const ACQUIRE_ATTEMPTS: u8 = 3;

// ── Session ───────────────────────────────────────────────────────────────────

/// Static TX ring buffer, reborrowed for each powered period.
static TX_BUF: StaticCell<[u8; TX_BUF_LEN]> = StaticCell::new();
/// Static RX ring buffer, reborrowed for each powered period.
static RX_BUF: StaticCell<[u8; RX_BUF_LEN]> = StaticCell::new();

/// The concrete driver for one powered period: buffered UART0 plus the GPIO30
/// power gate.
type LidarDriver<'a> = CoinD6<'a, BufferedUart, Output<'static>>;

/// Owner of the `LiDAR` branch's retained resources and power/UART sequencing.
///
/// The session keeps the raw [`Peri`] handles, one `Output` for `LIDAR_PWR`
/// (held low while off) and the `'static` TX/RX buffers. It does not store the
/// driver: [`Self::power_up`] returns one that borrows the caller's ingest chunk
/// for the powered period, and [`Self::power_down`] consumes it again. That
/// keeps the UART — and the pads muxed to it — alive only while the branch is
/// powered.
pub struct LidarSession {
    /// UART0 peripheral handle, retained for reconstruction.
    uart: Peri<'static, UART0>,
    /// UART0 TX pad (GPIO 28), retained for reconstruction and release.
    tx: Peri<'static, PIN_28>,
    /// UART0 RX pad (GPIO 29), retained for reconstruction and release.
    rx: Peri<'static, PIN_29>,
    /// `LIDAR_PWR` enable; `None` only while a driver owns the powered branch.
    power: Option<Output<'static>>,
    /// Static TX ring buffer, reborrowed on every construction.
    tx_buf: &'static mut [u8; TX_BUF_LEN],
    /// Static RX ring buffer, reborrowed on every construction.
    rx_buf: &'static mut [u8; RX_BUF_LEN],
    /// UART configuration used for every construction.
    uart_config: uart::Config,
    /// Instant of the most recent power-down, for the minimum off interval.
    last_power_down: Option<Instant>,
}

impl LidarSession {
    /// Create the session from the retained `LiDAR` resources.
    ///
    /// `LIDAR_PWR` starts low and no UART is constructed, so the branch is in
    /// its off-state at boot.
    #[must_use]
    pub fn new(
        uart: Peri<'static, UART0>,
        mut tx: Peri<'static, PIN_28>,
        mut rx: Peri<'static, PIN_29>,
        power_enable: Peri<'static, PIN_30>,
    ) -> Self {
        // Hold both UART pads high-Z with no pull from boot, before the branch
        // is ever powered: the off-state invariant must hold before the first
        // power-down.
        drop(Input::new(tx.reborrow(), Pull::None));
        drop(Input::new(rx.reborrow(), Pull::None));

        let mut uart_config = uart::Config::default();
        uart_config.baudrate = UART_BAUD;
        Self {
            uart,
            tx,
            rx,
            power: Some(Output::new(power_enable, Level::Low)),
            tx_buf: TX_BUF.init([0; TX_BUF_LEN]),
            rx_buf: RX_BUF.init([0; RX_BUF_LEN]),
            uart_config,
            last_power_down: None,
        }
    }

    /// Power the branch and build a driver for the powered period.
    ///
    /// Waits out the minimum off interval, asserts `LIDAR_PWR`, waits for the
    /// TPS22917 rise, then constructs the buffered UART from the retained handles
    /// and reborrowed static buffers. There is no await point between
    /// construction and returning the driver, so cancellation cannot leave a
    /// muxed UART owned by nobody while the branch powers down.
    ///
    /// Returns `None` only if a driver already owns the power enable, which the
    /// one-driver-at-a-time task contract makes unreachable.
    #[must_use = "the driver owns the powered branch until it is passed to `power_down`"]
    pub async fn power_up<'i>(&mut self, ingest: &'i mut [u8]) -> Option<LidarDriver<'i>> {
        if let Some(last) = self.last_power_down {
            let elapsed = Instant::now().saturating_duration_since(last);
            if let Some(remaining) = POWER_CYCLE_OFF.checked_sub(elapsed) {
                Timer::after(remaining).await;
            }
        }

        let mut power = self.power.take()?;
        // `embassy_rp::gpio::Output`'s inherent setter is infallible.
        power.set_high();
        Timer::after(BRANCH_RISE).await;

        let uart = BufferedUart::new(
            self.uart.reborrow(),
            self.tx.reborrow(),
            self.rx.reborrow(),
            crate::Irqs,
            &mut self.tx_buf[..],
            &mut self.rx_buf[..],
            self.uart_config,
        );
        Some(CoinD6::new(uart, power, ingest, Config::default()))
    }

    /// Quiesce and deinitialize the branch, then deassert `LIDAR_PWR`.
    ///
    /// Runs the ADR-0023 sequence in order: bounded best-effort stop, recover the
    /// UART and enable, bounded wait for the hardware shifter, drop the UART,
    /// release TX and RX to high-Z inputs with no pull, clear `UARTCR.EN`/`TXE`/
    /// `RXE`, deassert the enable and record the power-down instant. Every wait
    /// is bounded, so a dead or silent UART cannot hang teardown.
    pub async fn power_down(&mut self, mut driver: LidarDriver<'_>) {
        // 1. Best-effort stop, bounded so a dead or silent UART cannot hang
        //    teardown.
        match with_timeout(STOP_TIMEOUT, driver.stop()).await {
            Ok(Ok(())) => {}
            Ok(Err(error)) => warn!("[lidar] stop command failed (best-effort): {:?}", Debug2Format(&error)),
            Err(_) => warn!("[lidar] stop timed out during power-down"),
        }

        // 2. Recover the UART and the power enable.
        let (uart, mut power) = driver.release();
        // 3. Wait, bounded, for the hardware shifter to drain. `flush()` only
        //    drains the software ring, so this is what keeps the stop command
        //    from being truncated.
        let drained = with_timeout(BUSY_TIMEOUT, async {
            while uart.busy() {
                Timer::after(Duration::from_micros(100)).await;
            }
        })
        .await;
        if drained.is_err() {
            warn!("[lidar] UART shifter did not drain within {:?}", BUSY_TIMEOUT);
        }

        // 4. Drop the UART: this deinits its ring buffers and disables
        //    `UART0_IRQ`.
        drop(uart);

        // 5. Release both pads to high-Z inputs with no pull, so neither line can
        //    source current into the unpowered branch.
        drop(Input::new(self.tx.reborrow(), Pull::None));
        drop(Input::new(self.rx.reborrow(), Pull::None));

        // 6. Deinitialize the peripheral: no enable, transmitter or receiver.
        pac::UART0.uartcr().modify(|w| {
            w.set_uarten(false);
            w.set_txe(false);
            w.set_rxe(false);
        });

        // 7. Deassert the branch enable.
        power.set_low();
        self.power = Some(power);

        // 8. Record the instant so the next power-up enforces the minimum off
        //    interval.
        self.last_power_down = Some(Instant::now());
    }
}

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
    /// Return the lease: stop, power off, and clear stale state when it was the
    /// last one outstanding.
    ReturnLease,
}

/// Command channel to the `LiDAR` task (capacity 4).
static COMMAND: Channel<CriticalSectionRawMutex, LidarCommand, 4> = Channel::new();

/// One reply per [`LidarCommand::Acquire`]; a channel rather than a signal so
/// concurrent callers each receive one reply without losing a wake-up. A
/// successful reply carries the [`Lease`] the caller then owns.
static ACQUIRE_REPLY: Channel<CriticalSectionRawMutex, Result<Lease, AcquireError>, 4> = Channel::new();

/// One acknowledgement per [`LidarCommand::ReturnLease`].
static RETURN_LEASE_REPLY: Channel<CriticalSectionRawMutex, (), 4> = Channel::new();

/// Caller-allocated ingest chunk for the driver (at least 1 KiB).
static INGEST: StaticCell<[u8; INGEST_LEN]> = StaticCell::new();

// ── Public API ────────────────────────────────────────────────────────────────

/// Power on the sensor and wait until it is warmed and streaming.
///
/// On success returns the move-only [`Lease`] the caller must hand back to
/// [`return_lease`]. Sends the start command (tolerating a write failure, since
/// the device may already be streaming) and waits for the rotor to settle behind
/// a wall-clock bound. On timeout the task powers the branch down and retries a
/// bounded number of times, then reports [`AcquireError::Failed`] and leaves the
/// status `Failed`.
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
/// Consumes the lease the caller holds, so only an owner can return one.
/// Returns once the task has completed it, so a caller can rely on the cloud
/// being absent and the obstacle flag cleared. The sensor is torn down only when
/// this was the last outstanding lease; while another mode still holds one, the
/// device keeps streaming. The stop command is best-effort; the power gate and
/// the state clear always run on the last return.
pub async fn return_lease(lease: Lease) {
    // Destructure the token so it is provably consumed: only an owner of a lease
    // can hand one back, and the task alone decides whether it was the last.
    let Lease { _private: () } = lease;
    COMMAND.send(LidarCommand::ReturnLease).await;
    RETURN_LEASE_REPLY.receive().await;
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
/// streams revolutions until the last lease is returned or the device dies. Runs
/// on **core1** so `UART0_IRQ` is enabled on core1's NVIC.
#[embassy_executor::task]
#[allow(clippy::large_futures)]
pub async fn lidar_task(mut session: LidarSession) {
    let ingest: &'static mut [u8; INGEST_LEN] = INGEST.init([0u8; INGEST_LEN]);
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
            LidarCommand::ReturnLease => {
                // Already off — nothing to stop or clear.
                RETURN_LEASE_REPLY.send(()).await;
                continue;
            }
        }

        info!("[lidar] acquire requested");
        // The initial bring-up sends its own `Ok` reply. If it fails, the task
        // answers the waiting acquirer here; a later restart failure has no
        // waiting acquirer and is surfaced through the status cell only.
        if !serve_acquire(
            &mut session,
            &mut ingest[..],
            &mut scratch,
            &mut sequence,
            &mut last_obstacle,
        )
        .await
        {
            ACQUIRE_REPLY.send(Err(AcquireError::Failed)).await;
            warn!("[lidar] acquire failed");
        }
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

/// How a streaming run ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StreamOutcome {
    /// The last lease was returned; the driver is powered down and status `Off`.
    LastLeaseReturned,
    /// The device stopped responding; the driver is powered down and the caller
    /// should re-acquire it.
    Restart,
}

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

/// Serve one acquire request through streaming until the last lease is returned
/// or the branch fails permanently.
///
/// Brings the branch up with bounded retries. Power assertion and UART
/// construction happen outside the attempt timeout; each failed attempt goes
/// through [`LidarSession::power_down`], and the next
/// [`LidarSession::power_up`] enforces the minimum off interval. On the first
/// success this sends the `Ok` lease reply and seeds the obstacle edge tracker;
/// on a later success after a streaming failure it keeps the outstanding leases
/// and just resumes.
///
/// Returns `true` once the initial bring-up succeeded and its reply was sent.
/// Returns `false` only when the initial bring-up exhausted its retries; the
/// caller then answers the waiting acquirer with [`AcquireError::Failed`]. The
/// status is left `Failed` and stale state is cleared in both failure cases.
#[allow(clippy::large_futures)]
async fn serve_acquire(
    session: &mut LidarSession,
    ingest: &mut [u8],
    scratch: &mut Scan,
    sequence: &mut u64,
    last_obstacle: &mut Option<bool>,
) -> bool {
    let mut started = false;
    // Leases outstanding for the streaming run; preserved across a restart so a
    // mode that leased the sensor before the drop still owns it afterwards.
    let mut leases: u32 = 1;

    loop {
        // A bring-up beginning starts a new generation: every tag read before
        // this point describes an earlier phase and must not be served as a
        // lease.
        advance_generation();
        set_status(LidarStatus::Warming);

        let mut attempt: u8 = 1;
        let driver = loop {
            let Some(mut candidate) = session.power_up(&mut *ingest).await else {
                break None;
            };
            match with_timeout(WARMUP_TIMEOUT, attempt_acquire(&mut candidate, scratch)).await {
                Ok(true) => break Some(candidate),
                Ok(false) => warn!("[lidar] acquire attempt {} failed", attempt),
                Err(_) => warn!("[lidar] acquire attempt {} timed out", attempt),
            }
            // Power the branch down so the retry starts from a known off-state.
            session.power_down(candidate).await;
            if attempt >= ACQUIRE_ATTEMPTS {
                break None;
            }
            attempt += 1;
        };

        let Some(driver) = driver else {
            set_status(LidarStatus::Failed);
            clear_stale_state(last_obstacle).await;
            if started {
                warn!("[lidar] device failed permanently");
            }
            return started;
        };

        // Streaming is a new generation: tags read during bring-up are now
        // stale and must not be served as leases while this run streams.
        advance_generation();
        set_status(LidarStatus::Streaming);
        if started {
            info!("[lidar] device restarted");
        } else {
            started = true;
            info!("[lidar] streaming");
            ACQUIRE_REPLY.send(Ok(Lease::new())).await;
            // Seed the edge tracker with the current flag so the first scan only
            // raises an event on a real change.
            *last_obstacle = Some(perception::is_obstacle_detected());
        }

        match stream_scans(session, driver, scratch, sequence, last_obstacle, &mut leases).await {
            StreamOutcome::LastLeaseReturned => {
                info!("[lidar] streaming stopped");
                return true;
            }
            StreamOutcome::Restart => {
                info!("[lidar] restarting device");
            }
        }
    }
}

/// One acquire attempt: probe the device-info frame, send start (tolerated), and
/// warm up.
async fn attempt_acquire(driver: &mut LidarDriver<'_>, scratch: &mut Scan) -> bool {
    if let Err(error) = driver.probe().await {
        warn!("[lidar] device-info probe failed: {:?}", Debug2Format(&error));
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

/// Stream revolutions until the last lease is returned or the device dies.
///
/// Commands are polled between revolutions. An acquire tagged with the current
/// generation is a ref-count increment and yields another [`Lease`]; a stale tag
/// is refused as [`AcquireError::Busy`] rather than served as a lease. A lease
/// return decrements the count and tears the device down only as it reaches zero,
/// so a mode handing back its lease cannot stop a sensor another mode still
/// holds.
///
/// Each revolution is read behind [`STREAM_READ_TIMEOUT`], so the read cannot
/// block indefinitely on a silent link: a driver read error and a wall-clock
/// expiry both power the branch down and return [`StreamOutcome::Restart`], and
/// both keep honouring a lease return queued meanwhile. The driver is consumed
/// on every return path, and its ingest borrow is released with it.
async fn stream_scans(
    session: &mut LidarSession,
    mut driver: LidarDriver<'_>,
    scratch: &mut Scan,
    sequence: &mut u64,
    last_obstacle: &mut Option<bool>,
    leases: &mut u32,
) -> StreamOutcome {
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
                LidarCommand::ReturnLease => {
                    *leases = leases.saturating_sub(1);
                    if *leases == 0 {
                        session.power_down(driver).await;
                        clear_stale_state(last_obstacle).await;
                        set_status(LidarStatus::Off);
                        RETURN_LEASE_REPLY.send(()).await;
                        return StreamOutcome::LastLeaseReturned;
                    }
                    RETURN_LEASE_REPLY.send(()).await;
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
                session.power_down(driver).await;
                return StreamOutcome::Restart;
            }
            Err(_) => {
                // The driver's byte watchdog only counts bytes that arrive, so
                // it never fires on a silent link. Expiry means no revolution
                // arrived within the wall-clock bound; restart like any error.
                warn!("[lidar] scan read timed out (sensor silent), restarting device");
                session.power_down(driver).await;
                return StreamOutcome::Restart;
            }
        }
    }
}

/// Map one revolution into a cloud, publish it, and drive the obstacle edge.
async fn publish_scan(scan: &Scan, sequence: u64, last_obstacle: &mut Option<bool>) {
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

/// Clear the stale cloud and obstacle flag, raising the cleared edge if needed.
///
/// Lease return and terminal failure both funnel through here so a powered-down
/// sensor never leaves an obstacle flag or an old cloud behind. The cleared edge
/// is raised only when the flag had actually been set.
async fn clear_stale_state(last_obstacle: &mut Option<bool>) {
    if perception::clear_lidar_state().await == ChangeDetected::ChangedToCleared {
        raise_event(Events::ObstacleDetected {
            source: ObstacleSource::Lidar,
        })
        .await;
    }
    *last_obstacle = None;
}
