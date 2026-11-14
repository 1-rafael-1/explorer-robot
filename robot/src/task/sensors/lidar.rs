//! Real COIN-D6 `LiDAR` task — on-demand power, warm-up, and streaming.
//!
//! A single long-lived embassy task on
//! **core1** owns a [`LidarSession`], which retains the UART0/GPIO28/GPIO29
//! [`Peri`] handles, the `LIDAR_PWR` enable and the static buffers. The UART
//! exists only while the switched 5 V branch is powered: [`UartResources::power_up`]
//! asserts the enable, waits for the switch rise and constructs the buffered
//! UART; [`quiesce`] drains and drops it; then [`PowerControl::complete_power_down`]
//! releases TX and RX to high-Z through [`UartResources::release_pads`], clears
//! `UARTCR.EN`/`TXE`/`RXE`, and only then deasserts `LIDAR_PWR` (ADR-0023). The
//! task is driven by a one-way command channel.
//!
//! # The model
//!
//! A mode calls [`enable`] when it needs perception and [`disable`] when it is
//! done. The commands are one-way on a dedicated [`Channel`], as motor, flash and
//! encoder commands are: the task owns the branch power, the vendor start
//! command, the warm-up, the retries, streaming and the power-off, and rebuilds
//! the driver for each powered period. At most one mode needs perception at a
//! time (ADR-0028) — the panel is modal, and each mode slot admits one mode — so
//! the enabled sensor serves every reader.
//!
//! Readiness is observed by polling the lock-free [`status`] — `Off`, `Warming`,
//! `Streaming` or `Failed`; the task never signals. Bring-up is edge-triggered: a
//! duplicate [`enable`] is inert, and a failed attempt holds at `Failed` until a
//! [`disable`], so a caller cannot spin retries by polling. [`enable`] releases a
//! latched failure itself — sending the `Disable` and waiting for the task to
//! publish the reset as `Off` — so a caller that re-enters a mode after a failure
//! recovers rather than wedging on an ignored `Enable`. [`disable`] stops the
//! device, powers the branch down and clears the stale cloud, obstacle flag and
//! cleared edge (ADR-0015) in the same step.
//!
//! # Lifecycle
//!
//! The sensor is off at boot and no UART exists. [`enable`] powers the branch on,
//! best-effort probes the device-info frame (bounded and tolerated: the frame is
//! emitted on power-up and can be missed), sends the vendor start command (a
//! failure is logged and tolerated, because the device may already be streaming),
//! and waits for the rotor to settle. The driver's only watchdogs count bytes, so the
//! warm-up wait is bounded by a wall clock here: on timeout the task powers the
//! branch down and retries a bounded number of times before reporting
//! [`LidarStatus::Failed`]. Every path that deasserts `LIDAR_PWR` — bring-up
//! failure, retry, streaming restart, terminal failure and disable — goes through
//! [`quiesce`], so no path can switch the branch off with a live
//! UART.
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
//! forever; expiring the bound powers the branch down and re-brings it up, exactly
//! as a read error does. A persistently dead sensor therefore still ends in
//! [`LidarStatus::Failed`] and a queued disable is still honoured, rather than
//! silent spin.

use core::sync::atomic::{AtomicU8, Ordering};

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
    /// A bring-up or restart failed; the sensor is powered down and holds at
    /// `Failed` until a disable then enable.
    Failed,
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
/// Wall-clock bound on the optional device-info probe.
///
/// The sensor announces itself once, on power-up. The branch and UART come up
/// after that, so the frame can be missed entirely, and a device that is already
/// streaming has no reason to send it again. The probe only validates and drains
/// that frame; it must not hold back the start command. A missed frame therefore
/// costs this bound, then start and warm-up proceed.
const PROBE_TIMEOUT: Duration = Duration::from_millis(250);
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
/// error: the branch is powered down and re-brought up.
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
/// Bounded number of power-cycle attempts per enable.
const BRINGUP_ATTEMPTS: u8 = 3;
/// Poll interval while [`enable`] waits for a latched failure to reset (ms).
const FAILURE_RESET_POLL_MS: u64 = 50;

// ── Session ───────────────────────────────────────────────────────────────────

/// Static TX ring buffer, reborrowed for each powered period.
static TX_BUF: StaticCell<[u8; TX_BUF_LEN]> = StaticCell::new();
/// Static RX ring buffer, reborrowed for each powered period.
static RX_BUF: StaticCell<[u8; RX_BUF_LEN]> = StaticCell::new();

/// The concrete driver for one powered period: buffered UART0 plus the GPIO30
/// power gate.
type LidarDriver<'a, 'p> = CoinD6<'a, BufferedUart<'p>, Output<'static>>;

/// Owner of the `LiDAR` branch's retained resources and power/UART sequencing.
///
/// The session keeps the raw [`Peri`] handles, one `Output` for `LIDAR_PWR`
/// (held low while off) and the `'static` TX/RX buffers. It does not store the
/// driver: [`UartResources::power_up`] returns one that borrows the caller's ingest
/// chunk for the powered period, and [`quiesce`] consumes it again. That
/// keeps the UART — and the pads muxed to it — alive only while the branch is
/// powered.
pub struct LidarSession {
    /// UART handles and ring buffers, borrowed for one powered period.
    port: UartResources,
    /// Power line, usable while the driver borrows the port.
    control: PowerControl,
}

/// UART handles and ring buffers, borrowed for one powered period.
struct UartResources {
    /// UART0 peripheral handle, retained for reconstruction.
    uart: Peri<'static, UART0>,
    /// UART0 TX pad (GPIO 28), retained for reconstruction and release.
    tx: Peri<'static, PIN_28>,
    /// UART0 RX pad (GPIO 29), retained for reconstruction and release.
    rx: Peri<'static, PIN_29>,
    /// Static TX ring buffer, reborrowed on every construction.
    tx_buf: &'static mut [u8; TX_BUF_LEN],
    /// Static RX ring buffer, reborrowed on every construction.
    rx_buf: &'static mut [u8; RX_BUF_LEN],
    /// UART configuration used for every construction.
    uart_config: uart::Config,
}

/// Power line and timing retained across powered periods.
struct PowerControl {
    /// `LIDAR_PWR` enable; `None` only while a driver owns the powered branch.
    power: Option<Output<'static>>,
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
            port: UartResources {
                uart,
                tx,
                rx,
                tx_buf: TX_BUF.init([0; TX_BUF_LEN]),
                rx_buf: RX_BUF.init([0; RX_BUF_LEN]),
                uart_config,
            },
            control: PowerControl {
                power: Some(Output::new(power_enable, Level::Low)),
                last_power_down: None,
            },
        }
    }
}

impl UartResources {
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
    #[must_use = "the driver owns the powered branch until it is passed to `quiesce`"]
    async fn power_up<'i, 'p>(
        &'p mut self,
        control: &mut PowerControl,
        ingest: &'i mut [u8],
    ) -> Option<LidarDriver<'i, 'p>> {
        if let Some(last) = control.last_power_down {
            let elapsed = Instant::now().saturating_duration_since(last);
            if let Some(remaining) = POWER_CYCLE_OFF.checked_sub(elapsed) {
                Timer::after(remaining).await;
            }
        }

        let mut power = control.power.take()?;
        // `embassy_rp::gpio::Output`'s inherent setter is infallible.
        power.set_high();
        #[cfg(feature = "telemetry_logs")]
        info!("[lidar] LIDAR_PWR asserted");
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
        #[cfg(feature = "telemetry_logs")]
        info!("[lidar] UART up, probing");
        Some(CoinD6::new(uart, power, ingest, Config::default()))
    }

    /// Release both pads to high-Z inputs with no pull, so neither line can
    /// source current into the unpowered branch.
    fn release_pads(&mut self) {
        drop(Input::new(self.tx.reborrow(), Pull::None));
        drop(Input::new(self.rx.reborrow(), Pull::None));
    }
}

impl PowerControl {
    /// Release the pads, deinitialize the peripheral and deassert the branch
    /// after the UART has been dropped.
    ///
    /// Releases both UART pads to high-Z via [`UartResources::release_pads`],
    /// clears `UARTCR.EN`/`TXE`/`RXE`, deasserts the enable and records the
    /// power-down instant.
    ///
    /// The split between [`UartResources`] and `PowerControl` is forced by
    /// `BufferedUart` borrowing the UART handles, so the teardown order is fixed:
    /// `quiesce` → release pads → deassert power. Folding the pad release in here
    /// keeps that order at one call site.
    fn complete_power_down(&mut self, uart: &mut UartResources, mut power: Output<'static>) {
        uart.release_pads();

        pac::UART0.uartcr().modify(|w| {
            w.set_uarten(false);
            w.set_txe(false);
            w.set_rxe(false);
        });

        power.set_low();
        self.power = Some(power);
        self.last_power_down = Some(Instant::now());
    }
}

/// Stop the device, recover the UART, drain it, and drop it.
///
/// Runs the ADR-0023 order up to the drop: bounded best-effort stop, recover the
/// UART and enable, bounded wait for the hardware shifter, drop the UART. Every
/// wait is bounded, so a dead or silent UART cannot hang teardown.
async fn quiesce(mut driver: LidarDriver<'_, '_>) -> Output<'static> {
    // 1. Best-effort stop, bounded so a dead or silent UART cannot hang
    //    teardown.
    match with_timeout(STOP_TIMEOUT, driver.stop()).await {
        Ok(Ok(())) => {}
        Ok(Err(error)) => warn!("[lidar] stop command failed (best-effort): {:?}", Debug2Format(&error)),
        Err(_) => warn!("[lidar] stop timed out during power-down"),
    }

    // 2. Recover the UART and the power enable.
    let (uart, power) = driver.release();
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

    // 4. Drop the UART: this deinits its ring buffers and disables `UART0_IRQ`.
    drop(uart);

    power
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

// ── Public API ────────────────────────────────────────────────────────────────

/// Ask the task to enable the sensor, without waiting for it to stream.
///
/// One-way: this raises this mode's need and returns once the request is queued,
/// plus at most the short failure reset below. The task owns the bring-up, the
/// warm-up, the retries and streaming, so readiness is observed by polling
/// [`status`]. A duplicate is inert — no second bring-up and no extra power — so a
/// caller cannot spin retries by polling.
///
/// A previous run can leave the sensor latched [`LidarStatus::Failed`], and the
/// task ignores an `Enable` while it is held, so an enable sent on its own would
/// be swallowed. This therefore releases the latch first — a `Disable`, then a
/// wait for the task to publish the reset as [`LidarStatus::Off`] — so the enable
/// below always starts a fresh bring-up, whether the sensor is off, streaming or
/// failed, and a caller that re-enters a mode after a failure recovers.
pub async fn enable() {
    if status() == LidarStatus::Failed {
        COMMAND.send(LidarCommand::Disable).await;
        while status() == LidarStatus::Failed {
            Timer::after(Duration::from_millis(FAILURE_RESET_POLL_MS)).await;
        }
    }
    COMMAND.send(LidarCommand::Enable).await;
}

/// Ask the task to disable the sensor, without waiting for a reply.
///
/// One-way: clears this mode's need, and the task stops the device, powers the
/// branch down and clears the stale cloud, obstacle flag and cleared edge in the
/// same step. Disabling when not enabled is inert. The outcome is observed by
/// polling [`status`].
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

/// Why a powered period ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StreamEnd {
    /// A disable asked the task to power the sensor down.
    Disabled,
    /// A streaming read ended badly; the branch is off and the task should
    /// power-cycle and bring the device back up.
    Restart,
    /// The device failed and could not be brought up again.
    Failed,
}

/// `LiDAR` driver embassy task.
///
/// Idles (sensor off) until an enable asks for the sensor, runs the bring-up, then
/// streams revolutions until a disable powers it down or the device dies. Runs on
/// **core1** so `UART0_IRQ` is enabled on core1's NVIC.
#[embassy_executor::task]
#[allow(clippy::large_futures)]
pub async fn lidar_task(mut session: LidarSession) {
    let ingest: &'static mut [u8; INGEST_LEN] = INGEST.init([0u8; INGEST_LEN]);
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
        match serve_enabled(
            &mut session.port,
            &mut session.control,
            &mut ingest[..],
            &mut scratch,
            &mut sequence,
            &mut last_obstacle,
        )
        .await
        {
            StreamEnd::Disabled => continue,
            StreamEnd::Failed => {}
            StreamEnd::Restart => {
                // `serve_enabled` never returns this: it restarts internally.
                continue;
            }
        }

        // The device failed, either bringing up or while streaming, and the
        // branch is already powered down. Hold at `Failed` until a disable clears
        // the latch: a duplicate enable cannot spin retries while it is held. The
        // clear is published as `Off`, so a caller resetting a latched failure can
        // observe the reset land before it enables, rather than racing its enable
        // against the stale `Failed`.
        warn!("[lidar] device failed permanently");
        loop {
            match COMMAND.receive().await {
                LidarCommand::Disable => {
                    set_status(LidarStatus::Off);
                    break;
                }
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

/// Serve one enabled period: bring the branch up with bounded retries, stream,
/// and power-cycle to recover from streaming failures.
///
/// Power assertion and UART construction happen outside the attempt timeout; each
/// failed attempt goes through [`quiesce`] and [`PowerControl::complete_power_down`],
/// and the next [`UartResources::power_up`] enforces the minimum off interval. A device that
/// cannot be brought up leaves the status `Failed`, the branch off and stale
/// state cleared.
#[allow(clippy::large_futures)]
async fn serve_enabled(
    port: &mut UartResources,
    control: &mut PowerControl,
    ingest: &mut [u8],
    scratch: &mut Scan,
    sequence: &mut u64,
    last_obstacle: &mut Option<bool>,
) -> StreamEnd {
    let mut started = false;

    loop {
        set_status(LidarStatus::Warming);

        let mut attempt: u8 = 1;
        let (end, power) = {
            let driver = loop {
                let Some(mut candidate) = port.power_up(control, &mut *ingest).await else {
                    break None;
                };
                match with_timeout(WARMUP_TIMEOUT, attempt_bring_up(&mut candidate, scratch)).await {
                    Ok(true) => break Some(candidate),
                    Ok(false) => warn!("[lidar] bring-up attempt {} failed", attempt),
                    Err(_) => warn!("[lidar] bring-up attempt {} timed out", attempt),
                }
                // Power the branch down so the retry starts from a known off-state.
                let power = quiesce(candidate).await;
                control.complete_power_down(port, power);
                if attempt >= BRINGUP_ATTEMPTS {
                    break None;
                }
                attempt += 1;
            };

            let Some(mut driver) = driver else {
                set_status(LidarStatus::Failed);
                clear_stale_state(last_obstacle).await;
                return StreamEnd::Failed;
            };

            set_status(LidarStatus::Streaming);
            if started {
                info!("[lidar] device restarted");
            } else {
                started = true;
                info!("[lidar] streaming");
            }
            // Seed the edge tracker with the current flag so the first scan only
            // raises an event on a real change.
            *last_obstacle = Some(perception::is_obstacle_detected());

            let end = stream_scans(&mut driver, scratch, sequence, last_obstacle).await;
            (end, quiesce(driver).await)
        };
        control.complete_power_down(port, power);
        match end {
            StreamEnd::Disabled => {
                clear_stale_state(last_obstacle).await;
                set_status(LidarStatus::Off);
                return StreamEnd::Disabled;
            }
            StreamEnd::Restart => info!("[lidar] restarting device"),
            StreamEnd::Failed => unreachable!("stream_scans reports restart, not failure"),
        }
    }
}

/// One bring-up attempt: best-effort device-info probe, start (tolerated), and
/// warm up.
///
async fn attempt_bring_up(driver: &mut LidarDriver<'_, '_>, scratch: &mut Scan) -> bool {
    // The device-info frame is emitted on power-up, before this UART exists, and an
    // already-streaming device does not send it again. It is a validation/drain
    // step, not a prerequisite: a missed or malformed frame must not prevent the
    // start command, because the rotor warm-up is the real proof that data flows.
    match with_timeout(PROBE_TIMEOUT, driver.probe()).await {
        Ok(Ok(())) => {}
        Ok(Err(error)) => warn!(
            "[lidar] device-info probe failed (tolerated): {:?}",
            Debug2Format(&error)
        ),
        Err(_) => warn!("[lidar] device-info probe timed out (tolerated)"),
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

/// Stream revolutions until a disable or a failed read ends the powered period.
///
/// Commands are polled between revolutions. A duplicate enable is inert — the
/// sensor is already up. On a disable it returns [`StreamEnd::Disabled`]; the
/// caller, [`serve_enabled`], then quiesces the driver, completes the power-down
/// (which releases the pads), clears the stale cloud and obstacle flag, and
/// publishes [`LidarStatus::Off`]. This function does none of that itself.
///
/// Each revolution is read behind [`STREAM_READ_TIMEOUT`], so the read cannot
/// block indefinitely on a silent link: a driver read error and a wall-clock
/// expiry both return [`StreamEnd::Restart`], leaving the caller to power-cycle
/// the branch and bring the device back up.
async fn stream_scans(
    driver: &mut LidarDriver<'_, '_>,
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
                return StreamEnd::Restart;
            }
            Err(_) => {
                // The driver's byte watchdog only counts bytes that arrive, so
                // it never fires on a silent link. Expiry means no revolution
                // arrived within the wall-clock bound; restart like any error.
                warn!("[lidar] scan read timed out (sensor silent), restarting device");
                return StreamEnd::Restart;
            }
        }
    }
}

/// Map one revolution into a cloud, publish it, and drive the obstacle edge.
async fn publish_scan(scan: &Scan, sequence: u64, last_obstacle: &mut Option<bool>) {
    // Reduce onto the cloud's own slot grid rather than the driver's native 0.9°
    // default: the 1.0° width is deliberately wider than the sensor's sample
    // spacing, so a slot is normally sampled and a `None` slot usually means a
    // measured no-return rather than an unsampled hole (ADR-0029).
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
            detected,
        })
        .await;
        *last_obstacle = Some(detected);
    }

    #[cfg(feature = "telemetry_logs")]
    info!("[lidar] scan {} (obstacle: {})", sequence, detected);
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
            detected: false,
        })
        .await;
    }
    *last_obstacle = None;
}
