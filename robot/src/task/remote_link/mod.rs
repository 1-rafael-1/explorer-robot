//! The core0 Link Task — owner of the Link Port's power, UART and presence.
//!
//! One long-lived embassy task owns the Link Port: UART1, its TX/RX pads
//! (GPIO 36/37), the `EXP_PWR`/`EXP_EN`/`EXP_BOOT` outputs, the `EXP_IRQ`
//! presence input and the static TX/RX rings. It mirrors the `LiDAR` session's
//! retained-resource pattern (ADR-0023, ADR-0032): no UART exists at boot, and
//! [`RemoteLinkSession`] constructs a `BufferedUart` for each powered period and
//! drops it again before the branch powers down.
//!
//! # Lifecycle
//!
//! The task powers the branch at boot and leaves it powered. A one-way
//! [`Channel`] accepts [`LinkCommand::Enable`], [`LinkCommand::Disable`] and
//! [`LinkCommand::BridgeControl`] with the `LiDAR`'s inert-duplicate semantics
//! (ADR-0028): enabling while enabled and disabling while off do nothing. A
//! control command arriving while the port is off is ignored. [`enable`],
//! [`disable`] and [`request_control`] are the non-blocking callers; readiness is
//! observed through the lock-free [`status`], which reports [`LinkStatus::Off`],
//! [`LinkStatus::WaitingForBoard`] or [`LinkStatus::Ready`].
//!
//! Enabling constructs a fresh [`LinkCore`] with a newly minted Boot Id; disabling
//! drops it and clears the link state, so the next enable mints a new Boot Id.
//! A Bridge reset (a high→low edge on `EXP_IRQ`) does **not** reconstruct the
//! core: it resets the RX parser, discards a partially written TX frame, and
//! keeps command dedupe, the event ring and the Boot Id (ADR-0032).
//!
//! # Presence
//!
//! `EXP_IRQ` is a level, not an edge, sampled after a [`PRESENCE_GRACE`]. High
//! means the Bridge board is attached, not that a Remote is connected. A
//! stuck-high line is a normal idle link, not an error. If the line is still low
//! after the grace, the task issues exactly one [`RESET_PULSE`] `EXP_EN` pulse and
//! waits another grace; still low means the board is absent, and no further
//! retries are made. While not ready nothing is transmitted, but sampling, the
//! core tick and pending state continue.
//!
//! # Boot Id
//!
//! The Boot Id comes from the hardware TRNG (ADR-0032), with an uptime/nonce mix
//! as a fallback. A static cell refuses to repeat a value within the process. A
//! Bridge reset keeps the Boot Id; a Link Port power cycle mints a new one.
//!
//! # Telemetry and events
//!
//! The 10 Hz tick samples the six ADR-0031 Streams through [`streams`]; the
//! orchestrator forwards the curated bus Events through [`events`]. The core
//! owns every rate, change and retry rule, so sampling only reports the current
//! value and never blocks. The cloud is encoded under the perception lock and
//! skipped until its sequence advances.
//!
//! # Commands
//!
//! [`commands`] answers each polled command through the same dispatch seam the
//! panel uses, one command at a time and always before the next poll. The catalogue
//! and status queries encode the robot's own Procedure metadata and the Streams
//! mappings, so there is no second table.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use defmt::{Debug2Format, debug, info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_rp::{
    Peri,
    gpio::{Input, Level, Output, Pull},
    mode::Async,
    pac,
    peripherals::{PIN_33, PIN_36, PIN_37, TRNG, UART1},
    trng::{Config as TrngConfig, Trng},
    uart::{self, BufferedUart},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use link_core::{Config as LinkConfig, LinkCore};
use link_protocol::{BridgeControlAction, Caps, MAX_ROBOT_TO_PC_FRAME, Status};
use static_cell::StaticCell;
use touch_ui::LinkAction;

use crate::{resources::RemoteLinkResources, system::state};

mod commands;
mod drive_job;
mod events;
mod failsafe;
mod streams;

#[allow(unused_imports)] // Diagnostic getter for the bench and later tickets.
pub use events::event_drops;
pub use events::forward_event;

// ── Public types ──────────────────────────────────────────────────────────────

/// Lifecycle state of the Link Port, read lock-free from any context.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum LinkStatus {
    /// Powered down; there is no link core and the branch is off.
    Off,
    /// The branch is powered but the Bridge board is not present yet.
    WaitingForBoard,
    /// The Bridge board is attached; the link may carry traffic.
    Ready,
}

// ── Tuning constants ──────────────────────────────────────────────────────────

/// `BufferedUart` TX ring buffer size (ADR-0033: 2 KiB).
const TX_BUF_LEN: usize = 2048;
/// `BufferedUart` RX ring buffer size (ADR-0033: 4 KiB).
const RX_BUF_LEN: usize = 4096;
/// RX read chunk size; one maximum PC-to-robot frame is enough per wake-up.
const RX_CHUNK_LEN: usize = link_protocol::MAX_PC_TO_ROBOT_FRAME;
/// Minimum off interval between power-ups. Provisional, pending board measurement
/// (ADR-0023, ADR-0032).
const POWER_CYCLE_OFF: Duration = Duration::from_millis(200);
/// TPS22917 switch rise time after asserting `EXP_PWR`. Provisional, pending the
/// board's `CT` value (ADR-0023).
const BRANCH_RISE: Duration = Duration::from_millis(50);
/// Grace after power-up before `EXP_IRQ` is sampled, and after each recovery
/// attempt. Provisional, pending board measurement (ADR-0032).
const PRESENCE_GRACE: Duration = Duration::from_secs(1);
/// The one `EXP_EN` reset pulse issued when presence stays low. Provisional,
/// pending the Bridge's reset timing (ADR-0032).
const RESET_PULSE: Duration = Duration::from_millis(20);
/// Bounded wait for the TX shifter to drain during power-down. `flush()` only
/// drains the software ring; `busy()` covers the hardware shifter.
const BUSY_TIMEOUT: Duration = Duration::from_millis(10);
/// Bounded wait for one UART write to make progress before retrying next pass.
const TX_WRITE_TIMEOUT: Duration = Duration::from_millis(50);
/// Bounded wait for the TRNG to produce a Boot Id before falling back.
const TRNG_TIMEOUT: Duration = Duration::from_millis(50);
/// The 10 Hz task tick: presence sampling, the core tick and TX flushing.
const TICK: Duration = Duration::from_millis(100);
/// Number of `EXP_EN` reset pulses issued before declaring the board absent.
const RESET_PULSES: u8 = 1;

// ── Link Core configuration ───────────────────────────────────────────────────

/// The firmware's per-stream ceiling, in centi-Hz.
///
/// Every stream may run at up to 10 Hz, matching the sampler's tick. The core
/// takes the most restrictive of this baseline, a Session's Requested Rate, the
/// firmware's own per-publish hint and any active `THROTTLE`.
const RATE_MAX_CHZ: u16 = 1_000;

/// The firmware's Link Core configuration.
///
/// The baseline of 10 Hz per stream lets a Session request a higher rate
/// without the firmware fetching faster than its tick. The heartbeat array is
/// ADR-0031's: `POWER` and `CALIBRATION` every 5 s, `MOTION` and `ACTIVITY`
/// every 1 s, and `PERCEPTION`/`LIDAR_CLOUD` every 100 ms. The robot advertises
/// `THROTTLE_SUPPORT` because the core honours the Bridge's advisory hints; it
/// advertises no other mode in v1.
fn link_config() -> LinkConfig {
    LinkConfig {
        default_rates_chz: [RATE_MAX_CHZ; link_protocol::Stream::COUNT],
        heartbeat_ms: [5_000, 1_000, 1_000, 100, 100, 5_000],
        robot_caps: Caps::THROTTLE_SUPPORT,
        ..LinkConfig::default()
    }
}

// ── Session ───────────────────────────────────────────────────────────────────

/// Static TX ring buffer, reborrowed for each powered period.
static TX_BUF: StaticCell<[u8; TX_BUF_LEN]> = StaticCell::new();
/// Static RX ring buffer, reborrowed for each powered period.
static RX_BUF: StaticCell<[u8; RX_BUF_LEN]> = StaticCell::new();

/// UART handles and ring buffers, borrowed for one powered period.
struct LinkPort {
    /// UART1 peripheral handle, retained for reconstruction.
    uart: Peri<'static, UART1>,
    /// UART1 TX pad (GPIO 36), retained for reconstruction and release.
    tx: Peri<'static, PIN_36>,
    /// UART1 RX pad (GPIO 37), retained for reconstruction and release.
    rx: Peri<'static, PIN_37>,
    /// Static TX ring buffer, reborrowed on every construction.
    tx_buf: &'static mut [u8; TX_BUF_LEN],
    /// Static RX ring buffer, reborrowed on every construction.
    rx_buf: &'static mut [u8; RX_BUF_LEN],
    /// UART configuration used for every construction.
    uart_config: uart::Config,
}

/// Power, reset and presence lines retained across powered periods.
struct LinkControl {
    /// `EXP_PWR` branch enable (active high, GPIO 32).
    power: Output<'static>,
    /// `EXP_EN` Bridge enable/reset line (active high, GPIO 34).
    en: Output<'static>,
    /// `EXP_BOOT` Bridge boot-mode line (GPIO 35), held low.
    boot: Output<'static>,
    /// `EXP_IRQ` Bridge-ready presence input (GPIO 33).
    irq: Peri<'static, PIN_33>,
    /// Instant of the most recent power-down, for the minimum off interval.
    last_power_down: Option<Instant>,
}

/// Owner of the Link Port's retained resources and power/UART sequencing.
///
/// The session keeps the raw [`Peri`] handles, the `EXP_PWR`/`EXP_EN`/`EXP_BOOT`
/// outputs, the `EXP_IRQ` input handle and the `'static` TX/RX buffers. It
/// constructs no UART at boot; [`LinkPort::power_up`] builds one for the powered
/// period and the task drains and drops it again, so the pads and the UART
/// peripheral exist only while the branch is powered (ADR-0023).
pub struct RemoteLinkSession {
    /// UART handles and ring buffers, borrowed for one powered period.
    port: LinkPort,
    /// Power and presence lines, usable while the UART is borrowed.
    control: LinkControl,
    /// Hardware TRNG used to mint a Boot Id per powered period.
    trng: Trng<'static, Async>,
}

impl RemoteLinkSession {
    /// Create the session from the retained Link Port resources.
    ///
    /// `EXP_PWR`, `EXP_EN` and `EXP_BOOT` start low, the UART pads are held
    /// high-Z and `EXP_IRQ` is an input with no pull, so the off-state invariant
    /// holds before the first power-down. No UART is constructed.
    #[must_use]
    pub fn new(pins: RemoteLinkResources, trng: Peri<'static, TRNG>) -> Self {
        let RemoteLinkResources {
            uart,
            tx_dma: _,
            rx_dma: _,
            power_enable,
            mut irq,
            en,
            boot,
            mut tx,
            mut rx,
        } = pins;

        // Hold both UART pads high-Z with no pull from boot, before the branch
        // is ever powered.
        drop(Input::new(tx.reborrow(), Pull::None));
        drop(Input::new(rx.reborrow(), Pull::None));
        // `EXP_IRQ` is an input with no pull while the branch is off.
        drop(Input::new(irq.reborrow(), Pull::None));

        let mut uart_config = uart::Config::default();
        uart_config.baudrate = link_core::timing::BAUD;

        Self {
            port: LinkPort {
                uart,
                tx,
                rx,
                tx_buf: TX_BUF.init([0; TX_BUF_LEN]),
                rx_buf: RX_BUF.init([0; RX_BUF_LEN]),
                uart_config,
            },
            control: LinkControl {
                power: Output::new(power_enable, Level::Low),
                en: Output::new(en, Level::Low),
                boot: Output::new(boot, Level::Low),
                irq,
                last_power_down: None,
            },
            trng: Trng::new(trng, crate::Irqs, TrngConfig::default()),
        }
    }
}

impl LinkPort {
    /// Power the branch and build the UART for the powered period.
    ///
    /// Runs ADR-0023's order: wait out the minimum off interval, configure RX
    /// with a pull-up and `EXP_IRQ` with a pull-down, assert `EXP_PWR`, wait for
    /// the TPS22917 rise, drive `EXP_EN` high, then construct the 921600 8N1
    /// `BufferedUart` from the retained handles and reborrowed static rings.
    /// There is no await point between construction and returning, so
    /// cancellation cannot leave a muxed UART owned by nobody.
    async fn power_up(&mut self, control: &mut LinkControl) -> BufferedUart<'_> {
        if let Some(last) = control.last_power_down {
            let elapsed = Instant::now().saturating_duration_since(last);
            if let Some(remaining) = POWER_CYCLE_OFF.checked_sub(elapsed) {
                Timer::after(remaining).await;
            }
        }

        // Configure the pads before the branch is powered: an idle RX is pulled
        // high, and the Bridge's presence line is pulled low while it is absent.
        drop(Input::new(self.rx.reborrow(), Pull::Up));
        drop(Input::new(control.irq.reborrow(), Pull::Down));

        // Assert the branch, wait for the switch rise, then enable the Bridge.
        control.power.set_high();
        Timer::after(BRANCH_RISE).await;
        control.en.set_high();

        let uart = BufferedUart::new(
            self.uart.reborrow(),
            self.tx.reborrow(),
            self.rx.reborrow(),
            crate::Irqs,
            &mut self.tx_buf[..],
            &mut self.rx_buf[..],
            self.uart_config,
        );

        // `BufferedUart::new` writes the RX pad control from reset, which clears
        // the pull-up configured above. Re-apply it after construction, leaving
        // the UART's function select and input enable untouched.
        pac::PADS_BANK0.gpio(37).modify(|w| {
            w.set_pue(true);
            w.set_pde(false);
        });

        uart
    }

    /// Release both pads to high-Z inputs with no pull, so neither line can
    /// source current into the unpowered branch.
    fn release_pads(&mut self) {
        drop(Input::new(self.tx.reborrow(), Pull::None));
        drop(Input::new(self.rx.reborrow(), Pull::None));
    }
}

impl LinkControl {
    /// Read the `EXP_IRQ` presence level with the powered-period pull-down.
    fn presence_high(&mut self) -> bool {
        Input::new(self.irq.reborrow(), Pull::Down).is_high()
    }

    /// Issue exactly one bounded `EXP_EN` reset pulse, following the active-high
    /// polarity: drive low, hold, then drive high.
    async fn pulse_enable(&mut self) {
        self.en.set_low();
        Timer::after(RESET_PULSE).await;
        self.en.set_high();
    }

    /// Deinitialize the peripheral and deassert the branch after the UART has
    /// been dropped.
    ///
    /// Clears `UARTCR.EN`/`TXE`/`RXE`, drives `EXP_EN` and `EXP_BOOT` low,
    /// releases `EXP_IRQ`, deasserts `EXP_PWR`, and records the power-down
    /// instant.
    fn complete_power_down(&mut self) {
        pac::UART1.uartcr().modify(|w| {
            w.set_uarten(false);
            w.set_txe(false);
            w.set_rxe(false);
        });

        self.en.set_low();
        self.boot.set_low();
        drop(Input::new(self.irq.reborrow(), Pull::None));

        self.power.set_low();
        self.last_power_down = Some(Instant::now());
    }
}

/// Drain the UART's software ring and hardware shifter, then drop it.
///
/// Every wait is bounded, so a dead or silent peer cannot hang teardown.
async fn drain_uart(mut uart: BufferedUart<'_>) {
    if with_timeout(BUSY_TIMEOUT, uart.flush()).await.is_err() {
        warn!("[link] TX ring did not flush within {:?}", BUSY_TIMEOUT);
    }
    let drained = with_timeout(BUSY_TIMEOUT, async {
        while uart.busy() {
            Timer::after(Duration::from_micros(100)).await;
        }
    })
    .await;
    if drained.is_err() {
        warn!("[link] UART shifter did not drain within {:?}", BUSY_TIMEOUT);
    }

    // Drop the UART: this deinits its ring buffers and disables `UART1_IRQ`.
    drop(uart);
}

/// Mint a Boot Id from the TRNG, falling back to an uptime/nonce mix.
///
/// The value is forced unique within the process by [`unique_boot_id`], so a
/// failed or repeated sample cannot collide with an earlier powered period.
async fn mint_boot_id(trng: &mut Trng<'static, Async>) -> u32 {
    let mut bytes = [0u8; 4];
    let candidate = if with_timeout(TRNG_TIMEOUT, trng.fill_bytes(&mut bytes)).await.is_ok() {
        u32::from_ne_bytes(bytes)
    } else {
        warn!("[link] TRNG timed out; using the uptime fallback for the Boot Id");
        fallback_boot_id()
    };
    unique_boot_id(candidate)
}

// ── Status cell (lock-free) ───────────────────────────────────────────────────

/// Encoded value of [`LinkStatus::Off`] in [`LINK_STATUS`].
const STATUS_OFF: u8 = 0;
/// Encoded value of [`LinkStatus::WaitingForBoard`] in [`LINK_STATUS`].
const STATUS_WAITING: u8 = 1;
/// Encoded value of [`LinkStatus::Ready`] in [`LINK_STATUS`].
const STATUS_READY: u8 = 2;

/// Current lifecycle state, read lock-free from any context.
static LINK_STATUS: AtomicU8 = AtomicU8::new(STATUS_OFF);

// ── Boot Id uniqueness ────────────────────────────────────────────────────────

/// The last Boot Id minted, used to refuse an in-process repeat.
static LAST_BOOT_ID: AtomicU32 = AtomicU32::new(0);
/// Per-process nonce mixed into the fallback Boot Id.
static FALLBACK_NONCE: AtomicU32 = AtomicU32::new(0);

// ── Command channel ───────────────────────────────────────────────────────────

/// Commands accepted by the Link Task.
///
/// [`enable`] and [`disable`] are the bench-mode lifecycle seam; the panel calls
/// [`request_control`] for the Bridge Control actions.
#[derive(Debug, Clone, Copy)]
enum LinkCommand {
    /// Power the port on and construct the link core, if it is not already on.
    Enable,
    /// Drop the link core and power the port down, if it is not already off.
    Disable,
    /// Send one Bridge Control request for a panel action.
    BridgeControl(LinkAction),
}

/// Command channel to the Link Task (capacity 4).
static COMMAND: Channel<CriticalSectionRawMutex, LinkCommand, 4> = Channel::new();

// ── Public API ────────────────────────────────────────────────────────────────

/// Ask the task to enable the Link Port, without waiting.
///
/// One-way: readiness is observed by polling [`status`]. Enabling while already
/// enabled is inert, so there is no reference count and no extra power event.
#[allow(dead_code)] // Bench-mode lifecycle seam; no panel caller in v1.
pub async fn enable() {
    COMMAND.send(LinkCommand::Enable).await;
}

/// Ask the task to disable the Link Port, without waiting.
///
/// One-way: the task drops the link core, powers the branch down and publishes
/// [`LinkStatus::Off`]. Disabling while off is inert.
#[allow(dead_code)] // Bench-mode lifecycle seam; no panel caller in v1.
pub async fn disable() {
    COMMAND.send(LinkCommand::Disable).await;
}

/// Ask the task to send one Bridge Control request, without waiting.
///
/// The command is queued with `try_send`, so the caller never blocks on the Link
/// Task. Returns whether the command was accepted; a full channel drops the
/// request rather than queueing behind it. A request while another is in flight is
/// refused by the core, not queued, and leaves the panel's outcome untouched.
#[allow(clippy::unused_async)] // Callers await this one-way seam; it never blocks.
pub async fn request_control(action: LinkAction) -> bool {
    COMMAND.try_send(LinkCommand::BridgeControl(action)).is_ok()
}

/// Spawn the long-lived Remote drive job once, at boot.
///
/// The job outlives Link Port enable/disable periods, so it is spawned beside the
/// Link Task rather than inside one powered period.
#[allow(clippy::unwrap_used)]
pub fn spawn_drive_job(spawner: Spawner) {
    spawner.spawn(drive_job::drive_job_task().unwrap());
}

/// Current lifecycle state. Lock-free (an `AtomicU8`), safe from any context.
#[allow(dead_code)] // Bench-mode observation; no panel caller in v1.
#[must_use]
pub fn status() -> LinkStatus {
    match LINK_STATUS.load(Ordering::Relaxed) {
        STATUS_WAITING => LinkStatus::WaitingForBoard,
        STATUS_READY => LinkStatus::Ready,
        _ => LinkStatus::Off,
    }
}

// ── Embassy task ──────────────────────────────────────────────────────────────

/// The Link Port task.
///
/// Powers the branch at boot, constructs the link core, then serves the port
/// until a disable. Runs on **core0** so `UART1_IRQ` and `TRNG_IRQ` are enabled
/// on core0's NVIC. It is additive: every failure logs and leaves the port off
/// or waiting, and it never blocks boot or the panel.
#[embassy_executor::task]
#[allow(clippy::large_futures)]
pub async fn remote_link_task(mut session: RemoteLinkSession) {
    info!("[link] task started on core0");

    // The port powers up at boot; subsequent iterations wait for an enable.
    let mut auto_enable = true;

    loop {
        if !auto_enable {
            match COMMAND.receive().await {
                LinkCommand::Enable => {}
                LinkCommand::Disable | LinkCommand::BridgeControl(_) => continue,
            }
        }
        auto_enable = false;

        let uart = session.port.power_up(&mut session.control).await;
        let boot_id = mint_boot_id(&mut session.trng).await;
        let mut core = LinkCore::new(link_config(), boot_id, now_ms());
        set_status(LinkStatus::WaitingForBoard);
        info!("[link] port powered, boot id {=u32}", boot_id);

        let uart = serve_enabled(&mut session.control, &mut core, uart).await;
        drain_uart(uart).await;
        session.port.release_pads();
        session.control.complete_power_down();
        set_status(LinkStatus::Off);
        state::link::clear().await;
    }
}

// ── Enabled-period loop ───────────────────────────────────────────────────────

/// Presence sampling state for one powered period.
#[derive(Debug, Clone, Copy)]
enum PresencePhase {
    /// Powered, sampling after a grace; a reset pulse may still be pending.
    Waiting,
    /// The Bridge board is attached.
    Ready,
    /// The board stayed low after the one reset pulse; not retrying.
    Absent,
}

/// Tracks `EXP_IRQ` presence across the powered period.
struct PresenceWatch {
    /// Current presence phase.
    phase: PresencePhase,
    /// Reset pulses still allowed in this attempt.
    pulses_left: u8,
    /// When the next level sample in [`PresencePhase::Waiting`] is due.
    deadline: Instant,
}

impl PresenceWatch {
    /// Start waiting for the Bridge after a power-up.
    fn new(now: Instant) -> Self {
        Self {
            phase: PresencePhase::Waiting,
            pulses_left: RESET_PULSES,
            deadline: now + PRESENCE_GRACE,
        }
    }

    /// Whether the link may transmit.
    const fn is_ready(&self) -> bool {
        matches!(self.phase, PresencePhase::Ready)
    }

    /// Sample the presence level and advance the phase machine.
    ///
    /// A high→low edge while ready is a Bridge reset: the RX parser and any
    /// partially written TX frame are discarded, command dedupe and the event
    /// ring survive, and the link returns to waiting. A still-low line after the
    /// grace gets one [`RESET_PULSE`]; still low after another grace means absent,
    /// with no further retry.
    async fn update(&mut self, control: &mut LinkControl, core: &mut LinkCore, scratch: &mut TxScratch, now: Instant) {
        match self.phase {
            PresencePhase::Waiting => {
                if now < self.deadline {
                    return;
                }
                if control.presence_high() {
                    self.phase = PresencePhase::Ready;
                    set_status(LinkStatus::Ready);
                    info!("[link] bridge present");
                } else if self.pulses_left > 0 {
                    self.pulses_left -= 1;
                    info!("[link] bridge absent; issuing the reset pulse");
                    control.pulse_enable().await;
                    self.deadline = Instant::now() + PRESENCE_GRACE;
                } else {
                    self.phase = PresencePhase::Absent;
                    info!("[link] bridge absent; not retrying");
                }
            }
            PresencePhase::Ready => {
                if !control.presence_high() {
                    info!("[link] bridge reset");
                    core.reset_parser();
                    scratch.reset();
                    self.phase = PresencePhase::Waiting;
                    self.pulses_left = RESET_PULSES;
                    self.deadline = now + PRESENCE_GRACE;
                    set_status(LinkStatus::WaitingForBoard);
                }
            }
            PresencePhase::Absent => {
                if control.presence_high() {
                    self.phase = PresencePhase::Ready;
                    set_status(LinkStatus::Ready);
                    info!("[link] bridge present");
                }
            }
        }
    }
}

/// A partially written robot-to-PC frame.
///
/// `next_frame` builds a whole frame; the UART ring may accept it only in part.
/// Keeping the offset lets the next pass finish it, and [`Self::reset`] is the
/// "discard a partially written TX frame" half of a Bridge reset.
struct TxScratch {
    /// The encoded frame bytes.
    buf: [u8; MAX_ROBOT_TO_PC_FRAME],
    /// Number of valid bytes in [`Self::buf`].
    len: usize,
    /// Number of bytes already handed to the UART ring.
    written: usize,
}

impl TxScratch {
    /// Create an empty scratch frame.
    const fn new() -> Self {
        Self {
            buf: [0; MAX_ROBOT_TO_PC_FRAME],
            len: 0,
            written: 0,
        }
    }

    /// Whether the current frame has been fully written.
    const fn is_empty(&self) -> bool {
        self.written >= self.len
    }

    /// Drop any partly written frame.
    const fn reset(&mut self) {
        self.len = 0;
        self.written = 0;
    }
}

/// Serve one enabled period until a disable powers the port down.
///
/// Selects over the command channel, UART RX and a 10 Hz ticker. RX bytes go to
/// [`LinkCore::on_rx`], the tick drives [`LinkCore::tick`], the Stream sampler and
/// presence sampling, accepted Events are published into the core on every pass,
/// polled items are drained one at a time, and frames are flushed while the board
/// is present. A disable powers the branch down and returns; the caller's
/// [`LinkCore`] is dropped then.
#[allow(clippy::large_futures)]
async fn serve_enabled<'a>(
    control: &mut LinkControl,
    core: &mut LinkCore,
    mut uart: BufferedUart<'a>,
) -> BufferedUart<'a> {
    streams::reset();
    // Disable/Enable starts clean: the panel reads Off until the first tick
    // publishes this powered period's presence and peer.
    state::link::clear().await;
    let mut presence = PresenceWatch::new(Instant::now());
    let mut scratch = TxScratch::new();
    let mut ticker = Ticker::every(TICK);
    let mut rx = [0u8; RX_CHUNK_LEN];
    // The last values published to the link state, so a quiet tick writes
    // nothing and a Bridge reset keeps the retained Bridge Status.
    let mut published = PublishedLink::default();

    loop {
        match select3(COMMAND.receive(), uart.read(&mut rx), ticker.next()).await {
            Either3::First(command) => match command {
                // Already enabled; a duplicate is inert.
                LinkCommand::Enable => {}
                LinkCommand::Disable => {
                    // Deliberate power-down fires the same failsafe as peer loss,
                    // before the branch goes down and the core is dropped.
                    failsafe::link_lost();
                    return uart;
                }
                LinkCommand::BridgeControl(action) => {
                    handle_bridge_control(core, action, presence.is_ready()).await;
                }
            },
            Either3::Second(result) => match result {
                Ok(read) if read > 0 => core.on_rx(&rx[..read], now_ms()),
                Ok(_) => {}
                Err(error) => warn!("[link] UART receive error: {:?}", Debug2Format(&error)),
            },
            Either3::Third(()) => {
                let now = Instant::now();
                let now_ms = now.as_millis();
                // Sample immediately after the tick so the emission schedule is
                // evaluated at the same instant as the rest of the link state.
                core.tick(now_ms);
                // The core owns the Deadman threshold and its one-shot edge; the
                // firmware only reacts to it, and only for Remote-originated work.
                if core.take_peer_lost() {
                    failsafe::link_lost();
                }
                streams::sample_and_publish(core, now_ms).await;
                presence.update(control, core, &mut scratch, now).await;
                publish_link_state(&presence, core, &mut published, now_ms).await;
            }
        }

        events::drain(core, now_ms());
        commands::handle(core, now_ms()).await;
        flush_tx(core, &mut uart, &mut scratch, presence.is_ready()).await;
    }
}

/// Send one Bridge Control request for a panel action.
///
/// `ForgetNetwork` fires the Deadman first, because it deliberately kills the
/// station link; a Remote-originated claim must stand down before the link goes
/// away. Showing and rotating the pairing code stop nothing, so they never call
/// the failsafe. A request while the board is not ready is dropped: the core
/// would queue it without starting its timeout, and the panel would stay
/// `Pending`. The core refuses a second request while one is in flight, and that
/// refusal is deliberately silent state-wise: the panel's pending outcome is left
/// alone rather than replaced. The request itself is emitted later by [`flush_tx`],
/// once the board is present.
async fn handle_bridge_control(core: &mut LinkCore, action: LinkAction, ready: bool) {
    if !ready {
        debug!("[link] bridge control ignored: the board is not ready");
        return;
    }

    if matches!(action, LinkAction::ForgetNetwork) {
        failsafe::link_lost();
    }

    match core.request_bridge_control(map_link_action(action).bits(), now_ms()) {
        Ok(_) => state::link::set_control(link_state::ControlOutcome::Pending).await,
        // One request in flight: refused, not queued, and the outcome stays as it is.
        Err(Status::Busy) => {}
        Err(status) => warn!("[link] bridge control request refused: {:?}", Debug2Format(&status)),
    }
}

/// Map a panel action onto the wire action it requests.
const fn map_link_action(action: LinkAction) -> BridgeControlAction {
    match action {
        LinkAction::ForgetNetwork => BridgeControlAction::ForgetNetwork,
        LinkAction::ShowPairingCode => BridgeControlAction::ShowPairingCode,
        LinkAction::RotatePairingCode => BridgeControlAction::RotatePairingCode,
    }
}

/// Flush queued frames to the UART while the board is present.
///
/// When no frame is being written, asks [`LinkCore::next_frame`] for the next
/// one; when the ring is full, defers the remainder to the next pass. A partial
/// write is retained in `scratch` so a frame is never silently truncated.
async fn flush_tx(core: &mut LinkCore, uart: &mut BufferedUart<'_>, scratch: &mut TxScratch, ready: bool) {
    loop {
        if scratch.is_empty() {
            if !ready {
                return;
            }
            let Some(len) = core.next_frame(&mut scratch.buf) else {
                return;
            };
            scratch.len = len;
            scratch.written = 0;
        }

        let remaining = &scratch.buf[scratch.written..scratch.len];
        match with_timeout(TX_WRITE_TIMEOUT, uart.write(remaining)).await {
            // The ring is full, or another pass made no progress: retry next time.
            Ok(Ok(0)) | Err(_) => return,
            Ok(Ok(written)) => scratch.written += written,
            Ok(Err(error)) => {
                warn!("[link] UART transmit error: {:?}", Debug2Format(&error));
                scratch.reset();
                return;
            }
        }
    }
}

// ── Link state publication ────────────────────────────────────────────────────

/// Map the presence watch onto the neutral presence the panel shows.
///
/// A powered port that is not ready — still waiting, or known absent — reads as
/// [`link_state::Presence::WaitingForBoard`]; only an attached Bridge is
/// [`link_state::Presence::Ready`].
const fn map_presence(presence: &PresenceWatch) -> link_state::Presence {
    if presence.is_ready() {
        link_state::Presence::Ready
    } else {
        link_state::Presence::WaitingForBoard
    }
}

/// Map the Link Core's peer lease onto the neutral peer state.
const fn map_peer(peer: link_core::PeerState) -> link_state::Peer {
    match peer {
        link_core::PeerState::NoPeer => link_state::Peer::NoPeer,
        link_core::PeerState::PeerLive => link_state::Peer::PeerLive,
    }
}

/// Map the wire Bridge Station onto the neutral one the panel renders.
const fn map_station(station: link_protocol::BridgeStation) -> link_state::Station {
    match station {
        link_protocol::BridgeStation::Down => link_state::Station::Down,
        link_protocol::BridgeStation::Connecting => link_state::Station::Connecting,
        link_protocol::BridgeStation::Associated => link_state::Station::Associated,
    }
}

/// Map the wire Bridge Status onto the neutral one the panel renders.
const fn map_bridge_status(status: link_protocol::BridgeStatus) -> link_state::BridgeStatus {
    link_state::BridgeStatus {
        rssi_dbm: status.rssi_dbm,
        channel: status.channel,
        station: map_station(status.station),
        flags: link_state::BridgeFlags {
            provisioned: status.flags.contains(link_protocol::BridgeStatusFlags::PROVISIONED),
            setup_open: status.flags.contains(link_protocol::BridgeStatusFlags::SETUP_OPEN),
            link_ready: status.flags.contains(link_protocol::BridgeStatusFlags::LINK_READY),
        },
        uart_dropped_bytes: status.uart_dropped_bytes,
        dma_stalls: status.dma_stalls,
        crc_failures: status.crc_failures,
        resyncs: status.resyncs,
        wdt_feeds: status.wdt_feeds,
        wdt_withheld: status.wdt_withheld,
    }
}

/// Publish the tick's presence, peer and any new Bridge Status.
///
/// Each value is written only when it differs from the last one written, so a
/// quiet tick touches no state. A byte-identical Bridge Status is conservatively
/// treated as no refresh: the Bridge's counters change every second in practice,
/// so a silent Bridge is the case where the timestamp must not be renewed, and
/// the retained status goes stale after [`link_state::STALE_AFTER_MS`]. A Bridge
/// reset only changes presence and peer, so the retained Bridge Status survives
/// it. The link-state lock is taken per call and never held across an await.
/// The values last written into `state::link` for this powered period.
///
/// Presence and peer are written on change; the Bridge Status is written when
/// the core's receipt counter advances, which includes a byte-identical 1 Hz
/// repeat and leaves the retained status untouched when the Bridge goes silent.
#[derive(Default)]
struct PublishedLink {
    /// The last presence written.
    presence: Option<link_state::Presence>,
    /// The last peer state written.
    peer: Option<link_state::Peer>,
    /// The Bridge Status receipt counter at the last write.
    bridge_updates: Option<u32>,
}

/// Publish the changed presence, peer and Bridge Status into `state::link`.
///
/// Presence and peer are written on change; the Bridge Status is written when the
/// core's receipt counter advances. The state lock is released before returning
/// and no UART/power await is held across it.
async fn publish_link_state(presence: &PresenceWatch, core: &LinkCore, published: &mut PublishedLink, now_ms: u64) {
    let presence_now = map_presence(presence);
    if published.presence != Some(presence_now) {
        state::link::set_presence(presence_now).await;
        published.presence = Some(presence_now);
    }

    let peer_now = map_peer(core.peer());
    if published.peer != Some(peer_now) {
        state::link::set_peer(peer_now).await;
        published.peer = Some(peer_now);
    }

    // The receipt counter advances on every accepted snapshot, including a byte-identical
    // repeat. Keying the write on it, rather than on the payload, keeps the Bridge's 1 Hz
    // cadence as the freshness signal: a silent Bridge goes stale after 2 s even when its last
    // counters happened to repeat.
    let updates = core.bridge_status_updates();
    if let Some(status) = core.bridge_status()
        && published.bridge_updates != Some(updates)
    {
        state::link::set_bridge_status(map_bridge_status(status), now_ms).await;
        published.bridge_updates = Some(updates);
    }
}

// ── Private helpers ───────────────────────────────────────────────────────────

/// Store a new lifecycle state in the lock-free status cell.
fn set_status(status: LinkStatus) {
    let encoded = match status {
        LinkStatus::Off => STATUS_OFF,
        LinkStatus::WaitingForBoard => STATUS_WAITING,
        LinkStatus::Ready => STATUS_READY,
    };
    LINK_STATUS.store(encoded, Ordering::Relaxed);
}

/// Current uptime in milliseconds.
fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// An uptime/per-boot-nonce fallback Boot Id.
///
/// Used only when the TRNG does not answer within [`TRNG_TIMEOUT`]. The nonce
/// advances on every fallback, so two mints in the same millisecond still differ.
fn fallback_boot_id() -> u32 {
    let nonce = FALLBACK_NONCE.fetch_add(1, Ordering::Relaxed);
    let uptime = u32::try_from(Instant::now().as_millis()).unwrap_or(u32::MAX);
    uptime.wrapping_mul(0x9E37_79B9) ^ nonce.rotate_left(13)
}

/// Force `candidate` to differ from the last Boot Id minted.
///
/// Zero is remapped to one so the value is always non-zero. Only the Link Task
/// mints Boot Ids, so a plain load/store pair is sufficient.
fn unique_boot_id(mut candidate: u32) -> u32 {
    if candidate == 0 {
        candidate = 1;
    }
    let last = LAST_BOOT_ID.load(Ordering::Relaxed);
    while candidate == last {
        candidate = candidate.wrapping_add(1);
        if candidate == 0 {
            candidate = 1;
        }
    }
    LAST_BOOT_ID.store(candidate, Ordering::Relaxed);
    candidate
}
