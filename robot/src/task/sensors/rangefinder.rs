//! On-demand VL53L1X rangefinder pair task and floor-drop detection.
//!
//! One task owns both sensors, the command channel and a 50 ms ticker. It arms
//! the pair on demand, reads both sensors through their `INT` lines, recovers a
//! wedged sensor with bounded retries, publishes the panel-only telemetry and is
//! the sole writer of `FLOOR_DROP` (the OR of the two per-sensor floor states).
//!
//! The drive path arms the pair through [`arm_for_motion`] before every movement
//! action. Motion needs both sensors armed and a clear floor-drop flag; a failed
//! or degraded pair refuses motion rather than driving blind.
//!
//! Disable on an explicit `Idle`/`Standby` or when an Activity ends. Do not
//! disable on an `EmergencyBrake`: the pair stays armed and a drop stays visible.

use core::{
    pin::pin,
    sync::atomic::{AtomicU8, AtomicU32, Ordering},
};

use defmt::{Debug2Format, info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::{Either4, select4};
use embassy_rp::{
    gpio::{Input, Level, OutputOpenDrain, Pull},
    i2c::I2c,
    mode::Async,
    peripherals::{PIN_1, PIN_2, PIN_3, PIN_31},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use rangefinder::{
    DROP_THRESHOLD_MM, GATE_TIMEOUT_MS, GateRefusal, PairState, RANGING_PERIOD_MS, Rangefinder, RangefinderReading,
    SensorLifecycle, SensorState, gate, pair_state,
};

use crate::{
    I2c1BusShared, I2cBusShared,
    system::{
        event::{Events, raise_event},
        state::{
            perception::{self, ChangeDetected},
            rangefinder::{RangefinderTelemetry, SensorTelemetry},
        },
    },
};

/// One VL53L1X on its dedicated I2C bus, with open-drain `XSHUT` and `INT`.
type SensorDriver = Rangefinder<
    I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
    OutputOpenDrain<'static>,
    Input<'static>,
>;

/// The two sensors and their pins, built once at boot.
pub struct RangefinderSession {
    /// The front sensor.
    front: SensorDriver,
    /// The rear sensor.
    rear: SensorDriver,
}

impl RangefinderSession {
    /// Build the two sensor wrappers, holding both modules off via `XSHUT`.
    ///
    /// The buses must have been initialised on core0 so their interrupts are
    /// enabled on core0's NVIC.
    #[must_use]
    pub fn new(
        front_bus: &'static I2cBusShared,
        rear_bus: &'static I2c1BusShared,
        front_xshut: embassy_rp::Peri<'static, PIN_1>,
        front_interrupt: embassy_rp::Peri<'static, PIN_2>,
        rear_xshut: embassy_rp::Peri<'static, PIN_3>,
        rear_interrupt: embassy_rp::Peri<'static, PIN_31>,
    ) -> Self {
        Self {
            front: Rangefinder::new(
                I2cDevice::new(front_bus),
                OutputOpenDrain::new(front_xshut, Level::Low),
                Input::new(front_interrupt, Pull::Up),
            ),
            rear: Rangefinder::new(
                I2cDevice::new(rear_bus),
                OutputOpenDrain::new(rear_xshut, Level::Low),
                Input::new(rear_interrupt, Pull::Up),
            ),
        }
    }
}

// ── Pair status (lock-free) ───────────────────────────────────────────────────

/// Current pair state, read lock-free by the drive gate.
static PAIR_STATE: AtomicU8 = AtomicU8::new(0);

/// Next enable-request id. Every enable carries one so the gate can tell a
/// fresh arming request from an `Armed` state left over from the previous
/// enabled period.
static NEXT_REQUEST: AtomicU32 = AtomicU32::new(1);

/// The most recent enable-request id the task has begun serving.
static LAST_ENABLE_REQUEST: AtomicU32 = AtomicU32::new(0);

/// Commands accepted by the rangefinder task.
#[derive(Debug, Clone, Copy)]
enum RangefinderCommand {
    /// Arm the pair, if it is not already armed; carries the request id.
    Enable(u32),
    /// Power the pair down and clear its telemetry.
    Disable,
}

/// Command channel to the rangefinder task (capacity 4).
static COMMAND: Channel<CriticalSectionRawMutex, RangefinderCommand, 4> = Channel::new();

// ── Public API ────────────────────────────────────────────────────────────────

/// Ask the task to arm the pair, without waiting for it to become ready.
///
/// One-way and idempotent: a duplicate enable is inert. A latched
/// [`PairState::Failed`] is released first with a disable/enable pair, so a
/// caller that retries after fixing the hardware gets a fresh bring-up.
///
/// Returns the request id. [`arm_for_motion`] waits until the task has begun
/// serving this request, so an `Armed` state from an earlier enabled period
/// cannot satisfy a gate whose `Disable` is still queued.
#[allow(clippy::must_use_candidate)]
pub async fn enable() -> u32 {
    let request = NEXT_REQUEST.fetch_add(1, Ordering::Relaxed);
    if status() == PairState::Failed {
        COMMAND.send(RangefinderCommand::Disable).await;
    }
    COMMAND.send(RangefinderCommand::Enable(request)).await;
    request
}

/// Ask the task to disarm and power down the pair.
///
/// One-way: the task stops ranging, holds `XSHUT` low and clears the telemetry.
/// Disabling when the pair is already off is inert.
pub async fn disable() {
    COMMAND.send(RangefinderCommand::Disable).await;
}

/// Current pair state. Lock-free, safe from any context.
#[must_use]
pub fn status() -> PairState {
    PairState::from_u8(PAIR_STATE.load(Ordering::Relaxed))
}

/// Arm the pair and apply the Floor-Drop Gate before a movement action.
///
/// Enables the pair, waits up to [`GATE_TIMEOUT_MS`] for the task to begin
/// serving this enable request and for both sensors to arm, then requires a
/// clear floor-drop flag. A missing, degraded or failed pair fails with
/// [`GateRefusal::NotReady`]; a drop already present fails with
/// [`GateRefusal::FloorDrop`].
///
/// # Errors
///
/// Returns the [`GateRefusal`] reason the drive completion reports.
pub async fn arm_for_motion() -> Result<(), GateRefusal> {
    let request = enable().await;

    let deadline = Instant::now() + Duration::from_millis(GATE_TIMEOUT_MS);
    loop {
        let state = status();
        let floor_drop = perception::is_floor_drop_detected();
        let acknowledged = LAST_ENABLE_REQUEST.load(Ordering::Relaxed) >= request;
        if acknowledged && (state == PairState::Armed || Instant::now() >= deadline) {
            return gate(state, floor_drop);
        }
        if Instant::now() >= deadline {
            return gate(state, floor_drop);
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}

// ── Embassy task ──────────────────────────────────────────────────────────────

/// Why an enabled period ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EnabledEnd {
    /// A disable asked the task to power the pair down.
    Disabled,
    /// Both sensors failed and could not be brought up again.
    Failed,
}

/// Whether the task should keep serving the enabled period.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Maintenance {
    /// Keep serving the pair.
    Continue,
    /// A disable arrived during recovery; power down and end the period.
    Disabled,
}

/// How one recovery attempt ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AttemptOutcome {
    /// The reset succeeded.
    Recovered,
    /// The reset failed; try the next attempt.
    Failed,
    /// A disable arrived; stop recovering.
    Disabled,
}

/// How a sensor's recovery ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RecoveryEnd {
    /// The reset succeeded.
    Recovered,
    /// Every attempt failed and the sensor latched failed.
    Failed,
    /// A disable arrived during recovery.
    Disabled,
}

/// VL53L1X pair task.
///
/// Idles with both modules held off until an enable asks for the pair, then
/// serves one enabled period until a disable or a terminal failure.
#[embassy_executor::task]
#[allow(clippy::large_futures)]
pub async fn rangefinder_task(session: RangefinderSession) {
    let RangefinderSession { mut front, mut rear } = session;
    let mut front_runtime = SensorRuntime::new();
    let mut rear_runtime = SensorRuntime::new();
    let mut delay = Delay;

    info!("[rangefinder] task started (pair off)");

    loop {
        match COMMAND.receive().await {
            RangefinderCommand::Enable(request) => {
                LAST_ENABLE_REQUEST.store(request, Ordering::Relaxed);
            }
            RangefinderCommand::Disable => continue,
        }

        info!("[rangefinder] enable requested");
        match serve_enabled(&mut front, &mut rear, &mut front_runtime, &mut rear_runtime, &mut delay).await {
            EnabledEnd::Disabled => {}
            EnabledEnd::Failed => {
                warn!("[rangefinder] pair failed permanently");
                hold_failed().await;
            }
        }
    }
}

// ── Enabled period ────────────────────────────────────────────────────────────

/// Serve one enabled period: bring both sensors up, stream, recover, publish.
#[allow(clippy::large_futures)]
async fn serve_enabled(
    front: &mut SensorDriver,
    rear: &mut SensorDriver,
    front_runtime: &mut SensorRuntime,
    rear_runtime: &mut SensorRuntime,
    delay: &mut Delay,
) -> EnabledEnd {
    let now = now_ms();
    front_runtime.reset_for_enable(now);
    rear_runtime.reset_for_enable(now);
    set_pair_state(PairState::Arming);
    publish(front_runtime, rear_runtime, PairState::Arming, false).await;

    let front_ok = init_sensor(front, front_runtime, delay).await;
    let rear_ok = init_sensor(rear, rear_runtime, delay).await;

    // Hold a sensor that could not come up off, so a floating INT line cannot
    // spin the task while the healthy sensor keeps streaming.
    if !front_ok {
        power_down_one(front, "front").await;
    }
    if !rear_ok {
        power_down_one(rear, "rear").await;
    }

    if !front_ok && !rear_ok {
        set_pair_state(PairState::Failed);
        publish(front_runtime, rear_runtime, PairState::Failed, false).await;
        power_down(front, rear).await;
        return EnabledEnd::Failed;
    }

    let initial = pair_state(true, front_runtime.lifecycle.state(), rear_runtime.lifecycle.state());
    let initial_drop = front_runtime.lifecycle.is_drop() || rear_runtime.lifecycle.is_drop();
    set_pair_state(initial);
    publish(front_runtime, rear_runtime, initial, initial_drop).await;

    let mut ticker = Ticker::every(Duration::from_millis(u64::from(RANGING_PERIOD_MS)));

    loop {
        match select4(front.wait_ready(), rear.wait_ready(), COMMAND.receive(), ticker.next()).await {
            Either4::First(result) => {
                handle_read(front, front_runtime, result).await;
            }
            Either4::Second(result) => {
                handle_read(rear, rear_runtime, result).await;
            }
            Either4::Third(command) => match command {
                RangefinderCommand::Enable(request) => LAST_ENABLE_REQUEST.store(request, Ordering::Relaxed),
                RangefinderCommand::Disable => {
                    power_down(front, rear).await;
                    disarm(front_runtime, rear_runtime).await;
                    return EnabledEnd::Disabled;
                }
            },
            Either4::Fourth(()) => {}
        }

        let now = now_ms();
        let mut maintenance = maintain(front, front_runtime, rear, rear_runtime, delay, &mut ticker, now).await;
        if matches!(maintenance, Maintenance::Continue) {
            maintenance = maintain(rear, rear_runtime, front, front_runtime, delay, &mut ticker, now).await;
        }
        if matches!(maintenance, Maintenance::Disabled) {
            power_down(front, rear).await;
            disarm(front_runtime, rear_runtime).await;
            return EnabledEnd::Disabled;
        }

        let pair = pair_state(true, front_runtime.lifecycle.state(), rear_runtime.lifecycle.state());
        let floor_drop = front_runtime.lifecycle.is_drop() || rear_runtime.lifecycle.is_drop();
        set_pair_state(pair);
        publish(front_runtime, rear_runtime, pair, floor_drop).await;
        set_floor_drop(floor_drop).await;
    }
}

/// Handle one `INT`-ready result for a sensor.
///
/// A failure marks the sensor for recovery; the caller's `maintain` pass runs
/// the reset alongside the other sensor's reads.
#[allow(clippy::large_futures)]
async fn handle_read<I2C, X, I>(
    driver: &mut Rangefinder<I2C, X, I>,
    runtime: &mut SensorRuntime,
    ready: Result<(), rangefinder::Error<I2C::Error>>,
) where
    I2C: embedded_hal_async::i2c::I2c,
    X: embedded_hal::digital::OutputPin,
    I: embedded_hal_async::digital::Wait,
{
    if let Err(error) = ready {
        warn!("[rangefinder] interrupt wait failed: {:?}", Debug2Format(&error));
        runtime.needs_recovery = true;
        return;
    }

    match driver.read().await {
        Ok(reading) => runtime.apply_reading(reading, now_ms()),
        Err(rangefinder::Error::NotReady) => {}
        Err(error) => {
            warn!("[rangefinder] read failed: {:?}", Debug2Format(&error));
            runtime.needs_recovery = true;
        }
    }
}

/// Check a sensor's stale and invalid watchdogs, recovering it when needed.
///
/// Recovery drives the reset attempts while still servicing the other sensor's
/// `INT` line, commands and periodic publication, so a long boot poll cannot
/// starve the healthy sensor into a false stale reset.
#[allow(clippy::large_futures, clippy::too_many_arguments)]
async fn maintain(
    driver: &mut SensorDriver,
    runtime: &mut SensorRuntime,
    other_driver: &mut SensorDriver,
    other_runtime: &mut SensorRuntime,
    delay: &mut Delay,
    ticker: &mut Ticker,
    now_ms: u64,
) -> Maintenance {
    if runtime.needs_recovery {
        runtime.needs_recovery = false;
        warn!("[rangefinder] read or interrupt failure");
        return recovery_maintenance(recover_sensor(driver, runtime, other_driver, other_runtime, delay, ticker).await);
    }
    if let Some(cause) = runtime.lifecycle.recovery_cause(now_ms) {
        warn!("[rangefinder] sensor unhealthy: {:?}", Debug2Format(&cause));
        return recovery_maintenance(recover_sensor(driver, runtime, other_driver, other_runtime, delay, ticker).await);
    }
    Maintenance::Continue
}

/// Map a recovery result onto the enabled period's next action.
const fn recovery_maintenance(end: RecoveryEnd) -> Maintenance {
    match end {
        RecoveryEnd::Recovered | RecoveryEnd::Failed => Maintenance::Continue,
        RecoveryEnd::Disabled => Maintenance::Disabled,
    }
}

/// Bring one sensor up with bounded attempts.
#[allow(clippy::large_futures)]
async fn init_sensor<I2C, X, I>(
    driver: &mut Rangefinder<I2C, X, I>,
    runtime: &mut SensorRuntime,
    delay: &mut Delay,
) -> bool
where
    I2C: embedded_hal_async::i2c::I2c,
    X: embedded_hal::digital::OutputPin,
    I: embedded_hal_async::digital::Wait,
{
    while let Some(attempt) = runtime.lifecycle.begin_attempt() {
        match driver.init(delay).await {
            Ok(()) => {
                runtime.on_sensor_ready(now_ms());
                info!("[rangefinder] sensor initialised");
                return true;
            }
            Err(error) => warn!(
                "[rangefinder] init attempt {} failed: {:?}",
                attempt,
                Debug2Format(&error)
            ),
        }
    }

    runtime.lifecycle.mark_failed();
    false
}

/// Reset one failed sensor with the remaining per-enable attempts.
///
/// The reset future is selected alongside the other sensor's `INT` line, the
/// command channel and the publish ticker, so the healthy sensor keeps being
/// read and published while a boot poll runs. A `Disable` cancels the reset;
/// the caller powers both sensors down and disarms the pair.
#[allow(clippy::large_futures, clippy::too_many_arguments)]
async fn recover_sensor(
    driver: &mut SensorDriver,
    runtime: &mut SensorRuntime,
    other_driver: &mut SensorDriver,
    other_runtime: &mut SensorRuntime,
    delay: &mut Delay,
    ticker: &mut Ticker,
) -> RecoveryEnd {
    // A latched failure stays put until a fresh enable; an INT edge must not
    // re-enter recovery and flap the pair.
    if matches!(runtime.lifecycle.state(), SensorState::Failed) {
        return RecoveryEnd::Failed;
    }
    runtime.needs_recovery = false;

    // An armed sensor fails safe immediately, so a drive already in progress
    // brakes instead of continuing blind while the reset attempts run. Publish
    // the non-ready pair and the flag before the first await; a successful
    // reset deliberately retains the drop until fresh valid floor readings
    // clear it.
    runtime.lifecycle.begin_recovery();
    set_pair_state(PairState::Degraded);
    let pair = pair_state(true, runtime.lifecycle.state(), other_runtime.lifecycle.state());
    let floor_drop = runtime.lifecycle.is_drop() || other_runtime.lifecycle.is_drop();
    publish(runtime, other_runtime, pair, floor_drop).await;
    set_floor_drop(floor_drop).await;

    while let Some(attempt) = runtime.lifecycle.begin_attempt() {
        let outcome = {
            let mut reset = pin!(driver.reset(delay));
            loop {
                match select4(
                    reset.as_mut(),
                    other_driver.wait_ready(),
                    COMMAND.receive(),
                    ticker.next(),
                )
                .await
                {
                    Either4::First(result) => match result {
                        Ok(()) => break AttemptOutcome::Recovered,
                        Err(error) => {
                            warn!(
                                "[rangefinder] reset attempt {} failed: {:?}",
                                attempt,
                                Debug2Format(&error)
                            );
                            break AttemptOutcome::Failed;
                        }
                    },
                    Either4::Second(ready) => {
                        handle_read(other_driver, other_runtime, ready).await;
                    }
                    Either4::Third(command) => match command {
                        RangefinderCommand::Enable(request) => {
                            LAST_ENABLE_REQUEST.store(request, Ordering::Relaxed);
                        }
                        RangefinderCommand::Disable => break AttemptOutcome::Disabled,
                    },
                    Either4::Fourth(()) => {}
                }

                let pair = pair_state(true, runtime.lifecycle.state(), other_runtime.lifecycle.state());
                let floor_drop = runtime.lifecycle.is_drop() || other_runtime.lifecycle.is_drop();
                set_pair_state(pair);
                publish(runtime, other_runtime, pair, floor_drop).await;
                set_floor_drop(floor_drop).await;
            }
        };

        match outcome {
            AttemptOutcome::Recovered => {
                runtime.on_sensor_ready(now_ms());
                info!("[rangefinder] sensor recovered");
                return RecoveryEnd::Recovered;
            }
            AttemptOutcome::Disabled => return RecoveryEnd::Disabled,
            AttemptOutcome::Failed => {}
        }
    }

    runtime.lifecycle.mark_failed();
    warn!("[rangefinder] sensor failed after its reset attempts");
    power_down_one(driver, "sensor").await;
    RecoveryEnd::Failed
}

/// Power both sensors down, ignoring a wedged sensor's stop error.
#[allow(clippy::large_futures)]
async fn power_down(front: &mut SensorDriver, rear: &mut SensorDriver) {
    power_down_one(front, "front").await;
    power_down_one(rear, "rear").await;
}

/// Power one sensor down, ignoring a wedged sensor's stop error.
async fn power_down_one<I2C, X, I>(driver: &mut Rangefinder<I2C, X, I>, label: &'static str)
where
    I2C: embedded_hal_async::i2c::I2c,
    X: embedded_hal::digital::OutputPin,
    I: embedded_hal_async::digital::Wait,
{
    if let Err(error) = driver.standby().await {
        warn!("[rangefinder] {} standby failed: {:?}", label, Debug2Format(&error));
    }
}

/// Clear the pair's telemetry and flag after an explicit disable.
async fn disarm(front_runtime: &mut SensorRuntime, rear_runtime: &mut SensorRuntime) {
    front_runtime.reset();
    rear_runtime.reset();
    set_pair_state(PairState::Off);
    crate::system::state::rangefinder::clear().await;
    set_floor_drop(false).await;
}

/// Hold a latched failure until an explicit disable releases it.
async fn hold_failed() {
    loop {
        match COMMAND.receive().await {
            RangefinderCommand::Disable => {
                set_pair_state(PairState::Off);
                crate::system::state::rangefinder::clear().await;
                set_floor_drop(false).await;
                return;
            }
            RangefinderCommand::Enable(request) => {
                LAST_ENABLE_REQUEST.store(request, Ordering::Relaxed);
            }
        }
    }
}

/// Publish the current telemetry snapshot.
async fn publish(front: &SensorRuntime, rear: &SensorRuntime, pair: PairState, floor_drop: bool) {
    let now = now_ms();
    crate::system::state::rangefinder::publish(RangefinderTelemetry {
        front: front.telemetry(now),
        rear: rear.telemetry(now),
        pair,
        floor_drop,
        threshold_mm: DROP_THRESHOLD_MM,
    })
    .await;
}

/// Write the floor-drop flag and raise its edge on a change.
async fn set_floor_drop(detected: bool) {
    if perception::set_floor_drop(detected) != ChangeDetected::NoChange {
        raise_event(Events::FloorDropDetected { detected }).await;
    }
}

/// Store a new pair state in the lock-free status cell.
fn set_pair_state(state: PairState) {
    PAIR_STATE.store(state.as_u8(), Ordering::Relaxed);
}

/// Monotonic milliseconds since boot.
fn now_ms() -> u64 {
    Instant::now().as_millis()
}

// ── Per-sensor runtime ────────────────────────────────────────────────────────

/// The mutable state one sensor carries through an enabled period.
struct SensorRuntime {
    /// The pure arming, recovery and floor-detection state machine.
    lifecycle: SensorLifecycle,
    /// Timestamp of the last reading read from the device.
    last_reading_ms: Option<u64>,
    /// The last Rangefinder Reading, kept whole so its fields cannot drift.
    last_reading: Option<RangefinderReading>,
    /// A read or interrupt failure that should start recovery on the next
    /// maintenance pass.
    needs_recovery: bool,
}

impl SensorRuntime {
    /// A sensor that has not been enabled yet.
    const fn new() -> Self {
        Self {
            lifecycle: SensorLifecycle::new(DROP_THRESHOLD_MM),
            last_reading_ms: None,
            last_reading: None,
            needs_recovery: false,
        }
    }

    /// Drop the last reading so the readout keeps its placeholders.
    const fn clear_reading(&mut self) {
        self.last_reading_ms = None;
        self.last_reading = None;
        self.needs_recovery = false;
    }

    /// Reset the runtime for a fresh enable.
    const fn reset_for_enable(&mut self, now_ms: u64) {
        self.lifecycle.begin_enable(now_ms);
        self.clear_reading();
    }

    /// Mark the sensor ready for a first valid reading after init or recovery.
    const fn on_sensor_ready(&mut self, now_ms: u64) {
        self.lifecycle.attempt_succeeded(now_ms);
        self.clear_reading();
    }

    /// Reset the runtime after an explicit disable.
    const fn reset(&mut self) {
        self.lifecycle.begin_disable();
        self.clear_reading();
    }

    /// Apply one reading to the detector and the sensor's telemetry.
    const fn apply_reading(&mut self, reading: RangefinderReading, now_ms: u64) {
        self.last_reading = Some(reading);
        self.last_reading_ms = Some(now_ms);
        self.needs_recovery = false;
        self.lifecycle.on_reading(reading, now_ms);
    }

    /// Build this sensor's telemetry.
    fn telemetry(&self, now_ms: u64) -> SensorTelemetry {
        let range_mm = self.last_reading.and_then(|reading| match reading.reading {
            rangefinder::Reading::Valid(range_mm) => Some(range_mm),
            rangefinder::Reading::NoTarget | rangefinder::Reading::Invalid(_) => None,
        });
        SensorTelemetry {
            state: self.lifecycle.state(),
            range_mm,
            status: self.last_reading.map(|reading| reading.status),
            raw_status: self.last_reading.map(|reading| reading.raw_status),
            diagnostics: self.last_reading.map(|reading| reading.diagnostics),
            floor: self.lifecycle.floor(),
            age_ms: self
                .last_reading_ms
                .map(|last| u32::try_from(now_ms.saturating_sub(last)).unwrap_or(u32::MAX)),
        }
    }
}
