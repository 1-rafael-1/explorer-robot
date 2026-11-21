//! The measured pair sweep: twenty simultaneous Power Steps, driven end to end.
//!
//! This is the single driver of [`motor_cal::MeasurementEngine`], shared by the
//! Basic Motor Test and the Motor Calibration Procedure. One Power Step commands
//! **both** tracks at the same power and measures both encoder deltas over the same
//! Measurement Window, so the calibration is derived in the load and supply
//! condition it is applied in. One pair step is:
//!
//! 1. brake both tracks and wait for **Quiescence** — each track's engine sees no
//!    pulses for the engine's hold window, with its timeout invalidating the point;
//! 2. reset both counters, then wait for the encoder task to publish a fresh
//!    all-zero sample — not a fixed sleep — and confirm the reset with that
//!    sample's timestamp, dropping the stale drive-side copy;
//! 3. command both tracks at the step power in the same direction;
//! 4. discard the settle interval and count each Measurement Window as the
//!    wrap-safe delta between the timestamped samples at each end.
//!
//! The engines own every rule and every timeout. This module only performs the
//! actions they ask for (brake, reset, command), feeds them the timestamped
//! samples, and reports both live counts to a [`StepObserver`] so the panel can
//! follow the wheels. There is no second measurement implementation.
//!
//! # The sweep
//!
//! [`run_sweep`] is the whole cycle: forward and reverse, each over the ten
//! [`STEP_POWERS`], with the Stop and floor-drop checks before each step and the
//! encoder sampler started and stopped around it. That is [`TOTAL_STEPS`] = 20
//! pair steps. The caller supplies only what is specific to its Procedure through
//! [`SweepSink`]: the Basic Motor Test draws a readout and records a summary, the
//! Motor Calibration stores the raw measurements and judges the rail. The
//! schedule, the cycle position and the terminal classification are the same for
//! both, so they live here rather than in two loops that must be kept in step.
//!
//! The command is [`crate::task::motor_driver::MotorCommand::SetTracksRaw`]: the
//! sweep measures the raw drivetrain. A calibration that swept through the loaded
//! curve would measure the corrected duty and re-derive a near-identity curve,
//! and the Basic Motor Test and the Motor Calibration would no longer agree.
//!
//! A point's count is never read as an absolute-since-reset value: each engine
//! reports its Measurement Window delta, and an invalid point carries its
//! [`InvalidReason`] so the panel can show `--` instead of a silent zero.

use defmt::{debug, warn};
use embassy_time::{Duration, Instant, Timer};
use motor_cal::{
    Curve, Direction, EncoderSample, InvalidReason, MeasurementEngine, Phase, PointResult, TestLeg, Track,
};
use touch_ui::Correction;

use crate::{
    system::state::{activity, perception, power},
    task::{
        drive::{clear_encoder_measurement, get_latest_encoder_measurement},
        motor_driver::{self, MotorCommand},
        procedure::Claim,
        sensors::encoders::{self, EncoderCommand},
    },
};

/// The encoder sampling interval for the duration of a sweep, in milliseconds.
const SAMPLE_INTERVAL_MS: u64 = 20;

/// How long a Power Step waits for a fresh encoder sample before treating the
/// encoder task as dead, in milliseconds.
///
/// The task samples every [`SAMPLE_INTERVAL_MS`], so half a second of repeated or
/// missing samples is far beyond a scheduling hiccup. The engines' own timeouts
/// only advance when samples arrive, so this wall-clock bound is the only thing
/// that stops a dead encoder task from leaving a step waiting forever.
const NO_SAMPLE_TIMEOUT_MS: u64 = 500;

/// The reason a dead encoder task reports as a structural abort.
const NO_SAMPLE_REASON: &str = "encoder timeout";

/// The reason a floor drop trips, shared by both Procedures.
const FLOOR_DROP_REASON: &str = "floor drop detected";

/// The reason no fresh all-zero sample arrived to confirm the counter reset.
const RESET_CONFIRM_REASON: &str = "reset unconfirmed: no fresh zero sample after Reset";

/// How long a step waits for the encoder task to publish a fresh all-zero
/// sample after a Reset, in milliseconds.
const RESET_CONFIRM_TIMEOUT_MS: u64 = 500;

/// The directions a pair sweep runs, in cycle order.
pub const DIRECTIONS: [Direction; 2] = [Direction::Forward, Direction::Reverse];

/// The Power Steps of one direction, in whole percent, in ascending order.
///
/// Aliased from the measurement model so the schedule and the curve maths can
/// never disagree.
pub const STEP_POWERS: [u8; motor_cal::POWER_STEPS] = motor_cal::POWER_STEP_PERCENTS;

/// The pair steps in a full one-cycle sweep: both directions over every Power
/// Step.
pub const TOTAL_STEPS: usize = DIRECTIONS.len() * motor_cal::POWER_STEPS;

/// Enable both drivers and (re)start the encoder sampler at [`SAMPLE_INTERVAL_MS`].
///
/// [`run_sweep`] calls this before its first step and [`stop_encoders`] after its
/// last. Clearing the drive-side sample after the reset means the first read can
/// only be a fresh one.
async fn start_encoders() {
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
    Timer::after(Duration::from_millis(10)).await;

    encoders::send_command(EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(100)).await;
    encoders::send_command(EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await;
    clear_encoder_measurement().await;
    encoders::send_command(EncoderCommand::Start {
        interval_ms: SAMPLE_INTERVAL_MS,
    })
    .await;
    Timer::after(Duration::from_millis(100)).await;
}

/// Stop the encoder sampler at the end of a sweep.
///
/// Idempotent, so the terminal paths can call it again to be sure a sampler that
/// was started for an earlier run is not left publishing.
pub async fn stop_encoders() {
    encoders::send_command(EncoderCommand::Stop).await;
}

/// A sink for the live encoder counts while a Power Step runs.
///
/// The caller owns the readout; the sweep reports every fresh sample of both
/// tracks so the panel can follow the wheels. The Motor Calibration uses a no-op
/// sink.
// `async fn` in a trait keeps the observer's publish in the caller's task; the
// trait is crate-internal, so no public API promise is being made here.
#[allow(async_fn_in_trait)]
pub trait StepObserver {
    /// Accept the current raw encoder counts for both tracks.
    async fn sample(&mut self, left: u16, right: u16);
}

/// What one measured pair step leaves behind.
#[derive(Clone, Copy)]
pub struct PairOutcome {
    /// The left engine's verdict: a valid Measurement Window or the reason it is
    /// invalid, which the panel renders as `--`.
    pub left: PointResult,
    /// The right engine's verdict, with the same meaning as [`left`](Self::left).
    pub right: PointResult,
    /// The motor rail voltage read at the instant the command was made, or
    /// `None` when no sample was available. Both tracks share the one rail.
    pub rail_volts: Option<f32>,
    /// The voltage compensation the driver's rule derives at that rail, or
    /// `None` when the rail was unreadable.
    pub compensation: Option<f32>,
}

/// Why a pair step ended.
enum StepEnd {
    /// Both engines reached a verdict, valid or invalid.
    Measured(PairOutcome),
    /// The operator pressed Stop.
    Stopped,
    /// Floor-drop protection tripped.
    Fault,
    /// A structural problem outside the engines' verdicts cut the step short.
    ///
    /// The reason is the short phrase the caller records, e.g. a dead encoder
    /// task. The caller brakes and fails; this step has already given up.
    Aborted(&'static str),
}

/// Where one pair step sits in the sweep.
///
/// The sweep hands this to the [`SweepSink`], which needs the direction and
/// power to name the step and its position to store the outcome in cycle order.
#[derive(Clone, Copy)]
pub struct StepRef {
    /// The direction both tracks turn.
    pub direction: Direction,
    /// The commanded power, in whole percent.
    pub power: u8,
    /// The pair step's zero-based position in the cycle.
    pub index: u8,
    /// The number of pair steps in a full cycle.
    pub total: u8,
}

impl StepRef {
    /// The one-based position of this step in the cycle.
    #[must_use]
    pub const fn position(self) -> u8 {
        self.index.saturating_add(1)
    }

    /// The Power Step's position within its direction.
    ///
    /// The sweep runs the directions in order, each over the same ten powers, so
    /// the position within a direction is the cycle index modulo the step count.
    #[must_use]
    pub const fn step_index(self) -> usize {
        (self.index as usize) % motor_cal::POWER_STEPS
    }
}

/// What a [`SweepSink`] wants after a measured pair step.
pub enum StepAction {
    /// Continue with the next pair step.
    Continue,
    /// Cut the sweep short, reporting this reason as the outcome.
    Abort(&'static str),
}

/// What a Procedure adds to the sweep it runs.
///
/// The sweep owns the schedule, the cycle position, the encoder sampler and the
/// terminal classification; a sink is only the Procedure-specific half. It also
/// receives every fresh encoder count from both tracks, so a run that shows no
/// live readout answers [`sample`](StepObserver::sample) with nothing.
// `async fn` in a trait keeps the sink's work in the sweep's task; the trait is
// crate-internal in practice, so no public API promise is being made here.
#[allow(async_fn_in_trait)]
pub trait SweepSink: StepObserver {
    /// A pair step is about to command both tracks.
    async fn step_started(&mut self, step: StepRef);

    /// A pair step reached both engines' verdicts: valid or invalid.
    ///
    /// Returning [`StepAction::Abort`] ends the sweep before the next step, with
    /// the reason the caller reports.
    async fn step_measured(&mut self, step: StepRef, outcome: PairOutcome) -> StepAction;
}

/// Why a whole sweep ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SweepEnd {
    /// Every pair step of every direction was measured, valid or invalid.
    Complete,
    /// The operator pressed Stop.
    Stopped,
    /// Floor-drop protection tripped, or a structural problem cut the sweep
    /// short, with the reason the panel shows.
    Aborted(&'static str),
}

/// Drive and measure one simultaneous pair step in `direction` at `power`.
///
/// Performs every caller action the engines ask for, feeds each engine its own
/// track's timestamped samples, and reports each fresh pair of counts to
/// `observer`. Returns both verdicts, or the reason the step was cut short.
///
/// The engines advance independently until they must act together: the reset is
/// taken only when both are Quiescent, and the command only when both are
/// `ReadyToCommand`. If either engine invalidates, the other is aborted with the
/// same reason, so a bad point cannot be paired with a good one by accident. A
/// valid/invalid split is still possible when one engine had already finished a
/// valid window before the other invalidated; the calibration fails on the
/// invalid side.
async fn run_power_pair<O>(claim: &Claim, direction: Direction, power: u8, observer: &mut O) -> StepEnd
where
    O: StepObserver,
{
    let mut left = MeasurementEngine::new();
    let mut right = MeasurementEngine::new();
    let started_ms = now_ms();
    left.begin_step(started_ms);
    right.begin_step(started_ms);

    // Quiescence is the precondition for attributing a count to a command, and a
    // coast alone does not establish it: brake both tracks first.
    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

    let mut last_timestamp: Option<u64> = None;
    let mut last_fresh_ms = started_ms;

    // The rail and compensation read at the instant the command is made. A
    // sample taken at the end of the window is one the driver never used.
    let mut rail_volts: Option<f32> = None;
    let mut compensation: Option<f32> = None;

    loop {
        if claim.wait_or_stop(SAMPLE_INTERVAL_MS).await {
            return StepEnd::Stopped;
        }
        if perception::is_floor_drop_detected() {
            return StepEnd::Fault;
        }
        // The engines only advance on samples, so they cannot notice that the
        // encoder task stopped producing them. A wall-clock bound turns that
        // silence into a structural abort instead of an endless step.
        if now_ms().saturating_sub(last_fresh_ms) >= NO_SAMPLE_TIMEOUT_MS {
            return StepEnd::Aborted(NO_SAMPLE_REASON);
        }

        // One invalid track spoils the pair: abort the other with the same
        // reason. The abort is a no-op on an engine that already finished, so a
        // valid/invalid split is preserved rather than rewritten.
        if let Some(reason) = invalid_reason(&left).or_else(|| invalid_reason(&right)) {
            left.abort(reason);
            right.abort(reason);
            break;
        }
        if matches!(left.phase(), Phase::Finished) && matches!(right.phase(), Phase::Finished) {
            break;
        }

        // Perform the action both engines need, so their rules stay their own.
        if matches!(left.phase(), Phase::Quiescent) && matches!(right.phase(), Phase::Quiescent) {
            if let Some(end) =
                reset_and_confirm(claim, &mut left, &mut right, &mut last_timestamp, &mut last_fresh_ms).await
            {
                return end;
            }
        } else if matches!(left.phase(), Phase::ReadyToCommand) && matches!(right.phase(), Phase::ReadyToCommand) {
            // Both tracks at the same requested power: this is the load/supply
            // condition the correction will be applied in.
            let signed = signed_speed(direction, power);
            (rail_volts, compensation) = rail_at_command();
            motor_driver::send_motor_command(MotorCommand::SetTracksRaw {
                left_speed: signed,
                right_speed: signed,
            })
            .await;
            let command_ms = now_ms();
            left.command_applied(command_ms);
            right.command_applied(command_ms);
        }

        // A missing sample is not a zero count: leave the engines alone rather
        // than credit pulses that did not happen. A repeated sample cannot
        // advance any window, so it is not fed twice.
        let Some(measurement) = get_latest_encoder_measurement().await else {
            continue;
        };
        if last_timestamp == Some(measurement.timestamp_ms) {
            continue;
        }
        last_timestamp = Some(measurement.timestamp_ms);
        last_fresh_ms = now_ms();

        observer.sample(measurement.left, measurement.right).await;

        left.observe(EncoderSample {
            timestamp_ms: measurement.timestamp_ms,
            count: measurement.left,
        });
        right.observe(EncoderSample {
            timestamp_ms: measurement.timestamp_ms,
            count: measurement.right,
        });
    }

    // The window is over: brake both tracks before the caller publishes the
    // result, so a wheel is never left commanded between Power Steps.
    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

    let left = result_or_no_window(&left);
    let right = result_or_no_window(&right);
    debug!(
        "sweep: {=str} {=u8}% L={=str} R={=str} rail={=bool}",
        direction_label(direction),
        power,
        result_label(left),
        result_label(right),
        rail_volts.is_some()
    );

    StepEnd::Measured(PairOutcome {
        left,
        right,
        rail_volts,
        compensation,
    })
}

/// Reset both counters and confirm the reset on both engines, or report why the
/// step ended early.
///
/// The encoder task may be mid-interval (or delayed), so a fixed sleep can
/// record the reset instant before the task has applied the reset and then see
/// a stale pre-reset sample. Wait for a fresh all-zero sample instead: that is
/// positive proof the reset took. Because quiescence was established first, the
/// counters are stable, so a fresh nonzero sample can only be a pre-reset
/// sample; it is ignored while the wait continues.
///
/// Returns `None` once the reset is confirmed, or the [`StepEnd`] that cuts the
/// step short: stopped, floor drop, or [`RESET_CONFIRM_REASON`] on timeout.
async fn reset_and_confirm(
    claim: &Claim,
    left: &mut MeasurementEngine,
    right: &mut MeasurementEngine,
    last_timestamp: &mut Option<u64>,
    last_fresh_ms: &mut u64,
) -> Option<StepEnd> {
    encoders::send_command(EncoderCommand::Reset).await;
    let deadline = now_ms().saturating_add(RESET_CONFIRM_TIMEOUT_MS);
    loop {
        if claim.wait_or_stop(SAMPLE_INTERVAL_MS).await {
            return Some(StepEnd::Stopped);
        }
        if perception::is_floor_drop_detected() {
            return Some(StepEnd::Fault);
        }
        if now_ms() >= deadline {
            return Some(StepEnd::Aborted(RESET_CONFIRM_REASON));
        }
        let Some(measurement) = get_latest_encoder_measurement().await else {
            continue;
        };
        if *last_timestamp == Some(measurement.timestamp_ms) {
            continue;
        }
        *last_timestamp = Some(measurement.timestamp_ms);
        *last_fresh_ms = now_ms();
        if measurement.left == 0 && measurement.right == 0 {
            let reset_ms = measurement.timestamp_ms;
            left.reset_now(reset_ms);
            right.reset_now(reset_ms);
            // Confirm with the zero sample we just consumed. Its timestamp is
            // the reset instant, which observe_reset accepts.
            left.observe(EncoderSample {
                timestamp_ms: reset_ms,
                count: 0,
            });
            right.observe(EncoderSample {
                timestamp_ms: reset_ms,
                count: 0,
            });
            clear_encoder_measurement().await;
            return None;
        }
        // A nonzero sample taken after we asked for the reset is a pre-reset
        // sample; the task has not applied the reset yet. Keep waiting.
    }
}

/// Run one full sweep: forward and reverse, each over the ten Power Steps.
///
/// The cycle is the same for the Basic Motor Test and the Motor Calibration, so
/// it lives here: the encoder sampler is started before the first step and
/// stopped after the last, both tracks are braked between steps, and the Stop
/// latch and the floor-drop flag are checked before each step so a latched stop
/// is never overridden by the next command. The caller's [`SweepSink`] owns
/// everything specific to its Procedure.
pub async fn run_sweep<S>(claim: &Claim, sink: &mut S) -> SweepEnd
where
    S: SweepSink,
{
    start_encoders().await;

    let end = 'cycle: {
        let total = u8::try_from(TOTAL_STEPS).unwrap_or(u8::MAX);
        for direction in DIRECTIONS {
            for (step_index, power) in STEP_POWERS.iter().copied().enumerate() {
                if claim.is_stop_requested() {
                    break 'cycle SweepEnd::Stopped;
                }
                if perception::is_floor_drop_detected() {
                    break 'cycle SweepEnd::Aborted(FLOOR_DROP_REASON);
                }

                let cycle_index = direction_index(direction) * motor_cal::POWER_STEPS + step_index;
                let index = u8::try_from(cycle_index).unwrap_or(u8::MAX);
                let step = StepRef {
                    direction,
                    power,
                    index,
                    total,
                };
                // The bar shows where the operator is in the twenty-step cycle,
                // so the last step reads 100 % while it runs; a finished view
                // drops the bar entirely.
                let percent = activity::percent_done(cycle_index + 1, TOTAL_STEPS);
                claim.phase(direction_label(direction), Some(percent)).await;
                sink.step_started(step).await;

                match run_power_pair(claim, direction, power, &mut *sink).await {
                    StepEnd::Measured(outcome) => match sink.step_measured(step, outcome).await {
                        StepAction::Continue => {}
                        StepAction::Abort(reason) => break 'cycle SweepEnd::Aborted(reason),
                    },
                    StepEnd::Stopped => break 'cycle SweepEnd::Stopped,
                    StepEnd::Fault => break 'cycle SweepEnd::Aborted(FLOOR_DROP_REASON),
                    StepEnd::Aborted(reason) => break 'cycle SweepEnd::Aborted(reason),
                }
            }
        }
        SweepEnd::Complete
    };

    stop_encoders().await;
    end
}

/// The motor rail and the compensation the driver's rule derives at it, read
/// once at the instant the command is made.
///
/// A sample taken at the end of the window is one the driver never used, so the
/// record carries these values instead.
fn rail_at_command() -> (Option<f32>, Option<f32>) {
    power::try_get_rail_sample().map_or_else(
        || {
            warn!("sweep: no rail sample at command time; recording none");
            (None, None)
        },
        |(volts, _)| (Some(volts), Some(motor_driver::voltage_compensation(volts))),
    )
}

/// The signed track command for both tracks of a pair step in `direction` at
/// `power` percent.
///
/// Both tracks are commanded at the same requested power, which is what makes
/// the measured condition match the condition a corrected `SetTracks` runs in.
pub fn signed_speed(direction: Direction, power: u8) -> i8 {
    // The caller only ever passes one of the ten Power Steps, all at most 100 %.
    let magnitude = i8::try_from(power).unwrap_or(i8::MAX);
    match direction {
        Direction::Forward => magnitude,
        Direction::Reverse => -magnitude,
    }
}

/// The two Test Legs a pair step in `direction` writes: left then right.
#[must_use]
pub const fn legs(direction: Direction) -> [TestLeg; 2] {
    [
        TestLeg::new(Track::Left, direction),
        TestLeg::new(Track::Right, direction),
    ]
}

/// The index of `leg` in [`motor_cal::LEG_ORDER`].
///
/// The model keeps its own leg order private; this is the one translation the
/// calibration needs to store a pair's two legs in cycle order.
#[must_use]
pub const fn leg_index(leg: TestLeg) -> usize {
    match (leg.track, leg.direction) {
        (Track::Left, Direction::Forward) => 0,
        (Track::Left, Direction::Reverse) => 1,
        (Track::Right, Direction::Forward) => 2,
        (Track::Right, Direction::Reverse) => 3,
    }
}

/// The curve in force for `track` and `direction` at `power`, as the panel shows
/// it.
///
/// With no curve loaded the command is used unchanged (`1.00`); a curve that
/// corrects nothing at or below its Calibration Floor yields `raw`. This is the
/// single formatting of a correction for both the live readout and the summary.
pub fn correction(curve: Option<&Curve>, track: Track, direction: Direction, power: u8) -> Correction {
    let Some(curve) = curve else {
        return Correction::Identity;
    };
    curve
        .correction(track, direction, power)
        .map_or(Correction::Raw, |factor| Correction::Factor(per_mille(factor)))
}

/// The operator-facing direction name for the phase line and the bench log.
#[must_use]
pub const fn direction_label(direction: Direction) -> &'static str {
    match direction {
        Direction::Forward => "Forward",
        Direction::Reverse => "Reverse",
    }
}

/// The zero-based cycle position of `direction`'s first pair step.
const fn direction_index(direction: Direction) -> usize {
    match direction {
        Direction::Forward => 0,
        Direction::Reverse => 1,
    }
}

/// Round a curve factor to per-mille for the readout's fixed-point cell.
///
/// The readout stays `Eq`-comparable because the factor crosses its boundary as
/// an integer; the curve maths itself never leaves `motor-cal`.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn per_mille(factor: f32) -> u16 {
    let scaled = (factor * 1000.0) + 0.5;
    if scaled <= 0.0 {
        0
    } else if scaled >= 1000.0 {
        1000
    } else {
        scaled as u16
    }
}

/// The current time, in milliseconds since boot.
fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// An engine's invalid verdict, or `None` while it is running or valid.
const fn invalid_reason(engine: &MeasurementEngine) -> Option<InvalidReason> {
    match engine.result() {
        Some(PointResult::Invalid(reason)) => Some(reason),
        Some(PointResult::Valid(_)) | None => None,
    }
}

/// An engine's verdict, or the reason a step cut short leaves it with none.
fn result_or_no_window(engine: &MeasurementEngine) -> PointResult {
    engine.result().unwrap_or(PointResult::Invalid(InvalidReason::NoWindow))
}

/// The panel's Test Leg for the measurement model's.
///
/// The two models name the same four legs; this is the one translation between
/// them, so the schedule, the phase line and the readout cannot disagree about
/// which leg is running.
pub const fn ui_leg(leg: TestLeg) -> touch_ui::TestLeg {
    touch_ui::TestLeg {
        track: match leg.track {
            Track::Left => touch_ui::Track::Left,
            Track::Right => touch_ui::Track::Right,
        },
        direction: ui_direction(leg.direction),
    }
}

/// The panel's direction for the measurement model's.
pub const fn ui_direction(direction: Direction) -> touch_ui::Direction {
    match direction {
        Direction::Forward => touch_ui::Direction::Forward,
        Direction::Reverse => touch_ui::Direction::Reverse,
    }
}

/// The operator-facing name of a Test Leg, for the bench log.
pub const fn leg_label(leg: TestLeg) -> &'static str {
    match (leg.track, leg.direction) {
        (Track::Left, Direction::Forward) => "left forward",
        (Track::Left, Direction::Reverse) => "left reverse",
        (Track::Right, Direction::Forward) => "right forward",
        (Track::Right, Direction::Reverse) => "right reverse",
    }
}

/// A short name for a point's verdict, for the bench log.
const fn result_label(result: PointResult) -> &'static str {
    match result {
        PointResult::Valid(_) => "valid",
        PointResult::Invalid(reason) => reason.label(),
    }
}
