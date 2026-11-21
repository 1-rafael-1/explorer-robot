//! The measured sweep: forty Power Steps, driven end to end.
//!
//! This is the single driver of [`motor_cal::MeasurementEngine`], shared by the
//! Basic Motor Test and the Motor Calibration Procedure. One Power Step is:
//!
//! 1. brake both tracks and wait for **Quiescence** — no encoder pulses on
//!    either track for the engine's hold window, with its timeout invalidating
//!    the point;
//! 2. reset both counters, let the encoder task serve the reset, drop the stale
//!    drive-side sample, and wait for a fresh sample at zero to confirm it;
//! 3. command only the tested track at the step power and hold the other at zero;
//! 4. discard the settle interval and count the Measurement Window as the
//!    wrap-safe delta between the timestamped samples at each end.
//!
//! The engine owns every rule and every timeout. This module only performs the
//! actions it asks for (brake, reset, command), feeds it the timestamped samples,
//! and reports live counts to a [`StepObserver`] so the panel can follow the
//! wheel. There is no second measurement implementation.
//!
//! # The sweep
//!
//! [`run_sweep`] is the whole cycle: the four Test Legs of [`STEP_POWERS`], in
//! [`motor_cal::LEG_ORDER`] order, with the Stop and floor-drop checks before each
//! step and the encoder sampler started and stopped around it. The caller supplies
//! only what is specific to its Procedure through [`SweepSink`]: the Basic Motor
//! Test draws a readout and records a summary, the Motor Calibration stores the
//! raw measurements and judges the rail. The schedule, the cycle position and the
//! terminal classification are the same for both, so they live here rather than
//! in two loops that must be kept in step.
//!
//! The command is [`crate::task::motor_driver::MotorCommand::SetTracksRaw`]: the
//! sweep measures the raw drivetrain. A calibration that swept through the loaded
//! curve would measure the corrected duty and re-derive a near-identity curve,
//! and the Basic Motor Test and the Motor Calibration would no longer agree.
//!
//! The point's count is never read as an absolute-since-reset value: the engine
//! reports the Measurement Window delta, and an invalid point carries its
//! [`InvalidReason`] so the panel can show `--` instead of a silent zero.

use defmt::{debug, warn};
use embassy_time::{Duration, Instant, Timer};
use motor_cal::{
    Curve, EncoderSample, InvalidReason, MeasurementEngine, Phase, PointResult, QUIESCENCE_HOLD_MS,
    QUIESCENCE_TIMEOUT_MS, TestLeg,
};
use touch_ui::Correction;

use crate::{
    system::state::{activity, perception, power},
    task::{
        drive::{clear_encoder_measurement, get_latest_encoder_measurement},
        motor_driver::{self, MotorCommand},
        procedure::Claim,
        sensors::encoders::{self, EncoderCommand, EncoderMeasurement},
    },
};

/// The encoder sampling interval for the duration of a sweep, in milliseconds.
const SAMPLE_INTERVAL_MS: u64 = 20;

/// How long a Power Step waits for a fresh encoder sample before treating the
/// encoder task as dead, in milliseconds.
///
/// The task samples every [`SAMPLE_INTERVAL_MS`], so half a second of repeated or
/// missing samples is far beyond a scheduling hiccup. The engine's own timeouts
/// only advance when samples arrive, so this wall-clock bound is the only thing
/// that stops a dead encoder task from leaving a step waiting forever.
const NO_SAMPLE_TIMEOUT_MS: u64 = 500;

/// The reason a dead encoder task reports as a structural abort.
const NO_SAMPLE_REASON: &str = "encoder timeout";

/// The reason a floor drop trips, shared by both Procedures.
const FLOOR_DROP_REASON: &str = "floor drop detected";

/// The Power Steps of one Test Leg, in whole percent, in ascending order.
///
/// Aliased from the measurement model so the schedule and the curve maths can
/// never disagree.
pub const STEP_POWERS: [u8; motor_cal::POWER_STEPS] = motor_cal::POWER_STEP_PERCENTS;

/// The Power Steps in a full one-cycle sweep.
pub const TOTAL_STEPS: usize = motor_cal::TEST_LEGS * motor_cal::POWER_STEPS;

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

/// A sink for the live encoder count while a Power Step runs.
///
/// The caller owns the readout; the sweep reports every fresh sample of the
/// track under test so the panel can follow the wheel. The Motor Calibration
/// uses a no-op sink.
// `async fn` in a trait keeps the observer's publish in the caller's task; the
// trait is crate-internal, so no public API promise is being made here.
#[allow(async_fn_in_trait)]
pub trait StepObserver {
    /// Accept the current raw encoder count for the track under test.
    async fn sample(&mut self, count: u16);
}

/// What one measured Power Step leaves behind.
#[derive(Clone, Copy)]
pub struct StepOutcome {
    /// The engine's verdict: a valid Measurement Window or the reason it is
    /// invalid, which the panel renders as `--`.
    pub result: PointResult,
    /// The motor rail voltage read at the instant the command was made, or
    /// `None` when no sample was available.
    pub rail_volts: Option<f32>,
    /// The voltage compensation the driver's rule derives at that rail, or
    /// `None` when the rail was unreadable.
    pub compensation: Option<f32>,
}

/// Why a Power Step ended.
enum StepEnd {
    /// The engine reached a verdict, valid or invalid.
    Measured(StepOutcome),
    /// The operator pressed Stop.
    Stopped,
    /// Floor-drop protection tripped.
    Fault,
    /// A structural problem outside the engine's verdicts cut the step short.
    ///
    /// The reason is the short phrase the caller records, e.g. a dead encoder
    /// task. The caller brakes and fails; this step has already given up.
    Aborted(&'static str),
}

/// Where one Power Step sits in the sweep.
///
/// The sweep hands this to the [`SweepSink`], which needs the leg and power to
/// name the step and its indexes to store the outcome in cycle order.
#[derive(Clone, Copy)]
pub struct StepRef {
    /// The Test Leg under test.
    pub leg: TestLeg,
    /// The commanded power, in whole percent.
    pub power: u8,
    /// The Test Leg's position in [`motor_cal::LEG_ORDER`].
    pub leg_index: usize,
    /// The Power Step's position within its Test Leg.
    pub step_index: usize,
}

impl StepRef {
    /// The zero-based number of Power Steps completed before this one.
    #[must_use]
    pub const fn done(self) -> usize {
        (self.leg_index * motor_cal::POWER_STEPS) + self.step_index
    }
}

/// What a [`SweepSink`] wants after a measured Power Step.
pub enum StepAction {
    /// Continue with the next Power Step.
    Continue,
    /// Cut the sweep short, reporting this reason as the outcome.
    Abort(&'static str),
}

/// What a Procedure adds to the sweep it runs.
///
/// The sweep owns the schedule, the cycle position, the encoder sampler and the
/// terminal classification; a sink is only the Procedure-specific half. It also
/// receives every fresh encoder count from the track under test, so a run that
/// shows no live readout answers [`sample`](StepObserver::sample) with nothing.
// `async fn` in a trait keeps the sink's work in the sweep's task; the trait is
// crate-internal in practice, so no public API promise is being made here.
#[allow(async_fn_in_trait)]
pub trait SweepSink: StepObserver {
    /// A Power Step is about to command its track.
    async fn step_started(&mut self, step: StepRef);

    /// A Power Step reached the engine's verdict: valid or invalid.
    ///
    /// Returning [`StepAction::Abort`] ends the sweep before the next step, with
    /// the reason the caller reports.
    async fn step_measured(&mut self, step: StepRef, outcome: StepOutcome) -> StepAction;
}

/// Why a whole sweep ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SweepEnd {
    /// Every Power Step of every Test Leg was measured, valid or invalid.
    Complete,
    /// The operator pressed Stop.
    Stopped,
    /// Floor-drop protection tripped, or a structural problem cut the sweep
    /// short, with the reason the panel shows.
    Aborted(&'static str),
}

/// Drive and measure one Power Step of `leg` at `power` percent.
///
/// Performs every caller action the engine asks for, feeds it the timestamped
/// samples of `leg`'s track, and reports each fresh count to `observer`. Returns
/// the engine's verdict, or the reason the step was cut short.
async fn run_power_step<O>(claim: &Claim, leg: TestLeg, power: u8, observer: &mut O) -> StepEnd
where
    O: StepObserver,
{
    let mut engine = MeasurementEngine::new();
    let started_ms = now_ms();
    engine.begin_step(started_ms);

    // Quiescence is the precondition for attributing a count to a command, and a
    // coast alone does not establish it: brake both tracks first.
    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

    let mut last_timestamp: Option<u64> = None;
    let mut last_fresh_ms = started_ms;

    // The engine's Quiescence verdict only observes the tested track. The runner
    // tracks the untested one as well and holds the reset until neither count has
    // changed for the hold window, so a pulse from the other wheel cannot be
    // credited to this command.
    let mut last_tested_change_ms = started_ms;
    let mut last_other_change_ms = started_ms;
    let mut previous_tested: Option<u16> = None;
    let mut previous_other: Option<u16> = None;
    let mut last_sample_ms = started_ms;

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
        // The engine only advances on samples, so it cannot notice that the
        // encoder task stopped producing them. A wall-clock bound turns that
        // silence into a structural abort instead of an endless step.
        if now_ms().saturating_sub(last_fresh_ms) >= NO_SAMPLE_TIMEOUT_MS {
            return StepEnd::Aborted(NO_SAMPLE_REASON);
        }

        // Perform the action the engine's current phase asks for, so its rules
        // stay its own.
        match engine.phase() {
            Phase::Quiescent => {
                // The tested track is still; the reset waits for the untested
                // one to settle too. If the pair never settles, the point cannot
                // be trusted and the engine cannot see it, so abort with the
                // quiescence verdict.
                let both_still_since = last_tested_change_ms.max(last_other_change_ms);
                if last_sample_ms.saturating_sub(both_still_since) >= QUIESCENCE_HOLD_MS {
                    // Reset both counters, give the encoder task two sample
                    // intervals to serve it, then drop the stale drive-side copy
                    // so the confirming sample can only be a fresh one.
                    encoders::send_command(EncoderCommand::Reset).await;
                    Timer::after(Duration::from_millis(2 * SAMPLE_INTERVAL_MS)).await;
                    // Fix the reset instant before dropping the stale drive-side
                    // copy: a sample published between the clear and the reset
                    // would otherwise carry a timestamp at or before the reset
                    // and be rejected as stale even though the counters had
                    // already been reset.
                    engine.reset_now(now_ms());
                    clear_encoder_measurement().await;
                } else if last_sample_ms.saturating_sub(started_ms) >= QUIESCENCE_TIMEOUT_MS {
                    engine.abort(InvalidReason::QuiescenceTimeout);
                    break;
                }
            }
            Phase::ReadyToCommand => {
                let (left_speed, right_speed) = speeds(leg, power);
                // Read the rail once, here: this is the instant the step's
                // compensation is fixed, so the record must carry this value
                // rather than one derived from a sample the driver never saw.
                (rail_volts, compensation) = rail_at_command();
                motor_driver::send_motor_command(MotorCommand::SetTracksRaw {
                    left_speed,
                    right_speed,
                })
                .await;
                engine.command_applied(now_ms());
            }
            Phase::Finished => break,
            Phase::Idle | Phase::SeekingQuiescence | Phase::ConfirmingReset | Phase::Settling | Phase::Measuring => {}
        }

        // A missing sample is not a zero count: leave the engine alone rather
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
        last_sample_ms = measurement.timestamp_ms;

        let tested = track_count(leg, measurement);
        let other = other_track_count(leg, measurement);
        if previous_tested != Some(tested) {
            previous_tested = Some(tested);
            last_tested_change_ms = measurement.timestamp_ms;
        }
        if previous_other != Some(other) {
            previous_other = Some(other);
            last_other_change_ms = measurement.timestamp_ms;
        }

        observer.sample(tested).await;

        engine.observe(EncoderSample {
            timestamp_ms: measurement.timestamp_ms,
            count: tested,
        });
    }

    // The window is over: brake the tested track before the caller publishes the
    // result, so a wheel is never left commanded between Power Steps.
    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

    let result = engine.result().unwrap_or(PointResult::Invalid(InvalidReason::NoWindow));
    debug!(
        "sweep: {=str} {=u8}% result={=str} rail={=bool}",
        leg_label(leg),
        power,
        result_label(result),
        rail_volts.is_some()
    );

    StepEnd::Measured(StepOutcome {
        result,
        rail_volts,
        compensation,
    })
}

/// Run one full sweep: four Test Legs of ten Power Steps, in cycle order.
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
        for (leg_index, leg) in motor_cal::LEG_ORDER.iter().copied().enumerate() {
            for (step_index, power) in STEP_POWERS.iter().copied().enumerate() {
                if claim.is_stop_requested() {
                    break 'cycle SweepEnd::Stopped;
                }
                if perception::is_floor_drop_detected() {
                    return SweepEnd::Aborted(FLOOR_DROP_REASON);
                }

                let step = StepRef {
                    leg,
                    power,
                    leg_index,
                    step_index,
                };
                // The bar shows where the operator is in the forty-step cycle, so
                // the last step reads 100 % while it runs; a finished view drops
                // the bar entirely.
                let percent = activity::percent_done(step.done() + 1, TOTAL_STEPS);
                claim.phase(ui_leg(leg).label(), Some(percent)).await;
                sink.step_started(step).await;

                match run_power_step(claim, leg, power, &mut *sink).await {
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

/// The left and right track commands for `leg` at `power` percent: only the
/// tested track is commanded, and the other is held at zero (a high-Z coast).
pub fn speeds(leg: TestLeg, power: u8) -> (i8, i8) {
    // The caller only ever passes one of the ten Power Steps, all at most 100 %.
    let magnitude = i8::try_from(power).unwrap_or(i8::MAX);
    let signed = match leg.direction {
        motor_cal::Direction::Forward => magnitude,
        motor_cal::Direction::Reverse => -magnitude,
    };
    match leg.track {
        motor_cal::Track::Left => (signed, 0),
        motor_cal::Track::Right => (0, signed),
    }
}

/// The encoder count of the track under test in `measurement`.
pub const fn track_count(leg: TestLeg, measurement: EncoderMeasurement) -> u16 {
    match leg.track {
        motor_cal::Track::Left => measurement.left,
        motor_cal::Track::Right => measurement.right,
    }
}

/// The encoder count of the untested track in `measurement`.
const fn other_track_count(leg: TestLeg, measurement: EncoderMeasurement) -> u16 {
    match leg.track {
        motor_cal::Track::Left => measurement.right,
        motor_cal::Track::Right => measurement.left,
    }
}

/// The curve in force at `leg` and `power`, as the panel shows it.
///
/// With no curve loaded the command is used unchanged (`1.00`); a curve that
/// corrects nothing at or below its Calibration Floor yields `raw`. This is the
/// single formatting of a correction for both the live readout and the summary.
pub fn correction(curve: Option<&Curve>, leg: TestLeg, power: u8) -> Correction {
    let Some(curve) = curve else {
        return Correction::Identity;
    };
    curve
        .correction(leg.track, leg.direction, power)
        .map_or(Correction::Raw, |factor| Correction::Factor(per_mille(factor)))
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

/// The panel's Test Leg for the measurement model's.
///
/// The two models name the same four legs; this is the one translation between
/// them, so the schedule, the phase line and the readout cannot disagree about
/// which leg is running.
pub const fn ui_leg(leg: TestLeg) -> touch_ui::TestLeg {
    touch_ui::TestLeg {
        track: match leg.track {
            motor_cal::Track::Left => touch_ui::Track::Left,
            motor_cal::Track::Right => touch_ui::Track::Right,
        },
        direction: match leg.direction {
            motor_cal::Direction::Forward => touch_ui::Direction::Forward,
            motor_cal::Direction::Reverse => touch_ui::Direction::Reverse,
        },
    }
}

/// The operator-facing name of a Test Leg, for the bench log.
pub const fn leg_label(leg: TestLeg) -> &'static str {
    match (leg.track, leg.direction) {
        (motor_cal::Track::Left, motor_cal::Direction::Forward) => "left forward",
        (motor_cal::Track::Left, motor_cal::Direction::Reverse) => "left reverse",
        (motor_cal::Track::Right, motor_cal::Direction::Forward) => "right forward",
        (motor_cal::Track::Right, motor_cal::Direction::Reverse) => "right reverse",
    }
}

/// A short name for a point's verdict, for the bench log.
const fn result_label(result: PointResult) -> &'static str {
    match result {
        PointResult::Valid(_) => "valid",
        PointResult::Invalid(reason) => reason.label(),
    }
}
