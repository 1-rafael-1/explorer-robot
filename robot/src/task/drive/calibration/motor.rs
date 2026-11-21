//! Motor calibration procedure.
//!
//! This is the writer of the measured **Motor Calibration Curve**. It runs the
//! same one-cycle sweep the Basic Motor Test displays — forward and reverse, each
//! over ten simultaneous pair Power Steps, with each track driven end to end by
//! its own [`motor_cal::MeasurementEngine`] through
//! [`crate::task::motor_sweep::run_sweep`] — and derives, stores and publishes the
//! curve from those measurements.
//!
//! # Lifecycle
//!
//! 1. Clear the panel leaf, arm the Floor-Drop Gate, and require a motor rail
//!    sample at or above the driver's 6 V compensation target. A missing sample
//!    after a short wait, or a low rail, refuses the run before any motion.
//! 2. Enable the drivers, start the encoder sampler at the sweep's 20 ms
//!    interval, and run the twenty pair steps. Each step publishes its
//!    direction and cycle position so the running screen shows where the run is.
//! 3. A structural problem — low rail, a dead encoder task, a floor drop — aborts
//!    immediately. The rail is checked before the run and again after every
//!    measured step, so a sag mid-sweep aborts rather than entering the curve.
//!    A Stop aborts without saving. A measurement problem finishes the whole
//!    sweep and then fails, so the operator sees the pattern.
//! 4. On a complete, valid sweep the run derives the curve, saves the versioned
//!    record, and waits for the flash task's reply. Only then does it publish the
//!    Result Report summary and complete. A failed save leaves the previous curve
//!    in force.
//!
//! Nothing is written flash-side unless the whole sweep measured cleanly, and a
//! failed run always leaves the previous curve in force. A Stop is honoured up to
//! the instant the save is dispatched; the write is one atomic `store_item` that
//! cannot be recalled after that.
//!
//! # The panel leaf
//!
//! The live six-row readout is not used here: the running screen carries the
//! Procedure's phase and progress. The Result Report reuses the panel-only
//! [`crate::system::state::motor_readout`] leaf that the Basic Motor Test also
//! publishes, because it is already the panel-only motor readout leaf and both
//! Procedures end in the same two tables. The leaf is cleared at the start and on
//! a Stop or structural abort, and carries the calibration's saved/not-saved
//! footer on a measurement or save failure.

use defmt::{info, warn};
use embassy_time::{Duration, Instant};
use motor_cal::{
    Curve, DistanceScale, InvalidReason, LEG_ORDER, MotorCalibrationRecord, POWER_STEPS, PointResult, StepMeasurement,
    TEST_LEGS, TestLeg,
};
use touch_ui::{BasicMotorSummary, BasicMotorSummaryBuilder, SummaryStatus};

use crate::{
    system::state::{calibration, motor_readout, power},
    task::{
        io::flash_storage,
        motor_driver::{self, MotorCommand, TARGET_MOTOR_VOLTAGE},
        motor_sweep::{self, PairOutcome, STEP_POWERS, StepAction, StepObserver, StepRef, SweepEnd, SweepSink},
        procedure::Claim,
        sensors::rangefinder,
    },
};

/// The reason a low rail refuses the run, or aborts it mid-sweep.
const LOW_RAIL_REASON: &str = "Motor rail below 6.0 V";

/// The reason no rail sample was available to judge.
const NO_RAIL_REASON: &str = "Motor rail unavailable";

/// How long to wait for a first rail sample, in milliseconds.
const RAIL_WAIT_TIMEOUT_MS: u64 = 1_000;

/// Poll interval while waiting for the rail sample, in milliseconds.
const RAIL_POLL_MS: u64 = 20;

/// Run the Motor Calibration Procedure under `claim`.
///
/// The menu entry, the Procedure identity and the landing screen are unchanged;
/// this body replaces the retired per-track multiplier with the measured curve.
/// The claim is already held and published, so the terminal call is made here on
/// every path.
pub async fn run_motor_calibration(claim: &Claim) {
    info!("=== Motor Calibration starting ===");

    // A new run never shows the previous run's numbers.
    motor_readout::clear().await;

    // This Procedure drives the motor driver directly, so it must run the same
    // Floor-Drop Gate the drive dispatch applies before a motor command.
    if let Err(refusal) = rangefinder::arm_for_motion().await {
        release_idle().await;
        claim.fail(refusal.reason()).await;
        return;
    }

    // Refuse to bake a sagging battery into the curve. This is a structural
    // refusal before any motion, so it fails without a Result Report summary.
    match wait_for_rail(claim).await {
        RailCheck::Ready => {}
        RailCheck::Low(volts) => {
            warn!("Motor Calibration refused: rail {} V is below the 6.0 V target", volts);
            release_idle().await;
            claim.fail(LOW_RAIL_REASON).await;
            return;
        }
        RailCheck::Missing => {
            warn!("Motor Calibration refused: no motor rail sample");
            release_idle().await;
            claim.fail(NO_RAIL_REASON).await;
            return;
        }
        RailCheck::Stopped => {
            release_idle().await;
            claim.finish().await;
            return;
        }
    }

    // The curve in force for the correction column if the run fails: one read, so
    // the operator compares the sweep against what the robot is applying now.
    let previous_curve = calibration::get_motor_curve().await;

    // The measured scale comes from Distance Calibration. A robot without one
    // measures against the inferred nominal fallback rather than refusing.
    let scale = calibration::get_distance_scale().await.unwrap_or_else(|| {
        info!("Motor Calibration: no measured distance scale; using the inferred nominal scale");
        DistanceScale::nominal()
    });

    let mut results = [[PointResult::Invalid(InvalidReason::NoWindow); POWER_STEPS]; TEST_LEGS];
    let mut measurements = [[StepMeasurement::new(0, 0, 0, 0.0, 1.0, false); POWER_STEPS]; TEST_LEGS];

    // The sweep owns the schedule, the stop and floor-drop checks and the encoder
    // sampler; this sink is the calibration's half: it keeps every measured point
    // and aborts the run if the rail sags below the compensation target.
    let end = {
        let mut sink = CalibrationSink {
            results: &mut results,
            measurements: &mut measurements,
        };
        motor_sweep::run_sweep(claim, &mut sink).await
    };

    match end {
        SweepEnd::Stopped => stop(claim).await,
        SweepEnd::Aborted(reason) => abort(claim, reason).await,
        SweepEnd::Complete => finish(claim, &results, &measurements, previous_curve.as_ref(), scale).await,
    }
}

/// Stores the calibration's own half of a sweep: the raw measurements.
///
/// The calibration shows no live six-row readout — the running screen carries the
/// Procedure's phases and progress — so it answers the sample hook with nothing.
struct CalibrationSink<'a> {
    /// Each track's engine verdict for every Power Step, in cycle order.
    results: &'a mut [[PointResult; POWER_STEPS]; TEST_LEGS],
    /// The stored measurement for every track and Power Step, in cycle order.
    measurements: &'a mut [[StepMeasurement; POWER_STEPS]; TEST_LEGS],
}

impl StepObserver for CalibrationSink<'_> {
    async fn sample(&mut self, _left: u16, _right: u16) {}
}

impl SweepSink for CalibrationSink<'_> {
    async fn step_started(&mut self, _step: StepRef) {}

    // The trait method is async for the sink that does await work; this sink only
    // records the outcome, so there is nothing to await here.
    #[allow(clippy::unused_async_trait_impl)]
    async fn step_measured(&mut self, step: StepRef, outcome: PairOutcome) -> StepAction {
        // A pair step writes the left and right Test Legs of its direction at the
        // same power, so the curve is derived from simultaneous measurements.
        let [left_leg, right_leg] = motor_sweep::legs(step.direction);
        let step_index = step.step_index();
        let left_index = motor_sweep::leg_index(left_leg);
        let right_index = motor_sweep::leg_index(right_leg);

        self.results[left_index][step_index] = outcome.left;
        self.results[right_index][step_index] = outcome.right;
        self.measurements[left_index][step_index] =
            step_measurement(step.power, outcome.left, outcome.rail_volts, outcome.compensation);
        self.measurements[right_index][step_index] =
            step_measurement(step.power, outcome.right, outcome.rail_volts, outcome.compensation);

        log_step(left_leg, step.power, outcome.left, outcome.rail_volts);
        log_step(right_leg, step.power, outcome.right, outcome.rail_volts);
        // A rail that sags below the compensation target mid-run would bake a
        // sagging battery into the curve, so this is a structural abort, not a
        // measurement verdict. Both tracks share the one rail sample.
        if outcome.rail_volts.is_some_and(|volts| volts < TARGET_MOTOR_VOLTAGE) {
            warn!("Motor Calibration aborted: rail fell below the 6.0 V target mid-run");
            StepAction::Abort(LOW_RAIL_REASON)
        } else {
            StepAction::Continue
        }
    }
}

/// The result of the pre-run rail check.
enum RailCheck {
    /// The rail is at or above the compensation target.
    Ready,
    /// The rail is measured but below the target, with the measured volts.
    Low(f32),
    /// No sample arrived within the short wait.
    Missing,
    /// The operator pressed Stop while the check waited.
    Stopped,
}

/// Wait for a motor rail sample and judge it against the 6 V target.
async fn wait_for_rail(claim: &Claim) -> RailCheck {
    let deadline = Instant::now() + Duration::from_millis(RAIL_WAIT_TIMEOUT_MS);
    loop {
        if claim.is_stop_requested() {
            return RailCheck::Stopped;
        }
        if let Some((volts, _)) = power::try_get_rail_sample() {
            return if volts < TARGET_MOTOR_VOLTAGE {
                RailCheck::Low(volts)
            } else {
                RailCheck::Ready
            };
        }
        if Instant::now() >= deadline {
            return RailCheck::Missing;
        }
        if claim.wait_or_stop(RAIL_POLL_MS).await {
            return RailCheck::Stopped;
        }
    }
}

/// Stop the sampler, coast both tracks and put the drivers in standby.
async fn release_idle() {
    motor_sweep::stop_encoders().await;
    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
}

/// End a stopped run: no Result Report and nothing of this run left on the panel.
async fn stop(claim: &Claim) {
    motor_readout::clear().await;
    release_idle().await;
    claim.finish().await;
}

/// End a structurally aborted run: brake, then fail with the reason.
///
/// This is the Basic Motor Test's fault path: the brake stays latched and the
/// drivers stay enabled until the next activity manages them. The leaf is
/// cleared so the reason is the result rather than an empty table.
async fn abort(claim: &Claim, reason: &'static str) {
    motor_readout::clear().await;
    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
    claim.fail(reason).await;
}

/// Derive, save and complete the calibration from a finished sweep.
async fn finish(
    claim: &Claim,
    results: &[[PointResult; POWER_STEPS]; TEST_LEGS],
    measurements: &[[StepMeasurement; POWER_STEPS]; TEST_LEGS],
    previous_curve: Option<&Curve>,
    scale: DistanceScale,
) {
    // A Stop that landed after the last Power Step but before any reporting must
    // still abort without writing.
    if claim.is_stop_requested() {
        stop(claim).await;
        return;
    }

    // An invalid point is a measurement problem: the sweep finished, so the
    // operator can see the pattern, but nothing is saved and the reason stays.
    if let Some(reason) = first_invalid(results) {
        warn!("Motor Calibration measurement problem: {}", reason.label());
        let summary = build_summary(results, previous_curve).with_status(SummaryStatus::NotSaved(reason.label()));
        fail_with_summary(claim, summary, reason.label()).await;
        return;
    }

    let run_id = next_run_id(calibration::get_motor_run_id().await);
    let record = match MotorCalibrationRecord::new(run_id, scale, *measurements) {
        Ok(record) => record,
        Err(error) => {
            // A derivation failure is also a measurement problem: save nothing and
            // leave the previous curve in force.
            warn!("Motor Calibration curve derivation failed: {}", error.label());
            let summary = build_summary(results, previous_curve).with_status(SummaryStatus::NotSaved(error.label()));
            fail_with_summary(claim, summary, error.label()).await;
            return;
        }
    };

    // The Result Report shows the corrections of the curve this run just derived.
    let derived_curve = record.curve().copied();

    // The flash write is one atomic `store_item` and cannot be interrupted once
    // it is dispatched, so this is the last moment a Stop can be honoured. A Stop
    // that lands in the microseconds after the dispatch is not covered by the
    // "saves nothing" guarantee; the reply below reports what flash actually
    // did.
    if claim.is_stop_requested() {
        stop(claim).await;
        return;
    }

    match flash_storage::save_motor_curve(record).await {
        Ok(()) => {
            let summary = build_summary(results, derived_curve.as_ref()).with_status(SummaryStatus::Saved);
            // Stop the motors before the report appears: a Result Report and
            // motor motion must never coexist on the bench.
            release_idle().await;
            motor_readout::publish_summary(summary).await;
            claim.complete("Calibration saved").await;
        }
        Err(reason) => {
            // The write did not confirm: the previous curve is still the one in
            // force, and the footer names the reason flash reported.
            warn!("Motor Calibration save failed: {}", reason);
            let summary = build_summary(results, previous_curve).with_status(SummaryStatus::NotSaved(reason));
            fail_with_summary(claim, summary, reason).await;
        }
    }
}

/// Stop the motors, publish the Result Report summary, then fail with `reason`.
async fn fail_with_summary(claim: &Claim, summary: BasicMotorSummary, reason: &'static str) {
    // Stop the motors before the report appears: a Result Report and motor
    // motion must never coexist on the bench.
    release_idle().await;
    motor_readout::publish_summary(summary).await;
    claim.fail(reason).await;
}

/// The next run id: one more than `current`, never zero.
const fn next_run_id(current: u32) -> u32 {
    let next = current.wrapping_add(1);
    if next == 0 { 1 } else { next }
}

/// The first invalid Power Step's reason, in cycle order, if any.
fn first_invalid(results: &[[PointResult; POWER_STEPS]; TEST_LEGS]) -> Option<InvalidReason> {
    for leg in results {
        for result in leg {
            if let PointResult::Invalid(reason) = *result {
                return Some(reason);
            }
        }
    }
    None
}

/// Build the stored measurement for one track's Power Step.
///
/// An invalid point carries no count and no window; the sweep is failed before
/// the record is built, so those zeros can never be saved as a curve.
fn step_measurement(
    power: u8,
    result: PointResult,
    rail_volts: Option<f32>,
    compensation: Option<f32>,
) -> StepMeasurement {
    let (count, window_ms, moved) = match result {
        PointResult::Valid(measurement) => (
            measurement.count(),
            u32::try_from(measurement.window_ms()).unwrap_or(u32::MAX),
            measurement.sustained_motion(),
        ),
        PointResult::Invalid(_) => (0, 0, false),
    };
    StepMeasurement::new(
        power,
        count,
        window_ms,
        rail_volts.unwrap_or(0.0),
        compensation.unwrap_or(1.0),
        moved,
    )
}

/// Build the Result Report summary from the sweep's outcomes and `curve`.
///
/// `curve` is the newly derived curve on success and the curve in force on a
/// failure, so the correction column always names what the robot is applying.
fn build_summary(results: &[[PointResult; POWER_STEPS]; TEST_LEGS], curve: Option<&Curve>) -> BasicMotorSummary {
    let mut builder = BasicMotorSummaryBuilder::new();
    for (leg_index, leg) in LEG_ORDER.iter().copied().enumerate() {
        let ui = motor_sweep::ui_leg(leg);
        for (step_index, power) in STEP_POWERS.iter().copied().enumerate() {
            let measurement = results[leg_index][step_index].measurement();
            let count = measurement.map(|measurement| measurement.count());
            let moved = measurement.is_some_and(|measurement| measurement.sustained_motion());
            builder.record(
                ui,
                power,
                count,
                moved,
                motor_sweep::correction(curve, leg.track, leg.direction, power),
            );
        }
    }
    builder.build()
}

/// Log one Power Step's verdict at info, so a bench log stays readable.
fn log_step(leg: TestLeg, power: u8, result: PointResult, rail_volts: Option<f32>) {
    match result {
        PointResult::Valid(measurement) => info!(
            "motor-cal: {=str} {=u8}% count={=u16} speed={=u32}/s window={=u64}ms rail={=bool}",
            motor_sweep::leg_label(leg),
            power,
            measurement.count(),
            measurement.speed_counts_per_s(),
            measurement.window_ms(),
            rail_volts.is_some()
        ),
        PointResult::Invalid(reason) => warn!(
            "motor-cal: {=str} {=u8}% invalid: {=str}",
            motor_sweep::leg_label(leg),
            power,
            reason.label()
        ),
    }
}
