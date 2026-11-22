//! Basic motor test mode task.
//!
//! Runs one raw cycle and, when a Motor Calibration Curve is loaded, two
//! curve-applied verification cycles, then completes. It does not loop: the run
//! ends in a held Result Report, and the operator's Back dismisses it. The cycles
//! answer different questions:
//!
//! - the **raw** cycle commands `SetTracksRaw`, so it measures the drivetrain and
//!   is the same sweep the Motor Calibration measures;
//! - the **corrected** cycles command `SetTracks`, so the loaded curve is applied,
//!   and the worst mean Track Divergence at 30–100 % is the accuracy the operator
//!   can actually expect.
//!
//! The cycle itself is [`crate::task::motor_sweep::run_sweep`]; every pair step is
//! driven end to end by two [`motor_cal::MeasurementEngine`]s through
//! [`crate::task::motor_sweep::run_power_pair`]: both tracks are braked and
//! Quiescence is established, the counters are reset and confirmed with a fresh
//! sample at zero, both tracks are commanded at the same requested power, the
//! settle interval is discarded, and each Measurement Window's wrap-safe delta is
//! that track's count. An invalid point carries its reason and is shown as `--`,
//! never as a silent zero.
//!
//! Before either cycle the task runs a discarded 20-step raw pair warm-up
//! ([`crate::task::motor_sweep::run_warm_up`]), so cold low-duty friction does not
//! bias the pair. Its results are thrown away; a Stop or floor drop during it ends
//! the run exactly as the measured sweeps would. The panel says `Warming up` for
//! its duration, because none of those numbers are results.
//!
//! This task supplies only the test's half of each sweep: it draws the live Basic
//! Motor Test Readout and records the Result Report summary. The panel's readout
//! travels in the shared panel-only `motor_readout` leaf: the task publishes the
//! typed readout as samples arrive and the UI formats and draws it. On success the
//! task also builds the Result Report summary from the raw twenty pair-step
//! outcomes, adds the corrected verification's worst point, and publishes it to
//! the same leaf before the Procedure completes, so the finished view draws both
//! tables and the verification line. The encoder counts never enter Activity State
//! or the Link Protocol. The readout and the summary survive completion so the
//! Result Report has its numbers, and the next run clears both at the start.

use defmt::{error, info};
use embassy_executor::Spawner;
use motor_cal::{Curve, PointResult, Track, VERIFICATION_MIN_POWER, VERIFICATION_REPEATS, Verification};
use touch_ui::{BasicMotorReadout, BasicMotorSummaryBuilder, CorrectedCheck, CorrectedMax, Correction, StepIndex};

use crate::{
    system::state::{calibration, motor_readout},
    task::{
        motor_driver::{self, MotorCommand},
        motor_sweep::{self, PairOutcome, StepAction, StepObserver, StepRef, SweepEnd, SweepMode, SweepSink},
        procedure::Claim,
    },
};

/// Spawn the basic motor test task via the controller.
///
/// The caller's claim travels with the task. Returns `false` when the spawn failed,
/// so the controller can abandon the claim rather than leave its running screen up
/// with no task behind it.
#[must_use]
pub(super) fn spawn(spawner: Spawner, claim: Claim) -> bool {
    basic_motor_test_task(claim).map_or_else(
        |_| {
            error!("basic-motor: spawn failed");
            false
        },
        |token| {
            spawner.spawn(token);
            true
        },
    )
}

/// Basic motor test mode runner: one cycle, then a held Result Report.
#[embassy_executor::task]
async fn basic_motor_test_task(claim: Claim) {
    // A new run never shows the previous run's numbers.
    motor_readout::clear().await;

    // This test drives the motor driver directly, so it must run the same
    // Floor-Drop Gate the drive dispatch applies before a motor command.
    if let Err(refusal) = crate::task::sensors::rangefinder::arm_for_motion().await {
        claim.fail(refusal.reason()).await;
        return;
    }

    // One readout follows the whole run. It names the pair it is commanding and
    // holds the live counts across the encoder reset between steps; the task only
    // drives its transitions and publishes copies.
    let mut readout = BasicMotorReadout::waiting();
    // The Result Report summary collects the same outcomes the readout sees, then
    // is built once on success and published for the finished view.
    let mut summary = BasicMotorSummaryBuilder::new();

    // The curve in force for the whole run: one read, so the correction row names
    // what the robot is actually applying at each Power Step. A missing curve is
    // identity rather than an error (user story 24); the corrected verification is
    // skipped because there is nothing to verify. Read it before the warm-up so the
    // progress total knows whether those cycles will run.
    let curve = calibration::get_motor_curve().await;

    // One bar spans every stage: the discarded warm-up, the raw cycle, and — when
    // a curve is loaded — both corrected cycles. It advances once instead of
    // resetting on each sweep.
    let total_steps = if curve.is_some() {
        4 * motor_sweep::TOTAL_STEPS
    } else {
        2 * motor_sweep::TOTAL_STEPS
    };
    let mut progress = motor_sweep::SweepProgress::whole(total_steps);

    // Publish the readout before the warm-up so the panel says "Warming up" while
    // the progress bar advances instead of showing a blank readout. The measured
    // sweep's first `start_pair` ends the warm-up state.
    readout.warming_up();
    motor_readout::publish(readout).await;

    // Warm the drivetrain with a discarded raw pair sweep before measuring, so
    // cold low-duty friction does not bias the pair. A Stop or structural abort
    // here ends the run through the same terminal paths as the measured sweep.
    if abandon(motor_sweep::run_warm_up(Some(&claim), progress).await, &claim).await {
        return;
    }
    progress = progress.after(motor_sweep::TOTAL_STEPS);

    // First cycle: the raw drivetrain. This is the sweep the calibration also
    // measures, so the raw tables and the curve's inputs stay the same numbers.
    let raw_end = {
        let mut sink = ReadoutSink {
            readout: &mut readout,
            summary: &mut summary,
            curve,
            applied_left: Correction::Identity,
            applied_right: Correction::Identity,
        };
        motor_sweep::run_sweep(&claim, &mut sink, SweepMode::Raw, progress).await
    };
    if abandon(raw_end, &claim).await {
        return;
    }
    progress = progress.after(motor_sweep::TOTAL_STEPS);

    // Second and third cycles: the same pairs through `SetTracks`, so the saved
    // curve is applied. Two repeats make the reported divergence robust against a
    // single noisy window. This is the number the operator can use to judge the
    // calibration, while the raw counts above are the drivetrain, not the
    // calibration error.
    let corrected = if curve.is_some() {
        let mut verification = Verification::new();
        for pass in 1..=VERIFICATION_REPEATS {
            info!("basic-motor: corrected pass {=u8}/{=u8}", pass, VERIFICATION_REPEATS);
            let mut sink = CorrectedSink {
                readout: &mut readout,
                curve,
                verification: &mut verification,
            };
            let corrected_end = motor_sweep::run_sweep(&claim, &mut sink, SweepMode::Corrected, progress).await;
            if abandon(corrected_end, &claim).await {
                return;
            }
            progress = progress.after(motor_sweep::TOTAL_STEPS);
        }
        finish_verification(verification)
    } else {
        info!("basic-motor: no curve; corrected verification skipped");
        CorrectedCheck::NoCurve
    };

    // A normal Procedure outcome: the summary and the readout stay published for
    // the Result Report, which holds until the operator's Back. The summary goes
    // first so the finished view has its tables on the very first draw.
    motor_readout::publish_summary(summary.build().with_corrected_check(corrected)).await;
    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
    claim.complete("Basic motor test complete").await;
}

/// End the run when a warm-up did not complete, and report whether it did.
///
/// The two arms match the measured sweep's Stopped and Aborted terminal paths:
/// a Stop coasts and clears the readout, while a structural abort keeps the brake
/// latched. `Complete` returns `false` so the caller proceeds into the measured
/// cycle unchanged.
async fn abandon(end: SweepEnd, claim: &Claim) -> bool {
    match end {
        SweepEnd::Complete => return false,
        SweepEnd::Stopped => {
            // An operator Stop ends the run with no Result Report, so nothing of
            // this run may linger on the panel.
            motor_readout::clear().await;
            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
            claim.finish().await;
        }
        SweepEnd::Aborted(reason) => {
            // The floor-drop EmergencyBrake (or the structural abort that took its
            // path) owns this stop: keep the brake latched and leave the drivers
            // enabled until the next activity manages them. The numbers go,
            // because the reason is the result.
            motor_readout::clear().await;
            motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            claim.fail(reason).await;
        }
    }
    true
}

/// Draws the live readout and collects the Result Report while the sweep runs.
///
/// The sweep owns the schedule and the pair step; this sink is the test's half: it
/// names the pair on the panel before it commands the tracks, publishes every
/// fresh pair of counts, and copies each track's measured point into the summary
/// builder.
struct ReadoutSink<'a> {
    /// The readout the whole run drives and publishes.
    readout: &'a mut BasicMotorReadout,
    /// The Result Report summary the run collects and builds on success.
    summary: &'a mut BasicMotorSummaryBuilder,
    /// The curve in force for the run, read once before the first step.
    curve: Option<Curve>,
    /// The correction the curve in force applies to the left track at the current
    /// step, resolved in [`step_started`](SweepSink::step_started) and recorded by
    /// [`step_measured`](SweepSink::step_measured) with the same step.
    applied_left: Correction,
    /// The correction the curve in force applies to the right track at the
    /// current step.
    applied_right: Correction,
}

impl StepObserver for ReadoutSink<'_> {
    async fn sample(&mut self, left: u16, right: u16) {
        self.readout.sample(left, right);
        motor_readout::publish(*self.readout).await;
    }
}

impl SweepSink for ReadoutSink<'_> {
    async fn step_started(&mut self, step: StepRef) {
        // Name the pair on the panel, and the correction in force for each track,
        // before it commands the wheels, so a count can never appear under the
        // previous pair's label. The start keeps the last displayed counts: the
        // encoder reset has cleared the hardware, not the operator's reading.
        self.applied_left = motor_sweep::correction(self.curve.as_ref(), Track::Left, step.direction, step.power);
        self.applied_right = motor_sweep::correction(self.curve.as_ref(), Track::Right, step.direction, step.power);
        self.readout.start_pair(
            motor_sweep::ui_direction(step.direction),
            step.power,
            step_position(step),
            self.applied_left,
            self.applied_right,
        );
        motor_readout::publish(*self.readout).await;
    }

    async fn step_measured(&mut self, step: StepRef, outcome: PairOutcome) -> StepAction {
        let [left_leg, right_leg] = motor_sweep::legs(step.direction);
        let left = outcome.left.measurement();
        let right = outcome.right.measurement();
        let left_count = left.map(|measurement| measurement.count());
        let right_count = right.map(|measurement| measurement.count());
        let left_moved = left.is_some_and(|measurement| measurement.sustained_motion());
        let right_moved = right.is_some_and(|measurement| measurement.sustained_motion());

        // User story 8: the `Last` row shows a count only for a side that showed
        // sustained motion; a side that never moved or never settled reads `--`,
        // not a silent zero. The summary still records the raw count and the
        // `moved` verdict.
        let shown_left = if left_moved { left_count } else { None };
        let shown_right = if right_moved { right_count } else { None };
        self.readout.complete(shown_left, shown_right, left_moved, right_moved);

        self.summary.record(
            motor_sweep::ui_leg(left_leg),
            step.power,
            left_count,
            left_moved,
            self.applied_left,
        );
        self.summary.record(
            motor_sweep::ui_leg(right_leg),
            step.power,
            right_count,
            right_moved,
            self.applied_right,
        );
        motor_readout::publish(*self.readout).await;
        log_step(motor_sweep::ui_leg(left_leg), step.power, outcome.left);
        log_step(motor_sweep::ui_leg(right_leg), step.power, outcome.right);
        StepAction::Continue
    }
}

/// Log one track's verdict at info, so a bench log stays readable.
fn log_step(leg: touch_ui::TestLeg, power: u8, result: PointResult) {
    match result {
        PointResult::Valid(measurement) => info!(
            "basic-motor: {=str} {=u8}% count={=u16} speed={=u32}/s",
            leg.label(),
            power,
            measurement.count(),
            measurement.speed_counts_per_s()
        ),
        PointResult::Invalid(reason) => info!(
            "basic-motor: {=str} {=u8}% invalid: {=str}",
            leg.label(),
            power,
            reason.label()
        ),
    }
}

/// Draws and measures the curve-applied verification cycle.
///
/// This is the same sweep through the normal `SetTracks` command, so the loaded
/// curve is applied. Its purpose is not to measure the drivetrain — the raw cycle
/// already did that — but to measure how closely the two tracks actually match
/// once the calibration has corrected them. Each pass records every gated point
/// in [`Verification`], which averages the repeats for the Result Report's
/// `Corr max` line.
struct CorrectedSink<'a> {
    /// The readout the whole run drives and publishes.
    readout: &'a mut BasicMotorReadout,
    /// The curve in force for the run, read once before the first pass.
    curve: Option<Curve>,
    /// The accumulated per-point divergences across the repeats.
    verification: &'a mut Verification,
}

/// Log the verified means and return the Result Report's verdict.
///
/// The report only names a maximum when every gated Power Step has both repeats;
/// a point measured once cannot become the headline. Missing gated points are
/// logged so an incomplete verification is visible in the bench log as well as
/// the `--` footer.
fn finish_verification(verification: Verification) -> CorrectedCheck {
    let report = verification.finish();

    for direction in motor_sweep::DIRECTIONS {
        let direction_index = direction.index();
        for (step_index, mean) in report.means[direction_index].iter().enumerate() {
            let power = motor_cal::POWER_STEP_PERCENTS[step_index];
            if let Some(mean) = mean {
                info!(
                    "basic-motor: corrected-mean {=str} {=u8}% div={=f32}",
                    motor_sweep::direction_label(direction),
                    power,
                    mean
                );
            } else if power >= VERIFICATION_MIN_POWER {
                info!(
                    "basic-motor: corrected missing {=str} {=u8}%",
                    motor_sweep::direction_label(direction),
                    power
                );
            }
        }
        if let Some(mean) = report.low_power_means[direction_index] {
            info!(
                "basic-motor: corrected low-power {=str} {=f32}% (informational, below {=u8}%)",
                motor_sweep::direction_label(direction),
                mean,
                VERIFICATION_MIN_POWER
            );
        }
    }

    if !report.complete {
        info!("basic-motor: corrected verification incomplete; max unavailable");
        return CorrectedCheck::Unavailable;
    }

    report.max.map_or_else(
        || {
            info!("basic-motor: corrected max unavailable");
            CorrectedCheck::Unavailable
        },
        |max| {
            info!("basic-motor: corrected max={=f32}", f32::from(max.tenths) / 10.0);
            CorrectedCheck::Measured(CorrectedMax {
                tenths: max.tenths,
                direction: motor_sweep::ui_direction(max.direction),
                power: max.power,
            })
        },
    )
}

impl StepObserver for CorrectedSink<'_> {
    async fn sample(&mut self, left: u16, right: u16) {
        self.readout.sample(left, right);
        motor_readout::publish(*self.readout).await;
    }
}

impl SweepSink for CorrectedSink<'_> {
    async fn step_started(&mut self, step: StepRef) {
        let left = motor_sweep::correction(self.curve.as_ref(), Track::Left, step.direction, step.power);
        let right = motor_sweep::correction(self.curve.as_ref(), Track::Right, step.direction, step.power);
        self.readout.start_corrected_pair(
            motor_sweep::ui_direction(step.direction),
            step.power,
            step_position(step),
            left,
            right,
        );
        motor_readout::publish(*self.readout).await;
    }

    async fn step_measured(&mut self, step: StepRef, outcome: PairOutcome) -> StepAction {
        let left = outcome.left.measurement();
        let right = outcome.right.measurement();
        let left_count = left.map(|measurement| measurement.count());
        let right_count = right.map(|measurement| measurement.count());
        let left_moved = left.is_some_and(|measurement| measurement.sustained_motion());
        let right_moved = right.is_some_and(|measurement| measurement.sustained_motion());

        let shown_left = if left_moved { left_count } else { None };
        let shown_right = if right_moved { right_count } else { None };
        self.readout.complete(shown_left, shown_right, left_moved, right_moved);
        motor_readout::publish(*self.readout).await;

        if left_moved && right_moved {
            if let (Some(left_count), Some(right_count)) = (left_count, right_count) {
                if let Some(divergence) = motor_cal::divergence_percent(left_count, right_count) {
                    info!(
                        "basic-motor: corrected {=str} {=u8}% L={=u16} R={=u16} div={=f32}",
                        motor_sweep::direction_label(step.direction),
                        step.power,
                        left_count,
                        right_count,
                        divergence
                    );
                    self.verification
                        .record(step.direction, step.power, left_count, right_count);
                } else {
                    info!(
                        "basic-motor: corrected {=str} {=u8}% L={=u16} R={=u16} no counts",
                        motor_sweep::direction_label(step.direction),
                        step.power,
                        left_count,
                        right_count
                    );
                }
            } else {
                info!(
                    "basic-motor: corrected {=str} {=u8}% invalid",
                    motor_sweep::direction_label(step.direction),
                    step.power
                );
            }
        } else {
            info!(
                "basic-motor: corrected {=str} {=u8}% invalid: no sustained motion",
                motor_sweep::direction_label(step.direction),
                step.power
            );
        }

        StepAction::Continue
    }
}

/// The one-based position of a pair step within the cycle, as the `Index` row
/// shows it.
const fn step_position(step: StepRef) -> StepIndex {
    StepIndex {
        position: step.position(),
        total: step.total,
    }
}
