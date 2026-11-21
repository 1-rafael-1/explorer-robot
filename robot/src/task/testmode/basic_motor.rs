//! Basic motor test mode task.
//!
//! Runs exactly one cycle — four Test Legs (left forward, left reverse, right
//! forward, right reverse) of ten Power Steps from 10 % to 100 % — and then
//! completes. It does not loop: the run ends in a held Result Report, and the
//! operator's Back dismisses it.
//!
//! The cycle itself is [`crate::task::motor_sweep::run_sweep`]; every Power Step
//! is driven end to end by [`motor_cal::MeasurementEngine`] through
//! [`crate::task::motor_sweep::run_power_step`]: both tracks are braked and
//! Quiescence is established, the counters are reset and confirmed with a fresh
//! sample at zero, only the tested track is commanded while the other is held at
//! zero, the settle interval is discarded, and the Measurement Window's wrap-safe
//! delta is the point's count. An invalid point carries its reason and is shown
//! as `--`, never as a silent zero. The command is raw — the sweep measures the
//! drivetrain, not the loaded curve — while the readout still shows the
//! correction in force as information.
//!
//! This task supplies only the test's half of the sweep: it draws the live
//! Basic Motor Test Readout and records the Result Report summary. The panel's
//! readout travels in the shared panel-only `motor_readout` leaf: the task
//! publishes the typed readout as samples arrive and the UI formats and draws it.
//! On success the task also builds the Result Report summary from the forty Power
//! Step outcomes and publishes it to the same leaf before the Procedure completes,
//! so the finished view draws both tables. The encoder count never enters Activity
//! State or the Link Protocol. The readout and the summary survive completion so
//! the Result Report has its numbers, and the next run clears both at the start.

use defmt::{error, info};
use embassy_executor::Spawner;
use motor_cal::{Curve, PointResult};
use touch_ui::{BasicMotorReadout, BasicMotorSummaryBuilder, Correction, StepIndex};

use crate::{
    system::state::{calibration, motor_readout},
    task::{
        motor_driver::{self, MotorCommand},
        motor_sweep::{self, StepAction, StepObserver, StepOutcome, StepRef, SweepEnd, SweepSink},
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

    // The curve in force for the whole run: one read, so the correction row
    // names what the robot is actually applying at each Power Step. A missing
    // curve is identity rather than an error (user story 24).
    let curve = calibration::get_motor_curve().await;

    // One readout follows the whole run. It names the step it is commanding and
    // holds the live count across the encoder reset between steps; the task only
    // drives its transitions and publishes copies.
    let mut readout = BasicMotorReadout::waiting();
    // The Result Report summary collects the same outcomes the readout sees, then
    // is built once on success and published for the finished view.
    let mut summary = BasicMotorSummaryBuilder::new();

    let end = {
        let mut sink = ReadoutSink {
            readout: &mut readout,
            summary: &mut summary,
            curve,
            applied: Correction::Identity,
        };
        motor_sweep::run_sweep(&claim, &mut sink).await
    };

    match end {
        SweepEnd::Complete => {
            // A normal Procedure outcome: the summary and the readout stay
            // published for the Result Report, which holds until the operator's
            // Back. The summary goes first so the finished view has its tables on
            // the very first draw.
            motor_readout::publish_summary(summary.build()).await;
            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
            claim.complete("Basic motor test complete").await;
        }
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
}

/// Draws the live readout and collects the Result Report while the sweep runs.
///
/// The sweep owns the schedule and the step; this sink is the test's half: it
/// names the step on the panel before it commands the track, publishes every
/// fresh count, and copies each measured point into the summary builder.
struct ReadoutSink<'a> {
    /// The readout the whole run drives and publishes.
    readout: &'a mut BasicMotorReadout,
    /// The Result Report summary the run collects and builds on success.
    summary: &'a mut BasicMotorSummaryBuilder,
    /// The curve in force for the run, read once before the first step.
    curve: Option<Curve>,
    /// The correction applied at the current step, resolved in
    /// [`step_started`](SweepSink::step_started) and recorded by
    /// [`step_measured`](SweepSink::step_measured) with the same step.
    applied: Correction,
}

impl StepObserver for ReadoutSink<'_> {
    async fn sample(&mut self, count: u16) {
        self.readout.sample(count);
        motor_readout::publish(*self.readout).await;
    }
}

impl SweepSink for ReadoutSink<'_> {
    async fn step_started(&mut self, step: StepRef) {
        // Name the step on the panel, and the correction in force, before it
        // commands the track, so a count can never appear under the previous
        // step's label. The start keeps the last displayed count: the encoder
        // reset has cleared the hardware, not the operator's reading.
        self.applied = motor_sweep::correction(self.curve.as_ref(), step.leg, step.power);
        self.readout.start_step(
            motor_sweep::ui_leg(step.leg),
            step.power,
            step_position(step.done()),
            self.applied,
        );
        motor_readout::publish(*self.readout).await;
    }

    async fn step_measured(&mut self, step: StepRef, outcome: StepOutcome) -> StepAction {
        let measurement = outcome.result.measurement();
        let count = measurement.map(|measurement| measurement.count());
        let moved = measurement.is_some_and(|measurement| measurement.sustained_motion());
        // User story 8: the `Last` row shows a count only for a step that showed
        // sustained motion; a step that never moved or never settled reads `--`,
        // not a silent zero. The summary still records the raw count and the
        // `moved` verdict.
        let shown = if moved { count } else { None };
        self.readout.complete(shown, moved);
        self.summary
            .record(motor_sweep::ui_leg(step.leg), step.power, count, moved, self.applied);
        motor_readout::publish(*self.readout).await;
        log_step(motor_sweep::ui_leg(step.leg), step.power, outcome.result);
        StepAction::Continue
    }
}

/// Log one Power Step's verdict at info, so a bench log stays readable.
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

/// The one-based position of a Power Step within the cycle, as the `Step` row
/// shows it, from the zero-based number of steps done before it.
fn step_position(done: usize) -> StepIndex {
    StepIndex {
        position: u8::try_from(done + 1).unwrap_or(u8::MAX),
        total: u8::try_from(motor_sweep::TOTAL_STEPS).unwrap_or(u8::MAX),
    }
}
