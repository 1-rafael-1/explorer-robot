//! DMP Compass Calibration procedure: the guided movement ceremony.
//!
//! The runner selects nine-axis fusion, starts IMU readings, and walks the
//! operator through the six Compass Steps of the DMP Compass Calibration. It
//! publishes a typed [`touch_ui::DmpCalibrationSnapshot`] at the panel's 10 Hz
//! telemetry rate for the Prompt screen; Activity State carries only the
//! lifecycle phase. `Done` and `Redo` reach the runner through [`DMP_ACTION`],
//! sent by the UI task when the Prompt reports them.
//!
//! Ticket 03 covers the movement ceremony only: after the sixth step it records
//! "Movement complete — not saved" and stops. It runs no Compass Check, reads no
//! DMP bias and writes no flash; ticket 04 replaces the sixth-step completion
//! branch with the two checks and adds the snapshot save.
//!
//! On every exit path the fusion mode returns to [`DEFAULT_FUSION_MODE`], IMU
//! readings stop, and the panel-only snapshot is cleared.

use defmt::{info, warn};
use dmp_cal::{CompassCalibration, CompassStep, MotionHint, MotionSample, compass_heading};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Instant;
use touch_ui::{DmpCalibrationSnapshot, DmpCheckState, DmpHint, PromptAction};

use crate::{
    system::state::dmp_cal as dmp_cal_state,
    task::{
        procedure::Claim,
        sensors::imu::{
            DEFAULT_FUSION_MODE, DmpFusionMode, ImuReadings, get_latest_readings, is_mag_available,
            set_dmp_fusion_mode, start_imu_readings, stop_imu_readings,
        },
    },
};

/// The panel's existing telemetry rate: one snapshot per 100 ms.
const PROMPT_INTERVAL_MS: u64 = 100;

/// The temporary result this slice records after the sixth step.
///
/// Ticket 04 replaces it with the Compass Check outcome and the saved snapshot.
const RESULT_NOT_SAVED: &str = "Movement complete — not saved";

/// The reason recorded when the magnetometer never initialised.
const MAG_UNAVAILABLE: &str = "Magnetometer unavailable";

/// An operator action from the Prompt screen.
///
/// The panel names what was tapped; the runner decides what it means for the
/// running Procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DmpAction {
    /// Accept the current step and advance.
    Done,
    /// Restart the movement sequence. Only the Compass Checks offer it.
    Redo,
}

impl From<PromptAction> for DmpAction {
    /// Map the panel's neutral action onto this Procedure's action.
    fn from(action: PromptAction) -> Self {
        match action {
            PromptAction::Done => Self::Done,
            PromptAction::Redo => Self::Redo,
        }
    }
}

/// Operator actions waiting for the runner.
///
/// Capacity two is enough for a Done followed by a Redo: the runner drains the
/// queue on every 100 ms tick, so a tap is never far ahead of the loop.
static DMP_ACTION: Channel<CriticalSectionRawMutex, DmpAction, 2> = Channel::new();

/// Deliver one operator action to the running calibration.
///
/// Never blocks: the UI task must stay responsive, so a full queue drops the
/// tap and logs it. The runner drains the queue each tick.
pub fn send_action(action: DmpAction) {
    if DMP_ACTION.try_send(action).is_err() {
        warn!("dmp cal: operator action dropped — queue full");
    }
}

/// Starts the IMU in nine-axis fusion and restores the default on drop.
///
/// The guard makes the restore and the stop unconditional on every exit path,
/// including Stop and a failure. It sends the mode before the start because the
/// IMU command queue is ordered.
struct ReadingsGuard;

impl ReadingsGuard {
    /// Select nine-axis fusion and start the IMU readings.
    fn start() -> Self {
        set_dmp_fusion_mode(DmpFusionMode::Axis9);
        start_imu_readings();
        Self
    }
}

impl Drop for ReadingsGuard {
    fn drop(&mut self) {
        set_dmp_fusion_mode(DEFAULT_FUSION_MODE);
        stop_imu_readings();
    }
}

/// How the movement ceremony ended.
enum MovementOutcome {
    /// The sixth movement step was accepted.
    Complete,
    /// The operator stopped the Procedure.
    Stopped,
}

/// Run the DMP Compass Calibration movement ceremony under `claim`.
///
/// `claim` is the one Active Claim the caller took and published. The runner
/// makes the claim's terminal call itself, after restoring the IMU and clearing
/// the panel snapshot.
pub async fn run_dmp_compass_calibration(claim: &Claim) {
    info!("=== Starting DMP Compass Calibration ===");
    dmp_cal_state::clear().await;

    if !is_mag_available() {
        warn!("DMP compass calibration: magnetometer unavailable at start-up");
        claim.fail(MAG_UNAVAILABLE).await;
        return;
    }

    let imu_guard = ReadingsGuard::start();
    let mut calibration = CompassCalibration::new();
    let mut shown_step: Option<CompassStep> = None;

    // A tap that landed as the previous run ended is stale; discard it so it
    // cannot advance this run's first step.
    while DMP_ACTION.try_receive().is_ok() {}

    let outcome = 'run: loop {
        // Drain the operator's taps first, so a Done is reflected by this tick's
        // snapshot.
        while let Ok(action) = DMP_ACTION.try_receive() {
            match action {
                DmpAction::Done => {
                    let _ = calibration.done();
                }
                // Movement steps do not offer Redo, so this is inert until the
                // Compass Checks arrive in ticket 04. The plumbing stays.
                DmpAction::Redo => {
                    let _ = calibration.redo();
                }
            }
        }

        let Some(step) = calibration.step() else {
            break 'run MovementOutcome::Complete;
        };
        // Ticket 03 ends the ceremony after the sixth movement step. Ticket 04
        // replaces this branch with the two Compass Checks.
        if step == CompassStep::NorthCheck {
            break 'run MovementOutcome::Complete;
        }
        if shown_step != Some(step) {
            claim.phase(phase_label(step), None).await;
            shown_step = Some(step);
        }

        let readings = get_latest_readings().await;
        if let Some(orientation) = readings.orientation {
            calibration.observe(MotionSample {
                timestamp_ms: Instant::now().as_millis(),
                yaw_deg: orientation.yaw,
                pitch_deg: orientation.pitch,
                roll_deg: orientation.roll,
            });
        }
        dmp_cal_state::publish(build_snapshot(step, calibration.hint(), readings)).await;

        if claim.wait_or_stop(PROMPT_INTERVAL_MS).await {
            break 'run MovementOutcome::Stopped;
        }
    };

    // The guard's drop restores Axis6 and stops the readings before the claim's
    // terminal call, so the panel and the IMU agree about the end of the run.
    drop(imu_guard);
    dmp_cal_state::clear().await;

    match outcome {
        MovementOutcome::Complete => claim.complete(RESULT_NOT_SAVED).await,
        MovementOutcome::Stopped => claim.finish().await,
    }
}

/// Build the snapshot the Prompt formats for `step`.
fn build_snapshot(step: CompassStep, hint: MotionHint, readings: ImuReadings) -> DmpCalibrationSnapshot {
    DmpCalibrationSnapshot {
        step,
        heading_deg: readings.orientation.map(|orientation| compass_heading(orientation.yaw)),
        target_heading_deg: step.check_target_deg(),
        accuracy: readings
            .compass_accuracy
            .and_then(|accuracy| u8::try_from(accuracy).ok()),
        hint: DmpHint::from(hint),
        // Ticket 04 evaluates the Compass Checks; ticket 03 never enters one.
        check: DmpCheckState::NotChecking,
        redo: step.redo_allowed(),
    }
}

/// The Activity phase line for a Compass Step.
///
/// Activity State keeps a lifecycle record only; the live step, heading and hint
/// travel in the panel-only snapshot.
const fn phase_label(step: CompassStep) -> &'static str {
    match step {
        CompassStep::FlatHold => "Movement 1/6",
        CompassStep::FlatSpin => "Movement 2/6",
        CompassStep::NoseTilt => "Movement 3/6",
        CompassStep::SideTilt => "Movement 4/6",
        CompassStep::InvertedSpin => "Movement 5/6",
        CompassStep::FlatHoldBeforeCheck => "Movement 6/6",
        CompassStep::NorthCheck => "Check 1/2",
        CompassStep::EastCheck => "Check 2/2",
    }
}
