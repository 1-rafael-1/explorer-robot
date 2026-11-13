//! DMP Compass Calibration procedure: the guided movement ceremony, the two
//! Compass Checks, and the DMP Bias Snapshot save.
//!
//! The runner selects nine-axis fusion, starts IMU readings, and walks the
//! operator through the eight Compass Steps of the DMP Compass Calibration: six
//! movements followed by a check at magnetic north and one at magnetic east. It
//! publishes a typed [`touch_ui::DmpCalibrationSnapshot`] at the panel's 10 Hz
//! telemetry rate for the Prompt screen; Activity State carries only the
//! lifecycle phase. `Done` and `Redo` reach the runner through [`DMP_ACTION`],
//! sent by the UI task when the Prompt reports them.
//!
//! A `Done` on a Compass Check advances only when [`CompassCheck::accept`]
//! accepts a passing result, so a bad heading is never skipped or saved. `Redo`
//! restarts at the first movement step and clears only the DMP compass bias. The
//! save path reads the nine raw bias words through the IMU task, writes a
//! [`DmpBiasWords`] snapshot through flash storage, and completes the Procedure
//! only once flash acknowledges the write.
//!
//! Readings carry their acquisition time, and only fresh ones are evidence: a
//! driver that stops producing packets expires the Compass Check instead of
//! passing it on a favourite value it left behind.
//!
//! On every exit path the fusion mode returns to [`DEFAULT_FUSION_MODE`], IMU
//! readings stop, and the panel-only snapshot is cleared. A Stop or a terminal
//! failure restores the last saved DMP Bias Snapshot to the DMP when flash holds
//! one, and never writes flash.

use defmt::{info, warn};
use dmp_cal::{
    CompassCalibration, CompassCheck, CompassSample, CompassStep, MotionHint, MotionSample, compass_heading,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use touch_ui::{DmpCalibrationSnapshot, DmpCheckState, DmpHint, PromptAction};

use crate::{
    system::state::{CalibrationStatus, calibration, dmp_cal as dmp_cal_state},
    task::{
        io::flash_storage::{self, DmpBiasWords},
        procedure::Claim,
        sensors::imu::{
            DEFAULT_FUSION_MODE, DmpFusionMode, ImuReadings, READING_STALE_MS, apply_dmp_bias, clear_dmp_compass_bias,
            get_latest_readings, is_mag_available, read_dmp_bias, set_dmp_fusion_mode, start_imu_readings,
            stop_imu_readings,
        },
    },
};

/// The panel's existing telemetry rate: one snapshot per 100 ms.
const PROMPT_INTERVAL_MS: u64 = 100;

/// The result recorded once the snapshot is written to flash.
const RESULT_SAVED: &str = "Calibration saved";

/// The failure reason when the DMP bias words cannot be read, so nothing is
/// written.
const RESULT_READ_FAILED: &str = "DMP bias read failed";

/// The failure reason when flash does not take the snapshot, so the durable
/// calibration stays the previous one.
const RESULT_SAVE_FAILED: &str = "Calibration save failed";

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

/// How the run ended.
enum RunOutcome {
    /// Both Compass Checks were accepted.
    Complete,
    /// The operator stopped the Procedure.
    Stopped,
}

/// Run the DMP Compass Calibration under `claim`.
///
/// `claim` is the one Active Claim the caller took and published. The runner
/// makes the claim's terminal call itself, after restoring the IMU and clearing
/// the panel snapshot.
pub async fn run_dmp_compass_calibration(claim: &Claim) {
    info!("=== Starting DMP Compass Calibration ===");
    dmp_cal_state::clear().await;

    // Read once: the first Prompt tells the operator whether a saved calibration
    // exists before asking for any movement.
    let prior_status = calibration::get_dmp_cal_status().await;

    if !is_mag_available() {
        warn!("DMP compass calibration: magnetometer unavailable at start-up");
        restore_cached_dmp_bias().await;
        claim.fail(MAG_UNAVAILABLE).await;
        return;
    }

    let imu_guard = ReadingsGuard::start();
    let mut calibration = CompassCalibration::new();
    let mut check = CompassCheck::new();
    let mut shown_step: Option<CompassStep> = None;

    // A tap that landed as the previous run ended is stale; discard it so it
    // cannot advance this run's first step.
    while DMP_ACTION.try_receive().is_ok() {}

    let outcome = 'run: loop {
        // Drain the operator's taps first, so a Done or Redo is reflected by this
        // tick's snapshot.
        while let Ok(action) = DMP_ACTION.try_receive() {
            match action {
                DmpAction::Done => handle_done(&mut calibration, &mut check),
                DmpAction::Redo => {
                    // Only the two Compass Checks offer Redo; `redo` is inert on a
                    // movement step. It restarts at the first movement step,
                    // re-applies the saved gyroscope and accelerometer biases,
                    // and clears only the DMP compass bias — never flash.
                    if calibration.redo() {
                        check = CompassCheck::new();
                        if let Some(words) = flash_storage::get_cached_dmp_bias().await
                            && let Err(error) = apply_dmp_bias(words.into()).await
                        {
                            warn!("DMP calibration: Redo bias apply failed: {:?}", error);
                        }
                        clear_dmp_compass_bias();
                        shown_step = None;
                    }
                }
            }
        }

        let Some(step) = calibration.step() else {
            break 'run RunOutcome::Complete;
        };
        if shown_step != Some(step) {
            claim.phase(phase_label(step), None).await;
            shown_step = Some(step);
        }

        // Only readings the driver acquired recently are evidence: the shared
        // snapshot keeps its last value when packets stop, so a stalled IMU
        // must not look like a held heading or a converged compass.
        let readings = get_latest_readings().await.fresh(READING_STALE_MS);
        if let Some(reading) = readings.orientation {
            let orientation = reading.value;
            calibration.observe(MotionSample {
                // The acquisition time, not the polling time: a value left
                // behind by the driver must not extend the stability window.
                timestamp_ms: reading.at_ms,
                yaw_deg: orientation.yaw,
                pitch_deg: orientation.pitch,
                roll_deg: orientation.roll,
            });
        }

        let accuracy = readings
            .compass_accuracy
            .and_then(|reading| u8::try_from(reading.value).ok());
        let check_state = if is_check_step(Some(step)) {
            if let (Some(orientation), Some(accuracy)) = (readings.orientation, accuracy) {
                check.sample(CompassSample {
                    timestamp_ms: orientation.at_ms,
                    heading_deg: compass_heading(orientation.value.yaw),
                    accuracy,
                });
            } else {
                // No fresh compass evidence: a heading held before the gap
                // cannot carry the check.
                check.expire();
            }
            DmpCheckState::from(check.outcome())
        } else {
            DmpCheckState::NotChecking
        };

        dmp_cal_state::publish(build_snapshot(
            step,
            calibration.hint(),
            readings,
            check_state,
            prior_status,
        ))
        .await;

        if claim.wait_or_stop(PROMPT_INTERVAL_MS).await {
            break 'run RunOutcome::Stopped;
        }
    };

    // The guard's drop restores Axis6 and stops the readings before the claim's
    // terminal call, so the panel and the IMU agree about the end of the run.
    drop(imu_guard);
    dmp_cal_state::clear().await;

    match outcome {
        RunOutcome::Complete => save_bias_snapshot(claim).await,
        RunOutcome::Stopped => {
            restore_cached_dmp_bias().await;
            claim.finish().await;
        }
    }
}

/// Apply a `Done` tap to the step model and, on a Compass Check, to the check.
///
/// A movement step advances unconditionally because its hint never gates `Done`.
/// A Compass Check advances only when [`CompassCheck::accept`] accepts a passing
/// result: a bad heading leaves the Procedure on the Prompt with the reason and
/// `Redo`.
const fn handle_done(calibration: &mut CompassCalibration, check: &mut CompassCheck) {
    if is_check_step(calibration.step()) {
        if check.accept() {
            let _ = calibration.done();
        }
    } else {
        let _ = calibration.done();
    }
}

/// Whether `step` is one of the two Compass Checks.
const fn is_check_step(step: Option<CompassStep>) -> bool {
    matches!(step, Some(CompassStep::NorthCheck | CompassStep::EastCheck))
}

/// Read the nine bias words, save the snapshot, then finish the claim.
///
/// A Stop is honoured before the flash write and again after the read, because
/// the nine memory reads take long enough for the operator to stop the
/// ceremony; a stopped run restores the last saved snapshot and writes nothing.
/// A failed read fails the Procedure and writes nothing. A flash write that is
/// not acknowledged restores the last saved snapshot and reports the failure,
/// so the operator never sees "Calibration saved" for a snapshot that will
/// vanish on reboot.
async fn save_bias_snapshot(claim: &Claim) {
    if stopped(claim).await {
        return;
    }

    match read_dmp_bias().await {
        Ok(snapshot) => {
            if stopped(claim).await {
                return;
            }

            info!("DMP compass calibration: saving bias snapshot");
            if flash_storage::save_dmp_bias(DmpBiasWords::from(snapshot)).await {
                // The DMP already holds the biases that were just read, and
                // flash now holds them too, so the status is truthful.
                calibration::set_dmp_cal_status(CalibrationStatus::Loaded).await;
                claim.complete(RESULT_SAVED).await;
            } else {
                warn!("DMP compass calibration: flash write was not acknowledged");
                restore_cached_dmp_bias().await;
                claim.fail(RESULT_SAVE_FAILED).await;
            }
        }
        Err(error) => {
            warn!("DMP compass calibration: DMP bias read failed: {:?}", error);
            restore_cached_dmp_bias().await;
            claim.fail(RESULT_READ_FAILED).await;
        }
    }
}

/// Restore the saved snapshot and finish the claim when the operator stopped.
///
/// Returns `true` when it did, so the caller returns without writing flash.
/// `complete` and `fail` ignore the stop latch, so the check has to happen
/// here, before either terminal call.
async fn stopped(claim: &Claim) -> bool {
    if !claim.is_stop_requested() {
        return false;
    }
    restore_cached_dmp_bias().await;
    claim.finish().await;
    true
}

/// Restore the last saved DMP Bias Snapshot to the DMP after a Stop or failure.
///
/// Never writes flash. When flash holds a snapshot, its nine words are applied
/// and the status becomes `Loaded` only when the DMP write succeeds. With no
/// snapshot the DMP's current RAM state is left alone, and the status records
/// that no persisted calibration exists.
async fn restore_cached_dmp_bias() {
    if let Some(words) = flash_storage::get_cached_dmp_bias().await {
        info!("DMP calibration: restoring the saved bias snapshot");
        match apply_dmp_bias(words.into()).await {
            Ok(()) => {
                info!("DMP calibration: saved bias snapshot restored");
                calibration::set_dmp_cal_status(CalibrationStatus::Loaded).await;
            }
            Err(error) => {
                warn!("DMP calibration: restoring the saved bias snapshot failed: {:?}", error);
                calibration::set_dmp_cal_status(CalibrationStatus::NotAvailable).await;
            }
        }
    } else {
        info!("DMP calibration: no saved snapshot — leaving DMP RAM state alone");
        calibration::set_dmp_cal_status(CalibrationStatus::NotAvailable).await;
    }
}

/// Build the snapshot the Prompt formats for `step`.
fn build_snapshot(
    step: CompassStep,
    hint: MotionHint,
    readings: ImuReadings,
    check: DmpCheckState,
    prior_status: CalibrationStatus,
) -> DmpCalibrationSnapshot {
    DmpCalibrationSnapshot {
        step,
        heading_deg: readings.orientation.map(|reading| compass_heading(reading.value.yaw)),
        target_heading_deg: step.check_target_deg(),
        accuracy: readings
            .compass_accuracy
            .and_then(|reading| u8::try_from(reading.value).ok()),
        hint: DmpHint::from(hint),
        check,
        redo: step.redo_allowed(),
        prior_status: calibration::ui_status(prior_status),
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
