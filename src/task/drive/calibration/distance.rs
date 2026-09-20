//! Distance calibration: the fixed-distance drive step and its factor.
//!
//! # Flow
//!
//! 1. The panel's Distance entry backs up the factor in force and forces 1.0, so
//!    the commanded distance is nominal.
//! 2. [`run_drive_step`] counts down, drives the fixed distance, and brakes.
//! 3. The operator measures how far the robot actually travelled and enters it on
//!    the panel's value screen; [`commit`] turns it into the new factor, or
//!    [`abort`] restores the backed-up one.
//!
//! Zero is a legitimate measured distance: it clamps to [`MAX_FACTOR`], the
//! strongest correction the screen can express, and cancelling is the screen's
//! explicit Cancel action rather than a magic value.
//!
//! The running screen reads the drive step's phase from the activity state; the
//! step's failure reason goes there too.

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::info;
use touch_ui::Procedure;

use super::calibration_lifecycle;
use crate::{
    system::state::{CalibrationStatus, activity, calibration},
    task::{
        drive::{
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
            DriveQueueBuilder, DriveQueueCompletion, send_drive_command, types::DriveQueueBuildError,
        },
        procedure::Lifecycle,
    },
};

/// Distance the calibration drives, in centimetres.
pub const DRIVE_DISTANCE_CM: f32 = 150.0;

/// Speed the calibration drives at (0-100).
const DRIVE_SPEED: u8 = 70;

/// Countdown before the drive, in seconds.
const COUNTDOWN_SECONDS: u64 = 3;

/// Settle time after the brake, in milliseconds.
const SETTLE_MS: u64 = 500;

/// Minimum valid distance calibration factor.
const MIN_FACTOR: f32 = 0.1;

/// Maximum valid distance calibration factor.
const MAX_FACTOR: f32 = 10.0;

/// `PREVIOUS_FACTOR`'s value when no calibration is in flight.
///
/// `f32::from_bits` of it is a NaN, which no computed factor can be, so the
/// sentinel is unambiguous without a second flag or a lock.
const NO_BACKUP: u32 = u32::MAX;

/// The distance calibration's lifecycle: the calibration family's stop latch, no
/// slot, and no completion event — its flow crosses the value-entry screen, so the
/// Panel, not the procedure, reports the finished calibration.
const LIFECYCLE: Lifecycle = calibration_lifecycle(Procedure::DistanceCalibration);

/// The distance factor that was in force before the calibration started.
static PREVIOUS_FACTOR: AtomicU32 = AtomicU32::new(NO_BACKUP);

/// What the distance calibration's drive step did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriveOutcome {
    /// The drive finished; the operator now measures and enters the distance.
    Complete,
    /// The operator stopped the drive; the previous factor was restored.
    Stopped,
    /// The drive could not run; the reason is on the running screen.
    Failed,
}

/// Read the current factor into the backup slot and force 1.0 for the drive.
pub async fn begin() {
    let previous = calibration::get_distance_factor().await;
    PREVIOUS_FACTOR.store(previous.to_bits(), Ordering::Release);

    if (previous - 1.0).abs() > f32::EPSILON {
        info!("Distance calibration: forcing factor 1.0, was {=f32}", previous);
        let mut state = calibration::CALIBRATION_STATE.lock().await;
        state.distance_factor = 1.0;
        state.distance_cal_status = CalibrationStatus::Loaded;
    }
}

/// Commit the factor computed from the measured distance.
///
/// Returns the stored factor. A measured distance of zero is legitimate: the
/// ratio saturates and clamps to [`MAX_FACTOR`].
#[allow(clippy::cast_precision_loss)]
pub async fn commit(measured_cm: i32) -> f32 {
    let factor = (DRIVE_DISTANCE_CM / measured_cm as f32).clamp(MIN_FACTOR, MAX_FACTOR);

    {
        let mut state = calibration::CALIBRATION_STATE.lock().await;
        state.distance_factor = factor;
        state.distance_cal_status = CalibrationStatus::Loaded;
    }

    PREVIOUS_FACTOR.store(NO_BACKUP, Ordering::Release);
    info!(
        "Distance calibration: factor {=f32} saved from {=i32} cm",
        factor, measured_cm
    );
    factor
}

/// Restore the factor that was in force before the run.
///
/// Used when the operator cancels the value screen or stops the drive, so a
/// calibration that produced nothing does not leave the robot with a factor of
/// 1.0.
pub async fn abort() {
    let Some(previous) = take_previous_factor() else {
        return;
    };

    {
        let mut state = calibration::CALIBRATION_STATE.lock().await;
        state.distance_factor = previous;
        state.distance_cal_status = CalibrationStatus::Loaded;
    }
    info!("Distance calibration: restored factor {=f32}", previous);
}

/// Take the backed-up factor, clearing the slot.
fn take_previous_factor() -> Option<f32> {
    let bits = PREVIOUS_FACTOR.swap(NO_BACKUP, Ordering::AcqRel);
    (bits != NO_BACKUP).then(|| f32::from_bits(bits))
}

/// Run the calibration's fixed-distance drive step.
///
/// Counts down, drives [`DRIVE_DISTANCE_CM`] forward, and brakes, publishing each
/// phase to the activity state. The step is interruptible from the panel: the
/// Stop latches the calibration stop and interrupts the drive, and a stopped step
/// restores the previous factor and clears the activity.
pub async fn run_drive_step() -> DriveOutcome {
    LIFECYCLE.arm().await;
    let _ = LIFECYCLE.start("Driving 150 cm").await;

    for second in 0..COUNTDOWN_SECONDS {
        let elapsed = usize::try_from(second).unwrap_or(0);
        let total = usize::try_from(COUNTDOWN_SECONDS).unwrap_or(1);
        LIFECYCLE
            .phase("Get ready", Some(activity::percent_done(elapsed, total)))
            .await;
        if LIFECYCLE.wait_or_stop(1_000).await {
            return stopped().await;
        }
    }

    LIFECYCLE.phase("Driving 150 cm", None).await;

    let queue = match build_drive_queue() {
        Ok(queue) => queue,
        Err(error) => {
            LIFECYCLE.fail(error.label()).await;
            return DriveOutcome::Failed;
        }
    };

    let completion = match queue.submit().await {
        Ok(completion) => completion,
        Err(error) => {
            LIFECYCLE.fail(error.label()).await;
            return DriveOutcome::Failed;
        }
    };

    match completion.status {
        CompletionStatus::Success => {}
        CompletionStatus::Cancelled => return stopped().await,
        CompletionStatus::Failed(reason) => {
            // A leg that did not reach its target cannot calibrate anything, so
            // the reason is what the operator sees instead of the value screen.
            log_completion(&completion);
            LIFECYCLE.fail(reason).await;
            return DriveOutcome::Failed;
        }
    }

    // Brake so the robot is still while the operator measures.
    send_drive_command(DriveCommand::Drive(DriveAction::Brake)).await;
    if LIFECYCLE.wait_or_stop(SETTLE_MS).await {
        return stopped().await;
    }

    log_completion(&completion);
    LIFECYCLE.phase("Enter measured distance", None).await;
    DriveOutcome::Complete
}

/// Restore the previous factor, clear the activity, and report the stop.
async fn stopped() -> DriveOutcome {
    info!("Distance calibration: drive step stopped");
    abort().await;
    LIFECYCLE.abandon().await;
    DriveOutcome::Stopped
}

/// Build the drive queue: the fixed distance alone, so its telemetry is the
/// queue's last step.
fn build_drive_queue() -> Result<DriveQueueBuilder, DriveQueueBuildError> {
    let mut queue = DriveQueueBuilder::new();
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::DriveDistance {
        kind: DriveDistanceKind::Straight {
            distance_cm: DRIVE_DISTANCE_CM,
        },
        direction: DriveDirection::Forward,
        speed: DRIVE_SPEED,
    }))?;
    Ok(queue)
}

/// Log the drive's achieved distance, for the record next to the operator's
/// measurement.
fn log_completion(completion: &DriveQueueCompletion) {
    let Some(step) = completion.last_step_completion.as_ref() else {
        return;
    };
    if let CompletionTelemetry::DriveDistance {
        achieved_left_revs,
        achieved_right_revs,
        duration_ms,
        ..
    } = step.telemetry
    {
        info!(
            "Distance calibration: drove L={=f32} R={=f32} revs in {=u64} ms",
            achieved_left_revs, achieved_right_revs, duration_ms
        );
    }
    if let CompletionStatus::Failed(reason) = step.status {
        info!("Distance calibration: drive failed: {=str}", reason);
    }
}
