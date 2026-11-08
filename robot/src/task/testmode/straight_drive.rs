//! Straight drive test mode task.
//!
//! Validates encoder-based straight-line distance driving.
//!
//! # Test Sequence
//!
//! 1. Load calibration from flash (if available)
//! 2. Wait five seconds
//! 3. Drive forward 150 cm at speed 70, brake, settle
//! 4. Drive backward 150 cm at speed 70, brake, settle, coast
//!
//! Each direction is its own queue, so the running screen's phase and percent
//! follow the test, and the operator's Stop cancels the drive in flight or lands
//! between the legs. Completion telemetry (achieved left/right revs and status)
//! is logged per leg.

use defmt::{error, info, warn};
use embassy_executor::Spawner;

use super::{status_label, submit};
use crate::{
    system::{
        event::{Events, raise_event},
        state::{activity, calibration},
    },
    task::{
        drive::{
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
            DriveQueueBuilder, DriveQueueCompletion, send_drive_command, types::DriveQueueBuildError,
        },
        procedure::Claim,
        sensors::imu::{DmpFusionMode, set_dmp_fusion_mode},
    },
};

/// Distance each leg drives, in centimetres.
const LEG_DISTANCE_CM: f32 = 150.0;

/// Speed each leg drives at (0-100).
const LEG_SPEED: u8 = 70;

/// Countdown before the first leg, in milliseconds.
const COUNTDOWN_MS: u64 = 5_000;

/// Settle time after a brake, in milliseconds.
const SETTLE_MS: u64 = 500;

/// Spawn the straight drive test task via the controller.
///
/// The caller's claim travels with the task. Returns `false` when the spawn failed,
/// so the controller can abandon the claim rather than leave its running screen up
/// with no task behind it.
#[must_use]
pub(super) fn spawn(spawner: Spawner, claim: Claim) -> bool {
    straight_drive_test_task(claim).map_or_else(
        |_| {
            error!("straight: spawn failed");
            false
        },
        |token| {
            spawner.spawn(token);
            true
        },
    )
}

/// Straight drive test task: runs both legs, then reports the claim's terminal
/// outcome.
#[embassy_executor::task]
async fn straight_drive_test_task(claim: Claim) {
    if run_straight_drive_test(&claim).await {
        claim.complete("Straight drive complete").await;
    } else {
        // A stop: the owner records the latched outcome now that
        // `procedure::stop` does not. A failure already released the claim, so
        // this is a no-op there.
        claim.finish().await;
    }
}

/// Run the straight-line distance test, reporting whether both legs ran.
async fn run_straight_drive_test(claim: &Claim) -> bool {
    if !calibration::is_initialized().await {
        raise_event(Events::Initialize).await;
    }

    // Force 6-axis fusion (gyro + accel).
    info!("straight: setting IMU DMP fusion mode to Axis6");
    set_dmp_fusion_mode(DmpFusionMode::Axis6);

    if claim.wait_or_stop(COUNTDOWN_MS).await {
        return false;
    }

    let legs = [
        ("Forward 150 cm", DriveDirection::Forward),
        ("Backward 150 cm", DriveDirection::Backward),
    ];

    for (index, (phase, direction)) in legs.iter().enumerate() {
        if claim.is_stop_requested() {
            return false;
        }
        claim
            .phase(phase, Some(activity::percent_done(index, legs.len())))
            .await;

        match submit(build_leg(*direction)).await {
            Ok(completion) => {
                report_leg(&completion);
                // A cancelled leg is a stop, not a completed leg.
                if completion.status == CompletionStatus::Cancelled || claim.is_stop_requested() {
                    return false;
                }
            }
            Err(reason) => {
                claim.fail(reason).await;
                return false;
            }
        }

        // Brake, then let the robot settle before the operator's next leg or the
        // measurement. The settle is a queue step so a Stop can cancel it.
        match submit(build_settle()).await {
            Ok(completion) => {
                if completion.status == CompletionStatus::Cancelled || claim.is_stop_requested() {
                    return false;
                }
            }
            Err(reason) => {
                claim.fail(reason).await;
                return false;
            }
        }
    }

    send_drive_command(DriveCommand::Drive(DriveAction::Coast)).await;
    !claim.is_stop_requested()
}

/// Build one leg's queue: the straight drive alone, so its telemetry is the
/// queue's last step.
fn build_leg(direction: DriveDirection) -> Result<DriveQueueBuilder, DriveQueueBuildError> {
    let mut queue = DriveQueueBuilder::new();
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::DriveDistance {
        kind: DriveDistanceKind::Straight {
            distance_cm: LEG_DISTANCE_CM,
        },
        direction,
        speed: LEG_SPEED,
    }))?;
    Ok(queue)
}

/// Build the settle queue: brake, then idle for the settle time.
fn build_settle() -> Result<DriveQueueBuilder, DriveQueueBuildError> {
    let mut queue = DriveQueueBuilder::new();
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Brake))?;
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Idle { duration_ms: SETTLE_MS }))?;
    Ok(queue)
}

/// Log one leg's achieved distance telemetry.
fn report_leg(completion: &DriveQueueCompletion) {
    let Some(step) = completion.last_step_completion.as_ref() else {
        return;
    };

    if let CompletionTelemetry::DriveDistance {
        achieved_left_revs,
        achieved_right_revs,
        target_left_revs,
        target_right_revs,
        duration_ms,
    } = step.telemetry
    {
        info!(
            "straight: status={=str} target L={=f32} R={=f32} achieved L={=f32} R={=f32} duration_ms={=u64}",
            status_label(&step.status),
            target_left_revs,
            target_right_revs,
            achieved_left_revs,
            achieved_right_revs,
            duration_ms
        );
    }
    if let CompletionStatus::Failed(reason) = step.status {
        warn!("straight: leg failed: {=str}", reason);
    }
}
