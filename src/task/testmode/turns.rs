//! Turns test mode task.
//!
//! Validates in-place rotation accuracy at a range of speeds.
//!
//! # Test Sequence
//!
//! 1. Load calibration from flash (if available)
//! 2. Count down five seconds
//! 3. Perform one in-place 90° turn at each speed: 40, 60, 80, 100
//!
//! Each speed is its own queue, so the running screen's phase and percent follow
//! the test, and the operator's Stop cancels the turn in flight or lands between
//! stages. Per-turn telemetry (target, achieved, yaw, deviation) is logged; a
//! stopped test leaves without raising its completion event.

use defmt::{error, info, warn};
use embassy_executor::Spawner;

use super::submit;
use crate::{
    system::{
        event::{Events, raise_event},
        state::{activity, calibration},
    },
    task::{
        drive::{
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveQueueBuilder, DriveQueueCompletion,
            send_drive_command,
            types::{DriveQueueBuildError, RotationDirection, RotationMotion},
        },
        procedure::Claim,
        sensors::imu::{DmpFusionMode, set_dmp_fusion_mode},
    },
};

/// The angle every stage turns, in degrees.
const TARGET_DEG: f32 = 90.0;

/// Countdown before the first turn, in milliseconds.
const COUNTDOWN_MS: u64 = 5_000;

/// Settle time after the last turn, in milliseconds.
const SETTLE_MS: u64 = 1_000;

/// One stage of the test: a rotation speed and the phase line that names it.
struct TurnStage {
    /// Rotation speed (0-100).
    speed: u8,
    /// Phase line shown while this stage runs.
    phase: &'static str,
}

/// The stages, in order.
const STAGES: [TurnStage; 4] = [
    TurnStage {
        speed: 40,
        phase: "90 deg at 40",
    },
    TurnStage {
        speed: 60,
        phase: "90 deg at 60",
    },
    TurnStage {
        speed: 80,
        phase: "90 deg at 80",
    },
    TurnStage {
        speed: 100,
        phase: "90 deg at 100",
    },
];

/// Spawn the turns test task via the controller.
///
/// The caller's claim travels with the task. Returns `false` when the spawn failed,
/// so the controller can abandon the claim rather than leave its running screen up
/// with no task behind it.
#[must_use]
pub(super) fn spawn(spawner: Spawner, claim: Claim) -> bool {
    turns_test_task(claim).map_or_else(
        |_| {
            error!("turns: spawn failed");
            false
        },
        |token| {
            spawner.spawn(token);
            true
        },
    )
}

/// Turns test task: runs the stages, then reports the claim's terminal outcome.
#[embassy_executor::task]
async fn turns_test_task(claim: Claim) {
    if run_turns_test(&claim).await {
        claim.complete("Turns complete").await;
    } else {
        // A stop: the owner clears the Activity now that `procedure::stop` does
        // not. A failure already released the claim, so this is a no-op there.
        claim.abandon().await;
    }
}

/// Run the in-place turns test, reporting whether every stage ran.
async fn run_turns_test(claim: &Claim) -> bool {
    if !calibration::is_initialized().await {
        raise_event(Events::Initialize).await;
    }

    // Force 6-axis fusion (gyro + accel) to avoid magnetometer yaw issues.
    info!("turns: setting IMU DMP fusion mode to Axis6");
    set_dmp_fusion_mode(DmpFusionMode::Axis6);

    if claim.wait_or_stop(COUNTDOWN_MS).await {
        return false;
    }

    for (index, stage) in STAGES.iter().enumerate() {
        if claim.is_stop_requested() {
            return false;
        }
        claim
            .phase(stage.phase, Some(activity::percent_done(index, STAGES.len())))
            .await;

        match submit(build_turn_queue(stage.speed)).await {
            Ok(completion) => {
                report_turn(&completion);
                // A cancelled queue is a stop, not a completed stage.
                if completion.status == CompletionStatus::Cancelled || claim.is_stop_requested() {
                    return false;
                }
            }
            Err(reason) => {
                claim.fail(reason).await;
                return false;
            }
        }

        // Coast after each turn before the next speed, as the previous
        // single-queue version did.
        send_drive_command(DriveCommand::Drive(DriveAction::Coast)).await;
    }

    // Leave the robot settled with the last telemetry in the log. A stop during
    // the settle is still a stop, not a completed test.
    !claim.wait_or_stop(SETTLE_MS).await
}

/// Build one stage's queue: the turn alone, so its telemetry is the queue's last
/// step.
fn build_turn_queue(speed: u8) -> Result<DriveQueueBuilder, DriveQueueBuildError> {
    let mut queue = DriveQueueBuilder::new();
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::RotateExact {
        degrees: TARGET_DEG,
        direction: RotationDirection::Clockwise,
        motion: RotationMotion::Stationary { speed },
    }))?;
    Ok(queue)
}

/// Log one turn's telemetry, and its failure reason when it failed.
fn report_turn(completion: &DriveQueueCompletion) {
    let Some(step) = completion.last_step_completion.as_ref() else {
        return;
    };

    if let CompletionTelemetry::RotateExact {
        final_yaw_deg,
        angle_error_deg,
        duration_ms,
    } = step.telemetry
    {
        // error = achieved - target, so achieved = target + error.
        let achieved_deg = TARGET_DEG + angle_error_deg;
        info!(
            "turns: tgt={=f32} achieved={=f32} yaw={=f32} dev={=f32} duration_ms={=u64}",
            TARGET_DEG, achieved_deg, final_yaw_deg, angle_error_deg, duration_ms
        );
    } else {
        warn!("turns: unexpected completion telemetry");
    }

    if let CompletionStatus::Failed(reason) = step.status {
        warn!("turns: turn failed: {=str}", reason);
    }
}
