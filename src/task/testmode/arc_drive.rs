//! Arc drive test mode task.
//!
//! Validates curve arc driving by executing a 360° circle at 1 m radius.
//!
//! # Test Sequence
//!
//! 1. Load calibration from flash (if available)
//! 2. Count down ten seconds
//! 3. Drive a 360° left arc at radius 100 cm, speed 60, forward direction
//!
//! The running screen reads the phase and percent from the activity state, and
//! the operator's Stop cancels the arc in flight. Completion telemetry (achieved
//! left/right revs and status) is logged when the arc ends.

use defmt::{info, warn};
use embassy_executor::Spawner;
use touch_ui::Procedure;

use super::{status_label, submit, test_lifecycle};
use crate::{
    system::{
        event::{Events, raise_event},
        state::calibration,
    },
    task::{
        drive::{
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
            DriveQueueBuilder, DriveQueueCompletion, TurnDirection, types::DriveQueueBuildError,
        },
        procedure::Lifecycle,
        sensors::imu::{DmpFusionMode, set_dmp_fusion_mode},
    },
};

/// Radius of the test circle, in centimetres.
const ARC_RADIUS_CM: f32 = 100.0;

/// Speed the arc drives at (0-100).
const ARC_SPEED: u8 = 60;

/// Countdown before the arc, in milliseconds.
const COUNTDOWN_MS: u64 = 10_000;

/// The arc drive test's lifecycle: the test family's stop latch and slot, raising
/// `TestingCompleted` when the arc ran.
const LIFECYCLE: Lifecycle = test_lifecycle(Procedure::ArcDrive);

/// Spawn the arc drive test task via the controller.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(arc_drive_test_task().unwrap());
}

/// Arc drive test task: drives the circle, then reports completion.
#[embassy_executor::task]
async fn arc_drive_test_task() {
    if run_arc_drive_test().await {
        LIFECYCLE.complete("Arc drive complete").await;
    } else {
        LIFECYCLE.release();
    }
}

/// Run the arc drive test, reporting whether the arc ran.
async fn run_arc_drive_test() -> bool {
    LIFECYCLE.arm().await;

    if !calibration::is_initialized().await {
        raise_event(Events::Initialize).await;
    }

    // Force 6-axis fusion (gyro + accel).
    info!("arc: setting IMU DMP fusion mode to Axis6");
    set_dmp_fusion_mode(DmpFusionMode::Axis6);

    if LIFECYCLE.wait_or_stop(COUNTDOWN_MS).await {
        return false;
    }

    LIFECYCLE.phase("360 deg, r=100 cm", Some(0)).await;
    info!("arc: curve circle 360° at radius 1 m");

    match submit(build_arc_queue()).await {
        Ok(completion) => report_arc(&completion),
        Err(reason) => {
            LIFECYCLE.fail(reason).await;
            return false;
        }
    }
    true
}

/// Build the queue: one 360° arc, so its telemetry is the queue's last step.
fn build_arc_queue() -> Result<DriveQueueBuilder, DriveQueueBuildError> {
    let mut queue = DriveQueueBuilder::new();

    // 360° circle: arc length = 2π × radius.
    let circle_arc_cm = 2.0 * core::f32::consts::PI * ARC_RADIUS_CM;

    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::DriveDistance {
        kind: DriveDistanceKind::CurveArc {
            radius_cm: ARC_RADIUS_CM,
            arc_length_cm: circle_arc_cm,
            direction: TurnDirection::Left,
        },
        direction: DriveDirection::Forward,
        speed: ARC_SPEED,
    }))?;

    Ok(queue)
}

/// Log the arc's achieved telemetry, and its failure reason when it failed.
fn report_arc(completion: &DriveQueueCompletion) {
    let Some(step) = completion.last_step_completion.as_ref() else {
        return;
    };

    if let CompletionTelemetry::DriveDistance {
        achieved_left_revs,
        achieved_right_revs,
        ..
    } = step.telemetry
    {
        info!(
            "arc: status={=str} left={=f32} right={=f32}",
            status_label(&step.status),
            achieved_left_revs,
            achieved_right_revs
        );
    }
    if let CompletionStatus::Failed(reason) = step.status {
        warn!("arc: arc failed: {=str}", reason);
    }
}
