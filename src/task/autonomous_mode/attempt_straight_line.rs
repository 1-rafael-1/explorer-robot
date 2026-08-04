//! Attempt Straight Line autonomous drive mode: analyze `LiDAR` point cloud for gaps,
//! drive through the best gap, repeat until target distance reached or blocked.
//!
//! # Control flow
//!
//! ```text
//! start(target_cm) ──► ANALYZE ──► ORIENT ──► DRIVE
//!                          ▲              │        │
//!                          │              │        ├── leg complete ──► ANALYZE
//!                          │              │        ├── obstacle interrupt ──► ANALYZE
//!                          │              │        └── target reached ──► FINISHED_REACHED
//!                          │              │
//!                          │              └── no gap ──► FINISHED_BLOCKED
//!                          │
//!                          └── stop() ──► exit (brake, release, ShowMainMenu)
//! ```
//!
//! # v3 Changes from v2
//! - `LiDAR` point cloud gap analysis replaces ultrasonic sweep buffer.
//! - `perception::get_lidar_snapshot()` replaces `ultrasonic::SWEEP_BUFFER`.
//! - `gap_analysis::analyze_gaps(cloud, remaining)` uses `&LidarPointCloud`.
//! - All ultrasonic imports and usage removed.
//! - IMU drift correction and odometry unchanged.

#![allow(dead_code)]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};

use crate::{
    system::state::{calibration, perception},
    task::{
        autonomous_mode::{self, gap_analysis},
        drive::{
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
            DriveQueueBuilder, DriveQueueSubmitError, send_drive_command,
            types::{RotationDirection, RotationMotion, SPROCKET_CIRCUMFERENCE_CM},
        },
        ui::{UiEvent, send_ui_event},
    },
};

// ── Active flag ───────────────────────────────────────────────────────────────

/// Set while the attempt-straight-line loop is running.
static ACTIVE: AtomicBool = AtomicBool::new(false);

// ── Display state ─────────────────────────────────────────────────────────────

/// Shared state visible to the UI for display updates.
#[derive(Clone, Copy)]
pub struct ModeDisplayState {
    /// Total forward progress in cm.
    pub progress_cm: f32,
    /// Target distance in cm.
    pub target_cm: u16,
    /// Estimated current heading in world-frame degrees.
    pub heading_deg: f32,
    /// Absolute lateral offset from the original straight line (cm).
    pub offset_cm: f32,
    /// Current state label for display.
    pub state_label: &'static str,
}

/// Public display state for the UI to read live progress/drift values.
static DISPLAY_STATE: Mutex<CriticalSectionRawMutex, ModeDisplayState> = Mutex::new(ModeDisplayState {
    progress_cm: 0.0,
    target_cm: 0,
    heading_deg: 0.0,
    offset_cm: 0.0,
    state_label: "Idle",
});

/// Return a snapshot of the current display state.
pub async fn display_state_snapshot() -> ModeDisplayState {
    *DISPLAY_STATE.lock().await
}

// ── Mode entry constants ──────────────────────────────────────────────────────

/// Minimum target distance (cm).
const TARGET_MIN_CM: u16 = 100;
/// Maximum target distance (cm).
const TARGET_MAX_CM: u16 = 1000;
/// Target distance step size (cm).
const TARGET_STEP_CM: u16 = 10;
/// Preset target distance (cm).
const TARGET_PRESET_CM: u16 = 300;

// ── Tuning constants ──────────────────────────────────────────────────────────

/// Speed for forward driving through gaps (0–100).
const DRIVE_SPEED: u8 = 70;

/// Speed for in-place rotation to face gaps (0–100).
const TURN_SPEED: u8 = 60;

/// Threshold for obstacle detection during drive (cm).
const OBSTACLE_THRESHOLD_CM: f32 = 30.0;

/// Forward cone for obstacle check during drive (degrees).
const OBSTACLE_CONE_DEG: u16 = 60;

/// Polling interval for obstacle checks during drive (ms).
const OBSTACLE_CHECK_INTERVAL_MS: u64 = 100;

// ── State machine ─────────────────────────────────────────────────────────────

/// Mode state machine.
enum State {
    /// Analyzing the `LiDAR` point cloud for gaps.
    Analyzing,
    /// Executing a drive leg through the chosen gap.
    Driving(gap_analysis::GapDecision),
    /// Target distance reached.
    FinishedReached,
    /// No forward path available.
    FinishedBlocked,
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Spawn the attempt-straight-line autonomous task.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner, target_distance_cm: u16) {
    spawner.spawn(attempt_straight_line_task(target_distance_cm).unwrap());
}

/// Request a graceful stop of the mode.
pub fn stop() {
    ACTIVE.store(false, Ordering::Relaxed);
}

/// Activate the attempt-straight-line autonomous mode.
pub async fn start(target_distance_cm: u16) -> bool {
    if !autonomous_mode::request_start(autonomous_mode::AutonomousCommand::AttemptStraightLine { target_distance_cm })
        .await
    {
        return false;
    }

    ACTIVE.store(true, Ordering::Relaxed);
    true
}

/// Minimum target distance (cm).
pub const fn target_min_cm() -> u16 {
    TARGET_MIN_CM
}

/// Maximum target distance (cm).
pub const fn target_max_cm() -> u16 {
    TARGET_MAX_CM
}

/// Target distance step size (cm).
pub const fn target_step_cm() -> u16 {
    TARGET_STEP_CM
}

/// Preset target distance (cm).
pub const fn target_preset_cm() -> u16 {
    TARGET_PRESET_CM
}

// ── Task ──────────────────────────────────────────────────────────────────────

#[embassy_executor::task]
#[allow(clippy::too_many_lines, clippy::similar_names)]
pub async fn attempt_straight_line_task(target_distance_cm: u16) {
    info!("attempt-straight: activated, target {} cm", target_distance_cm);

    // Ensure a clean starting state.
    send_drive_command(DriveCommand::Drive(DriveAction::Brake)).await;
    Timer::after(Duration::from_millis(200)).await;

    // Initialize display state.
    {
        let mut ds = DISPLAY_STATE.lock().await;
        ds.progress_cm = 0.0;
        ds.target_cm = target_distance_cm;
        ds.heading_deg = 0.0;
        ds.offset_cm = 0.0;
        ds.state_label = "Starting...";
    }

    // Read distance calibration factor once — it doesn't change during operation.
    let distance_factor = calibration::get_distance_factor().await;

    let mut robot_x_cm: f32 = 0.0;
    let mut robot_y_cm: f32 = 0.0;
    let mut robot_heading_deg: f32 = 0.0;
    let mut total_progress_cm: f32 = 0.0;
    let mut state = State::Analyzing;

    let exit_label = loop {
        if !ACTIVE.load(Ordering::Relaxed) {
            break "Aborted";
        }

        state = match state {
            State::Analyzing => {
                {
                    let mut ds = DISPLAY_STATE.lock().await;
                    ds.state_label = "Analyzing...";
                }
                info!("attempt-straight: analyzing LiDAR point cloud");

                let remaining = f32::from(target_distance_cm) - total_progress_cm;
                let cloud = perception::get_lidar_snapshot().await;

                cloud.map_or_else(
                    || {
                        info!("attempt-straight: no LiDAR data available");
                        State::FinishedBlocked
                    },
                    |cloud| {
                        let decision = gap_analysis::analyze_gaps(&cloud, remaining);
                        decision.map_or_else(
                            || {
                                info!("attempt-straight: blocked — no path");
                                State::FinishedBlocked
                            },
                            |gap| {
                                info!(
                                    "attempt-straight: gap chosen, center={}, rot={}, drive={} cm",
                                    gap.gap_center_deg, gap.rotation_degrees, gap.drive_distance_cm
                                );
                                State::Driving(gap)
                            },
                        )
                    },
                )
            }
            State::Driving(gap) => {
                {
                    let mut ds = DISPLAY_STATE.lock().await;
                    ds.state_label = "Driving...";
                }
                info!("attempt-straight: driving leg");

                // Build drive queue: rotate to gap center, then drive straight.
                let mut queue = DriveQueueBuilder::new();

                // If rotation is needed (> 1 degree threshold).
                let rotation_dir = if gap.clockwise {
                    RotationDirection::Clockwise
                } else {
                    RotationDirection::CounterClockwise
                };

                if gap.rotation_degrees > 1.0
                    && queue
                        .push_abort_on_fail(DriveCommand::Drive(DriveAction::RotateExact {
                            degrees: gap.rotation_degrees,
                            direction: rotation_dir,
                            motion: RotationMotion::Stationary { speed: TURN_SPEED },
                        }))
                        .is_err()
                {
                    info!("attempt-straight: queue full (rotate)");
                    break "Error: queue full";
                }

                if queue
                    .push_abort_on_fail(DriveCommand::Drive(DriveAction::DriveDistance {
                        kind: DriveDistanceKind::Straight {
                            distance_cm: gap.drive_distance_cm,
                        },
                        direction: DriveDirection::Forward,
                        speed: DRIVE_SPEED,
                    }))
                    .is_err()
                {
                    info!("attempt-straight: queue full (drive)");
                    break "Error: queue full";
                }

                let completion = match queue.submit().await {
                    Ok(completion) => completion,
                    Err(DriveQueueSubmitError::QueueBusy) => {
                        info!("attempt-straight: queue busy");
                        break "Error: queue busy";
                    }
                };

                // Extract partial progress from telemetry regardless of status.
                let progress = extract_progress_cm(&completion, distance_factor);
                total_progress_cm += progress;

                // Update odometry: rotation first, then forward drive along new heading.
                (robot_x_cm, robot_y_cm, robot_heading_deg) = update_odometry(
                    robot_x_cm,
                    robot_y_cm,
                    robot_heading_deg,
                    gap.rotation_degrees,
                    rotation_dir,
                    progress,
                );

                // Update display state.
                {
                    let mut ds = DISPLAY_STATE.lock().await;
                    ds.progress_cm = total_progress_cm;
                    ds.heading_deg = robot_heading_deg;
                    ds.offset_cm = libm::fabsf(robot_y_cm);
                }

                let leg_status = if matches!(completion.status, CompletionStatus::Success) {
                    "done"
                } else {
                    "interrupted"
                };

                info!(
                    "attempt-straight: leg {} (progress {} cm, total {} cm, heading {} deg, offset {} cm)",
                    leg_status,
                    progress,
                    total_progress_cm,
                    robot_heading_deg,
                    libm::fabsf(robot_y_cm)
                );

                if !ACTIVE.load(Ordering::Relaxed) {
                    break "Aborted";
                }

                if total_progress_cm >= f32::from(target_distance_cm) {
                    State::FinishedReached
                } else {
                    State::Analyzing
                }
            }
            State::FinishedReached => break "Target reached",
            State::FinishedBlocked => break "Blocked - finished",
        };
    };

    finish(exit_label).await;
}

/// Extract forward progress in cm from a `DriveQueueCompletion`.
///
/// `distance_factor` is the calibration factor read from `CALIBRATION_STATE`.
/// It is applied *in reverse* so the cm reported here matches the cm originally
/// requested by the drive command (which multiplies by the same factor when
/// converting cm → target revolutions).
fn extract_progress_cm(completion: &crate::task::drive::types::DriveQueueCompletion, distance_factor: f32) -> f32 {
    completion
        .last_step_completion
        .as_ref()
        .map_or(0.0, |last| match &last.telemetry {
            CompletionTelemetry::DriveDistance {
                achieved_left_revs,
                achieved_right_revs,
                ..
            } => {
                let avg_revs = (achieved_left_revs + achieved_right_revs) / 2.0;
                if distance_factor > 0.0 {
                    avg_revs * SPROCKET_CIRCUMFERENCE_CM / distance_factor
                } else {
                    avg_revs * SPROCKET_CIRCUMFERENCE_CM
                }
            }
            _ => 0.0,
        })
}

/// Pure per-leg dead-reckoning update.
///
/// Rotation is applied **first** (heading changes), then forward drive happens
/// along the new heading.
fn update_odometry(
    x_cm: f32,
    y_cm: f32,
    heading_deg: f32,
    rotation_deg: f32,
    direction: RotationDirection,
    distance_cm: f32,
) -> (f32, f32, f32) {
    // Apply rotation to heading first.
    let new_heading_deg = match direction {
        RotationDirection::Clockwise => heading_deg + rotation_deg,
        RotationDirection::CounterClockwise => heading_deg - rotation_deg,
    };

    // Forward drive along the new heading.
    let heading_rad = new_heading_deg.to_radians();
    let new_x = x_cm + distance_cm * libm::cosf(heading_rad);
    let new_y = y_cm + distance_cm * libm::sinf(heading_rad);

    (new_x, new_y, new_heading_deg)
}

/// Clean up and exit the mode, showing the finish message until the user
/// dismisses it with the encoder button.
async fn finish(label: &'static str) {
    // Update display state with final message.
    {
        let mut ds = DISPLAY_STATE.lock().await;
        ds.state_label = label;
    }
    send_drive_command(DriveCommand::Drive(DriveAction::Brake)).await;
    Timer::after(Duration::from_millis(200)).await;
    autonomous_mode::release_autonomous_mode();

    // Wait for the user to dismiss by pressing the encoder button.
    while ACTIVE.load(Ordering::Relaxed) {
        Timer::after(Duration::from_millis(100)).await;
    }

    // Return to main menu.
    send_ui_event(UiEvent::ShowMainMenu).await;
    info!("attempt-straight: deactivated ({})", label);
}
