//! Obstacle-avoidance autonomous drive mode: coast until `LiDAR` detects an obstacle
//! within the forward threshold, then back up and turn a random angle before resuming.
//!
//! # Control flow
//!
//! ```text
//! start() ──► drive forward (SetTracks)
//!                  │
//!           obstacle detected
//!  (perception::is_obstacle_detected)
//!                  │
//!          ┌── ACTIVE? ──┐
//!          No            Yes
//!          │             │
//!        brake        back up (SetTracks, reverse)
//!        exit            │
//!                   random turn (SetTracks, differential)
//!                        │
//!                  raise ObstacleAvoidanceAttempted
//!                        │
//!                  ◄─────┘ (loop)
//! ```
//!
//! # Starting and stopping
//!
//! Call [`start`] to begin the mode and [`stop`] to request a graceful exit.
//!
//! # v3 Changes from v2
//! - `LiDAR` `is_obstacle_ahead(30.0, 60)` replaces ultrasonic polling.
//! - `MotorCommand::SetTracks` for direct motor control replaces `DriveQueueBuilder`.
//! - All ultrasonic/IR imports and usage removed.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use nanorand::{Rng, WyRand};

use crate::{
    system::{
        event::{Events, raise_event},
        state::perception,
    },
    task::{
        autonomous_mode::{self, AutonomousCommand},
        motor_driver::{self, MotorCommand},
    },
};

// ── Active flag ───────────────────────────────────────────────────────────────

/// Set while the coast-and-avoid loop is running.
static ACTIVE: AtomicBool = AtomicBool::new(false);

// ── Tuning constants ──────────────────────────────────────────────────────────

/// Speed for forward coasting (−100 … +100).
const FORWARD_SPEED: i8 = 80;

/// Speed for reverse backup (−100 … +100).
const REVERSE_SPEED: i8 = -60;

/// In-place rotation speed magnitude (−100 … +100).
const TURN_SPEED: i8 = 60;

/// Minimum random turn angle (degrees).
const TURN_ANGLE_MIN: u8 = 45;

/// Maximum random turn angle (degrees).
const TURN_ANGLE_MAX: u8 = 180;

/// Duration to drive backward after obstacle detection (milliseconds).
const BACKUP_DURATION_MS: u64 = 600;

/// Interval between perception checks while driving forward (milliseconds).
const PERCEPTION_CHECK_INTERVAL_MS: u64 = 100;

/// Approximate time to turn 90 degrees with differential drive at `TURN_SPEED` (ms).
const TURN_90_DEG_MS: u64 = 500;

// ── State machine ────────────────────────────────────────────────────────────────

/// States of the coast-and-avoid state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// Driving forward, periodically checking for obstacles.
    Forward,
    /// Backing up after an obstacle was detected.
    BackingUp,
    /// Turning a random angle to avoid the obstacle.
    Turning,
}

// ── Public API ───────────────────────────────────────────────────────────────────

/// Spawn the coast-and-avoid autonomous task.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(coast_obstacle_avoid_task().unwrap());
}

/// Activate the coast-and-avoid autonomous mode.
///
/// Requests mode start through the autonomous mode controller.
pub async fn start() -> bool {
    if !autonomous_mode::request_start(AutonomousCommand::CoastObstacleAvoid).await {
        return false;
    }

    ACTIVE.store(true, Ordering::Relaxed);
    true
}

/// Request a graceful stop of the coast-and-avoid mode.
///
/// Clears the active flag so the loop exits after the current drive command
/// resolves.
pub fn stop() {
    ACTIVE.store(false, Ordering::Relaxed);
}

// ── Task ──────────────────────────────────────────────────────────────────────

/// Coast-and-avoid autonomous drive task.
///
/// Spawned on demand by the autonomous mode controller each time the mode is
/// activated. Runs until deactivated, then returns so the task slot is freed
/// for the next activation.
#[embassy_executor::task]
pub async fn coast_obstacle_avoid_task() {
    info!("coast-avoid: activated");

    // Ensure a clean starting state.
    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
    Timer::after(Duration::from_millis(200)).await;

    // Enable motor drivers.
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
    Timer::after(Duration::from_millis(10)).await;

    let mut turn_direction_clockwise: bool = true;
    let mut turn_degrees: u8 = TURN_ANGLE_MIN;
    let mut state = State::Forward;

    while ACTIVE.load(Ordering::Relaxed) {
        state = match state {
            State::Forward => run_forward().await,
            State::BackingUp => run_backing_up().await,
            State::Turning => {
                let next = run_turning(turn_degrees, turn_direction_clockwise).await;

                // After turning, pick a new random angle and flip direction
                // for the next avoidance cycle.
                turn_degrees = random_turn_angle();
                turn_direction_clockwise = !turn_direction_clockwise;

                next
            }
        };
    }

    // Clean stop.
    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
    Timer::after(Duration::from_millis(200)).await;
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
    autonomous_mode::release_autonomous_mode();
    info!("coast-avoid: deactivated");
}

// ── State handlers ───────────────────────────────────────────────────────────────

/// Drive forward until an obstacle is detected or the mode is stopped.
async fn run_forward() -> State {
    info!("coast-avoid: driving forward");

    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: FORWARD_SPEED,
        right_speed: FORWARD_SPEED,
    })
    .await;

    loop {
        if !ACTIVE.load(Ordering::Relaxed) {
            return State::Forward;
        }

        // Check perception state for obstacles ahead.
        if perception::is_obstacle_detected() {
            info!("coast-avoid: obstacle detected ahead, braking");
            motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            Timer::after(Duration::from_millis(200)).await;

            raise_event(Events::ObstacleDetected {
                source: crate::system::event::ObstacleSource::Lidar,
                detected: true,
            })
            .await;

            return State::BackingUp;
        }

        Timer::after(Duration::from_millis(PERCEPTION_CHECK_INTERVAL_MS)).await;
    }
}

/// Back up a fixed distance (time-based) to clear the obstacle.
async fn run_backing_up() -> State {
    info!("coast-avoid: backing up");

    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: REVERSE_SPEED,
        right_speed: REVERSE_SPEED,
    })
    .await;

    Timer::after(Duration::from_millis(BACKUP_DURATION_MS)).await;

    if !ACTIVE.load(Ordering::Relaxed) {
        return State::Forward;
    }

    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
    Timer::after(Duration::from_millis(200)).await;

    State::Turning
}

/// Turn a random angle in the given direction.
///
/// Uses differential drive: one track forward, the other backward.
async fn run_turning(degrees: u8, clockwise: bool) -> State {
    info!(
        "coast-avoid: turning {} deg {}",
        degrees,
        if clockwise { "CW" } else { "CCW" }
    );

    let (left_speed, right_speed) = if clockwise {
        (TURN_SPEED, -TURN_SPEED)
    } else {
        (-TURN_SPEED, TURN_SPEED)
    };

    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed,
        right_speed,
    })
    .await;

    // Compute turn duration proportional to the angle.
    let turn_ms = u64::from(degrees) * TURN_90_DEG_MS / 90;
    Timer::after(Duration::from_millis(turn_ms)).await;

    if !ACTIVE.load(Ordering::Relaxed) {
        return State::Forward;
    }

    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
    Timer::after(Duration::from_millis(200)).await;

    raise_event(Events::ObstacleAvoidanceAttempted).await;

    State::Forward
}

// ── Helpers ──────────────────────────────────────────────────────────────────────

/// Generate a random turn angle between `TURN_ANGLE_MIN` and `TURN_ANGLE_MAX`.
fn random_turn_angle() -> u8 {
    let seed = Instant::now().as_micros();
    let mut rng = WyRand::new_seed(seed);
    rng.generate_range(TURN_ANGLE_MIN..=TURN_ANGLE_MAX)
}
