//! Obstacle- and floor-drop-avoidance autonomous drive mode: coast until
//! `LiDAR` detects an obstacle within the forward threshold or a floor-drop
//! sensor detects a ledge, then back up and turn a random angle before resuming.
//!
//! # Control flow
//!
//! ```text
//! start() ──► drive forward (SetTracks)
//!                  │
//!     obstacle or floor drop detected
//!  (perception::is_obstacle_detected ||
//!   perception::is_floor_drop_detected)
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
//! Call [`start`] to enable the `LiDAR` and begin the mode, and [`stop`] to
//! request a graceful exit. Enabling can take seconds, so [`stop`] latches its
//! intent in [`STOP_REQUESTED`]: a stop that lands while [`start`] is still warming
//! the sensor makes the start disable the sensor and abandon the start instead of
//! driving. If the `LiDAR` cannot be enabled, [`start`] fails and the mode never
//! drives: the downward rangefinder is a floor-drop sensor, not an obstacle sensor.
//!
//! The obstacle decision reads perception's lock-free flag, which the `LiDAR`
//! task drives through the Front Sector test in the `lidar-cloud` crate (half-angle
//! and threshold defined there, once). Floor-drop sensors cover ledge detection, and
//! `MotorCommand::SetTracks` does direct motor control.

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
        sensors::lidar::{self, LidarStatus},
    },
};

// ── Active flag ───────────────────────────────────────────────────────────────

/// Set while the coast-and-avoid loop is running.
///
/// Raised by [`start`] once the sensor is streaming and the start request is
/// accepted, and cleared again if the request is refused. Also cleared by
/// [`stop`], which latches the request in [`STOP_REQUESTED`] at the same time so
/// a start still waiting for the sensor can notice it.
static ACTIVE: AtomicBool = AtomicBool::new(false);

/// Records a [`stop`] that lands while [`start`] is still warming the `LiDAR`.
///
/// Enabling can take seconds, and the running screen is up the whole time, so the
/// operator can tap Stop before [`start`] has raised [`ACTIVE`] — a stop that
/// would otherwise have nothing to clear and would be lost. [`stop`] latches the
/// intent here, and [`start`] checks it while polling the sensor status: a latched
/// stop makes the start disable the sensor and abandon the start. [`start`] arms
/// the latch at its top, so a stop from a finished run is inert while a fresh one
/// during the warm-up is honoured. The store uses `Release` and the check
/// `Acquire`, making the ordering explicit even though every access happens on
/// core0.
static STOP_REQUESTED: AtomicBool = AtomicBool::new(false);

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

/// Interval between `LiDAR` status polls while waiting for the sensor to warm up (milliseconds).
const LIDAR_STATUS_POLL_MS: u64 = 50;

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

// ── Start errors ───────────────────────────────────────────────────────────────

/// Why coast-and-avoid could not start.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StartError {
    /// Another autonomous mode is already active.
    Busy,
    /// The `LiDAR` could not be enabled. The mode refuses to run, because the
    /// downward rangefinder is a floor-drop sensor and not an obstacle sensor.
    LidarUnavailable,
    /// A stop arrived while the `LiDAR` was still warming, so the mode was
    /// abandoned instead of started.
    ///
    /// The UI records the [`label`](Self::label) through its `activity::fail`,
    /// but that is a no-op while the activity is already Idle — which it is after
    /// a Stop — so a cancelled start cannot resurrect a failure screen.
    Cancelled,
}

impl StartError {
    /// A short, operator-facing description for the running screen.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Busy => "LiDAR busy",
            Self::LidarUnavailable => "LiDAR unavailable",
            Self::Cancelled => "Stopped",
        }
    }
}

// ── Public API ───────────────────────────────────────────────────────────────────

/// Spawn the coast-and-avoid autonomous task.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(coast_obstacle_avoid_task().unwrap());
}

/// Enable the `LiDAR` and activate the coast-and-avoid autonomous mode.
///
/// The sensor is enabled and waited on *before* the mode is spawned, so the
/// mode never drives while the sensor is still warming or unavailable. A stop
/// that arrives during the wait disables the sensor and abandons the start,
/// leaving nothing enabled.
///
/// # Errors
///
/// Returns [`StartError::LidarUnavailable`] if the `LiDAR` cannot be brought
/// up, [`StartError::Busy`] if another autonomous mode is already active, or
/// [`StartError::Cancelled`] if a stop arrived while the sensor was warming.
pub async fn start() -> Result<(), StartError> {
    // Arm the cancel latch before enabling: a stop from an earlier run must
    // not count, but one that arrives during the warm-up must.
    STOP_REQUESTED.store(false, Ordering::Release);

    // A previous run can leave the sensor latched `Failed`, and only a disable
    // releases that latch. Clear it here, before the enable, and wait for the
    // reset to publish `Off`, so this start's own bring-up decides the outcome
    // rather than a stale `Failed` that would refuse it before it begins.
    if lidar::status() == LidarStatus::Failed {
        lidar::disable().await;
        loop {
            if STOP_REQUESTED.load(Ordering::Acquire) {
                return Err(StartError::Cancelled);
            }
            if lidar::status() != LidarStatus::Failed {
                break;
            }
            Timer::after(Duration::from_millis(LIDAR_STATUS_POLL_MS)).await;
        }
    }

    lidar::enable().await;

    // The sensor's own task owns the bring-up, so poll its lock-free status
    // until it is streaming, has failed, or a stop arrives. The enable is
    // one-way, so this poll is the only wait.
    loop {
        if STOP_REQUESTED.load(Ordering::Acquire) {
            lidar::disable().await;
            return Err(StartError::Cancelled);
        }
        match lidar::status() {
            LidarStatus::Streaming => break,
            LidarStatus::Failed => {
                // The sensor is powered down and will not retry until a fresh
                // disable-then-enable, so clear the need and refuse to drive.
                lidar::disable().await;
                return Err(StartError::LidarUnavailable);
            }
            LidarStatus::Off | LidarStatus::Warming => {
                Timer::after(Duration::from_millis(LIDAR_STATUS_POLL_MS)).await;
            }
        }
    }

    ACTIVE.store(true, Ordering::Relaxed);

    if !autonomous_mode::request_start(AutonomousCommand::CoastObstacleAvoid).await {
        // No task was spawned, so roll the enable back. No running coast task
        // can observe this, because the mode active now is another autonomous
        // mode.
        ACTIVE.store(false, Ordering::Relaxed);
        lidar::disable().await;
        return Err(StartError::Busy);
    }

    Ok(())
}

/// Request a graceful stop of the coast-and-avoid mode.
///
/// Clears the active flag so the loop exits after the current drive command
/// resolves, and latches the request in [`STOP_REQUESTED`] so a start still
/// waiting for the `LiDAR` disables the sensor and abandons the start instead of
/// driving.
pub fn stop() {
    ACTIVE.store(false, Ordering::Relaxed);
    STOP_REQUESTED.store(true, Ordering::Release);
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
    // Leaving the mode disables the sensor; the sensor's task stops it, drops
    // its power, and clears its stale cloud and obstacle flag.
    lidar::disable().await;
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

        // Check perception state for obstacles or floor drops ahead.
        if perception::is_obstacle_detected() || perception::is_floor_drop_detected() {
            info!("coast-avoid: obstacle or floor drop detected ahead, braking");
            motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            Timer::after(Duration::from_millis(200)).await;

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
