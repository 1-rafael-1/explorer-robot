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
//! Call [`start`] to acquire the `LiDAR` and begin the mode, and [`stop`] to
//! request a graceful exit. Acquisition can take seconds, so [`stop`] latches its
//! intent in [`STOP_REQUESTED`]: a stop that lands while [`start`] is still warming
//! the sensor makes the start hand its lease straight back instead of driving. If
//! the `LiDAR` cannot be acquired, [`start`] fails and the mode never drives: the
//! downward rangefinder is a floor-drop sensor, not an obstacle sensor.
//!
//! The obstacle decision reads perception's lock-free flag, which the `LiDAR`
//! task drives through the Front Sector test in the `lidar-cloud` crate (half-angle
//! and threshold defined there, once). Floor-drop sensors cover ledge detection, and
//! `MotorCommand::SetTracks` does direct motor control.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
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
        sensors::lidar::{self, AcquireError},
    },
};

// ── Active flag ───────────────────────────────────────────────────────────────

/// Set while the coast-and-avoid loop is running.
///
/// Raised only after [`LIDAR_LEASE`] holds the lease and cleared again if the
/// start is refused, so a running loop always has a lease to hand back. Also
/// cleared by [`stop`], which latches the request in [`STOP_REQUESTED`] at the
/// same time so a start still in flight can notice it.
static ACTIVE: AtomicBool = AtomicBool::new(false);

/// Records a [`stop`] that lands while [`start`] is still warming the `LiDAR`.
///
/// Acquisition can take seconds, and the running screen is up the whole time, so
/// the operator can tap Stop before [`start`] has raised [`ACTIVE`] — a stop that
/// would otherwise have nothing to clear and would be lost. [`stop`] latches the
/// intent here, and [`start`] checks it immediately before raising [`ACTIVE`]: a
/// latched stop makes the start hand its lease straight back and refuse to drive.
/// [`start`] arms the latch at its top, so a stop from a finished run is inert
/// while a fresh one during the warm-up is honoured. The store uses `Release` and
/// the check `Acquire`, making the hand-off explicit even though every access
/// happens on core0.
static STOP_REQUESTED: AtomicBool = AtomicBool::new(false);

// ── LiDAR lease ───────────────────────────────────────────────────────────────

/// The `LiDAR` lease held by the running mode.
///
/// [`start`] stores the lease here and [`coast_obstacle_avoid_task`] takes and
/// releases it on exit. The task is spawned by the autonomous-mode controller
/// rather than by [`start`], so the token cannot travel as a task argument and
/// crosses the two through this cell instead. Because the cell is published
/// before the controller can spawn the task, a lease placed here is always
/// consumed: the task takes it on exit, or [`start`] takes it back when the
/// start request is refused.
static LIDAR_LEASE: Mutex<CriticalSectionRawMutex, Option<lidar::Lease>> = Mutex::new(None);

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

// ── Start errors ───────────────────────────────────────────────────────────────

/// Why coast-and-avoid could not start.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StartError {
    /// Another autonomous mode is already active, or a `LiDAR` acquisition is
    /// already in flight and not yet streaming.
    Busy,
    /// The `LiDAR` could not be acquired. The mode refuses to run, because the
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

/// Acquire the `LiDAR` and activate the coast-and-avoid autonomous mode.
///
/// The sensor is acquired *before* the mode is spawned, so the mode never drives
/// while the sensor is still warming or unavailable.
///
/// # Hand-off ordering
///
/// `request_start` is the call that makes the autonomous-mode controller spawn
/// [`coast_obstacle_avoid_task`], so the lease is published in [`LIDAR_LEASE`] and
/// [`ACTIVE`] is raised *before* it. A spawned task therefore can never observe
/// `ACTIVE == true` with no lease to consume, and can never exit holding a lease
/// it was not handed. If the request is refused, both are rolled back — nothing
/// is left active and the lease is released. The mode active at that point is
/// another autonomous mode, or a coast run already past its loop, so briefly
/// toggling this module's `ACTIVE` cannot be observed by a running coast task.
///
/// A stop during the warm-up is preserved rather than lost. [`STOP_REQUESTED`] is
/// armed before the acquisition, so only a stop from this run counts, and it is
/// checked immediately before [`ACTIVE`] is raised, with no await between the
/// check and the store. Raising `ACTIVE` first and checking after would leave the
/// [`LIDAR_LEASE`] lock await between the check and the flag, letting a stop be
/// overwritten; and a stop after the check clears `ACTIVE`, which the spawned
/// loop's `while ACTIVE` test observes before it drives.
///
/// # Errors
///
/// Returns [`StartError::LidarUnavailable`] if the `LiDAR` cannot be brought up,
/// [`StartError::Busy`] if another autonomous mode is already active or a
/// previous coast run still holds its lease, or [`StartError::Cancelled`] if a
/// stop arrived while the sensor was warming.
pub async fn start() -> Result<(), StartError> {
    // Arm the cancel latch before the acquisition: a stop from an earlier run
    // must not count, but one that arrives during the warm-up must.
    STOP_REQUESTED.store(false, Ordering::Release);

    let lease = match lidar::acquire().await {
        Ok(lease) => lease,
        Err(AcquireError::Busy) => return Err(StartError::Busy),
        Err(AcquireError::Failed) => return Err(StartError::LidarUnavailable),
    };

    // Publish the lease before the task can be spawned. If a previous run still
    // holds one, refuse rather than overwrite it: dropping that lease unreleased
    // would leak its ref count and keep the sensor powered.
    {
        let mut slot = LIDAR_LEASE.lock().await;
        if slot.is_some() {
            drop(slot);
            lidar::release(lease).await;
            return Err(StartError::Busy);
        }
        *slot = Some(lease);
    }

    // Check the latch with no await before raising ACTIVE: on this cooperative
    // executor a stop cannot land between the check and the store, and a stop
    // after the store clears ACTIVE, which the spawned loop sees before driving.
    if STOP_REQUESTED.load(Ordering::Acquire) {
        let lease = LIDAR_LEASE.lock().await.take();
        if let Some(lease) = lease {
            lidar::release(lease).await;
        }
        return Err(StartError::Cancelled);
    }

    ACTIVE.store(true, Ordering::Relaxed);

    if !autonomous_mode::request_start(AutonomousCommand::CoastObstacleAvoid).await {
        // No task was spawned, so roll the hand-off back: lower the flag and
        // take the lease back to release it. No running coast task can observe
        // this, because the mode active now is another autonomous mode.
        ACTIVE.store(false, Ordering::Relaxed);
        let lease = LIDAR_LEASE.lock().await.take();
        if let Some(lease) = lease {
            lidar::release(lease).await;
        }
        return Err(StartError::Busy);
    }

    Ok(())
}

/// Request a graceful stop of the coast-and-avoid mode.
///
/// Clears the active flag so the loop exits after the current drive command
/// resolves, and latches the request in [`STOP_REQUESTED`] so a start still
/// warming the `LiDAR` refuses to drive when it reaches the hand-off.
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
    // Leaving the mode hands back the lease it holds: the sensor stops, drops
    // its power, and clears its stale cloud and obstacle flag only when no other
    // mode still holds a lease.
    let lease = LIDAR_LEASE.lock().await.take();
    if let Some(lease) = lease {
        lidar::release(lease).await;
    }
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
