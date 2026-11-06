//! Obstacle- and floor-drop-avoidance autonomous drive mode: coast forward until
//! the `LiDAR` reports an obstacle within the forward threshold or a floor-drop
//! sensor reports a ledge, then brake, back up and turn a random angle before
//! resuming.
//!
//! # Control flow
//!
//! ```text
//! pre-flight: obstacle || floor drop?
//!        │ no                              │ yes
//!        ▼                                 │
//! forward leg [DriveContinuous]             │
//!        │ obstacle edge / stop            │
//!        ▼                                 ▼
//! avoidance manoeuvre [Brake, back up, turn]
//!        │
//!        └──► raise ObstacleAvoidanceAttempted ──► pre-flight
//! ```
//!
//! Every leg is one closed-loop [`DriveQueueBuilder`] queue. The forward leg is a
//! single [`DriveAction::DriveContinuous`] step that ends only on an interrupt: the
//! obstacle and floor-drop behaviour handlers send `EmergencyBrake` on their edges.
//! The avoidance manoeuvre is one queue of [`DriveAction::Brake`],
//! [`DriveAction::DriveDistance`] and [`DriveAction::RotateExact`] steps, all
//! abort-on-fail, so a failure on any step stops the rest.
//!
//! # Claim and lifecycle
//!
//! The UI's `LiDAR` warm-up helper spawns this task with the one Active Claim the
//! UI took with [`crate::task::procedure::start`]. The claim's stop latch is the
//! only cancellation authority: a stop makes the in-flight queue resolve
//! `Cancelled`, and the mode then brakes, disables the `LiDAR` and releases the
//! claim with [`Claim::abandon`]. A failure records its reason with
//! [`Claim::fail`] instead. The mode never requests or releases any other
//! controller: the one Active Claim is the only authority.
//!
//! The obstacle decision reads perception's lock-free flag, which the `LiDAR` task
//! drives through the Front Sector test in the `lidar-cloud` crate (half-angle and
//! threshold defined there, once). Floor-drop sensors cover ledge detection.

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::Instant;
use nanorand::{Rng, WyRand};

use crate::{
    system::{
        event::{Events, raise_event},
        state::perception,
    },
    task::{
        drive::{
            CompletionStatus, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind, DriveQueueBuilder,
            DriveQueueCompletion, DriveQueueSubmitError,
            types::{DriveQueueBuildError, RotationDirection, RotationMotion},
        },
        procedure::Claim,
        sensors::lidar,
    },
};

// ── Tuning constants ──────────────────────────────────────────────────────────

/// Speed for forward coasting (0-100).
const FORWARD_SPEED: u8 = 80;

/// Speed for reverse backup (0-100).
const REVERSE_SPEED: u8 = 60;

/// In-place rotation speed magnitude (0-100).
const TURN_SPEED: u8 = 60;

/// Minimum random turn angle (degrees).
const TURN_ANGLE_MIN: u8 = 45;

/// Maximum random turn angle (degrees).
const TURN_ANGLE_MAX: u8 = 180;

/// Distance to reverse after detecting a blocked route, in centimetres.
///
/// Provisional: tuned at the first physical bring-up.
const BACKUP_DISTANCE_CM: f32 = 20.0;

/// Consecutive avoidance manoeuvres without a forward leg that end the mode.
///
/// The counter resets only when a forward leg starts, so a flag that never clears
/// cannot loop forever: three manoeuvres without a forward leg fail with
/// `"Blocked"`.
const BLOCKED_MANOEUVRE_LIMIT: u8 = 3;

// ── Spawn ─────────────────────────────────────────────────────────────────────

/// Spawn the coast-and-avoid mode task under `claim`.
///
/// Returns `true` once the task is spawned. If the task slot is unavailable this
/// logs and returns `false` without touching `claim`: the caller still owns it and
/// is responsible for powering the `LiDAR` down and abandoning the claim.
#[must_use]
pub fn spawn(spawner: Spawner, claim: Claim) -> bool {
    let Ok(token) = coast_obstacle_avoid_task(claim) else {
        error!("coast-avoid: spawn failed");
        return false;
    };

    spawner.spawn(token);
    true
}

// ── Task ──────────────────────────────────────────────────────────────────────

/// Coast-and-avoid autonomous drive task.
///
/// Spawned once the `LiDAR` is streaming. Runs until a stop or a failure, then
/// stops the hardware and records the terminal claim outcome.
#[embassy_executor::task]
async fn coast_obstacle_avoid_task(claim: Claim) {
    info!("coast-avoid: activated");

    let outcome = run(&claim).await;
    stop_hardware().await;

    match outcome {
        Outcome::Stopped => claim.abandon().await,
        Outcome::Failed(reason) => claim.fail(reason).await,
    }

    info!("coast-avoid: deactivated");
}

// ── Mode loop ─────────────────────────────────────────────────────────────────

/// How the mode ended, before the terminal claim call.
enum Outcome {
    /// The operator stopped the mode; abandon the claim.
    Stopped,
    /// The mode failed for `reason`; fail the claim.
    Failed(&'static str),
}

/// How a forward leg ended.
enum ForwardOutcome {
    /// The operator stopped the mode.
    Stopped,
    /// An obstacle or floor-drop flag is set; run an avoidance manoeuvre.
    Blocked,
    /// The leg failed without a set flag; fail the mode with `reason`.
    Failed(&'static str),
}

/// How one avoidance manoeuvre ended.
enum ManoeuvreOutcome {
    /// The manoeuvre completed; raise the avoidance event and re-check the flags.
    Attempted,
    /// The operator stopped the mode.
    Stopped,
    /// The queue was cancelled without a stop; count another manoeuvre and retry.
    Restart,
    /// The manoeuvre failed; fail the mode with `reason`.
    Failed(&'static str),
}

/// Run the mode until it is stopped or fails.
///
/// The blocked counter resets only when a forward leg starts, never when a
/// manoeuvre succeeds: a flag that stays set therefore runs at most
/// [`BLOCKED_MANOEUVRE_LIMIT`] manoeuvres before failing with `"Blocked"`.
async fn run(claim: &Claim) -> Outcome {
    let mut blocked_manoeuvres: u8 = 0;
    let mut turn_clockwise = true;

    loop {
        // Pre-flight: only clear ground starts a forward leg, and starting one
        // resets the blocked counter.
        if !blocked_ahead() {
            blocked_manoeuvres = 0;
            match run_forward(claim).await {
                ForwardOutcome::Stopped => return Outcome::Stopped,
                ForwardOutcome::Failed(reason) => return Outcome::Failed(reason),
                ForwardOutcome::Blocked => {}
            }
        }

        // Avoidance: keep manoeuvring until the pre-flight is clear.
        loop {
            if blocked_manoeuvres >= BLOCKED_MANOEUVRE_LIMIT {
                return Outcome::Failed("Blocked");
            }
            blocked_manoeuvres += 1;

            match run_manoeuvre(claim, turn_clockwise).await {
                ManoeuvreOutcome::Stopped => return Outcome::Stopped,
                ManoeuvreOutcome::Failed(reason) => return Outcome::Failed(reason),
                ManoeuvreOutcome::Restart => {
                    // A cancellation without a stop restarts the manoeuvre; the
                    // pre-flight is deliberately not consulted here.
                    turn_clockwise = !turn_clockwise;
                }
                ManoeuvreOutcome::Attempted => {
                    raise_event(Events::ObstacleAvoidanceAttempted).await;
                    turn_clockwise = !turn_clockwise;
                    if !blocked_ahead() {
                        break;
                    }
                }
            }
        }
    }
}

/// Run the forward leg and classify how it ended.
async fn run_forward(claim: &Claim) -> ForwardOutcome {
    info!("coast-avoid: driving forward");

    match submit_queue(build_forward()).await {
        Err(reason) => ForwardOutcome::Failed(reason),
        Ok(completion) => match completion.status {
            CompletionStatus::Cancelled => {
                if claim.is_stop_requested() {
                    ForwardOutcome::Stopped
                } else if blocked_ahead() {
                    ForwardOutcome::Blocked
                } else {
                    ForwardOutcome::Failed("Drive interrupted")
                }
            }
            CompletionStatus::Failed(reason) => {
                if blocked_ahead() {
                    ForwardOutcome::Blocked
                } else {
                    ForwardOutcome::Failed(reason)
                }
            }
            CompletionStatus::Success => ForwardOutcome::Failed("Continuous Drive completed"),
        },
    }
}

/// Run one avoidance manoeuvre and classify how it ended.
async fn run_manoeuvre(claim: &Claim, clockwise: bool) -> ManoeuvreOutcome {
    let degrees = random_turn_angle();
    let direction = if clockwise {
        RotationDirection::Clockwise
    } else {
        RotationDirection::CounterClockwise
    };
    info!(
        "coast-avoid: backing up {=f32} cm and turning {} deg {}",
        BACKUP_DISTANCE_CM,
        degrees,
        if clockwise { "CW" } else { "CCW" }
    );

    match submit_queue(build_manoeuvre(f32::from(degrees), direction)).await {
        Err(reason) => ManoeuvreOutcome::Failed(reason),
        Ok(completion) => match completion.status {
            CompletionStatus::Success => ManoeuvreOutcome::Attempted,
            CompletionStatus::Cancelled => {
                if claim.is_stop_requested() {
                    ManoeuvreOutcome::Stopped
                } else {
                    ManoeuvreOutcome::Restart
                }
            }
            CompletionStatus::Failed(reason) => ManoeuvreOutcome::Failed(reason),
        },
    }
}

// ── Queues ────────────────────────────────────────────────────────────────────

/// Build the forward leg: one continuous-drive step.
fn build_forward() -> Result<DriveQueueBuilder, DriveQueueBuildError> {
    let mut queue = DriveQueueBuilder::new();
    queue.push(DriveCommand::Drive(DriveAction::DriveContinuous {
        direction: DriveDirection::Forward,
        speed: FORWARD_SPEED,
    }))?;
    Ok(queue)
}

/// Build the avoidance manoeuvre: brake, reverse a fixed distance, then turn.
fn build_manoeuvre(degrees: f32, direction: RotationDirection) -> Result<DriveQueueBuilder, DriveQueueBuildError> {
    let mut queue = DriveQueueBuilder::new();
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Brake))?;
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::DriveDistance {
        kind: DriveDistanceKind::Straight {
            distance_cm: BACKUP_DISTANCE_CM,
        },
        direction: DriveDirection::Backward,
        speed: REVERSE_SPEED,
    }))?;
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::RotateExact {
        degrees,
        direction,
        motion: RotationMotion::Stationary { speed: TURN_SPEED },
    }))?;
    Ok(queue)
}

/// Build the cleanup queue: brake and enter standby.
fn build_stop() -> Result<DriveQueueBuilder, DriveQueueBuildError> {
    let mut queue = DriveQueueBuilder::new();
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Brake))?;
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::Standby))?;
    Ok(queue)
}

/// Build and submit a queue, mapping build and submit errors to a short reason.
async fn submit_queue(
    queue: Result<DriveQueueBuilder, DriveQueueBuildError>,
) -> Result<DriveQueueCompletion, &'static str> {
    let queue = queue.map_err(DriveQueueBuildError::label)?;
    queue.submit().await.map_err(DriveQueueSubmitError::label)
}

// ── Cleanup ───────────────────────────────────────────────────────────────────

/// Stop the hardware once: brake, enter standby, then power the `LiDAR` down.
///
/// The interrupt that ended the run has usually braked already, but an explicit
/// `Brake`/`Standby` pair leaves the drive subsystem idle on every exit path. A
/// failed cleanup queue is logged only; the terminal claim call still runs.
async fn stop_hardware() {
    if let Err(reason) = submit_queue(build_stop()).await {
        warn!("coast-avoid: cleanup queue failed: {}", reason);
    }
    lidar::disable().await;
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Whether either forward-blocking perception flag is set.
fn blocked_ahead() -> bool {
    perception::is_obstacle_detected() || perception::is_floor_drop_detected()
}

/// Generate a random turn angle between `TURN_ANGLE_MIN` and `TURN_ANGLE_MAX`.
fn random_turn_angle() -> u8 {
    let seed = Instant::now().as_micros();
    let mut rng = WyRand::new_seed(seed);
    rng.generate_range(TURN_ANGLE_MIN..=TURN_ANGLE_MAX)
}
