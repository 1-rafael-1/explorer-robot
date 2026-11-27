//! Rotation adapter: drives `drive-core`'s `RotateExact` intent, then runs the
//! provisional post-stop correction phase.
//!
//! The control law itself lives in [`drive_core::Intent`]. This module is the
//! adapter half: it starts and stops the rotation IMU, drains the IMU feedback
//! channel, builds an [`Observation`], executes the model's [`IntentAction`]s,
//! keeps the drive interrupt selected around every `WaitUntil`, and maps the
//! model's terminal outcome onto the firmware's completion.
//!
//! # Correction phase
//!
//! The post-stop correction phase is still firmware-owned in this ticket. It
//! runs only for a stationary turn, seeded from the model's anchored target yaw
//! and flip count (the two temporary `Intent` accessors), and it keeps the
//! existing torque ladder, convergence, overshoot counting, per-iteration
//! timeout, exhaustion and fault labels. Ticket 04 moves it into `drive-core`.
//!
//! # Telemetry logging
//!
//! Rotation trace fields are logged only when the `telemetry_logs` feature is
//! enabled; that feature also turns on `drive-core`'s `trace` feature. The
//! model never logs and never depends on `defmt`.

use drive_core::{
    CompletionTelemetry, DriveFault, DriveOutcome, Intent, IntentAction, IntentParams, Observation, RotationDirection,
    RotationMotion, YawSample,
};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use libm::roundf;

use crate::{
    system::state::motion,
    task::{
        drive::{
            api::DRIVE_INTERRUPT,
            clear_imu_measurements,
            sensors::{control as lifecycle, data::IMU_FEEDBACK_CHANNEL},
            types,
        },
        motor_driver::{self, MotorCommand},
        sensors::imu::ImuMeasurement,
    },
};

// ── Model adapter ─────────────────────────────────────────────────────────────

/// Maximum non-blocking actions the adapter executes against one observation.
///
/// The model never needs more than a handful: the start command, a control
/// command and the stop command are each followed by a `WaitUntil` or
/// `Finished`. The guard turns a model defect into a terminal fault instead of
/// a busy loop.
const MAX_ACTIONS_PER_OBSERVATION: u8 = 8;

/// Completion detail for the adapter's non-blocking action guard.
///
/// Unreachable with the current model; it exists so a future defect cannot hang
/// the drive task.
const ACTION_LOOP_LABEL: &str = "RotationActionLoop";

/// The intermediate outcome of polling the model.
pub(super) enum RotationPoll {
    /// A drive interrupt arrived while the adapter waited.
    Interrupt(types::InterruptKind),
    /// The intent reached a terminal result.
    Step(RotationStepResult),
}

/// Result of running a rotation intent to a terminal outcome.
pub(super) enum RotationStepResult {
    /// Rotation completed successfully with final telemetry.
    Completed {
        /// Completion telemetry captured at success.
        telemetry: CompletionTelemetry,
    },
    /// Rotation failed with a static reason and telemetry snapshot.
    Failed {
        /// Failure reason identifier.
        reason: &'static str,
        /// Completion telemetry captured at failure.
        telemetry: CompletionTelemetry,
    },
}

/// Initialise a rotation intent: start the IMU, zero the motors and build the
/// model intent.
pub(super) async fn init(
    degrees: f32,
    direction: RotationDirection,
    motion: RotationMotion,
    completion_requested: bool,
) -> super::state::ActiveIntent {
    lifecycle::start_rotation_imu();
    clear_imu_measurements();

    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: 0,
        right_speed: 0,
    })
    .await;
    motion::set_track_speeds(0, 0).await;

    let started_at_ms = Instant::now().as_millis();
    let model = Intent::new(
        IntentParams::RotateExact {
            degrees,
            direction,
            motion,
        },
        started_at_ms,
    );

    super::state::ActiveIntent::RotateExact {
        model,
        completion_requested,
        started_at_ms,
        direction,
        motion,
    }
}

/// Run the model until it finishes, fails or an interrupt preempts a wait.
pub(super) async fn run_rotation_poll(
    model: &mut Intent,
    started_at_ms: u64,
    direction: RotationDirection,
    motion: RotationMotion,
) -> RotationPoll {
    let mut last_measurement: Option<ImuMeasurement> = None;
    // The intent's start action must not consume a sample that arrives before the
    // first control tick; the host contract starts with an empty observation.
    let mut observation = Observation {
        now_ms: Instant::now().as_millis(),
        encoder: None,
        yaw: None,
    };
    let mut non_blocking_actions: u8 = 0;
    let mut first_sample_logged = false;

    loop {
        let step = model.step(&observation);

        if !first_sample_logged && let Some(target_yaw) = model.rotation_target_yaw() {
            first_sample_logged = true;
            defmt::info!(
                "rotate_exact start: yaw={=f32}° target_yaw={=f32}°",
                observation.yaw.map_or(0.0, |sample| sample.yaw_deg),
                target_yaw,
            );
        }

        #[cfg(feature = "telemetry_logs")]
        if let Some(trace) = step.trace
            && observation.now_ms % 100 < 25
        {
            defmt::info!(
                "rotate_exact trace: yaw={=?} target={=?} remaining={=f32} l={=?} r={=?} deadline={=?}",
                trace.yaw_deg,
                trace.target_yaw_deg,
                trace.remaining_deg,
                trace.left,
                trace.right,
                trace.deadline_ms,
            );
        }

        match step.action {
            IntentAction::SetTracks { left, right } => {
                send_tracks(left, right).await;
                non_blocking_actions = non_blocking_actions.saturating_add(1);
                if non_blocking_actions > MAX_ACTIONS_PER_OBSERVATION {
                    return RotationPoll::Step(action_loop_failure(model));
                }
            }
            IntentAction::BrakeAll => {
                motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
                motion::set_track_speeds(0, 0).await;
                non_blocking_actions = non_blocking_actions.saturating_add(1);
                if non_blocking_actions > MAX_ACTIONS_PER_OBSERVATION {
                    return RotationPoll::Step(action_loop_failure(model));
                }
            }
            IntentAction::CoastAll => {
                motor_driver::send_motor_command(MotorCommand::CoastAll).await;
                motion::set_track_speeds(0, 0).await;
                non_blocking_actions = non_blocking_actions.saturating_add(1);
                if non_blocking_actions > MAX_ACTIONS_PER_OBSERVATION {
                    return RotationPoll::Step(action_loop_failure(model));
                }
            }
            IntentAction::WaitUntil { deadline_ms } => {
                non_blocking_actions = 0;
                let wait_ms = deadline_ms.saturating_sub(Instant::now().as_millis());
                match select(DRIVE_INTERRUPT.wait(), Timer::after(Duration::from_millis(wait_ms))).await {
                    Either::First(kind) => return RotationPoll::Interrupt(kind),
                    Either::Second(()) => {}
                }
                observation = build_observation(started_at_ms, &mut last_measurement);
            }
            IntentAction::Finished => {
                let Some(outcome) = model.outcome() else {
                    return RotationPoll::Step(RotationStepResult::Failed {
                        reason: DriveFault::RotationTimeout.label(),
                        telemetry: model.cancellation_telemetry(Instant::now().as_millis()),
                    });
                };
                let result = finish_rotation(model, outcome, motion, direction, started_at_ms, last_measurement).await;
                return RotationPoll::Step(result);
            }
        }
    }
}

/// Build an observation from the adapter clock and the newest IMU sample.
///
/// Rotation does not start the encoder task, so `encoder` is always absent here;
/// the model ignores it for rotation. `last_measurement` keeps the raw sample
/// so the correction phase can anchor on the turn's final sample.
fn build_observation(started_at_ms: u64, last_measurement: &mut Option<ImuMeasurement>) -> Observation {
    let now_ms = Instant::now().as_millis();
    let sample = drain_latest_imu_since(started_at_ms);
    if let Some(measurement) = sample {
        *last_measurement = Some(measurement);
    }

    Observation {
        now_ms,
        encoder: None,
        yaw: sample.map(|measurement| YawSample {
            yaw_deg: measurement.orientation.yaw,
            timestamp_ms: measurement.timestamp_ms,
        }),
    }
}

/// Map the model's terminal outcome onto the firmware's rotation result.
///
/// A stationary turn runs the firmware-owned correction phase after the model
/// reports `Finished`; a while-moving turn keeps the model's turn-stop
/// telemetry. Failures keep the model's telemetry and fault label.
async fn finish_rotation(
    model: &Intent,
    outcome: DriveOutcome,
    motion: RotationMotion,
    direction: RotationDirection,
    started_at_ms: u64,
    last_measurement: Option<ImuMeasurement>,
) -> RotationStepResult {
    match outcome {
        DriveOutcome::Completed { telemetry } => {
            if matches!(motion, RotationMotion::Stationary { .. })
                && let Some(initial_measurement) = last_measurement
                && let Some(target_yaw) = model.rotation_target_yaw()
            {
                defmt::info!(
                    "rotate_exact correction start: flips={=u8} target_yaw={=f32}°",
                    model.rotation_flips(),
                    target_yaw,
                );
                let mut correction = CorrectionState::new(target_yaw, direction);
                return run_correction_phase(&mut correction, started_at_ms, initial_measurement).await;
            }
            RotationStepResult::Completed { telemetry }
        }
        DriveOutcome::Failed { fault, telemetry } => RotationStepResult::Failed {
            reason: fault.label(),
            telemetry,
        },
    }
}

/// A terminal failure for the adapter's non-blocking action guard.
fn action_loop_failure(model: &Intent) -> RotationStepResult {
    defmt::error!(
        "rotate_exact: more than {} non-blocking actions per observation",
        MAX_ACTIONS_PER_OBSERVATION
    );
    RotationStepResult::Failed {
        reason: ACTION_LOOP_LABEL,
        telemetry: model.cancellation_telemetry(Instant::now().as_millis()),
    }
}

/// Send a calibrated track command and publish the logical speeds.
async fn send_tracks(left: i8, right: i8) {
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: left,
        right_speed: right,
    })
    .await;
    motion::set_track_speeds(left, right).await;
}

// ── Provisional post-stop correction phase (ticket 04 moves this) ─────────────

/// Acceptable angle error in degrees.
const TOLERANCE_DEG: f32 = 0.5;

/// Maximum correction iterations after the initial turn stops.
const CORRECTION_MAX_ITERATIONS: u8 = 15;

/// Settle wait after each correction pulse (ms).
const CORRECTION_SETTLE_MS: u64 = 400;

/// Starting speed for the first correction pulse.
const CORRECTION_SPEED_START: u8 = 35;

/// Speed reduction per correction iteration.
const CORRECTION_SPEED_STEP: u8 = 2;

/// Minimum speed floor for correction pulses.
const CORRECTION_SPEED_MIN: u8 = 25;

/// Maximum time for a single correction iteration before giving up and moving on (ms).
const CORRECTION_ITER_TIMEOUT_MS: u64 = 400;

/// The correction phase's angle state.
///
/// It carries only what the correction policy reads: the model's anchored target
/// yaw, the newest yaw sample and the commanded direction.
struct CorrectionState {
    /// Absolute target yaw in degrees, anchored by the model on its first sample.
    target_yaw: f32,
    /// Last measured yaw angle in degrees.
    last_yaw: Option<f32>,
    /// Direction of the original turn.
    direction: RotationDirection,
}

impl CorrectionState {
    /// Create a correction state from the model's anchored target and direction.
    const fn new(target_yaw: f32, direction: RotationDirection) -> Self {
        Self {
            target_yaw,
            last_yaw: None,
            direction,
        }
    }

    /// Record the newest yaw sample.
    const fn update(&mut self, measurement: &ImuMeasurement) {
        self.last_yaw = Some(measurement.orientation.yaw);
    }

    /// Remaining signed angle to target (degrees).
    ///
    /// Positive  → still needs to turn in the commanded direction (undershoot).
    /// Negative  → has gone past the target (overshoot).
    fn remaining(&self) -> f32 {
        let Some(current_yaw) = self.last_yaw else {
            return 0.0;
        };
        let diff = normalize_angle(current_yaw - self.target_yaw);
        match self.direction {
            RotationDirection::Clockwise => diff,
            RotationDirection::CounterClockwise => -diff,
        }
    }
}

/// Normalize an angle in degrees to the range (−180, +180].
fn normalize_angle(deg: f32) -> f32 {
    let d = deg % 360.0;
    if d > 180.0 {
        d - 360.0
    } else if d < -180.0 {
        d + 360.0
    } else {
        d
    }
}

/// Stop rotation motors and update the motion state.
async fn stop_rotation_motors() {
    send_tracks(0, 0).await;
}

/// Build a success result using the final IMU measurement.
fn rotation_success(
    measurement: &ImuMeasurement,
    correction: &CorrectionState,
    started_at_ms: u64,
) -> RotationStepResult {
    let duration_ms = Instant::now().as_millis() - started_at_ms;

    RotationStepResult::Completed {
        telemetry: CompletionTelemetry::RotateExact {
            final_yaw_deg: measurement.orientation.yaw,
            angle_error_deg: -correction.remaining(),
            duration_ms,
        },
    }
}

/// Drain the IMU feedback channel and return the newest sample since rotation start.
fn drain_latest_imu_since(started_at_ms: u64) -> Option<ImuMeasurement> {
    let mut latest: Option<ImuMeasurement> = None;
    while let Ok(m) = IMU_FEEDBACK_CHANNEL.receiver().try_receive() {
        if m.timestamp_ms >= started_at_ms {
            latest = Some(m);
        }
    }
    latest
}

/// Wait for a fresh IMU sample, then return the newest available sample.
async fn wait_for_latest_imu_since(started_at_ms: u64, wait_ms: u64) -> Option<ImuMeasurement> {
    if let Some(latest) = drain_latest_imu_since(started_at_ms) {
        return Some(latest);
    }

    Timer::after(Duration::from_millis(wait_ms)).await;
    drain_latest_imu_since(started_at_ms)
}

/// Compute track speeds for a single correction pulse using the torque ladder.
///
/// Base speed scales proportionally with error magnitude (~6 speed-units per
/// degree), clamped between [`CORRECTION_SPEED_MIN`] and
/// [`CORRECTION_SPEED_START`], then stepped down by [`CORRECTION_SPEED_STEP`]
/// for each prior iteration.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn compute_correction_speeds(error_deg: f32, iteration: u8, direction: RotationDirection) -> (i8, i8) {
    let proportional =
        (roundf(error_deg.abs() * 6.0).clamp(0.0, 255.0) as u8).clamp(CORRECTION_SPEED_MIN, CORRECTION_SPEED_START);

    let correction_speed = proportional
        .saturating_sub(iteration.saturating_mul(CORRECTION_SPEED_STEP))
        .max(CORRECTION_SPEED_MIN);

    let effective_direction = if error_deg > 0.0 {
        direction
    } else {
        match direction {
            RotationDirection::Clockwise => RotationDirection::CounterClockwise,
            RotationDirection::CounterClockwise => RotationDirection::Clockwise,
        }
    };

    let speed = i8::try_from(correction_speed).unwrap_or(i8::MAX);
    match effective_direction {
        RotationDirection::Clockwise => (speed, -speed),
        RotationDirection::CounterClockwise => (-speed, speed),
    }
}

/// Check whether the remaining angle error is within tolerance.
///
/// Returns `Some(RotationStepResult::Completed)` if converged, `None` otherwise.
fn try_correction_convergence(
    correction: &CorrectionState,
    measurement: &ImuMeasurement,
    iteration: u8,
    started_at_ms: u64,
) -> Option<RotationStepResult> {
    let error_deg = correction.remaining();
    if error_deg.abs() <= TOLERANCE_DEG {
        let yaw = correction.last_yaw.unwrap_or(measurement.orientation.yaw);
        defmt::info!(
            "correction converged iter={=u8} yaw={=f32}° err={=f32}°",
            iteration,
            yaw,
            error_deg,
        );
        Some(rotation_success(measurement, correction, started_at_ms))
    } else {
        None
    }
}

/// Outcome of a single correction IMU polling cycle.
enum PollOutcome {
    /// Converged to the target within tolerance.
    Converged(RotationStepResult),
    /// Overshot the target; contains the updated overshoot count.
    Overshot(u8),
    /// Timed out without reaching the target.
    TimedOut,
}

/// Poll the IMU while correction motors run.
///
/// Stops on convergence (within [`TOLERANCE_DEG`]), overshoot (error sign
/// flips relative to `iter_sign`), or a per-iteration timeout
/// ([`CORRECTION_ITER_TIMEOUT_MS`]).
async fn poll_correction_imu(
    correction: &mut CorrectionState,
    latest_measurement: &mut ImuMeasurement,
    started_at_ms: u64,
    iteration: u8,
    iter_sign: i8,
    overshoot_count: u8,
) -> PollOutcome {
    let start_ms = Instant::now().as_millis();

    loop {
        if Instant::now().as_millis() - start_ms > CORRECTION_ITER_TIMEOUT_MS {
            stop_rotation_motors().await;
            defmt::info!(
                "correction iter={=u8} timeout yaw={=f32}° err={=f32}°",
                iteration,
                correction.last_yaw.unwrap_or(latest_measurement.orientation.yaw),
                correction.remaining(),
            );
            return PollOutcome::TimedOut;
        }

        if let Some(m) = drain_latest_imu_since(started_at_ms) {
            correction.update(&m);
            *latest_measurement = m;
            let new_error = correction.remaining();

            if new_error.abs() <= TOLERANCE_DEG {
                stop_rotation_motors().await;
                return PollOutcome::Converged(rotation_success(latest_measurement, correction, started_at_ms));
            }

            let new_sign: i8 = if new_error > 0.0 { 1 } else { -1 };
            if new_sign != iter_sign {
                stop_rotation_motors().await;
                let count = overshoot_count.saturating_add(1);
                defmt::info!(
                    "correction overshoot iter={=u8} yaw={=f32}° err={=f32}° total_overshoots={=u8}",
                    iteration,
                    correction.last_yaw.unwrap_or(latest_measurement.orientation.yaw),
                    new_error,
                    count,
                );
                return PollOutcome::Overshot(count);
            }
        } else {
            Timer::after(Duration::from_millis(5)).await;
        }
    }
}

/// Post-stop multi-iteration correction loop for stationary turns.
///
/// Uses a torque-ladder strategy: motor speed is stepped down by
/// [`CORRECTION_SPEED_STEP`] each iteration, starting at
/// [`CORRECTION_SPEED_START`] and floored at [`CORRECTION_SPEED_MIN`]. In each
/// iteration the motors run while the IMU is polled continuously until the yaw
/// converges, an overshoot is detected, or a timeout expires.
///
/// The high starting speed reliably breaks static friction; subsequent
/// iterations trade torque for precision as the remaining error shrinks.
///
/// Up to [`CORRECTION_MAX_ITERATIONS`] attempts are made before the correction
/// is declared exhausted.
async fn run_correction_phase(
    correction: &mut CorrectionState,
    started_at_ms: u64,
    initial_measurement: ImuMeasurement,
) -> RotationStepResult {
    // Anchor state on the pre-stop measurement.
    correction.update(&initial_measurement);

    // Wait for initial settle and capture a fresh IMU sample.
    let mut latest_measurement = wait_for_latest_imu_since(started_at_ms, CORRECTION_SETTLE_MS)
        .await
        .map_or(initial_measurement, |settled| {
            correction.update(&settled);
            settled
        });

    let mut overshoot_count: u8 = 0;
    let mut last_error_sign: i8 = 0;
    let mut iteration: u8 = 0;

    while iteration < CORRECTION_MAX_ITERATIONS {
        // Check convergence at loop entry (from coast between iterations).
        if let Some(result) = try_correction_convergence(correction, &latest_measurement, iteration, started_at_ms) {
            return result;
        }

        let error_deg = correction.remaining();

        // Track overshoots between iterations.
        let current_sign: i8 = if error_deg > 0.0 { 1 } else { -1 };
        if last_error_sign != 0 && current_sign != last_error_sign {
            overshoot_count = overshoot_count.saturating_add(1);
        }
        last_error_sign = current_sign;

        // Compute and apply correction speeds.
        let (left_speed, right_speed) = compute_correction_speeds(error_deg, iteration, correction.direction);

        defmt::info!(
            "correction iter={=u8} yaw={=f32}° err={=f32}° l={=i8} r={=i8}",
            iteration,
            correction.last_yaw.unwrap_or(latest_measurement.orientation.yaw),
            error_deg,
            left_speed,
            right_speed,
        );

        send_tracks(left_speed, right_speed).await;

        // Poll IMU while motors run.
        match poll_correction_imu(
            correction,
            &mut latest_measurement,
            started_at_ms,
            iteration,
            current_sign,
            overshoot_count,
        )
        .await
        {
            PollOutcome::Converged(result) => return result,
            PollOutcome::Overshot(count) => overshoot_count = count,
            PollOutcome::TimedOut => {}
        }

        // After stopping: settle and re-check for coast landing in tolerance.
        if let Some(settled) = wait_for_latest_imu_since(started_at_ms, CORRECTION_SETTLE_MS).await {
            correction.update(&settled);
            latest_measurement = settled;
        }

        if let Some(result) = try_correction_convergence(correction, &latest_measurement, iteration, started_at_ms) {
            return result;
        }

        iteration = iteration.saturating_add(1);
    }

    // Exhausted all correction iterations without reaching tolerance.
    let error_deg = correction.remaining();
    defmt::warn!(
        "correction exhausted after {=u8} iters, final_err={=f32}°",
        iteration,
        error_deg,
    );
    RotationStepResult::Failed {
        reason: "CorrectionExhausted",
        telemetry: CompletionTelemetry::RotateExact {
            final_yaw_deg: latest_measurement.orientation.yaw,
            angle_error_deg: -correction.remaining(),
            duration_ms: Instant::now().as_millis() - started_at_ms,
        },
    }
}
