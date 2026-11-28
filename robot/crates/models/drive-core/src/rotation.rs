//! The rotation control law.
//!
//! This module owns the `RotateExact` intent: first-sample yaw anchoring, the
//! absolute target, the remaining-sign handling, ±180° normalisation, the
//! stationary and while-moving speed laws, ramp-down near the threshold,
//! tolerance completion, overshoot reversal, the flip limit, the overall
//! timeout and the turn's completion telemetry. A stationary turn that reaches
//! tolerance hands over to the provisional post-stop correction phase in
//! [`crate::correction`], which owns the remaining policy and telemetry.
//!
//! It is a pure step machine. It never reads a clock, starts a sensor,
//! commands a motor or allocates: every timestamp comes from the
//! [`Observation`], and every action is returned to the adapter to execute.
//! A sleep becomes [`IntentAction::WaitUntil`].

#[cfg(feature = "trace")]
use crate::Trace;
use crate::{
    CompletionTelemetry, DriveFault, DriveOutcome, IntentAction, Observation, RotationDirection, RotationMotion, Step,
    YawSample, correction::Correction,
};

// ── Rotation control constants ───────────────────────────────────────────────

/// Maximum rotation speed (0-100%).
const SPEED_MAX: u8 = 100;
/// Minimum rotation speed to overcome friction.
const SPEED_MIN: u8 = 45;
/// Acceptable angle error in degrees.
const TOLERANCE_DEG: f32 = 0.5;
/// Overshoot deadband before applying corrective reverse (degrees).
const CORRECTION_DEADBAND_DEG: f32 = 0.5;
/// Maximum time allowed for overshoot correction before bailing (milliseconds).
const CORRECTION_TIMEOUT_MS: u64 = 4_000;
/// Rotation ramp-down begins when remaining degrees drop below this value.
const RAMP_DOWN_START_DEG: f32 = 60.0;
/// Maximum number of corrective direction flips allowed during rotation.
const CORRECTION_MAX_FLIPS: u8 = 12;
/// Safety timeout for `RotateExact` (milliseconds).
const TIMEOUT_MS: u64 = 8_000;
/// Maximum time to wait for a fresh IMU sample per tick (milliseconds).
const IMU_WAIT_TIMEOUT_MS: u64 = 20;
/// Rotation control loop tick interval (ms). Must align with the IMU sampling rate.
const ROTATION_CONTROL_INTERVAL_MS: u64 = 20;
/// Maximum speed differential during combined motion.
const SPEED_DIFF_MAX: i8 = 30;

// ── Pure state / maths ────────────────────────────────────────────────────────

/// Normalize an angle in degrees to the range (−180, +180].
pub const fn normalize_angle(deg: f32) -> f32 {
    let d = deg % 360.0;
    if d > 180.0 {
        d - 360.0
    } else if d < -180.0 {
        d + 360.0
    } else {
        d
    }
}

/// Clamp a float speed value to the valid `u8` range 0–100.
#[allow(clippy::cast_possible_truncation)]
fn clamp_speed_u8(value: f32) -> u8 {
    let clamped = value.clamp(0.0, 100.0);
    if clamped.is_nan() {
        0
    } else {
        let truncated = clamped as i32;
        u8::try_from(truncated).unwrap_or(0)
    }
}

/// Where the step machine is between adapter actions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// The initial zero-speed command has not been emitted yet.
    Start,
    /// Waiting for the next control tick at `deadline_ms`.
    Waiting {
        /// Absolute time at which the next control tick may begin.
        deadline_ms: u64,
    },
    /// A tick began at `tick_started_ms` with no sample; the adapter is serving
    /// the per-tick IMU sample wait and the next step performs the retry drain.
    RetrySample {
        /// Absolute time at which the tick began, before the sample wait.
        tick_started_ms: u64,
    },
    /// The stop command has been emitted; the terminal `Finished` follows.
    Stopping,
    /// The turn reached tolerance; the stationary correction phase is running.
    Correction,
    /// The terminal `Finished` action has been emitted.
    Finished,
}

/// Private state of a `RotateExact` intent.
pub struct RotationState {
    /// Target rotation angle in degrees.
    target_angle: f32,
    /// Last measured yaw angle (in degrees).
    last_yaw: Option<f32>,
    /// Absolute yaw at the start of the maneuver (degrees), set on the first sample.
    start_yaw: Option<f32>,
    /// Absolute target yaw (degrees), anchored on the first sample.
    target_yaw: Option<f32>,
    /// Direction of rotation.
    direction: RotationDirection,
    /// Type of motion during rotation.
    motion: RotationMotion,
    /// Base forward/reverse speed when rotating while moving.
    base_speed: i8,
    /// Count of corrective direction flips detected.
    correction_flips: u8,
    /// Timestamp when overshoot correction started (ms since boot).
    correction_started_at_ms: Option<u64>,
    /// Last signed error direction for flip detection.
    last_error_sign: i8,
    /// Absolute time the intent started, from `Intent::new`.
    started_at_ms: u64,
    /// Where the step machine is between adapter actions.
    phase: Phase,
    /// The provisional post-stop correction, present once a stationary turn
    /// has reached tolerance and stopped.
    correction: Option<Correction>,
    /// The terminal outcome, recorded when the stop command is emitted and
    /// published when `Finished` is emitted.
    pending_outcome: Option<DriveOutcome>,
    /// The terminal outcome, present only once `Finished` has been emitted.
    outcome: Option<DriveOutcome>,
}

impl RotationState {
    /// Create a rotation state for an intent that started at `started_at_ms`.
    pub const fn new(
        target_angle: f32,
        direction: RotationDirection,
        motion: RotationMotion,
        started_at_ms: u64,
    ) -> Self {
        let base_speed = match motion {
            RotationMotion::Stationary { speed: _ } => 0,
            RotationMotion::WhileMoving(speed) => speed,
        };

        Self {
            target_angle,
            last_yaw: None,
            start_yaw: None,
            target_yaw: None,
            direction,
            motion,
            base_speed,
            correction_flips: 0,
            correction_started_at_ms: None,
            last_error_sign: 0,
            started_at_ms,
            phase: Phase::Start,
            correction: None,
            pending_outcome: None,
            outcome: None,
        }
    }

    /// Advance the intent one step and return the action the adapter executes.
    pub fn step(&mut self, obs: &Observation) -> Step {
        match self.phase {
            Phase::Start => {
                self.phase = Phase::Waiting {
                    deadline_ms: self.started_at_ms.saturating_add(ROTATION_CONTROL_INTERVAL_MS),
                };
                self.make_step(IntentAction::SetTracks { left: 0, right: 0 })
            }
            Phase::Waiting { deadline_ms } => {
                if obs.now_ms < deadline_ms {
                    self.make_step(IntentAction::WaitUntil { deadline_ms })
                } else {
                    self.begin_tick(obs)
                }
            }
            Phase::RetrySample { tick_started_ms } => self.retry_sample(obs, tick_started_ms),
            Phase::Correction => self.correction_step(obs),
            Phase::Stopping => {
                self.outcome = self.pending_outcome.take();
                self.phase = Phase::Finished;
                self.make_step(IntentAction::Finished)
            }
            Phase::Finished => self.make_step(IntentAction::Finished),
        }
    }

    /// The terminal outcome, once the `Finished` action has been emitted.
    pub fn outcome(&self) -> Option<DriveOutcome> {
        self.outcome.clone()
    }

    /// Telemetry for an intent that the adapter cancels at `now_ms`.
    pub fn cancellation_telemetry(&self, now_ms: u64) -> CompletionTelemetry {
        CompletionTelemetry::RotateExact {
            final_yaw_deg: self.current_yaw().unwrap_or(0.0),
            angle_error_deg: -self.current_remaining(),
            duration_ms: now_ms.saturating_sub(self.started_at_ms),
        }
    }

    /// Begin a control tick at `obs.now_ms`: check the overall timeout, then take
    /// a fresh sample or wait for one.
    fn begin_tick(&mut self, obs: &Observation) -> Step {
        let now_ms = obs.now_ms;
        if now_ms.saturating_sub(self.started_at_ms) > TIMEOUT_MS {
            return self.fail(DriveFault::RotationTimeout, now_ms);
        }

        if let Some(sample) = self.fresh_sample(obs) {
            self.process(sample, now_ms, now_ms)
        } else {
            self.phase = Phase::RetrySample {
                tick_started_ms: now_ms,
            };
            self.make_step(IntentAction::WaitUntil {
                deadline_ms: now_ms.saturating_add(IMU_WAIT_TIMEOUT_MS),
            })
        }
    }

    /// Retry the sample drain after the per-tick IMU wait.
    fn retry_sample(&mut self, obs: &Observation, tick_started_ms: u64) -> Step {
        if let Some(sample) = self.fresh_sample(obs) {
            self.process(sample, tick_started_ms, obs.now_ms)
        } else {
            let deadline_ms = obs.now_ms.saturating_add(ROTATION_CONTROL_INTERVAL_MS);
            self.phase = Phase::Waiting { deadline_ms };
            self.make_step(IntentAction::WaitUntil { deadline_ms })
        }
    }

    /// The newest yaw sample that is not older than the intent start.
    fn fresh_sample(&self, obs: &Observation) -> Option<YawSample> {
        obs.yaw.filter(|sample| sample.timestamp_ms >= self.started_at_ms)
    }

    /// Process one control sample at `now_ms`; `tick_started_ms` is the tick's
    /// start, which the flip-limit and correction-timeout checks measure from.
    fn process(&mut self, sample: YawSample, tick_started_ms: u64, now_ms: u64) -> Step {
        let done = self.update(&sample);

        if done {
            return self.complete(&sample, now_ms);
        }

        if self.correction_flips >= CORRECTION_MAX_FLIPS {
            return self.fail(DriveFault::CorrectionLimit, tick_started_ms);
        }

        if let Some(start_ms) = self.correction_started_at_ms
            && tick_started_ms.saturating_sub(start_ms) > CORRECTION_TIMEOUT_MS
        {
            return self.fail(DriveFault::CorrectionTimeout, tick_started_ms);
        }

        let (left, right) = self.calculate_motor_speeds();
        self.phase = Phase::Waiting {
            deadline_ms: now_ms.saturating_add(ROTATION_CONTROL_INTERVAL_MS),
        };
        self.make_step(IntentAction::SetTracks { left, right })
    }

    /// Record a successful turn and emit the stop command.
    ///
    /// A while-moving turn publishes its outcome and finishes. A stationary turn
    /// hands over to the correction phase, which owns the terminal telemetry
    /// from here on.
    fn complete(&mut self, sample: &YawSample, now_ms: u64) -> Step {
        if matches!(self.motion, RotationMotion::Stationary { .. }) {
            self.correction = Some(Correction::new(
                self.target_yaw.unwrap_or(sample.yaw_deg),
                self.direction,
                sample.yaw_deg,
                sample.timestamp_ms,
                self.started_at_ms,
                now_ms,
            ));
            self.phase = Phase::Correction;
            return self.make_step(IntentAction::SetTracks { left: 0, right: 0 });
        }

        self.pending_outcome = Some(DriveOutcome::Completed {
            telemetry: CompletionTelemetry::RotateExact {
                final_yaw_deg: sample.yaw_deg,
                angle_error_deg: -self.remaining(),
                duration_ms: now_ms.saturating_sub(self.started_at_ms),
            },
        });
        self.phase = Phase::Stopping;
        self.make_step(IntentAction::SetTracks { left: 0, right: 0 })
    }

    /// Advance the correction phase and publish its terminal outcome.
    fn correction_step(&mut self, obs: &Observation) -> Step {
        let action = match self.correction.as_mut() {
            Some(correction) => correction.step(obs),
            None => return self.fail(DriveFault::RotationTimeout, obs.now_ms),
        };

        if matches!(action, IntentAction::Finished) {
            self.outcome = self.correction.as_mut().and_then(Correction::take_outcome);
            self.phase = Phase::Finished;
        }

        self.make_step(action)
    }

    /// Record a terminal failure at `now_ms` and emit the stop command.
    fn fail(&mut self, fault: DriveFault, now_ms: u64) -> Step {
        self.pending_outcome = Some(DriveOutcome::Failed {
            fault,
            telemetry: CompletionTelemetry::RotateExact {
                final_yaw_deg: self.last_yaw.unwrap_or(0.0),
                angle_error_deg: -self.remaining(),
                duration_ms: now_ms.saturating_sub(self.started_at_ms),
            },
        });
        self.phase = Phase::Stopping;
        self.make_step(IntentAction::SetTracks { left: 0, right: 0 })
    }

    /// Build a step, attaching a trace snapshot only when the `trace` feature is on.
    #[allow(clippy::unused_self, clippy::missing_const_for_fn)]
    fn make_step(&self, action: IntentAction) -> Step {
        #[cfg(feature = "trace")]
        {
            Step {
                action,
                trace: Some(self.trace(action)),
            }
        }
        #[cfg(not(feature = "trace"))]
        {
            Step { action, trace: None }
        }
    }

    /// A snapshot of the decision behind `action`.
    #[cfg(feature = "trace")]
    fn trace(&self, action: IntentAction) -> Trace {
        let (left, right) = match action {
            IntentAction::SetTracks { left, right } => (Some(left), Some(right)),
            _ => (None, None),
        };
        let deadline_ms = match action {
            IntentAction::WaitUntil { deadline_ms } => Some(deadline_ms),
            _ => None,
        };

        Trace {
            yaw_deg: self.current_yaw(),
            target_yaw_deg: self.target_yaw,
            remaining_deg: self.current_remaining(),
            left,
            right,
            deadline_ms,
        }
    }

    /// The newest yaw: the turn's until it hands over, the correction's after.
    fn current_yaw(&self) -> Option<f32> {
        self.correction
            .as_ref()
            .map_or(self.last_yaw, |correction| Some(correction.last_yaw()))
    }

    /// The live remaining error, including during the correction phase.
    fn current_remaining(&self) -> f32 {
        self.correction
            .as_ref()
            .map_or_else(|| self.remaining(), Correction::remaining_deg)
    }

    /// Updates rotation state with a new yaw sample.
    ///
    /// Returns `true` if the target angle has been reached within tolerance.
    fn update(&mut self, sample: &YawSample) -> bool {
        let current_yaw = sample.yaw_deg;

        // On the very first sample: anchor start_yaw and compute the absolute target.
        if self.start_yaw.is_none() {
            self.start_yaw = Some(current_yaw);
            self.target_yaw = Some(match self.direction {
                RotationDirection::Clockwise => normalize_angle(current_yaw - self.target_angle),
                RotationDirection::CounterClockwise => normalize_angle(current_yaw + self.target_angle),
            });
        }

        self.last_yaw = Some(current_yaw);

        let remaining = self.remaining();
        let error_sign: i8 = if remaining >= 0.0 { 1 } else { -1 };
        if remaining.abs() > CORRECTION_DEADBAND_DEG {
            if self.last_error_sign != 0 && error_sign != self.last_error_sign {
                self.correction_flips = self.correction_flips.saturating_add(1);
                if self.correction_started_at_ms.is_none() {
                    self.correction_started_at_ms = Some(sample.timestamp_ms);
                }
            }
            self.last_error_sign = error_sign;
        }

        remaining.abs() <= TOLERANCE_DEG
    }

    /// Remaining signed angle to target (degrees).
    ///
    /// Positive  → still needs to turn in the commanded direction (undershoot).
    /// Negative  → has gone past the target (overshoot).
    /// Returns `target_angle` when no IMU sample has been received yet.
    fn remaining(&self) -> f32 {
        let Some(current_yaw) = self.last_yaw else {
            return self.target_angle;
        };
        let Some(target_yaw) = self.target_yaw else {
            return self.target_angle;
        };
        let diff = normalize_angle(current_yaw - target_yaw);
        match self.direction {
            RotationDirection::Clockwise => diff,
            RotationDirection::CounterClockwise => -diff,
        }
    }

    /// Calculates appropriate motor speeds for the current rotation state.
    ///
    /// Returns `(left_speed, right_speed)`.
    fn calculate_motor_speeds(&self) -> (i8, i8) {
        // Signed error: positive => undershoot, negative => overshoot.
        let error_deg = self.remaining();
        let remaining_degrees = error_deg.abs();

        // If we overshot beyond the deadband, reverse effective direction to hunt back toward the setpoint.
        let effective_direction = if error_deg < -CORRECTION_DEADBAND_DEG {
            match self.direction {
                RotationDirection::Clockwise => RotationDirection::CounterClockwise,
                RotationDirection::CounterClockwise => RotationDirection::Clockwise,
            }
        } else {
            self.direction
        };

        let rotation_speed = match self.motion {
            RotationMotion::Stationary { speed } => {
                // Keep a floor of SPEED_MIN to overcome static friction, but
                // still taper near the target to limit overshoot.
                let requested = speed.clamp(0, 100);
                if remaining_degrees < RAMP_DOWN_START_DEG {
                    let min = SPEED_MIN;
                    let max = requested.max(min);
                    let speed_range = max - min;
                    let speed_factor = remaining_degrees / RAMP_DOWN_START_DEG;
                    clamp_speed_u8(f32::from(min) + (f32::from(speed_range) * speed_factor))
                } else {
                    requested
                }
            }
            RotationMotion::WhileMoving(_) => {
                if remaining_degrees < RAMP_DOWN_START_DEG {
                    let speed_range = SPEED_MAX - SPEED_MIN;
                    let speed_factor = remaining_degrees / RAMP_DOWN_START_DEG;
                    clamp_speed_u8(f32::from(SPEED_MIN) + (f32::from(speed_range) * speed_factor))
                } else {
                    SPEED_MAX
                }
            }
        };
        let rotation_speed_signed = i8::try_from(rotation_speed).unwrap_or(i8::MAX);

        match self.motion {
            RotationMotion::Stationary { speed: _ } => match effective_direction {
                RotationDirection::Clockwise => (rotation_speed_signed, -rotation_speed_signed),
                RotationDirection::CounterClockwise => (-rotation_speed_signed, rotation_speed_signed),
            },
            RotationMotion::WhileMoving(_) => {
                let rotation_diff = rotation_speed_signed.min(SPEED_DIFF_MAX);
                match effective_direction {
                    RotationDirection::Clockwise => {
                        (self.base_speed, (self.base_speed - rotation_diff).clamp(-100, 100))
                    }
                    RotationDirection::CounterClockwise => {
                        ((self.base_speed - rotation_diff).clamp(-100, 100), self.base_speed)
                    }
                }
            }
        }
    }
}
