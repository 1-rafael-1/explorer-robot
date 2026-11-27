//! The provisional post-stop rotation correction policy.
//!
//! This is the firmware drive task's correction phase, extracted as an explicit
//! phase machine: settle after the turn stops, a torque-ladder pulse, a
//! post-pulse settle, then the next iteration or a terminal result. It runs only
//! for a stationary turn that reached the target tolerance; a while-moving turn
//! never enters it.
//!
//! The module is pure. It reads only [`Observation::now_ms`] and
//! [`Observation::yaw`], never reads a clock, starts a sensor, commands a motor
//! or allocates, and it returns [`IntentAction`]s for the adapter to execute. A
//! firmware sleep becomes [`IntentAction::WaitUntil`].
//!
//! The policy is provisional: the physical result is recorded on the bench, not
//! gated. The transitions, the torque ladder, the overshoot accounting, the
//! per-iteration timeout and the exhaustion path are pinned by host tests.
//!
//! # Policy
//!
//! 1. **Settle** — after the turn stop, wait up to [`SETTLE_MS`] for a fresh
//!    yaw sample. A sample that is already available ends the wait immediately;
//!    otherwise the previous sample is kept, exactly like the firmware's
//!    drain-first wait.
//! 2. **Iteration entry** — converge if `|error| <= TOLERANCE_DEG`. Otherwise
//!    count an overshoot when the error sign flipped since the previous
//!    iteration, and emit the next pulse.
//! 3. **Pulse** — a pulse at the torque-ladder speed whose sign follows the
//!    error. While it runs, a fresh sample either converges, flips the error
//!    sign (an overshoot) or neither; a pulse older than [`ITER_TIMEOUT_MS`]
//!    stops and moves on.
//! 4. **Post-pulse settle** — wait up to [`SETTLE_MS`] for the coast to land,
//!    then converge or start the next iteration. After [`MAX_ITERATIONS`]
//!    pulses without convergence the correction is exhausted.

use libm::roundf;

use crate::{
    CompletionTelemetry, DriveFault, DriveOutcome, IntentAction, Observation, RotationDirection,
    rotation::normalize_angle,
};

// ── Correction policy constants (carried verbatim from the firmware) ─────────

/// Acceptable angle error in degrees.
const TOLERANCE_DEG: f32 = 0.5;
/// Maximum correction iterations (pulses) after the initial turn stops.
const MAX_ITERATIONS: u8 = 15;
/// Settle wait after the turn stops and after each pulse (ms).
const SETTLE_MS: u64 = 400;
/// Starting speed for the first pulse (0-100 %).
const SPEED_START: u8 = 35;
/// Speed reduction per iteration.
const SPEED_STEP: u8 = 2;
/// Minimum speed floor for a pulse (0-100 %).
const SPEED_MIN: u8 = 25;
/// Maximum time a single pulse may run before it stops and moves on (ms).
const ITER_TIMEOUT_MS: u64 = 400;
/// Poll interval while a pulse runs and no fresh sample is available (ms).
const POLL_INTERVAL_MS: u64 = 5;
/// Proportional gain from degrees of error to speed units.
const SPEED_PER_DEG: f32 = 6.0;

// ── Phase machine ─────────────────────────────────────────────────────────────

/// Where the correction phase is between adapter actions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Waiting for a settle sample until `deadline_ms`.
    Settle {
        /// Absolute time at which the settle wait gives up on a fresh sample.
        deadline_ms: u64,
        /// Whether this settle follows a pulse, and so increments the iteration
        /// and checks the exhaustion limit when it ends.
        after_pulse: bool,
    },
    /// Check convergence, then emit a pulse or finish.
    IterationEntry,
    /// A pulse runs until `deadline_ms`; a fresh sample may converge, overshoot
    /// or be ignored.
    Pulse {
        /// Absolute per-iteration timeout.
        deadline_ms: u64,
        /// Error sign the pulse started with, for overshoot detection.
        sign: i8,
    },
    /// A terminal stop has been emitted; the next step is `Finished`.
    FinishAfterStop,
    /// `Finished` has been emitted.
    Finished,
}

/// The correction phase's state, seeded by the turn that just stopped.
pub struct Correction {
    /// Absolute target yaw in degrees, anchored by the turn on its first sample.
    target_yaw: f32,
    /// Direction of the original turn.
    direction: RotationDirection,
    /// Newest measured yaw in degrees.
    last_yaw: f32,
    /// Timestamp of the newest consumed sample, used to reject stale samples.
    last_sample_ms: u64,
    /// Absolute turn start time, for the correction-inclusive duration telemetry.
    started_at_ms: u64,
    /// Index of the current pulse, from zero.
    iteration: u8,
    /// Overshoots counted during the correction.
    overshoot_count: u8,
    /// Error sign at the previous iteration entry, for inter-iteration overshoots.
    last_error_sign: i8,
    /// The terminal outcome, present once the correction has finished.
    outcome: Option<DriveOutcome>,
    /// Where the phase machine is between adapter actions.
    phase: Phase,
}

impl Correction {
    /// Seed the correction from the stopped turn.
    ///
    /// `anchor_yaw` and `anchor_sample_ms` are the turn's final sample; they seed
    /// the phase's angle state exactly like the firmware's pre-stop update.
    pub const fn new(
        target_yaw: f32,
        direction: RotationDirection,
        anchor_yaw: f32,
        anchor_sample_ms: u64,
        started_at_ms: u64,
        now_ms: u64,
    ) -> Self {
        Self {
            target_yaw,
            direction,
            last_yaw: anchor_yaw,
            last_sample_ms: anchor_sample_ms,
            started_at_ms,
            iteration: 0,
            overshoot_count: 0,
            last_error_sign: 0,
            outcome: None,
            phase: Phase::Settle {
                deadline_ms: now_ms.saturating_add(SETTLE_MS),
                after_pulse: false,
            },
        }
    }

    /// Advance the correction by one action.
    ///
    /// Pure: every time and sample comes from `obs`. Decision-only phases
    /// (settle expiry, iteration entry) are resolved inside this call, so every
    /// return is exactly one action for the adapter.
    pub fn step(&mut self, obs: &Observation) -> IntentAction {
        loop {
            match self.phase {
                Phase::Settle {
                    deadline_ms,
                    after_pulse,
                } => {
                    let settled = self.absorb(obs);
                    if !settled && obs.now_ms < deadline_ms {
                        return IntentAction::WaitUntil { deadline_ms };
                    }

                    if after_pulse {
                        if self.converged() {
                            self.complete(obs.now_ms);
                            return IntentAction::Finished;
                        }
                        self.iteration = self.iteration.saturating_add(1);
                        if self.iteration >= MAX_ITERATIONS {
                            self.exhaust(obs.now_ms);
                            return IntentAction::Finished;
                        }
                    }
                    self.phase = Phase::IterationEntry;
                }
                Phase::IterationEntry => {
                    if self.converged() {
                        self.complete(obs.now_ms);
                        return IntentAction::Finished;
                    }

                    let sign = self.error_sign();
                    if self.last_error_sign != 0 && sign != self.last_error_sign {
                        self.overshoot_count = self.overshoot_count.saturating_add(1);
                    }
                    self.last_error_sign = sign;

                    let (left, right) = self.pulse_speeds();
                    self.phase = Phase::Pulse {
                        deadline_ms: obs.now_ms.saturating_add(ITER_TIMEOUT_MS),
                        sign,
                    };
                    return IntentAction::SetTracks { left, right };
                }
                Phase::Pulse { deadline_ms, sign } => {
                    if obs.now_ms > deadline_ms {
                        self.phase = Phase::Settle {
                            deadline_ms: obs.now_ms.saturating_add(SETTLE_MS),
                            after_pulse: true,
                        };
                        return IntentAction::SetTracks { left: 0, right: 0 };
                    }

                    if self.absorb(obs) {
                        if self.converged() {
                            self.complete_after_stop(obs.now_ms);
                            return IntentAction::SetTracks { left: 0, right: 0 };
                        }
                        if self.error_sign() != sign {
                            self.overshoot_count = self.overshoot_count.saturating_add(1);
                            self.phase = Phase::Settle {
                                deadline_ms: obs.now_ms.saturating_add(SETTLE_MS),
                                after_pulse: true,
                            };
                            return IntentAction::SetTracks { left: 0, right: 0 };
                        }
                    }

                    let deadline_ms = obs.now_ms.saturating_add(POLL_INTERVAL_MS);
                    return IntentAction::WaitUntil { deadline_ms };
                }
                Phase::FinishAfterStop => {
                    self.phase = Phase::Finished;
                    return IntentAction::Finished;
                }
                Phase::Finished => return IntentAction::Finished,
            }
        }
    }

    /// The newest yaw the correction has seen, in degrees.
    pub const fn last_yaw(&self) -> f32 {
        self.last_yaw
    }

    /// Signed remaining angle to the target, in degrees.
    ///
    /// Positive means the turn still needs to move in the commanded direction;
    /// negative means it has gone past the target.
    pub fn remaining_deg(&self) -> f32 {
        let diff = normalize_angle(self.last_yaw - self.target_yaw);
        match self.direction {
            RotationDirection::Clockwise => diff,
            RotationDirection::CounterClockwise => -diff,
        }
    }

    /// Take the terminal outcome once the correction has finished.
    pub const fn take_outcome(&mut self) -> Option<DriveOutcome> {
        self.outcome.take()
    }

    /// Consume a fresh yaw sample from `obs`, if there is one.
    ///
    /// Returns `true` when a sample newer than the last consumed one arrived.
    const fn absorb(&mut self, obs: &Observation) -> bool {
        match obs.yaw {
            Some(sample) if sample.timestamp_ms > self.last_sample_ms => {
                self.last_yaw = sample.yaw_deg;
                self.last_sample_ms = sample.timestamp_ms;
                true
            }
            _ => false,
        }
    }

    /// Whether the remaining error is inside the tolerance.
    fn converged(&self) -> bool {
        self.remaining_deg().abs() <= TOLERANCE_DEG
    }

    /// The current error sign; exactly zero counts as negative, as in the
    /// firmware's overshoot comparison.
    fn error_sign(&self) -> i8 {
        if self.remaining_deg() > 0.0 { 1 } else { -1 }
    }

    /// The track speeds for this iteration's pulse, using the torque ladder.
    ///
    /// The base speed scales with error magnitude (~6 speed units per degree),
    /// clamps between [`SPEED_MIN`] and [`SPEED_START`], then steps down by
    /// [`SPEED_STEP`] per iteration. A positive error keeps the commanded
    /// direction; a non-positive one reverses it.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn pulse_speeds(&self) -> (i8, i8) {
        let error_deg = self.remaining_deg();

        let proportional =
            (roundf(error_deg.abs() * SPEED_PER_DEG).clamp(0.0, 255.0) as u8).clamp(SPEED_MIN, SPEED_START);
        let speed = proportional
            .saturating_sub(self.iteration.saturating_mul(SPEED_STEP))
            .max(SPEED_MIN);

        let effective_direction = if error_deg > 0.0 {
            self.direction
        } else {
            opposite(self.direction)
        };

        let speed = i8::try_from(speed).unwrap_or(i8::MAX);
        match effective_direction {
            RotationDirection::Clockwise => (speed, -speed),
            RotationDirection::CounterClockwise => (-speed, speed),
        }
    }

    /// Record a successful convergence that needs no further motor command.
    fn complete(&mut self, now_ms: u64) {
        self.outcome = Some(DriveOutcome::Completed {
            telemetry: self.telemetry(now_ms),
        });
        self.phase = Phase::Finished;
    }

    /// Record a successful convergence that still has to stop the tracks.
    fn complete_after_stop(&mut self, now_ms: u64) {
        self.outcome = Some(DriveOutcome::Completed {
            telemetry: self.telemetry(now_ms),
        });
        self.phase = Phase::FinishAfterStop;
    }

    /// Record the exhaustion failure after the last iteration.
    fn exhaust(&mut self, now_ms: u64) {
        self.outcome = Some(DriveOutcome::Failed {
            fault: DriveFault::CorrectionExhausted,
            telemetry: self.telemetry(now_ms),
        });
        self.phase = Phase::Finished;
    }

    /// The correction-inclusive rotation telemetry at `now_ms`.
    fn telemetry(&self, now_ms: u64) -> CompletionTelemetry {
        CompletionTelemetry::RotateExact {
            final_yaw_deg: self.last_yaw,
            angle_error_deg: -self.remaining_deg(),
            duration_ms: now_ms.saturating_sub(self.started_at_ms),
        }
    }
}

/// The opposite rotation direction.
const fn opposite(direction: RotationDirection) -> RotationDirection {
    match direction {
        RotationDirection::Clockwise => RotationDirection::CounterClockwise,
        RotationDirection::CounterClockwise => RotationDirection::Clockwise,
    }
}
