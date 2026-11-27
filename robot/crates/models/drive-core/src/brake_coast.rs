//! The brake and coast settle control law, ported from the firmware.
//!
//! This module owns the `Brake` and `Coast` intents. Both emit their initial
//! motor action, then sample the encoder deltas at a fixed cadence until both
//! tracks report the required number of consecutive zero deltas. Any motion
//! resets the count, and a deadline turns a never-settling robot into the
//! lowercase `encoder settle timeout` fault.
//!
//! It is a pure step machine. It never reads a clock, starts a sensor, commands
//! a motor or allocates: every timestamp comes from the [`Observation`], and
//! every action is returned to the adapter to execute. A sleep becomes
//! [`IntentAction::WaitUntil`].

use crate::{CompletionTelemetry, DriveFault, DriveOutcome, EncoderSample, IntentAction, Observation, Step};

// ── Brake/coast control constants (ported verbatim from the firmware) ────────

/// Encoder settle interval for brake/coast completion (milliseconds).
const SETTLE_INTERVAL_MS: u64 = 100;
/// Consecutive zero-delta samples required to declare settled.
const SETTLE_CONSECUTIVE_SAMPLES: u8 = 3;
/// Maximum time allowed to settle before failing completion (milliseconds).
const SETTLE_TIMEOUT_MS: u64 = 2_000;

/// The motor action a brake/coast intent emits first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Brake both tracks electrically.
    Brake,
    /// Let both tracks freewheel.
    Coast,
}

/// Where the step machine is between adapter actions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// The initial brake/coast action has not been emitted yet.
    Start,
    /// Waiting for the next settle tick at `deadline_ms`.
    Wait {
        /// Absolute time at which the next settle sample may be read.
        deadline_ms: u64,
    },
    /// The terminal `Finished` action has been emitted.
    Finished,
}

/// Private state of a brake or coast intent.
pub struct BrakeCoastState {
    /// The initial action emitted on the first step.
    action: Action,
    /// Absolute deadline for encoder settle detection (milliseconds).
    deadline_ms: u64,
    /// Consecutive zero-delta samples observed.
    consecutive: u8,
    /// Previous encoder sample for delta computation.
    last_sample: Option<EncoderSample>,
    /// Timestamp of the last processed encoder sample (milliseconds).
    last_timestamp_ms: u64,
    /// Where the step machine is between adapter actions.
    phase: Phase,
    /// The terminal outcome, present only once `Finished` has been emitted.
    outcome: Option<DriveOutcome>,
}

impl BrakeCoastState {
    /// Create a brake or coast intent starting at `started_at_ms`.
    ///
    /// The settle deadline is two seconds from the start; the first zero-delta
    /// sample may only be read two ticks later, because the first sample only
    /// seeds the delta baseline.
    pub const fn new(action: Action, started_at_ms: u64) -> Self {
        Self {
            action,
            deadline_ms: started_at_ms.saturating_add(SETTLE_TIMEOUT_MS),
            consecutive: 0,
            last_sample: None,
            last_timestamp_ms: 0,
            phase: Phase::Start,
            outcome: None,
        }
    }

    /// Advance the intent one step and return the action the adapter executes.
    pub fn step(&mut self, obs: &Observation) -> Step {
        match self.phase {
            Phase::Start => {
                let action = match self.action {
                    Action::Brake => IntentAction::BrakeAll,
                    Action::Coast => IntentAction::CoastAll,
                };
                self.phase = Phase::Wait {
                    deadline_ms: obs.now_ms.saturating_add(SETTLE_INTERVAL_MS),
                };
                make_step(action)
            }
            Phase::Wait { deadline_ms } => {
                if obs.now_ms < deadline_ms {
                    make_step(IntentAction::WaitUntil { deadline_ms })
                } else {
                    self.settle(obs)
                }
            }
            Phase::Finished => make_step(IntentAction::Finished),
        }
    }

    /// The terminal outcome, once the `Finished` action has been emitted.
    pub fn outcome(&self) -> Option<DriveOutcome> {
        self.outcome.clone()
    }

    /// Brake/coast has no progress telemetry to report on cancellation.
    #[allow(clippy::unused_self)]
    pub const fn cancellation_telemetry(&self, _now_ms: u64) -> CompletionTelemetry {
        CompletionTelemetry::None
    }

    /// Check the deadline and fold one fresh encoder sample into the count.
    fn settle(&mut self, obs: &Observation) -> Step {
        if obs.now_ms >= self.deadline_ms {
            self.outcome = Some(DriveOutcome::Failed {
                fault: DriveFault::SettleTimeout,
                telemetry: CompletionTelemetry::None,
            });
            self.phase = Phase::Finished;
            return make_step(IntentAction::Finished);
        }

        if let Some(measurement) = obs
            .encoder
            .filter(|measurement| measurement.timestamp_ms != 0 && measurement.timestamp_ms != self.last_timestamp_ms)
        {
            self.last_timestamp_ms = measurement.timestamp_ms;

            if let Some(previous) = self.last_sample {
                let delta_left = u32::from(measurement.left.wrapping_sub(previous.left));
                let delta_right = u32::from(measurement.right.wrapping_sub(previous.right));

                if delta_left == 0 && delta_right == 0 {
                    self.consecutive = self.consecutive.saturating_add(1);
                    if self.consecutive >= SETTLE_CONSECUTIVE_SAMPLES {
                        self.last_sample = Some(measurement);
                        self.outcome = Some(DriveOutcome::Completed {
                            telemetry: CompletionTelemetry::None,
                        });
                        self.phase = Phase::Finished;
                        return make_step(IntentAction::Finished);
                    }
                } else {
                    self.consecutive = 0;
                }
            }

            self.last_sample = Some(measurement);
        }

        let deadline_ms = obs.now_ms.saturating_add(SETTLE_INTERVAL_MS);
        self.phase = Phase::Wait { deadline_ms };
        make_step(IntentAction::WaitUntil { deadline_ms })
    }
}

/// Build a step with no trace snapshot: brake/coast has no decision state the
/// firmware logs.
const fn make_step(action: IntentAction) -> Step {
    Step { action, trace: None }
}
