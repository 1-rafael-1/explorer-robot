//! The idle intent, ported from the firmware.
//!
//! Idle parks the robot: it emits [`IntentAction::CoastAll`] once and then
//! completes. A non-zero duration waits on an absolute deadline; a zero duration
//! finishes on the step after the coast action. It touches no sensors and
//! reports no telemetry.
//!
//! It is a pure step machine. It never reads a clock, starts a sensor, commands
//! a motor or allocates: every timestamp comes from the [`Observation`], and
//! every action is returned to the adapter to execute. A sleep becomes
//! [`IntentAction::WaitUntil`].

use crate::{CompletionTelemetry, DriveOutcome, IntentAction, Observation, Step};

/// Where the step machine is between adapter actions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// The initial coast action has not been emitted yet.
    Start,
    /// Waiting for the idle duration to elapse at `deadline_ms`.
    Wait {
        /// Absolute time at which the idle completes.
        deadline_ms: u64,
    },
    /// The terminal `Finished` action has been emitted.
    Finished,
}

/// Private state of an idle intent.
pub struct IdleState {
    /// How long to stay idle, in milliseconds.
    duration_ms: u64,
    /// Absolute completion deadline in milliseconds.
    deadline_ms: u64,
    /// Where the step machine is between adapter actions.
    phase: Phase,
    /// The terminal outcome, present only once `Finished` has been emitted.
    outcome: Option<DriveOutcome>,
}

impl IdleState {
    /// Create an idle intent lasting `duration_ms`, starting at `started_at_ms`.
    pub const fn new(duration_ms: u64, started_at_ms: u64) -> Self {
        Self {
            duration_ms,
            deadline_ms: started_at_ms.saturating_add(duration_ms),
            phase: Phase::Start,
            outcome: None,
        }
    }

    /// Advance the intent one step and return the action the adapter executes.
    pub const fn step(&mut self, obs: &Observation) -> Step {
        match self.phase {
            Phase::Start => {
                if self.duration_ms == 0 {
                    self.outcome = Some(DriveOutcome::Completed {
                        telemetry: CompletionTelemetry::None,
                    });
                    self.phase = Phase::Finished;
                } else {
                    self.phase = Phase::Wait {
                        deadline_ms: self.deadline_ms,
                    };
                }
                make_step(IntentAction::CoastAll)
            }
            Phase::Wait { deadline_ms } => {
                if obs.now_ms < deadline_ms {
                    make_step(IntentAction::WaitUntil { deadline_ms })
                } else {
                    self.outcome = Some(DriveOutcome::Completed {
                        telemetry: CompletionTelemetry::None,
                    });
                    self.phase = Phase::Finished;
                    make_step(IntentAction::Finished)
                }
            }
            Phase::Finished => make_step(IntentAction::Finished),
        }
    }

    /// The terminal outcome, once the `Finished` action has been emitted.
    pub fn outcome(&self) -> Option<DriveOutcome> {
        self.outcome.clone()
    }

    /// Idle has no progress telemetry to report on cancellation.
    #[allow(clippy::unused_self)]
    pub const fn cancellation_telemetry(&self, _now_ms: u64) -> CompletionTelemetry {
        CompletionTelemetry::None
    }
}

/// Build a step with no trace snapshot: idle has no decision state the firmware
/// logs.
const fn make_step(action: IntentAction) -> Step {
    Step { action, trace: None }
}
