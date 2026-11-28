//! Shared action executor for the drive task's `drive-core` model adapters.
//!
//! The rotation and distance adapters run the same executor contract: they call
//! [`Intent::step`](drive_core::Intent::step), execute the returned action, and
//! step again. This module owns the parts that are identical between them — the
//! motor commands, the interrupt-safe absolute wait and the mapping from the
//! model's terminal outcome to the adapter's step result — so there is one
//! implementation of the adapter contract rather than two subtly different
//! copies.
//!
//! The model never commands a motor, reads a clock or starts a sensor; every
//! action here is the adapter's duty.

use drive_core::{CompletionTelemetry, DriveOutcome, Intent, IntentAction};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};

use crate::{
    system::state::motion,
    task::{
        drive::{api::DRIVE_INTERRUPT, types},
        motor_driver::{self, MotorCommand},
    },
};

/// Maximum non-blocking actions the adapter executes against one observation.
///
/// The model never needs more than a handful: a command is always followed by a
/// `WaitUntil` or `Finished`. The guard turns a model defect into a terminal
/// fault instead of a busy loop.
pub(super) const MAX_ACTIONS_PER_OBSERVATION: u8 = 8;

/// The intermediate outcome of polling a model intent.
pub(super) enum IntentPoll {
    /// A drive interrupt arrived while the adapter waited.
    Interrupt(types::InterruptKind),
    /// The intent reached a terminal result.
    Step(IntentStepResult),
}

/// The terminal result of one model intent.
pub(super) enum IntentStepResult {
    /// The intent completed successfully with final telemetry.
    Completed {
        /// Completion telemetry captured at success.
        telemetry: CompletionTelemetry,
    },
    /// The intent failed with a static reason and telemetry snapshot.
    Failed {
        /// Failure reason identifier.
        reason: &'static str,
        /// Completion telemetry captured at failure.
        telemetry: CompletionTelemetry,
    },
}

/// Counts consecutive non-blocking actions executed against one observation.
pub(super) struct ActionGuard {
    /// How many non-blocking actions have run since the last wait.
    count: u8,
}

impl ActionGuard {
    /// Create a guard with no recorded actions.
    pub(super) const fn new() -> Self {
        Self { count: 0 }
    }

    /// Reset the count after a `WaitUntil` or `Finished` observation boundary.
    pub(super) const fn reset(&mut self) {
        self.count = 0;
    }

    /// Record one non-blocking action.
    ///
    /// Returns `false` once the guard limit is exceeded, which the caller turns
    /// into a terminal adapter fault.
    pub(super) const fn record(&mut self) -> bool {
        self.count = self.count.saturating_add(1);
        self.count <= MAX_ACTIONS_PER_OBSERVATION
    }
}

/// Execute one non-blocking model action.
///
/// `WaitUntil` and `Finished` are observation boundaries the caller handles; if
/// either reaches this function it is ignored.
pub(super) async fn execute_non_blocking(action: IntentAction) {
    match action {
        IntentAction::SetTracks { left, right } => send_tracks(left, right).await,
        IntentAction::BrakeAll => {
            motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            motion::set_track_speeds(0, 0).await;
        }
        IntentAction::CoastAll => {
            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            motion::set_track_speeds(0, 0).await;
        }
        IntentAction::WaitUntil { .. } | IntentAction::Finished => {}
    }
}

/// Sleep to an absolute deadline with the drive interrupt selected.
///
/// Returns the interrupt kind if `Stop` or `EmergencyBrake` preempted the wait, or
/// `None` once the deadline passed.
pub(super) async fn wait_until(deadline_ms: u64) -> Option<types::InterruptKind> {
    let wait_ms = deadline_ms.saturating_sub(Instant::now().as_millis());
    match select(DRIVE_INTERRUPT.wait(), Timer::after(Duration::from_millis(wait_ms))).await {
        Either::First(kind) => Some(kind),
        Either::Second(()) => None,
    }
}

/// Map the model's terminal outcome to an adapter step result.
///
/// A model defect that emits `Finished` without an outcome becomes a terminal
/// failure with `missing_reason` and cancellation telemetry.
pub(super) fn terminal(model: &Intent, missing_reason: &'static str) -> IntentStepResult {
    match model.outcome() {
        Some(DriveOutcome::Completed { telemetry }) => IntentStepResult::Completed { telemetry },
        Some(DriveOutcome::Failed { fault, telemetry }) => IntentStepResult::Failed {
            reason: fault.label(),
            telemetry,
        },
        None => adapter_failure(model, missing_reason),
    }
}

/// A terminal failure for an adapter guard or a missing model outcome.
pub(super) fn adapter_failure(model: &Intent, reason: &'static str) -> IntentStepResult {
    IntentStepResult::Failed {
        reason,
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
