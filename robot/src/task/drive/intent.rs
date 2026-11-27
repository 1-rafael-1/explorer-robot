//! Intent orchestration for the drive loop.
//!
//! This module wires the active-intent state machine to control-loop ticks and
//! interrupt handling. It owns the polling/apply flow and idle stepping.
//!
//! # Interrupt vs completion behaviour
//!
//! - Interrupts preempt any active intent and cause its completion (if requested)
//!   to resolve as `Cancelled`.
//! - The interrupt handler drains the queue. Any queued command that requested
//!   completion is resolved as `Cancelled`. Additional queued completion requests
//!   are dropped with a warning under the single-producer contract.
//! - The interrupt handler also increments an epoch counter. Any queued commands
//!   stamped before the interrupt are cancelled when dequeued.
//!
//! # Epoch invalidation rationale
//!
//! Epoch invalidation provides a simple, lock-free way to discard stale queued
//! commands after a preemption. Active intents are cancelled immediately, while
//! queued intents are cancelled when dequeued by comparing epochs.

use embassy_futures::select::{Either, select};

use crate::task::drive::{
    adapter::{IntentPoll, IntentStepResult},
    api::{DRIVE_INTERRUPT, DRIVE_QUEUE, send_completion},
    brake_coast,
    dispatch::execute_intent_teardown,
    distance, rotation,
    state::DriveLoop,
    types::{self, CompletionStatus, DriveCompletion, IntentTeardown},
};

/// Outcome of polling an active intent.
pub(super) enum ActiveIntentOutcome {
    /// An interrupt arrived while an intent was active.
    Interrupt(types::InterruptKind),
    /// A model-backed intent reached a terminal result.
    Step(IntentStepResult),
}

/// Poll the active intent (if any) and return the next action to perform.
pub(super) async fn poll_active_intent(loop_state: &mut DriveLoop) -> Option<ActiveIntentOutcome> {
    match loop_state.active_intent.as_mut() {
        Some(super::state::ActiveIntent::RotateExact {
            model, started_at_ms, ..
        }) => Some(intent_poll_to_outcome(
            rotation::run_rotation_poll(model, *started_at_ms).await,
        )),
        Some(
            super::state::ActiveIntent::DriveDistance { model, .. }
            | super::state::ActiveIntent::DriveContinuous { model, .. },
        ) => Some(intent_poll_to_outcome(distance::run_distance_poll(model).await)),
        Some(super::state::ActiveIntent::BrakeCoast { model, .. }) => {
            Some(intent_poll_to_outcome(brake_coast::run_brake_coast_poll(model).await))
        }
        Some(super::state::ActiveIntent::Idle { model, .. }) => {
            Some(intent_poll_to_outcome(brake_coast::run_idle_poll(model).await))
        }
        None => None,
    }
}

/// Map a model adapter's poll result onto the loop's outcome.
const fn intent_poll_to_outcome(poll: IntentPoll) -> ActiveIntentOutcome {
    match poll {
        IntentPoll::Interrupt(kind) => ActiveIntentOutcome::Interrupt(kind),
        IntentPoll::Step(result) => ActiveIntentOutcome::Step(result),
    }
}

/// Apply completion: teardown sensors via descriptor, send completion, clear intent.
pub(super) async fn apply_completion(
    loop_state: &mut DriveLoop,
    status: CompletionStatus,
    telemetry: types::CompletionTelemetry,
) {
    // Extract completion_requested and teardown before taking the intent.
    let (completion_requested, teardown) = loop_state
        .active_intent
        .as_ref()
        .map_or((false, IntentTeardown::None), |i| {
            (i.completion_requested(), i.teardown())
        });

    execute_intent_teardown(teardown).await;
    send_completion(completion_requested, DriveCompletion { status, telemetry }).await;
    loop_state.active_intent = None;
}

/// Idle loop step — wait for a queued command or an interrupt.
pub(super) async fn step_idle(loop_state: &mut DriveLoop) {
    let command_or_interrupt = select(DRIVE_QUEUE.receiver().receive(), DRIVE_INTERRUPT.wait()).await;
    match command_or_interrupt {
        Either::First(envelope) => loop_state.handle_envelope(envelope).await,
        Either::Second(kind) => loop_state.handle_interrupt(kind).await,
    }
}
