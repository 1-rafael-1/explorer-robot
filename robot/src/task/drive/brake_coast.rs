//! Brake, coast and idle adapter: drives `drive-core`'s settle and idle intents.
//!
//! The control law itself lives in [`drive_core::Intent`]: brake and coast emit
//! their initial motor action, sample encoder deltas at the 100 ms settle
//! cadence, reset the zero-delta count on motion and fail at the two-second
//! deadline; idle coasts and completes on its duration, or immediately at zero.
//!
//! This module is the adapter half. It starts and stops the encoder sensor for
//! brake/coast (idle starts no sensors), builds an [`Observation`] from the
//! latest encoder sample and the adapter clock, executes the model's
//! [`IntentAction`]s, keeps the drive interrupt selected around every
//! `WaitUntil`, and maps the model's terminal outcome onto the firmware's
//! completion and `fault.label()`.

use drive_core::{EncoderSample, Intent, IntentAction, IntentParams, Observation};
use embassy_time::Instant;

use crate::task::drive::{
    adapter::{self, ActionGuard, IntentPoll},
    sensors::{control as lifecycle, data as feedback},
    state::ActiveIntent,
};

/// Encoder sampling interval while a brake/coast settle is in progress.
const ENCODER_SAMPLING_INTERVAL_MS: u64 = 100;

/// Completion detail for the adapter's non-blocking action guard.
///
/// Unreachable with the current model; it exists so a future defect cannot hang
/// the drive task.
const ACTION_LOOP_LABEL: &str = "BrakeCoastActionLoop";

/// Completion detail for a `Finished` action without a model outcome.
///
/// Unreachable with the current model; it exists so a model defect still
/// resolves the completion.
const OUTCOME_MISSING_LABEL: &str = "BrakeCoastOutcomeMissing";

/// Initialise a brake settle intent and start encoder sampling.
pub(super) async fn init_brake(completion_requested: bool) -> ActiveIntent {
    init_settle(IntentParams::Brake, completion_requested).await
}

/// Initialise a coast settle intent and start encoder sampling.
pub(super) async fn init_coast(completion_requested: bool) -> ActiveIntent {
    init_settle(IntentParams::Coast, completion_requested).await
}

/// Initialise an idle intent. Idle starts no sensors and needs no async setup.
pub(super) fn init_idle(duration_ms: u64, completion_requested: bool) -> ActiveIntent {
    let model = Intent::new(IntentParams::Idle { duration_ms }, Instant::now().as_millis());
    ActiveIntent::Idle {
        model,
        completion_requested,
    }
}

/// Start encoder sampling and build a brake or coast intent.
async fn init_settle(params: IntentParams, completion_requested: bool) -> ActiveIntent {
    lifecycle::start_encoder_sampling(ENCODER_SAMPLING_INTERVAL_MS, true).await;
    let model = Intent::new(params, Instant::now().as_millis());
    ActiveIntent::BrakeCoast {
        model,
        completion_requested,
    }
}

/// Run a brake or coast settle until it finishes, fails or is interrupted.
pub(super) async fn run_brake_coast_poll(model: &mut Intent) -> IntentPoll {
    run_model_poll(model, true).await
}

/// Run an idle intent until it finishes or is interrupted.
pub(super) async fn run_idle_poll(model: &mut Intent) -> IntentPoll {
    run_model_poll(model, false).await
}

/// Run the model until it finishes, fails or an interrupt preempts a wait.
///
/// `watch_encoder` is true for brake/coast, whose model reads settle deltas, and
/// false for idle, which starts and reads no sensors.
async fn run_model_poll(model: &mut Intent, watch_encoder: bool) -> IntentPoll {
    let mut observation = build_observation(watch_encoder).await;
    let mut guard = ActionGuard::new();

    loop {
        let step = model.step(&observation);

        match step.action {
            IntentAction::WaitUntil { deadline_ms } => {
                guard.reset();
                match adapter::wait_until(deadline_ms).await {
                    Some(kind) => return IntentPoll::Interrupt(kind),
                    None => observation = build_observation(watch_encoder).await,
                }
            }
            IntentAction::Finished => {
                return IntentPoll::Step(adapter::terminal(model, OUTCOME_MISSING_LABEL));
            }
            action => {
                adapter::execute_non_blocking(action).await;
                if !guard.record() {
                    return IntentPoll::Step(adapter::adapter_failure(model, ACTION_LOOP_LABEL));
                }
            }
        }
    }
}

/// Build an observation from the adapter clock and, for brake/coast, the newest
/// encoder sample. Idle reads no sensor and always passes `encoder: None`.
async fn build_observation(watch_encoder: bool) -> Observation {
    let now_ms = Instant::now().as_millis();
    let encoder = if watch_encoder {
        feedback::get_latest_encoder_measurement()
            .await
            .map(|measurement| EncoderSample {
                left: measurement.left,
                right: measurement.right,
                timestamp_ms: measurement.timestamp_ms,
            })
    } else {
        None
    };

    Observation {
        now_ms,
        encoder,
        yaw: None,
    }
}
