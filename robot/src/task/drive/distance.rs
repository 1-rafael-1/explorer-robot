//! Distance and continuous adapter: drives `drive-core`'s distance intents.
//!
//! The control law itself lives in [`drive_core::Intent`]: encoder progress,
//! wrap-safe deltas, freshness, the stall and encoder-timeout checks, the
//! single-motor-zero anomaly, the speed ramp, tolerance completion, straight
//! heading correction and curve-yaw correction. This module is the adapter
//! half: it resolves the measured-or-nominal encoder scale, starts and stops
//! the distance IMU and encoder sampling, drains their feedback, builds an
//! [`Observation`], executes the model's [`IntentAction`]s, keeps the drive
//! interrupt selected around every `WaitUntil`, and maps the model's terminal
//! outcome onto the firmware's completion.
//!
//! The model decides freshness and timeouts; the adapter only drains and maps.
//!
//! # Telemetry logging
//!
//! `drive-core`'s trace is feature-gated and the firmware turns it on with the
//! `telemetry_logs` feature. The model never logs and never depends on `defmt`.

use defmt::info;
use drive_core::{
    DriveDirection, DriveDistanceKind, EncoderSample, Intent, IntentAction, IntentParams, Observation, YawSample,
};
use embassy_time::Instant;
use motor_cal::DistanceScale;

use crate::{
    system::state::calibration,
    task::{
        drive::{
            adapter::{self, ActionGuard, IntentPoll, IntentStepResult},
            api,
            sensors::{
                control as lifecycle,
                data::{self as feedback, IMU_FEEDBACK_CHANNEL},
            },
            state::ActiveIntent,
            types::{CompletionStatus, DriveCompletion},
        },
        sensors::imu::ImuMeasurement,
    },
};

/// Encoder sampling interval for a distance drive, matching the model's control
/// cadence.
const ENCODER_SAMPLING_INTERVAL_MS: u64 = 20;

/// Completion detail for the adapter's non-blocking action guard.
///
/// Unreachable with the current model; it exists so a future defect cannot hang
/// the drive task.
const ACTION_LOOP_LABEL: &str = "DistanceActionLoop";

/// Completion detail for a `Finished` action without a model outcome.
///
/// Unreachable with the current model; it exists so a model defect still
/// resolves the completion.
const OUTCOME_MISSING_LABEL: &str = "DistanceOutcomeMissing";

/// Initialise a target-distance intent.
///
/// Resolves the encoder scale, constructs the model intent and starts the
/// distance sensors. A target inside the completion tolerance finishes on the
/// model's first step before any sensor starts or motor command, exactly like
/// the old controller's early exit; in that case the completion is sent here
/// and `None` is returned.
pub(super) async fn init(
    kind: DriveDistanceKind,
    direction: DriveDirection,
    speed: u8,
    completion_requested: bool,
) -> Option<ActiveIntent> {
    let scale = drive_scale().await;
    let started_at_ms = Instant::now().as_millis();
    let mut model = Intent::new(
        IntentParams::DriveDistance {
            kind,
            direction,
            speed,
            scale,
        },
        started_at_ms,
    );

    // Probe the first step: a trivial distance emits `Finished` without
    // touching the sensors or motors. A non-trivial drive is left in its
    // heading-reference wait and re-stepped once the sensors are running.
    let first = model.step(&Observation {
        now_ms: started_at_ms,
        encoder: None,
        yaw: None,
    });
    if first.action == IntentAction::Finished {
        finish_without_controller(&model, completion_requested).await;
        return None;
    }

    start_sensors(&kind).await;

    Some(ActiveIntent::DriveDistance {
        model,
        completion_requested,
    })
}

/// Initialise a continuous drive intent.
///
/// The straight controller with no target: it starts the same sensors as
/// [`init`], but has no trivial-distance early exit because it never completes.
/// Only an interrupt or a fault can end it.
pub(super) async fn init_continuous(direction: DriveDirection, speed: u8, completion_requested: bool) -> ActiveIntent {
    let started_at_ms = Instant::now().as_millis();
    let model = Intent::new(IntentParams::DriveContinuous { direction, speed }, started_at_ms);

    start_sensors(&DriveDistanceKind::Straight { distance_cm: 0.0 }).await;

    ActiveIntent::DriveContinuous {
        model,
        completion_requested,
    }
}

/// Run the model until it finishes, fails or an interrupt preempts a wait.
pub(super) async fn run_distance_poll(model: &mut Intent) -> IntentPoll {
    // A distance drive starts from the first observation the sensors can give;
    // the model captures the heading reference or waits its 50 ms bound.
    let mut observation = build_observation().await;
    let mut guard = ActionGuard::new();

    loop {
        let step = model.step(&observation);

        match step.action {
            IntentAction::WaitUntil { deadline_ms } => {
                guard.reset();
                match adapter::wait_until(deadline_ms).await {
                    Some(kind) => return IntentPoll::Interrupt(kind),
                    None => observation = build_observation().await,
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

/// Build an observation from the adapter clock and the newest sensor samples.
async fn build_observation() -> Observation {
    let now_ms = Instant::now().as_millis();
    let encoder = feedback::get_latest_encoder_measurement()
        .await
        .map(|measurement| EncoderSample {
            left: measurement.left,
            right: measurement.right,
            timestamp_ms: measurement.timestamp_ms,
        });
    let yaw = drain_latest_imu().map(|measurement| YawSample {
        yaw_deg: measurement.orientation.yaw,
        timestamp_ms: measurement.timestamp_ms,
    });

    Observation { now_ms, encoder, yaw }
}

/// Drain the IMU feedback channel and return its newest sample.
///
/// The model decides whether a sample is fresh; the adapter only maps the one
/// it has.
fn drain_latest_imu() -> Option<ImuMeasurement> {
    let mut latest: Option<ImuMeasurement> = None;
    while let Ok(measurement) = IMU_FEEDBACK_CHANNEL.receiver().try_receive() {
        latest = Some(measurement);
    }
    latest
}

/// Start the distance IMU stream and encoder sampling.
async fn start_sensors(kind: &DriveDistanceKind) {
    lifecycle::start_distance_imu(kind);
    lifecycle::start_encoder_sampling(ENCODER_SAMPLING_INTERVAL_MS, true).await;
}

/// Send the completion for a drive that finished without a controller.
async fn finish_without_controller(model: &Intent, completion_requested: bool) {
    let (status, telemetry) = match adapter::terminal(model, OUTCOME_MISSING_LABEL) {
        IntentStepResult::Completed { telemetry } => (CompletionStatus::Success, telemetry),
        IntentStepResult::Failed { reason, telemetry } => (CompletionStatus::Failed(reason), telemetry),
    };
    api::send_completion(completion_requested, DriveCompletion { status, telemetry }).await;
}

/// The encoder scale a new distance drive uses.
///
/// A measured scale published to the calibration leaf wins; without one, the
/// drive uses the inferred nominal counts/cm so a fresh robot still drives the
/// distance it was asked for. The log names the fallback once per drive.
async fn drive_scale() -> DistanceScale {
    calibration::get_distance_scale().await.unwrap_or_else(|| {
        let scale = DistanceScale::nominal();
        info!(
            "Distance drive: no measured scale - using nominal {=f32} counts/cm",
            scale.counts_per_cm()
        );
        scale
    })
}
