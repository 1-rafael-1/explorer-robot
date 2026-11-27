//! Rotation adapter: drives `drive-core`'s `RotateExact` intent.
//!
//! The control law itself lives in [`drive_core::Intent`], including the
//! provisional post-stop correction phase in [`drive_core`]. This module is the
//! adapter half: it starts and stops the rotation IMU, drains the IMU feedback
//! channel, builds an [`Observation`], executes the model's [`IntentAction`]s,
//! keeps the drive interrupt selected around every `WaitUntil`, and maps the
//! model's terminal outcome onto the firmware's completion.
//!
//! # Telemetry logging
//!
//! Rotation trace fields are logged only when the `telemetry_logs` feature is
//! enabled; that feature also turns on `drive-core`'s `trace` feature. The
//! model never logs and never depends on `defmt`.

use drive_core::{
    DriveFault, Intent, IntentAction, IntentParams, Observation, RotationDirection, RotationMotion, YawSample,
};
use embassy_time::Instant;

use crate::{
    system::state::motion,
    task::{
        drive::{
            adapter::{self, ActionGuard, IntentPoll, MAX_ACTIONS_PER_OBSERVATION},
            clear_imu_measurements,
            sensors::{control as lifecycle, data::IMU_FEEDBACK_CHANNEL},
        },
        motor_driver::{self, MotorCommand},
        sensors::imu::ImuMeasurement,
    },
};

// ── Model adapter ─────────────────────────────────────────────────────────────

/// Completion detail for the adapter's non-blocking action guard.
///
/// Unreachable with the current model; it exists so a future defect cannot hang
/// the drive task.
const ACTION_LOOP_LABEL: &str = "RotationActionLoop";

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
    }
}

/// Run the model until it finishes, fails or an interrupt preempts a wait.
pub(super) async fn run_rotation_poll(model: &mut Intent, started_at_ms: u64) -> IntentPoll {
    // The intent's start action must not consume a sample that arrives before the
    // first control tick; the host contract starts with an empty observation.
    let mut observation = Observation {
        now_ms: Instant::now().as_millis(),
        encoder: None,
        yaw: None,
    };
    let mut guard = ActionGuard::new();
    #[cfg(feature = "telemetry_logs")]
    let mut first_sample_logged = false;

    loop {
        let step = model.step(&observation);

        #[cfg(feature = "telemetry_logs")]
        if !first_sample_logged && let Some(target_yaw) = step.trace.and_then(|trace| trace.target_yaw_deg) {
            first_sample_logged = true;
            defmt::info!(
                "rotate_exact start: yaw={=?} target_yaw={=f32}°",
                observation.yaw.map(|sample| sample.yaw_deg),
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
            IntentAction::WaitUntil { deadline_ms } => {
                guard.reset();
                match adapter::wait_until(deadline_ms).await {
                    Some(kind) => return IntentPoll::Interrupt(kind),
                    None => observation = build_observation(started_at_ms),
                }
            }
            IntentAction::Finished => {
                return IntentPoll::Step(adapter::terminal(model, DriveFault::RotationTimeout.label()));
            }
            action => {
                adapter::execute_non_blocking(action).await;
                if !guard.record() {
                    defmt::error!(
                        "rotate_exact: more than {} non-blocking actions per observation",
                        MAX_ACTIONS_PER_OBSERVATION
                    );
                    return IntentPoll::Step(adapter::adapter_failure(model, ACTION_LOOP_LABEL));
                }
            }
        }
    }
}

/// Build an observation from the adapter clock and the newest IMU sample.
///
/// Rotation does not start the encoder task, so `encoder` is always absent here;
/// the model ignores it for rotation.
fn build_observation(started_at_ms: u64) -> Observation {
    let now_ms = Instant::now().as_millis();
    let sample = drain_latest_imu_since(started_at_ms);

    Observation {
        now_ms,
        encoder: None,
        yaw: sample.map(|measurement| YawSample {
            yaw_deg: measurement.orientation.yaw,
            timestamp_ms: measurement.timestamp_ms,
        }),
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
