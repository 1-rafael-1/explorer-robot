//! Drive command dispatch — envelope routing, standby wake-up, and `IntentTeardown` execution.
//!
//! This module is the thin seam between the drive queue and the control modules.
//! It does NOT own per-intent knowledge — control modules declare their
//! teardown needs via [`IntentTeardown`] descriptors.
//!
//! v3 adaptation: direct 2-motor GPIO commands via `motor_driver::send_command`,
//! no port-expander. `SetTracks { left, right }` maps directly to the two motor
//! channels (left track, right track).

use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use rangefinder::{GateRefusal, gate};

use crate::{
    system::state::{motion, perception},
    task::{
        drive::{
            api::{self, DriveCommandEnvelope},
            brake_coast, differential, distance, rotation,
            sensors::{control as lifecycle, data::IMU_FEEDBACK_CHANNEL},
            state::DriveLoop,
            types::{self, CompletionStatus, DriveAction, DriveCommand, DriveCompletion, IntentTeardown},
        },
        motor_driver::{self, MotorCommand},
    },
};

impl DriveLoop {
    /// Handle a dequeued command envelope, honouring epoch cancellation.
    pub(super) async fn handle_envelope(&mut self, envelope: DriveCommandEnvelope) {
        let current_epoch = api::CURRENT_EPOCH.load(core::sync::atomic::Ordering::Relaxed);
        if envelope.epoch != current_epoch {
            api::send_completion(
                envelope.completion_requested,
                DriveCompletion {
                    status: CompletionStatus::Cancelled,
                    telemetry: types::CompletionTelemetry::None,
                },
            )
            .await;
            return;
        }

        let completion_requested = envelope.completion_requested;
        match envelope.command {
            DriveCommand::Drive(action) => {
                self.handle_drive_action(action, completion_requested).await;
            }
        }
    }

    /// Dispatch a [`DriveAction`] to the appropriate handler.
    pub(super) async fn handle_drive_action(&mut self, action: DriveAction, completion_requested: bool) {
        if action_requires_motion(&action) {
            if !self.run_motion_preflight(&action, completion_requested).await {
                return;
            }
        } else {
            self.wake_from_standby(&action).await;
        }

        match action {
            DriveAction::Differential { left, right } => {
                self.handle_differential(left, right).await;
                api::send_completion(
                    completion_requested,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
            }
            DriveAction::RotateExact {
                degrees,
                direction,
                motion,
            } => {
                self.handle_rotate_exact(degrees, direction, motion, completion_requested)
                    .await;
            }
            DriveAction::DriveDistance { kind, direction, speed } => {
                self.handle_drive_distance(kind, direction, speed, completion_requested)
                    .await;
            }
            DriveAction::DriveContinuous { direction, speed } => {
                self.handle_drive_continuous(direction, speed, completion_requested)
                    .await;
            }
            DriveAction::Coast => {
                self.handle_coast(completion_requested).await;
            }
            DriveAction::Brake => {
                self.handle_brake(completion_requested).await;
            }
            DriveAction::Idle { duration_ms } => {
                self.handle_idle(duration_ms, completion_requested).await;
            }
            DriveAction::Standby => {
                self.handle_standby().await;
                api::send_completion(
                    completion_requested,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
            }
        }
    }

    /// Run the interruptible motion pre-flight.
    ///
    /// Arms the pair, waits for both sensors, waits for the IMU and wakes the
    /// drivers. The whole wait is selected against the drive interrupt, so an
    /// emergency brake can preempt a long arming or IMU wait. A gate refusal or
    /// IMU timeout sends the completion and returns `false`; an interrupt is
    /// handled and also returns `false`. After a successful wait the live gate
    /// is rechecked before any motor command, because a drop can arrive during
    /// the wait without reaching the interrupt handler yet.
    async fn run_motion_preflight(&mut self, action: &DriveAction, completion_requested: bool) -> bool {
        let preflight = async {
            crate::task::sensors::rangefinder::arm_for_motion()
                .await
                .map_err(PreflightFailure::Gate)?;
            if !ensure_imu_ready().await {
                return Err(PreflightFailure::Imu);
            }
            self.wake_from_standby(action).await;
            Ok(())
        };
        match select(preflight, api::DRIVE_INTERRUPT.wait()).await {
            Either::First(Ok(())) => {}
            Either::First(Err(PreflightFailure::Gate(refusal))) => {
                send_preflight_failure(completion_requested, refusal.reason()).await;
                return false;
            }
            Either::First(Err(PreflightFailure::Imu)) => {
                send_preflight_failure(completion_requested, "IMU not responding").await;
                return false;
            }
            Either::Second(kind) => {
                self.handle_interrupt(kind).await;
                api::send_completion(
                    completion_requested,
                    DriveCompletion {
                        status: CompletionStatus::Cancelled,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
                return false;
            }
        }

        if let Err(refusal) = preflight_gate() {
            motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            motion::set_track_speeds(0, 0).await;
            send_preflight_failure(completion_requested, refusal.reason()).await;
            return false;
        }
        true
    }

    /// Wake motor drivers from standby if the action is a movement command.
    async fn wake_from_standby(&mut self, action: &DriveAction) {
        if !self.standby_enabled {
            return;
        }
        match action {
            DriveAction::Differential { .. }
            | DriveAction::RotateExact { .. }
            | DriveAction::DriveDistance { .. }
            | DriveAction::DriveContinuous { .. } => {
                motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
                self.standby_enabled = false;
                Timer::after(Duration::from_millis(100)).await;
            }
            _ => {}
        }
    }

    // ── Per-action handlers ────────────────────────────────────────────────

    /// Handle a `Differential` command — fire-and-forget passthrough.
    async fn handle_differential(&self, left: i8, right: i8) {
        let (left_adjusted, right_adjusted) = differential::set_speeds(left, right);

        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: left_adjusted,
            right_speed: right_adjusted,
        })
        .await;
        motion::set_track_speeds(left_adjusted, right_adjusted).await;
    }

    /// Handle a `RotateExact` command — builds the `drive-core` intent.
    async fn handle_rotate_exact(
        &mut self,
        degrees: f32,
        direction: types::RotationDirection,
        motion: types::RotationMotion,
        completion_requested: bool,
    ) {
        let intent = rotation::init(degrees, direction, motion, completion_requested).await;
        self.active_intent = Some(intent);
    }

    /// Handle a `DriveDistance` command — builds the `drive-core` intent.
    async fn handle_drive_distance(
        &mut self,
        kind: types::DriveDistanceKind,
        direction: types::DriveDirection,
        speed: u8,
        completion_requested: bool,
    ) {
        if let Some(intent) = distance::init(kind, direction, speed, completion_requested).await {
            self.active_intent = Some(intent);
        }
    }

    /// Handle a `DriveContinuous` command — builds the `drive-core` intent.
    async fn handle_drive_continuous(
        &mut self,
        direction: types::DriveDirection,
        speed: u8,
        completion_requested: bool,
    ) {
        self.active_intent = Some(distance::init_continuous(direction, speed, completion_requested).await);
    }

    /// Handle a `Coast` command — coasts through the `drive-core` intent.
    async fn handle_coast(&mut self, completion_requested: bool) {
        info!("coast");
        self.active_intent = Some(brake_coast::init_coast(completion_requested).await);
    }

    /// Handle a `Brake` command — brakes through the `drive-core` intent.
    async fn handle_brake(&mut self, completion_requested: bool) {
        info!("brake");
        self.active_intent = Some(brake_coast::init_brake(completion_requested).await);
    }

    /// Handle an `Idle` command — coasts and waits through the `drive-core` intent.
    async fn handle_idle(&mut self, duration_ms: u64, completion_requested: bool) {
        // An explicit idle disarms the pair: the robot is parked, and the next
        // movement re-arms it through the Floor-Drop Gate.
        crate::task::sensors::rangefinder::disable().await;
        self.active_intent = Some(brake_coast::init_idle(duration_ms, completion_requested));
    }

    /// Handle a `Standby` command.
    async fn handle_standby(&mut self) {
        crate::task::sensors::rangefinder::disable().await;
        lifecycle::stop_encoder_sampling().await;

        if !self.standby_enabled {
            motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            Timer::after(Duration::from_millis(100)).await;
            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(100)).await;
            motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
            self.standby_enabled = true;
            motion::set_track_speeds(0, 0).await;
        }
    }

    /// Handle an interrupt — cancel active intent, stop sensors, bump epoch, drain queue.
    pub(super) async fn handle_interrupt(&mut self, kind: types::InterruptKind) {
        match kind {
            types::InterruptKind::EmergencyBrake => {
                motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            }
            types::InterruptKind::Stop | types::InterruptKind::CancelCurrent => {
                motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            }
        }

        let mut completion_sent = false;

        // Cancel and resolve the active intent using teardown descriptors.
        if let Some(intent) = self.active_intent.take() {
            let completion_requested = intent.completion_requested();
            let teardown = intent.teardown();
            let telemetry = intent.cancellation_telemetry();

            execute_intent_teardown(teardown).await;
            api::send_completion(
                completion_requested,
                DriveCompletion {
                    status: CompletionStatus::Cancelled,
                    telemetry,
                },
            )
            .await;

            if completion_requested {
                completion_sent = true;
            }
        }

        // Bump epoch to invalidate queued commands.
        api::CURRENT_EPOCH.fetch_add(1, core::sync::atomic::Ordering::Relaxed);

        // Drain queued commands.
        while let Ok(envelope) = api::DRIVE_QUEUE.receiver().try_receive() {
            if envelope.completion_requested {
                if completion_sent {
                    defmt::warn!("interrupt: additional completion-requested command dropped");
                } else {
                    api::send_completion(
                        true,
                        DriveCompletion {
                            status: CompletionStatus::Cancelled,
                            telemetry: types::CompletionTelemetry::None,
                        },
                    )
                    .await;
                    completion_sent = true;
                }
            }
        }

        motion::set_track_speeds(0, 0).await;
    }
}

// ── Teardown execution ─────────────────────────────────────────────────────

/// Execute the sensor teardown declared by a control module.
pub(super) async fn execute_intent_teardown(teardown: IntentTeardown) {
    match teardown {
        IntentTeardown::RotationImu => {
            lifecycle::stop_rotation_imu();
        }
        IntentTeardown::DistanceImuAndMotors => {
            // Stop motors first (safety: covers failure paths where the controller
            // returned early without stopping motors, e.g. encoder timeout/stall).
            motor_driver::send_motor_command(MotorCommand::SetTracks {
                left_speed: 0,
                right_speed: 0,
            })
            .await;
            motion::set_track_speeds(0, 0).await;
            lifecycle::stop_encoder_sampling().await;
            lifecycle::stop_distance_imu(&types::DriveDistanceKind::Straight { distance_cm: 0.0 });
        }
        IntentTeardown::EncoderSettle => {
            lifecycle::stop_encoder_sampling().await;
        }
        IntentTeardown::None => {}
    }
}

/// Send the completion for a failed pre-flight.
async fn send_preflight_failure(completion_requested: bool, reason: &'static str) {
    api::send_completion(
        completion_requested,
        DriveCompletion {
            status: CompletionStatus::Failed(reason),
            telemetry: types::CompletionTelemetry::None,
        },
    )
    .await;
}

/// Why the interruptible movement pre-flight stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PreflightFailure {
    /// The Floor-Drop Gate refused the action.
    Gate(GateRefusal),
    /// The IMU did not start streaming within its timeout.
    Imu,
}

/// Returns `true` if the drive action requires motion, and so both the
/// Floor-Drop Gate and the IMU pre-flight.
const fn action_requires_motion(action: &DriveAction) -> bool {
    matches!(
        action,
        DriveAction::Differential { .. }
            | DriveAction::DriveDistance { .. }
            | DriveAction::DriveContinuous { .. }
            | DriveAction::RotateExact { .. }
    )
}

/// Recheck the live Floor-Drop Gate after the interruptible pre-flight.
///
/// # Errors
///
/// Returns the [`GateRefusal`] the action reports when the pair is not armed or
/// a drop is already present.
fn preflight_gate() -> Result<(), GateRefusal> {
    gate(
        crate::task::sensors::rangefinder::status(),
        perception::is_floor_drop_detected(),
    )
}

/// Ensure the IMU is streaming stabilised data.
///
/// On first call (and after every IMU stop): starts the IMU with default fusion
/// mode, waits for the first DMP FIFO sample (1 s timeout), waits 150 ms for DMP
/// filter stabilisation, drains stale samples. Returns `true` if the IMU is ready;
/// `false` on timeout. If the IMU is already streaming, returns `true` immediately.
async fn ensure_imu_ready() -> bool {
    use embassy_time::{Duration, Instant, Timer};

    const IMU_STABILISE_MS: u64 = 150;
    const IMU_WAIT_TIMEOUT_MS: u64 = 1000;

    if crate::task::sensors::imu::IMU_READY.load(core::sync::atomic::Ordering::Relaxed) {
        return true;
    }

    lifecycle::start_distance_imu(&types::DriveDistanceKind::Straight { distance_cm: 0.0 });

    // Wait for the first sample.
    let deadline = Instant::now() + Duration::from_millis(IMU_WAIT_TIMEOUT_MS);
    let mut got_sample = false;
    while Instant::now() < deadline {
        while IMU_FEEDBACK_CHANNEL.receiver().try_receive().is_ok() {
            got_sample = true;
        }
        if got_sample {
            break;
        }
        Timer::after(Duration::from_millis(10)).await;
    }

    if !got_sample {
        lifecycle::stop_rotation_imu();
        return false;
    }

    // DMP filter stabilisation.
    Timer::after(Duration::from_millis(IMU_STABILISE_MS)).await;

    // Drain any stale samples accumulated during stabilise.
    while IMU_FEEDBACK_CHANNEL.receiver().try_receive().is_ok() {}

    true
}
