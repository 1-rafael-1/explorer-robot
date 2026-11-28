//! Shared types and geometry constants for drive control.
//!
//! The hardware-independent drive vocabulary lives in `drive-core` and is
//! re-exported here, so drive code keeps one import path. This module keeps the
//! firmware-owned command envelope and completion plumbing — the command and
//! action types, the queue completion and error types — and the geometry
//! constants. Per-module control parameters (rotation tuning, distance tuning,
//! drift tuning, etc.) live in their respective control modules.
//!
//! Also defines the [`IntentTeardown`] descriptor enum that forms the
//! seam between the [`super::dispatch`] and the control modules.

// ── Shared vocabulary and geometry ───────────────────────────────────────

// The hardware-independent vocabulary lives in `drive-core`; the physical
// geometry lives in `drive-geometry`. Both are re-exported here so drive code
// keeps one import path.
pub use drive_core::{
    CompletionTelemetry, DriveDirection, DriveDistanceKind, RotationDirection, RotationMotion, TurnDirection,
};
pub use drive_geometry::SPROCKET_CIRCUMFERENCE_CM;

/// Combined motor control and sensor feedback commands
///
/// The wrapper stays even with a single variant: the drive queue and the command
/// envelope speak one command type.
#[derive(Debug, Clone)]
pub enum DriveCommand {
    /// Movement and control commands
    Drive(DriveAction),
}

/// Motion control commands with associated parameters.
#[derive(Debug, Clone, PartialEq)]
pub enum DriveAction {
    /// Set motor speeds directly using `motor_driver` convention
    ///
    /// Speed range: -100 (full reverse) to +100 (full forward)
    /// - Positive values: Forward motion
    /// - Negative values: Reverse motion
    /// - Zero: Coast (freewheel)
    ///
    /// This is intended for direct teleoperation or simple commands without feedback control.
    #[allow(dead_code)]
    Differential {
        /// Left track speed (-100 to +100)
        left: i8,
        /// Right track speed (-100 to +100)
        right: i8,
    },
    /// Drive a fixed distance using encoder feedback and drift compensation.
    DriveDistance {
        /// Straight or curved distance specification.
        kind: DriveDistanceKind,
        /// Drive direction (forward or reverse).
        direction: DriveDirection,
        /// Target speed magnitude (0-100).
        speed: u8,
    },
    /// Drive at a speed and hold a heading until interrupted.
    ///
    /// This is the straight controller without a target: there is no distance
    /// to reach, no ramp-down and no successful completion. It ends `Cancelled`
    /// on any drive interrupt, or `Failed` on a stall, encoder timeout/loss or
    /// IMU loss.
    DriveContinuous {
        /// Drive direction (forward or reverse).
        direction: DriveDirection,
        /// Speed magnitude (0-100).
        speed: u8,
    },
    /// Active electrical braking.
    ///
    /// Completion is reported only after encoder counts settle.
    Brake,
    /// Passive stop (freewheeling).
    ///
    /// Completion is reported only after encoder counts settle.
    Coast,
    /// Idle in place for a fixed duration.
    ///
    /// This issues a coast (freewheel) command, does not change standby state,
    /// and completes after the requested duration. Use it to pause between
    /// queued actions without braking.
    Idle {
        /// Duration to remain idle (milliseconds).
        duration_ms: u64,
    },
    /// Enter low-power standby mode.
    Standby,
    /// Precise rotation with optional forward/reverse motion
    RotateExact {
        /// Rotation angle in degrees
        degrees: f32,
        /// Rotation direction
        direction: RotationDirection,
        /// Combined motion type
        motion: RotationMotion,
    },
}

/// Interrupt commands that preempt active drive intents.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum InterruptKind {
    /// Immediately brake all motors.
    EmergencyBrake,
    /// Stop motors without braking.
    Stop,
    /// Cancel the active intent without issuing a stop/brake.
    CancelCurrent,
}

/// Completion status for long-running commands.
///
/// Used by per-command completions to report final outcomes.
#[derive(Debug, Clone, Eq, PartialEq)]
pub enum CompletionStatus {
    /// Command finished successfully.
    Success,
    /// Command was cancelled by an interrupt or newer epoch.
    Cancelled,
    /// Command failed with a reason string.
    Failed(&'static str),
}

/// Generic completion payload for drive commands.
#[derive(Debug, Clone, PartialEq)]
pub struct DriveCompletion {
    /// Overall status.
    pub status: CompletionStatus,
    /// Command-specific telemetry.
    pub telemetry: CompletionTelemetry,
}

/// Queue completion payload for multi-step drive queues.
#[derive(Debug, Clone, PartialEq)]
pub struct DriveQueueCompletion {
    /// Overall queue status.
    pub status: CompletionStatus,
    /// Index of the step that failed or was cancelled (0-based).
    pub failed_step_index: Option<usize>,
    /// Number of steps completed successfully before termination.
    pub completed_steps: usize,
    /// The last step's completion payload, if any steps were executed.
    pub last_step_completion: Option<DriveCompletion>,
}

/// Error returned when a queue builder exceeds capacity.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum DriveQueueBuildError {
    /// The queue is full.
    Full,
}

impl DriveQueueBuildError {
    /// A short, operator-facing description for the running screen.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Full => "Queue full",
        }
    }
}

/// Error returned when submitting a queue for execution.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum DriveQueueSubmitError {
    /// Another queue is already active.
    QueueBusy,
}

impl DriveQueueSubmitError {
    /// A short, operator-facing description for the running screen.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::QueueBusy => "Drive busy",
        }
    }
}

// ── Sensor teardown descriptors ───────────────────────────────────────────

/// Sensor teardown required when an intent completes or is cancelled.
/// Declared by the control module, executed by the dispatch.
#[derive(Debug, Clone, Copy)]
pub(super) enum IntentTeardown {
    /// Stop rotation IMU streaming.
    RotationImu,
    /// Stop distance IMU + motors.
    DistanceImuAndMotors,
    /// Stop encoder sampling.
    EncoderSettle,
    /// No teardown needed.
    None,
}
