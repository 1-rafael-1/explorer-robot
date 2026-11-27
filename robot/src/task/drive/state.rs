//! Drive loop data structures.
//!
//! This module defines the core state types owned by the drive task's main loop.
//! It is deliberately free of behaviour — no command handling, no motor commands,
//! no sensor calls. That logic lives in [`super::dispatch`].
//!
//! # Intent lifecycle
//!
//! An intent is a state machine for one motion behaviour. It is created by a
//! control module's async `init()`, which owns sensor setup, polled by the intent
//! loop via the module's `run_step()`, and torn down via its
//! [`teardown()`](ActiveIntent::teardown) descriptor. The dispatch owns teardown
//! (stopping sensors on completion or interrupt); the control modules own the
//! runtime logic.
//!
//! Each intent carries:
//! - A controller half: the `drive-core` model for rotation, distance and
//!   continuous, or the brake/coast settle state.
//! - A completion flag — whether the caller expects a [`DriveCompletion`]
//! - A teardown descriptor — which sensors to stop on completion or interrupt
//! - Cancellation telemetry — what data to report if interrupted
//!
//! Commands that complete instantly (e.g. `Differential`) are **not** intents;
//! they are fire-and-forget, handled directly by the dispatch.
//!
//! # Types
//!
//! - [`ActiveIntent`]: the currently executing drive intent, together with its
//!   completion flag, teardown descriptor, and runtime state.
//! - [`DriveLoop`]: top-level state struct — holds `standby_enabled` and the
//!   active intent (if any).

use drive_core::Intent;
use embassy_time::Instant;

use super::brake_coast::BrakeCoastState;

/// The drive intent currently being executed by the drive task.
pub(super) enum ActiveIntent {
    /// An in-place or while-moving rotation towards a target angle.
    RotateExact {
        /// The `drive-core` model that owns the turn and correction control law.
        model: Intent,
        /// Whether this intent should emit a completion event when finished.
        completion_requested: bool,
        /// Wall-clock start time used for duration telemetry (milliseconds).
        started_at_ms: u64,
    },
    /// A drive to a fixed straight or curved distance.
    DriveDistance {
        /// The `drive-core` model that owns the distance control law.
        model: Intent,
        /// Whether this intent should emit a completion event when finished.
        completion_requested: bool,
    },
    /// A continuous drive: a speed and heading held until interrupted.
    DriveContinuous {
        /// The `drive-core` model that owns the continuous control law.
        model: Intent,
        /// Whether this intent should emit a completion event when finished.
        completion_requested: bool,
    },
    /// Active idle intent with a fixed duration.
    Idle {
        /// Duration to remain idle (milliseconds).
        duration_ms: u64,
        /// Wall-clock start time used for duration tracking (milliseconds).
        started_at_ms: u64,
        /// Whether this intent should emit a completion event when finished.
        completion_requested: bool,
    },
    /// Active brake/coast settle intent with encoder settle tracking.
    BrakeCoast {
        /// Brake/coast settle state.
        state: BrakeCoastState,
        /// Whether this intent should emit a completion event when finished.
        completion_requested: bool,
    },
}

/// Top-level state for the drive task's main control loop.
pub(super) struct DriveLoop {
    /// Whether the motor drivers are currently in low-power standby mode.
    pub(super) standby_enabled: bool,
    /// The intent currently being executed, if any.
    pub(super) active_intent: Option<ActiveIntent>,
}

impl DriveLoop {
    /// Create a new `DriveLoop` with standby enabled and no active intent.
    pub(super) const fn new() -> Self {
        Self {
            standby_enabled: true,
            active_intent: None,
        }
    }
}

impl ActiveIntent {
    /// Return the `IntentTeardown` descriptor for this active intent.
    pub(super) const fn teardown(&self) -> super::types::IntentTeardown {
        match self {
            Self::RotateExact { .. } => super::types::IntentTeardown::RotationImu,
            Self::DriveDistance { .. } | Self::DriveContinuous { .. } => {
                super::types::IntentTeardown::DistanceImuAndMotors
            }
            Self::Idle { .. } => super::types::IntentTeardown::None,
            Self::BrakeCoast { .. } => super::types::IntentTeardown::EncoderSettle,
        }
    }

    /// Returns true if this intent requested a completion event.
    pub(super) const fn completion_requested(&self) -> bool {
        match self {
            Self::RotateExact {
                completion_requested, ..
            }
            | Self::DriveDistance {
                completion_requested, ..
            }
            | Self::DriveContinuous {
                completion_requested, ..
            }
            | Self::Idle {
                completion_requested, ..
            }
            | Self::BrakeCoast {
                completion_requested, ..
            } => *completion_requested,
        }
    }

    /// Returns cancellation telemetry appropriate for this intent type.
    pub(super) fn cancellation_telemetry(&self) -> super::types::CompletionTelemetry {
        match self {
            Self::RotateExact { model, .. }
            | Self::DriveDistance { model, .. }
            | Self::DriveContinuous { model, .. } => model.cancellation_telemetry(Instant::now().as_millis()),
            // Idle and brake/coast have no progress telemetry to report.
            Self::Idle { .. } | Self::BrakeCoast { .. } => super::types::CompletionTelemetry::None,
        }
    }
}
