//! The hardware-independent drive control law and the vocabulary it speaks.
//!
//! `drive-core` owns the drive model: the shared command vocabulary, the
//! observation the adapter feeds in, the actions the model emits and the typed
//! terminal outcome. The drive task remains the adapter — it owns the sensor
//! lifecycle, timers, motor commands, the queue, the epoch, interrupts and
//! completion.
//!
//! It is `#![no_std]` and allocation-free, and depends on no HAL, no
//! `embassy-*` crate and no `defmt`. The model never reads a clock, starts a
//! sensor or commands a motor; every timestamp arrives with the observation.
//! The `std` feature exists only to enable the host integration tests under
//! `tests/`; the library never links `std`.
//!
//! Units: every duration and timestamp is in milliseconds, every encoder count
//! is the raw 16-bit hardware counter value, and every angle is in degrees.

#![no_std]
#![warn(missing_docs)]

mod brake_coast;
mod correction;
mod distance;
mod idle;
mod rotation;

// ── Drive vocabulary ──────────────────────────────────────────────────────────

/// The resolved measured-or-nominal encoder scale a distance drive consumes.
///
/// The adapter resolves it before constructing the intent, so the model never
/// reads calibration state.
pub use motor_cal::DistanceScale;

/// Drive direction for distance-based commands.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum DriveDirection {
    /// Forward motion
    Forward,
    /// Backward motion
    Backward,
}

/// Turn direction for curved motion.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum TurnDirection {
    /// Left turn (counter-clockwise yaw).
    Left,
    /// Right turn (clockwise yaw).
    Right,
}

/// Distance command specification
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DriveDistanceKind {
    /// Straight-line distance in centimeters.
    Straight {
        /// Target distance along the centerline (cm).
        distance_cm: f32,
    },
    /// Curved distance specified by centerline radius and arc length (centimeters).
    CurveArc {
        /// Radius from robot centerline (cm).
        radius_cm: f32,
        /// Desired arc length along the robot centerline (cm).
        arc_length_cm: f32,
        /// Turn direction (left/right).
        direction: TurnDirection,
    },
}

/// Rotation direction for precise turning
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum RotationDirection {
    /// Clockwise rotation (right turn)
    Clockwise,
    /// Counter-clockwise rotation (left turn)
    CounterClockwise,
}

/// Combined motion options during rotation
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum RotationMotion {
    /// Rotate in place at the given rotation speed (0-100)
    ///
    /// This is the "speed differential" input for turns-in-place:
    /// the controller will command left/right tracks as equal-and-opposite
    /// using this value as the magnitude.
    Stationary {
        /// Rotation speed (0-100)
        speed: u8,
    },

    /// Rotate while moving at specified speed (-100 to +100)
    /// Positive = forward, Negative = backward
    WhileMoving(i8),
}

/// Completion telemetry for long-running commands.
///
/// Used by per-command completions to return command-specific details.
#[derive(Debug, Clone, PartialEq)]
pub enum CompletionTelemetry {
    /// No telemetry payload.
    None,
    /// Telemetry for `RotateExact`.
    RotateExact {
        /// IMU yaw at completion (degrees).
        final_yaw_deg: f32,
        /// Signed error (degrees): achieved - target.
        angle_error_deg: f32,
        /// Duration in milliseconds.
        duration_ms: u64,
    },
    /// Telemetry for `DriveDistance`.
    DriveDistance {
        /// Achieved revolutions (left track sprocket).
        achieved_left_revs: f32,
        /// Achieved revolutions (right track sprocket).
        achieved_right_revs: f32,
        /// Raw encoder counts accumulated on the left track.
        achieved_left_counts: u32,
        /// Raw encoder counts accumulated on the right track.
        achieved_right_counts: u32,
        /// Target revolutions (left track sprocket).
        target_left_revs: f32,
        /// Target revolutions (right track sprocket).
        target_right_revs: f32,
        /// Duration in milliseconds.
        duration_ms: u64,
    },
}

// ── Observation ───────────────────────────────────────────────────────────────

/// One model observation: the adapter's clock and the latest sensor samples.
///
/// The adapter maps the encoder task's measurement and the IMU's yaw into this
/// type; the model does not depend on the sensor task types. Either sample is
/// absent until its sensor has produced one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    /// Monotonic time for this observation, in milliseconds.
    pub now_ms: u64,
    /// Latest absolute encoder counts, if the encoder task has produced one.
    pub encoder: Option<EncoderSample>,
    /// Latest yaw sample, if the IMU has produced one.
    pub yaw: Option<YawSample>,
}

/// The absolute counts of both encoder tracks at one instant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderSample {
    /// Left encoder count, as the 16-bit hardware counter reports it.
    pub left: u16,
    /// Right encoder count, as the 16-bit hardware counter reports it.
    pub right: u16,
    /// The sample's timestamp, in milliseconds.
    pub timestamp_ms: u64,
}

/// One yaw measurement from the IMU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct YawSample {
    /// Yaw angle in degrees.
    pub yaw_deg: f32,
    /// The sample's timestamp, in milliseconds.
    pub timestamp_ms: u64,
}

// ── Intent actions ────────────────────────────────────────────────────────────

/// One non-blocking action the adapter executes for an intent step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntentAction {
    /// Command both tracks to the given speeds (-100 to +100).
    SetTracks {
        /// Left track speed.
        left: i8,
        /// Right track speed.
        right: i8,
    },
    /// Brake both tracks electrically.
    BrakeAll,
    /// Let both tracks freewheel.
    CoastAll,
    /// Sleep until the absolute deadline, then step again with a fresh observation.
    WaitUntil {
        /// Absolute deadline in milliseconds, on the same clock as `Observation::now_ms`.
        deadline_ms: u64,
    },
    /// The intent is finished.
    Finished,
}

// ── Outcome and faults ────────────────────────────────────────────────────────

/// The terminal result of a drive intent.
#[derive(Debug, Clone, PartialEq)]
pub enum DriveOutcome {
    /// The intent reached its goal.
    Completed {
        /// Command-specific telemetry.
        telemetry: CompletionTelemetry,
    },
    /// The intent stopped on a terminal fault.
    Failed {
        /// The terminal cause.
        fault: DriveFault,
        /// Command-specific telemetry.
        telemetry: CompletionTelemetry,
    },
}

/// The typed terminal cause of a failed drive intent.
///
/// The adapter turns [`DriveFault::label`] into the firmware's completion
/// detail string; the labels are wire detail and must not change.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum DriveFault {
    /// No fresh encoder sample arrived inside the timeout.
    EncoderTimeout,
    /// Encoder timestamps advanced but neither track's counts moved.
    Stall,
    /// One track moved while the other stayed still.
    EncoderAnomaly,
    /// The rotation exceeded its overall time bound.
    RotationTimeout,
    /// The correction phase overshot the target more times than allowed.
    CorrectionLimit,
    /// One correction iteration ran past its timeout.
    CorrectionTimeout,
    /// Every correction iteration was used without reaching tolerance.
    CorrectionExhausted,
    /// The brake or coast command did not settle to rest inside the timeout.
    SettleTimeout,
}

// ── Intent parameters, steps and the in-flight intent ─────────────────────────

/// The construction parameters of one drive intent.
///
/// This is one enum so later tickets can add the brake/coast and idle variants
/// without changing [`Intent::new`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntentParams {
    /// Rotate in place to an exact relative angle.
    RotateExact {
        /// Relative angle to turn, in degrees (positive magnitude).
        degrees: f32,
        /// Sign of the turn.
        direction: RotationDirection,
        /// Whether the turn happens in place or while the tracks keep moving.
        motion: RotationMotion,
    },
    /// Drive a fixed distance, straight or along an arc.
    DriveDistance {
        /// Straight or curved distance specification.
        kind: DriveDistanceKind,
        /// Drive direction (forward or backward).
        direction: DriveDirection,
        /// Target speed magnitude (0-100).
        speed: u8,
        /// The resolved measured-or-nominal encoder scale.
        scale: DistanceScale,
    },
    /// Drive at a speed and hold a heading until interrupted.
    ///
    /// There is no distance target, no ramp-down and no successful completion;
    /// it ends only on a fault or a cancellation.
    DriveContinuous {
        /// Drive direction (forward or backward).
        direction: DriveDirection,
        /// Speed magnitude (0-100).
        speed: u8,
    },
    /// Brake both tracks electrically and wait for the encoder to settle.
    ///
    /// Selects the `BrakeCoast` intent state, as does [`Self::Coast`].
    Brake,
    /// Let both tracks freewheel and wait for the encoder to settle.
    ///
    /// Selects the `BrakeCoast` intent state, as does [`Self::Brake`].
    Coast,
    /// Park the robot for a fixed duration; zero completes immediately.
    Idle {
        /// How long to stay idle, in milliseconds.
        duration_ms: u64,
    },
}

/// A minimal, feature-gated snapshot of the decision behind one [`Step`].
///
/// The model never logs: when the `trace` feature is on it fills this in and the
/// adapter decides what to do with it. When the feature is off, [`Step::trace`]
/// is `None` and the model pays nothing for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Trace {
    /// Last yaw used by the model, if a sample has been seen.
    pub yaw_deg: Option<f32>,
    /// Absolute target yaw, once the first sample has anchored it.
    pub target_yaw_deg: Option<f32>,
    /// Signed remaining angle to the target, in degrees.
    pub remaining_deg: f32,
    /// Left track speed when the action is `SetTracks`.
    pub left: Option<i8>,
    /// Right track speed when the action is `SetTracks`.
    pub right: Option<i8>,
    /// Absolute deadline when the action is `WaitUntil`.
    pub deadline_ms: Option<u64>,
}

/// The result of one [`Intent::step`] call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    /// The one action the adapter executes before stepping again.
    pub action: IntentAction,
    /// The decision snapshot; `None` unless the `trace` feature is on.
    pub trace: Option<Trace>,
}

/// An in-flight drive intent.
///
/// The adapter constructs it with [`Intent::new`], repeatedly calls
/// [`Intent::step`] with a fresh [`Observation`], executes each returned
/// action and stops when the action is [`IntentAction::Finished`]. The intents
/// are private state; the adapter never matches on them.
pub struct Intent {
    /// The private intent state.
    state: IntentState,
}

/// The private intent states.
enum IntentState {
    /// A precise rotation turn.
    RotateExact(rotation::RotationState),
    /// A drive to a fixed straight or curved distance.
    DriveDistance(distance::DistanceState),
    /// A drive held at a speed and heading until interrupted.
    DriveContinuous(distance::DistanceState),
    /// A brake or coast settle.
    BrakeCoast(brake_coast::BrakeCoastState),
    /// A fixed-duration park.
    Idle(idle::IdleState),
}

impl Intent {
    /// Construct an intent from its parameters, starting at `now_ms`.
    ///
    /// Pure: it stores the start time and emits nothing. The first
    /// [`Intent::step`] emits the intent's initial action.
    #[must_use]
    pub const fn new(params: IntentParams, now_ms: u64) -> Self {
        let state = match params {
            IntentParams::RotateExact {
                degrees,
                direction,
                motion,
            } => IntentState::RotateExact(rotation::RotationState::new(degrees, direction, motion, now_ms)),
            IntentParams::DriveDistance {
                kind,
                direction,
                speed,
                scale,
            } => IntentState::DriveDistance(distance::DistanceState::new_target(
                kind, direction, speed, scale, now_ms,
            )),
            IntentParams::DriveContinuous { direction, speed } => {
                IntentState::DriveContinuous(distance::DistanceState::new_continuous(direction, speed, now_ms))
            }
            IntentParams::Brake => {
                IntentState::BrakeCoast(brake_coast::BrakeCoastState::new(brake_coast::Action::Brake, now_ms))
            }
            IntentParams::Coast => {
                IntentState::BrakeCoast(brake_coast::BrakeCoastState::new(brake_coast::Action::Coast, now_ms))
            }
            IntentParams::Idle { duration_ms } => IntentState::Idle(idle::IdleState::new(duration_ms, now_ms)),
        };
        Self { state }
    }

    /// Advance the intent by one action.
    ///
    /// The adapter executes a non-blocking action (anything but `WaitUntil` and
    /// `Finished`) and calls `step` again immediately. A `WaitUntil` sleeps to
    /// the absolute deadline and then calls `step` with a fresh observation.
    #[must_use]
    pub fn step(&mut self, obs: &Observation) -> Step {
        match &mut self.state {
            IntentState::RotateExact(state) => state.step(obs),
            IntentState::DriveDistance(state) | IntentState::DriveContinuous(state) => state.step(obs),
            IntentState::BrakeCoast(state) => state.step(obs),
            IntentState::Idle(state) => state.step(obs),
        }
    }

    /// The terminal outcome, present once the `Finished` action has been emitted.
    #[must_use]
    pub fn outcome(&self) -> Option<DriveOutcome> {
        match &self.state {
            IntentState::RotateExact(state) => state.outcome(),
            IntentState::DriveDistance(state) | IntentState::DriveContinuous(state) => state.outcome(),
            IntentState::BrakeCoast(state) => state.outcome(),
            IntentState::Idle(state) => state.outcome(),
        }
    }

    /// Telemetry for the adapter to report if it cancels the intent at `now_ms`.
    ///
    /// `Cancelled` is never model output: the adapter constructs it on an
    /// interrupt and asks the model for this snapshot.
    #[must_use]
    pub fn cancellation_telemetry(&self, now_ms: u64) -> CompletionTelemetry {
        match &self.state {
            IntentState::RotateExact(state) => state.cancellation_telemetry(now_ms),
            IntentState::DriveDistance(state) | IntentState::DriveContinuous(state) => {
                state.cancellation_telemetry(now_ms)
            }
            IntentState::BrakeCoast(state) => state.cancellation_telemetry(now_ms),
            IntentState::Idle(state) => state.cancellation_telemetry(now_ms),
        }
    }
}

impl DriveFault {
    /// The exact operator-facing completion detail for this cause.
    ///
    /// These strings are wire detail and are preserved byte-for-byte: the first
    /// seven are unchanged and the settle timeout is lowercase.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::EncoderTimeout => "EncoderTimeout",
            Self::Stall => "StallTimeout",
            Self::EncoderAnomaly => "EncoderAnomaly",
            Self::RotationTimeout => "RotateTimeout",
            Self::CorrectionLimit => "RotateCorrectionLimit",
            Self::CorrectionTimeout => "RotateCorrectionTimeout",
            Self::CorrectionExhausted => "CorrectionExhausted",
            Self::SettleTimeout => "encoder settle timeout",
        }
    }
}
