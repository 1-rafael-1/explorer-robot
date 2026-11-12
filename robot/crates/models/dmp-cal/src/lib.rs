//! Pure model for the DMP Compass Calibration (ADR-0045).
//!
//! This crate owns the hardware-independent rules behind the `DMP Mag`
//! procedure: the eight [`CompassStep`]s, the DMP-yaw to [`compass_heading`]
//! conversion, the two-target [`CompassCheck`], the per-step [`MotionHint`]s,
//! and the [`DmpBiasSnapshot`] encoding and validation.
//!
//! It is `#![no_std]` and depends on no HAL, IMU, flash or UI code, so the
//! firmware and the host tests exercise the same logic. The `std` feature
//! exists only to enable the host integration tests; the library never links
//! `std`.
//!
//! Units: every angle is in degrees and every duration is in milliseconds.
//! Timestamps are caller-supplied monotonic milliseconds. The model never reads
//! a clock, so a step or a check behaves identically on the robot and on the
//! host.
//!
//! # One home for the conversion
//!
//! [`compass_heading`] is the only place the mounting/sign conversion from DMP
//! yaw to a magnetic Compass Heading exists. Call sites must not apply another
//! offset. The result is magnetic north, not true north: no declination is
//! applied anywhere in this crate.

#![no_std]
#![warn(missing_docs)]

/// Magnetic north, in degrees: the first [`CompassCheck`] target.
pub const MAGNETIC_NORTH_DEG: f32 = 0.0;

/// Magnetic east, in degrees: the second [`CompassCheck`] target.
pub const MAGNETIC_EAST_DEG: f32 = 90.0;

/// Acceptance tolerance around a target heading, in degrees (inclusive).
pub const CHECK_TOLERANCE_DEG: f32 = 10.0;

/// Largest heading span that counts as stable, in degrees (inclusive).
pub const CHECK_STABILITY_SPAN_DEG: f32 = 2.0;

/// Length of the stability window, in milliseconds.
pub const CHECK_STABILITY_WINDOW_MS: u64 = 2000;

/// The DMP Compass Accuracy a [`CompassCheck`] requires, on the DMP's `0..=3`
/// scale where `3` is the most accurate.
pub const CALIBRATED_COMPASS_ACCURACY: u8 = 3;

/// Angular change above which a hold step no longer counts as still, in
/// degrees.
///
/// This only shapes the informational [`MotionHint`]; it never gates `Done`.
pub const STILLNESS_EPSILON_DEG: f32 = 0.5;

/// The DMP yaw, in degrees, that points the robot's front at magnetic north.
///
/// [`compass_heading`] subtracts this single constant, so it carries both the
/// mounting rotation and the sign convention. Do not apply another offset at a
/// call site.
///
/// The value is bench-derived and is deliberately the identity (`0.0`) until it
/// is measured on the robot. It must be confirmed on the bench before the
/// displayed heading is trusted.
pub const MOUNTING_OFFSET_DEG: f32 = 0.0;

/// Format version of an encoded [`DmpBiasSnapshot`].
pub const DMP_BIAS_FORMAT_VERSION: u8 = 1;

/// Encoded length of a [`DmpBiasSnapshot`], in bytes.
///
/// Layout: one version byte, nine big-endian `i32` words (gyroscope,
/// accelerometer, compass), then a two-byte big-endian checksum.
pub const DMP_BIAS_ENCODED_LEN: usize = 1 + (9 * 4) + 2;

/// The kind of motion a [`CompassStep`] asks for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CompassStepKind {
    /// Spin about the vertical axis.
    Spin,
    /// Tilt about a horizontal axis.
    Tilt,
    /// Hold the robot still.
    Hold,
    /// A final Compass Check at a known magnetic heading.
    Check,
}

/// One instructed movement in a DMP Compass Calibration.
///
/// The eight steps exist in this order:
///
/// 1. [`CompassStep::FlatHold`] — place flat, hold still.
/// 2. [`CompassStep::FlatSpin`] — spin 360° both ways on the table.
/// 3. [`CompassStep::NoseTilt`] — tilt nose up and down 90°.
/// 4. [`CompassStep::SideTilt`] — tilt left and right 90°.
/// 5. [`CompassStep::InvertedSpin`] — flip wheels up, spin 360° both ways.
/// 6. [`CompassStep::FlatHoldBeforeCheck`] — place flat, hold still.
/// 7. [`CompassStep::NorthCheck`] — point at magnetic north.
/// 8. [`CompassStep::EastCheck`] — point at magnetic east.
///
/// The first six accept `Done`; the final two accept `Done` or `Redo`. Use
/// [`CompassCalibration`] for advancement and [`CompassStep::redo_allowed`] for
/// the action pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CompassStep {
    /// First prompt: place the robot flat and hold still.
    FlatHold,
    /// Second prompt: spin 360° both ways with the robot on the table.
    FlatSpin,
    /// Third prompt: tilt the nose up and down 90°.
    NoseTilt,
    /// Fourth prompt: tilt left and right 90°.
    SideTilt,
    /// Fifth prompt: flip the wheels up and spin 360° both ways.
    InvertedSpin,
    /// Sixth prompt: place flat and hold still before the compass checks.
    FlatHoldBeforeCheck,
    /// Seventh prompt: point the front at magnetic north (0°).
    NorthCheck,
    /// Eighth prompt: point the front at magnetic east (90°).
    EastCheck,
}

impl CompassStep {
    /// All eight steps in their required order.
    pub const ALL: [Self; 8] = [
        Self::FlatHold,
        Self::FlatSpin,
        Self::NoseTilt,
        Self::SideTilt,
        Self::InvertedSpin,
        Self::FlatHoldBeforeCheck,
        Self::NorthCheck,
        Self::EastCheck,
    ];

    /// The step's zero-based position in [`CompassStep::ALL`].
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::FlatHold => 0,
            Self::FlatSpin => 1,
            Self::NoseTilt => 2,
            Self::SideTilt => 3,
            Self::InvertedSpin => 4,
            Self::FlatHoldBeforeCheck => 5,
            Self::NorthCheck => 6,
            Self::EastCheck => 7,
        }
    }

    /// The next step, or [`None`] after the final check.
    #[must_use]
    pub const fn next(self) -> Option<Self> {
        let index = self.index() + 1;
        if index < Self::ALL.len() {
            Some(Self::ALL[index])
        } else {
            None
        }
    }

    /// The user-facing instruction for this step.
    ///
    /// The strings are the ones the operator sees on the Prompt screen; keep
    /// them stable so the panel and the telemetry identity stay in step.
    #[must_use]
    pub const fn instruction(self) -> &'static str {
        match self {
            Self::FlatHold | Self::FlatHoldBeforeCheck => "Place flat. Hold still.",
            Self::FlatSpin => "On the table, spin 360° both ways.",
            Self::NoseTilt => "Tilt nose up and down 90°.",
            Self::SideTilt => "Tilt left and right 90°.",
            Self::InvertedSpin => "Flip wheels up, spin 360° both ways.",
            Self::NorthCheck => "Point front at magnetic north (0°). Hold still.",
            Self::EastCheck => "Point front at magnetic east (90°). Hold still.",
        }
    }

    /// The kind of motion this step asks for.
    #[must_use]
    pub const fn kind(self) -> CompassStepKind {
        match self {
            Self::FlatHold | Self::FlatHoldBeforeCheck => CompassStepKind::Hold,
            Self::FlatSpin | Self::InvertedSpin => CompassStepKind::Spin,
            Self::NoseTilt | Self::SideTilt => CompassStepKind::Tilt,
            Self::NorthCheck | Self::EastCheck => CompassStepKind::Check,
        }
    }

    /// Whether the step offers the `Redo` action as well as `Done`.
    ///
    /// Only the two final Compass Checks allow `Redo`.
    #[must_use]
    pub const fn redo_allowed(self) -> bool {
        matches!(self, Self::NorthCheck | Self::EastCheck)
    }

    /// The magnetic heading this check step targets, in degrees, or [`None`]
    /// for a movement step.
    #[must_use]
    pub const fn check_target_deg(self) -> Option<f32> {
        match self {
            Self::NorthCheck => Some(MAGNETIC_NORTH_DEG),
            Self::EastCheck => Some(MAGNETIC_EAST_DEG),
            Self::FlatHold
            | Self::FlatSpin
            | Self::NoseTilt
            | Self::SideTilt
            | Self::InvertedSpin
            | Self::FlatHoldBeforeCheck => None,
        }
    }
}

/// The informational hint for the current Compass Step.
///
/// Hints report whether the DMP is seeing the intended motion. They are
/// informational only: [`CompassCalibration::done`] is accepted regardless of
/// the hint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MotionHint {
    /// No hint for this step (a Compass Check or a completed procedure).
    None,
    /// Wrap-aware yaw span observed during the step, in degrees.
    YawSpanDeg(f32),
    /// Pitch span observed during the step, in degrees.
    PitchSpanDeg(f32),
    /// Roll span observed during the step, in degrees.
    RollSpanDeg(f32),
    /// How long the attitude has been still, in milliseconds.
    StillMillis(u64),
}

/// One attitude observation used to build a [`MotionHint`].
#[derive(Clone, Copy, Debug)]
pub struct MotionSample {
    /// Monotonic timestamp, in milliseconds.
    pub timestamp_ms: u64,
    /// DMP yaw, in degrees; any range, wrapped internally.
    pub yaw_deg: f32,
    /// DMP pitch, in degrees.
    pub pitch_deg: f32,
    /// DMP roll, in degrees.
    pub roll_deg: f32,
}

/// The Compass Step state machine and its informational motion hints.
///
/// [`CompassCalibration::new`] starts at [`CompassStep::FlatHold`].
/// [`CompassCalibration::done`] advances one step and
/// [`CompassCalibration::redo`] restarts at the first step from either check
/// step. Advancing past [`CompassStep::EastCheck`] completes the procedure and
/// leaves [`CompassCalibration::step`] returning [`None`]; there is no
/// off-the-end step.
///
/// Motion hints accumulate from [`CompassCalibration::observe`] and reset on
/// every step change and on `Redo`.
#[derive(Clone, Copy, Debug)]
pub struct CompassCalibration {
    /// Current step; `None` once the procedure is complete.
    step: Option<CompassStep>,
    /// Motion accumulated for the current step.
    tracker: MotionTracker,
}

impl CompassCalibration {
    /// Starts at the first Compass Step with no motion accumulated.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            step: Some(CompassStep::FlatHold),
            tracker: MotionTracker::new(),
        }
    }

    /// The current step, or [`None`] when the procedure is complete.
    #[must_use]
    pub const fn step(&self) -> Option<CompassStep> {
        self.step
    }

    /// Whether the final check has been accepted.
    #[must_use]
    pub const fn is_complete(&self) -> bool {
        self.step.is_none()
    }

    /// Accepts `Done`: advances one step, completing the procedure after the
    /// final check.
    ///
    /// Returns `false` when the procedure is already complete. The hint tracker
    /// is reset for the new step.
    pub const fn done(&mut self) -> bool {
        let Some(step) = self.step else {
            return false;
        };
        self.step = step.next();
        self.tracker.reset();
        true
    }

    /// Accepts `Redo`: returns to the first step.
    ///
    /// `Redo` is offered only by the two Compass Checks. Returns `true` when the
    /// procedure restarted, or `false` when the current step does not allow
    /// `Redo` (including after completion). The hint tracker is reset.
    pub const fn redo(&mut self) -> bool {
        let Some(step) = self.step else {
            return false;
        };
        if !step.redo_allowed() {
            return false;
        }
        self.step = Some(CompassStep::FlatHold);
        self.tracker.reset();
        true
    }

    /// Feeds one attitude sample into the current step's motion hint.
    ///
    /// Samples are ignored once the procedure is complete. Timestamps are
    /// expected to be monotonic; a non-monotonic sample still contributes to
    /// the span but cannot extend a still period.
    pub fn observe(&mut self, sample: MotionSample) {
        if self.step.is_some() {
            self.tracker.observe(sample);
        }
    }

    /// The hint value for the current step, as of the most recent sample.
    #[must_use]
    pub const fn hint(&self) -> MotionHint {
        match self.step {
            Some(step) => self.tracker.hint(step),
            None => MotionHint::None,
        }
    }
}

impl Default for CompassCalibration {
    /// Starts at the first Compass Step.
    fn default() -> Self {
        Self::new()
    }
}

/// Accumulated motion for the current step.
#[derive(Clone, Copy, Debug)]
struct MotionTracker {
    /// Whether this step has seen a sample yet.
    have_sample: bool,
    /// Previous yaw, in degrees.
    previous_yaw_deg: f32,
    /// Previous pitch, in degrees.
    previous_pitch_deg: f32,
    /// Previous roll, in degrees.
    previous_roll_deg: f32,
    /// Yaw accumulated as an unwrapped angle, in degrees.
    unwrapped_yaw_deg: f32,
    /// Minimum unwrapped yaw, in degrees.
    yaw_min_deg: f32,
    /// Maximum unwrapped yaw, in degrees.
    yaw_max_deg: f32,
    /// Minimum pitch, in degrees.
    pitch_min_deg: f32,
    /// Maximum pitch, in degrees.
    pitch_max_deg: f32,
    /// Minimum roll, in degrees.
    roll_min_deg: f32,
    /// Maximum roll, in degrees.
    roll_max_deg: f32,
    /// Timestamp of the most recent sample, in milliseconds.
    now_ms: u64,
    /// Timestamp at which the current still period began, in milliseconds.
    still_since_ms: u64,
}

impl MotionTracker {
    /// An empty tracker with zero spans.
    const fn new() -> Self {
        Self {
            have_sample: false,
            previous_yaw_deg: 0.0,
            previous_pitch_deg: 0.0,
            previous_roll_deg: 0.0,
            unwrapped_yaw_deg: 0.0,
            yaw_min_deg: 0.0,
            yaw_max_deg: 0.0,
            pitch_min_deg: 0.0,
            pitch_max_deg: 0.0,
            roll_min_deg: 0.0,
            roll_max_deg: 0.0,
            now_ms: 0,
            still_since_ms: 0,
        }
    }

    /// Clears every accumulator for a new step.
    const fn reset(&mut self) {
        *self = Self::new();
    }

    /// Adds one sample to the spans and the still timer.
    fn observe(&mut self, sample: MotionSample) {
        if !self.have_sample {
            self.have_sample = true;
            self.previous_yaw_deg = sample.yaw_deg;
            self.previous_pitch_deg = sample.pitch_deg;
            self.previous_roll_deg = sample.roll_deg;
            self.pitch_min_deg = sample.pitch_deg;
            self.pitch_max_deg = sample.pitch_deg;
            self.roll_min_deg = sample.roll_deg;
            self.roll_max_deg = sample.roll_deg;
            self.now_ms = sample.timestamp_ms;
            self.still_since_ms = sample.timestamp_ms;
            return;
        }

        // Yaw is compared through the short way round, so a spin that crosses
        // 0°/360° accumulates a small span rather than 359°.
        let yaw_delta = signed_delta_deg(sample.yaw_deg, self.previous_yaw_deg);
        let pitch_delta = (sample.pitch_deg - self.previous_pitch_deg).abs();
        let roll_delta = (sample.roll_deg - self.previous_roll_deg).abs();

        self.previous_yaw_deg = sample.yaw_deg;
        self.previous_pitch_deg = sample.pitch_deg;
        self.previous_roll_deg = sample.roll_deg;

        self.unwrapped_yaw_deg += yaw_delta;
        self.yaw_min_deg = self.yaw_min_deg.min(self.unwrapped_yaw_deg);
        self.yaw_max_deg = self.yaw_max_deg.max(self.unwrapped_yaw_deg);
        self.pitch_min_deg = self.pitch_min_deg.min(sample.pitch_deg);
        self.pitch_max_deg = self.pitch_max_deg.max(sample.pitch_deg);
        self.roll_min_deg = self.roll_min_deg.min(sample.roll_deg);
        self.roll_max_deg = self.roll_max_deg.max(sample.roll_deg);

        if yaw_delta.abs() > STILLNESS_EPSILON_DEG
            || pitch_delta > STILLNESS_EPSILON_DEG
            || roll_delta > STILLNESS_EPSILON_DEG
        {
            self.still_since_ms = sample.timestamp_ms;
        }
        self.now_ms = sample.timestamp_ms;
    }

    /// The hint for `step`, as of the most recent sample.
    const fn hint(&self, step: CompassStep) -> MotionHint {
        match step {
            CompassStep::FlatSpin | CompassStep::InvertedSpin => {
                MotionHint::YawSpanDeg(self.yaw_max_deg - self.yaw_min_deg)
            }
            CompassStep::NoseTilt => MotionHint::PitchSpanDeg(self.pitch_max_deg - self.pitch_min_deg),
            CompassStep::SideTilt => MotionHint::RollSpanDeg(self.roll_max_deg - self.roll_min_deg),
            CompassStep::FlatHold | CompassStep::FlatHoldBeforeCheck => {
                if self.have_sample {
                    MotionHint::StillMillis(self.now_ms.saturating_sub(self.still_since_ms))
                } else {
                    MotionHint::StillMillis(0)
                }
            }
            CompassStep::NorthCheck | CompassStep::EastCheck => MotionHint::None,
        }
    }
}

/// One Compass Check target.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CompassTarget {
    /// Magnetic north, [`MAGNETIC_NORTH_DEG`].
    MagneticNorth,
    /// Magnetic east, [`MAGNETIC_EAST_DEG`].
    MagneticEast,
}

impl CompassTarget {
    /// The target Compass Heading, in degrees.
    #[must_use]
    pub const fn heading_deg(self) -> f32 {
        match self {
            Self::MagneticNorth => MAGNETIC_NORTH_DEG,
            Self::MagneticEast => MAGNETIC_EAST_DEG,
        }
    }

    /// The next target in the required order, or [`None`] after east.
    #[must_use]
    pub const fn next(self) -> Option<Self> {
        match self {
            Self::MagneticNorth => Some(Self::MagneticEast),
            Self::MagneticEast => None,
        }
    }
}

/// One timestamped Compass Check observation.
#[derive(Clone, Copy, Debug)]
pub struct CompassSample {
    /// Monotonic timestamp, in milliseconds.
    pub timestamp_ms: u64,
    /// Measured Compass Heading, in degrees; values outside `0..360` are
    /// wrapped on ingestion.
    pub heading_deg: f32,
    /// DMP Compass Accuracy, `0..=3`.
    pub accuracy: u8,
}

/// Why a [`CompassCheck`] is not passing.
///
/// These are transient evaluations, not latched failures: re-feeding samples
/// re-evaluates the check, so the UI can explain the current state of the
/// procedure.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CheckFailure {
    /// The window contains a sample whose Compass Accuracy is not
    /// [`CALIBRATED_COMPASS_ACCURACY`].
    Accuracy {
        /// The measured Compass Accuracy, `0..=3`.
        measured: u8,
    },
    /// The latest Compass Heading is outside [`CHECK_TOLERANCE_DEG`] of the
    /// target.
    Heading {
        /// The latest measured Compass Heading, in degrees.
        measured_deg: f32,
        /// The target Compass Heading, in degrees.
        target_deg: f32,
    },
}

/// The outcome of the current Compass Check target.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CheckOutcome {
    /// Not enough evidence yet, or the heading has not been held long enough.
    Waiting,
    /// The heading is moving: its span over the window exceeds
    /// [`CHECK_STABILITY_SPAN_DEG`].
    Unstable,
    /// The check is failing for a specific reason.
    Fail(CheckFailure),
    /// The heading is on target, accurate, and stable for the whole window.
    Pass,
}

/// One retained Compass Check sample.
#[derive(Clone, Copy, Debug)]
struct CheckSample {
    /// Timestamp, in milliseconds.
    timestamp_ms: u64,
    /// Compass Heading, in degrees in `0..360`.
    heading_deg: f32,
    /// DMP Compass Accuracy, `0..=3`.
    accuracy: u8,
}

impl CheckSample {
    /// A placeholder used to initialise the ring buffer.
    const EMPTY: Self = Self {
        timestamp_ms: 0,
        heading_deg: 0.0,
        accuracy: 0,
    };
}

/// Maximum number of samples a Compass Check retains.
///
/// At the firmware's 10 Hz prompt tick this covers 12.8 s, far more than the
/// 2 s window. A caller feeding faster than roughly 60 Hz can fill the ring
/// before the window is covered; the check then reports
/// [`CheckOutcome::Waiting`] rather than passing on a short window.
const CHECK_WINDOW_CAPACITY: usize = 128;

/// The two-target Compass Check at the end of a DMP Compass Calibration.
///
/// A check starts at [`CompassTarget::MagneticNorth`] with
/// [`CheckOutcome::Waiting`]. [`CompassCheck::sample`] evaluates the current
/// target; [`CompassCheck::accept`] moves from north to east only once the
/// current target is [`CheckOutcome::Pass`], and completes the check when east
/// passes. The targets are therefore always evaluated in order.
///
/// Passing requires, for the samples in the current target's window:
///
/// - every sample reports [`CALIBRATED_COMPASS_ACCURACY`];
/// - the latest heading is within [`CHECK_TOLERANCE_DEG`] of the target;
/// - the samples span at least [`CHECK_STABILITY_WINDOW_MS`]; and
/// - the wrap-aware heading span over that window is at most
///   [`CHECK_STABILITY_SPAN_DEG`].
///
/// The window is cleared when the check moves to east, so the two targets are
/// independent.
#[derive(Clone, Debug)]
pub struct CompassCheck {
    /// The target currently being evaluated.
    target: CompassTarget,
    /// The outcome of the most recent evaluation.
    outcome: CheckOutcome,
    /// Whether both targets have been accepted.
    complete: bool,
    /// Ring buffer of retained samples.
    entries: [CheckSample; CHECK_WINDOW_CAPACITY],
    /// Index of the oldest retained sample.
    head: usize,
    /// Number of retained samples.
    len: usize,
}

impl CompassCheck {
    /// Starts at magnetic north with no samples.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            target: CompassTarget::MagneticNorth,
            outcome: CheckOutcome::Waiting,
            complete: false,
            entries: [CheckSample::EMPTY; CHECK_WINDOW_CAPACITY],
            head: 0,
            len: 0,
        }
    }

    /// The target currently being evaluated.
    #[must_use]
    pub const fn target(&self) -> CompassTarget {
        self.target
    }

    /// The outcome of the most recent evaluation.
    #[must_use]
    pub const fn outcome(&self) -> CheckOutcome {
        self.outcome
    }

    /// Whether both targets have been accepted.
    #[must_use]
    pub const fn is_complete(&self) -> bool {
        self.complete
    }

    /// Feeds one sample and re-evaluates the current target.
    ///
    /// Returns the new outcome, also available from [`CompassCheck::outcome`].
    /// Samples with a timestamp older than the newest retained sample are
    /// ignored, and samples are ignored once the check is complete.
    pub fn sample(&mut self, sample: CompassSample) -> CheckOutcome {
        if self.complete {
            return self.outcome;
        }
        if self.len > 0 && sample.timestamp_ms < self.entry(self.len - 1).timestamp_ms {
            return self.outcome;
        }

        self.prune(sample.timestamp_ms);
        self.push(CheckSample {
            timestamp_ms: sample.timestamp_ms,
            heading_deg: wrap_360(sample.heading_deg),
            accuracy: sample.accuracy,
        });
        self.outcome = self.evaluate();
        self.outcome
    }

    /// Accepts the current target with `Done`.
    ///
    /// Moves from magnetic north to magnetic east when the north check passes,
    /// and completes the check when the east check passes. Returns `false` when
    /// the current target is not [`CheckOutcome::Pass`] or the check is already
    /// complete, so the operator cannot skip a target or accept a bad result.
    pub const fn accept(&mut self) -> bool {
        if self.complete || !matches!(self.outcome, CheckOutcome::Pass) {
            return false;
        }
        match self.target {
            CompassTarget::MagneticNorth => {
                self.target = CompassTarget::MagneticEast;
                self.head = 0;
                self.len = 0;
                self.outcome = CheckOutcome::Waiting;
            }
            CompassTarget::MagneticEast => {
                self.complete = true;
            }
        }
        true
    }

    /// The sample at `offset` from the oldest retained sample.
    const fn entry(&self, offset: usize) -> CheckSample {
        self.entries[(self.head + offset) % CHECK_WINDOW_CAPACITY]
    }

    /// Drops samples strictly older than the stability window.
    const fn prune(&mut self, now_ms: u64) {
        while self.len > 0 {
            let oldest = self.entries[self.head];
            if now_ms.saturating_sub(oldest.timestamp_ms) > CHECK_STABILITY_WINDOW_MS {
                self.head = (self.head + 1) % CHECK_WINDOW_CAPACITY;
                self.len -= 1;
            } else {
                break;
            }
        }
    }

    /// Appends `sample`, evicting the oldest when the ring is full.
    const fn push(&mut self, sample: CheckSample) {
        if self.len == CHECK_WINDOW_CAPACITY {
            self.head = (self.head + 1) % CHECK_WINDOW_CAPACITY;
            self.len -= 1;
        }
        let index = (self.head + self.len) % CHECK_WINDOW_CAPACITY;
        self.entries[index] = sample;
        self.len += 1;
    }

    /// Evaluates the retained window against the current target.
    fn evaluate(&self) -> CheckOutcome {
        if self.len == 0 {
            return CheckOutcome::Waiting;
        }

        // Accuracy is required for the whole window: a momentarily
        // uncalibrated engine is not evidence of a converged one.
        for offset in 0..self.len {
            let sample = self.entry(offset);
            if sample.accuracy != CALIBRATED_COMPASS_ACCURACY {
                return CheckOutcome::Fail(CheckFailure::Accuracy {
                    measured: sample.accuracy,
                });
            }
        }

        let latest = self.entry(self.len - 1);
        let target_deg = self.target.heading_deg();
        if signed_delta_deg(latest.heading_deg, target_deg).abs() > CHECK_TOLERANCE_DEG {
            return CheckOutcome::Fail(CheckFailure::Heading {
                measured_deg: latest.heading_deg,
                target_deg,
            });
        }

        let oldest = self.entry(0);
        if latest.timestamp_ms.saturating_sub(oldest.timestamp_ms) < CHECK_STABILITY_WINDOW_MS {
            return CheckOutcome::Waiting;
        }

        if self.span_deg() > CHECK_STABILITY_SPAN_DEG {
            return CheckOutcome::Unstable;
        }

        CheckOutcome::Pass
    }

    /// The wrap-aware heading span of the retained window, in degrees.
    fn span_deg(&self) -> f32 {
        let reference = self.entry(0).heading_deg;
        let mut min = 0.0_f32;
        let mut max = 0.0_f32;
        for offset in 1..self.len {
            let delta = signed_delta_deg(self.entry(offset).heading_deg, reference);
            min = min.min(delta);
            max = max.max(delta);
        }
        max - min
    }
}

impl Default for CompassCheck {
    /// Starts at magnetic north with no samples.
    fn default() -> Self {
        Self::new()
    }
}

/// Converts DMP yaw to a magnetic Compass Heading in `0..360`.
///
/// This is the one place the mounting/sign conversion exists. It subtracts
/// [`MOUNTING_OFFSET_DEG`], wraps the result, and applies no declination: the
/// result is magnetic north, not true north. A non-finite input yields a
/// non-finite result.
#[must_use]
pub fn compass_heading(dmp_yaw_deg: f32) -> f32 {
    wrap_360(dmp_yaw_deg - MOUNTING_OFFSET_DEG)
}

/// Coerces an angle into `0..360`.
///
/// `f32` has no `rem_euclid` in `no_std`, so the wrap is explicit.
fn wrap_360(angle_deg: f32) -> f32 {
    let wrapped = angle_deg % 360.0;
    if wrapped < 0.0 { wrapped + 360.0 } else { wrapped }
}

/// The signed shortest rotation from `from_deg` to `to_deg`, in `-180..180`.
fn signed_delta_deg(to_deg: f32, from_deg: f32) -> f32 {
    wrap_360(to_deg - from_deg + 180.0) - 180.0
}

/// The nine raw DMP bias words saved after a successful calibration.
///
/// Each group holds three big-endian `i32` words exactly as the DMP reports
/// them: the gyroscope bias first, then the accelerometer bias, then the
/// compass bias. No host unit conversion is involved.
///
/// [`DmpBiasSnapshot::encode`] and [`DmpBiasSnapshot::decode`] are the only
/// supported serialisation; the encoded form carries a format version and a
/// checksum so flash data is validated rather than trusted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DmpBiasSnapshot {
    /// Raw gyroscope bias words.
    gyro: [i32; 3],
    /// Raw accelerometer bias words.
    accel: [i32; 3],
    /// Raw compass bias words.
    compass: [i32; 3],
}

impl DmpBiasSnapshot {
    /// Builds a snapshot from the nine raw bias words.
    #[must_use]
    pub const fn new(gyro_bias: [i32; 3], accel_bias: [i32; 3], compass_bias: [i32; 3]) -> Self {
        Self {
            gyro: gyro_bias,
            accel: accel_bias,
            compass: compass_bias,
        }
    }

    /// The raw gyroscope bias words.
    #[must_use]
    pub const fn gyro_bias(&self) -> [i32; 3] {
        self.gyro
    }

    /// The raw accelerometer bias words.
    #[must_use]
    pub const fn accel_bias(&self) -> [i32; 3] {
        self.accel
    }

    /// The raw compass bias words.
    #[must_use]
    pub const fn compass_bias(&self) -> [i32; 3] {
        self.compass
    }

    /// The format version this snapshot encodes with.
    #[must_use]
    pub const fn version(&self) -> u8 {
        DMP_BIAS_FORMAT_VERSION
    }

    /// The checksum this snapshot encodes with.
    ///
    /// It is a CRC-16/CCITT-FALSE over the version byte and the nine
    /// big-endian words.
    #[must_use]
    pub fn checksum(&self) -> u16 {
        let encoded = self.encode();
        u16::from_be_bytes([encoded[DMP_BIAS_ENCODED_LEN - 2], encoded[DMP_BIAS_ENCODED_LEN - 1]])
    }

    /// Encodes the snapshot into its fixed-size flash representation.
    #[must_use]
    pub fn encode(&self) -> [u8; DMP_BIAS_ENCODED_LEN] {
        let mut encoded = [0_u8; DMP_BIAS_ENCODED_LEN];
        encoded[0] = DMP_BIAS_FORMAT_VERSION;

        let mut offset = 1;
        for word in self.gyro.iter().chain(self.accel.iter()).chain(self.compass.iter()) {
            encoded[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
            offset += 4;
        }

        let checksum = crc16(&encoded[..DMP_BIAS_ENCODED_LEN - 2]);
        let end = DMP_BIAS_ENCODED_LEN;
        encoded[end - 2..end].copy_from_slice(&checksum.to_be_bytes());
        encoded
    }

    /// Decodes a snapshot from exactly [`DMP_BIAS_ENCODED_LEN`] bytes.
    ///
    /// # Errors
    ///
    /// Returns [`SnapshotError::Length`] for a wrong length (including a
    /// truncated value), [`SnapshotError::UnsupportedVersion`] for an
    /// unsupported [`DMP_BIAS_FORMAT_VERSION`], and
    /// [`SnapshotError::ChecksumMismatch`] for a corrupt payload. A rejected
    /// value is never partially decoded.
    pub fn decode(bytes: &[u8]) -> Result<Self, SnapshotError> {
        if bytes.len() != DMP_BIAS_ENCODED_LEN {
            return Err(SnapshotError::Length { actual: bytes.len() });
        }
        if bytes[0] != DMP_BIAS_FORMAT_VERSION {
            return Err(SnapshotError::UnsupportedVersion { found: bytes[0] });
        }

        let checksum_end = DMP_BIAS_ENCODED_LEN - 2;
        let expected = crc16(&bytes[..checksum_end]);
        let found = u16::from_be_bytes([bytes[checksum_end], bytes[checksum_end + 1]]);
        if expected != found {
            return Err(SnapshotError::ChecksumMismatch { expected, found });
        }

        Ok(Self {
            gyro: words_at(bytes, 0),
            accel: words_at(bytes, 3),
            compass: words_at(bytes, 6),
        })
    }
}

/// Reads three consecutive big-endian `i32` words starting at word `start`.
fn words_at(bytes: &[u8], start: usize) -> [i32; 3] {
    let mut words = [0_i32; 3];
    for (index, word) in words.iter_mut().enumerate() {
        let offset = 1 + ((start + index) * 4);
        *word = i32::from_be_bytes([bytes[offset], bytes[offset + 1], bytes[offset + 2], bytes[offset + 3]]);
    }
    words
}

/// CRC-16/CCITT-FALSE checksum over `bytes`.
///
/// The bitwise form needs no lookup table. It detects every single-byte change,
/// including the erased-flash `0x00` to `0xFF` case.
fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0xFFFF_u16;
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Why an encoded DMP Bias Snapshot was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SnapshotError {
    /// The input was not exactly [`DMP_BIAS_ENCODED_LEN`] bytes.
    Length {
        /// The actual length, in bytes.
        actual: usize,
    },
    /// The format version is not [`DMP_BIAS_FORMAT_VERSION`].
    UnsupportedVersion {
        /// The version found in the input.
        found: u8,
    },
    /// The stored checksum does not match the encoded payload.
    ChecksumMismatch {
        /// The checksum computed from the payload.
        expected: u16,
        /// The checksum stored in the input.
        found: u16,
    },
}
