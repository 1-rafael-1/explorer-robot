//! Pure model for one motor Power Step measurement and the Motor Calibration
//! Curve derived from a full sweep.
//!
//! This crate owns the hardware-independent rules behind a **Power Step**: the
//! stop, the reset and its confirmation, the settle interval, and the
//! **Measurement Window** that produces one trustworthy encoder count. It turns
//! a stream of timestamped encoder samples into a [`PointResult`], and when the
//! point is not trustworthy it names the reason with an [`InvalidReason`] so the
//! panel can show `--` and the log can say why.
//!
//! It also owns the curve maths: [`derive_curve`] turns the four **Test Legs** of
//! one sweep into a [`Curve`], and [`Curve::resolve`] applies the per-track,
//! per-direction correction that a `SetTracks` request needs. The curve is
//! derived from the same measurements the Basic Motor Test displays, so the
//! instrument and the calibration cannot drift apart.
//!
//! Two pieces carry a calibration off the bench. The [`scale`] module turns the
//! accumulated counts of a driven leg and the operator's measured distance into
//! the **Distance Calibration** scale — counts per centimetre, from which counts
//! per sprocket revolution follows. The [`payload`] module is the versioned,
//! checksummed record that stores the scale and the raw sweep, and recomputes the
//! curve from those raw measurements on load.
//!
//! It is `#![no_std]` and depends on no HAL, motor driver, flash or UI code. The
//! `std` feature exists only to enable the host integration tests under
//! `tests/`; the library never links `std`.
//!
//! Units: every duration and timestamp is in milliseconds, and every encoder
//! count is the raw 16-bit hardware counter value. Timestamps are caller-supplied
//! monotonic milliseconds; the model never reads a clock.
//!
//! # The seam
//!
//! The caller owns the hardware and drives the state machine through its public
//! API: [`MeasurementEngine::begin_step`] starts a Power Step,
//! [`MeasurementEngine::observe`] feeds timestamped samples, and
//! [`MeasurementEngine::phase`] tells the caller when to reset the counters
//! ([`MeasurementEngine::reset_now`]) and when to apply the command
//! ([`MeasurementEngine::command_applied`]). Once the phase is
//! [`Phase::Finished`], [`MeasurementEngine::result`] holds the verdict.
//!
//! # Why a delta, not an absolute count
//!
//! The count is always the wrap-safe delta between the timestamped samples at
//! each end of the Measurement Window, never the absolute counter value since
//! the reset. That removes the reset race and makes a 16-bit counter wrap
//! harmless.

#![no_std]
#![warn(missing_docs)]

pub mod apply;
pub mod curve_maths;
pub mod payload;
pub mod scale;
pub mod verification;

pub use apply::resolve_track_speed;
pub use curve_maths::{
    Curve, CurveCommand, CurveError, Direction, FIRST_POWER_STEP_PERCENT, LAST_POWER_STEP_PERCENT, LEG_ORDER, LegPoint,
    MAX_CORRECTION, MIN_PLAUSIBLE_CORRECTION, POWER_STEP_PERCENT, POWER_STEP_PERCENTS, POWER_STEPS, Sweep, TEST_LEGS,
    TestLeg, Track, derive_curve,
};
pub use payload::{
    DecodeError, EncodeError, MotorCalibrationRecord, RECORD_ENCODED_LEN, RECORD_FORMAT_VERSION, StepMeasurement,
};
pub use scale::{
    DistanceScale, INFERRED_COUNTS_PER_SPROCKET_REV, MAX_PLAUSIBLE_COUNTS_PER_CM, MIN_PLAUSIBLE_COUNTS_PER_CM,
    NOMINAL_COUNTS_PER_CM, SPROCKET_CIRCUMFERENCE_CM, ScaleError,
};
pub use verification::{
    VERIFICATION_INFO_POWER, VERIFICATION_MIN_POWER, VERIFICATION_REPEATS, Verification, VerificationMax,
    VerificationReport, divergence_percent, divergence_tenths,
};

/// The Quiescence hold window, in milliseconds: a track that has produced no
/// encoder pulses for this long is **Quiescent**.
pub const QUIESCENCE_HOLD_MS: u64 = 250;

/// The Quiescence timeout, in milliseconds: a Power Step that has not reached
/// Quiescence within this long is invalid.
pub const QUIESCENCE_TIMEOUT_MS: u64 = 2_000;

/// The settle interval, in milliseconds, discarded after the command is applied.
pub const SETTLE_MS: u64 = 500;

/// The Measurement Window length, in milliseconds.
pub const MEASUREMENT_WINDOW_MS: u64 = 2_000;

/// The length of one motion sub-window, in milliseconds.
pub const MOTION_SUB_WINDOW_MS: u64 = 500;

/// The minimum encoder count in each of two consecutive motion sub-windows for
/// the Measurement Window to show sustained motion.
pub const MOTION_THRESHOLD_COUNTS: u32 = 10;

/// The number of motion sub-windows inside one Measurement Window.
const SUB_WINDOWS: usize = 4;

/// One timestamped encoder count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncoderSample {
    /// The sample's timestamp, in milliseconds.
    pub timestamp_ms: u64,
    /// The encoder count, as the 16-bit hardware counter reports it.
    pub count: u16,
}

/// The phase of a [`MeasurementEngine`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// No Power Step has begun.
    Idle,
    /// Waiting for Quiescence: no encoder pulses for [`QUIESCENCE_HOLD_MS`].
    SeekingQuiescence,
    /// Quiescence is confirmed; the caller should reset the counters and then
    /// call [`MeasurementEngine::reset_now`].
    Quiescent,
    /// The counters were reset; waiting for a fresh sample at zero.
    ConfirmingReset,
    /// The reset is confirmed; the caller should apply the command and then
    /// call [`MeasurementEngine::command_applied`].
    ReadyToCommand,
    /// The command is applied; the settle interval is being discarded.
    Settling,
    /// The Measurement Window is in progress.
    Measuring,
    /// The Power Step finished; [`MeasurementEngine::result`] holds the verdict.
    Finished,
}

/// Why a Power Step is invalid: the point is shown as `--`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InvalidReason {
    /// No Quiescence was established within [`QUIESCENCE_TIMEOUT_MS`].
    QuiescenceTimeout,
    /// The counter reset was never confirmed by a fresh sample at zero.
    ResetUnconfirmed,
    /// A sample was taken before the reset, or arrived out of order.
    StaleSample,
    /// The step ended before a full Measurement Window was captured.
    NoWindow,
}

impl InvalidReason {
    /// A short, stable phrase naming the reason for an operator log.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::QuiescenceTimeout => "quiescence timeout",
            Self::ResetUnconfirmed => "reset unconfirmed",
            Self::StaleSample => "stale sample",
            Self::NoWindow => "no window",
        }
    }
}

/// One trustworthy Power Step measurement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Measurement {
    /// The wrap-safe encoder delta across the Measurement Window.
    count: u16,
    /// The timestamped span of the Measurement Window, in milliseconds.
    ///
    /// This is the real end-minus-start span of the samples that bounded the
    /// window, not the nominal [`MEASUREMENT_WINDOW_MS`]: a late end sample makes
    /// the window longer, and the speed is computed over that same span.
    window_ms: u64,
    /// The speed over the Measurement Window, in counts per second.
    speed_counts_per_s: u32,
    /// Whether the Measurement Window showed sustained motion.
    sustained_motion: bool,
}

impl Measurement {
    /// The wrap-safe encoder delta across the Measurement Window.
    #[must_use]
    pub const fn count(&self) -> u16 {
        self.count
    }

    /// The timestamped span of the Measurement Window, in milliseconds.
    ///
    /// A stored measurement uses this real span so its count and speed agree
    /// with the window that produced them, rather than a nominal length.
    #[must_use]
    pub const fn window_ms(&self) -> u64 {
        self.window_ms
    }

    /// The speed over the Measurement Window, in counts per second.
    #[must_use]
    pub const fn speed_counts_per_s(&self) -> u32 {
        self.speed_counts_per_s
    }

    /// Whether the Measurement Window showed sustained motion above the motion
    /// threshold in each of two consecutive sub-windows.
    #[must_use]
    pub const fn sustained_motion(&self) -> bool {
        self.sustained_motion
    }
}

/// The verdict on one Power Step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointResult {
    /// A trustworthy measurement.
    Valid(Measurement),
    /// The point is `--` for this reason.
    Invalid(InvalidReason),
}

impl PointResult {
    /// The valid measurement, if the point is valid.
    #[must_use]
    pub const fn measurement(self) -> Option<Measurement> {
        match self {
            Self::Valid(measurement) => Some(measurement),
            Self::Invalid(_) => None,
        }
    }
}

/// The measurement state machine for one Power Step.
///
/// Drive it with [`begin_step`](Self::begin_step),
/// [`observe`](Self::observe), [`reset_now`](Self::reset_now),
/// [`command_applied`](Self::command_applied) and [`finish`](Self::finish);
/// read [`phase`](Self::phase) to learn which caller action the next transition
/// needs, and [`result`](Self::result) once the phase is [`Phase::Finished`].
#[derive(Clone, Copy, Debug)]
pub struct MeasurementEngine {
    /// The current phase.
    phase: Phase,
    /// When the Power Step began.
    step_started_ms: u64,
    /// The timestamp of the last accepted sample.
    last_sample_ms: Option<u64>,
    /// Whether any sample has been accepted for this Power Step.
    seen_sample: bool,
    /// The count of the last accepted sample.
    last_count: u16,
    /// When the count last changed, or when the first sample arrived.
    last_change_ms: u64,
    /// When the counters were reset.
    reset_ms: u64,
    /// When the command was applied.
    command_ms: u64,
    /// The Measurement Window's start sample.
    window_start: Option<EncoderSample>,
    /// The previous sample, for the wrap-safe delta accumulation.
    previous: Option<EncoderSample>,
    /// The encoder counts in each motion sub-window.
    sub_windows: [u32; SUB_WINDOWS],
    /// The finished Power Step's verdict.
    result: Option<PointResult>,
}

impl MeasurementEngine {
    /// A fresh engine with no Power Step begun.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            phase: Phase::Idle,
            step_started_ms: 0,
            last_sample_ms: None,
            seen_sample: false,
            last_count: 0,
            last_change_ms: 0,
            reset_ms: 0,
            command_ms: 0,
            window_start: None,
            previous: None,
            sub_windows: [0; SUB_WINDOWS],
            result: None,
        }
    }

    /// The current phase.
    #[must_use]
    pub const fn phase(&self) -> Phase {
        self.phase
    }

    /// The finished Power Step's verdict, or `None` while it is still running.
    #[must_use]
    pub const fn result(&self) -> Option<PointResult> {
        self.result
    }

    /// Begin a new Power Step at `now_ms`, discarding any previous result.
    pub const fn begin_step(&mut self, now_ms: u64) {
        self.phase = Phase::SeekingQuiescence;
        self.step_started_ms = now_ms;
        self.last_sample_ms = None;
        self.seen_sample = false;
        self.last_count = 0;
        self.last_change_ms = now_ms;
        self.reset_ms = 0;
        self.command_ms = 0;
        self.window_start = None;
        self.previous = None;
        self.sub_windows = [0; SUB_WINDOWS];
        self.result = None;
    }

    /// Feed one timestamped encoder sample.
    ///
    /// Samples are ignored in [`Phase::Idle`] and [`Phase::Finished`]. A sample
    /// that arrives out of order, or one taken before the reset, invalidates the
    /// point with [`InvalidReason::StaleSample`].
    pub fn observe(&mut self, sample: EncoderSample) {
        if matches!(self.phase, Phase::Idle | Phase::Finished) {
            return;
        }
        if let Some(last) = self.last_sample_ms
            && sample.timestamp_ms < last
        {
            self.invalidate(InvalidReason::StaleSample);
            return;
        }
        self.last_sample_ms = Some(sample.timestamp_ms);
        match self.phase {
            Phase::SeekingQuiescence => self.observe_quiescence(sample),
            Phase::ConfirmingReset => self.observe_reset(sample),
            Phase::Settling | Phase::Measuring => self.observe_window(sample),
            Phase::Idle | Phase::Quiescent | Phase::ReadyToCommand | Phase::Finished => {}
        }
    }

    /// Record that the command was applied at `now_ms`.
    ///
    /// Only meaningful in [`Phase::ReadyToCommand`]; ignored in every other
    /// phase.
    pub const fn command_applied(&mut self, now_ms: u64) {
        if matches!(self.phase, Phase::ReadyToCommand) {
            self.command_ms = now_ms;
            self.phase = Phase::Settling;
        }
    }

    /// Record that the counters were reset at `now_ms`.
    ///
    /// Only meaningful in [`Phase::Quiescent`]; ignored in every other phase.
    pub const fn reset_now(&mut self, now_ms: u64) {
        if matches!(self.phase, Phase::Quiescent) {
            self.reset_ms = now_ms;
            self.phase = Phase::ConfirmingReset;
        }
    }

    /// Advance the Quiescence search with one sample.
    const fn observe_quiescence(&mut self, sample: EncoderSample) {
        let changed = !self.seen_sample || self.last_count != sample.count;
        if changed {
            self.seen_sample = true;
            self.last_count = sample.count;
            self.last_change_ms = sample.timestamp_ms;
        }
        if sample.timestamp_ms.saturating_sub(self.last_change_ms) >= QUIESCENCE_HOLD_MS {
            self.phase = Phase::Quiescent;
        } else if sample.timestamp_ms.saturating_sub(self.step_started_ms) >= QUIESCENCE_TIMEOUT_MS {
            self.invalidate(InvalidReason::QuiescenceTimeout);
        }
    }

    /// Confirm the reset with a fresh sample at zero.
    ///
    /// The confirming sample must not predate the reset instant. A sample
    /// strictly before it was taken before the reset and is reported as stale;
    /// a sample in the same millisecond is accepted because the caller records
    /// the reset instant before clearing the stale drive-side copy. A fresh
    /// sample that is not zero means the reset did not take.
    const fn observe_reset(&mut self, sample: EncoderSample) {
        if sample.timestamp_ms < self.reset_ms {
            self.invalidate(InvalidReason::StaleSample);
        } else if sample.count == 0 {
            self.phase = Phase::ReadyToCommand;
        } else {
            self.invalidate(InvalidReason::ResetUnconfirmed);
        }
    }

    /// Advance the settle interval and the Measurement Window.
    fn observe_window(&mut self, sample: EncoderSample) {
        if matches!(self.phase, Phase::Settling) {
            if sample.timestamp_ms < self.command_ms.saturating_add(SETTLE_MS) {
                return;
            }
            self.window_start = Some(sample);
            self.previous = Some(sample);
            self.phase = Phase::Measuring;
            return;
        }

        let Some(start) = self.window_start else {
            return;
        };
        let Some(previous) = self.previous else {
            return;
        };
        let slot = sub_window_slot(start.timestamp_ms, sample.timestamp_ms);
        let delta = u32::from(sample.count.wrapping_sub(previous.count));
        self.sub_windows[slot] = self.sub_windows[slot].saturating_add(delta);
        self.previous = Some(sample);

        if sample.timestamp_ms.saturating_sub(start.timestamp_ms) >= MEASUREMENT_WINDOW_MS {
            self.complete(start, sample);
        }
    }

    /// Finish the Measurement Window from its start and end samples.
    fn complete(&mut self, start: EncoderSample, end: EncoderSample) {
        let count = end.count.wrapping_sub(start.count);
        let window_ms = end.timestamp_ms.saturating_sub(start.timestamp_ms);
        let speed = u64::from(count).saturating_mul(1_000).checked_div(window_ms);
        let speed_counts_per_s = speed.map_or(0, |speed| u32::try_from(speed).unwrap_or(u32::MAX));
        let sustained_motion = self
            .sub_windows
            .windows(2)
            .any(|pair| pair[0] >= MOTION_THRESHOLD_COUNTS && pair[1] >= MOTION_THRESHOLD_COUNTS);
        self.result = Some(PointResult::Valid(Measurement {
            count,
            window_ms,
            speed_counts_per_s,
            sustained_motion,
        }));
        self.phase = Phase::Finished;
    }

    /// End the Power Step now, without waiting for the Measurement Window.
    ///
    /// A point that was not measured has no trustworthy count, so it is
    /// invalidated with the reason that explains what was missing:
    /// [`InvalidReason::QuiescenceTimeout`] before Quiescence,
    /// [`InvalidReason::ResetUnconfirmed`] before the reset is confirmed, and
    /// [`InvalidReason::NoWindow`] once the reset is confirmed but the window has
    /// not completed. A finished point is left unchanged.
    pub const fn finish(&mut self) {
        match self.phase {
            Phase::Idle | Phase::Finished => {}
            Phase::SeekingQuiescence => self.invalidate(InvalidReason::QuiescenceTimeout),
            Phase::Quiescent | Phase::ConfirmingReset => self.invalidate(InvalidReason::ResetUnconfirmed),
            Phase::ReadyToCommand | Phase::Settling | Phase::Measuring => {
                self.invalidate(InvalidReason::NoWindow);
            }
        }
    }

    /// End the Power Step now with `reason`, without waiting for the Measurement
    /// Window.
    ///
    /// This is for a runner that observes a condition the engine itself cannot —
    /// such as an untested track that is still moving — and must record the point
    /// as invalid with that verdict. A finished point is left unchanged, and a
    /// point that has not begun has nothing to abort.
    pub const fn abort(&mut self, reason: InvalidReason) {
        if !matches!(self.phase, Phase::Idle | Phase::Finished) {
            self.invalidate(reason);
        }
    }

    /// Finish the Power Step with an invalid verdict.
    const fn invalidate(&mut self, reason: InvalidReason) {
        self.result = Some(PointResult::Invalid(reason));
        self.phase = Phase::Finished;
    }
}

impl Default for MeasurementEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// The motion sub-window that contains `timestamp_ms`, clamped to the last one.
const fn sub_window_slot(start_ms: u64, timestamp_ms: u64) -> usize {
    let mut slot = 0;
    let mut boundary = start_ms.saturating_add(MOTION_SUB_WINDOW_MS);
    while slot + 1 < SUB_WINDOWS && timestamp_ms >= boundary {
        slot += 1;
        boundary = boundary.saturating_add(MOTION_SUB_WINDOW_MS);
    }
    slot
}
