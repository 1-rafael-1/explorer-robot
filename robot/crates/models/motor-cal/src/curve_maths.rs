//! Motor Calibration Curve derivation and interpolation.
//!
//! Given the four Test Legs of one measured sweep — left forward, left reverse,
//! right forward and right reverse, each ten Power Steps from 10 % to 100 % —
//! [`derive_curve`] produces the per-track, per-direction [`Curve`] that makes
//! every tested leg produce the **Target Speed**.
//!
//! The model is defined in ADR-0054:
//!
//! - the Target Speed at a Power Step is the slower of the two tracks in that
//!   direction, so the slower leg's correction is always `1.00` and no
//!   correction exceeds it;
//! - the **Calibration Floor** is the highest **Breakaway** among the four legs,
//!   and below it [`Curve::resolve`] returns the raw command;
//! - each leg's measured speed series is made monotone (running maximum) before
//!   inversion, so a noisy point cannot demand more than `1.00`;
//! - corrections are interpolated linearly between measured Power Steps, a zero
//!   request stays zero, and at or above 100 % the top point holds.
//!
//! Derivations are fallible. A missing or malformed point, a leg that does not
//! move at 100 %, or a correction below the plausibility band returns a
//! [`CurveError`] naming the leg and Power Step; it is never clamped and saved.

use crate::{Measurement, PointResult};

/// The number of Power Steps in one Test Leg.
pub const POWER_STEPS: usize = 10;

/// The first Power Step, in whole percent.
pub const FIRST_POWER_STEP_PERCENT: u8 = POWER_STEP_PERCENTS[0];

/// The last Power Step, in whole percent.
pub const LAST_POWER_STEP_PERCENT: u8 = POWER_STEP_PERCENTS[TOP_STEP_INDEX];

/// The spacing between Power Steps, in whole percent.
pub const POWER_STEP_PERCENT: u8 = POWER_STEP_PERCENTS[1] - POWER_STEP_PERCENTS[0];

/// The largest correction a curve may apply.
pub const MAX_CORRECTION: f32 = 1.0;

/// The smallest plausible correction.
///
/// A derived correction below this fails the calibration instead of being
/// clamped and saved.
pub const MIN_PLAUSIBLE_CORRECTION: f32 = 0.50;

/// The number of Test Legs in one sweep.
pub const TEST_LEGS: usize = 4;

/// The zero-based index of the last Power Step.
const TOP_STEP_INDEX: usize = POWER_STEPS - 1;

/// The ten Power Steps, in whole percent, in ascending order.
pub const POWER_STEP_PERCENTS: [u8; POWER_STEPS] = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100];

/// The four Test Legs, in the order a cycle runs them.
pub const LEG_ORDER: [TestLeg; TEST_LEGS] = [
    TestLeg::new(Track::Left, Direction::Forward),
    TestLeg::new(Track::Left, Direction::Reverse),
    TestLeg::new(Track::Right, Direction::Forward),
    TestLeg::new(Track::Right, Direction::Reverse),
];

/// A track of the robot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Track {
    /// The left track.
    Left,
    /// The right track.
    Right,
}

/// The direction a Test Leg drives its track.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    /// Towards the front of the robot.
    Forward,
    /// Towards the back of the robot.
    Reverse,
}

/// One Test Leg: the track under test and the direction it turns.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TestLeg {
    /// The track under test.
    pub track: Track,
    /// The direction the track turns.
    pub direction: Direction,
}

impl TestLeg {
    /// Build a Test Leg from its track and direction.
    #[must_use]
    pub const fn new(track: Track, direction: Direction) -> Self {
        Self { track, direction }
    }

    /// The index of this leg in the fixed four-leg order.
    pub(crate) const fn index(self) -> usize {
        match (self.track, self.direction) {
            (Track::Left, Direction::Forward) => 0,
            (Track::Left, Direction::Reverse) => 1,
            (Track::Right, Direction::Forward) => 2,
            (Track::Right, Direction::Reverse) => 3,
        }
    }
}

/// One measured Power Step of one Test Leg.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LegPoint {
    /// The commanded Power Step, in whole percent.
    command_percent: u8,
    /// The wrap-safe encoder count across the Measurement Window.
    count: u16,
    /// The Measurement Window length, in milliseconds.
    window_ms: u64,
    /// Whether the Measurement Window showed sustained motion.
    sustained_motion: bool,
}

impl LegPoint {
    /// Build a point from the raw Measurement Window data.
    ///
    /// A command that is not one of the ten Power Steps, a zero-length window,
    /// a duplicate step or a moving point with a zero speed is rejected by
    /// [`derive_curve`] with [`CurveError::InvalidPoint`].
    #[must_use]
    pub const fn new(command_percent: u8, count: u16, window_ms: u64, sustained_motion: bool) -> Self {
        Self {
            command_percent,
            count,
            window_ms,
            sustained_motion,
        }
    }

    /// Build a point from a trustworthy [`Measurement`].
    ///
    /// `window_ms` must be the Measurement Window length that produced the
    /// measurement, so the reported speed matches the engine's own.
    #[must_use]
    pub const fn from_measurement(command_percent: u8, measurement: Measurement, window_ms: u64) -> Self {
        Self::new(
            command_percent,
            measurement.count(),
            window_ms,
            measurement.sustained_motion(),
        )
    }

    /// Build a point from a [`PointResult`], or `None` when the point is invalid.
    #[must_use]
    pub const fn from_point_result(command_percent: u8, result: PointResult, window_ms: u64) -> Option<Self> {
        match result {
            PointResult::Valid(measurement) => Some(Self::from_measurement(command_percent, measurement, window_ms)),
            PointResult::Invalid(_) => None,
        }
    }

    /// The commanded Power Step, in whole percent.
    #[must_use]
    pub const fn command_percent(&self) -> u8 {
        self.command_percent
    }

    /// The wrap-safe encoder count across the Measurement Window.
    #[must_use]
    pub const fn count(&self) -> u16 {
        self.count
    }

    /// The Measurement Window length, in milliseconds.
    #[must_use]
    pub const fn window_ms(&self) -> u64 {
        self.window_ms
    }

    /// Whether the Measurement Window showed sustained motion.
    #[must_use]
    pub const fn sustained_motion(&self) -> bool {
        self.sustained_motion
    }

    /// The speed over the Measurement Window, in counts per second.
    ///
    /// A zero-length window reports zero: it cannot define a speed.
    #[must_use]
    // The widening and narrowing casts are the const forms of `From`; the
    // numerator is at most 65 535 000, so the value always fits `u32`.
    #[allow(clippy::cast_lossless, clippy::cast_possible_truncation)]
    pub const fn speed_counts_per_s(&self) -> u32 {
        if self.window_ms == 0 {
            return 0;
        }
        let speed = (self.count as u64).saturating_mul(1_000) / self.window_ms;
        speed as u32
    }
}

/// The four Test Legs of one measured sweep.
#[derive(Clone, Copy, Debug)]
pub struct Sweep<'a> {
    /// The measured points, indexed in [`LEG_ORDER`] order.
    legs: [&'a [LegPoint]; TEST_LEGS],
}

impl<'a> Sweep<'a> {
    /// Build a sweep in cycle order: left forward, left reverse, right forward,
    /// right reverse.
    #[must_use]
    pub const fn new(
        left_forward: &'a [LegPoint],
        left_reverse: &'a [LegPoint],
        right_forward: &'a [LegPoint],
        right_reverse: &'a [LegPoint],
    ) -> Self {
        Self {
            legs: [left_forward, left_reverse, right_forward, right_reverse],
        }
    }

    /// The measured points of `leg`.
    #[must_use]
    pub const fn points(&self, leg: TestLeg) -> &'a [LegPoint] {
        self.legs[leg.index()]
    }
}

/// A derived Motor Calibration Curve.
///
/// Corrections are independent per track and per direction, and only exist at
/// or above the Calibration Floor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Curve {
    /// The Calibration Floor, in whole percent.
    floor_percent: u8,
    /// Corrections indexed by Test Leg and Power Step; `None` below the floor or
    /// where the leg did not move.
    corrections: [[Option<f32>; POWER_STEPS]; TEST_LEGS],
}

impl Curve {
    /// The Calibration Floor, in whole percent.
    #[must_use]
    pub const fn floor_percent(&self) -> u8 {
        self.floor_percent
    }

    /// The correction in force for `track` and `direction` at
    /// `command_percent`, or `None` below the Calibration Floor.
    ///
    /// Between measured Power Steps the result is interpolated linearly; at or
    /// above 100 % the top point holds.
    #[must_use]
    pub fn correction(&self, track: Track, direction: Direction, command_percent: u8) -> Option<f32> {
        if command_percent < self.floor_percent {
            return None;
        }
        Some(self.interpolate(TestLeg::new(track, direction).index(), command_percent))
    }

    /// Resolve a requested magnitude for `track` and `direction`.
    ///
    /// A zero request resolves to zero, a request below the Calibration Floor is
    /// returned unchanged, and any other request is scaled by the interpolated
    /// correction and rounded to the nearest whole percent, clamped to
    /// `0..=100`.
    #[must_use]
    pub fn resolve(&self, track: Track, direction: Direction, magnitude: u8) -> CurveCommand {
        if magnitude == 0 || magnitude < self.floor_percent {
            return CurveCommand::Raw(magnitude);
        }
        let correction = self.interpolate(TestLeg::new(track, direction).index(), magnitude);
        CurveCommand::Corrected(apply_correction(magnitude, correction))
    }

    /// The interpolated correction at `command_percent` for a leg index.
    fn interpolate(&self, leg_index: usize, command_percent: u8) -> f32 {
        let series = &self.corrections[leg_index];
        if command_percent >= LAST_POWER_STEP_PERCENT {
            return series[TOP_STEP_INDEX].unwrap_or(MAX_CORRECTION);
        }

        let mut lower: Option<(u8, f32)> = None;
        for (index, correction) in series.iter().enumerate() {
            let Some(correction) = *correction else {
                continue;
            };
            let percent = POWER_STEP_PERCENTS[index];
            if percent == command_percent {
                return correction;
            }
            if percent > command_percent {
                let Some((lower_percent, lower_correction)) = lower else {
                    return correction;
                };
                let span = f32::from(percent - lower_percent);
                let offset = f32::from(command_percent - lower_percent);
                let value = lower_correction + (correction - lower_correction) * (offset / span);
                return if value > MAX_CORRECTION { MAX_CORRECTION } else { value };
            }
            lower = Some((percent, correction));
        }
        lower.map_or(MAX_CORRECTION, |(_, correction)| correction)
    }
}

/// The command a curve resolves a requested magnitude to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CurveCommand {
    /// Zero or below the Calibration Floor: the request is used unchanged.
    Raw(u8),
    /// The request scaled by the interpolated correction, in `0..=100`.
    Corrected(u8),
}

impl CurveCommand {
    /// The command magnitude to apply, in `0..=100`.
    #[must_use]
    pub const fn command(self) -> u8 {
        match self {
            Self::Raw(command) | Self::Corrected(command) => command,
        }
    }

    /// Whether a calibration correction was applied.
    #[must_use]
    pub const fn is_corrected(self) -> bool {
        matches!(self, Self::Corrected(_))
    }
}

/// Why a Motor Calibration Curve could not be derived.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CurveError {
    /// A Test Leg is missing one of the ten Power Steps.
    MissingPoint {
        /// The incomplete Test Leg.
        leg: TestLeg,
        /// The missing Power Step, in whole percent.
        command_percent: u8,
    },
    /// A Test Leg carries a point that cannot be used.
    InvalidPoint {
        /// The Test Leg carrying the point.
        leg: TestLeg,
        /// The offending Power Step, in whole percent.
        command_percent: u8,
    },
    /// A Test Leg did not show sustained motion at 100 %.
    LegDidNotMove {
        /// The Test Leg that did not move.
        leg: TestLeg,
    },
    /// A derived correction fell outside the plausibility band.
    ImplausibleCorrection {
        /// The Test Leg whose correction is implausible.
        leg: TestLeg,
        /// The Power Step, in whole percent.
        command_percent: u8,
        /// The derived correction.
        correction: f32,
    },
}

impl CurveError {
    /// A short, stable phrase naming the reason for an operator log.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::MissingPoint { .. } => "missing point",
            Self::InvalidPoint { .. } => "invalid point",
            Self::LegDidNotMove { .. } => "leg did not move",
            Self::ImplausibleCorrection { .. } => "implausible correction",
        }
    }
}

/// Derive the Motor Calibration Curve from the four measured Test Legs.
///
/// The sweep must contain all ten Power Steps of every leg. A point that never
/// moved contributes neither to the Target Speed nor to its own leg's speed
/// series; a leg that does not move at 100 % fails with
/// [`CurveError::LegDidNotMove`].
///
/// # Errors
///
/// Returns [`CurveError`] when a point is missing or malformed, a leg does not
/// move at 100 %, or a derived correction falls below
/// [`MIN_PLAUSIBLE_CORRECTION`].
pub fn derive_curve(sweep: Sweep<'_>) -> Result<Curve, CurveError> {
    let mut speeds = [[0_u32; POWER_STEPS]; TEST_LEGS];
    let mut moved = [[false; POWER_STEPS]; TEST_LEGS];
    let mut breakaways = [0_usize; TEST_LEGS];

    for leg in LEG_ORDER {
        let leg_index = leg.index();
        let mut seen = [false; POWER_STEPS];
        for point in sweep.points(leg) {
            let Some(step_index) = step_index(point.command_percent()) else {
                return Err(CurveError::InvalidPoint {
                    leg,
                    command_percent: point.command_percent(),
                });
            };
            let speed = point.speed_counts_per_s();
            if seen[step_index] || point.window_ms() == 0 {
                return Err(CurveError::InvalidPoint {
                    leg,
                    command_percent: point.command_percent(),
                });
            }
            if point.sustained_motion() && speed == 0 {
                return Err(CurveError::InvalidPoint {
                    leg,
                    command_percent: point.command_percent(),
                });
            }
            seen[step_index] = true;
            speeds[leg_index][step_index] = speed;
            moved[leg_index][step_index] = point.sustained_motion();
        }

        for (step_index, &was_seen) in seen.iter().enumerate() {
            if !was_seen {
                return Err(CurveError::MissingPoint {
                    leg,
                    command_percent: POWER_STEP_PERCENTS[step_index],
                });
            }
        }

        if !moved[leg_index][TOP_STEP_INDEX] {
            return Err(CurveError::LegDidNotMove { leg });
        }
        breakaways[leg_index] = moved[leg_index]
            .iter()
            .position(|&moved| moved)
            .unwrap_or(TOP_STEP_INDEX);
    }

    let mut floor_index = 0;
    for breakaway in breakaways {
        floor_index = floor_index.max(breakaway);
    }

    let monotone = monotone_speeds(&speeds, &moved);

    let mut corrections = [[None; POWER_STEPS]; TEST_LEGS];
    for step_index in floor_index..POWER_STEPS {
        for direction in [Direction::Forward, Direction::Reverse] {
            let left = TestLeg::new(Track::Left, direction);
            let right = TestLeg::new(Track::Right, direction);
            let mut target: Option<u32> = None;
            for leg in [left, right] {
                let leg_index = leg.index();
                if moved[leg_index][step_index] {
                    let speed = monotone[leg_index][step_index];
                    target = Some(target.map_or(speed, |current| current.min(speed)));
                }
            }
            let Some(target) = target else {
                continue;
            };
            for leg in [left, right] {
                let leg_index = leg.index();
                if !moved[leg_index][step_index] {
                    continue;
                }
                let speed = monotone[leg_index][step_index];
                let correction = correction_ratio(target, speed);
                if correction < MIN_PLAUSIBLE_CORRECTION {
                    return Err(CurveError::ImplausibleCorrection {
                        leg,
                        command_percent: POWER_STEP_PERCENTS[step_index],
                        correction,
                    });
                }
                // The target is the slower moving leg, so the ratio is at most
                // 1.00 in exact arithmetic; the clamp only absorbs f32 rounding.
                corrections[leg_index][step_index] = Some(if correction > MAX_CORRECTION {
                    MAX_CORRECTION
                } else {
                    correction
                });
            }
        }
    }

    Ok(Curve {
        floor_percent: POWER_STEP_PERCENTS[floor_index],
        corrections,
    })
}

/// The zero-based index of `percent` in the ten-step sweep, or `None` when it is
/// not one of the Power Steps.
const fn step_index(percent: u8) -> Option<usize> {
    let mut index = 0;
    while index < POWER_STEPS {
        if POWER_STEP_PERCENTS[index] == percent {
            return Some(index);
        }
        index += 1;
    }
    None
}

/// Make each leg's measured speed series monotone with a running maximum.
///
/// A point that never moved carries the running maximum forward but remains
/// absent from the target: its own speed says nothing about the track's
/// capability.
fn monotone_speeds(
    speeds: &[[u32; POWER_STEPS]; TEST_LEGS],
    moved: &[[bool; POWER_STEPS]; TEST_LEGS],
) -> [[u32; POWER_STEPS]; TEST_LEGS] {
    let mut monotone = [[0_u32; POWER_STEPS]; TEST_LEGS];
    for leg_index in 0..TEST_LEGS {
        let mut running = 0;
        for ((out, &speed), &did_move) in monotone[leg_index]
            .iter_mut()
            .zip(&speeds[leg_index])
            .zip(&moved[leg_index])
        {
            if did_move {
                running = running.max(speed);
            }
            *out = running;
        }
    }
    monotone
}

/// The correction that scales `speed` down to `target`.
#[allow(clippy::cast_precision_loss)]
fn correction_ratio(target: u32, speed: u32) -> f32 {
    // Both speeds are counts per second over a two-second window, far below the
    // exact-integer range of `f32`; the cast is lossless for any real sweep.
    target as f32 / speed as f32
}

/// Apply a correction to a requested magnitude, rounded and clamped to 0..=100.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn apply_correction(magnitude: u8, correction: f32) -> u8 {
    // `f32::round` is unavailable in `no_std`; add a half and truncate. The
    // correction is positive and at most 1.00, so the result is in 0..=100
    // after the clamp.
    let scaled = f32::from(magnitude) * correction + 0.5;
    if scaled >= 100.0 { 100 } else { scaled as u8 }
}
