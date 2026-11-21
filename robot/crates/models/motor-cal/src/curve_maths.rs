//! Motor Calibration Curve derivation and interpolation.
//!
//! Given the four Test Legs of one measured sweep — left forward, left reverse,
//! right forward and right reverse, each ten Power Steps from 10 % to 100 % —
//! [`derive_curve`] produces the per-track, per-direction [`Curve`] that makes
//! every tested leg produce the **Target Speed**.
//!
//! The model is defined in ADR-0054 and refined by the simultaneous-pair
//! precision diagnosis:
//!
//! - the Target Speed at a Power Step is the slower of the two tracks in that
//!   direction, so the slower leg is never attenuated and no correction exceeds
//!   `1.00`;
//! - the **Calibration Floor** is the highest **Breakaway** among the four legs,
//!   and below it [`Curve::resolve`] returns the raw command;
//! - each leg's measured speed series is made monotone (running maximum) before
//!   inversion, so a noisy point cannot demand more than `1.00`;
//! - the command that produces the Target Speed is found by **inverse
//!   piecewise-linear interpolation** of the leg's measured (command, speed)
//!   points, with a `(0, 0)` anchor, rather than by a linear `target / speed`
//!   ratio. The motor's speed-versus-command response is nonlinear at low duty,
//!   and the ratio over-attenuates there;
//! - Target Speeds and the inverse curve are interpolated linearly between
//!   measured Power Steps, a zero request stays zero, and at or above 100 % the
//!   top point holds.
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

/// The number of directions a curve resolves independently.
const DIRECTIONS: usize = 2;

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

impl Direction {
    /// The index of this direction in the per-direction series.
    const fn index(self) -> usize {
        match self {
            Self::Forward => 0,
            Self::Reverse => 1,
        }
    }
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
/// The curve stores the slowest measurement at each moved Power Step, the
/// per-direction Target Speed series derived from them, and the Calibration
/// Floor. It resolves a requested magnitude by inverting each leg's measured
/// (command, speed) points, so it captures the motor's nonlinear response where
/// a linear ratio cannot.
///
/// Corrections are independent per track and per direction, and only exist at
/// or above the Calibration Floor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Curve {
    /// The Calibration Floor, in whole percent.
    floor_percent: u8,
    /// The monotone measured speed at each moved Power Step, in counts per
    /// second, indexed by Test Leg and Power Step. Zero marks a point that did
    /// not move; it is excluded from both the Target Speed and the inverse curve.
    speeds: [[u32; POWER_STEPS]; TEST_LEGS],
    /// The Target Speed per direction and Power Step, in counts per second.
    /// Zero marks a Power Step with no target: below the floor, or one of the two
    /// legs did not move there.
    targets: [[u32; POWER_STEPS]; DIRECTIONS],
}

impl Curve {
    /// The Calibration Floor, in whole percent.
    #[must_use]
    pub const fn floor_percent(&self) -> u8 {
        self.floor_percent
    }

    /// The correction in force for `track` and `direction` at
    /// `command_percent`, or `None` below the Calibration Floor or when the
    /// direction has no target there.
    ///
    /// The factor is the inverse-interpolated command that produces the
    /// interpolated Target Speed at `command_percent`, divided by
    /// `command_percent`, and clamped to `1.00`. Between measured Power Steps the
    /// Target Speed is interpolated linearly; at or above 100 % the top point
    /// holds.
    #[must_use]
    pub fn correction(&self, track: Track, direction: Direction, command_percent: u8) -> Option<f32> {
        if command_percent < self.floor_percent {
            return None;
        }
        let target = self.target_speed(direction, command_percent)?;
        let leg_index = TestLeg::new(track, direction).index();
        let command = inverse_command(&self.speeds[leg_index], target);
        let factor = command / f32::from(command_percent);
        // The target is at most this leg's measured speed, so the inverse is at
        // most the requested command; the clamp only absorbs f32 rounding.
        Some(if factor > MAX_CORRECTION {
            MAX_CORRECTION
        } else {
            factor
        })
    }

    /// Resolve a requested magnitude for `track` and `direction`.
    ///
    /// A zero request resolves to zero, a request below the Calibration Floor is
    /// returned unchanged (as `magnitude * 10` per-mille), and any other request
    /// is resolved to the command that produces the interpolated Target Speed,
    /// rounded once to the nearest per-mille and clamped to `0..=magnitude * 10`
    /// and `0..=1000`.
    #[must_use]
    pub fn resolve(&self, track: Track, direction: Direction, magnitude: u8) -> CurveCommand {
        let raw = u16::from(magnitude) * 10;
        if magnitude == 0 || magnitude < self.floor_percent {
            return CurveCommand::Raw(raw);
        }
        let Some(target) = self.target_speed(direction, magnitude) else {
            return CurveCommand::Raw(raw);
        };
        let leg_index = TestLeg::new(track, direction).index();
        let command = inverse_command(&self.speeds[leg_index], target);
        CurveCommand::Corrected(
            round_permille(command)
                .min(raw)
                .min(u16::from(LAST_POWER_STEP_PERCENT) * 10),
        )
    }

    /// The Target Speed for `direction` at `magnitude`, interpolated linearly
    /// between the measured Power Steps.
    ///
    /// Zero maps to zero, and a magnitude above the highest measured Power Step
    /// holds that point's target. `None` means no direction had a target at all.
    #[allow(clippy::cast_precision_loss)]
    fn target_speed(&self, direction: Direction, magnitude: u8) -> Option<f32> {
        if magnitude == 0 {
            return Some(0.0);
        }
        let series = &self.targets[direction.index()];
        let mut lower: Option<(u8, u32)> = None;
        for (index, &target) in series.iter().enumerate() {
            if target == 0 {
                continue;
            }
            let percent = POWER_STEP_PERCENTS[index];
            if percent == magnitude {
                return Some(target as f32);
            }
            if percent > magnitude {
                let upper_speed = target as f32;
                return Some(match lower {
                    Some((lower_percent, lower_speed)) => {
                        let span = f32::from(percent - lower_percent);
                        let offset = f32::from(magnitude - lower_percent);
                        (lower_speed as f32) + (upper_speed - (lower_speed as f32)) * (offset / span)
                    }
                    None => {
                        // The zero anchor: a target below the first measured
                        // Power Step ramps up from zero.
                        upper_speed * (f32::from(magnitude) / f32::from(percent))
                    }
                });
            }
            lower = Some((percent, target));
        }
        lower.map(|(_, speed)| speed as f32)
    }
}

/// The command a curve resolves a requested magnitude to, in per-mille
/// (tenths of a percent).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CurveCommand {
    /// Zero or below the Calibration Floor: the request is used unchanged.
    Raw(u16),
    /// The request resolved through the inverse curve, in `0..=1000`.
    Corrected(u16),
}

impl CurveCommand {
    /// The command magnitude to apply, in `0..=1000` (per-mille).
    #[must_use]
    pub const fn per_mille(self) -> u16 {
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
    /// A derived correction fell below the plausibility band.
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
/// A pair-measured sweep fills both tracks at the same command, so the slower of
/// the two tracks in a direction is the Target Speed at that Power Step. The
/// sweep must contain all ten Power Steps of every leg. A point that never
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

    let mut targets = [[0_u32; POWER_STEPS]; DIRECTIONS];
    for direction in [Direction::Forward, Direction::Reverse] {
        let left = TestLeg::new(Track::Left, direction).index();
        let right = TestLeg::new(Track::Right, direction).index();
        for step_index in floor_index..POWER_STEPS {
            let left_speed = monotone[left][step_index];
            let right_speed = monotone[right][step_index];
            if left_speed != 0 && right_speed != 0 {
                targets[direction.index()][step_index] = left_speed.min(right_speed);
            }
        }
    }

    let curve = Curve {
        floor_percent: POWER_STEP_PERCENTS[floor_index],
        speeds: monotone,
        targets,
    };

    for direction in [Direction::Forward, Direction::Reverse] {
        for leg in [
            TestLeg::new(Track::Left, direction),
            TestLeg::new(Track::Right, direction),
        ] {
            for &command_percent in POWER_STEP_PERCENTS.iter().skip(floor_index) {
                if let Some(correction) = curve.correction(leg.track, leg.direction, command_percent)
                    && correction < MIN_PLAUSIBLE_CORRECTION
                {
                    return Err(CurveError::ImplausibleCorrection {
                        leg,
                        command_percent,
                        correction,
                    });
                }
            }
        }
    }

    Ok(curve)
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
/// The running maximum is stored only at Power Steps that moved: a point that
/// never moved carries nothing forward and stays absent (zero) from both the
/// Target Speed and the leg's own inverse curve, because its speed says nothing
/// about the track's capability.
fn monotone_speeds(
    speeds: &[[u32; POWER_STEPS]; TEST_LEGS],
    moved: &[[bool; POWER_STEPS]; TEST_LEGS],
) -> [[u32; POWER_STEPS]; TEST_LEGS] {
    let mut monotone = [[0_u32; POWER_STEPS]; TEST_LEGS];
    for leg_index in 0..TEST_LEGS {
        let mut running = 0;
        for (step_index, out) in monotone[leg_index].iter_mut().enumerate() {
            if moved[leg_index][step_index] {
                running = running.max(speeds[leg_index][step_index]);
                *out = running;
            }
        }
    }
    monotone
}

/// Invert a leg's measured (command, speed) points at `target`.
///
/// The points include the `(0, 0)` anchor and every moved Power Step in command
/// order; `speed == 0` marks an excluded point. A target at or above the leg's
/// top speed resolves to the top command (100 %). The result is the un-rounded
/// command in percent, so the caller rounds it to per-mille exactly once.
#[allow(clippy::cast_precision_loss)]
fn inverse_command(speeds: &[u32; POWER_STEPS], target: f32) -> f32 {
    let mut lower_percent = 0.0_f32;
    let mut lower_speed = 0.0_f32;
    for (index, &speed) in speeds.iter().enumerate() {
        if speed == 0 {
            continue;
        }
        let percent = f32::from(POWER_STEP_PERCENTS[index]);
        let speed = speed as f32;
        if target <= speed {
            if speed <= lower_speed {
                return percent;
            }
            let fraction = (target - lower_speed) / (speed - lower_speed);
            return lower_percent + (percent - lower_percent) * fraction;
        }
        lower_percent = percent;
        lower_speed = speed;
    }
    f32::from(LAST_POWER_STEP_PERCENT)
}

/// Round a positive command in percent to the nearest per-mille, clamped to
/// `0..=1000`.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn round_permille(command_percent: f32) -> u16 {
    // `f32::round` is unavailable in `no_std`; add a half and truncate.
    let rounded = command_percent * 10.0 + 0.5;
    let max = f32::from(LAST_POWER_STEP_PERCENT) * 10.0;
    if rounded <= 0.0 {
        0
    } else if rounded >= max {
        max as u16
    } else {
        rounded as u16
    }
}
