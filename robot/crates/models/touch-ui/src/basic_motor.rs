//! The neutral Basic Motor Test Readout and its row formatting.
//!
//! The Basic Motor Test publishes one of these as it runs; this module owns the
//! labels, the percent, the correction and the placeholders the panel draws. A
//! Test Leg is named by [`TestLeg`], [`Track`] and [`Direction`], and the
//! firmware never formats text, so no display string crosses this boundary. The
//! same [`TestLeg::label`] names the Activity's phase line, so the panel and the
//! `ACTIVITY` stream cannot drift apart.
//!
//! The readout also owns the rules the task would otherwise spread through its
//! loops. It accepts four transitions — enter the discarded warm-up, start a
//! pair, sample both counts, and complete a pair — and derives every row from
//! them. [`BasicMotorReadout::waiting`] is the state before the run begins;
//! [`BasicMotorReadout::warming_up`] names the discarded raw warm-up sweep that
//! precedes the measured cycle, and the first [`BasicMotorReadout::start_pair`]
//! ends it. A pair commands both
//! tracks at the same direction and power and measures them over the same
//! window, so the live rows carry a value for each side. Starting a pair does
//! not clear the displayed counts: the encoder reset that precedes a pair empties
//! the hardware, not the operator's reading, so the live counts hold until the
//! next sample replaces them. Completing a pair latches each side's Breakaway at
//! the lowest power whose Measurement Window showed sustained motion, and each
//! direction keeps its own, so the row reports the raw Breakaway of the direction
//! currently running. A corrected pair never latches a Breakaway.
//!
//! # Rows
//!
//! Six rows carry the current pair (direction and power), both live encoder
//! counts, the last completed pair with the counts it finished with, both
//! Breakaways, the correction the curve in force applies to each track at the
//! current pair, and the position within the cycle. A value that does not exist
//! yet renders `--`, never `0`, so "no data yet" can never be mistaken for "zero
//! pulses". A Breakaway that is still `--` at the end of a direction is the
//! verdict that the track did not move at any power up to 100 %.
//!
//! # The correction row
//!
//! The correction is a factor the task resolves from the loaded Motor
//! Calibration Curve, so this model never does curve maths. It travels as
//! [`Correction`], which keeps a fixed-point factor in per-mille rather than an
//! `f32`, so the readout stays `PartialEq + Eq` and the running screen's redraw
//! comparison is exact.
//!
//! # The Result Report summary
//!
//! A completed run publishes one [`BasicMotorSummary`] instead of the live
//! readout: for each of the four Test Legs its Breakaway and the counts at 40,
//! 60, 80 and 100 %, the correction in force at those points, and one unjudged
//! line carrying the Calibration Floor and the spread at 100 %. The summary is
//! built once from the twenty Power Step outcomes the task recorded, so the live
//! publish path never copies the whole table. A point that did not show
//! sustained motion renders `--`, never `0`, and a Test Leg that never moved
//! renders `--` in its Breakaway and every count cell.

use crate::buffer::TextBuf;

/// The number of rows this readout draws.
pub const ROWS: usize = 6;

/// A track of the robot, as the readout names it.
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
    /// The operator-facing abbreviation the pair rows use, e.g. `Fwd`.
    ///
    /// [`TestLeg::label`] keeps its spelled-out form for the Activity phase
    /// line; this shorter form only labels the live pair rows.
    #[must_use]
    pub const fn short_label(self) -> &'static str {
        match self {
            Self::Forward => "Fwd",
            Self::Reverse => "Rev",
        }
    }
}

/// One Test Leg: the track under test and the direction it turns.
///
/// A Test Leg is a group of ten Power Steps; the firmware drives the steps and
/// names the leg with this value rather than with a string of its own.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TestLeg {
    /// The track under test.
    pub track: Track,
    /// The direction the track turns.
    pub direction: Direction,
}

impl TestLeg {
    /// The operator-facing label, e.g. `Left forward`.
    ///
    /// This is the single source for both the readout's current-step row and the
    /// Activity phase line, so the panel and the `ACTIVITY` stream cannot name
    /// the same leg differently.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match (self.track, self.direction) {
            (Track::Left, Direction::Forward) => "Left forward",
            (Track::Left, Direction::Reverse) => "Left reverse",
            (Track::Right, Direction::Forward) => "Right forward",
            (Track::Right, Direction::Reverse) => "Right reverse",
        }
    }
}

/// The correction in force at the current Power Step.
///
/// The task resolves this from the loaded Motor Calibration Curve and the current
/// Test Leg and power, so the readout never does curve maths. A factor travels in
/// per-mille rather than as an `f32`, so the readout stays `Eq`-comparable and the
/// running screen's redraw comparison stays exact.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Correction {
    /// No curve is loaded, so the requested command is used unchanged (`1.00`).
    Identity,
    /// The power is below the Calibration Floor, so the raw command is used.
    Raw,
    /// The interpolated curve factor, in per-mille: `930` is `0.93`.
    Factor(u16),
}

/// The Result Report's optional saved/not-saved footer.
///
/// A Motor Calibration adds it to say whether the curve reached flash. The Basic
/// Motor Test leaves it absent, because it never writes a curve; its footer
/// instead carries the corrected verification through [`CorrectedCheck`]. The
/// reason is a short stable phrase, so the footer stays `Copy + Eq` and never
/// allocates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SummaryStatus {
    /// The calibration was written to flash and is now the curve in force.
    Saved,
    /// The run was not saved, with the reason the operator needs to see.
    NotSaved(&'static str),
}

/// Where the Basic Motor Test's curve-applied verification cycle found its worst
/// Track Divergence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CorrectedMax {
    /// The divergence in tenths of a percent: `16` is `1.6 %`.
    pub tenths: u16,
    /// The direction of the worst gated pair.
    pub direction: Direction,
    /// The power of the worst gated pair, in percent.
    pub power: u8,
}

/// The Basic Motor Test's curve-applied verification result.
///
/// The raw cycle measures the drivetrain; the verification cycles command the
/// loaded curve and measure how closely the two tracks actually match. The panel
/// shows the worst mean in the Result Report's footer, next to the raw tables, so
/// the operator can tell the drivetrain difference apart from the residual the
/// calibration leaves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CorrectedCheck {
    /// No curve was loaded, so the verification cycles were skipped.
    NoCurve,
    /// The verification cycles measured this worst gated pair.
    Measured(CorrectedMax),
    /// The verification cycles produced no gated point.
    Unavailable,
}

/// The position of a Power Step within its cycle, as the task knows it.
///
/// The task owns the schedule — how many Power Steps a Test Leg holds and how
/// many Test Legs a cycle holds — and hands the readout a position to format, so
/// the readout never counts steps or knows the powers in a cycle itself.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StepIndex {
    /// One-based position within the cycle.
    pub position: u8,
    /// The number of Power Steps in a full cycle.
    pub total: u8,
}

/// Which measured pass a pair belongs to.
///
/// The Basic Motor Test runs a raw cycle and then, when a curve is loaded, two
/// curve-applied verification cycles. The panel prefixes the latter so a
/// corrected count can never be read as a raw one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pass {
    /// Raw `SetTracksRaw`: the drivetrain with the curve bypassed.
    Raw,
    /// Curve-applied `SetTracks`: the corrected pair the robot drives.
    Corrected,
}

impl Pass {
    /// The prefix the pair rows carry, or the empty string for the raw pass.
    const fn prefix(self) -> &'static str {
        match self {
            Self::Raw => "",
            Self::Corrected => "Corr ",
        }
    }
}

/// The pair the panel names: one direction at one power level, both tracks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CurrentPair {
    /// The pass that commanded the pair.
    pass: Pass,
    /// The direction both tracks turn.
    direction: Direction,
    /// The commanded power, in percent.
    power: u8,
}

/// A pair that has finished: the direction and power it commanded and each
/// track's Measurement Window count, or `None` when that side was invalid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CompletedPair {
    /// The pass that commanded the pair.
    pass: Pass,
    /// The direction both tracks turned.
    direction: Direction,
    /// The commanded power, in percent.
    power: u8,
    /// The left track's Measurement Window count, or `None` when invalid.
    left: Option<u16>,
    /// The right track's Measurement Window count, or `None` when invalid.
    right: Option<u16>,
}

/// The correction the curve in force applies to each track at the current pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CorrectionPair {
    /// The factor applied to the left track.
    left: Correction,
    /// The factor applied to the right track.
    right: Correction,
}

/// The neutral Basic Motor Test Readout: the current pair, both live encoder
/// counts, the last completed pair, both Breakaways, both corrections and the
/// step index.
///
/// [`BasicMotorReadout::waiting`] is the state before the run begins and
/// [`BasicMotorReadout::warming_up`] is the discarded warm-up stage that
/// precedes the first measured pair, which [`BasicMotorReadout::start_pair`]
/// ends. The firmware holds one locally, publishes a copy at the sampling rate,
/// and drives it with [`BasicMotorReadout::warming_up`],
/// [`BasicMotorReadout::start_pair`], [`BasicMotorReadout::sample`] and
/// [`BasicMotorReadout::complete`]; the model decides what each row says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BasicMotorReadout {
    /// The pair the panel names: the one commanding, or the one that last
    /// commanded. It is deliberately kept past a completion, so the row does not
    /// blank between pairs and the operator always knows which command the
    /// numbers belong to.
    step: Option<CurrentPair>,
    /// Whether `step` is still commanding, so a completion that arrives with no
    /// pair in progress cannot overwrite what the operator is reading.
    commanding: bool,
    /// The live encoder count for the left track, or `None` before the first
    /// sample.
    count_left: Option<u16>,
    /// The live encoder count for the right track, or `None` before the first
    /// sample.
    count_right: Option<u16>,
    /// The most recently completed pair, or `None` before the first completion.
    last: Option<CompletedPair>,
    /// The lowest power of the left Measurement Window that showed sustained
    /// motion, per direction (forward then reverse). A corrected pass reads these
    /// but never writes them: the Breakaway is a raw drivetrain fact.
    raw_breakaway_left: [Option<u8>; 2],
    /// The lowest power of the right Measurement Window that showed sustained
    /// motion, per direction.
    raw_breakaway_right: [Option<u8>; 2],
    /// The corrections the curve in force applies to both tracks, or `None`
    /// before the first pair.
    correction: Option<CorrectionPair>,
    /// The position within the cycle of the current or most recently completed
    /// pair, or `None` before the first pair.
    index: Option<StepIndex>,
    /// Whether the discarded warm-up stage is running. While it is, the `Step`
    /// row says `Warming up` and every other row is a placeholder, because none
    /// of the warm-up's numbers are results.
    warming: bool,
}

impl BasicMotorReadout {
    /// A readout with no pair, no sample and no completed pair yet.
    #[must_use]
    pub const fn waiting() -> Self {
        Self {
            step: None,
            commanding: false,
            count_left: None,
            count_right: None,
            last: None,
            raw_breakaway_left: [None; 2],
            raw_breakaway_right: [None; 2],
            correction: None,
            index: None,
            warming: false,
        }
    }

    /// Show the discarded warm-up stage on the panel.
    ///
    /// The Basic Motor Test drives a discarded raw pair sweep before it measures,
    /// so the panel says `Warming up` while the progress bar advances and every
    /// other row reads `--`: none of the warm-up's numbers are results. The first
    /// [`start_pair`](Self::start_pair) of the measured sweep ends the stage.
    pub const fn warming_up(&mut self) {
        self.warming = true;
        self.step = None;
        self.commanding = false;
        self.count_left = None;
        self.count_right = None;
        self.last = None;
        self.raw_breakaway_left = [None; 2];
        self.raw_breakaway_right = [None; 2];
        self.correction = None;
        self.index = None;
    }

    /// Begin a pair on `direction` at `power` percent, at `index` within the
    /// cycle, with the curve in force for each track.
    ///
    /// The displayed counts are deliberately left alone: the encoder reset has
    /// already happened, and the operator's last reading must not flicker to a
    /// placeholder while the new command settles. Each direction keeps its own raw
    /// Breakaways, so switching direction shows that direction's values. This first
    /// measured pair
    /// also ends the discarded warm-up stage.
    pub const fn start_pair(
        &mut self,
        direction: Direction,
        power: u8,
        index: StepIndex,
        correction_left: Correction,
        correction_right: Correction,
    ) {
        self.start_pair_in(Pass::Raw, direction, power, index, correction_left, correction_right);
    }

    /// Begin a pair of the curve-applied verification cycle.
    ///
    /// The rows are the same as [`start_pair`](Self::start_pair), with the pair
    /// labelled `Corr` and the Breakaway row left as the raw cycle latched it.
    pub const fn start_corrected_pair(
        &mut self,
        direction: Direction,
        power: u8,
        index: StepIndex,
        correction_left: Correction,
        correction_right: Correction,
    ) {
        self.start_pair_in(
            Pass::Corrected,
            direction,
            power,
            index,
            correction_left,
            correction_right,
        );
    }

    /// Begin a pair of `pass`, sharing the row rules between both passes.
    const fn start_pair_in(
        &mut self,
        pass: Pass,
        direction: Direction,
        power: u8,
        index: StepIndex,
        correction_left: Correction,
        correction_right: Correction,
    ) {
        self.step = Some(CurrentPair { pass, direction, power });
        self.index = Some(index);
        self.correction = Some(CorrectionPair {
            left: correction_left,
            right: correction_right,
        });
        self.commanding = true;
        // The measured sweep begins, so the discarded warm-up stage ends.
        self.warming = false;
    }

    /// Replace both live encoder counts with the latest raw samples.
    pub const fn sample(&mut self, left: u16, right: u16) {
        self.count_left = Some(left);
        self.count_right = Some(right);
    }

    /// End the current pair.
    ///
    /// `left` and `right` are the Measurement Window counts, or `None` for a
    /// side whose measurement was invalid, in which case the `Last` row shows
    /// `--` rather than `0`. `moved_left` and `moved_right` say whether each
    /// side's window showed sustained motion; only a moving side latches its
    /// Breakaway, so a stray pulse cannot be mistaken for motion.
    ///
    /// The pair's own direction and power are retained for the `Last` row, and
    /// each side's power latches that side's Breakaway while it is still absent.
    /// The current-pair row keeps naming the pair that just ran, so the panel
    /// does not blank for the coast between pairs. A completion with no pair in
    /// progress is ignored, so a stray one cannot overwrite what the operator is
    /// reading.
    pub const fn complete(&mut self, left: Option<u16>, right: Option<u16>, moved_left: bool, moved_right: bool) {
        if !self.commanding {
            return;
        }
        let Some(step) = self.step else {
            return;
        };

        self.last = Some(CompletedPair {
            pass: step.pass,
            direction: step.direction,
            power: step.power,
            left,
            right,
        });
        // The Breakaway is a raw drivetrain property: the corrected pass must not
        // relabel a corrected point as it.
        if matches!(step.pass, Pass::Raw) {
            let direction = direction_index(step.direction);
            if moved_left {
                self.raw_breakaway_left[direction] = Some(lower_power(self.raw_breakaway_left[direction], step.power));
            }
            if moved_right {
                self.raw_breakaway_right[direction] =
                    Some(lower_power(self.raw_breakaway_right[direction], step.power));
            }
        }
        self.commanding = false;
    }

    /// The formatted label/value pairs, in row order: the current pair, both
    /// live encoder counts, the last completed pair, both Breakaways, both
    /// corrections and the position within the cycle.
    ///
    /// A pair row renders its `L ... R ...` form once the run has named a pair;
    /// before the first pair every value is a bare `--`. During the discarded
    /// warm-up the `Step` row says `Warming up` and every other row is a bare
    /// `--`, because none of the warm-up's numbers are results.
    #[must_use]
    pub fn rows(&self) -> [(TextBuf, TextBuf); ROWS] {
        let warming = self.warming;
        let active = self.step.is_some() && !warming;

        let step_cell = if warming {
            labelled("Warming up")
        } else {
            self.step.map_or_else(placeholder, |step| {
                pair_step_value(step.pass, step.direction, step.power)
            })
        };

        let count_value = if active {
            side_counts(self.count_left, self.count_right)
        } else {
            placeholder()
        };

        let last_value = if warming {
            placeholder()
        } else {
            self.last.map_or_else(placeholder, completed_pair_value)
        };

        let breakaway_value = if active {
            let direction = self.step.map_or(0, |step| direction_index(step.direction));
            side_powers(self.raw_breakaway_left[direction], self.raw_breakaway_right[direction])
        } else {
            placeholder()
        };

        let correction_cell = if warming {
            placeholder()
        } else {
            self.correction.map_or_else(placeholder, correction_pair_value)
        };

        let index_value = if warming {
            placeholder()
        } else {
            self.index.map_or_else(placeholder, step_index_value)
        };

        [
            (labelled("Step"), step_cell),
            (labelled("Count"), count_value),
            (labelled("Last"), last_value),
            (labelled("Breakaway"), breakaway_value),
            (labelled("Correction"), correction_cell),
            (labelled("Index"), index_value),
        ]
    }
}

/// The index of `direction` in the readout's per-direction arrays.
const fn direction_index(direction: Direction) -> usize {
    match direction {
        Direction::Forward => 0,
        Direction::Reverse => 1,
    }
}

/// The lower of `power` and an already latched power, so a later, higher step
/// never replaces a Breakaway even if the task reports the steps out of order.
const fn lower_power(current: Option<u8>, power: u8) -> u8 {
    match current {
        Some(current) => {
            if current < power {
                current
            } else {
                power
            }
        }
        None => power,
    }
}

/// Spell a Test Leg as the Result Report's abbreviation, e.g. `L fwd`.
const fn abbreviated_leg(leg: TestLeg) -> &'static str {
    match (leg.track, leg.direction) {
        (Track::Left, Direction::Forward) => "L fwd",
        (Track::Left, Direction::Reverse) => "L rev",
        (Track::Right, Direction::Forward) => "R fwd",
        (Track::Right, Direction::Reverse) => "R rev",
    }
}

/// Format the current pair: the direction and power, e.g. `Fwd 40%`, with a
/// `Corr` prefix during the curve-applied verification cycle.
fn pair_step_value(pass: Pass, direction: Direction, power: u8) -> TextBuf {
    let mut value = TextBuf::new();
    value.fill(format_args!("{}{} {power}%", pass.prefix(), direction.short_label()));
    value
}

/// Format both live counts, e.g. `L 583 R 568`, with `--` for a side that has
/// no sample.
fn side_counts(left: Option<u16>, right: Option<u16>) -> TextBuf {
    let left = count_side(left);
    let right = count_side(right);
    let mut value = TextBuf::new();
    value.fill(format_args!("L {} R {}", left.as_str(), right.as_str()));
    value
}

/// Format one side's live count, or `--` when it has none.
fn count_side(count: Option<u16>) -> TextBuf {
    count.map_or_else(placeholder, number)
}

/// Format the last completed pair: its direction, power and both counts, e.g.
/// `Fwd 40% L 583 R 568`, with `--` for an invalid side and a `Corr` prefix for
/// the verification cycle.
fn completed_pair_value(pair: CompletedPair) -> TextBuf {
    let left = count_side(pair.left);
    let right = count_side(pair.right);
    let mut value = TextBuf::new();
    value.fill(format_args!(
        "{}{} {}% L {} R {}",
        pair.pass.prefix(),
        pair.direction.short_label(),
        pair.power,
        left.as_str(),
        right.as_str()
    ));
    value
}

/// Format both Breakaways, e.g. `L 20% R 10%`, with `--` for a side that has
/// not moved yet.
fn side_powers(left: Option<u8>, right: Option<u8>) -> TextBuf {
    let left = left.map_or_else(placeholder, power_value);
    let right = right.map_or_else(placeholder, power_value);
    let mut value = TextBuf::new();
    value.fill(format_args!("L {} R {}", left.as_str(), right.as_str()));
    value
}

/// Format both corrections, e.g. `L 0.97 R 1.00`.
fn correction_pair_value(pair: CorrectionPair) -> TextBuf {
    let left = correction_value(pair.left);
    let right = correction_value(pair.right);
    let mut value = TextBuf::new();
    value.fill(format_args!("L {} R {}", left.as_str(), right.as_str()));
    value
}

/// Format the correction in force: the fixed-point factor, `raw` below the
/// Calibration Floor, or `1.00` when no curve is loaded.
fn correction_value(correction: Correction) -> TextBuf {
    let mut value = TextBuf::new();
    match correction {
        Correction::Identity => value.fill(format_args!("1.00")),
        Correction::Raw => value.fill(format_args!("raw")),
        Correction::Factor(per_mille) => {
            // Round to the nearest hundredth for display; the per-mille value
            // itself stays the comparison token.
            let hundredths = (per_mille.saturating_add(5)) / 10;
            value.fill(format_args!("{}.{:02}", hundredths / 100, hundredths % 100));
        }
    }
    value
}

/// Format a position within the cycle as `n/total`, e.g. `12/20`.
fn step_index_value(index: StepIndex) -> TextBuf {
    let StepIndex { position, total } = index;
    let mut value = TextBuf::new();
    value.fill(format_args!("{position}/{total}"));
    value
}

/// Format a Power Step's commanded power as a percentage.
fn power_value(power: u8) -> TextBuf {
    let mut value = TextBuf::new();
    value.fill(format_args!("{power}%"));
    value
}

/// Spell `text` into a row cell.
fn labelled(text: &str) -> TextBuf {
    let mut cell = TextBuf::new();
    cell.fill(format_args!("{text}"));
    cell
}

/// Spell a number into a row cell.
fn number(value: u16) -> TextBuf {
    let mut cell = TextBuf::new();
    cell.fill(format_args!("{value}"));
    cell
}

/// A cell with no value yet.
fn placeholder() -> TextBuf {
    let mut cell = TextBuf::new();
    cell.fill(format_args!("--"));
    cell
}

// ── Result Report summary ──────────────────────────────────────────────────────

/// The number of Test Legs a Basic Motor Test runs.
pub const SUMMARY_LEGS: usize = 4;

/// The Power Steps the Result Report tabulates, in ascending percent.
pub const SUMMARY_POWERS: [u8; 4] = [40, 60, 80, 100];

/// The number of tabulated points in each Test Leg row.
pub const SUMMARY_POINTS: usize = SUMMARY_POWERS.len();

/// The number of rows the two tables draw: a header and four leg rows each.
pub const SUMMARY_TABLE_ROWS: usize = 2 * (SUMMARY_LEGS + 1);

/// The widest number of value cells in a Result Report row: the counts table's
/// Breakaway plus its four points.
pub const SUMMARY_CELLS: usize = SUMMARY_POINTS + 1;

/// The Test Legs in the order a cycle runs them.
const SUMMARY_LEG_ORDER: [TestLeg; SUMMARY_LEGS] = [
    TestLeg {
        track: Track::Left,
        direction: Direction::Forward,
    },
    TestLeg {
        track: Track::Left,
        direction: Direction::Reverse,
    },
    TestLeg {
        track: Track::Right,
        direction: Direction::Forward,
    },
    TestLeg {
        track: Track::Right,
        direction: Direction::Reverse,
    },
];

/// One Test Leg's Result Report row: its Breakaway, the counts at the four
/// tabulated Power Steps and the correction in force at each of them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct LegSummary {
    /// The lowest Power Step whose Measurement Window showed sustained motion,
    /// or `None` when the leg never moved.
    breakaway: Option<u8>,
    /// The count at each tabulated Power Step, or `None` when the point did not
    /// move or its measurement was invalid.
    counts: [Option<u16>; SUMMARY_POINTS],
    /// The correction in force at each tabulated Power Step, or `None` before
    /// the leg's row was recorded.
    corrections: [Option<Correction>; SUMMARY_POINTS],
}

impl LegSummary {
    /// A leg with no measurements yet.
    const EMPTY: Self = Self {
        breakaway: None,
        counts: [None; SUMMARY_POINTS],
        corrections: [None; SUMMARY_POINTS],
    };
}

/// One Result Report table row: a label and its value cells.
///
/// The counts table's rows carry a Breakaway cell and the four tabulated
/// points; the corrections table's rows carry the four points. A label names
/// the row (`L fwd`) or, on a header row, the table (`Counts`, `Corr`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SummaryRow {
    /// The row's label cell.
    label: TextBuf,
    /// The row's value cells, only the first `cell_count` valid.
    cells: [TextBuf; SUMMARY_CELLS],
    /// How many of `cells` are valid.
    cell_count: usize,
}

impl SummaryRow {
    /// An empty row.
    const EMPTY: Self = Self {
        label: TextBuf::new(),
        cells: [TextBuf::new(); SUMMARY_CELLS],
        cell_count: 0,
    };

    /// Build a row from a label and up to [`SUMMARY_CELLS`] value cells.
    fn new(label: TextBuf, cells: &[TextBuf]) -> Self {
        let mut row = Self::EMPTY;
        row.label = label;
        row.cell_count = cells.len().min(SUMMARY_CELLS);
        row.cells[..row.cell_count].copy_from_slice(&cells[..row.cell_count]);
        row
    }

    /// The row's label, e.g. `L fwd` or `Counts`.
    #[must_use]
    pub fn label(&self) -> &str {
        self.label.as_str()
    }

    /// The row's value cells, in column order.
    #[must_use]
    pub fn cells(&self) -> &[TextBuf] {
        &self.cells[..self.cell_count]
    }
}

/// Collects the twenty Power Step outcomes of a Basic Motor Test and builds the
/// [`BasicMotorSummary`] the Result Report draws.
///
/// The task records every Power Step as it completes, so the builder only ever
/// holds the four tabulated points and the running Breakaway per Test Leg. The
/// live readout keeps its own six-row model; the summary is built once, after
/// the run, and published instead.
#[derive(Clone, Copy, Debug)]
pub struct BasicMotorSummaryBuilder {
    /// The four Test Legs, in cycle order, as their Result Report rows.
    legs: [LegSummary; SUMMARY_LEGS],
}

impl BasicMotorSummaryBuilder {
    /// A builder with no outcomes recorded.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            legs: [LegSummary::EMPTY; SUMMARY_LEGS],
        }
    }

    /// Record one Power Step of `leg` at `power` percent.
    ///
    /// `count` is the Measurement Window count, or `None` when the measurement
    /// was invalid; `moved` says whether that window showed sustained motion,
    /// and `correction` is the curve in force at that step. Only the four
    /// tabulated powers keep a count or correction, but every power can latch
    /// the Breakaway, because a leg may first move below 40 %.
    pub fn record(&mut self, leg: TestLeg, power: u8, count: Option<u16>, moved: bool, correction: Correction) {
        let row = &mut self.legs[leg_index(leg)];
        if moved {
            row.breakaway = Some(row.breakaway.map_or(power, |current| current.min(power)));
        }
        let Some(point) = summary_point(power) else {
            return;
        };
        // A point that did not move keeps `--`, never a stray sub-threshold count,
        // even when the measurement produced one. Each point is recorded once, so
        // the verdict can be applied as it arrives.
        row.counts[point] = if moved { count } else { None };
        row.corrections[point] = Some(correction);
    }

    /// Build the summary.
    ///
    /// The Floor and the Spread are left unjudged unless every leg moved and
    /// every 100 % count is present.
    #[must_use]
    pub fn build(&self) -> BasicMotorSummary {
        let legs = self.legs;
        let complete = legs
            .iter()
            .all(|leg| leg.breakaway.is_some() && leg.counts[SUMMARY_POINTS - 1].is_some());
        let floor_percent = if complete {
            legs.iter().filter_map(|leg| leg.breakaway).max()
        } else {
            None
        };
        let spread_percent = if complete { raw_spread_percent(&legs) } else { None };

        BasicMotorSummary {
            legs,
            floor_percent,
            raw_spread_percent: spread_percent,
            corrected: None,
            status: None,
        }
    }
}

impl Default for BasicMotorSummaryBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// The Result Report's summary of a completed Basic Motor Test.
///
/// It carries, for each Test Leg, the Breakaway, the raw counts at 40, 60, 80 and
/// 100 % and the correction in force at those points, plus the Calibration Floor
/// (the highest Breakaway) and the raw spread at 100 %. A Motor Calibration adds
/// the optional [`SummaryStatus`] footer; the Basic Motor Test adds the optional
/// [`CorrectedCheck`] footer instead. Every value is `Copy + Eq`, so the finished
/// status view's redraw comparison stays exact.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BasicMotorSummary {
    /// The four Test Legs, in cycle order.
    legs: [LegSummary; SUMMARY_LEGS],
    /// The highest Breakaway among the four legs, or `None` while the line is
    /// unjudged.
    floor_percent: Option<u8>,
    /// The raw spread at 100 % as a whole percent, or `None` while the line is
    /// unjudged.
    raw_spread_percent: Option<u32>,
    /// The curve-applied verification, or `None` for a Motor Calibration and for
    /// a Basic Motor Test that has not finished its corrected cycle.
    corrected: Option<CorrectedCheck>,
    /// The optional saved/not-saved footer.
    status: Option<SummaryStatus>,
}

impl BasicMotorSummary {
    /// A summary with no measurements: every cell and the Floor/Spread line
    /// render `--`.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            legs: [LegSummary::EMPTY; SUMMARY_LEGS],
            floor_percent: None,
            raw_spread_percent: None,
            corrected: None,
            status: None,
        }
    }

    /// The same summary with a saved/not-saved footer.
    ///
    /// The Motor Calibration sets it before publishing the Result Report; the
    /// Basic Motor Test never does, so its footer row stays empty.
    #[must_use]
    pub const fn with_status(self, status: SummaryStatus) -> Self {
        Self {
            status: Some(status),
            ..self
        }
    }

    /// The same summary with the Basic Motor Test's curve-applied verification.
    #[must_use]
    pub const fn with_corrected_check(self, corrected: CorrectedCheck) -> Self {
        Self {
            corrected: Some(corrected),
            ..self
        }
    }

    /// The curve-applied verification, or `None` when this summary has none.
    #[must_use]
    pub const fn corrected_check(&self) -> Option<CorrectedCheck> {
        self.corrected
    }

    /// The saved/not-saved footer, or `None` when the summary carries none.
    #[must_use]
    pub const fn status(&self) -> Option<SummaryStatus> {
        self.status
    }

    /// Row `index` of the two tables, in draw order: the counts header and four
    /// leg rows, then the corrections header and four leg rows.
    ///
    /// An index beyond the ten table rows returns an empty row, so a renderer can
    /// walk the table without indexing the model. Rows are produced one at a time
    /// so drawing does not allocate the whole table on the task's stack.
    #[must_use]
    pub fn row(&self, index: usize) -> SummaryRow {
        match index {
            0 => SummaryRow::new(
                labelled("Raw cnt"),
                &[
                    labelled("Brk"),
                    labelled("40"),
                    labelled("60"),
                    labelled("80"),
                    labelled("100"),
                ],
            ),
            index if index <= SUMMARY_LEGS => self.count_row(index - 1),
            index if index == SUMMARY_LEGS + 1 => SummaryRow::new(
                labelled("Corr"),
                &[labelled("40"), labelled("60"), labelled("80"), labelled("100")],
            ),
            index if index <= 2 * SUMMARY_LEGS + 1 => self.correction_row(index - (SUMMARY_LEGS + 2)),
            _ => SummaryRow::EMPTY,
        }
    }

    /// The counts row of Test Leg `leg`: its Breakaway and the four counts.
    fn count_row(&self, leg: usize) -> SummaryRow {
        let row = self.legs[leg];
        SummaryRow::new(
            labelled(abbreviated_leg(SUMMARY_LEG_ORDER[leg])),
            &[
                row.breakaway.map_or_else(placeholder, power_value),
                count_cell(row.counts[0]),
                count_cell(row.counts[1]),
                count_cell(row.counts[2]),
                count_cell(row.counts[3]),
            ],
        )
    }

    /// The correction row of Test Leg `leg`: the four corrections in force.
    fn correction_row(&self, leg: usize) -> SummaryRow {
        let row = self.legs[leg];
        SummaryRow::new(
            labelled(abbreviated_leg(SUMMARY_LEG_ORDER[leg])),
            &[
                correction_cell(row.corrections[0]),
                correction_cell(row.corrections[1]),
                correction_cell(row.corrections[2]),
                correction_cell(row.corrections[3]),
            ],
        )
    }

    /// The one unjudged line carrying the Calibration Floor and the raw spread
    /// at 100 %, e.g. `Floor 40%  Raw spread 12%`.
    ///
    /// The spread is over the four **raw** 100 % counts, so it includes
    /// forward-versus-reverse as well as left-versus-right. It is not the
    /// calibration error; the corrected verification is.
    ///
    /// Both values render `--` together while the line is unjudged: a leg that
    /// did not move, or a missing 100 % count, means the robot has no verdict to
    /// offer rather than a passing one.
    #[must_use]
    pub fn floor_line(&self) -> TextBuf {
        let floor = self
            .floor_percent
            .map_or_else(placeholder, |percent| percent_value(u32::from(percent)));
        let spread = self.raw_spread_percent.map_or_else(placeholder, percent_value);
        let mut line = TextBuf::new();
        line.fill(format_args!("Floor {}  Raw spread {}", floor.as_str(), spread.as_str()));
        line
    }

    /// The footer row's line: the saved/not-saved status when there is one, else
    /// the Basic Motor Test's corrected verification.
    #[must_use]
    pub fn footer_line(&self) -> Option<TextBuf> {
        self.status_line().or_else(|| self.precision_line())
    }

    /// The corrected verification line, e.g. `Corr max 1.6% Fwd 90%`.
    ///
    /// `None` when this summary has no corrected result (a Motor Calibration, or
    /// a Basic Motor Test that has not finished its second cycle).
    #[must_use]
    pub fn precision_line(&self) -> Option<TextBuf> {
        let corrected = self.corrected?;
        let mut line = TextBuf::new();
        match corrected {
            CorrectedCheck::NoCurve => line.fill(format_args!("No curve - run Motor")),
            CorrectedCheck::Unavailable => line.fill(format_args!("Corr max --")),
            CorrectedCheck::Measured(max) => {
                let whole = max.tenths / 10;
                let fraction = max.tenths % 10;
                line.fill(format_args!(
                    "Corr max {whole}.{fraction}% {} {}%",
                    max.direction.short_label(),
                    max.power
                ));
            }
        }
        Some(line)
    }

    /// The optional saved/not-saved footer line, e.g. `Saved` or
    /// `Not saved: implausible correction`.
    ///
    /// `None` when the summary carries no footer, so the Basic Motor Test leaves
    /// the footer row blank.
    #[must_use]
    pub fn status_line(&self) -> Option<TextBuf> {
        self.status.map(|status| {
            let mut line = TextBuf::new();
            match status {
                SummaryStatus::Saved => line.fill(format_args!("Saved")),
                SummaryStatus::NotSaved(reason) => line.fill(format_args!("Not saved: {reason}")),
            }
            line
        })
    }
}

/// The index of a Test Leg in cycle order.
const fn leg_index(leg: TestLeg) -> usize {
    match (leg.track, leg.direction) {
        (Track::Left, Direction::Forward) => 0,
        (Track::Left, Direction::Reverse) => 1,
        (Track::Right, Direction::Forward) => 2,
        (Track::Right, Direction::Reverse) => 3,
    }
}

/// The tabulated-point index of `power`, or `None` when the Result Report does
/// not tabulate it.
const fn summary_point(power: u8) -> Option<usize> {
    let mut index = 0;
    while index < SUMMARY_POINTS {
        if SUMMARY_POWERS[index] == power {
            return Some(index);
        }
        index += 1;
    }
    None
}

/// The raw spread at 100 % as a whole percent: `(max - min) / min`, rounded half
/// up, over the four raw counts.
///
/// `None` when the slowest leg measured zero counts, because the ratio is
/// undefined and `--` is the honest answer.
fn raw_spread_percent(legs: &[LegSummary; SUMMARY_LEGS]) -> Option<u32> {
    let mut min = u16::MAX;
    let mut max = 0;
    for leg in legs {
        let count = leg.counts[SUMMARY_POINTS - 1]?;
        min = min.min(count);
        max = max.max(count);
    }
    if min == 0 {
        return None;
    }
    let range = u32::from(max - min) * 100;
    Some((range + u32::from(min) / 2) / u32::from(min))
}

/// Format a tabulated count cell, or `--` when the point did not move.
fn count_cell(count: Option<u16>) -> TextBuf {
    count.map_or_else(placeholder, number)
}

/// Format a correction cell, or `--` when the point was never recorded.
fn correction_cell(correction: Option<Correction>) -> TextBuf {
    correction.map_or_else(placeholder, correction_value)
}

/// Format a whole percent, e.g. `12%`.
fn percent_value(percent: u32) -> TextBuf {
    let mut value = TextBuf::new();
    value.fill(format_args!("{percent}%"));
    value
}
