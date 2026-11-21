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
//! loops. It accepts three transitions — start a Power Step, sample a count, and
//! complete a Power Step — and derives every row from them. Starting a step does
//! not clear the displayed count: the encoder reset that precedes a step empties
//! the hardware, not the operator's reading, so the live count holds until the
//! next sample replaces it. Completing a step latches the Breakaway at the first
//! step of a Test Leg whose Measurement Window showed sustained motion, and
//! starting a step on a different Test Leg clears it, so each leg reports its
//! own.
//!
//! # Rows
//!
//! Six rows carry the current Power Step, the live encoder count, the last
//! completed Power Step with the count it finished with, the Breakaway, the
//! correction the curve in force applies at the current Power Step, and the
//! position within the cycle. A value that does not exist yet renders `--`, never
//! `0`, so "no data yet" can never be mistaken for "zero pulses". A Breakaway
//! that is still `--` at the end of a Test Leg is the verdict that the track did
//! not move at any power up to 100 %.
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
//! built once from the forty Power Step outcomes the task recorded, so the live
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
/// Motor Test leaves it absent, because it never writes a curve. The reason is a
/// short stable phrase, so the footer stays `Copy + Eq` and never allocates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SummaryStatus {
    /// The calibration was written to flash and is now the curve in force.
    Saved,
    /// The run was not saved, with the reason the operator needs to see.
    NotSaved(&'static str),
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

/// The Power Step the panel names: one Test Leg at one power level.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CurrentStep {
    /// The leg under test.
    leg: TestLeg,
    /// The commanded power, in percent.
    power: u8,
}

/// A Power Step that has finished: the leg and power it commanded and the
/// Measurement Window count it ended with, or `None` when the point was invalid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CompletedStep {
    /// The leg under test.
    leg: TestLeg,
    /// The commanded power, in percent.
    power: u8,
    /// The Measurement Window count, or `None` when the measurement was invalid.
    count: Option<u16>,
}

/// The neutral Basic Motor Test Readout: the current Power Step, the live encoder
/// count, the last completed Power Step, the Breakaway, the correction in force
/// and the step index.
///
/// [`BasicMotorReadout::waiting`] is the state before the first Power Step. The
/// firmware holds one locally, publishes a copy at the sampling rate, and drives
/// it with [`BasicMotorReadout::start_step`], [`BasicMotorReadout::sample`] and
/// [`BasicMotorReadout::complete`]; the model decides what each row says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BasicMotorReadout {
    /// The step the panel names: the one commanding, or the one that last
    /// commanded. It is deliberately kept past a completion, so the row does not
    /// blank between Power Steps and the operator always knows which command the
    /// numbers belong to.
    step: Option<CurrentStep>,
    /// Whether `step` is still commanding, so a completion that arrives with no
    /// step in progress cannot overwrite what the operator is reading.
    commanding: bool,
    /// The live encoder count for the track under test, or `None` before the
    /// first sample.
    count: Option<u16>,
    /// The most recently completed Power Step, or `None` before the first
    /// completion.
    last: Option<CompletedStep>,
    /// The Test Leg currently or most recently under test, retained past a
    /// completion so the next leg's first step can clear the Breakaway.
    leg: Option<TestLeg>,
    /// The lowest power of `leg` whose Measurement Window showed sustained
    /// motion, or `None` before a step in the leg moved.
    breakaway: Option<u8>,
    /// The correction the curve in force applies to the current step, or `None`
    /// before the first step.
    correction: Option<Correction>,
    /// The position within the cycle of the current or most recently completed
    /// Power Step, or `None` before the first step.
    index: Option<StepIndex>,
}

impl BasicMotorReadout {
    /// A readout with no Power Step, no sample and no completed step yet.
    #[must_use]
    pub const fn waiting() -> Self {
        Self {
            step: None,
            commanding: false,
            count: None,
            last: None,
            leg: None,
            breakaway: None,
            correction: None,
            index: None,
        }
    }

    /// Begin a Power Step on `leg` at `power` percent, at `index` within the
    /// cycle, with `correction` the curve in force for this leg and power.
    ///
    /// The displayed count is deliberately left alone: the encoder reset has
    /// already happened, and the operator's last reading must not flicker to a
    /// placeholder while the new command settles. A step on a different Test Leg
    /// clears the Breakaway, so each leg reports its own.
    pub const fn start_step(&mut self, leg: TestLeg, power: u8, index: StepIndex, correction: Correction) {
        let same = match self.leg {
            Some(current) => same_leg(current, leg),
            None => false,
        };
        if !same {
            self.breakaway = None;
        }
        self.leg = Some(leg);
        self.index = Some(index);
        self.step = Some(CurrentStep { leg, power });
        self.correction = Some(correction);
        self.commanding = true;
    }

    /// Replace the live encoder count with the latest raw sample for the track
    /// under test.
    pub const fn sample(&mut self, count: u16) {
        self.count = Some(count);
    }

    /// End the current Power Step.
    ///
    /// `count` is the Measurement Window count, or `None` when the measurement
    /// was invalid, in which case the `Last` row shows `--` rather than `0`.
    /// `moved` says whether that window showed sustained motion; only a moving
    /// step latches the Breakaway, so a stray pulse cannot be mistaken for
    /// motion.
    ///
    /// The step's own leg and power are retained for the `Last` row, and the
    /// step's own power latches the Breakaway while it is still absent. The
    /// current-step row keeps naming the step that just ran, so the panel does
    /// not blank for the coast between steps. A completion with no step in
    /// progress is ignored, so a stray one cannot overwrite what the operator is
    /// reading.
    pub const fn complete(&mut self, count: Option<u16>, moved: bool) {
        if !self.commanding {
            return;
        }
        let Some(step) = self.step else {
            return;
        };

        self.last = Some(CompletedStep {
            leg: step.leg,
            power: step.power,
            count,
        });
        if moved && self.breakaway.is_none() {
            self.breakaway = Some(step.power);
        }
        self.commanding = false;
    }

    /// The formatted label/value pairs, in row order: the current Power Step, the
    /// live encoder count, the last completed Power Step, the Breakaway, the
    /// correction in force and the position within the cycle.
    #[must_use]
    pub fn rows(&self) -> [(TextBuf, TextBuf); ROWS] {
        let (step_label, step_value) = self.step.map_or_else(
            || (placeholder(), placeholder()),
            |step| (labelled(step.leg.label()), power_value(step.power)),
        );

        let count_value = self.count.map_or_else(placeholder, number);

        let last_value = self.last.map_or_else(placeholder, completed_value);

        let breakaway_value = self.breakaway.map_or_else(placeholder, power_value);

        let correction_cell = self.correction.map_or_else(placeholder, correction_value);

        let index_value = self.index.map_or_else(placeholder, step_index_value);

        [
            (step_label, step_value),
            (labelled("Count"), count_value),
            (labelled("Last"), last_value),
            (labelled("Breakaway"), breakaway_value),
            (labelled("Correction"), correction_cell),
            (labelled("Step"), index_value),
        ]
    }
}

/// Whether two Test Legs name the same track and direction.
const fn same_leg(a: TestLeg, b: TestLeg) -> bool {
    matches!(
        (a.track, a.direction, b.track, b.direction),
        (Track::Left, Direction::Forward, Track::Left, Direction::Forward)
            | (Track::Left, Direction::Reverse, Track::Left, Direction::Reverse)
            | (Track::Right, Direction::Forward, Track::Right, Direction::Forward)
            | (Track::Right, Direction::Reverse, Track::Right, Direction::Reverse)
    )
}

/// Spell a Test Leg as the `Last` row's abbreviation, e.g. `L fwd`.
const fn abbreviated_leg(leg: TestLeg) -> &'static str {
    match (leg.track, leg.direction) {
        (Track::Left, Direction::Forward) => "L fwd",
        (Track::Left, Direction::Reverse) => "L rev",
        (Track::Right, Direction::Forward) => "R fwd",
        (Track::Right, Direction::Reverse) => "R rev",
    }
}

/// Format the last completed Power Step: its abbreviated leg, power and count,
/// e.g. `L fwd 50%  987`, or `L fwd 50%  --` for an invalid point.
fn completed_value(step: CompletedStep) -> TextBuf {
    let CompletedStep { leg, power, count } = step;
    let leg = abbreviated_leg(leg);
    let mut value = TextBuf::new();
    match count {
        Some(count) => value.fill(format_args!("{leg} {power}%  {count}")),
        None => value.fill(format_args!("{leg} {power}%  --")),
    }
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

/// Format a position within the cycle as `n/total`, e.g. `12/40`.
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

/// Collects the forty Power Step outcomes of a Basic Motor Test and builds the
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
        let spread_percent = if complete { spread_percent(&legs) } else { None };

        BasicMotorSummary {
            legs,
            floor_percent,
            spread_percent,
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
/// It carries, for each Test Leg, the Breakaway, the counts at 40, 60, 80 and
/// 100 % and the correction in force at those points, plus the Calibration Floor
/// (the highest Breakaway) and the spread at 100 %. A Motor Calibration adds the
/// optional [`SummaryStatus`] footer; the Basic Motor Test leaves it absent.
/// Every value is `Copy + Eq`, so the finished status view's redraw comparison
/// stays exact.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BasicMotorSummary {
    /// The four Test Legs, in cycle order.
    legs: [LegSummary; SUMMARY_LEGS],
    /// The highest Breakaway among the four legs, or `None` while the line is
    /// unjudged.
    floor_percent: Option<u8>,
    /// The spread at 100 % as a whole percent, or `None` while the line is
    /// unjudged.
    spread_percent: Option<u32>,
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
            spread_percent: None,
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
                labelled("Counts"),
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

    /// The one unjudged line carrying the Calibration Floor and the spread at
    /// 100 %, e.g. `Floor 40%  Spread 12%`.
    ///
    /// Both values render `--` together while the line is unjudged: a leg that
    /// did not move, or a missing 100 % count, means the robot has no verdict to
    /// offer rather than a passing one.
    #[must_use]
    pub fn floor_line(&self) -> TextBuf {
        let floor = self
            .floor_percent
            .map_or_else(placeholder, |percent| percent_value(u32::from(percent)));
        let spread = self.spread_percent.map_or_else(placeholder, percent_value);
        let mut line = TextBuf::new();
        line.fill(format_args!("Floor {}  Spread {}", floor.as_str(), spread.as_str()));
        line
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

/// The spread at 100 % as a whole percent: `(max - min) / min`, rounded half up.
///
/// `None` when the slowest leg measured zero counts, because the ratio is
/// undefined and `--` is the honest answer.
fn spread_percent(legs: &[LegSummary; SUMMARY_LEGS]) -> Option<u32> {
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
