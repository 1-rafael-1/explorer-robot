//! The neutral Basic Motor Test Readout and its row formatting.
//!
//! The Basic Motor Test publishes one of these as it runs; this module owns the
//! labels, the percent formatting and the placeholders the panel draws. A Test
//! Leg is named by [`TestLeg`], [`Track`] and [`Direction`], and the firmware
//! never formats text, so no display string crosses this boundary. The same
//! [`TestLeg::label`] names the Activity's phase line, so the panel and the
//! `ACTIVITY` stream cannot drift apart.
//!
//! The readout also owns the rules the task would otherwise spread through its
//! loops. It accepts three transitions — start a Power Step, sample a count, and
//! complete a Power Step — and derives every row from them. Starting a step does
//! not clear the displayed count: the encoder reset that precedes a step empties
//! the hardware, not the operator's reading, so the live count holds until the
//! next sample replaces it. Completing a step latches the Breakaway at the first
//! step of a Test Leg that moved, and starting a step on a different Test Leg
//! clears it, so each leg reports its own.
//!
//! # Rows
//!
//! Five rows carry the current Power Step, the live encoder count, the last
//! completed Power Step with the count it finished with, the Breakaway, and the
//! position within the cycle. A value that does not exist yet renders `--`, never
//! `0`, so "no data yet" can never be mistaken for "zero pulses". A Breakaway
//! that is still `--` at the end of a Test Leg is the verdict that the track did
//! not move at any power up to 100 %.

use crate::buffer::TextBuf;

/// The number of rows this readout draws.
pub const ROWS: usize = 5;

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

/// A Power Step that has finished: the leg and power it commanded and the count
/// it ended with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CompletedStep {
    /// The leg under test.
    leg: TestLeg,
    /// The commanded power, in percent.
    power: u8,
    /// The count captured as the step ended, before the coast.
    count: u16,
}

/// The neutral Basic Motor Test Readout: the current Power Step, the live encoder
/// count, the last completed Power Step, the Breakaway and the step index.
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
    /// The lowest power of `leg` whose completed count was greater than zero, or
    /// `None` before a step in the leg moved.
    breakaway: Option<u8>,
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
            index: None,
        }
    }

    /// Begin a Power Step on `leg` at `power` percent, at `index` within the
    /// cycle.
    ///
    /// The displayed count is deliberately left alone: the encoder reset has
    /// already happened, and the operator's last reading must not flicker to a
    /// placeholder while the new command settles. A step on a different Test Leg
    /// clears the Breakaway, so each leg reports its own.
    pub const fn start_step(&mut self, leg: TestLeg, power: u8, index: StepIndex) {
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
        self.commanding = true;
    }

    /// Replace the live encoder count with the latest sample.
    pub const fn sample(&mut self, count: u16) {
        self.count = Some(count);
    }

    /// End the current Power Step, keeping the count it finished with.
    ///
    /// The step's own leg, power and final count are retained for the `Last`
    /// row. A step that moved while the leg's Breakaway is still absent latches
    /// it, so the row names the lowest power at which the track moved. The
    /// current-step row keeps naming the step that just ran, so the panel does
    /// not blank for the coast between steps. A completion with no step in
    /// progress is ignored, so a stray one cannot overwrite what the operator is
    /// reading.
    pub const fn complete(&mut self, count: u16) {
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
        if count > 0 && self.breakaway.is_none() {
            self.breakaway = Some(step.power);
        }
        self.count = Some(count);
        self.commanding = false;
    }

    /// The formatted label/value pairs, in row order: the current Power Step, the
    /// live encoder count, the last completed Power Step, the Breakaway and the
    /// position within the cycle.
    #[must_use]
    pub fn rows(&self) -> [(TextBuf, TextBuf); ROWS] {
        let (step_label, step_value) = self.step.map_or_else(
            || (placeholder(), placeholder()),
            |step| (leg_label(step.leg), power_value(step.power)),
        );

        let mut count_label = TextBuf::new();
        count_label.fill(format_args!("Count"));
        let count_value = self.count.map_or_else(placeholder, |count| {
            let mut value = TextBuf::new();
            value.fill(format_args!("{count}"));
            value
        });

        let mut last_label = TextBuf::new();
        last_label.fill(format_args!("Last"));
        let last_value = self.last.map_or_else(placeholder, completed_value);

        let mut breakaway_label = TextBuf::new();
        breakaway_label.fill(format_args!("Breakaway"));
        let breakaway_value = self.breakaway.map_or_else(placeholder, power_value);

        let mut index_label = TextBuf::new();
        index_label.fill(format_args!("Step"));
        let index_value = self.index.map_or_else(placeholder, step_index_value);

        [
            (step_label, step_value),
            (count_label, count_value),
            (last_label, last_value),
            (breakaway_label, breakaway_value),
            (index_label, index_value),
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

/// Spell a Test Leg out as its row label, e.g. `Left forward`.
fn leg_label(leg: TestLeg) -> TextBuf {
    let mut label = TextBuf::new();
    label.fill(format_args!("{}", leg.label()));
    label
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
/// e.g. `L fwd 50%  987`.
fn completed_value(step: CompletedStep) -> TextBuf {
    let CompletedStep { leg, power, count } = step;
    let leg = abbreviated_leg(leg);
    let mut value = TextBuf::new();
    value.fill(format_args!("{leg} {power}%  {count}"));
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

/// A cell with no value yet.
fn placeholder() -> TextBuf {
    let mut cell = TextBuf::new();
    cell.fill(format_args!("--"));
    cell
}
