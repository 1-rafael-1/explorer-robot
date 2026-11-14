//! The neutral Rangefinder Test Readout and its row formatting.
//!
//! The firmware's Rangefinder Test publishes typed per-sensor telemetry into a
//! [`RangefinderTestReadout`]; this module owns the labels, units, raw-status
//! naming and placeholders the panel draws. No firmware or driver type crosses
//! this boundary: the readout carries plain optional integers and enums, and the
//! model never allocates — every cell is a fixed-capacity [`TextBuf`].
//!
//! # Rows
//!
//! Each of the two sensor columns draws five rows, in order: the Floor-Drop
//! Protection state (`St`), the range in millimetres (`Rng`), the decoded raw
//! Range Status and its four-bit code (`Sts`), the floor state (`Flr`) and the
//! sample age in milliseconds (`Age`). A footer carries the pair state, the
//! aggregate floor state and the floor-drop threshold.
//!
//! Every missing value renders as `--`, so a stale reading is never shown as a
//! zero and a row is never omitted for a transient absence.

use crate::buffer::TextBuf;

/// The number of rows each sensor column draws.
pub const ROWS: usize = 5;

/// The Floor-Drop Protection state of the pair, or one sensor's contribution to
/// it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProtectionState {
    /// The pair is disabled.
    Off,
    /// The pair is waiting for a first valid reading from both sensors.
    Arming,
    /// Both sensors are healthy and motion is allowed.
    Armed,
    /// One sensor cannot arm or recover; motion is blocked.
    Degraded,
    /// Both sensors cannot arm or recover.
    Failed,
}

impl ProtectionState {
    /// The short label the rows draw, kept to four characters so two columns fit
    /// side by side.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Arming => "Arming",
            Self::Armed => "Armed",
            Self::Degraded => "Degr",
            Self::Failed => "Fail",
        }
    }
}

/// Whether one sensor currently sees the floor, a drop, or nothing usable yet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FloorState {
    /// The floor state is not known yet, or the reading is too old.
    Unknown,
    /// The sensor sees the floor inside the threshold.
    Floor,
    /// The sensor sees past the threshold, so the floor has dropped away.
    Drop,
}

impl FloorState {
    /// The label the floor row draws.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Unknown => "Unknown",
            Self::Floor => "Floor",
            Self::Drop => "Drop",
        }
    }
}

/// The name a four-bit VL53L0X Range Status code decodes to.
///
/// The raw code is the device's `0x14` bits 6:3, not the full register byte.
/// `11` is range complete; the other codes name the failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RangeStatusName {
    /// Range complete.
    Valid,
    /// The device reports no range.
    NoRange,
    /// A hardware failure (`1`–`3`).
    HardwareFail,
    /// A signal failure (`4`).
    SignalFail,
    /// A phase failure (`6` or `9`).
    PhaseFail,
    /// A minimum-range failure (`8` or `10`).
    MinRangeFail,
    /// A code with no known meaning.
    Unknown,
}

impl RangeStatusName {
    /// Decode a four-bit Range Status code.
    ///
    /// Any code outside the driver's vocabulary decodes to
    /// [`RangeStatusName::Unknown`] rather than being guessed.
    #[must_use]
    pub const fn from_raw(raw: u8) -> Self {
        match raw {
            11 => Self::Valid,
            0 => Self::NoRange,
            1..=3 => Self::HardwareFail,
            4 => Self::SignalFail,
            6 | 9 => Self::PhaseFail,
            8 | 10 => Self::MinRangeFail,
            _ => Self::Unknown,
        }
    }

    /// The short label the status row draws.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Valid => "Valid",
            Self::NoRange => "No range",
            Self::HardwareFail => "HW",
            Self::SignalFail => "Signal",
            Self::PhaseFail => "Phase",
            Self::MinRangeFail => "Min",
            Self::Unknown => "Unknown",
        }
    }
}

/// One sensor's neutral Rangefinder Reading and floor-drop telemetry.
///
/// Every field is optional so a missing sample stays distinguishable from a
/// real zero; the model draws `--` for each absent value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorReadout {
    /// The sensor's contribution to the pair's Floor-Drop Protection state, or
    /// `None` before the first sample.
    pub state: Option<ProtectionState>,
    /// The last range in millimetres, or `None` before the first sample.
    pub range_mm: Option<u16>,
    /// The last four-bit Range Status code, or `None` before the first sample.
    pub raw_status: Option<u8>,
    /// The derived floor state, or `None` before the first sample.
    pub floor: Option<FloorState>,
    /// The age of the last sample in milliseconds, or `None` before the first
    /// sample.
    pub age_ms: Option<u32>,
}

impl SensorReadout {
    /// A sensor with no sample yet.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            state: None,
            range_mm: None,
            raw_status: None,
            floor: None,
            age_ms: None,
        }
    }
}

/// The neutral Rangefinder Test Readout: both sensors, the pair state and the
/// threshold the panel formats into columns.
///
/// The firmware builds one of these each publish tick and hands it to the UI
/// through the telemetry snapshot. [`RangefinderTestReadout::waiting`] is the
/// state before either sensor has produced a sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RangefinderTestReadout {
    /// The front sensor's telemetry.
    pub front: SensorReadout,
    /// The rear sensor's telemetry.
    pub rear: SensorReadout,
    /// The pair's Floor-Drop Protection state, or `None` before the first sample.
    pub pair: Option<ProtectionState>,
    /// The aggregate floor state: `true` when either sensor sees a drop, or
    /// `None` before the first sample.
    pub floor_drop: Option<bool>,
    /// The floor-drop threshold in millimetres.
    pub threshold_mm: u16,
}

impl RangefinderTestReadout {
    /// A readout with no samples yet and the given floor-drop threshold.
    #[must_use]
    pub const fn waiting(threshold_mm: u16) -> Self {
        Self {
            front: SensorReadout::empty(),
            rear: SensorReadout::empty(),
            pair: None,
            floor_drop: None,
            threshold_mm,
        }
    }

    /// The formatted cells, `[front, rear]`, each in row order: state, range,
    /// status, floor and age.
    #[must_use]
    pub fn cells(&self) -> [[TextBuf; ROWS]; 2] {
        [sensor_cells(&self.front), sensor_cells(&self.rear)]
    }

    /// The formatted footer: the pair state, the aggregate floor state and the
    /// threshold.
    #[must_use]
    pub fn footer(&self) -> TextBuf {
        let pair = self.pair.map_or("--", ProtectionState::label);
        let floor = match self.floor_drop {
            Some(true) => "DROP",
            Some(false) => "CLEAR",
            None => "--",
        };
        let mut footer = TextBuf::new();
        footer.fill(format_args!("Pair {pair}  Flr {floor}  TH {}mm", self.threshold_mm));
        footer
    }
}

/// Format one sensor's five cells, in display order.
fn sensor_cells(sensor: &SensorReadout) -> [TextBuf; ROWS] {
    let mut cells = [TextBuf::new(); ROWS];
    cells[0] = state_cell(sensor.state);
    cells[1] = range_cell(sensor.range_mm);
    cells[2] = status_cell(sensor.raw_status);
    cells[3] = floor_cell(sensor.floor);
    cells[4] = age_cell(sensor.age_ms);
    cells
}

/// Format the state row, with `--` for a sensor that has not sampled yet.
fn state_cell(state: Option<ProtectionState>) -> TextBuf {
    let mut cell = TextBuf::new();
    match state {
        Some(state) => cell.fill(format_args!("St {}", state.label())),
        None => cell.fill(format_args!("St --")),
    }
    cell
}

/// Format the range row in millimetres, with `--` when there is no reading.
fn range_cell(range_mm: Option<u16>) -> TextBuf {
    let mut cell = TextBuf::new();
    match range_mm {
        Some(range_mm) => cell.fill(format_args!("Rng {range_mm}mm")),
        None => cell.fill(format_args!("Rng --")),
    }
    cell
}

/// Format the status row as the decoded name and its raw four-bit code, with
/// `--` when there is no reading.
fn status_cell(raw_status: Option<u8>) -> TextBuf {
    let mut cell = TextBuf::new();
    match raw_status {
        Some(raw) => cell.fill(format_args!("Sts {} ({raw})", RangeStatusName::from_raw(raw).label())),
        None => cell.fill(format_args!("Sts --")),
    }
    cell
}

/// Format the floor row, with `--` when the floor state is not known.
fn floor_cell(floor: Option<FloorState>) -> TextBuf {
    let mut cell = TextBuf::new();
    match floor {
        Some(floor) => cell.fill(format_args!("Flr {}", floor.label())),
        None => cell.fill(format_args!("Flr --")),
    }
    cell
}

/// Format the age row in milliseconds, with `--` when there is no sample.
fn age_cell(age_ms: Option<u32>) -> TextBuf {
    let mut cell = TextBuf::new();
    match age_ms {
        Some(age_ms) => cell.fill(format_args!("Age {age_ms}ms")),
        None => cell.fill(format_args!("Age --")),
    }
    cell
}
