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
//! Each of the two sensor columns draws seven rows, in order: the Floor-Drop
//! Protection state (`St`), the range in millimetres (`Rng`), the decoded
//! VL53L1X Range Status name and its raw 8-bit code (`Sts`), the signal rate
//! (`Sig`), the ambient rate (`Amb`), the floor state (`Flr`) and the sample age
//! in milliseconds (`Age`). A footer carries the pair state, the aggregate floor
//! state and the floor-drop threshold.
//!
//! The signal and ambient rows convert the driver's raw 7.7 fixed-point count
//! rates to tenths of Mcps, so the model needs no floating point. The status name
//! is decoded by the firmware adapter, not by this model.
//!
//! Every missing value renders as `--`, so a stale reading is never shown as a
//! zero and a row is never omitted for a transient absence.

use crate::buffer::TextBuf;

/// The number of rows each sensor column draws.
pub const ROWS: usize = 7;

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

/// The VL53L1X Range Status name supplied by the firmware adapter.
///
/// The model never decodes the device's raw 8-bit code itself: the adapter
/// turns the code into one of these names and passes both across the boundary.
/// The names and short labels follow the VL53L1X vocabulary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RangeStatusName {
    /// Range complete.
    Valid,
    /// The target is below the minimum range, so the range is clipped.
    MinRangeClipped,
    /// Range is valid but the wrap check was not performed.
    NoWrapCheck,
    /// Too little signal to range.
    SignalFail,
    /// The measured phase is out of bounds.
    OutOfBounds,
    /// The target is closer than the minimum range.
    MinRangeFail,
    /// The signal-to-noise ratio is too low.
    SigmaFail,
    /// The returned range wrapped around and could not be trusted.
    WrapFail,
    /// Crosstalk between the SPADs corrupted the range.
    XtalkFail,
    /// A hardware failure.
    HardwareFail,
    /// The measurement could not be synchronized.
    Synchronization,
    /// The measurement was not updated.
    NoUpdate,
    /// A status the adapter could not name.
    Unknown,
}

impl RangeStatusName {
    /// The short label the status row draws.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Valid => "Valid",
            Self::MinRangeClipped => "MinClip",
            Self::NoWrapCheck => "NoWrap",
            Self::SignalFail => "Signal",
            Self::OutOfBounds => "Phase",
            Self::MinRangeFail => "MinRange",
            Self::SigmaFail => "Sigma",
            Self::WrapFail => "Wrap",
            Self::XtalkFail => "Xtalk",
            Self::HardwareFail => "HW",
            Self::Synchronization => "Sync",
            Self::NoUpdate => "NoUpd",
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
    /// The decoded Range Status name, or `None` before the first sample.
    pub status: Option<RangeStatusName>,
    /// The last raw 8-bit Range Status code, or `None` before the first sample.
    pub raw_status: Option<u8>,
    /// The raw 7.7 fixed-point signal rate, or `None` before the first sample.
    pub signal_rate: Option<u16>,
    /// The raw 7.7 fixed-point ambient rate, or `None` before the first sample.
    pub ambient_rate: Option<u16>,
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
            status: None,
            raw_status: None,
            signal_rate: None,
            ambient_rate: None,
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
    /// status, signal, ambient, floor and age.
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

/// Format one sensor's seven cells, in display order.
fn sensor_cells(sensor: &SensorReadout) -> [TextBuf; ROWS] {
    let mut cells = [TextBuf::new(); ROWS];
    cells[0] = state_cell(sensor.state);
    cells[1] = range_cell(sensor.range_mm);
    cells[2] = status_cell(sensor.status, sensor.raw_status);
    cells[3] = rate_cell("Sig", sensor.signal_rate);
    cells[4] = rate_cell("Amb", sensor.ambient_rate);
    cells[5] = floor_cell(sensor.floor);
    cells[6] = age_cell(sensor.age_ms);
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

/// Format the status row as the adapter-decoded name and its raw 8-bit code,
/// with `--` when either part is missing.
fn status_cell(status: Option<RangeStatusName>, raw_status: Option<u8>) -> TextBuf {
    let mut cell = TextBuf::new();
    match (status, raw_status) {
        (Some(status), Some(raw)) => cell.fill(format_args!("Sts {} ({raw})", status.label())),
        _ => cell.fill(format_args!("Sts --")),
    }
    cell
}

/// Format a raw 7.7 fixed-point count rate as tenths of Mcps, e.g. `Sig 12.3`.
///
/// Integer rounding keeps the model free of floating point: a raw count of
/// `1574` renders as `12.3` and `51` as `0.4`.
fn rate_cell(label: &str, raw_rate: Option<u16>) -> TextBuf {
    let mut cell = TextBuf::new();
    match raw_rate {
        Some(raw) => {
            let tenths = rate_tenths(raw);
            cell.fill(format_args!("{label} {}.{}", tenths / 10, tenths % 10));
        }
        None => cell.fill(format_args!("{label} --")),
    }
    cell
}

/// Convert a raw 7.7 fixed-point rate to tenths of Mcps.
fn rate_tenths(raw: u16) -> u32 {
    (u32::from(raw) * 5 + 32) / 64
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
