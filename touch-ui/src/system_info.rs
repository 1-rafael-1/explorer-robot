//! The neutral System Info snapshot and its row formatting.
//!
//! The firmware maps its own sensor and calibration state onto [`SystemInfo`]
//! and hands it to the UI; no firmware type crosses this boundary. The row
//! wording matches the firmware's former `format_battery_level_line`,
//! `format_battery_voltage_line`, and `format_cal_line` helpers.

use crate::buffer::TextBuf;

/// The status of one calibration, as shown on the System Info screen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationStatus {
    /// A calibration is loaded and usable.
    Loaded,
    /// It is not known whether a calibration is loaded.
    Unknown,
    /// No calibration is available.
    Missing,
}

impl CalibrationStatus {
    /// The human-readable label for this status.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Loaded => "Loaded",
            Self::Unknown => "Unknown",
            Self::Missing => "Missing",
        }
    }
}

impl Default for CalibrationStatus {
    /// The conservative default: a calibration whose state is not yet known.
    fn default() -> Self {
        Self::Unknown
    }
}

/// A neutral snapshot of the robot state the System Info screen shows.
///
/// `battery_level` and `battery_voltage` are `None` until a reading is
/// available; the three calibration statuses default to
/// [`CalibrationStatus::Unknown`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SystemInfo {
    /// Battery level as a percentage, or `None` when unread.
    pub battery_level: Option<u8>,
    /// Battery voltage in volts, or `None` when unread.
    pub battery_voltage: Option<f32>,
    /// Motor calibration status.
    pub motor_calibration: CalibrationStatus,
    /// Magnetometer calibration status.
    pub mag_calibration: CalibrationStatus,
    /// Distance calibration status.
    pub distance_calibration: CalibrationStatus,
}

impl SystemInfo {
    /// A snapshot with no battery reading and every calibration unknown.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            battery_level: None,
            battery_voltage: None,
            motor_calibration: CalibrationStatus::Unknown,
            mag_calibration: CalibrationStatus::Unknown,
            distance_calibration: CalibrationStatus::Unknown,
        }
    }

    /// The five rows, in display order.
    #[must_use]
    pub fn rows(&self) -> [TextBuf; 5] {
        [
            battery_level_line(self.battery_level),
            battery_voltage_line(self.battery_voltage),
            calibration_line("Motor", self.motor_calibration),
            calibration_line("Mag", self.mag_calibration),
            calibration_line("Dist", self.distance_calibration),
        ]
    }
}

impl Default for SystemInfo {
    /// Equivalent to [`SystemInfo::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// Format the battery level row, e.g. `Batt  87%` or `Batt --%`.
fn battery_level_line(level: Option<u8>) -> TextBuf {
    let mut line = TextBuf::new();
    match level {
        Some(lvl) => line.fill(format_args!("Batt {lvl:>3}%")),
        None => line.fill(format_args!("Batt --%")),
    }
    line
}

/// Format the battery voltage row, e.g. `Batt  7.4V` or `Batt --.-V`.
fn battery_voltage_line(voltage: Option<f32>) -> TextBuf {
    let mut line = TextBuf::new();
    match voltage {
        Some(volts) => line.fill(format_args!("Batt {volts:>4.1}V")),
        None => line.fill(format_args!("Batt --.-V")),
    }
    line
}

/// Format one calibration row, e.g. `Motor: Loaded`.
fn calibration_line(prefix: &'static str, status: CalibrationStatus) -> TextBuf {
    let mut line = TextBuf::new();
    line.fill(format_args!("{prefix}: {label}", label = status.label()));
    line
}
