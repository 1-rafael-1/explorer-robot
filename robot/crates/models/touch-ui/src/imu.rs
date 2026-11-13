//! The neutral IMU Test Telemetry readout and its row formatting.
//!
//! The firmware's IMU test tasks publish typed sensor values into an
//! [`ImuReadout`]; this module owns the labels, units, precision, placeholders
//! and row layout the panel draws. No firmware type crosses this boundary: the
//! readout carries plain `f32` triples and optional values, and the model never
//! allocates — every row is a fixed-capacity [`TextBuf`].
//!
//! # Rows
//!
//! A six-axis readout has five rows, in order: Euler angles, the raw
//! accelerometer, the raw gyroscope, the DMP-calibrated gyroscope, and the health
//! line. A nine-axis readout has twelve rows: the same four vectors, the raw and
//! DMP-calibrated magnetometer, the heading line, the XY, XZ and YZ heading-remap
//! rows, the health line and the magnetometer gate. Absent values render as `--`,
//! so a row is never omitted for a transient absence. The health line names the
//! state, the consecutive missing-sample count and the DMP calibration;
//! the six-axis screen explains the absent magnetometer section with `dmpcal n/a`,
//! and an unavailable nine-axis magnetometer keeps the health line, shows
//! `mag unavailable - 6-axis fallback use_mag=no` on the gate row, and renders
//! `--` heading and remap rows.

use crate::{buffer::TextBuf, screens::Procedure, system_info::CalibrationStatus};

/// The fixed number of row slots a readout can fill.
///
/// The six-axis readout uses five and the nine-axis readout uses all twelve.
pub const MAX_IMU_ROWS: usize = 12;

/// Number of rows in the six-axis readout.
const SIX_AXIS_ROWS: usize = 5;

/// Number of rows in the nine-axis readout.
const NINE_AXIS_ROWS: usize = 12;

/// The largest missing count the health row prints.
const MAX_MISSING: u32 = 9999;

/// Which readout a Procedure shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuReadoutKind {
    /// The six-axis test: Euler, accelerometer, gyroscope and health.
    SixAxis,
    /// The nine-axis test: the six-axis rows plus the magnetometer vectors,
    /// health and gate line.
    NineAxis,
}

impl ImuReadoutKind {
    /// The readout a Procedure shows, or `None` for a Procedure that shows no
    /// readout.
    #[must_use]
    pub const fn for_procedure(procedure: Procedure) -> Option<Self> {
        match procedure {
            Procedure::Imu6Axis => Some(Self::SixAxis),
            Procedure::Imu9Axis => Some(Self::NineAxis),
            _ => None,
        }
    }
}

/// How the IMU test's samples are arriving.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImuHealth {
    /// The test has started but the IMU has not answered yet.
    Waiting,
    /// Samples are arriving.
    Live,
    /// About two seconds have passed without a sample.
    NotResponding,
}

impl ImuHealth {
    /// The short label the health row draws.
    const fn label(self) -> &'static str {
        match self {
            Self::Waiting => "Waiting",
            Self::Live => "Live",
            Self::NotResponding => "IMU not responding",
        }
    }
}

/// Fused orientation in Euler angles, in degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Euler {
    /// Heading / compass direction.
    pub yaw: f32,
    /// Forward / backward tilt.
    pub pitch: f32,
    /// Left / right tilt.
    pub roll: f32,
}

/// A three-component sensor vector in the row's unit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector3 {
    /// X component.
    pub x: f32,
    /// Y component.
    pub y: f32,
    /// Z component.
    pub z: f32,
}

/// The magnetometer diagnostics the nine-axis test computes.
///
/// The test task owns the diagnostic math: it publishes the raw-field norm, the
/// driver's gate verdict, the raw and tilt-compensated headings, and the eight
/// heading remaps for each plane. The model only formats them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagDiagnostics {
    /// Raw magnetometer norm in µT.
    pub norm: f32,
    /// The driver's norm gate verdict.
    pub use_mag: bool,
    /// The raw XY heading in degrees.
    pub raw_heading: f32,
    /// The tilt-compensated heading in degrees, or `None` when no accelerometer
    /// sample was available to compute it.
    pub tilt_heading: Option<f32>,
    /// The eight XY-plane heading remaps in degrees, one per axis-sign
    /// permutation.
    pub remap_xy: [f32; 8],
    /// The eight XZ-plane heading remaps in degrees, one per axis-sign
    /// permutation.
    pub remap_xz: [f32; 8],
    /// The eight YZ-plane heading remaps in degrees, one per axis-sign
    /// permutation.
    pub remap_yz: [f32; 8],
}

/// A neutral IMU Test Readout: the typed values the panel formats into rows.
///
/// Every optional value is `None` until the test has a sample, which the model
/// draws as `--`. The firmware builds one of these each publish tick with the
/// `with_*` methods and hands it to the UI through the telemetry snapshot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuReadout {
    /// Which readout this is.
    pub kind: ImuReadoutKind,
    /// How the samples are arriving.
    pub health: ImuHealth,
    /// Consecutive samples taken without a reading.
    pub missing: u32,
    /// Fused orientation, or `None` before the first sample.
    pub orientation: Option<Euler>,
    /// Raw accelerometer in `g`, or `None`.
    pub raw_accel: Option<Vector3>,
    /// Raw gyroscope in `deg/s`, or `None`.
    pub raw_gyro: Option<Vector3>,
    /// DMP-calibrated gyroscope in `deg/s`, or `None`.
    pub calibrated_gyro: Option<Vector3>,
    /// Raw magnetometer in µT, or `None`.
    pub raw_mag: Option<Vector3>,
    /// DMP-calibrated magnetometer in µT, or `None`.
    pub calibrated_mag: Option<Vector3>,
    /// The stored DMP calibration status.
    pub dmp_calibration: CalibrationStatus,
    /// Whether the magnetometer was available at start-up.
    pub mag_available: bool,
    /// The magnetometer gate diagnostics, or `None` before the first sample and
    /// when the magnetometer is unavailable.
    pub mag: Option<MagDiagnostics>,
}

impl ImuReadout {
    /// An empty readout for `kind`: Waiting, no missing samples, every value
    /// absent, the magnetometer assumed available and its calibration unknown.
    #[must_use]
    pub const fn waiting(kind: ImuReadoutKind) -> Self {
        Self {
            kind,
            health: ImuHealth::Waiting,
            missing: 0,
            orientation: None,
            raw_accel: None,
            raw_gyro: None,
            calibrated_gyro: None,
            raw_mag: None,
            calibrated_mag: None,
            dmp_calibration: CalibrationStatus::Unknown,
            mag_available: true,
            mag: None,
        }
    }

    /// The same readout with a health state and missing count.
    #[must_use]
    pub const fn with_health(self, health: ImuHealth, missing: u32) -> Self {
        Self {
            health,
            missing,
            ..self
        }
    }

    /// The same readout with a fused orientation.
    #[must_use]
    pub const fn with_orientation(self, orientation: Euler) -> Self {
        Self {
            orientation: Some(orientation),
            ..self
        }
    }

    /// The same readout with a raw accelerometer vector.
    #[must_use]
    pub const fn with_raw_accel(self, accel: Vector3) -> Self {
        Self {
            raw_accel: Some(accel),
            ..self
        }
    }

    /// The same readout with a raw gyroscope vector.
    #[must_use]
    pub const fn with_raw_gyro(self, gyro: Vector3) -> Self {
        Self {
            raw_gyro: Some(gyro),
            ..self
        }
    }

    /// The same readout with a calibrated gyroscope vector.
    #[must_use]
    pub const fn with_calibrated_gyro(self, gyro: Vector3) -> Self {
        Self {
            calibrated_gyro: Some(gyro),
            ..self
        }
    }

    /// The same readout with a raw magnetometer vector, in µT.
    #[must_use]
    pub const fn with_raw_mag(self, mag: Vector3) -> Self {
        Self {
            raw_mag: Some(mag),
            ..self
        }
    }

    /// The same readout with a DMP-calibrated magnetometer vector, in µT.
    #[must_use]
    pub const fn with_calibrated_mag(self, mag: Vector3) -> Self {
        Self {
            calibrated_mag: Some(mag),
            ..self
        }
    }

    /// The same readout with a DMP calibration status.
    #[must_use]
    pub const fn with_dmp_calibration(self, status: CalibrationStatus) -> Self {
        Self {
            dmp_calibration: status,
            ..self
        }
    }

    /// The same readout with whether the magnetometer was available at start-up.
    #[must_use]
    pub const fn with_mag_available(self, available: bool) -> Self {
        Self {
            mag_available: available,
            ..self
        }
    }

    /// The same readout with the magnetometer gate diagnostics.
    #[must_use]
    pub const fn with_mag(self, mag: MagDiagnostics) -> Self {
        Self { mag: Some(mag), ..self }
    }

    /// The number of rows this readout fills.
    #[must_use]
    pub const fn row_count(&self) -> usize {
        match self.kind {
            ImuReadoutKind::SixAxis => SIX_AXIS_ROWS,
            ImuReadoutKind::NineAxis => NINE_AXIS_ROWS,
        }
    }

    /// The formatted rows, in display order.
    ///
    /// Only the first [`Self::row_count`] entries are meaningful; the rest are
    /// empty [`TextBuf::new`] slots.
    #[must_use]
    #[allow(clippy::similar_names)]
    pub fn rows(&self) -> [TextBuf; MAX_IMU_ROWS] {
        let mut rows = [TextBuf::new(); MAX_IMU_ROWS];
        match self.kind {
            ImuReadoutKind::SixAxis => {
                rows[0] = euler_line(self.orientation);
                rows[1] = accel_line(self.raw_accel);
                rows[2] = gyro_line("raw", self.raw_gyro);
                rows[3] = gyro_line("cal", self.calibrated_gyro);
                rows[4] = health_line(self.health, self.missing);
            }
            ImuReadoutKind::NineAxis => {
                rows[0] = euler_line(self.orientation);
                rows[1] = accel_line(self.raw_accel);
                rows[2] = gyro_line("raw", self.raw_gyro);
                rows[3] = gyro_line("cal", self.calibrated_gyro);
                rows[4] = mag_line("M raw", self.raw_mag);
                rows[5] = mag_line("M dmp", self.calibrated_mag);
                let [heading, xy_row, xz_row, yz_row] =
                    heading_and_remap_rows(self.mag, self.mag_available, self.orientation);
                rows[6] = heading;
                rows[7] = xy_row;
                rows[8] = xz_row;
                rows[9] = yz_row;
                rows[10] = nine_axis_health_line(self.health, self.missing, self.dmp_calibration);
                rows[11] = mag_gate_line(self.mag, self.mag_available);
            }
        }
        rows
    }
}

/// Format the heading row and the three remap rows, or their placeholders when
/// the magnetometer is unavailable or has not produced diagnostics.
fn heading_and_remap_rows(
    mag: Option<MagDiagnostics>,
    mag_available: bool,
    orientation: Option<Euler>,
) -> [TextBuf; 4] {
    if !mag_available {
        return heading_and_remap_placeholders();
    }
    mag.map_or_else(heading_and_remap_placeholders, |diagnostics| {
        [
            heading_line(diagnostics, orientation),
            remap_line("XY", diagnostics.remap_xy),
            remap_line("XZ", diagnostics.remap_xz),
            remap_line("YZ", diagnostics.remap_yz),
        ]
    })
}

/// The heading and remap rows when the magnetometer is unavailable.
fn heading_and_remap_placeholders() -> [TextBuf; 4] {
    let mut rows = [TextBuf::new(); 4];
    rows[0].fill(format_args!("Hdg --"));
    rows[1].fill(format_args!("XY --"));
    rows[2].fill(format_args!("XZ --"));
    rows[3].fill(format_args!("YZ --"));
    rows
}

/// Format the heading row: the raw and tilt-compensated headings at one decimal
/// place and the DMP yaw, with `--` for a heading the samples cannot yet provide.
fn heading_line(diagnostics: MagDiagnostics, orientation: Option<Euler>) -> TextBuf {
    let mut line = TextBuf::new();
    match (diagnostics.tilt_heading, orientation) {
        (Some(tilt), Some(euler)) => line.fill(format_args!(
            "Hdg raw={:.1} tilt={:.1} dmp={:.1}",
            diagnostics.raw_heading, tilt, euler.yaw
        )),
        (Some(tilt), None) => line.fill(format_args!(
            "Hdg raw={:.1} tilt={:.1} dmp=--",
            diagnostics.raw_heading, tilt
        )),
        (None, Some(euler)) => line.fill(format_args!(
            "Hdg raw={:.1} tilt=-- dmp={:.1}",
            diagnostics.raw_heading, euler.yaw
        )),
        (None, None) => line.fill(format_args!("Hdg raw={:.1} tilt=-- dmp=--", diagnostics.raw_heading)),
    }
    line
}

/// Format one remap row: the plane label and eight integer headings, each
/// right-aligned in a five-character field so negatives and zero line up.
///
/// The diagnostics carry full-precision headings; the cast happens here, only
/// for the fixed-width row.
#[allow(clippy::cast_possible_truncation)]
fn remap_line(label: &'static str, headings: [f32; 8]) -> TextBuf {
    let mut line = TextBuf::new();
    line.fill(format_args!(
        "{label}{:>5}{:>5}{:>5}{:>5}{:>5}{:>5}{:>5}{:>5}",
        headings[0] as i32,
        headings[1] as i32,
        headings[2] as i32,
        headings[3] as i32,
        headings[4] as i32,
        headings[5] as i32,
        headings[6] as i32,
        headings[7] as i32
    ));
    line
}

/// Format the Euler row, one line in degrees.
fn euler_line(orientation: Option<Euler>) -> TextBuf {
    let mut line = TextBuf::new();
    match orientation {
        Some(euler) => line.fill(format_args!(
            "Euler yaw={:.1} pitch={:.1} roll={:.1}",
            euler.yaw, euler.pitch, euler.roll
        )),
        None => line.fill(format_args!("Euler yaw=-- pitch=-- roll=--")),
    }
    line
}

/// Format the raw accelerometer row, two decimals in `g`.
fn accel_line(accel: Option<Vector3>) -> TextBuf {
    let mut line = TextBuf::new();
    match accel {
        Some(vector) => line.fill(format_args!(
            "Accel x={:.2} y={:.2} z={:.2} g",
            vector.x, vector.y, vector.z
        )),
        None => line.fill(format_args!("Accel x=-- y=-- z=-- g")),
    }
    line
}

/// Format one gyroscope row, one decimal in `deg/s`.
fn gyro_line(prefix: &'static str, gyro: Option<Vector3>) -> TextBuf {
    let mut line = TextBuf::new();
    match gyro {
        Some(vector) => line.fill(format_args!(
            "Gyro {prefix} x={:.1} y={:.1} z={:.1} dps",
            vector.x, vector.y, vector.z
        )),
        None => line.fill(format_args!("Gyro {prefix} x=-- y=-- z=-- dps")),
    }
    line
}

/// Format the health row: the state, the missing count and the six-axis
/// magnetometer note.
fn health_line(health: ImuHealth, missing: u32) -> TextBuf {
    let mut line = TextBuf::new();
    line.fill(format_args!(
        "{} miss={} dmpcal n/a",
        health.label(),
        missing.min(MAX_MISSING)
    ));
    line
}

/// Format the nine-axis health row: the state, the missing count and the DMP
/// calibration. It always names those facts, even when the magnetometer was
/// unavailable; the fallback notice lives on the gate row.
fn nine_axis_health_line(health: ImuHealth, missing: u32, dmp_calibration: CalibrationStatus) -> TextBuf {
    let mut line = TextBuf::new();
    line.fill(format_args!(
        "{} miss={} dmpcal {}",
        health.label(),
        missing.min(MAX_MISSING),
        dmp_calibration_word(dmp_calibration)
    ));
    line
}

/// The lowercase word the nine-axis health row uses for a calibration status.
const fn dmp_calibration_word(status: CalibrationStatus) -> &'static str {
    match status {
        CalibrationStatus::Loaded => "loaded",
        CalibrationStatus::Missing => "missing",
        CalibrationStatus::Unknown => "unknown",
    }
}

/// Format one magnetometer row, one decimal in µT. `label` names whether the
/// vector is the raw `M raw` reading or the DMP-calibrated `M dmp` one.
fn mag_line(label: &'static str, mag: Option<Vector3>) -> TextBuf {
    let mut line = TextBuf::new();
    match mag {
        Some(vector) => line.fill(format_args!(
            "{label} x={:.1} y={:.1} z={:.1} uT",
            vector.x, vector.y, vector.z
        )),
        None => line.fill(format_args!("{label} x=-- y=-- z=-- uT")),
    }
    line
}

/// Format the magnetometer gate row: the raw-field norm and the driver's
/// verdict, a placeholder with no verdict when nothing has been sampled, or the
/// six-axis fallback notice when the magnetometer was unavailable at start-up.
fn mag_gate_line(mag: Option<MagDiagnostics>, mag_available: bool) -> TextBuf {
    let mut line = TextBuf::new();
    if !mag_available {
        line.fill(format_args!("mag unavailable - 6-axis fallback use_mag=no"));
        return line;
    }
    match mag {
        Some(diagnostics) => line.fill(format_args!(
            "|m|={:.1} uT use_mag={}",
            diagnostics.norm,
            if diagnostics.use_mag { "yes" } else { "no" }
        )),
        None => line.fill(format_args!("|m|=-- uT use_mag=no")),
    }
    line
}
