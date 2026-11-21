//! The stored Motor Calibration record: a versioned, checksummed payload that
//! carry a calibration across a power cycle.
//!
//! One record holds the schema version, the monotonic run id, the measured
//! **Distance Calibration** scale, and — for each of the four **Test Legs** — the
//! **Breakaway** and the raw measurement of every **Power Step**: the commanded
//! percent, the encoder count, the **Measurement Window** length, the rail
//! voltage, the applied compensation and whether the window showed sustained
//! motion (which is what the Breakaway and therefore the **Calibration Floor**
//! are derived from). The derived per-point corrections are stored alongside them
//! as a cache.
//!
//! Version 4 records are **pair-measured**: the sweep commands both tracks at the
//! same Power Step and fills both legs from that one simultaneous window. The
//! layout is unchanged — four legs of ten steps — but the same command now
//! appears in the left and right leg of a step, which is what lets the curve
//! compare the two tracks in the load condition normal driving uses. A version 3
//! record was measured one track at a time and is rejected as "not calibrated";
//! nothing is migrated.
//!
//! The raw measurements are the source of truth. [`MotorCalibrationRecord::decode`]
//! recomputes the curve **and** the four Breakaways from them and **ignores both
//! caches entirely**, so a changed curve model can re-derive a stored calibration
//! without re-measuring, and a stale or tampered cache can never disagree with
//! the curve the robot applies or move a leg's Breakaway.
//!
//! # Encoded layout
//!
//! Every multi-byte field is little-endian and the record is a fixed
//! [`RECORD_ENCODED_LEN`] bytes, so the firmware can size its buffer once:
//!
//! | Offset | Size | Field |
//! | --- | --- | --- |
//! | 0 | 1 | schema version ([`RECORD_FORMAT_VERSION`]) |
//! | 1 | 1 | header flags; bit 0 says the record carries sweep measurements |
//! | 2 | 4 | run id (`u32`) |
//! | 6 | 4 | counts per centimetre (`f32` bits) |
//! | 10 | 4 | Breakaway per leg, in cycle order (`u8` each; a cache) |
//! | 14 | 640 | 4 Test Legs × 10 Power Steps, in cycle order |
//! | 654 | 200 | 4 × 10 corrections cache entries |
//! | 854 | 2 | CRC-16/CCITT-FALSE over bytes `0..854` (`u16`) |
//!
//! A **scale-only** record clears the header's has-curve flag. It exists because
//! **Distance Calibration** can produce a measured encoder scale on a fresh robot
//! before any motor sweep has measured a curve; without that form the scale could
//! not be saved without inventing measurements. A scale-only record writes the
//! `0` Breakaway sentinel, decodes with `curve: None` and
//! [`breakaway`](MotorCalibrationRecord::breakaway) `None`, and its step and cache
//! regions are ignored, so the record stays one fixed-size unit and one flash key
//! either way. A full record sets the flag and decodes with the scale, the derived
//! curve and one Breakaway per leg.
//!
//! The four Breakaway bytes are a cache of the raw sustained-motion flags. A
//! full record always has one per leg because the curve derivation requires
//! movement at 100 %; `decode` recomputes them from the flags rather than
//! trusting the stored copies.
//!
//! One Power Step is 16 bytes: command percent (`u8`), count (`u16`), window
//! length in milliseconds (`u32`), rail voltage (`f32` bits), compensation
//! (`f32` bits), and a flags byte whose bit 0 is sustained motion. One cache
//! entry is 5 bytes: a presence byte and the correction (`f32` bits); a `None`
//! correction writes a zero presence byte and four zero bytes.
//!
//! A wrong length (including truncation), an unknown schema version, an unknown
//! header or step flag, a checksum mismatch or a full record whose measurements do
//! not derive a curve rejects the record: the firmware treats every
//! [`DecodeError`] as "not calibrated".

use crate::{
    Curve, LegPoint, POWER_STEPS, Sweep, TEST_LEGS, TestLeg, curve_maths::LEG_ORDER, derive_curve, scale::DistanceScale,
};

/// The schema version this crate writes.
///
/// Version 4 records are pair-measured (both tracks commanded at each Power
/// Step). A record with any other version is rejected rather than guessed at, so
/// a firmware update can introduce a new layout without misreading an old
/// record; in particular a version 3 single-track record is "not calibrated"
/// and is never migrated.
pub const RECORD_FORMAT_VERSION: u8 = 4;

/// The fixed encoded length of a Motor Calibration record, in bytes.
///
/// The firmware sizes its `sequential-storage` buffer from this and
/// [`MotorCalibrationRecord::encode_into`] reports [`EncodeError::BufferTooSmall`]
/// for anything shorter.
pub const RECORD_ENCODED_LEN: usize =
    CORRECTIONS_OFFSET + TEST_LEGS * POWER_STEPS * CORRECTION_ENCODED_LEN + CHECKSUM_ENCODED_LEN;

/// The encoded length of the integrity checksum, in bytes.
const CHECKSUM_ENCODED_LEN: usize = 2;

/// The encoded length of the fixed header, in bytes: version, flags, run id,
/// scale and the four Breakaway cache bytes.
const HEADER_ENCODED_LEN: usize = BREAKAWAYS_OFFSET + TEST_LEGS;

/// The offset of the schema version.
const VERSION_OFFSET: usize = 0;

/// The offset of the header flags.
const FLAGS_OFFSET: usize = 1;

/// The offset of the run id.
const RUN_ID_OFFSET: usize = 2;

/// The offset of the measured counts per centimetre.
const SCALE_OFFSET: usize = 6;

/// The offset of the four Breakaway cache bytes, one per leg in cycle order.
const BREAKAWAYS_OFFSET: usize = 10;

/// The header flag that says the record carries raw sweep measurements and a
/// derivable curve.
const RECORD_HAS_CURVE_FLAG: u8 = 0b0000_0001;

/// The header flag bits this version understands; every other bit is a rejection.
const RECORD_KNOWN_FLAGS: u8 = RECORD_HAS_CURVE_FLAG;

/// The offset of the first Power Step.
const STEPS_OFFSET: usize = HEADER_ENCODED_LEN;

/// The encoded length of one Power Step, in bytes.
const STEP_ENCODED_LEN: usize = 16;

/// The offset of a step's command percent.
const STEP_COMMAND_OFFSET: usize = 0;

/// The offset of a step's encoder count.
const STEP_COUNT_OFFSET: usize = 1;

/// The offset of a step's Measurement Window length.
const STEP_WINDOW_OFFSET: usize = 3;

/// The offset of a step's rail voltage.
const STEP_RAIL_OFFSET: usize = 7;

/// The offset of a step's applied compensation.
const STEP_COMPENSATION_OFFSET: usize = 11;

/// The offset of a step's flags byte.
const STEP_FLAGS_OFFSET: usize = 15;

/// The flags bit that records sustained motion in the Measurement Window.
const STEP_SUSTAINED_MOTION_FLAG: u8 = 0b0000_0001;

/// The flags bits this version understands; every other bit is a rejection.
const STEP_KNOWN_FLAGS: u8 = STEP_SUSTAINED_MOTION_FLAG;

/// The encoded length of one corrections cache entry, in bytes.
const CORRECTION_ENCODED_LEN: usize = 5;

/// The presence byte of a correction that exists.
const CORRECTION_PRESENT: u8 = 1;

/// The offset of the derived-corrections cache.
const CORRECTIONS_OFFSET: usize = HEADER_ENCODED_LEN + TEST_LEGS * POWER_STEPS * STEP_ENCODED_LEN;

/// One Power Step of one Test Leg exactly as it was measured.
///
/// It carries the rail voltage and the compensation that were in force so a
/// doubtful run can be recognised later, alongside everything the curve maths
/// needs to recompute the correction for this point.
///
/// The rail and compensation are recorded whenever a rail sample was available
/// at the instant the command was made. When none was available the sweep holds
/// `None` and the firmware writes `0.0 V` with `1.0` (identity) compensation;
/// those two values together are the documented "not measured" sentinel. A
/// point with a sample always carries the real pair, so the record still says
/// what the driver actually used.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepMeasurement {
    /// The commanded Power Step, in whole percent.
    command_percent: u8,
    /// The wrap-safe encoder count across the Measurement Window.
    count: u16,
    /// The Measurement Window length, in milliseconds.
    window_ms: u32,
    /// The motor rail voltage during the window, in volts.
    ///
    /// `0.0` is the sentinel for "no rail sample was available" — see the type
    /// documentation.
    rail_volts: f32,
    /// The voltage compensation applied during the window.
    ///
    /// `1.0` (identity) paired with a `0.0` rail is the sentinel for "not
    /// measured" — see the type documentation.
    compensation: f32,
    /// Whether the window showed sustained motion above the motion threshold.
    sustained_motion: bool,
}

impl StepMeasurement {
    /// Build a measurement from its raw fields.
    ///
    /// `rail_volts` and `compensation` must be the values in force at command
    /// time. A step measured without a rail sample writes `0.0` volts and `1.0`
    /// (identity) compensation, which is the documented "not measured"
    /// sentinel; a step with a sample always records the real pair.
    #[must_use]
    pub const fn new(
        command_percent: u8,
        count: u16,
        window_ms: u32,
        rail_volts: f32,
        compensation: f32,
        sustained_motion: bool,
    ) -> Self {
        Self {
            command_percent,
            count,
            window_ms,
            rail_volts,
            compensation,
            sustained_motion,
        }
    }

    /// A placeholder used to initialise a record's measurement array.
    const EMPTY: Self = Self::new(0, 0, 0, 0.0, 0.0, false);

    /// The commanded Power Step, in whole percent.
    #[must_use]
    pub const fn command_percent(self) -> u8 {
        self.command_percent
    }

    /// The wrap-safe encoder count across the Measurement Window.
    #[must_use]
    pub const fn count(self) -> u16 {
        self.count
    }

    /// The Measurement Window length, in milliseconds.
    #[must_use]
    pub const fn window_ms(self) -> u32 {
        self.window_ms
    }

    /// The motor rail voltage during the window, in volts.
    #[must_use]
    pub const fn rail_volts(self) -> f32 {
        self.rail_volts
    }

    /// The voltage compensation applied during the window.
    #[must_use]
    pub const fn compensation(self) -> f32 {
        self.compensation
    }

    /// Whether the window showed sustained motion above the motion threshold.
    #[must_use]
    pub const fn sustained_motion(self) -> bool {
        self.sustained_motion
    }

    /// The curve-maths view of this measurement.
    fn to_leg_point(self) -> LegPoint {
        LegPoint::new(
            self.command_percent,
            self.count,
            u64::from(self.window_ms),
            self.sustained_motion,
        )
    }

    /// Write this step into its fixed slice of the encoded record.
    fn encode_into(self, bytes: &mut [u8]) {
        bytes[STEP_COMMAND_OFFSET] = self.command_percent;
        bytes[STEP_COUNT_OFFSET..STEP_COUNT_OFFSET + 2].copy_from_slice(&self.count.to_le_bytes());
        bytes[STEP_WINDOW_OFFSET..STEP_WINDOW_OFFSET + 4].copy_from_slice(&self.window_ms.to_le_bytes());
        bytes[STEP_RAIL_OFFSET..STEP_RAIL_OFFSET + 4].copy_from_slice(&self.rail_volts.to_bits().to_le_bytes());
        bytes[STEP_COMPENSATION_OFFSET..STEP_COMPENSATION_OFFSET + 4]
            .copy_from_slice(&self.compensation.to_bits().to_le_bytes());
        bytes[STEP_FLAGS_OFFSET] = if self.sustained_motion {
            STEP_SUSTAINED_MOTION_FLAG
        } else {
            0
        };
    }

    /// Read this step from its fixed slice of the encoded record.
    const fn decode_from(bytes: &[u8]) -> Self {
        Self {
            command_percent: bytes[STEP_COMMAND_OFFSET],
            count: u16_at(bytes, STEP_COUNT_OFFSET),
            window_ms: u32_at(bytes, STEP_WINDOW_OFFSET),
            rail_volts: f32_at(bytes, STEP_RAIL_OFFSET),
            compensation: f32_at(bytes, STEP_COMPENSATION_OFFSET),
            sustained_motion: bytes[STEP_FLAGS_OFFSET] & STEP_SUSTAINED_MOTION_FLAG != 0,
        }
    }
}

/// A saved Motor Calibration, as the firmware stores and loads it.
///
/// Build a full record with [`new`](Self::new), which derives the curve from the
/// raw measurements and rejects a sweep that does not derive one, or a scale-only
/// record with [`scale_only`](Self::scale_only) when Distance Calibration has a
/// measured scale but no sweep. The per-point corrections and the four Breakaways
/// are caches; [`decode`](Self::decode) recomputes them from the raw measurements
/// and never trusts the stored copies.
#[derive(Clone, Debug, PartialEq)]
pub struct MotorCalibrationRecord {
    /// The monotonic id of the run that produced this record.
    run_id: u32,
    /// The measured encoder scale.
    scale: DistanceScale,
    /// The raw measurements, indexed in [`LEG_ORDER`] order.
    legs: [[StepMeasurement; POWER_STEPS]; TEST_LEGS],
    /// Each leg's Breakaway, indexed in [`LEG_ORDER`] order. `[0; TEST_LEGS]` for
    /// a scale-only record.
    breakaways: [u8; TEST_LEGS],
    /// The curve the raw measurements imply, or `None` for a scale-only record.
    curve: Option<Curve>,
}

impl MotorCalibrationRecord {
    /// Build a record from the measured scale and the four Test Legs' raw
    /// measurements, deriving the curve from those measurements.
    ///
    /// `legs` is in cycle order — left forward, left reverse, right forward,
    /// right reverse — and each leg holds its ten Power Steps. A pair-measured
    /// sweep fills the left and right leg of each direction at the same command,
    /// so the same Power Step appears in both. `run_id` must be monotonically
    /// increasing across saves; the record stores it unchanged.
    ///
    /// # Errors
    ///
    /// Returns the [`CurveError`](crate::CurveError) that
    /// [`derive_curve`](crate::derive_curve) reports when the raw measurements do
    /// not form a valid curve. A calibration that cannot derive a curve is never
    /// saved.
    pub fn new(
        run_id: u32,
        scale: DistanceScale,
        legs: [[StepMeasurement; POWER_STEPS]; TEST_LEGS],
    ) -> Result<Self, crate::CurveError> {
        let breakaways = breakaways_from(&legs);
        let curve = derive_from(&legs)?;
        Ok(Self {
            run_id,
            scale,
            legs,
            breakaways,
            curve: Some(curve),
        })
    }

    /// A record that carries only a measured encoder scale.
    ///
    /// Distance Calibration produces this on a robot whose motor sweep has not
    /// run yet: the scale is real and worth keeping, but there are no measurements
    /// to derive a curve from. The record decodes with no curve, so the drive path
    /// falls back to the identity curve and the measured scale.
    #[must_use]
    pub const fn scale_only(run_id: u32, scale: DistanceScale) -> Self {
        Self {
            run_id,
            scale,
            legs: [[StepMeasurement::EMPTY; POWER_STEPS]; TEST_LEGS],
            breakaways: [0; TEST_LEGS],
            curve: None,
        }
    }

    /// The same record with a different encoder scale.
    ///
    /// Storage uses this to update one record in place: a later Distance
    /// Calibration must not discard the curve a motor sweep already measured. The
    /// run id names the sweep that produced the measurements, so it is unchanged;
    /// only a new sweep starts a new id.
    #[must_use]
    pub const fn with_scale(self, scale: DistanceScale) -> Self {
        Self { scale, ..self }
    }

    /// The monotonic id of the run that produced this record.
    #[must_use]
    pub const fn run_id(&self) -> u32 {
        self.run_id
    }

    /// The measured encoder scale.
    #[must_use]
    pub const fn scale(&self) -> DistanceScale {
        self.scale
    }

    /// The raw measurements of `leg`, in Power Step order.
    #[must_use]
    pub const fn leg(&self, leg: TestLeg) -> &[StepMeasurement; POWER_STEPS] {
        &self.legs[leg.index()]
    }

    /// The **Breakaway** of `leg`: the lowest Power Step whose Measurement
    /// Window showed sustained motion, or `None` for a scale-only record.
    ///
    /// A full record always has one per leg, because the curve derivation
    /// requires movement at 100 %. The value is recomputed from the raw
    /// sustained-motion flags by [`decode`](Self::decode), so the bytes stored in
    /// the header are a cache and a tampered copy cannot move it.
    #[must_use]
    pub fn breakaway(&self, leg: TestLeg) -> Option<u8> {
        self.curve.as_ref()?;
        Some(self.breakaways[leg.index()])
    }

    /// The curve the raw measurements imply, or `None` for a scale-only record.
    #[must_use]
    pub const fn curve(&self) -> Option<&Curve> {
        self.curve.as_ref()
    }

    /// Encode the record into `buffer`, returning the bytes written.
    ///
    /// # Errors
    ///
    /// Returns [`EncodeError::BufferTooSmall`] when `buffer` is shorter than
    /// [`RECORD_ENCODED_LEN`]. On success exactly [`RECORD_ENCODED_LEN`] bytes of
    /// `buffer` are written.
    pub fn encode_into(&self, buffer: &mut [u8]) -> Result<usize, EncodeError> {
        if buffer.len() < RECORD_ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall {
                needed: RECORD_ENCODED_LEN,
            });
        }

        buffer[VERSION_OFFSET] = RECORD_FORMAT_VERSION;
        buffer[FLAGS_OFFSET] = if self.curve.is_some() { RECORD_HAS_CURVE_FLAG } else { 0 };
        buffer[RUN_ID_OFFSET..RUN_ID_OFFSET + 4].copy_from_slice(&self.run_id.to_le_bytes());
        buffer[SCALE_OFFSET..SCALE_OFFSET + 4].copy_from_slice(&self.scale.counts_per_cm().to_bits().to_le_bytes());
        buffer[BREAKAWAYS_OFFSET..BREAKAWAYS_OFFSET + TEST_LEGS].copy_from_slice(&self.breakaways);

        let mut offset = STEPS_OFFSET;
        for leg in &self.legs {
            for step in leg {
                step.encode_into(&mut buffer[offset..offset + STEP_ENCODED_LEN]);
                offset += STEP_ENCODED_LEN;
            }
        }

        for (leg_index, leg) in self.legs.iter().enumerate() {
            let test_leg = LEG_ORDER[leg_index];
            for step in leg {
                let correction = self
                    .curve
                    .as_ref()
                    .and_then(|curve| curve.correction(test_leg.track, test_leg.direction, step.command_percent()));
                write_correction(&mut buffer[offset..offset + CORRECTION_ENCODED_LEN], correction);
                offset += CORRECTION_ENCODED_LEN;
            }
        }

        let checksum_offset = RECORD_ENCODED_LEN - CHECKSUM_ENCODED_LEN;
        let checksum = checksum(&buffer[..checksum_offset]);
        buffer[checksum_offset..RECORD_ENCODED_LEN].copy_from_slice(&checksum.to_le_bytes());
        Ok(RECORD_ENCODED_LEN)
    }

    /// Decode a record from exactly [`RECORD_ENCODED_LEN`] bytes, recomputing the
    /// curve and the four Breakaways from the raw measurements when the record
    /// carries them.
    ///
    /// Both caches are ignored: the raw measurements are the source of truth, so
    /// a stored correction or Breakaway can never override them. A scale-only
    /// record decodes with [`curve`](Self::curve) `None`, no Breakaway, and no
    /// claim on its unused step and cache regions, whether they are zero or
    /// garbage.
    ///
    /// # Errors
    ///
    /// Returns [`DecodeError::Length`] for a wrong length (including a truncated
    /// record), [`DecodeError::UnsupportedVersion`] for an unknown schema version,
    /// [`DecodeError::ChecksumMismatch`] for a corrupt payload,
    /// [`DecodeError::InvalidHeaderFlags`] for an unknown header flag,
    /// [`DecodeError::InvalidFlags`] for an unknown step flag, and
    /// [`DecodeError::Curve`] when the raw measurements do not derive a curve.
    /// Every failure means "not calibrated".
    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() != RECORD_ENCODED_LEN {
            return Err(DecodeError::Length { actual: bytes.len() });
        }
        if bytes[VERSION_OFFSET] != RECORD_FORMAT_VERSION {
            return Err(DecodeError::UnsupportedVersion {
                found: bytes[VERSION_OFFSET],
            });
        }
        let flags = bytes[FLAGS_OFFSET];
        if flags & !RECORD_KNOWN_FLAGS != 0 {
            return Err(DecodeError::InvalidHeaderFlags { found: flags });
        }

        let checksum_offset = RECORD_ENCODED_LEN - CHECKSUM_ENCODED_LEN;
        let expected = checksum(&bytes[..checksum_offset]);
        let found = u16_at(bytes, checksum_offset);
        if expected != found {
            return Err(DecodeError::ChecksumMismatch { expected, found });
        }

        let run_id = u32_at(bytes, RUN_ID_OFFSET);
        let scale = DistanceScale::from_stored(f32_at(bytes, SCALE_OFFSET));

        if flags & RECORD_HAS_CURVE_FLAG == 0 {
            return Ok(Self::scale_only(run_id, scale));
        }

        // The stored Breakaway bytes at `BREAKAWAYS_OFFSET` are deliberately
        // skipped: they are a cache of the raw flags below.
        let mut legs = [[StepMeasurement::EMPTY; POWER_STEPS]; TEST_LEGS];
        let mut offset = STEPS_OFFSET;
        for (leg_index, measurements) in legs.iter_mut().enumerate() {
            for measurement in measurements.iter_mut() {
                *measurement = StepMeasurement::decode_from(&bytes[offset..offset + STEP_ENCODED_LEN]);
                let flags = bytes[offset + STEP_FLAGS_OFFSET];
                if flags & !STEP_KNOWN_FLAGS != 0 {
                    return Err(DecodeError::InvalidFlags {
                        leg: LEG_ORDER[leg_index],
                        command_percent: measurement.command_percent(),
                        found: flags,
                    });
                }
                offset += STEP_ENCODED_LEN;
            }
        }

        // The corrections cache at `offset` is deliberately skipped.
        let curve = derive_from(&legs).map_err(DecodeError::Curve)?;
        let breakaways = breakaways_from(&legs);

        Ok(Self {
            run_id,
            scale,
            legs,
            breakaways,
            curve: Some(curve),
        })
    }
}

/// The record's integrity checksum: CRC-16/CCITT-FALSE over `bytes`.
///
/// Polynomial `0x1021`, initial value `0xFFFF`, no reflection and no final XOR.
/// The check value for `"123456789"` is `0x29B1`. The bitwise form needs no
/// lookup table, and the result is stored little-endian.
#[must_use]
pub fn checksum(bytes: &[u8]) -> u16 {
    let mut crc = 0xFFFF_u16;
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Derive the curve from a record's raw measurements.
fn derive_from(legs: &[[StepMeasurement; POWER_STEPS]; TEST_LEGS]) -> Result<Curve, crate::CurveError> {
    let mut points = [[LegPoint::new(0, 0, 0, false); POWER_STEPS]; TEST_LEGS];
    for (leg_points, measurements) in points.iter_mut().zip(legs) {
        for (point, measurement) in leg_points.iter_mut().zip(measurements) {
            *point = measurement.to_leg_point();
        }
    }
    derive_curve(Sweep::new(&points[0], &points[1], &points[2], &points[3]))
}

/// Derive each leg's Breakaway from the raw sustained-motion flags.
///
/// The Breakaway is the lowest Power Step of the leg whose Measurement Window
/// showed sustained motion. `0` is the sentinel for a leg with no sustained
/// motion at all, which a full record cannot contain: the curve derivation
/// requires movement at 100 %.
fn breakaways_from(legs: &[[StepMeasurement; POWER_STEPS]; TEST_LEGS]) -> [u8; TEST_LEGS] {
    let mut breakaways = [0_u8; TEST_LEGS];
    for (breakaway, measurements) in breakaways.iter_mut().zip(legs) {
        *breakaway = measurements
            .iter()
            .filter(|measurement| measurement.sustained_motion())
            .map(|measurement| measurement.command_percent())
            .min()
            .unwrap_or(0);
    }
    breakaways
}

/// Write one corrections cache entry.
fn write_correction(bytes: &mut [u8], correction: Option<f32>) {
    if let Some(correction) = correction {
        bytes[0] = CORRECTION_PRESENT;
        bytes[1..CORRECTION_ENCODED_LEN].copy_from_slice(&correction.to_bits().to_le_bytes());
    } else {
        bytes[0] = 0;
        bytes[1..CORRECTION_ENCODED_LEN].copy_from_slice(&0_u32.to_le_bytes());
    }
}

/// Read a little-endian `u16` at `offset`.
const fn u16_at(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

/// Read a little-endian `u32` at `offset`.
const fn u32_at(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([bytes[offset], bytes[offset + 1], bytes[offset + 2], bytes[offset + 3]])
}

/// Read a little-endian `f32` bit pattern at `offset`.
const fn f32_at(bytes: &[u8], offset: usize) -> f32 {
    f32::from_bits(u32_at(bytes, offset))
}

/// Why a record could not be encoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    /// The destination buffer is shorter than [`RECORD_ENCODED_LEN`].
    BufferTooSmall {
        /// The encoded length required, in bytes.
        needed: usize,
    },
}

impl EncodeError {
    /// A short, stable phrase naming the reason for an operator log.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::BufferTooSmall { .. } => "record buffer too small",
        }
    }
}

/// Why an encoded record was rejected. Every variant means "not calibrated".
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DecodeError {
    /// The input was not exactly [`RECORD_ENCODED_LEN`] bytes.
    Length {
        /// The actual length, in bytes.
        actual: usize,
    },
    /// The schema version is not [`RECORD_FORMAT_VERSION`].
    UnsupportedVersion {
        /// The version found in the input.
        found: u8,
    },
    /// The stored checksum does not match the payload.
    ChecksumMismatch {
        /// The checksum computed from the payload.
        expected: u16,
        /// The checksum stored in the input.
        found: u16,
    },
    /// The header flags carry a bit this version does not understand.
    InvalidHeaderFlags {
        /// The flags byte found in the input.
        found: u8,
    },
    /// A step's flags byte carries a bit this version does not understand.
    InvalidFlags {
        /// The Test Leg carrying the step.
        leg: TestLeg,
        /// The step's commanded Power Step, in whole percent.
        command_percent: u8,
        /// The flags byte found in the input.
        found: u8,
    },
    /// The raw measurements do not derive a curve.
    Curve(crate::CurveError),
}

impl DecodeError {
    /// A short, stable phrase naming the reason for an operator log.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Length { .. } => "wrong record length",
            Self::UnsupportedVersion { .. } => "unsupported record version",
            Self::ChecksumMismatch { .. } => "record checksum mismatch",
            Self::InvalidHeaderFlags { .. } => "invalid record header flags",
            Self::InvalidFlags { .. } => "invalid record flags",
            Self::Curve(_) => "record curve is invalid",
        }
    }
}
