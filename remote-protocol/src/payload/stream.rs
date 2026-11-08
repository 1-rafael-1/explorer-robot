//! Typed payloads for the six implemented telemetry streams.
//!
//! | id | stream | layout | max bytes |
//! |---|---|---|---|
//! | `0x0001` | `POWER` | `level u8`, `voltage_mv u16` | 3 |
//! | `0x0002` | `MOTION` | `left i8`, `right i8` | 2 |
//! | `0x0003` | `ACTIVITY` | `procedure_id u16`, `stage u8`, `percent u8`, `detail_len u8`, detail | 37 |
//! | `0x0004` | `PERCEPTION` | `flags u8` | 1 |
//! | `0x0005` | `LIDAR_CLOUD` | `slots [u16; 360]` | 720 |
//! | `0x0006` | `CALIBRATION` | `motor u8`, `imu u8`, `mag u8`, `distance u8` | 4 |
//!
//! The sum of the maximum payloads is 767 bytes, which is the staging budget the robot's
//! latest-wins slots are sized against.

use crate::{
    codec::{PayloadCodec, read_u16_le, truncate_utf8, write_u16_le},
    error::{CodecError, EncodeError},
    ids::ProcedureId,
};

/// The `POWER.level` sentinel meaning "unknown".
pub const LEVEL_UNKNOWN: u8 = 0xFF;

/// The `POWER.voltage_mv` sentinel meaning "unknown".
pub const VOLTAGE_UNKNOWN_MV: u16 = 0xFFFF;

/// The number of angular slots in a `LIDAR_CLOUD` payload.
pub const LIDAR_SLOTS: usize = 360;

/// The `LIDAR_CLOUD` slot sentinel meaning "no return".
pub const LIDAR_NO_RETURN: u16 = 0xFFFF;

/// A `POWER` sample: battery level and pack voltage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Power {
    /// Battery level percentage, or [`LEVEL_UNKNOWN`].
    pub level: u8,
    /// Pack voltage in millivolts, or [`VOLTAGE_UNKNOWN_MV`].
    pub voltage_mv: u16,
}

impl Power {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 3;
}

impl<'a> PayloadCodec<'a> for Power {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self {
            level: bytes[0],
            voltage_mv: read_u16_le(bytes, 1),
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.level;
        write_u16_le(out, 1, self.voltage_mv);
        Ok(Self::ENCODED_LEN)
    }
}

/// A `MOTION` sample: the left and right motor output, signed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Motion {
    /// The left motor output.
    pub left: i8,
    /// The right motor output.
    pub right: i8,
}

impl Motion {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 2;
}

impl<'a> PayloadCodec<'a> for Motion {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self {
            left: i8::from_le_bytes([bytes[0]]),
            right: i8::from_le_bytes([bytes[1]]),
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.left.to_le_bytes()[0];
        out[1] = self.right.to_le_bytes()[0];
        Ok(Self::ENCODED_LEN)
    }
}

/// The `ACTIVITY.stage` byte.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ActivityStage {
    /// The Procedure is running.
    Running = 0,
    /// The Procedure completed successfully.
    Complete = 1,
    /// The Procedure failed.
    Failed = 2,
}

impl ActivityStage {
    /// Every stage, in wire order.
    pub const ALL: [Self; 3] = [Self::Running, Self::Complete, Self::Failed];

    /// Builds a stage from its wire value, returning [`None`] for an unassigned value.
    #[must_use]
    pub const fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Running),
            1 => Some(Self::Complete),
            2 => Some(Self::Failed),
            _ => None,
        }
    }

    /// Returns the wire value.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

/// An `ACTIVITY` sample: the current Procedure and its progress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Activity<'a> {
    /// The Procedure id, including one this version does not know.
    pub procedure_id: ProcedureId,
    /// The Procedure's stage.
    pub stage: ActivityStage,
    /// Completion percentage, or [`None`] for "not reported".
    pub percent: Option<u8>,
    /// The operator-facing detail; borrowed from the frame payload on decode.
    pub detail: &'a str,
}

impl Activity<'_> {
    /// The largest encoded payload length, including the longest allowed detail.
    pub const ENCODED_LEN: usize = 5 + Self::DETAIL_MAX;
    /// The largest detail length the encoder writes, in bytes.
    pub const DETAIL_MAX: usize = 32;
    /// The `percent` sentinel meaning "not reported".
    pub const PERCENT_NONE: u8 = 0xFF;
}

impl<'a> PayloadCodec<'a> for Activity<'a> {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() < 5 {
            return Err(CodecError::Length);
        }
        let detail_len = usize::from(bytes[4]);
        if bytes.len() != 5 + detail_len {
            return Err(CodecError::Length);
        }
        let stage = ActivityStage::from_wire(bytes[2]).ok_or(CodecError::Value)?;
        let percent = match bytes[3] {
            Self::PERCENT_NONE => None,
            value => Some(value),
        };
        let detail = core::str::from_utf8(&bytes[5..]).map_err(|_| CodecError::Utf8)?;
        Ok(Self {
            procedure_id: ProcedureId::new(read_u16_le(bytes, 0)),
            stage,
            percent,
            detail,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let percent = match self.percent {
            None => Self::PERCENT_NONE,
            Some(Self::PERCENT_NONE) => return Err(EncodeError::Value),
            Some(value) => value,
        };
        let detail = truncate_utf8(self.detail, Self::DETAIL_MAX);
        let detail_len = u8::try_from(detail.len()).map_err(|_| EncodeError::Value)?;
        let needed = 5 + detail.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }

        write_u16_le(out, 0, self.procedure_id.get());
        out[2] = self.stage.bits();
        out[3] = percent;
        out[4] = detail_len;
        out[5..needed].copy_from_slice(detail.as_bytes());
        Ok(needed)
    }
}

/// A `PERCEPTION` sample: the obstacle and floor-drop flags.
///
/// Bits 2–7 are reserved: they are cleared on decode and always sent as zero, so a peer can add a
/// meaning later without this version echoing garbage.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Perception(u8);

/// The shared name for the perception flags, also used by `GET_STATUS` (ADR-0031).
pub type PerceptionFlags = Perception;

impl Perception {
    /// The obstacle flag.
    pub const OBSTACLE: Self = Self(0x01);
    /// The floor-drop flag.
    pub const FLOOR_DROP: Self = Self(0x02);

    /// The reserved bits, cleared by [`Perception::from_bits`].
    const RESERVED_BITS: u8 = 0xFC;

    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 1;

    /// Builds the flags from a raw byte, clearing the reserved bits.
    #[must_use]
    pub const fn from_bits(bits: u8) -> Self {
        Self(bits & !Self::RESERVED_BITS)
    }

    /// Returns the wire byte.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Returns `true` when every bit in `other` is set.
    #[must_use]
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Returns `true` when no flag is set.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

impl core::ops::BitOr for Perception {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitOrAssign for Perception {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl<'a> PayloadCodec<'a> for Perception {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self::from_bits(bytes[0]))
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.is_empty() {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.bits();
        Ok(Self::ENCODED_LEN)
    }
}

/// A `LIDAR_CLOUD` sample: the 360-slot range grid in whole centimetres.
///
/// Slot 0 is dead ahead and the slots run clockwise. A slot equal to [`LIDAR_NO_RETURN`] has no
/// return.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LidarCloud {
    /// The range grid; `u16::MAX` means no return.
    pub slots: [u16; LIDAR_SLOTS],
}

impl LidarCloud {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = LIDAR_SLOTS * 2;
    /// The slot sentinel meaning "no return".
    pub const NO_RETURN: u16 = LIDAR_NO_RETURN;
}

impl<'a> PayloadCodec<'a> for LidarCloud {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        let mut slots = [0u16; LIDAR_SLOTS];
        for (slot, chunk) in slots.iter_mut().zip(bytes.chunks_exact(2)) {
            *slot = u16::from_le_bytes([chunk[0], chunk[1]]);
        }
        Ok(Self { slots })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        for (chunk, slot) in out.chunks_exact_mut(2).zip(self.slots.iter()) {
            chunk.copy_from_slice(&slot.to_le_bytes());
        }
        Ok(Self::ENCODED_LEN)
    }
}

/// A calibration status byte.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CalibrationStatus {
    /// The status is not known yet.
    Unknown = 0,
    /// The calibration data loaded successfully.
    Loaded = 1,
    /// The calibration data is missing.
    Missing = 2,
}

impl CalibrationStatus {
    /// Every status, in wire order.
    pub const ALL: [Self; 3] = [Self::Unknown, Self::Loaded, Self::Missing];

    /// Builds a status from its wire value, returning [`None`] for an unassigned value.
    #[must_use]
    pub const fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Unknown),
            1 => Some(Self::Loaded),
            2 => Some(Self::Missing),
            _ => None,
        }
    }

    /// Returns the wire value.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

/// A `CALIBRATION` sample: the four calibration statuses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Calibration {
    /// Motor calibration status.
    pub motor: CalibrationStatus,
    /// IMU calibration status.
    pub imu: CalibrationStatus,
    /// Magnetometer calibration status.
    pub mag: CalibrationStatus,
    /// Distance calibration status.
    pub distance: CalibrationStatus,
}

impl Calibration {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 4;
}

impl<'a> PayloadCodec<'a> for Calibration {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self {
            motor: CalibrationStatus::from_wire(bytes[0]).ok_or(CodecError::Value)?,
            imu: CalibrationStatus::from_wire(bytes[1]).ok_or(CodecError::Value)?,
            mag: CalibrationStatus::from_wire(bytes[2]).ok_or(CodecError::Value)?,
            distance: CalibrationStatus::from_wire(bytes[3]).ok_or(CodecError::Value)?,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.motor.bits();
        out[1] = self.imu.bits();
        out[2] = self.mag.bits();
        out[3] = self.distance.bits();
        Ok(Self::ENCODED_LEN)
    }
}
