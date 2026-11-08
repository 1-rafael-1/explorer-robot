//! Typed payloads for the seven v1 events (ADR-0031).
//!
//! | id | event | layout | max bytes |
//! |---|---|---|---|
//! | `0x0001` | `OBSTACLE` | `detected u8`, `source u8` | 2 |
//! | `0x0002` | `FLOOR_DROP` | `detected u8` | 1 |
//! | `0x0003` | `PROCEDURE_COMPLETED` | `procedure_id u16`, `stage u8`, `detail_len u8`, detail | 36 |
//! | `0x0004` | `DRIVE_DISTANCE_COMPLETED` | `command_seq u16`, `status u8`, `target_cm i16`, `distance_cm i32`, `duration_ms u32`, `detail_len u8`, detail | 46 |
//! | `0x0005` | `ROTATE_COMPLETED` | `command_seq u16`, `status u8`, `target_cdeg i16`, `error_cdeg i32`, `duration_ms u32`, `detail_len u8`, detail | 46 |
//! | `0x0006` | `CALIBRATION_LOADED` | `kind u8`, `status u8` | 2 |
//! | `0x0007` | `EVENT_OVERFLOW` | `dropped u16` | 2 |
//!
//! Every field is little-endian. The drive completions preserve the originating `command_seq`, so
//! the Remote can report "asked 100 cm, got 94 cm". The operator-facing detail is borrowed on
//! decode and truncated at a UTF-8 character boundary to the sender's 32-byte cap on encode; a
//! detail longer than the cap is still accepted on decode when it fits the payload, mirroring the
//! `ACTIVITY` stream and the status snapshot.
//!
//! An event ACK is the original frame with `IS_ACK` set; its payload is the shared status byte,
//! optionally followed by result bytes this version ignores.

use crate::{
    codec::{PayloadCodec, read_u16_le, read_u32_le, truncate_utf8, write_u16_le, write_u32_le},
    error::{CodecError, EncodeError},
    frame::Status,
    ids::ProcedureId,
    payload::stream::ActivityStage,
};

/// The largest encoded v1 event payload, including the longest allowed detail.
pub const EVENT_PAYLOAD_MAX: usize = DriveDistanceCompleted::ENCODED_LEN;

/// The `OBSTACLE.source` value naming the `LiDAR`.
pub const OBSTACLE_SOURCE_LIDAR: u8 = 0;

/// The largest detail text an event encoder writes, in bytes.
pub const EVENT_DETAIL_MAX: usize = 32;

/// A compile-time guard that every v1 event fits [`EVENT_PAYLOAD_MAX`].
const _: () = assert!(
    Obstacle::ENCODED_LEN <= EVENT_PAYLOAD_MAX
        && FloorDrop::ENCODED_LEN <= EVENT_PAYLOAD_MAX
        && ProcedureCompleted::ENCODED_LEN <= EVENT_PAYLOAD_MAX
        && DriveDistanceCompleted::ENCODED_LEN <= EVENT_PAYLOAD_MAX
        && RotateCompleted::ENCODED_LEN <= EVENT_PAYLOAD_MAX
        && CalibrationLoaded::ENCODED_LEN <= EVENT_PAYLOAD_MAX
        && EventOverflow::ENCODED_LEN <= EVENT_PAYLOAD_MAX
);

/// An `OBSTACLE` event: the obstacle flag changed.
///
/// `detected` is the raw wire byte rather than a `bool`; ADR-0031 defines no range rule here, and
/// the firmware is the only producer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Obstacle {
    /// Whether an obstacle is present, as the firmware reported it.
    pub detected: u8,
    /// The detector's wire id; [`OBSTACLE_SOURCE_LIDAR`] is the `LiDAR`.
    pub source: u8,
}

impl Obstacle {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 2;
}

impl<'a> PayloadCodec<'a> for Obstacle {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self {
            detected: bytes[0],
            source: bytes[1],
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.detected;
        out[1] = self.source;
        Ok(Self::ENCODED_LEN)
    }
}

/// A `FLOOR_DROP` event: the floor-drop flag changed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FloorDrop {
    /// Whether a floor drop is present, as the firmware reported it.
    pub detected: u8,
}

impl FloorDrop {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 1;
}

impl<'a> PayloadCodec<'a> for FloorDrop {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self { detected: bytes[0] })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.is_empty() {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.detected;
        Ok(Self::ENCODED_LEN)
    }
}

/// The completion `status` byte of the two drive-completion events.
///
/// The values mirror the drive subsystem's `CompletionStatus` (ADR-0031): success, cancelled or
/// failed. Any other byte is a [`CodecError::Value`], so a corrupt status is never mistaken for a
/// successful drive.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CompletionStatus {
    /// The move finished as requested.
    Success = 0,
    /// The move was cancelled before it finished.
    Cancelled = 1,
    /// The move failed.
    Failed = 2,
}

impl CompletionStatus {
    /// Every status, in wire order.
    pub const ALL: [Self; 3] = [Self::Success, Self::Cancelled, Self::Failed];

    /// Builds a status from its wire value, returning [`None`] for an unassigned value.
    #[must_use]
    pub const fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Success),
            1 => Some(Self::Cancelled),
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

/// A `PROCEDURE_COMPLETED` event: a Procedure reached a terminal outcome.
///
/// The stage reuses [`ActivityStage`], so a completion event and the `ACTIVITY` stream share their
/// vocabulary. No command sequence is carried because only one Activity runs at a time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProcedureCompleted<'a> {
    /// The Procedure that finished.
    pub procedure_id: ProcedureId,
    /// The terminal stage: [`ActivityStage::Complete`] or [`ActivityStage::Failed`].
    pub stage: ActivityStage,
    /// The operator-facing reason; borrowed from the payload on decode.
    pub detail: &'a str,
}

impl ProcedureCompleted<'_> {
    /// The largest encoded payload length, including the longest allowed detail.
    pub const ENCODED_LEN: usize = 4 + EVENT_DETAIL_MAX;
}

impl<'a> PayloadCodec<'a> for ProcedureCompleted<'a> {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() < 4 {
            return Err(CodecError::Length);
        }
        let detail_len = usize::from(bytes[3]);
        if bytes.len() != 4 + detail_len {
            return Err(CodecError::Length);
        }
        Ok(Self {
            procedure_id: ProcedureId::new(read_u16_le(bytes, 0)),
            stage: ActivityStage::from_wire(bytes[2]).ok_or(CodecError::Value)?,
            detail: core::str::from_utf8(&bytes[4..]).map_err(|_| CodecError::Utf8)?,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let detail = truncate_utf8(self.detail, EVENT_DETAIL_MAX);
        let detail_len = u8::try_from(detail.len()).map_err(|_| EncodeError::Value)?;
        let needed = 4 + detail.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }
        write_u16_le(out, 0, self.procedure_id.get());
        out[2] = self.stage.bits();
        out[3] = detail_len;
        out[4..needed].copy_from_slice(detail.as_bytes());
        Ok(needed)
    }
}

/// A `DRIVE_DISTANCE_COMPLETED` event: a `DRIVE_DISTANCE` command finished.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DriveDistanceCompleted<'a> {
    /// The originating command's sequence number.
    pub command_seq: u16,
    /// The drive outcome.
    pub status: CompletionStatus,
    /// The requested distance in centimetres; positive is forward.
    pub target_cm: i16,
    /// The distance actually travelled in centimetres.
    pub distance_cm: i32,
    /// The move's duration in milliseconds.
    pub duration_ms: u32,
    /// The operator-facing reason; borrowed from the payload on decode.
    pub detail: &'a str,
}

impl DriveDistanceCompleted<'_> {
    /// The largest encoded payload length, including the longest allowed detail.
    pub const ENCODED_LEN: usize = 14 + EVENT_DETAIL_MAX;
}

impl<'a> PayloadCodec<'a> for DriveDistanceCompleted<'a> {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() < 14 {
            return Err(CodecError::Length);
        }
        let detail_len = usize::from(bytes[13]);
        if bytes.len() != 14 + detail_len {
            return Err(CodecError::Length);
        }
        Ok(Self {
            command_seq: read_u16_le(bytes, 0),
            status: CompletionStatus::from_wire(bytes[2]).ok_or(CodecError::Value)?,
            target_cm: i16::from_le_bytes([bytes[3], bytes[4]]),
            distance_cm: i32::from_le_bytes([bytes[5], bytes[6], bytes[7], bytes[8]]),
            duration_ms: read_u32_le(bytes, 9),
            detail: core::str::from_utf8(&bytes[14..]).map_err(|_| CodecError::Utf8)?,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let detail = truncate_utf8(self.detail, EVENT_DETAIL_MAX);
        let detail_len = u8::try_from(detail.len()).map_err(|_| EncodeError::Value)?;
        let needed = 14 + detail.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }
        write_u16_le(out, 0, self.command_seq);
        out[2] = self.status.bits();
        out[3..5].copy_from_slice(&self.target_cm.to_le_bytes());
        out[5..9].copy_from_slice(&self.distance_cm.to_le_bytes());
        write_u32_le(out, 9, self.duration_ms);
        out[13] = detail_len;
        out[14..needed].copy_from_slice(detail.as_bytes());
        Ok(needed)
    }
}

/// A `ROTATE_COMPLETED` event: a `ROTATE_BY` command finished.
///
/// The wire's angles are clockwise; ADR-0039's boundary rule defines [`Self::error_cdeg`] as
/// **actual − target** in clockwise centidegrees, and the daemon negates both angles once when it
/// converts to the App Boundary's counter-clockwise degrees.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RotateCompleted<'a> {
    /// The originating command's sequence number.
    pub command_seq: u16,
    /// The rotation outcome.
    pub status: CompletionStatus,
    /// The requested angle in centidegrees; positive is clockwise.
    pub target_cdeg: i16,
    /// The signed error, **actual − target**, in clockwise centidegrees.
    ///
    /// The daemon negates it once to report the App Boundary's counter-clockwise `error_deg`
    /// (ADR-0039's boundary rule); the wire field itself stays clockwise.
    pub error_cdeg: i32,
    /// The rotation's duration in milliseconds.
    pub duration_ms: u32,
    /// The operator-facing reason; borrowed from the payload on decode.
    pub detail: &'a str,
}

impl RotateCompleted<'_> {
    /// The largest encoded payload length, including the longest allowed detail.
    pub const ENCODED_LEN: usize = 14 + EVENT_DETAIL_MAX;
}

impl<'a> PayloadCodec<'a> for RotateCompleted<'a> {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() < 14 {
            return Err(CodecError::Length);
        }
        let detail_len = usize::from(bytes[13]);
        if bytes.len() != 14 + detail_len {
            return Err(CodecError::Length);
        }
        Ok(Self {
            command_seq: read_u16_le(bytes, 0),
            status: CompletionStatus::from_wire(bytes[2]).ok_or(CodecError::Value)?,
            target_cdeg: i16::from_le_bytes([bytes[3], bytes[4]]),
            error_cdeg: i32::from_le_bytes([bytes[5], bytes[6], bytes[7], bytes[8]]),
            duration_ms: read_u32_le(bytes, 9),
            detail: core::str::from_utf8(&bytes[14..]).map_err(|_| CodecError::Utf8)?,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let detail = truncate_utf8(self.detail, EVENT_DETAIL_MAX);
        let detail_len = u8::try_from(detail.len()).map_err(|_| EncodeError::Value)?;
        let needed = 14 + detail.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }
        write_u16_le(out, 0, self.command_seq);
        out[2] = self.status.bits();
        out[3..5].copy_from_slice(&self.target_cdeg.to_le_bytes());
        out[5..9].copy_from_slice(&self.error_cdeg.to_le_bytes());
        write_u32_le(out, 9, self.duration_ms);
        out[13] = detail_len;
        out[14..needed].copy_from_slice(detail.as_bytes());
        Ok(needed)
    }
}

/// Which calibration data a `CALIBRATION_LOADED` event reports.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CalibrationKind {
    /// Motor calibration.
    Motor = 0,
    /// IMU calibration flags.
    ImuFlags = 1,
    /// Distance calibration.
    Distance = 2,
}

impl CalibrationKind {
    /// Every kind, in wire order.
    pub const ALL: [Self; 3] = [Self::Motor, Self::ImuFlags, Self::Distance];

    /// Builds a kind from its wire value, returning [`None`] for an unassigned value.
    #[must_use]
    pub const fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Motor),
            1 => Some(Self::ImuFlags),
            2 => Some(Self::Distance),
            _ => None,
        }
    }

    /// Returns the wire value.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

/// The `CALIBRATION_LOADED.status` byte.
///
/// This is deliberately not [`CalibrationStatus`](crate::payload::stream::CalibrationStatus):
/// that vocabulary has a distinct `0 = unknown` value, while a load event only reports loaded or
/// missing.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CalibrationLoadStatus {
    /// The calibration data loaded.
    Loaded = 0,
    /// The calibration data is missing.
    Missing = 1,
}

impl CalibrationLoadStatus {
    /// Every status, in wire order.
    pub const ALL: [Self; 2] = [Self::Loaded, Self::Missing];

    /// Builds a status from its wire value, returning [`None`] for an unassigned value.
    #[must_use]
    pub const fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Loaded),
            1 => Some(Self::Missing),
            _ => None,
        }
    }

    /// Returns the wire value.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

/// A `CALIBRATION_LOADED` event: a boot-time calibration load finished.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CalibrationLoaded {
    /// Which calibration data the event concerns.
    pub kind: CalibrationKind,
    /// Whether it loaded or was missing.
    pub status: CalibrationLoadStatus,
}

impl CalibrationLoaded {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 2;
}

impl<'a> PayloadCodec<'a> for CalibrationLoaded {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self {
            kind: CalibrationKind::from_wire(bytes[0]).ok_or(CodecError::Value)?,
            status: CalibrationLoadStatus::from_wire(bytes[1]).ok_or(CodecError::Value)?,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.kind.bits();
        out[1] = self.status.bits();
        Ok(Self::ENCODED_LEN)
    }
}

/// An `EVENT_OVERFLOW` event: the robot's event ring dropped `dropped` events.
///
/// The firmware never sees this overflow; `remote-link` synthesises the event into its own ring.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EventOverflow {
    /// How many events were dropped before room was found for this report.
    pub dropped: u16,
}

impl EventOverflow {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 2;
}

impl<'a> PayloadCodec<'a> for EventOverflow {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self {
            dropped: read_u16_le(bytes, 0),
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        write_u16_le(out, 0, self.dropped);
        Ok(Self::ENCODED_LEN)
    }
}

/// An event acknowledgement: the shared status byte (ADR-0030).
///
/// An ACK mirrors the original frame's class, id and seq. The status is diagnostic: it says whether
/// the peer accepted the event, not whether the underlying drive or Procedure succeeded. Any result
/// bytes after the status are ignored by this version.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EventAck {
    /// The peer's diagnostic status.
    pub status: Status,
}

impl EventAck {
    /// The smallest encoded payload length: the status byte alone.
    pub const MIN_ENCODED_LEN: usize = 1;
}

impl<'a> PayloadCodec<'a> for EventAck {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        let Some(&first) = bytes.first() else {
            return Err(CodecError::Length);
        };
        Ok(Self {
            status: Status::from_byte(first).ok_or(CodecError::Value)?,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.is_empty() {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.status.bits();
        Ok(Self::MIN_ENCODED_LEN)
    }
}
