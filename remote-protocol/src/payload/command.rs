//! Typed payloads for the seven v1 commands and their two typed query responses (ADR-0031).
//!
//! | id | command | request layout | max bytes |
//! |---|---|---|---|
//! | `0x0001` | `START_PROCEDURE` | `procedure_id u16`, opaque params | 34 |
//! | `0x0002` | `STOP_PROCEDURE` | `procedure_id u16` | 2 |
//! | `0x0003` | `DRIVE_DISTANCE` | `distance_cm i16`, `speed u8` | 3 |
//! | `0x0004` | `ROTATE_BY` | `angle_cdeg i16`, `speed u8` | 3 |
//! | `0x0005` | `STOP_MOTION` | `brake u8` | 1 |
//! | `0x0006` | `GET_CATALOGUE` | `page u8` | 1 |
//! | `0x0007` | `GET_STATUS` | none | 0 |
//!
//! Every ordinary command ACK is an [`AckPayload`]: one status byte plus optional result bytes.
//! The two query commands replace that generic ACK with [`CataloguePage`] and [`StatusSnapshot`].
//!
//! Semantic validation — availability, claims, drive-busy and per-Procedure parameter schemas —
//! belongs to the firmware. These codecs enforce only ADR-0031's layout, the speed range
//! (`0..=100`, the firmware's documented drive percentage), the signed angle range and the v1
//! sentinels.

use crate::{
    codec::{PayloadCodec, read_u16_le, write_u16_le},
    error::{CodecError, EncodeError},
    frame::Status,
    ids::ProcedureId,
    payload::stream::{ActivityStage, Calibration, CalibrationStatus, Perception},
};

/// The largest `START_PROCEDURE` parameters block the layout allows.
pub const START_PROCEDURE_PARAMS_MAX: usize = 32;

/// The largest catalogue page label the encoder writes, in bytes.
pub const CATALOGUE_LABEL_MAX: usize = 24;

/// The number of entries the robot's catalogue encoder puts on one page.
pub const CATALOGUE_PAGE_SIZE: usize = 8;

/// The largest status detail the encoder writes, in bytes.
pub const STATUS_DETAIL_MAX: usize = 32;

/// The lowest drive speed the firmware documents, as a percentage.
pub const SPEED_MIN: u8 = 0;

/// The highest drive speed the firmware documents, as a percentage.
pub const SPEED_MAX: u8 = 100;

/// The smallest `ROTATE_BY` angle, in centidegrees (`-180°`).
pub const ANGLE_MIN_CDEG: i16 = -18_000;

/// The largest `ROTATE_BY` angle, in centidegrees (`+180°`).
pub const ANGLE_MAX_CDEG: i16 = 18_000;

/// Returns `true` when `speed` is the firmware's documented `0..=100` percentage.
const fn valid_speed(speed: u8) -> bool {
    speed <= SPEED_MAX
}

/// A `START_PROCEDURE` request: a Procedure id and its opaque parameters.
///
/// The parameters are opaque because only the firmware knows the per-Procedure schema; the crate
/// bounds them and nothing else.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StartProcedure<'a> {
    /// The Procedure to start.
    pub procedure_id: ProcedureId,
    /// The opaque parameter bytes; borrowed from the payload on decode.
    pub params: &'a [u8],
}

impl StartProcedure<'_> {
    /// The largest encoded payload length, including the longest allowed parameter block.
    pub const ENCODED_LEN: usize = 2 + START_PROCEDURE_PARAMS_MAX;
}

impl<'a> PayloadCodec<'a> for StartProcedure<'a> {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() < 2 || bytes.len() > Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self {
            procedure_id: ProcedureId::new(read_u16_le(bytes, 0)),
            params: &bytes[2..],
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if self.params.len() > START_PROCEDURE_PARAMS_MAX {
            return Err(EncodeError::Value);
        }
        let needed = 2 + self.params.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }
        write_u16_le(out, 0, self.procedure_id.get());
        out[2..needed].copy_from_slice(self.params);
        Ok(needed)
    }
}

/// A `STOP_PROCEDURE` request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StopProcedure {
    /// The Procedure to stop.
    pub procedure_id: ProcedureId,
}

impl StopProcedure {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 2;
}

impl<'a> PayloadCodec<'a> for StopProcedure {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self {
            procedure_id: ProcedureId::new(read_u16_le(bytes, 0)),
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        write_u16_le(out, 0, self.procedure_id.get());
        Ok(Self::ENCODED_LEN)
    }
}

/// A `DRIVE_DISTANCE` request: a relative distance and a speed percentage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DriveDistance {
    /// The signed relative distance in centimetres; positive is forward.
    pub distance_cm: i16,
    /// The speed magnitude as a `0..=100` percentage.
    pub speed: u8,
}

impl DriveDistance {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 3;
}

impl<'a> PayloadCodec<'a> for DriveDistance {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        if !valid_speed(bytes[2]) {
            return Err(CodecError::Value);
        }
        Ok(Self {
            distance_cm: i16::from_le_bytes([bytes[0], bytes[1]]),
            speed: bytes[2],
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if !valid_speed(self.speed) {
            return Err(EncodeError::Value);
        }
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        let [low, high] = self.distance_cm.to_le_bytes();
        out[0] = low;
        out[1] = high;
        out[2] = self.speed;
        Ok(Self::ENCODED_LEN)
    }
}

/// A `ROTATE_BY` request: a relative signed angle and a speed percentage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RotateBy {
    /// The signed relative angle in centidegrees; positive is clockwise.
    pub angle_cdeg: i16,
    /// The speed magnitude as a `0..=100` percentage.
    pub speed: u8,
}

impl RotateBy {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 3;
}

impl<'a> PayloadCodec<'a> for RotateBy {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        if !valid_speed(bytes[2]) {
            return Err(CodecError::Value);
        }
        let angle_cdeg = i16::from_le_bytes([bytes[0], bytes[1]]);
        if !(ANGLE_MIN_CDEG..=ANGLE_MAX_CDEG).contains(&angle_cdeg) {
            return Err(CodecError::Value);
        }
        Ok(Self {
            angle_cdeg,
            speed: bytes[2],
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if !valid_speed(self.speed) {
            return Err(EncodeError::Value);
        }
        if !(ANGLE_MIN_CDEG..=ANGLE_MAX_CDEG).contains(&self.angle_cdeg) {
            return Err(EncodeError::Value);
        }
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        let [low, high] = self.angle_cdeg.to_le_bytes();
        out[0] = low;
        out[1] = high;
        out[2] = self.speed;
        Ok(Self::ENCODED_LEN)
    }
}

/// A `STOP_MOTION` request: coast or brake immediately.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StopMotion {
    /// The brake mode: [`StopMotion::COAST`] (`0`) or [`StopMotion::BRAKE`] (`1`).
    pub brake: u8,
}

impl StopMotion {
    /// Coast (freewheel).
    pub const COAST: u8 = 0;
    /// Brake.
    pub const BRAKE: u8 = 1;

    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 1;
}

impl<'a> PayloadCodec<'a> for StopMotion {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        match bytes[0] {
            Self::COAST | Self::BRAKE => Ok(Self { brake: bytes[0] }),
            _ => Err(CodecError::Value),
        }
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        match self.brake {
            Self::COAST | Self::BRAKE => {}
            _ => return Err(EncodeError::Value),
        }
        if out.is_empty() {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.brake;
        Ok(Self::ENCODED_LEN)
    }
}

/// A `GET_CATALOGUE` request for one page.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GetCatalogue {
    /// The zero-based page index.
    pub page: u8,
}

impl GetCatalogue {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 1;
}

impl<'a> PayloadCodec<'a> for GetCatalogue {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self { page: bytes[0] })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.is_empty() {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.page;
        Ok(Self::ENCODED_LEN)
    }
}

/// A `GET_STATUS` request, which has no payload.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GetStatus;

impl GetStatus {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 0;
}

impl<'a> PayloadCodec<'a> for GetStatus {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.is_empty() {
            Ok(Self)
        } else {
            Err(CodecError::Length)
        }
    }

    fn encode_into(&self, _out: &mut [u8]) -> Result<usize, EncodeError> {
        Ok(Self::ENCODED_LEN)
    }
}

/// The generic ACK payload of every command except the two query responses.
///
/// ADR-0030: an ACK is the original frame with `IS_ACK` set and its payload is a status byte plus
/// optional result bytes. An unassigned status byte is a value error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AckPayload<'a> {
    /// The command outcome.
    pub status: Status,
    /// Command-specific result bytes; borrowed from the payload on decode.
    pub result: &'a [u8],
}

impl AckPayload<'_> {
    /// The smallest encoded payload length: the status byte alone.
    pub const MIN_ENCODED_LEN: usize = 1;
}

impl<'a> PayloadCodec<'a> for AckPayload<'a> {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        let Some(&first) = bytes.first() else {
            return Err(CodecError::Length);
        };
        let status = Status::from_byte(first).ok_or(CodecError::Value)?;
        Ok(Self {
            status,
            result: &bytes[1..],
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let needed = 1 + self.result.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.status.bits();
        out[1..needed].copy_from_slice(self.result);
        Ok(needed)
    }
}

/// The `GET_CATALOGUE` entry flags (ADR-0031).
///
/// Bits 4–7 are reserved: they are cleared on decode and always sent as zero, mirroring
/// [`Perception`](crate::payload::stream::Perception).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct CatalogueFlags(u8);

impl CatalogueFlags {
    /// The Procedure is interactive.
    pub const INTERACTIVE: Self = Self(0x01);
    /// The Procedure is a calibration.
    pub const CALIBRATION: Self = Self(0x02);
    /// The Procedure takes parameters.
    pub const TAKES_PARAMS: Self = Self(0x04);
    /// The Procedure is startable from the Remote now.
    pub const AVAILABLE: Self = Self(0x08);

    /// The reserved bits, cleared by [`CatalogueFlags::from_bits`].
    const RESERVED_BITS: u8 = 0xF0;

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

impl core::ops::BitOr for CatalogueFlags {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitOrAssign for CatalogueFlags {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// One `GET_CATALOGUE` entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CatalogueEntry<'a> {
    /// The Procedure id.
    pub procedure_id: ProcedureId,
    /// The entry's flags.
    pub flags: CatalogueFlags,
    /// The operator-facing label; borrowed from the payload on decode.
    pub label: &'a str,
}

impl CatalogueEntry<'_> {
    /// The largest encoded entry length, including the longest allowed label.
    pub const ENCODED_LEN: usize = 4 + CATALOGUE_LABEL_MAX;
}

/// Returns the longest prefix of `text` that fits in `max` bytes without splitting a character.
fn truncate_utf8(text: &str, max: usize) -> &str {
    if text.len() <= max {
        return text;
    }
    let mut end = max;
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    &text[..end]
}

/// Decodes one entry from the start of `bytes`, returning it and its encoded length.
fn decode_entry(bytes: &[u8]) -> Result<(CatalogueEntry<'_>, usize), CodecError> {
    if bytes.len() < 4 {
        return Err(CodecError::Length);
    }
    let label_len = usize::from(bytes[3]);
    let total = 4 + label_len;
    if bytes.len() < total {
        return Err(CodecError::Length);
    }
    let label = core::str::from_utf8(&bytes[4..total]).map_err(|_| CodecError::Utf8)?;
    Ok((
        CatalogueEntry {
            procedure_id: ProcedureId::new(read_u16_le(bytes, 0)),
            flags: CatalogueFlags::from_bits(bytes[2]),
            label,
        },
        total,
    ))
}

/// A `GET_CATALOGUE` response: the header plus a borrowed block of encoded entries.
///
/// Decoding validates the whole page: `count` entries must consume the payload exactly, and every
/// label must be valid UTF-8. Use [`CataloguePage::entries`] to iterate the decoded entries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CataloguePage<'a> {
    /// The total number of catalogue entries, across every page.
    pub total: u8,
    /// The zero-based page index this response carries.
    pub page: u8,
    /// The number of entries on this page.
    count: u8,
    /// The raw encoded entries, exactly as they arrived.
    entries: &'a [u8],
}

impl<'a> CataloguePage<'a> {
    /// The fixed header length in bytes.
    pub const HEADER_LEN: usize = 3;

    /// The largest encoded payload length: a full page of maximum-length entries.
    pub const ENCODED_LEN: usize = Self::HEADER_LEN + CATALOGUE_PAGE_SIZE * CatalogueEntry::ENCODED_LEN;

    /// Returns the number of entries on this page.
    #[must_use]
    pub const fn count(&self) -> u8 {
        self.count
    }

    /// Returns an iterator over the page's entries.
    #[must_use]
    pub const fn entries(&self) -> CatalogueEntries<'a> {
        CatalogueEntries {
            bytes: self.entries,
            remaining: self.count,
        }
    }
}

impl<'a> PayloadCodec<'a> for CataloguePage<'a> {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() < Self::HEADER_LEN {
            return Err(CodecError::Length);
        }
        let count = bytes[2];
        let mut rest = &bytes[Self::HEADER_LEN..];
        for _ in 0..count {
            let (_, consumed) = decode_entry(rest)?;
            rest = &rest[consumed..];
        }
        if !rest.is_empty() {
            return Err(CodecError::Length);
        }
        Ok(Self {
            total: bytes[0],
            page: bytes[1],
            count,
            entries: &bytes[Self::HEADER_LEN..],
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let needed = Self::HEADER_LEN + self.entries.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.total;
        out[1] = self.page;
        out[2] = self.count;
        out[Self::HEADER_LEN..needed].copy_from_slice(self.entries);
        Ok(needed)
    }
}

/// An iterator over the entries of a decoded [`CataloguePage`].
#[derive(Clone, Debug)]
pub struct CatalogueEntries<'a> {
    /// The remaining encoded entries.
    bytes: &'a [u8],
    /// The number of entries still to yield.
    remaining: u8,
}

impl<'a> Iterator for CatalogueEntries<'a> {
    type Item = CatalogueEntry<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }
        self.remaining -= 1;
        let (entry, consumed) = decode_entry(self.bytes).ok()?;
        self.bytes = &self.bytes[consumed..];
        Some(entry)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = usize::from(self.remaining);
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for CatalogueEntries<'_> {}

/// Encodes `total`, `page` and `entries` as a `GET_CATALOGUE` response.
///
/// The caller supplies the entries as a slice or any other iterator; labels longer than
/// [`CATALOGUE_LABEL_MAX`] are truncated at a UTF-8 character boundary. A page larger than 255
/// entries cannot be represented and is [`EncodeError::Value`].
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] when `out` cannot hold the page and [`EncodeError::Value`] when
/// the entry count does not fit a `u8`.
pub fn encode_catalogue_page<'a, I>(total: u8, page: u8, entries: I, out: &mut [u8]) -> Result<usize, EncodeError>
where
    I: IntoIterator<Item = CatalogueEntry<'a>>,
{
    if out.len() < CataloguePage::HEADER_LEN {
        return Err(EncodeError::BufferTooSmall);
    }
    out[0] = total;
    out[1] = page;

    let mut written = CataloguePage::HEADER_LEN;
    let mut count = 0u8;
    for entry in entries {
        let label = truncate_utf8(entry.label, CATALOGUE_LABEL_MAX);
        let label_len = u8::try_from(label.len()).map_err(|_| EncodeError::Value)?;
        let needed = written + 4 + label.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }
        write_u16_le(out, written, entry.procedure_id.get());
        out[written + 2] = entry.flags.bits();
        out[written + 3] = label_len;
        out[written + 4..needed].copy_from_slice(label.as_bytes());
        written = needed;
        count = count.checked_add(1).ok_or(EncodeError::Value)?;
    }
    out[2] = count;
    Ok(written)
}

/// A `GET_STATUS` response: one snapshot of every observable state stream.
///
/// The fields mirror the individual streams: `level`/`voltage_mv` are `POWER`, `left`/`right` are
/// `MOTION`, the Procedure fields are `ACTIVITY`, `flags` is `PERCEPTION` and `calibration` is
/// `CALIBRATION`. `percent` uses the `ACTIVITY` sentinel [`StatusSnapshot::PERCENT_NONE`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StatusSnapshot<'a> {
    /// Battery level percentage, or `POWER`'s `0xFF` unknown sentinel.
    pub level: u8,
    /// Pack voltage in millivolts, or `POWER`'s `0xFFFF` unknown sentinel.
    pub voltage_mv: u16,
    /// The left motor output.
    pub left: i8,
    /// The right motor output.
    pub right: i8,
    /// The Procedure id, including one this version does not know.
    pub procedure_id: ProcedureId,
    /// The Procedure's stage.
    pub stage: ActivityStage,
    /// Completion percentage, or [`None`] for "not reported".
    pub percent: Option<u8>,
    /// The operator-facing detail; borrowed from the payload on decode.
    pub detail: &'a str,
    /// The perception flags, mirroring the `PERCEPTION` stream.
    pub flags: Perception,
    /// The four calibration statuses, mirroring the `CALIBRATION` stream.
    pub calibration: Calibration,
}

impl StatusSnapshot<'_> {
    /// The `percent` sentinel meaning "not reported".
    pub const PERCENT_NONE: u8 = 0xFF;

    /// The largest encoded payload length, including the longest allowed detail.
    pub const ENCODED_LEN: usize = 15 + STATUS_DETAIL_MAX;
}

impl<'a> PayloadCodec<'a> for StatusSnapshot<'a> {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() < 15 {
            return Err(CodecError::Length);
        }
        let detail_len = usize::from(bytes[9]);
        if bytes.len() != 15 + detail_len {
            return Err(CodecError::Length);
        }
        let stage = ActivityStage::from_wire(bytes[7]).ok_or(CodecError::Value)?;
        let percent = match bytes[8] {
            Self::PERCENT_NONE => None,
            value => Some(value),
        };
        let detail = core::str::from_utf8(&bytes[10..10 + detail_len]).map_err(|_| CodecError::Utf8)?;
        let flags = Perception::from_bits(bytes[10 + detail_len]);
        let calibration_offset = 11 + detail_len;
        let calibration = Calibration {
            motor: CalibrationStatus::from_wire(bytes[calibration_offset]).ok_or(CodecError::Value)?,
            imu: CalibrationStatus::from_wire(bytes[calibration_offset + 1]).ok_or(CodecError::Value)?,
            mag: CalibrationStatus::from_wire(bytes[calibration_offset + 2]).ok_or(CodecError::Value)?,
            distance: CalibrationStatus::from_wire(bytes[calibration_offset + 3]).ok_or(CodecError::Value)?,
        };
        Ok(Self {
            level: bytes[0],
            voltage_mv: read_u16_le(bytes, 1),
            left: i8::from_le_bytes([bytes[3]]),
            right: i8::from_le_bytes([bytes[4]]),
            procedure_id: ProcedureId::new(read_u16_le(bytes, 5)),
            stage,
            percent,
            detail,
            flags,
            calibration,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let percent = match self.percent {
            None => Self::PERCENT_NONE,
            Some(Self::PERCENT_NONE) => return Err(EncodeError::Value),
            Some(value) => value,
        };
        let detail = truncate_utf8(self.detail, STATUS_DETAIL_MAX);
        let detail_len = u8::try_from(detail.len()).map_err(|_| EncodeError::Value)?;
        let needed = 15 + detail.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }

        out[0] = self.level;
        write_u16_le(out, 1, self.voltage_mv);
        out[3] = self.left.to_le_bytes()[0];
        out[4] = self.right.to_le_bytes()[0];
        write_u16_le(out, 5, self.procedure_id.get());
        out[7] = self.stage.bits();
        out[8] = percent;
        out[9] = detail_len;
        out[10..10 + detail.len()].copy_from_slice(detail.as_bytes());
        let flags_offset = 10 + detail.len();
        out[flags_offset] = self.flags.bits();
        let calibration_offset = flags_offset + 1;
        out[calibration_offset] = self.calibration.motor.bits();
        out[calibration_offset + 1] = self.calibration.imu.bits();
        out[calibration_offset + 2] = self.calibration.mag.bits();
        out[calibration_offset + 3] = self.calibration.distance.bits();
        Ok(needed)
    }
}
