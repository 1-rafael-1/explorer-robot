//! The frozen 12-byte Link Protocol envelope (ADR-0030).
//!
//! A frame is a twelve-byte header followed by a class-specific payload:
//!
//! | offset | size | field |
//! |---|---|---|
//! | 0 | 2 | magic `0xA5 0x5A` |
//! | 2 | 1 | `major` |
//! | 3 | 1 | class in bits 7:6, flags in bits 5:0 |
//! | 4 | 2 | `id`, little-endian |
//! | 6 | 2 | `seq`, little-endian |
//! | 8 | 2 | payload length, little-endian |
//! | 10 | 2 | CRC-16/CCITT-FALSE, little-endian |
//! | 12 | `len` | payload |
//!
//! The magic, header size, length and CRC offsets and byte order are frozen across major
//! versions. An unknown `major` still parses; the layer above decides what it means.

use crate::{
    crc::Crc16,
    error::{EncodeError, FrameError},
};

/// The size of the fixed frame header in bytes.
pub const HEADER_LEN: usize = 12;

/// The two magic bytes that start every frame.
pub const MAGIC: [u8; 2] = [0xA5, 0x5A];

/// The mask covering the six flag bits in the class byte.
const FLAGS_MASK: u8 = 0x3F;

/// The number of bits the class value is shifted left in the class byte.
const CLASS_SHIFT: u8 = 6;

/// The frame's delivery class, encoded in bits 7:6 of the class byte.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Class {
    /// Lossy, latest-wins telemetry from the robot.
    Stream = 0,
    /// Reliable, at-most-once commands from the remote.
    Command = 1,
    /// Reliable events from the robot.
    Event = 2,
    /// Control frames such as `HELLO`, `PING` and `THROTTLE`.
    Control = 3,
}

impl Class {
    /// Every class, in wire order.
    pub const ALL: [Self; 4] = [Self::Stream, Self::Command, Self::Event, Self::Control];

    /// Builds a class from its two-bit wire value.
    ///
    /// Values above `3` do not fit the two-bit field and return [`None`].
    #[must_use]
    pub const fn from_byte(bits: u8) -> Option<Self> {
        match bits {
            0 => Some(Self::Stream),
            1 => Some(Self::Command),
            2 => Some(Self::Event),
            3 => Some(Self::Control),
            _ => None,
        }
    }

    /// Returns the two-bit wire value, ready to be shifted into bits 7:6.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }

    /// Returns the position of the class in [`Class::ALL`].
    #[must_use]
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// The frame's flag bits, encoded in bits 5:0 of the class byte.
///
/// Bits 4 and 5 are reserved: they are cleared by [`Flags::from_bits`] and therefore always sent
/// as zero, and an incoming frame that sets them still decodes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Flags(u8);

impl Flags {
    /// The sender asks for an acknowledgement.
    pub const ACK_REQUESTED: Self = Self(0x01);
    /// The frame is an acknowledgement of the original frame with the same class, id and seq.
    pub const IS_ACK: Self = Self(0x02);
    /// The frame is a retransmission.
    pub const RETRY: Self = Self(0x04);
    /// A fragmented frame; reserved and rejected in v1.
    pub const FRAGMENT: Self = Self(0x08);

    /// Builds flags from the six flag bits, clearing the two reserved bits.
    ///
    /// Clearing the reserved bits is what makes them ignored on receive while ensuring every
    /// encoded frame sends them as zero.
    #[must_use]
    pub const fn from_bits(bits: u8) -> Self {
        Self(bits & FLAGS_MASK & !(Self::RESERVED_BITS))
    }

    /// The mask of the reserved flag bits.
    const RESERVED_BITS: u8 = 0x30;

    /// Returns the six flag bits.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Returns `true` when every bit in `other` is set.
    #[must_use]
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Returns `true` when no flag bit is set.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

impl core::ops::BitOr for Flags {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitOrAssign for Flags {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// An ACK status code carried in the first payload byte of an acknowledgement.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Status {
    /// The command succeeded.
    Ok = 0,
    /// The command was understood but refused.
    Rejected = 1,
    /// The id is not known for the class.
    UnknownId = 2,
    /// A payload value is malformed or out of range.
    BadArgument = 3,
    /// The robot is busy with another activity.
    Busy = 4,
    /// The peer's major version does not match.
    VersionMismatch = 5,
    /// A payload has the wrong structural length.
    BadLength = 6,
    /// A frame has an invalid flag combination.
    BadFlags = 7,
    /// The command is older than the last executed one.
    Stale = 8,
    /// The robot hit an internal error.
    Internal = 9,
}

impl Status {
    /// Every status code, in wire order.
    pub const ALL: [Self; 10] = [
        Self::Ok,
        Self::Rejected,
        Self::UnknownId,
        Self::BadArgument,
        Self::Busy,
        Self::VersionMismatch,
        Self::BadLength,
        Self::BadFlags,
        Self::Stale,
        Self::Internal,
    ];

    /// Builds a status from its wire value.
    ///
    /// Values above `9` are not assigned and return [`None`].
    #[must_use]
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Ok),
            1 => Some(Self::Rejected),
            2 => Some(Self::UnknownId),
            3 => Some(Self::BadArgument),
            4 => Some(Self::Busy),
            5 => Some(Self::VersionMismatch),
            6 => Some(Self::BadLength),
            7 => Some(Self::BadFlags),
            8 => Some(Self::Stale),
            9 => Some(Self::Internal),
            _ => None,
        }
    }

    /// Returns the wire value.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }

    /// Returns the position of the status in [`Status::ALL`].
    #[must_use]
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// A frame sequence number.
///
/// The scope of a sequence number depends on the class: per stream id, per session epoch, per
/// robot boot or per sender, as ADR-0030 defines.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Seq(u16);

impl Seq {
    /// Wraps a raw sequence number.
    #[must_use]
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    /// Returns the raw sequence number.
    #[must_use]
    pub const fn get(self) -> u16 {
        self.0
    }
}

impl From<u16> for Seq {
    fn from(value: u16) -> Self {
        Self(value)
    }
}

/// The direction a frame travels, which fixes its payload limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Direction {
    /// Robot to PC: streams and events, up to 1024 payload bytes.
    RobotToPc,
    /// PC to robot: commands and control, up to 256 payload bytes.
    PcToRobot,
}

impl Direction {
    /// Returns the largest payload this direction allows.
    #[must_use]
    pub const fn max_payload(self) -> usize {
        match self {
            Self::RobotToPc => 1024,
            Self::PcToRobot => 256,
        }
    }

    /// Returns the largest complete frame this direction allows, header included.
    #[must_use]
    pub const fn max_frame(self) -> usize {
        HEADER_LEN + self.max_payload()
    }
}

/// The decoded header of one frame, without its payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameHeader {
    /// The envelope/payload major version.
    pub major: u8,
    /// The frame's delivery class.
    pub class: Class,
    /// The frame's flags.
    pub flags: Flags,
    /// The class-specific id.
    pub id: u16,
    /// The sequence number.
    pub seq: Seq,
    /// The payload length in bytes; equal to the payload slice's length.
    pub len: u16,
}

/// A borrowed frame: a decoded header plus its payload.
///
/// Neither [`Frame::encode_into`] nor [`Frame::decode`] allocates; the payload is either copied
/// into the caller's buffer or borrowed from the input.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame<'a> {
    /// The frame's header.
    pub header: FrameHeader,
    /// The class-specific payload bytes.
    pub payload: &'a [u8],
}

impl<'a> Frame<'a> {
    /// Builds a frame, deriving the header's length field from the payload.
    ///
    /// # Errors
    ///
    /// Returns [`EncodeError::Value`] when the payload is longer than a `u16` can describe.
    pub fn new(
        major: u8,
        class: Class,
        flags: Flags,
        id: u16,
        seq: Seq,
        payload: &'a [u8],
    ) -> Result<Self, EncodeError> {
        let len = u16::try_from(payload.len()).map_err(|_| EncodeError::Value)?;
        Ok(Self {
            header: FrameHeader {
                major,
                class,
                flags,
                id,
                seq,
                len,
            },
            payload,
        })
    }

    /// Returns the number of bytes the frame occupies on the wire.
    #[must_use]
    pub const fn encoded_len(&self) -> usize {
        HEADER_LEN + self.payload.len()
    }

    /// Encodes the frame into the start of `buffer` and returns the number of bytes written.
    ///
    /// The CRC covers header bytes `0..10` and the payload, never the CRC field itself.
    ///
    /// # Errors
    ///
    /// Returns [`EncodeError::BufferTooSmall`] when `buffer` cannot hold the whole frame, and
    /// [`EncodeError::Value`] when the header's length field disagrees with the payload or the
    /// payload cannot be represented as a `u16` length.
    pub fn encode_into(&self, buffer: &mut [u8]) -> Result<usize, EncodeError> {
        let len = u16::try_from(self.payload.len()).map_err(|_| EncodeError::Value)?;
        if self.header.len != len {
            return Err(EncodeError::Value);
        }

        let needed = self.encoded_len();
        if buffer.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }

        buffer[0] = MAGIC[0];
        buffer[1] = MAGIC[1];
        buffer[2] = self.header.major;
        buffer[3] = (self.header.class.bits() << CLASS_SHIFT) | self.header.flags.bits();
        write_u16_le(buffer, 4, self.header.id);
        write_u16_le(buffer, 6, self.header.seq.get());
        write_u16_le(buffer, 8, len);

        let mut crc = Crc16::new();
        crc.update(&buffer[..10]);
        crc.update(self.payload);
        write_u16_le(buffer, 10, crc.finish());

        buffer[HEADER_LEN..needed].copy_from_slice(self.payload);
        Ok(needed)
    }

    /// Decodes one frame from the start of `bytes` for the given direction.
    ///
    /// Trailing bytes after the frame are ignored, so a decode loop can walk several frames in
    /// one input chunk. An unknown `major` is not an error.
    ///
    /// # Errors
    ///
    /// Returns [`FrameError`] for a short header, bad magic, a length above the direction's
    /// limit, the reserved `FRAGMENT` flag, a truncated payload or a bad CRC.
    pub fn decode(bytes: &[u8], direction: Direction) -> Result<Frame<'_>, FrameError> {
        if bytes.len() < HEADER_LEN {
            return Err(FrameError::ShortHeader);
        }
        if bytes[0] != MAGIC[0] || bytes[1] != MAGIC[1] {
            return Err(FrameError::BadMagic);
        }

        let Some(class) = Class::from_byte(bytes[3] >> CLASS_SHIFT) else {
            return Err(FrameError::BadMagic);
        };
        let flags = Flags::from_bits(bytes[3] & FLAGS_MASK);
        let major = bytes[2];
        let id = read_u16_le(bytes, 4);
        let seq = Seq::new(read_u16_le(bytes, 6));
        let len = read_u16_le(bytes, 8);

        if usize::from(len) > direction.max_payload() {
            return Err(FrameError::LengthExceeded {
                len,
                max: direction.max_payload(),
            });
        }
        if flags.contains(Flags::FRAGMENT) {
            return Err(FrameError::Fragment);
        }

        let end = HEADER_LEN + usize::from(len);
        if bytes.len() < end {
            return Err(FrameError::Truncated);
        }

        let payload = &bytes[HEADER_LEN..end];
        let mut crc = Crc16::new();
        crc.update(&bytes[..10]);
        crc.update(payload);
        if crc.finish() != read_u16_le(bytes, 10) {
            return Err(FrameError::BadCrc);
        }

        Ok(Frame {
            header: FrameHeader {
                major,
                class,
                flags,
                id,
                seq,
                len,
            },
            payload,
        })
    }
}

/// Writes a little-endian `u16` at `offset`; the caller has already checked the length.
fn write_u16_le(buffer: &mut [u8], offset: usize, value: u16) {
    let [low, high] = value.to_le_bytes();
    buffer[offset] = low;
    buffer[offset + 1] = high;
}

/// Reads a little-endian `u16` at `offset`; the caller has already checked the length.
fn read_u16_le(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}
