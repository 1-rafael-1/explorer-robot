//! Typed payloads for the control frames `HELLO`, `HELLO_ACK`, `PING` and `THROTTLE` (ADR-0030,
//! ADR-0031).
//!
//! The layouts are little-endian and the rate-entry list is variable-length; it is bounded only
//! by the 256-byte PC-to-robot cap. Decoding borrows the entries from the input, so no codec
//! allocates.
//!
//! `BRIDGE_CONTROL` is part of the [`Control`](crate::ids::Control) vocabulary but its payload
//! codec arrives with a later ticket. `THROTTLE` has no acknowledgement.
//!
//! | payload | layout |
//! |---|---|
//! | `HELLO` | `major u8`, `minor u8`, `caps u32`, `epoch u32`, entries |
//! | `HELLO_ACK` | `major u8`, `minor u8`, `caps u32`, `boot_id u32`, `status u8`, entries |
//! | `PING` | empty |
//! | `PING_ACK` | `status u8` |
//! | `THROTTLE` | `stream_id u16`, `action u8`, `rate_chz u16`, `ttl_ms u16` |
//!
//! Each rate entry is `stream_id u16`, `rate_chz u16`, `flags u8`.

use crate::{
    codec::{PayloadCodec, read_u16_le, read_u32_le, write_u16_le, write_u32_le},
    error::{CodecError, EncodeError},
    frame::Status,
    ids::Caps,
};

/// The rate-entry flag bit meaning the stream is requested.
const RATE_FLAG_ENABLED: u8 = 0x01;

/// The reserved rate-entry flag bits, cleared on decode and always sent as zero.
const RATE_FLAG_RESERVED: u8 = 0xFE;

/// The flags byte of a `HELLO`/`HELLO_ACK` rate entry.
///
/// Bit 0 is `ENABLED`; bits 1–7 are reserved. They are cleared by [`RateFlags::from_bits`], so
/// this version always sends them as zero and ignores them on receive.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct RateFlags(u8);

impl RateFlags {
    /// The stream is requested for the Session.
    pub const ENABLED: Self = Self(RATE_FLAG_ENABLED);

    /// Builds flags from a raw byte, clearing the reserved bits.
    #[must_use]
    pub const fn from_bits(bits: u8) -> Self {
        Self(bits & !RATE_FLAG_RESERVED)
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

    /// Returns `true` when no flag bit is set.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Returns `true` when `ENABLED` is set.
    #[must_use]
    pub const fn is_enabled(self) -> bool {
        self.contains(Self::ENABLED)
    }
}

impl core::ops::BitOr for RateFlags {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitOrAssign for RateFlags {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// One `HELLO`/`HELLO_ACK` rate entry: a Requested Rate for one stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RateEntry {
    /// The stream id; a receiver ignores one it does not implement.
    pub stream_id: u16,
    /// The Requested Rate in centi-Hz.
    pub rate_chz: u16,
    /// The entry's flags.
    pub flags: RateFlags,
}

impl RateEntry {
    /// The encoded entry length in bytes.
    pub const ENCODED_LEN: usize = 5;

    /// Builds an enabled entry for `stream_id`.
    #[must_use]
    pub const fn enabled(stream_id: u16, rate_chz: u16) -> Self {
        Self {
            stream_id,
            rate_chz,
            flags: RateFlags::ENABLED,
        }
    }
}

/// Reads one entry from the start of `bytes`, returning it and its encoded length.
fn decode_entry(bytes: &[u8]) -> Result<(RateEntry, usize), CodecError> {
    if bytes.len() < RateEntry::ENCODED_LEN {
        return Err(CodecError::Length);
    }
    Ok((
        RateEntry {
            stream_id: read_u16_le(bytes, 0),
            rate_chz: read_u16_le(bytes, 2),
            flags: RateFlags::from_bits(bytes[4]),
        },
        RateEntry::ENCODED_LEN,
    ))
}

/// An iterator over the rate entries of a decoded [`Hello`] or [`HelloAck`].
#[derive(Clone, Debug)]
pub struct RateEntries<'a> {
    /// The remaining encoded entries.
    bytes: &'a [u8],
    /// The number of entries still to yield.
    remaining: usize,
}

impl Iterator for RateEntries<'_> {
    type Item = RateEntry;

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
        let remaining = self.remaining;
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for RateEntries<'_> {}

/// A `HELLO` request: the peer's versions, capabilities, epoch and Requested Rates.
///
/// The rate entries borrow the frame payload on decode; use [`Hello::entries`] to iterate them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Hello<'a> {
    /// The peer's envelope/payload major.
    pub major: u8,
    /// The peer's payload minor.
    pub minor: u8,
    /// The peer's capability bitset, including bits this version does not know.
    pub caps: Caps,
    /// The peer's Session epoch.
    pub epoch: u32,
    /// The raw encoded rate entries.
    entries: &'a [u8],
}

impl<'a> Hello<'a> {
    /// The fixed header length in bytes.
    pub const HEADER_LEN: usize = 10;

    /// Returns the number of rate entries.
    #[must_use]
    pub const fn entry_count(&self) -> usize {
        self.entries.len() / RateEntry::ENCODED_LEN
    }

    /// Returns an iterator over the rate entries.
    #[must_use]
    pub const fn entries(&self) -> RateEntries<'a> {
        RateEntries {
            bytes: self.entries,
            remaining: self.entry_count(),
        }
    }
}

impl<'a> PayloadCodec<'a> for Hello<'a> {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() < Self::HEADER_LEN {
            return Err(CodecError::Length);
        }
        let entries = &bytes[Self::HEADER_LEN..];
        if !entries.len().is_multiple_of(RateEntry::ENCODED_LEN) {
            return Err(CodecError::Length);
        }
        Ok(Self {
            major: bytes[0],
            minor: bytes[1],
            caps: Caps::from_bits(read_u32_le(bytes, 2)),
            epoch: read_u32_le(bytes, 6),
            entries,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let needed = Self::HEADER_LEN + self.entries.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.major;
        out[1] = self.minor;
        write_u32_le(out, 2, self.caps.bits());
        write_u32_le(out, 6, self.epoch);
        out[Self::HEADER_LEN..needed].copy_from_slice(self.entries);
        Ok(needed)
    }
}

/// Encodes a `HELLO` request into `out`, returning the number of bytes written.
///
/// The entries are written in iteration order; duplicate and unknown stream ids are the
/// receiver's concern, not the codec's.
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] when `out` cannot hold the header and every entry.
pub fn encode_hello<I>(
    major: u8,
    minor: u8,
    caps: Caps,
    epoch: u32,
    entries: I,
    out: &mut [u8],
) -> Result<usize, EncodeError>
where
    I: IntoIterator<Item = RateEntry>,
{
    if out.len() < Hello::HEADER_LEN {
        return Err(EncodeError::BufferTooSmall);
    }
    out[0] = major;
    out[1] = minor;
    write_u32_le(out, 2, caps.bits());
    write_u32_le(out, 6, epoch);

    let mut written = Hello::HEADER_LEN;
    for entry in entries {
        let end = written + RateEntry::ENCODED_LEN;
        if out.len() < end {
            return Err(EncodeError::BufferTooSmall);
        }
        write_u16_le(out, written, entry.stream_id);
        write_u16_le(out, written + 2, entry.rate_chz);
        out[written + 4] = entry.flags.bits();
        written = end;
    }
    Ok(written)
}

/// A `HELLO_ACK`: the robot's versions, capabilities, Boot Id, status and default rates.
///
/// The rate entries borrow the frame payload on decode; use [`HelloAck::entries`] to iterate them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HelloAck<'a> {
    /// The robot's envelope/payload major.
    pub major: u8,
    /// The robot's payload minor.
    pub minor: u8,
    /// The capabilities both peers support.
    pub caps: Caps,
    /// The robot's Boot Id for this powered link period.
    pub boot_id: u32,
    /// The handshake status; v1 produces `OK`, `VERSION_MISMATCH` or `BAD_ARGUMENT`.
    pub status: Status,
    /// The raw encoded default rate entries.
    entries: &'a [u8],
}

impl<'a> HelloAck<'a> {
    /// The fixed header length in bytes.
    pub const HEADER_LEN: usize = 11;

    /// Returns the number of rate entries.
    #[must_use]
    pub const fn entry_count(&self) -> usize {
        self.entries.len() / RateEntry::ENCODED_LEN
    }

    /// Returns an iterator over the default rate entries.
    #[must_use]
    pub const fn entries(&self) -> RateEntries<'a> {
        RateEntries {
            bytes: self.entries,
            remaining: self.entry_count(),
        }
    }
}

impl<'a> PayloadCodec<'a> for HelloAck<'a> {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() < Self::HEADER_LEN {
            return Err(CodecError::Length);
        }
        let entries = &bytes[Self::HEADER_LEN..];
        if !entries.len().is_multiple_of(RateEntry::ENCODED_LEN) {
            return Err(CodecError::Length);
        }
        let status = Status::from_byte(bytes[10]).ok_or(CodecError::Value)?;
        Ok(Self {
            major: bytes[0],
            minor: bytes[1],
            caps: Caps::from_bits(read_u32_le(bytes, 2)),
            boot_id: read_u32_le(bytes, 6),
            status,
            entries,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let needed = Self::HEADER_LEN + self.entries.len();
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.major;
        out[1] = self.minor;
        write_u32_le(out, 2, self.caps.bits());
        write_u32_le(out, 6, self.boot_id);
        out[10] = self.status.bits();
        out[Self::HEADER_LEN..needed].copy_from_slice(self.entries);
        Ok(needed)
    }
}

/// Encodes a `HELLO_ACK` into `out`, returning the number of bytes written.
///
/// The entries are written in iteration order.
///
/// # Errors
///
/// [`EncodeError::BufferTooSmall`] when `out` cannot hold the header and every entry.
pub fn encode_hello_ack<I>(
    major: u8,
    minor: u8,
    caps: Caps,
    boot_id: u32,
    status: Status,
    entries: I,
    out: &mut [u8],
) -> Result<usize, EncodeError>
where
    I: IntoIterator<Item = RateEntry>,
{
    if out.len() < HelloAck::HEADER_LEN {
        return Err(EncodeError::BufferTooSmall);
    }
    out[0] = major;
    out[1] = minor;
    write_u32_le(out, 2, caps.bits());
    write_u32_le(out, 6, boot_id);
    out[10] = status.bits();

    let mut written = HelloAck::HEADER_LEN;
    for entry in entries {
        let end = written + RateEntry::ENCODED_LEN;
        if out.len() < end {
            return Err(EncodeError::BufferTooSmall);
        }
        write_u16_le(out, written, entry.stream_id);
        write_u16_le(out, written + 2, entry.rate_chz);
        out[written + 4] = entry.flags.bits();
        written = end;
    }
    Ok(written)
}

/// A `PING` request: an empty payload.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ping;

impl Ping {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 0;
}

impl<'a> PayloadCodec<'a> for Ping {
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

/// A `PING` acknowledgement: a single status byte, which v1 always sets to `OK`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PingAck {
    /// The reply status.
    pub status: Status,
}

impl PingAck {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 1;
}

impl<'a> PayloadCodec<'a> for PingAck {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        Ok(Self {
            status: Status::from_byte(bytes[0]).ok_or(CodecError::Value)?,
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.is_empty() {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.status.bits();
        Ok(Self::ENCODED_LEN)
    }
}

/// The action of a `THROTTLE` hint (ADR-0030).
///
/// The discriminants are the wire values. A `SET_RATE` with a zero rate is not a request to emit
/// at zero: the receiver treats it as the most restrictive state, a pause.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ThrottleAction {
    /// `SET_RATE`: set a maximum rate in centi-Hz for the hint's lifetime.
    SetRate = 0,
    /// `PAUSE`: stop emission for the hint's lifetime.
    Pause = 1,
    /// `RESUME`: clear a pause immediately.
    Resume = 2,
}

impl ThrottleAction {
    /// Every action in wire order.
    pub const ALL: [Self; 3] = [Self::SetRate, Self::Pause, Self::Resume];

    /// Returns the wire value.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }

    /// Builds an action from its wire value, returning [`None`] for an unassigned value.
    #[must_use]
    pub const fn from_wire(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::SetRate),
            1 => Some(Self::Pause),
            2 => Some(Self::Resume),
            _ => None,
        }
    }
}

/// A `THROTTLE` hint: an advisory per-stream rate or pause with a time-to-live.
///
/// The hint is never acknowledged. `stream_id` addresses one stream or, with
/// [`Throttle::ALL_STREAMS`], every stream. `rate_chz` is a maximum in centi-Hz and is only
/// meaningful for [`ThrottleAction::SetRate`]; `ttl_ms` is a lifetime in milliseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Throttle {
    /// The stream id, or [`Throttle::ALL_STREAMS`] for every stream.
    pub stream_id: u16,
    /// The action to apply.
    pub action: ThrottleAction,
    /// The maximum rate in centi-Hz; zero means the most restrictive state (a pause).
    pub rate_chz: u16,
    /// The lifetime of the hint in milliseconds.
    pub ttl_ms: u16,
}

impl Throttle {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 7;

    /// The stream id that addresses every stream.
    pub const ALL_STREAMS: u16 = 0xFFFF;

    /// Builds a hint for one stream or every stream.
    #[must_use]
    pub const fn new(stream_id: u16, action: ThrottleAction, rate_chz: u16, ttl_ms: u16) -> Self {
        Self {
            stream_id,
            action,
            rate_chz,
            ttl_ms,
        }
    }

    /// Builds a hint that addresses every stream.
    #[must_use]
    pub const fn all(action: ThrottleAction, rate_chz: u16, ttl_ms: u16) -> Self {
        Self::new(Self::ALL_STREAMS, action, rate_chz, ttl_ms)
    }
}

impl<'a> PayloadCodec<'a> for Throttle {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        let action = ThrottleAction::from_wire(bytes[2]).ok_or(CodecError::Value)?;
        Ok(Self {
            stream_id: read_u16_le(bytes, 0),
            action,
            rate_chz: read_u16_le(bytes, 3),
            ttl_ms: read_u16_le(bytes, 5),
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        write_u16_le(out, 0, self.stream_id);
        out[2] = self.action.bits();
        write_u16_le(out, 3, self.rate_chz);
        write_u16_le(out, 5, self.ttl_ms);
        Ok(Self::ENCODED_LEN)
    }
}
