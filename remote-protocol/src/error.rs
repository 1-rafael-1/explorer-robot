//! The three separated error families (ADR-0030).
//!
//! A frame that does not satisfy the envelope is a [`FrameError`]; a payload that does not match
//! its layout is a [`CodecError`]; and a caller buffer that cannot hold an encoded value is an
//! [`EncodeError`]. An unknown wire major version is deliberately **not** an error here: the
//! frame parses and the layer above decides.

use crate::frame::Status;

/// A byte sequence that does not satisfy the envelope.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// The input ended before the 12-byte header was complete.
    ShortHeader,
    /// The input ended before the declared payload was complete.
    Truncated,
    /// The first two bytes were not the frame magic.
    BadMagic,
    /// The CRC field did not match the computed CRC.
    BadCrc,
    /// The declared payload length exceeds the direction's limit.
    LengthExceeded {
        /// The length declared in the header.
        len: u16,
        /// The largest payload the direction allows.
        max: usize,
    },
    /// The `FRAGMENT` flag is reserved and rejected in v1.
    Fragment,
}

/// A payload that does not match its declared layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CodecError {
    /// The payload length does not match the layout.
    Length,
    /// A value is outside the range the layout allows.
    Value,
    /// A text field is not valid UTF-8.
    Utf8,
}

impl CodecError {
    /// Maps a decode failure onto the ACK status the robot reports (ADR-0030).
    #[must_use]
    pub const fn status(self) -> Status {
        match self {
            Self::Length => Status::BadLength,
            Self::Value | Self::Utf8 => Status::BadArgument,
        }
    }
}

/// A value that cannot be written into the caller's buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    /// The caller's buffer is smaller than the encoded form.
    BufferTooSmall,
    /// The value cannot be represented in the wire layout.
    Value,
}
