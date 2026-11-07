//! The stub's wire decode errors, shared by the preamble and Beacon codecs.

use std::fmt;

/// Why a preamble message or a Beacon datagram did not decode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// The magic bytes do not match the codec's.
    BadMagic,
    /// The message type byte is not part of the preamble vocabulary.
    UnknownType(u8),
    /// The deny reason byte is not part of the vocabulary.
    UnknownReason(u8),
    /// The payload length does not fit the message type.
    Length {
        /// The length the message type requires.
        expected: usize,
        /// The length the payload carried.
        got: usize,
    },
    /// The name field is not valid UTF-8.
    NameUtf8,
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadMagic => write!(f, "the magic bytes do not match"),
            Self::UnknownType(kind) => write!(f, "unknown preamble message type {kind}"),
            Self::UnknownReason(reason) => write!(f, "unknown deny reason {reason}"),
            Self::Length { expected, got } => {
                write!(f, "expected {expected} payload bytes, got {got}")
            }
            Self::NameUtf8 => write!(f, "the name field is not valid UTF-8"),
        }
    }
}

impl std::error::Error for DecodeError {}
