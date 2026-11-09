//! The preamble's transport helpers (ADR-0037, ADR-0039).
//!
//! The message codec, framing and proofs are `bridge-net`'s, so both sides of the exchange share
//! one implementation. This module re-exports them and adds the blocking `std::io` reads and
//! writes the stub's listener needs; the daemon's `handshake` adapter adds the async equivalent.

use std::io::{Read, Write};

pub use bridge_net::preamble::{
    AUTH_DEADLINE_MS, BRIDGE_PROOF_LABEL, CLIENT_PROOF_LABEL, DenyReason, HEADER_LEN, Kind, MAGIC, MAJOR,
    MAX_PAYLOAD_LEN, MINOR, Message, NONCE_LEN, PROOF_LEN, Received, bridge_proof, client_proof, encode_into,
    frame_len, proofs_match, try_decode,
};

use crate::error::DecodeError;

/// Writes one framed message to `w`.
///
/// # Errors
///
/// Returns the I/O error `w` produced.
pub fn write_message<W: Write>(w: &mut W, major: u8, message: &Message) -> std::io::Result<()> {
    let mut buffer = [0u8; HEADER_LEN + MAX_PAYLOAD_LEN];
    let Ok(written) = encode_into(major, message, &mut buffer) else {
        // The buffer is sized for the largest message; this arm is unreachable by construction.
        return Ok(());
    };
    w.write_all(&buffer[..written])
}

/// Reads one framed message from a blocking reader.
///
/// # Errors
///
/// Returns the reader's I/O error, or an invalid-data error for a bad frame.
pub fn read_message<R: Read>(reader: &mut R) -> std::io::Result<Received> {
    let mut buffer = [0u8; HEADER_LEN + MAX_PAYLOAD_LEN];
    reader.read_exact(&mut buffer[..HEADER_LEN])?;
    // The framing offsets belong to `bridge-net`; a complete header always announces a length.
    let Some(total) = frame_len(&buffer[..HEADER_LEN]).map_err(invalid_data)? else {
        return Err(invalid_data(DecodeError::Length {
            expected: HEADER_LEN,
            got: HEADER_LEN,
        }));
    };
    reader.read_exact(&mut buffer[HEADER_LEN..total])?;
    match try_decode(&buffer[..total]) {
        Ok(Some(received)) => Ok(received),
        Ok(None) => Err(invalid_data(DecodeError::Length {
            expected: total,
            got: total,
        })),
        Err(error) => Err(invalid_data(error)),
    }
}

/// Wraps a decode error as an invalid-data I/O error.
fn invalid_data(error: DecodeError) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::InvalidData, error)
}
