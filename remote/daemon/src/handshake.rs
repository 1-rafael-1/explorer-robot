//! The daemon's side of the ADR-0037 HMAC preamble (ADR-0039's `handshake` module).
//!
//! This is a thin adapter over [`bridge_net::preamble`]: it generates the client nonce, sends
//! `CLIENT_HELLO`, verifies the Bridge's proof **before** sending any token-derived byte, then
//! sends `CLIENT_AUTH` and reads the verdict. The proofs, framing and payload codecs are the
//! shared crate's; this module only adds the async transport and the 5 s deadline.
//!
//! The daemon never sends its proof first: a Beacon spoofer can make the daemon connect, but it
//! cannot make it reveal `CLIENT_AUTH` without proving the token first (ADR-0037).

use core::fmt;
use std::time::Duration;

use bridge_net::{
    error::DecodeError,
    identity::Identity,
    preamble::{self, DenyReason, HEADER_LEN, MAX_PAYLOAD_LEN, Message, NONCE_LEN, Received},
};
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

/// How long the daemon allows the whole preamble, including a slow network (ADR-0037's 5 s).
pub const AUTH_DEADLINE: Duration = Duration::from_millis(preamble::AUTH_DEADLINE_MS);

/// The Bridge's accepted identity, learned from the challenge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Accepted {
    /// The device id, name and flags the Bridge serves.
    pub identity: Identity,
}

/// Why the preamble did not reach `ACCEPT`.
#[derive(Debug)]
pub enum HandshakeError {
    /// The socket read or write failed.
    Io(std::io::Error),
    /// A preamble frame did not decode.
    Decode(DecodeError),
    /// The operating system's random source did not provide the client nonce.
    Random,
    /// The Bridge could not prove the token; the daemon sent no token-derived byte.
    BadBridgeProof,
    /// The Bridge refused the connection, with its reason and optional retry hint.
    Denied {
        /// Why the Bridge refused.
        reason: DenyReason,
        /// When to retry, when the refusal carries one.
        retry_ms: Option<u32>,
    },
    /// The peer sent a message the exchange does not allow at that point.
    UnexpectedMessage {
        /// The message type that arrived.
        kind: u8,
    },
    /// The peer closed the connection mid-preamble.
    Closed,
    /// The whole exchange exceeded the deadline.
    Deadline,
}

impl fmt::Display for HandshakeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "the preamble socket failed: {error}"),
            Self::Decode(error) => write!(f, "the preamble did not decode: {error}"),
            Self::Random => write!(f, "the client nonce could not be generated"),
            Self::BadBridgeProof => write!(f, "the Bridge did not prove the pairing token"),
            Self::Denied { reason, retry_ms } => {
                write!(f, "the Bridge denied the preamble ({reason:?}, retry {retry_ms:?})")
            }
            Self::UnexpectedMessage { kind } => write!(f, "unexpected preamble message {kind}"),
            Self::Closed => write!(f, "the peer closed mid-preamble"),
            Self::Deadline => write!(f, "the preamble exceeded its deadline"),
        }
    }
}

impl std::error::Error for HandshakeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Decode(error) => Some(error),
            _ => None,
        }
    }
}

impl From<std::io::Error> for HandshakeError {
    fn from(error: std::io::Error) -> Self {
        Self::Io(error)
    }
}

/// Runs the client side of the preamble over `stream`, bounded by `deadline`.
///
/// The daemon sends only its nonce before it has verified the Bridge's proof. A peer that closes
/// the connection, sends a malformed frame or fails to answer within `deadline` leaves the
/// adapter with an error and no token-derived byte written.
///
/// # Errors
///
/// Returns [`HandshakeError`] for a bad proof, a denial, a short or malformed exchange, an I/O
/// failure or an elapsed deadline.
pub async fn run<S>(stream: &mut S, token: &[u8], deadline: Duration) -> Result<Accepted, HandshakeError>
where
    S: AsyncRead + AsyncWrite + Unpin,
{
    tokio::time::timeout(deadline, exchange(stream, token))
        .await
        .unwrap_or(Err(HandshakeError::Deadline))
}

/// The unbounded exchange; [`run`] imposes the deadline.
async fn exchange<S>(stream: &mut S, token: &[u8]) -> Result<Accepted, HandshakeError>
where
    S: AsyncRead + AsyncWrite + Unpin,
{
    let mut client_nonce = [0u8; NONCE_LEN];
    getrandom::fill(&mut client_nonce).map_err(|_| HandshakeError::Random)?;
    write_message(stream, &Message::ClientHello { nonce: client_nonce }).await?;

    let received = read_message(stream).await?;
    let Message::BridgeChallenge {
        bridge_proof: served_proof,
        bridge_nonce,
        identity,
    } = received.message
    else {
        return match received.message {
            Message::Deny { reason, retry_ms } => Err(HandshakeError::Denied { reason, retry_ms }),
            message => Err(HandshakeError::UnexpectedMessage {
                kind: message.kind().bits(),
            }),
        };
    };
    if received.major != preamble::MAJOR {
        return Err(HandshakeError::Denied {
            reason: DenyReason::Version,
            retry_ms: None,
        });
    }

    // ADR-0037's rule: verify the Bridge before revealing a token-derived byte.
    let expected = preamble::bridge_proof(token, &client_nonce, &bridge_nonce, &identity);
    if !preamble::proofs_match(&expected, &served_proof) {
        return Err(HandshakeError::BadBridgeProof);
    }

    let proof = preamble::client_proof(token, &client_nonce, &bridge_nonce);
    write_message(stream, &Message::ClientAuth { proof }).await?;

    let received = read_message(stream).await?;
    match received.message {
        Message::Accept => Ok(Accepted { identity }),
        Message::Deny { reason, retry_ms } => Err(HandshakeError::Denied { reason, retry_ms }),
        message => Err(HandshakeError::UnexpectedMessage {
            kind: message.kind().bits(),
        }),
    }
}

/// Encodes and writes one framed preamble message.
async fn write_message<S>(stream: &mut S, message: &Message) -> Result<(), HandshakeError>
where
    S: AsyncWrite + Unpin,
{
    let mut buffer = [0u8; HEADER_LEN + MAX_PAYLOAD_LEN];
    let written = preamble::encode_into(preamble::MAJOR, message, &mut buffer).map_err(|_| HandshakeError::Closed)?;
    stream.write_all(&buffer[..written]).await?;
    Ok(())
}

/// Reads one complete framed preamble message without over-reading into Link traffic.
async fn read_message<S>(stream: &mut S) -> Result<Received, HandshakeError>
where
    S: AsyncRead + Unpin,
{
    let mut frame = [0u8; HEADER_LEN + MAX_PAYLOAD_LEN];
    stream.read_exact(&mut frame[..HEADER_LEN]).await?;
    // The framing offsets belong to `bridge-net`; a complete header always announces a length.
    let Some(total) = preamble::frame_len(&frame[..HEADER_LEN]).map_err(HandshakeError::Decode)? else {
        return Err(HandshakeError::Closed);
    };
    stream.read_exact(&mut frame[HEADER_LEN..total]).await?;
    match preamble::try_decode(&frame[..total]) {
        Ok(Some(received)) => Ok(received),
        Ok(None) => Err(HandshakeError::Closed),
        Err(error) => Err(HandshakeError::Decode(error)),
    }
}
