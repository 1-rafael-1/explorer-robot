//! The pre-Link HMAC-SHA256 preamble exchanged before any Link byte (ADR-0037, ADR-0039).
//!
//! Both sides of the exchange compose this module, so a Bridge/daemon divergence is a compile
//! error, not a bench finding. One message is framed as `magic "EXPH"`, the preamble major, the
//! message type, a little-endian `u16` payload length and the payload:
//!
//! | type | message | payload |
//! |---|---|---|
//! | 1 | `CLIENT_HELLO` | the 32-byte client nonce |
//! | 2 | `BRIDGE_CHALLENGE` | the 32-byte bridge proof, the 32-byte bridge nonce, then the [`Identity`] |
//! | 3 | `CLIENT_AUTH` | the 32-byte client proof |
//! | 4 | `ACCEPT` | empty; the Link Protocol starts |
//! | 5 | `DENY` | the reason byte, then the optional little-endian `retry_ms` |
//!
//! The proofs are HMAC-SHA256 over this module's domain-separated labels, and the bridge proof
//! binds the identity fields exactly as they are served on the wire. Everything here is `no_std`
//! and allocation-free; transport reads and writes stay in the consuming crates.

use hmac::{Hmac, Mac};
use remote_protocol::EncodeError;
use sha2::Sha256;

use crate::{error::DecodeError, identity::Identity};

/// The preamble's magic, `EXPH`.
pub const MAGIC: [u8; 4] = *b"EXPH";

/// The preamble major version.
pub const MAJOR: u8 = 1;

/// The preamble minor version.
pub const MINOR: u8 = 0;

/// How long the Bridge waits for a connection to authenticate, in milliseconds.
pub const AUTH_DEADLINE_MS: u64 = 5_000;

/// The domain-separation label for the Bridge's proof.
pub const BRIDGE_PROOF_LABEL: &[u8] = b"explorer-link-v1 bridge\0";

/// The domain-separation label for the client's proof.
pub const CLIENT_PROOF_LABEL: &[u8] = b"explorer-link-v1 client\0";

/// The fixed header length: magic, major, type and the payload length.
pub const HEADER_LEN: usize = 8;

/// The nonce length.
pub const NONCE_LEN: usize = 32;

/// The proof length; HMAC-SHA256's full output.
pub const PROOF_LEN: usize = 32;

/// The largest payload: the challenge, its two 32-byte fields ahead of the identity.
pub const MAX_PAYLOAD_LEN: usize = PROOF_LEN + NONCE_LEN + Identity::ENCODED_LEN;

/// The preamble message types, in exchange order.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Kind {
    /// The daemon's nonce.
    ClientHello = 1,
    /// The Bridge's proof, nonce and identity.
    BridgeChallenge = 2,
    /// The daemon's proof.
    ClientAuth = 3,
    /// The Bridge's verdict; the Link Protocol starts.
    Accept = 4,
    /// The Bridge's refusal, with a reason and an optional retry hint.
    Deny = 5,
}

impl Kind {
    /// Every message type, in exchange order.
    pub const ALL: [Self; 5] = [
        Self::ClientHello,
        Self::BridgeChallenge,
        Self::ClientAuth,
        Self::Accept,
        Self::Deny,
    ];

    /// Builds a type from its wire value, returning [`None`] for an unassigned byte.
    #[must_use]
    pub const fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            1 => Some(Self::ClientHello),
            2 => Some(Self::BridgeChallenge),
            3 => Some(Self::ClientAuth),
            4 => Some(Self::Accept),
            5 => Some(Self::Deny),
            _ => None,
        }
    }

    /// Returns the wire value.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

/// The deny reasons of ADR-0039's message table.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DenyReason {
    /// The client's proof did not verify; surfaced and not auto-retried.
    Unauthorized = 0,
    /// Another authenticated client is live; retried after `retry_ms`.
    Busy = 1,
    /// The Bridge is in Setup Mode and declines the Link Protocol.
    SetupMode = 2,
    /// The peer's preamble major does not match.
    Version = 3,
}

impl DenyReason {
    /// Every reason, in wire order.
    pub const ALL: [Self; 4] = [Self::Unauthorized, Self::Busy, Self::SetupMode, Self::Version];

    /// Builds a reason from its wire value, returning [`None`] for an unassigned byte.
    #[must_use]
    pub const fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Unauthorized),
            1 => Some(Self::Busy),
            2 => Some(Self::SetupMode),
            3 => Some(Self::Version),
            _ => None,
        }
    }

    /// Returns the wire value.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

/// One preamble message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Message {
    /// `CLIENT_HELLO`: the daemon's nonce.
    ClientHello {
        /// The 32-byte client nonce.
        nonce: [u8; NONCE_LEN],
    },
    /// `BRIDGE_CHALLENGE`: the Bridge's proof, nonce and identity.
    BridgeChallenge {
        /// The Bridge's proof over the whole transcript.
        bridge_proof: [u8; PROOF_LEN],
        /// The Bridge's nonce.
        bridge_nonce: [u8; NONCE_LEN],
        /// The identity fields the daemon learns here.
        identity: Identity,
    },
    /// `CLIENT_AUTH`: the daemon's proof.
    ClientAuth {
        /// The client's proof.
        proof: [u8; PROOF_LEN],
    },
    /// `ACCEPT`: the Link Protocol starts.
    Accept,
    /// `DENY`: the refusal, with its reason and optional retry hint.
    Deny {
        /// Why the Bridge refused.
        reason: DenyReason,
        /// When to retry, when the reason carries one.
        retry_ms: Option<u32>,
    },
}

impl Message {
    /// Returns the message's wire type.
    #[must_use]
    pub const fn kind(&self) -> Kind {
        match self {
            Self::ClientHello { .. } => Kind::ClientHello,
            Self::BridgeChallenge { .. } => Kind::BridgeChallenge,
            Self::ClientAuth { .. } => Kind::ClientAuth,
            Self::Accept => Kind::Accept,
            Self::Deny { .. } => Kind::Deny,
        }
    }

    /// Returns the encoded payload length.
    #[must_use]
    pub const fn payload_len(&self) -> usize {
        match self {
            Self::ClientHello { .. } => NONCE_LEN,
            Self::BridgeChallenge { .. } => PROOF_LEN + NONCE_LEN + Identity::ENCODED_LEN,
            Self::ClientAuth { .. } => PROOF_LEN,
            Self::Accept => 0,
            Self::Deny { retry_ms, .. } => match retry_ms {
                Some(_) => 5,
                None => 1,
            },
        }
    }

    /// Encodes the payload into `out`.
    ///
    /// # Errors
    ///
    /// Returns [`EncodeError::BufferTooSmall`] when `out` cannot hold the payload.
    pub fn encode_payload_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        match self {
            Self::ClientHello { nonce } => {
                if out.len() < NONCE_LEN {
                    return Err(EncodeError::BufferTooSmall);
                }
                out[..NONCE_LEN].copy_from_slice(nonce);
                Ok(NONCE_LEN)
            }
            Self::BridgeChallenge {
                bridge_proof,
                bridge_nonce,
                identity,
            } => {
                let needed = PROOF_LEN + NONCE_LEN + Identity::ENCODED_LEN;
                if out.len() < needed {
                    return Err(EncodeError::BufferTooSmall);
                }
                out[..PROOF_LEN].copy_from_slice(bridge_proof);
                out[PROOF_LEN..PROOF_LEN + NONCE_LEN].copy_from_slice(bridge_nonce);
                identity.encode_into(&mut out[PROOF_LEN + NONCE_LEN..])?;
                Ok(needed)
            }
            Self::ClientAuth { proof } => {
                if out.len() < PROOF_LEN {
                    return Err(EncodeError::BufferTooSmall);
                }
                out[..PROOF_LEN].copy_from_slice(proof);
                Ok(PROOF_LEN)
            }
            Self::Accept => Ok(0),
            Self::Deny { reason, retry_ms } => {
                let Some(retry_ms) = retry_ms else {
                    if out.is_empty() {
                        return Err(EncodeError::BufferTooSmall);
                    }
                    out[0] = reason.bits();
                    return Ok(1);
                };
                if out.len() < 5 {
                    return Err(EncodeError::BufferTooSmall);
                }
                out[0] = reason.bits();
                out[1..5].copy_from_slice(&retry_ms.to_le_bytes());
                Ok(5)
            }
        }
    }

    /// Decodes the payload of `kind`.
    ///
    /// # Errors
    ///
    /// Returns [`DecodeError`] when the payload's length or fields do not follow the layout.
    pub fn decode_payload(kind: Kind, payload: &[u8]) -> Result<Self, DecodeError> {
        match kind {
            Kind::ClientHello => {
                let nonce = fixed_field::<NONCE_LEN>(payload)?;
                Ok(Self::ClientHello { nonce })
            }
            Kind::BridgeChallenge => {
                let expected = PROOF_LEN + NONCE_LEN + Identity::ENCODED_LEN;
                if payload.len() != expected {
                    return Err(DecodeError::Length {
                        expected,
                        got: payload.len(),
                    });
                }
                let bridge_proof = fixed_field::<PROOF_LEN>(&payload[..PROOF_LEN])?;
                let bridge_nonce = fixed_field::<NONCE_LEN>(&payload[PROOF_LEN..PROOF_LEN + NONCE_LEN])?;
                let identity = Identity::decode(&payload[PROOF_LEN + NONCE_LEN..])?;
                Ok(Self::BridgeChallenge {
                    bridge_proof,
                    bridge_nonce,
                    identity,
                })
            }
            Kind::ClientAuth => {
                let proof = fixed_field::<PROOF_LEN>(payload)?;
                Ok(Self::ClientAuth { proof })
            }
            Kind::Accept => {
                if !payload.is_empty() {
                    return Err(DecodeError::Length {
                        expected: 0,
                        got: payload.len(),
                    });
                }
                Ok(Self::Accept)
            }
            Kind::Deny => match payload.len() {
                1 => Ok(Self::Deny {
                    reason: deny_reason(payload[0])?,
                    retry_ms: None,
                }),
                5 => Ok(Self::Deny {
                    reason: deny_reason(payload[0])?,
                    retry_ms: Some(u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]])),
                }),
                got => Err(DecodeError::Length { expected: 5, got }),
            },
        }
    }
}

/// One received preamble message, with the major its peer framed it with.
///
/// The framing is frozen across majors, so the major is returned for the policy check rather than
/// rejected here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Received {
    /// The preamble major the peer used.
    pub major: u8,
    /// The decoded message.
    pub message: Message,
}

/// Encodes `message` framed with `major` into `out`, returning the number of bytes written.
///
/// # Errors
///
/// Returns [`EncodeError::BufferTooSmall`] when `out` cannot hold the framed message.
pub fn encode_into(major: u8, message: &Message, out: &mut [u8]) -> Result<usize, EncodeError> {
    if out.len() < HEADER_LEN {
        return Err(EncodeError::BufferTooSmall);
    }
    let written = message.encode_payload_into(&mut out[HEADER_LEN..])?;
    out[..4].copy_from_slice(&MAGIC);
    out[4] = major;
    out[5] = message.kind().bits();
    out[6..8].copy_from_slice(&u16::try_from(written).unwrap_or(0).to_le_bytes());
    Ok(HEADER_LEN + written)
}

/// Decodes one complete framed message from the start of `bytes`.
///
/// Returns `Ok(None)` while `bytes` holds only part of a message, so a reader can accumulate
/// chunks. The magic is checked as soon as it is present, and a payload length beyond
/// [`MAX_PAYLOAD_LEN`] is refused, so a hostile length cannot force an unbounded read.
///
/// # Errors
///
/// Returns [`DecodeError`] when the framing or the payload does not follow the layout.
pub fn try_decode(bytes: &[u8]) -> Result<Option<Received>, DecodeError> {
    if bytes.len() < HEADER_LEN {
        if bytes.len() >= MAGIC.len() && bytes[..MAGIC.len()] != MAGIC {
            return Err(DecodeError::BadMagic);
        }
        return Ok(None);
    }
    if bytes[..MAGIC.len()] != MAGIC {
        return Err(DecodeError::BadMagic);
    }
    let len = usize::from(u16::from_le_bytes([bytes[6], bytes[7]]));
    if len > MAX_PAYLOAD_LEN {
        return Err(DecodeError::Length {
            expected: MAX_PAYLOAD_LEN,
            got: len,
        });
    }
    if bytes.len() < HEADER_LEN + len {
        return Ok(None);
    }
    decode_complete(bytes).map(Some)
}

/// Computes the Bridge's proof (ADR-0039): the label, both nonces and the served identity.
#[must_use]
pub fn bridge_proof(
    token: &[u8],
    client_nonce: &[u8; NONCE_LEN],
    bridge_nonce: &[u8; NONCE_LEN],
    identity: &Identity,
) -> [u8; PROOF_LEN] {
    let mut identity_bytes = [0u8; Identity::ENCODED_LEN];
    match identity.encode_into(&mut identity_bytes) {
        // The buffer is exactly the identity's size; this arm is unreachable by construction.
        Err(_) => [0u8; PROOF_LEN],
        Ok(_) => hmac_sha256(
            token,
            &[BRIDGE_PROOF_LABEL, client_nonce, bridge_nonce, &identity_bytes],
        ),
    }
}

/// Computes the client's proof (ADR-0039): the label and both nonces.
#[must_use]
pub fn client_proof(token: &[u8], client_nonce: &[u8; NONCE_LEN], bridge_nonce: &[u8; NONCE_LEN]) -> [u8; PROOF_LEN] {
    hmac_sha256(token, &[CLIENT_PROOF_LABEL, client_nonce, bridge_nonce])
}

/// Compares two proofs without short-circuiting on the first differing byte.
#[must_use]
pub fn proofs_match(expected: &[u8; PROOF_LEN], actual: &[u8; PROOF_LEN]) -> bool {
    expected
        .iter()
        .zip(actual.iter())
        .fold(0u8, |difference, (a, b)| difference | (a ^ b))
        == 0
}

/// HMAC-SHA256 of `parts` under `key`.
fn hmac_sha256(key: &[u8], parts: &[&[u8]]) -> [u8; PROOF_LEN] {
    // HMAC accepts every key length; the API is fallible in general.
    let Ok(mut mac) = <Hmac<Sha256> as Mac>::new_from_slice(key) else {
        return [0u8; PROOF_LEN];
    };
    for part in parts {
        mac.update(part);
    }
    let mut proof = [0u8; PROOF_LEN];
    for (out, byte) in proof.iter_mut().zip(mac.finalize().into_bytes().iter()) {
        *out = *byte;
    }
    proof
}

/// Copies a fixed-length field out of `payload`, checking its length first.
fn fixed_field<const N: usize>(payload: &[u8]) -> Result<[u8; N], DecodeError> {
    if payload.len() != N {
        return Err(DecodeError::Length {
            expected: N,
            got: payload.len(),
        });
    }
    let mut field = [0u8; N];
    field.copy_from_slice(payload);
    Ok(field)
}

/// Decodes a deny reason byte.
fn deny_reason(byte: u8) -> Result<DenyReason, DecodeError> {
    DenyReason::from_wire(byte).ok_or(DecodeError::UnknownReason(byte))
}

/// Decodes a complete message from exactly its framed bytes.
fn decode_complete(bytes: &[u8]) -> Result<Received, DecodeError> {
    if bytes.len() < HEADER_LEN {
        return Err(DecodeError::Length {
            expected: HEADER_LEN,
            got: bytes.len(),
        });
    }
    if bytes[..MAGIC.len()] != MAGIC {
        return Err(DecodeError::BadMagic);
    }
    let len = usize::from(u16::from_le_bytes([bytes[6], bytes[7]]));
    if len > MAX_PAYLOAD_LEN {
        return Err(DecodeError::Length {
            expected: MAX_PAYLOAD_LEN,
            got: len,
        });
    }
    let end = HEADER_LEN + len;
    if bytes.len() < end {
        return Err(DecodeError::Length {
            expected: end,
            got: bytes.len(),
        });
    }
    let Some(kind) = Kind::from_wire(bytes[5]) else {
        return Err(DecodeError::UnknownType(bytes[5]));
    };
    Ok(Received {
        major: bytes[4],
        message: Message::decode_payload(kind, &bytes[HEADER_LEN..end])?,
    })
}
