//! The pre-Link HMAC-SHA256 preamble exchanged before any Link byte (ADR-0037, ADR-0039).
//!
//! Every message is `magic "EXPH"`, preamble major, type, length and payload. The message types,
//! payload codecs and proof functions land with the Bridge and daemon implementations; the
//! framing constants and domain-separation labels live here now so both sides share one source.

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
