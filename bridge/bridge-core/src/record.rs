//! The versioned, CRC-checked credential record (ADR-0036).
//!
//! One record lives on the Bridge's dedicated data partition: the setup identity's station
//! credentials, the pairing token and a small flag word. The layout is fixed here so the
//! HAL-free codec is host-testable; the firmware only supplies the flash and the
//! `sequential-storage` [`Value`](https://docs.rs/sequential-storage) glue.
//!
//! Every failure means **unprovisioned** rather than a best-effort read: a missing record, a
//! wrong magic, a wrong version, a bad CRC, a malformed flag or credential field, or a token
//! outside the pairing alphabet. The caller fails closed into Setup Mode, so a changed layout is
//! a recoverable re-provision instead of a misread.
//!
//! The CRC is the Link Protocol's CRC-16/CCITT-FALSE ([`link_protocol::crc16`]), reused rather
//! than inventing a second checksum for the same board.

use link_protocol::crc16;

use crate::{
    pairing::{self, CODE_LEN},
    portal::{Credentials, MAX_PASSPHRASE_BYTES, MAX_SSID_BYTES},
};

/// The record's magic, `BRID`.
pub const MAGIC: [u8; 4] = *b"BRID";

/// The record layout version. Bump it whenever a field moves or changes meaning.
pub const VERSION: u8 = 1;

/// The record's flag bit: the station credential fields are present.
const FLAG_HAS_CREDENTIALS: u8 = 0x01;

/// The number of flag bits this version defines; every other bit is reserved and must be zero.
const DEFINED_FLAGS: u8 = FLAG_HAS_CREDENTIALS;

/// The CRC-16's encoded length.
const CRC_LEN: usize = 2;

/// The fixed overhead around the variable-length fields: magic, version, flags, the two length
/// bytes, the token and the CRC.
const FIXED_LEN: usize = MAGIC.len() + 1 + 1 + 1 + 1 + CODE_LEN + CRC_LEN;

/// The longest encoded record: both credential fields full.
pub const MAX_ENCODED_LEN: usize = FIXED_LEN + MAX_SSID_BYTES + MAX_PASSPHRASE_BYTES;

/// The shortest encoded record: no credentials, only the token slot.
pub const MIN_ENCODED_LEN: usize = FIXED_LEN;

/// Why a record could not be encoded or decoded.
///
/// Every variant makes the Bridge unprovisioned; the variants exist so the boot log can say
/// *why* the record was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecordError {
    /// The byte length is shorter than [`MIN_ENCODED_LEN`], longer than [`MAX_ENCODED_LEN`], or
    /// the length bytes do not match the slice.
    Length,
    /// The magic is not [`MAGIC`].
    Magic,
    /// The version is not [`VERSION`].
    Version,
    /// A reserved flag bit is set, or the flag does not match the credential fields.
    Flags,
    /// The CRC does not match the bytes it covers.
    Crc,
    /// The token is not twelve characters of [`pairing::ALPHABET`].
    Token,
    /// The `SSID` or passphrase length is outside the WPA2-PSK range, or only one of the two is
    /// present.
    Credentials,
}

impl RecordError {
    /// Returns a short, stable label for the boot log.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Length => "bad length",
            Self::Magic => "wrong magic",
            Self::Version => "wrong version",
            Self::Flags => "bad flags",
            Self::Crc => "bad crc",
            Self::Token => "bad pairing token",
            Self::Credentials => "bad credentials",
        }
    }
}

/// The credential record's contents.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CredentialRecord {
    /// The station credentials, or `None` for a token-only record (unprovisioned).
    pub credentials: Option<Credentials>,
    /// The twelve base32 characters that double as the HMAC key (ADR-0037).
    pub token: [u8; CODE_LEN],
}

impl CredentialRecord {
    /// Creates a token-only record: the token is present, the network is not.
    #[must_use]
    pub const fn token_only(token: [u8; CODE_LEN]) -> Self {
        Self {
            credentials: None,
            token,
        }
    }

    /// Returns whether a usable station credential pair is stored.
    #[must_use]
    pub const fn provisioned(&self) -> bool {
        self.credentials.is_some()
    }

    /// Borrows the token as a `str`.
    ///
    /// [`Self::decode`] and [`Self::encode_into`] only accept alphabet characters, so the bytes
    /// are always valid ASCII.
    #[must_use]
    pub fn token_str(&self) -> &str {
        core::str::from_utf8(&self.token).unwrap_or("")
    }

    /// Encodes the record into `out`, returning the number of bytes written.
    ///
    /// The encoding is self-delimiting: `out` may be longer than the record.
    ///
    /// # Errors
    ///
    /// [`RecordError::Length`] when `out` cannot hold [`MAX_ENCODED_LEN`],
    /// [`RecordError::Token`] when the token is not twelve alphabet characters, and
    /// [`RecordError::Credentials`] when a credential field is outside the WPA2-PSK range.
    #[allow(
        clippy::large_stack_frames,
        reason = "the record is a fixed-size value (at most 117 bytes); the MIR estimate counts every temporary copy"
    )]
    pub fn encode_into(&self, out: &mut [u8]) -> Result<usize, RecordError> {
        if out.len() < MAX_ENCODED_LEN {
            return Err(RecordError::Length);
        }
        if !pairing::is_valid(&self.token) {
            return Err(RecordError::Token);
        }

        let (ssid, passphrase, flags) = match &self.credentials {
            Some(credentials) => {
                if !credentials.is_valid() {
                    return Err(RecordError::Credentials);
                }
                (credentials.ssid(), credentials.passphrase(), FLAG_HAS_CREDENTIALS)
            }
            None => (&[][..], &[][..], 0),
        };

        let mut cursor = 0;
        out[cursor..cursor + MAGIC.len()].copy_from_slice(&MAGIC);
        cursor += MAGIC.len();
        out[cursor] = VERSION;
        cursor += 1;
        out[cursor] = flags;
        cursor += 1;
        out[cursor] = length_byte(ssid)?;
        cursor += 1;
        out[cursor..cursor + ssid.len()].copy_from_slice(ssid);
        cursor += ssid.len();
        out[cursor] = length_byte(passphrase)?;
        cursor += 1;
        out[cursor..cursor + passphrase.len()].copy_from_slice(passphrase);
        cursor += passphrase.len();
        out[cursor..cursor + CODE_LEN].copy_from_slice(&self.token);
        cursor += CODE_LEN;

        let crc = crc16(&out[..cursor]);
        out[cursor..cursor + CRC_LEN].copy_from_slice(&crc.to_le_bytes());
        cursor += CRC_LEN;
        Ok(cursor)
    }

    /// Decodes a record, rejecting every field and check that is not exactly as expected.
    ///
    /// # Errors
    ///
    /// Returns the first [`RecordError`] that applies. The caller treats every error as
    /// unprovisioned.
    #[allow(
        clippy::large_stack_frames,
        reason = "the record is a fixed-size value (at most 117 bytes); the MIR estimate counts every temporary copy"
    )]
    pub fn decode(bytes: &[u8]) -> Result<Self, RecordError> {
        if bytes.len() < MIN_ENCODED_LEN || bytes.len() > MAX_ENCODED_LEN {
            return Err(RecordError::Length);
        }
        if bytes[..MAGIC.len()] != MAGIC {
            return Err(RecordError::Magic);
        }
        let mut cursor = MAGIC.len();

        if bytes[cursor] != VERSION {
            return Err(RecordError::Version);
        }
        cursor += 1;

        let flags = bytes[cursor];
        cursor += 1;
        if flags & !DEFINED_FLAGS != 0 {
            return Err(RecordError::Flags);
        }

        let ssid_len = usize::from(bytes[cursor]);
        cursor += 1;
        if ssid_len > MAX_SSID_BYTES || cursor + ssid_len >= bytes.len() {
            return Err(RecordError::Length);
        }
        let ssid = &bytes[cursor..cursor + ssid_len];
        cursor += ssid_len;

        let pass_len = usize::from(bytes[cursor]);
        cursor += 1;
        if pass_len > MAX_PASSPHRASE_BYTES || cursor + pass_len >= bytes.len() {
            return Err(RecordError::Length);
        }
        let passphrase = &bytes[cursor..cursor + pass_len];
        cursor += pass_len;

        if cursor + CODE_LEN + CRC_LEN != bytes.len() {
            return Err(RecordError::Length);
        }
        let mut token = [0u8; CODE_LEN];
        token.copy_from_slice(&bytes[cursor..cursor + CODE_LEN]);
        cursor += CODE_LEN;

        let expected = crc16(&bytes[..cursor]);
        let stored = u16::from_le_bytes([bytes[cursor], bytes[cursor + 1]]);
        if expected != stored {
            return Err(RecordError::Crc);
        }

        if flags & FLAG_HAS_CREDENTIALS != 0 {
            if ssid.is_empty() {
                return Err(RecordError::Flags);
            }
        } else if !ssid.is_empty() || !passphrase.is_empty() {
            return Err(RecordError::Flags);
        }

        if !pairing::is_valid(&token) {
            return Err(RecordError::Token);
        }

        let credentials = if ssid.is_empty() {
            None
        } else {
            Some(Credentials::from_slices(ssid, passphrase).ok_or(RecordError::Credentials)?)
        };

        Ok(Self { credentials, token })
    }
}

/// Converts a field length to the record's single length byte.
fn length_byte(field: &[u8]) -> Result<u8, RecordError> {
    u8::try_from(field.len()).map_err(|_| RecordError::Credentials)
}
