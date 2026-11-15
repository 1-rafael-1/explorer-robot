//! The Bridge identity served in the challenge and the Beacon (ADR-0037, ADR-0039, ADR-0050).
//!
//! The device id, name and flag bits are the same fields in both places: the challenge's identity
//! lets a manual `host:port` connection learn what the Beacon would have advertised, and the flag
//! bits are this crate's Beacon definitions, so both sides share one source.
//!
//! [`DeviceName`] derives the operator-facing name from the device id once, so the DHCP hostname,
//! the setup AP's SSID and the Beacon's short label share one suffix.
//!
//! Everything here is `no_std` and allocation-free: the fields are fixed-size arrays and the
//! identity is a plain `Copy` value.

use link_protocol::EncodeError;

use crate::{beacon, error::DecodeError};

/// The MAC-derived device id, stable across IP changes (ADR-0037).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct DeviceId([u8; 6]);

impl DeviceId {
    /// Wraps six raw bytes.
    #[must_use]
    pub const fn new(bytes: [u8; 6]) -> Self {
        Self(bytes)
    }

    /// Returns the raw bytes.
    #[must_use]
    pub const fn bytes(self) -> [u8; 6] {
        self.0
    }
}

/// The display name: at most sixteen UTF-8 bytes, zero-padded on the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Name([u8; Self::ENCODED_LEN]);

impl Name {
    /// The encoded field length in bytes, padding included.
    pub const ENCODED_LEN: usize = 16;

    /// Builds a name from UTF-8 text, returning [`None`] when it does not fit or embeds a zero.
    #[must_use]
    pub fn new(text: &str) -> Option<Self> {
        if text.len() > Self::ENCODED_LEN || text.as_bytes().contains(&0) {
            return None;
        }
        let mut bytes = [0u8; Self::ENCODED_LEN];
        bytes[..text.len()].copy_from_slice(text.as_bytes());
        Some(Self(bytes))
    }

    /// Wraps already-padded wire bytes without validation.
    #[must_use]
    pub const fn from_padded(bytes: [u8; Self::ENCODED_LEN]) -> Self {
        Self(bytes)
    }

    /// Decodes a name from its zero-padded wire bytes, trimming at the first zero.
    ///
    /// # Errors
    ///
    /// Returns [`DecodeError::NameUtf8`] when the text before the padding is not UTF-8.
    pub fn from_wire(bytes: [u8; Self::ENCODED_LEN]) -> Result<Self, DecodeError> {
        let name = Self::from_padded(bytes);
        let end = name.0.iter().position(|byte| *byte == 0).unwrap_or(Self::ENCODED_LEN);
        if core::str::from_utf8(&name.0[..end]).is_err() {
            return Err(DecodeError::NameUtf8);
        }
        Ok(name)
    }

    /// Returns the name as text, without the wire padding.
    #[must_use]
    pub fn as_str(&self) -> &str {
        let end = self.0.iter().position(|byte| *byte == 0).unwrap_or(Self::ENCODED_LEN);
        core::str::from_utf8(&self.0[..end]).unwrap_or("")
    }

    /// Returns the zero-padded wire bytes.
    #[must_use]
    pub const fn padded(&self) -> [u8; Self::ENCODED_LEN] {
        self.0
    }
}

/// The three defined flag bits, served in the Beacon and the challenge (ADR-0037).
///
/// The bits are this crate's Beacon definitions; reserved bits are cleared on decode and never
/// sent, so a peer that adds a meaning later is ignored rather than misread by this version.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Flags(u8);

impl Flags {
    /// Bit 0: the Bridge has a credential record.
    pub const PROVISIONED: Self = Self(beacon::FLAG_PROVISIONED);
    /// Bit 1: the robot link is ready.
    pub const LINK_READY: Self = Self(beacon::FLAG_LINK_READY);
    /// Bit 2: the Bridge is in Setup Mode and still reachable on the station interface.
    pub const SETUP_MODE: Self = Self(beacon::FLAG_SETUP_MODE);

    /// Builds flags from a raw byte, keeping only the three defined bits.
    #[must_use]
    pub const fn from_bits(bits: u8) -> Self {
        Self(bits & (beacon::FLAG_PROVISIONED | beacon::FLAG_LINK_READY | beacon::FLAG_SETUP_MODE))
    }

    /// Returns the wire bits; reserved bits are always zero.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Returns `true` when every bit in `other` is set.
    #[must_use]
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Returns `true` when no bit is set.
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

/// The operator-facing Device Name (ADR-0050): `ExplorerRobot-XXXX`, with `XXXX` the last two
/// bytes of the station MAC as upper-case hex.
///
/// The full eighteen bytes are both the DHCP hostname (option 12) and the setup AP's SSID. The
/// Discovery Beacon's name field is [`Name::ENCODED_LEN`] bytes and cannot hold them, so the Beacon
/// carries [`DeviceName::beacon_label`]'s short form `Explorer-XXXX`; this one derivation produces
/// all three, so their suffix cannot drift.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct DeviceName([u8; Self::ENCODED_LEN]);

impl DeviceName {
    /// The full name's length in bytes: the prefix plus four hex digits.
    pub const ENCODED_LEN: usize = 18;

    /// The full name's prefix, shared by the DHCP hostname and the setup AP's SSID.
    pub const PREFIX: &str = "ExplorerRobot-";

    /// The Beacon label's shorter prefix, so the label fits [`Name::ENCODED_LEN`].
    pub const BEACON_PREFIX: &str = "Explorer-";

    /// The shared suffix length in upper-case hex digits.
    pub const SUFFIX_LEN: usize = 4;

    /// Derives the name from the device id's last two bytes.
    #[must_use]
    pub fn from_device_id(device_id: DeviceId) -> Self {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        let bytes = device_id.bytes();
        let mut name = [0u8; Self::ENCODED_LEN];
        name[..Self::PREFIX.len()].copy_from_slice(Self::PREFIX.as_bytes());
        let mut index = Self::PREFIX.len();
        for byte in [bytes[4], bytes[5]] {
            name[index] = HEX[usize::from(byte >> 4)];
            name[index + 1] = HEX[usize::from(byte & 0x0F)];
            index += 2;
        }
        Self(name)
    }

    /// Returns the full name's bytes: the DHCP hostname and the setup AP's SSID.
    #[must_use]
    pub const fn bytes(&self) -> &[u8; Self::ENCODED_LEN] {
        &self.0
    }

    /// Returns the full name as text, without padding; it is always ASCII.
    #[must_use]
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.0).unwrap_or("")
    }

    /// Returns the four upper-case hex characters the three representations share.
    #[must_use]
    pub fn suffix(&self) -> &str {
        core::str::from_utf8(&self.0[Self::PREFIX.len()..]).unwrap_or("")
    }

    /// Returns the Discovery Beacon's short label, zero-padded to the Beacon's name field.
    #[must_use]
    pub fn beacon_label(&self) -> Name {
        let mut label = [0u8; Name::ENCODED_LEN];
        let suffix_start = Self::BEACON_PREFIX.len();
        label[..suffix_start].copy_from_slice(Self::BEACON_PREFIX.as_bytes());
        label[suffix_start..suffix_start + Self::SUFFIX_LEN].copy_from_slice(self.suffix().as_bytes());
        Name::from_padded(label)
    }
}

/// The identity the Bridge serves everywhere: device id, name and flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Identity {
    /// The MAC-derived device id.
    pub device_id: DeviceId,
    /// The display name.
    pub name: Name,
    /// The flag bits.
    pub flags: Flags,
}

impl Identity {
    /// The encoded length in the challenge: device id, flags, then the zero-padded name.
    pub const ENCODED_LEN: usize = 6 + 1 + Name::ENCODED_LEN;

    /// Encodes the identity into `out`, in the challenge's field order.
    ///
    /// # Errors
    ///
    /// Returns [`EncodeError::BufferTooSmall`] when `out` is shorter than
    /// [`Identity::ENCODED_LEN`].
    pub fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        out[..6].copy_from_slice(&self.device_id.bytes());
        out[6] = self.flags.bits();
        out[7..Self::ENCODED_LEN].copy_from_slice(&self.name.padded());
        Ok(Self::ENCODED_LEN)
    }

    /// Decodes the identity from its challenge encoding.
    ///
    /// # Errors
    ///
    /// Returns [`DecodeError::Length`] when the slice is short and [`DecodeError::NameUtf8`]
    /// when the name's text is not UTF-8.
    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() < Self::ENCODED_LEN {
            return Err(DecodeError::Length {
                expected: Self::ENCODED_LEN,
                got: bytes.len(),
            });
        }
        let mut device_id = [0u8; 6];
        device_id.copy_from_slice(&bytes[..6]);
        let mut name = [0u8; Name::ENCODED_LEN];
        name.copy_from_slice(&bytes[7..Self::ENCODED_LEN]);
        Ok(Self {
            device_id: DeviceId::new(device_id),
            name: Name::from_wire(name)?,
            flags: Flags::from_bits(bytes[6]),
        })
    }
}
