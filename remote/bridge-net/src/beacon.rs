//! The Discovery Beacon: a 3 s UDP broadcast on the Bridge's station interface (ADR-0037).
//!
//! One small datagram, broadcast every 3 s: `magic "EXPR"`, the Link Protocol major, the device
//! id, the zero-padded name, the little-endian Link port and the flag bits. A `tcp_port` of zero
//! advertises a portal-only Bridge with no Link listener, which the daemon treats as Setup Mode
//! and does not connect to. The codec is `no_std` and allocation-free; the socket stays in the
//! consumer.

use remote_protocol::EncodeError;

use crate::{
    error::DecodeError,
    identity::{DeviceId, Flags, Name},
};

/// The Beacon's magic, `EXPR`.
pub const MAGIC: [u8; 4] = *b"EXPR";

/// The Beacon interval in milliseconds.
pub const INTERVAL_MS: u64 = 3_000;

/// The Link Protocol major advertised in the Beacon, so an old daemon can filter rather than
/// misparse.
pub const LINK_MAJOR: u8 = remote_protocol::PROTOCOL_MAJOR;

/// A `tcp_port` of zero advertises a portal-only Bridge with no Link listener; the daemon treats it
/// as Setup Mode and does not connect.
pub const PORTAL_ONLY: u16 = 0;

/// Flag bit 0: the Bridge has a credential record.
pub const FLAG_PROVISIONED: u8 = 1 << 0;

/// Flag bit 1: the robot link is ready.
///
/// Pinned meaning: a valid Link Protocol frame arrived from the robot within the last
/// [`LINK_READY_WINDOW_MS`] milliseconds, so the Beacon never claims a silent robot is ready. The
/// Bridge clears the bit when that window expires; the daemon may still try to connect, but must
/// not read the flag as a liveness promise.
pub const FLAG_LINK_READY: u8 = 1 << 1;

/// How long a robot frame keeps [`FLAG_LINK_READY`] set, in milliseconds (ADR-0037's 5 s).
///
/// This is the flag's whole definition: one valid Link Protocol frame from the robot starts the
/// window, and the flag is false once the newest valid frame is older than this. It is a shared
/// constant so the Bridge's flag source and the daemon's reading of it cannot drift.
pub const LINK_READY_WINDOW_MS: u64 = 5_000;

/// Flag bit 2: the Bridge is in Setup Mode and still reachable on the station interface.
pub const FLAG_SETUP_MODE: u8 = 1 << 2;

/// The encoded datagram length.
pub const ENCODED_LEN: usize = 4 + 1 + 6 + Name::ENCODED_LEN + 2 + 1;

/// One Discovery Beacon.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Beacon {
    /// The Link Protocol major advertised in the datagram's byte 4.
    ///
    /// A daemon compares this with its own `PROTOCOL_MAJOR` and ignores a mismatch rather than
    /// misparse a future layout (ADR-0037).
    pub major: u8,
    /// The MAC-derived device id.
    pub device_id: DeviceId,
    /// The display name.
    pub name: Name,
    /// The Link Protocol port; zero is the portal-only advertisement.
    pub tcp_port: u16,
    /// The flag bits: provisioned, link-ready, setup-mode.
    pub flags: Flags,
}

impl Beacon {
    /// Encodes the datagram into `out`, returning the number of bytes written.
    ///
    /// # Errors
    ///
    /// Returns [`EncodeError::BufferTooSmall`] when `out` is shorter than [`ENCODED_LEN`].
    pub fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        out[..4].copy_from_slice(&MAGIC);
        out[4] = self.major;
        out[5..11].copy_from_slice(&self.device_id.bytes());
        out[11..11 + Name::ENCODED_LEN].copy_from_slice(&self.name.padded());
        out[27..29].copy_from_slice(&self.tcp_port.to_le_bytes());
        out[29] = self.flags.bits();
        Ok(ENCODED_LEN)
    }

    /// Decodes a datagram, ignoring trailing bytes so a later minor can extend it.
    ///
    /// # Errors
    ///
    /// Returns [`DecodeError`] when the datagram is short, the magic is wrong or the name is not
    /// UTF-8.
    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() >= MAGIC.len() && bytes[..MAGIC.len()] != MAGIC {
            return Err(DecodeError::BadMagic);
        }
        if bytes.len() < ENCODED_LEN {
            return Err(DecodeError::Length {
                expected: ENCODED_LEN,
                got: bytes.len(),
            });
        }
        let mut device_id = [0u8; 6];
        device_id.copy_from_slice(&bytes[5..11]);
        let mut name = [0u8; Name::ENCODED_LEN];
        name.copy_from_slice(&bytes[11..27]);
        Ok(Self {
            major: bytes[4],
            device_id: DeviceId::new(device_id),
            name: Name::from_wire(name)?,
            tcp_port: u16::from_le_bytes([bytes[27], bytes[28]]),
            flags: Flags::from_bits(bytes[29]),
        })
    }
}
