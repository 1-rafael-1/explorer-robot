//! The `BRIDGE_STATUS` payload: the Bridge's link and resilience snapshot (ADR-0031).
//!
//! The Bridge publishes it at 1 Hz to the robot as an unacknowledged `control` frame and to the
//! daemon as a latest-wins `stream` frame; both carry the same 28-byte little-endian layout.
//!
//! | offset | size | field | encoding |
//! |---|---|---|---|
//! | 0 | 1 | `rssi_dbm` | `i8`; [`RSSI_UNKNOWN_DBM`] is unknown |
//! | 1 | 1 | `channel` | `u8`; [`CHANNEL_UNKNOWN`] is unknown |
//! | 2 | 1 | `station` | [`BridgeStation`] |
//! | 3 | 1 | `flags` | [`BridgeStatusFlags`]; bits 3–7 reserved |
//! | 4 | 4 | `uart_dropped_bytes` | `u32` little-endian |
//! | 8 | 4 | `dma_stalls` | `u32` little-endian |
//! | 12 | 4 | `crc_failures` | `u32` little-endian |
//! | 16 | 4 | `resyncs` | `u32` little-endian |
//! | 20 | 4 | `wdt_feeds` | `u32` little-endian |
//! | 24 | 4 | `wdt_withheld` | `u32` little-endian |
//!
//! The sentinels are ordinary wire values and survive a round trip; `None` maps to a sentinel on
//! encode and back to `None` on decode.

use crate::{
    codec::{PayloadCodec, read_u32_le, write_u32_le},
    error::{CodecError, EncodeError},
};

/// The `rssi_dbm` sentinel meaning "unknown".
pub const RSSI_UNKNOWN_DBM: i8 = i8::MIN;

/// The `channel` sentinel meaning "unknown".
pub const CHANNEL_UNKNOWN: u8 = 0;

/// The `station` byte: the Bridge's Wi-Fi association state.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BridgeStation {
    /// The station interface is down.
    Down = 0,
    /// The station is connecting.
    Connecting = 1,
    /// The station is associated.
    Associated = 2,
}

impl BridgeStation {
    /// Every station state, in wire order.
    pub const ALL: [Self; 3] = [Self::Down, Self::Connecting, Self::Associated];

    /// Returns the wire value.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self as u8
    }

    /// Builds a station state from its wire value, returning [`None`] for an unassigned value.
    #[must_use]
    pub const fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::Down),
            1 => Some(Self::Connecting),
            2 => Some(Self::Associated),
            _ => None,
        }
    }
}

/// The flags byte of a `BRIDGE_STATUS` payload.
///
/// Bit 0 is `PROVISIONED`, bit 1 is `SETUP_OPEN` and bit 2 is `LINK_READY`; bits 3–7 are
/// reserved. They are cleared by [`BridgeStatusFlags::from_bits`], so this version always sends
/// them as zero and ignores them on receive.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct BridgeStatusFlags(u8);

impl BridgeStatusFlags {
    /// The Bridge is provisioned.
    pub const PROVISIONED: Self = Self(0x01);
    /// The Bridge's setup mode is open.
    pub const SETUP_OPEN: Self = Self(0x02);
    /// The Bridge's link is ready.
    pub const LINK_READY: Self = Self(0x04);

    /// The reserved bits, cleared by [`BridgeStatusFlags::from_bits`].
    const RESERVED_BITS: u8 = 0xF8;

    /// Builds flags from a raw byte, clearing the reserved bits.
    #[must_use]
    pub const fn from_bits(bits: u8) -> Self {
        Self(bits & !Self::RESERVED_BITS)
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
}

impl core::ops::BitOr for BridgeStatusFlags {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitOrAssign for BridgeStatusFlags {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// A `BRIDGE_STATUS` snapshot: the Bridge's link and resilience counters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BridgeStatus {
    /// The station RSSI in dBm, or [`None`] for unknown.
    pub rssi_dbm: Option<i8>,
    /// The station channel, or [`None`] for unknown.
    pub channel: Option<u8>,
    /// The station association state.
    pub station: BridgeStation,
    /// The status flags.
    pub flags: BridgeStatusFlags,
    /// Bytes dropped by the Bridge's UART receive path.
    pub uart_dropped_bytes: u32,
    /// DMA stalls observed by the Bridge.
    pub dma_stalls: u32,
    /// Frame CRC failures observed by the Bridge.
    pub crc_failures: u32,
    /// Resynchronisations after a framing error.
    pub resyncs: u32,
    /// Watchdog feeds.
    pub wdt_feeds: u32,
    /// Watchdog feeds withheld because a task was late.
    pub wdt_withheld: u32,
}

impl BridgeStatus {
    /// The encoded payload length in bytes.
    pub const ENCODED_LEN: usize = 28;
}

impl<'a> PayloadCodec<'a> for BridgeStatus {
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CodecError::Length);
        }
        let rssi_dbm = match i8::from_le_bytes([bytes[0]]) {
            RSSI_UNKNOWN_DBM => None,
            value => Some(value),
        };
        let channel = match bytes[1] {
            CHANNEL_UNKNOWN => None,
            value => Some(value),
        };
        let station = BridgeStation::from_wire(bytes[2]).ok_or(CodecError::Value)?;
        Ok(Self {
            rssi_dbm,
            channel,
            station,
            flags: BridgeStatusFlags::from_bits(bytes[3]),
            uart_dropped_bytes: read_u32_le(bytes, 4),
            dma_stalls: read_u32_le(bytes, 8),
            crc_failures: read_u32_le(bytes, 12),
            resyncs: read_u32_le(bytes, 16),
            wdt_feeds: read_u32_le(bytes, 20),
            wdt_withheld: read_u32_le(bytes, 24),
        })
    }

    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0] = self.rssi_dbm.unwrap_or(RSSI_UNKNOWN_DBM).to_le_bytes()[0];
        out[1] = self.channel.unwrap_or(CHANNEL_UNKNOWN);
        out[2] = self.station.bits();
        out[3] = self.flags.bits();
        write_u32_le(out, 4, self.uart_dropped_bytes);
        write_u32_le(out, 8, self.dma_stalls);
        write_u32_le(out, 12, self.crc_failures);
        write_u32_le(out, 16, self.resyncs);
        write_u32_le(out, 20, self.wdt_feeds);
        write_u32_le(out, 24, self.wdt_withheld);
        Ok(Self::ENCODED_LEN)
    }
}
