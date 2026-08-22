//! Pure byte-stream decoder for the COIN-D6 `LiDAR`'s UART protocol.
//!
//! [`Decoder`] turns a raw byte stream into [`Point`]s, framing packets on the
//! `AA 55` header and validating each packet's XOR checksum before decoding its
//! samples. Decoded points are written into the caller's [`Scan`] only when a
//! complete revolution has been assembled.
//!
//! # Checksum
//!
//! The checksum is a 16-bit XOR, matching the vendor reference. It accumulates
//! the header value `0x55AA`, the `CT | LSN<<8` word, `FSA`, `LSA`, and — for
//! each three-byte sample — `Si_L` plus `(Si_H << 8) | Si_2nd`. The two `CS`
//! bytes are excluded from the accumulation and compared as a little-endian
//! `u16`.

use crate::types::{Point, Scan};

/// Outcome of feeding raw bytes to the decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decode {
    /// A complete revolution was assembled and written into the output `Scan`.
    Revolution,
    /// A checksum mismatch discarded the in-progress revolution; the decoder
    /// has resynchronised and is waiting for the next ring-start.
    Resync,
    /// No complete revolution yet.
    InProgress,
}

/// First byte of the two-byte packet header.
const HEADER_0: u8 = 0xAA;
/// Second byte of the two-byte packet header.
const HEADER_1: u8 = 0x55;
/// Longest possible packet: a 10-byte preamble plus three bytes for each of the
/// at most `u8::MAX` samples.
const MAX_PACKET_LEN: usize = 10 + 3 * 255;
/// Maximum number of points in a single revolution (native 0.9° resolution).
const MAX_POINTS: usize = 400;

/// A stateful COIN-D6 byte-stream decoder.
pub struct Decoder {
    /// Carry buffer holding the bytes of the packet currently being assembled.
    /// An empty buffer (`carry_len == 0`) means the decoder is scanning for a
    /// header.
    carry: [u8; MAX_PACKET_LEN],
    /// Number of valid bytes in `carry`.
    carry_len: usize,
    /// Whether the most recently scanned byte was a `0xAA` that may begin a
    /// header.
    seen_header_0: bool,
    /// Points accumulated for the revolution currently being assembled.
    points: [Point; MAX_POINTS],
    /// Number of valid points in `points`.
    points_len: usize,
}

impl Decoder {
    /// Create an empty decoder.
    #[must_use]
    pub fn new() -> Self {
        Self {
            carry: [0; MAX_PACKET_LEN],
            carry_len: 0,
            seen_header_0: false,
            points: [Point::default(); MAX_POINTS],
            points_len: 0,
        }
    }

    /// Feed raw bytes. Decoded points are written into `scan` (setting
    /// `scan.len`) only when a revolution completes; returns the most
    /// significant event.
    ///
    /// Callers should feed at most one revolution's worth of bytes per call (a
    /// UART ingest chunk is smaller than a full revolution). If a call happens
    /// to contain more than one complete revolution, only the last one is
    /// retained in `scan`.
    pub fn push(&mut self, bytes: &[u8], scan: &mut Scan) -> Decode {
        let mut result = Decode::InProgress;
        for &byte in bytes {
            match self.feed_byte(byte, scan) {
                Some(Decode::Revolution) => result = Decode::Revolution,
                Some(Decode::Resync) => {
                    if result == Decode::InProgress {
                        result = Decode::Resync;
                    }
                }
                Some(Decode::InProgress) | None => {}
            }
        }
        result
    }

    /// Advance the decoder by one input byte, returning an event when a packet
    /// completes (a checksum mismatch → [`Decode::Resync`], or a revolution
    /// boundary → [`Decode::Revolution`]).
    fn feed_byte(&mut self, byte: u8, scan: &mut Scan) -> Option<Decode> {
        if self.carry_len == 0 {
            match byte {
                HEADER_0 => self.seen_header_0 = true,
                HEADER_1 if self.seen_header_0 => {
                    self.carry[0] = HEADER_0;
                    self.carry[1] = HEADER_1;
                    self.carry_len = 2;
                    self.seen_header_0 = false;
                }
                HEADER_1 => {}
                _ => self.seen_header_0 = false,
            }
            return None;
        }

        self.carry[self.carry_len] = byte;
        self.carry_len += 1;

        // The 10-byte preamble must arrive before the packet length is known.
        if self.carry_len < 10 {
            return None;
        }

        let lsn = usize::from(self.carry[3]);
        let total_len = 10 + 3 * lsn;
        if self.carry_len < total_len {
            return None;
        }

        let event = self.finish_packet(scan);
        self.carry_len = 0;
        self.seen_header_0 = false;
        event
    }

    /// Validate and decode a fully-assembled packet stored in `carry`, returning
    /// an event if it is corrupt or closes a revolution.
    fn finish_packet(&mut self, scan: &mut Scan) -> Option<Decode> {
        // 16-bit XOR checksum, matching the vendor reference. It accumulates
        // the 16-bit header value 0x55AA, the `CT | LSN<<8` word, `FSA`, `LSA`,
        // and — for each sample — `Si_L` plus `(Si_H << 8) | Si_2nd`. The two
        // `CS` bytes at offsets 8 and 9 are excluded from the accumulation.
        let lsn = u16::from(self.carry[3]);
        let mut checksum: u16 = 0x55AA;
        checksum ^= u16::from(self.carry[2]) | (u16::from(self.carry[3]) << 8);
        checksum ^= u16::from_le_bytes([self.carry[4], self.carry[5]]);
        checksum ^= u16::from_le_bytes([self.carry[6], self.carry[7]]);
        for i in 0..usize::from(lsn) {
            let base = 10 + 3 * i;
            let si_l = self.carry[base];
            let si_2nd = self.carry[base + 1];
            let si_h = self.carry[base + 2];
            checksum ^= u16::from(si_l);
            checksum ^= (u16::from(si_h) << 8) | u16::from(si_2nd);
        }
        let cs = u16::from_le_bytes([self.carry[8], self.carry[9]]);
        if checksum != cs {
            // Corrupt packet: discard it and the in-progress revolution.
            self.points_len = 0;
            return Some(Decode::Resync);
        }

        let is_ring_start = (self.carry[2] & 0b1) == 1;
        let fsa = u16::from_le_bytes([self.carry[4], self.carry[5]]);
        let lsa = u16::from_le_bytes([self.carry[6], self.carry[7]]);

        let fsa_deg = f32::from(fsa >> 1) / 64.0;
        let lsa_deg = f32::from(lsa >> 1) / 64.0;
        // A packet that crosses 0° has an end angle smaller than its start.
        let lsa_adj = if lsa_deg < fsa_deg { lsa_deg + 360.0 } else { lsa_deg };
        let step = if lsn > 1 {
            (lsa_adj - fsa_deg) / f32::from(lsn - 1)
        } else {
            0.0
        };

        let event = if is_ring_start && self.points_len > 0 {
            // A ring-start delimits the previous revolution: flush it.
            let n = self.points_len;
            scan.points[..n].copy_from_slice(&self.points[..n]);
            scan.len = n;
            self.points_len = 0;
            Some(Decode::Revolution)
        } else {
            None
        };

        for i in 0..lsn {
            let base = 10 + 3 * usize::from(i);
            // Sample byte order on the wire is `Si_L`, `Si_2nd`, `Si_H`.
            let si_l = self.carry[base];
            let si_2nd = self.carry[base + 1];
            let si_h = self.carry[base + 2];

            let distance_mm = u16::from(si_h) * 64 + u16::from(si_2nd >> 2);
            let intensity = (si_2nd & 0b11) * 64 + (si_l >> 2);
            // The interpolated angle is always non-negative, so the `%`
            // operator matches `rem_euclid(360.0)` (which is not available in
            // `core`).
            let angle = (fsa_deg + f32::from(i) * step) % 360.0;

            if self.points_len < MAX_POINTS {
                self.points[self.points_len] = Point {
                    angle_deg: angle,
                    distance_mm,
                    intensity,
                };
                self.points_len += 1;
            }
        }

        event
    }
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}
