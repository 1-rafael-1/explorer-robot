//! CRC-16/CCITT-FALSE (ADR-0030).
//!
//! Polynomial `0x1021`, initial value `0xFFFF`, no reflection and no final XOR. The check value
//! for `"123456789"` is `0x29B1`. The calculator is incremental so a large payload can be
//! checksummed without copying it.

/// The CRC-16/CCITT-FALSE polynomial.
const POLYNOMIAL: u16 = 0x1021;

/// The CRC-16/CCITT-FALSE initial value.
const INITIAL: u16 = 0xFFFF;

/// An incremental CRC-16/CCITT-FALSE calculator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Crc16 {
    /// The running remainder.
    state: u16,
}

impl Crc16 {
    /// Creates a calculator with the initial value `0xFFFF`.
    #[must_use]
    pub const fn new() -> Self {
        Self { state: INITIAL }
    }

    /// Feeds one byte into the calculator.
    pub fn update_byte(&mut self, byte: u8) {
        self.state ^= u16::from(byte) << 8;
        for _ in 0..8 {
            self.state = if self.state & 0x8000 == 0 {
                self.state << 1
            } else {
                (self.state << 1) ^ POLYNOMIAL
            };
        }
    }

    /// Feeds a byte slice into the calculator.
    pub fn update(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.update_byte(byte);
        }
    }

    /// Returns the CRC of everything fed so far.
    #[must_use]
    pub const fn finish(self) -> u16 {
        self.state
    }
}

impl Default for Crc16 {
    fn default() -> Self {
        Self::new()
    }
}

/// Computes the CRC-16/CCITT-FALSE of a whole slice in one pass.
#[must_use]
pub fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = Crc16::new();
    crc.update(bytes);
    crc.finish()
}
