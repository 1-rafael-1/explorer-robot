//! Decoded VL53L1X result block.
//!
//! The 17-byte block at `RESULT__RANGE_STATUS` is the sensor's whole answer:
//! the 8-bit status, the stream count, the effective SPAD count, the ambient and
//! signal count rates, and the crosstalk-corrected range. This module owns the
//! field offsets, the one-time gain correction and the DSS target calculation, so
//! the async driver only moves the bytes.
//!
//! The layout and gain correction are ported from `vl53l1x-simple` 0.2.0
//! (MIT OR Apache-2.0); see `THIRD_PARTY_LICENSES.md`.

use crate::types::{RangeDiagnostics, RangeStatus, RangefinderReading, Reading};

/// Length of the result block at `RESULT__RANGE_STATUS`, in bytes.
pub const RESULT_BLOCK_LEN: usize = 17;

/// The low-power auto-mode target total rate, in Mcps fixed point.
pub const TARGET_RATE: u16 = 0x0A00;

/// The gain factor from ST's tuning defaults.
const GAIN_FACTOR: u32 = 2011;

/// The gain correction's rounding constant, 1024 in 8.11 fixed point.
const GAIN_ROUNDING: u32 = 0x0400;

/// The gain correction's divisor, 2048 in 8.11 fixed point.
const GAIN_DIVISOR: u32 = 0x0800;

/// The raw 17-byte result block read from `RESULT__RANGE_STATUS`.
pub struct ResultBlock {
    /// The raw 8-bit range status.
    status: u8,
    /// The number of completed stream samples.
    stream_count: u8,
    /// The actual effective SPAD count.
    effective_spads: u16,
    /// The ambient count rate in 7.7 fixed point.
    ambient_rate: u16,
    /// The crosstalk-corrected range, in millimetres.
    range_mm: u16,
    /// The crosstalk-corrected peak signal count rate in 7.7 fixed point.
    signal_rate: u16,
}

impl ResultBlock {
    /// Decode the 17-byte block.
    pub const fn parse(block: &[u8; RESULT_BLOCK_LEN]) -> Self {
        Self {
            status: block[0],
            stream_count: block[2],
            effective_spads: u16::from_be_bytes([block[3], block[4]]),
            ambient_rate: u16::from_be_bytes([block[7], block[8]]),
            range_mm: u16::from_be_bytes([block[13], block[14]]),
            signal_rate: u16::from_be_bytes([block[15], block[16]]),
        }
    }

    /// Decode the status, gain-corrected range and diagnostics.
    pub const fn reading(&self) -> RangefinderReading {
        let status = RangeStatus::from_raw(self.status, self.stream_count);
        RangefinderReading {
            reading: Reading::from_status(status, self.gain_corrected_range()),
            status,
            raw_status: self.status,
            diagnostics: RangeDiagnostics {
                signal_rate: self.signal_rate,
                ambient_rate: self.ambient_rate,
                effective_spads: self.effective_spads,
            },
        }
    }

    /// Calculate the DSS effective-SPAD target, or `None` when it would divide
    /// by zero.
    pub fn required_spads(&self) -> Option<u16> {
        let total_rate = u32::from(self.signal_rate) + u32::from(self.ambient_rate);
        let total_rate_per_spad = (total_rate.min(0xFFFF) << 16).checked_div(u32::from(self.effective_spads))?;
        let required_spads = (u32::from(TARGET_RATE) << 16).checked_div(total_rate_per_spad)?;
        Some(u16::try_from(required_spads.min(0xFFFF)).unwrap_or(u16::MAX))
    }

    /// Apply the ST gain correction once, saturating at the `u16` limit.
    const fn gain_corrected_range(&self) -> u16 {
        #[allow(clippy::cast_lossless)]
        let corrected = (self.range_mm as u32 * GAIN_FACTOR + GAIN_ROUNDING) / GAIN_DIVISOR;
        #[allow(clippy::cast_lossless, clippy::cast_possible_truncation)]
        if corrected > u16::MAX as u32 {
            u16::MAX
        } else {
            corrected as u16
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{RESULT_BLOCK_LEN, ResultBlock};
    use crate::types::{RangeDiagnostics, RangeStatus, Reading};

    /// A result block with a valid status, 1000 SPADs, a 1234 mm range and
    /// non-zero signal and ambient rates.
    const fn valid_block() -> [u8; RESULT_BLOCK_LEN] {
        let mut block = [0_u8; RESULT_BLOCK_LEN];
        block[0] = 9;
        block[2] = 1;
        block[3] = 0x03;
        block[4] = 0xE8;
        block[7] = 0x00;
        block[8] = 0x80;
        block[13] = 0x04;
        block[14] = 0xD2;
        block[15] = 0x01;
        block[16] = 0x00;
        block
    }

    #[test]
    fn parse_decodes_the_documented_offsets_and_gain() {
        let sample = ResultBlock::parse(&valid_block()).reading();

        assert_eq!(sample.status, RangeStatus::Valid);
        assert_eq!(sample.raw_status, 9);
        assert_eq!(sample.reading, Reading::Valid(1_212));
        assert_eq!(
            sample.diagnostics,
            RangeDiagnostics {
                signal_rate: 0x0100,
                ambient_rate: 0x0080,
                effective_spads: 1_000,
            }
        );
    }

    #[test]
    fn required_spads_targets_the_dss_rate_or_falls_back_on_zero_spads() {
        let block = ResultBlock::parse(&valid_block());
        assert_eq!(block.required_spads(), Some(6_666));

        let mut no_spads = valid_block();
        no_spads[3] = 0x00;
        no_spads[4] = 0x00;
        assert_eq!(ResultBlock::parse(&no_spads).required_spads(), None);
    }
}
