//! VL53L1X timing maths.
//!
//! The sensor encodes its timing budgets as macro-period counts with a
//! `(LSByte * 2^MSByte) + 1` register format. This module owns the conversion
//! between microseconds, macro-period counts and that register format, so the
//! async driver only sequences the transactions.
//!
//! The formulas are ported from `vl53l1x-simple` 0.2.0 (MIT OR Apache-2.0); see
//! `THIRD_PARTY_LICENSES.md`.

/// Timing budget guard, in microseconds.
pub const TIMING_GUARD_US: u32 = 4528;

/// Calculate the macro period in microseconds (12.12 format).
///
/// `fast_osc_frequency` is the sensor's 4.12 fixed-point oscillator frequency
/// and `vcsel_period` is the raw `RANGE_CONFIG__VCSEL_PERIOD_A`/`_B` value.
pub const fn calc_macro_period(fast_osc_frequency: u16, vcsel_period: u8) -> u32 {
    // Fast oscillator frequency is 4.12 format; the PLL period is 0.24 format.
    let pll_period_us = (0x01 << 30) / (fast_osc_frequency as u32);

    let vcsel_period_pclks = (vcsel_period + 1) << 1;

    let mut macro_period_us = 2304_u32 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks as u32;
    macro_period_us >>= 6;

    macro_period_us
}

/// Convert a timeout in macro periods to microseconds (12.12 format).
#[allow(clippy::cast_lossless, clippy::cast_possible_truncation)]
pub const fn timeout_mclks_to_microseconds(timeout_mclks: u32, macro_period_us: u32) -> u32 {
    ((timeout_mclks as u64 * macro_period_us as u64 + 0x800) >> 12) as u32
}

/// Convert a timeout in microseconds to macro periods.
///
/// The intermediate product is widened to `u64`, so a large budget cannot wrap
/// the `u32` left shift.
#[allow(clippy::cast_lossless, clippy::cast_possible_truncation)]
pub const fn timeout_microseconds_to_mclks(timeout_us: u32, macro_period_us: u32) -> u32 {
    let scaled = ((timeout_us as u64) << 12) + (macro_period_us as u64 >> 1);
    (scaled / (macro_period_us as u64)) as u32
}

/// Decode the `(LSByte * 2^MSByte) + 1` timeout register format.
#[allow(clippy::cast_lossless)]
pub const fn decode_timeout(register: u16) -> u32 {
    (((register & 0xFF) as u32) << (register >> 8)) + 1
}

/// Encode a macro-period count in the `(LSByte * 2^MSByte) + 1` format.
#[allow(clippy::cast_possible_truncation)]
pub const fn encode_timeout(timeout_mclks: u32) -> u16 {
    if timeout_mclks == 0 {
        return 0;
    }

    let mut ls_byte = timeout_mclks - 1;
    let mut ms_byte = 0_u16;
    while (ls_byte & 0xFFFF_FF00) > 0 {
        ls_byte >>= 1;
        ms_byte += 1;
    }

    (ms_byte << 8) | ((ls_byte & 0xFF) as u16)
}

#[cfg(test)]
mod tests {
    use super::{
        calc_macro_period, decode_timeout, encode_timeout, timeout_mclks_to_microseconds, timeout_microseconds_to_mclks,
    };

    /// The healthy sensor's 0x0C00 oscillator and short-mode VCSEL period.
    const SHORT_MACRO_PERIOD_US: u32 = calc_macro_period(0x0C00, 0x07);

    #[test]
    fn macro_period_uses_the_oscillator_and_vcsel_period() {
        assert_eq!(SHORT_MACRO_PERIOD_US, 3_145_725);
    }

    #[test]
    fn timeout_register_round_trips_for_representable_values() {
        for value in [1, 2, 3, 255, 256, 257, 513, 1_025, 4_097, 32_769, 65_281] {
            let encoded = encode_timeout(value);
            assert_eq!(
                decode_timeout(encoded),
                value,
                "value {value} encoded as {encoded:#06x}"
            );
        }
    }

    #[test]
    fn timeout_conversions_round_trip_through_microseconds() {
        for mclks in [1, 16, 1_000, 14_236] {
            let micros = timeout_mclks_to_microseconds(mclks, SHORT_MACRO_PERIOD_US);
            assert_eq!(
                timeout_microseconds_to_mclks(micros, SHORT_MACRO_PERIOD_US),
                mclks,
                "{mclks} mclks"
            );
        }
    }
}
