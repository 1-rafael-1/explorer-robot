//! Decoded range status, readings and Rangefinder Readings.
//!
//! The sensor reports a 4-bit status code in bits 6:3 of register `0x14`. A
//! code of `11` means the measurement completed; the other codes name the
//! reason it did not. A range of [`OUT_OF_RANGE_MILLIMETERS`] means the sensor
//! saw no target, even when the status says the measurement completed.

use crate::constants::OUT_OF_RANGE_MILLIMETERS;

/// The sensor's classification of a measurement.
///
/// This is the raw 4-bit device code, not ST's folded PAL range status. See
/// [`from_raw`](Self::from_raw) for the mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RangeStatus {
    /// Raw `11`: the measurement completed.
    Valid,
    /// Raw `0`: no error and no range.
    NoRange,
    /// Raw `1..=3`: a VCSEL continuity, watchdog or VHV failure.
    HardwareFail,
    /// Raw `4`: the minimum signal rate was not reached.
    SignalFail,
    /// Raw `6` or `9`: the range phase check failed.
    PhaseFail,
    /// Raw `8` or `10`: the target is below the minimum range.
    MinRangeFail,
    /// Any other raw code, preserved verbatim.
    Unknown(u8),
}

impl RangeStatus {
    /// Decode the 4-bit device code (`(byte & 0x78) >> 3`).
    #[must_use]
    pub const fn from_raw(raw: u8) -> Self {
        match raw {
            11 => Self::Valid,
            0 => Self::NoRange,
            1..=3 => Self::HardwareFail,
            4 => Self::SignalFail,
            6 | 9 => Self::PhaseFail,
            8 | 10 => Self::MinRangeFail,
            other => Self::Unknown(other),
        }
    }
}

/// A decoded measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reading {
    /// A completed measurement of this many millimetres.
    Valid(u16),
    /// No target was detected or the target is out of range.
    NoTarget,
    /// The sensor reported a failure instead of a trustworthy range.
    Invalid(RangeStatus),
}

/// One Rangefinder Reading: the decoded measurement together with the raw
/// 4-bit status code that produced it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RangefinderReading {
    /// The decoded reading.
    pub reading: Reading,
    /// The 4-bit device status code, not the full `0x14` register byte.
    pub raw_status: u8,
}

impl RangefinderReading {
    /// Decode the full register byte from `0x14` and the millimetre range.
    ///
    /// The status is taken from bits 6:3. A range of
    /// [`OUT_OF_RANGE_MILLIMETERS`] is [`Reading::NoTarget`] regardless of the
    /// status; otherwise a valid status yields [`Reading::Valid`], a no-range
    /// status also yields [`Reading::NoTarget`], and every other status yields
    /// [`Reading::Invalid`] so hardware and signal failures stay visible.
    #[must_use]
    pub const fn decode(raw_status_byte: u8, range_mm: u16) -> Self {
        let raw_status = (raw_status_byte & 0x78) >> 3;
        let status = RangeStatus::from_raw(raw_status);
        let reading = if range_mm == OUT_OF_RANGE_MILLIMETERS {
            Reading::NoTarget
        } else {
            match status {
                RangeStatus::Valid => Reading::Valid(range_mm),
                RangeStatus::NoRange => Reading::NoTarget,
                other => Reading::Invalid(other),
            }
        };
        Self { reading, raw_status }
    }
}
