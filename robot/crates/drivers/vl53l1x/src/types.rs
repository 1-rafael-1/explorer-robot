//! Decoded range status, readings and diagnostics.
//!
//! The sensor reports an 8-bit status code as the first byte of the 17-byte
//! result block at `0x0089`. The code names how the measurement completed; the
//! range itself is only trusted for a valid status. Unlike the VL53L0X there is
//! no out-of-range sentinel: **No Target** is a status classification.

/// The sensor's classification of a measurement.
///
/// This is the decoded 8-bit device code, not ST's folded PAL range status.
/// See [`from_raw`](Self::from_raw) for the mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RangeStatus {
    /// The measurement completed and the range is trustworthy.
    Valid,
    /// The measurement completed but the target is below the minimum range.
    MinRangeClipped,
    /// The range is valid but the wrap-around check has not been done.
    NoWrapCheck,
    /// The signal rate is below the internal threshold.
    SignalFail,
    /// The phase is out of bounds, so nothing was detected in range.
    OutOfBounds,
    /// The target is below the minimum detection threshold.
    MinRangeFail,
    /// The sigma estimator check is above the internal threshold.
    SigmaFail,
    /// The target wrapped: no matching phase in the other VCSEL period.
    WrapFail,
    /// The crosstalk or range-ignore threshold was reached.
    XtalkFail,
    /// A hardware or VCSEL failure.
    HardwareFail,
    /// The synchronization interrupt fired when ranging started.
    Synchronization,
    /// No update was available.
    NoUpdate,
    /// Any other raw code, preserved verbatim.
    Unknown(u8),
}

impl RangeStatus {
    /// Decode the raw 8-bit status and the stream count that came with it.
    ///
    /// Raw `9` is only valid when at least one stream sample completed; with a
    /// zero stream count it is a no-wrap-check result.
    #[must_use]
    pub const fn from_raw(raw: u8, stream_count: u8) -> Self {
        match raw {
            8 => Self::MinRangeClipped,
            9 if stream_count == 0 => Self::NoWrapCheck,
            9 => Self::Valid,
            4 => Self::SignalFail,
            5 => Self::OutOfBounds,
            13 => Self::MinRangeFail,
            6 => Self::SigmaFail,
            7 => Self::WrapFail,
            12 => Self::XtalkFail,
            1 | 2 | 3 | 17 => Self::HardwareFail,
            255 => Self::NoUpdate,
            18 => Self::Synchronization,
            other => Self::Unknown(other),
        }
    }

    /// Whether this status may drive the lifecycle's invalid-streak recovery.
    ///
    /// This is the recovery set only: hardware, sigma, wrap, crosstalk,
    /// no-update and unknown codes. No-target, min-range, min-clip,
    /// no-wrap-check and synchronization readings keep the last decision but do
    /// not reset the sensor.
    #[must_use]
    pub const fn is_fault(self) -> bool {
        matches!(
            self,
            Self::HardwareFail | Self::SigmaFail | Self::WrapFail | Self::XtalkFail | Self::NoUpdate | Self::Unknown(_)
        )
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

impl Reading {
    /// Classify a decoded status and the gain-corrected range.
    ///
    /// Raw `9` with a zero stream count decodes to [`RangeStatus::NoWrapCheck`]
    /// and is deliberately [`NoTarget`](Self::NoTarget), not
    /// [`Valid`](Self::Valid): it is one sample of caution against a wrapped
    /// far target, and the next sample is one inter-measurement period away.
    #[must_use]
    pub const fn from_status(status: RangeStatus, range_mm: u16) -> Self {
        match status {
            RangeStatus::Valid | RangeStatus::MinRangeClipped => Self::Valid(range_mm),
            RangeStatus::SignalFail
            | RangeStatus::OutOfBounds
            | RangeStatus::MinRangeFail
            | RangeStatus::NoWrapCheck => Self::NoTarget,
            other => Self::Invalid(other),
        }
    }
}

/// The raw fixed-point rates and effective SPAD count of one measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RangeDiagnostics {
    /// The peak signal count rate in 8.7 fixed point.
    pub signal_rate: u16,
    /// The ambient count rate in 8.7 fixed point.
    pub ambient_rate: u16,
    /// The actual effective SPAD count.
    pub effective_spads: u16,
}

/// One Rangefinder Reading: the classified measurement, the decoded status, the
/// raw 8-bit status code and the diagnostics that produced it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RangefinderReading {
    /// The classified reading.
    pub reading: Reading,
    /// The decoded 8-bit device status.
    pub status: RangeStatus,
    /// The raw 8-bit device status code, preserved verbatim.
    pub raw_status: u8,
    /// The raw rates and effective SPAD count.
    pub diagnostics: RangeDiagnostics,
}
