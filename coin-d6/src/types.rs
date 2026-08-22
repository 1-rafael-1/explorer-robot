//! Public domain types shared across the decoder, post-processing, and driver
//! stages.

use core::num::NonZeroU16;

/// A single `LiDAR` return, in polar coordinates.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Point {
    /// Bearing of the return, in degrees.
    pub angle_deg: f32,
    /// Measured range in millimetres, or `None` when the sensor got no return.
    pub distance_mm: Option<NonZeroU16>,
    /// Return signal strength (0–255). `0` is a valid intensity on a return.
    pub intensity: u8,
}

/// A single 360° revolution of `LiDAR` returns.
///
/// `points` holds up to `N` [`Point`]s; `len` records how many are actually
/// present (always `<= N`). The COIN-D6 emits at a native 0.9° resolution
/// (400 points per revolution); the default `N = 512` leaves headroom so a
/// slow-spinning revolution cannot be truncated.
#[derive(Debug, Clone)]
pub struct Scan<const N: usize = 512> {
    /// The measured points, in increasing angle order.
    pub points: [Point; N],
    /// The number of points in `points` that are valid.
    pub len: usize,
}

impl<const N: usize> Scan<N> {
    /// Create a new, empty scan with `len = 0`.
    #[must_use]
    pub fn new() -> Self {
        Self {
            points: [Point::default(); N],
            len: 0,
        }
    }
}

impl<const N: usize> Default for Scan<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Driver configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Whether to apply angular correction.
    pub angle_correction: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self { angle_correction: true }
    }
}

/// How several revolutions are combined into a single [`Scan`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AggregationMethod {
    /// Use the median distance across revolutions.
    #[default]
    Median,
    /// Use the arithmetic mean distance across revolutions.
    Mean,
}

/// Post-processing parameters that control multi-revolution aggregation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AggregationConfig {
    /// Number of revolutions to aggregate.
    pub spins: usize,
    /// Minimum fraction of revolutions that must report a valid distance for a
    /// point to be kept.
    pub validity_ratio: f32,
    /// How the aggregated distances are combined.
    pub method: AggregationMethod,
    /// Angular bin width in degrees. Revolutions are reduced by angle, so this
    /// must match the device's native angular resolution (0.9°, i.e. 400 points
    /// per revolution).
    pub resolution_deg: f32,
}

impl Default for AggregationConfig {
    fn default() -> Self {
        Self {
            spins: 5,
            validity_ratio: 0.5,
            method: AggregationMethod::Median,
            resolution_deg: 0.9,
        }
    }
}

/// Driver error.
///
/// Combines the transient resync condition with fatal UART and power-pin errors.
#[derive(Debug)]
pub enum Error<UartE, PinE> {
    /// Malformed device-info frame during power-on (checksum or type mismatch).
    /// The point-stream readers recover from transient checksum mismatches
    /// transparently and do not return this variant.
    Resync,
    /// Fatal UART error.
    Uart(UartE),
    /// Power-pin GPIO error.
    Pin(PinE),
    /// Timed out waiting for the start of a revolution.
    Timeout,
}
