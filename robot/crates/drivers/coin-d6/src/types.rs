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
    /// The measured points, in increasing angle order within a revolution; the
    /// array wraps through 0° and so is not globally sorted.
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

/// The result of a by-angle reduction over several [`Scan`]s.
///
/// A reduction is a fixed grid of up to `BUCKETS` angular buckets, one [`Point`]
/// per bucket, in increasing bearing order. Unlike [`Scan`], the bucket count is
/// the reduction's own constant, independent of the capacity of the scans it
/// consumes: a caller that needs `BUCKETS` output slots can name them directly
/// instead of borrowing a native scan's capacity, so a bucket count smaller than
/// the caller's slot count cannot be expressed.
///
/// `len` records how many buckets were actually emitted (`<= BUCKETS`); the
/// points beyond `len` are left at their default value.
#[derive(Debug, Clone)]
pub struct Reduction<const BUCKETS: usize> {
    /// The fused points, one per emitted bucket, in increasing bearing order.
    pub points: [Point; BUCKETS],
    /// The number of buckets in `points` that are valid (`<= BUCKETS`).
    pub len: usize,
}

impl<const BUCKETS: usize> Reduction<BUCKETS> {
    /// Create a new, empty reduction with `len = 0`.
    #[must_use]
    pub fn new() -> Self {
        Self {
            points: [Point::default(); BUCKETS],
            len: 0,
        }
    }
}

impl<const BUCKETS: usize> Default for Reduction<BUCKETS> {
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

/// How several revolutions are combined into a single [`Reduction`].
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
    /// Minimum fraction of revolutions that must report a valid distance for a
    /// point to be kept.
    pub validity_ratio: f32,
    /// How the aggregated distances are combined.
    pub method: AggregationMethod,
    /// Angular bin width in degrees. Revolutions are reduced by angle into
    /// fixed-width buckets of this size.
    ///
    /// The device emits at a native 0.9° spacing (400 points per revolution).
    /// Widths **coarser** than that are supported and are the intended use: each
    /// bucket gathers several native samples and collapses them to the nearest
    /// valid return, so coarsening never hides a closer obstacle. Reducing at a
    /// width wider than the native spacing is also what keeps buckets sampled
    /// under normal spacing, so a `None` bucket usually means a measured no-return
    /// rather than an unsampled hole. That is not a guarantee: the driver's
    /// per-point angle correction displaces each return by its range, so two
    /// neighbours at very different distances can be spread past a bucket boundary
    /// and leave the bucket between them unsampled. A non-finite or non-positive
    /// value falls back to the default.
    pub resolution_deg: f32,
}

impl Default for AggregationConfig {
    fn default() -> Self {
        Self {
            validity_ratio: 0.5,
            method: AggregationMethod::Median,
            resolution_deg: 0.9,
        }
    }
}

/// Points per revolution the COIN-D6 emits at its native 0.9° resolution
/// (steady state).
pub const NATIVE_POINTS: usize = 400;

/// Parameters controlling the optional rotor warm-up phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WarmupConfig {
    /// A spin counts as "settled" when within this many points of
    /// [`NATIVE_POINTS`].
    pub settle_tolerance: usize,
    /// Consecutive in-band spins (or, conversely, spins without a new
    /// point-count high) needed to declare the rotor settled or plateaued.
    pub settle_stable_spins: usize,
    /// Upper bound on how many spins to discard before giving up.
    pub max_spins: usize,
}

impl Default for WarmupConfig {
    fn default() -> Self {
        Self {
            settle_tolerance: 3,
            settle_stable_spins: 12,
            max_spins: 50,
        }
    }
}

/// The result of an optional rotor warm-up phase (see `CoinD6::warm_up`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WarmupOutcome {
    /// The rotor reached steady state (point count within the settle band).
    Settled {
        /// Warm-up spins discarded before settling.
        spins: usize,
        /// Point count at which the rotor settled.
        points: usize,
    },
    /// The point count plateaued below the settle band.
    Plateaued {
        /// Warm-up spins discarded before giving up.
        spins: usize,
        /// The plateaued point count (highest seen).
        points: usize,
    },
    /// The spin budget was exhausted without settling or plateauing.
    Exhausted {
        /// Warm-up spins discarded.
        spins: usize,
    },
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
    /// Data stopped flowing while waiting for a revolution: the byte-count
    /// watchdog was exhausted, or the stream ended (end-of-stream).
    Timeout,
}
