//! Post-processing: aggregates multiple revolutions into a single stable scan.
//!
//! The COIN-D6 spins continuously and reports the same nominal angle across
//! successive revolutions, but every revolution starts at whatever angle its
//! ring-start packet lands on and can contain a slightly different number of
//! points (both effects are especially pronounced while the motor is spinning
//! up). [`aggregate`] therefore reduces revolutions **by angle** — each point is
//! binned into a fixed-width angular bucket — rather than by index, so that a
//! revolution whose index-0 point sits at a different bearing cannot skew the
//! fused scan.

use core::num::NonZeroU16;

use crate::types::{AggregationConfig, AggregationMethod, Point, Scan};

/// Numerator constant of the vendor's distance-dependent angle correction.
pub const ANGLE_CORRECTION_COEFF: f32 = 19.16;

/// Zero-distance constant (mm) of the vendor's angle correction.
pub const ANGLE_CORRECTION_ZERO_MM: f32 = 90.15;

/// The vendor's distance-dependent angle correction, in degrees.
///
/// Transcribed verbatim from the vendor SDK:
/// `atan(19.16 * (d - 90.15) / (90.15 * d))`, added directly as degrees.
///
/// The `atan` result is *not* converted from radians to degrees — the vendor
/// treats the raw `atan` output as a degree offset, so this quirk is preserved
/// verbatim. Takes a [`NonZeroU16`] because a no-return point (`None`) has no
/// meaningful correction.
#[must_use]
pub fn angle_correction_deg(distance_mm: NonZeroU16) -> f32 {
    let d = f32::from(distance_mm.get());
    libm::atanf(ANGLE_CORRECTION_COEFF * (d - ANGLE_CORRECTION_ZERO_MM) / (ANGLE_CORRECTION_ZERO_MM * d))
}

/// Reduce multiple revolutions to a single sanitized [`Scan`].
///
/// Each revolution is reduced **by angle**, not by index: points are assigned to
/// fixed-width angular buckets of width [`AggregationConfig::resolution_deg`],
/// and each bucket is fused independently. A point belongs to bucket `b` when
/// its (angle-corrected, wrap-normalised) bearing falls in
/// `[b * resolution, (b + 1) * resolution)`, so revolutions that start at
/// different angles are matched correctly.
///
/// For each bucket, a sample is *valid* when its `distance_mm` is `Some`. If the
/// fraction of revolutions contributing a valid sample falls below
/// [`AggregationConfig::validity_ratio`], the output point is a no-return
/// (`distance_mm == None`, `intensity == 0`); otherwise the valid distances and
/// intensities are reduced with the configured [`AggregationMethod`].
///
/// The output is a fixed grid of `bucket_count` points (one per bucket), sorted
/// by bearing, with each point's angle set to its bucket's lower edge (the
/// device's native angle for that bucket). An empty input slice yields an empty
/// [`Scan`].
#[must_use]
pub fn aggregate<const N: usize>(scans: &[Scan<N>], config: &AggregationConfig) -> Scan<N> {
    if scans.is_empty() {
        return Scan::new();
    }

    let resolution = sanitise_resolution(config.resolution_deg);
    let bins = bucket_count::<N>(resolution);

    let mut out = Scan::new();
    out.len = bins;

    for bin in 0..bins {
        let angle = bucket_angle_deg(bin, resolution);
        let (distance, intensity) = reduce_bucket(scans, bin, resolution, config);
        out.points[bin] = Point {
            angle_deg: angle,
            distance_mm: NonZeroU16::new(distance),
            intensity,
        };
    }

    out
}

/// The number of angular buckets `aggregate` emits, clamped to `[1, N]`.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn bucket_count<const N: usize>(resolution_deg: f32) -> usize {
    // `resolution_deg` is already sanitised to a positive, finite value, so the
    // quotient is finite and the cast saturates safely before the clamp.
    (libm::roundf(360.0 / resolution_deg) as usize).clamp(1, N)
}

/// The representative bearing of bucket `bin`, in degrees.
///
/// Uses the bucket's lower edge so the output grid lines up with the device's
/// native angles (`0°, 0.9°, 1.8°, …`) rather than an arbitrary half-bin
/// offset.
#[allow(clippy::cast_precision_loss)]
fn bucket_angle_deg(bin: usize, resolution_deg: f32) -> f32 {
    bin as f32 * resolution_deg
}

/// Coerce an angle into `[0, 360)`.
///
/// Angle correction can push a bearing just past 360° or just below 0°, so
/// binning must normalise before deciding which bucket a point lands in. The
/// `%` operator matches `rem_euclid(360.0)` here because the input is always
/// within one correction step of the valid range.
fn normalise_angle(angle_deg: f32) -> f32 {
    let wrapped = angle_deg % 360.0;
    if wrapped < 0.0 { wrapped + 360.0 } else { wrapped }
}

/// The point in `scan` that belongs to bucket `bin`, if any.
///
/// Points within a revolution are already in increasing bearing order, but the
/// array is not globally sorted because it wraps through 360°, so this does a
/// linear scan. At the native 0.9° resolution each revolution contributes at most
/// one point per bucket, so the first match is returned.
#[allow(clippy::cast_precision_loss)]
fn point_at_bin<const N: usize>(scan: &Scan<N>, bin: usize, resolution_deg: f32) -> Option<Point> {
    let lower = bin as f32 * resolution_deg;
    let upper = lower + resolution_deg;

    for point in &scan.points[..scan.len] {
        let bearing = normalise_angle(point.angle_deg);
        if bearing >= lower && bearing < upper {
            return Some(*point);
        }
    }
    None
}

/// The valid distance (in millimetres) of `scan` in bucket `bin`, if any.
///
/// `None` when the scan has no point in the bucket, or its point there is a
/// no-return.
fn distance_at_bin<const N: usize>(scan: &Scan<N>, bin: usize, resolution_deg: f32) -> Option<u16> {
    point_at_bin(scan, bin, resolution_deg)
        .and_then(|point| point.distance_mm)
        .map(NonZeroU16::get)
}

/// The intensity of `scan`'s valid point in bucket `bin`, if any.
///
/// Validity is keyed on `distance_mm.is_some()`, so a no-return point's
/// intensity is ignored even if it is non-zero.
fn intensity_at_bin<const N: usize>(scan: &Scan<N>, bin: usize, resolution_deg: f32) -> Option<u8> {
    point_at_bin(scan, bin, resolution_deg)
        .filter(|point| point.distance_mm.is_some())
        .map(|point| point.intensity)
}

/// Reduce all revolutions at one angular bucket to a single `(distance,
/// intensity)`, applying the validity gate and the configured method.
///
/// The returned distance is raw millimetres, with `0` used internally to mean
/// "no return"; [`aggregate`] maps it back onto `Option<NonZeroU16>`.
fn reduce_bucket<const N: usize>(
    scans: &[Scan<N>],
    bin: usize,
    resolution_deg: f32,
    config: &AggregationConfig,
) -> (u16, u8) {
    let spins = scans.len();
    let valid = scans
        .iter()
        .filter(|scan| distance_at_bin(scan, bin, resolution_deg).is_some())
        .count();

    // `valid` and `spins` are tiny (bounded by the number of spins, far below
    // `f32`'s exact-integer range), so these casts are lossless.
    #[allow(clippy::cast_precision_loss)]
    let valid_fraction = (valid as f32) / (spins as f32);

    if valid == 0 || valid_fraction < config.validity_ratio {
        return (0, 0);
    }

    let distance = match config.method {
        AggregationMethod::Median => median_distance(scans, bin, resolution_deg, valid),
        AggregationMethod::Mean => mean_distance(scans, bin, resolution_deg, valid),
    };
    let intensity = match config.method {
        AggregationMethod::Median => median_intensity(scans, bin, resolution_deg, valid),
        AggregationMethod::Mean => mean_intensity(scans, bin, resolution_deg, valid),
    };

    (distance, intensity)
}

/// The median of the valid distances in bucket `bin` (reduced over `valid`
/// samples), computed with an `O(n²)` selection so the crate stays heap-free.
fn median_distance<const N: usize>(scans: &[Scan<N>], bin: usize, resolution_deg: f32, valid: usize) -> u16 {
    let upper = valid / 2;
    if valid % 2 == 1 {
        select_distance(scans, bin, resolution_deg, upper)
    } else {
        let lower = select_distance(scans, bin, resolution_deg, upper - 1);
        let upper = select_distance(scans, bin, resolution_deg, upper);
        // Both are `u16` and `lower <= upper`, so this stays in range.
        lower + (upper - lower) / 2
    }
}

/// The arithmetic mean of the valid distances in bucket `bin`, truncated to a
/// whole millimetre.
fn mean_distance<const N: usize>(scans: &[Scan<N>], bin: usize, resolution_deg: f32, valid: usize) -> u16 {
    let sum: usize = scans
        .iter()
        .filter_map(|scan| distance_at_bin(scan, bin, resolution_deg))
        .map(usize::from)
        .sum();
    // An average of `u16` values cannot exceed `u16::MAX`, so the division is
    // always in range; `unwrap_or` only guards the (unreachable) conversion
    // failure path without panicking.
    u16::try_from(sum / valid).unwrap_or(u16::MAX)
}

/// The median of the valid intensities in bucket `bin` (validity is keyed on
/// `distance_mm.is_some()`).
fn median_intensity<const N: usize>(scans: &[Scan<N>], bin: usize, resolution_deg: f32, valid: usize) -> u8 {
    let upper = valid / 2;
    if valid % 2 == 1 {
        select_intensity(scans, bin, resolution_deg, upper)
    } else {
        let lower = select_intensity(scans, bin, resolution_deg, upper - 1);
        let upper = select_intensity(scans, bin, resolution_deg, upper);
        // Both are `u8` and `lower <= upper`, so this stays in range.
        lower + (upper - lower) / 2
    }
}

/// The arithmetic mean of the valid intensities in bucket `bin`, truncated to a
/// whole intensity step.
fn mean_intensity<const N: usize>(scans: &[Scan<N>], bin: usize, resolution_deg: f32, valid: usize) -> u8 {
    let sum: usize = scans
        .iter()
        .filter_map(|scan| intensity_at_bin(scan, bin, resolution_deg))
        .map(usize::from)
        .sum();
    // An average of `u8` values cannot exceed `u8::MAX`, so the division is
    // always in range; `unwrap_or` only guards the (unreachable) conversion
    // failure path without panicking.
    u8::try_from(sum / valid).unwrap_or(u8::MAX)
}

/// The `k`-th smallest (0-indexed) valid distance in bucket `bin`.
///
/// Selection by counting: for each candidate distance, count how many valid
/// distances are strictly smaller and how many are equal; the first candidate
/// whose rank bracket contains `k` is the answer. This is `O(n²)` over the small
/// set of valid samples, avoiding a heap allocation.
fn select_distance<const N: usize>(scans: &[Scan<N>], bin: usize, resolution_deg: f32, k: usize) -> u16 {
    for scan in scans {
        let Some(value) = distance_at_bin(scan, bin, resolution_deg) else {
            continue;
        };

        let (less, equal) = rank_distance(scans, bin, resolution_deg, value);
        if less <= k && k < less + equal {
            return value;
        }
    }
    // Unreachable: `reduce_bucket` only calls this when `valid > 0`, so at least
    // one scan has a valid distance to select.
    0
}

/// Count the valid distances in bucket `bin` that are strictly smaller than
/// (`less`) and equal to (`equal`) `value`.
fn rank_distance<const N: usize>(scans: &[Scan<N>], bin: usize, resolution_deg: f32, value: u16) -> (usize, usize) {
    let mut less = 0usize;
    let mut equal = 0usize;
    for scan in scans {
        let Some(d) = distance_at_bin(scan, bin, resolution_deg) else {
            continue;
        };
        if d < value {
            less += 1;
        } else if d == value {
            equal += 1;
        }
    }
    (less, equal)
}

/// The `k`-th smallest (0-indexed) valid intensity in bucket `bin`.
///
/// Validity is keyed on `distance_mm.is_some()` (not intensity), so an
/// intensity of zero on a valid sample is still a real value.
fn select_intensity<const N: usize>(scans: &[Scan<N>], bin: usize, resolution_deg: f32, k: usize) -> u8 {
    for scan in scans {
        let Some(value) = intensity_at_bin(scan, bin, resolution_deg) else {
            continue;
        };

        let (less, equal) = rank_intensity(scans, bin, resolution_deg, value);
        if less <= k && k < less + equal {
            return value;
        }
    }
    0
}

/// Count the valid intensities in bucket `bin` that are strictly smaller than
/// (`less`) and equal to (`equal`) `value`.
fn rank_intensity<const N: usize>(scans: &[Scan<N>], bin: usize, resolution_deg: f32, value: u8) -> (usize, usize) {
    let mut less = 0usize;
    let mut equal = 0usize;
    for scan in scans {
        let Some(v) = intensity_at_bin(scan, bin, resolution_deg) else {
            continue;
        };
        if v < value {
            less += 1;
        } else if v == value {
            equal += 1;
        }
    }
    (less, equal)
}

/// Coerce the configured bucket width to a safe, positive, finite value.
fn sanitise_resolution(resolution_deg: f32) -> f32 {
    if resolution_deg.is_finite() && resolution_deg > 0.0 {
        resolution_deg
    } else {
        1.0
    }
}
