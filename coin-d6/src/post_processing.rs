//! Post-processing: aggregates multiple revolutions into a single stable scan.
//!
//! The COIN-D6 spins continuously and reports the same nominal angle across
//! successive revolutions. [`aggregate`] combines several index-aligned
//! [`Scan`]s into one, using a validity gate plus a median/mean reducer so that
//! occasional no-return points (where `distance_mm == 0`) do not corrupt the
//! fused scan.

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
/// verbatim. `distance_mm` must be `> 0`; no-return points (`distance_mm == 0`)
/// are filtered by callers before this is applied.
#[must_use]
pub fn angle_correction_deg(distance_mm: u16) -> f32 {
    let d = f32::from(distance_mm);
    libm::atanf(ANGLE_CORRECTION_COEFF * (d - ANGLE_CORRECTION_ZERO_MM) / (ANGLE_CORRECTION_ZERO_MM * d))
}

/// Reduce multiple aligned scans to a single sanitized [`Scan`].
///
/// `scans` must contain full revolutions that share the same nominal angle at
/// every index: scan `s` is assumed to report the point at bearing `i` in
/// `scans[s].points[i]`, so the output slot `i` is fused purely by index. A
/// scan shorter than the longest one is treated as missing its trailing points,
/// which are interpreted as no-return (`distance_mm == 0`).
///
/// For each index, a sample is *valid* when its `distance_mm` is non-zero. If
/// the fraction of valid samples falls below
/// [`AggregationConfig::validity_ratio`], the output point is a no-return
/// (`distance_mm == 0`, `intensity == 0`); otherwise the valid distances and
/// intensities are reduced with the configured [`AggregationMethod`].
///
/// An empty input slice yields an empty [`Scan`].
#[must_use]
pub fn aggregate(scans: &[Scan], config: &AggregationConfig) -> Scan {
    if scans.is_empty() {
        return Scan::new();
    }

    let width = scans.iter().fold(0, |acc, scan| acc.max(scan.len));

    let mut out = Scan::new();
    out.len = width;

    for i in 0..width {
        let (distance, intensity) = reduce_slot(scans, i, config);
        out.points[i] = Point {
            angle_deg: angle_at(scans, i),
            distance_mm: distance,
            intensity,
        };
    }

    out
}

/// The nominal angle of slot `i`: the first scan that reaches index `i` wins,
/// falling back to `i * 0.9` degrees (the native 0.9° resolution) when none do.
///
/// In practice every scan shares the same nominal angle at each index, so the
/// fallback is only a defensive default.
fn angle_at(scans: &[Scan], i: usize) -> f32 {
    for scan in scans {
        if i < scan.len {
            return scan.points[i].angle_deg;
        }
    }
    // The index is bounded by the scan length (`<= 400`), which `f32` represents
    // exactly, so this cast is lossless despite `clippy::cast_precision_loss`.
    #[allow(clippy::cast_precision_loss)]
    let fallback = i as f32 * 0.9;
    fallback
}

/// Reduce the valid samples at index `i` to a single `(distance, intensity)`,
/// applying the validity gate and the configured aggregation method.
fn reduce_slot(scans: &[Scan], i: usize, config: &AggregationConfig) -> (u16, u8) {
    let count = scans.len();
    let valid = scans
        .iter()
        .filter(|scan| i < scan.len && scan.points[i].distance_mm > 0)
        .count();

    // `valid` and `count` are tiny (bounded by the number of spins, far below
    // `f32`'s exact-integer range), so these casts are lossless.
    #[allow(clippy::cast_precision_loss)]
    let valid_fraction = (valid as f32) / (count as f32);

    if valid == 0 || valid_fraction < config.validity_ratio {
        return (0, 0);
    }

    let distance = match config.method {
        AggregationMethod::Median => median_distance(scans, i, valid),
        AggregationMethod::Mean => mean_distance(scans, i, valid),
    };
    let intensity = match config.method {
        AggregationMethod::Median => median_intensity(scans, i, valid),
        AggregationMethod::Mean => mean_intensity(scans, i, valid),
    };

    (distance, intensity)
}

/// The median of the valid distances at index `i` (reduced over `valid`
/// samples), computed with an `O(n²)` selection so the crate stays heap-free.
fn median_distance(scans: &[Scan], i: usize, valid: usize) -> u16 {
    let upper = valid / 2;
    if valid % 2 == 1 {
        select_valid_distance(scans, i, upper)
    } else {
        let lower = select_valid_distance(scans, i, upper - 1);
        let upper = select_valid_distance(scans, i, upper);
        // Both are `u16` and `lower <= upper`, so this stays in range.
        lower + (upper - lower) / 2
    }
}

/// The arithmetic mean of the valid distances at index `i`, truncated to a
/// whole millimetre.
fn mean_distance(scans: &[Scan], i: usize, valid: usize) -> u16 {
    let sum: usize = scans
        .iter()
        .filter(|scan| i < scan.len && scan.points[i].distance_mm > 0)
        .map(|scan| usize::from(scan.points[i].distance_mm))
        .sum();
    // An average of `u16` values cannot exceed `u16::MAX`, so the division is
    // always in range; `unwrap_or` only guards the (unreachable) conversion
    // failure path without panicking.
    u16::try_from(sum / valid).unwrap_or(u16::MAX)
}

/// The median of the valid intensities at index `i` (validity is still keyed on
/// `distance_mm > 0`).
fn median_intensity(scans: &[Scan], i: usize, valid: usize) -> u8 {
    let upper = valid / 2;
    if valid % 2 == 1 {
        select_valid_intensity(scans, i, upper)
    } else {
        let lower = select_valid_intensity(scans, i, upper - 1);
        let upper = select_valid_intensity(scans, i, upper);
        // Both are `u8` and `lower <= upper`, so this stays in range.
        lower + (upper - lower) / 2
    }
}

/// The arithmetic mean of the valid intensities at index `i`, truncated to a
/// whole intensity step.
fn mean_intensity(scans: &[Scan], i: usize, valid: usize) -> u8 {
    let sum: usize = scans
        .iter()
        .filter(|scan| i < scan.len && scan.points[i].distance_mm > 0)
        .map(|scan| usize::from(scan.points[i].intensity))
        .sum();
    // An average of `u8` values cannot exceed `u8::MAX`, so the division is
    // always in range; `unwrap_or` only guards the (unreachable) conversion
    // failure path without panicking.
    u8::try_from(sum / valid).unwrap_or(u8::MAX)
}

/// The `k`-th smallest (0-indexed) valid distance at index `i`.
///
/// Selection by counting: for each candidate distance, count how many valid
/// distances are strictly smaller and how many are equal; the first candidate
/// whose rank bracket contains `k` is the answer. This is `O(n²)` over the small
/// set of valid samples, avoiding a heap allocation.
fn select_valid_distance(scans: &[Scan], i: usize, k: usize) -> u16 {
    for candidate in scans {
        if i >= candidate.len {
            continue;
        }
        let value = candidate.points[i].distance_mm;
        if value == 0 {
            continue;
        }

        let (less, equal) = rank_distance(scans, i, value);
        if less <= k && k < less + equal {
            return value;
        }
    }
    0
}

/// Count the valid distances at index `i` that are strictly smaller than
/// (`less`) and equal to (`equal`) `value`.
fn rank_distance(scans: &[Scan], i: usize, value: u16) -> (usize, usize) {
    let mut less = 0usize;
    let mut equal = 0usize;
    for scan in scans {
        if i < scan.len && scan.points[i].distance_mm > 0 {
            let d = scan.points[i].distance_mm;
            if d < value {
                less += 1;
            } else if d == value {
                equal += 1;
            }
        }
    }
    (less, equal)
}

/// The `k`-th smallest (0-indexed) valid intensity at index `i`.
///
/// Validity is keyed on `distance_mm > 0` (not intensity), so an intensity of
/// zero on a valid sample is still a real value.
fn select_valid_intensity(scans: &[Scan], i: usize, k: usize) -> u8 {
    for candidate in scans {
        if i >= candidate.len || candidate.points[i].distance_mm == 0 {
            continue;
        }
        let value = candidate.points[i].intensity;

        let (less, equal) = rank_intensity(scans, i, value);
        if less <= k && k < less + equal {
            return value;
        }
    }
    0
}

/// Count the valid intensities at index `i` that are strictly smaller than
/// (`less`) and equal to (`equal`) `value`.
fn rank_intensity(scans: &[Scan], i: usize, value: u8) -> (usize, usize) {
    let mut less = 0usize;
    let mut equal = 0usize;
    for scan in scans {
        if i < scan.len && scan.points[i].distance_mm > 0 {
            let v = scan.points[i].intensity;
            if v < value {
                less += 1;
            } else if v == value {
                equal += 1;
            }
        }
    }
    (less, equal)
}
