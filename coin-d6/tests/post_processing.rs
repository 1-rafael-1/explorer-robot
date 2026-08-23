//! Host-side integration tests for the post-processing stage.
//!
//! These exercise [`coin_d6::aggregate`]'s angle binning, validity gate and
//! median/mean reducers, plus [`coin_d6::angle_correction_deg`]'s sign and
//! magnitude.

use core::num::NonZeroU16;

use coin_d6::{AggregationConfig, AggregationMethod, Point, Scan, aggregate, angle_correction_deg};

/// Build a [`Scan`] from a compact list of `(angle_deg, distance_mm, intensity)`
/// triples; the scan's `len` is set to the number of triples and the remaining
/// points stay zeroed. A `distance_mm` of `0` becomes a no-return (`None`).
fn build_scan(points: &[(f32, u16, u8)]) -> Scan<400> {
    let mut scan = Scan::<400>::new();
    scan.len = points.len();
    for (i, &(angle_deg, distance_mm, intensity)) in points.iter().enumerate() {
        scan.points[i] = Point {
            angle_deg,
            distance_mm: NonZeroU16::new(distance_mm),
            intensity,
        };
    }
    scan
}

/// Build an [`AggregationConfig`] with the given parameters and a 1° bin width.
/// (The general tests use 1° for clean integer bucket indices; the crate's
/// default is 0.9°, which is covered separately.)
fn config(validity_ratio: f32, method: AggregationMethod) -> AggregationConfig {
    AggregationConfig {
        validity_ratio,
        method,
        resolution_deg: 1.0,
    }
}

/// The number of output buckets for a 1° bin width.
const BINS: usize = 360;

#[test]
fn post_processing_validity_gate_emits_no_return_below_ratio() {
    let scans = [
        build_scan(&[(0.0, 100, 10)]),
        build_scan(&[(0.0, 200, 20)]),
        build_scan(&[(0.0, 0, 0)]),
        build_scan(&[(0.0, 0, 0)]),
        build_scan(&[(0.0, 0, 0)]),
    ];

    let out = aggregate(&scans, &config(0.5, AggregationMethod::Median));

    // 2/5 valid = 0.4 < 0.5, so the 0° bucket is dropped.
    assert_eq!(out.len, BINS);
    assert_eq!(out.points[0].distance_mm, None);
    assert_eq!(out.points[0].intensity, 0);
}

#[test]
fn post_processing_validity_gate_keeps_point_at_or_above_ratio() {
    let scans = [
        build_scan(&[(0.0, 100, 10)]),
        build_scan(&[(0.0, 200, 20)]),
        build_scan(&[(0.0, 300, 30)]),
        build_scan(&[(0.0, 0, 0)]),
        build_scan(&[(0.0, 0, 0)]),
    ];

    let out = aggregate(&scans, &config(0.5, AggregationMethod::Median));

    // 3/5 valid = 0.6 >= 0.5, so the 0° bucket survives with the median.
    assert_eq!(out.len, BINS);
    assert_eq!(out.points[0].distance_mm, NonZeroU16::new(200));
    assert_eq!(out.points[0].intensity, 20);
}

#[test]
fn post_processing_median_and_mean_differ() {
    let distances = [10u16, 10, 40];
    let intensities = [1u8, 1, 7];
    let scans = distances
        .iter()
        .zip(intensities)
        .map(|(&d, int)| build_scan(&[(0.0, d, int)]))
        .collect::<Vec<_>>();

    let median_out = aggregate(&scans, &config(0.5, AggregationMethod::Median));
    assert_eq!(median_out.points[0].distance_mm, NonZeroU16::new(10));
    assert_eq!(median_out.points[0].intensity, 1);

    let mean_out = aggregate(&scans, &config(0.5, AggregationMethod::Mean));
    // (10 + 10 + 40) / 3 == 20; (1 + 1 + 7) / 3 == 3.
    assert_eq!(mean_out.points[0].distance_mm, NonZeroU16::new(20));
    assert_eq!(mean_out.points[0].intensity, 3);
}

#[test]
fn post_processing_all_no_return_yields_no_return() {
    let scans = [
        build_scan(&[(0.0, 0, 0)]),
        build_scan(&[(0.0, 0, 0)]),
        build_scan(&[(0.0, 0, 0)]),
    ];

    let out = aggregate(&scans, &config(0.5, AggregationMethod::Median));

    assert_eq!(out.len, BINS);
    assert_eq!(out.points[0].distance_mm, None);
    assert_eq!(out.points[0].intensity, 0);
}

#[test]
fn post_processing_aggregate_bins_by_angle_not_index() {
    // The two scans report the 20° return at different indices (1 and 0), so
    // index-aligned aggregation would fuse 10° with 20° and corrupt both. By
    // angle, 10° and 20° land in separate buckets and are reduced correctly.
    let scans = [
        build_scan(&[(10.0, 100, 10), (20.0, 150, 15)]),
        build_scan(&[(20.0, 250, 25)]),
    ];

    let out = aggregate(&scans, &config(0.5, AggregationMethod::Median));

    // 10° bucket: only the first scan contributes (1/2 = 0.5, kept) → 100.
    assert_eq!(out.points[10].distance_mm, NonZeroU16::new(100));
    // 20° bucket: median of [150, 250].
    assert_eq!(out.points[20].distance_mm, NonZeroU16::new(200));
    // Empty buckets are no-returns.
    assert_eq!(out.points[0].distance_mm, None);
}

#[test]
fn post_processing_aggregate_sets_bucket_start_angle() {
    let scans = [build_scan(&[(10.2, 100, 10)])];

    let out = aggregate(&scans, &config(0.5, AggregationMethod::Median));

    // A point at 10.2° falls in the [10°, 11°) bucket, labelled 10° (the
    // bucket's lower edge, matching the device's native angle grid).
    assert_eq!(out.points[10].distance_mm, NonZeroU16::new(100));
    assert_eq!(out.points[10].angle_deg, 10.0);
}

#[test]
fn post_processing_default_resolution_is_native_0_9_degrees() {
    let scans = [build_scan(&[(0.9, 100, 10)])];

    let out = aggregate(&scans, &AggregationConfig::default());

    // 0.9° is the native resolution → 360 / 0.9 = 400 buckets.
    assert_eq!(out.len, 400);
    // A point at 0.9° falls in bucket 1, labelled with the native 0.9° angle.
    assert_eq!(out.points[1].distance_mm, NonZeroU16::new(100));
    assert!((out.points[1].angle_deg - 0.9).abs() < 1e-4);
    // Bucket 0 has no return below 0.9°.
    assert_eq!(out.points[0].distance_mm, None);
}

#[test]
fn post_processing_angle_correction_matches_known_distances() {
    // Positive at 1000 mm.
    assert!((angle_correction_deg(NonZeroU16::new(1000).unwrap()) - 0.191_017).abs() < 1e-4);
    // Negative at 50 mm.
    assert!((angle_correction_deg(NonZeroU16::new(50).unwrap()) - (-0.169_037)).abs() < 1e-4);
}

#[test]
fn post_processing_angle_correction_crosses_zero_at_the_zero_distance() {
    // The zero-distance constant is 90.15 mm, which a `u16` distance cannot
    // represent exactly; the nearest integers must therefore straddle zero.
    assert!(angle_correction_deg(NonZeroU16::new(90).unwrap()) < 0.0);
    assert!(angle_correction_deg(NonZeroU16::new(91).unwrap()) > 0.0);
    // Both are tiny compared with the correction's ~0.19° peak magnitude.
    assert!((angle_correction_deg(NonZeroU16::new(90).unwrap())).abs() < 0.01);
    assert!((angle_correction_deg(NonZeroU16::new(91).unwrap())).abs() < 0.01);
}

#[test]
fn post_processing_bucket_count_ceils_to_cover_full_range() {
    // A point near the end of the revolution must still land in the last bucket
    // even though the bucket width (1.9°) does not divide 360° evenly.
    let scans = [build_scan(&[(359.5, 100, 10)])];

    let out = aggregate(
        &scans,
        &AggregationConfig {
            validity_ratio: 1.0,
            method: AggregationMethod::Median,
            resolution_deg: 1.9,
        },
    );

    // 360 / 1.9 ≈ 189.47, so `ceil` produces 190 buckets (not 189).
    assert_eq!(out.len, 190);
    // 359.5° falls in the last bucket [359.1°, 361°).
    assert_eq!(out.points[189].distance_mm, NonZeroU16::new(100));
}

#[test]
fn post_processing_coarsens_when_resolution_exceeds_capacity() {
    // 0.5° would need 720 buckets, more than `Scan<400>` can hold. The grid is
    // coarsened to 400 × 0.9° so the full circle is still represented.
    let scans = [build_scan(&[(300.0, 100, 10)])];

    let out = aggregate(
        &scans,
        &AggregationConfig {
            validity_ratio: 1.0,
            method: AggregationMethod::Median,
            resolution_deg: 0.5,
        },
    );

    assert_eq!(out.len, 400);
    // 300° lands in bucket floor(300 / 0.9) = 333, not silently dropped.
    assert_eq!(out.points[333].distance_mm, NonZeroU16::new(100));
}

#[test]
fn post_processing_bucket_reduces_to_nearest_valid_return() {
    // Two samples in the same 1° bucket (10.1° at 150mm, 10.9° at 90mm). The
    // nearest return (90mm) is the representative, not the first sample.
    let scans = [build_scan(&[(10.1, 150, 50), (10.9, 90, 10)])];

    let out = aggregate(&scans, &config(0.5, AggregationMethod::Median));

    assert_eq!(out.points[10].distance_mm, NonZeroU16::new(90));
    assert_eq!(out.points[10].intensity, 10);
}

#[test]
fn post_processing_valid_return_not_shadowed_by_no_return() {
    // A no-return (10.1°) precedes a valid 90mm return (10.9°) in the same 1°
    // bucket. The valid return must surface, not the no-return.
    let scans = [build_scan(&[(10.1, 0, 0), (10.9, 90, 10)])];

    let out = aggregate(&scans, &config(0.5, AggregationMethod::Median));

    assert_eq!(out.points[10].distance_mm, NonZeroU16::new(90));
    assert_eq!(out.points[10].intensity, 10);
}

#[test]
fn post_processing_counts_validity_once_per_revolution() {
    // One revolution contributes two samples to the same 1° bucket; the other
    // contributes none. Validity is keyed per revolution, so 1/2 = 0.5 < 0.75
    // and the bucket must drop — even though there are two valid *samples*.
    let scans = [build_scan(&[(10.1, 90, 10), (10.9, 150, 50)]), build_scan(&[])];

    let out = aggregate(&scans, &config(0.75, AggregationMethod::Median));

    assert_eq!(out.points[10].distance_mm, None);
}

#[test]
fn post_processing_tie_break_prefers_higher_intensity() {
    // Two samples in the same bucket at equal distance (100mm), with intensities
    // 5 and 90. The tie is broken by higher intensity.
    let scans = [build_scan(&[(10.1, 100, 5), (10.9, 100, 90)])];

    let out = aggregate(&scans, &config(0.5, AggregationMethod::Median));

    assert_eq!(out.points[10].distance_mm, NonZeroU16::new(100));
    assert_eq!(out.points[10].intensity, 90);
}
