//! Host-side integration tests for the post-processing stage.
//!
//! These exercise [`coin_d6::aggregate`]'s validity gate and median/mean
//! reducers, plus [`coin_d6::angle_correction_deg`]'s sign and magnitude.

use coin_d6::{AggregationConfig, AggregationMethod, Point, Scan, aggregate, angle_correction_deg};

/// Build a [`Scan`] from a compact list of `(angle_deg, distance_mm, intensity)`
/// triples; the scan's `len` is set to the number of triples and the remaining
/// points stay zeroed.
fn build_scan(points: &[(f32, u16, u8)]) -> Scan<400> {
    let mut scan = Scan::<400>::new();
    scan.len = points.len();
    for (i, &(angle_deg, distance_mm, intensity)) in points.iter().enumerate() {
        scan.points[i] = Point {
            angle_deg,
            distance_mm,
            intensity,
        };
    }
    scan
}

/// Build an [`AggregationConfig`] with the given parameters.
fn config(spins: usize, validity_ratio: f32, method: AggregationMethod) -> AggregationConfig {
    AggregationConfig {
        spins,
        validity_ratio,
        method,
    }
}

#[test]
fn post_processing_validity_gate_emits_no_return_below_ratio() {
    let scans = [
        build_scan(&[(0.0, 100, 10)]),
        build_scan(&[(0.0, 200, 20)]),
        build_scan(&[(0.0, 0, 0)]),
        build_scan(&[(0.0, 0, 0)]),
        build_scan(&[(0.0, 0, 0)]),
    ];

    let out = aggregate(&scans, &config(5, 0.5, AggregationMethod::Median));

    // 2/5 valid = 0.4 < 0.5, so the point is dropped.
    assert_eq!(out.len, 1);
    assert_eq!(out.points[0].distance_mm, 0);
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

    let out = aggregate(&scans, &config(5, 0.5, AggregationMethod::Median));

    // 3/5 valid = 0.6 >= 0.5, so the point survives with the median.
    assert_eq!(out.len, 1);
    assert_eq!(out.points[0].distance_mm, 200);
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

    let median_out = aggregate(&scans, &config(3, 0.5, AggregationMethod::Median));
    assert_eq!(median_out.points[0].distance_mm, 10);
    assert_eq!(median_out.points[0].intensity, 1);

    let mean_out = aggregate(&scans, &config(3, 0.5, AggregationMethod::Mean));
    // (10 + 10 + 40) / 3 == 20; (1 + 1 + 7) / 3 == 3.
    assert_eq!(mean_out.points[0].distance_mm, 20);
    assert_eq!(mean_out.points[0].intensity, 3);
}

#[test]
fn post_processing_all_no_return_yields_no_return() {
    let scans = [
        build_scan(&[(0.0, 0, 0)]),
        build_scan(&[(0.0, 0, 0)]),
        build_scan(&[(0.0, 0, 0)]),
    ];

    let out = aggregate(&scans, &config(3, 0.5, AggregationMethod::Median));

    assert_eq!(out.len, 1);
    assert_eq!(out.points[0].distance_mm, 0);
    assert_eq!(out.points[0].intensity, 0);
}

#[test]
fn post_processing_aggregate_aligns_by_index_and_uses_max_width() {
    // First scan has two points, second has one; the second scan's missing
    // trailing point is treated as a no-return.
    let scans = [
        build_scan(&[(0.0, 100, 10), (0.9, 200, 20)]),
        build_scan(&[(0.0, 150, 15)]),
    ];

    let out = aggregate(&scans, &config(2, 0.5, AggregationMethod::Median));

    // Width is the longest scan's length.
    assert_eq!(out.len, 2);
    // Index 0: both scans valid -> median of [100, 150].
    assert_eq!(out.points[0].distance_mm, 125);
    // Index 1: only the first scan is valid (1/2 = 0.5, kept) -> 200.
    assert_eq!(out.points[1].distance_mm, 200);
    // Angles come from the first scan that reaches each index.
    assert_eq!(out.points[0].angle_deg, 0.0);
    assert_eq!(out.points[1].angle_deg, 0.9);
}

#[test]
fn post_processing_angle_correction_matches_known_distances() {
    // Positive at 1000 mm.
    assert!((angle_correction_deg(1000) - 0.191_017).abs() < 1e-4);
    // Negative at 50 mm.
    assert!((angle_correction_deg(50) - (-0.169_037)).abs() < 1e-4);
}

#[test]
fn post_processing_angle_correction_crosses_zero_at_the_zero_distance() {
    // The zero-distance constant is 90.15 mm, which a `u16` distance cannot
    // represent exactly; the nearest integers must therefore straddle zero.
    assert!(angle_correction_deg(90) < 0.0);
    assert!(angle_correction_deg(91) > 0.0);
    // Both are tiny compared with the correction's ~0.19° peak magnitude.
    assert!((angle_correction_deg(90)).abs() < 0.01);
    assert!((angle_correction_deg(91)).abs() < 0.01);
}
