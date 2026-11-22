//! Host-side integration tests for the `motor-cal` corrected verification.
//!
//! Every assertion is made through the crate's public API. The expected values
//! are the divergent count pairs and report shapes the Basic Motor Test produces,
//! never a re-run of the aggregation the module does.

use motor_cal::{
    Direction, POWER_STEP_PERCENTS, VERIFICATION_MIN_POWER, VERIFICATION_REPEATS, Verification, VerificationMax,
    divergence_percent, divergence_tenths,
};

/// The index of a Power Step in [`POWER_STEP_PERCENTS`], for readable assertions.
fn step(power: u8) -> usize {
    POWER_STEP_PERCENTS
        .iter()
        .position(|&percent| percent == power)
        .unwrap_or(POWER_STEP_PERCENTS.len())
}

/// Record `VERIFICATION_REPEATS` identical pairs at one point.
fn full_repeats(verification: &mut Verification, direction: Direction, power: u8, left: u16, right: u16) {
    for _ in 0..VERIFICATION_REPEATS {
        verification.record(direction, power, left, right);
    }
}

/// Fill every gated point with an identical pair, skipping the listed points so
/// the caller can record them differently.
fn fill_gated_zero(verification: &mut Verification, skip: &[(Direction, u8)]) {
    for direction in [Direction::Forward, Direction::Reverse] {
        for &power in &POWER_STEP_PERCENTS {
            if power < VERIFICATION_MIN_POWER || skip.contains(&(direction, power)) {
                continue;
            }
            full_repeats(verification, direction, power, 100, 100);
        }
    }
}

#[test]
fn two_repeats_average_and_the_worst_point_is_the_headline() {
    let mut verification = Verification::new();
    fill_gated_zero(&mut verification, &[(Direction::Forward, 50)]);

    // 10 % and 20 % at the same point average to 15 %.
    verification.record(Direction::Forward, 50, 100, 110);
    verification.record(Direction::Forward, 50, 100, 120);

    let report = verification.finish();
    assert!(report.complete);
    assert_eq!(
        report.max,
        Some(VerificationMax {
            tenths: 150,
            direction: Direction::Forward,
            power: 50,
        })
    );
    assert!(report.means[Direction::Forward.index()][step(50)].is_some_and(|mean| (mean - 15.0).abs() < 1e-3));
}

#[test]
fn a_gated_point_measured_once_makes_the_report_incomplete() {
    let mut verification = Verification::new();
    fill_gated_zero(&mut verification, &[(Direction::Reverse, 70)]);

    verification.record(Direction::Reverse, 70, 100, 150);

    let report = verification.finish();
    assert!(!report.complete);
    assert_eq!(report.max, None);
    // The single sample is still averaged and reported, but cannot headline.
    assert!(report.means[Direction::Reverse.index()][step(70)].is_some());
}

#[test]
fn invalid_and_zero_count_points_are_not_samples() {
    let mut verification = Verification::new();
    fill_gated_zero(&mut verification, &[(Direction::Forward, 40)]);

    // Zero counts on both sides: no ratio, so no sample.
    verification.record(Direction::Forward, 40, 0, 0);
    // One side zero: no ratio, so no sample.
    verification.record(Direction::Forward, 40, 100, 0);
    // Not a Power Step: ignored outright.
    verification.record(Direction::Forward, 45, 100, 120);

    let report = verification.finish();
    assert!(!report.complete);
    assert_eq!(report.max, None);
    assert_eq!(report.means[Direction::Forward.index()][step(40)], None);
}

#[test]
fn twenty_percent_is_informational_only() {
    let mut verification = Verification::new();
    fill_gated_zero(&mut verification, &[]);
    full_repeats(&mut verification, Direction::Forward, 20, 100, 300);

    let report = verification.finish();
    assert!(report.complete);
    assert!(report.low_power_means[Direction::Forward.index()].is_some_and(|mean| (mean - 200.0).abs() < 1e-3));
    assert_eq!(report.low_power_means[Direction::Reverse.index()], None);
    // Every gated mean is zero, so 20 % never becomes the headline.
    assert_eq!(
        report.max,
        Some(VerificationMax {
            tenths: 0,
            direction: Direction::Forward,
            power: 30,
        })
    );
    assert_eq!(report.means[Direction::Forward.index()][step(20)], None);
}

#[test]
fn the_max_is_chosen_before_rounding() {
    let mut verification = Verification::new();
    fill_gated_zero(&mut verification, &[(Direction::Forward, 40), (Direction::Reverse, 60)]);

    // 161 / 10 000 is 1.61 % and 164 / 10 000 is 1.64 %: both round to 16
    // tenths, but the raw means differ, and 1.64 % is the worse point.
    full_repeats(&mut verification, Direction::Forward, 40, 10_000, 10_161);
    full_repeats(&mut verification, Direction::Reverse, 60, 10_000, 10_164);

    let report = verification.finish();
    assert!(report.complete);
    assert_eq!(
        report.max,
        Some(VerificationMax {
            tenths: 16,
            direction: Direction::Reverse,
            power: 60,
        })
    );
}

#[test]
fn equal_counts_have_no_divergence() {
    assert!(divergence_percent(123, 123).is_some_and(|divergence| divergence.abs() < 1e-6));
}

#[test]
fn a_zero_count_has_no_divergence() {
    assert!(divergence_percent(0, 0).is_none());
    assert!(divergence_percent(0, 5).is_none());
    assert!(divergence_percent(5, 0).is_none());
}

#[test]
fn tenths_round_and_saturate() {
    assert_eq!(divergence_tenths(0.0), 0);
    assert_eq!(divergence_tenths(1.61), 16);
    assert_eq!(divergence_tenths(1.66), 17);
    assert_eq!(divergence_tenths(100.0), 1_000);
    assert_eq!(divergence_tenths(7_000.0), u16::MAX);
    assert_eq!(divergence_tenths(f32::MAX), u16::MAX);
}

#[test]
fn no_records_is_incomplete_and_has_no_max() {
    let report = Verification::new().finish();

    assert!(!report.complete);
    assert_eq!(report.max, None);
    assert!(report.means.iter().flatten().all(Option::is_none));
    assert_eq!(report.low_power_means, [None, None]);
}
