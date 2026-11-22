//! Host-side integration tests for the `motor-cal` Distance Calibration scale.
//!
//! Every assertion is made through the crate's public API. The expected values
//! are worked examples from `.scratch/motor-calibration/spec.md` and the ADR —
//! literal counts per centimetre and counts per revolution — never a re-run of
//! the arithmetic the crate does.

use motor_cal::{
    DistanceScale, MAX_PLAUSIBLE_COUNTS_PER_CM, MIN_PLAUSIBLE_COUNTS_PER_CM, NOMINAL_COUNTS_PER_CM,
    SPROCKET_CIRCUMFERENCE_CM, ScaleError,
};

/// The nominal Distance Calibration drive, in centimetres.
const NOMINAL_DRIVE_CM: f32 = 150.0;

/// Assert a measured value is the worked-example `expected` value.
fn assert_close(actual: f32, expected: f32) {
    assert!((actual - expected).abs() < 1e-3, "expected {expected}, got {actual}");
}

#[test]
fn counts_per_cm_comes_from_the_accumulated_counts_and_the_measured_distance() {
    // 2 604 counts over a tape-measured 150 cm is 17.36 counts/cm.
    let scale = DistanceScale::from_measurement(2_604, NOMINAL_DRIVE_CM);

    assert!(scale.is_ok());
    if let Ok(scale) = scale {
        assert_close(scale.counts_per_cm(), 17.36);
    }
}

#[test]
fn counts_per_sprocket_revolution_follows_from_the_scale_and_the_circumference() {
    // The hardware implies 330 counts per sprocket revolution. At the nominal
    // 19.01 cm circumference that is 17.36 counts/cm, and the derivation must
    // return to 330.
    let scale = DistanceScale::from_measurement(330, SPROCKET_CIRCUMFERENCE_CM);

    assert!(scale.is_ok());
    if let Ok(scale) = scale {
        assert_close(scale.counts_per_cm(), 17.359);
        let counts_per_rev = scale.counts_per_rev(SPROCKET_CIRCUMFERENCE_CM);
        assert!(counts_per_rev.is_some());
        if let Some(counts_per_rev) = counts_per_rev {
            assert_close(counts_per_rev, 330.0);
        }
    }
}

#[test]
fn a_scale_inside_the_band_can_be_built_from_its_counts_per_cm() {
    assert!(DistanceScale::from_counts_per_cm(MIN_PLAUSIBLE_COUNTS_PER_CM).is_ok());
    assert!(DistanceScale::from_counts_per_cm(MAX_PLAUSIBLE_COUNTS_PER_CM).is_ok());

    let scale = DistanceScale::from_counts_per_cm(17.36);
    assert!(scale.is_ok());
    if let Ok(scale) = scale {
        assert_close(scale.counts_per_cm(), 17.36);
    }
}

#[test]
fn a_zero_or_negative_measured_distance_is_rejected_instead_of_dividing_by_zero() {
    assert_eq!(DistanceScale::from_measurement(2_604, 0.0), Err(ScaleError::NoDistance));
    assert_eq!(
        DistanceScale::from_measurement(2_604, -NOMINAL_DRIVE_CM),
        Err(ScaleError::NoDistance)
    );
    assert_eq!(
        DistanceScale::from_measurement(2_604, f32::NAN),
        Err(ScaleError::NoDistance)
    );
    assert_eq!(
        DistanceScale::from_measurement(2_604, f32::INFINITY),
        Err(ScaleError::NoDistance)
    );
}

#[test]
fn a_scale_below_the_plausibility_band_asks_for_a_rerun() {
    // Exactly 5.00 counts/cm is inside the inclusive band; 4.99 is not.
    let boundary = DistanceScale::from_measurement(1_000, 200.0);
    assert!(boundary.is_ok());
    if let Ok(boundary) = boundary {
        assert_close(boundary.counts_per_cm(), MIN_PLAUSIBLE_COUNTS_PER_CM);
    }
    assert_eq!(
        DistanceScale::from_measurement(998, 200.0),
        Err(ScaleError::Implausible { counts_per_cm: 4.99 })
    );
}

#[test]
fn a_scale_above_the_plausibility_band_asks_for_a_rerun() {
    // Exactly 100.00 counts/cm is inside the inclusive band; 100.5 is not.
    assert!(DistanceScale::from_measurement(2_000, 20.0).is_ok());
    assert_eq!(
        DistanceScale::from_measurement(2_010, 20.0),
        Err(ScaleError::Implausible { counts_per_cm: 100.5 })
    );
}

#[test]
fn zero_accumulated_counts_are_implausible_rather_than_a_silent_zero() {
    assert_eq!(
        DistanceScale::from_measurement(0, NOMINAL_DRIVE_CM),
        Err(ScaleError::Implausible { counts_per_cm: 0.0 })
    );
}

#[test]
fn the_nominal_fallback_is_the_inferred_counts_per_sprocket_revolution_over_the_circumference() {
    // The hardware inference is 11 pulses per motor revolution behind a 30:1
    // gearbox: 330 counts per sprocket revolution. Over the nominal 19.01 cm
    // circumference that is 17.36 counts/cm, the figure the drive path falls back
    // to before the first Distance Calibration.
    assert_close(NOMINAL_COUNTS_PER_CM, 17.36);
    assert_close(NOMINAL_COUNTS_PER_CM * SPROCKET_CIRCUMFERENCE_CM, 330.0);

    let scale = DistanceScale::nominal();
    assert_close(scale.counts_per_cm(), NOMINAL_COUNTS_PER_CM);
    let counts_per_rev = scale.counts_per_rev(SPROCKET_CIRCUMFERENCE_CM).unwrap_or(0.0);
    assert_close(counts_per_rev, 330.0);

    // A fallback must itself be a usable scale: the plausibility band rejects
    // anything else, and the nominal plan is inside it.
    assert!(DistanceScale::from_counts_per_cm(NOMINAL_COUNTS_PER_CM).is_ok());
}

#[test]
fn a_spocket_with_no_circumference_has_no_counts_per_revolution() {
    let scale = DistanceScale::from_counts_per_cm(17.36);

    assert!(scale.is_ok());
    if let Ok(scale) = scale {
        assert_eq!(scale.counts_per_rev(0.0), None);
        assert_eq!(scale.counts_per_rev(-SPROCKET_CIRCUMFERENCE_CM), None);
        assert_eq!(scale.counts_per_rev(f32::NAN), None);
    }
}

#[test]
fn scale_failures_name_themselves_for_the_log() {
    assert_eq!(ScaleError::NoDistance.label(), "no measured distance");
    assert_eq!(
        ScaleError::Implausible { counts_per_cm: 4.99 }.label(),
        "implausible counts/cm - re-run"
    );
}
