//! Host-side integration tests for [`Calibration`]'s raw-to-pixel mapping.
//!
//! Every case pairs a literal raw sample with the literal pixel it must map to,
//! so the tests pin the exact integer arithmetic (including clamping) rather
//! than re-deriving it.

use embedded_graphics::geometry::Point;
use touch_async::{Calibration, CalibrationError, TouchSample};

/// Build a [`TouchSample`] from literal raw coordinates; the Z channels are
/// irrelevant to pixel mapping and are set to arbitrary non-zero values.
const fn sample(x: u16, y: u16) -> TouchSample {
    TouchSample { x, y, z1: 1, z2: 2 }
}

#[test]
fn calibration_reference_maps_first_corner_to_origin() {
    assert_eq!(Calibration::REFERENCE.to_pixels(sample(3880, 262)), Point::new(0, 0));
}

#[test]
fn calibration_reference_maps_last_corner_to_bottom_right() {
    assert_eq!(
        Calibration::REFERENCE.to_pixels(sample(340, 3850)),
        Point::new(319, 239)
    );
}

#[test]
fn calibration_reference_maps_centre_to_middle_pixel() {
    assert_eq!(
        Calibration::REFERENCE.to_pixels(sample(2110, 2056)),
        Point::new(159, 119)
    );
}

#[test]
fn calibration_reference_clamps_samples_outside_the_corners() {
    assert_eq!(Calibration::REFERENCE.to_pixels(sample(0, 4095)), Point::new(319, 239));
}

#[test]
fn calibration_default_is_the_reference() {
    assert_eq!(Calibration::default(), Calibration::REFERENCE);
}

#[test]
fn calibration_measured_maps_its_own_endpoints_to_the_corners() {
    assert_eq!(Calibration::MEASURED.to_pixels(sample(3810, 276)), Point::new(0, 0));
    assert_eq!(Calibration::MEASURED.to_pixels(sample(160, 3844)), Point::new(319, 239));
}

#[test]
fn calibration_measured_maps_the_bring_up_targets_to_their_coordinates() {
    // Raw readings captured by touching the four 12 px-inset corner targets,
    // paired with those targets' known framebuffer coordinates. A pixel of slack
    // absorbs the per-touch jitter in the raw averages.
    let measured = Calibration::MEASURED;
    let cases = [
        (3663, 455, 12, 12),
        (295, 469, 307, 12),
        (299, 3679, 307, 227),
        (3683, 3651, 12, 227),
    ];
    for (raw_x, raw_y, fb_x, fb_y) in cases {
        let point = measured.to_pixels(sample(raw_x, raw_y));
        assert!(
            (point.x - fb_x).abs() <= 2 && (point.y - fb_y).abs() <= 2,
            "({raw_x}, {raw_y}) mapped to {point:?}, expected ~({fb_x}, {fb_y})"
        );
    }
}

#[test]
fn mirrored_x_swaps_the_raw_x_endpoints_only() {
    let mirrored = Calibration::MEASURED.mirrored_x();
    assert_eq!(mirrored, Calibration::new(160, 3810, 276, 3844, 320, 240));
    // The mirrored X axis maps the swapped endpoints back onto the same corners,
    // leaving the Y mapping untouched.
    assert_eq!(mirrored.to_pixels(sample(160, 276)), Point::new(0, 0));
    assert_eq!(mirrored.to_pixels(sample(3810, 3844)), Point::new(319, 239));
}

#[test]
fn try_new_accepts_a_valid_calibration() {
    assert_eq!(
        Calibration::try_new(3880, 340, 262, 3850, 320, 240),
        Ok(Calibration::REFERENCE)
    );
}

#[test]
fn try_new_rejects_a_non_positive_dimension() {
    assert_eq!(
        Calibration::try_new(3880, 340, 262, 3850, 0, 240),
        Err(CalibrationError::NonPositiveDimension)
    );
    assert_eq!(
        Calibration::try_new(3880, 340, 262, 3850, 320, 0),
        Err(CalibrationError::NonPositiveDimension)
    );
}

#[test]
fn try_new_rejects_a_degenerate_axis() {
    assert_eq!(
        Calibration::try_new(3880, 3880, 262, 3850, 320, 240),
        Err(CalibrationError::DegenerateAxis)
    );
    assert_eq!(
        Calibration::try_new(3880, 340, 262, 262, 320, 240),
        Err(CalibrationError::DegenerateAxis)
    );
}

#[test]
#[should_panic(expected = "raw endpoints must differ")]
fn new_panics_on_a_degenerate_axis() {
    let _ = Calibration::new(3880, 3880, 262, 3850, 320, 240);
}
