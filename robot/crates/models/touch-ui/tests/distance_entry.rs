//! Host-side test for the Distance Calibration value-entry range.
//!
//! The range belongs to the model, not the glass: the calibration's first run
//! happens on a robot whose encoder scale is still the retired datasheet-derived
//! one, so it can travel roughly three times the 150 cm nominal leg. The screen
//! must therefore accept the operator's measurement instead of clamping it.

use touch_ui::{
    ValueFlow,
    geometry::{FB_W, SLIDER_MARGIN, slider_value},
};

/// The left end of the slider's value mapping, in pixels.
fn slider_left() -> i32 {
    i32::try_from(SLIDER_MARGIN).unwrap_or(0)
}

/// The right end of the slider's value mapping, in pixels.
fn slider_right() -> i32 {
    i32::try_from(FB_W).unwrap_or(0) - i32::try_from(SLIDER_MARGIN).unwrap_or(0)
}

#[test]
fn distance_calibration_accepts_a_three_times_overshoot() {
    let flow = ValueFlow::DistanceCalibration;

    assert_eq!(flow.min(), 0);
    assert_eq!(flow.max(), 500);
    assert_eq!(flow.step(), 1);
    assert_eq!(flow.preset(), 150);
    assert_eq!(flow.unit(), "cm");
}

#[test]
fn the_slider_reaches_both_ends_of_the_widened_range() {
    let flow = ValueFlow::DistanceCalibration;

    assert_eq!(slider_value(flow, slider_left() - 10), 0);
    assert_eq!(slider_value(flow, slider_left()), 0);
    assert_eq!(slider_value(flow, slider_right()), 500);
    assert_eq!(slider_value(flow, slider_right() + 10), 500);

    let middle = i32::midpoint(slider_left(), slider_right());
    assert_eq!(slider_value(flow, middle), 250);
}

#[test]
fn the_slider_can_express_a_distance_above_the_old_ceiling() {
    let flow = ValueFlow::DistanceCalibration;
    let width = slider_right() - slider_left();
    let three_quarters = slider_left() + width * 3 / 4;
    let value = slider_value(flow, three_quarters);

    assert!(
        value > 200,
        "the old ceiling must not still clamp the slider: got {value}"
    );
    assert!(value <= 500);
}
