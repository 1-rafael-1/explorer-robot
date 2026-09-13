//! Host-side integration tests for the [`pressure`] proxy.
//!
//! Each case pins a literal `(z1, z2, x_plate_resistance)` input to a literal
//! output, covering the valid-ratio branch and every `None` guard.

use touch_async::pressure;

#[test]
fn pressure_scales_with_the_z_difference() {
    assert_eq!(pressure(100, 300, 400), Some(800));
}

#[test]
fn pressure_is_linear_in_the_z_difference() {
    assert_eq!(pressure(100, 200, 400), Some(400));
}

#[test]
fn pressure_rejects_equal_z_readings() {
    assert_eq!(pressure(300, 300, 400), None);
}

#[test]
fn pressure_rejects_z2_below_z1() {
    assert_eq!(pressure(300, 100, 400), None);
}

#[test]
fn pressure_rejects_zero_z1() {
    assert_eq!(pressure(0, 300, 400), None);
}

#[test]
fn pressure_uses_the_full_z_range_without_overflow() {
    assert_eq!(pressure(1, 4095, 1000), Some(4_094_000));
}
