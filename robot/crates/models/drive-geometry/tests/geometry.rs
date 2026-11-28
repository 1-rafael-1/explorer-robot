//! Host-side tests pinning the shared drive geometry.
//!
//! Both constants are physical measurements. The assertions compare raw bits so
//! an accidental edit to either figure fails loudly instead of drifting the
//! Distance Calibration scale and the curve geometry apart.

use drive_geometry::{SPROCKET_CIRCUMFERENCE_CM, TRACK_WIDTH_CM};

#[test]
fn track_width_is_the_measured_centerline_distance() {
    assert_eq!(TRACK_WIDTH_CM.to_bits(), 14.5_f32.to_bits());
}

#[test]
fn sprocket_circumference_is_the_effective_circumference_with_the_track_installed() {
    assert_eq!(SPROCKET_CIRCUMFERENCE_CM.to_bits(), 19.01_f32.to_bits());
}
