//! Host-side tests pinning the indicator's colour anchors and shape.
//!
//! The `std` feature exists only to enable this harness; the crate itself is
//! always `no_std`.

use indicator_led::{GREEN_VOLTAGE, RED_VOLTAGE, YELLOW_VOLTAGE, battery_color};

/// Green holds, with no red, from the green anchor upward.
#[test]
fn green_holds_at_and_above_the_green_anchor() {
    for millivolts in 7_700_i16..=8_400_i16 {
        let voltage = f32::from(millivolts) / 1_000.0;
        let color = battery_color(voltage);
        assert_eq!(color.green, 100, "green at {voltage} V");
        assert_eq!(color.red, 0, "no red at {voltage} V");
    }
}

/// The yellow anchor is the documented red-dominant split.
#[test]
fn yellow_anchor_uses_the_biased_split() {
    let color = battery_color(YELLOW_VOLTAGE);
    assert!(
        (69..=71).contains(&color.red),
        "red at the yellow anchor: {}",
        color.red
    );
    assert!(
        (29..=31).contains(&color.green),
        "green at the yellow anchor: {}",
        color.green
    );
}

/// Red holds, with no green, from the red anchor downward.
#[test]
fn red_holds_at_and_below_the_red_anchor() {
    let at_anchor = battery_color(RED_VOLTAGE);
    assert_eq!(at_anchor.red, 100, "red at the red anchor");
    assert_eq!(at_anchor.green, 0, "no green at the red anchor");

    for millivolts in 6_000_i16..6_200_i16 {
        let voltage = f32::from(millivolts) / 1_000.0;
        let color = battery_color(voltage);
        assert_eq!(color.red, 100, "red at {voltage} V");
        assert_eq!(color.green, 0, "no green at {voltage} V");
    }
}

/// Both channels move monotonically and keep the total duty at 100.
#[test]
fn blend_is_monotonic_and_normalized() {
    let mut previous = battery_color(GREEN_VOLTAGE);
    for millivolts in (6_000_i16..=8_400_i16).rev() {
        let voltage = f32::from(millivolts) / 1_000.0;
        let color = battery_color(voltage);

        assert!(color.red >= previous.red, "red must not fall at {voltage} V");
        assert!(color.green <= previous.green, "green must not rise at {voltage} V");

        let total = u16::from(color.red) + u16::from(color.green);
        assert!((99..=101).contains(&total), "total duty {total} at {voltage} V");

        previous = color;
    }
}
