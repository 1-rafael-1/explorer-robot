//! Pure pack-voltage to red/green duty mapping for the RGB indicator LED.
//!
//! The firmware's `rgb_led_indicate` task owns the PIO channels; this crate
//! owns only the colour curve, so it has no `embassy` or hardware dependency
//! and the host tests can pin the anchors directly.
//!
//! The blend is continuous and monotonic: pure green at or above
//! [`GREEN_VOLTAGE`], a yellow mix at [`YELLOW_VOLTAGE`], pure red at or below
//! [`RED_VOLTAGE`]. Green is perceptually brighter than red on this LED, so
//! [`MIX_NUDGE`] biases the split toward red; the returned duties are
//! normalized to sum to 100.

#![no_std]
#![warn(missing_docs)]

/// Pack voltage at or above which the indicator is pure green.
///
/// These anchors are indicator thresholds, deliberately independent of the
/// 6.0–8.4 V charge map: green holds above [`GREEN_VOLTAGE`], and
/// [`RED_VOLTAGE`] sits above the pack's 6.0 V cutoff so the operator sees red
/// before the pack is cut off.
pub const GREEN_VOLTAGE: f32 = 7.7;

/// Pack voltage at which the green→red blend reads as yellow.
pub const YELLOW_VOLTAGE: f32 = 6.7;

/// Pack voltage at or below which the indicator is pure red.
pub const RED_VOLTAGE: f32 = 6.2;

/// How far the red/green blend is biased toward red (`0.0` = neutral).
///
/// Green is perceptually brighter than red on this LED even with its larger
/// series resistor, so a neutral cross-fade stays green-looking well past the
/// midpoint and then snaps to red. `0.6` puts yellow at about a 70/30
/// red/green split, which is the split that read as yellow on the bench.
pub const MIX_NUDGE: f32 = 0.6;

// The shaping exponent is only finite while the anchors are strictly ordered,
// and red must appear before the pack's 6.0 V cutoff.
const _: () = assert!(6.0 < RED_VOLTAGE && RED_VOLTAGE < YELLOW_VOLTAGE && YELLOW_VOLTAGE < GREEN_VOLTAGE);

/// Duty cycles for the indicator's red and green channels (0–100).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RgbDuty {
    /// Red channel duty cycle.
    pub red: u8,
    /// Green channel duty cycle.
    pub green: u8,
}

/// Smoothstep, `0.0` at or below `0.0` and `1.0` at or above `1.0`.
///
/// `t` must be in `0.0..=1.0`. Its slope is zero at both ends, so the tint
/// starts and stops without a visible step.
fn smoothstep(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

/// Maps a pack voltage to red/green PWM duty cycles.
///
/// The blend is continuous and monotonic: pure green at or above
/// [`GREEN_VOLTAGE`], a yellow mix at [`YELLOW_VOLTAGE`], pure red at or below
/// [`RED_VOLTAGE`]. The returned duties sum to 100, so the mix changes hue
/// without brightening or dimming the LED in the middle of the range.
#[must_use]
pub fn battery_color(voltage: f32) -> RgbDuty {
    // 0 at the green anchor, 1 at the red anchor.
    let t = ((GREEN_VOLTAGE - voltage) / (GREEN_VOLTAGE - RED_VOLTAGE)).clamp(0.0, 1.0);

    // Ease the ramp so its slope is zero at the green edge, which keeps the
    // tint from switching on abruptly, and shape it so progress is 0.5 at the
    // yellow anchor.
    let eased = smoothstep(t);
    let yellow_eased = smoothstep((GREEN_VOLTAGE - YELLOW_VOLTAGE) / (GREEN_VOLTAGE - RED_VOLTAGE));
    let exponent = core::f32::consts::LN_2 / -libm::logf(yellow_eased);
    let progress = libm::powf(eased, exponent);

    // Bias red ahead of green, then normalize so both channels always sum to
    // full duty instead of bulging in the middle.
    let red = libm::powf(progress, 1.0 - MIX_NUDGE);
    let green = libm::powf(1.0 - progress, 1.0 + MIX_NUDGE);
    let total = red + green;

    RgbDuty {
        red: duty(red / total),
        green: duty(green / total),
    }
}

/// Converts a `0.0..=1.0` fraction to a 0–100 duty cycle.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn duty(fraction: f32) -> u8 {
    libm::roundf(fraction * 100.0) as u8
}
