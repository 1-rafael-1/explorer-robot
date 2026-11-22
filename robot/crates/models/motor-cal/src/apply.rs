//! Pure `SetTracks` resolution: the Motor Calibration Curve, voltage
//! compensation, a single rounding and the output clamp.
//!
//! The hardware pins, the left-track inversion and the Motion State publication
//! stay in the driver task. This module only decides the per-mille duty that one
//! requested track speed becomes.

use crate::curve_maths::{Curve, Direction, Track};

/// Resolve one requested track speed to the per-mille duty the driver applies.
///
/// The order is fixed: zero stays zero, a magnitude below the Calibration Floor
/// is passed through raw, otherwise the curve resolves the command to the nearest
/// per-mille (0.1 %) for (track, direction, magnitude), clamped to `0..=1000`.
/// Voltage compensation is applied to that per-mille value, and the compensated
/// duty is rounded once to the nearest per-mille for
/// `set_duty_cycle_fraction(_, 1000)`, then clamped to `-1000..=1000` with the
/// original sign. There are therefore two quantisations: the curve's resolution to
/// the nearest per-mille, then the compensated duty's rounding to the nearest
/// per-mille.
///
/// An absent curve is the identity: the raw magnitude goes straight to voltage
/// compensation, so a missing or failed calibration never blocks driving.
#[must_use]
// Every cast is bounded before it runs: the curve's per-mille is a `u16` of at
// most 1000, the signed value is therefore at most 1000 in magnitude, and the
// final `as i16` only runs after `clamp` has bounded the result to
// `-1000..=1000`. The `as f32` on an integer no larger than 1000 is exact, and
// the `as i32` truncation is the intended half-away-from-zero rounding.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
pub fn resolve_track_speed(curve: Option<&Curve>, track: Track, speed: i8, voltage_comp: f32) -> i16 {
    let magnitude = speed.unsigned_abs();
    let direction = if speed < 0 {
        Direction::Reverse
    } else {
        Direction::Forward
    };
    let per_mille = curve.map_or_else(
        || u16::from(magnitude) * 10,
        |curve| curve.resolve(track, direction, magnitude).per_mille(),
    );

    let promised = i32::from(per_mille);
    let signed = if speed < 0 { -promised } else { promised };
    let compensated = signed as f32 * voltage_comp;
    // Round half away from zero exactly once, after compensation.
    let rounded = if compensated >= 0.0 {
        (compensated + 0.5) as i32
    } else {
        (compensated - 0.5) as i32
    };
    rounded.clamp(-1000, 1000) as i16
}
