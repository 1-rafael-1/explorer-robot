//! Pure, I/O-free pressure proxy computed from the Z1/Z2 conversions.

/// Estimate a position-independent touch-resistance/pressure-like proxy.
///
/// The full TI TSC2046 touch-resistance equation additionally scales by the
/// X-position fraction (`x / 4096`); this function deliberately omits that
/// factor and returns a position-independent resistance proxy that is enough to
/// distinguish a light from a firm press. Despite the name it is a proxy, not a
/// calibrated physical pressure.
///
/// `x_plate_resistance_ohms` is the known resistance across the X plate, in
/// ohms. A `u16` covers every realistic resistive panel; because both operands
/// are 16-bit, the intermediate product cannot wrap even before it is widened.
/// The maths is done in `u64` regardless, and the quotient is bounded by
/// `u16::MAX * (u16::MAX - 1)`, which fits the `u32` return type.
///
/// Returns `None` when there is no valid pressure reading: `z1 == 0` (no touch,
/// so there is no current path through the touch layer) or `z2 <= z1` (Z2 did
/// not exceed Z1, so the ratio would be zero or nonsensical).
///
/// This is pure and unfiltered. It applies no thresholding, averaging, or
/// smoothing; the caller owns both.
#[must_use]
pub fn pressure(z1: u16, z2: u16, x_plate_resistance_ohms: u16) -> Option<u32> {
    if z1 == 0 || z2 <= z1 {
        None
    } else {
        let resistance = u64::from(x_plate_resistance_ohms) * u64::from(z2 - z1) / u64::from(z1);
        // The quotient is provably in `u32` range for `u16` inputs; the fallback
        // only documents that guarantee without introducing a panic path.
        Some(u32::try_from(resistance).unwrap_or(u32::MAX))
    }
}
