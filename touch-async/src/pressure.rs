//! Pure, I/O-free pressure proxy computed from the Z1/Z2 conversions.

/// Estimate a position-independent touch-resistance/pressure-like proxy.
///
/// The full TI TSC2046 touch-resistance equation additionally scales by the
/// X-position fraction (`x / 4096`); this function deliberately omits that
/// factor and returns a position-independent resistance proxy that is enough to
/// distinguish a light from a firm press. `x_plate_resistance` is the known
/// resistance across the X plate.
///
/// Returns `None` when there is no valid pressure reading: `z1 == 0` (no touch,
/// so there is no current path through the touch layer) or `z2 <= z1` (Z2 did
/// not exceed Z1, so the ratio would be zero or nonsensical).
///
/// This is pure and unfiltered. It applies no thresholding, averaging, or
/// smoothing; the caller owns both.
#[must_use]
pub fn pressure(z1: u16, z2: u16, x_plate_resistance: u32) -> Option<u32> {
    if z1 == 0 || z2 <= z1 {
        None
    } else {
        Some(x_plate_resistance * u32::from(z2 - z1) / u32::from(z1))
    }
}
