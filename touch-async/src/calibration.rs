//! Pure, I/O-free conversion of raw touch samples into screen pixels.

use embedded_graphics::geometry::Point;

use crate::types::TouchSample;

/// Why a [`Calibration`] could not be built.
///
/// Returned by [`Calibration::try_new`]; [`Calibration::new`] panics on the same
/// checks instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationError {
    /// `width` or `height` is less than 1, so there is no pixel grid to map onto.
    NonPositiveDimension,
    /// An axis's two raw endpoints are equal (`x1 == x2` or `y1 == y2`), so the
    /// axis has no span and interpolation would divide by zero.
    DegenerateAxis,
}

/// Two-point panel calibration and the target pixel dimensions.
///
/// Each axis is described by two raw endpoints, and
/// [`to_pixels`](Self::to_pixels) linearly interpolates any raw conversion onto
/// the pixel grid `0..width` × `0..height`. The raw endpoints are not assumed to
/// be ordered: an inverted axis simply has `x1 > x2` (or `y1 > y2`) and the
/// interpolation still lands on the correct pixel.
///
/// The fields are private because the interpolation carries real invariants:
/// both dimensions must be at least 1, and each axis's raw endpoints must
/// differ. Build a value with [`new`](Self::new) (const, panics on a violation)
/// or [`try_new`](Self::try_new) (fallible).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// Raw X conversion measured at the screen's left edge (pixel column `0`).
    x1: i32,
    /// Raw X conversion measured at the screen's right edge (pixel column `width - 1`).
    x2: i32,
    /// Raw Y conversion measured at the screen's top edge (pixel row `0`).
    y1: i32,
    /// Raw Y conversion measured at the screen's bottom edge (pixel row `height - 1`).
    y2: i32,
    /// Screen width in pixels; at least 1.
    width: i32,
    /// Screen height in pixels; at least 1.
    height: i32,
}

impl Calibration {
    /// Reference calibration values for a 320 × 240 panel.
    ///
    /// # UNCALIBRATED REFERENCE VALUES — NOT OUR PANEL'S CALIBRATION
    ///
    /// These numbers are transcribed verbatim from the Waveshare
    /// Pico-ResTouch-LCD-2.8 reference driver. They describe *that vendor's*
    /// board, not the panel wired to this robot. They exist only so that the
    /// pixel mapping can be exercised end-to-end before bring-up, and they are
    /// what [`Default`] returns. They **must** be re-measured on our panel's four
    /// corners during bring-up and must not be treated as correct for our
    /// hardware; this panel's measured values live in [`MEASURED`](Self::MEASURED).
    pub const REFERENCE: Self = Self::new(3880, 340, 262, 3850, 320, 240);

    /// Calibration measured on this project's 2.8″ panel during bring-up.
    ///
    /// The four `touch_coexistence` corner targets were touched in turn, and the
    /// raw readings at those known framebuffer coordinates (inset 12 px from the
    /// edges) were extrapolated out to the real
    /// corners. Unlike [`REFERENCE`](Self::REFERENCE), these numbers describe the
    /// panel actually wired to this robot, in the display orientation the
    /// coexistence example uses: raw X falls as the pixel column rises, and raw Y
    /// rises as the pixel row rises.
    pub const MEASURED: Self = Self::new(3810, 160, 276, 3844, 320, 240);

    /// This calibration for a display orientation that is [`MEASURED`](Self::MEASURED)'s
    /// orientation plus a horizontal mirror.
    ///
    /// A horizontal mirror reverses the on-screen X axis, so the two raw X
    /// endpoints swap and the Y endpoints and dimensions are unchanged. The
    /// `touch_menu` example, which displays with `Orientation::Deg90` alone, uses
    /// this rather than restating the measured numbers, so a re-measurement of
    /// [`MEASURED`](Self::MEASURED) flows through instead of drifting.
    #[must_use]
    pub const fn mirrored_x(self) -> Self {
        Self::new(self.x2, self.x1, self.y1, self.y2, self.width, self.height)
    }

    /// Create a calibration from its two raw endpoints per axis and the target
    /// pixel dimensions.
    ///
    /// # Panics
    ///
    /// Panics if `width` or `height` is less than 1, or if either axis's raw
    /// endpoints are equal (`x1 == x2` or `y1 == y2`). Use
    /// [`try_new`](Self::try_new) to reject such values instead of panicking.
    #[must_use]
    pub const fn new(x1: i32, x2: i32, y1: i32, y2: i32, width: i32, height: i32) -> Self {
        assert!(
            Self::validate(x1, x2, y1, y2, width, height).is_ok(),
            "calibration dimensions must be positive and each axis's raw endpoints must differ"
        );
        Self {
            x1,
            x2,
            y1,
            y2,
            width,
            height,
        }
    }

    /// Create a calibration, reporting an invalid value instead of panicking.
    ///
    /// # Errors
    ///
    /// Returns [`CalibrationError::NonPositiveDimension`] if `width` or `height`
    /// is less than 1, or [`CalibrationError::DegenerateAxis`] if either axis's
    /// raw endpoints are equal.
    pub const fn try_new(
        x1: i32,
        x2: i32,
        y1: i32,
        y2: i32,
        width: i32,
        height: i32,
    ) -> Result<Self, CalibrationError> {
        match Self::validate(x1, x2, y1, y2, width, height) {
            Ok(()) => Ok(Self {
                x1,
                x2,
                y1,
                y2,
                width,
                height,
            }),
            Err(error) => Err(error),
        }
    }

    /// Check [`Calibration`]'s invariants without constructing a value.
    const fn validate(x1: i32, x2: i32, y1: i32, y2: i32, width: i32, height: i32) -> Result<(), CalibrationError> {
        if width < 1 || height < 1 {
            return Err(CalibrationError::NonPositiveDimension);
        }
        if x1 == x2 || y1 == y2 {
            return Err(CalibrationError::DegenerateAxis);
        }
        Ok(())
    }

    /// Map one raw sample onto the pixel grid.
    ///
    /// Each axis is linearly interpolated from its two raw endpoints onto
    /// `0..width` (or `0..height`) and clamped, so samples outside the
    /// calibrated corners — or produced by noise — never yield a negative or
    /// out-of-range coordinate. The mapping is pure: no I/O, no filtering, and
    /// no mutation of `self`. Given a valid calibration it cannot panic: the
    /// constructor guarantees distinct endpoints and positive dimensions, and
    /// the arithmetic widens to `i64` so extreme (but valid) endpoints cannot
    /// overflow.
    #[must_use]
    pub fn to_pixels(&self, raw: TouchSample) -> Point {
        let x = Self::interpolate(i64::from(raw.x), self.x1, self.x2, self.width);
        let y = Self::interpolate(i64::from(raw.y), self.y1, self.y2, self.height);
        Point::new(x, y)
    }

    /// Linearly map one raw axis value onto `0..extent` and clamp it there.
    ///
    /// `at_zero` and `at_max` are the axis's raw endpoints at pixel `0` and
    /// pixel `extent - 1`; validation guarantees they differ. The `i64`
    /// intermediates keep the arithmetic free of overflow for any endpoint
    /// values that fit an `i32`.
    fn interpolate(value: i64, at_zero: i32, at_max: i32, extent: i32) -> i32 {
        let span = i64::from(at_max) - i64::from(at_zero);
        let numerator = (value - i64::from(at_zero)) * i64::from(extent - 1);
        let mapped = (numerator / span).clamp(0, i64::from(extent - 1));
        // `mapped` is within `0..=extent - 1` and `extent` is a positive `i32`,
        // so converting back cannot fail.
        i32::try_from(mapped).unwrap_or(0)
    }
}

impl Default for Calibration {
    /// The vendor [`REFERENCE`](Calibration::REFERENCE) values, for first
    /// bring-up only; this robot's panel uses [`MEASURED`](Calibration::MEASURED).
    fn default() -> Self {
        Self::REFERENCE
    }
}
