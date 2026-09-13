//! Pure, I/O-free conversion of raw touch samples into screen pixels.

use embedded_graphics::geometry::Point;

use crate::types::TouchSample;

/// Two-point panel calibration and the target pixel dimensions.
///
/// Each axis is described by two raw endpoints, and
/// [`to_pixels`](Self::to_pixels) linearly interpolates any raw conversion onto
/// the pixel grid `0..width` × `0..height`. The raw endpoints are not assumed to
/// be ordered: an inverted axis simply has `x1 > x2` (or `y1 > y2`) and the
/// interpolation still lands on the correct pixel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Calibration {
    /// Raw X conversion measured at the screen's left edge (pixel column `0`).
    pub x1: i32,
    /// Raw X conversion measured at the screen's right edge (pixel column `width - 1`).
    pub x2: i32,
    /// Raw Y conversion measured at the screen's top edge (pixel row `0`).
    pub y1: i32,
    /// Raw Y conversion measured at the screen's bottom edge (pixel row `height - 1`).
    pub y2: i32,
    /// Screen width in pixels.
    pub width: i32,
    /// Screen height in pixels.
    pub height: i32,
}

impl Calibration {
    /// Reference calibration values for a 320 × 240 panel.
    ///
    /// # UNCALIBRATED REFERENCE VALUES — NOT OUR PANEL'S CALIBRATION
    ///
    /// These numbers are transcribed verbatim from the Waveshare
    /// Pico-ResTouch-LCD-2.8 reference driver. They describe *that vendor's*
    /// board, not the panel wired to this robot. They exist only so that the
    /// pixel mapping can be exercised end-to-end before bring-up. They **must**
    /// be re-measured on our panel's four corners during bring-up and must not
    /// be treated as correct for our hardware.
    pub const REFERENCE: Self = Self {
        x1: 3880,
        x2: 340,
        y1: 262,
        y2: 3850,
        width: 320,
        height: 240,
    };

    /// Calibration measured on this project's 2.8″ panel during bring-up.
    ///
    /// The four `touch_coexistence` corner targets were touched in turn, and the
    /// raw readings at those known framebuffer coordinates (inset 12 px from the
    /// edges) were extrapolated out to the real
    /// corners. Unlike [`REFERENCE`](Self::REFERENCE), these numbers describe the
    /// panel actually wired to this robot, in the display orientation the
    /// coexistence example uses: raw X falls as the pixel column rises, and raw Y
    /// rises as the pixel row rises.
    pub const MEASURED: Self = Self {
        x1: 3810,
        x2: 160,
        y1: 276,
        y2: 3844,
        width: 320,
        height: 240,
    };

    /// Map one raw sample onto the pixel grid.
    ///
    /// Each axis is linearly interpolated from its two raw endpoints onto
    /// `0..width` (or `0..height`) and clamped, so samples outside the
    /// calibrated corners — or produced by noise — never yield a negative or
    /// out-of-range coordinate. The mapping is pure: no I/O, no filtering, and
    /// no mutation of `self`.
    #[must_use]
    pub fn to_pixels(&self, raw: TouchSample) -> Point {
        let x = ((i32::from(raw.x) - self.x1) * (self.width - 1) / (self.x2 - self.x1)).clamp(0, self.width - 1);
        let y = ((i32::from(raw.y) - self.y1) * (self.height - 1) / (self.y2 - self.y1)).clamp(0, self.height - 1);
        Point::new(x, y)
    }
}

impl Default for Calibration {
    fn default() -> Self {
        Self::REFERENCE
    }
}
