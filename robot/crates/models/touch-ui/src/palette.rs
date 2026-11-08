//! The panel palette and typography.
//!
//! Every colour and font lives here so the look can be restyled on glass
//! without touching layout or interaction code. Colours are built with the
//! panel's RGB subpixel order.

use embedded_graphics::{
    mono_font::{
        MonoFont,
        ascii::{FONT_6X10, FONT_9X15, FONT_9X15_BOLD, FONT_10X20},
    },
    pixelcolor::{Rgb565, RgbColor, WebColors},
};

/// Screen background: a near-black blue-grey.
pub const BG: Rgb565 = Rgb565::new(2, 4, 6);

/// Unpressed button fill: a dark slate that reads as a raised surface.
pub const BUTTON_BG: Rgb565 = Rgb565::new(8, 12, 16);

/// Button outline: a subtle step up from [`BUTTON_BG`].
pub const BUTTON_BORDER: Rgb565 = Rgb565::new(18, 24, 30);

/// Primary text (titles and labels): near-white.
pub const TEXT: Rgb565 = Rgb565::new(30, 60, 30);

/// Muted chrome (the header divider and the scroll track): a mid grey, dimmer
/// than [`TEXT`].
pub const MUTED: Rgb565 = Rgb565::new(14, 28, 14);

/// Accent used for the pressed/selected state: a saturated teal.
pub const ACCENT: Rgb565 = Rgb565::new(3, 40, 31);

/// Radar range-ring and crosshair colour.
pub const RADAR_RING: Rgb565 = Rgb565::WHITE;

/// Radar mark colour for a valid return.
pub const RADAR_RETURN: Rgb565 = Rgb565::CSS_GREEN_YELLOW;

/// Bold font used for the header title.
pub const TITLE_FONT: &MonoFont<'static> = &FONT_9X15_BOLD;

/// Regular font used for button labels.
pub const LABEL_FONT: &MonoFont<'static> = &FONT_9X15;

/// Small font used for the System Info rows, the status body text, and the
/// Room Scan caption.
pub const SMALL_FONT: &MonoFont<'static> = &FONT_6X10;

/// Large font used for the value-entry readout.
pub const READOUT_FONT: &MonoFont<'static> = &FONT_10X20;
