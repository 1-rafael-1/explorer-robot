//! Display driver — SSD1306 OLED over I2C.
//!
//! Ported from v2 `task/io/display.rs`, simplified for v3:
//! - No radar sweep visualization (removed ultrasonic sweeper dependency).
//! - Text-only display with 4 lines × 20 characters.
//!
//! # Coordinate System
//! - Origin (0,0): top-left
//! - Y axis increases downward
//! - Line 0: y=0, Line 1: y=16, Line 2: y=32, Line 3: y=48
//! - Each line is 16 px tall (7×14 font with 2 px padding)

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    geometry::Size,
    mono_font::{
        MonoTextStyle, MonoTextStyleBuilder,
        ascii::{FONT_7X14, FONT_7X14_BOLD},
    },
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::String;
use ssd1306_async::{
    I2CDisplayInterface, Ssd1306, i2c_interface::I2CInterface, mode::BufferedGraphicsMode, prelude::*,
};

use crate::I2cBusShared;

/// Handle for a device on the shared I2C bus.
///
/// Obtained via `I2cDevice::new(&shared_bus)`.
type I2cDeviceHandle = I2cDevice<
    'static,
    CriticalSectionRawMutex,
    embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
>;

/// Text style for display rendering.
#[derive(Clone, Copy)]
pub enum TextStyle {
    /// Normal weight text.
    Normal,
    /// Bold text.
    Bold,
}

/// Display actions that can be requested by other tasks.
pub enum DisplayAction {
    /// Show a text message on the display (bold style).
    ShowText(String<20>, u8),
    /// Show a text message with an explicit style.
    ShowTextStyled(String<20>, u8, TextStyle),
    /// Show all 4 text lines in a single update.
    ShowLines([String<20>; 4]),
    /// Clear the entire display.
    Clear,
}

/// Display error types.
#[derive(Debug)]
enum DisplayError {
    /// Invalid text line number.
    InvalidLine,
    /// Drawing operation failed.
    DrawError,
}

/// SSD1306 display driver type.
type DisplayDriver = Ssd1306<I2CInterface<I2cDeviceHandle>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

/// Control channel for display update requests.
static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, DisplayAction, 16> = Channel::new();

/// Request a display update — blocks until the action is queued.
pub async fn display_update(display_action: DisplayAction) {
    DISPLAY_CHANNEL.send(display_action).await;
}

/// Try to request a display update without blocking.
/// Returns true if the update was queued.
pub fn display_try_update(display_action: DisplayAction) -> bool {
    DISPLAY_CHANNEL.sender().try_send(display_action).is_ok()
}

/// Blocks until the next update request arrives.
async fn wait_for_action() -> DisplayAction {
    DISPLAY_CHANNEL.receive().await
}

/// Display dimensions.
const DISPLAY_WIDTH: i32 = 128;

/// Main display task — manages the SSD1306 OLED screen.
///
/// Handles text-only display actions (Clear, `ShowText`, `ShowTextStyled`, `ShowLines`).
/// The radar sweep visualization from v2 is removed — v3 uses `LiDAR` for
/// spatial awareness and the display is purely text-based.
#[embassy_executor::task]
pub async fn display(i2c_bus: &'static I2cBusShared) {
    const INIT_RETRIES: u8 = 5;
    const INIT_RETRY_DELAY: Duration = Duration::from_millis(200);
    const REINIT_BACKOFF: Duration = Duration::from_secs(2);

    let i2c = I2cDevice::new(i2c_bus);
    let interface = I2CDisplayInterface::new(i2c);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();

    let text_style_bold = MonoTextStyleBuilder::new()
        .font(&FONT_7X14_BOLD)
        .text_color(BinaryColor::On)
        .build();

    let text_style_regular = MonoTextStyleBuilder::new()
        .font(&FONT_7X14)
        .text_color(BinaryColor::On)
        .build();

    // Try to initialize the display. If it fails, continue in
    // "display-offline" mode: drain the channel without rendering
    // and periodically retry.
    let mut display_online = false;
    for attempt in 1..=INIT_RETRIES {
        if display.init().await.is_ok() {
            display_online = true;
            break;
        }
        defmt::warn!("display init failed (attempt {}/{})", attempt, INIT_RETRIES);
        Timer::after(INIT_RETRY_DELAY).await;
    }

    if display_online {
        display.clear();

        let mut txt: String<20> = String::new();
        let _ = txt.push_str("explorer-robot v3");
        let _ = handle_show_text(&mut display, text_style_bold, &txt, 0);

        if display.flush().await.is_err() {
            defmt::warn!("display flush failed after init; going offline");
            display_online = false;
        }
    } else {
        defmt::warn!("display unavailable; continuing without display output");
    }

    loop {
        let action = wait_for_action().await;

        if !display_online {
            // Drain actions so senders don't block. Periodically retry init.
            for attempt in 1..=INIT_RETRIES {
                if display.init().await.is_ok() {
                    display_online = true;
                    display.clear();
                    if display.flush().await.is_err() {
                        defmt::warn!("display flush failed after re-init; staying offline");
                        display_online = false;
                    } else {
                        defmt::warn!("display re-initialized successfully");
                    }
                    break;
                }
                if attempt == 1 {
                    defmt::warn!("retrying display init...");
                }
                Timer::after(INIT_RETRY_DELAY).await;
            }
            if !display_online {
                Timer::after(REINIT_BACKOFF).await;
            }
            continue;
        }

        if let Err(error) = handle_display_action(&mut display, text_style_bold, text_style_regular, action) {
            defmt::warn!(
                "display action failed ({}); taking display offline",
                defmt::Debug2Format(&error)
            );
            display_online = false;
            continue;
        }

        if display.flush().await.is_err() {
            defmt::warn!("display flush failed; taking display offline");
            display_online = false;
        }
    }
}

/// Handle a single display action.
fn handle_display_action(
    display: &mut DisplayDriver,
    text_style_bold: MonoTextStyle<'_, BinaryColor>,
    text_style_regular: MonoTextStyle<'_, BinaryColor>,
    action: DisplayAction,
) -> Result<(), DisplayError> {
    match action {
        DisplayAction::ShowText(text, line) => handle_show_text(display, text_style_bold, &text, line),
        DisplayAction::ShowTextStyled(text, line, style) => {
            let chosen = match style {
                TextStyle::Normal => text_style_regular,
                TextStyle::Bold => text_style_bold,
            };
            handle_show_text(display, chosen, &text, line)
        }
        DisplayAction::ShowLines(lines) => {
            display.clear();
            handle_show_text(display, text_style_bold, &lines[0], 0)?;
            handle_show_text(display, text_style_regular, &lines[1], 1)?;
            handle_show_text(display, text_style_regular, &lines[2], 2)?;
            handle_show_text(display, text_style_regular, &lines[3], 3)
        }
        DisplayAction::Clear => {
            display.clear();
            Ok(())
        }
    }
}

/// Render a single text line at the requested display row.
fn handle_show_text(
    display: &mut DisplayDriver,
    text_style: MonoTextStyle<BinaryColor>,
    text: &String<20>,
    line: u8,
) -> Result<(), DisplayError> {
    let point: Point = match line {
        0 => Point::new(0, 0),
        1 => Point::new(0, 16),
        2 => Point::new(0, 32),
        3 => Point::new(0, 48),
        _ => return Err(DisplayError::InvalidLine),
    };

    // Clear the line area first.
    Rectangle::new(point, Size::new(DISPLAY_WIDTH as u32, 16))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(display)
        .map_err(|_| DisplayError::DrawError)?;

    // Draw the text.
    Text::with_baseline(text, point, text_style, Baseline::Top)
        .draw(display)
        .map_err(|_| DisplayError::DrawError)?;

    Ok(())
}
