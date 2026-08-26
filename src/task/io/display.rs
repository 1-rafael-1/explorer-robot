//! Display driver — ST7789 TFT over a write-only SPI1 bus.
//!
//! Text-only display with 14 lines × 26 characters. `LiDAR` provides spatial
//! awareness independently, so the display focuses on status and menu text.
//!
//! # Coordinate System
//! - Origin (0,0): top-left
//! - Y axis increases downward
//! - Line 0: y=0, Line n: y=n×17
//! - Each line is 17 px tall (9×15 font with 2 px padding)

use embassy_rp::{
    gpio::Output,
    peripherals::SPI1,
    spi::{Async as SpiAsync, Spi},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::{
    geometry::Size,
    mono_font::{
        MonoTextStyle, MonoTextStyleBuilder,
        ascii::{FONT_9X15, FONT_9X15_BOLD},
    },
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};
use heapless::String;
use st7789_async::{ColorOrder, Config, Orientation, St7789};
use static_cell::ConstStaticCell;

/// The dedicated write-only SPI1 bus.
type SpiPeripheral = Spi<'static, SPI1, SpiAsync>;

/// The dedicated SPI1 bus wrapped in a critical-section mutex.
type SpiMutex = Mutex<CriticalSectionRawMutex, SpiPeripheral>;

/// CS-less [`SpiDevice`] adapter: locks the dedicated bus for each transaction.
///
/// The panel has no chip-select line, so the bus is never shared; the mutex is
/// a formality that satisfies the `SpiDevice` API used by the ST7789 driver.
struct NoCsSpiDevice {
    /// The mutex-protected SPI bus.
    bus: &'static SpiMutex,
}

impl ErrorType for NoCsSpiDevice {
    type Error = embassy_rp::spi::Error;
}

impl SpiDevice<u8> for NoCsSpiDevice {
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        let mut bus = self.bus.lock().await;
        for op in operations {
            match op {
                Operation::Read(buf) => bus.read(buf).await?,
                Operation::Write(buf) => bus.write(buf).await?,
                Operation::Transfer(read, write) => bus.transfer(read, write).await?,
                Operation::TransferInPlace(buf) => bus.transfer_in_place(buf).await?,
                // The ST7789 driver issues its command delays through a separate
                // `DelayNs` handle, never via `Operation::DelayNs`.
                Operation::DelayNs(_) => {}
            }
        }

        // The transaction's operations are complete; release the bus before
        // returning so the guard doesn't outlive the critical section.
        drop(bus);

        Ok(())
    }
}

/// Text style for display rendering.
#[derive(Clone, Copy)]
pub enum TextStyle {
    /// Normal weight text.
    Normal,
    /// Bold text.
    Bold,
}

/// Display actions that can be requested by other tasks.
#[allow(clippy::large_enum_variant)]
pub enum DisplayAction {
    /// Show a text message on the display (bold style).
    ShowText(String<MAX_LINE_LEN>, u8),
    /// Show a text message with an explicit style.
    ShowTextStyled(String<MAX_LINE_LEN>, u8, TextStyle),
    /// Show all text lines in a single update.
    ShowLines([String<MAX_LINE_LEN>; DISPLAY_LINES]),
    /// Clear the entire display.
    Clear,
}

/// Display error types.
#[derive(Debug)]
enum DisplayError {
    /// Invalid text line number.
    InvalidLine,
}

/// ST7789 display driver type.
type DisplayDriver = St7789<'static, NoCsSpiDevice, Output<'static>>;

/// Control channel for display update requests.
static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, DisplayAction, 16> = Channel::new();

/// Display width in pixels.
const DISPLAY_WIDTH: u16 = 240;

/// Display height in pixels.
const DISPLAY_HEIGHT: u16 = 240;

/// Number of text lines the display can show.
pub const DISPLAY_LINES: usize = 14;

/// Maximum line length in characters supported by the display text contract.
pub const MAX_LINE_LEN: usize = 26;

/// Height of a single text line in pixels (9×15 font with 2 px padding).
const LINE_HEIGHT: u16 = 17;

/// Framebuffer size in bytes (240 × 240 pixels × 2 bytes/pixel RGB565).
const FRAMEBUFFER_LEN: usize = 240 * 240 * 2;

/// Framebuffer type.
type Framebuffer = [u8; FRAMEBUFFER_LEN];

/// Statically-allocated framebuffer (115,200 bytes of zeroed `.bss`).
static FRAMEBUFFER: ConstStaticCell<Framebuffer> = ConstStaticCell::new([0; FRAMEBUFFER_LEN]);

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

/// Main display task — manages the ST7789 TFT screen.
///
/// Handles text-only display actions (Clear, `ShowText`, `ShowTextStyled`,
/// `ShowLines`). `LiDAR` provides spatial awareness independently.
#[embassy_executor::task]
pub async fn display(
    spi_bus: &'static SpiMutex,
    dc: Output<'static>,
    mut rst: Output<'static>,
    mut blk: Output<'static>,
) {
    const INIT_RETRIES: u8 = 5;
    const INIT_RETRY_DELAY: Duration = Duration::from_millis(200);
    const REINIT_BACKOFF: Duration = Duration::from_secs(2);

    // Turn on the backlight before bringing the controller up.
    blk.set_high();

    // Hardware reset: RST low, settle, RST high, settle.
    rst.set_low();
    Timer::after(Duration::from_millis(10)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(120)).await;

    let fb = FRAMEBUFFER.take();
    let mut display = St7789::new(NoCsSpiDevice { bus: spi_bus }, dc, fb, DISPLAY_WIDTH, DISPLAY_HEIGHT);

    let config = Config {
        color_order: ColorOrder::Bgr,
        orientation: Orientation::new(),
        invert_colors: false,
    };

    let text_style_bold = MonoTextStyleBuilder::new()
        .font(&FONT_9X15_BOLD)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    let text_style_regular = MonoTextStyleBuilder::new()
        .font(&FONT_9X15)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    // Try to initialize the display. If it fails, continue in
    // "display-offline" mode: drain the channel without rendering
    // and periodically retry.
    let mut display_online = false;
    for attempt in 1..=INIT_RETRIES {
        if display.init(&config, &mut Delay).await.is_ok() {
            display_online = true;
            break;
        }
        defmt::warn!("display init failed (attempt {}/{})", attempt, INIT_RETRIES);
        Timer::after(INIT_RETRY_DELAY).await;
    }

    if display_online {
        let _ = display.clear(Rgb565::BLACK);

        let mut txt: String<MAX_LINE_LEN> = String::new();
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
            // Display offline: handle one action per iteration while retrying
            // init. Blocking senders under sustained updates is acceptable — a
            // display that won't initialize is a defective robot, not a state
            // worth optimizing for.
            for attempt in 1..=INIT_RETRIES {
                if display.init(&config, &mut Delay).await.is_ok() {
                    display_online = true;
                    let _ = display.clear(Rgb565::BLACK);
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
    text_style_bold: MonoTextStyle<'_, Rgb565>,
    text_style_regular: MonoTextStyle<'_, Rgb565>,
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
            let _ = display.clear(Rgb565::BLACK);
            for (index, text) in lines.iter().enumerate() {
                let line = u8::try_from(index).unwrap_or(u8::MAX);
                let style = if index == 0 {
                    text_style_bold
                } else {
                    text_style_regular
                };
                handle_show_text(display, style, text, line)?;
            }
            Ok(())
        }
        DisplayAction::Clear => {
            let _ = display.clear(Rgb565::BLACK);
            Ok(())
        }
    }
}

/// Render a single text line at the requested display row.
fn handle_show_text(
    display: &mut DisplayDriver,
    text_style: MonoTextStyle<'_, Rgb565>,
    text: &String<MAX_LINE_LEN>,
    line: u8,
) -> Result<(), DisplayError> {
    if usize::from(line) >= DISPLAY_LINES {
        return Err(DisplayError::InvalidLine);
    }

    let point = Point::new(0, i32::from(line) * i32::from(LINE_HEIGHT));

    // Clear the line area first.
    let _ = Rectangle::new(point, Size::new(u32::from(DISPLAY_WIDTH), u32::from(LINE_HEIGHT)))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(display);

    // Draw the text.
    let _ = Text::with_baseline(text, point, text_style, Baseline::Top).draw(display);

    Ok(())
}
