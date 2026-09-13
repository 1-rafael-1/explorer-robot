//! Touch → TFT coexistence on one shared full-duplex SPI0 bus.
//!
//! Brings up the ST7789 TFT and the XPT2046/TSC2046-class touch controller on a
//! single full-duplex SPI0 bus and draws the calibrated touch point on the TFT
//! as the panel is touched. Bus arbitration and per-device bus configuration are
//! delegated to `embassy_embedded_hal`'s
//! [`SpiDeviceWithConfig`](embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig):
//! the display runs at 64 MHz and the touch controller at 200 kHz, and the shared
//! `Mutex` serialises the slow touch read against the full-frame display flush.
//!
//! A caller-side 5-sample moving-median filter smooths the raw `x`/`y` counts
//! before calibration; the driver itself stays unfiltered and returns raw counts.
//!
//! The touch mapping uses [`Calibration::default`], which is the vendor's
//! reference curve rather than this panel's measured calibration, so the drawn
//! point is indicative until the corners are re-measured with `touch_probe`.
//!
//! Run from the repository root with:
//!
//! ```sh
//! cargo run -p hardware-tests --example touch_coexistence --release
//! ```

#![no_std]
#![no_main]
// Demo code: `.unwrap()` on driver/GPIO/draw results is intentional, and the
// `main` future is large because it holds both device drivers and the shared bus.
#![allow(
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::cast_precision_loss,
    clippy::used_underscore_binding,
    clippy::large_futures
)]

use defmt::info;
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH6, DMA_CH7, SPI0},
    spi::{self, Spi},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::{
    draw_target::DrawTarget,
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use moving_median::MovingMedian;
use panic_probe as _;
use st7789_async::{ColorOrder, Config as DisplayConfig, Orientation, Rotation, St7789};
use static_cell::{ConstStaticCell, StaticCell};
use touch_async::{Calibration, TouchPanel, TouchSample};

// Full-duplex SPI0 needs one DMA channel per direction: `DMA_IRQ_0` is bound to
// both the TX (`DMA_CH6`) and RX (`DMA_CH7`) handlers.
bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH6>, embassy_rp::dma::InterruptHandler<DMA_CH7>;
});

/// Display SPI clock frequency in Hz (64 MHz): fast enough to flush a full frame.
const DISPLAY_FREQ: u32 = 64_000_000;

/// Touch SPI clock frequency in Hz (200 kHz): the low speed the XPT2046-class
/// controller is specified for.
const TOUCH_FREQ: u32 = 200_000;

/// Framebuffer width in pixels (landscape after 90° rotation).
const FB_W: usize = 320;
/// Framebuffer height in pixels.
const FB_H: usize = 240;

/// Side length of the square marker, in pixels.
const MARKER_SIZE: u32 = 9;

/// Inset of the four calibration targets from the framebuffer edges, in pixels.
///
/// The targets are inset so they can be pressed without fighting the panel
/// bezel; the exact framebuffer coordinate of each is therefore
/// `(TARGET_INSET, TARGET_INSET)` and friends, not the corners proper.
const TARGET_INSET: i32 = 12;

/// Side length of a calibration target square, in pixels.
const TARGET_SIZE: u32 = 13;

/// Length of the moving-median window, in samples, for the raw `x`/`y` counts.
const MEDIAN_WINDOW: usize = 5;

/// The caller-allocated framebuffer (big-endian RGB565 bytes).
type Fb = [u8; FB_W * FB_H * 2];

/// Statically-allocated framebuffer (`153_600` bytes of zeroed `.bss`).
static FB: ConstStaticCell<Fb> = ConstStaticCell::new([0; FB_W * FB_H * 2]);

/// Mutable SPI bus shared by the display and the touch controller.
type SpiBus = Mutex<NoopRawMutex, Spi<'static, SPI0, spi::Async>>;

/// Statically-allocated home for the shared SPI bus.
static SPI_BUS: StaticCell<SpiBus> = StaticCell::new();

/// Build the display's bus configuration: Mode 3 at [`DISPLAY_FREQ`].
///
/// The ST7789 and the touch controller share Mode 3 and differ only in clock
/// speed, so the per-device [`SpiDeviceWithConfig`] reconfigures the bus on each
/// transaction.
fn display_spi_config() -> spi::Config {
    let mut config = spi::Config::default();
    config.frequency = DISPLAY_FREQ;
    config.phase = spi::Phase::CaptureOnSecondTransition;
    config.polarity = spi::Polarity::IdleHigh;
    config
}

/// Build the touch controller's bus configuration: Mode 3 at [`TOUCH_FREQ`].
fn touch_spi_config() -> spi::Config {
    let mut config = spi::Config::default();
    config.frequency = TOUCH_FREQ;
    config.phase = spi::Phase::CaptureOnSecondTransition;
    config.polarity = spi::Polarity::IdleHigh;
    config
}

/// Draw a filled square with a black crosshair centred on `(x, y)`.
///
/// The square makes the point visible at a glance and the crosshair marks the
/// exact calibrated pixel. Out-of-bounds pixels are clipped by the draw target.
fn draw_marker<D>(display: &mut D, x: i32, y: i32) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let center = Point::new(x, y);
    Rectangle::with_center(center, Size::new(MARKER_SIZE, MARKER_SIZE))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::CSS_GREEN_YELLOW))
        .draw(display)?;
    Rectangle::with_center(center, Size::new(1, MARKER_SIZE))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(display)?;
    Rectangle::with_center(center, Size::new(MARKER_SIZE, 1))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(display)?;
    Ok(())
}

/// Draw the four calibration targets at the framebuffer corners.
///
/// Each target is a distinct colour and sits at a known framebuffer
/// coordinate, so touching it pairs a raw reading with that coordinate no matter
/// how the display `Orientation` maps the framebuffer onto the glass:
///
/// | Target | Framebuffer | Colour |
/// |--------|-------------|--------|
/// | top-left | `(TARGET_INSET, TARGET_INSET)` | red |
/// | top-right | `(FB_W-1-TARGET_INSET, TARGET_INSET)` | green |
/// | bottom-right | `(FB_W-1-TARGET_INSET, FB_H-1-TARGET_INSET)` | blue |
/// | bottom-left | `(TARGET_INSET, FB_H-1-TARGET_INSET)` | yellow |
fn draw_targets<D>(display: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let inset = TARGET_INSET;
    let right = FB_W as i32 - 1 - inset;
    let bottom = FB_H as i32 - 1 - inset;
    let targets = [
        (inset, inset, Rgb565::RED),
        (right, inset, Rgb565::GREEN),
        (right, bottom, Rgb565::BLUE),
        (inset, bottom, Rgb565::YELLOW),
    ];
    for (x, y, color) in targets {
        Rectangle::with_center(Point::new(x, y), Size::new(TARGET_SIZE, TARGET_SIZE))
            .into_styled(PrimitiveStyle::with_fill(color))
            .draw(display)?;
    }
    Ok(())
}

/// The embassy entry point: bring up the shared bus, display, and touch panel,
/// then draw each filtered, calibrated touch point until reset.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(embassy_rp::config::Config::default());
    info!("touch -> tft coexistence on shared SPI0");

    // ── Display: ST7789 over the shared bus ─────────────────────────────────
    // Backlight on.
    let _backlight = Output::new(p.PIN_20, Level::High);

    // Hardware reset: RST low, settle, RST high, settle.
    let mut rst = Output::new(p.PIN_27, Level::Low);
    rst.set_low();
    Timer::after(Duration::from_millis(10)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(120)).await;

    // Full-duplex SPI0 with per-transaction DMA. The bus is initialised with the
    // touch config; `SpiDeviceWithConfig` reconfigures it before each use.
    let spi = Spi::new(
        p.SPI0,
        p.PIN_18,
        p.PIN_19,
        p.PIN_16,
        p.DMA_CH6,
        p.DMA_CH7,
        Irqs,
        touch_spi_config(),
    );
    let spi_bus: &'static SpiBus = SPI_BUS.init(Mutex::new(spi));

    // Both devices arbitrate on the shared bus; their CS pins select which is
    // listening. CS idles high so the deselected device stays quiet.
    let display_dev = SpiDeviceWithConfig::new(spi_bus, Output::new(p.PIN_17, Level::High), display_spi_config());
    let touch_dev = SpiDeviceWithConfig::new(spi_bus, Output::new(p.PIN_21, Level::High), touch_spi_config());

    let dcx = Output::new(p.PIN_26, Level::Low);
    let fb = FB.take();
    let mut display = St7789::new(display_dev, dcx, fb, FB_W as u16, FB_H as u16);

    let config = DisplayConfig {
        // This panel is RGB-ordered: with `Bgr` the red and blue channels swap
        // (red targets show blue, yellow shows cyan). The LiDAR example's `Bgr`
        // is not correct for this panel.
        color_order: ColorOrder::Rgb,
        // Matches `lidar_tft_radar`'s mounting-derived orientation: rotate 90°,
        // flip vertically, then rotate 180°.
        orientation: Orientation::new()
            .rotate(Rotation::Deg90)
            .flip_vertical()
            .rotate(Rotation::Deg180),
        invert_colors: false,
    };
    display.init(&config, &mut Delay).await.unwrap();
    info!("display initialised");

    // Clear and flush immediately: the panel retains whatever was last written
    // to it across a reset, so without this the previous flash's image would
    // linger until the first touch.
    display.clear(Rgb565::BLACK).unwrap();
    draw_targets(&mut display).unwrap();
    display.flush().await.unwrap();

    // ── Touch: XPT2046/TSC2046-class controller on the same bus ─────────────
    // PENIRQ is active-low, so pull it up and treat a low level as pen-down.
    let mut irq = Input::new(p.PIN_22, Pull::Up);
    let mut panel = TouchPanel::new(touch_dev);

    // Caller-side smoothing: the driver stays raw, the example owns the filter.
    let mut x_filter = MovingMedian::<u16, MEDIAN_WINDOW>::new();
    let mut y_filter = MovingMedian::<u16, MEDIAN_WINDOW>::new();
    // Measured on this panel during bring-up (see `Calibration::MEASURED`);
    // `Calibration::default()` would be the uncalibrated reference values.
    let calibration = Calibration::MEASURED;

    info!("touch the panel; the calibrated point is drawn on the TFT");
    info!("calibration targets: red=top-left green=top-right blue=bottom-right yellow=bottom-left");
    info!("touch each target and note the logged raw x/y");

    loop {
        let raw = panel.wait_for_touch(&mut irq).await.unwrap();

        // `add_value` only fails on NaN, which `u16` cannot produce.
        x_filter.add_value(raw.x).unwrap();
        y_filter.add_value(raw.y).unwrap();
        let median_x = x_filter.median().unwrap_or(raw.x);
        let median_y = y_filter.median().unwrap_or(raw.y);

        let sample = TouchSample {
            x: median_x,
            y: median_y,
            z1: raw.z1,
            z2: raw.z2,
        };
        let point = calibration.to_pixels(sample);

        display.clear(Rgb565::BLACK).unwrap();
        draw_targets(&mut display).unwrap();
        draw_marker(&mut display, point.x, point.y).unwrap();
        display.flush().await.unwrap();

        info!(
            "raw x={} y={} -> median x={} y={} -> pixel x={} y={} (z1={} z2={})",
            raw.x, raw.y, median_x, median_y, point.x, point.y, raw.z1, raw.z2
        );
    }
}
