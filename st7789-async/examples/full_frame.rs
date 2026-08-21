//! ST7789 async driver — full-frame flush example.
//!
//! Adapted from the embassy `spi_display_framebuffer` example: draw a moving
//! ferris sprite into a RAM framebuffer and flush the whole frame over an
//! async, write-only SPI bus. This is the simplest use of the driver, but it
//! is bandwidth-bound because it rewrites all 320×240 pixels every frame.

#![no_std]
#![no_main]
// Demo code: coordinate casts between embedded-graphics `i32` and the display's
// `u16`, and `.unwrap()` on draw/flush results, are intentional.
#![allow(
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::cast_precision_loss,
    clippy::used_underscore_binding
)]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::DMA_CH6,
    spi::{self, Spi},
};
use embassy_time::{Delay, Duration, Ticker, Timer};
use embedded_graphics::{
    framebuffer::{Framebuffer, buffer_size},
    image::{Image, ImageRawLE},
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    pixelcolor::{
        Rgb565,
        raw::{BigEndian, RawU16},
    },
    prelude::*,
    text::Text,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_probe as _;
use st7789_async::{ColorOrder, Config, Orientation, Rotation, St7789};

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH6>;
});

/// SPI clock frequency in Hz.
const DISPLAY_FREQ: u32 = 64_000_000;

/// Landscape width in pixels after 90° rotation (native panel is 240×320 portrait).
const SCREEN_W: i32 = 320;
/// Landscape height in pixels.
const SCREEN_H: i32 = 240;
/// Framebuffer width in pixels.
const FB_W: usize = 320;
/// Framebuffer height in pixels.
const FB_H: usize = 240;

/// Ferris sprite width in pixels.
const FERRIS_W: i32 = 86;
/// Ferris sprite height in pixels.
const FERRIS_H: i32 = 64;
/// Vertical centre of the sprite.
const FERRIS_CENTER_Y: i32 = 100;
/// Vertical oscillation amplitude in pixels.
const FERRIS_AMPLITUDE: i32 = 40;
/// Horizontal pixels per full sine cycle.
const WAVELENGTH: i32 = 160;
/// Animation tick interval in milliseconds.
const ANIM_INTERVAL_MS: u64 = 30;

use core::f32::consts::PI;

/// Full-frame RGB565 framebuffer, stored big-endian so it can be flushed to the
/// panel verbatim (the ST7789 expects 16-bit pixels MSB-first on the wire).
type Fb = Framebuffer<Rgb565, RawU16, BigEndian, FB_W, FB_H, { buffer_size::<Rgb565>(FB_W, FB_H) }>;

/// Statically-allocated framebuffer (`153_600` bytes of zeroed `.bss`).
static mut FB: Fb = Fb::new();

/// Borrow the statically-allocated framebuffer (`main` runs once).
#[allow(static_mut_refs)]
fn fb() -> &'static mut Fb {
    unsafe { &mut *core::ptr::addr_of_mut!(FB) }
}

/// Bhaskara I's sine approximation, valid on `[0, PI]`.
fn sin_half_pi(x: f32) -> f32 {
    let y = x * (PI - x);
    16.0 * y / (5.0 * PI * PI - 4.0 * y)
}

/// Full-cycle sine (approximate). Accurate enough for animation.
fn sin(x: f32) -> f32 {
    let x = x % (2.0 * PI);
    if x <= PI { sin_half_pi(x) } else { -sin_half_pi(x - PI) }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(embassy_rp::config::Config::default());
    info!("st7789-async smoke test");

    let bl = p.PIN_20;
    let rst = p.PIN_27;
    let cs = p.PIN_17;
    let dcx = p.PIN_26;
    let mosi = p.PIN_19;
    let clk = p.PIN_18;

    // Backlight on.
    let _backlight = Output::new(bl, Level::High);

    // Hardware reset: RST low, settle, RST high, settle.
    let mut rst = Output::new(rst, Level::Low);
    rst.set_low();
    Timer::after(Duration::from_millis(10)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(120)).await;

    // Write-only async SPI (SPI0, no MISO).
    let mut spi_config = spi::Config::default();
    spi_config.frequency = DISPLAY_FREQ;
    spi_config.phase = spi::Phase::CaptureOnSecondTransition;
    spi_config.polarity = spi::Polarity::IdleHigh;
    let spi = Spi::new_txonly(p.SPI0, clk, mosi, p.DMA_CH6, Irqs, spi_config);

    // Exclusive CS-toggling device on the dedicated bus.
    let cs = Output::new(cs, Level::High);
    let dev = ExclusiveDevice::new(spi, cs, Delay);

    let dcx = Output::new(dcx, Level::Low);
    let mut display = St7789::new(dev, dcx);

    let config = Config {
        color_order: ColorOrder::Bgr,
        orientation: Orientation::new().rotate(Rotation::Deg90).flip_vertical(),
        invert_colors: false,
    };

    display.init(&config, &mut Delay).await.unwrap();
    info!("display initialised");

    let fb = fb();

    let raw = ImageRawLE::new(include_bytes!("../assets/ferris.raw"), FERRIS_W as u32);
    let style = MonoTextStyle::new(&FONT_10X20, Rgb565::BLUE);

    let mut x: i32 = 0;
    let mut ticker = Ticker::every(Duration::from_millis(ANIM_INTERVAL_MS));
    loop {
        ticker.next().await;

        fb.clear(Rgb565::BLACK).unwrap();

        Text::new("Hello, async ST7789!", Point::new(20, 200), style)
            .draw(fb)
            .unwrap();

        let phase = 2.0 * PI * (x as f32) / (WAVELENGTH as f32);
        let y = FERRIS_CENTER_Y - FERRIS_H / 2 + (FERRIS_AMPLITUDE as f32 * sin(phase)) as i32;

        Image::new(&raw, Point::new(x, y)).draw(fb).unwrap();
        if x + FERRIS_W > SCREEN_W {
            Image::new(&raw, Point::new(x - SCREEN_W, y)).draw(fb).unwrap();
        }

        display
            .fill_region(0, 0, (SCREEN_W - 1) as u16, (SCREEN_H - 1) as u16, fb.data())
            .await
            .unwrap();

        x += 1;
        if x >= SCREEN_W {
            x = 0;
        }
    }
}
