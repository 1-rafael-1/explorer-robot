//! ST7789 async driver — partial-redraw animation example.
//!
//! Same ferris animation as `full_frame.rs`, but instead of flushing the whole
//! 320×240 frame every tick, it blits only the rectangle that changed (the
//! union of the previous and current sprite bounding boxes). Per-frame SPI
//! traffic stays tiny, so the animation runs at the ticker's full rate rather
//! than being SPI-bandwidth-bound.

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
    clippy::used_underscore_binding,
    clippy::too_many_lines
)]

use core::f32::consts::PI;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{DMA_CH6, SPI0},
    spi::{self, Spi},
};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embedded_graphics::{
    framebuffer::{Framebuffer, buffer_size},
    geometry::Size,
    image::{Image, ImageRawLE},
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    pixelcolor::{
        Rgb565,
        raw::{BigEndian, RawU16},
    },
    prelude::*,
    primitives::Rectangle,
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

/// Full-frame RGB565 framebuffer, stored big-endian so it can be flushed to the
/// panel verbatim (the ST7789 expects 16-bit pixels MSB-first on the wire).
type Fb = Framebuffer<Rgb565, RawU16, BigEndian, FB_W, FB_H, { buffer_size::<Rgb565>(FB_W, FB_H) }>;

/// The concrete display type built by this example.
type Display = St7789<ExclusiveDevice<Spi<'static, SPI0, spi::Async>, Output<'static>, Delay>, Output<'static>>;

/// Statically-allocated framebuffer (`153_600` bytes of zeroed `.bss`).
static mut FB: Fb = Fb::new();

/// Borrow the statically-allocated framebuffer (`main` runs once).
#[allow(static_mut_refs)]
fn fb() -> &'static mut Fb {
    unsafe { &mut *core::ptr::addr_of_mut!(FB) }
}

/// Max dirty-rectangle size for this animation: full screen width (the
/// wrap-around case) by the sprite height plus the per-frame vertical delta.
const DIRTY_MAX_BYTES: usize = (SCREEN_W as usize) * (FERRIS_H as usize + 2) * 2;

/// Scratch buffer used to gather the (non-contiguous) dirty-rectangle rows into
/// one contiguous block, so the blit is a single SPI transaction instead of one
/// per row.
static mut BUF: [u8; DIRTY_MAX_BYTES] = [0; DIRTY_MAX_BYTES];

/// Borrow the scratch buffer (`main` runs once).
#[allow(static_mut_refs)]
fn buf() -> &'static mut [u8; DIRTY_MAX_BYTES] {
    unsafe { &mut *core::ptr::addr_of_mut!(BUF) }
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

/// Blit a rectangular region of the framebuffer to the panel.
///
/// The dirty rectangle's rows aren't contiguous in the framebuffer, so they are
/// gathered into a scratch buffer first, then written as a single SPI
/// transaction. Many small async SPI writes are far more expensive than one
/// write of the same total byte count.
async fn blit_region(display: &mut Display, fb: &Fb, rect: Rectangle) {
    let x0 = rect.top_left.x as u16;
    let y0 = rect.top_left.y as u16;
    let x1 = (rect.top_left.x + rect.size.width as i32 - 1) as u16;
    let y1 = (rect.top_left.y + rect.size.height as i32 - 1) as u16;
    let width = rect.size.width as usize;
    let height = rect.size.height as usize;
    let bytes = width * height * 2;
    defmt::assert!(bytes <= DIRTY_MAX_BYTES);

    // Gather the (non-contiguous) framebuffer rows into one contiguous buffer.
    let t0 = Instant::now();
    let out = buf();
    let data = fb.data();
    for (i, row) in (0..height).enumerate() {
        let y = rect.top_left.y as usize + row;
        let start = (y * FB_W + rect.top_left.x as usize) * 2;
        let end = start + width * 2;
        let dst = i * width * 2;
        out[dst..dst + width * 2].copy_from_slice(&data[start..end]);
    }
    let gather_us = t0.elapsed().as_micros();

    let t1 = Instant::now();
    display.set_address_window(x0, y0, x1, y1).await.unwrap();
    let window_us = t1.elapsed().as_micros();

    let t2 = Instant::now();
    display.write_pixels(&out[..bytes]).await.unwrap();
    let pixels_us = t2.elapsed().as_micros();

    info!(
        "gather {} µs, window {} µs, pixels {} µs ({} bytes)",
        gather_us, window_us, pixels_us, bytes
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(embassy_rp::config::Config::default());
    info!("st7789-async dirty-rect example");

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
    let mut prev_x: i32 = 0;
    let mut prev_y: i32 = FERRIS_CENTER_Y - FERRIS_H / 2;

    // Compose and flush the base frame once: static text + initial sprite.
    // The text is never redrawn; subsequent frames only blit the sprite region.
    fb.clear(Rgb565::BLACK).unwrap();
    Text::new("Hello, async ST7789!", Point::new(20, 200), style)
        .draw(fb)
        .unwrap();
    Image::new(&raw, Point::new(x, prev_y)).draw(fb).unwrap();
    display
        .fill_region(0, 0, (SCREEN_W - 1) as u16, (SCREEN_H - 1) as u16, fb.data())
        .await
        .unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(ANIM_INTERVAL_MS));
    loop {
        ticker.next().await;

        let phase = 2.0 * PI * (x as f32) / (WAVELENGTH as f32);
        let y = FERRIS_CENTER_Y - FERRIS_H / 2 + (FERRIS_AMPLITUDE as f32 * sin(phase)) as i32;

        // Compose only the sprite in RAM (the text on the panel is untouched).
        fb.clear(Rgb565::BLACK).unwrap();
        Image::new(&raw, Point::new(x, y)).draw(fb).unwrap();
        if x + FERRIS_W > SCREEN_W {
            Image::new(&raw, Point::new(x - SCREEN_W, y)).draw(fb).unwrap();
        }

        // Dirty rectangle = union of the previous and current sprite copies.
        let cur_wrap = x + FERRIS_W > SCREEN_W;
        let prev_wrap = prev_x + FERRIS_W > SCREEN_W;

        let mut min_x = x.min(prev_x);
        let mut max_x = (x + FERRIS_W).max(prev_x + FERRIS_W);
        if cur_wrap {
            min_x = min_x.min(x - SCREEN_W);
        }
        if prev_wrap {
            min_x = min_x.min(prev_x - SCREEN_W);
        }
        min_x = min_x.max(0);
        max_x = max_x.min(SCREEN_W);

        let min_y = y.min(prev_y).max(0);
        let max_y = (y + FERRIS_H).max(prev_y + FERRIS_H).min(SCREEN_H);

        let dirty = Rectangle::new(
            Point::new(min_x, min_y),
            Size::new((max_x - min_x) as u32, (max_y - min_y) as u32),
        );

        blit_region(&mut display, fb, dirty).await;

        prev_x = x;
        prev_y = y;
        x += 1;
        if x >= SCREEN_W {
            x = 0;
        }
    }
}
