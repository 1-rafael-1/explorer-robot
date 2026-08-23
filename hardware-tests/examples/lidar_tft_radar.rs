//! `LiDAR` → TFT radar view.
//!
//! Reads the COIN-D6 `LiDAR` over UART0 and draws a top-down radar on an ST7789
//! TFT over SPI0: concentric white rings at 1 m intervals (out to 5 m) centred
//! on the `LiDAR`, with one red cross per valid return placed at its angle and
//! range. Frames are produced either by aggregating `SPINS` revolutions
//! (`CoinD6::read_aggregated`, smoother) or from a single raw revolution
//! (`CoinD6::read_scan`, fastest) — flip the `AGGREGATE` constant. Each frame is
//! flushed in full and the update rate is logged in Hz.
//!
//! Run from the repository root with:
//!
//! ```sh
//! cargo run -p hardware-tests --example lidar_tft_radar --release
//! ```

#![no_std]
#![no_main]
// Demo code: `.unwrap()` on driver/GPIO/draw results and the `i32`/`u32`/`f32`
// coordinate casts between the LiDAR's polar data and embedded-graphics are
// intentional. The aggregation buffers make the `main` future large.
#![allow(
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::cast_precision_loss,
    clippy::used_underscore_binding,
    clippy::large_futures
)]

use coin_d6::{AggregationConfig, CoinD6, Config as LidarConfig, Scan, WarmupConfig, WarmupOutcome};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{DMA_CH6, UART0},
    spi::{self, Spi},
    uart::{self, BufferedUart},
};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::{
    Pixel,
    draw_target::DrawTarget,
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle},
};
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_probe as _;
use st7789_async::{ColorOrder, Config as DisplayConfig, Orientation, Rotation, St7789};
use static_cell::{ConstStaticCell, StaticCell};

// Interrupt bindings for the `LiDAR` UART and the display's SPI DMA channel.
bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH6>;
});

/// `LiDAR` UART baud rate (230400 8N1).
const BAUD_RATE: u32 = 230_400;
/// `LiDAR` ingest buffer size; large enough for the largest possible packet.
const INGEST_BUF_LEN: usize = 1024;
/// Number of revolutions aggregated into one radar frame.
const SPINS: usize = 3;

/// Read mode: `true` aggregates `SPINS` revolutions per frame (smoother but
/// slower); `false` draws each raw revolution as it arrives (fastest update
/// rate). Flip this to compare update rates.
const AGGREGATE: bool = false;

/// `BufferedUart` RX ring buffer size. Absorbs the ~178 ms of stream data that
/// arrives while the display is flushed between frames.
const RX_BUF_LEN: usize = 4096;
/// `BufferedUart` TX ring buffer size (start/stop commands are 4 bytes).
const TX_BUF_LEN: usize = 16;

/// SPI clock frequency in Hz.
const DISPLAY_FREQ: u32 = 64_000_000;

/// Framebuffer width in pixels (landscape after 90° rotation).
const FB_W: usize = 320;
/// Framebuffer height in pixels.
const FB_H: usize = 240;

/// Radar centre on screen, X (pixels).
const CENTER_X: i32 = 160;
/// Radar centre on screen, Y (pixels).
const CENTER_Y: i32 = 120;
/// Pixels per metre: the 5 m range maps onto the 120 px radius.
const PX_PER_M: f32 = 24.0;
/// Radius of one range ring (1 m) in pixels.
const RING_STEP_PX: i32 = 24;
/// Number of range rings (1 m … 5 m).
const RING_COUNT: i32 = 5;
/// Outer ring radius in pixels (5 m).
const MAX_RADIUS_PX: i32 = 120;

/// The caller-allocated framebuffer (big-endian RGB565 bytes).
type Fb = [u8; FB_W * FB_H * 2];

/// Statically-allocated framebuffer (`153_600` bytes of zeroed `.bss`).
static FB: ConstStaticCell<Fb> = ConstStaticCell::new([0; FB_W * FB_H * 2]);

/// Static TX ring buffer for the `LiDAR`'s buffered UART.
static TX_BUF: StaticCell<[u8; TX_BUF_LEN]> = StaticCell::new();
/// Static RX ring buffer for the `LiDAR`'s buffered UART.
static RX_BUF: StaticCell<[u8; RX_BUF_LEN]> = StaticCell::new();

/// Draw one cross-shaped dot (centre plus four orthogonal neighbours) at
/// `(x, y)`. Out-of-bounds pixels are clipped by the draw target.
fn draw_cross<D>(display: &mut D, x: i32, y: i32, color: Rgb565) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    for (dx, dy) in [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)] {
        Pixel(Point::new(x + dx, y + dy), color).draw(display)?;
    }
    Ok(())
}

/// Draw the static range rings, the orientation crosshair, and one aggregated
/// scan's returns into `display`.
fn draw_radar<D>(display: &mut D, scan: &Scan) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let center = Point::new(CENTER_X, CENTER_Y);

    // White range rings at 1 m intervals, out to 5 m.
    for ring in 1..=RING_COUNT {
        let radius = ring * RING_STEP_PX;
        Circle::with_center(center, (radius * 2) as u32)
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
            .draw(display)?;
    }

    // Crosshair through the centre: the 0°/90°/180°/270° reference.
    Line::new(
        Point::new(CENTER_X - MAX_RADIUS_PX, CENTER_Y),
        Point::new(CENTER_X + MAX_RADIUS_PX, CENTER_Y),
    )
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
    .draw(display)?;
    Line::new(
        Point::new(CENTER_X, CENTER_Y - MAX_RADIUS_PX),
        Point::new(CENTER_X, CENTER_Y + MAX_RADIUS_PX),
    )
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
    .draw(display)?;

    // Centre marker: the `LiDAR` position.
    draw_cross(display, CENTER_X, CENTER_Y, Rgb565::WHITE)?;

    // One highly visible cross per valid return, at its angle (0° = up, clockwise) and
    // range (clamped to the outer ring).
    for point in &scan.points[..scan.len] {
        let Some(distance) = point.distance_mm else {
            continue;
        };
        let metres = f32::from(distance.get()) / 1000.0;
        let radius = (metres * PX_PER_M).min(MAX_RADIUS_PX as f32);
        let angle_rad = point.angle_deg.to_radians();
        let x = CENTER_X + (radius * libm::sinf(angle_rad)) as i32;
        let y = CENTER_Y - (radius * libm::cosf(angle_rad)) as i32;
        draw_cross(display, x, y, Rgb565::CSS_GREEN_YELLOW)?;
    }

    Ok(())
}

/// The embassy entry point: bring up the display and `LiDAR`, then continuously
/// aggregate revolutions and redraw the radar.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(embassy_rp::config::Config::default());
    info!("lidar -> tft radar");

    // ── Display: ST7789 over write-only SPI0 ────────────────────────────────
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

    let mut spi_config = spi::Config::default();
    spi_config.frequency = DISPLAY_FREQ;
    spi_config.phase = spi::Phase::CaptureOnSecondTransition;
    spi_config.polarity = spi::Polarity::IdleHigh;
    let spi = Spi::new_txonly(p.SPI0, clk, mosi, p.DMA_CH6, Irqs, spi_config);

    // Exclusive CS-toggling device on the dedicated bus.
    let cs = Output::new(cs, Level::High);
    let dev = ExclusiveDevice::new(spi, cs, Delay);

    let dcx = Output::new(dcx, Level::Low);
    let fb = FB.take();
    let mut display = St7789::new(dev, dcx, fb, FB_W as u16, FB_H as u16);

    let config = DisplayConfig {
        color_order: ColorOrder::Bgr,
        // The LiDAR's physical mounting inverts front/back and mirrors
        // left/right relative to the panel, so rotate the whole frame 180° to
        // match physical reality.
        orientation: Orientation::new()
            .rotate(Rotation::Deg90)
            .flip_vertical()
            .rotate(Rotation::Deg180),
        invert_colors: false,
    };
    display.init(&config, &mut Delay).await.unwrap();
    info!("display initialised");

    // ── LiDAR: COIN-D6 over UART0 ───────────────────────────────────────────
    let power = Output::new(p.PIN_15, Level::Low);

    let mut uart_config = uart::Config::default();
    uart_config.baudrate = BAUD_RATE;
    let uart = BufferedUart::new(
        p.UART0,
        p.PIN_12,
        p.PIN_13,
        Irqs,
        TX_BUF.init([0; TX_BUF_LEN]),
        RX_BUF.init([0; RX_BUF_LEN]),
        uart_config,
    );

    let mut ingest = [0u8; INGEST_BUF_LEN];
    let mut lidar = CoinD6::new(uart, power, &mut ingest, LidarConfig::default());

    lidar.power_on().await.unwrap();
    info!("lidar powered on");
    lidar.start().await.unwrap();
    info!("lidar started");

    // Aggregation buffers: `spins` holds the raw revolutions, `frame` is reused
    // first as the warm-up scratch scan, then as the aggregated output.
    let mut spins: [Scan; SPINS] = core::array::from_fn(|_| Scan::new());
    let mut frame = Scan::new();

    match lidar.warm_up(&mut frame, &WarmupConfig::default()).await.unwrap() {
        WarmupOutcome::Settled { spins, points } => {
            info!("warm-up settled: {} spins, {} points", spins, points);
        }
        WarmupOutcome::Plateaued { spins, points } => {
            info!("warm-up plateaued: {} spins, {} points", spins, points);
        }
        WarmupOutcome::Exhausted { spins } => {
            info!("warm-up exhausted after {} spins", spins);
        }
    }

    loop {
        let started = Instant::now();

        if AGGREGATE {
            lidar
                .read_aggregated(&mut spins, &mut frame, &AggregationConfig::default())
                .await
                .unwrap();
        } else {
            lidar.read_scan(&mut frame).await.unwrap();
        }

        display.clear(Rgb565::BLACK).unwrap();
        draw_radar(&mut display, &frame).unwrap();
        display.flush().await.unwrap();

        let elapsed = started.elapsed();
        let micros = elapsed.as_micros();
        let hz = if micros > 0 { 1_000_000.0 / (micros as f32) } else { 0.0 };
        info!("update: {} Hz ({} ms)", hz, elapsed.as_millis());
    }
}
