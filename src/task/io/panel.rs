//! Shared-bus panel bring-up: ST7789 display and resistive touch on SPI1.
//!
//! The display and the touch controller share one full-duplex `SPI1` bus. Each
//! device speaks through its own [`SpiDeviceWithConfig`], which locks the bus (a
//! [`CriticalSectionRawMutex`]), applies its own bus configuration, asserts its
//! own chip select for the transaction, and releases. The display runs at 64 MHz
//! and the touch controller at ~200 kHz, both Mode 3, so only the clock differs
//! between the two configurations.
//!
//! One task owns the whole panel: it turns the backlight on, pulses the
//! controller's reset line, initialises the display with retry, samples touch,
//! maps the samples into landscape framebuffer pixels, and flushes full frames.
//! When the display will not initialise the task keeps the firmware alive in a
//! degraded state and retries with a backoff, mirroring the previous text
//! display's offline behaviour.
//!
//! The smoke screen drawn here is bring-up scaffolding: it is the consumer that
//! proves the landscape orientation, the colour order, and the touch mapping on
//! glass. It is not UI; the menu model lands on top of this task in a later
//! ticket, at which point the smoke screen is replaced.
//!
//! Pin map (ADR-0008): `SPI1` SCK/MOSI/MISO = GPIO 42/43/44; display
//! CS/DC/RST/BLK = GPIO 41/45/46/47; touch CS/PENIRQ = GPIO 38/39.

use core::fmt::Write as _;

use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_rp::{
    gpio::{Input, Output},
    peripherals::{DMA_CH6, DMA_CH7, PIN_42, PIN_43, PIN_44, SPI1},
    spi::{self, Spi},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X10, FONT_9X15_BOLD, FONT_10X20},
    },
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::String;
use moving_median::MovingMedian;
use st7789_async::{ColorOrder, Config as DisplayConfig, Orientation, Rotation, St7789};
use static_cell::ConstStaticCell;
use touch_async::{Calibration, TouchPanel, TouchSample};

use crate::Irqs;

// ── Bus and framebuffer ───────────────────────────────────────────────────────

/// The shared full-duplex `SPI1` bus carrying both panel devices.
pub type PanelBus = Mutex<CriticalSectionRawMutex, Spi<'static, SPI1, spi::Async>>;

/// The chip-select-aware SPI device both panel drivers speak through.
type PanelDevice =
    SpiDeviceWithConfig<'static, CriticalSectionRawMutex, Spi<'static, SPI1, spi::Async>, Output<'static>>;

/// The ST7789 display driver owned by the panel task.
type Display = St7789<'static, PanelDevice, Output<'static>>;

/// The touch driver owned by the panel task.
type Touch = TouchPanel<PanelDevice>;

/// Display SPI clock in Hz (64 MHz): fast enough to flush a full frame.
const DISPLAY_FREQ: u32 = 64_000_000;

/// Touch SPI clock in Hz (200 kHz): the `XPT2046`-class controller's speed.
const TOUCH_FREQ: u32 = 200_000;

/// Framebuffer width in pixels (landscape after the 90° rotation).
const FB_W: usize = 320;

/// Framebuffer height in pixels.
const FB_H: usize = 240;

/// Framebuffer byte length (big-endian RGB565).
const FB_LEN: usize = FB_W * FB_H * 2;

/// Attempts before the display is declared offline.
const INIT_RETRIES: u8 = 5;

/// Delay between initialisation attempts.
const INIT_RETRY_DELAY: Duration = Duration::from_millis(200);

/// Backoff before retrying an offline display.
const REINIT_BACKOFF: Duration = Duration::from_secs(2);

/// Panel reset low pulse, in milliseconds.
const PANEL_RESET_LOW_MS: u64 = 10;

/// Panel settle after the reset is released, in milliseconds.
const PANEL_RESET_HIGH_MS: u64 = 120;

/// Statically-allocated framebuffer (`153_600` bytes of zeroed `.bss`).
static FB: ConstStaticCell<[u8; FB_LEN]> = ConstStaticCell::new([0; FB_LEN]);

/// Build the display's bus configuration: Mode 3 at [`DISPLAY_FREQ`].
///
/// The ST7789 and the touch controller share Mode 3 and differ only in clock
/// speed, so each device's [`SpiDeviceWithConfig`] reconfigures the bus before
/// its own transactions.
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

/// Construct the shared full-duplex `SPI1` bus from the panel's pins and DMA.
///
/// The bus is created with the touch controller's configuration; each
/// [`SpiDeviceWithConfig`] reconfigures it before its own transactions, so the
/// initial configuration is immaterial. Transmit and receive each own a DMA
/// channel because the bus is full duplex.
#[must_use]
pub fn new_shared_bus(
    spi: embassy_rp::Peri<'static, SPI1>,
    sck: embassy_rp::Peri<'static, PIN_42>,
    mosi: embassy_rp::Peri<'static, PIN_43>,
    miso: embassy_rp::Peri<'static, PIN_44>,
    tx_dma: embassy_rp::Peri<'static, DMA_CH6>,
    rx_dma: embassy_rp::Peri<'static, DMA_CH7>,
    irqs: Irqs,
) -> PanelBus {
    Mutex::new(Spi::new(spi, sck, mosi, miso, tx_dma, rx_dma, irqs, touch_spi_config()))
}

// ── Touch tuning ──────────────────────────────────────────────────────────────

/// Length of the moving-median window, in samples, for the raw `x`/`y` counts.
const MEDIAN_WINDOW: usize = 5;

/// Poll period, in milliseconds, while the pen is down.
///
/// 20 ms (50 Hz) tracks a finger closely without spending more time on SPI
/// reads than the panel needs.
const TICK_MS: u64 = 20;

/// Maximum pointer movement, in pixels, that still counts as a tap.
///
/// A fingertip rolls several pixels when it presses; 20 px absorbs that.
const TAP_MAX_MOVE: i32 = 20;

/// Minimum press duration, in milliseconds, for a release to count as a tap.
///
/// Set just below [`TICK_MS`] so it rejects contact bounce without rejecting a
/// quick deliberate tap.
const TAP_MIN_DURATION_MS: u64 = 20;

/// Minimum interval, in milliseconds, between drag redraws.
///
/// A full-frame flush costs a few milliseconds, so 50 ms (20 fps) keeps a drag
/// responsive without saturating the shared bus.
const DRAG_RENDER_MS: u64 = 50;

/// The panel's measured touch calibration, mirrored for this orientation.
///
/// [`Calibration::MEASURED`] was measured in the `touch_coexistence`
/// orientation, which is this landscape orientation plus a horizontal mirror; a
/// mirror reverses the on-screen X axis, so mirroring the raw X endpoints
/// derives this orientation's calibration from the measured values.
const CALIBRATION: Calibration = Calibration::MEASURED.mirrored_x();

// ── Smoke screen layout ───────────────────────────────────────────────────────

/// Smoke-screen background (dark navy).
const SMOKE_BG: Rgb565 = Rgb565::new(0, 1, 8);

/// Side length of the tracking marker, in pixels.
const MARKER_SIZE: u32 = 9;

/// Y coordinate of the colour-swatch row, in pixels.
const SWATCH_Y: i32 = 70;

/// Side length of a colour swatch, in pixels.
const SWATCH_SIZE: u32 = 44;

/// Y coordinate of the swatch labels, in pixels.
const SWATCH_LABEL_Y: i32 = 118;

/// Y coordinate of the live touch readout, in pixels.
const READOUT_Y: i32 = 150;

/// Y coordinate of the footer hint, in pixels.
const HINT_Y: i32 = 190;

/// The colour swatches drawn by the smoke screen: colour, label, and left edge.
///
/// The swatches double as a colour-order check: with the wrong subpixel order
/// the red swatch renders blue and the magenta swatch renders green.
const SWATCHES: [(Rgb565, &str, i32); 6] = [
    (Rgb565::CSS_RED, "RED", 10),
    (Rgb565::CSS_GREEN, "GRN", 60),
    (Rgb565::CSS_BLUE, "BLU", 110),
    (Rgb565::CSS_YELLOW, "YEL", 160),
    (Rgb565::CSS_CYAN, "CYN", 210),
    (Rgb565::CSS_MAGENTA, "MAG", 260),
];

// ── Task ──────────────────────────────────────────────────────────────────────

/// The panel task: brings the display up, samples touch, and draws the smoke
/// screen.
///
/// Runs on core0 and owns the display, the touch controller, and the
/// framebuffer for the lifetime of the firmware.
#[embassy_executor::task]
pub async fn panel(
    bus: &'static PanelBus,
    display_cs: Output<'static>,
    dc: Output<'static>,
    mut rst: Output<'static>,
    mut blk: Output<'static>,
    touch_cs: Output<'static>,
    mut irq: Input<'static>,
) {
    let display_dev = SpiDeviceWithConfig::new(bus, display_cs, display_spi_config());
    let width = u16::try_from(FB_W).unwrap_or(u16::MAX);
    let height = u16::try_from(FB_H).unwrap_or(u16::MAX);
    let mut display = St7789::new(display_dev, dc, FB.take(), width, height);
    let mut touch = TouchPanel::new(SpiDeviceWithConfig::new(bus, touch_cs, touch_spi_config()));

    let config = DisplayConfig {
        // This panel is RGB-ordered: with `Bgr` the red and blue channels swap.
        color_order: ColorOrder::Rgb,
        // Readable landscape: the robot's target orientation, and the one the
        // mirrored calibration above was derived for.
        orientation: Orientation::new().rotate(Rotation::Deg90),
        invert_colors: false,
    };

    // Backlight on, then a hardware reset, before the controller is initialised.
    blk.set_high();
    hardware_reset(&mut rst).await;

    loop {
        if bring_up(&mut display, &config).await {
            serve(&mut display, &mut touch, &mut irq).await;
            defmt::warn!("panel went offline; retrying initialisation");
        }
        Timer::after(REINIT_BACKOFF).await;
    }
}

/// Pulse the panel's reset line low, then high, letting the controller settle.
async fn hardware_reset(rst: &mut Output<'static>) {
    rst.set_low();
    Timer::after(Duration::from_millis(PANEL_RESET_LOW_MS)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(PANEL_RESET_HIGH_MS)).await;
}

/// Initialise the display with retry and draw the smoke screen.
///
/// Returns `true` once the display is initialised and the smoke screen has been
/// flushed; `false` if every attempt failed.
async fn bring_up(display: &mut Display, config: &DisplayConfig) -> bool {
    for attempt in 1..=INIT_RETRIES {
        if display.init(config, &mut Delay).await.is_ok() && render_smoke(display, None).await.is_ok() {
            info!("panel online");
            return true;
        }
        defmt::warn!("panel init failed (attempt {}/{})", attempt, INIT_RETRIES);
        Timer::after(INIT_RETRY_DELAY).await;
    }
    false
}

/// Sample touch and track the finger until a display access fails.
///
/// `PENIRQ` is the pen-down and pen-up authority: the task polls while the pen
/// is down and leaves the inner loop only once the interrupt line is high.
/// Returns when a draw or flush fails, so the caller can re-initialise.
async fn serve(display: &mut Display, touch: &mut Touch, irq: &mut Input<'static>) {
    let mut filters = Filters::new();
    loop {
        let Ok(raw) = touch.wait_for_touch(irq).await else {
            defmt::warn!("touch wait failed; retrying");
            Timer::after(Duration::from_millis(TICK_MS)).await;
            continue;
        };
        // Start each gesture with fresh filters: carried-over samples would pull
        // the first point toward wherever the finger last was.
        filters.clear();
        let start = filters.point(raw);
        let now_ms = Instant::now().as_millis();
        let mut press = Press::new(start, now_ms);
        let mut last_render = now_ms;
        if render_smoke(display, Some(start)).await.is_err() {
            return;
        }

        // Poll while the pen is down so continuous drag positions are available.
        loop {
            Timer::after(Duration::from_millis(TICK_MS)).await;
            match touch.read().await {
                Ok(Some(raw)) => {
                    let point = filters.point(raw);
                    press.update(point);
                    let now_ms = Instant::now().as_millis();
                    if now_ms.saturating_sub(last_render) >= DRAG_RENDER_MS {
                        if render_smoke(display, Some(point)).await.is_err() {
                            return;
                        }
                        last_render = now_ms;
                    }
                }
                Ok(None) | Err(_) => {
                    if irq.is_high() {
                        info!("gesture tap={}", press.is_tap(Instant::now().as_millis()));
                        // Release always renders the final position cleared.
                        if render_smoke(display, None).await.is_err() {
                            return;
                        }
                        break;
                    }
                }
            }
        }
    }
}

// ── Input filtering ───────────────────────────────────────────────────────────

/// The caller-side smoothing filters applied to raw touch samples.
struct Filters {
    /// Moving median of the raw X counts.
    x: MovingMedian<u16, MEDIAN_WINDOW>,
    /// Moving median of the raw Y counts.
    y: MovingMedian<u16, MEDIAN_WINDOW>,
}

impl Filters {
    /// Create filters with empty windows.
    fn new() -> Self {
        Self {
            x: MovingMedian::new(),
            y: MovingMedian::new(),
        }
    }

    /// Clear both windows so a new gesture starts from fresh samples.
    fn clear(&mut self) {
        self.x.clear();
        self.y.clear();
    }

    /// Filter one raw sample and map it into landscape framebuffer pixels.
    ///
    /// This is the delivery boundary: the smoke screen (and, later, the UI)
    /// receives a calibrated [`Point`] in framebuffer space, never raw counts.
    fn point(&mut self, raw: TouchSample) -> Point {
        // `add_value` only rejects NaN, which `u16` cannot produce.
        let _ = self.x.add_value(raw.x);
        let _ = self.y.add_value(raw.y);
        let x = self.x.median().unwrap_or(raw.x);
        let y = self.y.median().unwrap_or(raw.y);
        CALIBRATION.to_pixels(TouchSample {
            x,
            y,
            z1: raw.z1,
            z2: raw.z2,
        })
    }
}

/// The in-progress pen gesture, used to classify taps and throttle drags.
///
/// This is bring-up instrumentation for the bench, not menu logic; the UI crate
/// takes over gesture interpretation once it lands.
struct Press {
    /// Framebuffer point where the press began.
    start: Point,
    /// Most recent framebuffer point observed.
    last: Point,
    /// Uptime, in milliseconds, when the press began.
    started_ms: u64,
}

impl Press {
    /// Begin tracking a press that started at `start` at `now_ms`.
    const fn new(start: Point, now_ms: u64) -> Self {
        Self {
            start,
            last: start,
            started_ms: now_ms,
        }
    }

    /// Record the latest observed point.
    const fn update(&mut self, point: Point) {
        self.last = point;
    }

    /// Whether the completed gesture is a tap rather than a drag.
    ///
    /// A tap lasted at least [`TAP_MIN_DURATION_MS`] and never moved more than
    /// [`TAP_MAX_MOVE`] pixels from where it began; anything else is a drag.
    const fn is_tap(&self, now_ms: u64) -> bool {
        let dx = self.last.x - self.start.x;
        let dy = self.last.y - self.start.y;
        now_ms.saturating_sub(self.started_ms) >= TAP_MIN_DURATION_MS
            && dx * dx + dy * dy <= TAP_MAX_MOVE * TAP_MAX_MOVE
    }
}

// ── Smoke screen ──────────────────────────────────────────────────────────────

/// Redraw the full smoke screen with an optional tracking marker and flush it.
///
/// Returns `Err(())` if drawing or flushing fails, which takes the panel
/// offline until the next successful re-initialisation.
async fn render_smoke(display: &mut Display, point: Option<Point>) -> Result<(), ()> {
    draw_smoke(display, point).map_err(|_| ())?;
    display.flush().await.map_err(|_| ())
}

/// Draw the static smoke-screen chrome and an optional tracking marker.
fn draw_smoke<D>(display: &mut D, point: Option<Point>) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    display.clear(SMOKE_BG)?;
    draw_title(display)?;
    draw_swatches(display)?;
    draw_readout(display, point)?;
    Text::with_baseline(
        "the dot tracks the touched pixel",
        Point::new(10, HINT_Y),
        MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_LIGHT_GRAY),
        Baseline::Top,
    )
    .draw(display)?;
    if let Some(point) = point {
        draw_marker(display, point)?;
    }
    Ok(())
}

/// Draw the title and subtitle.
fn draw_title<D>(display: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Text::with_baseline(
        "explorer-robot v3",
        Point::new(10, 10),
        MonoTextStyle::new(&FONT_10X20, Rgb565::CSS_WHITE),
        Baseline::Top,
    )
    .draw(display)?;
    Text::with_baseline(
        "panel smoke test",
        Point::new(10, 36),
        MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_LIGHT_GRAY),
        Baseline::Top,
    )
    .draw(display)?;
    Ok(())
}

/// Draw labelled colour swatches so the subpixel order is verifiable.
fn draw_swatches<D>(display: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    for (colour, label, x) in SWATCHES {
        Rectangle::new(Point::new(x, SWATCH_Y), Size::new(SWATCH_SIZE, SWATCH_SIZE))
            .into_styled(PrimitiveStyle::with_fill(colour))
            .draw(display)?;
        Text::with_baseline(
            label,
            Point::new(x + 2, SWATCH_LABEL_Y),
            MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_WHITE),
            Baseline::Top,
        )
        .draw(display)?;
    }
    Ok(())
}

/// Draw the live touch readout, or a hint while the panel is untouched.
fn draw_readout<D>(display: &mut D, point: Option<Point>) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut text: String<32> = String::new();
    match point {
        Some(point) => {
            let x = point.x;
            let y = point.y;
            let _ = write!(text, "touch x={x} y={y}");
        }
        None => {
            let _ = text.push_str("touch the panel");
        }
    }
    Text::with_baseline(
        text.as_str(),
        Point::new(10, READOUT_Y),
        MonoTextStyle::new(&FONT_9X15_BOLD, Rgb565::CSS_ORANGE),
        Baseline::Top,
    )
    .draw(display)?;
    Ok(())
}

/// Draw a filled square with a black crosshair centred on `point`.
///
/// Out-of-bounds pixels are clipped by the draw target.
fn draw_marker<D>(display: &mut D, point: Point) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    Rectangle::with_center(point, Size::new(MARKER_SIZE, MARKER_SIZE))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::CSS_GREEN_YELLOW))
        .draw(display)?;
    Rectangle::with_center(point, Size::new(1, MARKER_SIZE))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::CSS_BLACK))
        .draw(display)?;
    Rectangle::with_center(point, Size::new(MARKER_SIZE, 1))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::CSS_BLACK))
        .draw(display)?;
    Ok(())
}
