//! Shared-bus panel primitives: ST7789 display and resistive touch on SPI1.
//!
//! The display and the touch controller share one full-duplex `SPI1` bus. Each
//! device speaks through its own [`SpiDeviceWithConfig`], which locks the bus (a
//! [`CriticalSectionRawMutex`]), applies its own bus configuration, asserts its
//! own chip select for the transaction, and releases. The display runs at 64 MHz
//! and the touch controller at ~200 kHz, both Mode 3, so only the clock differs
//! between the two configurations.
//!
//! This module owns the hardware primitives and nothing else: it does not spawn
//! a task. [`Panel`] wraps the display, the touch controller, and the
//! caller-side touch filter, and exposes bring-up with retry, full-frame flush,
//! and calibrated touch sampling for the touch UI task to consume. When the
//! display will not initialise [`Panel::bring_up`] reports failure so the UI task
//! can keep the firmware alive in a degraded state and retry with a backoff,
//! mirroring the previous text display's offline behaviour.
//!
//! Pin map (ADR-0008): `SPI1` SCK/MOSI/MISO = GPIO 42/43/44; display
//! CS/DC/RST/BLK = GPIO 41/45/46/47; touch CS/PENIRQ = GPIO 38/39.

use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_rp::{
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH6, DMA_CH7, PIN_38, PIN_39, PIN_41, PIN_42, PIN_43, PIN_44, PIN_45, PIN_46, PIN_47, SPI1},
    spi::{self, Spi},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::prelude::*;
use moving_median::MovingMedian;
use st7789_async::{ColorOrder, Config as DisplayConfig, Orientation, Rotation, St7789};
use static_cell::{ConstStaticCell, StaticCell};
use touch_async::{Calibration, TouchPanel, TouchSample};

use crate::Irqs;

// ── Pin resources ─────────────────────────────────────────────────────────────

/// Pin and DMA resources for the shared `SPI1` panel bus (ADR-0008).
///
/// The panel occupies the contiguous high GPIO block 41–47, with the touch
/// controller on 38/39 and the battery ADC keeping GPIO 40. `DMA_CH6` carries
/// the bus's transmit direction and `DMA_CH7` its receive direction, because the
/// bus is full duplex.
pub struct PanelPins {
    /// `SPI1` peripheral instance.
    pub spi: embassy_rp::Peri<'static, SPI1>,
    /// SPI clock (GPIO 42).
    pub sck: embassy_rp::Peri<'static, PIN_42>,
    /// SPI data out / MOSI (GPIO 43).
    pub mosi: embassy_rp::Peri<'static, PIN_43>,
    /// SPI data in / MISO (GPIO 44).
    pub miso: embassy_rp::Peri<'static, PIN_44>,
    /// DMA channel for the bus's transmit direction.
    pub tx_dma: embassy_rp::Peri<'static, DMA_CH6>,
    /// DMA channel for the bus's receive direction.
    pub rx_dma: embassy_rp::Peri<'static, DMA_CH7>,
    /// Display chip select (GPIO 41).
    pub display_cs: embassy_rp::Peri<'static, PIN_41>,
    /// Display data/command (GPIO 45).
    pub dc: embassy_rp::Peri<'static, PIN_45>,
    /// Display reset (GPIO 46).
    pub rst: embassy_rp::Peri<'static, PIN_46>,
    /// Display backlight (GPIO 47).
    pub blk: embassy_rp::Peri<'static, PIN_47>,
    /// Touch chip select (GPIO 38).
    pub touch_cs: embassy_rp::Peri<'static, PIN_38>,
    /// Touch pen-down interrupt (GPIO 39, active-low with a pull-up).
    pub penirq: embassy_rp::Peri<'static, PIN_39>,
}

// ── Bus and framebuffer ───────────────────────────────────────────────────────

/// The shared full-duplex `SPI1` bus carrying both panel devices.
pub type PanelBus = Mutex<CriticalSectionRawMutex, Spi<'static, SPI1, spi::Async>>;

/// The chip-select-aware SPI device both panel drivers speak through.
pub type PanelDevice =
    SpiDeviceWithConfig<'static, CriticalSectionRawMutex, Spi<'static, SPI1, spi::Async>, Output<'static>>;

/// The ST7789 display driver, exposed so the UI task can draw into it.
pub type Display = St7789<'static, PanelDevice, Output<'static>>;

/// The touch driver owned by the panel.
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

/// Panel reset low pulse, in milliseconds.
const PANEL_RESET_LOW_MS: u64 = 10;

/// Panel settle after the reset is released, in milliseconds.
const PANEL_RESET_HIGH_MS: u64 = 120;

/// Statically-allocated framebuffer (`153_600` bytes of zeroed `.bss`).
static FB: ConstStaticCell<[u8; FB_LEN]> = ConstStaticCell::new([0; FB_LEN]);

/// Static storage for the shared bus, created once by [`Panel::new`].
static PANEL_SPI_BUS: StaticCell<PanelBus> = StaticCell::new();

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

/// Build the panel's display configuration.
///
/// This panel is RGB-ordered (with `Bgr` the red and blue channels swap), and
/// the 90° rotation gives the readable landscape orientation that the mirrored
/// [`CALIBRATION`] was derived for.
const fn display_config() -> DisplayConfig {
    DisplayConfig {
        color_order: ColorOrder::Rgb,
        orientation: Orientation::new().rotate(Rotation::Deg90),
        invert_colors: false,
    }
}

// ── Touch tuning ──────────────────────────────────────────────────────────────

/// Length of the moving-median window, in samples, for the raw `x`/`y` counts.
const MEDIAN_WINDOW: usize = 5;

/// The panel's measured touch calibration, mirrored for this orientation.
///
/// [`Calibration::MEASURED`] was measured in the `touch_coexistence`
/// orientation, which is this landscape orientation plus a horizontal mirror; a
/// mirror reverses the on-screen X axis, so mirroring the raw X endpoints
/// derives this orientation's calibration from the measured values.
const CALIBRATION: Calibration = Calibration::MEASURED.mirrored_x();

// ── Panel ─────────────────────────────────────────────────────────────────────

/// The panel hardware handed to the touch UI task.
///
/// Create one with [`Panel::new`] and drive it with [`Panel::bring_up`],
/// [`Panel::display_mut`], [`Panel::flush`], [`Panel::wait_for_pen_down`], and
/// [`Panel::read_touch`]. The caller owns it for the lifetime of the firmware,
/// so the display, the touch controller, and the framebuffer have a single
/// owner.
pub struct Panel {
    /// The ST7789 display driver.
    display: Display,
    /// The touch controller driver.
    touch: Touch,
    /// The pen-down interrupt line, the gesture's authority for pen-up.
    irq: Input<'static>,
    /// The display's reset line, pulsed once at first bring-up.
    rst: Output<'static>,
    /// The display's backlight line, held high while the panel is powered.
    blk: Output<'static>,
    /// Whether the backlight/reset pulse has already happened.
    reset_done: bool,
    /// The caller-side smoothing filters applied to raw touch samples.
    filters: Filters,
}

impl Panel {
    /// Take the panel's pins, build the shared bus, and construct the drivers.
    ///
    /// The backlight is turned on here; the reset pulse and controller
    /// initialisation happen in [`Panel::bring_up`], which is the first
    /// operation that must await. This must be called exactly once per boot:
    /// it consumes the static framebuffer.
    #[must_use]
    pub fn new(pins: PanelPins) -> Self {
        let bus = PANEL_SPI_BUS.init(new_shared_bus(
            pins.spi,
            pins.sck,
            pins.mosi,
            pins.miso,
            pins.tx_dma,
            pins.rx_dma,
            Irqs,
        ));
        let display_dev =
            SpiDeviceWithConfig::new(bus, Output::new(pins.display_cs, Level::High), display_spi_config());
        let touch_dev = SpiDeviceWithConfig::new(bus, Output::new(pins.touch_cs, Level::High), touch_spi_config());
        let width = u16::try_from(FB_W).unwrap_or(u16::MAX);
        let height = u16::try_from(FB_H).unwrap_or(u16::MAX);
        Self {
            display: St7789::new(display_dev, Output::new(pins.dc, Level::Low), FB.take(), width, height),
            touch: TouchPanel::new(touch_dev),
            irq: Input::new(pins.penirq, Pull::Up),
            rst: Output::new(pins.rst, Level::Low),
            blk: Output::new(pins.blk, Level::High),
            reset_done: false,
            filters: Filters::new(),
        }
    }

    /// Bring the display up, pulsing the reset line on the first call.
    ///
    /// Returns `true` once the display is initialised; `false` if every attempt
    /// failed, in which case the caller should back off and retry.
    pub async fn bring_up(&mut self) -> bool {
        if !self.reset_done {
            self.reset_done = true;
            self.blk.set_high();
            hardware_reset(&mut self.rst).await;
        }

        let config = display_config();
        for attempt in 1..=INIT_RETRIES {
            if self.display.init(&config, &mut Delay).await.is_ok() {
                info!("panel online");
                return true;
            }
            defmt::warn!("panel init failed (attempt {}/{})", attempt, INIT_RETRIES);
            Timer::after(INIT_RETRY_DELAY).await;
        }
        false
    }

    /// The display draw target the UI renders a full frame into.
    pub const fn display_mut(&mut self) -> &mut Display {
        &mut self.display
    }

    /// Flush the whole framebuffer to the panel.
    ///
    /// Returns `Err(())` if the transfer fails, which takes the panel offline
    /// until the next successful bring-up.
    pub async fn flush(&mut self) -> Result<(), ()> {
        self.display.flush().await.map_err(|_| ())
    }

    /// Wait for the pen to go down, returning its first calibrated pixel.
    ///
    /// The filters are cleared first so carried-over samples cannot pull the
    /// first point toward wherever the finger last was. Returns `None` if the
    /// wait or the sample transaction fails; the caller should retry after a
    /// tick.
    pub async fn wait_for_pen_down(&mut self) -> Option<Point> {
        match self.touch.wait_for_touch(&mut self.irq).await {
            Ok(raw) => {
                // Start each gesture with fresh filters: carried-over samples would
                // pull the first point toward wherever the finger last was.
                self.filters.clear();
                Some(self.filters.point(raw))
            }
            Err(_) => None,
        }
    }

    /// Read one sample while the pen is down.
    ///
    /// Returns `Ok(Some(point))` for a calibrated pixel, `Ok(None)` when the
    /// controller reports no contact, and `Err(())` on a transient bus error.
    /// `PENIRQ` remains the pen-up authority: a finger still down can produce
    /// `Ok(None)` on noise or `Err(())` on a bad read, so the gesture ends only
    /// once [`Panel::pen_up`] reports high.
    pub async fn read_touch(&mut self) -> Result<Option<Point>, ()> {
        match self.touch.read().await {
            Ok(Some(raw)) => Ok(Some(self.filters.point(raw))),
            Ok(None) => Ok(None),
            Err(_) => Err(()),
        }
    }

    /// Whether the pen has been lifted (`PENIRQ` is high).
    #[must_use]
    // `Input::is_high` is not `const`, so clippy's `missing_const_for_fn`
    // suggestion does not compile here.
    #[allow(clippy::missing_const_for_fn)]
    pub fn pen_up(&self) -> bool {
        self.irq.is_high()
    }
}

/// Pulse the panel's reset line low, then high, letting the controller settle.
async fn hardware_reset(rst: &mut Output<'static>) {
    rst.set_low();
    Timer::after(Duration::from_millis(PANEL_RESET_LOW_MS)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(PANEL_RESET_HIGH_MS)).await;
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
    /// This is the delivery boundary: the UI receives a calibrated [`Point`] in
    /// framebuffer space, never raw counts.
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
