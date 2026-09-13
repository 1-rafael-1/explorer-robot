//! Touch probe: bring up the unmarked resistive touch controller alone.
//!
//! The 2.8″ panel's touch header (`T_CLK`/`T_CS`/`T_DIN`/`T_DO`/`T_IRQ`) is
//! wired to a dedicated, full-duplex SPI0 bus and nothing else, so the
//! controller can be characterised before the display and touch layer are made
//! to share a bus. The IC markings are sanded off, so this example exists to
//! prove the assumed XPT2046/TSC2046-class protocol — including the `Z1`/`Z2`
//! pressure channels that the reference design never exercised — and to capture
//! the raw counts needed to calibrate.
//!
//! `PENIRQ` is active-low: [`TouchPanel::wait_for_touch`] blocks until it goes
//! low, then returns one valid raw sample. While the pen is held down the
//! example polls `read()` every 50 ms; on pen-up it prints the raw extents seen
//! so far. The operator is asked to touch each screen corner in turn so that,
//! after a full pass, the extents span the panel's raw range.
//!
//! Run from the repository root with:
//!
//! ```sh
//! cargo run -p hardware-tests --example touch_probe --release
//! ```
//!
//! # Confirming the protocol
//!
//! The logged `z1`/`z2` values and the `read()` `Some`/`None` behaviour are the
//! evidence. With the panel untouched there is no current path through the touch
//! layer, so `z1` decodes as `0` and `read()` returns `None`; a real touch
//! completes that path and lifts `z1` above `0`. After each touch the example
//! logs a verdict on the `Z2` channel, which is the one the reference design
//! never exercised. If `z1` stays `0` on touch, or `z2` never rises above `0`,
//! the assumed XPT2046/TSC2046-class framing is not confirmed by this run.

#![no_std]
#![no_main]
// Demo code: `.unwrap()` on driver/GPIO results is intentional, and the
// `main` future is large because of the driver's transaction buffers.
#![allow(clippy::unwrap_used, clippy::large_futures, clippy::used_underscore_binding)]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH6, DMA_CH7},
    spi::{self, Spi},
};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_probe as _;
use touch_async::{TouchPanel, TouchSample};

// Full-duplex SPI0 needs one DMA channel per direction: `DMA_IRQ_0` is bound to
// both the TX (`DMA_CH6`) and RX (`DMA_CH7`) handlers.
bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH6>, embassy_rp::dma::InterruptHandler<DMA_CH7>;
});

/// SPI0 clock frequency in Hz (200 kHz): the low speed the XPT2046-class
/// controller is specified for.
const TOUCH_FREQ: u32 = 200_000;

/// Delay between polls while the pen is held down (50 ms).
const POLL_INTERVAL: Duration = Duration::from_millis(50);

/// Running raw-sample extents, accumulated across every touch since startup.
///
/// Each bound is an `Option<u16>` so that "this channel has never been
/// observed" stays distinct from a legitimate raw count of `0` (no sentinel).
/// The operator touches each screen corner in turn, so after a full pass the
/// bounds span the panel's raw range and can seed calibration.
struct Extents {
    /// Smallest raw `x` observed, or `None` before the first sample.
    x_min: Option<u16>,
    /// Largest raw `x` observed, or `None` before the first sample.
    x_max: Option<u16>,
    /// Smallest raw `y` observed, or `None` before the first sample.
    y_min: Option<u16>,
    /// Largest raw `y` observed, or `None` before the first sample.
    y_max: Option<u16>,
    /// Smallest raw `z1` observed, or `None` before the first sample.
    z1_min: Option<u16>,
    /// Largest raw `z1` observed, or `None` before the first sample.
    z1_max: Option<u16>,
    /// Smallest raw `z2` observed, or `None` before the first sample.
    z2_min: Option<u16>,
    /// Largest raw `z2` observed, or `None` before the first sample.
    z2_max: Option<u16>,
}

impl Extents {
    /// Start with every bound unset.
    const fn new() -> Self {
        Self {
            x_min: None,
            x_max: None,
            y_min: None,
            y_max: None,
            z1_min: None,
            z1_max: None,
            z2_min: None,
            z2_max: None,
        }
    }

    /// Fold one raw sample into the running bounds.
    fn update(&mut self, sample: TouchSample) {
        update_bound(&mut self.x_min, &mut self.x_max, sample.x);
        update_bound(&mut self.y_min, &mut self.y_max, sample.y);
        update_bound(&mut self.z1_min, &mut self.z1_max, sample.z1);
        update_bound(&mut self.z2_min, &mut self.z2_max, sample.z2);
    }

    /// Log the bounds observed so far, one line per channel.
    fn log(&self) {
        info!("raw extents so far (min..max):");
        info!("  x  {} .. {}", self.x_min, self.x_max);
        info!("  y  {} .. {}", self.y_min, self.y_max);
        info!("  z1 {} .. {}", self.z1_min, self.z1_max);
        info!("  z2 {} .. {}", self.z2_min, self.z2_max);
    }

    /// Log whether the `Z` (pressure) channels actually responded.
    ///
    /// A valid `read()` already guarantees `z1 > 0`, so the interesting evidence
    /// is `z2`: the reference design never read it, and this is where the
    /// unproven part of the protocol is confirmed or called out.
    fn log_z_verdict(&self) {
        match self.z2_max {
            Some(z2) if z2 > 0 => {
                info!("Z path: z2 reached {} (>0) — Z1/Z2 channels confirmed", z2);
            }
            Some(z2) => {
                info!("Z path: z2 never rose above {} — Z2 channel NOT confirmed", z2);
            }
            None => info!("Z path: no valid samples captured yet"),
        }
    }
}

/// Widen the `min`/`max` pair to include `value`, treating an unset bound as
/// absent rather than as a sentinel.
fn update_bound(min: &mut Option<u16>, max: &mut Option<u16>, value: u16) {
    *min = Some((*min).map_or(value, |current| current.min(value)));
    *max = Some((*max).map_or(value, |current| current.max(value)));
}

/// The embassy entry point: bring up the touch controller alone on SPI0, then
/// log every interrupt transition and raw sample until reset.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(embassy_rp::config::Config::default());

    info!("touch probe: unmarked XPT2046/TSC2046-class controller on SPI0");
    info!("wiring: SCK=18 MOSI=19 MISO=16 T_CS=21 T_IRQ=22 (active low)");
    info!("touch each screen corner in turn (hold briefly) to capture raw extents");
    info!("evidence: untouched should read z1=0 (read() => None); a touch lifts z1>0 (=> Some)");

    // PENIRQ is active-low, so pull it up and treat a low level as pen-down.
    let mut irq = Input::new(p.PIN_22, Pull::Up);

    // Touch chip select, idle high so the controller stays deselected.
    let cs = Output::new(p.PIN_21, Level::High);

    // Full-duplex SPI0 with per-transaction DMA, Mode 3 at 200 kHz.
    let mut spi_config = spi::Config::default();
    spi_config.frequency = TOUCH_FREQ;
    spi_config.phase = spi::Phase::CaptureOnSecondTransition;
    spi_config.polarity = spi::Polarity::IdleHigh;
    let spi = Spi::new(
        p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, p.DMA_CH6, p.DMA_CH7, Irqs, spi_config,
    );

    // Exclusive CS-toggling device on the dedicated bus; this is the async
    // `SpiDevice<u8>` the driver wants.
    let dev = ExclusiveDevice::new(spi, cs, Delay);
    let mut panel = TouchPanel::new(dev);

    let mut extents = Extents::new();

    loop {
        // Await the pen-down edge; the driver's generic `digital::Wait` path
        // keeps retrying until a valid sample is returned.
        let first = panel.wait_for_touch(&mut irq).await.unwrap();
        info!(
            "PENIRQ low (touch down): first raw x={} y={} z1={} z2={}",
            first.x, first.y, first.z1, first.z2
        );
        extents.update(first);

        // Keep sampling while the pen is held down.
        while irq.is_low() {
            Timer::after(POLL_INTERVAL).await;
            if let Some(sample) = panel.read().await.unwrap() {
                info!("raw x={} y={} z1={} z2={}", sample.x, sample.y, sample.z1, sample.z2);
                extents.update(sample);
            } else {
                info!("(no valid sample)");
            }
        }

        info!("PENIRQ high (touch up)");
        extents.log();
        extents.log_z_verdict();
    }
}
