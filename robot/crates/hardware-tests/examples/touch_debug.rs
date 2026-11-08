//! Touch bring-up diagnostic: dump the raw channels and the `PENIRQ` level.
//!
//! Unlike [`touch_probe`](super), this example never blocks on the interrupt. It
//! polls the controller directly and logs both the four raw conversions and the
//! current `PENIRQ` level every [`POLL_INTERVAL`]. That separates the two
//! independent failures a silent `touch_probe` can hide:
//!
//! - **SPI path** (SCK/MOSI/MISO/CS, power, protocol): the raw `x`/`y`/`z1`/`z2`
//!   columns. Touching should raise `z1`/`z2` above zero.
//! - **IRQ path** (wiring, pull-up, `PENIRQ` enabling): the `irq_low` column.
//!
//! Read it like this:
//!
//! | `irq_low` on touch | raw channels on touch | meaning |
//! |--------------------|-----------------------|---------|
//! | `true`             | non-zero `z1`         | both paths work; `touch_probe` should too |
//! | `false`            | non-zero `z1`         | SPI is fine; the problem is the IRQ line |
//! | either             | `z1` stays `0`        | SPI/power/protocol problem (check MISO/CS) |
//!
//! Run from the repository root with:
//!
//! ```sh
//! cargo run -p hardware-tests --example touch_debug --release
//! ```
//!
//! This is a temporary bring-up aid; it can be deleted once the panel is known.

#![no_std]
#![no_main]
// Demo code: `.unwrap()` on driver/GPIO results is intentional.
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
use embedded_hal_async::spi::{Operation, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_probe as _;

// Full-duplex SPI0 needs one DMA channel per direction: `DMA_IRQ_0` is bound to
// both the TX (`DMA_CH6`) and RX (`DMA_CH7`) handlers.
bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH6>, embassy_rp::dma::InterruptHandler<DMA_CH7>;
});

/// Command byte selecting the X channel.
const CMD_X: [u8; 1] = [0x90];
/// Command byte selecting the Y channel.
const CMD_Y: [u8; 1] = [0xD0];
/// Command byte selecting the Z1 channel.
const CMD_Z1: [u8; 1] = [0xB0];
/// Command byte selecting the Z2 channel.
const CMD_Z2: [u8; 1] = [0xC0];

/// SPI0 clock frequency in Hz (200 kHz).
const TOUCH_FREQ: u32 = 200_000;

/// Delay between diagnostic polls (300 ms).
const POLL_INTERVAL: Duration = Duration::from_millis(300);

/// Decode a two-byte big-endian frame into a raw 12-bit conversion.
const fn decode(frame: [u8; 2]) -> u16 {
    (u16::from_be_bytes(frame) >> 3) & 0x0FFF
}

/// The embassy entry point: poll the raw channels and `PENIRQ` level forever.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(embassy_rp::config::Config::default());
    info!("touch debug: raw channel dump + PENIRQ level (IRQ-independent)");
    info!("wiring: SCK=18 MOSI=19 MISO=16 T_CS=21 T_IRQ=22 (active low)");

    // PENIRQ is active-low, so pull it up and log its level each poll.
    let irq = Input::new(p.PIN_22, Pull::Up);

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
    let mut dev = ExclusiveDevice::new(spi, cs, Delay);

    info!("PENIRQ low before any read: {}", irq.is_low());
    info!("touch the panel; z1/z2 should rise above 0 and irq_low should become true");

    loop {
        let mut x = [0u8; 2];
        let mut y = [0u8; 2];
        let mut z1 = [0u8; 2];
        let mut z2 = [0u8; 2];
        let mut operations = [
            Operation::Write(&CMD_X),
            Operation::Read(&mut x),
            Operation::Write(&CMD_Y),
            Operation::Read(&mut y),
            Operation::Write(&CMD_Z1),
            Operation::Read(&mut z1),
            Operation::Write(&CMD_Z2),
            Operation::Read(&mut z2),
        ];

        if dev.transaction(&mut operations).await.is_ok() {
            info!(
                "irq_low={} raw x={} y={} z1={} z2={}",
                irq.is_low(),
                decode(x),
                decode(y),
                decode(z1),
                decode(z2)
            );
        } else {
            info!("SPI transaction failed");
        }

        Timer::after(POLL_INTERVAL).await;
    }
}
