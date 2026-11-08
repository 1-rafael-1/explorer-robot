//! The [`TouchPanel`] driver: owns the SPI device and issues raw conversions.

use embedded_hal_async::{
    digital::Wait,
    spi::{Operation, SpiDevice},
};

use crate::types::{Error, TouchSample};

/// Command byte selecting the X channel.
///
/// `0x90` is the start bit set, 12-bit conversion, single-ended channel select
/// with auto-power-down between conversions.
const CMD_X: [u8; 1] = [0x90];
/// Command byte selecting the Y channel (`0xD0`).
const CMD_Y: [u8; 1] = [0xD0];
/// Command byte selecting the Z1 channel (`0xB0`).
const CMD_Z1: [u8; 1] = [0xB0];
/// Command byte selecting the Z2 channel (`0xC0`).
const CMD_Z2: [u8; 1] = [0xC0];

/// An asynchronous XPT2046/TSC2046-class touch-panel driver.
///
/// The driver is HAL-agnostic: it holds any
/// [`SpiDevice`] and asserts chip select
/// around exactly one transaction per [`read`](Self::read).
pub struct TouchPanel<D: SpiDevice<u8>> {
    /// The chip-select-aware SPI device the panel is attached to.
    spi: D,
}

impl<D: SpiDevice<u8>> TouchPanel<D> {
    /// Create a driver owning the chip-select-aware SPI device `spi`.
    #[must_use]
    pub const fn new(spi: D) -> Self {
        Self { spi }
    }

    /// Read one raw sample, or `Ok(None)` when no touch is present.
    ///
    /// All four channels are read inside a single chip-select assertion: one
    /// two-byte [`Operation::Read`] against a preceding one-byte
    /// [`Operation::Write`] of X, Y, Z1, then Z2, all in one
    /// [`transaction`](SpiDevice::transaction) call.
    ///
    /// Each two-byte response is decoded big-endian as
    /// `(u16::from_be_bytes(frame) >> 3) & 0x0FFF`.
    ///
    /// No averaging, median filtering, or normalisation is performed.
    ///
    /// A touch is reported iff the decoded `z1` is non-zero. With the panel
    /// untouched there is no current path through the touch layer, so the Z1
    /// conversion reads zero; a touch completes that path and lifts Z1 above
    /// zero. The controller's `PENIRQ` line is the authoritative pen-down signal
    /// and is handled by [`wait_for_touch`](Self::wait_for_touch), so `read`
    /// does not consult it and can be polled without an interrupt pin wired.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Spi`] if the SPI transaction fails.
    pub async fn read(&mut self) -> Result<Option<TouchSample>, Error<D::Error>> {
        let mut frame_x = [0u8; 2];
        let mut frame_y = [0u8; 2];
        let mut frame_z1 = [0u8; 2];
        let mut frame_z2 = [0u8; 2];
        let mut operations = [
            Operation::Write(&CMD_X),
            Operation::Read(&mut frame_x),
            Operation::Write(&CMD_Y),
            Operation::Read(&mut frame_y),
            Operation::Write(&CMD_Z1),
            Operation::Read(&mut frame_z1),
            Operation::Write(&CMD_Z2),
            Operation::Read(&mut frame_z2),
        ];
        self.spi.transaction(&mut operations).await.map_err(Error::Spi)?;

        let sample = TouchSample {
            x: decode(frame_x),
            y: decode(frame_y),
            z1: decode(frame_z1),
            z2: decode(frame_z2),
        };
        if sample.z1 == 0 { Ok(None) } else { Ok(Some(sample)) }
    }

    /// Await a pen-down edge and return one sample.
    ///
    /// `PENIRQ` is active-low, so this awaits [`Wait::wait_for_low`]. Before
    /// waiting it performs one discarded [`read`](Self::read): the controller
    /// only asserts `PENIRQ` once it has been put into auto-power-down mode,
    /// which the PD bits of a conversion command select. On a cold start no
    /// conversion has run, so `PENIRQ` is not yet armed and would never fire.
    ///
    /// A low level that yields no valid sample ([`read`](Self::read) returning
    /// `Ok(None)`) is not an error — it can happen on noise or when the pen was
    /// released mid-read. Rather than awaiting the next low edge immediately,
    /// which would busy-spin on a line that is still low, the loop first waits
    /// for `PENIRQ` to return high (pen release) and only then awaits the next
    /// low edge. That makes every read after the first correspond to a genuine
    /// new pen-down instead of hammering SPI/CPU on a stuck-low line.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Wait`] if awaiting the interrupt fails, or
    /// [`Error::Spi`] if the SPI transaction fails.
    pub async fn wait_for_touch<W>(&mut self, irq: &mut W) -> Result<TouchSample, Error<D::Error, W::Error>>
    where
        W: Wait,
    {
        // Arm `PENIRQ` with one discard conversion (see the method docs).
        let _ = self.read().await.map_err(lift_wait)?;

        loop {
            irq.wait_for_low().await.map_err(Error::Wait)?;
            match self.read().await {
                Ok(Some(sample)) => return Ok(sample),
                // No valid sample while the line is low: wait for release so the
                // next `wait_for_low` is a new pen-down, not an immediate re-read
                // of a line that never went high.
                Ok(None) => irq.wait_for_high().await.map_err(Error::Wait)?,
                Err(error) => return Err(lift_wait(error)),
            }
        }
    }

    /// Give the SPI device back to the caller.
    #[must_use]
    pub fn release(self) -> D {
        self.spi
    }
}

/// Re-tag a [`read`](TouchPanel::read) error, whose wait error is
/// [`Infallible`](core::convert::Infallible), for a caller that supplies a real
/// interrupt wait type.
fn lift_wait<SpiE, WaitE>(error: Error<SpiE, core::convert::Infallible>) -> Error<SpiE, WaitE> {
    match error {
        Error::Spi(e) => Error::Spi(e),
        // `read` cannot produce a wait error (its `WaitE` is `Infallible`).
        Error::Wait(never) => match never {},
    }
}

/// Decode a two-byte big-endian frame into a raw 12-bit conversion.
///
/// The controller returns the conversion left-justified in the 16-bit frame, so
/// the three least-significant padding bits are shifted out.
const fn decode(frame: [u8; 2]) -> u16 {
    (u16::from_be_bytes(frame) >> 3) & 0x0FFF
}
