//! The async VL53L0X wrapper.
//!
//! [`Rangefinder`] owns the I2C device, the open-drain `XSHUT` output and the
//! `INT` input. It delegates the ST initialisation sequence and ranging setup to
//! `edrv-vl53l0x`, but reads results itself so the raw range status is kept.

use edrv_vl53l0x::VL53L0X;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, digital::Wait, i2c::I2c};

use crate::{
    constants::{RANGING_PERIOD_MS, TIMING_BUDGET_US},
    types::RangefinderReading,
};

/// `RESULT_RANGE_STATUS`, whose bits 6:3 hold the 4-bit status code.
const REG_RESULT_RANGE_STATUS: u8 = 0x14;

/// `RESULT_RANGE_STATUS + 10`, the first of two big-endian range bytes.
const REG_RESULT_RANGE_STATUS_PLUS_10: u8 = 0x1E;

/// `SYSTEM_INTERRUPT_CLEAR`.
const REG_SYSTEM_INTERRUPT_CLEAR: u8 = 0x0B;

/// Delay after each `XSHUT` edge during a power cycle.
const XSHUT_DELAY_MS: u32 = 10;

/// Errors returned by [`Rangefinder`].
#[derive(Debug, Clone, Copy)]
pub enum Error<E> {
    /// The I2C bus failed.
    Bus(E),
    /// The device did not identify as a VL53L0X; carries the value read.
    InvalidDevice(u8),
    /// The driver's polling limit elapsed.
    Timeout,
    /// The sensor rejected the requested measurement timing budget.
    TimingBudget,
    /// The `XSHUT` line could not be driven.
    Shutdown,
    /// The `INT` line could not be awaited.
    Interrupt,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Self::Bus(error)
    }
}

/// An async VL53L0X rangefinder.
///
/// `XSHUT` must be an open-drain output: [`init`](Self::init) and
/// [`reset`](Self::reset) drive it low to hold the module off and release it
/// high to let the module's own pull-up bring it up. The wrapper never drives
/// the line push-pull high.
///
/// `INT` is the module's active-low interrupt, so [`wait_ready`](Self::wait_ready)
/// waits for a low level.
pub struct Rangefinder<I2C, XSHUT, INT>
where
    I2C: I2c,
    XSHUT: OutputPin,
    INT: Wait,
{
    /// The underlying ST driver, which owns the I2C device.
    sensor: VL53L0X<I2C>,
    /// The open-drain shutdown line.
    xshut: XSHUT,
    /// The active-low interrupt input.
    interrupt: INT,
}

impl<I2C, XSHUT, INT> Rangefinder<I2C, XSHUT, INT>
where
    I2C: I2c,
    XSHUT: OutputPin,
    INT: Wait,
{
    /// Create a wrapper for a sensor at the primary address (`0x29`).
    #[must_use]
    pub fn new(i2c: I2C, xshut: XSHUT, interrupt: INT) -> Self {
        Self {
            sensor: VL53L0X::new_primary(i2c),
            xshut,
            interrupt,
        }
    }

    /// Power-cycle the sensor, initialise it and start continuous ranging.
    ///
    /// Drives `XSHUT` low, waits 10 ms, releases it high, waits 10 ms, then
    /// runs the ST initialisation sequence, applies [`TIMING_BUDGET_US`] and
    /// starts continuous timed ranging every [`RANGING_PERIOD_MS`].
    ///
    /// # Errors
    ///
    /// Returns [`Error::Shutdown`] if `XSHUT` cannot be driven, or the mapped
    /// [`edrv_vl53l0x::Error`] if the sensor does not identify or the bus fails.
    pub async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.cycle_shutdown(delay).await?;
        self.sensor.init().await.map_err(map_error)?;
        let budget_set = self
            .sensor
            .set_measurement_timing_budget(TIMING_BUDGET_US)
            .await
            .map_err(map_error)?;
        if !budget_set {
            return Err(Error::TimingBudget);
        }
        self.sensor
            .start_continuous(RANGING_PERIOD_MS)
            .await
            .map_err(map_error)?;
        Ok(())
    }

    /// Read the latest sample: status first, then range, then clear `INT`.
    ///
    /// Reads register `0x14` (one byte), then the two big-endian range bytes at
    /// `0x1E..=0x1F`, then writes `0x0B = 0x01`. This is ST's order; the range
    /// must be captured before the interrupt is cleared. The `edrv` continuous
    /// helper is deliberately not used because it skips the status byte.
    ///
    /// # Errors
    ///
    /// Returns the mapped [`edrv_vl53l0x::Error`] if a bus transaction fails.
    pub async fn read(&mut self) -> Result<RangefinderReading, Error<I2C::Error>> {
        let status_byte = self.sensor.read_reg(REG_RESULT_RANGE_STATUS).await.map_err(map_error)?;
        let mut range = [0u8; 2];
        self.sensor
            .read_regs(REG_RESULT_RANGE_STATUS_PLUS_10, &mut range)
            .await
            .map_err(map_error)?;
        self.sensor
            .write_reg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            .await
            .map_err(map_error)?;
        Ok(RangefinderReading::decode(status_byte, u16::from_be_bytes(range)))
    }

    /// Wait for the sensor's interrupt to go low.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Interrupt`] if awaiting the `INT` line fails.
    pub async fn wait_ready(&mut self) -> Result<(), Error<I2C::Error>> {
        self.interrupt.wait_for_low().await.map_err(|_| Error::Interrupt)
    }

    /// Pulse `XSHUT` and re-initialise the sensor.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`init`](Self::init).
    pub async fn reset<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.init(delay).await
    }

    /// Stop continuous ranging and hold the sensor off via `XSHUT`.
    ///
    /// The `XSHUT` line is always driven low, even when stopping ranging fails:
    /// an explicit disable must power the module down rather than leave a wedged
    /// sensor running.
    ///
    /// # Errors
    ///
    /// Returns the mapped [`edrv_vl53l0x::Error`] if stopping ranging fails, or
    /// [`Error::Shutdown`] if `XSHUT` cannot be driven low.
    pub async fn standby(&mut self) -> Result<(), Error<I2C::Error>> {
        let stopped = self.sensor.stop_continuous().await.map_err(map_error);
        let released = self.xshut.set_low().map_err(|_| Error::Shutdown);
        stopped?;
        released?;
        Ok(())
    }

    /// Drive `XSHUT` low, wait, release it high, and wait.
    async fn cycle_shutdown<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.xshut.set_low().map_err(|_| Error::Shutdown)?;
        delay.delay_ms(XSHUT_DELAY_MS).await;
        self.xshut.set_high().map_err(|_| Error::Shutdown)?;
        delay.delay_ms(XSHUT_DELAY_MS).await;
        Ok(())
    }
}

/// Translate an `edrv-vl53l0x` error into this crate's error type.
///
/// The address variant cannot be produced by the wrapper because it never
/// changes the sensor's address; it is folded into `InvalidDevice`.
fn map_error<E>(error: edrv_vl53l0x::Error<E>) -> Error<E> {
    match error {
        edrv_vl53l0x::Error::Bus(error) => Error::Bus(error),
        edrv_vl53l0x::Error::InvalidDevice(id) => Error::InvalidDevice(id),
        edrv_vl53l0x::Error::Timeout => Error::Timeout,
        edrv_vl53l0x::Error::InvalidAddress(address) => Error::InvalidDevice(address),
    }
}
