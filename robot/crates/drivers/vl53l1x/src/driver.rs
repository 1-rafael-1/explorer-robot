//! The async VL53L1X driver.
//!
//! [`Rangefinder`] owns the I2C device, the open-drain `XSHUT` output and the
//! active-low `INT` input. The register sequences are ported from
//! `vl53l1x-simple` 0.2.0 (MIT OR Apache-2.0); see the crate documentation for
//! provenance.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, digital::Wait, i2c::I2c};

use crate::{
    registers as reg,
    results::{RESULT_BLOCK_LEN, ResultBlock, TARGET_RATE},
    timing::{
        TIMING_GUARD_US, calc_macro_period, decode_timeout, encode_timeout, timeout_mclks_to_microseconds,
        timeout_microseconds_to_mclks,
    },
    types::RangefinderReading,
};

/// The model ID a VL53L1X reports at `IDENTIFICATION__MODEL_ID`.
const MODEL_ID: u16 = 0xEACC;

/// The module's default 7-bit I2C address.
const DEFAULT_ADDRESS: u8 = 0x29;

/// Delay after each `XSHUT` edge during a power cycle, in milliseconds.
const XSHUT_DELAY_MS: u32 = 10;

/// Delay after each soft-reset write, in milliseconds.
const SOFT_RESET_DELAY_MS: u32 = 1;

/// Boot-completion polling limit, in milliseconds.
const BOOT_TIMEOUT_MS: u32 = 500;

/// Delay between boot-completion polls, in milliseconds.
const BOOT_POLL_DELAY_MS: u32 = 1;

/// Short distance mode measurement timing budget, in microseconds.
pub const SHORT_TIMING_BUDGET_US: u32 = 33_000;

/// The commanded inter-measurement period, in milliseconds.
pub const INTERMEASUREMENT_PERIOD_MS: u32 = 50;

/// The oscillator calibration value the reference uses when the sensor reports
/// zero, which would otherwise produce a zero inter-measurement period.
const DEFAULT_OSC_CALIBRATE_VAL: u16 = 256;

/// Errors returned by [`Rangefinder`].
#[derive(Debug, Clone, Copy)]
pub enum Error<E> {
    /// The I2C bus failed.
    Bus(E),
    /// The device did not identify as a VL53L1X; carries the raw model ID.
    InvalidDevice(u16),
    /// The boot poll or the oscillator calibration did not produce a usable
    /// value before the driver's limit elapsed.
    Timeout,
    /// The sensor rejected the requested measurement timing budget.
    TimingBudget,
    /// The `XSHUT` line could not be driven.
    Shutdown,
    /// The `INT` line could not be awaited.
    Interrupt,
    /// The sensor had no new reading ready for this read.
    NotReady,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Self::Bus(error)
    }
}

/// An async VL53L1X rangefinder.
///
/// `XSHUT` must be an open-drain output: [`init`](Self::init) and
/// [`reset`](Self::reset) drive it low to hold the module off and release it
/// high to let the module's own pull-up bring it up. The driver never drives the
/// line push-pull high.
///
/// `INT` is the module's active-low interrupt, so
/// [`wait_ready`](Self::wait_ready) waits for a low level. The driver also
/// checks `GPIO__TIO_HV_STATUS` before reading, so it does not depend on the
/// interrupt edge alone.
pub struct Rangefinder<I2C, XSHUT, INT>
where
    I2C: I2c,
    XSHUT: OutputPin,
    INT: Wait,
{
    /// The I2C device at the module's default address.
    i2c: I2C,
    /// The open-drain shutdown line.
    xshut: XSHUT,
    /// The active-low interrupt input.
    interrupt: INT,
    /// The 7-bit address the device answers on.
    address: u8,
    /// The measured fast oscillator frequency, in 4.12 format.
    fast_osc_frequency: u16,
    /// The oscillator calibration value used to scale timeouts.
    osc_calibrate_val: u16,
    /// Whether the one-time VHV/phasecal setup has run.
    calibrated: bool,
}

impl<I2C, XSHUT, INT> Rangefinder<I2C, XSHUT, INT>
where
    I2C: I2c,
    XSHUT: OutputPin,
    INT: Wait,
{
    /// Create a driver for a sensor at the primary address (`0x29`).
    #[must_use]
    pub const fn new(i2c: I2C, xshut: XSHUT, interrupt: INT) -> Self {
        Self {
            i2c,
            xshut,
            interrupt,
            address: DEFAULT_ADDRESS,
            fast_osc_frequency: 0,
            osc_calibrate_val: 0,
            calibrated: false,
        }
    }

    /// Bring the sensor up and start continuous timed ranging.
    ///
    /// Pulses `XSHUT` low then high, requires the `0xEACC` model ID, soft-resets
    /// and polls the firmware status for up to 500 ms while treating NACKs as
    /// "still booting". Then it configures the 2.8 V pads, the active-low
    /// range/error `GPIO1` interrupt and the new-reading-ready interrupt, applies
    /// the static configuration subset and short distance mode at a 33 ms
    /// timing budget, sets the part-to-part offset and the 50 ms
    /// inter-measurement period, clears the interrupt and starts timed ranging.
    ///
    /// The one-time VHV/phasecal calibration is deferred to the first
    /// [`read`](Self::read), because it consumes that measurement's phasecal
    /// result.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Shutdown`] if `XSHUT` cannot be driven,
    /// [`Error::InvalidDevice`] with the raw ID if the sensor is not a
    /// VL53L1X, [`Error::Timeout`] if boot does not complete,
    /// [`Error::TimingBudget`] if the budget is out of range, or
    /// [`Error::Bus`] if an I2C transaction fails.
    pub async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.power_cycle(delay).await?;

        let model_id = self.read_u16(reg::IDENTIFICATION_MODEL_ID).await?;
        if model_id != MODEL_ID {
            return Err(Error::InvalidDevice(model_id));
        }

        self.write_u8(reg::SOFT_RESET, 0x00).await?;
        delay.delay_ms(SOFT_RESET_DELAY_MS).await;
        self.write_u8(reg::SOFT_RESET, 0x01).await?;
        delay.delay_ms(SOFT_RESET_DELAY_MS).await;

        self.wait_for_boot(delay).await?;

        let pad_config = self.read_u8(reg::PAD_I2C_HV_EXTSUP_CONFIG).await?;
        self.write_u8(reg::PAD_I2C_HV_EXTSUP_CONFIG, pad_config | 0x01).await?;

        self.fast_osc_frequency = self.read_u16(reg::OSC_MEASURED_FAST_OSC_FREQUENCY).await?;
        // A zero frequency would divide by zero in the timing maths below; the
        // sensor has not produced a usable calibration, so fail closed.
        if self.fast_osc_frequency == 0 {
            return Err(Error::Timeout);
        }
        self.osc_calibrate_val = self.read_u16(reg::RESULT_OSC_CALIBRATE_VAL).await?;

        self.configure_static().await?;

        // The poll-only reference never writes these; without them GPIO1 may
        // never assert or may be active high. Active low is safer for a
        // powered-down module, whose line idles high through the pull-up.
        self.write_u8(reg::GPIO_HV_MUX__CTRL, 0x11).await?;
        self.write_u8(reg::SYSTEM_INTERRUPT_CONFIG_GPIO, 0x20).await?;

        self.set_distance_mode_short().await?;
        self.set_measurement_timing_budget(SHORT_TIMING_BUDGET_US).await?;

        let range_offset = u32::from(self.read_u16(reg::MM_CONFIG_OUTER_OFFSET_MM).await?) * 4;
        let range_offset = u16::try_from(range_offset).unwrap_or(u16::MAX);
        self.write_u16(reg::ALGO_PART_TO_PART_RANGE_OFFSET_MM, range_offset)
            .await?;

        self.start_ranging().await?;

        self.calibrated = false;
        Ok(())
    }

    /// Read the latest Rangefinder Reading.
    ///
    /// Checks `GPIO__TIO_HV_STATUS` first. If no reading is ready it clears the
    /// interrupt so the task cannot spin, and returns [`Error::NotReady`]
    /// without consuming a stale block. Otherwise it reads the 17-byte result
    /// block, runs the one-time calibration on the first result, updates the DSS
    /// selection, decodes the reading and diagnostics, and clears the interrupt
    /// only after the block is consumed — that clear re-arms the next
    /// measurement.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotReady`] for a spurious wake, or [`Error::Bus`] if an
    /// I2C transaction fails.
    pub async fn read(&mut self) -> Result<RangefinderReading, Error<I2C::Error>> {
        let gpio_status = self.read_u8(reg::GPIO_TIO_HV_STATUS).await?;
        if gpio_status & 0x01 != 0 {
            self.clear_interrupt().await?;
            return Err(Error::NotReady);
        }

        let mut data = [0u8; RESULT_BLOCK_LEN];
        self.i2c
            .write_read(self.address, &reg::RESULT_RANGE_STATUS.to_be_bytes(), &mut data)
            .await?;
        let block = ResultBlock::parse(&data);

        if !self.calibrated {
            self.setup_manual_calibration().await?;
            self.calibrated = true;
        }

        self.update_dss(&block).await?;

        let reading = block.reading();

        self.clear_interrupt().await?;

        Ok(reading)
    }

    /// Wait for the sensor's interrupt to go low.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Interrupt`] if awaiting the `INT` line fails.
    pub async fn wait_ready(&mut self) -> Result<(), Error<I2C::Error>> {
        self.interrupt.wait_for_low().await.map_err(|_| Error::Interrupt)
    }

    /// Pulse `XSHUT` and re-run the full bring-up.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`init`](Self::init).
    pub async fn reset<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.init(delay).await
    }

    /// Stop ranging and hold the sensor off via `XSHUT`.
    ///
    /// The `XSHUT` line is always driven low, even when stopping ranging fails:
    /// an explicit disable must power the module down rather than leave a
    /// wedged sensor running.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Bus`] if stopping ranging fails, or [`Error::Shutdown`]
    /// if `XSHUT` cannot be driven low.
    pub async fn standby(&mut self) -> Result<(), Error<I2C::Error>> {
        let stopped = self.write_u8(reg::SYSTEM_MODE_START, 0x00).await;
        let shutdown = self.xshut.set_low().map_err(|_| Error::Shutdown);
        stopped?;
        shutdown?;
        Ok(())
    }

    /// Drive `XSHUT` low, wait, release it high, and wait.
    async fn power_cycle<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.xshut.set_low().map_err(|_| Error::Shutdown)?;
        delay.delay_ms(XSHUT_DELAY_MS).await;
        self.xshut.set_high().map_err(|_| Error::Shutdown)?;
        delay.delay_ms(XSHUT_DELAY_MS).await;
        Ok(())
    }

    /// Poll the firmware status until boot completes or the limit elapses.
    ///
    /// A NACK during boot is expected and treated as "still booting" rather than
    /// as a hard bus error. The generic `I2C::Error` cannot distinguish a NACK
    /// from another bus error, so every error during this poll is tolerated; a
    /// device that never answers ends as [`Error::Timeout`].
    async fn wait_for_boot<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        let mut elapsed_ms = 0;
        loop {
            let ready = matches!(self.read_u8(reg::FIRMWARE_SYSTEM_STATUS).await, Ok(status) if status & 0x01 != 0);
            if ready {
                return Ok(());
            }
            if elapsed_ms >= BOOT_TIMEOUT_MS {
                return Err(Error::Timeout);
            }
            delay.delay_ms(BOOT_POLL_DELAY_MS).await;
            elapsed_ms += BOOT_POLL_DELAY_MS;
        }
    }

    /// Write the reference's static-configuration subset.
    async fn configure_static(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_u16(reg::DSS_CONFIG_TARGET_TOTAL_RATE_MCPS, TARGET_RATE)
            .await?;
        self.write_u8(reg::GPIO_TIO_HV_STATUS, 0x02).await?;
        self.write_u8(reg::SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH_NS, 8).await?;
        self.write_u8(reg::SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH_NS, 16)
            .await?;
        self.write_u8(reg::ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01)
            .await?;
        self.write_u8(reg::ALGO_RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF).await?;
        self.write_u8(reg::ALGO_RANGE_MIN_CLIP, 0).await?;
        self.write_u8(reg::ALGO_CONSISTENCY_CHECK_TOLERANCE, 2).await?;

        self.write_u16(reg::SYSTEM_THRESH_RATE_HIGH_HI, 0x0000).await?;
        self.write_u16(reg::SYSTEM_THRESH_RATE_HIGH_LO, 0x0000).await?;
        self.write_u8(reg::DSS_CONFIG_APERTURE_ATTENUATION, 0x38).await?;

        self.write_u16(reg::RANGE_CONFIG_SIGMA_THRESH, 360).await?;
        self.write_u16(reg::RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192)
            .await?;

        self.write_u8(reg::SYSTEM_GROUPED_PARAMETER_HOLD_0, 0x01).await?;
        self.write_u8(reg::SYSTEM_GROUPED_PARAMETER_HOLD_1, 0x01).await?;
        self.write_u8(reg::SD_CONFIG_QUANTIFIER, 2).await?;

        // GPH is 0 after reset, but writing GPH0/GPH1 above sets it to 1; the
        // timed-ranging preset brings it back to 0.
        self.write_u8(reg::SYSTEM_GROUPED_PARAMETER_HOLD, 0x00).await?;
        self.write_u8(reg::SYSTEM_SEED_CONFIG, 1).await?;

        // Low-power auto mode: VHV, PHASECAL, DSS1, RANGE.
        self.write_u8(reg::SYSTEM_SEQUENCE_CONFIG, 0x8B).await?;
        self.write_u16(reg::DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8)
            .await?;
        self.write_u8(reg::DSS_CONFIG_ROI_MODE_CONTROL, 2).await?;

        Ok(())
    }

    /// Apply short distance mode, preserving the current timing budget.
    async fn set_distance_mode_short(&mut self) -> Result<(), Error<I2C::Error>> {
        let budget_us = self.measurement_timing_budget().await?;

        self.write_u8(reg::RANGE_CONFIG_VCSEL_PERIOD_A, 0x07).await?;
        self.write_u8(reg::RANGE_CONFIG_VCSEL_PERIOD_B, 0x05).await?;
        self.write_u8(reg::RANGE_CONFIG_VALID_PHASE_HIGH, 0x38).await?;

        self.write_u8(reg::SD_CONFIG_WOI_SD0, 0x07).await?;
        self.write_u8(reg::SD_CONFIG_WOI_SD1, 0x05).await?;
        self.write_u8(reg::SD_CONFIG_INITIAL_PHASE_SD0, 6).await?;
        self.write_u8(reg::SD_CONFIG_INITIAL_PHASE_SD1, 6).await?;

        self.set_measurement_timing_budget(budget_us).await
    }

    /// Read back the configured measurement timing budget, in microseconds.
    async fn measurement_timing_budget(&mut self) -> Result<u32, Error<I2C::Error>> {
        let period = self.read_u8(reg::RANGE_CONFIG_VCSEL_PERIOD_A).await?;
        let macro_period_us = calc_macro_period(self.fast_osc_frequency, period);
        let range_timeout_mclks = decode_timeout(self.read_u16(reg::RANGE_CONFIG_TIMEOUT_MACROP_A).await?);
        let range_timeout_us = timeout_mclks_to_microseconds(range_timeout_mclks, macro_period_us);

        Ok(2 * range_timeout_us + TIMING_GUARD_US)
    }

    /// Apply a measurement timing budget, in microseconds.
    async fn set_measurement_timing_budget(&mut self, budget_us: u32) -> Result<(), Error<I2C::Error>> {
        if budget_us <= TIMING_GUARD_US {
            return Err(Error::TimingBudget);
        }

        let budget_us = budget_us - TIMING_GUARD_US;
        if budget_us > 1_100_000 {
            return Err(Error::TimingBudget);
        }
        let range_timeout_us = budget_us / 2;

        let period_a = self.read_u8(reg::RANGE_CONFIG_VCSEL_PERIOD_A).await?;
        let macro_period_a = calc_macro_period(self.fast_osc_frequency, period_a);

        let phasecal_timeout_mclks = timeout_microseconds_to_mclks(1000, macro_period_a).min(0xFF);
        let phasecal_timeout_mclks = u8::try_from(phasecal_timeout_mclks).unwrap_or(u8::MAX);
        self.write_u8(reg::PHASECAL_CONFIG_TIMEOUT_MACROP, phasecal_timeout_mclks)
            .await?;

        self.write_u16(
            reg::MM_CONFIG_TIMEOUT_MACROP_A,
            encode_timeout(timeout_microseconds_to_mclks(1, macro_period_a)),
        )
        .await?;
        self.write_u16(
            reg::RANGE_CONFIG_TIMEOUT_MACROP_A,
            encode_timeout(timeout_microseconds_to_mclks(range_timeout_us, macro_period_a)),
        )
        .await?;

        let period_b = self.read_u8(reg::RANGE_CONFIG_VCSEL_PERIOD_B).await?;
        let macro_period_b = calc_macro_period(self.fast_osc_frequency, period_b);

        self.write_u16(
            reg::MM_CONFIG_TIMEOUT_MACROP_B,
            encode_timeout(timeout_microseconds_to_mclks(1, macro_period_b)),
        )
        .await?;
        self.write_u16(
            reg::RANGE_CONFIG_TIMEOUT_MACROP_B,
            encode_timeout(timeout_microseconds_to_mclks(range_timeout_us, macro_period_b)),
        )
        .await?;

        Ok(())
    }

    /// Set the inter-measurement period, clear the interrupt and start ranging.
    async fn start_ranging(&mut self) -> Result<(), Error<I2C::Error>> {
        let osc_calibrate_val = if self.osc_calibrate_val == 0 {
            DEFAULT_OSC_CALIBRATE_VAL
        } else {
            self.osc_calibrate_val
        };
        let period_ms = INTERMEASUREMENT_PERIOD_MS * u32::from(osc_calibrate_val);
        self.write_u32(reg::SYSTEM_INTERMEASUREMENT_PERIOD, period_ms).await?;
        self.clear_interrupt().await?;
        self.write_u8(reg::SYSTEM_MODE_START, 0x40).await?;
        Ok(())
    }

    /// Run ST's one-time manual VHV/phasecal setup on the first result.
    async fn setup_manual_calibration(&mut self) -> Result<(), Error<I2C::Error>> {
        let saved_vhv_init = self.read_u8(reg::VHV_CONFIG_INIT).await?;
        let saved_vhv_timeout = self.read_u8(reg::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND).await?;

        self.write_u8(reg::VHV_CONFIG_INIT, saved_vhv_init & 0x7F).await?;
        self.write_u8(
            reg::VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND,
            (saved_vhv_timeout & 0x03) + (3 << 2),
        )
        .await?;
        self.write_u8(reg::PHASECAL_CONFIG_OVERRIDE, 0x01).await?;

        let phasecal_result = self.read_u8(reg::PHASECAL_RESULT_VCSEL_START).await?;
        self.write_u8(reg::CAL_CONFIG_VCSEL_START, phasecal_result).await?;
        Ok(())
    }

    /// Update the DSS effective-SPAD selection from the latest rates.
    async fn update_dss(&mut self, block: &ResultBlock) -> Result<(), Error<I2C::Error>> {
        // A rate or SPAD count that would divide by zero falls back to the
        // mid-point target instead of failing the read.
        let required_spads = block.required_spads().unwrap_or(0x8000);
        self.write_u16(reg::DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT, required_spads)
            .await?;
        Ok(())
    }

    /// Clear the range interrupt so the next measurement can complete.
    async fn clear_interrupt(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_u8(reg::SYSTEM_INTERRUPT_CLEAR, 0x01).await?;
        Ok(())
    }

    /// Read one byte at a 16-bit register address.
    async fn read_u8(&mut self, register: u16) -> Result<u8, I2C::Error> {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(self.address, &register.to_be_bytes(), &mut data)
            .await?;
        Ok(data[0])
    }

    /// Read two big-endian bytes at a 16-bit register address.
    async fn read_u16(&mut self, register: u16) -> Result<u16, I2C::Error> {
        let mut data = [0u8; 2];
        self.i2c
            .write_read(self.address, &register.to_be_bytes(), &mut data)
            .await?;
        Ok(u16::from_be_bytes(data))
    }

    /// Write one byte at a 16-bit register address.
    async fn write_u8(&mut self, register: u16, value: u8) -> Result<(), I2C::Error> {
        let [high, low] = register.to_be_bytes();
        self.i2c.write(self.address, &[high, low, value]).await
    }

    /// Write two big-endian bytes at a 16-bit register address.
    async fn write_u16(&mut self, register: u16, value: u16) -> Result<(), I2C::Error> {
        let [high, low] = register.to_be_bytes();
        let [value_high, value_low] = value.to_be_bytes();
        self.i2c.write(self.address, &[high, low, value_high, value_low]).await
    }

    /// Write four big-endian bytes at a 16-bit register address.
    async fn write_u32(&mut self, register: u16, value: u32) -> Result<(), I2C::Error> {
        let [high, low] = register.to_be_bytes();
        let [byte_3, byte_2, byte_1, byte_0] = value.to_be_bytes();
        self.i2c
            .write(self.address, &[high, low, byte_3, byte_2, byte_1, byte_0])
            .await
    }
}
