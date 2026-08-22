//! Top-level driver: owns the async UART and power pin and produces scans.
//!
//! [`CoinD6`] is a passive, executor-agnostic shell that wires the pure
//! [`Decoder`](crate::Decoder) and [`aggregate`](crate::aggregate) stages to an
//! `embedded_io_async` UART and a synchronous power `OutputPin`. The caller owns
//! the task and awaits each future; the driver never spawns work of its own.
//!
//! # Buffer ownership
//!
//! The caller supplies an ingest chunk (at least 1 KB, large enough for the
//! largest possible packet) that the driver reads the UART into. The decoder's
//! own carry buffer holds a packet split across a chunk boundary, so no
//! double-buffering is required.
//!
//! # Timeouts
//!
//! The driver is executor-agnostic and has no wall clock, so [`CoinD6::read_scan`]
//! and [`CoinD6::read_aggregated`] use a byte-count watchdog rather than a timer:
//! if no revolution is assembled within [`RING_START_WATCHDOG_BYTES`] bytes read
//! since the start of the current revolution, they return [`Error::Timeout`].
//! Callers that need a hard wall-clock bound should additionally wrap the call in
//! their own timeout.

use embedded_hal::digital::OutputPin;
use embedded_io_async::{Read, ReadExactError, Write};

use crate::{
    decoder::{Decode, Decoder},
    post_processing::{aggregate, angle_correction_deg},
    types::{AggregationConfig, Config, Error, Scan},
};

/// The vendor start command, little-endian as transmitted.
const START_CMD: [u8; 4] = [0xAA, 0x55, 0xF0, 0x0F];
/// The vendor stop command, little-endian as transmitted.
const STOP_CMD: [u8; 4] = [0xAA, 0x55, 0xF5, 0x0A];
/// First byte of the two-byte device-info frame header.
const DEVICE_INFO_HEADER_0: u8 = 0xA5;
/// Second byte of the two-byte device-info frame header.
const DEVICE_INFO_HEADER_1: u8 = 0x5A;
/// The device-info frame type byte.
const DEVICE_INFO_TYPE: u8 = 0x01;
/// Number of times to retry reading the device-info frame, tolerating the
/// transient break/framing noise the device produces while powering up.
const DEVICE_INFO_RETRIES: u32 = 16;
/// Byte watchdog: give up waiting for a ring-start after this many bytes.
const RING_START_WATCHDOG_BYTES: usize = 128 * 1024;

/// An asynchronous COIN-D6 driver owning the UART and power-MOSFET GPIO.
pub struct CoinD6<'a, UART, POWER> {
    /// The UART used for both the point stream and the start/stop commands.
    uart: UART,
    /// The active-high power-enable pin (high = on).
    power: POWER,
    /// Caller-allocated chunk the driver reads the UART into.
    ingest: &'a mut [u8],
    /// Pure byte-stream decoder.
    decoder: Decoder,
    /// Driver configuration (angle-correction toggle).
    config: Config,
}

impl<'a, UART, POWER> CoinD6<'a, UART, POWER>
where
    UART: Read + Write,
    POWER: OutputPin,
{
    /// Create a driver owning `uart` and `power`, reading into the caller's
    /// `ingest` chunk.
    ///
    /// `ingest` must be at least 1 KB so it can hold the largest possible
    /// packet plus a packet split across a chunk boundary.
    #[must_use]
    pub fn new(uart: UART, power: POWER, ingest: &'a mut [u8], config: Config) -> Self {
        Self {
            uart,
            power,
            ingest,
            decoder: Decoder::new(),
            config,
        }
    }

    /// Assert the power pin and wait for the device-info frame.
    ///
    /// On power-on the device announces its identity with an `A5 5A` frame (type
    /// `0x01`). This validates that frame — header, type, and checksum — and
    /// drains its data area, so a subsequent [`Self::start`] is not preceded by
    /// stale bytes.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Pin`] if asserting the pin fails, [`Error::Uart`] on a
    /// fatal UART error, [`Error::Timeout`] if the frame never arrives, or
    /// [`Error::Resync`] if the frame is malformed.
    pub async fn power_on(&mut self) -> Result<(), Error<UART::Error, POWER::Error>> {
        self.power.set_high().map_err(Error::Pin)?;
        self.read_device_info().await
    }

    /// Read and validate the device-info frame, draining its data area.
    ///
    /// The frame layout is `A5 5A <len LE> <checksum LE> <type> <data...>`,
    /// where the checksum is the sum of every byte except the checksum field.
    /// The power-on transition briefly glitches the RX line (a UART break), so
    /// an interrupted or malformed frame is retried, bounded by a retry budget.
    async fn read_device_info(&mut self) -> Result<(), Error<UART::Error, POWER::Error>> {
        let mut retries = DEVICE_INFO_RETRIES;
        loop {
            match self.try_read_device_info().await {
                Ok(()) => return Ok(()),
                // A break/framing glitch (`Uart`) or a malformed frame
                // (`Resync`) is expected during power-on; restart the scan.
                Err(Error::Uart(_) | Error::Resync) if retries > 0 => retries -= 1,
                Err(e) => return Err(e),
            }
        }
    }

    /// One attempt at reading and validating the device-info frame.
    async fn try_read_device_info(&mut self) -> Result<(), Error<UART::Error, POWER::Error>> {
        // The motor emits a few speed-adjust bytes (0xFE/0xFF/0xFA) while it
        // spins up, so scan past them to the `A5 5A` header.
        let mut seen_header_0 = false;
        loop {
            let mut byte = [0u8; 1];
            self.read_exact(&mut byte).await?;
            match byte[0] {
                DEVICE_INFO_HEADER_0 => seen_header_0 = true,
                DEVICE_INFO_HEADER_1 if seen_header_0 => break,
                _ => seen_header_0 = false,
            }
        }

        // length(2 LE) + checksum(2 LE) + type(1).
        let mut fixed = [0u8; 5];
        self.read_exact(&mut fixed).await?;
        let len = usize::from(u16::from_le_bytes([fixed[0], fixed[1]]));
        let expected_sum = u16::from_le_bytes([fixed[2], fixed[3]]);
        let kind = fixed[4];

        if kind != DEVICE_INFO_TYPE {
            return Err(Error::Resync);
        }

        // The checksum is the sum of every byte except the checksum field.
        // Drain the data area while accumulating it.
        let mut sum = u16::from(DEVICE_INFO_HEADER_0)
            + u16::from(DEVICE_INFO_HEADER_1)
            + u16::from(fixed[0])
            + u16::from(fixed[1])
            + u16::from(fixed[4]);
        let mut scratch = [0u8; 32];
        let mut remaining = len;
        while remaining > 0 {
            let chunk = remaining.min(scratch.len());
            self.read_exact(&mut scratch[..chunk]).await?;
            for &byte in &scratch[..chunk] {
                sum += u16::from(byte);
            }
            remaining -= chunk;
        }

        if sum != expected_sum {
            return Err(Error::Resync);
        }
        Ok(())
    }

    /// Read exactly `buf.len()` bytes, mapping EOF to [`Error::Timeout`].
    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), Error<UART::Error, POWER::Error>> {
        self.uart.read_exact(buf).await.map_err(|e| match e {
            ReadExactError::UnexpectedEof => Error::Timeout,
            ReadExactError::Other(e) => Error::Uart(e),
        })
    }

    /// Send the vendor start command (best-effort ack).
    ///
    /// The vendor spec's ack bytes are internally inconsistent, so the ack is
    /// not verified; this only writes the command and awaits the write.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Uart`] if writing the command fails.
    pub async fn start(&mut self) -> Result<(), Error<UART::Error, POWER::Error>> {
        self.uart.write_all(&START_CMD).await.map_err(Error::Uart)
    }

    /// Send the vendor stop command (best-effort ack).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Uart`] if writing the command fails.
    pub async fn stop(&mut self) -> Result<(), Error<UART::Error, POWER::Error>> {
        self.uart.write_all(&STOP_CMD).await.map_err(Error::Uart)
    }

    /// Deassert the power pin.
    ///
    /// Kept `async` for API symmetry with the rest of the lifecycle; it performs
    /// no I/O itself.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Pin`] if deasserting the pin fails.
    #[allow(clippy::unused_async)]
    pub async fn power_off(&mut self) -> Result<(), Error<UART::Error, POWER::Error>> {
        self.power.set_low().map_err(Error::Pin)
    }

    /// Continuously drain the UART and return one complete revolution in `scan`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Resync`] on a transient checksum mismatch, [`Error::Uart`]
    /// on a fatal UART error, or [`Error::Timeout`] if no revolution arrives
    /// within [`RING_START_WATCHDOG_BYTES`] bytes.
    pub async fn read_scan(&mut self, scan: &mut Scan) -> Result<(), Error<UART::Error, POWER::Error>> {
        self.read_revolution(scan).await
    }

    /// Capture one revolution per entry in `spins`, then reduce them into `out`.
    ///
    /// `spins.len()` is the spin count; the caller allocates
    /// `spins.len() == config.spins`.
    ///
    /// # Errors
    ///
    /// Propagates the first error from [`Self::read_scan`] across any of the
    /// spins.
    pub async fn read_aggregated(
        &mut self,
        spins: &mut [Scan],
        out: &mut Scan,
        config: &AggregationConfig,
    ) -> Result<(), Error<UART::Error, POWER::Error>> {
        for scan in spins.iter_mut() {
            self.read_revolution(scan).await?;
        }
        *out = aggregate(spins, config);
        Ok(())
    }

    /// Return the UART and power pin (borrowed buffers are returned by drop).
    #[must_use]
    pub fn release(self) -> (UART, POWER) {
        (self.uart, self.power)
    }

    /// Drain the UART until a single complete revolution is assembled in `scan`.
    async fn read_revolution(&mut self, scan: &mut Scan) -> Result<(), Error<UART::Error, POWER::Error>> {
        let mut consumed = 0usize;
        loop {
            let n = self.uart.read(&mut *self.ingest).await.map_err(Error::Uart)?;
            match self.decoder.push(&self.ingest[..n], scan) {
                Decode::Revolution => {
                    self.correct(scan);
                    return Ok(());
                }
                Decode::Resync => return Err(Error::Resync),
                Decode::InProgress => {}
            }
            consumed += n;
            if consumed > RING_START_WATCHDOG_BYTES {
                return Err(Error::Timeout);
            }
        }
    }

    /// Apply distance-dependent angle correction in place, if enabled.
    fn correct(&self, scan: &mut Scan) {
        if self.config.angle_correction {
            for point in &mut scan.points[..scan.len] {
                if point.distance_mm > 0 {
                    point.angle_deg += angle_correction_deg(point.distance_mm);
                }
            }
        }
    }
}
