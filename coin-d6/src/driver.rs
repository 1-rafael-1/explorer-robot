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
//! The driver is executor-agnostic and has no wall clock, so it uses byte-count
//! watchdogs rather than timers. [`CoinD6::read_scan`] and
//! [`CoinD6::read_aggregated`] return [`Error::Timeout`] if no revolution is
//! assembled within [`RING_START_WATCHDOG_BYTES`] bytes read since the start of
//! the current revolution; [`CoinD6::power_on`] returns [`Error::Timeout`] if the
//! device-info frame does not arrive within [`DEVICE_INFO_WATCHDOG_BYTES`] bytes.
//! Callers that need a hard wall-clock bound should additionally wrap the call in
//! their own timeout.

use embedded_hal::digital::OutputPin;
use embedded_io_async::{Read, ReadExactError, Write};

use crate::{
    decoder::{Decode, Decoder},
    post_processing::{aggregate, angle_correction_deg, normalise_angle},
    types::{AggregationConfig, Config, Error, Scan, WarmupConfig, WarmupOutcome},
    warmup::Warmup,
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
/// Byte watchdog: give up waiting for the device-info frame after this many
/// bytes (header scan + fixed fields + declared data).
const DEVICE_INFO_WATCHDOG_BYTES: usize = 1024;

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
        // `read` returns `Ok(0)` at end-of-stream only when `ingest` is
        // non-empty; an empty buffer would return `Ok(0)` even while data
        // flows, which the ingest loop would mistake for EOF. Catch the
        // violation at construction.
        debug_assert!(!ingest.is_empty(), "ingest buffer must be non-empty");
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
    /// A single attempt also gives up with [`Error::Timeout`] after
    /// [`DEVICE_INFO_WATCHDOG_BYTES`] bytes without the frame.
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
        let mut consumed = 0usize;

        // The motor emits a few speed-adjust bytes (0xFE/0xFF/0xFA) while it
        // spins up, so scan past them to the `A5 5A` header.
        let mut seen_header_0 = false;
        loop {
            let mut byte = [0u8; 1];
            self.read_exact(&mut byte).await?;
            consumed += 1;
            if consumed > DEVICE_INFO_WATCHDOG_BYTES {
                return Err(Error::Timeout);
            }
            match byte[0] {
                DEVICE_INFO_HEADER_0 => seen_header_0 = true,
                DEVICE_INFO_HEADER_1 if seen_header_0 => break,
                _ => seen_header_0 = false,
            }
        }

        // length(2 LE) + checksum(2 LE) + type(1).
        let mut fixed = [0u8; 5];
        self.read_exact(&mut fixed).await?;
        consumed += fixed.len();
        if consumed > DEVICE_INFO_WATCHDOG_BYTES {
            return Err(Error::Timeout);
        }
        let len = usize::from(u16::from_le_bytes([fixed[0], fixed[1]]));
        let expected_sum = u16::from_le_bytes([fixed[2], fixed[3]]);
        let kind = fixed[4];

        // The checksum is the sum of every byte except the checksum field.
        // Always drain the declared data area first (accumulating the checksum),
        // then validate the type and checksum together, so a malformed frame
        // still consumes its data bytes before being rejected.
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
                // Wrapping addition: the vendor sum is a `u16`, and the 1 KiB
                // watchdog permits frames larger than 255 bytes.
                sum = sum.wrapping_add(u16::from(byte));
            }
            remaining -= chunk;
            consumed += chunk;
            if consumed > DEVICE_INFO_WATCHDOG_BYTES {
                return Err(Error::Timeout);
            }
        }

        if kind != DEVICE_INFO_TYPE || sum != expected_sum {
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
    /// not verified; this writes the command, flushes the UART, and awaits the
    /// write. The device's response and spin-up bytes are left for the decoder
    /// to skip.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Uart`] if writing or flushing the command fails.
    pub async fn start(&mut self) -> Result<(), Error<UART::Error, POWER::Error>> {
        self.uart.write_all(&START_CMD).await.map_err(Error::Uart)?;
        // Flush so a buffered writer (e.g. `BufferedUart`) has actually
        // transmitted the command before `start` reports success.
        self.uart.flush().await.map_err(Error::Uart)
    }

    /// Send the vendor stop command (best-effort ack).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Uart`] if writing or flushing the command fails.
    pub async fn stop(&mut self) -> Result<(), Error<UART::Error, POWER::Error>> {
        self.uart.write_all(&STOP_CMD).await.map_err(Error::Uart)?;
        // Flush so the stop command is actually transmitted before the caller
        // (which typically powers the device off immediately) returns.
        self.uart.flush().await.map_err(Error::Uart)
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
    /// A transient checksum mismatch is recovered transparently: the decoder
    /// discards the in-progress revolution and re-synchronises on the next
    /// ring-start, and this keeps reading until a full revolution is assembled.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Uart`] on a fatal UART error, or [`Error::Timeout`] if no
    /// revolution is assembled within [`RING_START_WATCHDOG_BYTES`] bytes.
    pub async fn read_scan(&mut self, scan: &mut Scan) -> Result<(), Error<UART::Error, POWER::Error>> {
        self.read_revolution(scan).await
    }

    /// Capture one revolution per entry in `spins`, then reduce them into `out`.
    ///
    /// The number of revolutions to aggregate is `spins.len()`.
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

    /// Discard revolutions until the rotor reaches steady state.
    ///
    /// The point count proxies rotor speed: it starts below
    /// [`crate::NATIVE_POINTS`] and climbs as the rotor accelerates, and both
    /// the ring-start bearing and the point count drift until it settles.
    /// `scratch` is reused for each discarded revolution. The settle band,
    /// patience, and spin budget come from `config`.
    ///
    /// Returns a [`WarmupOutcome`] describing how the phase ended; the caller
    /// decides how to report it (the driver is log-agnostic).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Timeout`] if the stream ends or the byte watchdog trips
    /// while waiting for a revolution.
    pub async fn warm_up(
        &mut self,
        scratch: &mut Scan,
        config: &WarmupConfig,
    ) -> Result<WarmupOutcome, Error<UART::Error, POWER::Error>> {
        let mut warmup = Warmup::new(config);
        loop {
            self.read_revolution(scratch).await?;
            if let Some(outcome) = warmup.observe(scratch.len) {
                return Ok(outcome);
            }
        }
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
            // A UART glitch (overrun, break, framing, parity) means bytes were
            // lost and the stream position is untrustworthy. Resync the decoder
            // and keep reading rather than failing the capture. The missed chunk
            // counts toward the watchdog so a dead UART cannot spin forever.
            let Ok(n) = self.uart.read(&mut *self.ingest).await else {
                self.decoder.resync();
                consumed += self.ingest.len();
                if consumed > RING_START_WATCHDOG_BYTES {
                    return Err(Error::Timeout);
                }
                continue;
            };
            // `read` returns `Ok(0)` only at end-of-stream (an empty `ingest`
            // slice is a contract violation caught by the `debug_assert` in
            // `new`); treat it as a timeout rather than spinning forever.
            if n == 0 {
                return Err(Error::Timeout);
            }
            match self.decoder.push(&self.ingest[..n], scan) {
                Decode::Revolution => {
                    self.correct(scan);
                    return Ok(());
                }
                // A checksum mismatch has already reset the decoder (discarding
                // the in-progress revolution and waiting for the next
                // ring-start), so keep reading rather than failing the capture
                // on a single corrupt packet during spin-up.
                Decode::Resync | Decode::InProgress => {}
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
                if let Some(distance) = point.distance_mm {
                    point.angle_deg = normalise_angle(point.angle_deg + angle_correction_deg(distance));
                }
            }
        }
    }
}
