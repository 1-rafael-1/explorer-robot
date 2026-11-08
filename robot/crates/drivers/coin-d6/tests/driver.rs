//! Host-side integration tests for the driver's ingest loop.
//!
//! The decoder, post-processing, and warm-up stages are pure and covered
//! separately; these exercise the driver behaviour that is host-testable without
//! hardware — how [`CoinD6::read_scan`] reacts when the UART stops producing
//! bytes, and how [`CoinD6::probe`] validates or rejects the device-info frame.

use core::convert::Infallible;
use std::{
    future::Future,
    pin::pin,
    task::{Context, Poll, Waker},
};

use coin_d6::{CoinD6, Config, Error, Scan};
use embedded_hal::digital::OutputPin;
use embedded_io_async::{ErrorType, Read, Write};

/// A UART that is already at end-of-stream: `read` returns `Ok(0)` immediately.
struct EofUart;

impl ErrorType for EofUart {
    type Error = Infallible;
}

impl Read for EofUart {
    async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
}

impl Write for EofUart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// A power pin that never fails and is never exercised by the probe paths.
struct NoopPin;

impl embedded_hal::digital::ErrorType for NoopPin {
    type Error = Infallible;
}

impl OutputPin for NoopPin {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// A UART backed by a fixed byte sequence that reports end-of-stream once the
/// sequence is exhausted.
struct BufferUart {
    /// Bytes the mock device presents to the driver, consumed left to right.
    bytes: Vec<u8>,
    /// Index of the next byte to return from `read`.
    pos: usize,
}

impl ErrorType for BufferUart {
    type Error = Infallible;
}

impl Read for BufferUart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let available = self.bytes.len() - self.pos;
        let n = available.min(buf.len());
        buf[..n].copy_from_slice(&self.bytes[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }
}

impl Write for BufferUart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Drive `fut` to completion on the calling thread.
///
/// Only valid for futures that resolve without external wake-ups — the mock
/// UARTs below resolve their reads immediately, so a no-op waker is sufficient.
fn block_on<F: Future>(fut: F) -> F::Output {
    let mut cx = Context::from_waker(Waker::noop());
    let mut fut = pin!(fut);
    loop {
        if let Poll::Ready(value) = fut.as_mut().poll(&mut cx) {
            return value;
        }
    }
}

#[test]
fn read_scan_maps_eof_to_timeout() {
    let mut ingest = [0u8; 1024];
    let mut driver = CoinD6::new(EofUart, NoopPin, &mut ingest, Config::default());
    let mut scan = Scan::new();

    // A UART at EOF must yield `Timeout`, not spin forever (the bug this guards
    // is `Ok(0)` adding zero to `consumed`, so the byte watchdog never fired).
    let result = block_on(driver.read_scan(&mut scan));

    assert!(matches!(result, Err(Error::Timeout)));
}

#[test]
fn probe_succeeds_on_valid_device_info_frame() {
    // A5 5A, length = 1 LE, checksum = 0x0143 LE, type = 0x01, data = 0x42.
    let frame = [0xA5, 0x5A, 0x01, 0x00, 0x43, 0x01, 0x01, 0x42];
    let uart = BufferUart {
        bytes: frame.to_vec(),
        pos: 0,
    };
    let mut ingest = [0u8; 1024];
    let mut driver = CoinD6::new(uart, NoopPin, &mut ingest, Config::default());

    let result = block_on(driver.probe());

    assert!(result.is_ok());
}

#[test]
fn probe_times_out_on_silence() {
    let uart = BufferUart {
        bytes: Vec::new(),
        pos: 0,
    };
    let mut ingest = [0u8; 1024];
    let mut driver = CoinD6::new(uart, NoopPin, &mut ingest, Config::default());

    let result = block_on(driver.probe());

    assert!(matches!(result, Err(Error::Timeout)));
}

#[test]
fn probe_rejects_malformed_frame() {
    // Valid header, length, and type, but the checksum (0x0000) does not match
    // the sum of the frame bytes.
    let malformed = [0xA5, 0x5A, 0x01, 0x00, 0x00, 0x00, 0x01, 0x42];
    // The driver retries a malformed frame 16 times, so supply 17 copies: the
    // 17th attempt exhausts the retry budget and returns `Resync` rather than
    // hitting EOF and reporting `Timeout`.
    let mut bytes = Vec::with_capacity(malformed.len() * 17);
    for _ in 0..17 {
        bytes.extend_from_slice(&malformed);
    }
    let uart = BufferUart { bytes, pos: 0 };
    let mut ingest = [0u8; 1024];
    let mut driver = CoinD6::new(uart, NoopPin, &mut ingest, Config::default());

    let result = block_on(driver.probe());

    assert!(matches!(result, Err(Error::Resync)));
}
