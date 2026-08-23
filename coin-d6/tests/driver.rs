//! Host-side integration tests for the driver's ingest loop.
//!
//! The decoder, post-processing, and warm-up stages are pure and covered
//! separately; this exercises the one driver behaviour that is host-testable
//! without hardware — how [`CoinD6::read_scan`] reacts when the UART stops
//! producing bytes.

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

/// A power pin that never fails and is never exercised by the EOF path.
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

/// Drive `fut` to completion on the calling thread.
///
/// Only valid for futures that resolve without external wake-ups — the
/// [`EofUart`] future below is ready on its first poll, so a no-op waker is
/// sufficient.
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
