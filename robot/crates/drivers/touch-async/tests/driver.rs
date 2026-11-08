//! Host-side integration tests for [`TouchPanel`]'s single-transaction read.
//!
//! A scripted [`SpiDevice`] records the bytes written and the number of
//! chip-select assertions, and replays a canned response for each channel.

use core::convert::Infallible;
use std::{
    collections::HashMap,
    future::Future,
    pin::pin,
    task::{Context, Poll, Waker},
};

use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};
use touch_async::{TouchPanel, TouchSample};

/// A scripted `SpiDevice` that records traffic and replays canned responses.
struct FakeSpi {
    /// Response value keyed by the command byte that precedes a read.
    responses: HashMap<u8, u16>,
    /// Per-transaction Z1 responses, consumed front-to-back before `responses`
    /// is consulted; lets a test script "no touch, then touch".
    z1_sequence: Vec<u16>,
    /// Number of `transaction` calls, i.e. chip-select assertions.
    transactions: usize,
    /// Every byte written on MOSI, in order.
    writes: Vec<u8>,
}

impl ErrorType for FakeSpi {
    type Error = Infallible;
}

impl SpiDevice<u8> for FakeSpi {
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        self.transactions += 1;
        let mut command = 0u8;
        for operation in operations.iter_mut() {
            match operation {
                Operation::Write(bytes) => {
                    self.writes.extend_from_slice(bytes);
                    if let Some(&byte) = bytes.first() {
                        command = byte;
                    }
                }
                Operation::Read(buf) => {
                    let value = match command {
                        0xB0 if !self.z1_sequence.is_empty() => self.z1_sequence.remove(0),
                        _ => self.responses.get(&command).copied().unwrap_or(0),
                    };
                    buf.copy_from_slice(&(value << 3).to_be_bytes());
                }
                _ => {}
            }
        }
        Ok(())
    }
}

/// An interrupt input that is always ready, standing in for a pin already at
/// the requested level.
struct AlwaysReady;

impl embedded_hal::digital::ErrorType for AlwaysReady {
    type Error = Infallible;
}

impl embedded_hal_async::digital::Wait for AlwaysReady {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// One recorded `Wait` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WaitCall {
    /// `wait_for_high`.
    High,
    /// `wait_for_low`.
    Low,
}

/// An interrupt input that records the level waits it was asked for, standing in
/// for a line whose state the test controls.
#[derive(Default)]
struct RecordingWait {
    /// The waits requested, in call order.
    calls: Vec<WaitCall>,
}

impl embedded_hal::digital::ErrorType for RecordingWait {
    type Error = Infallible;
}

impl embedded_hal_async::digital::Wait for RecordingWait {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.calls.push(WaitCall::High);
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.calls.push(WaitCall::Low);
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Drive `fut` to completion on the calling thread.
///
/// Only valid for futures that resolve without external wake-ups — the fake SPI
/// device is ready on its first poll, so a no-op waker is sufficient.
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
fn read_decodes_all_channels_in_one_transaction() {
    let mut responses = HashMap::new();
    responses.insert(0x90, 1234u16);
    responses.insert(0xD0, 2345u16);
    responses.insert(0xB0, 100u16);
    responses.insert(0xC0, 300u16);
    let spi = FakeSpi {
        responses,
        z1_sequence: Vec::new(),
        transactions: 0,
        writes: Vec::new(),
    };
    let mut panel = TouchPanel::new(spi);

    let result = block_on(panel.read());

    let spi = panel.release();
    assert_eq!(spi.transactions, 1);
    assert_eq!(spi.writes, vec![0x90, 0xD0, 0xB0, 0xC0]);
    assert_eq!(
        result,
        Ok(Some(TouchSample {
            x: 1234,
            y: 2345,
            z1: 100,
            z2: 300,
        }))
    );
}

#[test]
fn read_reports_none_when_z1_is_zero() {
    let spi = FakeSpi {
        responses: HashMap::new(),
        z1_sequence: Vec::new(),
        transactions: 0,
        writes: Vec::new(),
    };
    let mut panel = TouchPanel::new(spi);

    let result = block_on(panel.read());

    let spi = panel.release();
    assert_eq!(spi.transactions, 1);
    assert!(matches!(result, Ok(None)));
}

#[test]
fn wait_for_touch_arms_penirq_before_waiting() {
    let mut responses = HashMap::new();
    responses.insert(0x90, 1234u16);
    responses.insert(0xD0, 2345u16);
    responses.insert(0xB0, 100u16);
    responses.insert(0xC0, 300u16);
    let spi = FakeSpi {
        responses,
        z1_sequence: Vec::new(),
        transactions: 0,
        writes: Vec::new(),
    };
    let mut panel = TouchPanel::new(spi);
    let mut irq = AlwaysReady;

    let result = block_on(panel.wait_for_touch(&mut irq));

    let spi = panel.release();
    // One discarded read to arm PENIRQ, then one read for the returned sample.
    assert_eq!(spi.transactions, 2);
    assert_eq!(
        result,
        Ok(TouchSample {
            x: 1234,
            y: 2345,
            z1: 100,
            z2: 300,
        })
    );
}

#[test]
fn wait_for_touch_gates_on_release_after_an_invalid_sample() {
    let mut responses = HashMap::new();
    responses.insert(0x90, 1234u16);
    responses.insert(0xD0, 2345u16);
    responses.insert(0xC0, 300u16);
    // The first two Z1 reads report no touch — the arming read, then a low level
    // that yields no valid sample — and the third reports the real touch.
    let spi = FakeSpi {
        responses,
        z1_sequence: vec![0, 0, 100],
        transactions: 0,
        writes: Vec::new(),
    };
    let mut panel = TouchPanel::new(spi);
    let mut irq = RecordingWait::default();

    let result = block_on(panel.wait_for_touch(&mut irq));

    let spi = panel.release();
    assert_eq!(spi.transactions, 3);
    assert_eq!(
        result,
        Ok(TouchSample {
            x: 1234,
            y: 2345,
            z1: 100,
            z2: 300,
        })
    );
    // After the invalid sample the driver must wait for release (high) before
    // awaiting the next low edge, rather than re-reading a stuck-low line.
    assert_eq!(irq.calls, vec![WaitCall::Low, WaitCall::High, WaitCall::Low]);
}
