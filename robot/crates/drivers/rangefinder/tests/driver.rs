//! Host-side tests for the async wrapper.
//!
//! A scripted async I2C mock records every transaction. The mock answers the
//! reads the ST initialisation sequence performs and replays the range-status
//! and range registers the read path consumes. Pin and interrupt mocks record
//! the levels and waits the wrapper requests.

// The embedded-hal traits require `async fn`, but every mock resolves on its
// first poll and therefore has nothing to await.
#![allow(clippy::unused_async_trait_impl)]

use core::convert::Infallible;
use std::{
    cell::RefCell,
    collections::HashMap,
    future::Future,
    pin::pin,
    rc::Rc,
    task::{Context, Poll, Waker},
};

use embedded_hal_async::{
    delay::DelayNs,
    i2c::{ErrorType, I2c, Operation},
};
use rangefinder::{RangeStatus, Rangefinder, RangefinderReading, Reading};

/// The shared transaction log.
type Log = Rc<RefCell<Vec<Transaction>>>;

/// One recorded I2C transaction.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Transaction {
    /// A plain write of `bytes`.
    Write { address: u8, bytes: Vec<u8> },
    /// A plain read of `len` bytes.
    Read { address: u8, len: usize },
    /// A register write followed by a read in one transaction.
    WriteRead {
        address: u8,
        write: Vec<u8>,
        read_len: usize,
    },
    /// A generic transaction, recorded so an unexpected helper is visible.
    Other { address: u8 },
}

/// A scripted async I2C device that records transactions.
struct MockI2c {
    /// Values returned for register reads; missing registers read as zero.
    registers: HashMap<u8, u8>,
    /// Shared transaction log.
    log: Log,
}

impl ErrorType for MockI2c {
    type Error = Infallible;
}

impl MockI2c {
    /// Create a mock that answers from `registers` and logs to `log`.
    fn new(registers: HashMap<u8, u8>, log: &Log) -> Self {
        Self {
            registers,
            log: Rc::clone(log),
        }
    }

    /// Return the value for `register`, defaulting to zero.
    fn value(&self, register: u8) -> u8 {
        self.registers.get(&register).copied().unwrap_or(0)
    }
}

impl I2c for MockI2c {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.log.borrow_mut().push(Transaction::Read {
            address,
            len: read.len(),
        });
        for byte in read.iter_mut() {
            *byte = 0;
        }
        Ok(())
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.log.borrow_mut().push(Transaction::Write {
            address,
            bytes: write.to_vec(),
        });
        Ok(())
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.log.borrow_mut().push(Transaction::WriteRead {
            address,
            write: write.to_vec(),
            read_len: read.len(),
        });
        let register = write.first().copied().unwrap_or(0);
        for (offset, byte) in read.iter_mut().enumerate() {
            let offset = u8::try_from(offset).unwrap_or(u8::MAX);
            *byte = self.value(register.wrapping_add(offset));
        }
        Ok(())
    }

    async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        self.log.borrow_mut().push(Transaction::Other { address });
        for operation in operations {
            if let Operation::Read(buffer) = operation {
                for byte in buffer.iter_mut() {
                    *byte = 0;
                }
            }
        }
        Ok(())
    }
}

/// The levels recorded on the `XSHUT` line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Level {
    /// The line was driven low.
    Low,
    /// The line was released high.
    High,
}

/// An open-drain output mock that records every level it was asked to drive.
#[derive(Clone, Default)]
struct MockXshut {
    /// Shared level log.
    log: Rc<RefCell<Vec<Level>>>,
}

impl embedded_hal::digital::ErrorType for MockXshut {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for MockXshut {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.log.borrow_mut().push(Level::High);
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.log.borrow_mut().push(Level::Low);
        Ok(())
    }
}

/// The interrupt waits recorded by [`MockInt`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WaitCall {
    /// `wait_for_low`.
    Low,
    /// `wait_for_high`.
    High,
}

/// An interrupt input mock that records the level waits it was asked for.
#[derive(Clone, Default)]
struct MockInt {
    /// Shared wait log.
    calls: Rc<RefCell<Vec<WaitCall>>>,
}

impl embedded_hal::digital::ErrorType for MockInt {
    type Error = Infallible;
}

impl embedded_hal_async::digital::Wait for MockInt {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.calls.borrow_mut().push(WaitCall::High);
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.calls.borrow_mut().push(WaitCall::Low);
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

/// A delay mock that records millisecond and nanosecond requests.
#[derive(Default)]
struct MockDelay {
    /// Millisecond delays, in call order.
    ms: Rc<RefCell<Vec<u32>>>,
    /// Nanosecond delays, in call order.
    ns: Rc<RefCell<Vec<u32>>>,
}

impl DelayNs for MockDelay {
    async fn delay_ns(&mut self, ns: u32) {
        self.ns.borrow_mut().push(ns);
    }

    async fn delay_ms(&mut self, ms: u32) {
        self.ms.borrow_mut().push(ms);
    }
}

/// Registers that let the ST initialisation sequence complete on the mock.
fn readable_registers() -> HashMap<u8, u8> {
    let mut registers = HashMap::new();
    registers.insert(0xC0, 0xEE);
    registers.insert(0x83, 0x01);
    registers.insert(0x92, 0x01);
    registers.insert(0x13, 0x01);
    registers
}

/// Drive `fut` to completion on the calling thread.
///
/// Every mock resolves on its first poll, so a no-op waker is sufficient.
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
fn read_issues_status_range_and_clear_in_order() {
    let mut registers = readable_registers();
    registers.insert(0x14, 0x58);
    registers.insert(0x1E, 0x04);
    registers.insert(0x1F, 0xD2);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let sample = block_on(rangefinder.read()).ok();

    assert_eq!(
        sample,
        Some(RangefinderReading {
            reading: Reading::Valid(1234),
            raw_status: 11,
        })
    );
    assert_eq!(
        log.borrow().as_slice(),
        &[
            Transaction::WriteRead {
                address: 0x29,
                write: vec![0x14],
                read_len: 1,
            },
            Transaction::WriteRead {
                address: 0x29,
                write: vec![0x1E],
                read_len: 2,
            },
            Transaction::Write {
                address: 0x29,
                bytes: vec![0x0B, 0x01],
            },
        ]
    );
}

#[test]
fn read_decodes_the_out_of_range_sentinel_as_no_target() {
    let mut registers = readable_registers();
    registers.insert(0x14, 0x58);
    registers.insert(0x1E, 0x1F);
    registers.insert(0x1F, 0xFE);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let sample = block_on(rangefinder.read()).ok();

    assert_eq!(
        sample,
        Some(RangefinderReading {
            reading: Reading::NoTarget,
            raw_status: 11,
        })
    );
}

#[test]
fn read_preserves_a_failure_status() {
    let mut registers = readable_registers();
    registers.insert(0x14, 1 << 3);
    registers.insert(0x1E, 0x01);
    registers.insert(0x1F, 0xF4);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let sample = block_on(rangefinder.read()).ok();

    assert_eq!(
        sample,
        Some(RangefinderReading {
            reading: Reading::Invalid(RangeStatus::HardwareFail),
            raw_status: 1,
        })
    );
}

#[test]
fn init_pulses_xshut_and_starts_timed_ranging() {
    let log = Log::default();
    let i2c = MockI2c::new(readable_registers(), &log);
    let xshut = MockXshut::default();
    let levels = Rc::clone(&xshut.log);
    let mut delay = MockDelay::default();
    let delays = Rc::clone(&delay.ms);
    let mut rangefinder = Rangefinder::new(i2c, xshut, MockInt::default());

    let result = block_on(rangefinder.init(&mut delay));

    assert!(result.is_ok(), "{result:?}");
    assert_eq!(levels.borrow().as_slice(), &[Level::Low, Level::High]);
    assert_eq!(delays.borrow().as_slice(), &[10, 10]);
    assert_eq!(
        log.borrow().last(),
        Some(&Transaction::Write {
            address: 0x29,
            bytes: vec![0x00, 0x04],
        })
    );
}

#[test]
fn wait_ready_waits_for_the_interrupt_low() {
    let log = Log::default();
    let i2c = MockI2c::new(readable_registers(), &log);
    let interrupt = MockInt::default();
    let calls = Rc::clone(&interrupt.calls);
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), interrupt);

    let result = block_on(rangefinder.wait_ready());

    assert!(result.is_ok(), "{result:?}");
    assert_eq!(calls.borrow().as_slice(), &[WaitCall::Low]);
}

#[test]
fn standby_stops_ranging_and_holds_xshut_low() {
    let log = Log::default();
    let i2c = MockI2c::new(readable_registers(), &log);
    let xshut = MockXshut::default();
    let levels = Rc::clone(&xshut.log);
    let mut rangefinder = Rangefinder::new(i2c, xshut, MockInt::default());

    let result = block_on(rangefinder.standby());

    assert!(result.is_ok(), "{result:?}");
    assert_eq!(levels.borrow().as_slice(), &[Level::Low]);
    assert_eq!(
        log.borrow().first(),
        Some(&Transaction::Write {
            address: 0x29,
            bytes: vec![0x00, 0x01],
        })
    );
}

#[test]
fn reset_pulses_xshut_and_reinitialises() {
    let log = Log::default();
    let i2c = MockI2c::new(readable_registers(), &log);
    let xshut = MockXshut::default();
    let levels = Rc::clone(&xshut.log);
    let mut delay = MockDelay::default();
    let delays = Rc::clone(&delay.ms);
    let mut rangefinder = Rangefinder::new(i2c, xshut, MockInt::default());

    let first = block_on(rangefinder.init(&mut delay));
    let second = block_on(rangefinder.reset(&mut delay));

    assert!(first.is_ok(), "{first:?}");
    assert!(second.is_ok(), "{second:?}");
    assert_eq!(
        levels.borrow().as_slice(),
        &[Level::Low, Level::High, Level::Low, Level::High]
    );
    assert_eq!(delays.borrow().as_slice(), &[10, 10, 10, 10]);
    let identity_reads = log
        .borrow()
        .iter()
        .filter(|transaction| match transaction {
            Transaction::WriteRead { write, .. } => write.first() == Some(&0xC0),
            _ => false,
        })
        .count();
    assert_eq!(identity_reads, 2);
}
