//! Host-side tests for the async VL53L1X driver.
//!
//! A scripted async I2C mock records every transaction and maintains a
//! byte-addressable 16-bit register file. Pin, interrupt and delay mocks record
//! the levels, waits and delays the driver requests.
//!
//! The expected register sequences are derived from `vl53l1x-simple` 0.2.0
//! (MIT OR Apache-2.0); see `THIRD_PARTY_LICENSES.md`.

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
use vl53l1x::{Error, RangeDiagnostics, RangeStatus, Rangefinder, RangefinderReading, Reading};

/// The shared transaction log.
type Log = Rc<RefCell<Vec<Transaction>>>;

/// One recorded I2C transaction.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Transaction {
    /// A plain write of `bytes`.
    Write {
        /// The 7-bit device address.
        address: u8,
        /// The bytes written, starting with the register address.
        bytes: Vec<u8>,
    },
    /// A plain read of `len` bytes.
    Read {
        /// The 7-bit device address.
        address: u8,
        /// The number of bytes read.
        len: usize,
    },
    /// A register-address write followed by a read in one transaction.
    WriteRead {
        /// The 7-bit device address.
        address: u8,
        /// The register-address bytes written before the read.
        write: Vec<u8>,
        /// The number of bytes read.
        read_len: usize,
    },
    /// A generic transaction, recorded so an unexpected helper is visible.
    Other {
        /// The 7-bit device address.
        address: u8,
    },
}

/// The bus error the mock returns when it is scripted to NACK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct BusNack;

impl embedded_hal::i2c::Error for BusNack {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

/// A scripted async I2C device that records transactions.
struct MockI2c {
    /// The byte-addressable register file; writes update it.
    registers: HashMap<u16, u8>,
    /// Shared transaction log.
    log: Log,
    /// Per-register reads that NACK before succeeding.
    nacks: HashMap<u16, u32>,
    /// Registers whose writes always fail.
    fail_writes: Vec<u16>,
}

impl MockI2c {
    /// Create a mock that answers from `registers` and logs to `log`.
    fn new(registers: HashMap<u16, u8>, log: &Log) -> Self {
        Self {
            registers,
            log: Rc::clone(log),
            nacks: HashMap::new(),
            fail_writes: Vec::new(),
        }
    }

    /// Return the byte at `register`, defaulting to zero.
    fn value(&self, register: u16) -> u8 {
        self.registers.get(&register).copied().unwrap_or(0)
    }
}

impl ErrorType for MockI2c {
    type Error = BusNack;
}

impl I2c for MockI2c {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.log.borrow_mut().push(Transaction::Read {
            address,
            len: read.len(),
        });
        read.fill(0);
        Ok(())
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.log.borrow_mut().push(Transaction::Write {
            address,
            bytes: write.to_vec(),
        });

        let Some(register) = register_of(write) else {
            return Ok(());
        };
        if self.fail_writes.contains(&register) {
            return Err(BusNack);
        }
        for (offset, byte) in write.iter().skip(2).enumerate() {
            let offset = u16::try_from(offset).unwrap_or(0);
            self.registers.insert(register.wrapping_add(offset), *byte);
        }
        Ok(())
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.log.borrow_mut().push(Transaction::WriteRead {
            address,
            write: write.to_vec(),
            read_len: read.len(),
        });

        let Some(register) = register_of(write) else {
            read.fill(0);
            return Ok(());
        };
        if let Some(remaining) = self.nacks.get_mut(&register)
            && *remaining > 0
        {
            *remaining -= 1;
            return Err(BusNack);
        }
        for (offset, byte) in read.iter_mut().enumerate() {
            let offset = u16::try_from(offset).unwrap_or(0);
            *byte = self.value(register.wrapping_add(offset));
        }
        Ok(())
    }

    async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        self.log.borrow_mut().push(Transaction::Other { address });
        for operation in operations.iter_mut() {
            if let Operation::Read(buffer) = operation {
                buffer.fill(0);
            }
        }
        Ok(())
    }
}

/// Decode the big-endian register address at the front of a write.
const fn register_of(write: &[u8]) -> Option<u16> {
    match write {
        [high, low, ..] => Some(u16::from_be_bytes([*high, *low])),
        _ => None,
    }
}

/// The register a transaction addressed, if it carries a register address.
fn requested_register(transaction: &Transaction) -> Option<u16> {
    match transaction {
        Transaction::Write { bytes, .. } | Transaction::WriteRead { write: bytes, .. } => register_of(bytes),
        _ => None,
    }
}

/// Every value written to `register`, in order.
fn writes_to(log: &[Transaction], register: u16) -> Vec<Vec<u8>> {
    log.iter()
        .filter_map(|transaction| {
            let Transaction::Write { bytes, .. } = transaction else {
                return None;
            };
            let (_address, value) = bytes.split_at_checked(2)?;
            if requested_register(transaction) == Some(register) {
                Some(value.to_vec())
            } else {
                None
            }
        })
        .collect()
}

/// The last value written to `register`, if any.
fn last_write(log: &[Transaction], register: u16) -> Option<Vec<u8>> {
    writes_to(log, register).into_iter().next_back()
}

/// The number of register reads issued for `register`.
fn reads_of(log: &[Transaction], register: u16) -> usize {
    log.iter()
        .filter(|transaction| {
            matches!(transaction, Transaction::WriteRead { .. }) && requested_register(transaction) == Some(register)
        })
        .count()
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

/// A register file for a healthy sensor with a booted firmware status.
fn healthy_registers() -> HashMap<u16, u8> {
    let mut registers = HashMap::new();
    // Model ID 0xEACC.
    registers.insert(0x010F, 0xEA);
    registers.insert(0x0110, 0xCC);
    // Boot complete.
    registers.insert(0x00E5, 0x01);
    // Fast oscillator frequency 0x0C00.
    registers.insert(0x0006, 0x0C);
    registers.insert(0x0007, 0x00);
    // Oscillator calibration value 0x0100.
    registers.insert(0x00DE, 0x01);
    registers.insert(0x00DF, 0x00);
    // Part-to-part outer offset 7 mm.
    registers.insert(0x0022, 0x00);
    registers.insert(0x0023, 0x07);
    // Saved VHV/phasecal state.
    registers.insert(0x000B, 0x88);
    registers.insert(0x0008, 0x07);
    registers.insert(0x00D8, 0x42);
    registers
}

/// Insert a result block whose diagnostics are fixed and whose range is
/// 1234 mm before gain correction.
fn insert_result_block(registers: &mut HashMap<u16, u8>, status: u8, stream_count: u8) {
    let block = [
        status,
        0x00,
        stream_count,
        0x03,
        0xE8,
        0x00,
        0x00,
        0x00,
        0x80,
        0x00,
        0x00,
        0x00,
        0x00,
        0x04,
        0xD2,
        0x01,
        0x00,
    ];
    for (offset, byte) in block.iter().enumerate() {
        let offset = u16::try_from(offset).unwrap_or(0);
        registers.insert(0x0089 + offset, *byte);
    }
}

/// Drive `fut` to completion on the calling thread.
///
/// Every mock resolves on its first poll, so a no-op waker is sufficient.
fn block_on<F: Future>(fut: F) -> F::Output {
    let mut context = Context::from_waker(Waker::noop());
    let mut fut = pin!(fut);
    loop {
        if let Poll::Ready(value) = fut.as_mut().poll(&mut context) {
            return value;
        }
    }
}

#[test]
fn init_pulses_xshut_and_configures_the_sensor() {
    let log = Log::default();
    let i2c = MockI2c::new(healthy_registers(), &log);
    let xshut = MockXshut::default();
    let levels = Rc::clone(&xshut.log);
    let mut delay = MockDelay::default();
    let delays = Rc::clone(&delay.ms);
    let mut rangefinder = Rangefinder::new(i2c, xshut, MockInt::default());

    let result = block_on(rangefinder.init(&mut delay));

    assert!(result.is_ok(), "{result:?}");
    assert_eq!(levels.borrow().as_slice(), &[Level::Low, Level::High]);
    assert_eq!(delays.borrow().as_slice(), &[10, 10, 1, 1]);

    let log = log.borrow();
    let log = log.as_slice();
    assert_eq!(reads_of(log, 0x010F), 1);
    assert_eq!(
        writes_to(log, 0x0000),
        vec![vec![0x00], vec![0x01]],
        "soft reset low then high"
    );

    assert_eq!(last_write(log, 0x002E), Some(vec![0x01]), "2.8 V pads");
    assert_eq!(last_write(log, 0x0030), Some(vec![0x11]), "GPIO mux");
    assert_eq!(last_write(log, 0x0046), Some(vec![0x20]), "new-reading-ready interrupt");

    assert_eq!(last_write(log, 0x0060), Some(vec![0x07]), "short VCSEL A");
    assert_eq!(last_write(log, 0x0063), Some(vec![0x05]), "short VCSEL B");
    assert_eq!(last_write(log, 0x0069), Some(vec![0x38]), "valid phase high");
    assert_eq!(last_write(log, 0x0078), Some(vec![0x07]), "short WOI SD0");
    assert_eq!(last_write(log, 0x0079), Some(vec![0x05]), "short WOI SD1");
    assert_eq!(last_write(log, 0x007A), Some(vec![0x06]), "short initial phase SD0");
    assert_eq!(last_write(log, 0x007B), Some(vec![0x06]), "short initial phase SD1");

    // 33 ms budget: 33_000 - 4528 = 28_472 us, halved to 14_236 us, encoded
    // against the 0x0C00 fast oscillator and the short VCSEL periods.
    assert_eq!(last_write(log, 0x005E), Some(vec![0x00, 0x12]), "33 ms range timeout A");
    assert_eq!(last_write(log, 0x0061), Some(vec![0x00, 0x18]), "33 ms range timeout B");

    assert_eq!(
        last_write(log, 0x001E),
        Some(vec![0x00, 0x1C]),
        "part-to-part offset 7 mm * 4"
    );
    assert_eq!(
        last_write(log, 0x006C),
        Some(vec![0x00, 0x00, 0x32, 0x00]),
        "50 ms * oscillator calibration"
    );
    assert_eq!(last_write(log, 0x0086), Some(vec![0x01]), "interrupt clear");
    assert_eq!(
        log.last(),
        Some(&Transaction::Write {
            address: 0x29,
            bytes: vec![0x00, 0x87, 0x40],
        }),
        "timed start is the last transaction"
    );
}

#[test]
fn init_rejects_a_foreign_model_id() {
    let mut registers = healthy_registers();
    registers.insert(0x010F, 0x12);
    registers.insert(0x0110, 0x34);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut delay = MockDelay::default();
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let result = block_on(rangefinder.init(&mut delay));

    assert!(matches!(result, Err(Error::InvalidDevice(0x1234))));
}

#[test]
fn init_tolerates_boot_nacks() {
    let log = Log::default();
    let mut i2c = MockI2c::new(healthy_registers(), &log);
    i2c.nacks.insert(0x00E5, 2);
    let mut delay = MockDelay::default();
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let result = block_on(rangefinder.init(&mut delay));

    assert!(result.is_ok(), "{result:?}");
    assert_eq!(reads_of(log.borrow().as_slice(), 0x00E5), 3);
}

#[test]
fn init_times_out_when_boot_never_completes() {
    let mut registers = healthy_registers();
    registers.insert(0x00E5, 0x00);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut delay = MockDelay::default();
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let result = block_on(rangefinder.init(&mut delay));

    assert!(matches!(result, Err(Error::Timeout)));
}

#[test]
fn init_rejects_a_zero_oscillator_frequency() {
    let mut registers = healthy_registers();
    registers.insert(0x0006, 0x00);
    registers.insert(0x0007, 0x00);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut delay = MockDelay::default();
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let result = block_on(rangefinder.init(&mut delay));

    assert!(
        matches!(result, Err(Error::Timeout)),
        "a zero frequency must fail closed instead of dividing by zero"
    );
}

#[test]
fn init_falls_back_to_256_when_the_oscillator_calibration_is_zero() {
    let mut registers = healthy_registers();
    registers.insert(0x00DE, 0x00);
    registers.insert(0x00DF, 0x00);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut delay = MockDelay::default();
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let result = block_on(rangefinder.init(&mut delay));

    assert!(result.is_ok(), "{result:?}");
    assert_eq!(
        last_write(log.borrow().as_slice(), 0x006C),
        Some(vec![0x00, 0x00, 0x32, 0x00]),
        "50 ms * the 256 fallback, never a zero period"
    );
}

#[test]
fn read_checks_readiness_reads_the_block_calibrates_then_clears() {
    let mut registers = healthy_registers();
    insert_result_block(&mut registers, 9, 1);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut delay = MockDelay::default();
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let initialized = block_on(rangefinder.init(&mut delay));
    assert!(initialized.is_ok(), "{initialized:?}");
    log.borrow_mut().clear();

    let sample = block_on(rangefinder.read());

    assert!(sample.is_ok(), "{sample:?}");
    assert_eq!(
        sample.ok(),
        Some(RangefinderReading {
            reading: Reading::Valid(1212),
            status: RangeStatus::Valid,
            raw_status: 9,
            diagnostics: RangeDiagnostics {
                signal_rate: 256,
                ambient_rate: 128,
                effective_spads: 1000,
            },
        })
    );
    assert_eq!(
        log.borrow().as_slice(),
        &[
            Transaction::WriteRead {
                address: 0x29,
                write: vec![0x00, 0x31],
                read_len: 1,
            },
            Transaction::WriteRead {
                address: 0x29,
                write: vec![0x00, 0x89],
                read_len: 17,
            },
            Transaction::WriteRead {
                address: 0x29,
                write: vec![0x00, 0x0B],
                read_len: 1,
            },
            Transaction::WriteRead {
                address: 0x29,
                write: vec![0x00, 0x08],
                read_len: 1,
            },
            Transaction::Write {
                address: 0x29,
                bytes: vec![0x00, 0x0B, 0x08],
            },
            Transaction::Write {
                address: 0x29,
                bytes: vec![0x00, 0x08, 0x0F],
            },
            Transaction::Write {
                address: 0x29,
                bytes: vec![0x00, 0x4D, 0x01],
            },
            Transaction::WriteRead {
                address: 0x29,
                write: vec![0x00, 0xD8],
                read_len: 1,
            },
            Transaction::Write {
                address: 0x29,
                bytes: vec![0x00, 0x47, 0x42],
            },
            Transaction::Write {
                address: 0x29,
                bytes: vec![0x00, 0x54, 0x1A, 0x0A],
            },
            Transaction::Write {
                address: 0x29,
                bytes: vec![0x00, 0x86, 0x01],
            },
        ]
    );
}

#[test]
fn subsequent_reads_skip_calibration_and_still_update_dss() {
    let mut registers = healthy_registers();
    insert_result_block(&mut registers, 9, 1);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut delay = MockDelay::default();
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let initialized = block_on(rangefinder.init(&mut delay));
    assert!(initialized.is_ok(), "{initialized:?}");
    let first = block_on(rangefinder.read());
    assert!(first.is_ok(), "{first:?}");
    log.borrow_mut().clear();

    let second = block_on(rangefinder.read());

    assert!(second.is_ok(), "{second:?}");
    let log = log.borrow();
    let log = log.as_slice();
    assert_eq!(reads_of(log, 0x000B), 0, "calibration only runs once");
    assert_eq!(reads_of(log, 0x00D8), 0, "phasecal only runs once");
    assert_eq!(
        log,
        &[
            Transaction::WriteRead {
                address: 0x29,
                write: vec![0x00, 0x31],
                read_len: 1,
            },
            Transaction::WriteRead {
                address: 0x29,
                write: vec![0x00, 0x89],
                read_len: 17,
            },
            Transaction::Write {
                address: 0x29,
                bytes: vec![0x00, 0x54, 0x1A, 0x0A],
            },
            Transaction::Write {
                address: 0x29,
                bytes: vec![0x00, 0x86, 0x01],
            },
        ]
    );
}

#[test]
fn not_ready_clears_the_interrupt_and_does_not_read_the_block() {
    let mut registers = healthy_registers();
    registers.insert(0x0031, 0x01);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let sample = block_on(rangefinder.read());

    assert!(matches!(sample, Err(Error::NotReady)));
    assert_eq!(
        log.borrow().as_slice(),
        &[
            Transaction::WriteRead {
                address: 0x29,
                write: vec![0x00, 0x31],
                read_len: 1,
            },
            Transaction::Write {
                address: 0x29,
                bytes: vec![0x00, 0x86, 0x01],
            },
        ]
    );
}

#[test]
fn read_classifies_no_wrap_check_as_no_target() {
    let mut registers = healthy_registers();
    insert_result_block(&mut registers, 9, 0);
    let log = Log::default();
    let i2c = MockI2c::new(registers, &log);
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), MockInt::default());

    let sample = block_on(rangefinder.read());

    assert!(sample.is_ok(), "{sample:?}");
    assert_eq!(sample.ok().map(|sample| sample.reading), Some(Reading::NoTarget));
    assert_eq!(sample.ok().map(|sample| sample.status), Some(RangeStatus::NoWrapCheck));
    assert_eq!(sample.ok().map(|sample| sample.raw_status), Some(9));
}

#[test]
fn wait_ready_waits_for_the_interrupt_low() {
    let log = Log::default();
    let i2c = MockI2c::new(healthy_registers(), &log);
    let interrupt = MockInt::default();
    let calls = Rc::clone(&interrupt.calls);
    let mut rangefinder = Rangefinder::new(i2c, MockXshut::default(), interrupt);

    let result = block_on(rangefinder.wait_ready());

    assert!(result.is_ok(), "{result:?}");
    assert_eq!(calls.borrow().as_slice(), &[WaitCall::Low]);
}

#[test]
fn standby_holds_xshut_low_even_when_the_stop_write_fails() {
    let log = Log::default();
    let mut i2c = MockI2c::new(healthy_registers(), &log);
    i2c.fail_writes.push(0x0087);
    let xshut = MockXshut::default();
    let levels = Rc::clone(&xshut.log);
    let mut rangefinder = Rangefinder::new(i2c, xshut, MockInt::default());

    let result = block_on(rangefinder.standby());

    assert!(matches!(result, Err(Error::Bus(BusNack))));
    assert_eq!(levels.borrow().as_slice(), &[Level::Low]);
    assert_eq!(last_write(log.borrow().as_slice(), 0x0087), Some(vec![0x00]));
}

#[test]
fn reset_pulses_xshut_and_reinitialises() {
    let log = Log::default();
    let i2c = MockI2c::new(healthy_registers(), &log);
    let xshut = MockXshut::default();
    let levels = Rc::clone(&xshut.log);
    let mut delay = MockDelay::default();
    let mut rangefinder = Rangefinder::new(i2c, xshut, MockInt::default());

    let first = block_on(rangefinder.init(&mut delay));
    let second = block_on(rangefinder.reset(&mut delay));

    assert!(first.is_ok(), "{first:?}");
    assert!(second.is_ok(), "{second:?}");
    assert_eq!(
        levels.borrow().as_slice(),
        &[Level::Low, Level::High, Level::Low, Level::High]
    );
    assert_eq!(reads_of(log.borrow().as_slice(), 0x010F), 2);
}
