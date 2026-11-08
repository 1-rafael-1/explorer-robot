//! The serial seam: raw UART bytes in both directions (ticket 02).
//!
//! The stub's personas, the integration tests and the binary all speak through [`Transport`],
//! the same bytes the Bridge's UART will see. The in-memory [`Duplex`] channel serves the tests;
//! the binary wraps a real serial port behind the same trait, so the bench and the tests exercise
//! one path.

use std::{
    collections::VecDeque,
    sync::{Arc, Mutex, MutexGuard},
};

/// Raw UART bytes crossing the serial seam.
///
/// Implementations move whole chunks in both directions and never fail: a read returning zero
/// means "nothing pending", matching a drained serial ring. The stub polls [`Transport::read`] and
/// flushes [`Transport::write`] on every step, so an implementation may buffer internally.
pub trait Transport {
    /// Moves pending inbound bytes into `out`, returning how many were read.
    fn read(&mut self, out: &mut [u8]) -> usize;

    /// Writes `bytes` onto the wire.
    fn write(&mut self, bytes: &[u8]);
}

/// One end of an in-memory duplex byte channel.
///
/// [`duplex`] returns the two ends of a byte pipe: bytes written at one end are read at the
/// other, in order, exactly as a serial cable would deliver them. The integration tests give one
/// end to the stub and keep the other as the bench peer.
#[derive(Debug)]
pub struct Duplex {
    /// The bytes waiting for this end to read.
    inbox: Arc<Mutex<VecDeque<u8>>>,
    /// The bytes waiting for the other end to read.
    outbox: Arc<Mutex<VecDeque<u8>>>,
}

/// Creates the two ends of an in-memory duplex byte channel.
#[must_use]
pub fn duplex() -> (Duplex, Duplex) {
    let left = Arc::new(Mutex::new(VecDeque::new()));
    let right = Arc::new(Mutex::new(VecDeque::new()));
    (
        Duplex {
            inbox: Arc::clone(&left),
            outbox: Arc::clone(&right),
        },
        Duplex {
            inbox: right,
            outbox: left,
        },
    )
}

impl Duplex {
    /// Locks a queue, recovering from a poisoned lock rather than panicking.
    fn lock(queue: &Mutex<VecDeque<u8>>) -> MutexGuard<'_, VecDeque<u8>> {
        match queue.lock() {
            Ok(guard) => guard,
            Err(poisoned) => poisoned.into_inner(),
        }
    }
}

impl Transport for Duplex {
    fn read(&mut self, out: &mut [u8]) -> usize {
        let mut inbox = Self::lock(&self.inbox);
        let count = out.len().min(inbox.len());
        for slot in &mut out[..count] {
            *slot = inbox.pop_front().unwrap_or(0);
        }
        count
    }

    fn write(&mut self, bytes: &[u8]) {
        let mut outbox = Self::lock(&self.outbox);
        outbox.extend(bytes.iter().copied());
    }
}
