//! A deterministic simulated wire for host tests (ADR-0030, ADR-0033).
//!
//! The wire models a UART between the robot and the PC as two byte queues with an explicit
//! millisecond clock. Faults are deterministic for a fixed seed: a small xorshift generator
//! supplies corrupted bytes, and periodic counters drop, corrupt or truncate traffic. There is
//! no real time, no thread and no async runtime, so a test can replay a link exactly.
//!
//! This module is compiled only with the `std` feature.

use std::{collections::VecDeque, vec::Vec};

use crate::frame::Direction;

/// The replacement seed used when the caller supplies zero, so the generator never sticks.
const FALLBACK_SEED: u32 = 0x9E37_79B9;

/// The seed offset that gives the second direction an independent generator.
const DIRECTION_SEED_OFFSET: u32 = 0x5A5A_A5A5;

/// Fault injection settings for a [`Wire`].
///
/// Zero disables a fault. Byte and chunk counters are maintained per direction, so traffic in
/// one direction never changes the fault pattern in the other.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    /// Drop every `drop_every`-th byte (1-based); zero disables dropping.
    pub drop_every: u32,
    /// Replace every `corrupt_every`-th byte with a pseudo-random byte; zero disables corruption.
    pub corrupt_every: u32,
    /// Drop the last byte of every `truncate_every`-th non-empty chunk; zero disables truncation.
    pub truncate_every: u32,
    /// Advance the wire clock by this many milliseconds after every non-empty send.
    pub stall_ms: u32,
}

/// A small deterministic xorshift32 generator.
#[derive(Clone, Copy, Debug)]
struct XorShift {
    /// The generator's current state.
    state: u32,
}

impl XorShift {
    /// Creates a generator, replacing a zero seed so the sequence is never stuck.
    #[must_use]
    const fn new(seed: u32) -> Self {
        Self {
            state: if seed == 0 { FALLBACK_SEED } else { seed },
        }
    }

    /// Returns the next pseudo-random `u32`.
    const fn next_u32(&mut self) -> u32 {
        let mut value = self.state;
        value ^= value << 13;
        value ^= value >> 17;
        value ^= value << 5;
        self.state = value;
        value
    }

    /// Returns the next pseudo-random byte without a narrowing cast.
    const fn next_u8(&mut self) -> u8 {
        self.next_u32().to_le_bytes()[0]
    }
}

/// Maps a direction to its queue and counter slot.
const fn slot(direction: Direction) -> usize {
    match direction {
        Direction::RobotToPc => 0,
        Direction::PcToRobot => 1,
    }
}

/// A deterministic byte pipe between the robot and the PC.
///
/// A caller sends bytes in a [`Direction`] and drains them with [`Wire::take`]. The clock only
/// moves when [`Wire::advance`] is called or a send applies [`Faults::stall_ms`], so the wire
/// never reads the wall clock.
#[derive(Debug)]
pub struct Wire {
    /// The simulated millisecond clock.
    now_ms: u32,
    /// The queued bytes in each direction; index with [`slot`].
    queues: [VecDeque<u8>; 2],
    /// The number of bytes sent in each direction, used by the periodic byte faults.
    byte_counts: [u64; 2],
    /// The number of chunks sent in each direction, used by truncation.
    chunk_counts: [u64; 2],
    /// One independent generator per direction so faults do not couple.
    rng: [XorShift; 2],
    /// The active fault settings.
    faults: Faults,
}

impl Wire {
    /// Creates a wire with an explicit seed and fault settings.
    ///
    /// The same seed and the same sequence of calls always produce the same bytes and clock.
    #[must_use]
    pub const fn new(seed: u32, faults: Faults) -> Self {
        Self {
            now_ms: 0,
            queues: [VecDeque::new(), VecDeque::new()],
            byte_counts: [0; 2],
            chunk_counts: [0; 2],
            rng: [XorShift::new(seed), XorShift::new(seed ^ DIRECTION_SEED_OFFSET)],
            faults,
        }
    }

    /// Returns the simulated clock in milliseconds.
    #[must_use]
    pub const fn now_ms(&self) -> u32 {
        self.now_ms
    }

    /// Advances the simulated clock by `ms` milliseconds.
    pub const fn advance(&mut self, ms: u32) {
        self.now_ms = self.now_ms.wrapping_add(ms);
    }

    /// Returns the active fault settings.
    #[must_use]
    pub const fn faults(&self) -> Faults {
        self.faults
    }

    /// Replaces the fault settings; bytes already queued are not changed.
    pub const fn set_faults(&mut self, faults: Faults) {
        self.faults = faults;
    }

    /// Sends `bytes` in `direction`, applying every active fault.
    ///
    /// Bytes are counted across sends and per direction. A chunk is truncated before its bytes
    /// are dropped or corrupted, and a non-empty chunk advances the clock by
    /// [`Faults::stall_ms`].
    pub fn send(&mut self, direction: Direction, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }

        let index = slot(direction);
        self.chunk_counts[index] = self.chunk_counts[index].wrapping_add(1);
        let chunk = self.chunk_counts[index];
        let truncate = self.faults.truncate_every != 0 && chunk.is_multiple_of(u64::from(self.faults.truncate_every));
        let end = if truncate {
            bytes.len().saturating_sub(1)
        } else {
            bytes.len()
        };

        for &original in &bytes[..end] {
            self.byte_counts[index] = self.byte_counts[index].wrapping_add(1);
            let count = self.byte_counts[index];
            if self.faults.drop_every != 0 && count.is_multiple_of(u64::from(self.faults.drop_every)) {
                continue;
            }
            let corrupt = self.faults.corrupt_every != 0 && count.is_multiple_of(u64::from(self.faults.corrupt_every));
            let byte = if corrupt { self.rng[index].next_u8() } else { original };
            self.queues[index].push_back(byte);
        }

        self.now_ms = self.now_ms.wrapping_add(self.faults.stall_ms);
    }

    /// Returns how many bytes are queued in `direction`.
    #[must_use]
    pub fn available(&self, direction: Direction) -> usize {
        self.queues[slot(direction)].len()
    }

    /// Drains and returns every byte queued in `direction`.
    #[must_use]
    pub fn take(&mut self, direction: Direction) -> Vec<u8> {
        self.queues[slot(direction)].drain(..).collect()
    }
}
