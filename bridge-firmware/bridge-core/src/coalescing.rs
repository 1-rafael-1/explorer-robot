//! The latest-wins coalescing ring (ADR-0030, ADR-0033).
//!
//! The ring is the Bridge's client-bound buffer. A stream has exactly one pending slot: a newer
//! sample replaces the older one, so a Wi-Fi stall drops stale telemetry instead of queueing
//! seconds of it. Reliable entries — events, command acknowledgements and control replies — are
//! not coalesced; they may use the whole ring, and the stream budget reserves their room so
//! stream pressure can never take it.
//!
//! The ring is sized by ADR-0033: 8 KiB total, with eight worst-case event frames (under 0.5 KiB)
//! reserved. It owns no clock and no I/O; the caller pushes parsed frames and drains entries.
//!
//! Every entry carries the wire frame's sequence number and flags, so a drained entry can be
//! re-encoded exactly as it arrived. Coalescing replaces an older stream sample with a newer one;
//! the surviving entry always describes the most recently pushed frame, header included.

use remote_protocol::{Class, Flags, Seq};

/// The ring's total byte capacity: ADR-0033's 8 KiB.
pub const CAPACITY: usize = 8 * 1024;

/// The bytes reserved for reliable entries: ADR-0033's "eight entries fit in under 0.5 KiB".
pub const EVENT_RESERVED_BYTES: usize = 512;

/// The bytes streams may occupy: the total less the reliable reservation.
pub const STREAM_CAPACITY: usize = CAPACITY - EVENT_RESERVED_BYTES;

/// The largest payload a robot-to-PC frame can carry (ADR-0030).
const MAX_PAYLOAD: usize = 1024;

/// The largest number of entries the descriptor table can hold.
const MAX_ENTRIES: usize = 16;

/// The identity and wire header of one queued entry.
///
/// `seq` and `flags` are the values from the parsed frame's header. The ring stores them verbatim
/// so the firmware can re-encode the frame without inventing an implied header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EntryKind {
    /// A telemetry stream sample, coalesced by its stream id.
    Stream {
        /// The stream id.
        id: u16,
        /// The frame's sequence number.
        seq: Seq,
        /// The frame's flags.
        flags: Flags,
    },
    /// A reliable frame that stream pressure cannot displace.
    Reliable {
        /// The frame's delivery class.
        class: Class,
        /// The class-specific id.
        id: u16,
        /// The frame's sequence number.
        seq: Seq,
        /// The frame's flags.
        flags: Flags,
    },
}

/// One queued entry, borrowed from the ring's drain scratch.
///
/// The payload stays valid until the next call that mutates the ring, exactly like an iterator
/// item borrowed from the ring.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Entry<'a> {
    /// What the entry is.
    pub kind: EntryKind,
    /// The frame payload.
    pub payload: &'a [u8],
}

/// What happened to a stream sample offered to the ring.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PushOutcome {
    /// A new stream sample was queued.
    Stored,
    /// An existing stream sample was replaced.
    Replaced,
    /// The sample did not fit and was discarded.
    Dropped,
}

/// A reliable entry that could not be queued.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PushError {
    /// The payload is larger than one robot-to-PC frame can carry.
    TooLarge,
}

/// One descriptor in the ring's table, pointing at bytes in the arena.
#[derive(Clone, Copy)]
struct Slot {
    /// What the slot holds.
    kind: EntryKind,
    /// The payload's offset in the arena.
    start: usize,
    /// The payload's length in bytes.
    len: usize,
}

impl Slot {
    /// An unused descriptor; `count` keeps it from being read.
    const EMPTY: Self = Self {
        kind: EntryKind::Stream {
            id: 0,
            seq: Seq::new(0),
            flags: Flags::from_bits(0),
        },
        start: 0,
        len: 0,
    };
}

/// The 8 KiB latest-wins coalescing ring.
pub struct Ring {
    /// The byte arena holding every entry's payload back to back.
    bytes: [u8; CAPACITY],
    /// A copy target for [`Ring::pop`], so a drained payload can outlive its arena slot.
    scratch: [u8; MAX_PAYLOAD],
    /// The descriptor table, ordered by arrival.
    slots: [Slot; MAX_ENTRIES],
    /// The number of valid descriptors in `slots`.
    count: usize,
    /// The bytes in use across every entry.
    used: usize,
    /// The bytes used by stream entries.
    stream_used: usize,
    /// The bytes used by reliable entries.
    reliable_used: usize,
    /// How many stream samples have been discarded.
    dropped_streams: u32,
    /// How many reliable entries have been discarded or evicted.
    dropped_reliable: u32,
}

impl Ring {
    /// Creates an empty ring.
    ///
    /// The ring is a fixed 8 KiB buffer that the firmware owns for its whole lifetime (a static or
    /// a long-lived task field), so the constructor's frame size is expected.
    #[must_use]
    #[allow(
        clippy::large_stack_frames,
        reason = "the 8 KiB ring is long-lived state, not a per-call buffer"
    )]
    pub const fn new() -> Self {
        Self {
            bytes: [0; CAPACITY],
            scratch: [0; MAX_PAYLOAD],
            slots: [Slot::EMPTY; MAX_ENTRIES],
            count: 0,
            used: 0,
            stream_used: 0,
            reliable_used: 0,
            dropped_streams: 0,
            dropped_reliable: 0,
        }
    }

    /// Returns the ring's total capacity.
    #[must_use]
    pub const fn capacity_bytes(&self) -> usize {
        CAPACITY
    }

    /// Returns the bytes reserved for reliable entries.
    #[must_use]
    pub const fn reserved_bytes(&self) -> usize {
        EVENT_RESERVED_BYTES
    }

    /// Returns the bytes currently in use.
    #[must_use]
    pub const fn used_bytes(&self) -> usize {
        self.used
    }

    /// Returns the bytes currently used by stream samples.
    #[must_use]
    pub const fn stream_bytes(&self) -> usize {
        self.stream_used
    }

    /// Returns the bytes currently used by reliable entries.
    #[must_use]
    pub const fn reliable_bytes(&self) -> usize {
        self.reliable_used
    }

    /// Returns how many stream samples have been discarded.
    #[must_use]
    pub const fn dropped_streams(&self) -> u32 {
        self.dropped_streams
    }

    /// Returns how many reliable entries have been discarded or evicted.
    #[must_use]
    pub const fn dropped_reliable(&self) -> u32 {
        self.dropped_reliable
    }

    /// Offers a stream sample, replacing the same stream's pending sample when one exists.
    ///
    /// `seq` and `flags` are stored with the sample and returned unchanged by [`Ring::pop`], even
    /// when a newer sample replaces an older one.
    #[must_use]
    pub fn push_stream(&mut self, stream_id: u16, seq: Seq, flags: Flags, payload: &[u8]) -> PushOutcome {
        let len = payload.len();
        if len > MAX_PAYLOAD {
            self.dropped_streams += 1;
            return PushOutcome::Dropped;
        }

        if let Some(index) = self.stream_slot(stream_id) {
            let old = self.slots[index];
            let fits = self.used - old.len + len <= CAPACITY && self.stream_used - old.len + len <= STREAM_CAPACITY;
            if !fits {
                self.dropped_streams += 1;
                return PushOutcome::Dropped;
            }
            let _ = self.remove_at(index);
            self.append(
                EntryKind::Stream {
                    id: stream_id,
                    seq,
                    flags,
                },
                payload,
            );
            return PushOutcome::Replaced;
        }

        if self.count < MAX_ENTRIES && self.used + len <= CAPACITY && self.stream_used + len <= STREAM_CAPACITY {
            self.append(
                EntryKind::Stream {
                    id: stream_id,
                    seq,
                    flags,
                },
                payload,
            );
            PushOutcome::Stored
        } else {
            self.dropped_streams += 1;
            PushOutcome::Dropped
        }
    }

    /// Queues an event payload without coalescing it.
    ///
    /// # Errors
    ///
    /// Returns [`PushError::TooLarge`] when the payload cannot fit one robot-to-PC frame.
    pub fn push_event(&mut self, event_id: u16, seq: Seq, flags: Flags, payload: &[u8]) -> Result<(), PushError> {
        self.push_reliable(Class::Event, event_id, seq, flags, payload)
    }

    /// Queues a reliable robot-to-PC payload without coalescing it.
    ///
    /// Reliable entries are not coalesced. When the ring is full, the oldest stream is evicted
    /// first and the oldest reliable entry only after that, so stream pressure yields before a
    /// reliable entry does.
    ///
    /// # Errors
    ///
    /// Returns [`PushError::TooLarge`] when the payload cannot fit one robot-to-PC frame.
    pub fn push_reliable(
        &mut self,
        class: Class,
        id: u16,
        seq: Seq,
        flags: Flags,
        payload: &[u8],
    ) -> Result<(), PushError> {
        let len = payload.len();
        if len > MAX_PAYLOAD {
            self.dropped_reliable += 1;
            return Err(PushError::TooLarge);
        }

        loop {
            if self.count < MAX_ENTRIES && self.used + len <= CAPACITY {
                self.append(EntryKind::Reliable { class, id, seq, flags }, payload);
                return Ok(());
            }
            let Some(index) = self.oldest_stream().or_else(|| self.oldest_reliable()) else {
                self.dropped_reliable += 1;
                return Err(PushError::TooLarge);
            };
            let evicted = self.remove_at(index);
            match evicted.kind {
                EntryKind::Stream { .. } => self.dropped_streams += 1,
                EntryKind::Reliable { .. } => self.dropped_reliable += 1,
            }
        }
    }

    /// Removes and returns the oldest entry, or [`None`] when the ring is empty.
    pub fn pop(&mut self) -> Option<Entry<'_>> {
        if self.count == 0 {
            return None;
        }
        let slot = self.slots[0];
        self.scratch[..slot.len].copy_from_slice(&self.bytes[slot.start..slot.start + slot.len]);
        let kind = slot.kind;
        let _ = self.remove_at(0);
        Some(Entry {
            kind,
            payload: &self.scratch[..slot.len],
        })
    }

    /// Drains every entry in arrival order, calling `sink` for each.
    pub fn drain(&mut self, mut sink: impl FnMut(Entry<'_>)) {
        while let Some(entry) = self.pop() {
            sink(entry);
        }
    }

    /// Returns the descriptor index of `stream_id`'s pending sample.
    fn stream_slot(&self, stream_id: u16) -> Option<usize> {
        self.slots[..self.count]
            .iter()
            .position(|slot| matches!(slot.kind, EntryKind::Stream { id, .. } if id == stream_id))
    }

    /// Returns the descriptor index of the oldest stream entry.
    fn oldest_stream(&self) -> Option<usize> {
        self.slots[..self.count]
            .iter()
            .position(|slot| matches!(slot.kind, EntryKind::Stream { .. }))
    }

    /// Returns the descriptor index of the oldest reliable entry.
    fn oldest_reliable(&self) -> Option<usize> {
        self.slots[..self.count]
            .iter()
            .position(|slot| matches!(slot.kind, EntryKind::Reliable { .. }))
    }

    /// Appends a payload; the caller has already checked the capacity and descriptor space.
    fn append(&mut self, kind: EntryKind, payload: &[u8]) {
        let start = self.used;
        let len = payload.len();
        self.bytes[start..start + len].copy_from_slice(payload);
        self.slots[self.count] = Slot { kind, start, len };
        self.count += 1;
        self.used += len;
        match kind {
            EntryKind::Stream { .. } => self.stream_used += len,
            EntryKind::Reliable { .. } => self.reliable_used += len,
        }
    }

    /// Removes the descriptor at `index`, compacting the arena and the table.
    fn remove_at(&mut self, index: usize) -> Slot {
        let slot = self.slots[index];
        let tail_start = slot.start + slot.len;
        let used = self.used;
        self.bytes.copy_within(tail_start..used, slot.start);
        for offset in index..self.count - 1 {
            self.slots[offset] = self.slots[offset + 1];
            self.slots[offset].start -= slot.len;
        }
        self.count -= 1;
        self.used -= slot.len;
        match slot.kind {
            EntryKind::Stream { .. } => self.stream_used -= slot.len,
            EntryKind::Reliable { .. } => self.reliable_used -= slot.len,
        }
        slot
    }
}

impl Default for Ring {
    #[allow(
        clippy::large_stack_frames,
        reason = "the 8 KiB ring is long-lived state, not a per-call buffer"
    )]
    fn default() -> Self {
        Self::new()
    }
}
