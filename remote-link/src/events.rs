//! The robot's encoded event ring and its retransmission schedule (ADR-0030, ADR-0031, ADR-0041).
//!
//! The firmware publishes a typed [`EventMessage`]; this module encodes it immediately into a fixed
//! ring of [`timing::EVENT_RING`](crate::timing::EVENT_RING) entries. `publish` never blocks and
//! allocates nothing: a full ring drops its oldest entry and counts the loss, and the counter is
//! reported later as a synthesised `EVENT_OVERFLOW` when a slot is free.
//!
//! Each entry is transmitted once and then retransmitted with the same sequence and the `RETRY`
//! flag on [`RetryPolicy::EVENT`]'s schedule. An ACK retires the entry; the ACK's status is
//! diagnostic only. While the peer's lease is expired no entry is transmitted or polled for a
//! retry. When the peer re-attaches, [`EventRing::rearm`] gives every surviving entry a fresh
//! schedule: an exhausted entry is retried again rather than forgotten.

use remote_protocol::{
    AckOutcome, AckTracker, Class, Event, EventMessage, Flags, Frame, PROTOCOL_MAJOR, RetryPolicy, Seq, Status,
    codec::encode_event,
    payload::event::{EVENT_PAYLOAD_MAX, EventOverflow},
};

/// The largest encoded event payload, sized from the protocol catalogue.
pub const EVENT_PAYLOAD_BYTES: usize = EVENT_PAYLOAD_MAX;

/// The fixed number of encoded events the ring can hold.
const EVENT_RING_CAPACITY: usize = crate::timing::EVENT_RING;

/// The number of inbound ACK notices buffered before further ones are dropped.
///
/// A conforming peer ACKs once per event and the firmware drains [`crate::LinkCore::poll`]
/// regularly; dropping a notice never loses the event because the ring entry is already retired.
const EVENT_ACK_CAPACITY: usize = 8;

/// The outcome of [`crate::LinkCore::publish_event`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Publish {
    /// The event was encoded and queued with this sequence.
    Queued(Seq),
    /// The event was encoded and queued with this sequence, and the ring dropped its oldest entry
    /// to make room. An `EVENT_OVERFLOW` is reported as soon as a slot is free.
    Overflowed(Seq),
    /// The value was an acknowledgement, or could not be encoded. Nothing was queued.
    Rejected,
}

impl Publish {
    /// Returns the sequence assigned to the event, if one was queued.
    #[must_use]
    pub const fn seq(self) -> Option<Seq> {
        match self {
            Self::Queued(seq) | Self::Overflowed(seq) => Some(seq),
            Self::Rejected => None,
        }
    }
}

/// An event acknowledgement surfaced to the firmware by [`crate::LinkCore::poll`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EventAckNotice {
    /// The acknowledged event sequence.
    pub seq: Seq,
    /// The peer's diagnostic status.
    pub status: Status,
}

/// Where one ring entry sits in its transmission schedule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EventPhase {
    /// Encoded and waiting for its first transmission.
    Fresh,
    /// Transmitted at least once; the tracker owns the retry schedule.
    Sent,
    /// Every attempt and the final grace interval elapsed; the entry waits for an ACK or a
    /// re-attach.
    Exhausted,
}

/// One encoded event and its schedule.
struct EventEntry {
    /// The event sequence, unchanged by every retransmission.
    seq: Seq,
    /// The event id carried in the frame header.
    event: Event,
    /// The number of valid bytes in `payload`.
    len: usize,
    /// The encoded event payload.
    payload: [u8; EVENT_PAYLOAD_BYTES],
    /// The retry schedule of this entry.
    tracker: AckTracker,
    /// The entry's place in the schedule.
    phase: EventPhase,
    /// Whether a retransmission is due and waiting for [`EventRing::next_frame`].
    retry_due: bool,
}

impl EventEntry {
    /// Creates a fresh entry for a just-published event.
    const fn fresh(
        seq: Seq,
        event: Event,
        len: usize,
        payload: [u8; EVENT_PAYLOAD_BYTES],
        policy: RetryPolicy,
    ) -> Self {
        Self {
            seq,
            event,
            len,
            payload,
            tracker: AckTracker::new(policy),
            phase: EventPhase::Fresh,
            retry_due: false,
        }
    }

    /// Returns `true` when this entry is ready to go on the wire.
    const fn ready(&self) -> bool {
        match self.phase {
            EventPhase::Fresh => true,
            EventPhase::Sent => self.retry_due,
            EventPhase::Exhausted => false,
        }
    }
}

/// The event ring, its sequence counter, the overflow counter and the ACK notices.
pub struct EventRing {
    /// The encoded entries, oldest first.
    entries: heapless::Vec<EventEntry, EVENT_RING_CAPACITY>,
    /// The number of usable entries; `Config::event_ring` can shrink it below the fixed array.
    capacity: usize,
    /// The next event sequence; it starts at zero for each Boot Id and wraps.
    next_seq: u16,
    /// The Boot Id event sequences are scoped to.
    boot_id: u32,
    /// The retry schedule every entry uses.
    policy: RetryPolicy,
    /// The number of events dropped since the last `EVENT_OVERFLOW` was queued.
    dropped: u16,
    /// The most recent millisecond stamp observed by `publish` or `tick`.
    now_ms: u64,
    /// ACK notices waiting for [`crate::LinkCore::poll`].
    acks: heapless::Vec<EventAckNotice, EVENT_ACK_CAPACITY>,
}

impl EventRing {
    /// Creates an empty ring scoped to `boot_id`.
    pub fn new(config: &crate::timing::Config, boot_id: u32) -> Self {
        Self {
            entries: heapless::Vec::new(),
            capacity: config.event_ring.clamp(1, EVENT_RING_CAPACITY),
            next_seq: 0,
            boot_id,
            policy: RetryPolicy::new(
                config.event_retry_ms,
                u64::from(config.event_retry_factor),
                config.event_retry_cap_ms,
                config.event_attempts,
            ),
            dropped: 0,
            now_ms: 0,
            acks: heapless::Vec::new(),
        }
    }

    /// Returns how many events are in the ring, retrying or exhausted.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Returns the number of events dropped since the last `EVENT_OVERFLOW` was queued.
    #[must_use]
    pub const fn dropped(&self) -> u16 {
        self.dropped
    }

    /// Returns how many entries have exhausted their schedule and are waiting for an ACK.
    #[must_use]
    pub fn exhausted(&self) -> usize {
        self.entries
            .iter()
            .filter(|entry| entry.phase == EventPhase::Exhausted)
            .count()
    }

    /// Iterates over the queued `(seq, event)` pairs, oldest first.
    pub fn iter(&self) -> impl Iterator<Item = (Seq, Event)> + '_ {
        self.entries.iter().map(|entry| (entry.seq, entry.event))
    }

    /// Encodes and queues one event, dropping the oldest when the ring is full.
    pub fn publish(&mut self, message: &EventMessage<'_>, now_ms: u64) -> Publish {
        self.now_ms = now_ms;
        let Some(event) = message.event() else {
            return Publish::Rejected;
        };
        let mut encoded = [0u8; EVENT_PAYLOAD_BYTES];
        let Ok(len) = encode_event(message, &mut encoded) else {
            return Publish::Rejected;
        };

        let overflowed = if self.entries.len() >= self.capacity {
            let _ = self.entries.remove(0);
            self.dropped = self.dropped.saturating_add(1);
            true
        } else {
            false
        };

        let seq = self.next_sequence();
        let mut payload = [0u8; EVENT_PAYLOAD_BYTES];
        payload[..len].copy_from_slice(&encoded[..len]);
        if self
            .entries
            .push(EventEntry::fresh(seq, event, len, payload, self.policy))
            .is_err()
        {
            // Unreachable: a full entry was removed above, or the ring had room.
            return Publish::Rejected;
        }
        self.synthesise_overflow();
        if overflowed {
            Publish::Overflowed(seq)
        } else {
            Publish::Queued(seq)
        }
    }

    /// Encodes one ready entry into `out`, returning the bytes written.
    ///
    /// A fresh entry is sent once and begins its retry schedule; a retransmission sets `RETRY` and
    /// keeps the original sequence. Returns [`None`] when nothing is ready or `out` is too small,
    /// leaving every entry untouched.
    pub fn next_frame(&mut self, out: &mut [u8]) -> Option<usize> {
        let index = self.entries.iter().position(EventEntry::ready)?;
        let entry = self.entries.get(index)?;
        let fresh = entry.phase == EventPhase::Fresh;
        let mut flags = Flags::ACK_REQUESTED;
        if !fresh {
            flags |= Flags::RETRY;
        }
        let frame = Frame::new(
            PROTOCOL_MAJOR,
            Class::Event,
            flags,
            entry.event.wire_id(),
            entry.seq,
            &entry.payload[..entry.len],
        )
        .ok()?;
        let written = frame.encode_into(out).ok()?;

        let entry = &mut self.entries[index];
        if fresh {
            // The tracker owns the schedule from the first transmission onwards.
            let _ = entry.tracker.begin(entry.seq, self.boot_id, self.now_ms);
            entry.phase = EventPhase::Sent;
        } else {
            entry.retry_due = false;
        }
        Some(written)
    }

    /// Advances every entry's schedule to `now_ms`.
    ///
    /// Nothing is polled while `live` is `false`, so an expired lease pauses retransmissions
    /// instead of consuming them. A due retransmission is marked ready for
    /// [`EventRing::next_frame`]; a final deadline moves the entry to [`EventPhase::Exhausted`]
    /// and leaves it in the ring.
    pub fn tick(&mut self, now_ms: u64, live: bool) {
        self.now_ms = now_ms;
        if !live {
            return;
        }
        for entry in &mut self.entries {
            if entry.phase != EventPhase::Sent || entry.retry_due {
                continue;
            }
            match entry.tracker.poll(now_ms) {
                AckOutcome::Retransmit { .. } => entry.retry_due = true,
                AckOutcome::Indeterminate { .. } => entry.phase = EventPhase::Exhausted,
                AckOutcome::Idle | AckOutcome::DiscardedOnBootChange { .. } => {}
            }
        }
    }

    /// Retires the entry matching `(event, seq)` and buffers the ACK notice.
    ///
    /// An ACK with an unknown sequence or a different event id is ignored. The status is only
    /// diagnostic, so any status retires the entry.
    pub fn on_ack(&mut self, event: Event, seq: Seq, status: Status) {
        let Some(index) = self
            .entries
            .iter()
            .position(|entry| entry.seq == seq && entry.event == event)
        else {
            return;
        };
        let _ = self.entries.remove(index);
        let notice = EventAckNotice { seq, status };
        if self.acks.push(notice).is_err() {
            // The firmware has stopped polling; keep the newest notice rather than grow.
            let _ = self.acks.remove(0);
            let _ = self.acks.push(notice);
        }
        self.synthesise_overflow();
    }

    /// Pops the oldest ACK notice.
    pub fn pop_ack(&mut self) -> Option<EventAckNotice> {
        if self.acks.is_empty() {
            return None;
        }
        Some(self.acks.remove(0))
    }

    /// Gives every transmitted entry a fresh retry schedule after the peer re-attaches.
    ///
    /// An entry that exhausted its previous schedule is retried again, so a reconnect never
    /// silently forgets an event. Entries that were never transmitted are left for their first
    /// send.
    pub fn rearm(&mut self, now_ms: u64) {
        self.now_ms = now_ms;
        let policy = self.policy;
        let boot_id = self.boot_id;
        for entry in &mut self.entries {
            if entry.phase == EventPhase::Fresh {
                continue;
            }
            entry.tracker = AckTracker::new(policy);
            let _ = entry.tracker.begin(entry.seq, boot_id, now_ms);
            entry.phase = EventPhase::Sent;
            entry.retry_due = true;
        }
    }

    /// Assigns the next event sequence, wrapping through zero.
    const fn next_sequence(&mut self) -> Seq {
        let seq = Seq::new(self.next_seq);
        self.next_seq = self.next_seq.wrapping_add(1);
        seq
    }

    /// Queues an `EVENT_OVERFLOW` report when events were dropped and a slot is free.
    fn synthesise_overflow(&mut self) {
        if self.dropped == 0 || self.entries.len() >= self.capacity {
            return;
        }
        let dropped = self.dropped;
        let message = EventMessage::EventOverflow(EventOverflow { dropped });
        let mut encoded = [0u8; EVENT_PAYLOAD_BYTES];
        let Ok(len) = encode_event(&message, &mut encoded) else {
            return;
        };
        let seq = self.next_sequence();
        let mut payload = [0u8; EVENT_PAYLOAD_BYTES];
        payload[..len].copy_from_slice(&encoded[..len]);
        if self
            .entries
            .push(EventEntry::fresh(seq, Event::EventOverflow, len, payload, self.policy))
            .is_ok()
        {
            self.dropped = 0;
        }
    }
}
