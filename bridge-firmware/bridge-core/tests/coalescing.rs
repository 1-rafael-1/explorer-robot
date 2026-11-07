//! Host tests for the latest-wins coalescing ring (ADR-0030, ADR-0033).
//!
//! The ring is the client-bound buffer: one pending slot per stream, with reliable entries in
//! reserved room. Expected sizes are the ADRs' worked numbers, never a recomputation of the
//! implementation's arithmetic.

use bridge_core::{Class, EntryKind, PushOutcome, Ring};
use remote_protocol::Seq;

/// The ADR-0033 ring size: 8 KiB.
const EIGHT_KIB: usize = 8 * 1024;

/// The ADR-0033 event reservation: eight event entries fit in under 0.5 KiB.
const HALF_KIB: usize = 512;

/// A payload of `len` copies of `fill`.
fn bytes(len: usize, fill: u8) -> Vec<u8> {
    vec![fill; len]
}

/// An empty ring reports the ADR's capacity and the event reservation.
#[test]
fn the_ring_reports_eight_kib_and_the_event_reservation() {
    let ring = Ring::new();
    assert_eq!(ring.capacity_bytes(), EIGHT_KIB);
    assert_eq!(ring.reserved_bytes(), HALF_KIB);
    assert_eq!(ring.used_bytes(), 0);
    assert_eq!(ring.dropped_streams(), 0);
    assert_eq!(ring.dropped_reliable(), 0);
}

/// A newer sample for the same stream replaces the pending one instead of queueing.
#[test]
fn a_newer_stream_sample_replaces_the_pending_one() {
    let mut ring = Ring::new();
    assert_eq!(ring.push_stream(0x0002, &bytes(2, 0x11)), PushOutcome::Stored);
    assert_eq!(ring.used_bytes(), 2);

    assert_eq!(ring.push_stream(0x0002, &bytes(2, 0x22)), PushOutcome::Replaced);
    assert_eq!(ring.used_bytes(), 2, "the old sample is gone, not queued");

    let Some(entry) = ring.pop() else {
        return;
    };
    assert_eq!(entry.kind, EntryKind::Stream(0x0002));
    assert_eq!(entry.payload, &[0x22, 0x22]);
    assert!(ring.pop().is_none(), "one pending slot per stream");
}

/// Replacing a sample with a differently sized one reclaims the old bytes.
#[test]
fn a_replacement_reclaims_the_previous_size() {
    let mut ring = Ring::new();
    assert_eq!(ring.push_stream(0x0002, &bytes(8, 0x11)), PushOutcome::Stored);
    assert_eq!(ring.push_stream(0x0002, &bytes(3, 0x22)), PushOutcome::Replaced);
    assert_eq!(ring.used_bytes(), 3);

    let Some(entry) = ring.pop() else {
        return;
    };
    assert_eq!(entry.payload.len(), 3);
}

/// Stream pressure stops at 8 KiB less the 512-byte reservation: the eighth sample is dropped even
/// though the raw ring still has room for it.
#[test]
fn a_stream_sample_is_dropped_before_it_can_take_the_event_reservation() {
    let mut ring = Ring::new();

    // Seven 1 KiB streams use 7 KiB; an eighth would reach the full 8 KiB and would need the
    // reservation the ADR keeps for reliable entries.
    for id in 0x0010..0x0017 {
        assert_eq!(ring.push_stream(id, &bytes(1024, 0)), PushOutcome::Stored);
    }
    assert_eq!(ring.used_bytes(), 7 * 1024);

    assert_eq!(ring.push_stream(0x0017, &bytes(1024, 0)), PushOutcome::Dropped);
    assert_eq!(ring.dropped_streams(), 1);
    assert_eq!(ring.used_bytes(), 7 * 1024, "the dropped sample left no trace");
    assert!(ring.used_bytes() <= EIGHT_KIB - HALF_KIB);
}

/// Reliable events still fit after stream pressure has filled the stream budget.
#[test]
fn reliable_events_survive_full_stream_pressure() {
    let mut ring = Ring::new();
    for id in 0x0010..0x0017 {
        assert_eq!(ring.push_stream(id, &bytes(1024, 0)), PushOutcome::Stored);
    }
    assert_eq!(ring.push_stream(0x0017, &bytes(1024, 0)), PushOutcome::Dropped);

    // Eight events at the ADR-0033 worst case (58-byte frame, hence 46 payload bytes; this test
    // uses 58 payload bytes, which is still inside the reservation).
    for seq in 0..8u16 {
        assert!(ring.push_event(0x0001, Seq::new(seq), &bytes(58, 0)).is_ok());
    }
    assert_eq!(ring.dropped_reliable(), 0);
    assert_eq!(ring.used_bytes(), 7 * 1024 + 8 * 58);

    let mut events = 0usize;
    ring.drain(|entry| {
        if matches!(
            entry.kind,
            EntryKind::Reliable {
                class: Class::Event,
                ..
            }
        ) {
            events += 1;
        }
    });
    assert_eq!(events, 8, "every reserved event survived the stream pressure");
    assert_eq!(ring.used_bytes(), 0, "the drain reclaimed every byte");
}

/// Reliable entries keep their class and sequence so the network edge can forward them unchanged.
#[test]
fn a_reliable_entry_carries_its_class_id_and_sequence() {
    let mut ring = Ring::new();
    assert!(
        ring.push_reliable(Class::Command, 0x0005, Seq::new(42), &bytes(1, 0))
            .is_ok()
    );

    let Some(entry) = ring.pop() else {
        return;
    };
    assert_eq!(
        entry.kind,
        EntryKind::Reliable {
            class: Class::Command,
            id: 0x0005,
            seq: Seq::new(42),
        }
    );
}

/// The drain yields entries in arrival order, one at a time, and the ring empties.
#[test]
fn drain_yields_each_entry_once_in_arrival_order() {
    let mut ring = Ring::new();
    assert_eq!(ring.push_stream(0x0001, &bytes(3, 1)), PushOutcome::Stored);
    assert!(ring.push_event(0x0002, Seq::new(7), &bytes(2, 2)).is_ok());
    assert_eq!(ring.push_stream(0x0003, &bytes(1, 3)), PushOutcome::Stored);

    let mut kinds = Vec::new();
    ring.drain(|entry| kinds.push(entry.kind));
    assert_eq!(
        kinds,
        vec![
            EntryKind::Stream(0x0001),
            EntryKind::Reliable {
                class: Class::Event,
                id: 0x0002,
                seq: Seq::new(7),
            },
            EntryKind::Stream(0x0003),
        ]
    );
    assert_eq!(ring.used_bytes(), 0);
}
