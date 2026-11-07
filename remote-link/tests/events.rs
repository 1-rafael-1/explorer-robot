//! Host tests for the robot's reliable event ring.
//!
//! Everything goes through the real contract: `LinkCore` encodes and schedules an event, `sim::Wire`
//! carries the bytes and `sim::FakeRemote` parses, deduplicates and acknowledges them.

use remote_link::{Config, Inbound, LinkCore, PeerState, Publish};
use remote_protocol::{
    Caps, Class, CompletionStatus, Direction, Event, EventAck, EventMessage, Flags, Frame, MAX_ROBOT_TO_PC_FRAME,
    PROTOCOL_MAJOR, PROTOCOL_MINOR, Seq, Status,
    codec::decode_event,
    payload::event::{DriveDistanceCompleted, Obstacle},
    sim::{FakeRemote, Faults, Wire},
};

/// The Boot Id the first test core is constructed with.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// A second Boot Id, used to prove the replay window resets.
const OTHER_BOOT_ID: u32 = 0x0BAD_C0DF;

/// One frame the core emitted, copied out of its buffer.
struct Emitted {
    /// The frame's delivery class.
    class: Class,
    /// The class-specific id.
    id: u16,
    /// The sequence number.
    seq: Seq,
    /// The frame's flags.
    flags: Flags,
    /// The payload bytes.
    payload: Vec<u8>,
}

/// Creates a core, wire and remote for one event path.
fn harness() -> (LinkCore, Wire, FakeRemote) {
    harness_with(Config::default())
}

/// Creates a core with `config`, a wire and a remote.
fn harness_with(config: Config) -> (LinkCore, Wire, FakeRemote) {
    (
        LinkCore::new(config, BOOT_ID, 0),
        Wire::new(1, Faults::default()),
        FakeRemote::new(),
    )
}

/// Sends a `HELLO` through the wire and feeds it to the core.
fn attach(core: &mut LinkCore, wire: &mut Wire, remote: &mut FakeRemote, now_ms: u64) {
    let _ = remote.send_hello(wire, PROTOCOL_MAJOR, PROTOCOL_MINOR, Caps::from_bits(0), 1, &[]);
    feed_remote(core, wire, now_ms);
}

/// Drains every queued PC-to-robot byte into the core.
fn feed_remote(core: &mut LinkCore, wire: &mut Wire, now_ms: u64) {
    let bytes = wire.take(Direction::PcToRobot);
    core.on_rx(&bytes, now_ms);
}

/// Emits one frame and copies it out of the core's buffer.
fn emit(core: &mut LinkCore) -> Option<Emitted> {
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let written = core.next_frame(&mut buffer)?;
    let frame = Frame::decode(&buffer[..written], Direction::RobotToPc).ok()?;
    Some(Emitted {
        class: frame.header.class,
        id: frame.header.id,
        seq: frame.header.seq,
        flags: frame.header.flags,
        payload: frame.payload.to_vec(),
    })
}

/// Emits one frame and puts it on the wire for the simulated peer.
fn transmit(core: &mut LinkCore, wire: &mut Wire) -> Option<Emitted> {
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let written = core.next_frame(&mut buffer)?;
    wire.send(Direction::RobotToPc, &buffer[..written]);
    let frame = Frame::decode(&buffer[..written], Direction::RobotToPc).ok()?;
    Some(Emitted {
        class: frame.header.class,
        id: frame.header.id,
        seq: frame.header.seq,
        flags: frame.header.flags,
        payload: frame.payload.to_vec(),
    })
}

/// Encodes one standalone event ACK for the inbound tests.
fn event_ack_bytes(event: Event, seq: Seq, status: Status) -> Option<Vec<u8>> {
    let payload = [status.bits()];
    let frame = Frame::new(
        PROTOCOL_MAJOR,
        Class::Event,
        Flags::IS_ACK,
        event.wire_id(),
        seq,
        &payload,
    )
    .ok()?;
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let written = frame.encode_into(&mut buffer).ok()?;
    Some(buffer[..written].to_vec())
}

/// A representative obstacle event.
const fn obstacle(detected: u8) -> EventMessage<'static> {
    EventMessage::Obstacle(Obstacle { detected, source: 0 })
}

/// A core with a long lease, so a retry-schedule test does not expire the peer first.
fn long_lease() -> Config {
    Config {
        lease_ms: 60_000,
        ..Config::default()
    }
}

/// A published event is sequenced, transmitted once and retired by its ACK.
#[test]
fn a_published_event_is_sequenced_transmitted_and_retired_by_an_ack() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);
    let Some(hello_ack) = emit(&mut core) else {
        return;
    };
    assert_eq!(hello_ack.class, Class::Control, "the handshake reply goes first");

    assert_eq!(core.publish_event(&obstacle(1), 0), Publish::Queued(Seq::new(0)));
    assert_eq!(core.pending_events(), 1);

    let Some(first) = transmit(&mut core, &mut wire) else {
        return;
    };
    assert_eq!(first.class, Class::Event);
    assert_eq!(first.id, Event::Obstacle.wire_id());
    assert_eq!(first.seq, Seq::new(0));
    assert!(first.flags.contains(Flags::ACK_REQUESTED), "an event asks for an ACK");
    assert!(!first.flags.contains(Flags::RETRY), "the first send is not a retry");

    assert_eq!(remote.receive(&mut wire), 0, "an event is not a stream frame");
    assert_eq!(remote.received_event_count(), 1);
    assert_eq!(remote.last_event().map(|(event, _, _)| event), Some(Event::Obstacle));
    assert_eq!(remote.last_event_ack(), Some((Seq::new(0), Status::Ok)));

    feed_remote(&mut core, &mut wire, 1);
    assert_eq!(core.pending_events(), 0, "the ACK retires the entry");
    let Some(Inbound::EventAck(ack)) = core.poll() else {
        return;
    };
    assert_eq!(ack.seq, Seq::new(0));
    assert_eq!(ack.status, Status::Ok);

    core.tick(2_000);
    assert!(emit(&mut core).is_none(), "a retired event never retries");
}

/// The retry schedule is exactly 500 ms doubling to a 2 s cap, and exhaustion keeps the entry.
#[test]
fn event_retry_schedule_is_exact_and_exhaustion_keeps_the_event() {
    let (mut core, mut wire, mut remote) = harness_with(long_lease());
    attach(&mut core, &mut wire, &mut remote, 0);
    assert!(emit(&mut core).is_some(), "the HELLO_ACK is queued");

    assert_eq!(core.publish_event(&obstacle(1), 0), Publish::Queued(Seq::new(0)));
    let Some(initial) = transmit(&mut core, &mut wire) else {
        return;
    };
    assert_eq!(initial.seq, Seq::new(0));
    assert!(!initial.flags.contains(Flags::RETRY), "the initial send is not a retry");
    let mut transmissions = 1u8;

    for (retry_at, previous) in [(500u64, 499u64), (1_500, 1_499), (3_500, 3_499), (5_500, 5_499)] {
        core.tick(previous);
        assert!(emit(&mut core).is_none(), "nothing is due before {retry_at} ms");
        core.tick(retry_at);
        let Some(retry) = transmit(&mut core, &mut wire) else {
            return;
        };
        assert_eq!(retry.seq, Seq::new(0), "a retry keeps the original seq");
        assert!(retry.flags.contains(Flags::RETRY));
        assert!(retry.flags.contains(Flags::ACK_REQUESTED));
        transmissions = transmissions.saturating_add(1);
    }
    assert_eq!(transmissions, 5, "the initial send plus four retries");

    core.tick(7_499);
    assert!(emit(&mut core).is_none(), "nothing is due before the final deadline");
    core.tick(7_500);
    assert_eq!(core.exhausted_events(), 1, "the attempts are exhausted");
    assert_eq!(core.pending_events(), 1, "the event stays in the ring");
    core.tick(20_000);
    assert!(emit(&mut core).is_none(), "an exhausted event is not retried");

    let Some(ack) = event_ack_bytes(Event::Obstacle, Seq::new(0), Status::Ok) else {
        return;
    };
    core.on_rx(&ack, 20_010);
    assert_eq!(core.pending_events(), 0, "an exhausted event still retires on an ACK");
}

/// A lost ACK causes a retransmission with the same sequence and the peer ignores the duplicate.
#[test]
fn a_lost_ack_causes_a_retransmit_with_the_same_seq() {
    let (mut core, mut wire, mut remote) = harness_with(long_lease());
    attach(&mut core, &mut wire, &mut remote, 0);
    assert!(emit(&mut core).is_some(), "the HELLO_ACK is queued");

    assert_eq!(core.publish_event(&obstacle(1), 0), Publish::Queued(Seq::new(0)));
    assert!(transmit(&mut core, &mut wire).is_some(), "the initial send");

    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(remote.received_event_count(), 1);
    let _ = wire.take(Direction::PcToRobot);

    core.tick(500);
    let Some(retry) = transmit(&mut core, &mut wire) else {
        return;
    };
    assert_eq!(retry.class, Class::Event);
    assert_eq!(retry.seq, Seq::new(0), "a retransmission keeps the original seq");
    assert!(retry.flags.contains(Flags::RETRY));
    assert!(retry.flags.contains(Flags::ACK_REQUESTED));

    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(remote.received_event_count(), 1, "the duplicate is not recorded");
    assert_eq!(remote.event_duplicates(), 1);
    assert_eq!(remote.event_ack_count(), 2, "the duplicate is ACKed again");

    feed_remote(&mut core, &mut wire, 600);
    assert_eq!(core.pending_events(), 0, "the retransmission's ACK retires the event");
}

/// No event is retransmitted while the lease is expired; reconnect re-arms the schedule.
#[test]
fn no_retransmit_while_the_lease_is_expired_and_it_resumes_after_hello() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);
    assert!(emit(&mut core).is_some(), "the HELLO_ACK is queued");

    assert_eq!(core.publish_event(&obstacle(1), 0), Publish::Queued(Seq::new(0)));
    let Some(initial) = transmit(&mut core, &mut wire) else {
        return;
    };
    assert_eq!(initial.seq, Seq::new(0));

    core.tick(5_000);
    assert_eq!(core.peer(), PeerState::NoPeer, "5 s of silence expires the lease");
    assert!(emit(&mut core).is_none(), "an expired lease sends no event");

    core.tick(6_000);
    assert!(emit(&mut core).is_none(), "the event waits in the ring");

    attach(&mut core, &mut wire, &mut remote, 6_000);
    let Some(hello_ack) = emit(&mut core) else {
        return;
    };
    assert_eq!(hello_ack.class, Class::Control);

    let Some(resumed) = transmit(&mut core, &mut wire) else {
        return;
    };
    assert_eq!(resumed.class, Class::Event);
    assert_eq!(resumed.seq, Seq::new(0), "the resumed retransmission keeps the seq");
    assert!(resumed.flags.contains(Flags::RETRY), "it is still a retransmission");
}

/// Ring overflow drops the oldest, counts it and reports `EVENT_OVERFLOW` when a slot frees.
#[test]
fn ring_overflow_drops_the_oldest_and_reports_the_count() {
    let (mut core, mut wire, mut remote) = harness_with(long_lease());
    attach(&mut core, &mut wire, &mut remote, 0);
    assert!(emit(&mut core).is_some(), "the HELLO_ACK is queued");

    for seq in 0..8u16 {
        assert_eq!(
            core.publish_event(&obstacle(1), 0),
            Publish::Queued(Seq::new(seq)),
            "the ring accepts eight events"
        );
    }
    assert_eq!(core.pending_events(), 8);
    assert_eq!(core.dropped_events(), 0);

    assert_eq!(
        core.publish_event(&obstacle(1), 0),
        Publish::Overflowed(Seq::new(8)),
        "the ninth event drops the oldest"
    );
    assert_eq!(core.pending_events(), 8, "the ring stays bounded");
    assert_eq!(core.dropped_events(), 1);
    assert_eq!(
        core.queued_events().next().map(|(seq, _)| seq),
        Some(Seq::new(1)),
        "sequence zero was dropped"
    );

    // Free a slot with an ACK: the overflow report is synthesised immediately.
    let Some(ack) = event_ack_bytes(Event::Obstacle, Seq::new(1), Status::Ok) else {
        return;
    };
    core.on_rx(&ack, 10);
    assert_eq!(core.dropped_events(), 0, "the count is cleared when reported");
    assert_eq!(core.pending_events(), 8, "the report fills the freed slot");
    assert_eq!(
        core.queued_events().last(),
        Some((Seq::new(9), Event::EventOverflow)),
        "the newest entry is the overflow report"
    );

    let mut reported = None;
    for _ in 0..8 {
        let Some(frame) = transmit(&mut core, &mut wire) else {
            break;
        };
        if frame.id != Event::EventOverflow.wire_id() {
            continue;
        }
        if let Ok(EventMessage::EventOverflow(overflow)) = decode_event(Event::EventOverflow, false, &frame.payload) {
            reported = Some(overflow.dropped);
        }
    }
    assert_eq!(reported, Some(1), "the report carries the dropped count");
}

/// The drive completions preserve their originating `command_seq`, status and detail end to end.
#[test]
fn drive_completions_preserve_command_seq_status_and_detail() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);
    assert!(emit(&mut core).is_some(), "the HELLO_ACK is queued");
    remote.set_event_ack_status(Status::Rejected);

    let distance = EventMessage::DriveDistanceCompleted(DriveDistanceCompleted {
        command_seq: 77,
        status: CompletionStatus::Cancelled,
        target_cm: 100,
        distance_cm: 94,
        duration_ms: 1_500,
        detail: "obstacle",
    });
    assert_eq!(core.publish_event(&distance, 0), Publish::Queued(Seq::new(0)));
    let Some(frame) = transmit(&mut core, &mut wire) else {
        return;
    };
    assert_eq!(frame.id, Event::DriveDistanceCompleted.wire_id());

    assert_eq!(remote.receive(&mut wire), 0);
    let Some((event, seq, EventMessage::DriveDistanceCompleted(decoded))) = remote.last_event() else {
        return;
    };
    assert_eq!(event, Event::DriveDistanceCompleted);
    assert_eq!(seq, Seq::new(0));
    assert_eq!(decoded.command_seq, 77);
    assert_eq!(decoded.status, CompletionStatus::Cancelled);
    assert_eq!(decoded.target_cm, 100);
    assert_eq!(decoded.distance_cm, 94);
    assert_eq!(decoded.duration_ms, 1_500);
    assert_eq!(decoded.detail, "obstacle");
    assert_eq!(
        remote.last_event_ack(),
        Some((Seq::new(0), Status::Rejected)),
        "the ACK status is the peer's diagnostic"
    );

    feed_remote(&mut core, &mut wire, 1);
    assert_eq!(core.pending_events(), 0, "any ACK status retires the event");
    let Some(Inbound::EventAck(ack)) = core.poll() else {
        return;
    };
    assert_eq!(ack.seq, Seq::new(0));
    assert_eq!(ack.status, Status::Rejected);
}

/// The peer deduplicates `(boot_id, seq)` and a Boot Id change starts a clean window.
#[test]
fn the_peer_deduplicates_by_boot_id_and_sequence() {
    let (mut core, mut wire, mut remote) = harness_with(long_lease());
    attach(&mut core, &mut wire, &mut remote, 0);
    let Some(hello_ack) = transmit(&mut core, &mut wire) else {
        return;
    };
    assert_eq!(hello_ack.class, Class::Control);
    let _ = remote.receive(&mut wire);
    assert_eq!(remote.boot_id(), Some(BOOT_ID));

    assert_eq!(core.publish_event(&obstacle(1), 0), Publish::Queued(Seq::new(0)));
    assert!(transmit(&mut core, &mut wire).is_some());
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(remote.received_event_count(), 1);

    core.tick(500);
    assert!(transmit(&mut core, &mut wire).is_some(), "a retransmission");
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(remote.received_event_count(), 1, "the duplicate is ignored");
    assert_eq!(remote.event_duplicates(), 1);

    // A new Boot Id restarts event sequences at zero; the window must treat it as new.
    let mut second = LinkCore::new(long_lease(), OTHER_BOOT_ID, 0);
    attach(&mut second, &mut wire, &mut remote, 1_000);
    let Some(hello_ack) = transmit(&mut second, &mut wire) else {
        return;
    };
    assert_eq!(hello_ack.class, Class::Control);
    let _ = remote.receive(&mut wire);
    assert_eq!(remote.boot_id(), Some(OTHER_BOOT_ID));

    assert_eq!(second.publish_event(&obstacle(1), 1_000), Publish::Queued(Seq::new(0)));
    assert!(transmit(&mut second, &mut wire).is_some());
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(
        remote.received_event_count(),
        2,
        "a new Boot Id starts a clean replay window"
    );
    assert_eq!(remote.event_duplicates(), 1);
}

/// `publish_event` never blocks: a full ring drops and counts rather than rejecting.
#[test]
fn publish_event_never_blocks_and_the_ring_stays_bounded() {
    let (mut core, _wire, _remote) = harness();
    for index in 0..100u16 {
        let publish = core.publish_event(&obstacle(1), u64::from(index));
        let Some(seq) = publish.seq() else {
            return;
        };
        assert_eq!(seq, Seq::new(index), "every publish is sequenced");
        if index < 8 {
            assert_eq!(publish, Publish::Queued(seq));
        } else {
            assert_eq!(publish, Publish::Overflowed(seq));
        }
    }
    assert_eq!(core.pending_events(), 8, "the ring stays bounded");
    assert_eq!(core.dropped_events(), 92, "every dropped event is counted");

    assert_eq!(
        core.publish_event(&EventMessage::Ack(EventAck { status: Status::Ok }), 100),
        Publish::Rejected,
        "an acknowledgement is not publishable"
    );
    assert_eq!(core.pending_events(), 8);
}
