//! Cross-crate end-to-end acceptance over the simulated wire.
//!
//! Each test composes `LinkCore`, `sim::Wire` and `sim::FakeRemote` and exercises one contract
//! behaviour that only appears across the seam: resynchronisation, command dedupe and the
//! indeterminate outcome, event retransmit and overflow, throttle merging and expiry, version
//! mismatch, a Boot Id change, and the lease with its Deadman edge.

use remote_link::{Config, Inbound, LinkCore, PeerState, Publish};
use remote_protocol::{
    AckOutcome, AckTracker, Caps, Class, Command, Direction, Event, EventMessage, Flags, Frame, MAX_PC_TO_ROBOT_FRAME,
    MAX_ROBOT_TO_PC_FRAME, PROTOCOL_MAJOR, PROTOCOL_MINOR, RetryPolicy, Seq, Status, Stream, StreamSample,
    ThrottleAction, ThrottleHint,
    codec::PayloadCodec,
    payload::{
        control::{HelloAck, RateEntry},
        event::{EventOverflow, Obstacle},
        stream::{Motion, Power},
    },
    sim::{FakeRemote, Faults, Wire},
};

/// The Boot Id the first core is constructed with.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// A second Boot Id, used to prove the peer window and daemon state reset.
const OTHER_BOOT_ID: u32 = 0x0BAD_C0DF;

/// The command payload used by the command tests.
const STOP_MOTION_COAST: [u8; 1] = [0];

/// Creates a core, wire and remote for one acceptance path.
fn harness() -> (LinkCore, Wire, FakeRemote) {
    (
        LinkCore::new(Config::default(), BOOT_ID, 0),
        Wire::new(1, Faults::default()),
        FakeRemote::new(),
    )
}

/// Drains the wire's PC-to-robot queue into the core at `now_ms`.
fn feed(core: &mut LinkCore, wire: &mut Wire, now_ms: u64) {
    let bytes = wire.take(Direction::PcToRobot);
    core.on_rx(&bytes, now_ms);
}

/// Polls until a command appears, skipping any event or control notice that precedes it.
fn poll_command(core: &mut LinkCore) -> Option<Seq> {
    loop {
        match core.poll()? {
            Inbound::Command(request) => return Some(request.seq),
            Inbound::EventAck(_) | Inbound::ControlReply(_) | Inbound::ControlTimeout { .. } => {}
        }
    }
}

/// Emits one frame into `out`, returning its encoded bytes.
fn emit(core: &mut LinkCore) -> Option<Vec<u8>> {
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let written = core.next_frame(&mut buffer)?;
    Some(buffer[..written].to_vec())
}

/// Emits one frame and puts it on the wire for the simulated peer.
fn transmit(core: &mut LinkCore, wire: &mut Wire) -> Option<Vec<u8>> {
    let bytes = emit(core)?;
    wire.send(Direction::RobotToPc, &bytes);
    Some(bytes)
}

/// Sends a `HELLO`, answers it and delivers the `HELLO_ACK`, so the peer learns the Boot Id.
fn attach(core: &mut LinkCore, wire: &mut Wire, remote: &mut FakeRemote, now_ms: u64) {
    let _ = remote.send_hello(wire, PROTOCOL_MAJOR, PROTOCOL_MINOR, Caps::from_bits(0), 1, &[]);
    feed(core, wire, now_ms);
    let Some(bytes) = emit(core) else {
        return;
    };
    wire.send(Direction::RobotToPc, &bytes);
    let _ = remote.receive(wire);
    assert_eq!(core.peer(), PeerState::PeerLive);
}

/// A representative obstacle event.
const fn obstacle(detected: u8) -> EventMessage<'static> {
    EventMessage::Obstacle(Obstacle { detected, source: 0 })
}

/// Encodes one standalone command frame for the resynchronisation test.
fn command_bytes(id: u16, seq: u16, flags: Flags, payload: &[u8]) -> Option<Vec<u8>> {
    let frame = Frame::new(PROTOCOL_MAJOR, Class::Command, flags, id, Seq::new(seq), payload).ok()?;
    let mut buffer = [0u8; MAX_PC_TO_ROBOT_FRAME];
    let written = frame.encode_into(&mut buffer).ok()?;
    Some(buffer[..written].to_vec())
}

/// A damaged frame is skipped and the next complete frame is decoded on both sides of the link.
#[test]
fn resync_after_loss() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);

    // Robot -> PC: lose the last byte of an event, then deliver the complete frame.
    assert_eq!(core.publish_event(&obstacle(1), 0), Publish::Queued(Seq::new(0)));
    let Some(frame) = emit(&mut core) else {
        return;
    };
    wire.send(Direction::RobotToPc, &frame[..frame.len().saturating_sub(1)]);
    wire.send(Direction::RobotToPc, &frame);
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(
        remote.received_event_count(),
        1,
        "the parser resynchronises on the complete frame"
    );

    // PC -> robot: the same loss followed by the complete command.
    let Some(command) = command_bytes(
        Command::StopMotion.wire_id(),
        1,
        Flags::ACK_REQUESTED,
        &STOP_MOTION_COAST,
    ) else {
        return;
    };
    wire.send(Direction::PcToRobot, &command[..command.len().saturating_sub(1)]);
    wire.send(Direction::PcToRobot, &command);
    feed(&mut core, &mut wire, 10);
    assert_eq!(
        poll_command(&mut core),
        Some(Seq::new(1)),
        "a complete command survives the damaged prefix"
    );
}

/// A duplicate command is re-ACKed with the stored result and never yielded to the firmware again.
#[test]
fn duplicate_command_reacks_without_reexecution() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);

    let Some(seq) = remote.send_command(&mut wire, Command::StopMotion, &STOP_MOTION_COAST) else {
        return;
    };
    feed(&mut core, &mut wire, 1);
    let Some(Inbound::Command(request)) = core.poll() else {
        return;
    };
    assert_eq!(request.seq, seq);
    assert!(core.ack_command(seq.get(), Status::Ok, &[0xAA]));
    let Some(first) = transmit(&mut core, &mut wire) else {
        return;
    };
    let _ = remote.receive(&mut wire);
    assert_eq!(remote.last_ack_status(), Some(Status::Ok));

    // The daemon retries the same sequence: the robot replays the stored ACK and never yields.
    assert!(remote.send_command_frame(
        &mut wire,
        Command::StopMotion.wire_id(),
        seq,
        Flags::ACK_REQUESTED,
        &STOP_MOTION_COAST,
    ));
    feed(&mut core, &mut wire, 300);
    assert!(core.poll().is_none(), "a duplicate is never executed again");
    let Some(replay) = transmit(&mut core, &mut wire) else {
        return;
    };
    assert_eq!(replay, first, "the stored ACK is replayed byte for byte");
    let _ = remote.receive(&mut wire);
    assert_eq!(remote.ack_count(), 2, "the peer sees the re-ACK");
    assert_eq!(remote.last_ack_status(), Some(Status::Ok));
}

/// A lost command ACK drives the daemon to an indeterminate outcome; the robot never retries.
#[test]
fn lost_command_ack_is_indeterminate_and_the_robot_does_not_retry() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);

    let payload = [100u8, 0, 50];
    let Some(seq) = remote.send_command(&mut wire, Command::DriveDistance, &payload) else {
        return;
    };
    feed(&mut core, &mut wire, 0);
    let Some(Inbound::Command(request)) = core.poll() else {
        return;
    };
    assert_eq!(request.seq, seq);
    assert!(core.ack_command(seq.get(), Status::Ok, &[]));
    let Some(_ack) = emit(&mut core) else {
        return;
    };
    assert!(
        emit(&mut core).is_none(),
        "the robot never retransmits a command on its own"
    );

    let mut tracker = AckTracker::new(RetryPolicy::COMMAND);
    assert!(tracker.begin(seq, BOOT_ID, 0));

    assert_eq!(tracker.poll(300), AckOutcome::Retransmit { seq, boot_id: BOOT_ID });
    assert!(remote.send_command_frame(
        &mut wire,
        Command::DriveDistance.wire_id(),
        seq,
        Flags::ACK_REQUESTED | Flags::RETRY,
        &payload,
    ));
    feed(&mut core, &mut wire, 300);
    assert!(
        core.poll().is_none(),
        "a retransmission is a duplicate, not a new command"
    );
    let Some(_ack) = emit(&mut core) else {
        return;
    };

    assert_eq!(tracker.poll(900), AckOutcome::Retransmit { seq, boot_id: BOOT_ID });
    assert!(remote.send_command_frame(
        &mut wire,
        Command::DriveDistance.wire_id(),
        seq,
        Flags::ACK_REQUESTED | Flags::RETRY,
        &payload,
    ));
    feed(&mut core, &mut wire, 900);
    assert!(core.poll().is_none(), "the robot still never re-executes");
    let Some(_ack) = emit(&mut core) else {
        return;
    };

    assert_eq!(
        tracker.poll(1_900),
        AckOutcome::Indeterminate { seq, boot_id: BOOT_ID },
        "the daemon reports an unknown outcome, not a failure"
    );
    assert_eq!(tracker.poll(1_901), AckOutcome::Idle);
    assert!(!tracker.is_in_flight());
}

/// A lost event ACK triggers a retransmit with the same sequence; the next ACK retires it.
#[test]
fn event_retransmit_and_ack_retirement() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);

    assert_eq!(core.publish_event(&obstacle(1), 0), Publish::Queued(Seq::new(0)));
    wire.set_direction_faults(
        Direction::PcToRobot,
        Faults {
            drop_every: 1,
            ..Faults::default()
        },
    );
    let Some(_first) = transmit(&mut core, &mut wire) else {
        return;
    };
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(remote.received_event_count(), 1);
    feed(&mut core, &mut wire, 0);
    assert_eq!(core.pending_events(), 1, "a lost ACK leaves the event queued");

    core.tick(500);
    let Some(retry) = transmit(&mut core, &mut wire) else {
        return;
    };
    let Some(decoded) = Frame::decode(&retry, Direction::RobotToPc).ok() else {
        return;
    };
    assert!(
        decoded.header.flags.contains(Flags::RETRY),
        "a retransmission sets RETRY"
    );
    assert_eq!(decoded.header.seq, Seq::new(0), "the sequence is unchanged");
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(remote.received_event_count(), 1, "the peer deduplicates the retry");
    assert_eq!(remote.event_duplicates(), 1);
    feed(&mut core, &mut wire, 500);
    assert_eq!(core.pending_events(), 1, "the ACK is still lost");

    wire.set_direction_faults(Direction::PcToRobot, Faults::default());
    core.tick(1_500);
    let Some(_retry) = transmit(&mut core, &mut wire) else {
        return;
    };
    assert_eq!(remote.receive(&mut wire), 0);
    feed(&mut core, &mut wire, 1_500);
    assert_eq!(core.pending_events(), 0, "the delivered ACK retires the event");
    let Some(Inbound::EventAck(ack)) = core.poll() else {
        return;
    };
    assert_eq!(ack.seq, Seq::new(0));
    assert_eq!(ack.status, Status::Ok);
}

/// Ring overflow drops the oldest event and later reports the count in `EVENT_OVERFLOW`.
#[test]
fn event_ring_overflow_reports_event_overflow_with_the_dropped_count() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);

    for value in 0..9u8 {
        let _ = core.publish_event(&obstacle(value), 0);
    }
    assert_eq!(core.pending_events(), 8, "the ring holds eight events");
    assert_eq!(core.dropped_events(), 1, "the ninth publish drops the oldest");

    for step in 0..32u64 {
        if core.pending_events() == 0 {
            break;
        }
        let Some(_frame) = transmit(&mut core, &mut wire) else {
            break;
        };
        assert_eq!(remote.receive(&mut wire), 0);
        feed(&mut core, &mut wire, step * 10);
    }

    let Some((event, _, message)) = remote.last_event() else {
        return;
    };
    assert_eq!(event, Event::EventOverflow, "the overflow report is the last event");
    assert_eq!(
        message,
        EventMessage::EventOverflow(EventOverflow { dropped: 1 }),
        "the report carries the dropped count"
    );
    assert_eq!(core.dropped_events(), 0, "the report consumes the counter");
    assert!(
        remote.events().any(|(event, _, _)| event == Event::Obstacle),
        "the surviving events were delivered too"
    );
}

/// The most restrictive active hint wins, and an expired hint reverts to the next rate.
#[test]
fn throttle_ttl_expiry_and_most_restrictive_wins() {
    let config = Config {
        robot_caps: Caps::THROTTLE_SUPPORT,
        default_rates_chz: [100; Stream::COUNT],
        heartbeat_ms: [1_000; Stream::COUNT],
        ..Config::default()
    };
    let mut core = LinkCore::new(config, BOOT_ID, 0);
    let mut wire = Wire::new(1, Faults::default());
    let mut remote = FakeRemote::new();

    let rates = [RateEntry::enabled(Stream::Power.wire_id(), 200)];
    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, PROTOCOL_MINOR, Caps::from_bits(0), 1, &rates) else {
        return;
    };
    feed(&mut core, &mut wire, 0);
    let _ = emit(&mut core);

    let Some(_) = remote.send_throttle(&mut wire, Stream::Power.wire_id(), ThrottleAction::SetRate, 20, 250) else {
        return;
    };
    let Some(_) = remote.send_throttle(
        &mut wire,
        ThrottleHint::ALL_STREAMS,
        ThrottleAction::SetRate,
        50,
        10_000,
    ) else {
        return;
    };
    feed(&mut core, &mut wire, 0);

    core.publish_stream(
        &StreamSample::Power(Power {
            level: 1,
            voltage_mv: 3_000,
        }),
        0,
        0,
    );
    assert!(emit(&mut core).is_some(), "the first sample is due");

    core.publish_stream(
        &StreamSample::Power(Power {
            level: 2,
            voltage_mv: 3_100,
        }),
        0,
        100,
    );
    core.tick(100);
    assert!(
        emit(&mut core).is_none(),
        "the 20 centi-Hz per-stream hint wins over the 50 centi-Hz global hint"
    );

    core.tick(250);
    assert!(
        emit(&mut core).is_some(),
        "when the per-stream hint expires, the 50 centi-Hz global hint still allows the change"
    );
}

/// A mismatched major drops control and streams, answers commands, and keeps the robot streaming.
#[test]
fn version_mismatch() {
    let config = Config {
        robot_caps: Caps::MAP,
        ..Config::default()
    };
    let mut core = LinkCore::new(config, BOOT_ID, 0);
    let mut wire = Wire::new(1, Faults::default());
    let mut remote = FakeRemote::new();

    let Some(seq) = remote.send_hello(
        &mut wire,
        PROTOCOL_MAJOR + 1,
        PROTOCOL_MINOR,
        Caps::MAP | Caps::DELTA_CLOUD,
        3,
        &[],
    ) else {
        return;
    };
    feed(&mut core, &mut wire, 0);
    assert!(core.version_mismatch());
    assert_eq!(core.caps(), Caps::MAP, "caps still intersect on a mismatch");

    let Some(frame) = emit(&mut core) else {
        return;
    };
    let Some(decoded) = Frame::decode(&frame, Direction::RobotToPc).ok() else {
        return;
    };
    let Some(ack) = HelloAck::decode(decoded.payload).ok() else {
        return;
    };
    assert_eq!(decoded.header.seq, seq);
    assert_eq!(ack.status, Status::VersionMismatch);
    assert_eq!(ack.boot_id, BOOT_ID);
    wire.send(Direction::RobotToPc, &frame);
    let _ = remote.receive(&mut wire);

    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed(&mut core, &mut wire, 1);
    assert!(emit(&mut core).is_none(), "a mismatched peer's control is ignored");

    let Some(_) = remote.send_command(&mut wire, Command::StopMotion, &STOP_MOTION_COAST) else {
        return;
    };
    feed(&mut core, &mut wire, 2);
    assert!(core.poll().is_none(), "a mismatched command is never dispatched");
    let Some(response) = emit(&mut core) else {
        return;
    };
    let Some(decoded) = Frame::decode(&response, Direction::RobotToPc).ok() else {
        return;
    };
    assert!(decoded.header.flags.contains(Flags::IS_ACK));
    assert_eq!(decoded.payload, &[Status::VersionMismatch.bits()][..]);

    core.publish_stream(&StreamSample::Motion(Motion { left: 1, right: 2 }), 0, 3);
    let Some(stream) = emit(&mut core) else {
        return;
    };
    let Some(decoded) = Frame::decode(&stream, Direction::RobotToPc).ok() else {
        return;
    };
    assert_eq!(
        decoded.header.class,
        Class::Stream,
        "the robot keeps streaming on a mismatch"
    );
}

/// A new Boot Id resets the peer's event window and discards the daemon's in-flight command.
#[test]
fn boot_id_change_resets_the_peer_window_and_daemon_state() {
    let mut wire = Wire::new(1, Faults::default());
    let mut remote = FakeRemote::new();
    let mut old = LinkCore::new(Config::default(), BOOT_ID, 0);
    attach(&mut old, &mut wire, &mut remote, 0);
    assert_eq!(remote.boot_id(), Some(BOOT_ID));

    // One command is in flight under the old Boot Id and its ACK is lost.
    let Some(seq) = remote.send_command(&mut wire, Command::StopMotion, &STOP_MOTION_COAST) else {
        return;
    };
    feed(&mut old, &mut wire, 1);
    let Some(Inbound::Command(request)) = old.poll() else {
        return;
    };
    assert_eq!(request.seq, seq);
    assert!(old.ack_command(seq.get(), Status::Ok, &[]));
    let _ = emit(&mut old);

    let mut tracker = AckTracker::new(RetryPolicy::COMMAND);
    assert!(tracker.begin(seq, BOOT_ID, 1));

    // The peer has also seen event sequence zero from the old boot.
    assert_eq!(old.publish_event(&obstacle(1), 1), Publish::Queued(Seq::new(0)));
    let Some(_) = transmit(&mut old, &mut wire) else {
        return;
    };
    let _ = remote.receive(&mut wire);
    assert_eq!(remote.received_event_count(), 1);

    // The robot restarts with a new Boot Id and re-handshakes.
    let mut new = LinkCore::new(Config::default(), OTHER_BOOT_ID, 2);
    attach(&mut new, &mut wire, &mut remote, 2);
    assert_eq!(remote.boot_id(), Some(OTHER_BOOT_ID), "the peer learns the new Boot Id");

    assert_eq!(
        tracker.on_boot_id(OTHER_BOOT_ID),
        Some(AckOutcome::DiscardedOnBootChange { seq }),
        "the in-flight command is discarded, never retried"
    );
    assert_eq!(tracker.poll(300), AckOutcome::Idle);
    assert!(!tracker.is_in_flight());

    // The event window restarts: sequence zero is accepted again under the new boot.
    assert_eq!(new.publish_event(&obstacle(2), 2), Publish::Queued(Seq::new(0)));
    let Some(_) = transmit(&mut new, &mut wire) else {
        return;
    };
    let _ = remote.receive(&mut wire);
    assert_eq!(remote.received_event_count(), 2, "the new boot's event is accepted");
    assert_eq!(remote.event_duplicates(), 0, "the window reset on the Boot Id change");
}

/// The 2 s Deadman edge fires before the 5 s lease expires, and re-attach re-arms it.
#[test]
fn lease_expiry_and_peer_lost() {
    let (mut core, mut wire, mut remote) = harness();

    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed(&mut core, &mut wire, 0);
    assert_eq!(core.peer(), PeerState::PeerLive);

    core.tick(1_999);
    assert!(!core.take_peer_lost(), "1.999 s is not the threshold");
    core.tick(2_000);
    assert!(core.take_peer_lost(), "2 s of silence raises the edge");
    assert!(!core.take_peer_lost(), "the edge is one-shot");

    core.tick(4_999);
    assert_eq!(core.peer(), PeerState::PeerLive);
    core.tick(5_000);
    assert_eq!(core.peer(), PeerState::NoPeer, "5 s of silence expires the lease");

    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed(&mut core, &mut wire, 5_000);
    assert_eq!(core.peer(), PeerState::PeerLive, "a ping re-attaches");
    core.tick(7_000);
    assert!(core.take_peer_lost(), "the Deadman re-arms after re-attach");
}
