//! Host tests for the Session handshake, the lease and the Peer/Deadman (ADR-0030, ADR-0035).

use remote_link::{Config, Inbound, LinkCore, PeerState};
use remote_protocol::{
    Caps, Class, Command, Control, Direction, Flags, Frame, MAX_PC_TO_ROBOT_FRAME, MAX_ROBOT_TO_PC_FRAME,
    PROTOCOL_MAJOR, PROTOCOL_MINOR, Seq, Status, Stream, StreamSample,
    codec::PayloadCodec,
    payload::{
        control::{HelloAck, PingAck, RateEntry, RateFlags},
        stream::Motion,
    },
    sim::{FakeRemote, Faults, Wire},
};

/// The Boot Id the tests hand to the link core.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// The robot's capabilities in these tests, chosen to make the intersection observable.
fn config() -> Config {
    Config {
        robot_caps: Caps::MAP | Caps::THROTTLE_SUPPORT,
        ..Config::default()
    }
}

/// Creates a core, wire and remote for one session path.
fn harness() -> (LinkCore, Wire, FakeRemote) {
    (
        LinkCore::new(config(), BOOT_ID, 0),
        Wire::new(1, Faults::default()),
        FakeRemote::new(),
    )
}

/// Drains the wire's PC-to-robot queue into the core at `now_ms`.
fn feed_at(core: &mut LinkCore, wire: &mut Wire, now_ms: u64) {
    let bytes = wire.take(Direction::PcToRobot);
    core.on_rx(&bytes, now_ms);
}

/// Emits one frame from the core, puts it on the wire and returns its encoded bytes.
fn emit_to(core: &mut LinkCore, wire: &mut Wire) -> Option<Vec<u8>> {
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let written = core.next_frame(&mut buffer)?;
    let bytes = buffer[..written].to_vec();
    wire.send(Direction::RobotToPc, &bytes);
    Some(bytes)
}

/// Decodes one emitted frame.
fn decode_frame(bytes: &[u8]) -> Option<Frame<'_>> {
    Frame::decode(bytes, Direction::RobotToPc).ok()
}

/// Builds a standalone PC-to-robot stream frame for the "drop on mismatch" test.
fn pc_stream_frame(id: u16, payload: &[u8]) -> Option<Vec<u8>> {
    let frame = Frame::new(
        PROTOCOL_MAJOR,
        Class::Stream,
        Flags::default(),
        id,
        Seq::new(0),
        payload,
    )
    .ok()?;
    let mut buffer = [0u8; MAX_PC_TO_ROBOT_FRAME];
    let written = frame.encode_into(&mut buffer).ok()?;
    Some(buffer[..written].to_vec())
}

/// Feeds a queued command, polls it and acknowledges it as `OK`.
fn accept_command(core: &mut LinkCore, wire: &mut Wire, now_ms: u64) -> bool {
    feed_at(core, wire, now_ms);
    let Some(inbound) = core.poll() else {
        return false;
    };
    let seq = match inbound {
        Inbound::Command(request) => request.seq.get(),
        Inbound::EventAck(_) => return false,
    };
    core.ack_command(seq, Status::Ok, &[])
}

/// `HELLO` intersects caps, stores Requested Rates, suppresses a disabled stream and reports the
/// robot's defaults.
#[test]
fn hello_ack_intersects_caps_and_reports_default_rates() {
    let (mut core, mut wire, mut remote) = harness();
    let rates = [
        RateEntry::enabled(Stream::Power.wire_id(), 100),
        RateEntry {
            stream_id: Stream::Motion.wire_id(),
            rate_chz: 50,
            flags: RateFlags::from_bits(0),
        },
    ];
    let peer_caps = Caps::MAP | Caps::DELTA_CLOUD | Caps::from_bits(1 << 31);
    let Some(hello_seq) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, 0, peer_caps, 7, &rates) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);

    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    assert_eq!(frame.header.class, Class::Control);
    assert!(frame.header.flags.contains(Flags::IS_ACK));
    assert_eq!(frame.header.id, Control::Hello.wire_id());
    assert_eq!(frame.header.seq, hello_seq);

    // Decode the reply through the simulated peer as well.
    let _ = remote.receive(&mut wire);
    let Some((seq, ack)) = remote.last_hello_ack() else {
        return;
    };
    assert_eq!(seq, hello_seq);
    assert_eq!(ack.major, PROTOCOL_MAJOR);
    assert_eq!(ack.minor, PROTOCOL_MINOR);
    assert_eq!(ack.boot_id, BOOT_ID);
    assert_eq!(ack.status, Status::Ok);
    assert_eq!(ack.caps, Caps::MAP, "caps are intersected with the robot's");
    assert_eq!(ack.entry_count(), Stream::COUNT, "one default entry per stream");
    let entries: Vec<_> = ack.entries().collect();
    for stream in Stream::ALL {
        let entry = entries[stream.index()];
        assert_eq!(entry.stream_id, stream.wire_id());
        assert_eq!(entry.rate_chz, config().default_rates_chz[stream.index()]);
        assert!(entry.flags.is_enabled());
    }

    assert_eq!(core.epoch(), 7);
    assert_eq!(core.caps(), Caps::MAP);
    assert!(!core.version_mismatch());
    assert_eq!(core.requested_rate(Stream::Power), Some(100));
    assert_eq!(
        core.requested_rate(Stream::Motion),
        None,
        "a disabled stream is suppressed"
    );
    assert!(!core.stream_requested(Stream::Motion));
}

/// The default rate table is part of the timing configuration, so a host test can override it.
#[test]
fn default_rates_are_overridable_through_config() {
    let rates = [1, 2, 3, 4, 5, 6];
    let mut core = LinkCore::new(
        Config {
            default_rates_chz: rates,
            ..config()
        },
        BOOT_ID,
        0,
    );
    let mut wire = Wire::new(1, Faults::default());
    let mut remote = FakeRemote::new();
    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, 0, Caps::from_bits(0), 0, &[]) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);

    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    let Some(ack) = HelloAck::decode(frame.payload).ok() else {
        return;
    };
    let reported: Vec<_> = ack.entries().map(|entry| entry.rate_chz).collect();
    assert_eq!(reported, rates.to_vec());
}

/// A `PING` is answered on the same sequence with `OK`.
#[test]
fn ping_is_answered_on_the_same_sequence_with_ok() {
    let (mut core, mut wire, mut remote) = harness();
    let Some(seq) = remote.send_ping(&mut wire) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);

    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    assert_eq!(frame.header.class, Class::Control);
    assert!(frame.header.flags.contains(Flags::IS_ACK));
    assert_eq!(frame.header.id, Control::Ping.wire_id());
    assert_eq!(frame.header.seq, seq);
    assert_eq!(
        PingAck::decode(frame.payload).ok(),
        Some(PingAck { status: Status::Ok })
    );

    let _ = remote.receive(&mut wire);
    assert_eq!(remote.last_ping_ack(), Some((seq, PingAck { status: Status::Ok })));
}

/// Any valid PC-to-robot frame renews the lease; 5 s of silence expires it.
#[test]
fn any_valid_frame_renews_the_lease() {
    let (mut core, mut wire, mut remote) = harness();
    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    assert_eq!(core.peer(), PeerState::PeerLive);

    core.tick(4_999);
    assert_eq!(core.peer(), PeerState::PeerLive);

    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed_at(&mut core, &mut wire, 4_999);
    core.tick(9_998);
    assert_eq!(core.peer(), PeerState::PeerLive, "5 s after the renewal");

    core.tick(9_999);
    assert_eq!(core.peer(), PeerState::NoPeer, "the lease expires at 5 s");
}

/// The robot keeps streaming at baseline after the lease expires.
#[test]
fn the_robot_keeps_streaming_after_the_lease_expires() {
    let (mut core, mut wire, mut remote) = harness();
    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    assert!(emit_to(&mut core, &mut wire).is_some(), "drain the PING reply");
    core.tick(5_000);
    assert_eq!(core.peer(), PeerState::NoPeer);

    core.publish_stream(&StreamSample::Motion(Motion { left: 1, right: 2 }), 0, 5_000);
    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    assert_eq!(frame.header.class, Class::Stream);
    assert_eq!(frame.header.id, Stream::Motion.wire_id());
}

/// A major mismatch drops peer streams and control, and answers commands with `VERSION_MISMATCH`.
#[test]
fn a_major_mismatch_drops_peer_traffic_and_answers_commands() {
    let (mut core, mut wire, mut remote) = harness();
    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR + 1, 0, Caps::MAP, 3, &[]) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    assert!(core.version_mismatch());
    assert_eq!(core.caps(), Caps::MAP, "the acknowledgement still intersects caps");

    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    let Some(ack) = HelloAck::decode(frame.payload).ok() else {
        return;
    };
    assert_eq!(ack.status, Status::VersionMismatch);

    // Control is ignored after the mismatch.
    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed_at(&mut core, &mut wire, 1);
    assert!(
        emit_to(&mut core, &mut wire).is_none(),
        "a mismatched peer's control is ignored"
    );

    // Commands are answered but never yielded to the firmware.
    let Some(_) = remote.send_command(&mut wire, Command::StopMotion, &[0]) else {
        return;
    };
    feed_at(&mut core, &mut wire, 2);
    assert!(core.poll().is_none(), "a mismatched command is not dispatched");
    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    assert_eq!(frame.header.class, Class::Command);
    assert!(frame.header.flags.contains(Flags::IS_ACK));
    assert_eq!(frame.payload, &[Status::VersionMismatch.bits()][..]);

    // Streams from the peer are dropped without being counted as unknown ids.
    let Some(stream_frame) = pc_stream_frame(Stream::Power.wire_id(), &[0]) else {
        return;
    };
    wire.send(Direction::PcToRobot, &stream_frame);
    feed_at(&mut core, &mut wire, 3);
    assert_eq!(core.unknown_streams(), 0, "a mismatched stream is dropped");

    // The robot still streams at baseline.
    core.publish_stream(&StreamSample::Motion(Motion { left: 1, right: 2 }), 0, 3);
    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    assert_eq!(frame.header.class, Class::Stream);
}

/// Commands before `HELLO` run under epoch 0, and a new epoch resets command dedupe.
#[test]
fn commands_before_hello_work_and_a_new_epoch_resets_dedupe() {
    let (mut core, mut wire, mut remote) = harness();
    assert_eq!(core.epoch(), 0);

    let Some(_) = remote.send_command(&mut wire, Command::StopMotion, &[0]) else {
        return;
    };
    assert!(accept_command(&mut core, &mut wire, 0));
    assert_eq!(core.last_seq(), 1, "a command before HELLO is accepted");

    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, 0, Caps::from_bits(0), 9, &[]) else {
        return;
    };
    feed_at(&mut core, &mut wire, 1);
    assert_eq!(core.epoch(), 9);
    assert_eq!(core.last_seq(), 0, "a new epoch restarts dedupe");

    assert!(remote.send_command_frame(
        &mut wire,
        Command::StopMotion.wire_id(),
        Seq::new(1),
        Flags::ACK_REQUESTED,
        &[0],
    ));
    assert!(accept_command(&mut core, &mut wire, 2));
    assert_eq!(core.last_seq(), 1, "sequence 1 is accepted again");
}

/// A retransmitted `HELLO` on the same epoch keeps command dedupe.
#[test]
fn a_hello_retry_on_the_same_epoch_keeps_command_dedupe() {
    let (mut core, mut wire, mut remote) = harness();
    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, 0, Caps::from_bits(0), 4, &[]) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    assert!(emit_to(&mut core, &mut wire).is_some());

    let Some(_) = remote.send_command(&mut wire, Command::StopMotion, &[0]) else {
        return;
    };
    assert!(accept_command(&mut core, &mut wire, 1));
    assert_eq!(core.last_seq(), 1);

    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, 0, Caps::from_bits(0), 4, &[]) else {
        return;
    };
    feed_at(&mut core, &mut wire, 2);
    assert_eq!(core.epoch(), 4);
    assert_eq!(core.last_seq(), 1, "a same-epoch retry does not restart dedupe");

    assert!(remote.send_command_frame(
        &mut wire,
        Command::StopMotion.wire_id(),
        Seq::new(1),
        Flags::ACK_REQUESTED,
        &[0],
    ));
    feed_at(&mut core, &mut wire, 3);
    assert!(core.poll().is_none(), "the retried sequence is still a duplicate");
}

/// The Deadman fires at exactly 2 s, is one-shot, and a later attach re-arms it.
#[test]
fn peer_lost_fires_at_exactly_the_deadman_threshold() {
    let (mut core, mut wire, mut remote) = harness();
    assert_eq!(core.peer(), PeerState::NoPeer);

    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    assert_eq!(core.peer(), PeerState::PeerLive);

    core.tick(1_999);
    assert!(!core.take_peer_lost(), "1.999 s is not the threshold");
    core.tick(2_000);
    assert!(core.take_peer_lost(), "2 s raises the edge");
    assert!(!core.take_peer_lost(), "the edge is one-shot");

    core.tick(5_000);
    assert_eq!(core.peer(), PeerState::NoPeer, "the lease expires");

    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed_at(&mut core, &mut wire, 5_000);
    core.tick(7_000);
    assert!(core.take_peer_lost(), "a later attach re-arms the Deadman");
}

/// A reconnect processed before the tick renews the lease and does not fire.
#[test]
fn a_reconnect_before_the_tick_does_not_fire() {
    let (mut core, mut wire, mut remote) = harness();
    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    core.tick(1_999);
    assert!(!core.take_peer_lost());

    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed_at(&mut core, &mut wire, 1_999);
    core.tick(1_999);
    assert!(!core.take_peer_lost(), "the reconnect renews the lease first");
    core.tick(3_998);
    assert!(!core.take_peer_lost(), "1999 ms after the reconnect");
    core.tick(3_999);
    assert!(core.take_peer_lost(), "2000 ms after the reconnect");
}

/// The Boot Id survives a Bridge reset and changes only when a new link period is constructed.
#[test]
fn the_boot_id_survives_a_bridge_reset_and_changes_only_on_construction() {
    let mut core = LinkCore::new(config(), BOOT_ID, 0);
    let mut wire = Wire::new(1, Faults::default());
    let mut remote = FakeRemote::new();
    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, 0, Caps::from_bits(0), 0, &[]) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    let Some(ack) = HelloAck::decode(frame.payload).ok() else {
        return;
    };
    assert_eq!(ack.boot_id, BOOT_ID);

    core.reset_parser();
    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, 0, Caps::from_bits(0), 0, &[]) else {
        return;
    };
    feed_at(&mut core, &mut wire, 1);
    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    let Some(ack) = HelloAck::decode(frame.payload).ok() else {
        return;
    };
    assert_eq!(ack.boot_id, BOOT_ID, "a Bridge reset keeps the Boot Id");
    assert_eq!(core.boot_id(), BOOT_ID);

    let other = LinkCore::new(config(), BOOT_ID + 1, 0);
    assert_eq!(
        other.boot_id(),
        BOOT_ID + 1,
        "a new link period is the only thing that changes it"
    );
}

/// An enabled entry with rate zero is `BAD_ARGUMENT`; unknown and duplicate entries are ignored.
#[test]
fn bad_arguments_are_reported_and_ignored_entries_do_not_corrupt_the_session() {
    let (mut core, mut wire, mut remote) = harness();
    let zero = [RateEntry::enabled(Stream::Power.wire_id(), 0)];
    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, 0, Caps::from_bits(0), 1, &zero) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    let Some(ack) = HelloAck::decode(frame.payload).ok() else {
        return;
    };
    assert_eq!(ack.status, Status::BadArgument);
    assert_eq!(
        core.requested_rate(Stream::Power),
        None,
        "a bad HELLO does not establish a session"
    );

    let mixed = [
        RateEntry::enabled(0x0099, 5),
        RateEntry::enabled(Stream::Power.wire_id(), 100),
        RateEntry::enabled(Stream::Power.wire_id(), 999),
        RateEntry {
            stream_id: Stream::Motion.wire_id(),
            rate_chz: 50,
            flags: RateFlags::from_bits(0),
        },
    ];
    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, 0, Caps::from_bits(0), 2, &mixed) else {
        return;
    };
    feed_at(&mut core, &mut wire, 1);
    let Some(bytes) = emit_to(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode_frame(&bytes) else {
        return;
    };
    let Some(ack) = HelloAck::decode(frame.payload).ok() else {
        return;
    };
    assert_eq!(ack.status, Status::Ok);
    assert_eq!(core.requested_rate(Stream::Power), Some(100), "the first entry wins");
    assert_eq!(core.requested_rate(Stream::Motion), None);
}
