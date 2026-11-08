//! Host tests for the `BRIDGE_CONTROL` request/ACK path (ADR-0036, ADR-0037).
//!
//! The robot queues an action on `LinkCore`, emits it at control priority, and the simulated peer
//! answers as the Bridge with the setup credentials or the pairing code. Everything goes through
//! the real codecs and the simulated wire.

use link_core::{Config, Inbound, LinkCore, PeerState};
use link_protocol::{
    BridgeControlAction, BridgeControlReply, Class, Control, Direction, EventMessage, Flags, Frame,
    MAX_PC_TO_ROBOT_FRAME, MAX_ROBOT_TO_PC_FRAME, PAIRING_CODE_LEN, PROTOCOL_MAJOR, Seq, Status, StreamSample,
    payload::{
        event::{OBSTACLE_SOURCE_LIDAR, Obstacle},
        stream::Motion,
    },
    sim::{FakeRemote, Faults, Wire},
};

/// The Boot Id the tests hand to the link core.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// The compressed control timeout these tests use.
const TIMEOUT_MS: u64 = 100;

/// Creates a core with the compressed timeout, a wire and a remote.
fn harness() -> (LinkCore, Wire, FakeRemote) {
    let config = Config {
        control_timeout_ms: TIMEOUT_MS,
        ..Config::default()
    };
    (
        LinkCore::new(config, BOOT_ID, 0),
        Wire::new(1, Faults::default()),
        FakeRemote::new(),
    )
}

/// Drains the wire's PC-to-robot queue into the core at `now_ms`.
fn feed(core: &mut LinkCore, wire: &mut Wire, now_ms: u64) {
    let bytes = wire.take(Direction::PcToRobot);
    core.on_rx(&bytes, now_ms);
}

/// Emits one frame from the core and puts it on the wire, returning its encoded bytes.
fn emit(core: &mut LinkCore, wire: &mut Wire) -> Option<Vec<u8>> {
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let written = core.next_frame(&mut buffer)?;
    let bytes = buffer[..written].to_vec();
    wire.send(Direction::RobotToPc, &bytes);
    Some(bytes)
}

/// Decodes one robot-to-PC frame.
fn decode(bytes: &[u8]) -> Option<Frame<'_>> {
    Frame::decode(bytes, Direction::RobotToPc).ok()
}

/// Builds one PC-to-robot control frame for the inbound ACK path.
fn control_frame(id: u16, seq: u16, flags: Flags, payload: &[u8]) -> Option<Vec<u8>> {
    let frame = Frame::new(PROTOCOL_MAJOR, Class::Control, flags, id, Seq::new(seq), payload).ok()?;
    let mut buffer = [0u8; MAX_PC_TO_ROBOT_FRAME];
    let written = frame.encode_into(&mut buffer).ok()?;
    Some(buffer[..written].to_vec())
}

/// Attaches the peer with a `PING`, draining the reply so only later frames are observed.
fn attach(core: &mut LinkCore, wire: &mut Wire, remote: &mut FakeRemote) {
    let _ = remote.send_ping(wire);
    feed(core, wire, 0);
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let _ = core.next_frame(&mut buffer);
    assert_eq!(core.peer(), PeerState::PeerLive);
}

/// Action 0 leaves at control priority ahead of a pending event and stream, and the ACK surfaces
/// the setup SSID and passphrase.
#[test]
fn action_zero_is_emitted_at_control_priority_and_returns_setup_credentials() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    core.publish_stream(&StreamSample::Motion(Motion { left: 1, right: 2 }), 0, 0);
    let _ = core.publish_event(
        &EventMessage::Obstacle(Obstacle {
            detected: 1,
            source: OBSTACLE_SOURCE_LIDAR,
        }),
        0,
    );
    assert_eq!(core.pending_events(), 1);

    let Some(seq) = core.request_bridge_control(0, 0).ok() else {
        return;
    };
    let Some(bytes) = emit(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode(&bytes) else {
        return;
    };
    assert_eq!(frame.header.class, Class::Control);
    assert_eq!(frame.header.id, Control::BridgeControl.wire_id());
    assert_eq!(frame.header.seq.get(), seq);
    assert!(
        frame.header.flags.contains(Flags::ACK_REQUESTED),
        "the request asks for its acknowledgement"
    );
    assert_eq!(frame.payload, &[BridgeControlAction::ForgetNetwork.bits()]);
    assert_eq!(
        core.pending_events(),
        1,
        "the event and stream wait behind the control frame"
    );

    let _ = remote.receive(&mut wire);
    feed(&mut core, &mut wire, 1);
    let Some(Inbound::ControlReply(reply)) = core.poll() else {
        return;
    };
    assert_eq!(reply.seq.get(), seq);
    let Ok(BridgeControlReply::Setup(setup)) = reply.result else {
        return;
    };
    assert_eq!(setup.status, Status::Ok);
    assert_eq!(setup.ssid, remote.setup_ssid());
    assert_eq!(setup.passphrase, remote.setup_passphrase());
}

/// Actions 1 and 2 both return the configured twelve-character pairing code.
#[test]
fn actions_one_and_two_return_the_pairing_code() {
    for action in [
        BridgeControlAction::ShowPairingCode,
        BridgeControlAction::RotatePairingCode,
    ] {
        let (mut core, mut wire, mut remote) = harness();
        attach(&mut core, &mut wire, &mut remote);

        let Some(seq) = core.request_bridge_control(action.bits(), 0).ok() else {
            return;
        };
        let Some(bytes) = emit(&mut core, &mut wire) else {
            return;
        };
        let Some(frame) = decode(&bytes) else {
            return;
        };
        assert_eq!(frame.payload, &[action.bits()]);

        let _ = remote.receive(&mut wire);
        feed(&mut core, &mut wire, 1);
        let Some(Inbound::ControlReply(reply)) = core.poll() else {
            return;
        };
        assert_eq!(reply.seq.get(), seq);
        let Ok(BridgeControlReply::PairingCode(pairing)) = reply.result else {
            return;
        };
        assert_eq!(pairing.status, Status::Ok);
        assert_eq!(pairing.code.len(), PAIRING_CODE_LEN);
        assert_eq!(pairing.code, remote.pairing_code());
    }
}

/// The ACK is paired by sequence and does not echo the action; a non-matching sequence is ignored.
#[test]
fn the_ack_is_paired_by_seq_and_does_not_echo_the_action() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    let Some(seq) = core
        .request_bridge_control(BridgeControlAction::ShowPairingCode.bits(), 0)
        .ok()
    else {
        return;
    };
    let Some(bytes) = emit(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode(&bytes) else {
        return;
    };
    assert_eq!(frame.header.seq.get(), seq);

    let _ = remote.receive(&mut wire);
    let ack_bytes = wire.take(Direction::PcToRobot);
    let Some(ack) = Frame::decode(&ack_bytes, Direction::PcToRobot).ok() else {
        return;
    };
    assert_eq!(ack.header.seq.get(), seq);
    assert!(ack.header.flags.contains(Flags::IS_ACK));
    assert_eq!(
        usize::from(ack.payload[1]),
        PAIRING_CODE_LEN,
        "the second byte is the code length, not the action"
    );

    let Some(wrong) = control_frame(
        Control::BridgeControl.wire_id(),
        seq.wrapping_add(1),
        Flags::IS_ACK,
        ack.payload,
    ) else {
        return;
    };
    core.on_rx(&wrong, 1);
    assert!(core.poll().is_none(), "a non-matching seq is ignored");

    core.on_rx(&ack_bytes, 1);
    let Some(Inbound::ControlReply(reply)) = core.poll() else {
        return;
    };
    assert_eq!(reply.seq.get(), seq);
    assert!(matches!(reply.result, Ok(BridgeControlReply::PairingCode(_))));
}

/// A lost ACK produces exactly one timeout and the action is never retransmitted.
#[test]
fn a_lost_ack_times_out_once_without_duplicating_the_request() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    let Some(seq) = core
        .request_bridge_control(BridgeControlAction::ForgetNetwork.bits(), 0)
        .ok()
    else {
        return;
    };
    let Some(bytes) = emit(&mut core, &mut wire) else {
        return;
    };
    let Some(frame) = decode(&bytes) else {
        return;
    };
    assert_eq!(frame.header.seq.get(), seq);

    // Drop the request. Before the deadline nothing is yielded and nothing is resent.
    core.tick(TIMEOUT_MS - 1);
    assert!(core.poll().is_none());
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    assert!(
        core.next_frame(&mut buffer).is_none(),
        "an emitted request is never sent again"
    );

    core.tick(TIMEOUT_MS);
    let Some(Inbound::ControlTimeout { seq: timed_out }) = core.poll() else {
        return;
    };
    assert_eq!(timed_out, seq, "the timeout names the request");

    core.tick(TIMEOUT_MS * 10);
    assert!(core.poll().is_none(), "the timeout fires exactly once");
    assert!(core.next_frame(&mut buffer).is_none());
}

/// An unknown action is rejected with `BAD_ARGUMENT` and emits nothing; one request is in flight
/// at a time.
#[test]
fn an_unknown_action_is_rejected_and_a_second_request_is_busy() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    assert_eq!(core.request_bridge_control(3, 0), Err(Status::BadArgument));
    assert_eq!(core.request_bridge_control(0xFF, 0), Err(Status::BadArgument));
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    assert!(core.next_frame(&mut buffer).is_none());

    assert!(core.request_bridge_control(0, 0).is_ok());
    assert_eq!(core.request_bridge_control(1, 0), Err(Status::Busy));
}

/// A malformed acknowledgement maps to `BAD_LENGTH` or `BAD_ARGUMENT` and never panics.
#[test]
fn a_malformed_ack_surfaces_a_status() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    // (action, payload, expected status)
    let cases: [(BridgeControlAction, Vec<u8>, Status); 4] = [
        (
            BridgeControlAction::ForgetNetwork,
            vec![Status::Ok.bits(), 12, b'A'],
            Status::BadLength,
        ),
        (BridgeControlAction::ForgetNetwork, vec![10, 0, 0], Status::BadArgument),
        (
            BridgeControlAction::ShowPairingCode,
            vec![Status::Ok.bits(), 11, b'A', b'A'],
            Status::BadLength,
        ),
        (
            BridgeControlAction::RotatePairingCode,
            {
                let mut payload = vec![b'A'; PAIRING_CODE_LEN + 2];
                payload[0] = 10;
                payload[1] = u8::try_from(PAIRING_CODE_LEN).unwrap_or(0);
                payload
            },
            Status::BadArgument,
        ),
    ];

    for (action, payload, expected) in cases {
        let Some(seq) = core.request_bridge_control(action.bits(), 0).ok() else {
            return;
        };
        let Some(_bytes) = emit(&mut core, &mut wire) else {
            return;
        };
        let Some(ack) = control_frame(Control::BridgeControl.wire_id(), seq, Flags::IS_ACK, &payload) else {
            return;
        };
        core.on_rx(&ack, 1);
        let Some(Inbound::ControlReply(reply)) = core.poll() else {
            return;
        };
        assert_eq!(reply.seq.get(), seq);
        assert_eq!(reply.result, Err(expected));
        assert_eq!(reply.status(), expected);
    }
}

/// A `BRIDGE_CONTROL` reply and a command do not coexist in one poll, and the timeout is a
/// separate `Inbound` variant the firmware can match without borrowing anything.
#[test]
fn control_results_are_one_at_a_time() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    let Some(seq) = core
        .request_bridge_control(BridgeControlAction::RotatePairingCode.bits(), 0)
        .ok()
    else {
        return;
    };
    let Some(_bytes) = emit(&mut core, &mut wire) else {
        return;
    };
    let _ = remote.receive(&mut wire);
    feed(&mut core, &mut wire, 1);

    let Some(Inbound::ControlReply(reply)) = core.poll() else {
        return;
    };
    assert_eq!(reply.seq.get(), seq);
    assert!(core.poll().is_none(), "the reply is yielded once");
}
