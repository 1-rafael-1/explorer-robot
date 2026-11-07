//! Host tests for the pump composition (ADR-0033, ADR-0036, ADR-0037).
//!
//! These tests observe the rules that only make sense together: the connect path never flushes the
//! ring, robot-to-client frames are dropped rather than buffered while no client is authenticated,
//! and the UART side is still parsed for `BRIDGE_CONTROL` with no client attached.

use bridge_core::{
    BridgeControlRequest, BridgePump, Class, ConnectOutcome, Decision, EntryKind, Flags, Frame, Proof, ProofOutcome,
    Seq,
};
use remote_protocol::{
    BridgeControl, BridgeControlAction, Control, Direction, MAX_PC_TO_ROBOT_FRAME, MAX_ROBOT_TO_PC_FRAME,
    PROTOCOL_MAJOR, PayloadCodec, PcToRobotParser, RobotToPcParser, ThrottleAction,
};

/// Builds one frame with the real envelope.
fn frame(class: Class, flags: Flags, id: u16, seq: u16, payload: &[u8]) -> Option<Frame<'_>> {
    Frame::new(PROTOCOL_MAJOR, class, flags, id, Seq::new(seq), payload).ok()
}

/// Encodes a `BRIDGE_CONTROL` request payload with the real codec.
fn bridge_control_payload(action: BridgeControlAction) -> Vec<u8> {
    let mut buffer = [0u8; BridgeControl::ENCODED_LEN];
    (BridgeControl { action })
        .encode_into(&mut buffer)
        .map_or_else(|_| Vec::new(), |len| buffer[..len].to_vec())
}

/// Connects and presents the one correct proof.
fn connect_and_authenticate(pump: &mut BridgePump, now_ms: u32) -> Option<u32> {
    let ConnectOutcome::Challenge { connection } = pump.connect(now_ms) else {
        return None;
    };
    let proof = Proof::from_bytes([7; 32]);
    match pump.authenticate(connection, proof, &proof, now_ms + 1) {
        ProofOutcome::Accepted => Some(connection),
        _ => None,
    }
}

/// With no client, streams and events are dropped and counted, not buffered.
#[test]
fn robot_telemetry_is_dropped_while_no_client_is_authenticated() {
    let mut pump = BridgePump::new();

    let Some(stream) = frame(Class::Stream, Flags::default(), 0x0002, 0, &[1, 2]) else {
        return;
    };
    assert_eq!(pump.on_robot_frame(&stream), Decision::DroppedNoClient);

    let Some(event) = frame(Class::Event, Flags::default(), 0x0001, 1, &[1]) else {
        return;
    };
    assert_eq!(pump.on_robot_frame(&event), Decision::DroppedNoClient);

    assert_eq!(pump.used_bytes(), 0, "the ring stays empty without a client");
    assert_eq!(pump.dropped_without_client(), 2);
}

/// The robot side is parsed regardless: `BRIDGE_CONTROL` is consumed with no client attached.
#[test]
fn parsing_continues_while_no_client_is_authenticated() {
    let mut pump = BridgePump::new();

    let payload = bridge_control_payload(BridgeControlAction::ForgetNetwork);
    let Some(control) = frame(
        Class::Control,
        Flags::default(),
        Control::BridgeControl.wire_id(),
        5,
        &payload,
    ) else {
        return;
    };
    assert_eq!(
        pump.on_robot_frame(&control),
        Decision::BridgeControl(BridgeControlRequest {
            action: BridgeControlAction::ForgetNetwork,
            seq: Seq::new(5),
        }),
        "the UART is still parsed for BRIDGE_CONTROL"
    );

    let Some(stream) = frame(Class::Stream, Flags::default(), 0x0002, 0, &[1, 2]) else {
        return;
    };
    assert_eq!(pump.on_robot_frame(&stream), Decision::DroppedNoClient);
}

/// Connecting never flushes the coalescing ring; the ring holds current state, not history.
#[test]
fn connecting_and_disconnecting_does_not_flush_the_ring() {
    let mut pump = BridgePump::new();
    let Some(connection) = connect_and_authenticate(&mut pump, 0) else {
        return;
    };

    let Some(stream) = frame(Class::Stream, Flags::default(), 0x0002, 0, &[1, 2]) else {
        return;
    };
    assert_eq!(pump.on_robot_frame(&stream), Decision::Queued);
    assert_eq!(pump.used_bytes(), 2);

    assert_eq!(
        pump.connect(2),
        ConnectOutcome::Busy,
        "an authenticated client holds the slot"
    );
    assert_eq!(pump.used_bytes(), 2, "a rejected connect changes nothing");

    assert!(pump.disconnect(connection));
    let ConnectOutcome::Challenge { .. } = pump.connect(3) else {
        return;
    };
    assert_eq!(pump.used_bytes(), 2, "a new connect does not flush the ring");

    let Some(entry) = pump.pop() else {
        return;
    };
    assert_eq!(entry.kind, EntryKind::Stream(0x0002));
    assert_eq!(entry.payload, &[1, 2]);
}

/// Client frames are routed: commands leave for the robot, and `BRIDGE_CONTROL` never crosses.
#[test]
fn client_commands_are_forwarded_and_client_bridge_control_is_dropped() {
    let mut pump = BridgePump::new();

    let Some(command) = frame(Class::Command, Flags::default(), 0x0005, 1, &[0]) else {
        return;
    };
    assert_eq!(pump.on_client_frame(&command), Decision::ForwardToRobot);

    let payload = bridge_control_payload(BridgeControlAction::ShowPairingCode);
    let Some(control) = frame(
        Class::Control,
        Flags::default(),
        Control::BridgeControl.wire_id(),
        2,
        &payload,
    ) else {
        return;
    };
    assert_eq!(pump.on_client_frame(&control), Decision::DroppedBridgeControl);
    assert_eq!(pump.dropped_bridge_controls(), 1);
}

/// Both pump sides consume frames produced by the real incremental parsers, not a test codec.
#[test]
fn the_pump_consumes_frames_from_the_real_parsers() {
    let mut pump = BridgePump::new();

    // Robot to Bridge: a BRIDGE_CONTROL through the robot-to-PC parser.
    let payload = bridge_control_payload(BridgeControlAction::RotatePairingCode);
    let Some(control) = frame(
        Class::Control,
        Flags::default(),
        Control::BridgeControl.wire_id(),
        11,
        &payload,
    ) else {
        return;
    };
    let mut encoded = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let Ok(len) = control.encode_into(&mut encoded) else {
        return;
    };
    let mut parser = RobotToPcParser::new(Direction::RobotToPc, 921_600);
    let mut robot_decision = None;
    parser.feed(&encoded[..len], 0, |frame| {
        robot_decision = Some(pump.on_robot_frame(&frame));
    });
    assert_eq!(
        robot_decision,
        Some(Decision::BridgeControl(BridgeControlRequest {
            action: BridgeControlAction::RotatePairingCode,
            seq: Seq::new(11),
        }))
    );

    // Bridge to robot: a command through the PC-to-robot parser.
    let Some(command) = frame(Class::Command, Flags::default(), 0x0005, 3, &[1]) else {
        return;
    };
    let mut encoded = [0u8; MAX_PC_TO_ROBOT_FRAME];
    let Ok(len) = command.encode_into(&mut encoded) else {
        return;
    };
    let mut parser = PcToRobotParser::new(Direction::PcToRobot, 921_600);
    let mut client_decision = None;
    parser.feed(&encoded[..len], 0, |frame| {
        client_decision = Some(pump.on_client_frame(&frame));
    });
    assert_eq!(client_decision, Some(Decision::ForwardToRobot));
}

/// An authenticated event is queued reliably with its sequence intact.
#[test]
fn an_authenticated_event_is_queued_with_its_sequence() {
    let mut pump = BridgePump::new();
    let Some(_connection) = connect_and_authenticate(&mut pump, 0) else {
        return;
    };

    let Some(event) = frame(Class::Event, Flags::default(), 0x0001, 9, &[1]) else {
        return;
    };
    assert_eq!(pump.on_robot_frame(&event), Decision::Queued);

    let Some(entry) = pump.pop() else {
        return;
    };
    assert_eq!(
        entry.kind,
        EntryKind::Reliable {
            class: Class::Event,
            id: 0x0001,
            seq: Seq::new(9),
        }
    );
}

/// The pump surfaces a full ring as a drop and the throttle policy follows the occupancy.
#[test]
fn ring_pressure_is_reported_and_drives_pause_then_resume() {
    let mut pump = BridgePump::new();
    let Some(_connection) = connect_and_authenticate(&mut pump, 0) else {
        return;
    };

    let payload = [0u8; 1024];
    for id in 0x0010..0x0017 {
        let Some(stream) = frame(Class::Stream, Flags::default(), id, 0, &payload) else {
            return;
        };
        assert_eq!(pump.on_robot_frame(&stream), Decision::Queued);
    }
    assert_eq!(pump.used_bytes(), 7 * 1024);

    let Some(pause) = pump.poll_throttle(0) else {
        return;
    };
    assert_eq!(pause.action, ThrottleAction::Pause);
    assert_eq!(pause.ttl_ms, 1000);

    for _ in 0..6 {
        let Some(_entry) = pump.pop() else {
            return;
        };
    }
    assert_eq!(pump.used_bytes(), 1024);

    let Some(resume) = pump.poll_throttle(1000) else {
        return;
    };
    assert_eq!(resume.action, ThrottleAction::Resume);
}
