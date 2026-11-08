//! Host tests for the routing filter (ADR-0030, ADR-0036, ADR-0037).
//!
//! Every route is observed at the crate's public API using `link-protocol`'s real frame and
//! payload types. The filter has exactly two control exceptions: `THROTTLE` out to the robot and
//! `BRIDGE_CONTROL` in, which the Bridge consumes and answers.

use bridge_core::{BridgeControlRequest, BridgeReply, Destination, Route, Router, Side};
use link_protocol::{
    BridgeControl, BridgeControlAction, BridgeStream, Class, Control, Flags, Frame, PROTOCOL_MAJOR, PayloadCodec, Seq,
    Stream, ThrottleAction, ThrottleHint,
};

/// Encodes a `BRIDGE_CONTROL` request payload with the real codec.
fn control_payload(action: BridgeControlAction) -> Vec<u8> {
    let mut buffer = [0u8; BridgeControl::ENCODED_LEN];
    (BridgeControl { action })
        .encode_into(&mut buffer)
        .map_or_else(|_| Vec::new(), |len| buffer[..len].to_vec())
}

/// Builds one frame with the real envelope.
fn frame(class: Class, flags: Flags, id: u16, seq: u16, payload: &[u8]) -> Option<Frame<'_>> {
    Frame::new(PROTOCOL_MAJOR, class, flags, id, Seq::new(seq), payload).ok()
}

/// A `THROTTLE` hint is the one control exception that leaves for the robot.
#[test]
fn throttle_from_the_client_is_forwarded_to_the_robot() {
    let mut router = Router::new();
    let throttle = ThrottleHint::all(ThrottleAction::Pause, 0, 1000);
    let mut payload = [0u8; ThrottleHint::ENCODED_LEN];
    let Ok(len) = throttle.encode_into(&mut payload) else {
        return;
    };
    let Some(frame) = frame(
        Class::Control,
        Flags::default(),
        Control::Throttle.wire_id(),
        9,
        &payload[..len],
    ) else {
        return;
    };

    assert_eq!(router.route(Side::Client, &frame), Route::Forward(Destination::Robot));
}

/// The robot never emits `THROTTLE`; a robot-side one is ignored, not forwarded anywhere.
#[test]
fn throttle_from_the_robot_is_ignored() {
    let mut router = Router::new();
    let Some(frame) = frame(
        Class::Control,
        Flags::default(),
        Control::Throttle.wire_id(),
        9,
        &[0; 7],
    ) else {
        return;
    };
    assert_eq!(router.route(Side::Robot, &frame), Route::Ignore);
}

/// A robot-side `BRIDGE_CONTROL` is consumed with its action byte and sequence; action 0 means
/// setup credentials, actions 1 and 2 mean the pairing code.
#[test]
fn bridge_control_from_the_robot_is_consumed_with_its_action_and_sequence() {
    let mut router = Router::new();
    for action in BridgeControlAction::ALL {
        let payload = control_payload(action);
        let Some(frame) = frame(
            Class::Control,
            Flags::default(),
            Control::BridgeControl.wire_id(),
            7,
            &payload,
        ) else {
            return;
        };

        assert_eq!(
            router.route(Side::Robot, &frame),
            Route::BridgeControl(BridgeControlRequest {
                action,
                seq: Seq::new(7),
            })
        );
    }
}

/// The ADR's action byte selects the reply layout: action 0 the setup credentials, actions 1 and 2
/// the pairing code.
#[test]
fn the_action_byte_selects_the_reply_layout() {
    let forget = BridgeControlRequest {
        action: BridgeControlAction::ForgetNetwork,
        seq: Seq::new(1),
    };
    assert_eq!(forget.reply(), BridgeReply::SetupCredentials);

    let show = BridgeControlRequest {
        action: BridgeControlAction::ShowPairingCode,
        seq: Seq::new(1),
    };
    assert_eq!(show.reply(), BridgeReply::PairingCode);

    let rotate = BridgeControlRequest {
        action: BridgeControlAction::RotatePairingCode,
        seq: Seq::new(1),
    };
    assert_eq!(rotate.reply(), BridgeReply::PairingCode);
}

/// A client-side `BRIDGE_CONTROL` is dropped and counted; the Remote cannot take the Bridge off
/// its own network, and the frame is never acknowledged.
#[test]
fn bridge_control_from_the_client_is_dropped_and_counted() {
    let mut router = Router::new();
    let mut expected = 0u32;
    for action in BridgeControlAction::ALL {
        let payload = control_payload(action);
        let Some(frame) = frame(
            Class::Control,
            Flags::default(),
            Control::BridgeControl.wire_id(),
            7,
            &payload,
        ) else {
            return;
        };

        assert_eq!(router.route(Side::Client, &frame), Route::DroppedBridgeControl);
        expected += 1;
        assert_eq!(router.dropped_bridge_controls(), expected);
    }
}

/// `BRIDGE_CONTROL` never crosses to the TCP side, from either direction.
#[test]
fn bridge_control_never_crosses_to_the_client() {
    let mut router = Router::new();
    for side in [Side::Robot, Side::Client] {
        for action in BridgeControlAction::ALL {
            let payload = control_payload(action);
            let Some(frame) = frame(
                Class::Control,
                Flags::default(),
                Control::BridgeControl.wire_id(),
                1,
                &payload,
            ) else {
                return;
            };
            assert_ne!(
                router.route(side, &frame),
                Route::Forward(Destination::Client),
                "BRIDGE_CONTROL must never reach the Remote"
            );
        }
    }
}

/// Streams cross from the robot to the client.
#[test]
fn streams_cross_to_the_client() {
    let mut router = Router::new();
    let Some(frame) = frame(Class::Stream, Flags::default(), 0x0002, 3, &[1, 2]) else {
        return;
    };
    assert_eq!(router.route(Side::Robot, &frame), Route::Forward(Destination::Client));
}

/// The Bridge owns the `BRIDGE_STATUS` stream, so a robot-side frame carrying its id is ignored
/// rather than forwarded to the client where it could masquerade as Bridge telemetry (ADR-0042).
#[test]
fn bridge_status_stream_from_the_robot_is_ignored() {
    let mut router = Router::new();
    let Some(frame) = frame(
        Class::Stream,
        Flags::default(),
        BridgeStream::Status.wire_id(),
        5,
        &[0; 8],
    ) else {
        return;
    };
    assert_eq!(router.route(Side::Robot, &frame), Route::Ignore);
}

/// A normal robot stream still crosses; only the Bridge-originated id is filtered.
#[test]
fn a_normal_stream_from_the_robot_still_crosses() {
    let mut router = Router::new();
    let Some(frame) = frame(Class::Stream, Flags::default(), Stream::Power.wire_id(), 5, &[0; 2]) else {
        return;
    };
    assert_eq!(router.route(Side::Robot, &frame), Route::Forward(Destination::Client));
}

/// The Bridge-originated stream id is never accepted from the client either.
#[test]
fn bridge_status_stream_from_the_client_is_ignored() {
    let mut router = Router::new();
    let Some(frame) = frame(
        Class::Stream,
        Flags::default(),
        BridgeStream::Status.wire_id(),
        5,
        &[0; 8],
    ) else {
        return;
    };
    assert_eq!(router.route(Side::Client, &frame), Route::Ignore);
}

/// The `BRIDGE_STATUS` control message is Bridge-originated; the router ignores it from either
/// peer, since the Bridge injects its snapshot into the pump's ring directly.
#[test]
fn bridge_status_control_is_ignored_from_either_side() {
    let mut router = Router::new();
    for side in [Side::Robot, Side::Client] {
        let Some(frame) = frame(
            Class::Control,
            Flags::default(),
            Control::BridgeStatus.wire_id(),
            2,
            &[],
        ) else {
            return;
        };
        assert_eq!(router.route(side, &frame), Route::Ignore);
    }
}

/// Events cross from the robot to the client.
#[test]
fn events_cross_to_the_client() {
    let mut router = Router::new();
    let Some(frame) = frame(Class::Event, Flags::default(), 0x0001, 4, &[1]) else {
        return;
    };
    assert_eq!(router.route(Side::Robot, &frame), Route::Forward(Destination::Client));
}

/// Commands cross from the client to the robot.
#[test]
fn commands_cross_to_the_robot() {
    let mut router = Router::new();
    let Some(frame) = frame(Class::Command, Flags::default(), 0x0005, 4, &[1]) else {
        return;
    };
    assert_eq!(router.route(Side::Client, &frame), Route::Forward(Destination::Robot));
}

/// Acknowledgements reverse the normal direction: an event ACK goes to the robot and a command
/// ACK goes to the client.
#[test]
fn acknowledgements_cross_by_direction() {
    let mut router = Router::new();

    let Some(event_ack) = frame(Class::Event, Flags::IS_ACK, 0x0001, 4, &[0]) else {
        return;
    };
    assert_eq!(
        router.route(Side::Client, &event_ack),
        Route::Forward(Destination::Robot)
    );

    let Some(command_ack) = frame(Class::Command, Flags::IS_ACK, 0x0005, 4, &[0]) else {
        return;
    };
    assert_eq!(
        router.route(Side::Robot, &command_ack),
        Route::Forward(Destination::Client)
    );
}

/// `HELLO` and `PING` cross by direction, requests to the robot and replies to the client.
#[test]
fn hello_and_ping_cross_by_direction() {
    let mut router = Router::new();
    for id in [Control::Hello.wire_id(), Control::Ping.wire_id()] {
        let Some(request) = frame(Class::Control, Flags::default(), id, 1, &[]) else {
            return;
        };
        assert_eq!(router.route(Side::Client, &request), Route::Forward(Destination::Robot));
        assert_eq!(router.route(Side::Robot, &request), Route::Ignore);

        let Some(reply) = frame(Class::Control, Flags::IS_ACK, id, 1, &[]) else {
            return;
        };
        assert_eq!(router.route(Side::Robot, &reply), Route::Forward(Destination::Client));
        assert_eq!(router.route(Side::Client, &reply), Route::Ignore);
    }
}

/// A frame travelling the wrong way is ignored; the class rules are directional.
#[test]
fn wrong_direction_frames_are_ignored() {
    let mut router = Router::new();

    let Some(stream) = frame(Class::Stream, Flags::default(), 0x0002, 0, &[]) else {
        return;
    };
    assert_eq!(router.route(Side::Client, &stream), Route::Ignore);

    let Some(command) = frame(Class::Command, Flags::default(), 0x0005, 0, &[]) else {
        return;
    };
    assert_eq!(router.route(Side::Robot, &command), Route::Ignore);
}

/// An unknown control id is ignored, matching ADR-0030's rule for control frames.
#[test]
fn unknown_control_ids_are_ignored() {
    let mut router = Router::new();
    let Some(frame) = frame(Class::Control, Flags::default(), 0x0006, 0, &[]) else {
        return;
    };
    assert_eq!(router.route(Side::Robot, &frame), Route::Ignore);
    assert_eq!(router.route(Side::Client, &frame), Route::Ignore);
}

/// A malformed robot-side `BRIDGE_CONTROL` is ignored rather than answered with a bogus action.
#[test]
fn a_malformed_bridge_control_is_ignored() {
    let mut router = Router::new();
    let Some(frame) = frame(
        Class::Control,
        Flags::default(),
        Control::BridgeControl.wire_id(),
        3,
        &[],
    ) else {
        return;
    };
    assert_eq!(router.route(Side::Robot, &frame), Route::Ignore);
    assert_eq!(router.dropped_bridge_controls(), 0);
}
