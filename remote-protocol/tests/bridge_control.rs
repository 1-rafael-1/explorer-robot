//! Host tests for the `BRIDGE_CONTROL` request and its two action-dependent ACK layouts
//! (ADR-0036, ADR-0037).

use remote_protocol::{
    CodecError, Control, ControlMessage, EncodeError, Status,
    codec::PayloadCodec,
    decode_control, encode_control,
    payload::control::{
        BridgeControl, BridgeControlAction, BridgeControlReply, PAIRING_CODE_LEN, PairingCode, SetupCredentials,
        decode_bridge_control_ack, encode_bridge_control_ack,
    },
};

/// The three actions are the wire values zero, one and two; anything else is unassigned.
#[test]
fn bridge_control_actions_cover_the_wire_values() {
    assert_eq!(BridgeControlAction::ALL.len(), 3);
    for (index, action) in BridgeControlAction::ALL.iter().enumerate() {
        assert_eq!(usize::from(action.bits()), index, "bits follow ALL order");
        assert_eq!(
            BridgeControlAction::from_wire(action.bits()),
            Some(*action),
            "from_wire is the inverse of bits"
        );
    }
    assert_eq!(BridgeControlAction::from_wire(3), None);
    assert_eq!(BridgeControlAction::from_wire(0xFF), None);
}

/// The one-byte request round-trips every action.
#[test]
fn bridge_control_request_round_trips() {
    for action in BridgeControlAction::ALL {
        let request = BridgeControl { action };
        let mut buffer = [0u8; BridgeControl::ENCODED_LEN];
        let Some(written) = request.encode_into(&mut buffer).ok() else {
            return;
        };
        assert_eq!(written, BridgeControl::ENCODED_LEN);
        assert_eq!(buffer[0], action.bits());
        assert_eq!(BridgeControl::decode(&buffer[..written]).ok(), Some(request));
    }
}

/// An unknown action or a wrong length is an error, never a panic, through the codec and the
/// class dispatcher.
#[test]
fn bridge_control_rejects_unknown_actions_and_bad_lengths() {
    assert_eq!(BridgeControl::decode(&[3]), Err(CodecError::Value));
    assert_eq!(BridgeControl::decode(&[0xFF]), Err(CodecError::Value));
    assert_eq!(BridgeControl::decode(&[]), Err(CodecError::Length));
    assert_eq!(BridgeControl::decode(&[0, 1]), Err(CodecError::Length));

    let request = BridgeControl {
        action: BridgeControlAction::ShowPairingCode,
    };
    let mut small = [0u8; 0];
    assert_eq!(request.encode_into(&mut small), Err(EncodeError::BufferTooSmall));

    let Some(message) = decode_control(Control::BridgeControl, false, &[2]).ok() else {
        return;
    };
    assert_eq!(
        message,
        ControlMessage::BridgeControl(BridgeControl {
            action: BridgeControlAction::RotatePairingCode
        })
    );
    assert_eq!(
        decode_control(Control::BridgeControl, false, &[9]),
        Err(CodecError::Value)
    );
    assert!(
        decode_control(Control::BridgeControl, true, &[0]).is_err(),
        "the ACK needs the outstanding action and cannot be decoded here"
    );

    let mut out = [0u8; BridgeControl::ENCODED_LEN];
    let Some(written) = encode_control(&ControlMessage::BridgeControl(request), &mut out).ok() else {
        return;
    };
    assert_eq!(written, BridgeControl::ENCODED_LEN);
    assert_eq!(out[0], BridgeControlAction::ShowPairingCode.bits());
}

/// The action-0 acknowledgement round-trips the status and both borrowed strings.
#[test]
fn setup_credentials_round_trip() {
    let ack = SetupCredentials {
        status: Status::Ok,
        ssid: "ExplorerRobot-ABCD",
        passphrase: "setup-mode",
    };
    let mut buffer = [0u8; 64];
    let Some(written) = ack.encode_into(&mut buffer).ok() else {
        return;
    };
    assert_eq!(written, 2 + ack.ssid.len() + 1 + ack.passphrase.len());
    assert_eq!(buffer[0], Status::Ok.bits());
    assert_eq!(usize::from(buffer[1]), ack.ssid.len());

    let Some(decoded) = SetupCredentials::decode(&buffer[..written]).ok() else {
        return;
    };
    assert_eq!(decoded, ack);

    let Some(reply) = decode_bridge_control_ack(BridgeControlAction::ForgetNetwork, &buffer[..written]).ok() else {
        return;
    };
    assert_eq!(reply, BridgeControlReply::Setup(ack));
    assert_eq!(reply.status(), Status::Ok);

    let mut encoded_buffer = [0u8; 64];
    let Some(encoded) = encode_bridge_control_ack(&reply, &mut encoded_buffer).ok() else {
        return;
    };
    assert_eq!(encoded, written);
    assert_eq!(encoded_buffer[..encoded], buffer[..written]);
}

/// An empty SSID or passphrase is structurally valid and the length bytes stay consistent.
#[test]
fn setup_credentials_allow_empty_strings() {
    let ack = SetupCredentials {
        status: Status::Rejected,
        ssid: "",
        passphrase: "",
    };
    let mut buffer = [0u8; SetupCredentials::MIN_ENCODED_LEN];
    let Some(written) = ack.encode_into(&mut buffer).ok() else {
        return;
    };
    assert_eq!(written, SetupCredentials::MIN_ENCODED_LEN);
    assert_eq!(SetupCredentials::decode(&buffer[..written]).ok(), Some(ack));
}

/// The action-0 layout rejects truncation, trailing bytes, an unassigned status and invalid text.
#[test]
fn setup_credentials_reject_malformed_layouts() {
    assert_eq!(SetupCredentials::decode(&[]), Err(CodecError::Length));
    assert_eq!(SetupCredentials::decode(&[0, 0]), Err(CodecError::Length));
    assert_eq!(
        SetupCredentials::decode(&[0, 5, b'a']),
        Err(CodecError::Length),
        "the SSID length runs past the payload"
    );
    assert_eq!(
        SetupCredentials::decode(&[0, 1, b'a', 2, b'p']),
        Err(CodecError::Length),
        "the passphrase length runs past the payload"
    );
    assert_eq!(
        SetupCredentials::decode(&[0, 1, b'a', 1, b'p', 0]),
        Err(CodecError::Length),
        "trailing bytes are a structural mismatch"
    );
    assert_eq!(
        SetupCredentials::decode(&[10, 1, b'a', 1, b'p']),
        Err(CodecError::Value),
        "status 10 is unassigned"
    );
    assert_eq!(
        SetupCredentials::decode(&[0, 1, 0xFF, 0]),
        Err(CodecError::Utf8),
        "the SSID must be UTF-8"
    );
    assert_eq!(
        SetupCredentials::decode(&[0, 0, 1, 0xFF]),
        Err(CodecError::Utf8),
        "the passphrase must be UTF-8"
    );

    let ack = SetupCredentials {
        status: Status::Ok,
        ssid: "abc",
        passphrase: "def",
    };
    let mut small = [0u8; 3];
    assert_eq!(ack.encode_into(&mut small), Err(EncodeError::BufferTooSmall));
}

/// Actions 1 and 2 share the twelve-character pairing-code layout.
#[test]
fn pairing_code_round_trips_for_both_actions() {
    let ack = PairingCode {
        status: Status::Ok,
        code: "ABCD2345EFGH",
    };
    assert_eq!(ack.code.len(), PAIRING_CODE_LEN);

    let mut buffer = [0u8; PairingCode::ENCODED_LEN];
    let Some(written) = ack.encode_into(&mut buffer).ok() else {
        return;
    };
    assert_eq!(written, PairingCode::ENCODED_LEN);
    assert_eq!(usize::from(buffer[1]), PAIRING_CODE_LEN);

    for action in [
        BridgeControlAction::ShowPairingCode,
        BridgeControlAction::RotatePairingCode,
    ] {
        let Some(reply) = decode_bridge_control_ack(action, &buffer[..written]).ok() else {
            return;
        };
        assert_eq!(reply, BridgeControlReply::PairingCode(ack));
        assert_eq!(reply.status(), Status::Ok);

        let mut encoded = [0u8; PairingCode::ENCODED_LEN];
        let Some(count) = encode_bridge_control_ack(&reply, &mut encoded).ok() else {
            return;
        };
        assert_eq!(encoded[..count], buffer[..written]);
    }
}

/// The pairing-code layout rejects a wrong length field, truncation, an unassigned status and
/// invalid text, and encoding rejects a code that is not twelve characters.
#[test]
fn pairing_code_rejects_malformed_layouts() {
    assert_eq!(PairingCode::decode(&[]), Err(CodecError::Length));
    assert_eq!(PairingCode::decode(&[0]), Err(CodecError::Length));
    assert_eq!(
        PairingCode::decode(&[0, 12]),
        Err(CodecError::Length),
        "the declared code is missing"
    );

    let mut wrong_length = [b'A'; 13];
    wrong_length[0] = 0;
    wrong_length[1] = 11;
    assert_eq!(
        PairingCode::decode(&wrong_length),
        Err(CodecError::Length),
        "a pairing code is exactly twelve characters"
    );

    let mut bad_status = [b'A'; 14];
    bad_status[0] = 10;
    bad_status[1] = 12;
    assert_eq!(PairingCode::decode(&bad_status), Err(CodecError::Value));

    let mut bad_utf8 = [0xFFu8; 14];
    bad_utf8[0] = Status::Ok.bits();
    bad_utf8[1] = 12;
    assert_eq!(PairingCode::decode(&bad_utf8), Err(CodecError::Utf8));

    let short = PairingCode {
        status: Status::Ok,
        code: "short",
    };
    let mut buffer = [0u8; PairingCode::ENCODED_LEN];
    assert_eq!(short.encode_into(&mut buffer), Err(EncodeError::Value));

    let valid = PairingCode {
        status: Status::Ok,
        code: "ABCD2345EFGH",
    };
    let mut small = [0u8; PairingCode::ENCODED_LEN - 1];
    assert_eq!(valid.encode_into(&mut small), Err(EncodeError::BufferTooSmall));
}

/// The stored action selects the layout, so the same bytes decode differently under each action.
#[test]
fn the_action_selects_the_acknowledgement_layout() {
    let code = PairingCode {
        status: Status::Ok,
        code: "ABCD2345EFGH",
    };
    let mut code_bytes = [0u8; PairingCode::ENCODED_LEN];
    let Some(code_len) = code.encode_into(&mut code_bytes).ok() else {
        return;
    };
    assert_eq!(
        decode_bridge_control_ack(BridgeControlAction::ForgetNetwork, &code_bytes[..code_len]),
        Err(CodecError::Length),
        "action 0 expects the setup layout, not the pairing code"
    );

    let setup = SetupCredentials {
        status: Status::Ok,
        ssid: "",
        passphrase: "",
    };
    let mut setup_bytes = [0u8; SetupCredentials::MIN_ENCODED_LEN];
    let Some(setup_len) = setup.encode_into(&mut setup_bytes).ok() else {
        return;
    };
    assert_eq!(
        decode_bridge_control_ack(BridgeControlAction::ShowPairingCode, &setup_bytes[..setup_len]),
        Err(CodecError::Length),
        "actions 1 and 2 expect the pairing code, not the setup layout"
    );
}

/// Arbitrary bytes never panic either layout or the action dispatcher.
#[test]
fn bridge_control_codecs_never_panic_on_arbitrary_bytes() {
    for len in 0..=48 {
        let bytes = vec![0xFFu8; len];
        let _ = BridgeControl::decode(&bytes);
        let _ = SetupCredentials::decode(&bytes);
        let _ = PairingCode::decode(&bytes);
        for action in BridgeControlAction::ALL {
            let _ = decode_bridge_control_ack(action, &bytes);
        }
    }
}
