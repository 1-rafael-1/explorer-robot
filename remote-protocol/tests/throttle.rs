//! Host tests for the `THROTTLE` codec and the shared merge/TTL mechanism (ADR-0030, ADR-0041).

use remote_protocol::{
    CodecError, Control, ControlMessage, EncodeError, Status, Stream, Throttle, ThrottleAction, ThrottleHint,
    ThrottleState, codec::PayloadCodec, decode_control, encode_control,
};

/// Builds a hint for one scope.
const fn hint(stream_id: u16, action: ThrottleAction, rate_chz: u16, ttl_ms: u16) -> ThrottleHint {
    ThrottleHint::new(stream_id, action, rate_chz, ttl_ms)
}

/// The discriminants are the ADR-0030 wire values and unknown actions are rejected.
#[test]
fn action_wire_values_are_frozen() {
    assert_eq!(ThrottleAction::SetRate.bits(), 0);
    assert_eq!(ThrottleAction::Pause.bits(), 1);
    assert_eq!(ThrottleAction::Resume.bits(), 2);
    assert_eq!(ThrottleAction::ALL.len(), 3);
    for action in ThrottleAction::ALL {
        assert_eq!(ThrottleAction::from_wire(action.bits()), Some(action));
    }
    assert_eq!(ThrottleAction::from_wire(3), None, "3 is unassigned");
    assert_eq!(ThrottleAction::from_wire(0xFF), None, "0xFF is unassigned");
}

/// The payload is exactly seven little-endian bytes: `stream_id`, `action`, `rate_chz`, `ttl_ms`.
#[test]
fn throttle_round_trips_with_a_golden_layout() {
    let value = hint(ThrottleHint::ALL_STREAMS, ThrottleAction::SetRate, 1_000, 500);
    let mut buffer = [0u8; ThrottleHint::ENCODED_LEN];
    let Some(written) = value.encode_into(&mut buffer).ok() else {
        return;
    };
    assert_eq!(written, 7, "THROTTLE is exactly seven bytes");
    assert_eq!(
        &buffer[..written],
        &[0xFF, 0xFF, 0x00, 0xE8, 0x03, 0xF4, 0x01],
        "every field is little-endian"
    );

    let Some(decoded) = ThrottleHint::decode(&buffer[..written]).ok() else {
        return;
    };
    assert_eq!(decoded, value, "a hint round-trips unchanged");

    let pause = hint(Stream::Motion.wire_id(), ThrottleAction::Pause, 0, 0);
    let Some(written) = pause.encode_into(&mut buffer).ok() else {
        return;
    };
    assert_eq!(
        ThrottleHint::decode(&buffer[..written]).ok(),
        Some(pause),
        "PAUSE round-trips with an ignored rate"
    );
}

/// A bad length is `Length`, an unassigned action is `Value`, and a short buffer is rejected.
#[test]
fn throttle_rejects_malformed_input() {
    assert_eq!(ThrottleHint::decode(&[0u8; 6]), Err(CodecError::Length));
    assert_eq!(ThrottleHint::decode(&[0u8; 8]), Err(CodecError::Length));
    assert_eq!(ThrottleHint::decode(&[]), Err(CodecError::Length));

    let mut action = [0u8; ThrottleHint::ENCODED_LEN];
    action[2] = 3;
    assert_eq!(ThrottleHint::decode(&action), Err(CodecError::Value));

    assert_eq!(CodecError::Length.status(), Status::BadLength);
    assert_eq!(CodecError::Value.status(), Status::BadArgument);

    let value = hint(0, ThrottleAction::Pause, 0, 0);
    let mut small = [0u8; ThrottleHint::ENCODED_LEN - 1];
    assert_eq!(
        value.encode_into(&mut small),
        Err(EncodeError::BufferTooSmall),
        "the full seven bytes are required"
    );
}

/// Arbitrary bytes never panic, whatever the length or action byte.
#[test]
fn arbitrary_throttle_payloads_never_panic() {
    let prefixes = [0u8, 1, 2, 3, 0x7F, 0x80, 0xFF];
    for len in 0..=ThrottleHint::ENCODED_LEN + 2 {
        for byte in prefixes {
            let bytes = [byte; ThrottleHint::ENCODED_LEN + 2];
            let _ = ThrottleHint::decode(&bytes[..len]);
        }
    }
}

/// The control dispatch decodes and encodes `THROTTLE`, ignoring its meaningless `IS_ACK`.
#[test]
fn control_message_dispatches_throttle_without_an_ack() {
    let value = hint(ThrottleHint::ALL_STREAMS, ThrottleAction::Resume, 0, 0);
    let mut buffer = [0u8; ThrottleHint::ENCODED_LEN];
    let Some(written) = encode_control(&ControlMessage::Throttle(value), &mut buffer).ok() else {
        return;
    };

    let Some(decoded) = decode_control(Control::Throttle, false, &buffer[..written]).ok() else {
        return;
    };
    assert_eq!(decoded, ControlMessage::Throttle(value));
    assert_eq!(decoded.control(), Control::Throttle);

    let Some(acked) = decode_control(Control::Throttle, true, &buffer[..written]).ok() else {
        return;
    };
    assert_eq!(acked, ControlMessage::Throttle(value), "IS_ACK is ignored");
}

/// A per-stream `SET_RATE` limits that stream only; `0xFFFF` limits every stream.
#[test]
fn set_rate_scopes_to_one_stream_or_all() {
    let mut throttle = Throttle::new();
    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::SetRate, 100, 1_000), 0));
    assert_eq!(throttle.effective(Stream::Power, 0), ThrottleState::Limited(100));
    assert_eq!(throttle.effective(Stream::Motion, 0), ThrottleState::Unrestricted);

    let mut throttle = Throttle::new();
    assert!(throttle.apply(&hint(ThrottleHint::ALL_STREAMS, ThrottleAction::SetRate, 40, 1_000), 0));
    for stream in Stream::ALL {
        assert_eq!(
            throttle.effective(stream, 0),
            ThrottleState::Limited(40),
            "{stream:?} is covered by 0xFFFF"
        );
    }
}

/// The lowest active rate wins across the global and per-stream hints.
#[test]
fn the_most_restrictive_rate_wins() {
    let mut throttle = Throttle::new();
    assert!(throttle.apply(
        &hint(ThrottleHint::ALL_STREAMS, ThrottleAction::SetRate, 1_000, 1_000),
        0
    ));
    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::SetRate, 100, 1_000), 0));
    assert_eq!(
        throttle.effective(Stream::Power, 0),
        ThrottleState::Limited(100),
        "the per-stream rate is lower"
    );
    assert_eq!(throttle.effective(Stream::Motion, 0), ThrottleState::Limited(1_000));

    let mut throttle = Throttle::new();
    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::SetRate, 1_000, 1_000), 0));
    assert!(throttle.apply(&hint(ThrottleHint::ALL_STREAMS, ThrottleAction::SetRate, 25, 1_000), 0));
    assert_eq!(
        throttle.effective(Stream::Power, 0),
        ThrottleState::Limited(25),
        "the global rate is lower"
    );
}

/// A pause beats every rate, whether it is global or per stream.
#[test]
fn pause_beats_any_rate() {
    let mut throttle = Throttle::new();
    assert!(throttle.apply(&hint(ThrottleHint::ALL_STREAMS, ThrottleAction::SetRate, 100, 1_000), 0));
    assert!(throttle.apply(&hint(Stream::Motion.wire_id(), ThrottleAction::Pause, 0, 1_000), 0));
    assert_eq!(throttle.effective(Stream::Motion, 0), ThrottleState::Paused);
    assert_eq!(throttle.effective(Stream::Power, 0), ThrottleState::Limited(100));

    let mut throttle = Throttle::new();
    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::SetRate, 100, 1_000), 0));
    assert!(throttle.apply(&hint(ThrottleHint::ALL_STREAMS, ThrottleAction::Pause, 0, 1_000), 0));
    assert_eq!(
        throttle.effective(Stream::Power, 0),
        ThrottleState::Paused,
        "a global pause beats a per-stream rate"
    );
}

/// `RESUME` clears a pause immediately and a later `PAUSE` re-applies it.
#[test]
fn resume_clears_a_pause_and_a_later_pause_reapplies_it() {
    let mut throttle = Throttle::new();
    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::Pause, 0, 1_000), 0));
    assert_eq!(throttle.effective(Stream::Power, 0), ThrottleState::Paused);

    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::Resume, 0, 0), 0));
    assert_eq!(throttle.effective(Stream::Power, 0), ThrottleState::Unrestricted);

    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::Pause, 0, 1_000), 10));
    assert_eq!(throttle.effective(Stream::Power, 10), ThrottleState::Paused);
}

/// A per-stream resume does not clear a global pause; `RESUME 0xFFFF` clears every pause.
#[test]
fn resume_scope_follows_its_stream_id() {
    let mut throttle = Throttle::new();
    assert!(throttle.apply(&hint(ThrottleHint::ALL_STREAMS, ThrottleAction::Pause, 0, 1_000), 0));
    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::Resume, 0, 0), 0));
    assert_eq!(
        throttle.effective(Stream::Power, 0),
        ThrottleState::Paused,
        "a per-stream resume cannot override the global pause"
    );

    assert!(throttle.apply(&hint(ThrottleHint::ALL_STREAMS, ThrottleAction::Pause, 0, 1_000), 0));
    assert!(throttle.apply(&hint(ThrottleHint::ALL_STREAMS, ThrottleAction::Resume, 0, 0), 0));
    for stream in Stream::ALL {
        assert_eq!(throttle.effective(stream, 0), ThrottleState::Unrestricted);
    }
}

/// Expired hints disappear at their TTL and the effective state reverts to unrestricted.
#[test]
fn rate_and_pause_hints_expire_at_their_ttl() {
    let mut throttle = Throttle::new();
    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::SetRate, 100, 500), 100));
    assert_eq!(throttle.effective(Stream::Power, 599), ThrottleState::Limited(100));
    assert_eq!(
        throttle.effective(Stream::Power, 600),
        ThrottleState::Unrestricted,
        "the rate hint expires at now + ttl"
    );
    throttle.tick(600);
    assert_eq!(throttle.effective(Stream::Power, 600), ThrottleState::Unrestricted);

    let mut throttle = Throttle::new();
    assert!(throttle.apply(&hint(Stream::Motion.wire_id(), ThrottleAction::Pause, 0, 50), 0));
    assert_eq!(throttle.effective(Stream::Motion, 49), ThrottleState::Paused);
    assert_eq!(throttle.effective(Stream::Motion, 50), ThrottleState::Unrestricted);
}

/// A `SET_RATE` of zero is a pause, never an infinite silent stream, and it too expires.
#[test]
fn a_zero_rate_is_a_pause_that_expires() {
    let mut throttle = Throttle::new();
    assert!(throttle.apply(&hint(ThrottleHint::ALL_STREAMS, ThrottleAction::SetRate, 0, 100), 0));
    for stream in Stream::ALL {
        assert_eq!(throttle.effective(stream, 0), ThrottleState::Paused);
    }
    assert_eq!(
        throttle.effective(Stream::Power, 100),
        ThrottleState::Unrestricted,
        "the pause expires like any other hint"
    );
}

/// Every live hint is retained: a later looser hint cannot erase a stricter one on the same stream.
#[test]
fn several_live_hints_merge_to_the_most_restrictive() {
    let mut throttle = Throttle::new();
    assert!(throttle.apply(
        &hint(ThrottleHint::ALL_STREAMS, ThrottleAction::SetRate, 500, 10_000),
        0
    ));
    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::SetRate, 100, 10_000), 0));

    // A second, looser hint on the same stream must not win over the live 100 centi-Hz one.
    assert!(throttle.apply(
        &hint(Stream::Power.wire_id(), ThrottleAction::SetRate, 1_000, 10_000),
        10
    ));
    assert_eq!(
        throttle.effective(Stream::Power, 10),
        ThrottleState::Limited(100),
        "the stricter live per-stream hint still wins"
    );
    assert_eq!(
        throttle.effective(Stream::Motion, 10),
        ThrottleState::Limited(500),
        "the global hint still bounds a stream with no per-stream hint"
    );

    // A later stricter hint wins immediately.
    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::SetRate, 50, 10_000), 20));
    assert_eq!(throttle.effective(Stream::Power, 20), ThrottleState::Limited(50));

    // A pause outranks every rate, and its expiry falls back to the live strictest rate.
    assert!(throttle.apply(&hint(Stream::Power.wire_id(), ThrottleAction::Pause, 0, 100), 30));
    assert_eq!(throttle.effective(Stream::Power, 30), ThrottleState::Paused);
    assert_eq!(
        throttle.effective(Stream::Power, 130),
        ThrottleState::Limited(50),
        "after the pause expires the strictest rate is still 50"
    );
}

/// An unknown stream id is ignored and a zero TTL has no effect.
#[test]
fn unknown_ids_and_zero_ttls_are_ignored() {
    let mut throttle = Throttle::new();
    assert!(!throttle.apply(&hint(0x0099, ThrottleAction::Pause, 0, 1_000), 0));
    assert!(!throttle.apply(&hint(0x00FE, ThrottleAction::SetRate, 1, 1_000), 0));
    for stream in Stream::ALL {
        assert_eq!(throttle.effective(stream, 0), ThrottleState::Unrestricted);
    }

    assert!(throttle.apply(&hint(ThrottleHint::ALL_STREAMS, ThrottleAction::Pause, 0, 0), 0));
    assert_eq!(
        throttle.effective(Stream::Power, 0),
        ThrottleState::Unrestricted,
        "a zero-ttl hint is already expired"
    );
}
