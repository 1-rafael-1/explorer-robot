//! Host tests for the `THROTTLE`-aware emission schedule (ADR-0030, ADR-0031, ADR-0041).
//!
//! The tests drive time explicitly: `publish_stream`, `on_rx` and `tick` set the core's notion of
//! `now`, and `next_frame` only emits a stream whose sample is due. The baseline in these tests is
//! 100 centi-Hz (a 1 s minimum spacing) with a 1 s heartbeat, so a 20 centi-Hz throttle (5 s) is
//! clearly visible.

use remote_link::{Config, LinkCore};
use remote_protocol::{
    Caps, Class, Control, Direction, Flags, Frame, MAX_PC_TO_ROBOT_FRAME, MAX_ROBOT_TO_PC_FRAME, PROTOCOL_MAJOR, Seq,
    Status, StreamSample, ThrottleAction, ThrottleHint, ThrottleState,
    codec::{PayloadCodec, decode_stream},
    ids::Stream as StreamId,
    payload::{
        control::{HelloAck, RateEntry, RateFlags},
        stream::{Motion, Power},
    },
    sim::{FakeRemote, Faults, Wire},
};

/// The Boot Id the tests hand to the link core.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// The baseline for these tests: 100 centi-Hz and a 1 s heartbeat on every stream.
fn config() -> Config {
    Config {
        robot_caps: Caps::THROTTLE_SUPPORT,
        default_rates_chz: [100; StreamId::COUNT],
        heartbeat_ms: [1_000; StreamId::COUNT],
        ..Config::default()
    }
}

/// Creates a core, wire and remote for one throttle path.
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

/// Emits one frame from the core, returning its bytes.
fn emit(core: &mut LinkCore) -> Option<Vec<u8>> {
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let written = core.next_frame(&mut buffer)?;
    Some(buffer[..written].to_vec())
}

/// Emits one frame and returns the stream it carried, or [`None`] when nothing was due.
fn emit_stream(core: &mut LinkCore) -> Option<StreamId> {
    let bytes = emit(core)?;
    let frame = Frame::decode(&bytes, Direction::RobotToPc).ok()?;
    if frame.header.class != Class::Stream {
        return None;
    }
    StreamId::from_wire(frame.header.id)
}

/// Sends a `HELLO` with `rates`, feeds it and consumes the `HELLO_ACK`.
fn establish_hello(core: &mut LinkCore, wire: &mut Wire, remote: &mut FakeRemote, rates: &[RateEntry]) {
    let Some(_) = remote.send_hello(wire, PROTOCOL_MAJOR, 0, Caps::from_bits(0), 1, rates) else {
        return;
    };
    feed_at(core, wire, 0);
    let Some(bytes) = emit(core) else {
        return;
    };
    let Some(frame) = Frame::decode(&bytes, Direction::RobotToPc).ok() else {
        return;
    };
    assert_eq!(frame.header.class, Class::Control, "the HELLO_ACK is emitted first");
}

/// Publishes one `POWER` sample at `now_ms`.
fn publish_power(core: &mut LinkCore, level: u8, now_ms: u64) {
    core.publish_stream(
        &StreamSample::Power(Power {
            level,
            voltage_mv: 3_000,
        }),
        0,
        now_ms,
    );
}

/// Publishes one `MOTION` sample at `now_ms`.
fn publish_motion(core: &mut LinkCore, left: i8, right: i8, now_ms: u64) {
    core.publish_stream(&StreamSample::Motion(Motion { left, right }), 0, now_ms);
}

/// Writes one raw control frame to the wire so a malformed payload can be tested.
fn send_raw_control(wire: &mut Wire, seq: u16, payload: &[u8]) -> bool {
    let Ok(frame) = Frame::new(
        PROTOCOL_MAJOR,
        Class::Control,
        Flags::default(),
        Control::Throttle.wire_id(),
        Seq::new(seq),
        payload,
    ) else {
        return false;
    };
    let mut buffer = [0u8; MAX_PC_TO_ROBOT_FRAME];
    let Ok(written) = frame.encode_into(&mut buffer) else {
        return false;
    };
    wire.send(Direction::PcToRobot, &buffer[..written]);
    true
}

/// A `SET_RATE` for one stream lowers only that stream's cadence; `0xFFFF` covers every stream.
#[test]
fn set_rate_applies_per_stream_and_for_all() {
    let (mut core, mut wire, mut remote) = harness();
    establish_hello(&mut core, &mut wire, &mut remote, &[]);
    let Some(_) = remote.send_throttle(
        &mut wire,
        StreamId::Power.wire_id(),
        ThrottleAction::SetRate,
        20,
        10_000,
    ) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);

    publish_power(&mut core, 1, 0);
    publish_motion(&mut core, 1, 1, 0);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power), "the first sample is due");
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Motion),
        "MOTION is not addressed by the POWER hint"
    );

    publish_power(&mut core, 2, 100);
    core.tick(100);
    assert!(emit_stream(&mut core).is_none(), "POWER now waits 5 s at 20 centi-Hz");
    core.tick(5_000);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "POWER is due 5 s after its previous send"
    );

    let (mut core, mut wire, mut remote) = harness();
    establish_hello(&mut core, &mut wire, &mut remote, &[]);
    let Some(_) = remote.send_throttle(
        &mut wire,
        ThrottleHint::ALL_STREAMS,
        ThrottleAction::SetRate,
        20,
        10_000,
    ) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    publish_power(&mut core, 1, 0);
    publish_motion(&mut core, 1, 1, 0);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power));
    assert_eq!(emit_stream(&mut core), Some(StreamId::Motion));

    publish_power(&mut core, 2, 100);
    publish_motion(&mut core, 2, 2, 100);
    core.tick(100);
    assert!(emit_stream(&mut core).is_none(), "0xFFFF throttles both streams");
    core.tick(5_000);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power));
    assert_eq!(emit_stream(&mut core), Some(StreamId::Motion));
}

/// `PAUSE` beats any rate, `RESUME` clears it, and a later `PAUSE` re-applies it.
#[test]
fn pause_beats_rate_and_resume_clears_it() {
    let (mut core, mut wire, mut remote) = harness();
    establish_hello(&mut core, &mut wire, &mut remote, &[]);
    let Some(_) = remote.send_throttle(
        &mut wire,
        StreamId::Power.wire_id(),
        ThrottleAction::SetRate,
        1_000,
        10_000,
    ) else {
        return;
    };
    let Some(_) = remote.send_throttle(&mut wire, StreamId::Power.wire_id(), ThrottleAction::Pause, 0, 10_000) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    assert_eq!(core.throttle_state(StreamId::Power), ThrottleState::Paused);
    assert_eq!(core.throttle_state(StreamId::Motion), ThrottleState::Unrestricted);
    assert_eq!(
        core.throttle_hints(),
        2,
        "the rate hint and the pause hint were applied"
    );

    publish_power(&mut core, 1, 0);
    core.tick(1_000);
    publish_power(&mut core, 2, 1_000);
    core.tick(1_000);
    assert!(emit_stream(&mut core).is_none(), "PAUSE suppresses the stream");

    let Some(_) = remote.send_throttle(&mut wire, StreamId::Power.wire_id(), ThrottleAction::Resume, 0, 0) else {
        return;
    };
    feed_at(&mut core, &mut wire, 1_000);
    assert_eq!(core.throttle_state(StreamId::Power), ThrottleState::Limited(1_000));
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "RESUME lets the pending change out"
    );

    let Some(_) = remote.send_throttle(&mut wire, ThrottleHint::ALL_STREAMS, ThrottleAction::Pause, 0, 10_000) else {
        return;
    };
    feed_at(&mut core, &mut wire, 2_000);
    publish_power(&mut core, 3, 2_000);
    core.tick(2_000);
    assert!(emit_stream(&mut core).is_none(), "a later global PAUSE re-applies");

    let Some(_) = remote.send_throttle(&mut wire, ThrottleHint::ALL_STREAMS, ThrottleAction::Resume, 0, 0) else {
        return;
    };
    feed_at(&mut core, &mut wire, 2_000);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power));
}

/// The Requested Rate and every active hint merge most-restrictively; the lower rate wins.
#[test]
fn the_most_restrictive_of_requested_rate_and_hints_wins() {
    let (mut core, mut wire, mut remote) = harness();
    let rates = [RateEntry::enabled(StreamId::Power.wire_id(), 200)];
    establish_hello(&mut core, &mut wire, &mut remote, &rates);
    let Some(_) = remote.send_throttle(
        &mut wire,
        StreamId::Power.wire_id(),
        ThrottleAction::SetRate,
        50,
        10_000,
    ) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);

    publish_power(&mut core, 1, 0);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power));

    // 50 centi-Hz is 2 s, so the hint wins over the 200 centi-Hz Requested Rate (500 ms).
    publish_power(&mut core, 2, 100);
    core.tick(100);
    assert!(emit_stream(&mut core).is_none(), "the lower hint rate gates emission");
    core.tick(2_000);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power));

    // A later, looser hint must not erase the still-live 50 centi-Hz hint.
    let Some(_) = remote.send_throttle(
        &mut wire,
        StreamId::Power.wire_id(),
        ThrottleAction::SetRate,
        5_000,
        10_000,
    ) else {
        return;
    };
    feed_at(&mut core, &mut wire, 2_000);
    publish_power(&mut core, 3, 2_000);
    core.tick(2_500);
    assert!(
        emit_stream(&mut core).is_none(),
        "the live 50 centi-Hz hint still wins over the later 5,000 centi-Hz hint"
    );
    core.tick(4_000);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "the change is due at the 50 centi-Hz (2 s) boundary"
    );

    // A hint above the Requested Rate must not raise the cadence.
    let (mut core, mut wire, mut remote) = harness();
    let rates = [RateEntry::enabled(StreamId::Power.wire_id(), 200)];
    establish_hello(&mut core, &mut wire, &mut remote, &rates);
    let Some(_) = remote.send_throttle(
        &mut wire,
        StreamId::Power.wire_id(),
        ThrottleAction::SetRate,
        5_000,
        10_000,
    ) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    publish_power(&mut core, 1, 0);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power));
    publish_power(&mut core, 2, 100);
    core.tick(100);
    assert!(
        emit_stream(&mut core).is_none(),
        "the 200 centi-Hz Requested Rate is still the ceiling"
    );
    core.tick(500);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "200 centi-Hz means a 500 ms spacing"
    );
}

/// A hint TTL expires back to the Requested Rate, never to zero.
#[test]
fn ttl_expiry_reverts_to_the_requested_rate() {
    let (mut core, mut wire, mut remote) = harness();
    let rates = [RateEntry::enabled(StreamId::Power.wire_id(), 200)];
    establish_hello(&mut core, &mut wire, &mut remote, &rates);
    let Some(_) = remote.send_throttle(&mut wire, StreamId::Power.wire_id(), ThrottleAction::SetRate, 20, 250) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);

    publish_power(&mut core, 1, 0);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power));

    publish_power(&mut core, 2, 50);
    core.tick(50);
    assert!(emit_stream(&mut core).is_none(), "the 5 s hint gates the change");

    core.tick(499);
    assert!(
        emit_stream(&mut core).is_none(),
        "the 200 centi-Hz rate is a 500 ms spacing"
    );
    core.tick(500);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "at the TTL the 200 centi-Hz Requested Rate resumes, not zero"
    );
}

/// With no `HELLO`, a hint TTL expires back to the configured baseline.
#[test]
fn ttl_expiry_reverts_to_the_configured_baseline() {
    let (mut core, mut wire, mut remote) = harness();
    let Some(_) = remote.send_throttle(&mut wire, StreamId::Power.wire_id(), ThrottleAction::SetRate, 20, 250) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);

    publish_power(&mut core, 1, 0);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power));

    publish_power(&mut core, 2, 50);
    core.tick(50);
    assert!(emit_stream(&mut core).is_none(), "the 5 s hint gates the change");

    core.tick(999);
    assert!(
        emit_stream(&mut core).is_none(),
        "the 100 centi-Hz baseline is a 1 s spacing"
    );
    core.tick(1_000);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "the 100 centi-Hz baseline resumes, not zero"
    );
}

/// An unknown stream id is ignored and a malformed payload is counted without panicking.
#[test]
fn unknown_streams_and_malformed_payloads_are_ignored() {
    let (mut core, mut wire, mut remote) = harness();
    establish_hello(&mut core, &mut wire, &mut remote, &[]);

    let Some(_) = remote.send_throttle(&mut wire, 0x0099, ThrottleAction::SetRate, 1, 10_000) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    assert_eq!(
        core.throttle_hints(),
        0,
        "a hint for an unknown stream is not an applied hint"
    );
    publish_power(&mut core, 1, 0);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power));
    publish_power(&mut core, 2, 100);
    core.tick(100);
    assert!(
        emit_stream(&mut core).is_none(),
        "the 1 s baseline spacing still applies"
    );
    core.tick(1_000);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "the unknown-stream hint leaves the baseline intact"
    );

    assert!(send_raw_control(&mut wire, 50, &[0u8; 6]), "short payload");
    feed_at(&mut core, &mut wire, 1_100);
    assert_eq!(core.invalid_throttles(), 1, "a length error is counted");

    let mut bad_action = [0u8; 7];
    bad_action[2] = 9;
    assert!(send_raw_control(&mut wire, 51, &bad_action), "unknown action");
    feed_at(&mut core, &mut wire, 1_200);
    assert_eq!(core.invalid_throttles(), 2, "an action error is counted");

    publish_power(&mut core, 3, 1_200);
    core.tick(1_999);
    assert!(emit_stream(&mut core).is_none(), "the change waits for the 1 s spacing");
    core.tick(2_000);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "malformed hints never disturb the schedule"
    );
}

/// `ENABLED=0` suppresses a stream; `ENABLED=1` with `rate=0` is `BAD_ARGUMENT`.
#[test]
fn disabled_streams_are_suppressed_and_zero_rate_is_bad_argument() {
    let (mut core, mut wire, mut remote) = harness();
    let rates = [
        RateEntry {
            stream_id: StreamId::Power.wire_id(),
            rate_chz: 100,
            flags: RateFlags::from_bits(0),
        },
        RateEntry::enabled(StreamId::Motion.wire_id(), 100),
    ];
    establish_hello(&mut core, &mut wire, &mut remote, &rates);

    publish_power(&mut core, 1, 0);
    publish_motion(&mut core, 1, 1, 0);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Motion),
        "the disabled POWER stream is suppressed"
    );
    assert!(emit_stream(&mut core).is_none(), "nothing else is due");

    let (mut core, mut wire, mut remote) = harness();
    let bad = [RateEntry::enabled(StreamId::Power.wire_id(), 0)];
    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR, 0, Caps::from_bits(0), 1, &bad) else {
        return;
    };
    feed_at(&mut core, &mut wire, 0);
    let Some(bytes) = emit(&mut core) else {
        return;
    };
    let Some(frame) = Frame::decode(&bytes, Direction::RobotToPc).ok() else {
        return;
    };
    let Some(ack) = HelloAck::decode(frame.payload).ok() else {
        return;
    };
    assert_eq!(ack.status, Status::BadArgument);
    assert_eq!(
        core.requested_rate(StreamId::Power),
        None,
        "the bad HELLO is not applied"
    );
}

/// A changed sample respects the effective rate; an unchanged sample waits for its heartbeat.
#[test]
fn changed_samples_respect_the_rate_and_unchanged_samples_wait_for_the_heartbeat() {
    let (mut core, mut wire, mut remote) = harness();
    establish_hello(&mut core, &mut wire, &mut remote, &[]);

    publish_power(&mut core, 1, 0);
    assert_eq!(emit_stream(&mut core), Some(StreamId::Power), "the first sample is due");

    publish_power(&mut core, 1, 100);
    core.tick(100);
    assert!(
        emit_stream(&mut core).is_none(),
        "an unchanged sample is not resent before its heartbeat"
    );
    core.tick(999);
    assert!(emit_stream(&mut core).is_none());
    core.tick(1_000);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "the heartbeat resends an unchanged sample"
    );

    publish_power(&mut core, 2, 1_100);
    core.tick(1_100);
    assert!(
        emit_stream(&mut core).is_none(),
        "the 1 s spacing has not elapsed since the heartbeat"
    );
    core.tick(1_999);
    assert!(emit_stream(&mut core).is_none());
    core.tick(2_000);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "a changed sample is sent at the 1 s rate boundary"
    );

    publish_power(&mut core, 3, 2_050);
    core.tick(2_050);
    assert!(emit_stream(&mut core).is_none(), "50 ms is below the 1 s spacing");
    core.tick(2_999);
    assert!(emit_stream(&mut core).is_none());
    core.tick(3_000);
    assert_eq!(
        emit_stream(&mut core),
        Some(StreamId::Power),
        "the change is sent at the next rate boundary"
    );
}

/// Latest-wins coalescing still holds: the newest sample is the one emitted.
#[test]
fn latest_wins_coalescing_still_holds() {
    let (mut core, mut wire, mut remote) = harness();
    establish_hello(&mut core, &mut wire, &mut remote, &[]);

    publish_power(&mut core, 10, 0);
    publish_power(&mut core, 20, 0);
    let Some(bytes) = emit(&mut core) else {
        return;
    };
    let Some(frame) = Frame::decode(&bytes, Direction::RobotToPc).ok() else {
        return;
    };
    let Some(sample) = decode_stream(StreamId::Power, frame.payload).ok() else {
        return;
    };
    assert_eq!(
        sample,
        StreamSample::Power(Power {
            level: 20,
            voltage_mv: 3_000,
        }),
        "the newer sample replaces the pending one"
    );
    assert!(
        emit_stream(&mut core).is_none(),
        "the coalesced sample is not retransmitted"
    );
}
