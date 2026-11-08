//! Host tests for the robot's typed telemetry path.
//!
//! Everything goes through the real contract: `LinkCore` encodes a sample, `sim::Wire` carries the
//! bytes and `sim::FakeRemote` parses them with the committed parser and codecs.

use link_core::{Config, LinkCore, STREAM_STAGING_BUDGET_BYTES, STREAM_STAGING_BYTES};
use link_protocol::{
    Class, Direction, Flags, Frame, MAX_ROBOT_TO_PC_FRAME, PROTOCOL_MAJOR, Seq, StreamSample,
    codec::decode_stream,
    ids::{ProcedureId, Stream},
    payload::stream::{Activity, ActivityStage, Calibration, CalibrationStatus, LidarCloud, Motion, Perception, Power},
    sim::{FakeRemote, Faults, Wire},
};

/// The Boot Id the tests hand to the link core.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// Creates a core, wire and remote for one stream path.
fn harness() -> (LinkCore, Wire, FakeRemote) {
    (
        LinkCore::new(Config::default(), BOOT_ID, 0),
        Wire::new(1, Faults::default()),
        FakeRemote::new(),
    )
}

/// Publishes `sample`, emits one frame and returns the encoded bytes.
fn emit(core: &mut LinkCore, sample: &StreamSample<'_>) -> Option<Vec<u8>> {
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    core.publish_stream(sample, 0, 0);
    let written = core.next_frame(&mut buffer)?;
    Some(buffer[..written].to_vec())
}

/// Publishes `sample` at `now_ms`, emits one frame and returns its sequence number.
fn emit_sequence(core: &mut LinkCore, sample: &StreamSample<'_>, now_ms: u64) -> Option<u16> {
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    core.publish_stream(sample, 0, now_ms);
    let written = core.next_frame(&mut buffer)?;
    let frame = Frame::decode(&buffer[..written], Direction::RobotToPc).ok()?;
    Some(frame.header.seq.get())
}

/// Encodes one standalone frame for the inbound (`on_rx`) tests.
fn frame_bytes(id: u16, class: Class, payload: &[u8]) -> Option<Vec<u8>> {
    let frame = Frame::new(PROTOCOL_MAJOR, class, Flags::default(), id, Seq::new(0), payload).ok()?;
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let written = frame.encode_into(&mut buffer).ok()?;
    Some(buffer[..written].to_vec())
}

/// A `LiDAR` sample with a representative no-return slot.
const fn cloud() -> LidarCloud {
    let mut slots = [7u16; 360];
    slots[0] = 0;
    slots[179] = LidarCloud::NO_RETURN;
    LidarCloud { slots }
}

/// Every v1 stream round-trips through `LinkCore` -> `Wire` -> parser -> `FakeRemote`.
#[test]
fn every_v1_stream_round_trips_through_the_link() {
    let samples = [
        StreamSample::Power(Power {
            level: 0xFF,
            voltage_mv: 0xFFFF,
        }),
        StreamSample::Motion(Motion { left: -12, right: 34 }),
        StreamSample::Activity(Activity {
            procedure_id: ProcedureId::MOTOR_CALIBRATION,
            stage: ActivityStage::Complete,
            percent: None,
            detail: "calibrated",
        }),
        StreamSample::Perception(Perception::OBSTACLE | Perception::FLOOR_DROP),
        StreamSample::LidarCloud(cloud()),
        StreamSample::Calibration(Calibration {
            motor: CalibrationStatus::Loaded,
            imu: CalibrationStatus::Unknown,
            mag: CalibrationStatus::Missing,
            distance: CalibrationStatus::Loaded,
        }),
    ];

    for sample in samples {
        let (mut core, mut wire, mut remote) = harness();
        let Some(bytes) = emit(&mut core, &sample) else {
            return;
        };
        wire.send(Direction::RobotToPc, &bytes);

        assert_eq!(
            remote.receive(&mut wire),
            1,
            "{:?} must produce one frame",
            sample.stream()
        );
        assert_eq!(remote.unknown_streams(), 0, "no id may be unknown");
        assert_eq!(
            remote.last_sample(sample.stream()),
            Some(sample),
            "{:?} must decode to the published sample",
            sample.stream()
        );
    }
}

/// A newer sample replaces the pending one: nothing is queued and nothing is retransmitted.
#[test]
fn a_newer_sample_replaces_the_pending_one() {
    let (mut core, _wire, _remote) = harness();
    core.publish_stream(
        &StreamSample::Power(Power {
            level: 10,
            voltage_mv: 3_000,
        }),
        0,
        0,
    );
    core.publish_stream(
        &StreamSample::Power(Power {
            level: 20,
            voltage_mv: 3_100,
        }),
        0,
        10,
    );

    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let Some(written) = core.next_frame(&mut buffer) else {
        return;
    };
    let Ok(frame) = Frame::decode(&buffer[..written], Direction::RobotToPc) else {
        return;
    };
    let Ok(decoded) = decode_stream(Stream::Power, frame.payload) else {
        return;
    };
    assert_eq!(
        decoded,
        StreamSample::Power(Power {
            level: 20,
            voltage_mv: 3_100,
        }),
        "the newer sample must be the one emitted"
    );

    assert!(
        core.next_frame(&mut buffer).is_none(),
        "the coalesced sample must not be retransmitted"
    );
}

/// Each stream has its own sequence counter, incremented on every emission.
#[test]
fn per_stream_sequences_increment_on_emission() {
    let (mut core, _wire, _remote) = harness();
    let power = StreamSample::Power(Power {
        level: 50,
        voltage_mv: 4_000,
    });
    let motion = StreamSample::Motion(Motion { left: 1, right: 1 });

    // An unchanged sample waits for its heartbeat (5 s for POWER), so advance time between sends.
    assert_eq!(emit_sequence(&mut core, &power, 0), Some(0), "first POWER");
    assert_eq!(emit_sequence(&mut core, &power, 5_000), Some(1), "second POWER");
    assert_eq!(emit_sequence(&mut core, &power, 10_000), Some(2), "third POWER");
    assert_eq!(
        emit_sequence(&mut core, &motion, 10_000),
        Some(0),
        "MOTION must have its own counter"
    );
}

/// Pending streams are emitted round-robin rather than starving a later slot.
#[test]
fn pending_streams_are_emitted_round_robin() {
    let (mut core, _wire, _remote) = harness();
    let power = StreamSample::Power(Power {
        level: 1,
        voltage_mv: 1,
    });
    let motion = StreamSample::Motion(Motion { left: 2, right: 3 });
    core.publish_stream(&power, 0, 0);
    core.publish_stream(&motion, 0, 0);

    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let first = {
        let Some(written) = core.next_frame(&mut buffer) else {
            return;
        };
        let Ok(frame) = Frame::decode(&buffer[..written], Direction::RobotToPc) else {
            return;
        };
        frame.header.id
    };
    let second = {
        let Some(written) = core.next_frame(&mut buffer) else {
            return;
        };
        let Ok(frame) = Frame::decode(&buffer[..written], Direction::RobotToPc) else {
            return;
        };
        frame.header.id
    };

    assert_eq!(first, Stream::Power.wire_id(), "the first slot comes first");
    assert_eq!(second, Stream::Motion.wire_id(), "then the next slot");
}

/// Unknown inbound stream ids are ignored and counted, never a panic.
#[test]
fn unknown_inbound_stream_ids_are_ignored_and_counted() {
    let (mut core, _wire, _remote) = harness();
    assert_eq!(core.unknown_streams(), 0, "a fresh core has no unknown ids");

    let Some(unknown) = frame_bytes(0x00FF, Class::Stream, &[1, 2]) else {
        return;
    };
    core.on_rx(&unknown, 0);
    assert_eq!(core.unknown_streams(), 1, "an unassigned id is counted");

    let Some(known) = frame_bytes(Stream::Power.wire_id(), Class::Stream, &[50, 0, 0]) else {
        return;
    };
    core.on_rx(&known, 0);
    assert_eq!(core.unknown_streams(), 1, "a known stream id must not be counted");

    let Some(reserved) = frame_bytes(0x0010, Class::Stream, &[]) else {
        return;
    };
    core.on_rx(&reserved, 0);
    assert_eq!(core.unknown_streams(), 2, "a reserved id is counted");

    let Some(other_class) = frame_bytes(0x00FF, Class::Command, &[]) else {
        return;
    };
    core.on_rx(&other_class, 0);
    core.on_rx(&[0xA5, 0x5A, 0x00], 0);
    core.on_rx(&[], 100);
    assert_eq!(
        core.unknown_streams(),
        2,
        "other classes and garbage must not change the count"
    );
}

/// The six v1 slots fit the staging budget with the `LiDAR` cloud dominating.
#[test]
fn stream_staging_fits_the_adr_budget() {
    assert_eq!(
        STREAM_STAGING_BYTES, 767,
        "the v1 payloads plus the activity detail sum to 767 bytes"
    );
    const {
        assert!(
            STREAM_STAGING_BYTES <= STREAM_STAGING_BUDGET_BYTES,
            "the slots must fit ADR-0030's staging budget"
        );
    };
    const {
        assert!(STREAM_STAGING_BYTES < 1_100, "the slots must stay well below 1.1 KB");
    };
}

/// A long ACTIVITY detail is truncated at a character boundary before it reaches the wire.
#[test]
fn activity_detail_truncates_at_a_utf8_boundary_end_to_end() {
    let detail = format!("{}{}", "b".repeat(31), "é");
    let sample = StreamSample::Activity(Activity {
        procedure_id: ProcedureId::STRAIGHT_DRIVE,
        stage: ActivityStage::Running,
        percent: Some(7),
        detail: &detail,
    });

    let (mut core, mut wire, mut remote) = harness();
    let Some(bytes) = emit(&mut core, &sample) else {
        return;
    };
    wire.send(Direction::RobotToPc, &bytes);
    remote.receive(&mut wire);

    let Some(StreamSample::Activity(decoded)) = remote.last_sample(Stream::Activity) else {
        return;
    };
    assert_eq!(
        decoded.detail,
        "b".repeat(31).as_str(),
        "the detail must be cut at a character boundary"
    );
}

/// A link with no traffic stays detached and has no Deadman edge.
#[test]
fn peer_state_is_no_peer_until_a_frame_arrives() {
    let (mut core, _wire, _remote) = harness();
    assert_eq!(core.peer(), link_core::PeerState::NoPeer, "no peer yet");
    assert!(!core.take_peer_lost(), "silence without an attach does not fire");
    assert_eq!(core.boot_id(), BOOT_ID, "the Boot Id is retained");
    assert_eq!(core.config(), Config::default(), "the config is retained");
}
