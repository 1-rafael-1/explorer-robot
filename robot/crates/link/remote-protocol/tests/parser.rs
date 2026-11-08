//! Host tests for the resynchronising parser and the deterministic wire.
//!
//! The parser tests exercise the external contract: an arbitrary byte stream in, complete
//! frames out. The wire tests prove that a fixed seed replays the same faults in both
//! directions and that a parser recovers from faulted traffic.

use remote_protocol::{
    Class, Direction, Flags, Frame, PROTOCOL_MAJOR, Seq,
    parser::{Parser, PcToRobotParser, RobotToPcParser},
    sim::{Faults, Wire},
};

/// The ADR-0033 baud; both directions derive a 50 ms timeout from it.
const BAUD: u32 = 921_600;

/// Encodes a stream frame with `id` and `payload` into a fresh byte vector.
fn encoded(id: u16, payload: &[u8]) -> Option<Vec<u8>> {
    let frame = Frame::new(
        PROTOCOL_MAJOR,
        Class::Stream,
        Flags::default(),
        id,
        Seq::new(1),
        payload,
    )
    .ok()?;
    let mut buffer = vec![0u8; frame.encoded_len()];
    let written = frame.encode_into(&mut buffer).ok()?;
    buffer.truncate(written);
    Some(buffer)
}

/// A robot-to-PC parser at the ADR-0033 baud.
fn robot_parser() -> RobotToPcParser {
    RobotToPcParser::new(Direction::RobotToPc, BAUD)
}

/// Feeds `bytes` one at a time and collects each completed frame's id and payload.
fn collect<const CAP: usize>(parser: &mut Parser<CAP>, bytes: &[u8], now_ms: u32) -> Vec<(u16, Vec<u8>)> {
    let mut frames = Vec::new();
    for &byte in bytes {
        if let Some(frame) = parser.push(byte, now_ms) {
            frames.push((frame.header.id, frame.payload.to_vec()));
        }
    }
    frames
}

/// Generates a deterministic noise corpus with a small xorshift.
fn garbage(seed: u32, len: usize) -> Vec<u8> {
    let mut state = seed | 1;
    let mut bytes = Vec::with_capacity(len);
    for _ in 0..len {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        bytes.push(state.to_le_bytes()[0]);
    }
    bytes
}

/// Runs a fixed traffic pattern through a wire and returns both directions plus the clock.
fn wire_run(seed: u32) -> (Vec<u8>, Vec<u8>, u32) {
    let Some(first) = encoded(1, b"robot to pc") else {
        return (Vec::new(), Vec::new(), 0);
    };
    let Some(second) = encoded(2, b"pc to robot") else {
        return (Vec::new(), Vec::new(), 0);
    };
    let faults = Faults {
        drop_every: 7,
        corrupt_every: 5,
        truncate_every: 3,
        stall_ms: 2,
    };
    let mut wire = Wire::new(seed, faults);
    for _ in 0..3 {
        wire.send(Direction::RobotToPc, &first);
        wire.send(Direction::PcToRobot, &second);
    }
    wire.advance(3);
    let to_pc = wire.take(Direction::RobotToPc);
    let to_robot = wire.take(Direction::PcToRobot);
    (to_pc, to_robot, wire.now_ms())
}

/// Leading garbage is skipped and the following frame is yielded.
#[test]
fn leading_garbage_is_skipped() {
    let Some(frame) = encoded(0x0101, &[1, 2, 3]) else {
        return;
    };
    let mut bytes = vec![0x00, 0xFF, 0x5A, 0xA5, 0x00, 0x11];
    bytes.extend_from_slice(&frame);

    let mut parser = robot_parser();
    assert_eq!(
        collect(&mut parser, &bytes, 0),
        vec![(0x0101, vec![1, 2, 3])],
        "the frame after the garbage must be yielded"
    );
}

/// A magic pair inside a payload is not a frame boundary.
#[test]
fn magic_inside_a_payload_is_not_a_boundary() {
    let Some(first) = encoded(1, &[0xA5, 0x5A, 0x11, 0x22]) else {
        return;
    };
    let Some(second) = encoded(2, &[0x33]) else {
        return;
    };
    let mut bytes = first;
    bytes.extend_from_slice(&second);

    let mut parser = robot_parser();
    assert_eq!(
        collect(&mut parser, &bytes, 0),
        vec![(1, vec![0xA5, 0x5A, 0x11, 0x22]), (2, vec![0x33])],
        "the payload must be collected verbatim"
    );
}

/// An inserted byte breaks one frame but the next frame is still recovered.
#[test]
fn an_inserted_byte_is_recovered_from() {
    let Some(first) = encoded(1, &[0x10, 0x20, 0x30, 0x40]) else {
        return;
    };
    let Some(second) = encoded(2, &[0x50, 0x60]) else {
        return;
    };
    let mut bytes = first;
    bytes.insert(8, 0xFF);
    bytes.extend_from_slice(&second);

    let mut parser = robot_parser();
    assert_eq!(
        collect(&mut parser, &bytes, 0),
        vec![(2, vec![0x50, 0x60])],
        "the corrupted frame must be discarded and the next one found"
    );
}

/// A deleted byte breaks one frame but the next frame is still recovered.
#[test]
fn a_deleted_byte_is_recovered_from() {
    let Some(first) = encoded(1, &[0x10, 0x20, 0x30, 0x40]) else {
        return;
    };
    let Some(second) = encoded(2, &[0x50, 0x60]) else {
        return;
    };
    let mut bytes = first;
    bytes.remove(13);
    bytes.extend_from_slice(&second);

    let mut parser = robot_parser();
    assert_eq!(
        collect(&mut parser, &bytes, 0),
        vec![(2, vec![0x50, 0x60])],
        "the truncated frame must be discarded and the next one found"
    );
}

/// A truncated candidate followed by a valid frame yields the valid frame.
#[test]
fn a_truncated_candidate_does_not_hide_the_next_frame() {
    let Some(first) = encoded(1, &[0x10, 0x20, 0x30, 0x40]) else {
        return;
    };
    let Some(second) = encoded(2, &[0x50, 0x60]) else {
        return;
    };
    let mut bytes = first[..14].to_vec();
    bytes.extend_from_slice(&second);

    let mut parser = robot_parser();
    assert_eq!(
        collect(&mut parser, &bytes, 0),
        vec![(2, vec![0x50, 0x60])],
        "the trailing valid frame must be recovered"
    );
}

/// A stall past the derived timeout resets the parser instead of wedging it.
#[test]
fn a_stall_past_the_timeout_does_not_wedge_the_parser() {
    let Some(frame) = encoded(3, &[0x09, 0x09]) else {
        return;
    };
    let mut parser = robot_parser();
    assert_eq!(parser.timeout_ms(), 50, "921600 must hit the 50 ms floor");

    let prefix = [0xA5, 0x5A, 0x01, 0x00, 0x07, 0x00, 0x01];
    let mut early = Vec::new();
    for &byte in &prefix {
        if let Some(frame) = parser.push(byte, 0) {
            early.push(frame.header.id);
        }
    }
    assert!(early.is_empty(), "a partial candidate must not yield");

    assert_eq!(
        collect(&mut parser, &frame, 100),
        vec![(3, vec![0x09, 0x09])],
        "the parser must resynchronise after the stall"
    );
}

/// Two or more complete frames in one feed are yielded in order.
#[test]
fn several_frames_in_one_feed_are_yielded_in_order() {
    let Some(first) = encoded(1, &[0xA1]) else {
        return;
    };
    let Some(second) = encoded(2, &[0xB2, 0xB3]) else {
        return;
    };
    let Some(third) = encoded(3, &[]) else {
        return;
    };
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&first);
    bytes.extend_from_slice(&second);
    bytes.extend_from_slice(&third);

    let mut parser = robot_parser();
    let mut ids = Vec::new();
    parser.feed(&bytes, 0, |frame| ids.push(frame.header.id));
    assert_eq!(ids, vec![1, 2, 3], "frames must arrive in wire order");
}

/// Worst-case garbage is bounded and never panics for either direction.
#[test]
fn worst_case_garbage_is_bounded_and_does_not_panic() {
    let Some(frame) = encoded(0x4242, &[0x77]) else {
        return;
    };
    let mut bytes = Vec::new();
    bytes.resize(4096, 0xA5);
    bytes.extend(garbage(0x1234_5678, 8192));
    bytes.extend_from_slice(&frame);

    let mut robot = robot_parser();
    let robot_frames = collect(&mut robot, &bytes, 0);
    assert_eq!(
        robot_frames.last(),
        Some(&(0x4242, vec![0x77])),
        "the trailing frame must be found after garbage"
    );

    let mut pc = PcToRobotParser::new(Direction::PcToRobot, BAUD);
    let pc_frames = collect(&mut pc, &bytes, 0);
    assert_eq!(
        pc_frames.last(),
        Some(&(0x4242, vec![0x77])),
        "the PC-to-robot limit must recover too"
    );
}

/// A corrupted CRC is counted as a failure and the candidate is discarded.
#[test]
fn a_bad_crc_is_counted_and_rejected() {
    let Some(mut frame) = encoded(7, &[0x10, 0x20, 0x30]) else {
        return;
    };
    // The CRC field is at offsets 10 and 11; flipping one byte makes it disagree.
    frame[10] ^= 0xFF;

    let mut parser = robot_parser();
    assert!(
        collect(&mut parser, &frame, 0).is_empty(),
        "a bad CRC must not yield a frame"
    );
    assert_eq!(parser.crc_failures(), 1, "the CRC failure must be counted");
    assert_eq!(parser.resyncs(), 1, "the candidate must be resynchronised");
    assert!(
        parser.discarded_bytes() > 0,
        "the rejected candidate's bytes must be counted as discarded"
    );
}

/// A resync after a bad CRC still yields the next clean frame.
#[test]
fn a_resync_after_a_bad_crc_recovers_and_is_counted() {
    let Some(mut first) = encoded(1, &[0x10, 0x20]) else {
        return;
    };
    let Some(second) = encoded(2, &[0x30]) else {
        return;
    };
    first[10] ^= 0xFF;
    let mut bytes = first;
    bytes.extend_from_slice(&second);

    let mut parser = robot_parser();
    assert_eq!(
        collect(&mut parser, &bytes, 0),
        vec![(2, vec![0x30])],
        "the clean frame after the resync must be yielded"
    );
    assert_eq!(parser.crc_failures(), 1, "one bad CRC must be counted");
    assert_eq!(parser.resyncs(), 1, "one resync must be counted");
}

/// A clean stream leaves the reject counters at zero.
#[test]
fn a_clean_frame_leaves_the_statistics_at_zero() {
    let Some(frame) = encoded(9, &[0x01, 0x02]) else {
        return;
    };

    let mut parser = robot_parser();
    assert_eq!(collect(&mut parser, &frame, 0), vec![(9, vec![0x01, 0x02])]);
    assert_eq!(parser.crc_failures(), 0);
    assert_eq!(parser.resyncs(), 0);
    assert_eq!(parser.discarded_bytes(), 0);
}

/// The same seed and the same calls replay the same bytes in both directions.
#[test]
fn wire_is_deterministic_for_a_fixed_seed_in_both_directions() {
    let first = wire_run(0xDEAD_BEEF);
    let second = wire_run(0xDEAD_BEEF);

    assert_eq!(first, second, "a fixed seed must replay exactly");
    assert!(!first.0.is_empty(), "the robot-to-PC queue must carry bytes");
    assert!(!first.1.is_empty(), "the PC-to-robot queue must carry bytes");
}

/// The stall fault advances the wire clock and leaves the queue intact.
#[test]
fn stall_ms_advances_the_wire_clock() {
    let mut wire = Wire::new(
        1,
        Faults {
            stall_ms: 100,
            ..Faults::default()
        },
    );
    wire.advance(5);
    wire.send(Direction::RobotToPc, &[0x01, 0x02]);

    assert_eq!(wire.now_ms(), 105, "the stall must advance the clock");
    assert_eq!(
        wire.available(Direction::RobotToPc),
        2,
        "stalling must not drop queued bytes"
    );
}

/// A faulted frame is discarded and a parser recovers the clean frame that follows it.
#[test]
fn wire_faults_are_exercised_by_the_parser() {
    let Some(first) = encoded(1, &[0x11, 0x22, 0x33]) else {
        return;
    };
    let Some(second) = encoded(2, &[0x44, 0x55]) else {
        return;
    };
    let mut wire = Wire::new(
        0x0BAD_F00D,
        Faults {
            truncate_every: 1,
            ..Faults::default()
        },
    );
    wire.send(Direction::RobotToPc, &first);
    let mut bytes = wire.take(Direction::RobotToPc);
    wire.set_faults(Faults::default());
    wire.send(Direction::RobotToPc, &second);
    bytes.extend_from_slice(&wire.take(Direction::RobotToPc));

    let mut parser = robot_parser();
    assert_eq!(
        collect(&mut parser, &bytes, 0),
        vec![(2, vec![0x44, 0x55])],
        "the parser must recover from the wire's truncated frame"
    );
}
