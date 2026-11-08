//! Deterministic cross-crate fault matrix over the simulated wire.
//!
//! Each fault mode is injected into one direction at a time and then removed. The tests assert
//! that the fault visibly loses the expected frame and that the same traffic recovers once the
//! fault is gone. The wire advances only through its explicit clock, so every run is identical for
//! a fixed seed.

use link_core::{Config, Inbound, LinkCore};
use link_protocol::{
    Caps, Class, Command, Direction, Event, EventMessage, Flags, Frame, MAX_PC_TO_ROBOT_FRAME, MAX_ROBOT_TO_PC_FRAME,
    PROTOCOL_MAJOR, PROTOCOL_MINOR, Seq,
    payload::event::Obstacle,
    sim::{FakeRemote, Faults, Wire},
};

/// The Boot Id the tests hand to the link core.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// The command payload used for the PC-to-robot direction.
const STOP_MOTION_COAST: [u8; 1] = [0];

/// Creates a core, wire and remote for one fault path.
fn harness() -> (LinkCore, Wire, FakeRemote) {
    (
        LinkCore::new(Config::default(), BOOT_ID, 0),
        Wire::new(0x0BAD_F00D, Faults::default()),
        FakeRemote::new(),
    )
}

/// Sends a `HELLO`, feeds it to the core and drains the `HELLO_ACK`, attaching the peer.
fn attach(core: &mut LinkCore, wire: &mut Wire, remote: &mut FakeRemote, now_ms: u64) {
    let _ = remote.send_hello(wire, PROTOCOL_MAJOR, PROTOCOL_MINOR, Caps::from_bits(0), 1, &[]);
    feed(core, wire, now_ms);
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let _ = core.next_frame(&mut buffer);
}

/// Drains the wire's PC-to-robot queue into the core at `now_ms`.
fn feed(core: &mut LinkCore, wire: &mut Wire, now_ms: u64) {
    let bytes = wire.take(Direction::PcToRobot);
    core.on_rx(&bytes, now_ms);
}

/// Emits one frame into `out`, returning the number of bytes written.
fn emit(core: &mut LinkCore, out: &mut [u8]) -> Option<usize> {
    core.next_frame(out)
}

/// Polls until a command appears, skipping any event or control notice that precedes it.
fn take_command_seq(core: &mut LinkCore) -> Option<Seq> {
    loop {
        match core.poll()? {
            Inbound::Command(request) => return Some(request.seq),
            Inbound::EventAck(_) | Inbound::ControlReply(_) | Inbound::ControlTimeout { .. } => {}
        }
    }
}

/// Publishes the event the fault tests expect to lose.
fn publish_obstacle(core: &mut LinkCore) {
    let _ = core.publish_event(&EventMessage::Obstacle(Obstacle { detected: 1, source: 0 }), 0);
}

/// Encodes one standalone command frame for a directional test.
fn command_frame(seq: u16, flags: Flags, payload: &[u8]) -> Option<Vec<u8>> {
    let frame = Frame::new(
        PROTOCOL_MAJOR,
        Class::Command,
        flags,
        Command::StopMotion.wire_id(),
        Seq::new(seq),
        payload,
    )
    .ok()?;
    let mut buffer = [0u8; MAX_PC_TO_ROBOT_FRAME];
    let written = frame.encode_into(&mut buffer).ok()?;
    Some(buffer[..written].to_vec())
}

/// Dropping every byte loses an event and a command in each direction, then recovery delivers both.
#[test]
fn drop_in_both_directions_loses_then_recovers() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];

    // Robot -> PC: every byte of the event frame is dropped.
    publish_obstacle(&mut core);
    wire.set_direction_faults(
        Direction::RobotToPc,
        Faults {
            drop_every: 1,
            ..Faults::default()
        },
    );
    let Some(written) = emit(&mut core, &mut buffer) else {
        return;
    };
    wire.send(Direction::RobotToPc, &buffer[..written]);
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(remote.received_event_count(), 0, "a dropped frame never arrives");

    wire.set_direction_faults(Direction::RobotToPc, Faults::default());
    core.tick(500);
    let Some(written) = emit(&mut core, &mut buffer) else {
        return;
    };
    wire.send(Direction::RobotToPc, &buffer[..written]);
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(
        remote.received_event_count(),
        1,
        "the retransmission recovers once the drop is removed"
    );
    assert_eq!(remote.last_event().map(|(event, _, _)| event), Some(Event::Obstacle));

    // PC -> robot: the command frame is dropped, then a retry after recovery is accepted.
    wire.set_direction_faults(
        Direction::PcToRobot,
        Faults {
            drop_every: 1,
            ..Faults::default()
        },
    );
    let Some(seq) = remote.send_command(&mut wire, Command::StopMotion, &STOP_MOTION_COAST) else {
        return;
    };
    feed(&mut core, &mut wire, 600);
    assert!(
        take_command_seq(&mut core).is_none(),
        "a dropped command never reaches the core"
    );

    wire.set_direction_faults(Direction::PcToRobot, Faults::default());
    assert!(remote.send_command_frame(
        &mut wire,
        Command::StopMotion.wire_id(),
        seq,
        Flags::ACK_REQUESTED,
        &STOP_MOTION_COAST,
    ));
    feed(&mut core, &mut wire, 700);
    assert_eq!(
        take_command_seq(&mut core),
        Some(seq),
        "the retry arrives once the drop is removed"
    );
}

/// Corrupting every byte fails the CRC in each direction, then recovery delivers both frames.
#[test]
fn corruption_in_both_directions_loses_then_recovers() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];

    // Robot -> PC: the corrupted event frame fails its CRC.
    publish_obstacle(&mut core);
    wire.set_direction_faults(
        Direction::RobotToPc,
        Faults {
            corrupt_every: 1,
            ..Faults::default()
        },
    );
    let Some(written) = emit(&mut core, &mut buffer) else {
        return;
    };
    wire.send(Direction::RobotToPc, &buffer[..written]);
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(remote.received_event_count(), 0, "corrupt bytes never decode");

    wire.set_direction_faults(Direction::RobotToPc, Faults::default());
    core.tick(500);
    let Some(written) = emit(&mut core, &mut buffer) else {
        return;
    };
    wire.send(Direction::RobotToPc, &buffer[..written]);
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(
        remote.received_event_count(),
        1,
        "the retransmission recovers once corruption is removed"
    );

    // PC -> robot: the corrupted command never reaches the core.
    wire.set_direction_faults(
        Direction::PcToRobot,
        Faults {
            corrupt_every: 1,
            ..Faults::default()
        },
    );
    let Some(seq) = remote.send_command(&mut wire, Command::StopMotion, &STOP_MOTION_COAST) else {
        return;
    };
    feed(&mut core, &mut wire, 600);
    assert!(
        take_command_seq(&mut core).is_none(),
        "a corrupted command never decodes"
    );

    wire.set_direction_faults(Direction::PcToRobot, Faults::default());
    assert!(remote.send_command_frame(
        &mut wire,
        Command::StopMotion.wire_id(),
        seq,
        Flags::ACK_REQUESTED,
        &STOP_MOTION_COAST,
    ));
    feed(&mut core, &mut wire, 700);
    assert_eq!(
        take_command_seq(&mut core),
        Some(seq),
        "the retry arrives once corruption is removed"
    );
}

/// Truncating every chunk loses the last byte of each frame, then recovery delivers both.
#[test]
fn truncation_in_both_directions_loses_then_recovers() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];

    // Robot -> PC: the event loses its final payload byte and never completes.
    publish_obstacle(&mut core);
    wire.set_direction_faults(
        Direction::RobotToPc,
        Faults {
            truncate_every: 1,
            ..Faults::default()
        },
    );
    let Some(written) = emit(&mut core, &mut buffer) else {
        return;
    };
    wire.send(Direction::RobotToPc, &buffer[..written]);
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(remote.received_event_count(), 0, "a truncated frame never completes");

    wire.set_direction_faults(Direction::RobotToPc, Faults::default());
    wire.advance(100);
    core.tick(500);
    let Some(written) = emit(&mut core, &mut buffer) else {
        return;
    };
    wire.send(Direction::RobotToPc, &buffer[..written]);
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(
        remote.received_event_count(),
        1,
        "the retransmission recovers once truncation is removed"
    );

    // PC -> robot: the command loses its payload byte and is never yielded.
    wire.set_direction_faults(
        Direction::PcToRobot,
        Faults {
            truncate_every: 1,
            ..Faults::default()
        },
    );
    let Some(seq) = remote.send_command(&mut wire, Command::StopMotion, &STOP_MOTION_COAST) else {
        return;
    };
    feed(&mut core, &mut wire, 600);
    assert!(
        take_command_seq(&mut core).is_none(),
        "a truncated command never completes"
    );

    wire.set_direction_faults(Direction::PcToRobot, Faults::default());
    wire.advance(100);
    assert!(remote.send_command_frame(
        &mut wire,
        Command::StopMotion.wire_id(),
        seq,
        Flags::ACK_REQUESTED,
        &STOP_MOTION_COAST,
    ));
    let now_ms = u64::from(wire.now_ms());
    feed(&mut core, &mut wire, now_ms);
    assert_eq!(
        take_command_seq(&mut core),
        Some(seq),
        "the retry arrives once truncation is removed"
    );
}

/// A stall past the frame timeout breaks a frame assembled in two chunks in each direction.
#[test]
fn stall_in_both_directions_loses_then_recovers() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote, 0);
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];

    // Robot -> PC: the frame is split around a stall longer than the 50 ms timeout.
    publish_obstacle(&mut core);
    let Some(written) = emit(&mut core, &mut buffer) else {
        return;
    };
    wire.set_direction_faults(
        Direction::RobotToPc,
        Faults {
            stall_ms: 60,
            ..Faults::default()
        },
    );
    let split = written / 2;
    let (first, second) = buffer[..written].split_at(split);
    wire.send(Direction::RobotToPc, first);
    assert_eq!(remote.receive(&mut wire), 0);
    wire.send(Direction::RobotToPc, second);
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(remote.received_event_count(), 0, "a stalled frame is discarded");

    wire.set_direction_faults(Direction::RobotToPc, Faults::default());
    core.tick(500);
    let Some(written) = emit(&mut core, &mut buffer) else {
        return;
    };
    wire.send(Direction::RobotToPc, &buffer[..written]);
    assert_eq!(remote.receive(&mut wire), 0);
    assert_eq!(
        remote.received_event_count(),
        1,
        "the retransmission recovers once the stall is removed"
    );

    // PC -> robot: the command is split around a stall.
    let Some(frame) = command_frame(1, Flags::ACK_REQUESTED, &STOP_MOTION_COAST) else {
        return;
    };
    wire.set_direction_faults(
        Direction::PcToRobot,
        Faults {
            stall_ms: 60,
            ..Faults::default()
        },
    );
    let split = frame.len() / 2;
    let (first, second) = frame.split_at(split);
    wire.send(Direction::PcToRobot, first);
    let now_ms = u64::from(wire.now_ms());
    feed(&mut core, &mut wire, now_ms);
    wire.send(Direction::PcToRobot, second);
    let now_ms = u64::from(wire.now_ms());
    feed(&mut core, &mut wire, now_ms);
    assert!(take_command_seq(&mut core).is_none(), "a stalled command is discarded");

    wire.set_direction_faults(Direction::PcToRobot, Faults::default());
    wire.advance(100);
    assert!(remote.send_command_frame(
        &mut wire,
        Command::StopMotion.wire_id(),
        Seq::new(1),
        Flags::ACK_REQUESTED,
        &STOP_MOTION_COAST,
    ));
    let now_ms = u64::from(wire.now_ms());
    feed(&mut core, &mut wire, now_ms);
    assert_eq!(
        take_command_seq(&mut core),
        Some(Seq::new(1)),
        "the retry arrives once the stall is removed"
    );
}
