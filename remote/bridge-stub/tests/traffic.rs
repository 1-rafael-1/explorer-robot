//! The scripted robot traffic behind `ACCEPT` (ticket 09).

mod support;

use std::time::Duration;

use bridge_stub::{Scenario, ScriptEvent, StreamRate};
use remote_protocol::{Class, Command, Event, Flags as LinkFlags, OBSTACLE_SOURCE_LIDAR, RetryPolicy, Status, Stream};
use support::BenchError;

/// The token the tests pair with.
const TOKEN: &[u8] = b"pairing-token-01";

/// `HELLO` is answered with a `HELLO_ACK` that mirrors the sequence and serves the Boot Id.
#[test]
fn hello_is_answered_with_hello_ack() -> Result<(), BenchError> {
    let stub = support::start(&Scenario::new(TOKEN))?;
    let (mut client, _) = support::accepted(stub.link_addr(), TOKEN)?;
    let seq = client.send_hello(0xABCD_1234)?;
    client.pump(Duration::from_millis(300))?;

    let replies = client.frames_of(Class::Control, remote_protocol::Control::Hello.wire_id());
    assert_eq!(replies.len(), 1, "exactly one HELLO_ACK is sent");
    let reply = &replies[0];
    assert!(
        reply.flags.contains(LinkFlags::IS_ACK),
        "the reply is an acknowledgement"
    );
    assert_eq!(reply.seq, seq, "the reply mirrors the HELLO's sequence");

    let ack = support::hello_ack(&reply.payload)?;
    assert_eq!(ack.boot_id, 1, "the stub's Boot Id is served");
    assert_eq!(ack.status, Status::Ok);
    assert_eq!(ack.major, remote_protocol::PROTOCOL_MAJOR);
    assert_eq!(ack.minor, remote_protocol::PROTOCOL_MINOR);
    Ok(())
}

/// `PING` is answered with a `PING_ACK` that mirrors the sequence.
#[test]
fn ping_is_answered_with_ping_ack() -> Result<(), BenchError> {
    let stub = support::start(&Scenario::new(TOKEN))?;
    let (mut client, _) = support::accepted(stub.link_addr(), TOKEN)?;
    let seq = client.send_ping()?;
    client.pump(Duration::from_millis(300))?;

    let replies = client.frames_of(Class::Control, remote_protocol::Control::Ping.wire_id());
    assert_eq!(replies.len(), 1, "exactly one PING_ACK is sent");
    let reply = &replies[0];
    assert!(
        reply.flags.contains(LinkFlags::IS_ACK),
        "the reply is an acknowledgement"
    );
    assert_eq!(reply.seq, seq, "the reply mirrors the PING's sequence");
    assert_eq!(support::ping_ack(&reply.payload)?.status, Status::Ok);
    Ok(())
}

/// Streams flow at their configured rates, with per-stream sequence numbers for gap detection.
#[test]
fn streams_flow_at_the_configured_rates() -> Result<(), BenchError> {
    let mut scenario = Scenario::new(TOKEN);
    scenario.traffic.streams = vec![
        StreamRate {
            stream: Stream::Motion,
            rate_chz: 2_000, // 20 Hz
        },
        StreamRate {
            stream: Stream::Calibration,
            rate_chz: 100, // 1 Hz
        },
    ];
    let stub = support::start(&scenario)?;
    let (mut client, _) = support::accepted(stub.link_addr(), TOKEN)?;
    client.pump(Duration::from_millis(500))?;

    let motion = client.frames_of(Class::Stream, Stream::Motion.wire_id());
    assert!(
        motion.len() >= 5,
        "a 20 Hz stream delivers at least five samples in half a second"
    );
    let calibration = client.frames_of(Class::Stream, Stream::Calibration.wire_id());
    assert!(
        calibration.len() <= 2,
        "a 1 Hz stream delivers at most two samples in half a second"
    );
    for (index, frame) in motion.iter().enumerate() {
        assert_eq!(
            frame.seq,
            u16::try_from(index + 1).unwrap_or(u16::MAX),
            "the motion sequence is consecutive from one"
        );
    }
    let sample = support::motion_sample(&motion[0].payload)?;
    assert!((sample.left.abs()) <= 100 && sample.right.abs() <= 100);
    Ok(())
}

/// Events are retried until acknowledged, and only then does the next one flow.
#[test]
fn events_retry_until_acked() -> Result<(), BenchError> {
    let mut scenario = Scenario::new(TOKEN);
    scenario.traffic.events = vec![
        ScriptEvent::Obstacle {
            detected: 1,
            source: OBSTACLE_SOURCE_LIDAR,
        },
        ScriptEvent::FloorDrop { detected: 1 },
    ];
    scenario.traffic.event_retry = RetryPolicy::new(400, 2, 1_600, 5);
    let stub = support::start(&scenario)?;
    let (mut client, _) = support::accepted(stub.link_addr(), TOKEN)?;

    // Without an ACK the first event is retransmitted, and the second event waits.
    client.pump(Duration::from_millis(550))?;
    let obstacle = client.frames_of(Class::Event, Event::Obstacle.wire_id());
    assert!(obstacle.len() >= 2, "the unacknowledged event is retransmitted");
    assert!(
        obstacle
            .iter()
            .all(|frame| frame.flags.contains(LinkFlags::ACK_REQUESTED)),
        "every transmission asks for acknowledgement"
    );
    assert!(
        obstacle[1..].iter().all(|frame| frame.flags.contains(LinkFlags::RETRY)),
        "retransmissions carry the RETRY flag"
    );
    assert_eq!(obstacle[0].seq, 1, "the event sequence starts at one");
    assert!(
        client.frames_of(Class::Event, Event::FloorDrop.wire_id()).is_empty(),
        "the second event waits for the first's ACK"
    );

    // The ACK retires the event and the next one flows.
    client.ack_event(Event::Obstacle, 1)?;
    client.pump(Duration::from_millis(250))?;
    let floor = client.frames_of(Class::Event, Event::FloorDrop.wire_id());
    assert_eq!(floor.len(), 1, "the second event flows once the first is acknowledged");
    assert_eq!(floor[0].seq, 2, "the event sequence is per Boot Id");

    // Acknowledged events are never retransmitted; the script is exhausted.
    client.ack_event(Event::FloorDrop, 2)?;
    client.pump(Duration::from_millis(500))?;
    assert_eq!(
        client.frames_of(Class::Event, Event::FloorDrop.wire_id()).len(),
        1,
        "an acknowledged event is not retransmitted"
    );
    assert_eq!(
        client.frames_of(Class::Event, Event::Obstacle.wire_id()).len(),
        2,
        "the acknowledged first event is not retransmitted either"
    );
    Ok(())
}

/// Command ACKs carry the configured statuses, and the two queries answer their typed responses.
#[test]
fn command_acks_carry_the_configured_statuses() -> Result<(), BenchError> {
    let mut scenario = Scenario::new(TOKEN);
    scenario.traffic.command_acks = vec![
        (Command::DriveDistance, Status::Rejected),
        (Command::StopMotion, Status::Ok),
    ];
    let stub = support::start(&scenario)?;
    let (mut client, _) = support::accepted(stub.link_addr(), TOKEN)?;

    client.send_drive_distance(1, 50, 50)?;
    client.send_stop_motion(2, 1)?;
    client.pump(Duration::from_millis(300))?;

    let drive = client.frames_of(Class::Command, Command::DriveDistance.wire_id());
    assert_eq!(drive.len(), 1, "exactly one DRIVE_DISTANCE ACK");
    assert!(drive[0].flags.contains(LinkFlags::IS_ACK));
    assert_eq!(drive[0].seq, 1, "the ACK mirrors the command's sequence");
    assert_eq!(
        support::ack_status(Command::DriveDistance, &drive[0].payload)?,
        Status::Rejected,
        "the configured status is served"
    );

    let stop = client.frames_of(Class::Command, Command::StopMotion.wire_id());
    assert_eq!(stop.len(), 1, "exactly one STOP_MOTION ACK");
    assert_eq!(
        support::ack_status(Command::StopMotion, &stop[0].payload)?,
        Status::Ok,
        "an unlisted command is acknowledged OK"
    );

    // The two query commands answer their typed responses, not a bare status byte.
    client.send_get_status(3)?;
    client.send_command(Command::GetCatalogue, 4, &[0])?;
    client.pump(Duration::from_millis(300))?;
    let status = client.frames_of(Class::Command, Command::GetStatus.wire_id());
    assert_eq!(status.len(), 1);
    let snapshot = support::status_snapshot(&status[0].payload)?;
    assert_eq!(snapshot.level, 87);
    assert_eq!(snapshot.voltage_mv, 7_400);
    let catalogue = client.frames_of(Class::Command, Command::GetCatalogue.wire_id());
    assert_eq!(catalogue.len(), 1);
    assert_eq!(support::catalogue_page(&catalogue[0].payload)?.total, 0);
    Ok(())
}

/// The traffic only flows behind `ACCEPT`: a denied client never sees a Link frame.
#[test]
fn traffic_only_flows_behind_accept() -> Result<(), BenchError> {
    let mut scenario = Scenario::new(TOKEN);
    scenario.deny = Some(bridge_stub::Deny {
        reason: bridge_stub::preamble::DenyReason::Busy,
        retry_ms: Some(1_000),
    });
    scenario.traffic.streams = vec![StreamRate {
        stream: Stream::Motion,
        rate_chz: 2_000,
    }];
    let stub = support::start(&scenario)?;
    let (reason, _) = support::denied(stub.link_addr(), TOKEN)?;
    assert_eq!(reason, bridge_stub::preamble::DenyReason::Busy);
    Ok(())
}
