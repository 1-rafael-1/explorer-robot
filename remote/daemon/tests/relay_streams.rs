//! Stream slots, snapshot assembly and `last_terminal` (ADR-0039).
//!
//! The tests drive the relay through the in-memory app transport and a fake link: they assert the
//! messages an app would receive, never the relay's internal slots.

mod support;

use link_protocol::{
    ids::ProcedureId,
    payload::{
        command::StatusSnapshot,
        stream::{ActivityStage, CalibrationStatus},
    },
};
use remote_daemon::{
    relay::Relay,
    schema::{PROTOCOL, Stage, StreamName, StreamValue},
};
use support::{
    activity, calibration, entry, must, perception, power, session_with_app, snapshot, stream_named, streams,
};

/// A snapshot carries the latest sample of every seen stream and omits the unseen ones.
#[test]
fn a_snapshot_carries_only_seen_streams() {
    let mut relay = Relay::new();
    relay.session_up(7, 1_000);
    let app = relay.connect_app(1_000);
    let _ = relay.on_power(&power(80, 7_400), 1_100);
    let _ = relay.on_power(&power(81, 7_500), 1_200);
    let _ = relay.on_motion(&link_protocol::payload::stream::Motion { left: 10, right: -10 }, 1_300);

    let messages = relay.poll(app, 1_400);
    let assembled = must(snapshot(&messages), "snapshot");
    assert_eq!(assembled.protocol, PROTOCOL);
    assert_eq!(assembled.boot_id, Some(7));
    assert_eq!(assembled.ts, 1_400);
    assert_eq!(assembled.streams.len(), 2);

    let power_entry = must(entry(&assembled.streams, StreamName::Power), "power");
    assert_eq!(power_entry.seq, 2, "the snapshot carries the daemon's latest sequence");
    assert_eq!(power_entry.ts, 1_200, "ts is the receive time of that sample");
    assert_eq!(
        power_entry.v,
        StreamValue::Power {
            level: Some(81),
            voltage_mv: Some(7_500),
        }
    );
    assert!(
        entry(&assembled.streams, StreamName::Lidar).is_none(),
        "an unseen stream is omitted"
    );
    assert!(
        streams(&messages).is_empty(),
        "the snapshot already carries the slots it delivered"
    );
}

/// Stream sequences are the daemon's, monotonic and coalesced latest-wins.
#[test]
fn stream_sequences_are_the_daemons_and_only_the_latest_wins() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert!(
        streams(&relay.poll(app, 0)).is_empty(),
        "the first poll only carries the snapshot"
    );

    let _ = relay.on_power(&power(80, 7_400), 10);
    let _ = relay.on_power(&power(81, 7_500), 11);
    let messages = relay.poll(app, 12);
    let entries = streams(&messages);
    assert_eq!(entries.len(), 1, "two samples coalesce into one latest-wins slot");
    assert_eq!(entries[0].seq, 2);
    assert_eq!(entries[0].ts, 11);
    assert_eq!(relay.poll(app, 13), Vec::new());

    let _ = relay.on_power(&power(82, 7_600), 20);
    let messages = relay.poll(app, 21);
    let latest = must(stream_named(&messages, StreamName::Power), "power");
    assert_eq!(latest.seq, 3, "the daemon sequence keeps counting");

    let assembled = must(relay.snapshot(app, 30), "snapshot");
    let entry = must(entry(&assembled.streams, StreamName::Power), "power");
    assert_eq!(entry.seq, 3);
}

/// A terminal Activity sample is retained until another terminal sample or a Boot Id change.
#[test]
fn a_terminal_activity_is_retained_as_last_terminal() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.on_activity(&activity(5, ActivityStage::Running, Some(20), "running"), 10);
    let _ = relay.on_activity(&activity(5, ActivityStage::Complete, Some(100), "done"), 20);

    let messages = relay.poll(app, 30);
    let assembled = must(snapshot(&messages), "snapshot");
    let terminal = must(assembled.last_terminal, "last_terminal");
    assert!(matches!(
        &terminal.v,
        StreamValue::Activity {
            id: 5,
            stage: Stage::Complete,
            percent: Some(100),
            ..
        }
    ));
    let current = must(entry(&assembled.streams, StreamName::Activity), "activity");
    assert_eq!(current.v, terminal.v, "the latest slot is the terminal sample too");

    let _ = relay.on_activity(&activity(5, ActivityStage::Running, Some(50), "again"), 40);
    let assembled = must(relay.snapshot(app, 50), "snapshot");
    let terminal = must(assembled.last_terminal, "last_terminal");
    assert!(
        matches!(
            &terminal.v,
            StreamValue::Activity {
                stage: Stage::Complete,
                ..
            }
        ),
        "a later running sample does not erase the Result Report"
    );
    let current = must(entry(&assembled.streams, StreamName::Activity), "activity");
    assert!(matches!(
        &current.v,
        StreamValue::Activity {
            stage: Stage::Running,
            ..
        }
    ));
}

/// `GET_STATUS` seeds the five streams it carries and never invents a `LiDAR` scan.
#[test]
fn a_status_snapshot_seeds_its_five_streams() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let status = StatusSnapshot {
        level: 0xFF,
        voltage_mv: 0xFFFF,
        left: 5,
        right: -5,
        procedure_id: ProcedureId::new(5),
        stage: ActivityStage::Running,
        percent: None,
        detail: "panel",
        flags: perception(0b11),
        calibration: calibration(
            CalibrationStatus::Loaded,
            CalibrationStatus::Unknown,
            CalibrationStatus::Missing,
            CalibrationStatus::Loaded,
        ),
    };
    relay.on_status_snapshot(&status, 10);

    let messages = relay.poll(app, 20);
    let assembled = must(snapshot(&messages), "snapshot");
    assert_eq!(assembled.streams.len(), 5);
    assert!(entry(&assembled.streams, StreamName::Lidar).is_none());

    let power_entry = must(entry(&assembled.streams, StreamName::Power), "power");
    assert_eq!(
        power_entry.v,
        StreamValue::Power {
            level: None,
            voltage_mv: None,
        },
        "the sentinels become null"
    );
    let motion = must(entry(&assembled.streams, StreamName::Motion), "motion");
    assert_eq!(motion.v, StreamValue::Motion { left: 5, right: -5 });
    let activity_entry = must(entry(&assembled.streams, StreamName::Activity), "activity");
    assert_eq!(
        activity_entry.v,
        StreamValue::Activity {
            id: 5,
            stage: Stage::Running,
            percent: None,
            detail: "panel".to_owned(),
        }
    );
    let perception_entry = must(entry(&assembled.streams, StreamName::Perception), "perception");
    assert_eq!(
        perception_entry.v,
        StreamValue::Perception {
            obstacle: true,
            floor_drop: true,
        }
    );
    let calibration_entry = must(entry(&assembled.streams, StreamName::Calibration), "calibration");
    assert_eq!(
        calibration_entry.v,
        StreamValue::Calibration {
            motor: remote_daemon::schema::CalibrationStatus::Loaded,
            imu: remote_daemon::schema::CalibrationStatus::Unknown,
            mag: remote_daemon::schema::CalibrationStatus::Missing,
            distance: remote_daemon::schema::CalibrationStatus::Loaded,
        }
    );
}

/// A snapshot may be assembled at any time and advances the app past what it carries.
#[test]
fn a_snapshot_assembled_later_does_not_repeat_its_streams() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.on_motion(&link_protocol::payload::stream::Motion { left: 1, right: 2 }, 10);

    let assembled = must(relay.snapshot(app, 20), "snapshot");
    assert_eq!(assembled.streams.len(), 1);
    assert_eq!(relay.poll(app, 21), Vec::new());
}
