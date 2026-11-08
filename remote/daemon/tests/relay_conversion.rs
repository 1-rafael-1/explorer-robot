//! The unit and sign conversion table (ADR-0039).
//!
//! Distances stay centimetres, angles flip from the wire's clockwise convention to the App
//! Boundary's counter-clockwise convention, and every unknown sentinel becomes `null`.

mod support;

use remote_daemon::{
    relay::Relay,
    schema::{
        CalibrationStatus as JsonCalibrationStatus, CompletionStatus, EventName, EventValue, Stage, Status, StreamValue,
    },
};
use link_protocol::{
    CommandMessage, Status as WireStatus,
    payload::{
        event::{
            CalibrationKind, CalibrationLoadStatus, CalibrationLoaded, CompletionStatus as WireCompletionStatus,
            DriveDistanceCompleted, RotateCompleted,
        },
        stream::ActivityStage,
    },
};
use serde_json::json;
use support::{activity, calibration, floor_drop, frame, lidar, must, obstacle, perception, power, session_with_app};

/// Submits, sends and acknowledges one `rotate_by`, returning the wire centidegrees or the
/// app-facing rejection status.
fn rotate(relay: &mut Relay, app: u64, command_seq: u64, angle_deg: f64) -> Result<i16, Status> {
    if let Some(rejection) = relay.submit_command(
        app,
        frame(command_seq, "rotate_by", json!({"angle_deg": angle_deg, "speed": 50})),
        command_seq,
    ) {
        return Err(rejection.status);
    }
    let outbound = must(relay.poll_link(command_seq), "rotate frame");
    relay.on_command_ack(outbound.wire_seq, WireStatus::Ok, command_seq + 1);
    match outbound.message {
        CommandMessage::RotateBy(value) => Ok(value.angle_cdeg),
        _ => Err(Status::Unknown),
    }
}

/// `POWER` sentinels become `null` and ordinary values survive.
#[test]
fn power_sentinels_become_null() {
    let mut relay = Relay::new();
    let entry = relay.on_power(&power(0xFF, 0xFFFF), 10);
    assert_eq!(
        entry.v,
        StreamValue::Power {
            level: None,
            voltage_mv: None,
        }
    );
    let entry = relay.on_power(&power(42, 7_400), 11);
    assert_eq!(
        entry.v,
        StreamValue::Power {
            level: Some(42),
            voltage_mv: Some(7_400),
        }
    );
    assert_eq!(entry.seq, 2);
    assert_eq!(entry.ts, 11);
}

/// Activity stages map one-to-one and an unreported percent stays `null`.
#[test]
fn activity_stages_map_one_to_one() {
    let mut relay = Relay::new();
    for (wire, expected) in [
        (ActivityStage::Running, Stage::Running),
        (ActivityStage::Complete, Stage::Complete),
        (ActivityStage::Failed, Stage::Failed),
    ] {
        let entry = relay.on_activity(&activity(5, wire, Some(50), "detail"), 10);
        assert_eq!(
            entry.v,
            StreamValue::Activity {
                id: 5,
                stage: expected,
                percent: Some(50),
                detail: "detail".to_owned(),
            }
        );
    }
    let entry = relay.on_activity(&activity(5, ActivityStage::Running, None, ""), 20);
    assert!(matches!(&entry.v, StreamValue::Activity { percent: None, .. }));
}

/// The four calibration status bytes map onto the app's vocabulary.
#[test]
fn calibration_statuses_map() {
    let mut relay = Relay::new();
    let entry = relay.on_calibration(
        &calibration(
            link_protocol::payload::stream::CalibrationStatus::Unknown,
            link_protocol::payload::stream::CalibrationStatus::Loaded,
            link_protocol::payload::stream::CalibrationStatus::Missing,
            link_protocol::payload::stream::CalibrationStatus::Loaded,
        ),
        10,
    );
    assert_eq!(
        entry.v,
        StreamValue::Calibration {
            motor: JsonCalibrationStatus::Unknown,
            imu: JsonCalibrationStatus::Loaded,
            mag: JsonCalibrationStatus::Missing,
            distance: JsonCalibrationStatus::Loaded,
        }
    );
}

/// The perception flag bits become booleans.
#[test]
fn perception_bits_become_booleans() {
    let mut relay = Relay::new();
    let entry = relay.on_perception(perception(0), 10);
    assert_eq!(
        entry.v,
        StreamValue::Perception {
            obstacle: false,
            floor_drop: false,
        }
    );
    let entry = relay.on_perception(perception(0b01), 11);
    assert_eq!(
        entry.v,
        StreamValue::Perception {
            obstacle: true,
            floor_drop: false,
        }
    );
    let entry = relay.on_perception(perception(0b10), 12);
    assert_eq!(
        entry.v,
        StreamValue::Perception {
            obstacle: false,
            floor_drop: true,
        }
    );
}

/// A `LIDAR_CLOUD` keeps 360 slots, maps `0xFFFF` to `null` and reports clockwise geometry.
#[test]
fn lidar_geometry_and_nulls() {
    let mut relay = Relay::new();
    let entry = relay.on_lidar(&lidar(), 10);
    let mut expected = vec![Some(100); 360];
    expected[0] = None;
    expected[1] = Some(42);
    assert_eq!(
        entry.v,
        StreamValue::Lidar {
            ranges_cm: expected,
            first_deg: 0,
            step_deg: 1,
            clockwise: true,
        }
    );
}

/// `rotate_by` negates the app's counter-clockwise degrees into clockwise centidegrees.
#[test]
fn rotate_by_negates_degrees_and_validates_the_range() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert_eq!(rotate(&mut relay, app, 1, 90.0), Ok(-9_000));
    assert_eq!(rotate(&mut relay, app, 2, -90.0), Ok(9_000));
    assert_eq!(rotate(&mut relay, app, 3, 180.0), Ok(-18_000));
    assert_eq!(rotate(&mut relay, app, 4, -180.0), Ok(18_000));
    assert_eq!(rotate(&mut relay, app, 5, 0.0), Ok(0));
    assert_eq!(rotate(&mut relay, app, 6, 180.01), Err(Status::BadArgument));
    assert_eq!(rotate(&mut relay, app, 7, 200.0), Err(Status::BadArgument));
}

/// `rotate_completed` reports the target and actual-minus-target error counter-clockwise.
#[test]
fn rotate_completed_flips_both_angles() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    relay.on_rotate_completed(
        1,
        &RotateCompleted {
            command_seq: 0,
            status: WireCompletionStatus::Success,
            target_cdeg: 9_000,
            error_cdeg: -50,
            duration_ms: 2_100,
            detail: "done",
        },
        10,
    );
    let assembled = must(relay.snapshot(app, 20), "snapshot");
    let event = must(
        assembled
            .events
            .into_iter()
            .find(|event| event.name == EventName::RotateCompleted),
        "rotate event",
    );
    assert_eq!(
        event.v,
        EventValue::RotateCompleted {
            command_seq: None,
            status: CompletionStatus::Ok,
            target_deg: -90.0,
            error_deg: 0.5,
            duration_ms: 2_100,
            detail: "done".to_owned(),
        }
    );
}

/// `drive_distance_completed` passes the wire's forward-positive centimetres through.
#[test]
fn drive_completed_keeps_forward_positive_centimetres() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    relay.on_drive_completed(
        1,
        &DriveDistanceCompleted {
            command_seq: 0,
            status: WireCompletionStatus::Failed,
            target_cm: 100,
            distance_cm: -94,
            duration_ms: 4_000,
            detail: "Obstacle",
        },
        10,
    );
    let assembled = must(relay.snapshot(app, 20), "snapshot");
    let event = must(
        assembled
            .events
            .into_iter()
            .find(|event| event.name == EventName::DriveDistanceCompleted),
        "drive event",
    );
    assert_eq!(
        event.v,
        EventValue::DriveDistanceCompleted {
            command_seq: None,
            status: CompletionStatus::Failed,
            target_cm: 100,
            actual_cm: -94,
            duration_ms: 4_000,
            detail: "Obstacle".to_owned(),
        }
    );
}

/// `OBSTACLE` names the `LiDAR` source and calls every other source unknown.
#[test]
fn obstacle_sources_map() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    relay.on_obstacle(1, &obstacle(1, 0), 10);
    relay.on_obstacle(2, &obstacle(1, 1), 11);
    let assembled = must(relay.snapshot(app, 20), "snapshot");
    let sources: Vec<String> = assembled
        .events
        .iter()
        .filter_map(|event| match &event.v {
            EventValue::Obstacle { source, .. } => Some(source.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(sources, vec!["lidar".to_owned(), "unknown".to_owned()]);
}

/// `FLOOR_DROP` turns the wire byte into a boolean.
#[test]
fn floor_drop_detected_becomes_a_boolean() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    relay.on_floor_drop(1, &floor_drop(1), 10);
    let assembled = must(relay.snapshot(app, 20), "snapshot");
    let event = must(
        assembled
            .events
            .into_iter()
            .find(|event| event.name == EventName::FloorDrop),
        "floor-drop event",
    );
    assert_eq!(event.v, EventValue::FloorDrop { detected: true });
}

/// `CALIBRATION_LOADED` names its kind and its load outcome.
#[test]
fn calibration_loaded_names_its_kind_and_status() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    relay.on_calibration_loaded(
        1,
        &CalibrationLoaded {
            kind: CalibrationKind::Motor,
            status: CalibrationLoadStatus::Loaded,
        },
        10,
    );
    relay.on_calibration_loaded(
        2,
        &CalibrationLoaded {
            kind: CalibrationKind::ImuFlags,
            status: CalibrationLoadStatus::Missing,
        },
        11,
    );
    relay.on_calibration_loaded(
        3,
        &CalibrationLoaded {
            kind: CalibrationKind::Distance,
            status: CalibrationLoadStatus::Loaded,
        },
        12,
    );
    let assembled = must(relay.snapshot(app, 20), "snapshot");
    let pairs: Vec<(String, String)> = assembled
        .events
        .iter()
        .filter_map(|event| match &event.v {
            EventValue::CalibrationLoaded { kind, status } => Some((kind.clone(), status.clone())),
            _ => None,
        })
        .collect();
    assert_eq!(
        pairs,
        vec![
            ("motor".to_owned(), "loaded".to_owned()),
            ("imu".to_owned(), "missing".to_owned()),
            ("distance".to_owned(), "loaded".to_owned()),
        ]
    );
}
