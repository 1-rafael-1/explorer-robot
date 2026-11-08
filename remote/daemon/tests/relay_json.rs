//! JSON round-trips of the App Boundary's schema (ADR-0038, ADR-0039).
//!
//! Every message type crosses the WebSocket as one text frame: the literal below is the frame the
//! app sees, and the value that comes back is the value the adapter sent.

use remote_daemon::schema::{
    Auth, BridgeStation, BridgeStatus, Catalogue, CatalogueProcedure, ClientHello, ClientMessage, CommandFrame,
    CommandResult, CompletionStatus, Connect, Connection, ConnectionState, EventEntry, EventName, EventValue, Hello,
    Outcome, PROTOCOL, Pair, RobotModel, ServerMessage, ServerName, Snapshot, Stage, Status, StreamEntry, StreamName,
    StreamValue,
};
use serde_json::{Value, json};

/// A server `hello` names the stable service, not the process (ADR-0039).
#[test]
fn a_server_hello_round_trips() {
    let message = ServerMessage::Hello(Hello {
        protocol: PROTOCOL,
        server: ServerName {
            name: "explorer-remote".to_owned(),
        },
        robot: RobotModel { model: None },
        auth_required: false,
    });
    let value = json!({
        "type": "hello",
        "protocol": 1,
        "server": {"name": "explorer-remote"},
        "robot": {"model": null},
        "auth_required": false,
    });
    assert_round_trip(&message, value);
}

/// A `connection` message carries the reachability states of ADR-0038.
#[test]
fn a_connection_message_round_trips() {
    let message = ServerMessage::Connection(Connection {
        state: ConnectionState::Reconnecting,
        robot: Some("Explorer-0001".to_owned()),
        detail: Some("Link stalled".to_owned()),
        retry_in_ms: Some(500),
        boot_id: Some(0xA11CE),
        ts: 12_345,
    });
    let value = json!({
        "type": "connection",
        "state": "reconnecting",
        "robot": "Explorer-0001",
        "detail": "Link stalled",
        "retry_in_ms": 500,
        "boot_id": 659_918,
        "ts": 12345,
    });
    assert_round_trip(&message, value);
}

/// A `snapshot` carries the streams, `last_terminal`, a bounded replay, the cursor and the catalogue.
#[test]
fn a_snapshot_round_trips() {
    let message = ServerMessage::Snapshot(Snapshot {
        protocol: PROTOCOL,
        boot_id: Some(0xA11CE),
        ts: 12_345,
        streams: vec![
            StreamEntry {
                name: StreamName::Power,
                seq: 4,
                ts: 12_000,
                v: StreamValue::Power {
                    level: Some(80),
                    voltage_mv: Some(7400),
                },
            },
            StreamEntry {
                name: StreamName::Lidar,
                seq: 2,
                ts: 12_001,
                v: StreamValue::Lidar {
                    ranges_cm: vec![Some(30), None],
                    first_deg: 0,
                    step_deg: 1,
                    clockwise: true,
                },
            },
        ],
        last_terminal: Some(StreamEntry {
            name: StreamName::Activity,
            seq: 7,
            ts: 11_000,
            v: StreamValue::Activity {
                id: 5,
                stage: Stage::Complete,
                percent: Some(100),
                detail: "done".to_owned(),
            },
        }),
        events: vec![EventEntry {
            boot_id: Some(0xA11CE),
            seq: Some(3),
            ts: 12_100,
            name: EventName::Obstacle,
            v: EventValue::Obstacle {
                detected: true,
                source: "lidar".to_owned(),
            },
        }],
        event_cursor: Some(3),
        catalogue: Some(catalogue()),
        bridge: None,
    });
    let value = json!({
        "type": "snapshot",
        "protocol": 1,
        "boot_id": 659_918,
        "ts": 12345,
        "streams": [
            {"name": "power", "seq": 4, "ts": 12000, "v": {"level": 80, "voltage_mv": 7400}},
            {
                "name": "lidar",
                "seq": 2,
                "ts": 12001,
                "v": {"ranges_cm": [30, null], "first_deg": 0, "step_deg": 1, "clockwise": true}
            },
        ],
        "last_terminal": {
            "name": "activity",
            "seq": 7,
            "ts": 11000,
            "v": {"id": 5, "stage": "complete", "percent": 100, "detail": "done"}
        },
        "events": [
            {
                "boot_id": 659_918,
                "seq": 3,
                "ts": 12100,
                "name": "obstacle",
                "v": {"detected": true, "source": "lidar"}
            }
        ],
        "event_cursor": 3,
        "catalogue": {
            "procedures": [
                {
                    "id": 2,
                    "label": "Motor calibration",
                    "available": true,
                    "interactive": false,
                    "calibration": true,
                    "takes_params": false
                }
            ]
        },
        "bridge": null,
    });
    assert_round_trip(&message, value);
}

/// A `bridge` message carries the converted Bridge status as a latest-wins snapshot.
#[test]
fn a_bridge_message_round_trips() {
    let message = ServerMessage::Bridge(BridgeStatus {
        ts: 12_345,
        rssi_dbm: Some(-54),
        channel: Some(6),
        station: BridgeStation::Associated,
        provisioned: true,
        setup_open: false,
        link_ready: true,
        uart_dropped_bytes: 1,
        dma_stalls: 2,
        crc_failures: 3,
        resyncs: 4,
        wdt_feeds: 5,
        wdt_withheld: 6,
    });
    let value = json!({
        "type": "bridge",
        "ts": 12345,
        "rssi_dbm": -54,
        "channel": 6,
        "station": "associated",
        "provisioned": true,
        "setup_open": false,
        "link_ready": true,
        "uart_dropped_bytes": 1,
        "dma_stalls": 2,
        "crc_failures": 3,
        "resyncs": 4,
        "wdt_feeds": 5,
        "wdt_withheld": 6,
    });
    assert_round_trip(&message, value);
}

/// A `stream` message is `{type, name, seq, ts, v}` with the daemon's own sequence.
#[test]
fn a_stream_message_round_trips() {
    let message = ServerMessage::Stream(StreamEntry {
        name: StreamName::Perception,
        seq: 9,
        ts: 12_345,
        v: StreamValue::Perception {
            obstacle: true,
            floor_drop: false,
        },
    });
    let value = json!({
        "type": "stream",
        "name": "perception",
        "seq": 9,
        "ts": 12345,
        "v": {"obstacle": true, "floor_drop": false},
    });
    assert_round_trip(&message, value);
}

/// An `event` message keeps the wire's `(boot_id, seq)`; the completion carries the outcome.
#[test]
fn an_event_message_round_trips() {
    let rotate = ServerMessage::Event(EventEntry {
        boot_id: Some(0xA11CE),
        seq: Some(4),
        ts: 12_345,
        name: EventName::RotateCompleted,
        v: EventValue::RotateCompleted {
            command_seq: Some(1),
            status: CompletionStatus::Ok,
            target_deg: -90.0,
            error_deg: -0.5,
            duration_ms: 2100,
            detail: "done".to_owned(),
        },
    });
    assert_round_trip(
        &rotate,
        json!({
            "type": "event",
            "boot_id": 659_918,
            "seq": 4,
            "ts": 12345,
            "name": "rotate_completed",
            "v": {
                "command_seq": 1,
                "status": "ok",
                "target_deg": -90.0,
                "error_deg": -0.5,
                "duration_ms": 2100,
                "detail": "done"
            }
        }),
    );

    let drive = ServerMessage::Event(EventEntry {
        boot_id: Some(0xA11CE),
        seq: Some(5),
        ts: 12_346,
        name: EventName::DriveDistanceCompleted,
        v: EventValue::DriveDistanceCompleted {
            command_seq: None,
            status: CompletionStatus::Failed,
            target_cm: 100,
            actual_cm: 94,
            duration_ms: 4000,
            detail: "Obstacle".to_owned(),
        },
    });
    assert_round_trip(
        &drive,
        json!({
            "type": "event",
            "boot_id": 659_918,
            "seq": 5,
            "ts": 12346,
            "name": "drive_distance_completed",
            "v": {
                "command_seq": null,
                "status": "failed",
                "target_cm": 100,
                "actual_cm": 94,
                "duration_ms": 4000,
                "detail": "Obstacle"
            }
        }),
    );
}

/// The daemon's own overflow notice replays no wire event, so it carries no `(boot_id, seq)`.
#[test]
fn an_overflow_notice_round_trips() {
    let message = ServerMessage::Event(EventEntry {
        boot_id: None,
        seq: None,
        ts: 12_345,
        name: EventName::EventOverflow,
        v: EventValue::EventOverflow { dropped: 5 },
    });
    let value = json!({
        "type": "event",
        "boot_id": null,
        "seq": null,
        "ts": 12345,
        "name": "event_overflow",
        "v": {"dropped": 5},
    });
    assert_round_trip(&message, value);
}

/// A `command_result` is the ACK or rejection, never the drive outcome.
#[test]
fn a_command_result_round_trips() {
    let message = ServerMessage::CommandResult(CommandResult {
        command_seq: 1,
        command: "drive_distance".to_owned(),
        outcome: Outcome::Acked,
        status: Status::Ok,
        detail: None,
        ts: 12_345,
    });
    let value = json!({
        "type": "command_result",
        "command_seq": 1,
        "command": "drive_distance",
        "outcome": "acked",
        "status": "ok",
        "detail": null,
        "ts": 12345,
    });
    assert_round_trip(&message, value);
}

/// The client messages round-trip: hello, auth, pair, connect, disconnect, heartbeat.
#[test]
fn the_client_messages_round_trip() {
    assert_round_trip(
        &ClientMessage::Hello(ClientHello { protocol: PROTOCOL }),
        json!({"type": "hello", "protocol": 1}),
    );
    assert_round_trip(
        &ClientMessage::Auth(Auth {
            code: "AB2C-DE4F-GH6J".to_owned(),
        }),
        json!({"type": "auth", "code": "AB2C-DE4F-GH6J"}),
    );
    assert_round_trip(
        &ClientMessage::Pair(Pair {
            code: "ab2c de4f gh6j".to_owned(),
        }),
        json!({"type": "pair", "code": "ab2c de4f gh6j"}),
    );
    assert_round_trip(
        &ClientMessage::Connect(Connect {
            host: "192.168.1.42".to_owned(),
            port: 9000,
        }),
        json!({"type": "connect", "host": "192.168.1.42", "port": 9000}),
    );
    assert_round_trip(&ClientMessage::Disconnect, json!({"type": "disconnect"}));
    assert_round_trip(&ClientMessage::Heartbeat, json!({"type": "heartbeat"}));
}

/// A `command` frame keeps `args` raw so the relay validates it per command.
#[test]
fn a_command_frame_round_trips() {
    let message = ClientMessage::Command(CommandFrame {
        command_seq: 1,
        command: "rotate_by".to_owned(),
        args: json!({"angle_deg": 90.0, "speed": 50}),
    });
    let value = json!({
        "type": "command",
        "command_seq": 1,
        "command": "rotate_by",
        "args": {"angle_deg": 90.0, "speed": 50},
    });
    assert_round_trip(&message, value);
}

/// The catalogue's flag bits become booleans.
fn catalogue() -> Catalogue {
    Catalogue {
        procedures: vec![CatalogueProcedure {
            id: 2,
            label: "Motor calibration".to_owned(),
            available: true,
            interactive: false,
            calibration: true,
            takes_params: false,
        }],
    }
}

/// Serialises `message`, asserts the frame is exactly `value`, and reads the value back.
fn assert_round_trip<T>(message: &T, value: Value)
where
    T: serde::Serialize + serde::de::DeserializeOwned + PartialEq + std::fmt::Debug,
{
    let written = serde_json::to_value(message).unwrap_or(Value::Null);
    assert_eq!(written, value, "the wire frame must match the ADR's shape");
    let read: Result<T, _> = serde_json::from_value(value);
    assert_eq!(
        read.as_ref().err().map(ToString::to_string),
        None,
        "the frame must parse"
    );
    if let Ok(parsed) = read {
        assert_eq!(&parsed, message, "the frame must round-trip");
    }
}
