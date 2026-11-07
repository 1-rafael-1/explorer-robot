//! The heartbeat-gated command path: a fresh heartbeat admits a command, a stale one is rejected,
//! and the command itself never renews liveness (ADR-0039).

mod app_support;

use std::{sync::Arc, time::Duration};

use app_support::{
    FakeGate, TestServer, a_client_hello, connect, json_of_type, must, must_some, relay_with_session, send_json,
    wait_for_outbound,
};
use remote_protocol::Command;
use serde_json::{Value, json};

/// The `drive_distance` command frame.
fn drive(distance_cm: i16, command_seq: u64) -> Value {
    json!({
        "type": "command",
        "command_seq": command_seq,
        "command": "drive_distance",
        "args": {"distance_cm": distance_cm, "speed": 50}
    })
}

/// Connects, reads the greeting and snapshot, and completes the protocol handshake.
async fn greet(server: &TestServer) -> app_support::Client {
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;
    let _snapshot = json_of_type(&mut client, "snapshot").await;
    send_json(&mut client, &a_client_hello()).await;
    client
}

/// After two seconds without a heartbeat a command is rejected `stale`.
#[tokio::test]
async fn a_command_without_a_heartbeat_after_two_seconds_is_stale() {
    let server = must(
        TestServer::start(relay_with_session(7), FakeGate::empty()).await,
        "server",
    );
    let mut client = greet(&server).await;

    tokio::time::sleep(Duration::from_millis(2_100)).await;
    send_json(&mut client, &drive(10, 1)).await;

    let result = json_of_type(&mut client, "command_result").await;
    assert_eq!(result["command_seq"], 1);
    assert_eq!(result["outcome"], "rejected");
    assert_eq!(result["status"], "stale");
    assert_eq!(result["detail"], "App heartbeat is stale");
}

/// A heartbeat admits a command, and that command does not renew the app's liveness.
#[tokio::test]
async fn a_command_does_not_renew_liveness() {
    let relay = relay_with_session(7);
    let watched = Arc::clone(&relay);
    let server = must(TestServer::start(relay, FakeGate::empty()).await, "server");
    let mut client = greet(&server).await;

    send_json(&mut client, &json!({"type": "heartbeat"})).await;
    send_json(&mut client, &drive(10, 1)).await;
    let outbound = must_some(
        wait_for_outbound(&watched).await,
        "the command admitted by a fresh heartbeat",
    );
    assert_eq!(outbound.command, Command::DriveDistance);
    assert_eq!(outbound.wire_seq, 1);

    // No heartbeat since the first command: the command itself must not have refreshed it.
    tokio::time::sleep(Duration::from_millis(2_100)).await;
    send_json(&mut client, &drive(20, 2)).await;
    let result = json_of_type(&mut client, "command_result").await;
    assert_eq!(result["command_seq"], 2);
    assert_eq!(result["status"], "stale");
}

/// A heartbeat after a stale command restores liveness.
#[tokio::test]
async fn a_heartbeat_after_a_stale_command_renews_liveness() {
    let relay = relay_with_session(7);
    let watched = Arc::clone(&relay);
    let server = must(TestServer::start(relay, FakeGate::empty()).await, "server");
    let mut client = greet(&server).await;

    tokio::time::sleep(Duration::from_millis(2_100)).await;
    send_json(&mut client, &drive(10, 1)).await;
    let stale = json_of_type(&mut client, "command_result").await;
    assert_eq!(stale["status"], "stale");

    send_json(&mut client, &json!({"type": "heartbeat"})).await;
    send_json(&mut client, &drive(10, 2)).await;
    let outbound = must_some(wait_for_outbound(&watched).await, "the renewed command");
    assert_eq!(outbound.command, Command::DriveDistance);
    assert_eq!(outbound.wire_seq, 1);
}
