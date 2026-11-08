//! The `/ws` seam: `hello`, the relay's snapshot, the protocol close codes and shutdown.
//!
//! The client is a real WebSocket over a real TCP connection to an ephemeral loopback port, so
//! these tests observe exactly what the app sees (ADR-0038, ADR-0039).

mod app_support;

use std::sync::Arc;

use app_support::{
    FakeGate, TestServer, a_client_hello, close_code, connect, json_of_type, must, must_some, relay_with_power,
    send_json, wait_for_outbound,
};
use remote_protocol::Command;
use serde_json::json;

/// A headless client receives the server `hello` and then a snapshot through the relay.
#[tokio::test]
async fn a_headless_client_receives_hello_and_a_snapshot_through_the_relay() {
    let relay = relay_with_power(0xA11CE, 5, 12_000);
    let server = must(TestServer::start(relay, FakeGate::empty()).await, "server");
    let mut client = must(connect(server.addr()).await, "connect");

    let hello = json_of_type(&mut client, "hello").await;
    assert_eq!(hello["protocol"], 1);
    assert_eq!(hello["server"]["name"], "explorer-remote");
    assert_eq!(hello["robot"]["model"], serde_json::Value::Null);
    assert_eq!(hello["auth_required"], false);

    let snapshot = json_of_type(&mut client, "snapshot").await;
    assert_eq!(snapshot["boot_id"], 0xA11CE);
    assert!(
        snapshot["streams"]
            .as_array()
            .is_some_and(|streams| streams.iter().any(|entry| {
                entry["name"] == "power" && entry["v"]["level"] == 5 && entry["v"]["voltage_mv"] == 12_000
            })),
        "snapshot should carry the relay's power slot: {snapshot}"
    );

    // The protocol handshake completes normally and leaves the socket open.
    send_json(&mut client, &a_client_hello()).await;
}

/// A client `hello` whose protocol is not 1 closes with 4426.
#[tokio::test]
async fn a_protocol_mismatch_closes_4426() {
    let server = must(
        TestServer::start(relay_with_power(1, 5, 12_000), FakeGate::empty()).await,
        "server",
    );
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;

    send_json(&mut client, &json!({"type": "hello", "protocol": 2})).await;
    assert_eq!(close_code(&mut client).await, 4426);
}

/// A first message that is not `hello` is a missing hello and closes with 4426.
#[tokio::test]
async fn a_missing_hello_closes_4426() {
    let server = must(
        TestServer::start(relay_with_power(1, 5, 12_000), FakeGate::empty()).await,
        "server",
    );
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;

    send_json(&mut client, &json!({"type": "heartbeat"})).await;
    assert_eq!(close_code(&mut client).await, 4426);
}

/// Shutdown closes every open socket with 1001.
#[tokio::test]
async fn shutdown_closes_1001() {
    let server = must(
        TestServer::start(relay_with_power(1, 5, 12_000), FakeGate::empty()).await,
        "server",
    );
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;
    let _snapshot = json_of_type(&mut client, "snapshot").await;
    send_json(&mut client, &a_client_hello()).await;

    server.shutdown();
    assert_eq!(close_code(&mut client).await, 1001);
}

/// An unknown message type is ignored, as ADR-0038's additive schema requires.
#[tokio::test]
async fn an_unknown_message_type_is_ignored() {
    let relay = relay_with_power(1, 5, 12_000);
    let watched = Arc::clone(&relay);
    let server = must(TestServer::start(relay, FakeGate::empty()).await, "server");
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;
    let _snapshot = json_of_type(&mut client, "snapshot").await;
    send_json(&mut client, &a_client_hello()).await;

    send_json(&mut client, &json!({"type": "future_gadget", "value": 7})).await;
    send_json(&mut client, &json!({"type": "heartbeat"})).await;
    send_json(
        &mut client,
        &json!({
            "type": "command",
            "command_seq": 1,
            "command": "drive_distance",
            "args": {"distance_cm": 10, "speed": 50}
        }),
    )
    .await;

    let outbound = must_some(wait_for_outbound(&watched).await, "command after an unknown type");
    assert_eq!(outbound.command, Command::DriveDistance);
}
