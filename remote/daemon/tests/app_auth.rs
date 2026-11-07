//! The per-peer-address auth gate: loopback is free, a gated peer must `auth` or `pair`, and
//! failures are constant-time compared and rate-limited per IP (ADR-0037, ADR-0039).
//!
//! The tests inject a policy that gates the loopback test client, so the LAN paths are exercised
//! without a second network interface; the production [`remote_daemon::app::LoopbackFree`] rule is
//! the default.

mod app_support;

use std::time::Duration;

use app_support::{
    FakeGate, TestServer, close_code, close_code_within, connect, gate_all, json_of_type, must, relay_with_power,
    send_json,
};
use serde_json::json;

/// Starts a server whose policy gates every peer.
async fn gated(gate: std::sync::Arc<FakeGate>) -> TestServer {
    must(
        TestServer::start_with(
            relay_with_power(1, 5, 12_000),
            gate,
            gate_all(),
            remote_daemon::app::Assets::embedded(),
        )
        .await,
        "server",
    )
}

/// A gated peer sees `auth_required: true`, unlike a loopback peer.
#[tokio::test]
async fn a_gated_peer_sees_auth_required() {
    let server = gated(FakeGate::empty()).await;
    let mut client = must(connect(server.addr()).await, "connect");
    let hello = json_of_type(&mut client, "hello").await;
    assert_eq!(hello["auth_required"], true);
    assert_eq!(hello["protocol"], 1);
    assert_eq!(hello["server"]["name"], "explorer-remote");
}

/// A gated peer that sends the stored token is admitted and receives the snapshot.
#[tokio::test]
async fn a_gated_peer_with_the_right_token_is_admitted() {
    let server = gated(FakeGate::with_token("ABCD2345EFGH")).await;
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;

    send_json(&mut client, &json!({"type": "auth", "code": "ABCD2345EFGH"})).await;
    let snapshot = json_of_type(&mut client, "snapshot").await;
    assert_eq!(snapshot["boot_id"], 1);
}

/// A missing `Origin` is allowed after auth on the LAN (the test client sends none).
#[tokio::test]
async fn a_missing_origin_is_allowed_after_auth_on_the_lan() {
    let server = gated(FakeGate::with_token("ABCD2345EFGH")).await;
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;
    send_json(&mut client, &json!({"type": "auth", "code": "ABCD2345EFGH"})).await;
    let _snapshot = json_of_type(&mut client, "snapshot").await;
}

/// A wrong token closes with 4401 after an `unauthorized` connection message.
#[tokio::test]
async fn a_gated_peer_with_a_wrong_token_closes_4401() {
    let server = gated(FakeGate::with_token("ABCD2345EFGH")).await;
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;

    send_json(&mut client, &json!({"type": "auth", "code": "WRONG2345EFGH"})).await;
    let notice = json_of_type(&mut client, "connection").await;
    assert_eq!(notice["state"], "unauthorized");
    assert_eq!(close_code(&mut client).await, 4401);
}

/// A gated peer's first message must be `auth` or `pair`; anything else closes 4401.
#[tokio::test]
async fn a_gated_peer_that_sends_anything_else_closes_4401() {
    let server = gated(FakeGate::with_token("ABCD2345EFGH")).await;
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;

    send_json(&mut client, &json!({"type": "heartbeat"})).await;
    assert_eq!(close_code(&mut client).await, 4401);
}

/// A malformed first message is also "anything else" and closes 4401.
#[tokio::test]
async fn a_malformed_auth_message_closes_4401() {
    let server = gated(FakeGate::with_token("ABCD2345EFGH")).await;
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;

    send_json(&mut client, &json!({"type": "auth"})).await;
    assert_eq!(close_code(&mut client).await, 4401);
}

/// Five seconds of silence closes 4401 (ADR-0037's deadline).
#[tokio::test]
async fn a_gated_peer_that_stays_silent_closes_4401_after_five_seconds() {
    let server = gated(FakeGate::with_token("ABCD2345EFGH")).await;
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;

    assert_eq!(close_code_within(&mut client, Duration::from_secs(8)).await, 4401);
}

/// `pair` stores the new code and admits its sender.
#[tokio::test]
async fn a_gated_peer_can_pair() {
    let gate = FakeGate::empty();
    let server = gated(std::sync::Arc::clone(&gate)).await;
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;

    send_json(&mut client, &json!({"type": "pair", "code": "ABCD2345EFGH"})).await;
    let snapshot = json_of_type(&mut client, "snapshot").await;
    assert_eq!(snapshot["boot_id"], 1);
    assert_eq!(gate.pairs(), vec![String::from("ABCD2345EFGH")]);
}

/// Five failures inside the window refuse the address for a minute, even with the right token.
#[tokio::test]
async fn five_failed_tokens_refuse_the_address() {
    let server = gated(FakeGate::with_token("ABCD2345EFGH")).await;

    for attempt in 0..5 {
        let mut client = must(connect(server.addr()).await, "connect");
        let _hello = json_of_type(&mut client, "hello").await;
        send_json(&mut client, &json!({"type": "auth", "code": "WRONG2345EFGH"})).await;
        assert_eq!(
            close_code(&mut client).await,
            4401,
            "wrong token attempt {attempt} should close 4401"
        );
    }

    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;
    send_json(&mut client, &json!({"type": "auth", "code": "ABCD2345EFGH"})).await;
    assert_eq!(
        close_code(&mut client).await,
        4401,
        "the refusal window must override a correct token"
    );
}

/// A success resets the failure count, as the Bridge's gate does (ADR-0037).
#[tokio::test]
async fn a_success_resets_the_failure_count() {
    let server = gated(FakeGate::with_token("ABCD2345EFGH")).await;

    for attempt in 0..4 {
        let mut client = must(connect(server.addr()).await, "connect");
        let _hello = json_of_type(&mut client, "hello").await;
        send_json(&mut client, &json!({"type": "auth", "code": "WRONG2345EFGH"})).await;
        assert_eq!(
            close_code(&mut client).await,
            4401,
            "wrong token attempt {attempt} should close 4401"
        );
    }

    let mut good = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut good, "hello").await;
    send_json(&mut good, &json!({"type": "auth", "code": "ABCD2345EFGH"})).await;
    let _snapshot = json_of_type(&mut good, "snapshot").await;

    for attempt in 0..4 {
        let mut client = must(connect(server.addr()).await, "connect");
        let _hello = json_of_type(&mut client, "hello").await;
        send_json(&mut client, &json!({"type": "auth", "code": "WRONG2345EFGH"})).await;
        assert_eq!(
            close_code(&mut client).await,
            4401,
            "post-reset wrong token attempt {attempt} should close 4401"
        );
    }

    let mut good = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut good, "hello").await;
    send_json(&mut good, &json!({"type": "auth", "code": "ABCD2345EFGH"})).await;
    let _snapshot = json_of_type(&mut good, "snapshot").await;
}
