//! The Session hooks ticket 13 wires: first authentication, last disconnect, an app `disconnect`
//! and a manual endpoint (ADR-0039).

mod app_support;

use std::{sync::Arc, time::Duration};

use app_support::{
    FakeGate, RecordingHooks, TestServer, a_client_hello, connect, json_of_type, must, relay_with_session, send_json,
    wait_for_count,
};
use remote_daemon::app::{AppHooks, Assets, LoopbackFree};
use serde_json::json;

/// Starts a loopback server that records its hooks.
async fn recording_server(hooks: Arc<RecordingHooks>) -> TestServer {
    must(
        TestServer::start_full(
            relay_with_session(1),
            FakeGate::empty(),
            Arc::new(LoopbackFree),
            Assets::embedded(),
            hooks as Arc<dyn AppHooks>,
        )
        .await,
        "server",
    )
}

/// Opens a connection and waits until the server has admitted it (the snapshot proves it).
async fn admit(server: &TestServer) -> app_support::Client {
    let mut client = must(connect(server.addr()).await, "connect");
    let _hello = json_of_type(&mut client, "hello").await;
    let _snapshot = json_of_type(&mut client, "snapshot").await;
    send_json(&mut client, &a_client_hello()).await;
    client
}

/// The first authentication fires once, and the last disconnect fires only when the final tab
/// closes.
#[tokio::test]
async fn the_first_auth_and_last_disconnect_hooks_fire_once() {
    let hooks = Arc::new(RecordingHooks::default());
    let server = recording_server(Arc::clone(&hooks)).await;

    let mut first = admit(&server).await;
    assert_eq!(hooks.first_auths(), 1);

    let mut second = admit(&server).await;
    assert_eq!(hooks.first_auths(), 1, "a second tab must not reopen the Session");

    let _ = second.close(None).await;
    tokio::time::sleep(Duration::from_millis(200)).await;
    assert_eq!(
        hooks.last_disconnects(),
        0,
        "one tab is still live, so the Session stays open"
    );

    let _ = first.close(None).await;
    assert!(
        wait_for_count(|| hooks.last_disconnects(), 1).await,
        "the last close releases the Session"
    );
}

/// An app `disconnect` calls the hook without closing the socket.
#[tokio::test]
async fn a_disconnect_message_calls_the_hook() {
    let hooks = Arc::new(RecordingHooks::default());
    let server = recording_server(Arc::clone(&hooks)).await;
    let mut client = admit(&server).await;

    send_json(&mut client, &json!({"type": "disconnect"})).await;
    assert!(wait_for_count(|| hooks.disconnects(), 1).await);
}

/// A manual `connect` endpoint reaches the hook.
#[tokio::test]
async fn a_manual_endpoint_reaches_the_hook() {
    let hooks = Arc::new(RecordingHooks::default());
    let server = recording_server(Arc::clone(&hooks)).await;
    let mut client = admit(&server).await;

    send_json(
        &mut client,
        &json!({"type": "connect", "host": "192.0.2.10", "port": 9000}),
    )
    .await;
    assert!(wait_for_count(|| hooks.endpoints().len(), 1).await);
    assert_eq!(hooks.endpoints(), vec![(String::from("192.0.2.10"), 9000)]);
}
