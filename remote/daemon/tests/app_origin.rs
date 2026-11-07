//! The WebSocket upgrade's `Origin` gate (ADR-0039): same-Host and the Tauri shell are accepted,
//! a cross-origin page is refused before the upgrade, and a missing `Origin` is allowed.

mod app_support;

use app_support::{
    FakeGate, TestServer, connect, connect_with_origin, json_of_type, must, must_some, relay_with_power,
};
use tokio_tungstenite::tungstenite::Error as WsError;

/// A missing `Origin` is allowed for local non-browser tools on loopback.
#[tokio::test]
async fn a_missing_origin_is_allowed_on_loopback() {
    let server = must(
        TestServer::start(relay_with_power(1, 5, 12_000), FakeGate::empty()).await,
        "server",
    );
    let mut client = must(connect(server.addr()).await, "connect");
    let hello = json_of_type(&mut client, "hello").await;
    assert_eq!(hello["type"], "hello");
}

/// A page whose `Origin` matches the request's `Host` may upgrade.
#[tokio::test]
async fn a_same_host_origin_is_accepted() {
    let server = must(
        TestServer::start(relay_with_power(1, 5, 12_000), FakeGate::empty()).await,
        "server",
    );
    let origin = format!("http://{}", server.addr());
    let mut client = must(connect_with_origin(server.addr(), &origin).await, "connect");
    let hello = json_of_type(&mut client, "hello").await;
    assert_eq!(hello["type"], "hello");
}

/// An `https` page origin matching the `Host` may upgrade too.
#[tokio::test]
async fn an_https_same_host_origin_is_accepted() {
    let server = must(
        TestServer::start(relay_with_power(1, 5, 12_000), FakeGate::empty()).await,
        "server",
    );
    let origin = format!("https://{}", server.addr());
    let mut client = must(connect_with_origin(server.addr(), &origin).await, "connect");
    let hello = json_of_type(&mut client, "hello").await;
    assert_eq!(hello["type"], "hello");
}

/// Both Tauri shell origins are accepted regardless of `Host`.
#[tokio::test]
async fn the_tauri_origins_are_accepted() {
    let server = must(
        TestServer::start(relay_with_power(1, 5, 12_000), FakeGate::empty()).await,
        "server",
    );
    for origin in ["tauri://localhost", "http://tauri.localhost"] {
        let mut client = must(connect_with_origin(server.addr(), origin).await, "tauri origin");
        let hello = json_of_type(&mut client, "hello").await;
        assert_eq!(hello["type"], "hello", "origin {origin}");
    }
}

/// A cross-origin page is refused before the WebSocket is accepted.
#[tokio::test]
async fn a_cross_origin_upgrade_is_rejected() {
    let server = must(
        TestServer::start(relay_with_power(1, 5, 12_000), FakeGate::empty()).await,
        "server",
    );
    let error = must_some(
        connect_with_origin(server.addr(), "http://evil.example").await.err(),
        "cross-origin upgrade should fail",
    );
    let WsError::Http(response) = error else {
        eprintln!("expected an HTTP rejection, got {error}");
        std::process::abort();
    };
    assert_eq!(response.status().as_u16(), 403);
}

/// An `Origin` with a different port from the `Host` is cross-origin and refused.
#[tokio::test]
async fn a_different_port_origin_is_rejected() {
    let server = must(
        TestServer::start(relay_with_power(1, 5, 12_000), FakeGate::empty()).await,
        "server",
    );
    let origin = format!("http://{}:1", server.addr().ip());
    let error = must_some(
        connect_with_origin(server.addr(), &origin).await.err(),
        "different-port origin should fail",
    );
    assert!(matches!(error, WsError::Http(_)));
}
