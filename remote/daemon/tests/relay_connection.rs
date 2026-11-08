//! The `connection{state}` lane the host wiring publishes onto (ADR-0039).
//!
//! The relay owns the app-facing state, so the `LinkState` mapping is broadcast from here: a
//! latest-wins slot every tab drains, delivered before the snapshot it describes. This test pins
//! the ordering and the one-per-update rule at the relay seam.

mod app_support;

use remote_daemon::{
    clock,
    relay::Relay,
    schema::{Connection, ConnectionState, ServerMessage},
};

/// Builds a connection message in `state`.
fn connection(state: ConnectionState) -> Connection {
    Connection {
        state,
        robot: None,
        detail: None,
        retry_in_ms: None,
        boot_id: None,
        ts: clock::now_ms(),
    }
}

/// Returns the connection messages in `messages`.
fn connections(messages: &[ServerMessage]) -> Vec<ConnectionState> {
    messages
        .iter()
        .filter_map(|message| match message {
            ServerMessage::Connection(connection) => Some(connection.state),
            _ => None,
        })
        .collect()
}

/// A connection update reaches every app once, and a new tab sees the latest state immediately.
#[test]
fn a_connection_update_is_broadcast_once_per_app() {
    let mut relay = Relay::new();
    let first = relay.connect_app(0);
    let second = relay.connect_app(0);

    relay.set_connection(connection(ConnectionState::Searching));
    let messages = relay.poll(first, 10);
    assert_eq!(connections(&messages), vec![ConnectionState::Searching]);
    assert!(
        matches!(messages.first(), Some(ServerMessage::Connection(_))),
        "the connection state precedes the snapshot it describes"
    );
    assert!(
        connections(&relay.poll(first, 11)).is_empty(),
        "an unchanged state is not repeated"
    );

    let messages = relay.poll(second, 12);
    assert_eq!(
        connections(&messages),
        vec![ConnectionState::Searching],
        "a tab that connects later sees the current state"
    );
}

/// The newest state wins; intermediate states are not queued.
#[test]
fn the_newest_connection_state_wins() {
    let mut relay = Relay::new();
    let app = relay.connect_app(0);
    let _ = relay.poll(app, 0);

    relay.set_connection(connection(ConnectionState::Connecting));
    relay.set_connection(connection(ConnectionState::Connected));
    let messages = relay.poll(app, 1);
    assert_eq!(connections(&messages), vec![ConnectionState::Connected]);
}

/// A Boot Id and retry hint travel with the state.
#[test]
fn the_connection_carries_its_detail_and_boot_id() {
    let mut relay = Relay::new();
    let app = relay.connect_app(0);
    let _ = relay.poll(app, 0);

    relay.set_connection(Connection {
        state: ConnectionState::Connected,
        robot: Some(String::from("Explorer-AAAA")),
        detail: Some(String::from("Selected from several robots")),
        retry_in_ms: None,
        boot_id: Some(42),
        ts: clock::now_ms(),
    });
    let messages = relay.poll(app, 1);
    let update = messages
        .iter()
        .find_map(|message| match message {
            ServerMessage::Connection(connection) => Some(connection.clone()),
            _ => None,
        })
        .unwrap_or_else(|| {
            assert_eq!(0, usize::MAX, "no connection update");
            std::process::abort()
        });
    assert_eq!(update.robot.as_deref(), Some("Explorer-AAAA"));
    assert_eq!(update.detail.as_deref(), Some("Selected from several robots"));
    assert_eq!(update.boot_id, Some(42));
}
