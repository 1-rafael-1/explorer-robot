//! The daemon's link against the Bridge Stub, end to end (ticket 11's TCP/preamble seam).
//!
//! The stub is the real ticket-09 library, started in-process on loopback. The test asserts the
//! behaviours ADR-0039 fixes: the preamble completes, `HELLO_ACK` opens the Session, scripted
//! streams and events reach the relay, an event is acknowledged only after it is stored (the
//! second scripted event can only flow once the first is `ACKed`), and a command round-trips with a
//! configurable ACK status.

use std::{
    sync::{Arc, Mutex, MutexGuard, PoisonError},
    time::{Duration, Instant},
};

use bridge_stub::{Deny, Scenario, ScriptEvent, StreamRate, Stub, preamble::DenyReason};
use link_protocol::{Command, OBSTACLE_SOURCE_LIDAR, RetryPolicy, Status as WireStatus, Stream};
use remote_daemon::{
    link::{Link, LinkConfig, LinkHandle, LinkState},
    relay::Relay,
    schema::{CommandFrame, EventName, Outcome, ServerMessage, Status, StreamName},
};

/// The token the tests pair with.
const TOKEN: &[u8] = b"pairing-token-01";

/// A fixed epoch, so the test is reproducible.
const EPOCH: u32 = 0x1234_5678;

/// A relay timestamp far enough ahead to keep the test's app heartbeat fresh for the link's clock.
const TEST_NOW: u64 = 1_000_000;

/// Locks the shared relay, recovering the guard if another thread panicked.
fn lock(relay: &Mutex<Relay>) -> MutexGuard<'_, Relay> {
    relay.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Extracts a value, failing the test rather than unwrapping (the workspace denies `unwrap`).
fn must<T>(value: Option<T>, what: &str) -> T {
    value.unwrap_or_else(|| {
        assert_eq!(what.len(), usize::MAX, "missing {what}");
        std::process::abort()
    })
}

/// Builds a fast config over the stub's endpoint.
fn config(stub: &Stub) -> LinkConfig {
    let mut config = LinkConfig::with_epoch(stub.link_addr(), TOKEN, EPOCH);
    config.connect_timeout = Duration::from_secs(1);
    config.hello_retry = Duration::from_millis(200);
    config.hello_attempts = 5;
    config.bring_up_budget = Duration::from_millis(700);
    config.query_timeout = Duration::from_millis(300);
    config.tick_interval = Duration::from_millis(20);
    config.ping_interval = Duration::from_millis(100);
    config.stall = Duration::from_millis(250);
    config.jitter = 0.0;
    config
}

/// Polls `predicate` until it is true or `timeout` elapses.
async fn wait_for(timeout: Duration, mut predicate: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if predicate() {
            return true;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    predicate()
}

/// Polls until the link reports `Connected`.
async fn wait_connected(handle: &LinkHandle) -> bool {
    wait_for(Duration::from_secs(5), || {
        matches!(handle.state(), LinkState::Connected { .. })
    })
    .await
}

/// Starts the stub and returns it, or fails the test.
fn start(scenario: Scenario) -> Stub {
    match Stub::start(scenario) {
        Ok(stub) => stub,
        Err(error) => {
            assert_eq!(format!("{error}").len(), usize::MAX, "the stub did not start");
            std::process::abort()
        }
    }
}

/// The full Session: preamble, `HELLO_ACK`, streams, ACK-gated events and a command round-trip.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_link_opens_a_session_and_relays_streams_events_and_commands() {
    let mut scenario = Scenario::new(TOKEN);
    scenario.traffic.boot_id = 42;
    scenario.traffic.streams = vec![
        StreamRate {
            stream: Stream::Motion,
            rate_chz: 2_000,
        },
        StreamRate {
            stream: Stream::Activity,
            rate_chz: 100,
        },
    ];
    scenario.traffic.events = vec![
        ScriptEvent::Obstacle {
            detected: 1,
            source: OBSTACLE_SOURCE_LIDAR,
        },
        ScriptEvent::FloorDrop { detected: 1 },
    ];
    // The first event's retransmission is far away, so the second can only flow once the first is
    // acknowledged.
    scenario.traffic.event_retry = RetryPolicy::new(30_000, 1, 30_000, 1);
    scenario.traffic.command_acks = vec![(Command::StopMotion, WireStatus::Rejected)];
    let stub = start(scenario);
    let relay = Arc::new(Mutex::new(Relay::new()));
    let app = lock(&relay).connect_app(0);
    let handle = Link::new(config(&stub), Arc::clone(&relay)).spawn();

    assert!(
        wait_connected(&handle).await,
        "the Session did not come up: {:?}",
        handle.state()
    );
    {
        let guard = lock(&relay);
        assert_eq!(guard.boot_id(), Some(42), "HELLO_ACK's Boot Id opened the Session");
        assert!(guard.is_connected(), "the relay sees an open Session");
        drop(guard);
    }

    let mut seen_motion = false;
    let mut seen_obstacle = false;
    let mut seen_floor_drop = false;
    let relayed = wait_for(Duration::from_secs(5), || {
        let messages = lock(&relay).poll(app, TEST_NOW);
        for message in messages {
            match message {
                ServerMessage::Stream(entry) if entry.name == StreamName::Motion => seen_motion = true,
                ServerMessage::Event(entry) if entry.name == EventName::Obstacle => seen_obstacle = true,
                ServerMessage::Event(entry) if entry.name == EventName::FloorDrop => seen_floor_drop = true,
                ServerMessage::Snapshot(snapshot) => {
                    seen_motion |= snapshot.streams.iter().any(|entry| entry.name == StreamName::Motion);
                    seen_obstacle |= snapshot.events.iter().any(|entry| entry.name == EventName::Obstacle);
                    seen_floor_drop |= snapshot.events.iter().any(|entry| entry.name == EventName::FloorDrop);
                }
                _ => {}
            }
        }
        seen_motion && seen_obstacle && seen_floor_drop
    })
    .await;
    assert!(
        relayed,
        "streams and ACK-gated events did not reach the relay: motion {seen_motion}, \
         obstacle {seen_obstacle}, floor drop {seen_floor_drop}"
    );

    {
        let mut guard = lock(&relay);
        guard.heartbeat(app, TEST_NOW);
        let command = CommandFrame {
            command_seq: 1,
            command: "stop_motion".to_owned(),
            args: serde_json::json!({"brake": true}),
        };
        assert!(
            guard.submit_command(app, command, TEST_NOW).is_none(),
            "the command was accepted"
        );
        drop(guard);
    }
    let mut result = None;
    let answered = wait_for(Duration::from_secs(5), || {
        let messages = lock(&relay).poll(app, TEST_NOW);
        for message in messages {
            if let ServerMessage::CommandResult(command) = message
                && command.command_seq == 1
            {
                result = Some(command);
                return true;
            }
        }
        false
    })
    .await;
    assert!(answered, "the command did not round-trip");
    let result = must(result, "command result");
    assert_eq!(
        result.outcome,
        Outcome::Rejected,
        "the stub's scripted status was served"
    );
    assert_eq!(result.status, Status::Rejected);

    handle.stop();
    handle.wait().await;
}

/// A stall with no robot frames is `reconnecting`, and the link retries (ADR-0039).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_silent_robot_becomes_reconnecting() {
    let mut scenario = Scenario::new(TOKEN);
    scenario.traffic.streams = Vec::new();
    scenario.traffic.events = Vec::new();
    let stub = start(scenario);
    let relay = Arc::new(Mutex::new(Relay::new()));
    let app = lock(&relay).connect_app(0);
    let handle = Link::new(config(&stub), Arc::clone(&relay)).spawn();

    assert!(
        wait_connected(&handle).await,
        "the Session did not come up: {:?}",
        handle.state()
    );
    // With no app live, PING stops and the idle robot sends nothing.
    lock(&relay).disconnect_app(app);
    assert!(
        wait_for(Duration::from_secs(2), || matches!(
            handle.state(),
            LinkState::Reconnecting { .. }
        ))
        .await,
        "a silent robot did not become reconnecting: {:?}",
        handle.state()
    );
    assert!(
        !lock(&relay).is_connected(),
        "the relay sees the Session closed while reconnecting"
    );
    assert!(!handle.is_finished(), "the link keeps retrying");

    handle.stop();
    handle.wait().await;
}

/// `unauthorized` is surfaced and never auto-retried (ADR-0039).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn unauthorized_is_surfaced_and_not_retried() {
    let mut scenario = Scenario::new(TOKEN);
    scenario.deny = Some(Deny {
        reason: DenyReason::Unauthorized,
        retry_ms: None,
    });
    let stub = start(scenario);
    let relay = Arc::new(Mutex::new(Relay::new()));
    let handle = Link::new(config(&stub), Arc::clone(&relay)).spawn();

    assert!(
        wait_for(Duration::from_secs(5), || matches!(
            handle.state(),
            LinkState::Unauthorized { .. }
        ))
        .await,
        "an unauthorized denial was not surfaced: {:?}",
        handle.state()
    );
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert!(handle.is_finished(), "unauthorized is never auto-retried");
    assert!(!lock(&relay).is_connected(), "no Session was opened");
    handle.wait().await;
}

/// `setup_mode` does not connect (ADR-0039).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn setup_mode_does_not_connect() {
    let mut scenario = Scenario::new(TOKEN);
    scenario.deny = Some(Deny {
        reason: DenyReason::SetupMode,
        retry_ms: None,
    });
    let stub = start(scenario);
    let relay = Arc::new(Mutex::new(Relay::new()));
    let handle = Link::new(config(&stub), Arc::clone(&relay)).spawn();

    assert!(
        wait_for(Duration::from_secs(5), || matches!(
            handle.state(),
            LinkState::SetupMode { .. }
        ))
        .await,
        "a setup-mode denial was not surfaced: {:?}",
        handle.state()
    );
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert!(handle.is_finished(), "setup mode does not connect");
    assert!(!lock(&relay).is_connected(), "no Session was opened");
    handle.wait().await;
}

/// `busy` retries after its hint instead of surfacing a fatal state (ADR-0039).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn busy_is_retried() {
    let mut scenario = Scenario::new(TOKEN);
    scenario.deny = Some(Deny {
        reason: DenyReason::Busy,
        retry_ms: Some(100),
    });
    let stub = start(scenario);
    let relay = Arc::new(Mutex::new(Relay::new()));
    let handle = Link::new(config(&stub), Arc::clone(&relay)).spawn();

    assert!(
        wait_for(Duration::from_secs(5), || matches!(
            handle.state(),
            LinkState::Busy { .. }
        ))
        .await,
        "a busy denial was not surfaced: {:?}",
        handle.state()
    );
    tokio::time::sleep(Duration::from_millis(400)).await;
    assert!(!handle.is_finished(), "busy is retried, not fatal");
    assert!(!lock(&relay).is_connected(), "the retries never opened a Session");

    handle.stop();
    handle.wait().await;
}
