//! The end-to-end host chain: daemon + Bridge Stub + headless WebSocket client.
//!
//! Each test binds discovery on an ephemeral loopback port, points the stub's Beacon there, and
//! drives the daemon's real [`Host`] wiring through a real `/ws` upgrade. The expected states and
//! details are ADR-0039's: discovery selects the Beacon, the link authenticates with a token from
//! the config store, and the app receives `connecting`, `connected`, a snapshot and the robot's
//! streams. The companions pin the honest failure states: Setup Mode does not connect, a manual
//! `host:port` works without broadcast, an unauthorized token is surfaced, and silence is
//! `no_robots`.

mod app_support;

use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::Arc,
    time::{Duration, Instant},
};

use app_support::{Client, Frame, a_client_hello, connect, must, must_some, next_frame, send_json};
use bridge_net::{
    beacon,
    identity::{DeviceId, Flags, Name},
};
use bridge_stub::{Scenario, StreamRate, Stub};
use link_protocol::Stream;
use remote_daemon::{
    app::Assets,
    config::Store,
    discovery::{Discovery, DiscoveryConfig},
    host::Host,
};
use serde_json::{Value, json};

/// The token the daemon stores and the stub verifies.
const TOKEN: &str = "pairing-token-01";

/// The stub's default device id, the key the token is stored under.
const DEVICE: [u8; 6] = *b"stub01";

/// Binds discovery on an ephemeral loopback port.
fn discovery() -> (Discovery, SocketAddr) {
    let discovery = must(
        Discovery::bind(DiscoveryConfig {
            bind: IpAddr::V4(Ipv4Addr::LOCALHOST),
            port: 0,
            last_device: None,
        }),
        "bind discovery",
    );
    let addr = must(discovery.local_addr(), "discovery address");
    (discovery, addr)
}

/// Starts a stub whose Beacon targets `beacon_target`, streaming motion quickly.
fn stub(beacon_target: SocketAddr, token: &str) -> Stub {
    stub_named(beacon_target, token, DEVICE, "Explorer-STUB")
}

/// Starts a stub with an explicit identity and the same fast motion script.
fn stub_named(beacon_target: SocketAddr, token: &str, device: [u8; 6], name: &str) -> Stub {
    let mut scenario = Scenario::new(token.as_bytes());
    scenario.identity.device_id = DeviceId::new(device);
    scenario.identity.name = must_some(Name::new(name), "stub name");
    scenario.beacon.target = beacon_target;
    scenario.beacon.interval_ms = 50;
    scenario.traffic.boot_id = 42;
    scenario.traffic.streams = vec![StreamRate {
        stream: Stream::Motion,
        rate_chz: 2_000,
    }];
    scenario.traffic.events = Vec::new();
    must(Stub::start(scenario), "start stub")
}

/// Opens a token store under a temporary directory and returns it with its directory.
fn store_with(device: Option<DeviceId>, token: &str) -> (tempfile::TempDir, Arc<Store>) {
    let dir = must(tempfile::tempdir(), "temp dir");
    let store = must(Store::open(dir.path().join("config.toml")), "open store");
    must(store.set_token(device, token), "store token");
    (dir, Arc::new(store))
}

/// Starts the wired daemon on an ephemeral loopback port.
async fn start_host(discovery: Discovery, store: Arc<Store>) -> remote_daemon::host::HostHandle {
    let host = Host {
        discovery,
        store,
        assets: Assets::embedded(),
        bind: IpAddr::V4(Ipv4Addr::LOCALHOST),
        port: Some(0),
        open_browser: false,
        verbose: false,
    };
    must(host.start().await, "start host")
}

/// Opens the app WebSocket, reads the greeting and sends the client `hello`.
async fn open_app(host: &remote_daemon::host::HostHandle) -> Client {
    let mut client = must(connect(host.addr()).await, "connect");
    let greeting = next_json(&mut client).await;
    assert_eq!(greeting["type"], "hello", "the server greets first");
    assert_eq!(greeting["protocol"], 1);
    assert_eq!(
        greeting["auth_required"], false,
        "a loopback peer needs no token (ADR-0039)"
    );
    send_json(&mut client, &a_client_hello()).await;
    client
}

/// Reads the next JSON frame, skipping control frames.
async fn next_json(client: &mut Client) -> Value {
    loop {
        match next_frame(client, Duration::from_secs(3)).await {
            Some(Frame::Json(value)) => return value,
            Some(Frame::Other) => {}
            Some(Frame::Close(code)) => {
                assert_eq!(
                    format!("{code:?}").len(),
                    usize::MAX,
                    "the socket closed before a JSON frame"
                );
                std::process::abort();
            }
            None => {
                assert_eq!(0, usize::MAX, "the socket produced no frame");
                std::process::abort();
            }
        }
    }
}

/// Polls the app until it reports a connection in `wanted`, or the budget elapses.
async fn wait_connection(client: &mut Client, wanted: &str, budget: Duration) -> bool {
    let deadline = Instant::now() + budget;
    while Instant::now() < deadline {
        let remaining = deadline.saturating_duration_since(Instant::now());
        match next_frame(client, remaining.min(Duration::from_millis(300))).await {
            Some(Frame::Json(value)) if value["type"] == "connection" && value["state"] == wanted => {
                return true;
            }
            Some(Frame::Close(_)) => return false,
            // A quiet read is not a closed socket: keep waiting inside the budget.
            Some(Frame::Json(_) | Frame::Other) | None => {}
        }
    }
    false
}

/// Polls the token store until `device` holds `token`, or the budget elapses.
async fn wait_token(store: &Store, device: DeviceId, token: &str, budget: Duration) -> bool {
    let deadline = Instant::now() + budget;
    while Instant::now() < deadline {
        if store.token_for(Some(device)).as_deref() == Some(token) {
            return true;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    false
}

/// Returns whether a snapshot carries a Motion sample.
fn has_motion(snapshot: &Value) -> bool {
    snapshot["streams"]
        .as_array()
        .is_some_and(|streams| streams.iter().any(|stream| stream["name"] == "motion"))
}

/// The full chain: discovery → preamble → `HELLO_ACK` → a snapshot and streams for the app.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_daemon_discovers_the_stub_and_serves_the_session() {
    let (discovery, beacon_target) = discovery();
    let _stub = stub(beacon_target, TOKEN);
    let (_dir, store) = store_with(Some(DeviceId::new(DEVICE)), TOKEN);
    let host = start_host(discovery, store).await;

    let mut client = open_app(&host).await;

    let mut connected = false;
    let mut session_snapshot = false;
    let mut saw_motion = false;
    let deadline = Instant::now() + Duration::from_secs(15);
    while Instant::now() < deadline && !(connected && session_snapshot && saw_motion) {
        let remaining = deadline.saturating_duration_since(Instant::now());
        let Some(Frame::Json(value)) = next_frame(&mut client, remaining.min(Duration::from_millis(500))).await else {
            continue;
        };
        match value["type"].as_str() {
            Some("connection") if value["state"] == "connected" => {
                connected = true;
                assert_eq!(value["boot_id"], 42, "HELLO_ACK's Boot Id reaches the app");
            }
            Some("snapshot") => {
                // A snapshot carrying a Motion sample can only exist after `HELLO_ACK` opened the
                // Session and the stub streamed behind it.
                session_snapshot |= has_motion(&value);
                saw_motion |= has_motion(&value);
            }
            Some("stream") => saw_motion |= value["name"] == "motion",
            _ => {}
        }
    }
    assert!(connected, "the Session's connection state did not reach the app");
    assert!(
        session_snapshot,
        "a snapshot from the opened Session did not reach the app"
    );
    assert!(saw_motion, "the stub's motion stream did not reach the app");

    must(host.stop().await, "stop host");
}

/// A portal-only Beacon is `setup_mode`, and the daemon does not connect.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_portal_only_beacon_is_setup_mode_and_does_not_connect() {
    let (discovery, beacon_target) = discovery();
    let mut scenario = Scenario::new(TOKEN.as_bytes());
    scenario.beacon.target = beacon_target;
    scenario.beacon.interval_ms = 50;
    scenario.beacon.tcp_port = Some(beacon::PORTAL_ONLY);
    let _stub = must(Stub::start(scenario), "start stub");
    let (_dir, store) = store_with(Some(DeviceId::new(DEVICE)), TOKEN);
    let host = start_host(discovery, store).await;

    let mut client = open_app(&host).await;
    assert!(
        wait_connection(&mut client, "setup_mode", Duration::from_secs(5)).await,
        "the portal-only Beacon was not reported as setup_mode"
    );
    assert!(
        !wait_connection(&mut client, "connected", Duration::from_millis(800)).await,
        "a setup_mode Beacon must not open a Session"
    );

    must(host.stop().await, "stop host");
}

/// The Setup Mode flag is the same honest state even when a Link port is advertised.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_setup_mode_flag_does_not_connect() {
    let (discovery, beacon_target) = discovery();
    let mut scenario = Scenario::new(TOKEN.as_bytes());
    scenario.beacon.target = beacon_target;
    scenario.beacon.interval_ms = 50;
    scenario.identity.flags = Flags::PROVISIONED | Flags::SETUP_MODE;
    let _stub = must(Stub::start(scenario), "start stub");
    let (_dir, store) = store_with(Some(DeviceId::new(DEVICE)), TOKEN);
    let host = start_host(discovery, store).await;

    let mut client = open_app(&host).await;
    assert!(
        wait_connection(&mut client, "setup_mode", Duration::from_secs(5)).await,
        "the Setup Mode flag was not reported"
    );
    assert!(
        !wait_connection(&mut client, "connected", Duration::from_millis(800)).await,
        "a setup_mode Bridge must not open a Session"
    );

    must(host.stop().await, "stop host");
}

/// A manual `host:port` connects with the `default_token` on a network that blocks broadcast.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_manual_endpoint_connects_without_a_beacon() {
    let (discovery, _beacon_target) = discovery();
    // The stub sends its Beacon nowhere useful, so only the manual endpoint can connect.
    let stub = stub(SocketAddr::from((Ipv4Addr::LOCALHOST, 9)), TOKEN);
    let (_dir, store) = store_with(None, TOKEN);
    let host = start_host(discovery, store).await;

    let mut client = open_app(&host).await;
    send_json(
        &mut client,
        &json!({
            "type": "connect",
            "host": "127.0.0.1",
            "port": stub.link_addr().port(),
        }),
    )
    .await;

    assert!(
        wait_connection(&mut client, "connected", Duration::from_secs(10)).await,
        "the manual endpoint did not open a Session"
    );

    must(host.stop().await, "stop host");
}

/// A wrong stored token is surfaced as `unauthorized` and never auto-retried.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_unauthorized_token_is_surfaced() {
    let (discovery, beacon_target) = discovery();
    let _stub = stub(beacon_target, "a-different-token");
    let (_dir, store) = store_with(Some(DeviceId::new(DEVICE)), TOKEN);
    let host = start_host(discovery, store).await;

    let mut client = open_app(&host).await;
    assert!(
        wait_connection(&mut client, "unauthorized", Duration::from_secs(5)).await,
        "a refused token was not surfaced as unauthorized"
    );
    let deadline = Instant::now() + Duration::from_millis(800);
    while Instant::now() < deadline {
        if let Some(Frame::Json(value)) = next_frame(&mut client, Duration::from_millis(200)).await
            && value["type"] == "connection"
        {
            assert_ne!(value["state"], "connecting", "unauthorized must not be auto-retried");
            assert_ne!(value["state"], "connected", "unauthorized must not open a Session");
        }
    }

    must(host.stop().await, "stop host");
}

/// Several Beacons with no history pick one deterministically and name it in the app's `detail`.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn several_beacons_pick_one_and_name_it_in_the_app_detail() {
    let (discovery, beacon_target) = discovery();
    let _first = stub_named(beacon_target, TOKEN, *b"stub01", "Explorer-AAAA");
    let _second = stub_named(beacon_target, TOKEN, *b"stub02", "Explorer-BBBB");
    let (_dir, store) = store_with(Some(DeviceId::new(*b"stub01")), TOKEN);
    must(store.set_token(Some(DeviceId::new(*b"stub02")), TOKEN), "second token");
    let host = start_host(discovery, store).await;

    // Let both Beacons land before the first authentication asks for a Session; otherwise the
    // daemon could honestly open the only one it had heard.
    tokio::time::sleep(Duration::from_millis(300)).await;
    let mut client = open_app(&host).await;

    let deadline = Instant::now() + Duration::from_secs(10);
    let mut named = false;
    while Instant::now() < deadline && !named {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if let Some(Frame::Json(value)) = next_frame(&mut client, remaining.min(Duration::from_millis(300))).await
            && value["type"] == "connection"
        {
            named = value["detail"]
                .as_str()
                .is_some_and(|detail| detail.contains("Several robots found") && detail.contains("Explorer-AAAA"));
        }
    }
    assert!(
        named,
        "the deterministic pick was not named in the app's connection detail"
    );

    must(host.stop().await, "stop host");
}

/// A discovered robot with no stored token is `pairing_required`, not a failed connection.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_discovered_robot_without_a_token_requires_pairing() {
    let (discovery, beacon_target) = discovery();
    let _stub = stub(beacon_target, TOKEN);
    let dir = must(tempfile::tempdir(), "temp dir");
    let store = must(Store::open(dir.path().join("config.toml")), "open store");
    let host = start_host(discovery, Arc::new(store)).await;

    let mut client = open_app(&host).await;
    assert!(
        wait_connection(&mut client, "pairing_required", Duration::from_secs(5)).await,
        "discovery without a stored token did not surface pairing_required"
    );
    assert!(
        !wait_connection(&mut client, "connected", Duration::from_millis(800)).await,
        "no Session may open without a token"
    );

    must(host.stop().await, "stop host");
}

/// A loopback app pairs after its hello, and the next Beacon opens the Session.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_loopback_app_pairs_after_hello_and_the_next_beacon_connects() {
    let (discovery, beacon_target) = discovery();
    let _stub = stub(beacon_target, TOKEN);
    let dir = must(tempfile::tempdir(), "temp dir");
    let store = Arc::new(must(Store::open(dir.path().join("config.toml")), "open store"));
    let host = start_host(discovery, Arc::clone(&store)).await;

    let mut client = open_app(&host).await;
    assert!(
        wait_connection(&mut client, "pairing_required", Duration::from_secs(5)).await,
        "the discovered robot was not reported as pairing_required"
    );

    // No pre-seeded config: the app pairs over loopback after its hello, and the token is filed
    // under the device the Beacon recorded.
    send_json(&mut client, &json!({"type": "pair", "code": TOKEN})).await;
    assert!(
        wait_token(&store, DeviceId::new(DEVICE), TOKEN, Duration::from_secs(5)).await,
        "the loopback pair message did not store the token under the last device"
    );
    assert!(
        wait_connection(&mut client, "connected", Duration::from_secs(10)).await,
        "the next Beacon after pairing did not open the Session"
    );

    must(host.stop().await, "stop host");
}

/// Silence is `no_robots` after ADR-0039's 3 s startup budget, never a guessed Setup Mode.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn silence_becomes_no_robots() {
    let (discovery, _beacon_target) = discovery();
    let (_dir, store) = store_with(Some(DeviceId::new(DEVICE)), TOKEN);
    let host = start_host(discovery, store).await;

    let mut client = open_app(&host).await;
    assert!(
        wait_connection(&mut client, "searching", Duration::from_secs(2)).await,
        "the daemon starts by searching"
    );
    assert!(
        wait_connection(&mut client, "no_robots", Duration::from_secs(5)).await,
        "silence did not settle into no_robots"
    );

    must(host.stop().await, "stop host");
}
