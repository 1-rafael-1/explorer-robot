//! Bridge telemetry: relay storage, the poll lane and Boot Id independence.

mod support;

use link_protocol::{
    BridgeStation as WireBridgeStation, BridgeStatus as WireBridgeStatus, BridgeStatusFlags, StationSsid,
};
use remote_daemon::{
    relay::Relay,
    schema::{BridgeStation, BridgeStatus, ServerMessage},
};
use support::{must, session_with_app, snapshot};

/// A wire status with every field set, one flag set and one clear.
fn wire_status() -> WireBridgeStatus {
    WireBridgeStatus {
        rssi_dbm: Some(-54),
        channel: Some(6),
        station: WireBridgeStation::Associated,
        flags: BridgeStatusFlags::PROVISIONED | BridgeStatusFlags::LINK_READY,
        uart_dropped_bytes: 1,
        dma_stalls: 2,
        crc_failures: 3,
        resyncs: 4,
        wdt_feeds: 5,
        wdt_withheld: 6,
        ssid: None,
    }
}

/// A wire status whose Station Network is `ssid`, or unknown for [`None`].
fn wire_status_with_ssid(ssid: Option<&[u8]>) -> WireBridgeStatus {
    WireBridgeStatus {
        ssid: ssid.and_then(StationSsid::new),
        ..wire_status()
    }
}

/// The schema conversion of [`wire_status`] at `ts`.
const fn expected(ts: u64) -> BridgeStatus {
    BridgeStatus {
        ts,
        rssi_dbm: Some(-54),
        channel: Some(6),
        station: BridgeStation::Associated,
        ssid: None,
        provisioned: true,
        setup_open: false,
        link_ready: true,
        uart_dropped_bytes: 1,
        dma_stalls: 2,
        crc_failures: 3,
        resyncs: 4,
        wdt_feeds: 5,
        wdt_withheld: 6,
    }
}

/// Returns every `bridge` message from a poll.
fn bridges(messages: &[ServerMessage]) -> Vec<BridgeStatus> {
    messages
        .iter()
        .filter_map(|message| match message {
            ServerMessage::Bridge(status) => Some(status.clone()),
            _ => None,
        })
        .collect()
}

/// Every tab receives the converted status exactly once, and a later poll does not repeat it.
#[test]
fn a_bridge_status_is_emitted_once_per_app() {
    let mut relay = Relay::new();
    relay.session_up(7, 0);
    let first = relay.connect_app(0);
    let second = relay.connect_app(0);
    let _ = relay.poll(first, 0);
    let _ = relay.poll(second, 0);

    relay.on_bridge_status(&wire_status(), 10);

    let messages = relay.poll(first, 11);
    assert_eq!(bridges(&messages), vec![expected(10)]);
    assert!(relay.poll(first, 12).is_empty(), "the cursor suppresses a duplicate");

    let messages = relay.poll(second, 13);
    assert_eq!(bridges(&messages), vec![expected(10)], "each tab sees its own copy");
    assert_eq!(relay.poll(second, 14), Vec::new());
}

/// A snapshot carries the latest status and advances the app past it.
#[test]
fn a_snapshot_carries_the_bridge_status() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.poll(app, 0);
    relay.on_bridge_status(&wire_status(), 10);

    let assembled = must(relay.snapshot(app, 20), "snapshot");
    assert_eq!(assembled.bridge, Some(expected(10)));
    assert!(relay.poll(app, 21).is_empty(), "the snapshot already carried it");
}

/// A robot Boot Id change leaves the Bridge status untouched.
#[test]
fn a_boot_id_change_keeps_the_bridge_status() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    relay.on_bridge_status(&wire_status(), 10);
    let _ = relay.poll(app, 11);

    relay.session_up(8, 100);
    let messages = relay.poll(app, 110);
    let assembled = must(snapshot(&messages), "snapshot after the Boot Id change");
    assert_eq!(assembled.boot_id, Some(8));
    assert_eq!(
        assembled.bridge,
        Some(expected(10)),
        "the Bridge status is independent of the robot Boot Id"
    );
}

/// Before the first Bridge status the snapshot carries none, so the app has nothing to invent.
#[test]
fn a_snapshot_before_the_first_status_carries_no_bridge() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);

    let messages = relay.poll(app, 0);
    let assembled = must(snapshot(&messages), "snapshot");
    assert_eq!(assembled.bridge, None);
    assert!(
        bridges(&relay.poll(app, 1)).is_empty(),
        "no bridge message before the first status"
    );
}

/// The Station Network reaches the live message and the snapshot unchanged from the wire.
#[test]
fn a_station_network_reaches_the_app_unchanged() {
    const SSID: &str = "Home Network 0123456789 abcdefgh";
    assert_eq!(SSID.len(), 32, "the fixture is a maximum-length SSID");

    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.poll(app, 0);
    relay.on_bridge_status(&wire_status_with_ssid(Some(SSID.as_bytes())), 10);

    let messages = relay.poll(app, 11);
    let live = must(bridges(&messages).into_iter().next(), "live bridge message");
    assert_eq!(
        live.ssid.as_deref(),
        Some(SSID),
        "the live message keeps the value verbatim"
    );

    let assembled = must(relay.snapshot(app, 12), "snapshot");
    let bridge = must(assembled.bridge, "snapshot bridge");
    assert_eq!(
        bridge.ssid.as_deref(),
        Some(SSID),
        "the snapshot keeps the value verbatim"
    );
}

/// A later status replaces the Station Network, and an unknown SSID clears it to `None`.
#[test]
fn a_later_status_replaces_the_station_network() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.poll(app, 0);

    relay.on_bridge_status(&wire_status_with_ssid(Some(b"Home")), 10);
    let messages = relay.poll(app, 11);
    let live = must(bridges(&messages).into_iter().next(), "first bridge message");
    assert_eq!(live.ssid.as_deref(), Some("Home"));

    relay.on_bridge_status(&wire_status_with_ssid(Some(b"Rover Lab")), 20);
    let messages = relay.poll(app, 21);
    let live = must(bridges(&messages).into_iter().next(), "second bridge message");
    assert_eq!(
        live.ssid.as_deref(),
        Some("Rover Lab"),
        "a re-provisioning replaces the name"
    );

    relay.on_bridge_status(&wire_status_with_ssid(None), 30);
    let messages = relay.poll(app, 31);
    let live = must(bridges(&messages).into_iter().next(), "third bridge message");
    assert_eq!(live.ssid, None, "an unknown SSID is null, not a stale name");
}

/// A non-ASCII SSID is sanitised at the JSON boundary instead of dropping the status.
#[test]
fn a_non_ascii_station_network_is_sanitised_at_the_boundary() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.poll(app, 0);
    relay.on_bridge_status(&wire_status_with_ssid(Some(b"Caf\xe9")), 10);

    let messages = relay.poll(app, 11);
    let live = must(bridges(&messages).into_iter().next(), "live bridge message");
    assert_eq!(
        live.ssid.as_deref(),
        Some("Caf?"),
        "non-ASCII bytes become `?`, matching the panel"
    );
}

/// The boundary's JSON carries the SSID as a string, and as `null` when there is none.
#[test]
fn the_station_network_crosses_the_json_boundary() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.poll(app, 0);

    relay.on_bridge_status(&wire_status_with_ssid(Some(b"Rover Lab")), 10);
    let messages = relay.poll(app, 11);
    let live = must(bridges(&messages).into_iter().next(), "live bridge message");
    let value = must(serde_json::to_value(&live).ok(), "bridge json");
    assert_eq!(value.get("ssid").and_then(serde_json::Value::as_str), Some("Rover Lab"));

    relay.on_bridge_status(&wire_status_with_ssid(None), 20);
    let messages = relay.poll(app, 21);
    let live = must(bridges(&messages).into_iter().next(), "live bridge message");
    let value = must(serde_json::to_value(&live).ok(), "bridge json");
    assert_eq!(
        value.get("ssid"),
        Some(&serde_json::Value::Null),
        "an unknown network is null, never an empty string"
    );
}
