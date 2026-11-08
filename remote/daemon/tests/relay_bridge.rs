//! Bridge telemetry: relay storage, the poll lane and Boot Id independence.

mod support;

use remote_daemon::{
    relay::Relay,
    schema::{BridgeStation, BridgeStatus, ServerMessage},
};
use remote_protocol::{BridgeStation as WireBridgeStation, BridgeStatus as WireBridgeStatus, BridgeStatusFlags};
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
    }
}

/// The schema conversion of [`wire_status`] at `ts`.
const fn expected(ts: u64) -> BridgeStatus {
    BridgeStatus {
        ts,
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
    assert!(relay.poll(second, 14).is_empty());
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
