//! Boot Id resets and same-Boot-Id reconnects (ADR-0039).
//!
//! A same-Boot-Id reconnect keeps the retained state and the in-flight command; a Boot Id change
//! clears the slots, ring, `last_terminal`, Catalogue and command map, and marks an in-flight
//! command indeterminate.

mod support;

use link_protocol::{
    Status as WireStatus,
    payload::event::{CompletionStatus, DriveDistanceCompleted},
};
use remote_daemon::{
    relay::Relay,
    schema::{Catalogue, CatalogueProcedure, EventName, EventValue, StreamName},
};
use serde_json::json;
use support::{
    activity, entry, event_named, frame, motion, must, obstacle, power, session_with_app, snapshot, stream_named,
};

/// A one-Procedure catalogue used by the boot and retry tests.
fn test_catalogue() -> Catalogue {
    Catalogue {
        procedures: vec![CatalogueProcedure {
            id: 2,
            label: "Motor calibration".to_owned(),
            available: true,
            interactive: false,
            calibration: true,
            takes_params: false,
        }],
    }
}

/// Starts and acknowledges a `drive_distance`, returning its wire sequence.
fn start_drive(relay: &mut Relay, app: u64, now_ms: u64) -> u16 {
    assert!(
        relay
            .submit_command(
                app,
                frame(1, "drive_distance", json!({"distance_cm": 10, "speed": 10})),
                now_ms,
            )
            .is_none()
    );
    let outbound = must(relay.poll_link(now_ms), "drive frame");
    relay.on_command_ack(outbound.wire_seq, WireStatus::Ok, now_ms + 1);
    outbound.wire_seq
}

/// A same-Boot-Id reconnect keeps the slots, ring, `last_terminal` and the in-flight command.
#[test]
fn a_same_boot_reconnect_keeps_the_retained_state() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.poll(app, 0);
    let _ = relay.on_power(&power(80, 7_400), 10);
    let _ = relay.on_activity(
        &activity(
            5,
            link_protocol::payload::stream::ActivityStage::Complete,
            Some(100),
            "done",
        ),
        11,
    );
    relay.on_obstacle(1, &obstacle(1, 0), 12);
    assert!(
        relay
            .submit_command(app, frame(1, "stop_motion", json!({"brake": true})), 20)
            .is_none()
    );
    let outbound = must(relay.poll_link(20), "stop frame");

    relay.session_down();
    assert_eq!(relay.boot_id(), Some(7), "the last Boot Id survives the stall");
    relay.session_up(7, 100);

    let messages = relay.poll(app, 110);
    let assembled = must(snapshot(&messages), "fresh snapshot on reconnect");
    assert_eq!(assembled.boot_id, Some(7));
    assert_eq!(assembled.streams.len(), 2);
    assert!(entry(&assembled.streams, StreamName::Power).is_some());
    assert_eq!(assembled.events.len(), 1);
    assert!(assembled.last_terminal.is_some());

    assert!(
        relay.poll_link(110).is_none(),
        "the ACK window restarts at the reconnect"
    );
    let retry = must(relay.poll_link(1_100), "retry after the reconnect");
    assert!(retry.retry);
    assert_eq!(retry.wire_seq, outbound.wire_seq, "the in-flight command was kept");

    let _ = relay.on_power(&power(81, 7_500), 120);
    let next = must(stream_named(&relay.poll(app, 121), StreamName::Power), "power");
    assert_eq!(next.seq, 2, "the daemon stream sequence never resets");
}

/// A Boot Id change clears the slots, ring, terminal Activity, Catalogue and command map.
#[test]
fn a_boot_id_change_clears_the_retained_state() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.poll(app, 0);
    let _ = relay.on_power(&power(80, 7_400), 10);
    let _ = relay.on_motion(&motion(1, 2), 11);
    let _ = relay.on_activity(
        &activity(
            5,
            link_protocol::payload::stream::ActivityStage::Complete,
            Some(100),
            "done",
        ),
        12,
    );
    relay.on_obstacle(1, &obstacle(1, 0), 13);
    relay.on_catalogue(test_catalogue(), 14);
    let _ = relay.poll(app, 15);
    relay.on_obstacle(2, &obstacle(0, 0), 16);

    relay.session_up(8, 100);
    let messages = relay.poll(app, 110);
    let assembled = must(snapshot(&messages), "snapshot after the Boot Id change");
    assert_eq!(assembled.boot_id, Some(8));
    assert_eq!(assembled.streams, Vec::new());
    assert!(assembled.last_terminal.is_none());
    assert_eq!(assembled.events, Vec::new());
    assert_eq!(assembled.event_cursor, None);
    assert!(
        assembled.catalogue.is_none(),
        "the Catalogue is re-fetched after a boot"
    );

    let _ = relay.on_power(&power(81, 7_500), 120);
    let next = must(stream_named(&relay.poll(app, 121), StreamName::Power), "power");
    assert_eq!(next.seq, 2, "clearing the slots never resets the daemon sequence");
}

/// A Boot Id change clears the command map, so a late completion loses its attribution.
#[test]
fn a_boot_id_change_clears_the_command_map() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let wire_seq = start_drive(&mut relay, app, 10);
    let _ = relay.poll(app, 20);

    relay.session_up(8, 100);
    let _ = relay.poll(app, 110);
    let completed = DriveDistanceCompleted {
        command_seq: wire_seq,
        status: CompletionStatus::Success,
        target_cm: 10,
        distance_cm: 10,
        duration_ms: 100,
        detail: "",
    };
    relay.on_drive_completed(1, &completed, 120);
    let event = must(
        event_named(&relay.poll(app, 121), EventName::DriveDistanceCompleted),
        "completion",
    );
    assert!(matches!(
        &event.v,
        EventValue::DriveDistanceCompleted { command_seq: None, .. }
    ));
}

/// A catalogue retry sends a fresh snapshot and leaves the streams untouched.
#[test]
fn a_catalogue_retry_marks_a_fresh_snapshot() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.poll(app, 0);
    let _ = relay.on_power(&power(80, 7_400), 10);
    let _ = relay.poll(app, 11);

    relay.on_catalogue(test_catalogue(), 20);
    let messages = relay.poll(app, 21);
    let assembled = must(snapshot(&messages), "snapshot after the retry");
    assert!(assembled.catalogue.is_some());
    assert_eq!(assembled.streams.len(), 1, "the stream slots are untouched");
    assert_eq!(relay.poll(app, 22), Vec::new());
}
