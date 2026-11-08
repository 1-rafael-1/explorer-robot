//! The event ring, per-app cursors and overflow notices (ADR-0039).
//!
//! Events keep the wire's `(boot_id, seq)`; the ring retains 300 entries and an app that falls
//! behind the oldest receives one `event_overflow{dropped}` notice and jumps.

mod support;

use remote_daemon::{
    relay::Relay,
    schema::{EventName, EventValue},
};
use support::{event_named, events, must, obstacle, overflow, session_with_app, snapshot};

/// A wire event keeps its Boot Id, its wire sequence and the daemon's receive time.
#[test]
fn an_event_keeps_the_wire_boot_id_and_sequence() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert!(events(&relay.poll(app, 0)).is_empty());

    relay.on_obstacle(1, &obstacle(1, 0), 10);
    let messages = relay.poll(app, 11);
    let event = must(event_named(&messages, EventName::Obstacle), "obstacle");
    assert_eq!(event.boot_id, Some(7));
    assert_eq!(event.seq, Some(1));
    assert_eq!(event.ts, 10);
    assert_eq!(
        event.v,
        EventValue::Obstacle {
            detected: true,
            source: "lidar".to_owned(),
        }
    );
}

/// The ring retains 300 events; a cursor behind the oldest jumps with one overflow notice.
#[test]
fn the_ring_is_bounded_and_reports_how_many_were_dropped() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert!(events(&relay.poll(app, 0)).is_empty());

    for seq in 1..=305_u16 {
        relay.on_obstacle(seq, &obstacle(1, 0), u64::from(seq));
    }
    let messages = relay.poll(app, 1_000);
    let delivered = events(&messages);
    assert_eq!(delivered.len(), 301, "one overflow notice plus the 300 retained events");
    assert_eq!(delivered[0].name, EventName::EventOverflow);
    assert_eq!(delivered[0].boot_id, None, "the daemon's notice replays no wire event");
    assert_eq!(delivered[0].seq, None);
    assert_eq!(delivered[0].ts, 1_000);
    assert_eq!(delivered[0].v, EventValue::EventOverflow { dropped: 5 });
    assert_eq!(delivered[1].seq, Some(6), "the oldest retained event is seq 6");
    assert_eq!(delivered[300].seq, Some(305));

    let assembled = must(relay.snapshot(app, 2_000), "snapshot");
    assert_eq!(assembled.events.len(), 300);
    assert_eq!(assembled.event_cursor, Some(305));
}

/// A new app's snapshot carries the bounded replay, so its cursor starts at the ring tail.
#[test]
fn a_new_app_receives_the_bounded_replay_in_its_snapshot() {
    let mut relay = Relay::new();
    let app_a = session_with_app(&mut relay, 7, 0);
    assert!(events(&relay.poll(app_a, 0)).is_empty());
    relay.on_obstacle(1, &obstacle(1, 0), 10);
    relay.on_floor_drop(2, &support::floor_drop(1), 11);
    relay.on_obstacle(3, &obstacle(0, 0), 12);

    let app_b = relay.connect_app(100);
    let messages = relay.poll(app_b, 101);
    let assembled = must(snapshot(&messages), "snapshot");
    assert_eq!(assembled.events.len(), 3);
    assert_eq!(assembled.event_cursor, Some(3));
    assert!(
        events(&messages).is_empty(),
        "the snapshot already replays what the new app missed"
    );
}

/// Each connection keeps its own cursor over the shared ring.
#[test]
fn every_app_has_its_own_cursor() {
    let mut relay = Relay::new();
    let app_a = session_with_app(&mut relay, 7, 0);
    assert!(events(&relay.poll(app_a, 0)).is_empty());
    let app_b = relay.connect_app(1);
    assert!(events(&relay.poll(app_b, 2)).is_empty());

    relay.on_obstacle(1, &obstacle(1, 0), 10);
    relay.on_obstacle(2, &obstacle(0, 0), 11);
    assert_eq!(events(&relay.poll(app_a, 12)).len(), 2);
    assert_eq!(events(&relay.poll(app_b, 13)).len(), 2);

    relay.on_floor_drop(3, &support::floor_drop(1), 20);
    assert_eq!(events(&relay.poll(app_a, 21)).len(), 1);
    assert_eq!(events(&relay.poll(app_b, 22)).len(), 1);
}

/// The robot's own overflow event keeps the wire's sequence, unlike the daemon's notice.
#[test]
fn the_robots_overflow_event_keeps_its_wire_sequence() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert!(events(&relay.poll(app, 0)).is_empty());

    relay.on_robot_event_overflow(9, &overflow(3), 40);
    let messages = relay.poll(app, 41);
    let event = must(event_named(&messages, EventName::EventOverflow), "overflow");
    assert_eq!(event.boot_id, Some(7));
    assert_eq!(event.seq, Some(9));
    assert_eq!(event.v, EventValue::EventOverflow { dropped: 3 });
}

/// A wire event outside a Session is ignored rather than stamped with a null Boot Id.
#[test]
fn an_event_outside_a_session_is_ignored() {
    let mut relay = Relay::new();
    relay.on_obstacle(1, &obstacle(1, 0), 10);
    relay.session_up(7, 20);
    let app = relay.connect_app(20);
    let assembled = must(snapshot(&relay.poll(app, 30)), "snapshot");
    assert!(assembled.events.is_empty());
    assert_eq!(assembled.event_cursor, None);
}
