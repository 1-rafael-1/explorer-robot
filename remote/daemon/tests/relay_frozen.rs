//! The app-frozen rule: a conditional stop, then no PING (ADR-0039).
//!
//! Losing every app heartbeat for two seconds stops only a claim the daemon started; panel work is
//! never touched, because the robot's deadman already covers Remote-originated claims.

mod support;

use link_protocol::{
    Command, CommandMessage, Status as WireStatus,
    payload::{command::StopMotion, stream::ActivityStage},
};
use remote_daemon::{relay::Relay, schema::MANUAL_DRIVE_ID};
use serde_json::json;
use support::{activity, frame, must, session_with_app, snapshot};

/// Starts a Manual Drive the daemon owns: submit, send and acknowledge a `drive_distance`.
fn start_manual_drive(relay: &mut Relay, app: u64, now_ms: u64) {
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
}

/// A frozen app stops the Manual Drive the daemon started, then PING stops.
#[test]
fn a_frozen_app_stops_a_manual_drive_the_daemon_started() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.on_activity(
        &activity(MANUAL_DRIVE_ID, ActivityStage::Running, Some(0), "manual"),
        10,
    );
    start_manual_drive(&mut relay, app, 20);
    assert_eq!(relay.apps_live(), 1);
    assert!(relay.wants_ping());

    relay.tick(2_001);
    assert_eq!(relay.apps_live(), 0);
    assert!(!relay.wants_ping(), "PING stops when no app is live");
    let stop = must(relay.poll_link(2_001), "conditional stop");
    assert_eq!(stop.command, Command::StopMotion);
    assert_eq!(
        stop.message,
        CommandMessage::StopMotion(StopMotion {
            brake: StopMotion::BRAKE,
        })
    );
    assert!(relay.poll_link(2_002).is_none(), "freezing sends exactly one stop");
}

/// The freeze fires two seconds after the last heartbeat, not four: ticking during the liveness
/// window must not advance the deadline.
#[test]
fn the_freeze_fires_two_seconds_after_the_last_heartbeat() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.on_activity(
        &activity(MANUAL_DRIVE_ID, ActivityStage::Running, Some(0), "manual"),
        10,
    );
    start_manual_drive(&mut relay, app, 20);
    assert!(relay.wants_ping());

    let mut frozen_at = None;
    for now_ms in (1_900..=2_100).step_by(50) {
        relay.tick(now_ms);
        if frozen_at.is_none() && !relay.wants_ping() {
            frozen_at = Some(now_ms);
        }
    }

    let frozen_at = must(frozen_at, "the freeze fires by 2.1 s");
    assert!(
        frozen_at > 2_000 && frozen_at <= 2_100,
        "the freeze is measured from the last heartbeat (0 ms), not a refreshed tick: {frozen_at}"
    );
    assert_eq!(relay.apps_live(), 0, "PING stops at the two-second mark");
    let stop = must(relay.poll_link(frozen_at), "conditional stop");
    assert_eq!(stop.command, Command::StopMotion);
    assert_eq!(
        stop.message,
        CommandMessage::StopMotion(StopMotion {
            brake: StopMotion::BRAKE,
        })
    );
    assert!(
        relay.poll_link(frozen_at + 50).is_none(),
        "the conditional stop fires exactly once"
    );
}

/// A frozen app never stops panel-originated work.
#[test]
fn a_frozen_app_never_stops_panel_work() {
    let mut relay = Relay::new();
    let _ = session_with_app(&mut relay, 7, 0);
    let _ = relay.on_activity(&activity(5, ActivityStage::Running, Some(10), "panel"), 10);

    relay.tick(2_001);
    assert!(!relay.wants_ping());
    assert!(
        relay.poll_link(2_001).is_none(),
        "the daemon started no claim, so it sends no stop"
    );
}

/// A Procedure the daemon started is not stopped if another Activity is the running one.
#[test]
fn a_running_procedure_the_daemon_did_not_start_is_left_alone() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.on_activity(&activity(5, ActivityStage::Running, Some(10), "panel"), 10);
    assert!(
        relay
            .submit_command(app, frame(1, "start_procedure", json!({"id": 2})), 20)
            .is_none()
    );
    let outbound = must(relay.poll_link(20), "start frame");
    relay.on_command_ack(outbound.wire_seq, WireStatus::Ok, 21);

    relay.tick(2_001);
    assert!(relay.poll_link(2_001).is_none());
}

/// A Procedure the daemon started, and which is still running, is stopped on a freeze.
#[test]
fn a_procedure_the_daemon_started_is_stopped() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.on_activity(&activity(2, ActivityStage::Running, Some(10), "calibration"), 10);
    assert!(
        relay
            .submit_command(app, frame(1, "start_procedure", json!({"id": 2})), 20)
            .is_none()
    );
    let outbound = must(relay.poll_link(20), "start frame");
    relay.on_command_ack(outbound.wire_seq, WireStatus::Ok, 21);

    relay.tick(2_001);
    let stop = must(relay.poll_link(2_001), "conditional stop");
    assert_eq!(stop.command, Command::StopMotion);
}

/// A daemon-started Manual Drive that has already finished is not stopped again.
#[test]
fn a_finished_manual_drive_is_not_stopped_again() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.on_activity(
        &activity(MANUAL_DRIVE_ID, ActivityStage::Running, Some(0), "manual"),
        10,
    );
    start_manual_drive(&mut relay, app, 20);
    let _ = relay.on_activity(
        &activity(MANUAL_DRIVE_ID, ActivityStage::Complete, Some(100), "done"),
        30,
    );

    relay.tick(2_001);
    assert!(relay.poll_link(2_001).is_none());
}

/// A heartbeat resumes the Session and asks for a fresh snapshot.
#[test]
fn a_heartbeat_resumes_and_asks_for_a_fresh_snapshot() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let _ = relay.on_activity(&activity(5, ActivityStage::Running, None, "panel"), 10);
    relay.tick(2_001);
    assert!(!relay.wants_ping());
    let _ = must(snapshot(&relay.poll(app, 2_001)), "first snapshot");

    relay.heartbeat(app, 3_000);
    assert!(relay.wants_ping());
    let messages = relay.poll(app, 3_001);
    assert!(snapshot(&messages).is_some(), "resuming sends a fresh snapshot");
}

/// With zero app connections, PING stops on the same two-second timer.
#[test]
fn no_app_connections_stop_ping_on_the_same_timer() {
    let mut relay = Relay::new();
    relay.session_up(7, 0);
    assert!(!relay.wants_ping());
    relay.tick(2_001);
    assert!(!relay.wants_ping());
    assert!(relay.poll_link(2_001).is_none());
}
