//! The command map, retry/timeout rules and completion attribution (ADR-0039).
//!
//! Commands arrive from the in-memory app transport, leave through the fake link's `poll_link`,
//! and are acknowledged with a wire status. Results only reach the originating tab.

mod support;

use remote_daemon::{
    relay::Relay,
    schema::{EventName, EventValue, Outcome, Status},
};
use remote_protocol::{
    Command, CommandMessage, Status as WireStatus,
    payload::{
        command::{DriveDistance, StopMotion},
        event::{CompletionStatus, DriveDistanceCompleted},
    },
};
use serde_json::json;
use support::{event_named, frame, must, results, session_with_app, snapshot};

/// A volatile command is sent once and times out after three seconds without an ACK.
#[test]
fn a_volatile_command_is_sent_once_and_times_out() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let accepted = relay.submit_command(
        app,
        frame(1, "drive_distance", json!({"distance_cm": 100, "speed": 50})),
        100,
    );
    assert!(accepted.is_none(), "an accepted command is not rejected");

    let outbound = must(relay.poll_link(100), "first frame");
    assert_eq!(outbound.wire_seq, 1);
    assert_eq!(outbound.command, Command::DriveDistance);
    assert!(!outbound.retry);
    assert_eq!(
        outbound.message,
        CommandMessage::DriveDistance(DriveDistance {
            distance_cm: 100,
            speed: 50,
        })
    );

    assert!(relay.poll_link(1_000).is_none(), "a volatile command is never retried");
    relay.tick(3_100);
    let result = must(results(&relay.poll(app, 3_100)).into_iter().next(), "timeout result");
    assert_eq!(result.command_seq, 1);
    assert_eq!(result.command, "drive_distance");
    assert_eq!(result.outcome, Outcome::Timeout);
    assert_eq!(result.status, Status::Unknown);
    assert!(relay.poll_link(3_200).is_none());
}

/// An idempotent command retries three times with the same wire sequence, then times out.
#[test]
fn an_idempotent_command_retries_with_the_retry_flag_then_times_out() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert!(
        relay
            .submit_command(app, frame(4, "stop_motion", json!({"brake": true})), 0)
            .is_none()
    );

    let first = must(relay.poll_link(0), "first frame");
    assert_eq!(first.message, CommandMessage::StopMotion(StopMotion { brake: 1 }));
    assert!(!first.retry);
    for second in 1..=3 {
        let retry = must(relay.poll_link(second * 1_000), "retry");
        assert!(retry.retry, "every retry carries the RETRY flag");
        assert_eq!(retry.wire_seq, first.wire_seq, "the same wire sequence is reused");
        assert_eq!(retry.message, first.message);
    }

    relay.tick(4_000);
    let result = must(results(&relay.poll(app, 4_000)).into_iter().next(), "timeout result");
    assert_eq!(result.outcome, Outcome::Timeout);
    assert_eq!(result.command, "stop_motion");
    assert!(relay.poll_link(4_100).is_none());
}

/// A command result reaches the originating connection and no other tab.
#[test]
fn a_command_result_reaches_only_the_originating_tab() {
    let mut relay = Relay::new();
    let app_a = session_with_app(&mut relay, 7, 0);
    let app_b = relay.connect_app(1);
    assert!(
        relay
            .submit_command(
                app_a,
                frame(9, "drive_distance", json!({"distance_cm": -20, "speed": 30})),
                10,
            )
            .is_none()
    );

    let outbound = must(relay.poll_link(10), "frame");
    relay.on_command_ack(outbound.wire_seq, WireStatus::Busy, 20);
    let addressed = must(results(&relay.poll(app_a, 30)).into_iter().next(), "result");
    assert_eq!(addressed.command_seq, 9);
    assert_eq!(addressed.outcome, Outcome::Rejected);
    assert_eq!(addressed.status, Status::Busy);
    assert!(results(&relay.poll(app_b, 31)).is_empty());
}

/// `VERSION_MISMATCH` is `rejected` with a detail, because the app has no separate status.
#[test]
fn a_version_mismatch_ack_is_rejected_with_a_detail() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert!(
        relay
            .submit_command(app, frame(1, "stop_motion", json!({"brake": true})), 0)
            .is_none()
    );
    let outbound = must(relay.poll_link(0), "frame");
    relay.on_command_ack(outbound.wire_seq, WireStatus::VersionMismatch, 10);

    let result = must(results(&relay.poll(app, 11)).into_iter().next(), "result");
    assert_eq!(result.outcome, Outcome::Rejected);
    assert_eq!(result.status, Status::Rejected);
    assert_eq!(result.detail, Some("Robot protocol version mismatch".to_owned()));
}

/// `stop_motion` jumps the queue but never preempts the frame already on the wire.
#[test]
fn stop_motion_jumps_the_queue_without_preempting_the_inflight_frame() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert!(
        relay
            .submit_command(
                app,
                frame(1, "drive_distance", json!({"distance_cm": 10, "speed": 20})),
                0
            )
            .is_none()
    );
    let in_flight = must(relay.poll_link(0), "first frame");
    assert_eq!(in_flight.command, Command::DriveDistance);

    assert!(
        relay
            .submit_command(
                app,
                frame(2, "drive_distance", json!({"distance_cm": 20, "speed": 20})),
                1
            )
            .is_none()
    );
    assert!(
        relay
            .submit_command(app, frame(3, "stop_motion", json!({"brake": true})), 2)
            .is_none()
    );

    relay.on_command_ack(in_flight.wire_seq, WireStatus::Ok, 3);

    let next = must(relay.poll_link(3), "next frame");
    assert_eq!(next.command, Command::StopMotion);
    assert_eq!(next.message, CommandMessage::StopMotion(StopMotion { brake: 1 }));
    relay.on_command_ack(next.wire_seq, WireStatus::Ok, 4);

    let third = must(relay.poll_link(4), "third frame");
    assert_eq!(third.command, Command::DriveDistance);
    assert_eq!(
        third.message,
        CommandMessage::DriveDistance(DriveDistance {
            distance_cm: 20,
            speed: 20,
        })
    );
}

/// A command while the robot link is down is rejected `stale` and never queued.
#[test]
fn a_command_without_a_robot_link_is_rejected_stale() {
    let mut relay = Relay::new();
    let app = relay.connect_app(0);
    let rejection = must(
        relay.submit_command(app, frame(1, "stop_motion", json!({"brake": true})), 10),
        "rejection",
    );
    assert_eq!(rejection.outcome, Outcome::Rejected);
    assert_eq!(rejection.status, Status::Stale);
    assert_eq!(rejection.detail, Some("Robot link lost".to_owned()));
    assert!(relay.poll_link(10).is_none());
}

/// A heartbeat older than two seconds rejects a command; exactly two seconds is still fresh.
#[test]
fn a_stale_heartbeat_rejects_a_command() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert!(
        relay
            .submit_command(app, frame(1, "stop_motion", json!({"brake": true})), 2_000)
            .is_none(),
        "exactly 2 s is within the liveness window"
    );
    let rejection = must(
        relay.submit_command(app, frame(2, "stop_motion", json!({"brake": true})), 2_001),
        "rejection",
    );
    assert_eq!(rejection.status, Status::Stale);
    assert_eq!(rejection.detail, Some("App heartbeat is stale".to_owned()));
}

/// A completion carries the originating app's `command_seq` and `null` for every other tab.
#[test]
fn a_completion_belongs_to_the_originating_tab() {
    let mut relay = Relay::new();
    let app_a = session_with_app(&mut relay, 7, 0);
    let app_b = relay.connect_app(1);
    assert!(
        relay
            .submit_command(
                app_a,
                frame(7, "drive_distance", json!({"distance_cm": 100, "speed": 50})),
                10,
            )
            .is_none()
    );
    let outbound = must(relay.poll_link(10), "frame");
    relay.on_command_ack(outbound.wire_seq, WireStatus::Ok, 20);
    assert_eq!(results(&relay.poll(app_a, 30)).len(), 1, "drain the ACK result");
    let _ = relay.poll(app_b, 30);

    let completed = DriveDistanceCompleted {
        command_seq: outbound.wire_seq,
        status: CompletionStatus::Success,
        target_cm: 100,
        distance_cm: 97,
        duration_ms: 2_000,
        detail: "done",
    };
    relay.on_drive_completed(1, &completed, 40);
    let a_event = must(
        event_named(&relay.poll(app_a, 41), EventName::DriveDistanceCompleted),
        "A event",
    );
    let b_event = must(
        event_named(&relay.poll(app_b, 42), EventName::DriveDistanceCompleted),
        "B event",
    );
    assert!(matches!(
        &a_event.v,
        EventValue::DriveDistanceCompleted {
            command_seq: Some(7),
            actual_cm: 97,
            ..
        }
    ));
    assert!(matches!(
        &b_event.v,
        EventValue::DriveDistanceCompleted { command_seq: None, .. }
    ));

    relay.on_drive_completed(2, &completed, 50);
    let late_event = must(
        event_named(&relay.poll(app_a, 51), EventName::DriveDistanceCompleted),
        "replayed event",
    );
    assert!(
        matches!(
            &late_event.v,
            EventValue::DriveDistanceCompleted { command_seq: None, .. }
        ),
        "the completion record is evicted once the completion has been delivered"
    );
}

/// A Boot Id change marks the outstanding command indeterminate and requests a snapshot.
#[test]
fn a_boot_id_change_marks_an_inflight_command_indeterminate() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert!(
        relay
            .submit_command(app, frame(5, "rotate_by", json!({"angle_deg": 90.0, "speed": 40})), 10)
            .is_none()
    );
    let _ = must(relay.poll_link(10), "frame");
    relay.session_down();
    relay.session_up(8, 20);

    let messages = relay.poll(app, 30);
    let assembled = must(snapshot(&messages), "snapshot");
    assert_eq!(assembled.boot_id, Some(8));
    let result = must(results(&messages).into_iter().next(), "indeterminate result");
    assert_eq!(result.command_seq, 5);
    assert_eq!(result.command, "rotate_by");
    assert_eq!(result.outcome, Outcome::Indeterminate);
    assert!(relay.poll_link(30).is_none());
}

/// Malformed and unknown commands are rejected synchronously and never queued.
#[test]
fn malformed_commands_are_rejected_synchronously() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    let cases = [
        (frame(1, "stop_motion", json!({})), Status::BadArgument),
        (
            frame(2, "start_procedure", json!({"id": 2, "params": {"speed": 10}})),
            Status::BadArgument,
        ),
        (
            frame(3, "drive_distance", json!({"distance_cm": 10, "speed": 101})),
            Status::BadArgument,
        ),
        (frame(4, "teleport", json!({})), Status::UnknownId),
        (
            frame(5, "rotate_by", json!({"angle_deg": 180.01, "speed": 10})),
            Status::BadArgument,
        ),
    ];
    for (index, (command_frame, expected)) in cases.into_iter().enumerate() {
        let rejection = must(relay.submit_command(app, command_frame, 10), "rejection");
        assert_eq!(rejection.status, expected, "case {index}");
        assert_eq!(rejection.outcome, Outcome::Rejected);
    }
    assert!(relay.poll_link(20).is_none(), "a rejected command is never queued");
}

/// An unknown connection cannot submit a command.
#[test]
fn a_command_from_an_unknown_app_returns_nothing() {
    let mut relay = Relay::new();
    relay.session_up(7, 0);
    assert!(
        relay
            .submit_command(99, frame(1, "stop_motion", json!({"brake": true})), 0)
            .is_none()
    );
}

/// The command map holds 64 commands; a 65th queued command is rejected `busy`.
#[test]
fn the_command_map_is_bounded_at_sixty_four() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    for command_seq in 1..=64 {
        assert!(
            relay
                .submit_command(app, frame(command_seq, "stop_motion", json!({"brake": false})), 0,)
                .is_none()
        );
    }
    let rejection = must(
        relay.submit_command(app, frame(65, "stop_motion", json!({"brake": false})), 0),
        "rejection",
    );
    assert_eq!(rejection.status, Status::Busy);
    assert_eq!(rejection.outcome, Outcome::Rejected);
}

/// A completion record is evicted 60 s after its ACK, so a late completion loses attribution.
#[test]
fn a_completion_record_is_evicted_after_sixty_seconds() {
    let mut relay = Relay::new();
    let app = session_with_app(&mut relay, 7, 0);
    assert!(
        relay
            .submit_command(
                app,
                frame(1, "drive_distance", json!({"distance_cm": 10, "speed": 10})),
                0,
            )
            .is_none()
    );
    let outbound = must(relay.poll_link(0), "frame");
    relay.on_command_ack(outbound.wire_seq, WireStatus::Ok, 10);
    let _ = relay.poll(app, 11);

    relay.tick(60_010);
    let completed = DriveDistanceCompleted {
        command_seq: outbound.wire_seq,
        status: CompletionStatus::Success,
        target_cm: 10,
        distance_cm: 10,
        duration_ms: 100,
        detail: "",
    };
    relay.on_drive_completed(1, &completed, 60_020);
    let event = must(
        event_named(&relay.poll(app, 60_021), EventName::DriveDistanceCompleted),
        "event",
    );
    assert!(matches!(
        &event.v,
        EventValue::DriveDistanceCompleted { command_seq: None, .. }
    ));
}
