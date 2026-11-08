//! Host tests for the baseline persona (ticket 02): streams at the negotiated or baseline rates,
//! `HELLO`/`PING` answered, command ACK rules and event retries.
//!
//! Every observation is a frame at the serial seam; the expected values are the ADRs' (ADR-0031's
//! rates, ADR-0030's command rules, the shipped retry constants).

mod support;

use link_core::timing::EVENT_RETRY_MS;
use link_protocol::{
    Caps, Class, Command, Control, Event, Flags, Status, Stream,
    codec::PayloadCodec,
    ids::ProcedureId,
    payload::{
        control::{PingAck, RateEntry},
        event::DriveDistanceCompleted,
        stream::ActivityStage,
    },
};
use robot_stub::{Config, Persona};
use support::{Bench, Observed};

/// The Boot Id the stub mints for this powered link period.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// Returns the `DRIVE_DISTANCE` acknowledgements recorded so far.
fn drive_acks(bench: &Bench, after: Option<u64>) -> Vec<&Observed> {
    bench
        .peer
        .frames_of(Class::Command, Command::DriveDistance.wire_id(), after, bench.at())
}

/// Returns the drive completion events recorded so far.
fn completions(bench: &Bench) -> Vec<&Observed> {
    bench
        .peer
        .frames_of(Class::Event, Event::DriveDistanceCompleted.wire_id(), None, bench.at())
}

/// A `PING` is answered on the same sequence with `OK` (ADR-0030).
#[test]
fn ping_is_answered_through_the_transport() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::Baseline);
    bench.run_until(100);

    let seq = bench.peer.send_ping();
    bench.step_now();

    let replies = bench
        .peer
        .frames_of(Class::Control, Control::Ping.wire_id(), None, bench.at());
    assert_eq!(replies.len(), 1, "exactly one PING reply is sent");
    let reply = replies[0];
    assert!(reply.flags.contains(Flags::IS_ACK), "the reply is an acknowledgement");
    assert_eq!(reply.seq, seq, "the reply mirrors the PING's sequence");
    let ack = PingAck::decode(&reply.payload);
    assert!(ack.is_ok(), "the PING reply payload must decode");
    if let Ok(ack) = ack {
        assert_eq!(ack.status, Status::Ok);
    }
}

/// Without a Session, the baseline persona streams at the shipped baseline rates
/// (`link_core::timing::DEFAULT_RATES_CHZ`): the 10 Hz streams every 100 ms, `MOTION` and
/// `ACTIVITY` at 1 Hz, `POWER` and `CALIBRATION` at least every 5 s, counted over a one-second
/// window. A Session's Requested Rate raises a stream above its baseline (the next test).
#[test]
fn baseline_stream_rates_follow_the_catalogue() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::Baseline);
    bench.run_until(5_005);

    let window = (Some(4_005u64), 5_005u64);
    let count = |id: u16| bench.peer.frames_of(Class::Stream, id, window.0, window.1).len();
    assert_eq!(count(Stream::Motion.wire_id()), 1, "MOTION idles at the 1 Hz baseline");
    assert_eq!(count(Stream::Perception.wire_id()), 10, "PERCEPTION streams at 10 Hz");
    assert_eq!(count(Stream::LidarCloud.wire_id()), 10, "LIDAR_CLOUD streams at 10 Hz");
    assert_eq!(
        count(Stream::Activity.wire_id()),
        1,
        "ACTIVITY reports on change plus 1 Hz"
    );
    assert_eq!(
        count(Stream::Power.wire_id()),
        1,
        "POWER reports on change at least every 5 s"
    );
    assert_eq!(
        count(Stream::Calibration.wire_id()),
        1,
        "CALIBRATION reports on change at least every 5 s"
    );

    // The moving baseline carries the ADR-0031 payloads: a signed `MOTION` pair and the 720-byte
    // cloud, so the wire bytes match the catalogue too.
    let motions = bench
        .peer
        .frames_of(Class::Stream, Stream::Motion.wire_id(), window.0, window.1);
    assert_eq!(motions[0].payload.len(), 2, "MOTION carries the left/right pair");
    let clouds = bench
        .peer
        .frames_of(Class::Stream, Stream::LidarCloud.wire_id(), window.0, window.1);
    assert_eq!(clouds[0].payload.len(), 720, "LIDAR_CLOUD carries the 360-slot grid");
}

/// A Session's Requested Rate overrides the baseline: `MOTION` asked at 5 Hz streams at 5 Hz and
/// `LIDAR_CLOUD` asked at 2 Hz streams at 2 Hz (ADR-0030, ADR-0031).
#[test]
fn requested_rates_pace_the_negotiated_streams() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::Baseline);
    bench.run_until(100);

    let rates = [
        RateEntry::enabled(Stream::Motion.wire_id(), 500),
        RateEntry::enabled(Stream::LidarCloud.wire_id(), 200),
    ];
    let _ = bench.peer.send_hello(7, Caps::from_bits(0), &rates);
    bench.run_until(2_000);

    let window = (Some(1_000u64), 2_000u64);
    let count = |id: u16| bench.peer.frames_of(Class::Stream, id, window.0, window.1).len();
    assert_eq!(count(Stream::Motion.wire_id()), 5, "MOTION follows the requested 5 Hz");
    assert_eq!(
        count(Stream::LidarCloud.wire_id()),
        2,
        "LIDAR_CLOUD follows the requested 2 Hz"
    );
}

/// Commands follow the robot's rules (ADR-0030): `last + 1` executes and is acknowledged `OK`, a
/// duplicate is re-ACKed without re-executing, a gap is answered `STALE` without executing, and
/// each executed drive publishes exactly one completion event carrying the command's sequence.
#[test]
fn commands_follow_the_robot_ack_rules() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::Baseline);
    bench.run_until(100);
    let _ = bench.peer.send_hello(7, Caps::from_bits(0), &[]);
    bench.step_now();

    // Sequence 1 executes: an OK acknowledgement and one completion event.
    bench.peer.send_drive_distance(1, 100, 50);
    bench.step_now();
    assert_eq!(
        drive_acks(&bench, None).len(),
        1,
        "the executed command is acknowledged once"
    );
    let ack = drive_acks(&bench, None)[0];
    assert!(ack.flags.contains(Flags::IS_ACK));
    assert_eq!(ack.seq, 1);
    assert_eq!(ack.payload.first(), Some(&Status::Ok.bits()), "the command executed");
    assert_eq!(completions(&bench).len(), 1, "one completion event is published");
    let completion = DriveDistanceCompleted::decode(&completions(&bench)[0].payload);
    assert!(completion.is_ok(), "the completion payload must decode");
    if let Ok(completion) = completion {
        assert_eq!(
            completion.command_seq, 1,
            "the completion carries the command's sequence"
        );
        assert_eq!(completion.target_cm, 100);
        assert_eq!(completion.distance_cm, 100);
    }

    // A duplicate of sequence 1 is re-ACKed without re-executing: no second completion event.
    bench.peer.send_drive_distance(1, 100, 50);
    bench.run_until(200);
    assert_eq!(drive_acks(&bench, None).len(), 2, "the duplicate is re-acknowledged");
    assert_eq!(drive_acks(&bench, None)[1].seq, 1);
    assert_eq!(drive_acks(&bench, None)[1].payload.first(), Some(&Status::Ok.bits()));
    assert_eq!(completions(&bench).len(), 1, "the duplicate does not execute again");

    // Sequence 3 skips 2, so it is neither `last + 1` nor `last`: STALE, never executed.
    bench.peer.send_drive_distance(3, 100, 50);
    bench.run_until(300);
    let stale = drive_acks(&bench, Some(200));
    assert_eq!(stale.len(), 1, "the gap is answered once");
    assert_eq!(stale[0].seq, 3);
    assert_eq!(stale[0].payload.first(), Some(&Status::Stale.bits()), "a gap is STALE");
    assert_eq!(completions(&bench).len(), 1, "a STALE command does not execute");

    // Sequence 2 is `last + 1` again: it executes and publishes the second completion.
    bench.peer.send_drive_distance(2, 100, 50);
    bench.run_until(400);
    let executed = drive_acks(&bench, Some(300));
    assert_eq!(executed.len(), 1);
    assert_eq!(executed[0].seq, 2);
    assert_eq!(executed[0].payload.first(), Some(&Status::Ok.bits()));
    assert_eq!(
        completions(&bench).len(),
        2,
        "the second command executes and completes"
    );
    let second = DriveDistanceCompleted::decode(&completions(&bench)[1].payload);
    assert!(second.is_ok(), "the second completion must decode");
    if let Ok(second) = second {
        assert_eq!(second.command_seq, 2);
    }
}

/// An unacknowledged event is retransmitted with the `RETRY` flag on the shipped schedule, and a
/// late acknowledgement stops the retries (ADR-0030, ADR-0031).
#[test]
fn unacknowledged_events_are_retried() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::Baseline);
    bench.run_until(100);
    let _ = bench.peer.send_hello(7, Caps::from_bits(0), &[]);
    bench.step_now();

    bench.peer.send_drive_distance(1, 100, 50);
    bench.step_now();
    assert_eq!(completions(&bench).len(), 1, "the completion is sent once");
    let first_seq = completions(&bench)[0].seq;
    assert!(
        !completions(&bench)[0].flags.contains(Flags::RETRY),
        "the first transmission is not a retry"
    );

    // The peer stays silent: the first retransmission comes RETRY-flagged one retry interval
    // after the event was published, and nothing else is sent before then.
    bench.run_until(100 + EVENT_RETRY_MS);
    assert_eq!(completions(&bench).len(), 2, "exactly one retransmission is due");
    let retry = completions(&bench)[1];
    assert_eq!(retry.seq, first_seq, "the retransmission keeps the event's sequence");
    assert!(
        retry.flags.contains(Flags::RETRY),
        "the retransmission is RETRY-flagged"
    );
    assert_eq!(
        retry.at_ms,
        100 + EVENT_RETRY_MS,
        "the retry follows the shipped schedule"
    );

    // The acknowledgement retires the event; no further retransmissions follow.
    bench.peer.ack_event(Event::DriveDistanceCompleted, first_seq);
    bench.run_until(100 + 4 * EVENT_RETRY_MS);
    assert_eq!(
        completions(&bench).len(),
        2,
        "an acknowledged event is not retransmitted"
    );
}

/// The baseline `ACTIVITY` payload decodes as the catalogue's layout: the idle Procedure, no
/// percent and no detail (ADR-0031).
#[test]
fn baseline_activity_reports_the_idle_procedure() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::Baseline);
    bench.run_until(1_000);

    let samples = bench
        .peer
        .frames_of(Class::Stream, Stream::Activity.wire_id(), None, bench.at());
    assert!(!samples.is_empty(), "ACTIVITY streams at baseline");
    let activity = link_protocol::payload::stream::Activity::decode(&samples[0].payload);
    assert!(activity.is_ok(), "the ACTIVITY payload must decode");
    if let Ok(activity) = activity {
        assert_eq!(activity.procedure_id, ProcedureId::IDLE);
        assert_eq!(activity.stage, ActivityStage::Running);
        assert_eq!(activity.percent, None);
        assert_eq!(activity.detail, "");
    }
}
