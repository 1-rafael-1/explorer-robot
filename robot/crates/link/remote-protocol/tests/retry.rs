//! Host tests for the shared command retry mechanism (ADR-0041, ADR-0030, ADR-0031).
//!
//! The tests exercise the external contract: the exact time-driven schedule, the `RETRY` flag and
//! unchanged sequence on a retransmission, one command in flight, the indeterminate outcome, and
//! the Boot Id rule. The daemon-shaped harness in `support` composes the public API over the
//! simulated wire; no daemon or robot is implemented here.

mod support;

use remote_protocol::{
    AckOutcome, AckTracker, Direction, Flags, Frame, RetryPolicy, Seq,
    ids::Command,
    sim::{Faults, Wire},
};
use support::Harness;

/// Decodes the first frame of a PC-to-robot byte slice, if it is valid.
fn decode_one(bytes: &[u8]) -> Option<Frame<'_>> {
    Frame::decode(bytes, Direction::PcToRobot).ok()
}

/// The named defaults carry the contract's values and backoff.
#[test]
fn named_policies_match_the_contract() {
    let command = RetryPolicy::COMMAND;
    assert_eq!(command.first_delay_ms(), 300);
    assert_eq!(command.factor(), 2);
    assert_eq!(command.max_delay_ms(), 1_000);
    assert_eq!(command.attempts(), 3);
    assert_eq!(command.delay_after(1), 300);
    assert_eq!(command.delay_after(2), 600);
    assert_eq!(command.delay_after(3), 1_000);
    assert_eq!(command.delay_after(4), 1_000);

    let event = RetryPolicy::EVENT;
    assert_eq!(event.first_delay_ms(), 500);
    assert_eq!(event.factor(), 2);
    assert_eq!(event.max_delay_ms(), 2_000);
    assert_eq!(event.attempts(), 5);
    assert_eq!(event.delay_after(1), 500);
    assert_eq!(event.delay_after(2), 1_000);
    assert_eq!(event.delay_after(3), 2_000);
    assert_eq!(event.delay_after(4), 2_000);
    assert_eq!(event.delay_after(5), 2_000);
}

/// The command timeline is exactly send at `t0`, retransmit at 300 and 900, indeterminate at 1900.
#[test]
fn command_retry_schedule_is_exact() {
    let seq = Seq::new(1);
    let mut tracker = AckTracker::new(RetryPolicy::COMMAND);
    assert!(tracker.begin(seq, 7, 0), "the first command must begin");

    assert_eq!(tracker.poll(0), AckOutcome::Idle, "the initial send is not a retry");
    assert_eq!(tracker.poll(299), AckOutcome::Idle, "nothing is due before 300 ms");
    assert_eq!(
        tracker.poll(300),
        AckOutcome::Retransmit { seq, boot_id: 7 },
        "the first retransmission is due at 300 ms"
    );
    assert_eq!(tracker.poll(899), AckOutcome::Idle, "nothing is due before 900 ms");
    assert_eq!(
        tracker.poll(900),
        AckOutcome::Retransmit { seq, boot_id: 7 },
        "the second retransmission is due at 900 ms"
    );
    assert_eq!(tracker.poll(1_899), AckOutcome::Idle, "nothing is due before 1900 ms");
    assert_eq!(
        tracker.poll(1_900),
        AckOutcome::Indeterminate { seq, boot_id: 7 },
        "the outcome is declared at 1900 ms"
    );
    assert_eq!(
        tracker.poll(1_901),
        AckOutcome::Idle,
        "the terminal outcome is not repeated"
    );
}

/// The event policy reuses the same mechanism with its own exact timeline.
#[test]
fn event_retry_schedule_is_exact() {
    let seq = Seq::new(9);
    let mut tracker = AckTracker::new(RetryPolicy::EVENT);
    assert!(tracker.begin(seq, 1, 0), "the first event transmission must begin");

    assert_eq!(tracker.poll(499), AckOutcome::Idle);
    assert_eq!(tracker.poll(500), AckOutcome::Retransmit { seq, boot_id: 1 });
    assert_eq!(tracker.poll(1_499), AckOutcome::Idle);
    assert_eq!(tracker.poll(1_500), AckOutcome::Retransmit { seq, boot_id: 1 });
    assert_eq!(tracker.poll(3_499), AckOutcome::Idle);
    assert_eq!(tracker.poll(3_500), AckOutcome::Retransmit { seq, boot_id: 1 });
    assert_eq!(tracker.poll(5_499), AckOutcome::Idle);
    assert_eq!(tracker.poll(5_500), AckOutcome::Retransmit { seq, boot_id: 1 });
    assert_eq!(tracker.poll(7_499), AckOutcome::Idle);
    assert_eq!(tracker.poll(7_500), AckOutcome::Indeterminate { seq, boot_id: 1 });
}

/// The deadline comparison still works when the injected millisecond clock wraps.
#[test]
fn schedule_survives_the_millisecond_clock_wrapping() {
    let seq = Seq::new(3);
    let start = u64::MAX - 100;
    let mut tracker = AckTracker::new(RetryPolicy::COMMAND);
    assert!(tracker.begin(seq, 5, start));

    assert_eq!(tracker.poll(start.wrapping_add(299)), AckOutcome::Idle);
    assert_eq!(
        tracker.poll(start.wrapping_add(300)),
        AckOutcome::Retransmit { seq, boot_id: 5 }
    );
    assert_eq!(
        tracker.poll(start.wrapping_add(900)),
        AckOutcome::Retransmit { seq, boot_id: 5 }
    );
    assert_eq!(
        tracker.poll(start.wrapping_add(1_900)),
        AckOutcome::Indeterminate { seq, boot_id: 5 }
    );
}

/// The initial frame is a plain request and the retransmission sets `RETRY` with the same seq.
#[test]
fn retransmission_carries_retry_and_the_original_sequence() {
    let seq = Seq::new(42);
    let mut harness = Harness::new(RetryPolicy::COMMAND, Command::DriveDistance, seq, 3, &[100, 0, 50]);
    let mut wire = Wire::new(1, Faults::default());
    assert!(harness.start(&mut wire, 0), "the initial command must go out");

    let initial_bytes = wire.take(Direction::PcToRobot);
    let initial = decode_one(&initial_bytes);
    assert!(initial.is_some(), "the initial frame must decode");
    let Some(initial) = initial else { return };
    assert_eq!(initial.header.seq, seq, "the initial send uses the assigned seq");
    assert!(
        initial.header.flags.contains(Flags::ACK_REQUESTED),
        "the initial send asks for an ACK"
    );
    assert!(
        !initial.header.flags.contains(Flags::RETRY),
        "the initial send is not a retry"
    );

    assert_eq!(harness.tick(&mut wire, 300), AckOutcome::Retransmit { seq, boot_id: 3 });

    let retry_bytes = wire.take(Direction::PcToRobot);
    let retry = decode_one(&retry_bytes);
    assert!(retry.is_some(), "the retransmission must decode");
    let Some(retry) = retry else { return };
    assert_eq!(retry.header.seq, seq, "a retransmission keeps the original seq");
    assert!(retry.header.flags.contains(Flags::RETRY), "a retransmission sets RETRY");
    assert!(
        retry.header.flags.contains(Flags::ACK_REQUESTED),
        "a retransmission still asks for an ACK"
    );

    assert_eq!(harness.transmissions().len(), 2, "two transmissions happened");
    assert_eq!(harness.transmissions()[1].seq, seq);
}

/// Exhaustion is indeterminate: never success, never failure, and no further retries.
#[test]
fn exhaustion_reports_indeterminate_not_failed() {
    let seq = Seq::new(5);
    let mut harness = Harness::new(RetryPolicy::COMMAND, Command::StopMotion, seq, 3, &[1]);
    let mut wire = Wire::new(1, Faults::default());
    assert!(harness.start(&mut wire, 0));

    assert_eq!(harness.tick(&mut wire, 300), AckOutcome::Retransmit { seq, boot_id: 3 });
    assert_eq!(harness.tick(&mut wire, 900), AckOutcome::Retransmit { seq, boot_id: 3 });
    let outcome = harness.tick(&mut wire, 1_900);
    assert_eq!(outcome, AckOutcome::Indeterminate { seq, boot_id: 3 });
    assert_eq!(
        harness.outcome(),
        Some(AckOutcome::Indeterminate { seq, boot_id: 3 }),
        "the terminal outcome is recorded as indeterminate"
    );
    assert!(!harness.is_in_flight(), "an exhausted command retires from the tracker");
    assert_eq!(harness.transmissions().len(), 3, "three attempts were made");

    assert_eq!(harness.tick(&mut wire, 1_901), AckOutcome::Idle);
    assert_eq!(
        harness.transmissions().len(),
        3,
        "an indeterminate command is never retried again"
    );
}

/// One command in flight is enforced, and only a matching ACK retires it.
#[test]
fn one_command_in_flight_is_enforced() {
    let mut tracker = AckTracker::new(RetryPolicy::COMMAND);
    assert!(tracker.begin(Seq::new(1), 1, 0), "the first command is accepted");
    assert!(
        !tracker.begin(Seq::new(2), 1, 10),
        "a second command is refused while one is in flight"
    );
    assert_eq!(tracker.in_flight_seq(), Some(Seq::new(1)));

    assert!(
        !tracker.on_ack(Seq::new(9)),
        "an unrelated ACK does not retire the command"
    );
    assert!(tracker.is_in_flight(), "the original command is still in flight");

    assert!(tracker.on_ack(Seq::new(1)), "the matching ACK retires the command");
    assert!(!tracker.is_in_flight());
    assert!(
        tracker.begin(Seq::new(2), 1, 20),
        "the next command is accepted once idle"
    );
}

/// A Boot Id change discards the command and makes an automatic volatile re-send impossible.
#[test]
fn boot_id_change_discards_and_prevents_an_automatic_resend() {
    let seq = Seq::new(7);
    let mut tracker = AckTracker::new(RetryPolicy::COMMAND);
    assert!(tracker.begin(seq, 1, 0));

    assert_eq!(
        tracker.on_boot_id(2),
        Some(AckOutcome::DiscardedOnBootChange { seq }),
        "the change reports an explicit do-not-retry outcome"
    );
    assert!(!tracker.is_in_flight(), "the in-flight command is discarded");
    assert_eq!(
        tracker.poll(300),
        AckOutcome::Idle,
        "a discarded command is never retransmitted automatically"
    );
    assert!(!tracker.on_ack(seq), "a late ACK for the discarded command is ignored");
    assert_eq!(tracker.on_boot_id(2), None, "observing the same Boot Id is a no-op");
}

/// The harness does not put a discarded command back on the wire after a Boot Id change.
#[test]
fn harness_sends_nothing_after_a_boot_id_change() {
    let seq = Seq::new(8);
    let mut harness = Harness::new(RetryPolicy::COMMAND, Command::RotateBy, seq, 1, &[0, 0, 0]);
    let mut wire = Wire::new(1, Faults::default());
    assert!(harness.start(&mut wire, 0));
    let _ = wire.take(Direction::PcToRobot);

    assert_eq!(harness.on_boot_id(2), Some(AckOutcome::DiscardedOnBootChange { seq }));
    assert_eq!(harness.tick(&mut wire, 300), AckOutcome::Idle);
    assert_eq!(harness.transmissions().len(), 1, "no retransmission is emitted");
    assert!(wire.take(Direction::PcToRobot).is_empty(), "the wire stays silent");
}

/// A matched ACK retires the command before any retransmission.
#[test]
fn an_ack_before_the_deadline_succeeds_without_a_retry() {
    let seq = Seq::new(11);
    let mut harness = Harness::new(RetryPolicy::COMMAND, Command::GetStatus, seq, 3, &[]);
    let mut wire = Wire::new(1, Faults::default());
    assert!(harness.start(&mut wire, 0));
    let _ = wire.take(Direction::PcToRobot);

    assert!(harness.acknowledge(&mut wire), "the robot ACK must encode");
    assert_eq!(harness.tick(&mut wire, 0), AckOutcome::Idle);
    assert!(!harness.is_in_flight(), "the ACK retires the command");

    assert_eq!(harness.tick(&mut wire, 300), AckOutcome::Idle);
    assert_eq!(harness.transmissions().len(), 1, "a successful command never retries");
    assert_eq!(harness.outcome(), None, "success is not an indeterminate outcome");
}

/// A lost ACK is retried, and the retransmission's ACK succeeds.
#[test]
fn lost_ack_then_success_retries_once_and_retires() {
    let seq = Seq::new(12);
    let mut harness = Harness::new(RetryPolicy::COMMAND, Command::DriveDistance, seq, 3, &[50, 0, 40]);
    let mut wire = Wire::new(1, Faults::default());
    assert!(harness.start(&mut wire, 0));
    let _ = wire.take(Direction::PcToRobot);

    harness.set_drop_ack(true);
    assert!(harness.acknowledge(&mut wire), "the first ACK is sent but dropped");
    assert_eq!(harness.tick(&mut wire, 250), AckOutcome::Idle);
    assert!(harness.is_in_flight(), "a dropped ACK leaves the command in flight");

    assert_eq!(harness.tick(&mut wire, 300), AckOutcome::Retransmit { seq, boot_id: 3 });
    assert_eq!(harness.transmissions().len(), 2, "the command was retransmitted once");

    harness.set_drop_ack(false);
    assert!(harness.acknowledge(&mut wire), "the second ACK is delivered");
    assert_eq!(harness.tick(&mut wire, 320), AckOutcome::Idle);
    assert!(!harness.is_in_flight(), "the retransmission's ACK retires the command");

    assert_eq!(harness.tick(&mut wire, 900), AckOutcome::Idle);
    assert_eq!(harness.transmissions().len(), 2, "success stops the retry schedule");
    assert_eq!(harness.outcome(), None);
}
