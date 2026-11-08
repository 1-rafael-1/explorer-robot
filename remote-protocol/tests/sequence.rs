//! Host tests for the shared wrap-safe command-sequence mechanism (ADR-0041).

use remote_protocol::{CommandVerdict, CommandWindow, Seq};

/// Commands start at 1; `last + 1` is accepted, `last` is a duplicate and anything else is stale.
#[test]
fn last_plus_one_is_accepted_last_is_a_duplicate_and_gaps_are_stale() {
    let mut window = CommandWindow::new();
    assert_eq!(window.epoch(), 0, "commands before HELLO run under epoch zero");
    assert_eq!(window.last_seq(), 0, "nothing has been accepted yet");

    assert_eq!(
        window.classify(0, Seq::new(0)),
        CommandVerdict::Duplicate,
        "the robot starts at last = 0, so sequence 0 is a duplicate"
    );
    assert_eq!(window.classify(0, Seq::new(1)), CommandVerdict::Accept);
    assert_eq!(window.last_seq(), 0, "classify never commits");

    window.advance(Seq::new(1));
    assert_eq!(window.last_seq(), 1);
    assert_eq!(window.classify(0, Seq::new(2)), CommandVerdict::Accept);
    assert_eq!(window.classify(0, Seq::new(1)), CommandVerdict::Duplicate);
    assert_eq!(window.classify(0, Seq::new(3)), CommandVerdict::Stale);
    assert_eq!(window.classify(0, Seq::new(0)), CommandVerdict::Stale);
}

/// Sequence arithmetic wraps from `u16::MAX` to zero.
#[test]
fn sequence_arithmetic_is_wrap_safe() {
    let mut window = CommandWindow::new();
    window.advance(Seq::new(u16::MAX));

    assert_eq!(window.classify(0, Seq::new(0)), CommandVerdict::Accept);
    assert_eq!(window.classify(0, Seq::new(u16::MAX)), CommandVerdict::Duplicate);
    assert_eq!(window.classify(0, Seq::new(u16::MAX - 1)), CommandVerdict::Stale);
}

/// An explicit epoch reset restarts the sequence, and classifying under a new epoch resets too.
#[test]
fn a_new_epoch_restarts_the_sequence() {
    let mut window = CommandWindow::new();
    window.advance(Seq::new(5));
    assert_eq!(window.classify(0, Seq::new(6)), CommandVerdict::Accept);

    window.reset_epoch(9);
    assert_eq!(window.epoch(), 9);
    assert_eq!(window.last_seq(), 0, "the explicit reset starts at zero");
    assert_eq!(window.classify(9, Seq::new(1)), CommandVerdict::Accept);

    window.advance(Seq::new(1));
    window.advance(Seq::new(2));
    assert_eq!(
        window.classify(10, Seq::new(1)),
        CommandVerdict::Accept,
        "a different epoch resets before classifying"
    );
    assert_eq!(window.epoch(), 10);
    assert_eq!(window.last_seq(), 0, "classification does not commit the new sequence");
}

/// The default window matches a freshly constructed one.
#[test]
fn default_matches_new() {
    assert_eq!(CommandWindow::default(), CommandWindow::new());
}
