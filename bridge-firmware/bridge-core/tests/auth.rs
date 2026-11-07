//! Host tests for the authentication and single-client gate (ADR-0037).
//!
//! The gate is modelled with plain millisecond timestamps: a connection has five seconds to prove
//! the pairing secret, a new unauthenticated connection supersedes an older one, five failures
//! inside a minute close a one-minute refusal window, and a success resets the counter. The proof
//! comparison is constant time; the HMAC that produces the proof belongs to `bridge-net`.

use bridge_core::{ClientAuth, ConnectOutcome, Proof, ProofOutcome, TimerEvent, auth, constant_time_eq};

/// A proof filled with one byte.
const fn proof(byte: u8) -> Proof {
    Proof::from_bytes([byte; auth::PROOF_LEN])
}

/// Connects and returns the new connection's id when the gate issues a challenge.
fn challenge(auth: &mut ClientAuth, now_ms: u32) -> Option<u32> {
    match auth.connect(now_ms) {
        ConnectOutcome::Challenge { connection } => Some(connection),
        _ => None,
    }
}

/// A matching proof admits the one client.
#[test]
fn a_matching_proof_admits_the_client() {
    let mut auth = ClientAuth::new();
    let Some(connection) = challenge(&mut auth, 0) else {
        return;
    };

    assert_eq!(
        auth.authenticate(connection, proof(7), &proof(7), 1),
        ProofOutcome::Accepted
    );
    assert!(auth.is_authenticated());
    assert_eq!(auth.authenticated_connection(), Some(connection));
}

/// A wrong proof is rejected and the same connection may try again inside its deadline.
#[test]
fn a_wrong_proof_is_rejected_and_the_connection_may_retry() {
    let mut auth = ClientAuth::new();
    let Some(connection) = challenge(&mut auth, 0) else {
        return;
    };

    assert_eq!(
        auth.authenticate(connection, proof(1), &proof(2), 1),
        ProofOutcome::Rejected { failures: 1 }
    );
    assert!(!auth.is_authenticated());

    assert_eq!(
        auth.authenticate(connection, proof(2), &proof(2), 2),
        ProofOutcome::Accepted
    );
}

/// A connection that does not authenticate within five seconds is dropped.
#[test]
fn a_connection_must_authenticate_within_five_seconds() {
    let mut auth = ClientAuth::new();
    let Some(connection) = challenge(&mut auth, 0) else {
        return;
    };

    assert_eq!(auth.tick(4_999), None);
    assert_eq!(auth.tick(5_000), Some(TimerEvent::AuthenticationExpired { connection }));
    assert_eq!(auth.current_connection(), None);
    assert_eq!(
        auth.authenticate(connection, proof(1), &proof(1), 5_000),
        ProofOutcome::Superseded
    );
}

/// A proof offered at the deadline is too late even before the tick expires the connection.
#[test]
fn a_proof_at_the_deadline_finds_the_connection_expired() {
    let mut auth = ClientAuth::new();
    let Some(connection) = challenge(&mut auth, 0) else {
        return;
    };

    assert_eq!(
        auth.authenticate(connection, proof(1), &proof(1), 5_000),
        ProofOutcome::DeadlineExpired
    );
    assert!(!auth.is_authenticated());
}

/// A new unauthenticated connection supersedes the old one, so a probe cannot lock the owner out.
#[test]
fn an_unauthenticated_connection_never_blocks_a_new_one() {
    let mut auth = ClientAuth::new();
    let Some(first) = challenge(&mut auth, 0) else {
        return;
    };
    let Some(second) = challenge(&mut auth, 1) else {
        return;
    };
    assert_ne!(first, second, "the new attempt is a new connection");

    assert_eq!(
        auth.authenticate(first, proof(1), &proof(1), 2),
        ProofOutcome::Superseded
    );
    assert_eq!(
        auth.authenticate(second, proof(1), &proof(1), 2),
        ProofOutcome::Accepted
    );
}

/// Exactly one authenticated client holds the slot; a disconnect frees it.
#[test]
fn an_authenticated_client_holds_the_only_slot() {
    let mut auth = ClientAuth::new();
    let Some(connection) = challenge(&mut auth, 0) else {
        return;
    };
    assert_eq!(
        auth.authenticate(connection, proof(1), &proof(1), 1),
        ProofOutcome::Accepted
    );

    assert_eq!(auth.connect(2), ConnectOutcome::Busy);
    assert!(
        !auth.disconnect(connection.wrapping_add(1)),
        "an unknown connection frees nothing"
    );
    assert!(auth.disconnect(connection));
    assert_eq!(auth.current_connection(), None);
    assert!(
        challenge(&mut auth, 3).is_some(),
        "the freed slot accepts a new attempt"
    );
}

/// Five failures inside a minute close a one-minute refusal window, then the gate reopens.
#[test]
fn five_failed_proofs_within_one_minute_close_a_one_minute_refusal_window() {
    let mut auth = ClientAuth::new();
    let Some(connection) = challenge(&mut auth, 0) else {
        return;
    };

    for attempt in 0u32..4 {
        assert_eq!(
            auth.authenticate(connection, proof(1), &proof(2), attempt),
            ProofOutcome::Rejected { failures: attempt + 1 }
        );
    }
    assert_eq!(
        auth.authenticate(connection, proof(1), &proof(2), 4),
        ProofOutcome::Refused {
            until_ms: 4 + 60_000,
            failures: 5,
        }
    );

    assert_eq!(auth.connect(4), ConnectOutcome::Refused { until_ms: 60_004 });
    assert_eq!(
        auth.connect(60_003),
        ConnectOutcome::Refused { until_ms: 60_004 },
        "the refusal lasts the full 60 seconds"
    );
    assert!(
        challenge(&mut auth, 60_004).is_some(),
        "the gate reopens exactly at the deadline"
    );
}

/// A successful proof resets the failure counter, so four old failures do not arm a refusal.
#[test]
fn the_failure_counter_resets_on_success() {
    let mut auth = ClientAuth::new();
    let Some(connection) = challenge(&mut auth, 0) else {
        return;
    };

    for attempt in 0u32..4 {
        assert!(matches!(
            auth.authenticate(connection, proof(1), &proof(2), attempt),
            ProofOutcome::Rejected { .. }
        ));
    }
    assert_eq!(
        auth.authenticate(connection, proof(2), &proof(2), 4),
        ProofOutcome::Accepted
    );
    assert_eq!(auth.failures_within_window(4), 0);
    assert!(auth.disconnect(connection));

    let Some(next) = challenge(&mut auth, 10) else {
        return;
    };
    for attempt in 0u32..4 {
        assert!(matches!(
            auth.authenticate(next, proof(1), &proof(2), 10 + attempt),
            ProofOutcome::Rejected { .. }
        ));
    }
    assert!(
        challenge(&mut auth, 20).is_some(),
        "four failures after a success are not five"
    );
}

/// Failures older than the one-minute window do not count toward the limit.
#[test]
fn failures_older_than_the_window_do_not_count() {
    let mut auth = ClientAuth::new();
    let Some(connection) = challenge(&mut auth, 0) else {
        return;
    };
    for attempt in 0u32..4 {
        assert!(matches!(
            auth.authenticate(connection, proof(1), &proof(2), attempt),
            ProofOutcome::Rejected { .. }
        ));
    }

    // A fresh connection more than a minute later: only its own failure is inside the window.
    let Some(next) = challenge(&mut auth, 60_005) else {
        return;
    };
    assert_eq!(
        auth.authenticate(next, proof(1), &proof(2), 60_005),
        ProofOutcome::Rejected { failures: 1 }
    );
}

/// Proof comparison accepts only byte-for-byte equality, including every single-bit difference.
#[test]
fn proofs_match_only_when_every_byte_is_identical() {
    let expected = proof(0x5A);
    assert!(expected.matches(&proof(0x5A)));
    assert!(!expected.matches(&proof(0x5B)));

    for index in 0..auth::PROOF_LEN {
        let mut bytes = [0x5A; auth::PROOF_LEN];
        let Some(byte) = bytes.get_mut(index) else {
            return;
        };
        *byte = 0x5B;
        assert!(
            !expected.matches(&Proof::from_bytes(bytes)),
            "a difference at byte {index} must not match"
        );
    }

    assert!(constant_time_eq(expected.as_bytes(), &[0x5A; auth::PROOF_LEN]));
    assert!(!constant_time_eq(expected.as_bytes(), &[0x00; auth::PROOF_LEN]));
}
