//! Host tests for the Radio Session's station-link policy (ADR-0058).
//!
//! The session is driven with plain millisecond timestamps and observed radio facts: a provisioned
//! boot applies the station config, an existing association is adopted, a link loss reconnects on
//! the 1, 2, 4 … 60 s backoff, an association without a DHCP lease is recycled after 30 s, a failed
//! join waits for its retry gate, and the missing-credentials warning repeats once a minute. Every
//! assertion goes through the public reducer: `boot`, `step`, the accessors and the returned
//! outcome.

use bridge_core::{
    Action, CredentialRecord, Event, Input, JoinFailure, JoinResult, Observation, Outcome, Publication, RadioSession,
    portal::{self, Credentials},
    radio,
};
use link_protocol::StationSsid;

/// The Station Network the fixture record names.
const TEST_SSID: &[u8] = b"HomeNet";

/// The passphrase the fixture record carries; eight to sixty-three bytes is a usable WPA2-PSK pair.
const PASSPHRASE: &[u8] = b"correct horse";

/// The pairing token the fixture record carries; the session never reads it.
const TOKEN: [u8; 12] = *b"ABCDEFGHJKLM";

/// The station's leased address.
const STATION_IP: [u8; 4] = [192, 168, 1, 42];

/// The number of events one step can carry: the session's fixed capacity.
const EVENT_SLOTS: usize = 8;

/// A publication with no edges.
const NOTHING: Publication = Publication {
    provisioned: None,
    station_ssid: None,
    station_ssid_cleared: false,
    station_up: None,
};

/// Boots a provisioned session at `now_ms` from a record naming the test network, or returns `None`
/// if the fixture pair is not usable.
#[allow(
    clippy::large_stack_frames,
    reason = "the reducer's fixed-capacity outcome is returned by value; the helper only copies it"
)]
fn provisioned_boot(now_ms: u32) -> Option<(RadioSession, Outcome)> {
    let credentials = Credentials::from_slices(TEST_SSID, PASSPHRASE)?;
    let record = CredentialRecord {
        credentials: Some(credentials),
        token: TOKEN,
    };
    Some(RadioSession::boot(Some(&record), now_ms))
}

/// A credential pair the record decoder would reject: both fields are empty.
const fn unusable_credentials() -> Credentials {
    Credentials {
        ssid: [0; portal::MAX_SSID_BYTES],
        ssid_len: 0,
        passphrase: [0; portal::MAX_PASSPHRASE_BYTES],
        passphrase_len: 0,
    }
}

/// An observation of the radio at `now_ms`.
const fn observed(now_ms: u32, controller_connected: bool, station_ip: Option<[u8; 4]>) -> Observation {
    Observation {
        now_ms,
        controller_connected,
        station_ip,
    }
}

/// The actions of one outcome, in execution order.
fn actions(outcome: &Outcome) -> Vec<Action> {
    outcome.actions.iter().copied().collect()
}

/// The events of one outcome, in occurrence order.
fn events(outcome: &Outcome) -> Vec<Event> {
    outcome.events.iter().copied().collect()
}

/// A provisioned boot applies the station config and publishes the network; nothing else happens.
#[test]
fn a_provisioned_boot_applies_the_station_config_and_publishes_the_network() {
    let Some((session, boot)) = provisioned_boot(0) else {
        return;
    };

    assert_eq!(boot.actions.len(), 1);
    assert!(
        matches!(boot.actions.iter().next(), Some(Action::ApplyStation(_))),
        "a provisioned boot enters station mode with the record's credentials"
    );
    assert!(boot.events.is_empty(), "boot performs no I/O and logs no transition");
    assert_eq!(boot.publication.provisioned, Some(true));
    assert_eq!(boot.publication.station_ssid, StationSsid::new(TEST_SSID));
    assert!(!boot.publication.station_ssid_cleared);
    assert_eq!(boot.publication.station_up, Some(false));

    assert!(session.provisioned());
    assert_eq!(session.station_ssid(), StationSsid::new(TEST_SSID));
    assert!(!session.station_up());
    assert!(!session.retry_blocked());
}

/// An unprovisioned or unusable boot parks the station and publishes unprovisioned.
#[test]
fn an_unprovisioned_boot_parks_the_station_and_publishes_unprovisioned() {
    let record = CredentialRecord::token_only(TOKEN);
    let (session, boot) = RadioSession::boot(Some(&record), 0);

    assert_eq!(actions(&boot), vec![Action::ApplySetupParked]);
    assert!(boot.events.is_empty());
    assert_eq!(boot.publication.provisioned, Some(false));
    assert_eq!(boot.publication.station_ssid, None);
    assert!(
        boot.publication.station_ssid_cleared,
        "the unset network is published once at boot"
    );
    assert_eq!(boot.publication.station_up, Some(false));
    assert!(!session.provisioned());
    assert_eq!(session.station_ssid(), None);
    assert!(!session.station_up());

    // No record at all is the same unprovisioned boot.
    let (session, boot) = RadioSession::boot(None, 0);
    assert_eq!(actions(&boot), vec![Action::ApplySetupParked]);
    assert_eq!(boot.publication.provisioned, Some(false));
    assert!(!session.provisioned());

    // A record whose credential pair is unusable fails closed rather than joining.
    let record = CredentialRecord {
        credentials: Some(unusable_credentials()),
        token: TOKEN,
    };
    let (session, boot) = RadioSession::boot(Some(&record), 0);
    assert_eq!(actions(&boot), vec![Action::ApplySetupParked]);
    assert!(!session.provisioned());
    assert_eq!(session.station_ssid(), None);
}

/// An association the controller already holds is adopted, not rejoined, and is only station-up
/// once a DHCP lease arrives.
#[test]
fn adopting_an_existing_association_joins_nothing_and_waits_for_a_lease() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };

    let outcome = session.step(&observed(0, true, None), &[]);
    assert!(actions(&outcome).is_empty(), "the adopt path neither scans nor rejoins");
    assert!(events(&outcome).is_empty(), "adopting is not an association edge");
    assert_eq!(outcome.publication, NOTHING);
    assert!(
        !session.station_up(),
        "an association without a lease is not station-up"
    );

    let outcome = session.step(&observed(500, true, Some(STATION_IP)), &[]);
    assert!(actions(&outcome).is_empty(), "the adopted association keeps running");
    assert_eq!(outcome.publication.station_up, Some(true));
    assert!(session.station_up());
}

/// ADR-0058 preserves the quirk that the adopt path does not forgive a failed join.
#[test]
fn the_adopt_path_leaves_the_retry_block_alone() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };

    let outcome = session.step(&observed(0, false, None), &[]);
    assert_eq!(outcome.actions.len(), 1, "the first step asks for a join");
    let outcome = session.step(&observed(100, false, None), &[Input::Join(JoinResult::WrongPassword)]);
    assert_eq!(
        events(&outcome),
        vec![Event::JoinFailed {
            kind: JoinFailure::WrongPassword,
            retry_in_ms: radio::MIN_BACKOFF_MS,
        }]
    );
    assert!(session.retry_blocked());

    // The controller still holds an association, so the session adopts it; the block survives.
    let outcome = session.step(&observed(200, true, None), &[]);
    assert!(actions(&outcome).is_empty(), "the adopt path does not rejoin");
    assert!(events(&outcome).is_empty(), "adopting is not an association edge");
    assert!(session.retry_blocked(), "the adopt path must not reset the retry block");
}

/// A link loss is reported once, the rejoin waits for the one-second gate, and a connected result
/// re-associates with the minimum backoff restored.
#[test]
fn a_link_loss_reconnects_after_one_second_and_a_connected_result_reassociates() {
    let Some((mut session, boot)) = provisioned_boot(0) else {
        return;
    };
    let Some(ssid) = boot.publication.station_ssid else {
        return;
    };

    let outcome = session.step(&observed(0, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Join(ssid)]);

    let outcome = session.step(&observed(100, true, None), &[Input::Join(JoinResult::Connected)]);
    assert_eq!(events(&outcome), vec![Event::Associated]);
    assert!(actions(&outcome).is_empty(), "a connected join needs no further action");
    assert!(!session.station_up(), "associated without a lease is not station-up");

    // The controller drops the association.
    let outcome = session.step(&observed(5_000, false, None), &[]);
    assert_eq!(events(&outcome), vec![Event::LinkLost]);
    assert!(actions(&outcome).is_empty(), "the rejoin waits for the one-second gate");

    let outcome = session.step(&observed(5_999, false, None), &[]);
    assert!(actions(&outcome).is_empty(), "one millisecond before the gate");
    let outcome = session.step(&observed(6_000, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Join(ssid)]);

    let outcome = session.step(
        &observed(6_100, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    assert_eq!(events(&outcome), vec![Event::Associated]);
    assert_eq!(outcome.publication.station_up, Some(true));
    assert!(session.station_up());
}

/// An association that never receives a lease is disconnected after thirty seconds and retried.
#[test]
fn a_dhcp_timeout_disconnects_and_retries_after_one_second() {
    let Some((mut session, boot)) = provisioned_boot(0) else {
        return;
    };
    let Some(ssid) = boot.publication.station_ssid else {
        return;
    };

    let outcome = session.step(&observed(0, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Join(ssid)]);
    let outcome = session.step(&observed(100, true, None), &[Input::Join(JoinResult::Connected)]);
    assert_eq!(events(&outcome), vec![Event::Associated]);

    let outcome = session.step(&observed(30_099, true, None), &[]);
    assert!(actions(&outcome).is_empty(), "one millisecond before the timeout");
    assert!(
        events(&outcome).is_empty(),
        "the association is still inside its DHCP window"
    );

    let outcome = session.step(&observed(30_100, true, None), &[]);
    assert_eq!(events(&outcome), vec![Event::DhcpTimeout]);
    assert_eq!(actions(&outcome), vec![Action::Disconnect]);
    assert!(!session.station_up());

    let outcome = session.step(&observed(31_099, false, None), &[]);
    assert!(actions(&outcome).is_empty(), "the retry waits for the one-second gate");
    let outcome = session.step(&observed(31_100, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Join(ssid)]);
}

/// A failed join is not retried before its gate, and is retried at exactly its gate.
#[test]
fn a_failed_join_waits_for_the_retry_gate() {
    let Some((mut session, boot)) = provisioned_boot(0) else {
        return;
    };
    let Some(ssid) = boot.publication.station_ssid else {
        return;
    };

    let outcome = session.step(&observed(0, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Join(ssid)]);

    let outcome = session.step(&observed(1_000, false, None), &[Input::Join(JoinResult::Failed)]);
    assert_eq!(
        events(&outcome),
        vec![Event::JoinFailed {
            kind: JoinFailure::Failed,
            retry_in_ms: radio::MIN_BACKOFF_MS,
        }]
    );
    assert!(actions(&outcome).is_empty(), "the retry waits for its gate");

    let outcome = session.step(&observed(1_999, false, None), &[]);
    assert!(actions(&outcome).is_empty(), "one millisecond before the gate");
    let outcome = session.step(&observed(2_000, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Join(ssid)]);
}

/// Repeated failures double the wait from one second to the sixty-second cap.
#[test]
fn the_reconnect_backoff_doubles_to_one_minute() {
    let Some((mut session, boot)) = provisioned_boot(0) else {
        return;
    };
    let Some(ssid) = boot.publication.station_ssid else {
        return;
    };

    let mut now_ms = 0;
    let outcome = session.step(&observed(now_ms, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Join(ssid)]);

    for expected_retry_ms in [1_000, 2_000, 4_000, 8_000, 16_000, 32_000, 60_000, 60_000] {
        let outcome = session.step(&observed(now_ms, false, None), &[Input::Join(JoinResult::Failed)]);
        assert_eq!(
            events(&outcome),
            vec![Event::JoinFailed {
                kind: JoinFailure::Failed,
                retry_in_ms: expected_retry_ms,
            }],
            "the failure at {now_ms} ms reports the {expected_retry_ms} ms wait"
        );

        now_ms += expected_retry_ms;
        let outcome = session.step(&observed(now_ms - 1, false, None), &[]);
        assert!(
            actions(&outcome).is_empty(),
            "no join one millisecond before the {now_ms} ms gate"
        );
        let outcome = session.step(&observed(now_ms, false, None), &[]);
        assert_eq!(
            actions(&outcome),
            vec![Action::Join(ssid)],
            "a join at exactly the {now_ms} ms gate"
        );
    }
}

/// The bounded sets keep every slot they have and flag the entries they refused.
#[test]
fn a_step_with_more_events_than_slots_flags_the_overflow() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };

    let failures = [Input::Join(JoinResult::Failed); EVENT_SLOTS + 1];
    let outcome = session.step(&observed(0, false, None), &failures);

    assert_eq!(
        outcome.events.len(),
        EVENT_SLOTS,
        "the set stores as many events as it has slots"
    );
    assert!(
        outcome.events.overflowed(),
        "the refused event is flagged instead of dropped silently"
    );
    assert!(outcome.actions.is_empty(), "a blocked retry emits no action");
    assert!(!outcome.actions.overflowed());
}

/// A rejected passphrase and a generic failure are reported as distinct kinds, and the generic
/// failure doubles the wait the rejection reported.
#[test]
fn a_wrong_passphrase_and_a_generic_failure_are_distinct() {
    let Some((mut session, boot)) = provisioned_boot(0) else {
        return;
    };
    let Some(ssid) = boot.publication.station_ssid else {
        return;
    };

    let outcome = session.step(&observed(0, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Join(ssid)]);
    let outcome = session.step(&observed(10, false, None), &[Input::Join(JoinResult::WrongPassword)]);
    assert_eq!(
        events(&outcome),
        vec![Event::JoinFailed {
            kind: JoinFailure::WrongPassword,
            retry_in_ms: radio::MIN_BACKOFF_MS,
        }]
    );

    let outcome = session.step(&observed(1_010, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Join(ssid)]);
    let outcome = session.step(&observed(1_020, false, None), &[Input::Join(JoinResult::Failed)]);
    assert_eq!(
        events(&outcome),
        vec![Event::JoinFailed {
            kind: JoinFailure::Failed,
            retry_in_ms: 2_000,
        }]
    );
}

/// An unprovisioned session warns at the boot step and then exactly once a minute.
#[test]
fn the_missing_credentials_warning_repeats_every_minute() {
    let record = CredentialRecord::token_only(TOKEN);
    let (mut session, boot) = RadioSession::boot(Some(&record), 100);
    assert!(boot.events.is_empty(), "boot itself does not warn");

    let outcome = session.step(&observed(100, false, None), &[]);
    assert_eq!(events(&outcome), vec![Event::MissingCredentials]);
    assert!(actions(&outcome).is_empty(), "an unprovisioned session never joins");

    let outcome = session.step(&observed(60_099, false, None), &[]);
    assert!(events(&outcome).is_empty(), "one millisecond before the next warning");
    let outcome = session.step(&observed(60_100, false, None), &[]);
    assert_eq!(events(&outcome), vec![Event::MissingCredentials]);
    let outcome = session.step(&observed(120_100, false, None), &[]);
    assert_eq!(events(&outcome), vec![Event::MissingCredentials]);
}

/// A step that changes no published fact publishes nothing.
#[test]
fn a_step_that_changes_nothing_publishes_nothing() {
    let Some((mut session, boot)) = provisioned_boot(0) else {
        return;
    };
    assert_eq!(boot.publication.provisioned, Some(true));

    // The first step asks for a join but changes no published fact.
    let outcome = session.step(&observed(0, false, None), &[]);
    assert_eq!(outcome.actions.len(), 1);
    assert_eq!(outcome.publication, NOTHING);

    // A repeat step while the join is in flight is silent and does not re-emit the join.
    let outcome = session.step(&observed(500, false, None), &[]);
    assert!(actions(&outcome).is_empty(), "the pending join is not re-emitted");
    assert!(
        events(&outcome).is_empty(),
        "nothing happened while the join is in flight"
    );
    assert_eq!(outcome.publication, NOTHING);

    // The lease turns the station up; a repeat step does not republish it.
    let outcome = session.step(
        &observed(600, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    assert_eq!(outcome.publication.station_up, Some(true));
    let outcome = session.step(&observed(1_000, true, Some(STATION_IP)), &[]);
    assert_eq!(outcome.publication, NOTHING);
}
