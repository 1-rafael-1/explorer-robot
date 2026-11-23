//! Host tests for the Radio Session's station-link, Setup Mode and provisioning policy (ADR-0058).
//!
//! The session is driven with plain millisecond timestamps and observed radio facts: a provisioned
//! boot applies the station config, an existing association is adopted, a link loss reconnects on
//! the 1, 2, 4 … 60 s backoff, an association without a DHCP lease is recycled after 30 s, a failed
//! join waits for its retry gate, and the missing-credentials warning repeats once a minute. Setup
//! Mode raises on a blank record or a five-minute outage, closes on the expiry or a stable
//! association, and owns the portal and panel Forget paths. A portal provision persists before it
//! joins and fails closed on a bad write. The derived portal status, the five-minute scan gate and
//! its preserved clock quirks, the decision counters and the Bridge Status accessor's parity with
//! the firmware are pinned here too. Every assertion goes through the public reducer: `boot`,
//! `step`, the accessors and the returned outcome.

use bridge_core::{
    Action, BridgeStatusView, CloseReason, CredentialRecord, Event, Input, JoinFailure, JoinResult, Observation,
    Outcome, PanelCommand, PortalCommand, Publication, RadioSession, RaiseReason, SetupEvent, Stats,
    portal::{self, Credentials, Status},
    radio,
};
use link_protocol::{BridgeStation, BridgeStatusFlags, StationSsid};

/// The Station Network the fixture record names.
const TEST_SSID: &[u8] = b"HomeNet";

/// The passphrase the fixture record carries; eight to sixty-three bytes is a usable WPA2-PSK pair.
const PASSPHRASE_TEXT: &str = "correct horse";

/// The passphrase bytes the fixture record carries.
const PASSPHRASE: &[u8] = PASSPHRASE_TEXT.as_bytes();

/// A second Station Network a portal submission names.
const NEW_SSID: &[u8] = b"NewNet";

/// The second network's passphrase.
const NEW_PASSPHRASE: &[u8] = b"another secret";

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
    setup_open: None,
    station_up: None,
    portal_status: None,
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
    assert_eq!(boot.publication.setup_open, Some(false));
    assert_eq!(boot.publication.station_up, Some(false));
    assert_eq!(boot.publication.portal_status, Some(Status::Connecting));

    assert!(session.provisioned());
    assert_eq!(session.station_ssid(), StationSsid::new(TEST_SSID));
    assert!(!session.station_up());
    assert!(!session.setup_open());
    assert_eq!(session.portal_status(), Status::Connecting);
    assert!(!session.operator_blocked());
}

/// An unprovisioned or unusable boot parks the station and publishes unprovisioned.
#[test]
fn an_unprovisioned_boot_parks_the_station_and_publishes_unprovisioned() {
    let record = CredentialRecord::token_only(TOKEN);
    let (session, boot) = RadioSession::boot(Some(&record), 0);

    assert_eq!(actions(&boot), vec![Action::ApplySetupParked]);
    assert_eq!(
        events(&boot),
        vec![Event::Setup(SetupEvent::Opened(RaiseReason::Unprovisioned))],
        "an unusable record raises Setup Mode at boot"
    );
    assert_eq!(boot.publication.provisioned, Some(false));
    assert_eq!(boot.publication.station_ssid, None);
    assert!(
        boot.publication.station_ssid_cleared,
        "the unset network is published once at boot"
    );
    assert_eq!(boot.publication.setup_open, Some(true));
    assert_eq!(boot.publication.station_up, Some(false));
    assert_eq!(boot.publication.portal_status, Some(Status::Unprovisioned));
    assert!(!session.provisioned());
    assert_eq!(session.station_ssid(), None);
    assert!(!session.station_up());
    assert!(session.setup_open());
    assert_eq!(session.portal_status(), Status::Unprovisioned);

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
    assert!(session.operator_blocked());

    // The controller still holds an association, so the session adopts it; the block survives.
    let outcome = session.step(&observed(200, true, None), &[]);
    assert!(actions(&outcome).is_empty(), "the adopt path does not rejoin");
    assert!(events(&outcome).is_empty(), "adopting is not an association edge");
    assert!(
        session.operator_blocked(),
        "the adopt path must not reset the retry block"
    );
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
    assert_eq!(
        events(&boot),
        vec![Event::Setup(SetupEvent::Opened(RaiseReason::Unprovisioned))],
        "boot raises Setup Mode but logs no warning"
    );

    let outcome = session.step(&observed(100, false, None), &[]);
    assert_eq!(events(&outcome), vec![Event::MissingCredentials]);
    assert_eq!(
        actions(&outcome),
        vec![Action::Scan],
        "an unprovisioned session scans the open window and never joins"
    );

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

/// A blank or unusable Credential Record raises Setup Mode at boot.
#[test]
fn a_blank_or_unusable_record_raises_setup_mode_at_boot() {
    let record = CredentialRecord::token_only(TOKEN);
    let (session, boot) = RadioSession::boot(Some(&record), 0);

    assert_eq!(
        events(&boot),
        vec![Event::Setup(SetupEvent::Opened(RaiseReason::Unprovisioned))]
    );
    assert_eq!(actions(&boot), vec![Action::ApplySetupParked]);
    assert_eq!(boot.publication.setup_open, Some(true));
    assert_eq!(boot.publication.portal_status, Some(Status::Unprovisioned));
    assert!(session.setup_open());
    assert_eq!(session.portal_status(), Status::Unprovisioned);

    // A record whose credential pair is unusable fails closed the same way.
    let record = CredentialRecord {
        credentials: Some(unusable_credentials()),
        token: TOKEN,
    };
    let (session, boot) = RadioSession::boot(Some(&record), 0);
    assert_eq!(
        events(&boot),
        vec![Event::Setup(SetupEvent::Opened(RaiseReason::Unprovisioned))]
    );
    assert_eq!(actions(&boot), vec![Action::ApplySetupParked]);
    assert!(session.setup_open());
    assert!(!session.provisioned());
}

/// The five-minute outage raise fires once, the ten-minute expiry that follows closes it, and the
/// spent opportunity never raises again in the same boot.
#[test]
fn the_outage_raise_fires_once_and_the_expiry_closes_it() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };

    // The first step's join stays pending, so the station remains down without a second join.
    let _ = session.step(&observed(0, false, None), &[]);

    let outcome = session.step(&observed(299_999, false, None), &[]);
    assert!(events(&outcome).is_empty(), "one millisecond before the outage");

    let outcome = session.step(&observed(300_000, false, None), &[]);
    assert_eq!(
        events(&outcome),
        vec![Event::Setup(SetupEvent::Opened(RaiseReason::StationOutage))]
    );
    assert!(matches!(actions(&outcome).as_slice(), [Action::ApplyApStation(_)]));
    assert_eq!(outcome.publication.setup_open, Some(true));
    assert!(session.setup_open());

    let outcome = session.step(&observed(899_999, false, None), &[]);
    assert!(events(&outcome).is_empty(), "one millisecond before the expiry");

    let outcome = session.step(&observed(900_000, false, None), &[]);
    assert_eq!(
        events(&outcome),
        vec![Event::Setup(SetupEvent::Closed(CloseReason::Expired))]
    );
    assert!(matches!(actions(&outcome).as_slice(), [Action::ApplyStation(_)]));
    assert_eq!(outcome.publication.setup_open, Some(false));
    assert!(!session.setup_open());

    // The once-per-boot opportunity is spent: closing the window does not re-raise it.
    let outcome = session.step(&observed(1_500_000, false, None), &[]);
    assert!(events(&outcome).is_empty(), "the outage never raises twice");
}

/// A station-up edge inside an open window closes it thirty seconds later, boundary exact.
#[test]
fn a_stable_association_closes_the_window_thirty_seconds_after_the_rise() {
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let (mut session, _boot) = RadioSession::boot(None, 0);

    let _ = session.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    let _ = session.step(&observed(10, false, None), &[Input::Persist(true)]);

    // The lease turns the station up at 20 ms; the rise arms the close at 30 020 ms.
    let outcome = session.step(
        &observed(20, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    assert_eq!(outcome.publication.station_up, Some(true));

    let outcome = session.step(&observed(30_019, true, Some(STATION_IP)), &[]);
    assert!(events(&outcome).is_empty(), "one millisecond before the stable close");

    let outcome = session.step(&observed(30_020, true, Some(STATION_IP)), &[]);
    assert_eq!(
        events(&outcome),
        vec![Event::Setup(SetupEvent::Closed(CloseReason::StationStable))]
    );
    assert!(matches!(actions(&outcome).as_slice(), [Action::ApplyStation(_)]));
    assert_eq!(outcome.publication.setup_open, Some(false));
    assert!(!session.setup_open());
}

/// A portal provision stages the pair, then adopts it only after a successful write.
#[test]
fn a_portal_provision_persists_before_it_mutates_the_network() {
    let Some(credentials) = Credentials::from_slices(NEW_SSID, NEW_PASSPHRASE) else {
        return;
    };
    let Some(ssid) = StationSsid::new(NEW_SSID) else {
        return;
    };
    let (mut session, _boot) = RadioSession::boot(None, 0);

    // The submitted pair is only staged: nothing on the network changes and no join is emitted.
    let outcome = session.step(
        &observed(0, true, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    assert_eq!(
        actions(&outcome),
        vec![Action::Persist(credentials), Action::Scan],
        "staging the pair persists it, and the open window's first scan is due"
    );
    assert_eq!(
        events(&outcome),
        vec![Event::MissingCredentials],
        "staging a provision emits no network event beyond the pending warning"
    );
    assert_eq!(outcome.publication, NOTHING);
    assert!(!session.provisioned());
    assert_eq!(session.station_ssid(), None);

    // The write adopts the pair, disconnects the stale association and joins the new network.
    let outcome = session.step(&observed(10, true, None), &[Input::Persist(true)]);
    assert_eq!(
        actions(&outcome),
        vec![
            Action::Disconnect,
            Action::ApplyApStation(credentials),
            Action::Join(ssid),
        ]
    );
    assert_eq!(outcome.publication.provisioned, Some(true));
    assert_eq!(outcome.publication.station_ssid, Some(ssid));
    assert_eq!(outcome.publication.portal_status, Some(Status::Connecting));
    assert!(session.provisioned());
    assert_eq!(session.station_ssid(), Some(ssid));
}

/// A failed credential write fails closed: the old network stays in force and failure is published.
#[test]
fn a_failed_persist_keeps_the_old_network_and_reports_failure() {
    let Some(credentials) = Credentials::from_slices(NEW_SSID, NEW_PASSPHRASE) else {
        return;
    };
    let Some((mut session, boot)) = provisioned_boot(0) else {
        return;
    };
    let Some(old_ssid) = boot.publication.station_ssid else {
        return;
    };

    // Bring the old network up so "joins nothing" is observable.
    let _ = session.step(&observed(0, false, None), &[]);
    let _ = session.step(
        &observed(100, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    assert!(session.station_up());

    let outcome = session.step(
        &observed(200, true, Some(STATION_IP)),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    assert_eq!(actions(&outcome), vec![Action::Persist(credentials)]);

    let outcome = session.step(&observed(300, true, Some(STATION_IP)), &[Input::Persist(false)]);
    assert_eq!(events(&outcome), vec![Event::PersistFailed]);
    assert!(actions(&outcome).is_empty(), "a failed write joins nothing");
    // The old association is still up, and the Connected overwrite outranks the recorded failure
    // (ADR-0058).
    assert_eq!(outcome.publication.portal_status, None);
    assert_eq!(session.portal_status(), Status::Connected(STATION_IP));
    assert!(session.provisioned(), "the old credentials remain in force");
    assert_eq!(session.station_ssid(), Some(old_ssid));
    assert!(session.station_up(), "the old association is untouched");

    // The overwrite also clears the recorded failure, so the link-loss edge derives Connecting,
    // exactly as the firmware does.
    let outcome = session.step(&observed(400, false, None), &[]);
    assert_eq!(events(&outcome), vec![Event::LinkLost]);
    assert_eq!(outcome.publication.portal_status, Some(Status::Connecting));
    assert_eq!(session.portal_status(), Status::Connecting);
}

/// A staged credential write suppresses the old network's due join until the write resolves.
#[test]
fn a_pending_write_suppresses_the_old_networks_join() {
    let Some(replacement) = Credentials::from_slices(NEW_SSID, NEW_PASSPHRASE) else {
        return;
    };
    let Some(replacement_ssid) = StationSsid::new(NEW_SSID) else {
        return;
    };
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };

    // The join is in flight when the five-minute outage opens the window.
    let _ = session.step(&observed(0, false, None), &[]);
    let opening = session.step(&observed(300_000, false, None), &[]);
    assert!(events(&opening).contains(&Event::Setup(SetupEvent::Opened(RaiseReason::StationOutage))));

    // The association comes up and drops well before the stable close, so the window stays open
    // and the retry gate opens on the one-second backoff.
    let _ = session.step(
        &observed(300_100, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    let lost = session.step(&observed(300_200, false, None), &[]);
    assert_eq!(events(&lost), vec![Event::LinkLost]);
    assert!(session.setup_open());

    // Staging the pair persists it and suppresses the old network's join; no mode action is
    // emitted either, because the window was already open.
    let staging = session.step(
        &observed(301_200, false, None),
        &[Input::Portal(PortalCommand::Provision(replacement))],
    );
    let staged_actions = actions(&staging);
    assert!(staged_actions.contains(&Action::Persist(replacement)));
    assert!(
        !staged_actions.iter().any(|action| matches!(action, Action::Join(_))),
        "the old network is not rejoined while the write is pending"
    );
    assert!(
        !staged_actions.iter().any(|action| matches!(
            action,
            Action::ApplyStation(_) | Action::ApplyApStation(_) | Action::ApplySetupParked | Action::ApplyParkedStation
        )),
        "a staged write changes no radio mode"
    );

    // The adopted write joins the new network, not the old one.
    let adopted = session.step(&observed(301_210, false, None), &[Input::Persist(true)]);
    assert_eq!(
        actions(&adopted),
        vec![Action::ApplyApStation(replacement), Action::Join(replacement_ssid)]
    );
}

/// Both Forget paths clear the network and raise Setup Mode; only the portal keeps the token.
#[test]
fn both_forget_paths_clear_the_network_and_open_setup_mode() {
    let Some((mut portal, _boot)) = provisioned_boot(0) else {
        return;
    };
    let outcome = portal.step(&observed(0, true, None), &[Input::Portal(PortalCommand::Forget)]);
    assert_eq!(
        actions(&outcome),
        vec![
            Action::PersistTokenOnly,
            Action::Disconnect,
            Action::ApplySetupParked,
            Action::Scan,
        ]
    );
    assert_eq!(
        events(&outcome),
        vec![
            Event::Setup(SetupEvent::Opened(RaiseReason::Forgotten)),
            Event::MissingCredentials,
        ]
    );
    assert_eq!(outcome.publication.provisioned, Some(false));
    assert!(outcome.publication.station_ssid_cleared);
    assert_eq!(outcome.publication.setup_open, Some(true));
    assert_eq!(outcome.publication.portal_status, Some(Status::Unprovisioned));
    assert!(!portal.provisioned());
    assert!(portal.setup_open());
    assert_eq!(portal.portal_status(), Status::Unprovisioned);

    let Some((mut panel, _boot)) = provisioned_boot(0) else {
        return;
    };
    let outcome = panel.step(&observed(0, true, None), &[Input::Panel(PanelCommand::Forget)]);
    assert_eq!(
        actions(&outcome),
        vec![Action::Disconnect, Action::ApplySetupParked, Action::Scan]
    );
    assert_eq!(
        events(&outcome),
        vec![
            Event::Setup(SetupEvent::Opened(RaiseReason::Forgotten)),
            Event::MissingCredentials,
        ]
    );
    assert!(!panel.provisioned());
    assert!(panel.setup_open());
    assert_eq!(panel.portal_status(), Status::Unprovisioned);
}

/// Runs one Forget path from an identical station-up state and returns the time and events of the
/// outage raise that follows.
#[allow(
    clippy::large_stack_frames,
    reason = "the reducer's fixed-capacity outcome is returned by value; the helper only copies it"
)]
fn forget_outage_timeline(forget: Input) -> Option<(u32, Vec<Event>)> {
    let (mut session, _boot) = provisioned_boot(0)?;

    let _ = session.step(&observed(0, false, None), &[]);
    let _ = session.step(
        &observed(100, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    let _ = session.step(&observed(200, true, Some(STATION_IP)), &[forget]);

    // Bring the station back up so the raised window closes on stability before five minutes,
    // leaving the once-per-boot outage opportunity unspent for the drop that follows.
    let _ = session.step(
        &observed(300, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    let close = session.step(&observed(30_300, true, Some(STATION_IP)), &[]);
    assert_eq!(
        events(&close),
        vec![Event::Setup(SetupEvent::Closed(CloseReason::StationStable))]
    );

    let _ = session.step(&observed(30_301, false, None), &[]);
    let before = session.step(&observed(330_300, false, None), &[]);
    assert!(
        !events(&before).contains(&Event::Setup(SetupEvent::Opened(RaiseReason::StationOutage))),
        "one millisecond before the restarted outage clock matures"
    );
    let opened = session.step(&observed(330_301, false, None), &[]);
    Some((330_301, events(&opened)))
}

/// Portal Forget and panel Forget start the outage clock identically (ADR-0058's parity fix).
#[test]
fn both_forget_paths_start_the_outage_clock_identically() {
    let Some(portal) = forget_outage_timeline(Input::Portal(PortalCommand::Forget)) else {
        return;
    };
    let Some(panel) = forget_outage_timeline(Input::Panel(PanelCommand::Forget)) else {
        return;
    };

    assert_eq!(portal.0, panel.0, "both paths raise the outage at the same time");
    assert_eq!(portal.1, panel.1);
    assert!(
        portal
            .1
            .contains(&Event::Setup(SetupEvent::Opened(RaiseReason::StationOutage))),
        "the outage does raise"
    );
}

/// Inside an open window the operator block suppresses a failed join's retry; a written provision
/// clears it.
#[test]
fn an_open_window_blocks_a_failed_join_retry_until_a_provision_clears_it() {
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let Some(replacement) = Credentials::from_slices(NEW_SSID, NEW_PASSPHRASE) else {
        return;
    };
    let Some(replacement_ssid) = StationSsid::new(NEW_SSID) else {
        return;
    };
    let (mut session, _boot) = RadioSession::boot(None, 0);

    let _ = session.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    let outcome = session.step(&observed(10, false, None), &[Input::Persist(true)]);
    assert!(
        actions(&outcome).iter().any(|action| matches!(action, Action::Join(_))),
        "the successful write joins"
    );

    let outcome = session.step(&observed(20, false, None), &[Input::Join(JoinResult::Failed)]);
    assert_eq!(
        events(&outcome),
        vec![Event::JoinFailed {
            kind: JoinFailure::Failed,
            retry_in_ms: radio::MIN_BACKOFF_MS,
        }]
    );
    assert!(session.operator_blocked());
    assert!(session.setup_open());

    // The one-second gate is reached, but the open window suppresses the retry.
    let outcome = session.step(&observed(1_020, false, None), &[]);
    assert!(actions(&outcome).is_empty(), "the operator block suppresses the retry");
    let outcome = session.step(&observed(5_000, false, None), &[]);
    assert!(actions(&outcome).is_empty(), "the retry stays suppressed");

    // A new submission clears the block once its write succeeds, and the join is emitted.
    let outcome = session.step(
        &observed(6_000, false, None),
        &[Input::Portal(PortalCommand::Provision(replacement))],
    );
    assert_eq!(actions(&outcome), vec![Action::Persist(replacement)]);
    let outcome = session.step(&observed(6_010, false, None), &[Input::Persist(true)]);
    assert_eq!(
        actions(&outcome),
        vec![Action::ApplyApStation(replacement), Action::Join(replacement_ssid)]
    );
    assert!(!session.operator_blocked());
}

/// No event carries the passphrase, whatever the provisioning outcome.
#[test]
fn no_event_carries_the_passphrase() {
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let (mut session, boot) = RadioSession::boot(None, 0);
    let mut emitted = events(&boot);

    let outcome = session.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    emitted.extend(events(&outcome));
    let outcome = session.step(&observed(10, false, None), &[Input::Persist(true)]);
    emitted.extend(events(&outcome));
    let outcome = session.step(&observed(20, false, None), &[Input::Join(JoinResult::WrongPassword)]);
    emitted.extend(events(&outcome));
    let outcome = session.step(
        &observed(1_020, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    emitted.extend(events(&outcome));
    let outcome = session.step(&observed(1_030, false, None), &[Input::Persist(false)]);
    emitted.extend(events(&outcome));

    for event in &emitted {
        let text = format!("{event:?}");
        assert!(
            !text.contains(PASSPHRASE_TEXT),
            "an event leaked the passphrase: {text}"
        );
    }
}

/// The portal status follows the provisioned, join-failure and station-up facts.
#[test]
fn the_portal_status_follows_the_provision_join_and_link_facts() {
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let (mut session, boot) = RadioSession::boot(None, 0);
    assert_eq!(boot.publication.portal_status, Some(Status::Unprovisioned));

    // Staging a provision changes no status; the successful write moves to Connecting.
    let outcome = session.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    assert_eq!(session.portal_status(), Status::Unprovisioned);
    assert_eq!(outcome.publication.portal_status, None);
    let outcome = session.step(&observed(10, false, None), &[Input::Persist(true)]);
    assert_eq!(session.portal_status(), Status::Connecting);
    assert_eq!(outcome.publication.portal_status, Some(Status::Connecting));

    // A rejected passphrase wins until a later attempt associates.
    let outcome = session.step(&observed(20, false, None), &[Input::Join(JoinResult::WrongPassword)]);
    assert_eq!(session.portal_status(), Status::WrongPassword);
    assert_eq!(outcome.publication.portal_status, Some(Status::WrongPassword));

    let outcome = session.step(
        &observed(1_020, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    assert_eq!(session.portal_status(), Status::Connected(STATION_IP));
    assert_eq!(outcome.publication.portal_status, Some(Status::Connected(STATION_IP)));

    // A link loss falls back to Connecting.
    let outcome = session.step(&observed(2_000, false, None), &[]);
    assert_eq!(events(&outcome), vec![Event::LinkLost]);
    assert_eq!(session.portal_status(), Status::Connecting);
    assert_eq!(outcome.publication.portal_status, Some(Status::Connecting));
}

/// A generic join failure reports Failed, as a failed persist does when the station is down.
#[test]
fn a_generic_join_failure_and_a_failed_persist_report_failed() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };
    let _ = session.step(&observed(0, false, None), &[]);
    let outcome = session.step(&observed(10, false, None), &[Input::Join(JoinResult::Failed)]);
    assert_eq!(session.portal_status(), Status::Failed);
    assert_eq!(outcome.publication.portal_status, Some(Status::Failed));

    // A failed write with no association in force reports Failed directly.
    let Some(credentials) = Credentials::from_slices(NEW_SSID, NEW_PASSPHRASE) else {
        return;
    };
    let (mut unprovisioned, _boot) = RadioSession::boot(None, 0);
    let _ = unprovisioned.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    let outcome = unprovisioned.step(&observed(10, false, None), &[Input::Persist(false)]);
    assert_eq!(outcome.publication.portal_status, Some(Status::Failed));
    assert_eq!(unprovisioned.portal_status(), Status::Failed);
}

/// Connected is reported only while the station is up; an association without a lease is not.
#[test]
fn connected_is_reported_only_while_the_station_is_up() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };

    // Adopting an association that holds a lease is Connected.
    let outcome = session.step(&observed(0, true, Some(STATION_IP)), &[]);
    assert!(session.station_up());
    assert_eq!(session.portal_status(), Status::Connected(STATION_IP));
    assert_eq!(outcome.publication.portal_status, Some(Status::Connected(STATION_IP)));

    // The link loss drops the lease with it.
    let outcome = session.step(&observed(1_000, false, None), &[]);
    assert_eq!(events(&outcome), vec![Event::LinkLost]);
    assert!(!session.station_up());
    assert_eq!(session.portal_status(), Status::Connecting);

    // A connected result without an IP stays Connecting until DHCP leases one.
    let outcome = session.step(&observed(2_000, true, None), &[Input::Join(JoinResult::Connected)]);
    assert!(!session.station_up());
    assert_eq!(session.portal_status(), Status::Connecting);
    assert_eq!(outcome.publication.portal_status, None);
}

/// An open window scans on the first step and again only after the refresh interval.
#[test]
fn an_open_window_scans_once_and_again_only_after_five_minutes() {
    let (mut session, _boot) = RadioSession::boot(None, 0);

    let outcome = session.step(&observed(0, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Scan], "the first step scans");

    let outcome = session.step(&observed(radio::SCAN_REFRESH_MS - 1, false, None), &[]);
    assert!(
        !actions(&outcome).contains(&Action::Scan),
        "one millisecond before the refresh"
    );

    let outcome = session.step(&observed(radio::SCAN_REFRESH_MS, false, None), &[]);
    assert_eq!(actions(&outcome), vec![Action::Scan], "exactly at the refresh");
}

/// A join in flight derives Connecting and suppresses the scan, even when the clock is due.
#[test]
fn the_scan_is_suppressed_while_a_join_is_in_flight() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };
    let outcome = session.step(&observed(0, false, None), &[]);
    assert!(matches!(actions(&outcome).as_slice(), [Action::Join(_)]));

    // The five-minute outage raise opens the window while the join is still in flight.
    let outcome = session.step(&observed(300_000, false, None), &[]);
    assert!(events(&outcome).contains(&Event::Setup(SetupEvent::Opened(RaiseReason::StationOutage))));
    assert_eq!(session.portal_status(), Status::Connecting);
    assert!(
        !actions(&outcome).contains(&Action::Scan),
        "a sweep would disturb the association attempt"
    );
}

/// A station that is up suppresses the scan even when the window and clock are due.
#[test]
fn the_scan_is_suppressed_while_the_station_is_up() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };
    let _ = session.step(&observed(0, false, None), &[]);

    // The window opens at five minutes and the adopted association is up on the same step.
    let outcome = session.step(&observed(300_000, true, Some(STATION_IP)), &[]);
    assert!(events(&outcome).contains(&Event::Setup(SetupEvent::Opened(RaiseReason::StationOutage))));
    assert!(session.station_up());
    assert_eq!(session.portal_status(), Status::Connected(STATION_IP));
    assert!(!actions(&outcome).contains(&Action::Scan));
}

/// A failed join inside the open window lets the next due scan run.
#[test]
fn a_failed_join_inside_the_window_lets_the_next_scan_run() {
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let (mut session, _boot) = RadioSession::boot(None, 0);

    let _ = session.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    let _ = session.step(&observed(10, false, None), &[Input::Persist(true)]);
    let outcome = session.step(&observed(20, false, None), &[Input::Join(JoinResult::Failed)]);
    assert_eq!(session.portal_status(), Status::Failed);
    assert!(
        !actions(&outcome).contains(&Action::Scan),
        "the scan clock is not yet due"
    );

    let outcome = session.step(&observed(radio::SCAN_REFRESH_MS, false, None), &[]);
    assert_eq!(
        actions(&outcome),
        vec![Action::Scan],
        "a recorded failure no longer suppresses the sweep"
    );
}

/// A closed window never scans, even at the clock's deadline.
#[test]
fn a_closed_window_never_scans() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };
    let outcome = session.step(&observed(0, false, None), &[]);
    assert!(matches!(actions(&outcome).as_slice(), [Action::Join(_)]));
    assert!(!actions(&outcome).contains(&Action::Scan));
}

/// A panel Forget that opens a closed window restarts the scan clock, so a sweep is due at once.
#[test]
fn a_panel_forget_that_opens_the_window_restarts_the_scan_clock() {
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let (mut session, _boot) = RadioSession::boot(None, 0);

    let _ = session.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    let _ = session.step(&observed(10, false, None), &[Input::Persist(true)]);
    let _ = session.step(
        &observed(20, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    let close = session.step(&observed(30_020, true, Some(STATION_IP)), &[]);
    assert!(events(&close).contains(&Event::Setup(SetupEvent::Closed(CloseReason::StationStable))));
    assert!(!session.setup_open());

    // The window is closed, so the Forget opens it and resets the clock to now.
    let outcome = session.step(&observed(40_000, false, None), &[Input::Panel(PanelCommand::Forget)]);
    assert!(session.setup_open());
    assert!(
        actions(&outcome).contains(&Action::Scan),
        "the restarted clock makes a sweep due at the Forget"
    );
}

/// A portal Forget leaves the scan clock alone, even though it opens the window.
#[test]
fn a_portal_forget_does_not_restart_the_scan_clock() {
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let (mut session, _boot) = RadioSession::boot(None, 0);

    let _ = session.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    let _ = session.step(&observed(10, false, None), &[Input::Persist(true)]);
    let _ = session.step(
        &observed(20, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    let close = session.step(&observed(30_020, true, Some(STATION_IP)), &[]);
    assert!(events(&close).contains(&Event::Setup(SetupEvent::Closed(CloseReason::StationStable))));

    let outcome = session.step(
        &observed(40_000, true, Some(STATION_IP)),
        &[Input::Portal(PortalCommand::Forget)],
    );
    assert!(session.setup_open());
    assert!(
        !actions(&outcome).contains(&Action::Scan),
        "the portal Forget keeps the scan schedule"
    );
    let outcome = session.step(&observed(radio::SCAN_REFRESH_MS - 1, false, None), &[]);
    assert!(!actions(&outcome).contains(&Action::Scan));
    let outcome = session.step(&observed(radio::SCAN_REFRESH_MS, false, None), &[]);
    assert!(
        actions(&outcome).contains(&Action::Scan),
        "the original five-minute clock still governs"
    );
}

/// A successful provision leaves the scan clock alone (ADR-0058's preserved quirk).
#[test]
fn a_successful_provision_does_not_restart_the_scan_clock() {
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let (mut session, _boot) = RadioSession::boot(None, 0);

    let _ = session.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    let _ = session.step(&observed(10, false, None), &[Input::Persist(true)]);
    let _ = session.step(&observed(20, false, None), &[Input::Join(JoinResult::Failed)]);

    let outcome = session.step(&observed(radio::SCAN_REFRESH_MS - 1, false, None), &[]);
    assert!(!actions(&outcome).contains(&Action::Scan));
    let outcome = session.step(&observed(radio::SCAN_REFRESH_MS, false, None), &[]);
    assert_eq!(
        actions(&outcome),
        vec![Action::Scan],
        "the clock still dates from the window open, not the provision"
    );
}

/// Opening the window from the outage tick makes the first sweep due at the open.
#[test]
fn a_window_opened_by_the_outage_scans_at_once() {
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };

    let _ = session.step(&observed(0, false, None), &[]);
    let _ = session.step(&observed(10, false, None), &[Input::Join(JoinResult::Failed)]);
    assert_eq!(session.portal_status(), Status::Failed);
    // The backoff re-emits the join; leaving it in flight keeps the station down without a retry.
    let _ = session.step(&observed(1_010, false, None), &[]);

    let outcome = session.step(&observed(300_000, false, None), &[]);
    assert!(events(&outcome).contains(&Event::Setup(SetupEvent::Opened(RaiseReason::StationOutage))));
    assert_eq!(
        actions(&outcome),
        vec![Action::ApplyApStation(credentials), Action::Scan],
        "the newly opened window scans immediately"
    );
}

/// The portal status is published only when it changes.
#[test]
fn the_portal_status_is_published_only_when_it_changes() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };

    let view: BridgeStatusView = session.bridge_status(false, false);
    assert_eq!(view.station, BridgeStation::Connecting);

    let outcome = session.step(&observed(0, true, Some(STATION_IP)), &[]);
    assert_eq!(outcome.publication.portal_status, Some(Status::Connected(STATION_IP)));
    assert_eq!(outcome.publication.station_up, Some(true));

    let outcome = session.step(&observed(500, true, Some(STATION_IP)), &[]);
    assert_eq!(outcome.publication, NOTHING, "a repeat snapshot publishes nothing");

    let moved = [192, 168, 1, 43];
    let outcome = session.step(&observed(1_000, true, Some(moved)), &[]);
    assert_eq!(outcome.publication.portal_status, Some(Status::Connected(moved)));
    assert_eq!(outcome.publication.station_up, None, "the station stayed up");
}

/// Every reconnect trigger increments the station-reconnect counter.
#[test]
fn the_reconnect_counter_counts_every_reconnect_trigger() {
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };
    assert_eq!(session.stats(), Stats::default());

    // Both join failures count.
    let _ = session.step(&observed(0, false, None), &[]);
    let _ = session.step(&observed(10, false, None), &[Input::Join(JoinResult::WrongPassword)]);
    assert_eq!(session.stats().station_reconnects, 1);
    let _ = session.step(&observed(1_010, false, None), &[Input::Join(JoinResult::Failed)]);
    assert_eq!(session.stats().station_reconnects, 2);

    // A link loss counts.
    let _ = session.step(&observed(3_010, false, None), &[]);
    let _ = session.step(
        &observed(3_020, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    assert!(session.station_up());
    let outcome = session.step(&observed(4_000, false, None), &[]);
    assert_eq!(events(&outcome), vec![Event::LinkLost]);
    assert_eq!(session.stats().station_reconnects, 3);

    // A DHCP timeout counts.
    let _ = session.step(&observed(5_000, false, None), &[]);
    let _ = session.step(&observed(5_010, true, None), &[Input::Join(JoinResult::Connected)]);
    let outcome = session.step(&observed(35_010, true, None), &[]);
    assert_eq!(events(&outcome), vec![Event::DhcpTimeout]);
    assert_eq!(session.stats().station_reconnects, 4);
}

/// The portal counters count adopted writes and portal Forget commands only.
#[test]
fn the_portal_counters_count_provisions_and_portal_forgets_only() {
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let (mut session, _boot) = RadioSession::boot(None, 0);

    // A failed write is not a provision.
    let _ = session.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    let _ = session.step(&observed(10, false, None), &[Input::Persist(false)]);
    assert_eq!(session.stats().portal_provisions, 0);

    let _ = session.step(
        &observed(20, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    let _ = session.step(&observed(30, false, None), &[Input::Persist(true)]);
    assert_eq!(session.stats().portal_provisions, 1);

    let _ = session.step(&observed(40, false, None), &[Input::Portal(PortalCommand::Forget)]);
    assert_eq!(session.stats().portal_forgets, 1);

    // The panel's Forget is not a portal Forget.
    let Some((mut panel, _boot)) = provisioned_boot(0) else {
        return;
    };
    let _ = panel.step(&observed(0, false, None), &[Input::Panel(PanelCommand::Forget)]);
    assert_eq!(panel.stats().portal_forgets, 0);
    assert_eq!(panel.stats().portal_provisions, 0);
    assert_eq!(panel.stats().station_reconnects, 0);
}

/// The Bridge Status accessor reproduces the firmware's station and flag mapping.
#[test]
fn the_bridge_status_accessor_matches_the_firmware_mapping() {
    // Provisioned and associated: Associated with PROVISIONED, no Setup Mode flag.
    let Some((mut session, _boot)) = provisioned_boot(0) else {
        return;
    };
    let _ = session.step(&observed(0, true, Some(STATION_IP)), &[]);
    let view: BridgeStatusView = session.bridge_status(true, false);
    assert_eq!(view.station, BridgeStation::Associated);
    assert_eq!(view.flags, BridgeStatusFlags::PROVISIONED);

    // The caller's link-readiness argument adds LINK_READY.
    let ready = session.bridge_status(true, true);
    assert_eq!(ready.station, BridgeStation::Associated);
    assert!(ready.flags.contains(BridgeStatusFlags::LINK_READY));

    // A controller association without a lease is still Associated.
    let Some((mut associated, _boot)) = provisioned_boot(0) else {
        return;
    };
    let _ = associated.step(&observed(0, true, None), &[]);
    assert_eq!(associated.bridge_status(true, false).station, BridgeStation::Associated);

    // Provisioned but down and out of Setup Mode: Connecting with PROVISIONED.
    let Some((cold, _boot)) = provisioned_boot(0) else {
        return;
    };
    let view = cold.bridge_status(false, false);
    assert_eq!(view.station, BridgeStation::Connecting);
    assert_eq!(view.flags, BridgeStatusFlags::PROVISIONED);

    // Unprovisioned with the window open: Down with SETUP_OPEN only.
    let (mut open, _boot) = RadioSession::boot(None, 0);
    let view = open.bridge_status(false, false);
    assert_eq!(view.station, BridgeStation::Down);
    assert_eq!(view.flags, BridgeStatusFlags::SETUP_OPEN);

    // Unprovisioned after the window expires: Down with no flags.
    let _ = open.step(&observed(600_000, false, None), &[]);
    assert!(!open.setup_open());
    let view = open.bridge_status(false, false);
    assert_eq!(view.station, BridgeStation::Down);
    assert!(view.flags.is_empty(), "no flag is set while down and unprovisioned");
}

/// The ten-minute expiry and the thirty-second stable close fire exactly on their boundaries.
#[test]
fn the_expiry_and_stable_close_fire_at_their_exact_boundaries() {
    // Expiry: an unprovisioned boot parks the station when the window closes.
    let (mut session, _boot) = RadioSession::boot(None, 0);
    let outcome = session.step(&observed(599_999, false, None), &[]);
    assert!(
        !events(&outcome)
            .iter()
            .any(|event| matches!(event, Event::Setup(SetupEvent::Closed(_)))),
        "one millisecond before the expiry"
    );
    let outcome = session.step(&observed(600_000, false, None), &[]);
    assert_eq!(
        events(&outcome),
        vec![Event::Setup(SetupEvent::Closed(CloseReason::Expired))]
    );
    assert!(matches!(actions(&outcome).as_slice(), [Action::ApplyParkedStation]));
    assert!(!session.setup_open());

    // Stable close: a provisioned station that comes up inside the open window.
    let Some(credentials) = Credentials::from_slices(TEST_SSID, PASSPHRASE) else {
        return;
    };
    let (mut session, _boot) = RadioSession::boot(None, 0);
    let _ = session.step(
        &observed(0, false, None),
        &[Input::Portal(PortalCommand::Provision(credentials))],
    );
    let _ = session.step(&observed(10, false, None), &[Input::Persist(true)]);
    let _ = session.step(
        &observed(20, true, Some(STATION_IP)),
        &[Input::Join(JoinResult::Connected)],
    );
    let outcome = session.step(&observed(30_019, true, Some(STATION_IP)), &[]);
    assert!(
        !events(&outcome)
            .iter()
            .any(|event| matches!(event, Event::Setup(SetupEvent::Closed(_)))),
        "one millisecond before the stable close"
    );
    let outcome = session.step(&observed(30_020, true, Some(STATION_IP)), &[]);
    assert_eq!(
        events(&outcome),
        vec![Event::Setup(SetupEvent::Closed(CloseReason::StationStable))]
    );
    assert!(matches!(actions(&outcome).as_slice(), [Action::ApplyStation(_)]));
}
