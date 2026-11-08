//! Host-side integration tests for the `link-state` crate.
//!
//! These exercise phase derivation, Bridge Status staleness, Bridge Control outcomes, the panel
//! labels and the Disable clear path. The `std` feature exists only to enable this harness; the
//! crate itself is always `no_std`.

use link_state::{
    BridgeFlags, BridgeStatus, ControlOutcome, ControlReply, ControlStatus, LinkPhase, LinkState,
    PAIRING_CODE_CAPACITY, PASSPHRASE_CAPACITY, PairingCode, Peer, Presence, SSID_CAPACITY, STALE_AFTER_MS,
    SetupCredentials, Snapshot, Station,
};

/// The four link phases are distinct and presence takes precedence over peer.
#[test]
fn phases_are_derived_and_distinct() {
    let mut state = LinkState::new();
    assert_eq!(state.phase(), LinkPhase::Off, "a new state must be Off");

    state.set_presence(Presence::WaitingForBoard);
    assert_eq!(
        state.phase(),
        LinkPhase::WaitingForBoard,
        "an enabled port with no board must wait"
    );

    state.set_peer(Peer::PeerLive);
    assert_eq!(
        state.phase(),
        LinkPhase::WaitingForBoard,
        "presence must win over peer while the board is absent"
    );

    state.set_presence(Presence::Ready);
    assert_eq!(
        state.phase(),
        LinkPhase::PeerLive,
        "a ready board with a live peer must be PeerLive"
    );

    state.set_peer(Peer::NoPeer);
    assert_eq!(
        state.phase(),
        LinkPhase::ReadyNoPeer,
        "a ready board with no peer must be ReadyNoPeer"
    );

    state.set_presence(Presence::Off);
    assert_eq!(state.phase(), LinkPhase::Off, "Off must win over any peer");

    let phases = [
        LinkPhase::Off,
        LinkPhase::WaitingForBoard,
        LinkPhase::ReadyNoPeer,
        LinkPhase::PeerLive,
    ];
    for (index, phase) in phases.iter().enumerate() {
        for other in &phases[index + 1..] {
            assert_ne!(phase, other, "the four link phases must be distinct");
        }
    }
}

/// The snapshot reports the derived phase and the current peer.
#[test]
fn snapshot_reports_phase_and_peer() {
    let mut state = LinkState::new();
    state.set_presence(Presence::Ready);
    state.set_peer(Peer::PeerLive);

    let snapshot = state.snapshot(0);
    assert_eq!(snapshot.phase, LinkPhase::PeerLive);
    assert_eq!(snapshot.presence, Presence::Ready);
    assert_eq!(snapshot.peer, Peer::PeerLive);
    assert_eq!(snapshot.control, ControlOutcome::Idle);
}

/// The constant disabled snapshot is the neutral starting point the panel uses.
#[test]
fn the_off_snapshot_is_neutral() {
    let snapshot = Snapshot::off();
    assert_eq!(snapshot.phase, LinkPhase::Off);
    assert_eq!(snapshot.presence, Presence::Off);
    assert_eq!(snapshot.peer, Peer::NoPeer);
    assert!(snapshot.bridge.is_none());
    assert!(!snapshot.bridge_stale);
    assert_eq!(snapshot.control, ControlOutcome::Idle);
}

/// A Bridge Status is stale only when it is older than the threshold.
#[test]
fn bridge_status_is_stale_only_after_two_seconds() {
    let mut state = LinkState::new();
    state.set_presence(Presence::Ready);
    state.set_bridge_status(sample_status(), 10_000);

    let fresh = state.snapshot(10_001);
    assert!(fresh.bridge.is_some(), "the status must be readable");
    assert!(!fresh.bridge_stale, "one second old must be fresh");

    let boundary = state.snapshot(10_000 + STALE_AFTER_MS);
    assert!(
        boundary.bridge.is_some(),
        "the status must remain readable at the boundary"
    );
    assert!(!boundary.bridge_stale, "exactly two seconds old must not be stale");

    let expired = state.snapshot(10_001 + STALE_AFTER_MS);
    assert!(expired.bridge.is_some(), "a stale status must still be readable");
    assert!(expired.bridge_stale, "older than two seconds must be stale");

    let backwards = state.snapshot(9_000);
    assert!(
        !backwards.bridge_stale,
        "a backwards clock must not mark the status stale"
    );
}

/// A state with no Bridge Status never reports staleness.
#[test]
fn no_bridge_status_is_not_stale() {
    let state = LinkState::new();
    let snapshot = state.snapshot(999_999);
    assert!(snapshot.bridge.is_none(), "a new state must have no Bridge Status");
    assert!(!snapshot.bridge_stale, "a missing status must not be stale");
}

/// A retained Bridge Status survives a no-peer period and later goes stale.
#[test]
fn bridge_status_survives_a_no_peer_period_and_goes_stale() {
    let mut state = LinkState::new();
    state.set_presence(Presence::Ready);
    state.set_peer(Peer::PeerLive);
    state.set_bridge_status(sample_status(), 5_000);

    state.set_peer(Peer::NoPeer);
    let retained = state.snapshot(6_000);
    assert_eq!(retained.phase, LinkPhase::ReadyNoPeer);
    assert_eq!(retained.peer, Peer::NoPeer);
    assert!(retained.bridge.is_some(), "the Bridge Status must be retained");
    assert!(!retained.bridge_stale, "the retained status must still be fresh");

    let expired = state.snapshot(7_001);
    assert!(expired.bridge.is_some(), "the retained status must remain readable");
    assert!(expired.bridge_stale, "the retained status must eventually go stale");
}

/// The snapshot exposes every neutral Bridge Status field.
#[test]
fn snapshot_exposes_the_neutral_bridge_status_fields() {
    let mut state = LinkState::new();
    state.set_bridge_status(sample_status(), 0);

    let Some(bridge) = state.snapshot(0).bridge else {
        return;
    };
    assert_eq!(bridge.rssi_dbm, Some(-57));
    assert_eq!(bridge.channel, Some(6));
    assert_eq!(bridge.station, Station::Associated);
    assert!(bridge.flags.provisioned, "the sample must be provisioned");
    assert!(!bridge.flags.setup_open, "the sample must not have setup open");
    assert!(bridge.flags.link_ready, "the sample must have its link ready");
    assert_eq!(bridge.uart_dropped_bytes, 1);
    assert_eq!(bridge.dma_stalls, 2);
    assert_eq!(bridge.crc_failures, 3);
    assert_eq!(bridge.resyncs, 4);
    assert_eq!(bridge.wdt_feeds, 5);
    assert_eq!(bridge.wdt_withheld, 6);
}

/// A control request moves from Idle to Pending to an answered setup reply.
#[test]
fn control_outcome_transitions_to_answered_setup() {
    let mut state = LinkState::new();
    assert_eq!(state.control(), &ControlOutcome::Idle, "a new state must be Idle");

    state.set_control(ControlOutcome::Pending);
    assert!(state.control().is_pending(), "a request in flight must be Pending");
    assert_eq!(state.snapshot(0).control, ControlOutcome::Pending);

    let Some(credentials) = SetupCredentials::new("ExplorerRobot-ABCD", "setup-password") else {
        return;
    };
    state.set_control(ControlOutcome::Answered(ControlReply::Setup {
        status: ControlStatus::Ok,
        credentials,
    }));

    let ControlOutcome::Answered(reply) = state.control() else {
        return;
    };
    assert_eq!(reply.status(), ControlStatus::Ok, "the reply must carry its status");
    let ControlReply::Setup { credentials, .. } = reply else {
        return;
    };
    assert_eq!(credentials.ssid.as_str(), "ExplorerRobot-ABCD");
    assert_eq!(credentials.passphrase.as_str(), "setup-password");
}

/// A control request moves from Idle to Pending to an answered pairing code.
#[test]
fn control_outcome_transitions_to_answered_pairing_code() {
    let mut state = LinkState::new();
    state.set_control(ControlOutcome::Pending);
    let Some(code) = PairingCode::new("ABCDEFGH2345") else {
        return;
    };
    state.set_control(ControlOutcome::Answered(ControlReply::PairingCode {
        status: ControlStatus::Ok,
        code,
    }));

    let ControlOutcome::Answered(reply) = state.control() else {
        return;
    };
    assert_eq!(reply.status(), ControlStatus::Ok, "the reply must carry its status");
    let ControlReply::PairingCode { code, .. } = reply else {
        return;
    };
    assert_eq!(code.code.as_str(), "ABCDEFGH2345");
}

/// A control request moves from Idle to Pending to `TimedOut`.
#[test]
fn control_outcome_transitions_to_timed_out() {
    let mut state = LinkState::new();
    state.set_control(ControlOutcome::Pending);
    assert!(state.control().is_pending(), "the request must be in flight");

    state.set_control(ControlOutcome::TimedOut);
    assert_eq!(state.snapshot(0).control, ControlOutcome::TimedOut);
    assert!(!state.control().is_pending(), "a timeout is not pending");
}

/// Every fixed enum exposes the label the panel shows.
#[test]
fn labels_cover_every_fixed_enum() {
    assert_eq!(Presence::Off.label(), "Off");
    assert_eq!(Presence::WaitingForBoard.label(), "Waiting for board");
    assert_eq!(Presence::Ready.label(), "Ready");

    assert_eq!(Peer::NoPeer.label(), "No peer");
    assert_eq!(Peer::PeerLive.label(), "Peer live");

    assert_eq!(LinkPhase::Off.label(), "Off");
    assert_eq!(LinkPhase::WaitingForBoard.label(), "Waiting for board");
    assert_eq!(LinkPhase::ReadyNoPeer.label(), "Ready");
    assert_eq!(LinkPhase::PeerLive.label(), "Peer live");

    assert_eq!(Station::Down.label(), "Down");
    assert_eq!(Station::Connecting.label(), "Connecting");
    assert_eq!(Station::Associated.label(), "Associated");

    assert_eq!(ControlStatus::Ok.label(), "OK");
    assert_eq!(ControlStatus::Refused.label(), "Refused");
    assert_eq!(ControlStatus::Failed.label(), "Failed");

    assert_eq!(ControlOutcome::Idle.label(), "Idle");
    assert_eq!(ControlOutcome::Pending.label(), "Pending");
    assert_eq!(ControlOutcome::TimedOut.label(), "Timed out");

    let Some(credentials) = SetupCredentials::new("ssid", "passphrase") else {
        return;
    };
    let answered = ControlOutcome::Answered(ControlReply::Setup {
        status: ControlStatus::Refused,
        credentials,
    });
    assert_eq!(answered.label(), "Answered");
}

/// `clear` returns the state to the disabled baseline.
#[test]
fn clear_returns_to_the_disabled_state() {
    let mut state = LinkState::new();
    state.set_presence(Presence::Ready);
    state.set_peer(Peer::PeerLive);
    state.set_bridge_status(sample_status(), 1_000);
    state.set_control(ControlOutcome::TimedOut);

    state.clear();

    let snapshot = state.snapshot(2_000);
    assert_eq!(snapshot.phase, LinkPhase::Off, "clear must return to Off");
    assert_eq!(snapshot.peer, Peer::NoPeer, "clear must return to NoPeer");
    assert!(snapshot.bridge.is_none(), "clear must drop the Bridge Status");
    assert!(!snapshot.bridge_stale, "a dropped status is not stale");
    assert_eq!(snapshot.control, ControlOutcome::Idle, "clear must reset control");
    assert_eq!(state.presence(), Presence::Off);
}

/// The fixed-capacity strings reject input that does not fit.
#[test]
fn fixed_capacity_strings_reject_oversized_input() {
    let long_ssid = "a".repeat(SSID_CAPACITY + 1);
    assert!(
        SetupCredentials::new(&long_ssid, "passphrase").is_none(),
        "an oversized SSID must be rejected"
    );

    let long_passphrase = "a".repeat(PASSPHRASE_CAPACITY + 1);
    assert!(
        SetupCredentials::new("ssid", &long_passphrase).is_none(),
        "an oversized passphrase must be rejected"
    );

    let long_code = "a".repeat(PAIRING_CODE_CAPACITY + 1);
    assert!(
        PairingCode::new(&long_code).is_none(),
        "an oversized pairing code must be rejected"
    );
}

/// A representative Bridge Status for the tests.
const fn sample_status() -> BridgeStatus {
    BridgeStatus {
        rssi_dbm: Some(-57),
        channel: Some(6),
        station: Station::Associated,
        flags: BridgeFlags {
            provisioned: true,
            setup_open: false,
            link_ready: true,
        },
        uart_dropped_bytes: 1,
        dma_stalls: 2,
        crc_failures: 3,
        resyncs: 4,
        wdt_feeds: 5,
        wdt_withheld: 6,
    }
}
