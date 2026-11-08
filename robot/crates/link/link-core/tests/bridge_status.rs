//! Host tests for the Bridge's `BRIDGE_STATUS` telemetry and its lease rule (ADR-0035).
//!
//! The Bridge publishes an unacknowledged snapshot at 1 Hz on the PC-to-robot direction. It is
//! stored for the panel but must not renew the Remote's lease or re-arm the Deadman.

use link_core::{Config, LinkCore, PeerState};
use link_protocol::{
    BridgeStation, BridgeStatus, BridgeStatusFlags, Caps, Class, Control, Direction, Flags, Frame,
    MAX_PC_TO_ROBOT_FRAME, MAX_ROBOT_TO_PC_FRAME, PROTOCOL_MAJOR, Seq, ThrottleAction, ThrottleHint,
    codec::PayloadCodec,
    sim::{FakeRemote, Faults, Wire},
};

/// The Boot Id the tests hand to the link core.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// Creates a core, a wire and a remote for one test.
fn harness() -> (LinkCore, Wire, FakeRemote) {
    (
        LinkCore::new(Config::default(), BOOT_ID, 0),
        Wire::new(1, Faults::default()),
        FakeRemote::new(),
    )
}

/// Drains the wire's PC-to-robot queue into the core at `now_ms`.
fn feed(core: &mut LinkCore, wire: &mut Wire, now_ms: u64) {
    let bytes = wire.take(Direction::PcToRobot);
    core.on_rx(&bytes, now_ms);
}

/// Builds one PC-to-robot control frame.
fn control_frame(id: u16, seq: u16, flags: Flags, payload: &[u8]) -> Option<Vec<u8>> {
    let frame = Frame::new(PROTOCOL_MAJOR, Class::Control, flags, id, Seq::new(seq), payload).ok()?;
    let mut buffer = [0u8; MAX_PC_TO_ROBOT_FRAME];
    let written = frame.encode_into(&mut buffer).ok()?;
    Some(buffer[..written].to_vec())
}

/// A well-formed snapshot whose counters make it distinguishable from `other`.
fn status(uart_dropped_bytes: u32) -> BridgeStatus {
    BridgeStatus {
        rssi_dbm: Some(-67),
        channel: Some(6),
        station: BridgeStation::Associated,
        flags: BridgeStatusFlags::PROVISIONED | BridgeStatusFlags::LINK_READY,
        uart_dropped_bytes,
        dma_stalls: 2,
        crc_failures: 3,
        resyncs: 4,
        wdt_feeds: 100,
        wdt_withheld: 1,
    }
}

/// Encodes and feeds one `BRIDGE_STATUS` at `now_ms`.
fn send_status(core: &mut LinkCore, status: &BridgeStatus, seq: u16, now_ms: u64) {
    let mut payload = [0u8; BridgeStatus::ENCODED_LEN];
    let Ok(len) = status.encode_into(&mut payload) else {
        return;
    };
    let Some(bytes) = control_frame(Control::BridgeStatus.wire_id(), seq, Flags::default(), &payload[..len]) else {
        return;
    };
    core.on_rx(&bytes, now_ms);
}

/// Attaches the peer with a `PING` at t=0 and drains its reply.
fn attach(core: &mut LinkCore, wire: &mut Wire, remote: &mut FakeRemote) {
    let Some(_) = remote.send_ping(wire) else {
        return;
    };
    feed(core, wire, 0);
    assert_eq!(core.peer(), PeerState::PeerLive);
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let _ = core.next_frame(&mut buffer);
}

/// The first status is stored and a newer one replaces it; the accessor serves the latest.
#[test]
fn a_status_is_stored_and_the_newest_replaces_it() {
    let (mut core, _wire, _remote) = harness();
    assert_eq!(core.bridge_status(), None, "nothing has arrived yet");

    let first = status(1);
    send_status(&mut core, &first, 1, 0);
    assert_eq!(core.bridge_status(), Some(first));

    let second = status(2);
    send_status(&mut core, &second, 2, 1);
    assert_eq!(core.bridge_status(), Some(second), "the newest snapshot wins");
}

/// A malformed status never panics and leaves the previous value untouched.
#[test]
fn a_malformed_status_is_ignored_and_keeps_the_last_good_one() {
    let (mut core, _wire, _remote) = harness();

    // A short payload is rejected by the decoder.
    let Some(short) = control_frame(Control::BridgeStatus.wire_id(), 1, Flags::default(), &[0u8; 4]) else {
        return;
    };
    core.on_rx(&short, 0);
    assert_eq!(core.bridge_status(), None, "a malformed first status stores nothing");

    let good = status(7);
    send_status(&mut core, &good, 2, 1);
    assert_eq!(core.bridge_status(), Some(good));

    // A full-length payload with an unassigned station byte is rejected.
    let mut invalid = [0u8; BridgeStatus::ENCODED_LEN];
    invalid[2] = 9;
    let Some(bytes) = control_frame(Control::BridgeStatus.wire_id(), 3, Flags::default(), &invalid) else {
        return;
    };
    core.on_rx(&bytes, 2);
    assert_eq!(core.bridge_status(), Some(good), "the last good value is kept");
}

/// A status does not renew the lease, so the Deadman still fires 2 s after the last Remote frame.
#[test]
fn a_status_does_not_renew_the_lease_or_rearm_the_deadman() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    // The Remote last spoke at t=0; the Bridge's status at t=1000 must not extend that.
    send_status(&mut core, &status(1), 1, 1_000);
    assert_eq!(core.peer(), PeerState::PeerLive);

    core.tick(2_000);
    assert!(
        core.take_peer_lost(),
        "2 s of Remote silence fires even though a status arrived at 1 s"
    );
}

/// A status from a detached core must not attach a peer.
#[test]
fn a_status_from_no_peer_leaves_the_peer_detached() {
    let (mut core, _wire, _remote) = harness();

    send_status(&mut core, &status(1), 1, 500);
    assert_eq!(core.peer(), PeerState::NoPeer);
    assert!(!core.take_peer_lost(), "there was no claim to lose");
}

/// A `THROTTLE` shares the direction but does not renew the lease.
#[test]
fn a_throttle_does_not_renew_the_lease() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    let Some(_) = remote.send_throttle(&mut wire, ThrottleHint::ALL_STREAMS, ThrottleAction::Pause, 0, 60_000) else {
        return;
    };
    feed(&mut core, &mut wire, 1_000);

    core.tick(2_000);
    assert!(core.take_peer_lost(), "a THROTTLE is not a Remote claim");
}

/// The Bridge's `BRIDGE_CONTROL` ACK shares the direction but does not renew the lease.
#[test]
fn a_bridge_control_ack_does_not_renew_the_lease() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    let Some(bytes) = control_frame(Control::BridgeControl.wire_id(), 1, Flags::IS_ACK, &[]) else {
        return;
    };
    core.on_rx(&bytes, 1_000);

    core.tick(2_000);
    assert!(core.take_peer_lost(), "a BRIDGE_CONTROL ACK is not a Remote claim");
}

/// A non-ACK event is not a Remote liveness proof.
#[test]
fn a_non_ack_event_does_not_renew_the_lease() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    let frame = Frame::new(PROTOCOL_MAJOR, Class::Event, Flags::default(), 1, Seq::new(1), &[]).ok();
    let Some(frame) = frame else {
        return;
    };
    let mut buffer = [0u8; MAX_PC_TO_ROBOT_FRAME];
    let Ok(written) = frame.encode_into(&mut buffer) else {
        return;
    };
    core.on_rx(&buffer[..written], 1_000);

    core.tick(2_000);
    assert!(core.take_peer_lost(), "only an event ACK is Remote traffic");
}

/// A `PING` still renews the lease and re-arms the Deadman.
#[test]
fn a_ping_still_renews_the_lease() {
    let (mut core, mut wire, mut remote) = harness();
    attach(&mut core, &mut wire, &mut remote);

    let Some(_) = remote.send_ping(&mut wire) else {
        return;
    };
    feed(&mut core, &mut wire, 1_000);

    // 1.999 s of silence after the renewed PING is below the Deadman threshold.
    core.tick(2_999);
    assert!(!core.take_peer_lost());
}

/// A status under a version mismatch is ignored.
#[test]
fn a_status_under_a_version_mismatch_is_ignored() {
    let (mut core, mut wire, mut remote) = harness();

    let Some(_) = remote.send_hello(&mut wire, PROTOCOL_MAJOR.wrapping_add(1), 0, Caps::from_bits(0), 1, &[]) else {
        return;
    };
    feed(&mut core, &mut wire, 0);
    assert!(core.version_mismatch());

    send_status(&mut core, &status(1), 1, 1);
    assert_eq!(core.bridge_status(), None, "a mismatched peer's status is not stored");
}
