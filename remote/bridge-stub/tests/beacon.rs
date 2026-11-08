//! The Discovery Beacon: the daemon's discovery states, scripted (ticket 09).

mod support;

use std::{
    net::UdpSocket,
    time::{Duration, Instant},
};

use bridge_net::beacon as net_beacon;
use bridge_stub::{Flags, Scenario, discovery};
use support::BenchError;

/// The token the tests pair with.
const TOKEN: &[u8] = b"pairing-token-01";

/// The fast interval the beacon tests use instead of ADR-0037's 3 s.
const INTERVAL_MS: u64 = 25;

/// Starts a stub whose beacon unicasts to a fresh loopback listener, returning both.
fn beacon_on(scenario: &Scenario) -> Result<(bridge_stub::Stub, UdpSocket), BenchError> {
    let listener = UdpSocket::bind("127.0.0.1:0").map_err(|error| BenchError::Io(error.to_string()))?;
    let mut scenario = scenario.clone();
    scenario.beacon.target = listener
        .local_addr()
        .map_err(|error| BenchError::Io(error.to_string()))?;
    scenario.beacon.interval_ms = INTERVAL_MS;
    let stub = support::start(&scenario)?;
    Ok((stub, listener))
}

/// Receives one datagram within `timeout`.
fn receive(socket: &UdpSocket, timeout: Duration) -> Result<Vec<u8>, BenchError> {
    socket
        .set_read_timeout(Some(timeout))
        .map_err(|error| BenchError::Io(error.to_string()))?;
    let mut buffer = [0u8; 128];
    let (len, _) = socket
        .recv_from(&mut buffer)
        .map_err(|error| BenchError::Io(error.to_string()))?;
    Ok(buffer[..len].to_vec())
}

/// The Beacon carries the identity, the flag bits and the port the stub actually bound.
#[test]
fn the_beacon_carries_the_identity_flags_and_bound_port() -> Result<(), BenchError> {
    let (stub, socket) = beacon_on(&Scenario::new(TOKEN))?;
    let datagram = receive(&socket, Duration::from_secs(2))?;
    let beacon = discovery::Beacon::decode(&datagram).map_err(|error| BenchError::Unexpected(error.to_string()))?;

    assert_eq!(beacon.device_id.bytes(), *b"stub01");
    assert_eq!(beacon.name.as_str(), "Explorer-STUB");
    assert_eq!(beacon.flags, Flags::PROVISIONED | Flags::LINK_READY);
    assert_eq!(
        beacon.tcp_port,
        stub.link_addr().port(),
        "the beacon mirrors the port the stub actually bound"
    );
    Ok(())
}

/// The datagram pins ADR-0037's wire layout: magic, major, device id, name, port, flags.
#[test]
fn the_beacon_pins_the_adr_wire_layout() -> Result<(), BenchError> {
    let (stub, socket) = beacon_on(&Scenario::new(TOKEN))?;
    let datagram = receive(&socket, Duration::from_secs(2))?;

    assert_eq!(datagram.len(), discovery::ENCODED_LEN);
    assert_eq!(&datagram[..4], b"EXPR", "the magic is EXPR");
    assert_eq!(datagram[4], net_beacon::LINK_MAJOR, "the major is the Link Protocol's");
    assert_eq!(&datagram[5..11], b"stub01", "the device id follows the major");
    assert_eq!(
        &datagram[11..27],
        b"Explorer-STUB\0\0\0",
        "the name is zero-padded to sixteen"
    );
    assert_eq!(
        u16::from_le_bytes([datagram[27], datagram[28]]),
        stub.link_addr().port(),
        "the port is little-endian before the flags"
    );
    assert_eq!(
        datagram[29],
        net_beacon::FLAG_PROVISIONED | net_beacon::FLAG_LINK_READY,
        "the flags close the datagram"
    );
    Ok(())
}

/// The Setup Mode flag is scriptable, so the daemon's `setup_mode` discovery state is reachable.
#[test]
fn the_setup_mode_flag_is_scriptable() -> Result<(), BenchError> {
    let mut scenario = Scenario::new(TOKEN);
    scenario.identity.flags = Flags::PROVISIONED | Flags::SETUP_MODE;
    let (_stub, socket) = beacon_on(&scenario)?;
    let datagram = receive(&socket, Duration::from_secs(2))?;
    let beacon = discovery::Beacon::decode(&datagram).map_err(|error| BenchError::Unexpected(error.to_string()))?;

    assert!(beacon.flags.contains(Flags::SETUP_MODE));
    assert!(!beacon.flags.contains(Flags::LINK_READY));
    Ok(())
}

/// The portal-only variant advertises `tcp_port = 0`, which the daemon treats as Setup Mode.
#[test]
fn the_portal_only_variant_advertises_a_zero_port() -> Result<(), BenchError> {
    let mut scenario = Scenario::new(TOKEN);
    scenario.beacon.tcp_port = Some(net_beacon::PORTAL_ONLY);
    let (_stub, socket) = beacon_on(&scenario)?;
    let datagram = receive(&socket, Duration::from_secs(2))?;
    let beacon = discovery::Beacon::decode(&datagram).map_err(|error| BenchError::Unexpected(error.to_string()))?;

    assert_eq!(beacon.tcp_port, net_beacon::PORTAL_ONLY);
    Ok(())
}

/// The Beacon repeats at the configured interval, so discovery is a 3 s cadence in production.
#[test]
fn the_beacon_repeats_at_the_configured_interval() -> Result<(), BenchError> {
    let (_stub, socket) = beacon_on(&Scenario::new(TOKEN))?;
    let start = Instant::now();
    receive(&socket, Duration::from_secs(2))?;
    receive(&socket, Duration::from_secs(2))?;
    receive(&socket, Duration::from_secs(2))?;
    let elapsed = start.elapsed();

    assert!(
        elapsed >= Duration::from_millis(2 * INTERVAL_MS - 10),
        "three datagrams span at least two intervals"
    );
    assert!(elapsed <= Duration::from_secs(5), "the cadence does not stall");
    Ok(())
}
