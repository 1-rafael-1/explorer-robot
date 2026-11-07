//! Discovery through the real UDP seam: Beacons on the socket, the endpoint cache and selection
//! (ADR-0037, ADR-0039).
//!
//! Each test binds a `Discovery` on an ephemeral loopback port and sends encoded `bridge_net`
//! datagrams to it, so the behaviour under test is the datagram decode, deduplication and choice,
//! not an internal map.

mod app_support;

use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket},
    time::{Duration, Instant},
};

use app_support::{must, must_some};
use bridge_net::{
    beacon::{self, Beacon},
    identity::{DeviceId, Flags, Name},
};
use remote_daemon::{
    clock,
    discovery::{BEACON_TTL_MS, Choice, Discovery, DiscoveryConfig, DiscoveryHandle, LINK_PORT},
};

/// A device id.
const FIRST: [u8; 6] = *b"aaaaaa";

/// Another device id, sorting after [`FIRST`].
const SECOND: [u8; 6] = *b"bbbbbb";

/// A third device id, sorting after [`SECOND`].
const THIRD: [u8; 6] = *b"cccccc";

/// A name that fits the Beacon's sixteen-byte field.
fn name(text: &str) -> Name {
    must_some(Name::new(text), "name")
}

/// Builds a Beacon at the current major with the default identity flags.
fn beacon(device: [u8; 6], text: &str, tcp_port: u16, flags: Flags) -> Beacon {
    Beacon {
        major: beacon::LINK_MAJOR,
        device_id: DeviceId::new(device),
        name: name(text),
        tcp_port,
        flags,
    }
}

/// Encodes and sends one Beacon to `target`.
fn send(socket: &UdpSocket, target: SocketAddr, beacon: Beacon) {
    let mut bytes = [0u8; beacon::ENCODED_LEN];
    let Ok(len) = beacon.encode_into(&mut bytes) else {
        assert_eq!(0, usize::MAX, "the Beacon did not encode");
        std::process::abort()
    };
    must(socket.send_to(&bytes[..len], target), "send beacon");
}

/// Binds a discovery listener on an ephemeral loopback port.
fn bind(last_device: Option<DeviceId>) -> (Discovery, UdpSocket) {
    let discovery = must(
        Discovery::bind(DiscoveryConfig {
            bind: IpAddr::V4(Ipv4Addr::LOCALHOST),
            port: 0,
            last_device,
        }),
        "bind discovery",
    );
    let sender = must(UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)), "bind sender");
    (discovery, sender)
}

/// Spawns the listener and returns its handle and address.
fn spawn(discovery: Discovery) -> (DiscoveryHandle, SocketAddr) {
    let handle = must(discovery.spawn(), "spawn discovery");
    (handle.clone(), handle.local_addr())
}

/// Polls `choose` until `predicate` holds, or fails the test after a short budget.
async fn wait_choice(handle: &DiscoveryHandle, predicate: impl Fn(&Choice) -> bool) -> Choice {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let choice = handle.choose(clock::now_ms());
        if predicate(&choice) {
            return choice;
        }
        if Instant::now() >= deadline {
            assert_eq!(format!("{choice:?}").len(), usize::MAX, "discovery did not settle");
            std::process::abort();
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

/// Extracts the Beacon variant, failing the test otherwise.
fn selected(choice: Choice) -> (remote_daemon::discovery::BeaconEntry, Option<String>) {
    match choice {
        Choice::Beacon { entry, detail } => (entry, detail),
        other => {
            assert_eq!(format!("{other:?}").len(), usize::MAX, "expected a Beacon choice");
            std::process::abort()
        }
    }
}

/// One Beacon yields the datagram's source address and the advertised Link port.
#[tokio::test]
async fn a_beacon_yields_its_source_endpoint_and_port() {
    let (discovery, sender) = bind(None);
    let (handle, addr) = spawn(discovery);
    send(
        &sender,
        addr,
        beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED | Flags::LINK_READY),
    );

    let (entry, detail) = selected(wait_choice(&handle, |choice| matches!(choice, Choice::Beacon { .. })).await);
    assert_eq!(entry.device_id, DeviceId::new(FIRST));
    assert_eq!(entry.label(), "Explorer-AAAA");
    assert_eq!(entry.tcp_port, 9000);
    assert_eq!(
        entry.endpoint,
        SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 9000))
    );
    assert!(!entry.setup_mode());
    assert_eq!(detail, None);

    handle.stop();
    handle.wait().await;
}

/// Two Beacons from one device update one cache entry and its endpoint.
#[tokio::test]
async fn beacons_are_deduplicated_by_device_id() {
    let (discovery, sender) = bind(None);
    let (handle, addr) = spawn(discovery);
    send(&sender, addr, beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED));
    let _ = wait_choice(
        &handle,
        |choice| matches!(choice, Choice::Beacon { entry, .. } if entry.tcp_port == 9000),
    )
    .await;
    send(&sender, addr, beacon(FIRST, "Explorer-AAAA", 9100, Flags::PROVISIONED));

    let (entry, _) = selected(
        wait_choice(
            &handle,
            |choice| matches!(choice, Choice::Beacon { entry, .. } if entry.tcp_port == 9100),
        )
        .await,
    );
    assert_eq!(entry.device_id, DeviceId::new(FIRST));

    handle.stop();
    handle.wait().await;
}

/// The last-used device id wins when it is still seen, even beside another robot.
#[tokio::test]
async fn the_last_used_device_wins_when_it_is_still_seen() {
    let (discovery, sender) = bind(Some(DeviceId::new(SECOND)));
    let (handle, addr) = spawn(discovery);
    send(&sender, addr, beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED));
    send(&sender, addr, beacon(SECOND, "Explorer-BBBB", 9001, Flags::PROVISIONED));

    let (entry, detail) = selected(
        wait_choice(
            &handle,
            |choice| matches!(choice, Choice::Beacon { entry, .. } if entry.device_id == DeviceId::new(SECOND)),
        )
        .await,
    );
    assert_eq!(entry.device_id, DeviceId::new(SECOND));
    assert_eq!(detail, None, "history needs no explanation");

    handle.stop();
    handle.wait().await;
}

/// With no history, several Beacons pick one deterministically and name it in `detail`.
#[tokio::test]
async fn several_new_beacons_pick_one_and_name_it() {
    let (discovery, sender) = bind(None);
    let (handle, addr) = spawn(discovery);
    send(&sender, addr, beacon(THIRD, "Explorer-CCCC", 9002, Flags::PROVISIONED));
    send(&sender, addr, beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED));

    let (entry, detail) = selected(wait_choice(&handle, |choice| matches!(choice, Choice::Beacon { .. })).await);
    assert_eq!(entry.device_id, DeviceId::new(FIRST), "the lowest device id is chosen");
    let detail = must_some(detail, "the pick is named in detail");
    assert!(
        detail.contains("Explorer-AAAA"),
        "the detail names the chosen robot: {detail}"
    );

    handle.stop();
    handle.wait().await;
}

/// A portal-only advertisement (`tcp_port = 0`) is Setup Mode, not a connectable endpoint.
#[tokio::test]
async fn a_portal_only_beacon_is_setup_mode() {
    let (discovery, sender) = bind(None);
    let (handle, addr) = spawn(discovery);
    send(&sender, addr, beacon(FIRST, "Explorer-AAAA", 0, Flags::PROVISIONED));

    let (entry, _) = selected(wait_choice(&handle, |choice| matches!(choice, Choice::Beacon { .. })).await);
    assert!(entry.setup_mode(), "a zero Link port is the portal-only advertisement");
    assert_eq!(entry.tcp_port, beacon::PORTAL_ONLY);

    handle.stop();
    handle.wait().await;
}

/// The Setup Mode flag is Setup Mode even when a Link port is advertised.
#[tokio::test]
async fn the_setup_mode_flag_is_setup_mode() {
    let (discovery, sender) = bind(None);
    let (handle, addr) = spawn(discovery);
    send(
        &sender,
        addr,
        beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED | Flags::SETUP_MODE),
    );

    let (entry, _) = selected(wait_choice(&handle, |choice| matches!(choice, Choice::Beacon { .. })).await);
    assert!(entry.setup_mode());

    handle.stop();
    handle.wait().await;
}

/// A manual endpoint covers networks that block broadcast and wins over a Beacon.
#[tokio::test]
async fn a_manual_endpoint_wins_over_beacons() {
    let (discovery, sender) = bind(None);
    let (handle, addr) = spawn(discovery);
    send(&sender, addr, beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED));
    let _ = wait_choice(&handle, |choice| matches!(choice, Choice::Beacon { .. })).await;

    let manual = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(192, 0, 2, 50), 9000));
    handle.set_manual(Some(manual));
    assert_eq!(handle.choose(clock::now_ms()), Choice::Manual { endpoint: manual });

    handle.stop();
    handle.wait().await;
}

/// A Beacon older than the cache window is forgotten.
#[tokio::test]
async fn a_stale_beacon_is_forgotten() {
    let (discovery, sender) = bind(None);
    let (handle, addr) = spawn(discovery);
    send(&sender, addr, beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED));
    let _ = wait_choice(&handle, |choice| matches!(choice, Choice::Beacon { .. })).await;

    assert_eq!(
        handle.choose(clock::now_ms() + BEACON_TTL_MS + 1),
        Choice::None,
        "three missed Beacons at the 3 s interval expire the cache entry"
    );

    handle.stop();
    handle.wait().await;
}

/// A Beacon from a different Link Protocol major is ignored, not cached or connected to.
#[tokio::test]
async fn a_beacon_with_a_mismatched_major_is_ignored() {
    let (discovery, sender) = bind(None);
    let (handle, addr) = spawn(discovery);
    let mut wrong = beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED);
    wrong.major = beacon::LINK_MAJOR.wrapping_add(1);
    send(&sender, addr, wrong);

    // Give the listener time to receive the datagram, then prove it left no cache entry.
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert_eq!(
        handle.choose(clock::now_ms()),
        Choice::None,
        "a mismatched-major Beacon is not selectable"
    );
    assert!(handle.entries(clock::now_ms()).is_empty(), "it is not cached");

    // A matching-major Beacon from the same robot is accepted, so filtering keeps listening.
    send(&sender, addr, beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED));
    let (entry, _) = selected(wait_choice(&handle, |choice| matches!(choice, Choice::Beacon { .. })).await);
    assert_eq!(entry.device_id, DeviceId::new(FIRST));

    handle.stop();
    handle.wait().await;
}

/// A Beacon with the current major is accepted and selectable.
#[tokio::test]
async fn a_beacon_with_the_matching_major_is_accepted() {
    let (discovery, sender) = bind(None);
    let (handle, addr) = spawn(discovery);
    send(&sender, addr, beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED));

    let (entry, _) = selected(wait_choice(&handle, |choice| matches!(choice, Choice::Beacon { .. })).await);
    assert_eq!(entry.device_id, DeviceId::new(FIRST));

    handle.stop();
    handle.wait().await;
}

/// A malformed datagram is ignored, and a later valid one still wins.
#[tokio::test]
async fn a_malformed_datagram_is_ignored() {
    let (discovery, sender) = bind(None);
    let (handle, addr) = spawn(discovery);
    must(sender.send_to(b"EXPR not a beacon", addr), "send garbage");
    send(&sender, addr, beacon(FIRST, "Explorer-AAAA", 9000, Flags::PROVISIONED));

    let (entry, _) = selected(wait_choice(&handle, |choice| matches!(choice, Choice::Beacon { .. })).await);
    assert_eq!(entry.device_id, DeviceId::new(FIRST));

    handle.stop();
    handle.wait().await;
}

/// The module pins the ADR's port defaults.
#[test]
fn the_ports_are_the_adr_defaults() {
    assert_eq!(remote_daemon::discovery::BEACON_PORT, 9001);
    assert_eq!(LINK_PORT, 9000);
    assert_eq!(beacon::INTERVAL_MS, 3_000);
}
