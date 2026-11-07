//! Host tests for the hoisted Beacon codec (ADR-0037).
//!
//! The datagram layout is pinned against the ADR's field table, and the identity helpers are
//! checked for their padding and UTF-8 rules.

use bridge_net::{
    beacon::{self, Beacon},
    identity::{DeviceId, Flags, Identity, Name},
};

/// A bench identity used across the Beacon tests.
fn identity() -> Identity {
    Identity {
        device_id: DeviceId::new(*b"dev001"),
        name: Name::new("Explorer-0001").unwrap_or_else(|| Name::from_padded([0u8; Name::ENCODED_LEN])),
        flags: Flags::PROVISIONED | Flags::LINK_READY,
    }
}

/// Extracts a value, failing the test rather than unwrapping (the workspace denies `unwrap`).
fn must<T>(value: Option<T>, what: &str) -> T {
    value.unwrap_or_else(|| {
        assert_eq!(what.len(), usize::MAX, "missing {what}");
        std::process::abort()
    })
}

/// Extracts a result, failing the test rather than unwrapping it.
fn must_ok<T, E: std::fmt::Debug>(result: Result<T, E>, what: &str) -> T {
    match result {
        Ok(value) => value,
        Err(error) => {
            assert_eq!(format!("{what}: {error:?}").len(), usize::MAX, "unexpected error");
            std::process::abort()
        }
    }
}

/// The datagram pins ADR-0037's layout: magic, major, device id, name, port, flags.
#[test]
fn the_beacon_pins_the_adr_wire_layout() {
    let identity = identity();
    let beacon = Beacon {
        device_id: identity.device_id,
        name: identity.name,
        tcp_port: 9_000,
        flags: identity.flags,
    };
    let mut bytes = [0u8; beacon::ENCODED_LEN];
    let written = must_ok(beacon.encode_into(&mut bytes), "the beacon encodes");

    assert_eq!(written, beacon::ENCODED_LEN);
    assert_eq!(&bytes[..4], b"EXPR", "the magic is EXPR");
    assert_eq!(bytes[4], beacon::LINK_MAJOR, "the major is the Link Protocol's");
    assert_eq!(&bytes[5..11], b"dev001", "the device id follows the major");
    assert_eq!(
        &bytes[11..27],
        b"Explorer-0001\0\0\0",
        "the name is zero-padded to sixteen"
    );
    assert_eq!(
        u16::from_le_bytes([bytes[27], bytes[28]]),
        9_000,
        "the port is little-endian before the flags"
    );
    assert_eq!(
        bytes[29],
        beacon::FLAG_PROVISIONED | beacon::FLAG_LINK_READY,
        "the flags close the datagram"
    );
}

/// The datagram round-trips, including the portal-only zero port.
#[test]
fn the_beacon_round_trips() {
    let identity = identity();
    for tcp_port in [9_000u16, beacon::PORTAL_ONLY] {
        let beacon = Beacon {
            device_id: identity.device_id,
            name: identity.name,
            tcp_port,
            flags: identity.flags,
        };
        let mut bytes = [0u8; beacon::ENCODED_LEN];
        assert!(beacon.encode_into(&mut bytes).is_ok());
        assert_eq!(Beacon::decode(&bytes), Ok(beacon));
    }
}

/// Reserved flag bits are cleared on decode and never emitted.
#[test]
fn reserved_flag_bits_are_ignored() {
    let mut bytes = [0u8; beacon::ENCODED_LEN];
    let identity = identity();
    let beacon = Beacon {
        device_id: identity.device_id,
        name: identity.name,
        tcp_port: 9_000,
        flags: Flags::from_bits(0xFF),
    };
    assert!(beacon.encode_into(&mut bytes).is_ok());
    assert_eq!(
        bytes[29],
        beacon::FLAG_PROVISIONED | beacon::FLAG_LINK_READY | beacon::FLAG_SETUP_MODE,
        "reserved bits are never sent"
    );

    bytes[29] = 0xFF;
    let decoded = must_ok(Beacon::decode(&bytes), "the beacon decodes");
    assert_eq!(
        decoded.flags,
        Flags::PROVISIONED | Flags::LINK_READY | Flags::SETUP_MODE,
        "reserved bits are cleared on decode"
    );
}

/// A short datagram or a bad magic is rejected, and trailing bytes are ignored.
#[test]
fn malformed_datagrams_are_rejected() {
    let mut bytes = [0u8; beacon::ENCODED_LEN];
    let identity = identity();
    let beacon = Beacon {
        device_id: identity.device_id,
        name: identity.name,
        tcp_port: 9_000,
        flags: identity.flags,
    };
    assert!(beacon.encode_into(&mut bytes).is_ok());

    assert!(
        Beacon::decode(&bytes[..beacon::ENCODED_LEN - 1]).is_err(),
        "a short datagram is rejected"
    );
    let mut wrong_magic = bytes;
    wrong_magic[0] = b'X';
    assert!(Beacon::decode(&wrong_magic).is_err(), "a bad magic is rejected");

    let mut extended = bytes.to_vec();
    extended.push(0xAA);
    assert_eq!(Beacon::decode(&extended), Ok(beacon), "trailing bytes are ignored");
}

/// A name that is not UTF-8 on the wire is rejected, and the helpers pad and trim correctly.
#[test]
fn the_name_helpers_pad_trim_and_validate() {
    let name = must(Name::new("Explorer-STUB"), "the name fits");
    assert_eq!(name.as_str(), "Explorer-STUB");
    assert_eq!(name.padded(), *b"Explorer-STUB\0\0\0");

    assert!(
        Name::new("a name longer than sixteen").is_none(),
        "over-long names are refused"
    );
    assert!(Name::new("zero\0inside").is_none(), "embedded zeros are refused");

    let invalid = [0xFFu8; Name::ENCODED_LEN];
    assert!(
        Name::from_wire(invalid).is_err(),
        "invalid UTF-8 before the padding is rejected"
    );
}
