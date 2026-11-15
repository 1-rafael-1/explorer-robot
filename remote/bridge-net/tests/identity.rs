//! Host tests for the Device Name derivation (ADR-0050).
//!
//! One derivation produces the DHCP hostname, the setup AP's SSID and the Discovery Beacon's short
//! label, so the three representations share a suffix and cannot drift.

use bridge_net::identity::{DeviceId, DeviceName, Name};

/// A bench device id: its last two bytes are `AB CD`.
const DEVICE_ID: DeviceId = DeviceId::new([0x24, 0x0A, 0xC4, 0x12, 0xAB, 0xCD]);

/// The full name is the DHCP hostname and the setup AP's SSID, exactly eighteen bytes.
#[test]
fn the_full_name_is_the_hostname_and_the_setup_ssid() {
    let name = DeviceName::from_device_id(DEVICE_ID);

    assert_eq!(name.as_str(), "ExplorerRobot-ABCD");
    assert_eq!(name.bytes(), b"ExplorerRobot-ABCD");
    assert_eq!(name.bytes().len(), DeviceName::ENCODED_LEN);
    assert_eq!(DeviceName::ENCODED_LEN, 18);
}

/// The Beacon's label is the short form, with the same suffix and the field's zero padding.
#[test]
fn the_beacon_label_is_the_short_form_with_the_same_suffix() {
    let name = DeviceName::from_device_id(DEVICE_ID);
    let label = name.beacon_label();

    assert_eq!(label.as_str(), "Explorer-ABCD");
    assert_eq!(label.padded(), *b"Explorer-ABCD\0\0\0");
    assert_eq!(name.suffix(), "ABCD");
}

/// Both representations carry the station MAC's last two bytes as upper-case hex.
#[test]
fn the_suffix_is_the_last_two_mac_bytes_as_upper_case_hex() {
    for (id, suffix) in [
        ([0x00, 0x00, 0x00, 0x00, 0x00, 0x0F], "000F"),
        ([0x00, 0x00, 0x00, 0x00, 0x0F, 0xA0], "0FA0"),
        ([0xFF, 0xFF, 0xFF, 0xFF, 0xA5, 0x5A], "A55A"),
    ] {
        let name = DeviceName::from_device_id(DeviceId::new(id));

        assert_eq!(name.suffix(), suffix);
        assert_eq!(name.as_str(), format!("ExplorerRobot-{suffix}"));
        assert_eq!(name.beacon_label().as_str(), format!("Explorer-{suffix}"));
    }
}

/// Only the last two bytes matter, and the name is stable across derivations.
#[test]
fn the_name_is_stable_and_ignores_the_macs_first_four_bytes() {
    let name = DeviceName::from_device_id(DEVICE_ID);
    let same_suffix = DeviceName::from_device_id(DeviceId::new([0x01, 0x02, 0x03, 0x04, 0xAB, 0xCD]));
    let other_suffix = DeviceName::from_device_id(DeviceId::new([0x24, 0x0A, 0xC4, 0x12, 0xAB, 0xCE]));

    assert_eq!(name, DeviceName::from_device_id(DEVICE_ID));
    assert_eq!(name.as_str(), DeviceName::from_device_id(DEVICE_ID).as_str());
    assert_eq!(name, same_suffix);
    assert_ne!(name, other_suffix);
}

/// The full name cannot fit the Beacon's name field; the short label can.
#[test]
fn the_full_name_cannot_fit_the_beacon_field() {
    let name = DeviceName::from_device_id(DEVICE_ID);

    assert!(Name::new(name.as_str()).is_none());
    assert!(Name::new(name.beacon_label().as_str()).is_some());
}
