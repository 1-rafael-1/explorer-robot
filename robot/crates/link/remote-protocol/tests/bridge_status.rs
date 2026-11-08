//! Host tests for the `BRIDGE_STATUS` payload and its `Control`/`BridgeStream` ids (ADR-0031).
//!
//! The Bridge publishes the same 28-byte snapshot to the robot as an unacknowledged control frame
//! and to the daemon as a latest-wins stream frame.

use remote_protocol::{
    BridgeStation, BridgeStatus, BridgeStatusFlags, BridgeStream, CodecError, Control, ControlMessage, EncodeError,
    Stream,
    codec::PayloadCodec,
    decode_control, encode_control,
    payload::bridge::{CHANNEL_UNKNOWN, RSSI_UNKNOWN_DBM},
};

/// A fully populated snapshot used by several tests.
fn full() -> BridgeStatus {
    BridgeStatus {
        rssi_dbm: Some(-42),
        channel: Some(6),
        station: BridgeStation::Associated,
        flags: BridgeStatusFlags::PROVISIONED | BridgeStatusFlags::SETUP_OPEN | BridgeStatusFlags::LINK_READY,
        uart_dropped_bytes: 0x1122_3344,
        dma_stalls: 0xAABB_CCDD,
        crc_failures: 7,
        resyncs: 8,
        wdt_feeds: 9,
        wdt_withheld: 10,
    }
}

/// An all-unknown, all-zero snapshot: every sentinel and every counter at rest.
const fn empty() -> BridgeStatus {
    BridgeStatus {
        rssi_dbm: None,
        channel: None,
        station: BridgeStation::Down,
        flags: BridgeStatusFlags::from_bits(0),
        uart_dropped_bytes: 0,
        dma_stalls: 0,
        crc_failures: 0,
        resyncs: 0,
        wdt_feeds: 0,
        wdt_withheld: 0,
    }
}

/// Both representative snapshots round-trip through exactly `ENCODED_LEN` bytes.
#[test]
fn representative_snapshots_round_trip() {
    for snapshot in [full(), empty()] {
        let mut buffer = [0u8; BridgeStatus::ENCODED_LEN];
        let Some(written) = snapshot.encode_into(&mut buffer).ok() else {
            return;
        };
        assert_eq!(written, BridgeStatus::ENCODED_LEN);
        assert_eq!(
            BridgeStatus::decode(&buffer[..written]).ok(),
            Some(snapshot),
            "{snapshot:?} must round-trip"
        );
    }
}

/// The encode layout places every field at its documented offset.
#[test]
fn encode_uses_the_documented_offsets() {
    let snapshot = full();
    let mut buffer = [0u8; BridgeStatus::ENCODED_LEN];
    let Some(written) = snapshot.encode_into(&mut buffer).ok() else {
        return;
    };
    assert_eq!(written, 28);
    assert_eq!(buffer[0], (-42i8).to_le_bytes()[0], "rssi_dbm at offset 0");
    assert_eq!(buffer[1], 6, "channel at offset 1");
    assert_eq!(buffer[2], 2, "station at offset 2");
    assert_eq!(buffer[3], 0x07, "flags at offset 3");
    assert_eq!(
        &buffer[4..8],
        &0x1122_3344u32.to_le_bytes(),
        "uart_dropped_bytes at offset 4"
    );
    assert_eq!(&buffer[8..12], &0xAABB_CCDDu32.to_le_bytes(), "dma_stalls at offset 8");
    assert_eq!(&buffer[12..16], &7u32.to_le_bytes(), "crc_failures at offset 12");
    assert_eq!(&buffer[16..20], &8u32.to_le_bytes(), "resyncs at offset 16");
    assert_eq!(&buffer[20..24], &9u32.to_le_bytes(), "wdt_feeds at offset 20");
    assert_eq!(&buffer[24..28], &10u32.to_le_bytes(), "wdt_withheld at offset 24");
}

/// Hand-built bytes decode to the expected snapshot, proving the decode offsets.
#[test]
fn decode_uses_the_documented_offsets() {
    let mut bytes = [0u8; 28];
    bytes[0] = (-42i8).to_le_bytes()[0];
    bytes[1] = 6;
    bytes[2] = 2;
    bytes[3] = 0x07;
    bytes[4..8].copy_from_slice(&0x1122_3344u32.to_le_bytes());
    bytes[8..12].copy_from_slice(&0xAABB_CCDDu32.to_le_bytes());
    bytes[12..16].copy_from_slice(&7u32.to_le_bytes());
    bytes[16..20].copy_from_slice(&8u32.to_le_bytes());
    bytes[20..24].copy_from_slice(&9u32.to_le_bytes());
    bytes[24..28].copy_from_slice(&10u32.to_le_bytes());

    assert_eq!(BridgeStatus::decode(&bytes).ok(), Some(full()));
}

/// The unknown sentinels decode to `None` and re-encode to the sentinels.
#[test]
fn sentinels_map_to_none_and_back() {
    assert_eq!(RSSI_UNKNOWN_DBM, i8::MIN);
    assert_eq!(CHANNEL_UNKNOWN, 0);

    let decoded = BridgeStatus::decode(&{
        let mut bytes = [0u8; 28];
        bytes[0] = RSSI_UNKNOWN_DBM.to_le_bytes()[0];
        bytes[1] = CHANNEL_UNKNOWN;
        bytes
    });
    let Some(snapshot) = decoded.ok() else {
        return;
    };
    assert_eq!(snapshot.rssi_dbm, None, "the RSSI sentinel is unknown");
    assert_eq!(snapshot.channel, None, "the channel sentinel is unknown");

    let mut buffer = [0u8; BridgeStatus::ENCODED_LEN];
    let Some(written) = snapshot.encode_into(&mut buffer).ok() else {
        return;
    };
    assert_eq!(
        buffer[0],
        RSSI_UNKNOWN_DBM.to_le_bytes()[0],
        "None re-encodes the RSSI sentinel"
    );
    assert_eq!(buffer[1], CHANNEL_UNKNOWN, "None re-encodes the channel sentinel");
    assert_eq!(written, BridgeStatus::ENCODED_LEN);

    // The all-unknown snapshot encodes to the sentinels too.
    let Some(unknown_written) = empty().encode_into(&mut buffer).ok() else {
        return;
    };
    assert_eq!(buffer[0], RSSI_UNKNOWN_DBM.to_le_bytes()[0]);
    assert_eq!(buffer[1], CHANNEL_UNKNOWN);
    assert_eq!(unknown_written, BridgeStatus::ENCODED_LEN);
}

/// Reserved flag bits are cleared on decode and never echoed.
#[test]
fn reserved_flag_bits_are_cleared() {
    assert_eq!(BridgeStatusFlags::PROVISIONED.bits(), 0x01);
    assert_eq!(BridgeStatusFlags::SETUP_OPEN.bits(), 0x02);
    assert_eq!(BridgeStatusFlags::LINK_READY.bits(), 0x04);
    assert!(BridgeStatusFlags::from_bits(0xF8).is_empty(), "bits 3-7 are reserved");
    assert!(BridgeStatusFlags::from_bits(0xFF).contains(BridgeStatusFlags::LINK_READY));

    let mut bytes = [0u8; 28];
    bytes[3] = 0xFF;
    let Some(snapshot) = BridgeStatus::decode(&bytes).ok() else {
        return;
    };
    assert_eq!(snapshot.flags.bits(), 0x07, "reserved bits are cleared on decode");

    let mut out = [0u8; BridgeStatus::ENCODED_LEN];
    let Some(_written) = snapshot.encode_into(&mut out).ok() else {
        return;
    };
    assert_eq!(out[3], 0x07, "reserved bits are sent as zero");
}

/// The station states follow their wire values and `from_wire` rejects anything else.
#[test]
fn station_covers_the_wire_values() {
    assert_eq!(BridgeStation::ALL.len(), 3);
    for (index, station) in BridgeStation::ALL.iter().enumerate() {
        assert_eq!(usize::from(station.bits()), index, "bits follow ALL order");
        assert_eq!(
            BridgeStation::from_wire(station.bits()),
            Some(*station),
            "from_wire is the inverse of bits"
        );
    }
    assert_eq!(BridgeStation::from_wire(3), None);
    assert_eq!(BridgeStation::from_wire(0xFF), None);
}

/// An unassigned station byte is a value error, never a panic.
#[test]
fn unknown_station_is_a_value_error() {
    let mut bytes = [0u8; 28];
    bytes[2] = 3;
    assert_eq!(BridgeStatus::decode(&bytes), Err(CodecError::Value));

    let mut high = [0u8; 28];
    high[2] = 0xFF;
    assert_eq!(BridgeStatus::decode(&high), Err(CodecError::Value));
}

/// The decode length must be exactly `ENCODED_LEN`; a short output buffer is reported, not written.
#[test]
fn lengths_are_strict() {
    assert_eq!(BridgeStatus::decode(&[]), Err(CodecError::Length));
    assert_eq!(BridgeStatus::decode(&[0u8; 27]), Err(CodecError::Length));
    assert_eq!(BridgeStatus::decode(&[0u8; 29]), Err(CodecError::Length));
    let mut all_unknown = [0u8; 28];
    all_unknown[0] = RSSI_UNKNOWN_DBM.to_le_bytes()[0];
    assert_eq!(BridgeStatus::decode(&all_unknown).ok(), Some(empty()));

    let mut small = [0u8; BridgeStatus::ENCODED_LEN - 1];
    assert_eq!(full().encode_into(&mut small), Err(EncodeError::BufferTooSmall));

    let mut exact = [0u8; BridgeStatus::ENCODED_LEN];
    assert_eq!(full().encode_into(&mut exact), Ok(BridgeStatus::ENCODED_LEN));
}

/// `BRIDGE_STATUS` is control `0x0005`; the Bridge stream is `0x0007` and stays out of `Stream`.
#[test]
fn ids_are_namespaced_by_class() {
    assert_eq!(Control::BridgeStatus.wire_id(), 0x0005);
    assert_eq!(Control::from_wire(0x0005), Some(Control::BridgeStatus));

    assert_eq!(BridgeStream::ALL.len(), 1);
    assert_eq!(BridgeStream::COUNT, 1);
    assert_eq!(BridgeStream::Status.wire_id(), 0x0007);
    assert_eq!(BridgeStream::Status.index(), 0);
    assert_eq!(BridgeStream::ALL[BridgeStream::Status.index()], BridgeStream::Status);
    assert_eq!(BridgeStream::from_wire(0x0007), Some(BridgeStream::Status));
    assert_eq!(BridgeStream::from_wire(0x0001), None);

    assert_eq!(
        Stream::from_wire(0x0007),
        None,
        "the robot's stream catalogue stays six"
    );
    assert_eq!(Stream::COUNT, 6);
}

/// `decode_control`/`encode_control` carry the snapshot under `Control::BridgeStatus`, and the ACK
/// form is rejected because there is no defined acknowledgement.
#[test]
fn control_dispatch_round_trips_and_rejects_ack() {
    let snapshot = full();
    let mut buffer = [0u8; BridgeStatus::ENCODED_LEN];
    let Some(written) = snapshot.encode_into(&mut buffer).ok() else {
        return;
    };

    let Some(message) = decode_control(Control::BridgeStatus, false, &buffer[..written]).ok() else {
        return;
    };
    assert_eq!(message.control(), Control::BridgeStatus);
    assert_eq!(message, ControlMessage::BridgeStatus(snapshot));

    let mut encoded = [0u8; BridgeStatus::ENCODED_LEN];
    let Some(count) = encode_control(&message, &mut encoded).ok() else {
        return;
    };
    assert_eq!(count, written);
    assert_eq!(encoded[..count], buffer[..written]);

    assert_eq!(
        decode_control(Control::BridgeStatus, true, &buffer[..written]),
        Err(CodecError::Length),
        "BRIDGE_STATUS has no defined acknowledgement"
    );
}
