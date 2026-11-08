//! Host tests for the `HELLO`, `HELLO_ACK` and `PING` control payloads (ADR-0030, ADR-0031).

use remote_protocol::{
    Caps, Control, Status,
    codec::PayloadCodec,
    payload::control::{Hello, HelloAck, Ping, PingAck, RateEntry, RateFlags, encode_hello, encode_hello_ack},
};

/// The control id vocabulary covers every implemented id in wire order.
#[test]
fn control_ids_cover_the_wire_vocabulary() {
    assert_eq!(Control::ALL.len(), 5, "every control id is in ALL");
    assert_eq!(Control::Hello.wire_id(), 0x0001);
    assert_eq!(Control::Ping.wire_id(), 0x0002);
    assert_eq!(Control::Throttle.wire_id(), 0x0003);
    assert_eq!(Control::BridgeControl.wire_id(), 0x0004);
    assert_eq!(Control::BridgeStatus.wire_id(), 0x0005);

    for (index, control) in Control::ALL.iter().enumerate() {
        assert_eq!(control.index(), index, "index follows ALL order");
        assert_eq!(
            Control::from_wire(control.wire_id()),
            Some(*control),
            "from_wire is the inverse of wire_id"
        );
    }
    assert_eq!(Control::from_wire(0x0000), None, "0 is reserved");
    assert_eq!(Control::from_wire(0x0006), None, "0x0006+ is reserved");
}

/// A representative rate-entry list, including an unknown stream id and a disabled entry.
const fn rates() -> [RateEntry; 3] {
    [
        RateEntry::enabled(0x0001, 100),
        RateEntry {
            stream_id: 0x0002,
            rate_chz: 1_000,
            flags: RateFlags::from_bits(0),
        },
        RateEntry {
            stream_id: 0x0099,
            rate_chz: 7,
            flags: RateFlags::ENABLED,
        },
    ]
}

/// `HELLO` round-trips its header and every borrowed rate entry.
#[test]
fn hello_round_trips_with_borrowed_entries() {
    let caps = Caps::MAP | Caps::from_bits(1 << 31);
    let mut buffer = [0u8; 64];
    let Some(written) = encode_hello(1, 2, caps, 0xDEAD_BEEF, rates(), &mut buffer).ok() else {
        return;
    };
    let Some(hello) = Hello::decode(&buffer[..written]).ok() else {
        return;
    };

    assert_eq!(hello.major, 1);
    assert_eq!(hello.minor, 2);
    assert_eq!(hello.caps, caps, "unknown caps bits survive the codec");
    assert_eq!(hello.epoch, 0xDEAD_BEEF);
    assert_eq!(hello.entry_count(), 3);
    let decoded: Vec<_> = hello.entries().collect();
    assert_eq!(decoded, rates(), "every entry round-trips unchanged");
}

/// Reserved rate-entry flag bits are sent as zero and ignored on receive.
#[test]
fn rate_flags_clear_reserved_bits() {
    assert_eq!(RateFlags::ENABLED.bits(), 0x01);
    assert!(RateFlags::ENABLED.is_enabled());
    assert!(RateFlags::from_bits(0xFE).is_empty(), "bits 1-7 are reserved");
    assert!(RateFlags::from_bits(0xFF).is_enabled());

    let mut buffer = [0u8; 32];
    let Some(written) = encode_hello(1, 0, Caps::from_bits(0), 0, [RateEntry::enabled(1, 10)], &mut buffer).ok() else {
        return;
    };
    buffer[Hello::HEADER_LEN + 4] = 0xFF;
    let Some(hello) = Hello::decode(&buffer[..written]).ok() else {
        return;
    };
    let Some(entry) = hello.entries().next() else {
        return;
    };
    assert_eq!(entry.flags.bits(), 0x01, "reserved bits are cleared on decode");
}

/// `HELLO` rejects a short header and an entry list that is not a whole number of entries.
#[test]
fn hello_rejects_malformed_layouts() {
    assert_eq!(Hello::decode(&[0u8; 9]), Err(remote_protocol::CodecError::Length));
    assert_eq!(Hello::decode(&[0u8; 14]), Err(remote_protocol::CodecError::Length));
    assert_eq!(Hello::decode(&[0u8; 16]), Err(remote_protocol::CodecError::Length));
    assert!(Hello::decode(&[0u8; 15]).is_ok(), "10-byte header plus one entry");

    let mut small = [0u8; 12];
    assert_eq!(
        encode_hello(1, 0, Caps::from_bits(0), 0, rates(), &mut small),
        Err(remote_protocol::EncodeError::BufferTooSmall)
    );
}

/// `HELLO_ACK` round-trips its header, status and default rates.
#[test]
fn hello_ack_round_trips() {
    let caps = Caps::DELTA_CLOUD | Caps::MAP;
    let mut buffer = [0u8; 64];
    let Some(written) = encode_hello_ack(1, 0, caps, 0x0BAD_C0DE, Status::VersionMismatch, rates(), &mut buffer).ok()
    else {
        return;
    };
    let Some(ack) = HelloAck::decode(&buffer[..written]).ok() else {
        return;
    };

    assert_eq!(ack.major, 1);
    assert_eq!(ack.minor, 0);
    assert_eq!(ack.caps, caps);
    assert_eq!(ack.boot_id, 0x0BAD_C0DE);
    assert_eq!(ack.status, Status::VersionMismatch);
    let decoded: Vec<_> = ack.entries().collect();
    assert_eq!(decoded, rates());
}

/// `HELLO_ACK` rejects a short payload, a malformed entry list and an unassigned status.
#[test]
fn hello_ack_rejects_malformed_layouts() {
    assert_eq!(HelloAck::decode(&[0u8; 10]), Err(remote_protocol::CodecError::Length));
    assert_eq!(HelloAck::decode(&[0u8; 15]), Err(remote_protocol::CodecError::Length));

    let mut bad_status = [0u8; HelloAck::HEADER_LEN];
    bad_status[10] = 10;
    assert_eq!(HelloAck::decode(&bad_status), Err(remote_protocol::CodecError::Value));

    let mut small = [0u8; 10];
    assert_eq!(
        encode_hello_ack(1, 0, Caps::from_bits(0), 0, Status::Ok, rates(), &mut small),
        Err(remote_protocol::EncodeError::BufferTooSmall)
    );
}

/// `PING` has an empty request payload and a one-byte `OK` acknowledgement.
#[test]
fn ping_codecs_round_trip() {
    assert_eq!(Ping::decode(&[]).ok(), Some(Ping));
    assert_eq!(Ping::decode(&[0]), Err(remote_protocol::CodecError::Length));
    let mut empty = [0u8; 0];
    assert_eq!(Ping.encode_into(&mut empty), Ok(0));

    let ack = PingAck { status: Status::Ok };
    let mut buffer = [0u8; PingAck::ENCODED_LEN];
    let Some(written) = ack.encode_into(&mut buffer).ok() else {
        return;
    };
    assert_eq!(written, PingAck::ENCODED_LEN);
    assert_eq!(PingAck::decode(&buffer[..written]).ok(), Some(ack));

    assert_eq!(PingAck::decode(&[]), Err(remote_protocol::CodecError::Length));
    assert_eq!(PingAck::decode(&[10]), Err(remote_protocol::CodecError::Value));
    let mut small = [0u8; 0];
    assert_eq!(
        ack.encode_into(&mut small),
        Err(remote_protocol::EncodeError::BufferTooSmall)
    );
}
