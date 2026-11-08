//! The serial loopback self-test (ticket 02): a full `HELLO` round trip through the transport
//! with no Bridge present.
//!
//! The peer plays the Remote's side of the seam directly, so the round trip proves the transport
//! and the stub's `LinkCore`-driven handshake end to end, entirely in memory.

mod support;

use link_core::timing::DEFAULT_RATES_CHZ;
use link_protocol::{
    Caps, Class, Control, Flags, PROTOCOL_MAJOR, PROTOCOL_MINOR, PayloadCodec, Status, Stream,
    payload::control::HelloAck,
};
use robot_stub::{Config, Persona, Stub};
use support::Peer;

/// The Boot Id the stub mints for this powered link period.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// The robot's capabilities in this test, chosen to make the intersection observable.
fn config() -> Config {
    Config {
        robot_caps: Caps::MAP | Caps::THROTTLE_SUPPORT,
        ..Config::default()
    }
}

/// A full `HELLO` round trip crosses the transport with no Bridge present: the peer's `HELLO` is
/// answered by a `HELLO_ACK` that mirrors the sequence, reports the stub's Boot Id, intersects
/// the capabilities and carries one default rate entry per stream (ADR-0030).
#[test]
fn hello_round_trip_crosses_the_transport_without_a_bridge() {
    let (stub_end, peer_end) = robot_stub::duplex();
    let mut stub = Stub::new(config(), BOOT_ID, Persona::Baseline, stub_end);
    let mut peer = Peer::new(peer_end);

    let peer_caps = Caps::MAP | Caps::DELTA_CLOUD | Caps::from_bits(1 << 31);
    let seq = peer.send_hello(7, peer_caps, &[]);
    stub.step(0);
    let read = peer.pump(0);
    assert!(read > 0, "the HELLO_ACK must cross the transport");

    let replies = peer.frames_of(Class::Control, Control::Hello.wire_id(), None, 0);
    assert_eq!(replies.len(), 1, "exactly one HELLO_ACK is sent");
    let reply = replies[0];
    assert!(reply.flags.contains(Flags::IS_ACK), "the reply is an acknowledgement");
    assert_eq!(reply.seq, seq, "the reply mirrors the HELLO's sequence");

    let ack = HelloAck::decode(&reply.payload);
    assert!(ack.is_ok(), "the HELLO_ACK payload must decode");
    if let Ok(ack) = ack {
        assert_eq!(ack.major, PROTOCOL_MAJOR);
        assert_eq!(ack.minor, PROTOCOL_MINOR);
        assert_eq!(ack.boot_id, BOOT_ID);
        assert_eq!(ack.status, Status::Ok);
        assert_eq!(ack.caps, Caps::MAP, "the capabilities are intersected with the robot's");
        assert_eq!(ack.entry_count(), Stream::COUNT, "one default entry per stream");
        let entries: Vec<_> = ack.entries().collect();
        for stream in Stream::ALL {
            let entry = entries[stream.index()];
            assert_eq!(entry.stream_id, stream.wire_id());
            assert_eq!(entry.rate_chz, DEFAULT_RATES_CHZ[stream.index()]);
            assert!(entry.flags.is_enabled());
        }
    }
}
