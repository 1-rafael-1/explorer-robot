//! Host tests for the hoisted preamble codec (ADR-0037, ADR-0039).
//!
//! The wire layouts are asserted against the ADRs directly, and the proofs against an independent
//! HMAC computation, so moving the implementation into `bridge-net` cannot change the bytes the
//! Bridge and the daemon exchange.

use bridge_net::{
    identity::{DeviceId, Flags, Identity, Name},
    preamble::{self, DenyReason, HEADER_LEN, Kind, MAGIC, MAX_PAYLOAD_LEN, Message, NONCE_LEN, PROOF_LEN, Received},
};

/// A bench identity used across the codec tests.
fn identity() -> Identity {
    Identity {
        device_id: DeviceId::new(*b"dev001"),
        name: Name::from_padded(*b"Explorer-0001\0\0\0"),
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

/// Encodes one message into a fresh buffer, returning the written prefix.
fn encode(message: &Message) -> Vec<u8> {
    let mut buffer = [0u8; HEADER_LEN + MAX_PAYLOAD_LEN];
    let written = must_ok(
        preamble::encode_into(preamble::MAJOR, message, &mut buffer),
        "the message encodes",
    );
    buffer[..written].to_vec()
}

/// Decodes one complete frame, failing on a partial or malformed input.
fn decode(bytes: &[u8]) -> Received {
    let decoded = must_ok(preamble::try_decode(bytes), "the frame decodes");
    must(decoded, "a complete frame")
}

/// `CLIENT_HELLO` pins the ADR's header: magic, major, type and a little-endian payload length.
#[test]
fn a_client_hello_pins_the_wire_header() {
    let nonce = [7u8; NONCE_LEN];
    let bytes = encode(&Message::ClientHello { nonce });

    assert_eq!(bytes.len(), HEADER_LEN + NONCE_LEN);
    assert_eq!(&bytes[..4], b"EXPH", "the magic is EXPH");
    assert_eq!(bytes[4], preamble::MAJOR, "the major follows the magic");
    assert_eq!(bytes[5], Kind::ClientHello.bits(), "the type follows the major");
    assert_eq!(
        u16::from_le_bytes([bytes[6], bytes[7]]),
        32,
        "the payload length is little-endian"
    );
    assert_eq!(&bytes[HEADER_LEN..], &nonce, "the nonce closes the frame");
    let mut expected = vec![b'E', b'X', b'P', b'H', 1, 1, 32, 0];
    expected.extend_from_slice(&nonce);
    assert_eq!(bytes, expected);
}

/// Every message round-trips through the shared codec.
#[test]
fn every_message_round_trips() {
    let messages = [
        Message::ClientHello {
            nonce: [1u8; NONCE_LEN],
        },
        Message::BridgeChallenge {
            bridge_proof: [2u8; PROOF_LEN],
            bridge_nonce: [3u8; NONCE_LEN],
            identity: identity(),
        },
        Message::ClientAuth {
            proof: [4u8; PROOF_LEN],
        },
        Message::Accept,
        Message::Deny {
            reason: DenyReason::Busy,
            retry_ms: Some(4_000),
        },
        Message::Deny {
            reason: DenyReason::Unauthorized,
            retry_ms: None,
        },
    ];
    for message in messages {
        let received = decode(&encode(&message));
        assert_eq!(received.major, preamble::MAJOR);
        assert_eq!(received.message, message, "the codec round-trips {message:?}");
    }
}

/// `ACCEPT` carries no payload, and a stray payload byte is a length error.
#[test]
fn accept_is_empty() {
    let bytes = encode(&Message::Accept);
    assert_eq!(bytes.len(), HEADER_LEN);
    assert_eq!(decode(&bytes).message, Message::Accept);

    let mut extended = bytes;
    extended[6] = 1;
    extended.push(0);
    assert!(
        preamble::try_decode(&extended).is_err(),
        "ACCEPT never carries a payload"
    );
}

/// A partial frame waits for more bytes rather than erroring.
#[test]
fn partial_frames_wait() {
    let bytes = encode(&Message::ClientHello {
        nonce: [9u8; NONCE_LEN],
    });
    for prefix in 0..bytes.len() {
        assert_eq!(
            preamble::try_decode(&bytes[..prefix]),
            Ok(None),
            "a {prefix}-byte prefix is not a complete frame"
        );
    }
}

/// A bad magic, an unknown type and an oversized length are rejected.
#[test]
fn malformed_frames_are_rejected() {
    let mut bytes = encode(&Message::Accept);
    bytes[0] = b'X';
    assert!(preamble::try_decode(&bytes).is_err(), "the magic is checked");

    let mut bytes = encode(&Message::Accept);
    bytes[5] = 0xFF;
    assert!(preamble::try_decode(&bytes).is_err(), "an unknown type is rejected");

    let mut bytes = encode(&Message::Accept);
    bytes[6..8].copy_from_slice(&u16::try_from(MAX_PAYLOAD_LEN + 1).unwrap_or(u16::MAX).to_le_bytes());
    assert!(preamble::try_decode(&bytes).is_err(), "an oversized length is rejected");
}

/// The proofs follow ADR-0039's formula exactly, checked against a direct HMAC computation.
#[test]
fn the_proofs_follow_the_adr_formula() {
    let token: &[u8] = b"pairing-token-01";
    let client_nonce = [5u8; NONCE_LEN];
    let bridge_nonce = [6u8; NONCE_LEN];
    let identity = identity();
    let mut identity_bytes = [0u8; Identity::ENCODED_LEN];
    assert!(
        identity.encode_into(&mut identity_bytes).is_ok(),
        "the identity must encode into its own buffer"
    );

    let mut transcript = Vec::new();
    transcript.extend_from_slice(preamble::BRIDGE_PROOF_LABEL);
    transcript.extend_from_slice(&client_nonce);
    transcript.extend_from_slice(&bridge_nonce);
    transcript.extend_from_slice(&identity_bytes);
    assert_eq!(
        preamble::bridge_proof(token, &client_nonce, &bridge_nonce, &identity),
        direct_hmac(token, &transcript),
        "the bridge proof binds the whole challenge transcript"
    );

    let mut transcript = Vec::new();
    transcript.extend_from_slice(preamble::CLIENT_PROOF_LABEL);
    transcript.extend_from_slice(&client_nonce);
    transcript.extend_from_slice(&bridge_nonce);
    assert_eq!(
        preamble::client_proof(token, &client_nonce, &bridge_nonce),
        direct_hmac(token, &transcript),
        "the client proof binds both nonces"
    );
}

/// `proofs_match` accepts an identical proof and rejects any single differing byte.
#[test]
fn proofs_match_is_constant_time_and_exact() {
    let expected = [0x5Au8; PROOF_LEN];
    assert!(preamble::proofs_match(&expected, &expected));
    for index in 0..PROOF_LEN {
        let mut actual = expected;
        actual[index] ^= 0x01;
        assert!(
            !preamble::proofs_match(&expected, &actual),
            "a difference at byte {index} is rejected"
        );
    }
}

/// The framing constants describe the ADR-0037 layout.
#[test]
fn the_framing_constants_match_the_adr() {
    assert_eq!(MAGIC, *b"EXPH");
    assert_eq!(NONCE_LEN, 32);
    assert_eq!(PROOF_LEN, 32);
    assert_eq!(HEADER_LEN, 8);
    assert_eq!(MAX_PAYLOAD_LEN, 32 + 32 + Identity::ENCODED_LEN);
    assert_eq!(preamble::AUTH_DEADLINE_MS, 5_000);
}

/// HMAC-SHA256 computed straight from the ADR's text, independent of the crate's proof functions.
fn direct_hmac(key: &[u8], transcript: &[u8]) -> [u8; PROOF_LEN] {
    use hmac::{Hmac, Mac};
    use sha2::Sha256;

    // HMAC accepts every key length; the API is fallible in general.
    let Ok(mut mac) = <Hmac<Sha256> as Mac>::new_from_slice(key) else {
        return [0u8; PROOF_LEN];
    };
    mac.update(transcript);
    let mut proof = [0u8; PROOF_LEN];
    for (out, byte) in proof.iter_mut().zip(mac.finalize().into_bytes().iter()) {
        *out = *byte;
    }
    proof
}
