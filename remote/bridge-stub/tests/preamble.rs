//! The preamble's server side: the challenge, the proofs and the verdicts (ticket 09).

mod support;

use std::{
    io::Read,
    time::{Duration, Instant},
};

use bridge_stub::{DeviceId, Flags, Scenario, preamble::DenyReason};
use support::{BenchError, Handshake};

/// The token the tests pair with.
const TOKEN: &[u8] = b"pairing-token-01";

/// A correct client completes the preamble and reaches `ACCEPT`.
#[test]
fn a_correct_client_reaches_accept() -> Result<(), BenchError> {
    let stub = support::start(&Scenario::new(TOKEN))?;
    let (_client, _identity) = support::accepted(stub.link_addr(), TOKEN)?;
    Ok(())
}

/// The challenge serves the identity fields: device id, flags and name.
#[test]
fn the_challenge_serves_the_identity_fields() -> Result<(), BenchError> {
    let stub = support::start(&Scenario::new(TOKEN))?;
    let (_client, identity) = support::accepted(stub.link_addr(), TOKEN)?;
    assert_eq!(identity.device_id, DeviceId::new(*b"stub01"));
    assert_eq!(identity.name.as_str(), "Explorer-STUB");
    assert_eq!(identity.flags, Flags::PROVISIONED | Flags::LINK_READY);
    Ok(())
}

/// A wrong token is denied `unauthorized`, with no retry hint: the operator re-pairs.
#[test]
fn a_wrong_token_is_denied_unauthorized() -> Result<(), BenchError> {
    let stub = support::start(&Scenario::new(TOKEN))?;
    let handshake = support::Client::handshake_trusting(stub.link_addr(), b"not-the-token")?;
    let Handshake::Denied { reason, retry_ms } = handshake else {
        assert!(
            matches!(handshake, Handshake::Denied { .. }),
            "a wrong client proof is denied"
        );
        return Ok(());
    };
    assert_eq!(reason, DenyReason::Unauthorized);
    assert_eq!(retry_ms, None, "unauthorized is surfaced and not auto-retried");
    Ok(())
}

/// The proofs follow ADR-0039's formula exactly, checked against a direct HMAC computation.
#[test]
fn the_proofs_follow_the_adr_formula() {
    let token: &[u8] = b"token";
    let client_nonce = [7u8; bridge_stub::preamble::NONCE_LEN];
    let bridge_nonce = [9u8; bridge_stub::preamble::NONCE_LEN];
    let identity = bridge_stub::Identity {
        device_id: DeviceId::new(*b"device"),
        name: bridge_stub::Name::from_padded(*b"Explorer-0001\0\0\0"),
        flags: Flags::PROVISIONED | Flags::LINK_READY,
    };
    let mut identity_bytes = [0u8; bridge_stub::Identity::ENCODED_LEN];
    assert!(
        identity.encode_into(&mut identity_bytes).is_ok(),
        "the identity must encode into its own buffer"
    );

    // bridge_proof = HMAC(token, label || client_nonce || bridge_nonce || device_id || flags || name)
    let mut transcript = Vec::new();
    transcript.extend_from_slice(bridge_net::preamble::BRIDGE_PROOF_LABEL);
    transcript.extend_from_slice(&client_nonce);
    transcript.extend_from_slice(&bridge_nonce);
    transcript.extend_from_slice(&identity_bytes);
    assert_eq!(
        bridge_stub::preamble::bridge_proof(token, &client_nonce, &bridge_nonce, &identity),
        direct_hmac(token, &transcript),
        "the bridge proof binds the whole challenge transcript"
    );

    // client_proof = HMAC(token, label || client_nonce || bridge_nonce)
    let mut transcript = Vec::new();
    transcript.extend_from_slice(bridge_net::preamble::CLIENT_PROOF_LABEL);
    transcript.extend_from_slice(&client_nonce);
    transcript.extend_from_slice(&bridge_nonce);
    assert_eq!(
        bridge_stub::preamble::client_proof(token, &client_nonce, &bridge_nonce),
        direct_hmac(token, &transcript),
        "the client proof binds both nonces"
    );
}

/// The default auth deadline is ADR-0037's 5 s constant from the shared crate.
#[test]
fn the_default_auth_deadline_is_the_adr_constant() {
    assert_eq!(
        Scenario::new(TOKEN).auth_deadline_ms,
        bridge_net::preamble::AUTH_DEADLINE_MS
    );
}

/// A connection that does not authenticate within the deadline is dropped.
#[test]
fn a_silent_connection_is_dropped_at_the_deadline() -> Result<(), BenchError> {
    let mut scenario = Scenario::new(TOKEN);
    scenario.auth_deadline_ms = 250;
    let stub = support::start(&scenario)?;
    let mut stream = support::connect(stub.link_addr())?;

    let start = Instant::now();
    let mut buffer = [0u8; 8];
    let read = stream.read(&mut buffer);
    assert!(matches!(read, Ok(0)), "the silent connection is closed, not answered");
    assert!(
        start.elapsed() >= Duration::from_millis(250),
        "the drop waits for the deadline"
    );
    assert!(
        start.elapsed() < Duration::from_secs(10),
        "the drop does not outlive the deadline"
    );
    Ok(())
}

/// HMAC-SHA256 computed straight from the ADR's text, independent of the stub's proof functions.
fn direct_hmac(key: &[u8], transcript: &[u8]) -> [u8; bridge_stub::preamble::PROOF_LEN] {
    use hmac::{Hmac, Mac};
    use sha2::Sha256;

    // HMAC accepts every key length; the API is fallible in general.
    let Ok(mut mac) = <Hmac<Sha256> as Mac>::new_from_slice(key) else {
        return [0u8; bridge_stub::preamble::PROOF_LEN];
    };
    mac.update(transcript);
    let mut proof = [0u8; bridge_stub::preamble::PROOF_LEN];
    for (out, byte) in proof.iter_mut().zip(mac.finalize().into_bytes().iter()) {
        *out = *byte;
    }
    proof
}
