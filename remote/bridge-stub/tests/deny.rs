//! The scriptable `DENY` variants and the natural gates (ticket 09).

mod support;

use std::io::Read;

use bridge_stub::{Deny, Scenario, preamble::DenyReason};
use support::{BenchError, Handshake};

/// The token the tests pair with.
const TOKEN: &[u8] = b"pairing-token-01";

/// Every deny reason is scriptable, with its retry hint served verbatim.
#[test]
fn every_deny_reason_is_scriptable_with_retry_ms() -> Result<(), BenchError> {
    for (reason, retry_ms) in [
        (DenyReason::Unauthorized, Some(1_234u32)),
        (DenyReason::Busy, Some(4_000)),
        (DenyReason::SetupMode, Some(0)),
        (DenyReason::Version, None),
    ] {
        let mut scenario = Scenario::new(TOKEN);
        scenario.deny = Some(Deny { reason, retry_ms });
        let stub = support::start(&scenario)?;
        let (served_reason, served_retry) = support::denied(stub.link_addr(), TOKEN)?;
        assert_eq!(served_reason, reason, "the scripted reason is served");
        assert_eq!(served_retry, retry_ms, "the scripted retry hint is served");
    }
    Ok(())
}

/// A second connection while one authenticated client is live is denied `busy`.
#[test]
fn a_second_live_client_is_denied_busy() -> Result<(), BenchError> {
    let stub = support::start(&Scenario::new(TOKEN))?;
    let (first, _identity) = support::accepted(stub.link_addr(), TOKEN)?;

    let (reason, retry_ms) = support::denied(stub.link_addr(), TOKEN)?;
    assert_eq!(reason, DenyReason::Busy);
    assert_eq!(retry_ms, Some(4_000), "a busy denial retries after ADR-0039's default");

    drop(first);
    Ok(())
}

/// A wrong preamble major is denied `version`: the framing is frozen, the major is policy.
#[test]
fn a_wrong_preamble_major_is_denied_version() -> Result<(), BenchError> {
    let stub = support::start(&Scenario::new(TOKEN))?;
    let handshake = support::Client::handshake_with_major(stub.link_addr(), TOKEN, 0)?;
    let Handshake::Denied { reason, retry_ms } = handshake else {
        assert!(matches!(handshake, Handshake::Denied { .. }), "a wrong major is denied");
        return Ok(());
    };
    assert_eq!(reason, DenyReason::Version);
    assert_eq!(retry_ms, None);
    Ok(())
}

/// A scripted denial wins over a correct client's proof, whatever the reason.
#[test]
fn a_scripted_denial_wins_over_a_correct_proof() -> Result<(), BenchError> {
    let mut scenario = Scenario::new(TOKEN);
    scenario.deny = Some(Deny {
        reason: DenyReason::Busy,
        retry_ms: Some(250),
    });
    let stub = support::start(&scenario)?;
    let (reason, retry_ms) = support::denied(stub.link_addr(), TOKEN)?;
    assert_eq!(reason, DenyReason::Busy);
    assert_eq!(retry_ms, Some(250));
    Ok(())
}

/// A denial is served before the challenge, so a denied client never sees a Link frame.
#[test]
fn a_denied_client_never_receives_link_frames() -> Result<(), BenchError> {
    let mut scenario = Scenario::new(TOKEN);
    scenario.deny = Some(Deny {
        reason: DenyReason::SetupMode,
        retry_ms: None,
    });
    let stub = support::start(&scenario)?;
    let mut stream = support::connect(stub.link_addr())?;
    let nonce = [3u8; bridge_stub::preamble::NONCE_LEN];
    bridge_stub::preamble::write_message(
        &mut stream,
        bridge_net::preamble::MAJOR,
        &bridge_stub::preamble::Message::ClientHello { nonce },
    )
    .map_err(|error| BenchError::Io(error.to_string()))?;

    // The denial is the only answer; the stream then closes and no Link frame ever crosses.
    let received =
        bridge_stub::preamble::read_message(&mut stream).map_err(|error| BenchError::Io(error.to_string()))?;
    assert!(matches!(received.message, bridge_stub::preamble::Message::Deny { .. }));
    let mut buffer = [0u8; 64];
    let read = stream.read(&mut buffer);
    assert!(matches!(read, Ok(0)), "the connection closes after the denial");
    Ok(())
}
