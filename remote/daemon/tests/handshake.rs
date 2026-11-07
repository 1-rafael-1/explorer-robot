//! The daemon's preamble adapter against a fake Bridge (ADR-0037, ADR-0039).
//!
//! The seam is the byte stream, so the fake Bridge speaks the shared `bridge-net` framing and the
//! tests assert the ADR's rules directly: the proof is verified before `CLIENT_AUTH`, every denial
//! is mapped, and the deadline is enforced.

use std::time::Duration;

use bridge_net::{
    identity::{DeviceId, Flags, Identity, Name},
    preamble::{self, DenyReason, HEADER_LEN, MAX_PAYLOAD_LEN, Message, NONCE_LEN, Received},
};
use remote_daemon::handshake::{self, HandshakeError};
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

/// The token the tests pair with.
const TOKEN: &[u8] = b"pairing-token-01";

/// A bench identity the fake Bridge serves.
fn identity() -> Identity {
    Identity {
        device_id: DeviceId::new(*b"dev001"),
        name: Name::from_padded(*b"Explorer-0001\0\0\0"),
        flags: Flags::PROVISIONED | Flags::LINK_READY,
    }
}

/// Reads one complete preamble frame, failing the task on a short or malformed stream.
async fn read_message<S>(stream: &mut S) -> Result<Received, String>
where
    S: AsyncRead + Unpin,
{
    let mut buffer = [0u8; HEADER_LEN + MAX_PAYLOAD_LEN];
    let mut filled = 0usize;
    loop {
        match preamble::try_decode(&buffer[..filled]) {
            Ok(Some(received)) => return Ok(received),
            Ok(None) => {}
            Err(error) => return Err(format!("the frame did not decode: {error}")),
        }
        let count = stream
            .read(&mut buffer[filled..])
            .await
            .map_err(|error| format!("the read failed: {error}"))?;
        if count == 0 {
            return Err(String::from("the peer closed mid-preamble"));
        }
        filled += count;
    }
}

/// Writes one framed preamble message at the current major.
async fn write_message<S>(stream: &mut S, message: &Message) -> Result<(), String>
where
    S: AsyncWrite + Unpin,
{
    write_message_with_major(stream, preamble::MAJOR, message).await
}

/// Writes one framed preamble message at `major`.
async fn write_message_with_major<S>(stream: &mut S, major: u8, message: &Message) -> Result<(), String>
where
    S: AsyncWrite + Unpin,
{
    let mut buffer = [0u8; HEADER_LEN + MAX_PAYLOAD_LEN];
    let written = preamble::encode_into(major, message, &mut buffer)
        .map_err(|error| format!("the message did not encode: {error:?}"))?;
    stream
        .write_all(&buffer[..written])
        .await
        .map_err(|error| format!("the write failed: {error}"))
}

/// Serves a correct challenge for `token`, verifying the client's proof and accepting.
async fn accept_after_challenge<S>(stream: &mut S, token: &[u8]) -> Result<(), String>
where
    S: AsyncRead + AsyncWrite + Unpin,
{
    let hello = read_message(stream).await?;
    let Message::ClientHello { nonce } = hello.message else {
        return Err(String::from("expected CLIENT_HELLO"));
    };
    let bridge_nonce = [9u8; NONCE_LEN];
    let proof = preamble::bridge_proof(token, &nonce, &bridge_nonce, &identity());
    write_message(
        stream,
        &Message::BridgeChallenge {
            bridge_proof: proof,
            bridge_nonce,
            identity: identity(),
        },
    )
    .await?;

    let auth = read_message(stream).await?;
    let Message::ClientAuth { proof: served } = auth.message else {
        return Err(String::from("expected CLIENT_AUTH"));
    };
    let expected = preamble::client_proof(token, &nonce, &bridge_nonce);
    if !preamble::proofs_match(&expected, &served) {
        return Err(String::from("the client's proof did not verify"));
    }
    write_message(stream, &Message::Accept).await
}

/// A correct client verifies the Bridge and reaches `ACCEPT` with its identity.
#[tokio::test]
async fn a_correct_client_reaches_accept() {
    let (mut client, mut server) = tokio::io::duplex(4_096);
    let bridge = tokio::spawn(async move { accept_after_challenge(&mut server, TOKEN).await });

    let accepted = match handshake::run(&mut client, TOKEN, Duration::from_secs(1)).await {
        Ok(accepted) => accepted,
        Err(error) => {
            assert_eq!(format!("{error:?}").len(), usize::MAX, "the handshake failed");
            return;
        }
    };
    assert_eq!(accepted.identity, identity(), "the challenge's identity is surfaced");
    match bridge.await {
        Ok(result) => assert_eq!(result, Ok(())),
        Err(error) => assert_eq!(format!("{error}").len(), usize::MAX, "the bridge task panicked"),
    }
}

/// A bad Bridge proof aborts the exchange and the daemon sends no `CLIENT_AUTH` byte.
#[tokio::test]
async fn a_bad_bridge_proof_aborts_before_client_auth() {
    let (mut client, mut server) = tokio::io::duplex(4_096);
    let bridge = tokio::spawn(async move {
        let hello = read_message(&mut server).await?;
        let Message::ClientHello { nonce: _ } = hello.message else {
            return Err(String::from("expected CLIENT_HELLO"));
        };
        write_message(
            &mut server,
            &Message::BridgeChallenge {
                bridge_proof: [0u8; preamble::PROOF_LEN],
                bridge_nonce: [7u8; NONCE_LEN],
                identity: identity(),
            },
        )
        .await?;
        // The daemon must abort; a CLIENT_AUTH here would be the token-derived byte ADR-0037
        // forbids. It either closes the stream or leaves it silent until the timeout.
        match tokio::time::timeout(Duration::from_millis(200), read_message(&mut server)).await {
            Ok(Ok(received)) => match received.message {
                Message::ClientAuth { .. } => Err(String::from("the daemon revealed its proof")),
                _ => Ok(()),
            },
            _ => Ok(()),
        }
    });

    let result = handshake::run(&mut client, TOKEN, Duration::from_secs(1)).await;
    assert!(
        matches!(result, Err(HandshakeError::BadBridgeProof)),
        "a bad bridge proof is surfaced as such"
    );
    match bridge.await {
        Ok(result) => assert_eq!(result, Ok(())),
        Err(error) => assert_eq!(format!("{error}").len(), usize::MAX, "the bridge task panicked"),
    }
}

/// A `busy` denial carries its retry hint through to the caller.
#[tokio::test]
async fn a_busy_denial_carries_retry_ms() {
    let (mut client, mut server) = tokio::io::duplex(4_096);
    let bridge = tokio::spawn(async move {
        let _hello = read_message(&mut server).await?;
        write_message(
            &mut server,
            &Message::Deny {
                reason: DenyReason::Busy,
                retry_ms: Some(4_000),
            },
        )
        .await
    });

    match handshake::run(&mut client, TOKEN, Duration::from_secs(1)).await {
        Err(HandshakeError::Denied {
            reason: DenyReason::Busy,
            retry_ms: Some(4_000),
        }) => {}
        other => assert_eq!(format!("{other:?}").len(), usize::MAX, "expected a busy denial"),
    }
    match bridge.await {
        Ok(result) => assert_eq!(result, Ok(())),
        Err(error) => assert_eq!(format!("{error}").len(), usize::MAX, "the bridge task panicked"),
    }
}

/// A `setup_mode` denial is surfaced and not retried.
#[tokio::test]
async fn a_setup_mode_denial_is_surfaced() {
    let (mut client, mut server) = tokio::io::duplex(4_096);
    let bridge = tokio::spawn(async move {
        let _hello = read_message(&mut server).await?;
        write_message(
            &mut server,
            &Message::Deny {
                reason: DenyReason::SetupMode,
                retry_ms: None,
            },
        )
        .await
    });

    match handshake::run(&mut client, TOKEN, Duration::from_secs(1)).await {
        Err(HandshakeError::Denied {
            reason: DenyReason::SetupMode,
            retry_ms: None,
        }) => {}
        other => assert_eq!(format!("{other:?}").len(), usize::MAX, "expected a setup-mode denial"),
    }
    match bridge.await {
        Ok(result) => assert_eq!(result, Ok(())),
        Err(error) => assert_eq!(format!("{error}").len(), usize::MAX, "the bridge task panicked"),
    }
}

/// A challenge framed with the wrong major is surfaced as a version denial.
#[tokio::test]
async fn a_wrong_major_is_a_version_denial() {
    let (mut client, mut server) = tokio::io::duplex(4_096);
    let bridge = tokio::spawn(async move {
        let _hello = read_message(&mut server).await?;
        write_message_with_major(
            &mut server,
            preamble::MAJOR + 1,
            &Message::BridgeChallenge {
                bridge_proof: [0u8; preamble::PROOF_LEN],
                bridge_nonce: [7u8; NONCE_LEN],
                identity: identity(),
            },
        )
        .await
    });

    match handshake::run(&mut client, TOKEN, Duration::from_secs(1)).await {
        Err(HandshakeError::Denied {
            reason: DenyReason::Version,
            retry_ms: None,
        }) => {}
        other => assert_eq!(format!("{other:?}").len(), usize::MAX, "expected a version denial"),
    }
    match bridge.await {
        Ok(result) => assert_eq!(result, Ok(())),
        Err(error) => assert_eq!(format!("{error}").len(), usize::MAX, "the bridge task panicked"),
    }
}

/// A silent peer exhausts the deadline.
#[tokio::test]
async fn a_silent_peer_hits_the_deadline() {
    let (mut client, _server) = tokio::io::duplex(4_096);
    let result = handshake::run(&mut client, TOKEN, Duration::from_millis(80)).await;
    assert!(
        matches!(result, Err(HandshakeError::Deadline)),
        "the exchange is bounded by its deadline"
    );
}
