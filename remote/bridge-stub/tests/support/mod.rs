//! The far end of the TCP/preamble seam, shared by the Bridge Stub's integration tests.
//!
//! The peer owns the daemon's side of the exchange: it completes the preamble with the stub's own
//! codec (both sides compose `bridge-net`'s constants, so a divergence is a compile error), then
//! speaks `HELLO`, `PING`, commands and event acknowledgements, recording every robot-to-PC frame
//! it decodes exactly as the daemon would. It is test code only; the stub never depends on it.

// Each integration test compiles this module separately and uses a different subset of the
// peer's API, so unused-item warnings here are per-binary noise.
#![allow(dead_code)]

use std::{
    io::{Read, Write},
    net::{SocketAddr, TcpStream},
    time::{Duration, Instant},
};

use bridge_stub::{
    Identity, Scenario, Stub,
    preamble::{DenyReason, Message, NONCE_LEN, bridge_proof, client_proof, proofs_match, read_message, write_message},
};
use link_protocol::{
    Caps, Class, Command, CommandMessage, Control, ControlMessage, Direction, Event, EventMessage, Flags as LinkFlags,
    Frame, MAX_PC_TO_ROBOT_FRAME, PROTOCOL_MAJOR, PROTOCOL_MINOR, Seq, Status, Stream, decode_command, decode_control,
    decode_stream, encode_command, encode_event, encode_hello,
    parser::RobotToPcParser,
    payload::{
        command::{AckPayload, CataloguePage, DriveDistance, StatusSnapshot},
        control::RateEntry,
        event::EventAck,
        stream::Motion,
    },
};

/// ADR-0033's baud; the parser derives the same 50 ms assembly timeout from it.
const BAUD: u32 = 921_600;

/// The largest chunk one pump reads from the seam.
const READ_CHUNK: usize = 4096;

/// The pump's read slice.
const PUMP_SLICE: Duration = Duration::from_millis(10);

/// One Link frame observed at the seam.
#[derive(Clone, Debug)]
pub struct Observed {
    /// The frame's delivery class.
    pub class: Class,
    /// The class-specific id.
    pub id: u16,
    /// The sequence number.
    pub seq: u16,
    /// The frame's flags.
    pub flags: LinkFlags,
    /// The payload bytes.
    pub payload: Vec<u8>,
}

/// Why a bench action failed.
#[derive(Debug)]
pub enum BenchError {
    /// The peer could not connect.
    Connect(String),
    /// A socket or codec error, with its context.
    Io(String),
    /// The exchange took an unexpected turn.
    Unexpected(String),
    /// The Bridge's proof did not verify against the token.
    BadBridgeProof,
}

impl std::fmt::Display for BenchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Connect(error) => write!(f, "connect failed: {error}"),
            Self::Io(error) => write!(f, "socket error: {error}"),
            Self::Unexpected(note) => write!(f, "unexpected exchange: {note}"),
            Self::BadBridgeProof => write!(f, "the bridge proof did not verify"),
        }
    }
}

impl std::error::Error for BenchError {}

/// The handshake's outcome.
pub enum Handshake {
    /// The preamble completed; the client holds the Link stream.
    Accepted {
        /// The connected client.
        client: Box<Client>,
        /// The identity the challenge served.
        identity: Identity,
    },
    /// The Bridge denied the preamble.
    Denied {
        /// The deny reason.
        reason: DenyReason,
        /// The retry hint, when the Bridge sent one.
        retry_ms: Option<u32>,
    },
}

/// The daemon's side of the seam.
pub struct Client {
    /// The Link stream.
    stream: TcpStream,
    /// The robot-to-PC parser, so frames are decoded exactly as the daemon would.
    parser: RobotToPcParser,
    /// Every frame decoded so far, oldest first.
    frames: Vec<Observed>,
    /// The peer's control sequence counter.
    control_seq: u16,
}

impl Client {
    /// Connects and completes the preamble with `token` at the current preamble major.
    pub fn handshake(addr: SocketAddr, token: &[u8]) -> Result<Handshake, BenchError> {
        Self::handshake_with_major(addr, token, bridge_net::preamble::MAJOR)
    }

    /// Connects and completes the preamble with `token`, framing `CLIENT_HELLO` with `major`.
    pub fn handshake_with_major(addr: SocketAddr, token: &[u8], major: u8) -> Result<Handshake, BenchError> {
        Self::exchange(addr, token, major, true)
    }

    /// Connects and sends the client proof without verifying the bridge proof first.
    ///
    /// A daemon aborts when the bridge proof does not verify (ADR-0037's rule); this variant plays
    /// a client that presents its proof regardless, so the server's verdict on a wrong token is
    /// observable.
    pub fn handshake_trusting(addr: SocketAddr, token: &[u8]) -> Result<Handshake, BenchError> {
        Self::exchange(addr, token, bridge_net::preamble::MAJOR, false)
    }

    /// The preamble exchange; `verify` decides whether a bad bridge proof ends it.
    fn exchange(addr: SocketAddr, token: &[u8], major: u8, verify: bool) -> Result<Handshake, BenchError> {
        let mut stream = connect(addr)?;
        let nonce = client_nonce();
        write_hello(&mut stream, major, &nonce)?;
        let received = read_message(&mut stream).map_err(|error| BenchError::Io(error.to_string()))?;
        let Message::BridgeChallenge {
            bridge_proof: served_proof,
            bridge_nonce,
            identity,
        } = received.message
        else {
            return match received.message {
                Message::Deny { reason, retry_ms } => Ok(Handshake::Denied { reason, retry_ms }),
                message => Err(BenchError::Unexpected(format!(
                    "expected a challenge, got kind {}",
                    message.kind().bits()
                ))),
            };
        };
        let expected = bridge_proof(token, &nonce, &bridge_nonce, &identity);
        if verify && !proofs_match(&expected, &served_proof) {
            return Err(BenchError::BadBridgeProof);
        }
        let proof = client_proof(token, &nonce, &bridge_nonce);
        write_message(&mut stream, major, &Message::ClientAuth { proof })
            .map_err(|error| BenchError::Io(error.to_string()))?;
        let received = read_message(&mut stream).map_err(|error| BenchError::Io(error.to_string()))?;
        match received.message {
            Message::Accept => Ok(Handshake::Accepted {
                client: Box::new(Self {
                    parser: RobotToPcParser::new(Direction::RobotToPc, BAUD),
                    frames: Vec::new(),
                    control_seq: 0,
                    stream,
                }),
                identity,
            }),
            Message::Deny { reason, retry_ms } => Ok(Handshake::Denied { reason, retry_ms }),
            message => Err(BenchError::Unexpected(format!(
                "expected ACCEPT or DENY, got kind {}",
                message.kind().bits()
            ))),
        }
    }

    /// Sends one `HELLO` with `epoch` and a 1 Hz motion rate, returning its sequence.
    pub fn send_hello(&mut self, epoch: u32) -> Result<u16, BenchError> {
        self.control_seq = self.control_seq.wrapping_add(1);
        let seq = self.control_seq;
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let entries = [RateEntry::enabled(Stream::Motion.wire_id(), 100)];
        let written = encode_hello(
            PROTOCOL_MAJOR,
            PROTOCOL_MINOR,
            Caps::from_bits(0),
            epoch,
            entries,
            &mut payload,
        )
        .unwrap_or(0);
        assert!(written > 0, "the HELLO payload must encode");
        self.send(
            Class::Control,
            LinkFlags::default(),
            Control::Hello.wire_id(),
            seq,
            &payload[..written],
        )?;
        Ok(seq)
    }

    /// Sends one `PING`, returning its sequence.
    pub fn send_ping(&mut self) -> Result<u16, BenchError> {
        self.control_seq = self.control_seq.wrapping_add(1);
        let seq = self.control_seq;
        self.send(Class::Control, LinkFlags::default(), Control::Ping.wire_id(), seq, &[])?;
        Ok(seq)
    }

    /// Sends one `DRIVE_DISTANCE` command with `ACK_REQUESTED`.
    pub fn send_drive_distance(&mut self, seq: u16, distance_cm: i16, speed: u8) -> Result<(), BenchError> {
        let message = CommandMessage::DriveDistance(DriveDistance { distance_cm, speed });
        self.send_command_message(Command::DriveDistance, seq, &message)
    }

    /// Sends one `STOP_MOTION` command with `ACK_REQUESTED`.
    pub fn send_stop_motion(&mut self, seq: u16, brake: u8) -> Result<(), BenchError> {
        let message = CommandMessage::StopMotion(link_protocol::payload::command::StopMotion { brake });
        self.send_command_message(Command::StopMotion, seq, &message)
    }

    /// Sends one `GET_STATUS` command with `ACK_REQUESTED`.
    pub fn send_get_status(&mut self, seq: u16) -> Result<(), BenchError> {
        let message = CommandMessage::GetStatus(link_protocol::payload::command::GetStatus);
        self.send_command_message(Command::GetStatus, seq, &message)
    }

    /// Encodes and sends one typed command message with `ACK_REQUESTED`.
    fn send_command_message(
        &mut self,
        command: Command,
        seq: u16,
        message: &CommandMessage<'_>,
    ) -> Result<(), BenchError> {
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = match encode_command(message, &mut payload) {
            Ok(written) => written,
            Err(error) => {
                return Err(BenchError::Unexpected(format!(
                    "the command payload did not encode: {error:?}"
                )));
            }
        };
        self.send(
            Class::Command,
            LinkFlags::ACK_REQUESTED,
            command.wire_id(),
            seq,
            &payload[..written],
        )
    }

    /// Sends one raw command payload with `ACK_REQUESTED`.
    pub fn send_command(&mut self, command: Command, seq: u16, payload: &[u8]) -> Result<(), BenchError> {
        self.send(
            Class::Command,
            LinkFlags::ACK_REQUESTED,
            command.wire_id(),
            seq,
            payload,
        )
    }

    /// Acknowledges one event, mirroring the daemon's reply.
    pub fn ack_event(&mut self, event: Event, seq: u16) -> Result<(), BenchError> {
        let message = EventMessage::Ack(EventAck { status: Status::Ok });
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = encode_event(&message, &mut payload).unwrap_or(0);
        assert!(written > 0, "the event ACK payload must encode");
        self.send(
            Class::Event,
            LinkFlags::IS_ACK,
            event.wire_id(),
            seq,
            &payload[..written],
        )
    }

    /// Reads the seam for `window`, recording every frame that crosses.
    pub fn pump(&mut self, window: Duration) -> Result<(), BenchError> {
        self.stream
            .set_read_timeout(Some(PUMP_SLICE))
            .map_err(|error| BenchError::Io(error.to_string()))?;
        let start = Instant::now();
        let mut chunk = [0u8; READ_CHUNK];
        while start.elapsed() < window {
            match self.stream.read(&mut chunk) {
                Ok(0) => return Ok(()),
                Ok(count) => {
                    let stamp = u32::try_from(start.elapsed().as_millis()).unwrap_or(u32::MAX);
                    let frames = &mut self.frames;
                    self.parser.feed(&chunk[..count], stamp, |frame| {
                        frames.push(Observed {
                            class: frame.header.class,
                            id: frame.header.id,
                            seq: frame.header.seq.get(),
                            flags: frame.header.flags,
                            payload: frame.payload.to_vec(),
                        });
                    });
                }
                Err(error)
                    if matches!(
                        error.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut | std::io::ErrorKind::Interrupted
                    ) => {}
                Err(error) => return Err(BenchError::Io(error.to_string())),
            }
        }
        Ok(())
    }

    /// Returns the frames of one class and id, oldest first.
    #[must_use]
    pub fn frames_of(&self, class: Class, id: u16) -> Vec<Observed> {
        self.frames
            .iter()
            .filter(|frame| frame.class == class && frame.id == id)
            .cloned()
            .collect()
    }

    /// Sends one raw Link frame onto the stream.
    fn send(&mut self, class: Class, flags: LinkFlags, id: u16, seq: u16, payload: &[u8]) -> Result<(), BenchError> {
        let mut buffer = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = Frame::new(PROTOCOL_MAJOR, class, flags, id, Seq::new(seq), payload)
            .and_then(|frame| frame.encode_into(&mut buffer))
            .unwrap_or(0);
        assert!(written > 0, "the test frame must encode");
        self.stream
            .write_all(&buffer[..written])
            .map_err(|error| BenchError::Io(error.to_string()))
    }
}

/// Connects a raw socket to the stub's listener.
pub fn connect(addr: SocketAddr) -> Result<TcpStream, BenchError> {
    TcpStream::connect(addr).map_err(|error| BenchError::Connect(error.to_string()))
}

/// Starts a stub, mapping bind errors onto the bench error.
pub fn start(scenario: &Scenario) -> Result<Stub, BenchError> {
    Stub::start(scenario.clone()).map_err(|error| BenchError::Io(error.to_string()))
}

/// Completes the handshake and returns the client, or fails the test on any other outcome.
pub fn accepted(addr: SocketAddr, token: &[u8]) -> Result<(Client, Identity), BenchError> {
    match Client::handshake(addr, token)? {
        Handshake::Accepted { client, identity } => Ok((*client, identity)),
        Handshake::Denied { reason, retry_ms } => Err(BenchError::Unexpected(format!(
            "expected ACCEPT, got DENY {reason:?} (retry {retry_ms:?})"
        ))),
    }
}

/// Completes the handshake expecting a denial, and returns the reason and retry hint.
pub fn denied(addr: SocketAddr, token: &[u8]) -> Result<(DenyReason, Option<u32>), BenchError> {
    match Client::handshake(addr, token)? {
        Handshake::Denied { reason, retry_ms } => Ok((reason, retry_ms)),
        Handshake::Accepted { .. } => Err(BenchError::Unexpected(String::from("expected DENY, got ACCEPT"))),
    }
}

/// Decodes a `HELLO_ACK` payload.
pub fn hello_ack(payload: &[u8]) -> Result<link_protocol::HelloAck<'_>, BenchError> {
    match decode_control(Control::Hello, true, payload) {
        Ok(ControlMessage::HelloAck(ack)) => Ok(ack),
        Ok(_) => Err(BenchError::Unexpected(String::from("the reply is not a HELLO_ACK"))),
        Err(error) => Err(BenchError::Unexpected(format!(
            "the HELLO_ACK did not decode: {error:?}"
        ))),
    }
}

/// Decodes a `PING_ACK` payload.
pub fn ping_ack(payload: &[u8]) -> Result<link_protocol::PingAck, BenchError> {
    match decode_control(Control::Ping, true, payload) {
        Ok(ControlMessage::PingAck(ack)) => Ok(ack),
        Ok(_) => Err(BenchError::Unexpected(String::from("the reply is not a PING_ACK"))),
        Err(error) => Err(BenchError::Unexpected(format!(
            "the PING_ACK did not decode: {error:?}"
        ))),
    }
}

/// Decodes a generic command ACK's status.
pub fn ack_status(command: Command, payload: &[u8]) -> Result<Status, BenchError> {
    match decode_command(command, true, payload) {
        Ok(CommandMessage::Ack(AckPayload { status, .. })) => Ok(status),
        Ok(_) => Err(BenchError::Unexpected(String::from("the reply is not a generic ACK"))),
        Err(error) => Err(BenchError::Unexpected(format!(
            "the command ACK did not decode: {error:?}"
        ))),
    }
}

/// Decodes a `GET_STATUS` response.
pub fn status_snapshot(payload: &[u8]) -> Result<StatusSnapshot<'_>, BenchError> {
    match decode_command(Command::GetStatus, true, payload) {
        Ok(CommandMessage::StatusSnapshot(snapshot)) => Ok(snapshot),
        Ok(_) => Err(BenchError::Unexpected(String::from(
            "the reply is not a status snapshot",
        ))),
        Err(error) => Err(BenchError::Unexpected(format!(
            "the status snapshot did not decode: {error:?}"
        ))),
    }
}

/// Decodes a `GET_CATALOGUE` response.
pub fn catalogue_page(payload: &[u8]) -> Result<CataloguePage<'_>, BenchError> {
    match decode_command(Command::GetCatalogue, true, payload) {
        Ok(CommandMessage::CataloguePage(page)) => Ok(page),
        Ok(_) => Err(BenchError::Unexpected(String::from(
            "the reply is not a catalogue page",
        ))),
        Err(error) => Err(BenchError::Unexpected(format!(
            "the catalogue page did not decode: {error:?}"
        ))),
    }
}

/// Decodes a `MOTION` sample.
pub fn motion_sample(payload: &[u8]) -> Result<Motion, BenchError> {
    match decode_stream(Stream::Motion, payload) {
        Ok(link_protocol::StreamSample::Motion(sample)) => Ok(sample),
        Ok(_) => Err(BenchError::Unexpected(String::from(
            "the payload is not a motion sample",
        ))),
        Err(error) => Err(BenchError::Unexpected(format!(
            "the motion sample did not decode: {error:?}"
        ))),
    }
}

/// The daemon's nonce: a fixed pattern, so transcripts are reproducible.
fn client_nonce() -> [u8; NONCE_LEN] {
    let mut nonce = [0u8; NONCE_LEN];
    for (index, byte) in nonce.iter_mut().enumerate() {
        *byte = u8::try_from(index).unwrap_or(u8::MAX);
    }
    nonce
}

/// Sends `CLIENT_HELLO` framed with `major`.
fn write_hello(stream: &mut TcpStream, major: u8, nonce: &[u8; NONCE_LEN]) -> Result<(), BenchError> {
    write_message(stream, major, &Message::ClientHello { nonce: *nonce })
        .map_err(|error| BenchError::Io(error.to_string()))
}
