//! The far end of the serial seam, shared by the Robot Stub's integration tests.
//!
//! The peer owns one end of an in-memory duplex channel and plays everything the Bridge's UART
//! sees on the robot's bench: it parses robot-to-PC bytes into frames, records them with the
//! simulated time they arrived, and speaks `HELLO`, `PING`, commands, event acknowledgements and
//! `BRIDGE_CONTROL` acknowledgements. It is test code only; the stub never depends on it.

// Each integration test compiles this module separately and uses a different subset of the
// peer's API, so unused-item warnings here are per-binary noise.
#![allow(dead_code)]

use std::vec::Vec;

use link_protocol::{
    Caps, Class, Command, CommandMessage, Control, Direction, Event, EventMessage, Flags, Frame, MAX_PC_TO_ROBOT_FRAME,
    PROTOCOL_MAJOR, PROTOCOL_MINOR, Seq, Status, encode_bridge_control_ack, encode_command, encode_event, encode_hello,
    parser::RobotToPcParser,
    payload::{command::DriveDistance, control::RateEntry, event::EventAck},
};
use robot_stub::{Duplex, Transport};

/// ADR-0033's baud; the peer's parser derives the same 50 ms assembly timeout from it.
const BAUD: u32 = 921_600;

/// The simulated clock's cadence: the stub steps and the peer pumps every 5 ms.
const CADENCE_MS: u64 = 5;

/// The largest chunk one [`Peer::pump`] reads from the seam.
const READ_CHUNK: usize = 4096;

/// One frame observed at the seam, with the simulated millisecond it arrived.
#[derive(Clone, Debug)]
pub struct Observed {
    /// The frame's delivery class.
    pub class: Class,
    /// The class-specific id.
    pub id: u16,
    /// The frame's sequence number.
    pub seq: u16,
    /// The frame's flags.
    pub flags: Flags,
    /// The frame's payload bytes.
    pub payload: Vec<u8>,
    /// The simulated millisecond at which the frame arrived.
    pub at_ms: u64,
}

/// The bench peer: the far end of the serial seam.
pub struct Peer {
    /// The peer's end of the duplex channel.
    port: Duplex,
    /// The robot-to-PC parser, so frames are decoded exactly as the daemon would.
    parser: RobotToPcParser,
    /// Every frame decoded so far, oldest first.
    frames: Vec<Observed>,
    /// The raw bytes read from the seam so far.
    bytes_read: usize,
    /// The peer's own control sequence counter.
    control_seq: u16,
}

impl Peer {
    /// Creates a peer holding `port`, the far end of the seam.
    pub fn new(port: Duplex) -> Self {
        Self {
            port,
            parser: RobotToPcParser::new(Direction::RobotToPc, BAUD),
            frames: Vec::new(),
            bytes_read: 0,
            control_seq: 0,
        }
    }

    /// Reads every byte that has crossed the seam, parsing and recording whole frames.
    ///
    /// Returns the number of raw bytes read this call; corrupted or partial bytes stay in the
    /// parser, exactly as they would on the Bridge.
    pub fn pump(&mut self, now_ms: u64) -> usize {
        let mut chunk = [0u8; READ_CHUNK];
        let mut read = 0;
        loop {
            let count = self.port.read(&mut chunk);
            if count == 0 {
                break;
            }
            read += count;
            let now = u32::try_from(now_ms).unwrap_or(u32::MAX);
            let frames = &mut self.frames;
            self.parser.feed(&chunk[..count], now, |frame| {
                frames.push(Observed {
                    class: frame.header.class,
                    id: frame.header.id,
                    seq: frame.header.seq.get(),
                    flags: frame.header.flags,
                    payload: frame.payload.to_vec(),
                    at_ms: now_ms,
                });
            });
            if count < chunk.len() {
                break;
            }
        }
        self.bytes_read += read;
        read
    }

    /// Returns every frame recorded so far, oldest first.
    pub fn frames(&self) -> &[Observed] {
        &self.frames
    }

    /// Returns the frames of one class and id that arrived inside the time window, oldest first.
    ///
    /// The window is `after < at_ms <= until`; [`None`] means no lower bound, so frames that
    /// arrived at millisecond zero are included.
    pub fn frames_of(&self, class: Class, id: u16, after_ms: Option<u64>, until_ms: u64) -> Vec<&Observed> {
        self.frames
            .iter()
            .filter(|frame| {
                frame.class == class
                    && frame.id == id
                    && after_ms.is_none_or(|after| frame.at_ms > after)
                    && frame.at_ms <= until_ms
            })
            .collect()
    }

    /// Returns the raw bytes read from the seam so far.
    pub const fn bytes_read(&self) -> usize {
        self.bytes_read
    }

    /// Sends one raw frame onto the seam.
    fn send(&mut self, class: Class, flags: Flags, id: u16, seq: u16, payload: &[u8]) {
        let mut buffer = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = Frame::new(PROTOCOL_MAJOR, class, flags, id, Seq::new(seq), payload)
            .and_then(|frame| frame.encode_into(&mut buffer))
            .unwrap_or(0);
        assert!(written > 0, "the test frame must encode");
        self.port.write(&buffer[..written]);
    }

    /// Sends one `HELLO` and returns its sequence number.
    pub fn send_hello(&mut self, epoch: u32, caps: Caps, rates: &[RateEntry]) -> u16 {
        self.control_seq = self.control_seq.wrapping_add(1);
        let seq = self.control_seq;
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = encode_hello(
            PROTOCOL_MAJOR,
            PROTOCOL_MINOR,
            caps,
            epoch,
            rates.iter().copied(),
            &mut payload,
        )
        .unwrap_or(0);
        assert!(written > 0, "the HELLO payload must encode");
        self.send(
            Class::Control,
            Flags::default(),
            Control::Hello.wire_id(),
            seq,
            &payload[..written],
        );
        seq
    }

    /// Sends one `PING` and returns its sequence number.
    pub fn send_ping(&mut self) -> u16 {
        self.control_seq = self.control_seq.wrapping_add(1);
        let seq = self.control_seq;
        self.send(Class::Control, Flags::default(), Control::Ping.wire_id(), seq, &[]);
        seq
    }

    /// Sends one `DRIVE_DISTANCE` command with `ACK_REQUESTED`.
    pub fn send_drive_distance(&mut self, seq: u16, distance_cm: i16, speed: u8) {
        let message = CommandMessage::DriveDistance(DriveDistance { distance_cm, speed });
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = encode_command(&message, &mut payload).unwrap_or(0);
        assert!(written > 0, "the DRIVE_DISTANCE payload must encode");
        self.send(
            Class::Command,
            Flags::ACK_REQUESTED,
            Command::DriveDistance.wire_id(),
            seq,
            &payload[..written],
        );
    }

    /// Acknowledges one event with `OK`, mirroring the daemon's reply.
    pub fn ack_event(&mut self, event: Event, seq: u16) {
        let message = EventMessage::Ack(EventAck { status: Status::Ok });
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = encode_event(&message, &mut payload).unwrap_or(0);
        assert!(written > 0, "the event ACK payload must encode");
        self.send(Class::Event, Flags::IS_ACK, event.wire_id(), seq, &payload[..written]);
    }

    /// Acknowledges the outstanding `BRIDGE_CONTROL` request as the Bridge would.
    pub fn ack_bridge_control(&mut self, seq: u16, reply: &link_protocol::BridgeControlReply<'_>) {
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = encode_bridge_control_ack(reply, &mut payload).unwrap_or(0);
        assert!(written > 0, "the BRIDGE_CONTROL ACK payload must encode");
        self.send(
            Class::Control,
            Flags::IS_ACK,
            Control::BridgeControl.wire_id(),
            seq,
            &payload[..written],
        );
    }
}

/// The bench: a stub and a peer on one duplex channel, driven by a simulated clock.
///
/// [`Bench::run_until`] advances the clock on the 5 ms cadence, stepping the stub and pumping the
/// peer at each tick; [`Bench::step_now`] services what the peer just sent without advancing the
/// clock, so a reply lands on a known millisecond.
pub struct Bench {
    /// The stub under test, at the robot's end of the seam.
    pub stub: robot_stub::Stub<robot_stub::Duplex>,
    /// The bench peer at the far end of the seam.
    pub peer: Peer,
    /// The current simulated millisecond.
    now_ms: u64,
}

impl Bench {
    /// Creates a bench with `config`, `boot_id`, `persona` and a fresh duplex channel.
    pub fn new(config: robot_stub::Config, boot_id: u32, persona: robot_stub::Persona) -> Self {
        let (stub_end, peer_end) = robot_stub::duplex();
        Self {
            stub: robot_stub::Stub::new(config, boot_id, persona, stub_end),
            peer: Peer::new(peer_end),
            now_ms: 0,
        }
    }

    /// Returns the current simulated millisecond.
    pub const fn at(&self) -> u64 {
        self.now_ms
    }

    /// Steps the stub and pumps the peer at the current millisecond, without advancing the clock.
    pub fn step_now(&mut self) {
        self.stub.step(self.now_ms);
        self.peer.pump(self.now_ms);
    }

    /// Advances the clock by one cadence tick, then steps the stub and pumps the peer.
    pub fn tick(&mut self) {
        self.now_ms += CADENCE_MS;
        self.step_now();
    }

    /// Runs the clock until `until_ms`, ticking on the cadence.
    pub fn run_until(&mut self, until_ms: u64) {
        while self.now_ms < until_ms {
            self.tick();
        }
    }
}
