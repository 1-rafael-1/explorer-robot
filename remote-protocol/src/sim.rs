//! A deterministic simulated wire for host tests (ADR-0030, ADR-0033).
//!
//! The wire models a UART between the robot and the PC as two byte queues with an explicit
//! millisecond clock. Faults are deterministic for a fixed seed: a small xorshift generator
//! supplies corrupted bytes, and periodic counters drop, corrupt or truncate traffic. There is
//! no real time, no thread and no async runtime, so a test can replay a link exactly.
//!
//! This module is compiled only with the `std` feature.

use std::{collections::VecDeque, vec::Vec};

use crate::{
    PROTOCOL_MAJOR,
    codec::{CommandMessage, StreamSample, decode_command, decode_stream, encode_command},
    frame::{Class, Direction, Flags, Frame, Seq},
    ids::{Command, Stream},
    parser::{MAX_PC_TO_ROBOT_FRAME, RobotToPcParser},
};

/// The baud the simulated remote assumes; both directions derive a 50 ms timeout from it.
const SIM_BAUD: u32 = 921_600;

/// The replacement seed used when the caller supplies zero, so the generator never sticks.
const FALLBACK_SEED: u32 = 0x9E37_79B9;

/// The seed offset that gives the second direction an independent generator.
const DIRECTION_SEED_OFFSET: u32 = 0x5A5A_A5A5;

/// Fault injection settings for a [`Wire`].
///
/// Zero disables a fault. Byte and chunk counters are maintained per direction, so traffic in
/// one direction never changes the fault pattern in the other.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Faults {
    /// Drop every `drop_every`-th byte (1-based); zero disables dropping.
    pub drop_every: u32,
    /// Replace every `corrupt_every`-th byte with a pseudo-random byte; zero disables corruption.
    pub corrupt_every: u32,
    /// Drop the last byte of every `truncate_every`-th non-empty chunk; zero disables truncation.
    pub truncate_every: u32,
    /// Advance the wire clock by this many milliseconds after every non-empty send.
    pub stall_ms: u32,
}

/// A small deterministic xorshift32 generator.
#[derive(Clone, Copy, Debug)]
struct XorShift {
    /// The generator's current state.
    state: u32,
}

impl XorShift {
    /// Creates a generator, replacing a zero seed so the sequence is never stuck.
    #[must_use]
    const fn new(seed: u32) -> Self {
        Self {
            state: if seed == 0 { FALLBACK_SEED } else { seed },
        }
    }

    /// Returns the next pseudo-random `u32`.
    const fn next_u32(&mut self) -> u32 {
        let mut value = self.state;
        value ^= value << 13;
        value ^= value >> 17;
        value ^= value << 5;
        self.state = value;
        value
    }

    /// Returns the next pseudo-random byte without a narrowing cast.
    const fn next_u8(&mut self) -> u8 {
        self.next_u32().to_le_bytes()[0]
    }
}

/// Maps a direction to its queue and counter slot.
const fn slot(direction: Direction) -> usize {
    match direction {
        Direction::RobotToPc => 0,
        Direction::PcToRobot => 1,
    }
}

/// A deterministic byte pipe between the robot and the PC.
///
/// A caller sends bytes in a [`Direction`] and drains them with [`Wire::take`]. The clock only
/// moves when [`Wire::advance`] is called or a send applies [`Faults::stall_ms`], so the wire
/// never reads the wall clock.
#[derive(Debug)]
pub struct Wire {
    /// The simulated millisecond clock.
    now_ms: u32,
    /// The queued bytes in each direction; index with [`slot`].
    queues: [VecDeque<u8>; 2],
    /// The number of bytes sent in each direction, used by the periodic byte faults.
    byte_counts: [u64; 2],
    /// The number of chunks sent in each direction, used by truncation.
    chunk_counts: [u64; 2],
    /// One independent generator per direction so faults do not couple.
    rng: [XorShift; 2],
    /// The active fault settings.
    faults: Faults,
}

impl Wire {
    /// Creates a wire with an explicit seed and fault settings.
    ///
    /// The same seed and the same sequence of calls always produce the same bytes and clock.
    #[must_use]
    pub const fn new(seed: u32, faults: Faults) -> Self {
        Self {
            now_ms: 0,
            queues: [VecDeque::new(), VecDeque::new()],
            byte_counts: [0; 2],
            chunk_counts: [0; 2],
            rng: [XorShift::new(seed), XorShift::new(seed ^ DIRECTION_SEED_OFFSET)],
            faults,
        }
    }

    /// Returns the simulated clock in milliseconds.
    #[must_use]
    pub const fn now_ms(&self) -> u32 {
        self.now_ms
    }

    /// Advances the simulated clock by `ms` milliseconds.
    pub const fn advance(&mut self, ms: u32) {
        self.now_ms = self.now_ms.wrapping_add(ms);
    }

    /// Returns the active fault settings.
    #[must_use]
    pub const fn faults(&self) -> Faults {
        self.faults
    }

    /// Replaces the fault settings; bytes already queued are not changed.
    pub const fn set_faults(&mut self, faults: Faults) {
        self.faults = faults;
    }

    /// Sends `bytes` in `direction`, applying every active fault.
    ///
    /// Bytes are counted across sends and per direction. A chunk is truncated before its bytes
    /// are dropped or corrupted, and a non-empty chunk advances the clock by
    /// [`Faults::stall_ms`].
    pub fn send(&mut self, direction: Direction, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }

        let index = slot(direction);
        self.chunk_counts[index] = self.chunk_counts[index].wrapping_add(1);
        let chunk = self.chunk_counts[index];
        let truncate = self.faults.truncate_every != 0 && chunk.is_multiple_of(u64::from(self.faults.truncate_every));
        let end = if truncate {
            bytes.len().saturating_sub(1)
        } else {
            bytes.len()
        };

        for &original in &bytes[..end] {
            self.byte_counts[index] = self.byte_counts[index].wrapping_add(1);
            let count = self.byte_counts[index];
            if self.faults.drop_every != 0 && count.is_multiple_of(u64::from(self.faults.drop_every)) {
                continue;
            }
            let corrupt = self.faults.corrupt_every != 0 && count.is_multiple_of(u64::from(self.faults.corrupt_every));
            let byte = if corrupt { self.rng[index].next_u8() } else { original };
            self.queues[index].push_back(byte);
        }

        self.now_ms = self.now_ms.wrapping_add(self.faults.stall_ms);
    }

    /// Returns how many bytes are queued in `direction`.
    #[must_use]
    pub fn available(&self, direction: Direction) -> usize {
        self.queues[slot(direction)].len()
    }

    /// Drains and returns every byte queued in `direction`.
    #[must_use]
    pub fn take(&mut self, direction: Direction) -> Vec<u8> {
        self.queues[slot(direction)].drain(..).collect()
    }
}

/// A daemon-shaped peer that decodes robot-to-PC telemetry from, and sends commands to, a
/// [`Wire`].
///
/// Feed the wire's robot-to-PC bytes through the real [`RobotToPcParser`] and read back the typed
/// samples and command ACKs. [`FakeRemote::send_command`] encodes a request with the real codecs
/// and the next sequence number, so a test exercises the same bytes the daemon would put on the
/// wire. Later tickets extend the same type with `HELLO`/`PING`, events and `THROTTLE`.
pub struct FakeRemote {
    /// The real robot-to-PC parser, so the simulated path exercises the committed state machine.
    parser: RobotToPcParser,
    /// Received stream frames in arrival order, each payload copied out of the parser.
    received: Vec<(Stream, Seq, Vec<u8>)>,
    /// Received command ACKs in arrival order, each payload copied out of the parser.
    received_acks: Vec<(Command, Seq, Vec<u8>)>,
    /// Unknown stream ids seen so far; they are ignored, never decoded.
    unknown_streams: u32,
    /// Unknown command ids seen in ACKs; they are ignored, never decoded.
    unknown_acks: u32,
    /// The next command sequence; commands start at 1 and zero is never sent.
    command_seq: u16,
}

impl FakeRemote {
    /// Creates a remote with an empty receive state.
    #[must_use]
    pub fn new() -> Self {
        Self {
            parser: RobotToPcParser::new(Direction::RobotToPc, SIM_BAUD),
            received: Vec::new(),
            received_acks: Vec::new(),
            unknown_streams: 0,
            unknown_acks: 0,
            command_seq: 1,
        }
    }

    /// Drains the wire's robot-to-PC queue and returns how many stream frames it decoded.
    pub fn receive(&mut self, wire: &mut Wire) -> usize {
        let now_ms = wire.now_ms();
        let bytes = wire.take(Direction::RobotToPc);
        self.feed(&bytes, now_ms)
    }

    /// Feeds raw robot-to-PC bytes through the real parser and returns how many stream frames
    /// completed.
    ///
    /// Unknown stream ids are counted and ignored. Command frames carrying `IS_ACK` are copied
    /// into the ACK log; every other non-stream class is ignored until a later ticket implements
    /// it.
    pub fn feed(&mut self, bytes: &[u8], now_ms: u32) -> usize {
        let before = self.received.len();
        let received = &mut self.received;
        let received_acks = &mut self.received_acks;
        let mut unknown = 0u32;
        let mut unknown_acks = 0u32;
        self.parser.feed(bytes, now_ms, |frame| match frame.header.class {
            Class::Stream => {
                let Some(stream) = Stream::from_wire(frame.header.id) else {
                    unknown = unknown.saturating_add(1);
                    return;
                };
                received.push((stream, frame.header.seq, frame.payload.to_vec()));
            }
            Class::Command => {
                if !frame.header.flags.contains(Flags::IS_ACK) {
                    return;
                }
                let Some(command) = Command::from_wire(frame.header.id) else {
                    unknown_acks = unknown_acks.saturating_add(1);
                    return;
                };
                received_acks.push((command, frame.header.seq, frame.payload.to_vec()));
            }
            Class::Event | Class::Control => {}
        });
        self.unknown_streams = self.unknown_streams.saturating_add(unknown);
        self.unknown_acks = self.unknown_acks.saturating_add(unknown_acks);
        self.received.len() - before
    }

    /// Encodes `command` with `payload` and sends it to the robot with `ACK_REQUESTED`.
    ///
    /// The sequence number starts at 1 and advances on every send, so a test never has to invent
    /// one. Returns the sequence used, or [`None`] if the payload does not fit the frame.
    pub fn send_command(&mut self, wire: &mut Wire, command: Command, payload: &[u8]) -> Option<Seq> {
        let seq = Seq::new(self.command_seq);
        if !self.send_command_frame(wire, command.wire_id(), seq, Flags::ACK_REQUESTED, payload) {
            return None;
        }
        let next = self.command_seq.wrapping_add(1);
        self.command_seq = if next == 0 { 1 } else { next };
        Some(seq)
    }

    /// Encodes a typed request message and sends it with the next sequence number.
    ///
    /// The generic [`CommandMessage::Ack`] variant carries no id and is rejected.
    pub fn send_command_message(&mut self, wire: &mut Wire, message: &CommandMessage<'_>) -> Option<Seq> {
        let command = message.command()?;
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = encode_command(message, &mut payload).ok()?;
        self.send_command(wire, command, &payload[..written])
    }

    /// Sends one command frame with an explicit id, sequence and flags.
    ///
    /// This is the escape hatch for duplicate sequences, unknown ids and malformed flag
    /// combinations; it does not touch the automatic sequence counter. Returns `false` when the
    /// frame cannot be encoded.
    pub fn send_command_frame(&mut self, wire: &mut Wire, id: u16, seq: Seq, flags: Flags, payload: &[u8]) -> bool {
        let Ok(frame) = Frame::new(PROTOCOL_MAJOR, Class::Command, flags, id, seq, payload) else {
            return false;
        };
        let mut bytes = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let Ok(written) = frame.encode_into(&mut bytes) else {
            return false;
        };
        wire.send(Direction::PcToRobot, &bytes[..written]);
        true
    }

    /// Returns every received stream frame in arrival order with its decoded sample.
    ///
    /// A frame whose payload the codec rejects is skipped; the codec tests cover malformed input
    /// directly, and a valid sender never produces one.
    pub fn streams(&self) -> impl Iterator<Item = (Stream, Seq, StreamSample<'_>)> + '_ {
        self.received.iter().filter_map(|(stream, seq, payload)| {
            decode_stream(*stream, payload)
                .ok()
                .map(|sample| (*stream, *seq, sample))
        })
    }

    /// Returns the decoded samples only, in arrival order.
    pub fn samples(&self) -> impl Iterator<Item = StreamSample<'_>> + '_ {
        self.streams().map(|(_, _, sample)| sample)
    }

    /// Returns the most recently received sample of `stream`, if any survived decoding.
    #[must_use]
    pub fn last_sample(&self, stream: Stream) -> Option<StreamSample<'_>> {
        self.received
            .iter()
            .rev()
            .find(|(received_stream, _, _)| *received_stream == stream)
            .and_then(|(received_stream, _, payload)| decode_stream(*received_stream, payload).ok())
    }

    /// Returns how many stream frames have been received.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.received.len()
    }

    /// Returns `true` when no stream frame has been received.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.received.is_empty()
    }

    /// Returns how many unknown stream ids have been ignored.
    #[must_use]
    pub const fn unknown_streams(&self) -> u32 {
        self.unknown_streams
    }

    /// Returns every received command ACK in arrival order with its decoded message.
    ///
    /// An ACK whose payload the codec rejects is skipped, mirroring [`FakeRemote::streams`].
    pub fn acks(&self) -> impl Iterator<Item = (Command, Seq, CommandMessage<'_>)> + '_ {
        self.received_acks.iter().filter_map(|(command, seq, payload)| {
            decode_command(*command, true, payload)
                .ok()
                .map(|message| (*command, *seq, message))
        })
    }

    /// Returns the most recently received command ACK, if any survived decoding.
    #[must_use]
    pub fn last_ack(&self) -> Option<(Command, Seq, CommandMessage<'_>)> {
        self.acks().last()
    }

    /// Returns the status of the most recent generic ACK, if there is one.
    ///
    /// The two query responses carry a typed payload instead of a status byte and return [`None`].
    #[must_use]
    pub fn last_ack_status(&self) -> Option<crate::frame::Status> {
        match self.last_ack()?.2 {
            CommandMessage::Ack(ack) => Some(ack.status),
            CommandMessage::StartProcedure(_)
            | CommandMessage::StopProcedure(_)
            | CommandMessage::DriveDistance(_)
            | CommandMessage::RotateBy(_)
            | CommandMessage::StopMotion(_)
            | CommandMessage::GetCatalogue(_)
            | CommandMessage::GetStatus(_)
            | CommandMessage::CataloguePage(_)
            | CommandMessage::StatusSnapshot(_) => None,
        }
    }

    /// Returns how many command ACKs have been received.
    #[must_use]
    pub const fn ack_count(&self) -> usize {
        self.received_acks.len()
    }

    /// Returns how many unknown command ids have been ignored in ACKs.
    #[must_use]
    pub const fn unknown_acks(&self) -> u32 {
        self.unknown_acks
    }

    /// Returns the next command sequence number the remote will use.
    #[must_use]
    pub const fn next_command_seq(&self) -> u16 {
        self.command_seq
    }
}

impl Default for FakeRemote {
    fn default() -> Self {
        Self::new()
    }
}
