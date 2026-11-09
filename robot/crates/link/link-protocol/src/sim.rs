//! A deterministic simulated wire for host tests (ADR-0030, ADR-0033).
//!
//! The wire models a UART between the robot and the PC as two byte queues with an explicit
//! millisecond clock. Faults are deterministic for a fixed seed: a small xorshift generator
//! supplies corrupted bytes, and periodic counters drop, corrupt or truncate traffic. There is
//! no real time, no thread and no async runtime, so a test can replay a link exactly.
//!
//! This module is compiled only with the `std` feature.

use std::{collections::VecDeque, string::String, vec::Vec};

use crate::{
    PROTOCOL_MAJOR,
    codec::{
        CommandMessage, EventMessage, PayloadCodec, StreamSample, decode_command, decode_event, decode_stream,
        encode_command,
    },
    frame::{Class, Direction, Flags, Frame, Seq, Status},
    ids::{Caps, Command, Control, Event, Stream},
    parser::{MAX_PC_TO_ROBOT_FRAME, RobotToPcParser},
    payload::control::{
        BridgeControl, BridgeControlAction, BridgeControlReply, HelloAck, PairingCode, PingAck, RateEntry,
        SetupCredentials, Throttle, ThrottleAction, encode_bridge_control_ack, encode_hello,
    },
    reliability::ReplayWindow,
};

/// The baud the simulated remote assumes; both directions derive a 50 ms timeout from it.
const SIM_BAUD: u32 = 921_600;

/// The replacement seed used when the caller supplies zero, so the generator never sticks.
const FALLBACK_SEED: u32 = 0x9E37_79B9;

/// The seed offset that gives the second direction an independent generator.
const DIRECTION_SEED_OFFSET: u32 = 0x5A5A_A5A5;

/// Fault injection settings for one direction of a [`Wire`].
///
/// Zero disables a fault. Byte and chunk counters are maintained per direction, so traffic in
/// one direction never changes the fault pattern in the other. [`Wire::set_faults`] applies one
/// setting to both directions; [`Wire::set_direction_faults`] injects a fault into one direction
/// only.
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
    /// The active fault settings, indexed with [`slot`].
    faults: [Faults; 2],
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
            faults: [faults; 2],
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

    /// Returns the active fault settings for `direction`.
    #[must_use]
    pub const fn faults(&self, direction: Direction) -> Faults {
        self.faults[slot(direction)]
    }

    /// Replaces the fault settings in both directions; bytes already queued are not changed.
    pub const fn set_faults(&mut self, faults: Faults) {
        self.faults = [faults; 2];
    }

    /// Replaces the fault settings for one direction, leaving the other direction untouched.
    ///
    /// This is how a test injects a fault into, for example, the PC-to-robot direction without
    /// also corrupting the robot-to-PC ACKs that would otherwise let the sender recover.
    pub const fn set_direction_faults(&mut self, direction: Direction, faults: Faults) {
        self.faults[slot(direction)] = faults;
    }

    /// Sends `bytes` in `direction`, applying every active fault.
    ///
    /// Bytes are counted across sends and per direction. A chunk is truncated before its bytes
    /// are dropped or corrupted, and a non-empty chunk advances the clock by the direction's
    /// [`Faults::stall_ms`].
    pub fn send(&mut self, direction: Direction, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }

        let index = slot(direction);
        let faults = self.faults[index];
        self.chunk_counts[index] = self.chunk_counts[index].wrapping_add(1);
        let chunk = self.chunk_counts[index];
        let truncate = faults.truncate_every != 0 && chunk.is_multiple_of(u64::from(faults.truncate_every));
        let end = if truncate {
            bytes.len().saturating_sub(1)
        } else {
            bytes.len()
        };

        for &original in &bytes[..end] {
            self.byte_counts[index] = self.byte_counts[index].wrapping_add(1);
            let count = self.byte_counts[index];
            if faults.drop_every != 0 && count.is_multiple_of(u64::from(faults.drop_every)) {
                continue;
            }
            let corrupt = faults.corrupt_every != 0 && count.is_multiple_of(u64::from(faults.corrupt_every));
            let byte = if corrupt { self.rng[index].next_u8() } else { original };
            self.queues[index].push_back(byte);
        }

        self.now_ms = self.now_ms.wrapping_add(faults.stall_ms);
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

/// A daemon-shaped peer that decodes robot-to-PC telemetry from, and sends commands and control
/// frames to, a [`Wire`].
///
/// Feed the wire's robot-to-PC bytes through the real [`RobotToPcParser`] and read back the typed
/// samples, events, command ACKs and control replies. [`FakeRemote::send_command`] and
/// [`FakeRemote::send_hello`] encode a request with the real codecs and the next sequence number,
/// so a test exercises the same bytes the daemon would put on the wire. Events are acknowledged
/// automatically and deduplicated with the shared [`ReplayWindow`].
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
    /// Received `HELLO_ACK` payloads with their sequence numbers, in arrival order.
    received_hello_acks: Vec<(Seq, Vec<u8>)>,
    /// Received `PING` ACK payloads with their sequence numbers, in arrival order.
    received_ping_acks: Vec<(Seq, Vec<u8>)>,
    /// Unknown or unimplemented control ids seen in ACKs; they are ignored, never decoded.
    unknown_controls: u32,
    /// The next command sequence; commands start at 1 and zero is never sent.
    command_seq: u16,
    /// The next control sequence; control frames start at 1 and zero is never sent.
    control_seq: u16,
    /// The setup SSID the simulated Bridge returns for action 0.
    setup_ssid: String,
    /// The setup passphrase the simulated Bridge returns for action 0.
    setup_passphrase: String,
    /// The twelve-character pairing code the simulated Bridge returns for actions 1 and 2.
    pairing_code: String,
    /// `BRIDGE_CONTROL` requests received from the robot, in arrival order.
    received_bridge_controls: Vec<(BridgeControlAction, Seq)>,
    /// `BRIDGE_CONTROL` ids with an unknown action that were ignored.
    unknown_bridge_controls: u32,
    /// `BRIDGE_CONTROL` acknowledgements waiting to be written to the wire.
    pending_bridge_acks: Vec<(Seq, BridgeControlAction)>,
    /// The Boot Id most recently observed in a `HELLO_ACK`; scopes the event replay window.
    boot_id: Option<u32>,
    /// The event replay window keyed by `(boot_id, seq)`.
    event_window: ReplayWindow,
    /// Received event frames in arrival order, each payload copied out of the parser.
    received_events: Vec<(Event, Seq, Vec<u8>)>,
    /// Event retransmissions ignored by the replay window so far.
    event_duplicates: u32,
    /// Unknown or reserved event ids seen so far; they are ignored, never decoded.
    unknown_events: u32,
    /// Event ACKs waiting to be written to the wire.
    pending_event_acks: Vec<(u16, Seq)>,
    /// Event ACKs already written to the wire, with the status they carried.
    sent_event_acks: Vec<(Seq, Status)>,
    /// The status the peer reports in every event ACK.
    event_ack_status: Status,
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
            received_hello_acks: Vec::new(),
            received_ping_acks: Vec::new(),
            unknown_controls: 0,
            command_seq: 1,
            control_seq: 1,
            setup_ssid: String::from("ExplorerRobot-ABCD"),
            setup_passphrase: String::from("setup-mode"),
            pairing_code: String::from("ABCD2345EFGH"),
            received_bridge_controls: Vec::new(),
            unknown_bridge_controls: 0,
            pending_bridge_acks: Vec::new(),
            boot_id: None,
            event_window: ReplayWindow::new(),
            received_events: Vec::new(),
            event_duplicates: 0,
            unknown_events: 0,
            pending_event_acks: Vec::new(),
            sent_event_acks: Vec::new(),
            event_ack_status: Status::Ok,
        }
    }

    /// Drains the wire's robot-to-PC queue, returns how many stream frames it decoded, and writes
    /// an ACK for every event it saw back to the wire.
    pub fn receive(&mut self, wire: &mut Wire) -> usize {
        let now_ms = wire.now_ms();
        let bytes = wire.take(Direction::RobotToPc);
        let decoded = self.feed(&bytes, now_ms);
        self.flush_event_acks(wire);
        self.flush_bridge_acks(wire);
        decoded
    }

    /// Writes the `BRIDGE_CONTROL` acknowledgements collected by [`FakeRemote::feed`] to the wire.
    ///
    /// [`FakeRemote::receive`] calls this automatically; a caller that feeds bytes directly must
    /// call it once the robot-to-PC bytes have been consumed.
    pub fn flush_bridge_acks(&mut self, wire: &mut Wire) {
        let acks = std::mem::take(&mut self.pending_bridge_acks);
        for (seq, action) in acks {
            let reply = match action {
                BridgeControlAction::ForgetNetwork => BridgeControlReply::Setup(SetupCredentials {
                    status: Status::Ok,
                    ssid: &self.setup_ssid,
                    passphrase: &self.setup_passphrase,
                }),
                BridgeControlAction::ShowPairingCode | BridgeControlAction::RotatePairingCode => {
                    BridgeControlReply::PairingCode(PairingCode {
                        status: Status::Ok,
                        code: &self.pairing_code,
                    })
                }
            };
            let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
            let Ok(len) = encode_bridge_control_ack(&reply, &mut payload) else {
                continue;
            };
            let Ok(frame) = Frame::new(
                PROTOCOL_MAJOR,
                Class::Control,
                Flags::IS_ACK,
                Control::BridgeControl.wire_id(),
                seq,
                &payload[..len],
            ) else {
                continue;
            };
            let mut bytes = [0u8; MAX_PC_TO_ROBOT_FRAME];
            let Ok(written) = frame.encode_into(&mut bytes) else {
                continue;
            };
            wire.send(Direction::PcToRobot, &bytes[..written]);
        }
    }

    /// Writes the event ACKs collected by [`FakeRemote::feed`] to the wire.
    ///
    /// [`FakeRemote::receive`] calls this automatically; a caller that feeds bytes directly must
    /// call it once the robot-to-PC bytes have been consumed.
    pub fn flush_event_acks(&mut self, wire: &mut Wire) {
        let status = self.event_ack_status;
        let acks = std::mem::take(&mut self.pending_event_acks);
        for (id, seq) in acks {
            let payload = [status.bits()];
            let Ok(frame) = Frame::new(PROTOCOL_MAJOR, Class::Event, Flags::IS_ACK, id, seq, &payload) else {
                continue;
            };
            let mut bytes = [0u8; MAX_PC_TO_ROBOT_FRAME];
            let Ok(written) = frame.encode_into(&mut bytes) else {
                continue;
            };
            wire.send(Direction::PcToRobot, &bytes[..written]);
            self.sent_event_acks.push((seq, status));
        }
    }

    /// Feeds raw robot-to-PC bytes through the real parser and returns how many stream frames
    /// completed.
    ///
    /// Unknown stream and event ids are counted and ignored. Command frames carrying `IS_ACK` are
    /// copied into the ACK log. Event frames are deduplicated by `(boot_id, seq)`; a duplicate is
    /// counted and not recorded again, but it is still acknowledged so a lost ACK is repaired.
    /// Every other non-stream class is ignored for now.
    pub fn feed(&mut self, bytes: &[u8], now_ms: u32) -> usize {
        let before = self.received.len();
        let received = &mut self.received;
        let received_acks = &mut self.received_acks;
        let received_hello_acks = &mut self.received_hello_acks;
        let received_ping_acks = &mut self.received_ping_acks;
        let received_events = &mut self.received_events;
        let event_window = &mut self.event_window;
        let pending_event_acks = &mut self.pending_event_acks;
        let received_bridge_controls = &mut self.received_bridge_controls;
        let pending_bridge_acks = &mut self.pending_bridge_acks;
        let mut current_boot = self.boot_id.unwrap_or(0);
        let mut new_boot_id = None;
        let mut duplicates = 0u32;
        let mut unknown = 0u32;
        let mut unknown_acks = 0u32;
        let mut unknown_controls = 0u32;
        let mut unknown_events = 0u32;
        let mut unknown_bridge_controls = 0u32;
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
            Class::Control => {
                if !frame.header.flags.contains(Flags::IS_ACK) {
                    // The simulated peer is also the Bridge for the one robot-to-Bridge control.
                    if frame.header.id == Control::BridgeControl.wire_id() {
                        if let Ok(request) = BridgeControl::decode(frame.payload) {
                            received_bridge_controls.push((request.action, frame.header.seq));
                            pending_bridge_acks.push((frame.header.seq, request.action));
                        } else {
                            unknown_bridge_controls = unknown_bridge_controls.saturating_add(1);
                        }
                    }
                    return;
                }
                match Control::from_wire(frame.header.id) {
                    Some(Control::Hello) => {
                        if let Ok(ack) = HelloAck::decode(frame.payload) {
                            current_boot = ack.boot_id;
                            new_boot_id = Some(ack.boot_id);
                            received_hello_acks.push((frame.header.seq, frame.payload.to_vec()));
                        } else {
                            unknown_controls = unknown_controls.saturating_add(1);
                        }
                    }
                    Some(Control::Ping) if PingAck::decode(frame.payload).is_ok() => {
                        received_ping_acks.push((frame.header.seq, frame.payload.to_vec()));
                    }
                    _ => unknown_controls = unknown_controls.saturating_add(1),
                }
            }
            Class::Event => {
                if frame.header.flags.contains(Flags::IS_ACK) {
                    return;
                }
                let Some(event) = Event::from_wire(frame.header.id) else {
                    unknown_events = unknown_events.saturating_add(1);
                    return;
                };
                let seq = frame.header.seq;
                if event_window.accept(current_boot, seq) {
                    received_events.push((event, seq, frame.payload.to_vec()));
                } else {
                    duplicates = duplicates.saturating_add(1);
                }
                pending_event_acks.push((frame.header.id, seq));
            }
        });
        if new_boot_id.is_some() {
            self.boot_id = new_boot_id;
        }
        self.unknown_streams = self.unknown_streams.saturating_add(unknown);
        self.unknown_acks = self.unknown_acks.saturating_add(unknown_acks);
        self.unknown_controls = self.unknown_controls.saturating_add(unknown_controls);
        self.unknown_events = self.unknown_events.saturating_add(unknown_events);
        self.unknown_bridge_controls = self.unknown_bridge_controls.saturating_add(unknown_bridge_controls);
        self.event_duplicates = self.event_duplicates.saturating_add(duplicates);
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

    /// Sends a `HELLO` request with the next control sequence number.
    ///
    /// Returns the sequence used, or [`None`] when the payload does not fit the PC-to-robot cap.
    pub fn send_hello(
        &mut self,
        wire: &mut Wire,
        major: u8,
        minor: u8,
        caps: Caps,
        epoch: u32,
        rates: &[RateEntry],
    ) -> Option<Seq> {
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = encode_hello(major, minor, caps, epoch, rates.iter().copied(), &mut payload).ok()?;
        self.send_control(wire, Control::Hello, &payload[..written])
    }

    /// Sends a `PING` request with the next control sequence number.
    pub fn send_ping(&mut self, wire: &mut Wire) -> Option<Seq> {
        self.send_control(wire, Control::Ping, &[])
    }

    /// Sends a `THROTTLE` hint with the next control sequence number.
    ///
    /// `stream_id` is a stream id or [`Throttle::ALL_STREAMS`] for every stream. `ttl_ms` is the
    /// hint's lifetime in milliseconds. A hint is never acknowledged, so a valid call produces no
    /// reply. Returns the sequence used, or [`None`] when the payload cannot be encoded.
    pub fn send_throttle(
        &mut self,
        wire: &mut Wire,
        stream_id: u16,
        action: ThrottleAction,
        rate_chz: u16,
        ttl_ms: u16,
    ) -> Option<Seq> {
        let hint = Throttle::new(stream_id, action, rate_chz, ttl_ms);
        let mut payload = [0u8; Throttle::ENCODED_LEN];
        let written = hint.encode_into(&mut payload).ok()?;
        self.send_control(wire, Control::Throttle, &payload[..written])
    }

    /// Encodes and sends one control frame, returning the sequence used.
    fn send_control(&mut self, wire: &mut Wire, control: Control, payload: &[u8]) -> Option<Seq> {
        let seq = Seq::new(self.control_seq);
        let frame = Frame::new(
            PROTOCOL_MAJOR,
            Class::Control,
            Flags::default(),
            control.wire_id(),
            seq,
            payload,
        )
        .ok()?;
        let mut bytes = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let written = frame.encode_into(&mut bytes).ok()?;
        wire.send(Direction::PcToRobot, &bytes[..written]);
        let next = self.control_seq.wrapping_add(1);
        self.control_seq = if next == 0 { 1 } else { next };
        Some(seq)
    }

    /// Returns every received `HELLO_ACK` in arrival order with its decoded payload.
    pub fn hello_acks(&self) -> impl Iterator<Item = (Seq, HelloAck<'_>)> + '_ {
        self.received_hello_acks
            .iter()
            .filter_map(|(seq, payload)| HelloAck::decode(payload).ok().map(|ack| (*seq, ack)))
    }

    /// Returns the most recently received `HELLO_ACK`, if any survived decoding.
    #[must_use]
    pub fn last_hello_ack(&self) -> Option<(Seq, HelloAck<'_>)> {
        self.hello_acks().last()
    }

    /// Returns every received `PING` acknowledgement in arrival order.
    pub fn ping_acks(&self) -> impl Iterator<Item = (Seq, PingAck)> + '_ {
        self.received_ping_acks
            .iter()
            .filter_map(|(seq, payload)| PingAck::decode(payload).ok().map(|ack| (*seq, ack)))
    }

    /// Returns the most recently received `PING` acknowledgement, if any survived decoding.
    #[must_use]
    pub fn last_ping_ack(&self) -> Option<(Seq, PingAck)> {
        self.ping_acks().last()
    }

    /// Returns how many unknown or unimplemented control ids have been ignored.
    #[must_use]
    pub const fn unknown_controls(&self) -> u32 {
        self.unknown_controls
    }

    /// Returns the next control sequence number the remote will use.
    #[must_use]
    pub const fn next_control_seq(&self) -> u16 {
        self.control_seq
    }

    /// Returns every accepted event frame in arrival order with its decoded payload.
    ///
    /// A retransmission ignored by the replay window is absent. A frame whose payload the codec
    /// rejects is skipped, mirroring [`FakeRemote::streams`].
    pub fn events(&self) -> impl Iterator<Item = (Event, Seq, EventMessage<'_>)> + '_ {
        self.received_events.iter().filter_map(|(event, seq, payload)| {
            decode_event(*event, false, payload)
                .ok()
                .map(|message| (*event, *seq, message))
        })
    }

    /// Returns the most recently accepted event, if any survived decoding.
    #[must_use]
    pub fn last_event(&self) -> Option<(Event, Seq, EventMessage<'_>)> {
        self.events().last()
    }

    /// Returns how many distinct event frames were accepted.
    #[must_use]
    pub const fn received_event_count(&self) -> usize {
        self.received_events.len()
    }

    /// Returns how many event retransmissions the replay window ignored.
    #[must_use]
    pub const fn event_duplicates(&self) -> u32 {
        self.event_duplicates
    }

    /// Returns how many unknown or reserved event ids were ignored.
    #[must_use]
    pub const fn unknown_events(&self) -> u32 {
        self.unknown_events
    }

    /// Returns every event ACK written to the wire, with the status it carried.
    pub fn event_acks(&self) -> impl Iterator<Item = (Seq, Status)> + '_ {
        self.sent_event_acks.iter().copied()
    }

    /// Returns the most recent event ACK written to the wire, if any.
    #[must_use]
    pub fn last_event_ack(&self) -> Option<(Seq, Status)> {
        self.sent_event_acks.last().copied()
    }

    /// Returns how many event ACKs have been written to the wire.
    #[must_use]
    pub const fn event_ack_count(&self) -> usize {
        self.sent_event_acks.len()
    }

    /// Returns the Boot Id most recently observed in a `HELLO_ACK`.
    #[must_use]
    pub const fn boot_id(&self) -> Option<u32> {
        self.boot_id
    }

    /// Returns the status the peer reports in every event ACK.
    #[must_use]
    pub const fn event_ack_status(&self) -> Status {
        self.event_ack_status
    }

    /// Sets the status the peer reports in every event ACK.
    pub const fn set_event_ack_status(&mut self, status: Status) {
        self.event_ack_status = status;
    }

    /// Returns every `BRIDGE_CONTROL` request received, in arrival order.
    pub fn bridge_controls(&self) -> impl Iterator<Item = (BridgeControlAction, Seq)> + '_ {
        self.received_bridge_controls.iter().copied()
    }

    /// Returns the most recent `BRIDGE_CONTROL` request, if any.
    #[must_use]
    pub fn last_bridge_control(&self) -> Option<(BridgeControlAction, Seq)> {
        self.received_bridge_controls.last().copied()
    }

    /// Returns how many `BRIDGE_CONTROL` requests have been received.
    #[must_use]
    pub const fn bridge_control_count(&self) -> usize {
        self.received_bridge_controls.len()
    }

    /// Returns how many `BRIDGE_CONTROL` requests with an unknown action were ignored.
    #[must_use]
    pub const fn unknown_bridge_controls(&self) -> u32 {
        self.unknown_bridge_controls
    }

    /// Returns the setup SSID the simulated Bridge reports for action 0.
    #[must_use]
    pub fn setup_ssid(&self) -> &str {
        &self.setup_ssid
    }

    /// Returns the setup passphrase the simulated Bridge reports for action 0.
    #[must_use]
    pub fn setup_passphrase(&self) -> &str {
        &self.setup_passphrase
    }

    /// Returns the pairing code the simulated Bridge reports for actions 1 and 2.
    #[must_use]
    pub fn pairing_code(&self) -> &str {
        &self.pairing_code
    }

    /// Sets the setup SSID and passphrase the simulated Bridge reports for action 0.
    pub fn set_setup_credentials(&mut self, ssid: &str, passphrase: &str) {
        self.setup_ssid = String::from(ssid);
        self.setup_passphrase = String::from(passphrase);
    }

    /// Sets the pairing code the simulated Bridge reports for actions 1 and 2.
    ///
    /// A code that is not exactly twelve characters cannot be encoded, so no acknowledgement is
    /// written for a request while it is set.
    pub fn set_pairing_code(&mut self, code: &str) {
        self.pairing_code = String::from(code);
    }
}

impl Default for FakeRemote {
    fn default() -> Self {
        Self::new()
    }
}
