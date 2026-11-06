//! The robot side of the Link Protocol (ADR-0032).
//!
//! `LinkCore` owns the RX parser, the pending stream slots, the event ring, the lease, command
//! dedupe and TX priority. It is driven with plain millisecond timestamps rather than a clock so it
//! can be host-tested; the firmware keeps the UART, the port power, the sampling and the mapping.
//!
//! This crate is `no_std` and allocates nothing. The `std` feature exists only for host tests and
//! for the simulated link in `remote-protocol`.
//!
//! This ticket lands the stream half and the two-phase command seam. Commands are accepted one at
//! a time: [`LinkCore::poll`] yields a structurally valid, known, same-major, in-order command and
//! the firmware answers it with [`LinkCore::ack_command`] before polling again. Auto-ACKs for
//! unknown ids, malformed payloads, bad flags, version mismatches and stale sequences never reach
//! the firmware. Events, `HELLO`/`PING`/lease and the Deadman arrive in later tickets; their timing
//! values are already in [`timing`].

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(feature = "std")]
extern crate std;

pub mod timing;

use remote_protocol::{
    Class, Command, CommandMessage, Direction, EncodeError, Flags, Frame, PROTOCOL_MAJOR, Seq, Status,
    codec::{PayloadCodec, StreamSample, decode_command},
    ids::Stream,
    parser::PcToRobotParser,
    payload::stream::{Activity, Calibration, LidarCloud, Motion, Perception, Power},
};
pub use timing::Config;

/// The total encoded payload staging the six latest-wins slots reserve.
///
/// ADR-0030 allows roughly 1.1 KiB of staging on the robot; the v1 catalogue sums to 767 bytes,
/// dominated by the 720-byte `LiDAR` cloud.
pub const STREAM_STAGING_BYTES: usize = Power::ENCODED_LEN
    + Motion::ENCODED_LEN
    + Activity::ENCODED_LEN
    + Perception::ENCODED_LEN
    + LidarCloud::ENCODED_LEN
    + Calibration::ENCODED_LEN;

/// The staging budget ADR-0030 allows for the pending stream slots.
pub const STREAM_STAGING_BUDGET_BYTES: usize = 1024;

/// A compile-time guard that the stream catalogue fits the staging budget.
const _: () = assert!(STREAM_STAGING_BYTES <= STREAM_STAGING_BUDGET_BYTES);

/// The largest command payload the robot accepts, equal to the PC-to-robot cap.
///
/// One inbound slot and one stored ACK payload are sized to this value, so the command seam stays
/// bounded independently of the parser's input chunk.
pub const COMMAND_PAYLOAD_BYTES: usize = Direction::PcToRobot.max_payload();

/// The number of auto-ACK records held before further invalid frames in one chunk are dropped.
///
/// A conforming peer has one command in flight, so the ring is only stressed by malformed or
/// replayed traffic. Dropping an auto-ACK is safe: the peer retries and the frame is answered
/// again.
const AUTO_ACK_CAPACITY: usize = 8;

/// Whether a Remote is currently attached to the link.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PeerState {
    /// No valid PC-to-robot frame has arrived in this powered link period, or the lease expired.
    #[default]
    NoPeer,
    /// A valid PC-to-robot frame arrived and the lease has not expired.
    PeerLive,
}

/// One accepted command, ready to be polled.
#[derive(Clone, Copy, Debug)]
pub struct CommandRequest<'a> {
    /// The command's sequence number, mirrored by the ACK.
    pub seq: Seq,
    /// The frame's flags, including `ACK_REQUESTED`.
    pub flags: Flags,
    /// The command id the payload belongs to.
    pub command: Command,
    /// The typed request payload.
    pub message: CommandMessage<'a>,
}

/// A message the firmware must handle, yielded one at a time by [`LinkCore::poll`].
///
/// Later tickets add event acknowledgements and control replies as further variants; the enum is
/// the stable seam between the link and the firmware.
#[derive(Debug)]
pub enum Inbound<'a> {
    /// A structurally valid command that the firmware validates, dispatches and acknowledges.
    Command(CommandRequest<'a>),
}

/// The latest encoded payload of each stream.
///
/// Each slot is sized from the ADR-0031 catalogue and holds one encoded payload. A publish
/// replaces the slot's contents; nothing is queued.
struct StreamSlots {
    /// The pending `POWER` payload.
    power: heapless::Vec<u8, { Power::ENCODED_LEN }>,
    /// The pending `MOTION` payload.
    motion: heapless::Vec<u8, { Motion::ENCODED_LEN }>,
    /// The pending `ACTIVITY` payload.
    activity: heapless::Vec<u8, { Activity::ENCODED_LEN }>,
    /// The pending `PERCEPTION` payload.
    perception: heapless::Vec<u8, { Perception::ENCODED_LEN }>,
    /// The pending `LIDAR_CLOUD` payload.
    lidar_cloud: heapless::Vec<u8, { LidarCloud::ENCODED_LEN }>,
    /// The pending `CALIBRATION` payload.
    calibration: heapless::Vec<u8, { Calibration::ENCODED_LEN }>,
}

impl StreamSlots {
    /// Creates six empty slots.
    const fn new() -> Self {
        Self {
            power: heapless::Vec::new(),
            motion: heapless::Vec::new(),
            activity: heapless::Vec::new(),
            perception: heapless::Vec::new(),
            lidar_cloud: heapless::Vec::new(),
            calibration: heapless::Vec::new(),
        }
    }

    /// Returns the encoded payload currently stored for `stream`.
    fn payload(&self, stream: Stream) -> &[u8] {
        match stream {
            Stream::Power => &self.power,
            Stream::Motion => &self.motion,
            Stream::Activity => &self.activity,
            Stream::Perception => &self.perception,
            Stream::LidarCloud => &self.lidar_cloud,
            Stream::Calibration => &self.calibration,
        }
    }
}

/// The single accepted command's copied payload and metadata.
///
/// The parser yields a frame borrowed from its own buffer, so the payload is copied here before
/// the parser call returns. `poll` then decodes it again from this slot, which is what lets the
/// returned [`Inbound`] borrow the core rather than the parser.
struct InboundSlot {
    /// The command id the payload belongs to.
    command: Command,
    /// The command's sequence number.
    seq: u16,
    /// The frame's flags.
    flags: Flags,
    /// The number of valid bytes in `bytes`.
    len: usize,
    /// The copied payload.
    bytes: [u8; COMMAND_PAYLOAD_BYTES],
}

impl InboundSlot {
    /// Creates an empty slot.
    const fn empty() -> Self {
        Self {
            command: Command::GetStatus,
            seq: 0,
            flags: Flags::from_bits(0),
            len: 0,
            bytes: [0; COMMAND_PAYLOAD_BYTES],
        }
    }
}

/// The command the firmware has polled but not yet acknowledged.
#[derive(Clone, Copy, Debug)]
struct PendingCommand {
    /// The command id, mirrored by the ACK.
    id: u16,
    /// The sequence number, mirrored by the ACK.
    seq: u16,
    /// The flags of the original request.
    flags: Flags,
}

/// The full ACK payload of the last acknowledged command, replayable for a duplicate.
struct StoredAck {
    /// Whether `payload` holds an ACK.
    valid: bool,
    /// The command id, mirrored by the ACK.
    id: u16,
    /// The sequence number, mirrored by the ACK.
    seq: u16,
    /// The number of valid bytes in `payload`: the status byte plus the result.
    len: usize,
    /// The stored ACK payload.
    payload: [u8; COMMAND_PAYLOAD_BYTES],
}

impl StoredAck {
    /// Creates an empty stored ACK.
    const fn empty() -> Self {
        Self {
            valid: false,
            id: 0,
            seq: 0,
            len: 0,
            payload: [0; COMMAND_PAYLOAD_BYTES],
        }
    }
}

/// One buffered auto-ACK: the request's class is always `Command`, and the payload is the status.
#[derive(Clone, Copy, Debug)]
struct AutoAck {
    /// The id to mirror, including an id this version does not know.
    id: u16,
    /// The sequence number to mirror.
    seq: u16,
    /// The status to report.
    status: Status,
}

/// Command dedupe, the inbound slot, the stored ACK and the outbound auto-ACKs.
///
/// Grouped so `on_rx` can borrow the parser and this state as disjoint fields of [`LinkCore`].
struct CommandState {
    /// The session epoch; a reset restarts command dedupe. Zero until `HELLO` lands.
    epoch: u32,
    /// The last accepted command sequence; commands start at 1 because this starts at 0.
    last_seq: u16,
    /// The last status the firmware returned.
    last_status: Status,
    /// The single accepted command payload.
    inbound: InboundSlot,
    /// Whether `inbound` was accepted and has not been polled.
    ready: bool,
    /// The polled command waiting for `ack_command`.
    pending: Option<PendingCommand>,
    /// The last acknowledged command's payload.
    stored: StoredAck,
    /// Whether the stored ACK must be emitted on the next frame.
    stored_pending: bool,
    /// Auto-ACKs waiting to be emitted, oldest first.
    auto_acks: heapless::Vec<AutoAck, AUTO_ACK_CAPACITY>,
}

impl CommandState {
    /// Creates an empty command state for epoch zero.
    const fn new() -> Self {
        Self {
            epoch: 0,
            last_seq: 0,
            last_status: Status::Ok,
            inbound: InboundSlot::empty(),
            ready: false,
            pending: None,
            stored: StoredAck::empty(),
            stored_pending: false,
            auto_acks: heapless::Vec::new(),
        }
    }

    /// Consumes one PC-to-robot command frame.
    fn on_frame(&mut self, frame: Frame<'_>) {
        let flags = frame.header.flags;
        let id = frame.header.id;
        let seq = frame.header.seq.get();
        let ack_requested = flags.contains(Flags::ACK_REQUESTED);

        if flags.contains(Flags::IS_ACK) {
            // The robot never consumes command acknowledgements; an inbound ACK is ignored.
            return;
        }
        // ADR-0030: on a major mismatch the receiver still parses the frame — the lease will see
        // it later — and answers a command with VERSION_MISMATCH instead of executing it.
        if frame.header.major != PROTOCOL_MAJOR {
            self.auto_ack(id, seq, Status::VersionMismatch, ack_requested);
            return;
        }
        if flags.contains(Flags::FRAGMENT) {
            // The parser yields the reserved flag intact; v1 rejects it here.
            self.auto_ack(id, seq, Status::BadFlags, ack_requested);
            return;
        }
        let Some(command) = Command::from_wire(id) else {
            self.auto_ack(id, seq, Status::UnknownId, ack_requested);
            return;
        };
        if let Err(error) = decode_command(command, false, frame.payload) {
            self.auto_ack(id, seq, error.status(), ack_requested);
            return;
        }
        self.accept(command, seq, flags, frame.payload, ack_requested);
    }

    /// Applies the dedupe rules to a structurally valid command.
    fn accept(&mut self, command: Command, seq: u16, flags: Flags, payload: &[u8], ack_requested: bool) {
        if seq == self.last_seq.wrapping_add(1) {
            // Only one command may be in the slot, and the firmware answers a polled command
            // before the next one is accepted. A conforming peer stops and waits anyway.
            if self.ready || self.pending.is_some() {
                return;
            }
            let len = payload.len().min(COMMAND_PAYLOAD_BYTES);
            self.inbound.command = command;
            self.inbound.seq = seq;
            self.inbound.flags = flags;
            self.inbound.len = len;
            self.inbound.bytes[..len].copy_from_slice(&payload[..len]);
            self.ready = true;
            self.last_seq = seq;
        } else if seq == self.last_seq {
            // A duplicate never executes again. Before `ack_command` there is no stored ACK yet;
            // the outstanding ACK answers the retry when it arrives.
            if self.stored.valid && self.stored.seq == seq && ack_requested {
                self.stored_pending = true;
            }
        } else {
            self.auto_ack(command.wire_id(), seq, Status::Stale, ack_requested);
        }
    }

    /// Queues a status-only ACK when the request asked for one.
    fn auto_ack(&mut self, id: u16, seq: u16, status: Status, ack_requested: bool) {
        if !ack_requested {
            return;
        }
        // A full ring drops the record; the peer retries and is answered again.
        let _ = self.auto_acks.push(AutoAck { id, seq, status });
    }

    /// Encodes one buffered auto-ACK into `out`, returning the bytes written.
    fn pop_auto_ack(&mut self, out: &mut [u8]) -> Option<usize> {
        let ack = self.auto_acks.first().copied()?;
        let payload = [ack.status.bits()];
        let frame = Frame::new(
            PROTOCOL_MAJOR,
            Class::Command,
            Flags::IS_ACK,
            ack.id,
            Seq::new(ack.seq),
            &payload,
        )
        .ok()?;
        let written = frame.encode_into(out).ok()?;
        self.auto_acks.remove(0);
        Some(written)
    }

    /// Encodes the stored ACK into `out` when one is pending.
    ///
    /// A buffer too small leaves the ACK pending, matching the stream slots' contract.
    fn pop_stored_ack(&mut self, out: &mut [u8]) -> Option<usize> {
        if !self.stored_pending || !self.stored.valid {
            return None;
        }
        let stored = &self.stored;
        let frame = Frame::new(
            PROTOCOL_MAJOR,
            Class::Command,
            Flags::IS_ACK,
            stored.id,
            Seq::new(stored.seq),
            &stored.payload[..stored.len],
        )
        .ok()?;
        let written = frame.encode_into(out).ok()?;
        self.stored_pending = false;
        Some(written)
    }
}

/// The robot's link state and the composition point for the protocol mechanisms.
///
/// Construct it with the firmware's Boot Id and feed it bytes, time and samples. It never blocks,
/// never allocates and never holds an incoming frame past the call that produced it.
pub struct LinkCore {
    /// The timing configuration; later tickets consume its values.
    config: Config,
    /// The Boot Id the firmware minted for this powered link period.
    boot_id: u32,
    /// The inbound parser for PC-to-robot frames.
    parser: PcToRobotParser,
    /// The latest-wins encoded payload slots.
    slots: StreamSlots,
    /// One sequence counter per stream, incremented when a frame is emitted.
    sequences: [u16; Stream::COUNT],
    /// The pending bitmap, indexed by [`Stream::index`].
    pending: u8,
    /// The stream index to try first on the next emission, for round-robin fairness.
    cursor: usize,
    /// The number of unknown inbound stream ids ignored so far.
    unknown_streams: u32,
    /// Command dedupe, the inbound slot and the ACK state.
    commands: CommandState,
}

impl LinkCore {
    /// Creates a link core with `config`, the firmware's `boot_id` and the current time.
    #[must_use]
    pub fn new(config: Config, boot_id: u32, _now_ms: u64) -> Self {
        let mut parser = PcToRobotParser::new(Direction::PcToRobot, timing::BAUD);
        parser.set_timeout_ms(config.frame_timeout_ms);
        Self {
            config,
            boot_id,
            parser,
            slots: StreamSlots::new(),
            sequences: [0; Stream::COUNT],
            pending: 0,
            cursor: 0,
            unknown_streams: 0,
            commands: CommandState::new(),
        }
    }

    /// Returns the timing configuration this core was built with.
    #[must_use]
    pub const fn config(&self) -> Config {
        self.config
    }

    /// Returns the Boot Id the firmware minted for this link period.
    #[must_use]
    pub const fn boot_id(&self) -> u32 {
        self.boot_id
    }

    /// Returns the current command-dedupe epoch. Zero until `HELLO` lands in a later ticket.
    #[must_use]
    pub const fn epoch(&self) -> u32 {
        self.commands.epoch
    }

    /// Returns the last accepted command sequence.
    #[must_use]
    pub const fn last_seq(&self) -> u16 {
        self.commands.last_seq
    }

    /// Returns the last status the firmware returned for a command.
    #[must_use]
    pub const fn last_status(&self) -> Status {
        self.commands.last_status
    }

    /// Returns how many unknown inbound stream ids have been ignored.
    #[must_use]
    pub const fn unknown_streams(&self) -> u32 {
        self.unknown_streams
    }

    /// Publishes one telemetry sample into its per-stream latest-wins slot.
    ///
    /// The sample is encoded immediately; a newer publish replaces the pending one, so there is no
    /// queue and a coalesced sample is never retransmitted. `requested_rate` and `now_ms` are
    /// accepted for ticket 08's emission schedule and ignored until then.
    pub fn publish_stream(&mut self, sample: &StreamSample<'_>, _requested_rate: u32, _now_ms: u64) {
        let stored = match sample {
            StreamSample::Power(power) => store_fixed(&mut self.slots.power, |out| power.encode_into(out)),
            StreamSample::Motion(motion) => store_fixed(&mut self.slots.motion, |out| motion.encode_into(out)),
            StreamSample::Activity(activity) => self.store_activity(activity),
            StreamSample::Perception(perception) => {
                store_fixed(&mut self.slots.perception, |out| perception.encode_into(out))
            }
            StreamSample::LidarCloud(cloud) => store_fixed(&mut self.slots.lidar_cloud, |out| cloud.encode_into(out)),
            StreamSample::Calibration(calibration) => {
                store_fixed(&mut self.slots.calibration, |out| calibration.encode_into(out))
            }
        };
        self.set_pending(sample.stream(), stored);
    }

    /// Yields the one accepted command, if any, for the firmware to validate and dispatch.
    ///
    /// The returned value borrows the core's inbound slot, so it must be dropped before the next
    /// call. Nothing is yielded while an earlier polled command is unacknowledged: the firmware
    /// must call [`LinkCore::ack_command`] first.
    pub fn poll(&mut self) -> Option<Inbound<'_>> {
        let state = &mut self.commands;
        if state.pending.is_some() || !state.ready {
            return None;
        }
        let slot = &state.inbound;
        let Ok(message) = decode_command(slot.command, false, &slot.bytes[..slot.len]) else {
            // `on_rx` already validated the payload; this branch is unreachable but never panics.
            return None;
        };
        state.ready = false;
        state.pending = Some(PendingCommand {
            id: slot.command.wire_id(),
            seq: slot.seq,
            flags: slot.flags,
        });
        Some(Inbound::Command(CommandRequest {
            seq: Seq::new(slot.seq),
            flags: slot.flags,
            command: slot.command,
            message,
        }))
    }

    /// Records the firmware's outcome for the polled command and queues its ACK.
    ///
    /// `result` is the command-specific ACK payload after the status byte; it is truncated to the
    /// PC-to-robot cap so the stored ACK stays bounded. The ACK is emitted only when the request
    /// set `ACK_REQUESTED`, but a duplicate re-sends the stored payload.
    ///
    /// Returns `false` when `seq` is not the command awaiting an answer; nothing changes then.
    pub fn ack_command(&mut self, seq: u16, status: Status, result: &[u8]) -> bool {
        let state = &mut self.commands;
        let Some(pending) = state.pending else {
            return false;
        };
        if pending.seq != seq {
            return false;
        }

        let mut payload = [0u8; COMMAND_PAYLOAD_BYTES];
        payload[0] = status.bits();
        let result_len = result.len().min(COMMAND_PAYLOAD_BYTES - 1);
        payload[1..=result_len].copy_from_slice(&result[..result_len]);

        state.stored = StoredAck {
            valid: true,
            id: pending.id,
            seq: pending.seq,
            len: 1 + result_len,
            payload,
        };
        state.last_status = status;
        state.pending = None;
        state.stored_pending = pending.flags.contains(Flags::ACK_REQUESTED);
        true
    }

    /// Resets command dedupe for a new session epoch (the `HELLO` half lands in a later ticket).
    ///
    /// The accepted sequence restarts at 0, so the next valid command is sequence 1. Commands
    /// before any `HELLO` already run under epoch zero.
    pub fn reset_epoch(&mut self, epoch: u32) {
        let state = &mut self.commands;
        state.epoch = epoch;
        state.last_seq = 0;
        state.last_status = Status::Ok;
        state.inbound = InboundSlot::empty();
        state.ready = false;
        state.pending = None;
        state.stored = StoredAck::empty();
        state.stored_pending = false;
        state.auto_acks.clear();
    }

    /// Emits the next frame into `out`, returning the number of bytes written.
    ///
    /// Control and ACK frames take priority over streams. The buffered auto-ACKs go first, then
    /// the stored command ACK, then the pending streams in round-robin order. A frame that does not
    /// fit returns [`None`] without consuming the pending payload or advancing its sequence.
    pub fn next_frame(&mut self, out: &mut [u8]) -> Option<usize> {
        if let Some(written) = self.commands.pop_auto_ack(out) {
            return Some(written);
        }
        if let Some(written) = self.commands.pop_stored_ack(out) {
            return Some(written);
        }
        self.next_stream_frame(out)
    }

    /// Emits the next pending stream frame, round-robin between the slots.
    fn next_stream_frame(&mut self, out: &mut [u8]) -> Option<usize> {
        for offset in 0..Stream::COUNT {
            let index = (self.cursor + offset) % Stream::COUNT;
            if self.pending & (1u8 << index) == 0 {
                continue;
            }
            let stream = Stream::ALL[index];
            let frame = Frame::new(
                PROTOCOL_MAJOR,
                Class::Stream,
                Flags::default(),
                stream.wire_id(),
                Seq::new(self.sequences[index]),
                self.slots.payload(stream),
            )
            .ok()?;
            let written = frame.encode_into(out).ok()?;
            self.sequences[index] = self.sequences[index].wrapping_add(1);
            self.pending &= !(1u8 << index);
            self.cursor = (index + 1) % Stream::COUNT;
            return Some(written);
        }
        None
    }

    /// Advances the link's timers to `now_ms`.
    ///
    /// The lease, the Deadman, retries and the emission schedule land in later tickets, so this is
    /// a no-op for now.
    pub const fn tick(&mut self, _now_ms: u64) {}

    /// Returns the current peer state.
    ///
    /// The session lease arrives in ticket 06; until then no peer is ever reported live.
    #[must_use]
    pub const fn peer(&self) -> PeerState {
        PeerState::NoPeer
    }

    /// Takes the one-shot `peer_lost` edge.
    ///
    /// The Deadman arrives in ticket 06; until then the edge never fires.
    #[must_use]
    pub const fn take_peer_lost(&mut self) -> bool {
        false
    }

    /// Feeds PC-to-robot bytes through the parser.
    ///
    /// A command frame is validated and copied into the single inbound slot; a malformed, unknown,
    /// bad-flagged or stale command is auto-ACKed and never yielded. Unknown stream ids are counted
    /// and ignored. A major mismatch still counts as a valid frame for the lease later.
    pub fn on_rx(&mut self, bytes: &[u8], now_ms: u64) {
        let now = u32::try_from(now_ms).unwrap_or(u32::MAX);
        let Self {
            parser,
            commands,
            unknown_streams,
            ..
        } = self;
        let mut unknown = 0u32;
        parser.feed(bytes, now, |frame| match frame.header.class {
            Class::Stream => {
                if Stream::from_wire(frame.header.id).is_none() {
                    unknown = unknown.saturating_add(1);
                }
            }
            Class::Command => commands.on_frame(frame),
            Class::Event | Class::Control => {}
        });
        *unknown_streams = unknown_streams.saturating_add(unknown);
    }

    /// Encodes and stores an `ACTIVITY` payload, whose encoded length varies with the detail.
    fn store_activity(&mut self, activity: &Activity<'_>) -> bool {
        let mut buffer = [0u8; Activity::ENCODED_LEN];
        self.slots.activity.clear();
        if let Ok(len) = activity.encode_into(&mut buffer) {
            self.slots.activity.extend_from_slice(&buffer[..len]).is_ok()
        } else {
            false
        }
    }

    /// Sets or clears `stream`'s pending bit after a publish attempt.
    const fn set_pending(&mut self, stream: Stream, pending: bool) {
        let bit = 1u8 << stream.index();
        if pending {
            self.pending |= bit;
        } else {
            self.pending &= !bit;
        }
    }
}

/// Encodes a fixed-size payload into its slot, clearing the slot if encoding fails.
///
/// The slot's capacity is the payload's exact encoded length, so a successful encode fills it.
fn store_fixed<const N: usize>(
    slot: &mut heapless::Vec<u8, N>,
    encode: impl FnOnce(&mut [u8]) -> Result<usize, EncodeError>,
) -> bool {
    slot.clear();
    if slot.resize(N, 0).is_err() {
        return false;
    }
    match encode(&mut slot[..]) {
        Ok(len) if len <= N => {
            slot.truncate(len);
            true
        }
        _ => {
            slot.clear();
            false
        }
    }
}
