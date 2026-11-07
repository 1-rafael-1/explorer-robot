//! The robot side of the Link Protocol (ADR-0032).
//!
//! `LinkCore` owns the RX parser, the pending stream slots, the event ring, the lease, command
//! dedupe and TX priority. It is driven with plain millisecond timestamps rather than a clock so it
//! can be host-tested; the firmware keeps the UART, the port power, the sampling and the mapping.
//!
//! This crate is `no_std` and allocates nothing. The `std` feature exists only for host tests and
//! for the simulated link in `remote-protocol`.
//!
//! This ticket lands the stream half, the two-phase command seam, the Session handshake, lease and
//! Peer/Deadman, and the reliable event ring. Commands are accepted one at a time:
//! [`LinkCore::poll`] yields a structurally valid, known, same-major, in-order command and the
//! firmware answers it with [`LinkCore::ack_command`] before polling again. Auto-ACKs for unknown
//! ids, malformed payloads, bad flags, version mismatches and stale sequences never reach the
//! firmware. `HELLO` and `PING` are answered inside the core. Events are sequenced per Boot Id,
//! retransmitted until the peer ACKs them and reported honestly on ring overflow; throttle
//! application arrives in a later ticket.

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(feature = "std")]
extern crate std;

mod events;
pub mod timing;

use events::EventRing;
pub use events::{EventAckNotice, Publish};
use remote_protocol::{
    Caps, Class, Command, CommandMessage, Control, Direction, EncodeError, Event, EventMessage, Flags, Frame,
    PROTOCOL_MAJOR, PROTOCOL_MINOR, Seq, Status,
    codec::{PayloadCodec, StreamSample, decode_command},
    ids::Stream,
    parser::PcToRobotParser,
    payload::{
        control::{Hello, HelloAck, Ping, PingAck, RateEntry, RateFlags, encode_hello_ack},
        event::EventAck,
        stream::{Activity, Calibration, LidarCloud, Motion, Perception, Power},
    },
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
/// The enum is the stable seam between the link and the firmware.
#[derive(Debug)]
pub enum Inbound<'a> {
    /// A structurally valid command that the firmware validates, dispatches and acknowledges.
    Command(CommandRequest<'a>),
    /// The peer acknowledged one of the events published earlier.
    EventAck(EventAckNotice),
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
    ///
    /// `session_mismatch` is the `HELLO` major verdict; it answers a command with
    /// `VERSION_MISMATCH` even when the frame header itself carries this version's major.
    fn on_frame(&mut self, frame: Frame<'_>, session_mismatch: bool) {
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
        if session_mismatch || frame.header.major != PROTOCOL_MAJOR {
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

    /// Resets command dedupe for a new Session epoch.
    fn reset_epoch(&mut self, epoch: u32) {
        self.epoch = epoch;
        self.last_seq = 0;
        self.last_status = Status::Ok;
        self.inbound = InboundSlot::empty();
        self.ready = false;
        self.pending = None;
        self.stored = StoredAck::empty();
        self.stored_pending = false;
        self.auto_acks.clear();
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

/// The Session state a `HELLO` establishes.
///
/// The robot holds no Session object: this is only the link state the handshake leaves behind.
struct SessionState {
    /// Whether the most recent `HELLO` reported a different envelope major.
    mismatch: bool,
    /// The capabilities both peers support.
    caps: Caps,
    /// The Requested Rate per stream in centi-Hz, indexed by [`Stream::index`].
    requested_rate: [u16; Stream::COUNT],
    /// Whether the most recent `HELLO` contained an entry for the stream.
    requested_present: [bool; Stream::COUNT],
    /// Whether that entry set `ENABLED`.
    requested_enabled: [bool; Stream::COUNT],
    /// The epoch from the most recent accepted `HELLO`.
    epoch: u32,
}

impl SessionState {
    /// Creates the empty state of a link with no Session.
    const fn new() -> Self {
        Self {
            mismatch: false,
            caps: Caps::from_bits(0),
            requested_rate: [0; Stream::COUNT],
            requested_present: [false; Stream::COUNT],
            requested_enabled: [false; Stream::COUNT],
            epoch: 0,
        }
    }

    /// Forgets every Requested Rate while keeping the epoch and capability intersection.
    const fn clear_requested(&mut self) {
        self.requested_rate = [0; Stream::COUNT];
        self.requested_present = [false; Stream::COUNT];
        self.requested_enabled = [false; Stream::COUNT];
    }
}

/// The Peer view: the lease timestamp and the one-shot Deadman edge.
struct PeerTracker {
    /// Whether a peer is attached and its lease is live.
    state: PeerState,
    /// The time of the most recent valid PC-to-robot frame.
    last_frame_ms: Option<u64>,
    /// Whether the Deadman edge is waiting to be taken.
    lost_pending: bool,
    /// Whether the Deadman already fired in this attach period.
    deadman_fired: bool,
}

impl PeerTracker {
    /// Creates a detached tracker with no frame seen yet.
    const fn new() -> Self {
        Self {
            state: PeerState::NoPeer,
            last_frame_ms: None,
            lost_pending: false,
            deadman_fired: false,
        }
    }

    /// Records one valid PC-to-robot frame and attaches on the first frame of the period.
    const fn on_frame(&mut self, now_ms: u64) {
        self.last_frame_ms = Some(now_ms);
        match self.state {
            PeerState::NoPeer => {
                self.state = PeerState::PeerLive;
                self.deadman_fired = false;
            }
            PeerState::PeerLive => {}
        }
    }

    /// Returns `true` while the peer is attached and its lease is live.
    const fn is_live(&self) -> bool {
        matches!(self.state, PeerState::PeerLive)
    }

    /// Fires the one-shot edge at the Deadman threshold and expires the lease.
    ///
    /// A frame is always processed before this runs, so a reconnect renews the timestamp first and
    /// the elapsed silence is zero.
    const fn tick(&mut self, now_ms: u64, config: &Config) {
        let Some(last) = self.last_frame_ms else {
            return;
        };
        if !matches!(self.state, PeerState::PeerLive) {
            return;
        }
        let silence = now_ms.saturating_sub(last);
        if !self.deadman_fired && silence >= config.deadman_ms {
            self.lost_pending = true;
            self.deadman_fired = true;
        }
        if silence >= config.lease_ms {
            self.state = PeerState::NoPeer;
            self.last_frame_ms = None;
        }
    }
}

/// The largest encoded control reply: a `HELLO_ACK` with one default rate per stream.
const CONTROL_REPLY_BYTES: usize = HelloAck::HEADER_LEN + Stream::COUNT * RateEntry::ENCODED_LEN;

/// The number of control replies buffered before further requests in one chunk are dropped.
///
/// A conforming peer sends one control request at a time; dropping a reply is safe because a
/// `PING` or `HELLO` retry produces another one.
const CONTROL_REPLY_CAPACITY: usize = 4;

/// A queued `HELLO_ACK` or `PING` acknowledgement.
#[derive(Clone, Copy, Debug)]
struct ControlReply {
    /// The control id to mirror.
    id: u16,
    /// The sequence number to mirror.
    seq: u16,
    /// The number of valid bytes in `payload`.
    len: usize,
    /// The encoded reply payload.
    payload: [u8; CONTROL_REPLY_BYTES],
}

/// The outbound control replies, oldest first.
struct ControlState {
    /// The queued replies.
    replies: heapless::Vec<ControlReply, CONTROL_REPLY_CAPACITY>,
}

impl ControlState {
    /// Creates an empty queue.
    const fn new() -> Self {
        Self {
            replies: heapless::Vec::new(),
        }
    }

    /// Queues one reply, dropping it silently when the queue is full.
    fn push(&mut self, id: u16, seq: u16, payload: &[u8]) {
        if payload.len() > CONTROL_REPLY_BYTES {
            return;
        }
        let mut bytes = [0u8; CONTROL_REPLY_BYTES];
        bytes[..payload.len()].copy_from_slice(payload);
        let _ = self.replies.push(ControlReply {
            id,
            seq,
            len: payload.len(),
            payload: bytes,
        });
    }

    /// Encodes the oldest reply into `out` and removes it.
    fn pop(&mut self, out: &mut [u8]) -> Option<usize> {
        let reply = *self.replies.first()?;
        let frame = Frame::new(
            PROTOCOL_MAJOR,
            Class::Control,
            Flags::IS_ACK,
            reply.id,
            Seq::new(reply.seq),
            &reply.payload[..reply.len],
        )
        .ok()?;
        let written = frame.encode_into(out).ok()?;
        self.replies.remove(0);
        Some(written)
    }
}

/// Handles one inbound control frame.
///
/// `HELLO` and `PING` are the only controls the robot answers; an unimplemented id, an inbound
/// ACK and any control while the peer's major is mismatched are ignored.
fn handle_control(
    config: &Config,
    boot_id: u32,
    session: &mut SessionState,
    controls: &mut ControlState,
    commands: &mut CommandState,
    frame: Frame<'_>,
) {
    if frame.header.flags.contains(Flags::IS_ACK) {
        return;
    }
    let Some(control) = Control::from_wire(frame.header.id) else {
        return;
    };
    let seq = frame.header.seq.get();
    match control {
        Control::Hello => handle_hello(config, boot_id, session, controls, commands, frame.payload, seq),
        Control::Ping => {
            if session.mismatch || Ping::decode(frame.payload).is_err() {
                return;
            }
            let mut payload = [0u8; PingAck::ENCODED_LEN];
            let Ok(len) = (PingAck { status: Status::Ok }).encode_into(&mut payload) else {
                return;
            };
            controls.push(Control::Ping.wire_id(), seq, &payload[..len]);
        }
        Control::Throttle | Control::BridgeControl => {}
    }
}

/// Applies one `HELLO`, updates the Session and queues its `HELLO_ACK`.
fn handle_hello(
    config: &Config,
    boot_id: u32,
    session: &mut SessionState,
    controls: &mut ControlState,
    commands: &mut CommandState,
    payload: &[u8],
    seq: u16,
) {
    let Ok(hello) = Hello::decode(payload) else {
        return;
    };
    let caps = config.robot_caps.intersection(hello.caps);
    if hello.major != PROTOCOL_MAJOR {
        session.mismatch = true;
        session.caps = caps;
        session.clear_requested();
        queue_hello_ack(config, boot_id, controls, seq, Status::VersionMismatch, caps);
        return;
    }

    let mut requested_rate = [0u16; Stream::COUNT];
    let mut present = [false; Stream::COUNT];
    let mut enabled = [false; Stream::COUNT];
    for entry in hello.entries() {
        let Some(stream) = Stream::from_wire(entry.stream_id) else {
            continue;
        };
        let index = stream.index();
        if present[index] {
            continue;
        }
        if entry.flags.is_enabled() && entry.rate_chz == 0 {
            queue_hello_ack(config, boot_id, controls, seq, Status::BadArgument, caps);
            return;
        }
        present[index] = true;
        enabled[index] = entry.flags.is_enabled();
        requested_rate[index] = entry.rate_chz;
    }

    session.mismatch = false;
    session.caps = caps;
    session.requested_rate = requested_rate;
    session.requested_present = present;
    session.requested_enabled = enabled;
    if session.epoch != hello.epoch {
        commands.reset_epoch(hello.epoch);
        session.epoch = hello.epoch;
    }
    queue_hello_ack(config, boot_id, controls, seq, Status::Ok, caps);
}

/// Encodes and queues the robot's `HELLO_ACK` with the current defaults.
fn queue_hello_ack(config: &Config, boot_id: u32, controls: &mut ControlState, seq: u16, status: Status, caps: Caps) {
    let mut payload = [0u8; CONTROL_REPLY_BYTES];
    let entries = Stream::ALL.iter().map(|stream| RateEntry {
        stream_id: stream.wire_id(),
        rate_chz: config.default_rates_chz[stream.index()],
        flags: RateFlags::ENABLED,
    });
    let Ok(len) = encode_hello_ack(
        PROTOCOL_MAJOR,
        PROTOCOL_MINOR,
        caps,
        boot_id,
        status,
        entries,
        &mut payload,
    ) else {
        return;
    };
    controls.push(Control::Hello.wire_id(), seq, &payload[..len]);
}

/// The robot's link state and the composition point for the protocol mechanisms.
///
/// Construct it with the firmware's Boot Id and feed it bytes, time and samples. It never blocks,
/// never allocates and never holds an incoming frame past the call that produced it.
pub struct LinkCore {
    /// The timing configuration, robot capabilities and baseline rates.
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
    /// The Session a `HELLO` establishes.
    session: SessionState,
    /// The lease and the one-shot Deadman edge.
    peers: PeerTracker,
    /// Outbound `HELLO_ACK` and `PING` replies.
    controls: ControlState,
    /// The encoded event ring and its retransmission schedule.
    events: EventRing,
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
            session: SessionState::new(),
            peers: PeerTracker::new(),
            controls: ControlState::new(),
            events: EventRing::new(&config, boot_id),
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

    /// Returns the current command-dedupe epoch. Zero until a `HELLO` establishes one.
    #[must_use]
    pub const fn epoch(&self) -> u32 {
        self.commands.epoch
    }

    /// Returns the capabilities both peers supported in the most recent `HELLO`.
    #[must_use]
    pub const fn caps(&self) -> Caps {
        self.session.caps
    }

    /// Returns `true` while the most recent `HELLO` reported a different envelope major.
    ///
    /// A mismatched peer's streams and events are dropped, its control frames are ignored, and
    /// its commands are answered with `VERSION_MISMATCH`; the robot keeps streaming at baseline.
    #[must_use]
    pub const fn version_mismatch(&self) -> bool {
        self.session.mismatch
    }

    /// Returns the enabled Requested Rate for `stream`, or [`None`] when the most recent `HELLO`
    /// omitted it or disabled it.
    #[must_use]
    pub const fn requested_rate(&self, stream: Stream) -> Option<u16> {
        let index = stream.index();
        if self.session.requested_present[index] && self.session.requested_enabled[index] {
            Some(self.session.requested_rate[index])
        } else {
            None
        }
    }

    /// Returns `true` when the most recent `HELLO` requested `stream`.
    #[must_use]
    pub const fn stream_requested(&self, stream: Stream) -> bool {
        let index = stream.index();
        self.session.requested_present[index] && self.session.requested_enabled[index]
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

    /// Publishes one event into the encoded ring.
    ///
    /// Never blocks and never allocates. The event is sequenced immediately; a full ring drops its
    /// oldest entry and counts the loss, and the count is reported by a synthesised
    /// `EVENT_OVERFLOW` as soon as a slot is free. Nothing is transmitted until
    /// [`LinkCore::next_frame`] runs while a peer is attached, so an expired lease simply leaves
    /// the entry queued.
    pub fn publish_event(&mut self, message: &EventMessage<'_>, now_ms: u64) -> Publish {
        self.events.publish(message, now_ms)
    }

    /// Returns how many events are in the ring, retrying or exhausted.
    #[must_use]
    pub fn pending_events(&self) -> usize {
        self.events.len()
    }

    /// Returns how many events the ring has dropped since the last `EVENT_OVERFLOW` was queued.
    #[must_use]
    pub const fn dropped_events(&self) -> u16 {
        self.events.dropped()
    }

    /// Returns how many queued events have exhausted their schedule and await an ACK.
    #[must_use]
    pub fn exhausted_events(&self) -> usize {
        self.events.exhausted()
    }

    /// Iterates over the queued `(seq, event)` pairs, oldest first.
    pub fn queued_events(&self) -> impl Iterator<Item = (Seq, Event)> + '_ {
        self.events.iter()
    }

    /// Yields the one accepted command, or one buffered event ACK, if any.
    ///
    /// A command takes priority; nothing is yielded while an earlier polled command is
    /// unacknowledged, because the firmware must call [`LinkCore::ack_command`] first. An
    /// [`Inbound::EventAck`] borrows nothing and exists only so the firmware can log the peer's
    /// diagnostic status: the matching ring entry is already retired.
    pub fn poll(&mut self) -> Option<Inbound<'_>> {
        if self.commands.pending.is_some() {
            return None;
        }
        if !self.commands.ready {
            return self.events.pop_ack().map(Inbound::EventAck);
        }
        let state = &mut self.commands;
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

    /// Resets command dedupe for a new session epoch.
    ///
    /// The accepted sequence restarts at 0, so the next valid command is sequence 1. Commands
    /// before any `HELLO` already run under epoch zero. `HELLO` calls this when the epoch changes;
    /// a retransmitted `HELLO` on the same epoch keeps the dedupe state.
    pub fn reset_epoch(&mut self, epoch: u32) {
        self.commands.reset_epoch(epoch);
    }

    /// Emits the next frame into `out`, returning the number of bytes written.
    ///
    /// Priority is control before events before streams. The queued `HELLO_ACK`/`PING` replies go
    /// first, then the buffered auto-ACKs and the stored command ACK, then one ready event. Events
    /// are emitted only while the peer's lease is live. Pending streams follow in round-robin order.
    /// A frame that does not fit returns [`None`] without consuming the pending payload or
    /// advancing its sequence.
    pub fn next_frame(&mut self, out: &mut [u8]) -> Option<usize> {
        if let Some(written) = self.controls.pop(out) {
            return Some(written);
        }
        if let Some(written) = self.commands.pop_auto_ack(out) {
            return Some(written);
        }
        if let Some(written) = self.commands.pop_stored_ack(out) {
            return Some(written);
        }
        if self.peers.is_live()
            && let Some(written) = self.events.next_frame(out)
        {
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

    /// Advances the lease, Deadman and event retry timers to `now_ms`.
    ///
    /// A frame is processed before the tick, so a reconnect on the same epoch renews
    /// `last_peer_frame` first and this call sees no silence. The one-shot edge fires at exactly
    /// the Deadman threshold and the peer returns to [`PeerState::NoPeer`] at the lease timeout.
    /// Event schedules are only advanced while the lease is live.
    pub fn tick(&mut self, now_ms: u64) {
        self.peers.tick(now_ms, &self.config);
        self.events.tick(now_ms, self.peers.is_live());
    }

    /// Returns the current peer state.
    #[must_use]
    pub const fn peer(&self) -> PeerState {
        self.peers.state
    }

    /// Takes and clears the one-shot `peer_lost` edge.
    #[must_use]
    pub const fn take_peer_lost(&mut self) -> bool {
        let lost = self.peers.lost_pending;
        self.peers.lost_pending = false;
        lost
    }

    /// Resets the RX parser after a simulated Bridge reset.
    ///
    /// A Bridge reset is not a link power cycle: the Boot Id, command dedupe, event ring and
    /// lease survive, and only a partially assembled frame is discarded.
    pub const fn reset_parser(&mut self) {
        self.parser.reset();
    }

    /// Feeds PC-to-robot bytes through the parser.
    ///
    /// A command frame is validated and copied into the single inbound slot; a malformed, unknown,
    /// bad-flagged or stale command is auto-ACKed and never yielded. Unknown stream ids are counted
    /// and ignored. An event ACK retires the matching ring entry and queues a typed notice for
    /// [`LinkCore::poll`]. Any valid frame, including one for a mismatched major, renews the lease.
    /// The first frame after an expired lease re-attaches the peer and re-arms every surviving
    /// event's retry schedule.
    pub fn on_rx(&mut self, bytes: &[u8], now_ms: u64) {
        let now = u32::try_from(now_ms).unwrap_or(u32::MAX);
        let was_live = self.peers.is_live();
        let Self {
            config,
            boot_id,
            parser,
            commands,
            session,
            peers,
            controls,
            events,
            unknown_streams,
            ..
        } = self;
        let config: &Config = config;
        let mut unknown = 0u32;
        parser.feed(bytes, now, |frame| {
            // Any valid PC-to-robot frame renews the lease and re-attaches the peer before the
            // next tick is evaluated.
            peers.on_frame(now_ms);
            match frame.header.class {
                Class::Stream => {
                    if session.mismatch {
                        return;
                    }
                    if Stream::from_wire(frame.header.id).is_none() {
                        unknown = unknown.saturating_add(1);
                    }
                }
                Class::Command => commands.on_frame(frame, session.mismatch),
                Class::Event => {
                    if session.mismatch || !frame.header.flags.contains(Flags::IS_ACK) {
                        return;
                    }
                    let Some(event) = Event::from_wire(frame.header.id) else {
                        return;
                    };
                    let Ok(ack) = EventAck::decode(frame.payload) else {
                        return;
                    };
                    events.on_ack(event, frame.header.seq, ack.status);
                }
                Class::Control => handle_control(config, *boot_id, session, controls, commands, frame),
            }
        });
        *unknown_streams = unknown_streams.saturating_add(unknown);
        if !was_live && peers.is_live() {
            events.rearm(now_ms);
        }
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
