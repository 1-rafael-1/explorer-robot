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
//! Peer/Deadman, the reliable event ring, and the throttle/rate emission schedule. Commands are
//! accepted one at a time: [`LinkCore::poll`] yields a structurally valid, known, same-major,
//! in-order command and the firmware answers it with [`LinkCore::ack_command`] before polling
//! again. Auto-ACKs for unknown ids, malformed payloads, bad flags, version mismatches and stale
//! sequences never reach the firmware. `HELLO` and `PING` are answered inside the core. Events are
//! sequenced per Boot Id, retransmitted until the peer ACKs them and reported honestly on ring
//! overflow. Telemetry is emitted only when a sample is due under the most restrictive of the
//! firmware's request, the Session's Requested Rate (or the baseline) and the active `THROTTLE`
//! hints, with an on-change/heartbeat rule.

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
    Caps, Class, CodecError, Command, CommandMessage, Control, Direction, EncodeError, Event, EventMessage, Flags,
    Frame, PROTOCOL_MAJOR, PROTOCOL_MINOR, Seq, Status, Throttle, ThrottleState,
    codec::{PayloadCodec, StreamSample, decode_command},
    ids::Stream,
    parser::PcToRobotParser,
    payload::{
        control::{
            BridgeControl, BridgeControlAction, BridgeControlReply, Hello, HelloAck, Ping, PingAck, RateEntry,
            RateFlags, Throttle as ThrottleHint, decode_bridge_control_ack, encode_hello_ack,
        },
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
    /// The Bridge acknowledged the outstanding `BRIDGE_CONTROL` request.
    ControlReply(ControlReply<'a>),
    /// The outstanding `BRIDGE_CONTROL` request received no acknowledgement in time.
    ControlTimeout {
        /// The timed-out request's control sequence number.
        seq: u16,
    },
}

/// A `BRIDGE_CONTROL` acknowledgement surfaced by [`LinkCore::poll`].
///
/// The acknowledgement is paired to the request by `seq`; the wire payload does not repeat the
/// action. The link only carries the setup credentials or pairing code and never acts on them.
#[derive(Debug)]
pub struct ControlReply<'a> {
    /// The acknowledgement's sequence number, mirroring the request.
    pub seq: Seq,
    /// The typed reply, or the status a malformed acknowledgement maps to.
    pub result: Result<BridgeControlReply<'a>, Status>,
}

impl ControlReply<'_> {
    /// Returns the status the Bridge reported, or the status a malformed acknowledgement maps to.
    #[must_use]
    pub const fn status(&self) -> Status {
        match &self.result {
            Ok(reply) => reply.status(),
            Err(status) => *status,
        }
    }
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

/// The emission schedule of one stream's latest-wins slot.
#[derive(Clone, Copy, Debug, Default)]
struct StreamSchedule {
    /// Whether the stored sample differs from the one most recently emitted.
    changed: bool,
    /// The time the stored sample was most recently emitted, or [`None`] before the first send.
    last_emit_ms: Option<u64>,
    /// The fingerprint of the payload most recently emitted.
    last_emitted_hash: u32,
}

impl StreamSchedule {
    /// Creates a schedule for a stream that has never emitted.
    const fn new() -> Self {
        Self {
            changed: false,
            last_emit_ms: None,
            last_emitted_hash: 0,
        }
    }
}

/// Returns a deterministic 32-bit fingerprint of an encoded payload.
///
/// The schedule uses it to notice that a publish reproduces the sample that was last emitted, so
/// an unchanged sample waits for its heartbeat instead of being resent immediately. A collision
/// would only defer a change until the heartbeat; it never emits early.
fn payload_fingerprint(bytes: &[u8]) -> u32 {
    /// The FNV-1a 32-bit offset basis.
    const OFFSET_BASIS: u32 = 0x811C_9DC5;
    /// The FNV-1a 32-bit prime.
    const PRIME: u32 = 0x0100_0193;

    let mut hash = OFFSET_BASIS;
    for byte in bytes {
        hash ^= u32::from(*byte);
        hash = hash.wrapping_mul(PRIME);
    }
    hash
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
struct QueuedControlReply {
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
    replies: heapless::Vec<QueuedControlReply, CONTROL_REPLY_CAPACITY>,
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
        let _ = self.replies.push(QueuedControlReply {
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

/// The largest `BRIDGE_CONTROL` acknowledgement payload the robot accepts.
const BRIDGE_CONTROL_PAYLOAD_BYTES: usize = Direction::PcToRobot.max_payload();

/// An outbound `BRIDGE_CONTROL` request awaiting emission or an acknowledgement.
#[derive(Clone, Copy, Debug)]
struct PendingBridgeControl {
    /// The request's control sequence number.
    seq: u16,
    /// The action, kept so the acknowledgement can be decoded without an action echo.
    action: BridgeControlAction,
    /// Whether the request has been emitted.
    sent: bool,
    /// The timeout deadline, set when the request is first emitted.
    deadline_ms: Option<u64>,
}

/// A received `BRIDGE_CONTROL` acknowledgement copied out of the parser for `poll`.
struct InboundBridgeControl {
    /// The acknowledgement's sequence number.
    seq: u16,
    /// The action of the request it answers.
    action: BridgeControlAction,
    /// The number of valid bytes in `bytes`.
    len: usize,
    /// The copied acknowledgement payload.
    bytes: [u8; BRIDGE_CONTROL_PAYLOAD_BYTES],
}

impl InboundBridgeControl {
    /// Creates an empty slot.
    const fn empty() -> Self {
        Self {
            seq: 0,
            action: BridgeControlAction::ForgetNetwork,
            len: 0,
            bytes: [0; BRIDGE_CONTROL_PAYLOAD_BYTES],
        }
    }
}

/// The one outbound `BRIDGE_CONTROL` request and its acknowledgement or timeout.
///
/// A request is queued, emitted once at control priority and then either matched to an
/// acknowledgement by `seq` or timed out. It is never retransmitted, so a lost acknowledgement
/// cannot duplicate the action at the Bridge.
struct BridgeControlState {
    /// The next control sequence number; zero is skipped.
    next_seq: u16,
    /// The request in flight.
    outbound: Option<PendingBridgeControl>,
    /// The acknowledgement copied for `poll`.
    inbound: InboundBridgeControl,
    /// Whether `inbound` holds an unread acknowledgement.
    inbound_ready: bool,
    /// A timed-out request sequence waiting for `poll`.
    timeout: Option<u16>,
}

impl BridgeControlState {
    /// Creates the empty state with the control sequence at one.
    const fn new() -> Self {
        Self {
            next_seq: 1,
            outbound: None,
            inbound: InboundBridgeControl::empty(),
            inbound_ready: false,
            timeout: None,
        }
    }

    /// Allocates the next non-zero control sequence number.
    const fn allocate_seq(&mut self) -> u16 {
        let seq = self.next_seq;
        let next = seq.wrapping_add(1);
        self.next_seq = if next == 0 { 1 } else { next };
        seq
    }

    /// Queues one request, returning its sequence number or a rejection status.
    const fn request(&mut self, action_byte: u8, _now_ms: u64) -> Result<u16, Status> {
        if self.outbound.is_some() || self.inbound_ready || self.timeout.is_some() {
            return Err(Status::Busy);
        }
        let Some(action) = BridgeControlAction::from_wire(action_byte) else {
            return Err(Status::BadArgument);
        };
        let seq = self.allocate_seq();
        self.outbound = Some(PendingBridgeControl {
            seq,
            action,
            sent: false,
            deadline_ms: None,
        });
        Ok(seq)
    }

    /// Encodes the queued request into `out` if it has not been emitted yet.
    fn next_frame(&mut self, out: &mut [u8], now_ms: u64, timeout_ms: u64) -> Option<usize> {
        let pending = self.outbound.as_mut()?;
        if pending.sent {
            return None;
        }
        let request = BridgeControl { action: pending.action };
        let mut payload = [0u8; BridgeControl::ENCODED_LEN];
        let len = request.encode_into(&mut payload).ok()?;
        let frame = Frame::new(
            PROTOCOL_MAJOR,
            Class::Control,
            Flags::ACK_REQUESTED,
            Control::BridgeControl.wire_id(),
            Seq::new(pending.seq),
            &payload[..len],
        )
        .ok()?;
        let written = frame.encode_into(out).ok()?;
        pending.sent = true;
        pending.deadline_ms = Some(now_ms.saturating_add(timeout_ms));
        Some(written)
    }

    /// Copies an acknowledgement whose sequence matches the outstanding request.
    ///
    /// A mismatched sequence, an acknowledgement with no outstanding request, or a payload above
    /// the PC-to-robot cap is ignored.
    fn on_ack(&mut self, seq: u16, payload: &[u8]) {
        let Some(pending) = self.outbound else {
            return;
        };
        if pending.seq != seq || payload.len() > BRIDGE_CONTROL_PAYLOAD_BYTES {
            return;
        }
        self.inbound.seq = pending.seq;
        self.inbound.action = pending.action;
        self.inbound.len = payload.len();
        self.inbound.bytes[..payload.len()].copy_from_slice(payload);
        self.inbound_ready = true;
        self.outbound = None;
    }

    /// Marks an emitted request as timed out once its deadline passes.
    fn tick(&mut self, now_ms: u64) {
        let Some(pending) = self.outbound else {
            return;
        };
        let Some(deadline_ms) = pending.deadline_ms else {
            return;
        };
        if now_ms < deadline_ms {
            return;
        }
        self.outbound = None;
        self.timeout.get_or_insert(pending.seq);
    }

    /// Decodes the buffered acknowledgement and clears its ready flag.
    fn pop_reply(&mut self) -> Option<ControlReply<'_>> {
        if !self.inbound_ready {
            return None;
        }
        self.inbound_ready = false;
        let slot = &self.inbound;
        let result = decode_bridge_control_ack(slot.action, &slot.bytes[..slot.len]).map_err(CodecError::status);
        Some(ControlReply {
            seq: Seq::new(slot.seq),
            result,
        })
    }

    /// Takes the timed-out request sequence, if one is waiting.
    const fn pop_timeout(&mut self) -> Option<u16> {
        self.timeout.take()
    }
}

/// Handles one inbound `HELLO` or `PING` control frame.
///
/// `HELLO` and `PING` are the only controls the robot answers; an inbound ACK is ignored. An
/// unimplemented id and any control while the peer's major is mismatched are ignored. `THROTTLE`
/// is applied by [`LinkCore::on_rx`] before this runs because it has no reply.
fn handle_control(
    config: &Config,
    boot_id: u32,
    session: &mut SessionState,
    controls: &mut ControlState,
    commands: &mut CommandState,
    frame: Frame<'_>,
) {
    let Some(control) = Control::from_wire(frame.header.id) else {
        return;
    };
    let seq = frame.header.seq.get();
    match control {
        Control::Hello => {
            if frame.header.flags.contains(Flags::IS_ACK) {
                return;
            }
            handle_hello(config, boot_id, session, controls, commands, frame.payload, seq);
        }
        Control::Ping => {
            if session.mismatch || frame.header.flags.contains(Flags::IS_ACK) {
                return;
            }
            if Ping::decode(frame.payload).is_err() {
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
    /// The per-stream emission schedule, indexed by [`Stream::index`].
    schedule: [StreamSchedule; Stream::COUNT],
    /// The firmware's preferred rate ceiling per stream in centi-Hz; zero means no extra ceiling.
    firmware_rate_chz: [u16; Stream::COUNT],
    /// One sequence counter per stream, incremented when a frame is emitted.
    sequences: [u16; Stream::COUNT],
    /// The bitmap of streams with a valid sample, indexed by [`Stream::index`].
    has_sample: u8,
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
    /// The outbound `BRIDGE_CONTROL` request and its acknowledgement or timeout.
    bridge: BridgeControlState,
    /// The encoded event ring and its retransmission schedule.
    events: EventRing,
    /// The merged advisory `THROTTLE` hints.
    throttle: Throttle,
    /// The number of malformed `THROTTLE` payloads ignored so far.
    invalid_throttles: u32,
    /// The most recent time observed through [`LinkCore::publish_stream`], [`LinkCore::tick`] or
    /// [`LinkCore::on_rx`].
    now_ms: u64,
}

impl LinkCore {
    /// Creates a link core with `config`, the firmware's `boot_id` and the current time.
    #[must_use]
    pub fn new(config: Config, boot_id: u32, now_ms: u64) -> Self {
        let mut parser = PcToRobotParser::new(Direction::PcToRobot, timing::BAUD);
        parser.set_timeout_ms(config.frame_timeout_ms);
        Self {
            config,
            boot_id,
            parser,
            slots: StreamSlots::new(),
            schedule: [StreamSchedule::new(); Stream::COUNT],
            firmware_rate_chz: [0; Stream::COUNT],
            sequences: [0; Stream::COUNT],
            has_sample: 0,
            cursor: 0,
            unknown_streams: 0,
            commands: CommandState::new(),
            session: SessionState::new(),
            peers: PeerTracker::new(),
            controls: ControlState::new(),
            bridge: BridgeControlState::new(),
            events: EventRing::new(&config, boot_id),
            throttle: Throttle::new(),
            invalid_throttles: 0,
            now_ms,
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

    /// Returns how many malformed `THROTTLE` payloads have been ignored.
    ///
    /// A malformed hint is dropped without touching the schedule; the peer's next hint or the
    /// hint's own repetition repairs the state.
    #[must_use]
    pub const fn invalid_throttles(&self) -> u32 {
        self.invalid_throttles
    }

    /// Publishes one telemetry sample into its per-stream latest-wins slot.
    ///
    /// The sample is encoded immediately; a newer publish replaces the pending one, so there is no
    /// queue and a coalesced sample is never retransmitted. The schedule records whether the
    /// encoded payload differs from the sample most recently emitted, which is what lets an
    /// unchanged sample wait for its heartbeat.
    ///
    /// `requested_rate` is the firmware's preferred ceiling in centi-Hz. A non-zero value takes
    /// part in the most-restrictive merge with the Session's Requested Rate (or the configured
    /// baseline) and the active `Throttle`; zero means the firmware has no preference.
    pub fn publish_stream(&mut self, sample: &StreamSample<'_>, requested_rate: u32, now_ms: u64) {
        self.now_ms = now_ms;
        let stream = sample.stream();
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
        self.set_has_sample(stream, stored);
        if !stored {
            return;
        }

        let index = stream.index();
        self.firmware_rate_chz[index] = u16::try_from(requested_rate).unwrap_or(u16::MAX);
        let fingerprint = payload_fingerprint(self.slots.payload(stream));
        let schedule = &mut self.schedule[index];
        schedule.changed = match schedule.last_emit_ms {
            Some(_) => fingerprint != schedule.last_emitted_hash,
            None => true,
        };
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

    /// Queues one `BRIDGE_CONTROL` request and returns its control sequence number.
    ///
    /// One request is in flight at a time. A request while another is queued, awaiting its
    /// acknowledgement or its timeout notice is rejected with [`Status::Busy`]; an action outside
    /// `0..=2` is rejected with [`Status::BadArgument`]. The request is emitted by
    /// [`LinkCore::next_frame`] at control priority, is never retransmitted, and times out after
    /// [`Config::control_timeout_ms`] once it has been emitted.
    ///
    /// The link only carries the Bridge's credentials or pairing code; it never logs, stores
    /// beyond the pending reply, or acts on them.
    ///
    /// # Errors
    ///
    /// Returns [`Status::BadArgument`] for an unknown action and [`Status::Busy`] while an earlier
    /// request or its unread result is outstanding.
    pub const fn request_bridge_control(&mut self, action: u8, now_ms: u64) -> Result<u16, Status> {
        self.now_ms = now_ms;
        self.bridge.request(action, now_ms)
    }

    /// Yields the one accepted command, one `BRIDGE_CONTROL` result, or one buffered event ACK.
    ///
    /// A command takes priority; nothing is yielded while an earlier polled command is
    /// unacknowledged, because the firmware must call [`LinkCore::ack_command`] first. A
    /// `BRIDGE_CONTROL` reply or timeout follows the command seam and precedes an event ACK. An
    /// [`Inbound::EventAck`] borrows nothing and exists only so the firmware can log the peer's
    /// diagnostic status: the matching ring entry is already retired.
    pub fn poll(&mut self) -> Option<Inbound<'_>> {
        if self.commands.pending.is_some() {
            return None;
        }
        if self.commands.ready {
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
            return Some(Inbound::Command(CommandRequest {
                seq: Seq::new(slot.seq),
                flags: slot.flags,
                command: slot.command,
                message,
            }));
        }
        // A reply and a timeout are mutually exclusive, so their order does not matter; the
        // owned timeout is taken first so the reply's borrow does not overlap this borrow.
        if let Some(seq) = self.bridge.pop_timeout() {
            return Some(Inbound::ControlTimeout { seq });
        }
        if let Some(reply) = self.bridge.pop_reply() {
            return Some(Inbound::ControlReply(reply));
        }
        self.events.pop_ack().map(Inbound::EventAck)
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
    /// first, then the outbound `BRIDGE_CONTROL` request, then the buffered auto-ACKs and the
    /// stored command ACK, then one ready event. Events are emitted only while the peer's lease is
    /// live. Streams follow in round-robin order, but only a stream whose sample is due at the most
    /// recent observed time is emitted. A frame that does not fit returns [`None`] without
    /// consuming the pending payload or advancing its sequence.
    pub fn next_frame(&mut self, out: &mut [u8]) -> Option<usize> {
        if let Some(written) = self.controls.pop(out) {
            return Some(written);
        }
        let now_ms = self.now_ms;
        let timeout_ms = self.config.control_timeout_ms;
        if let Some(written) = self.bridge.next_frame(out, now_ms, timeout_ms) {
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

    /// Emits the next due stream frame, round-robin between the slots.
    fn next_stream_frame(&mut self, out: &mut [u8]) -> Option<usize> {
        for offset in 0..Stream::COUNT {
            let index = (self.cursor + offset) % Stream::COUNT;
            if !self.stream_due(index) {
                continue;
            }
            let stream = Stream::ALL[index];
            let written = {
                let payload = self.slots.payload(stream);
                let frame = Frame::new(
                    PROTOCOL_MAJOR,
                    Class::Stream,
                    Flags::default(),
                    stream.wire_id(),
                    Seq::new(self.sequences[index]),
                    payload,
                )
                .ok()?;
                frame.encode_into(out).ok()?
            };
            let fingerprint = payload_fingerprint(self.slots.payload(stream));
            self.sequences[index] = self.sequences[index].wrapping_add(1);
            let schedule = &mut self.schedule[index];
            schedule.last_emit_ms = Some(self.now_ms);
            schedule.last_emitted_hash = fingerprint;
            schedule.changed = false;
            self.cursor = (index + 1) % Stream::COUNT;
            return Some(written);
        }
        None
    }

    /// Returns `true` when the sample in `index`'s slot is due for emission at `now_ms`.
    ///
    /// A changed sample waits for the effective rate's minimum spacing. An unchanged sample waits
    /// for its heartbeat as well, so a steady stream still refreshes the peer. A sample that has
    /// never been emitted is due immediately.
    fn stream_due(&self, index: usize) -> bool {
        if self.has_sample & (1u8 << index) == 0 {
            return false;
        }
        let Some(base) = self.emission_rate_chz(index) else {
            return false;
        };
        let stream = Stream::ALL[index];
        let rate = match self.throttle.effective(stream, self.now_ms) {
            ThrottleState::Paused => return false,
            ThrottleState::Limited(limit) => base.min(limit),
            ThrottleState::Unrestricted => base,
        };
        if rate == 0 {
            return false;
        }
        let min_spacing_ms = 10_000u64.div_ceil(u64::from(rate));
        let schedule = &self.schedule[index];
        let Some(last_emit_ms) = schedule.last_emit_ms else {
            return true;
        };
        let elapsed_ms = self.now_ms.saturating_sub(last_emit_ms);
        let wait_ms = if schedule.changed {
            min_spacing_ms
        } else {
            min_spacing_ms.max(self.config.heartbeat_ms[index])
        };
        elapsed_ms >= wait_ms
    }

    /// Returns the most restrictive of the firmware's request, the HELLO Requested Rate (or the
    /// configured baseline when the Session omitted the stream).
    ///
    /// [`None`] means the Session disabled the stream, so it is suppressed entirely.
    fn emission_rate_chz(&self, index: usize) -> Option<u16> {
        let base = if self.session.requested_present[index] {
            if !self.session.requested_enabled[index] {
                return None;
            }
            self.session.requested_rate[index]
        } else {
            self.config.default_rates_chz[index]
        };
        let firmware = self.firmware_rate_chz[index];
        Some(if firmware == 0 { base } else { base.min(firmware) })
    }

    /// Advances the lease, Deadman, event retry timers and `THROTTLE` expiry to `now_ms`.
    ///
    /// A frame is processed before the tick, so a reconnect on the same epoch renews
    /// `last_peer_frame` first and this call sees no silence. The one-shot edge fires at exactly
    /// the Deadman threshold and the peer returns to [`PeerState::NoPeer`] at the lease timeout.
    /// Event schedules are only advanced while the lease is live. Expired `THROTTLE` hints are
    /// forgotten here, so emission reverts to the Requested Rate or the baseline; it never
    /// reverts to zero.
    pub fn tick(&mut self, now_ms: u64) {
        self.now_ms = now_ms;
        self.throttle.tick(now_ms);
        self.bridge.tick(now_ms);
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
    /// [`LinkCore::poll`]. A `BRIDGE_CONTROL` acknowledgement is paired to the outstanding request
    /// by `seq`; a mismatched or unsolicited one is ignored. Any valid frame, including one for a
    /// mismatched major, renews the lease. The first frame after an expired lease re-attaches the
    /// peer and re-arms every surviving event's retry schedule.
    pub fn on_rx(&mut self, bytes: &[u8], now_ms: u64) {
        self.now_ms = now_ms;
        self.throttle.tick(now_ms);
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
            bridge,
            events,
            unknown_streams,
            throttle,
            invalid_throttles,
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
                Class::Control => {
                    if frame.header.id == Control::Throttle.wire_id() {
                        if session.mismatch {
                            return;
                        }
                        // THROTTLE has no ACK, so IS_ACK is ignored rather than rejecting it.
                        let Ok(hint) = ThrottleHint::decode(frame.payload) else {
                            *invalid_throttles = (*invalid_throttles).saturating_add(1);
                            return;
                        };
                        let _ = throttle.apply(&hint, now_ms);
                    } else if frame.header.id == Control::BridgeControl.wire_id() {
                        // The robot is the sender of BRIDGE_CONTROL; only the Bridge's ACK is
                        // meaningful here. An inbound request is ignored.
                        if frame.header.flags.contains(Flags::IS_ACK) {
                            bridge.on_ack(frame.header.seq.get(), frame.payload);
                        }
                    } else {
                        handle_control(config, *boot_id, session, controls, commands, frame);
                    }
                }
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

    /// Sets or clears `stream`'s "has a sample" bit after a publish attempt.
    const fn set_has_sample(&mut self, stream: Stream, has_sample: bool) {
        let bit = 1u8 << stream.index();
        if has_sample {
            self.has_sample |= bit;
        } else {
            self.has_sample &= !bit;
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
