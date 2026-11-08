//! A daemon-shaped sender harness for the command retry tests.
//!
//! This models the daemon's retry loop without implementing the daemon. It composes the public
//! [`AckTracker`] with [`sim::Wire`], sends command frames, optionally ignores the robot's ACK,
//! and records every transmission and the final outcome. It is test-only and uses `std`.

use remote_protocol::{
    AckOutcome, AckTracker, Class, Direction, Flags, Frame, MAX_PC_TO_ROBOT_FRAME, PROTOCOL_MAJOR, RetryPolicy, Seq,
    Status, ids::Command, sim::Wire,
};

/// One command frame the harness put on the wire.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transmission {
    /// The injected time of the transmission, in milliseconds.
    pub at_ms: u64,
    /// The sequence carried by the frame; retransmissions keep the original.
    pub seq: Seq,
    /// The frame's flags; a retransmission sets `RETRY` as well as `ACK_REQUESTED`.
    pub flags: Flags,
}

/// A daemon-shaped sender that keeps one command in flight over a [`Wire`].
pub struct Harness {
    /// The shared ACK tracker, configured with the caller's policy.
    tracker: AckTracker,
    /// The command id placed in every transmission.
    command: Command,
    /// The command's assigned sequence.
    seq: Seq,
    /// The Boot Id the command was sent under.
    boot_id: u32,
    /// The payload bytes placed in every transmission.
    payload: Vec<u8>,
    /// Every transmission in order.
    transmissions: Vec<Transmission>,
    /// The terminal outcome reported by the tracker, if any.
    outcome: Option<AckOutcome>,
    /// When true, a delivered ACK is discarded to model a lost ACK.
    drop_ack: bool,
}

impl Harness {
    /// Creates a harness for `command` with the given policy, sequence and Boot Id.
    #[must_use]
    pub fn new(policy: RetryPolicy, command: Command, seq: Seq, boot_id: u32, payload: &[u8]) -> Self {
        Self {
            tracker: AckTracker::new(policy),
            command,
            seq,
            boot_id,
            payload: payload.to_vec(),
            transmissions: Vec::new(),
            outcome: None,
            drop_ack: false,
        }
    }

    /// Sends the initial command at `now_ms`, returning whether it went out.
    pub fn start(&mut self, wire: &mut Wire, now_ms: u64) -> bool {
        if !self.tracker.begin(self.seq, self.boot_id, now_ms) {
            return false;
        }
        self.send(wire, now_ms, Flags::ACK_REQUESTED);
        true
    }

    /// Sets whether a delivered ACK is discarded, modelling a lost ACK.
    pub const fn set_drop_ack(&mut self, drop_ack: bool) {
        self.drop_ack = drop_ack;
    }

    /// Tells the harness that the robot reported `boot_id`.
    ///
    /// A change discards the in-flight command and returns the explicit do-not-retry outcome.
    pub const fn on_boot_id(&mut self, boot_id: u32) -> Option<AckOutcome> {
        self.boot_id = boot_id;
        self.tracker.on_boot_id(boot_id)
    }

    /// Puts a robot ACK for the tracked command on the wire.
    ///
    /// Returns `false` when the frame cannot be encoded.
    pub fn acknowledge(&self, wire: &mut Wire) -> bool {
        let payload = [Status::Ok.bits()];
        let Ok(frame) = Frame::new(
            PROTOCOL_MAJOR,
            Class::Command,
            Flags::IS_ACK,
            self.command.wire_id(),
            self.seq,
            &payload,
        ) else {
            return false;
        };
        let mut bytes = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let Ok(written) = frame.encode_into(&mut bytes) else {
            return false;
        };
        wire.send(Direction::RobotToPc, &bytes[..written]);
        true
    }

    /// Advances the daemon one step at `now_ms`.
    ///
    /// First drains any robot ACK unless it is being dropped, then polls the tracker and writes a
    /// retransmission when due. Returns the tracker's action.
    pub fn tick(&mut self, wire: &mut Wire, now_ms: u64) -> AckOutcome {
        self.receive_acks(wire);
        let outcome = self.tracker.poll(now_ms);
        match outcome {
            AckOutcome::Retransmit { .. } => {
                self.send(wire, now_ms, Flags::ACK_REQUESTED | Flags::RETRY);
            }
            AckOutcome::Indeterminate { .. } | AckOutcome::DiscardedOnBootChange { .. } => {
                self.outcome = Some(outcome);
            }
            AckOutcome::Idle => {}
        }
        outcome
    }

    /// Drains robot-to-PC ACK frames, honours the drop setting and retires a matching command.
    fn receive_acks(&mut self, wire: &mut Wire) {
        let bytes = wire.take(Direction::RobotToPc);
        let mut offset = 0usize;
        while offset < bytes.len() {
            let Ok(frame) = Frame::decode(&bytes[offset..], Direction::RobotToPc) else {
                break;
            };
            let consumed = frame.encoded_len();
            if frame.header.class == Class::Command
                && frame.header.flags.contains(Flags::IS_ACK)
                && !self.drop_ack
                && frame.header.seq == self.seq
            {
                let _ = self.tracker.on_ack(self.seq);
            }
            offset += consumed;
        }
    }

    /// Encodes and sends one command frame, recording it.
    fn send(&mut self, wire: &mut Wire, now_ms: u64, flags: Flags) {
        let Ok(frame) = Frame::new(
            PROTOCOL_MAJOR,
            Class::Command,
            flags,
            self.command.wire_id(),
            self.seq,
            &self.payload,
        ) else {
            return;
        };
        let mut bytes = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let Ok(written) = frame.encode_into(&mut bytes) else {
            return;
        };
        wire.send(Direction::PcToRobot, &bytes[..written]);
        self.transmissions.push(Transmission {
            at_ms: now_ms,
            seq: self.seq,
            flags,
        });
    }

    /// Returns every transmission in order.
    #[must_use]
    pub fn transmissions(&self) -> &[Transmission] {
        &self.transmissions
    }

    /// Returns the terminal outcome, if the tracker reached one.
    #[must_use]
    pub const fn outcome(&self) -> Option<AckOutcome> {
        self.outcome
    }

    /// Returns `true` while the tracker still holds the command.
    #[must_use]
    pub const fn is_in_flight(&self) -> bool {
        self.tracker.is_in_flight()
    }
}
