//! The Robot Stub: the robot's real `LinkCore` behind a serial port.
//!
//! The stub replaces the robot on the Bridge's bench. It owns no wire semantics of its own:
//! [`link_core::LinkCore`] answers `HELLO` and `PING`, deduplicates commands, re-ACKs the last
//! sequence without re-executing, answers older ones with `STALE` and retries unacknowledged
//! events. The personas only configure rates and load around that core, and the fault knobs
//! drop, delay or corrupt bytes on the serial side, on demand.
//!
//! Everything crosses [`transport::Transport`], the serial seam: an in-memory duplex channel in
//! the tests, a real serial port in the binary. The library never depends on a port library; the
//! binary does.
//!
//! The stub is driven with plain millisecond timestamps through [`Stub::step`], so tests run on a
//! simulated clock and the binary feeds the wall clock.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod transport;

use std::{collections::VecDeque, string::String, vec::Vec};

pub use link_core::timing::Config;
use link_core::{Inbound, LinkCore, PeerState};
use link_protocol::{
    Command, CommandMessage, CompletionStatus, EventMessage, MAX_ROBOT_TO_PC_FRAME, Status, StreamSample,
    ThrottleState,
    ids::{ProcedureId, Stream},
    payload::{
        control::BridgeControlReply,
        event::{DriveDistanceCompleted, RotateCompleted},
        stream::{Activity, ActivityStage, Calibration, CalibrationStatus, LidarCloud, Motion, Perception, Power},
    },
};
pub use transport::{Duplex, Transport, duplex};

/// The robot's 10 Hz sampling tick (ADR-0032); the personas publish their samples this often.
const SAMPLING_TICK_MS: u64 = 100;

/// The worst-case persona's event pacing interval.
///
/// ADR-0033's budget assumes eight ring entries retried four times a second — 1,856 B/s — which
/// was priced with the 250 ms retry placeholder. The shipped retry constant is 500 ms, so honest
/// retries alone carry half that load; the persona therefore paces fresh publications of the
/// catalogue's largest event at 40 Hz, which sustains the budget's event bytes without inventing
/// wire behaviour: `LinkCore` still owns sequencing, the ring and the drop accounting.
const WORST_EVENT_INTERVAL_MS: u64 = 25;

/// The read chunk the stub drains from the transport on every step.
const RX_CHUNK_BYTES: usize = 1024;

/// The number of executions kept in the stub's log.
const EXECUTION_LOG: usize = 32;

/// The 32-byte detail the worst-case persona pads its `ACTIVITY` and completion payloads with,
/// matching ADR-0033's assumed maximum frame sizes.
const WORST_DETAIL: &str = "0123456789abcdef0123456789abcdef";

/// The stub's load profile, selected by the binary's `--persona`.
///
/// A persona never changes the wire semantics; it configures what the stub samples, how fast it
/// samples and how much event traffic it publishes. The fault knobs compose with every persona.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Persona {
    /// Streams a moving robot at the negotiated or baseline rates, answers `HELLO`/`PING`,
    /// ACKs commands with the robot's rules and publishes a completion event for each executed
    /// drive command.
    #[default]
    Baseline,
    /// Sustains ADR-0033's worst-case budget: every stream changed at the 10 Hz sampling tick
    /// plus a paced firehose of the catalogue's largest event. Requires [`worst_case_config`].
    WorstCase,
    /// The baseline profile; the bench selects it to exercise the `BRIDGE_CONTROL` path. The
    /// request API is available under every persona.
    BridgeControl,
}

/// The timing configuration [`Persona::WorstCase`] needs.
///
/// Every stream's baseline rate is raised to the 10 Hz sampling tick, matching ADR-0033's "every
/// change-based stream emitted at the 10 Hz sampling tick".
#[must_use]
pub fn worst_case_config() -> Config {
    Config {
        default_rates_chz: [1_000; Stream::COUNT],
        ..Config::default()
    }
}

/// The induced faults on the stub's serial side, on demand.
///
/// The knobs act on the bytes the stub writes: `drop_every` drops every n-th outbound byte,
/// `corrupt_every` flips every n-th outbound byte, and `delay_ms` holds outbound bytes for the
/// given milliseconds before they cross the seam. Zero disables a knob. The byte counters run
/// across frames, like the wire's own faults. The knobs never touch inbound bytes: the point is
/// what the Bridge's parser receives, and the stub's own parser is exercised by the peer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FaultKnobs {
    /// Drop every n-th outbound byte; zero disables dropping.
    pub drop_every: u32,
    /// Flip every n-th outbound byte; zero disables corruption.
    pub corrupt_every: u32,
    /// Hold outbound bytes for this many milliseconds; zero disables delay.
    pub delay_ms: u64,
}

/// An owned copy of the Bridge's `BRIDGE_CONTROL` reply.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OwnedReply {
    /// The setup access point's SSID and passphrase (action 0).
    Setup {
        /// The reply status.
        status: Status,
        /// The setup SSID.
        ssid: String,
        /// The build-fixed setup passphrase.
        passphrase: String,
    },
    /// The twelve-character pairing code (actions 1 and 2).
    PairingCode {
        /// The reply status.
        status: Status,
        /// The pairing code.
        code: String,
    },
    /// The acknowledgement did not decode; the status it mapped to.
    Malformed {
        /// The status the malformed acknowledgement maps to.
        status: Status,
    },
}

/// What the stub knows about one `BRIDGE_CONTROL` request: the Bridge's reply, or that none came
/// before the configured timeout.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BridgeControlReport {
    /// The Bridge acknowledged the request.
    Replied {
        /// The requested action byte.
        action: u8,
        /// The request's control sequence number.
        seq: u16,
        /// The Bridge's reply.
        reply: OwnedReply,
    },
    /// No acknowledgement arrived before the configured control timeout.
    TimedOut {
        /// The requested action byte.
        action: u8,
        /// The request's control sequence number.
        seq: u16,
    },
}

/// One executed command, as the stub's log records it for the binary's output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Execution {
    /// The command's sequence number.
    pub seq: u16,
    /// The command id.
    pub command: Command,
    /// The status the stub answered with.
    pub status: Status,
}

/// The drive a completion event reports, extracted from a polled command before the poll's
/// borrow of the core ends.
#[derive(Clone, Copy)]
enum Completion {
    /// A `DRIVE_DISTANCE` target in centimetres.
    Distance {
        /// The requested distance.
        target_cm: i16,
    },
    /// A `ROTATE_BY` target in centidegrees.
    Rotation {
        /// The requested angle.
        target_cdeg: i16,
    },
}

/// One of the core's polled messages, converted to owned data.
enum Served {
    /// An accepted command awaiting the stub's acknowledgement.
    Command {
        /// The command's sequence number.
        seq: u16,
        /// The command id.
        command: Command,
        /// The drive completion the persona publishes once the command is acknowledged.
        completion: Option<Completion>,
    },
    /// The Bridge's acknowledgement of the outstanding `BRIDGE_CONTROL` request.
    Reply {
        /// The request's control sequence number.
        seq: u16,
        /// The owned reply.
        reply: OwnedReply,
    },
    /// The outstanding `BRIDGE_CONTROL` request timed out.
    Timeout {
        /// The request's control sequence number.
        seq: u16,
    },
}

/// A `BRIDGE_CONTROL` request the stub has queued and not yet reported on.
struct Requested {
    /// The request's control sequence number.
    seq: u16,
    /// The requested action byte.
    action: u8,
}

/// The robot's bench persona: the real `LinkCore` behind a transport, plus load and faults.
///
/// Construct it with the robot's timing configuration, a Boot Id, a load persona and a transport,
/// then drive [`Stub::step`] with plain milliseconds. The stub never blocks, never reads the
/// wall clock and never touches the transport outside a step.
pub struct Stub<T> {
    /// The robot's real link state and wire semantics.
    core: LinkCore,
    /// The serial seam.
    transport: T,
    /// The load profile.
    persona: Persona,
    /// The active fault knobs.
    faults: FaultKnobs,
    /// The outbound byte counter the drop and corrupt knobs pace against.
    outbound_bytes: u64,
    /// Outbound chunks held by the delay knob, with the millisecond each becomes due.
    delayed: Vec<(u64, Vec<u8>)>,
    /// The next nominal sampling tick, or [`None`] before the first step.
    next_sample_ms: Option<u64>,
    /// The sampling tick counter, varying the generated samples.
    sample_tick: u32,
    /// The next nominal worst-case event publication, or [`None`] before the first step.
    next_event_ms: Option<u64>,
    /// The worst-case event counter.
    event_tick: u32,
    /// The stub's bounded execution log, oldest first.
    executions: VecDeque<Execution>,
    /// The `BRIDGE_CONTROL` requests awaiting their reply or timeout.
    requests: Vec<Requested>,
    /// The reports the caller has not taken yet, oldest first.
    reports: VecDeque<BridgeControlReport>,
}

impl<T> Stub<T>
where
    T: Transport,
{
    /// Creates a stub with the robot's `config`, `boot_id`, load `persona` and `transport`.
    #[must_use]
    pub fn new(config: Config, boot_id: u32, persona: Persona, transport: T) -> Self {
        Self {
            core: LinkCore::new(config, boot_id, 0),
            transport,
            persona,
            faults: FaultKnobs::default(),
            outbound_bytes: 0,
            delayed: Vec::new(),
            next_sample_ms: None,
            sample_tick: 0,
            next_event_ms: None,
            event_tick: 0,
            executions: VecDeque::new(),
            requests: Vec::new(),
            reports: VecDeque::new(),
        }
    }

    /// Advances the stub to `now_ms`: drains the transport, services the core, publishes the
    /// persona's load and flushes every frame the core has ready.
    ///
    /// One call is one iteration of the firmware's link loop: inbound bytes first, then the
    /// command seam, then sampling, then the schedule tick, then transmission. Nothing blocks;
    /// the caller owns time.
    pub fn step(&mut self, now_ms: u64) {
        self.drain_transport(now_ms);
        self.service_core(now_ms);
        self.publish_load(now_ms);
        self.core.tick(now_ms);
        self.flush_delayed(now_ms);
        self.emit(now_ms);
    }

    /// Returns the persona the stub runs.
    #[must_use]
    pub const fn persona(&self) -> Persona {
        self.persona
    }

    /// Returns the active fault knobs.
    #[must_use]
    pub const fn faults(&self) -> FaultKnobs {
        self.faults
    }

    /// Replaces the fault knobs; they take effect on the next [`Stub::step`].
    pub const fn set_faults(&mut self, faults: FaultKnobs) {
        self.faults = faults;
    }

    /// Returns the robot's current peer state.
    #[must_use]
    pub const fn peer(&self) -> PeerState {
        self.core.peer()
    }

    /// Returns how many events the ring has dropped since the last `EVENT_OVERFLOW` was queued.
    #[must_use]
    pub const fn dropped_events(&self) -> u16 {
        self.core.dropped_events()
    }

    /// Iterates over the stub's execution log, oldest first.
    pub fn executions(&self) -> impl Iterator<Item = Execution> + '_ {
        self.executions.iter().copied()
    }

    /// Queues one `BRIDGE_CONTROL` request and returns its control sequence number.
    ///
    /// The request is emitted at control priority by the next [`Stub::step`]; the Bridge's reply —
    /// or its timeout — surfaces through [`Stub::take_bridge_control_report`].
    ///
    /// # Errors
    ///
    /// Returns [`Status::BadArgument`] for an unknown action and [`Status::Busy`] while an
    /// earlier request or its unread result is outstanding. These are the robot's own rules,
    /// owned by `LinkCore`.
    pub fn request_bridge_control(&mut self, action: u8, now_ms: u64) -> Result<u16, Status> {
        let seq = self.core.request_bridge_control(action, now_ms)?;
        self.requests.push(Requested { seq, action });
        Ok(seq)
    }

    /// Returns how many of the six streams the Bridge has paused right now.
    ///
    /// The Bridge sends `THROTTLE` when its robot-facing ring crosses the high watermark and
    /// resumes at the low one, so this is the robot-side observation of back-pressure.
    #[must_use]
    pub fn paused_streams(&self) -> usize {
        Stream::ALL
            .into_iter()
            .filter(|stream| self.core.throttle_state(*stream) == ThrottleState::Paused)
            .count()
    }

    /// Returns how many well-formed `THROTTLE` hints the core has received.
    #[must_use]
    pub const fn throttle_hints(&self) -> u32 {
        self.core.throttle_hints()
    }

    /// Takes the oldest `BRIDGE_CONTROL` report: the Bridge's reply, or its timeout.
    #[must_use]
    pub fn take_bridge_control_report(&mut self) -> Option<BridgeControlReport> {
        self.reports.pop_front()
    }

    /// Drains the transport into the core's parser.
    fn drain_transport(&mut self, now_ms: u64) {
        let mut chunk = [0u8; RX_CHUNK_BYTES];
        loop {
            let read = self.transport.read(&mut chunk);
            if read == 0 {
                return;
            }
            self.core.on_rx(&chunk[..read], now_ms);
            if read < chunk.len() {
                return;
            }
        }
    }

    /// Polls the core until it has nothing left, acknowledging commands and recording replies.
    fn service_core(&mut self, now_ms: u64) {
        loop {
            // The poll result borrows the core; convert it to owned data, then act.
            let served = match self.core.poll() {
                None => return,
                Some(Inbound::Command(request)) => {
                    let seq = request.seq.get();
                    let command = request.command;
                    let completion = completion_of(&request.message);
                    Served::Command {
                        seq,
                        command,
                        completion,
                    }
                }
                Some(Inbound::EventAck(_)) => {
                    // The ring entry is already retired; the peer's status is diagnostic only.
                    continue;
                }
                Some(Inbound::ControlReply(reply)) => {
                    let seq = reply.seq.get();
                    let reply = match reply.result {
                        Ok(BridgeControlReply::Setup(setup)) => OwnedReply::Setup {
                            status: setup.status,
                            ssid: String::from(setup.ssid),
                            passphrase: String::from(setup.passphrase),
                        },
                        Ok(BridgeControlReply::PairingCode(code)) => OwnedReply::PairingCode {
                            status: code.status,
                            code: String::from(code.code),
                        },
                        Err(status) => OwnedReply::Malformed { status },
                    };
                    Served::Reply { seq, reply }
                }
                Some(Inbound::ControlTimeout { seq }) => Served::Timeout { seq },
            };
            self.serve(served, now_ms);
        }
    }

    /// Acts on one polled message with the core's borrow released.
    fn serve(&mut self, served: Served, now_ms: u64) {
        match served {
            Served::Command {
                seq,
                command,
                completion,
            } => {
                let status = Status::Ok;
                if !self.core.ack_command(seq, status, &[]) {
                    return;
                }
                self.record_execution(seq, command, status);
                if let Some(completion) = completion {
                    self.publish_completion(completion, seq, now_ms);
                }
            }
            Served::Reply { seq, reply } => {
                if let Some(action) = self.take_request(seq) {
                    self.reports
                        .push_back(BridgeControlReport::Replied { action, seq, reply });
                }
            }
            Served::Timeout { seq } => {
                if let Some(action) = self.take_request(seq) {
                    self.reports.push_back(BridgeControlReport::TimedOut { action, seq });
                }
            }
        }
    }

    /// Removes the request paired to `seq`, returning its action.
    fn take_request(&mut self, seq: u16) -> Option<u8> {
        let index = self.requests.iter().position(|request| request.seq == seq)?;
        Some(self.requests.remove(index).action)
    }

    /// Records one execution in the bounded log.
    fn record_execution(&mut self, seq: u16, command: Command, status: Status) {
        if self.executions.len() == EXECUTION_LOG {
            self.executions.pop_front();
        }
        self.executions.push_back(Execution { seq, command, status });
    }

    /// Publishes the completion event of one executed drive command (ADR-0032: the completion
    /// carries the originating command's sequence).
    fn publish_completion(&mut self, completion: Completion, seq: u16, now_ms: u64) {
        let message = match completion {
            Completion::Distance { target_cm } => EventMessage::DriveDistanceCompleted(DriveDistanceCompleted {
                command_seq: seq,
                status: CompletionStatus::Success,
                target_cm,
                distance_cm: i32::from(target_cm),
                duration_ms: 0,
                detail: "stub drive completed",
            }),
            Completion::Rotation { target_cdeg } => EventMessage::RotateCompleted(RotateCompleted {
                command_seq: seq,
                status: CompletionStatus::Success,
                target_cdeg,
                error_cdeg: 0,
                duration_ms: 0,
                detail: "stub rotation completed",
            }),
        };
        let _ = self.core.publish_event(&message, now_ms);
    }

    /// Publishes the persona's load on the nominal sampling grid, catching up after a late step.
    fn publish_load(&mut self, now_ms: u64) {
        let mut at = *self.next_sample_ms.get_or_insert(now_ms);
        while at <= now_ms {
            self.sample_tick = self.sample_tick.wrapping_add(1);
            self.publish_samples(self.sample_tick, at);
            at += SAMPLING_TICK_MS;
        }
        self.next_sample_ms = Some(at);

        if self.persona != Persona::WorstCase {
            return;
        }
        let mut at = *self.next_event_ms.get_or_insert(now_ms);
        while at <= now_ms {
            self.event_tick = self.event_tick.wrapping_add(1);
            self.publish_worst_event(self.event_tick, at);
            at += WORST_EVENT_INTERVAL_MS;
        }
        self.next_event_ms = Some(at);
    }

    /// Publishes one sampling tick's worth of stream samples.
    ///
    /// The baseline persona samples a moving robot: the 10 Hz streams change every tick, the
    /// slow streams stay steady, and `LinkCore`'s on-change/heartbeat rule produces ADR-0031's
    /// baseline rates. The worst-case persona changes every sample so every stream is due on
    /// every tick.
    fn publish_samples(&mut self, tick: u32, now_ms: u64) {
        let worst = self.persona == Persona::WorstCase;
        let swing = i32::from(i16::try_from(tick % 5).unwrap_or(0));
        let power = if worst {
            Power {
                level: 80,
                voltage_mv: u16::try_from(7_400 + tick % 100).unwrap_or(u16::MAX),
            }
        } else {
            Power {
                level: 80,
                voltage_mv: 7_400,
            }
        };
        let motion = Motion {
            left: i8::try_from(40 + swing).unwrap_or(i8::MAX),
            right: i8::try_from(40 - swing).unwrap_or(i8::MIN),
        };
        let activity = if worst {
            Activity {
                procedure_id: ProcedureId::COAST_AND_AVOID,
                stage: ActivityStage::Running,
                percent: Some(u8::try_from(tick % 100).unwrap_or(u8::MAX)),
                detail: WORST_DETAIL,
            }
        } else {
            Activity {
                procedure_id: ProcedureId::IDLE,
                stage: ActivityStage::Running,
                percent: None,
                detail: "",
            }
        };
        let perception = Perception::from_bits(if worst { u8::try_from(tick % 2).unwrap_or(0) } else { 0 });
        let calibration = if worst {
            Calibration {
                motor: calibration_at(tick, 0),
                imu: calibration_at(tick, 1),
                mag: calibration_at(tick, 2),
                distance: calibration_at(tick, 3),
            }
        } else {
            Calibration {
                motor: CalibrationStatus::Loaded,
                imu: CalibrationStatus::Loaded,
                mag: CalibrationStatus::Loaded,
                distance: CalibrationStatus::Loaded,
            }
        };
        let cloud = cloud(tick);
        self.core.publish_stream(&StreamSample::Power(power), 0, now_ms);
        self.core.publish_stream(&StreamSample::Motion(motion), 0, now_ms);
        self.core.publish_stream(&StreamSample::Activity(activity), 0, now_ms);
        self.core
            .publish_stream(&StreamSample::Perception(perception), 0, now_ms);
        self.core.publish_stream(&StreamSample::LidarCloud(cloud), 0, now_ms);
        self.core
            .publish_stream(&StreamSample::Calibration(calibration), 0, now_ms);
    }

    /// Publishes one worst-case event: the catalogue's largest payload, so the paced firehose
    /// carries ADR-0033's assumed event bytes.
    fn publish_worst_event(&mut self, tick: u32, now_ms: u64) {
        let message = EventMessage::DriveDistanceCompleted(DriveDistanceCompleted {
            command_seq: u16::try_from(tick % 1_000).unwrap_or(u16::MAX),
            status: CompletionStatus::Success,
            target_cm: 100,
            distance_cm: 100,
            duration_ms: 1_000,
            detail: WORST_DETAIL,
        });
        let _ = self.core.publish_event(&message, now_ms);
    }

    /// Emits every frame the core has ready, applying the fault knobs to the outbound bytes.
    fn emit(&mut self, now_ms: u64) {
        let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
        while let Some(written) = self.core.next_frame(&mut buffer) {
            self.send_outbound(&buffer[..written], now_ms);
        }
    }

    /// Applies the fault knobs to one frame's bytes and hands them to the transport.
    fn send_outbound(&mut self, bytes: &[u8], now_ms: u64) {
        let mut out = Vec::with_capacity(bytes.len());
        for &byte in bytes {
            self.outbound_bytes = self.outbound_bytes.wrapping_add(1);
            let count = self.outbound_bytes;
            if self.faults.drop_every != 0 && count.is_multiple_of(u64::from(self.faults.drop_every)) {
                continue;
            }
            let byte = if self.faults.corrupt_every != 0 && count.is_multiple_of(u64::from(self.faults.corrupt_every)) {
                byte ^ 0xFF
            } else {
                byte
            };
            out.push(byte);
        }
        if out.is_empty() {
            return;
        }
        if self.faults.delay_ms == 0 {
            self.transport.write(&out);
        } else {
            self.delayed.push((now_ms + self.faults.delay_ms, out));
        }
    }

    /// Writes the delayed chunks whose time has come, in order.
    fn flush_delayed(&mut self, now_ms: u64) {
        let mut index = 0;
        while index < self.delayed.len() {
            if self.delayed[index].0 <= now_ms {
                let (_, bytes) = self.delayed.remove(index);
                self.transport.write(&bytes);
            } else {
                index += 1;
            }
        }
    }
}

/// The completion a persona publishes for one executed command, extracted as owned data before
/// the poll's borrow of the core ends.
const fn completion_of(message: &CommandMessage<'_>) -> Option<Completion> {
    match message {
        CommandMessage::DriveDistance(request) => Some(Completion::Distance {
            target_cm: request.distance_cm,
        }),
        CommandMessage::RotateBy(request) => Some(Completion::Rotation {
            target_cdeg: request.angle_cdeg,
        }),
        _ => None,
    }
}

/// Builds one calibration status cycling through the wire's values with `tick`.
fn calibration_at(tick: u32, offset: u32) -> CalibrationStatus {
    CalibrationStatus::ALL[usize::from(u8::try_from((tick + offset) % 3).unwrap_or(0))]
}

/// Builds the 360-slot range grid, varying with `tick` so the worst-case sample is always due.
fn cloud(tick: u32) -> LidarCloud {
    let mut slots = [LidarCloud::NO_RETURN; 360];
    let drift = usize::try_from(tick % 100).unwrap_or(0);
    for (index, slot) in slots.iter_mut().enumerate() {
        let value = 200 + (index + drift) % 100;
        *slot = u16::try_from(value).unwrap_or(LidarCloud::NO_RETURN);
    }
    LidarCloud { slots }
}
