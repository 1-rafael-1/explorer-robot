//! The daemon's Bridge link: raw TCP, the preamble, `HELLO`/`PING`, decode, event ACK and
//! command sequence/retry (ADR-0039's `link` module).
//!
//! The adapter owns one TCP connection to the Bridge at a time. It runs the shared
//! [`bridge_net::preamble`] handshake, opens a Session with `HELLO`, seeds the relay from
//! `GET_STATUS`/`GET_CATALOGUE`, then feeds decoded frames into [`Relay`] and drains
//! [`Relay::poll_link`]. Every timer is driven from one loop, so neither peer ever waits on the
//! other.
//!
//! # Isolation from the app adapter
//!
//! The relay is shared through `Arc<Mutex<Relay>>`. The link never awaits an app: every stream and
//! event path is a bounded, synchronous relay update. [`Link::spawn`] returns a [`LinkHandle`] with
//! a cancellation channel and a state subscription, so ticket 13 can open the Session when the
//! first app authenticates and close it after the grace.
//!
//! # The epoch
//!
//! [`LinkConfig::new`] chooses the Session epoch once, from `getrandom`, and every reconnect reuses
//! it, so a retried command is still a duplicate rather than a fresh execution (ADR-0039).

use std::{
    fmt,
    net::SocketAddr,
    sync::{Arc, Mutex, MutexGuard, PoisonError},
    time::{Duration, Instant},
};

use bridge_net::preamble::DenyReason;
use link_protocol::{
    BridgeStatus, BridgeStream, Caps, Class, Command, CommandMessage, Control, ControlMessage, Direction, Event,
    EventMessage, Flags as LinkFlags, Frame, MAX_PC_TO_ROBOT_FRAME, PROTOCOL_MAJOR, PROTOCOL_MINOR, PayloadCodec,
    RateEntry, RobotToPcParser, Seq, Status, Stream, StreamSample, decode_command, decode_control, decode_event,
    decode_stream, encode_command, encode_event, encode_hello,
    payload::{
        command::{CatalogueFlags, CataloguePage, GetCatalogue, GetStatus},
        event::EventAck,
    },
};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::TcpStream,
    sync::watch,
    task::JoinHandle,
};

use crate::{
    handshake::{self, AUTH_DEADLINE, HandshakeError},
    relay::{OutboundCommand, Relay},
    schema::{Catalogue, CatalogueProcedure},
};

/// The baseline Requested Rate, in centi-Hz, for an idle robot's motion and activity streams.
pub const BASELINE_RATE_CHZ: u16 = 100;

/// How long `HELLO` waits for its acknowledgement before retrying (ADR-0039).
pub const HELLO_RETRY: Duration = Duration::from_secs(1);

/// How many `HELLO` attempts a bring-up makes before reconnecting (ADR-0039).
pub const HELLO_ATTEMPTS: u8 = 5;

/// The bring-up budget for `GET_STATUS` and `GET_CATALOGUE` (ADR-0039).
pub const BRING_UP_BUDGET: Duration = Duration::from_secs(1);

/// How long one daemon query waits for its typed response before the next one is tried.
pub const QUERY_TIMEOUT: Duration = Duration::from_millis(500);

/// The PING cadence while an app is live (ADR-0039).
pub const PING_INTERVAL: Duration = Duration::from_secs(1);

/// The relay tick cadence: command retries, timeouts and the app-frozen timer.
pub const TICK_INTERVAL: Duration = Duration::from_millis(100);

/// The silence after which the robot link is `reconnecting` (ADR-0039).
pub const STALL: Duration = Duration::from_secs(3);

/// The TCP connect timeout.
pub const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// A `busy` denial's default retry hint (ADR-0039's 4 s).
pub const BUSY_RETRY_DEFAULT_MS: u32 = 4_000;

/// The first reconnect delay (ADR-0039).
pub const BACKOFF_BASE_MS: u64 = 500;

/// The reconnect delay cap (ADR-0039).
pub const BACKOFF_CAP_MS: u64 = 8_000;

/// The fractional reconnect jitter, ±20 % (ADR-0039).
pub const JITTER_FRACTION: f64 = 0.20;

/// The nominal baud the incremental parser derives its assembly timeout from.
const BAUD: u32 = 921_600;

/// The largest single read chunk.
const READ_CHUNK: usize = 4_096;

/// Why a [`LinkConfig`] could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkError {
    /// The operating system's random source did not provide the epoch.
    Random,
}

impl fmt::Display for LinkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Random => write!(f, "the Session epoch could not be generated"),
        }
    }
}

impl std::error::Error for LinkError {}

/// The link's connection state, mirroring ADR-0038's app-facing connection states.
///
/// Ticket 13 maps these onto the app's `connection{state}` messages. [`Self::VersionMismatch`] has
/// no separate ADR-0038 state, so it is surfaced as `unauthorized` with a detail — the same pattern
/// ADR-0039 uses for an ACK `VERSION_MISMATCH` becoming `rejected` plus a detail.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LinkState {
    /// The adapter has not attempted a connection yet.
    Starting,
    /// TCP or the preamble is in progress.
    Connecting,
    /// The Session is open and `HELLO_ACK` supplied the Boot Id.
    ///
    /// Bring-up (`GET_STATUS`/`GET_CATALOGUE`) continues after this state; a late Catalogue
    /// seeds a fresh snapshot rather than blocking the connection (ADR-0039).
    Connected {
        /// The robot's Boot Id for this powered link period.
        boot_id: u32,
    },
    /// The connection was lost; the adapter retries after the delay.
    Reconnecting {
        /// The delay before the next attempt, in milliseconds, jitter included.
        retry_in_ms: u64,
    },
    /// `BUSY`: another client holds the Bridge; retried after its hint.
    Busy {
        /// The delay before the next attempt, in milliseconds.
        retry_in_ms: u64,
    },
    /// The token was refused or the Bridge could not prove it; never auto-retried.
    Unauthorized {
        /// A human-readable detail for the app.
        detail: String,
    },
    /// The Bridge is in Setup Mode; the link does not connect.
    SetupMode {
        /// A human-readable detail for the app.
        detail: String,
    },
    /// The preamble or Link version does not match; surfaced and never auto-retried.
    VersionMismatch {
        /// A human-readable detail for the app.
        detail: String,
    },
    /// The adapter was cancelled.
    Stopped,
}

/// What a `DENY` reason means for the adapter (ADR-0039's failure mapping).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DenyDisposition {
    /// Surface the state and stop; the operator must act (re-pair, provision or update).
    Fatal(LinkState),
    /// Retry the whole attempt after the delay.
    RetryAfter(Duration),
}

/// Maps a `DENY` reason and its optional hint onto the adapter's next action.
///
/// `unauthorized` is never retried; `setup_mode` does not connect at all; `version` is surfaced;
/// `busy` retries after its hint, defaulting to ADR-0039's 4 s.
#[must_use]
pub fn deny_disposition(reason: DenyReason, retry_ms: Option<u32>) -> DenyDisposition {
    match reason {
        DenyReason::Unauthorized => DenyDisposition::Fatal(LinkState::Unauthorized {
            detail: String::from("The pairing token was refused; re-pair from the panel's Link screen."),
        }),
        DenyReason::SetupMode => DenyDisposition::Fatal(LinkState::SetupMode {
            detail: String::from("The Bridge is in Setup Mode and does not accept a link."),
        }),
        DenyReason::Version => DenyDisposition::Fatal(LinkState::VersionMismatch {
            detail: String::from("The Bridge's preamble version does not match the daemon's."),
        }),
        DenyReason::Busy => DenyDisposition::RetryAfter(Duration::from_millis(u64::from(
            retry_ms.unwrap_or(BUSY_RETRY_DEFAULT_MS),
        ))),
    }
}

/// Returns the base reconnect delay for `attempt`, before jitter: 500 ms → 8 s cap (ADR-0039).
#[must_use]
pub fn backoff_delay(attempt: u8) -> Duration {
    let factor = 1u64 << u32::from(attempt.min(4));
    Duration::from_millis(BACKOFF_BASE_MS.saturating_mul(factor).min(BACKOFF_CAP_MS))
}

/// Applies ±`jitter` to `delay`, using `sample` as a deterministic draw from `0..=u16::MAX`.
#[must_use]
pub fn jittered(delay: Duration, jitter: f64, sample: u16) -> Duration {
    let jitter = jitter.clamp(0.0, 1.0);
    let unit = f64::from(sample) / f64::from(u16::MAX);
    let factor = 1.0 + jitter * unit.mul_add(2.0, -1.0);
    Duration::from_secs_f64(delay.as_secs_f64() * factor)
}

/// Chooses a Session epoch from the operating system's random source.
///
/// The daemon calls this once per process and reuses the value across reconnects (ADR-0039).
///
/// # Errors
///
/// Returns [`LinkError::Random`] when the source fails.
pub fn random_epoch() -> Result<u32, LinkError> {
    let mut bytes = [0u8; 4];
    getrandom::fill(&mut bytes).map_err(|_| LinkError::Random)?;
    Ok(u32::from_le_bytes(bytes))
}

/// The link adapter's configuration: where, with what token, and the ADR's timers.
#[derive(Clone, Debug)]
pub struct LinkConfig {
    /// The Bridge's `host:port`. Discovery lands in ticket 13; this is the manual endpoint.
    pub endpoint: SocketAddr,
    /// The pairing token's bytes; 60 bits in production.
    pub token: Vec<u8>,
    /// The Session epoch, chosen once per daemon process.
    pub epoch: u32,
    /// The TCP connect timeout.
    pub connect_timeout: Duration,
    /// The `HELLO` retry interval.
    pub hello_retry: Duration,
    /// The number of `HELLO` attempts.
    pub hello_attempts: u8,
    /// The `GET_STATUS`/`GET_CATALOGUE` budget.
    pub bring_up_budget: Duration,
    /// The per-query response timeout.
    pub query_timeout: Duration,
    /// The relay tick cadence.
    pub tick_interval: Duration,
    /// The PING cadence while an app is live.
    pub ping_interval: Duration,
    /// The decoded-robot-frame silence that means `reconnecting`.
    pub stall: Duration,
    /// The fractional reconnect jitter.
    pub jitter: f64,
}

impl LinkConfig {
    /// Builds a config with the ADR's timers and a fresh random epoch.
    ///
    /// # Errors
    ///
    /// Returns [`LinkError::Random`] when the epoch cannot be generated.
    pub fn new(endpoint: SocketAddr, token: impl Into<Vec<u8>>) -> Result<Self, LinkError> {
        Ok(Self::with_epoch(endpoint, token, random_epoch()?))
    }

    /// Builds a config with the ADR's timers and the supplied epoch.
    ///
    /// Tests and ticket 13 use this when the epoch is already fixed for the process.
    #[must_use]
    pub fn with_epoch(endpoint: SocketAddr, token: impl Into<Vec<u8>>, epoch: u32) -> Self {
        Self {
            endpoint,
            token: token.into(),
            epoch,
            connect_timeout: CONNECT_TIMEOUT,
            hello_retry: HELLO_RETRY,
            hello_attempts: HELLO_ATTEMPTS,
            bring_up_budget: BRING_UP_BUDGET,
            query_timeout: QUERY_TIMEOUT,
            tick_interval: TICK_INTERVAL,
            ping_interval: PING_INTERVAL,
            stall: STALL,
            jitter: JITTER_FRACTION,
        }
    }
}

/// A running link: spawn it and keep the handle to observe state and cancel.
pub struct LinkHandle {
    /// The published link state.
    states: watch::Receiver<LinkState>,
    /// The cancellation channel.
    stop: watch::Sender<bool>,
    /// The link task.
    join: JoinHandle<()>,
}

impl LinkHandle {
    /// Returns the latest published state.
    #[must_use]
    pub fn state(&self) -> LinkState {
        self.states.borrow().clone()
    }

    /// Subscribes to state changes for the app adapter.
    #[must_use]
    pub fn subscribe(&self) -> watch::Receiver<LinkState> {
        self.states.clone()
    }

    /// Returns whether the link task has finished (a fatal state, or cancellation).
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.join.is_finished()
    }

    /// Asks the link to close its connection and stop retrying.
    pub fn stop(&self) {
        let _ = self.stop.send(true);
    }

    /// Waits for the link task to finish.
    pub async fn wait(self) {
        let _ = self.join.await;
    }
}

/// The Bridge link adapter.
pub struct Link {
    /// The configuration and timers.
    config: LinkConfig,
    /// The shared relay the adapter feeds.
    relay: Arc<Mutex<Relay>>,
}

impl Link {
    /// Builds a link over `relay`.
    #[must_use]
    pub const fn new(config: LinkConfig, relay: Arc<Mutex<Relay>>) -> Self {
        Self { config, relay }
    }

    /// Returns the configuration the link will use.
    #[must_use]
    pub const fn config(&self) -> &LinkConfig {
        &self.config
    }

    /// Spawns the link on the current tokio runtime.
    #[must_use]
    pub fn spawn(self) -> LinkHandle {
        let (stop, stop_rx) = watch::channel(false);
        let (states_tx, states) = watch::channel(LinkState::Starting);
        let join = tokio::spawn(async move {
            self.run(stop_rx, states_tx).await;
        });
        LinkHandle { states, stop, join }
    }

    /// Runs the adapter until a fatal state or cancellation.
    ///
    /// Ticket 13 can also drive this future directly when it needs to own the task.
    pub async fn run(self, mut stop: watch::Receiver<bool>, states: watch::Sender<LinkState>) -> LinkState {
        // The app adapter stamps its relay updates from the same process origin, so the relay's
        // liveness and command timers compare like values across the two adapters (ADR-0039).
        let started = crate::clock::origin();
        let mut backoff = Backoff::new();
        Self::publish(&states, LinkState::Starting);
        loop {
            if *stop.borrow() {
                self.close_session();
                Self::publish(&states, LinkState::Stopped);
                return LinkState::Stopped;
            }
            match self.connect_once(started, &mut stop, &states, &mut backoff).await {
                Attempt::Fatal(state) => {
                    self.close_session();
                    Self::publish(&states, state.clone());
                    return state;
                }
                Attempt::Stopped => {
                    self.close_session();
                    Self::publish(&states, LinkState::Stopped);
                    return LinkState::Stopped;
                }
                Attempt::Retry { after, state } => {
                    self.close_session();
                    let delay = after.unwrap_or_else(|| backoff.next(self.config.jitter));
                    let published = match state {
                        LinkState::Reconnecting { .. } => LinkState::Reconnecting {
                            retry_in_ms: millis(delay),
                        },
                        other => other,
                    };
                    Self::publish(&states, published);
                    if !sleep_or_stop(delay, &mut stop).await {
                        self.close_session();
                        Self::publish(&states, LinkState::Stopped);
                        return LinkState::Stopped;
                    }
                }
            }
        }
    }

    /// One TCP + preamble + Session attempt.
    async fn connect_once(
        &self,
        started: Instant,
        stop: &mut watch::Receiver<bool>,
        states: &watch::Sender<LinkState>,
        backoff: &mut Backoff,
    ) -> Attempt {
        Self::publish(states, LinkState::Connecting);

        let connect = tokio::time::timeout(self.config.connect_timeout, TcpStream::connect(self.config.endpoint));
        let stream = tokio::select! {
            _ = stop.changed() => return Attempt::Stopped,
            result = connect => match result {
                Ok(Ok(stream)) => stream,
                _ => return retry(),
            },
        };
        if stream.set_nodelay(true).is_err() {
            return retry();
        }
        let mut conn = Conn::new(stream);

        let handshake = tokio::select! {
            _ = stop.changed() => return Attempt::Stopped,
            result = handshake::run(&mut conn.stream, &self.config.token, AUTH_DEADLINE) => result,
        };
        match handshake {
            Ok(accepted) => {
                if accepted
                    .identity
                    .flags
                    .contains(bridge_net::identity::Flags::SETUP_MODE)
                {
                    return Attempt::Fatal(LinkState::SetupMode {
                        detail: String::from("The Bridge advertises Setup Mode."),
                    });
                }
            }
            Err(HandshakeError::BadBridgeProof) => {
                return Attempt::Fatal(LinkState::Unauthorized {
                    detail: String::from("The Bridge did not prove the pairing token."),
                });
            }
            Err(HandshakeError::Denied { reason, retry_ms }) => {
                return match deny_disposition(reason, retry_ms) {
                    DenyDisposition::Fatal(state) => Attempt::Fatal(state),
                    DenyDisposition::RetryAfter(delay) => Attempt::Retry {
                        after: Some(delay),
                        state: LinkState::Busy {
                            retry_in_ms: millis(delay),
                        },
                    },
                };
            }
            Err(_) => return retry(),
        }

        let (boot_id, deferred) = match hello_handshake(&self.config, &mut conn, stop).await {
            HelloOutcome::Accepted { boot_id, deferred } => (boot_id, deferred),
            HelloOutcome::Stopped => return Attempt::Stopped,
            HelloOutcome::Version => {
                return Attempt::Fatal(LinkState::VersionMismatch {
                    detail: String::from("The robot refused HELLO with a version mismatch."),
                });
            }
            HelloOutcome::Refused(refusal) => {
                return Attempt::Fatal(LinkState::Unauthorized {
                    detail: format!("The robot refused HELLO ({refusal:?})."),
                });
            }
            HelloOutcome::Retry => return retry(),
        };

        backoff.reset();
        let now_ms = millis(started.elapsed());
        let previous = {
            let guard = lock(&self.relay);
            guard.boot_id()
        };
        {
            let mut guard = lock(&self.relay);
            guard.session_up(boot_id, now_ms);
            if previous.is_some_and(|old| old != boot_id) {
                guard.restart_wire_sequence();
            }
        }
        Self::publish(states, LinkState::Connected { boot_id });

        let mut session = Session::new(conn, started, &self.config);
        session.admit(deferred, &self.relay);
        session.run(&self.relay, &self.config, stop).await
    }

    /// Publishes a state to the app adapter.
    fn publish(states: &watch::Sender<LinkState>, state: LinkState) {
        states.send_replace(state);
    }

    /// Closes the relay Session after a fatal or cancelled link.
    fn close_session(&self) {
        lock(&self.relay).session_down();
    }
}

/// The result of one connection attempt.
enum Attempt {
    /// The attempt ended; retry after `after`, or the backoff when `None`.
    Retry {
        /// A reason-specific delay, such as `BUSY`'s hint.
        after: Option<Duration>,
        /// The state to publish while waiting.
        state: LinkState,
    },
    /// A state that ends the adapter until the operator acts.
    Fatal(LinkState),
    /// The adapter was cancelled.
    Stopped,
}

/// A retry with the ordinary backoff and a `reconnecting` state.
const fn retry() -> Attempt {
    Attempt::Retry {
        after: None,
        state: LinkState::Reconnecting { retry_in_ms: 0 },
    }
}

/// The reconnect schedule (ADR-0039's 500 ms → 8 s cap, reset on `HELLO_ACK`).
struct Backoff {
    /// The number of attempts since the last reset.
    attempt: u8,
}

impl Backoff {
    /// Starts at the first delay.
    const fn new() -> Self {
        Self { attempt: 0 }
    }

    /// Resets after a successful `HELLO_ACK`.
    const fn reset(&mut self) {
        self.attempt = 0;
    }

    /// Returns the next jittered delay and advances the schedule.
    fn next(&mut self, jitter: f64) -> Duration {
        let delay = jittered(backoff_delay(self.attempt), jitter, random_u16());
        self.attempt = self.attempt.saturating_add(1);
        delay
    }
}

/// The outcome of the `HELLO` exchange.
enum HelloOutcome {
    /// The robot accepted; its Boot Id and any frames read ahead are carried.
    Accepted {
        /// The robot's Boot Id for this powered link period.
        boot_id: u32,
        /// Frames that arrived before `HELLO_ACK`, held for the Session.
        deferred: Vec<RawFrame>,
    },
    /// The robot refused with a non-OK status.
    Refused(Status),
    /// The robot refused with `VERSION_MISMATCH`.
    Version,
    /// No acknowledgement arrived within the attempts.
    Retry,
    /// The adapter was cancelled.
    Stopped,
}

/// Encodes the v1 `HELLO`: the process epoch, no capabilities and the baseline stream rates.
fn hello_payload(config: &LinkConfig, out: &mut [u8]) -> Option<usize> {
    let rates = [
        RateEntry::enabled(Stream::Motion.wire_id(), BASELINE_RATE_CHZ),
        RateEntry::enabled(Stream::Activity.wire_id(), BASELINE_RATE_CHZ),
    ];
    encode_hello(
        PROTOCOL_MAJOR,
        PROTOCOL_MINOR,
        Caps::from_bits(0),
        config.epoch,
        rates,
        out,
    )
    .ok()
}

/// Sends `HELLO` with the baseline rates, retrying every interval up to the configured attempts.
///
/// Frames other than `HELLO_ACK` are ignored here: the robot may already be streaming, and the
/// Session does not exist until `HELLO_ACK`. Ignored events are retransmitted by the robot once
/// the Session is open.
async fn hello_handshake(config: &LinkConfig, conn: &mut Conn, stop: &mut watch::Receiver<bool>) -> HelloOutcome {
    let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
    let Some(length) = hello_payload(config, &mut payload) else {
        return HelloOutcome::Retry;
    };

    let mut deferred = Vec::new();
    for _ in 0..config.hello_attempts.max(1) {
        let seq = conn.next_control_seq();
        conn.queue_frame(
            Class::Control,
            LinkFlags::default(),
            Control::Hello.wire_id(),
            seq,
            &payload[..length],
        );
        if conn.flush().await.is_err() {
            return HelloOutcome::Retry;
        }

        let deadline = Instant::now() + config.hello_retry;
        let mut buffer = [0u8; READ_CHUNK];
        loop {
            let sleep = tokio::time::sleep_until(tokio::time::Instant::from_std(deadline));
            tokio::select! {
                _ = stop.changed() => return HelloOutcome::Stopped,
                () = sleep => break,
                result = conn.stream.read(&mut buffer) => match result {
                    Ok(0) | Err(_) => return HelloOutcome::Retry,
                    Ok(count) => {
                        for frame in decode_frames(&mut conn.parser, &buffer[..count], 0) {
                            if frame.class == Class::Control
                                && frame.flags.contains(LinkFlags::IS_ACK)
                                && frame.id == Control::Hello.wire_id()
                                && let Ok(ControlMessage::HelloAck(ack)) =
                                    decode_control(Control::Hello, true, &frame.payload)
                            {
                                return match ack.status {
                                    Status::Ok => HelloOutcome::Accepted {
                                        boot_id: ack.boot_id,
                                        deferred,
                                    },
                                    Status::VersionMismatch => HelloOutcome::Version,
                                    other => HelloOutcome::Refused(other),
                                };
                            }
                            deferred.push(frame);
                        }
                    }
                },
            }
        }
    }
    HelloOutcome::Retry
}

/// One decoded robot frame, owned so the parser can move on before handling.
struct RawFrame {
    /// The frame's class.
    class: Class,
    /// The frame's flags.
    flags: LinkFlags,
    /// The class-specific id.
    id: u16,
    /// The frame's sequence.
    seq: u16,
    /// The payload bytes.
    payload: Vec<u8>,
}

/// Returns `true` when `frame` is Bridge-originated telemetry rather than a robot frame.
///
/// The robot's stall timer must measure the robot, not the Bridge: a healthy Bridge publishing
/// `BRIDGE_STATUS` while the robot itself is silent would otherwise mask a dead robot.
fn is_bridge_telemetry(frame: &RawFrame) -> bool {
    frame.class == Class::Stream && BridgeStream::from_wire(frame.id).is_some()
}

/// Decodes every complete frame in `bytes`, returning them in wire order.
fn decode_frames(parser: &mut RobotToPcParser, bytes: &[u8], stamp: u32) -> Vec<RawFrame> {
    let mut frames = Vec::new();
    parser.feed(bytes, stamp, |frame| {
        frames.push(RawFrame {
            class: frame.header.class,
            flags: frame.header.flags,
            id: frame.header.id,
            seq: frame.header.seq.get(),
            payload: frame.payload.to_vec(),
        });
    });
    frames
}

/// The link's TCP stream plus the incremental parser and the queued writes.
struct Conn {
    /// The connected stream.
    stream: TcpStream,
    /// The robot-to-PC parser, exactly as the Bridge's bytes arrive.
    parser: RobotToPcParser,
    /// The control sequence counter shared by `HELLO` and `PING`.
    control_seq: u16,
    /// Encoded frames waiting to be written.
    outbox: Vec<Vec<u8>>,
}

impl Conn {
    /// Wraps a connected stream.
    fn new(stream: TcpStream) -> Self {
        Self {
            stream,
            parser: RobotToPcParser::new(Direction::RobotToPc, BAUD),
            control_seq: 0,
            outbox: Vec::new(),
        }
    }

    /// Returns the next control sequence, skipping zero.
    const fn next_control_seq(&mut self) -> u16 {
        self.control_seq = self.control_seq.wrapping_add(1);
        if self.control_seq == 0 {
            self.control_seq = 1;
        }
        self.control_seq
    }

    /// Encodes one PC-to-robot frame into the outbox.
    fn queue_frame(&mut self, class: Class, flags: LinkFlags, id: u16, seq: u16, payload: &[u8]) {
        let Ok(frame) = Frame::new(PROTOCOL_MAJOR, class, flags, id, Seq::new(seq), payload) else {
            return;
        };
        let mut buffer = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let Ok(length) = frame.encode_into(&mut buffer) else {
            return;
        };
        self.outbox.push(buffer[..length].to_vec());
    }

    /// Writes and clears every queued frame.
    async fn flush(&mut self) -> std::io::Result<()> {
        for frame in self.outbox.drain(..) {
            self.stream.write_all(&frame).await?;
        }
        Ok(())
    }
}

/// One daemon-owned query awaiting its typed response.
struct PendingQuery {
    /// When it was sent.
    sent_at: Instant,
}

/// An open Session: the timers, bring-up state and frame handling.
// The five flags form one small bring-up state machine; two-variant enums would obscure it.
#[allow(clippy::struct_excessive_bools)]
struct Session {
    /// The transport.
    conn: Conn,
    /// The link's start time; every relay timestamp is relative to it.
    started: Instant,
    /// The last decoded robot frame; the stall timer measures from here.
    last_robot_frame: Instant,
    /// The next relay tick.
    next_tick: Instant,
    /// The next PING.
    next_ping: Instant,
    /// The bring-up budget's end.
    bring_up_deadline: Instant,
    /// Whether bring-up is currently scheduling queries.
    bring_up_active: bool,
    /// Whether the one background retry has been scheduled.
    bring_up_retried: bool,
    /// When to restart a failed bring-up, once.
    bring_up_retry_at: Option<Instant>,
    /// The query in flight, if any.
    pending_query: Option<PendingQuery>,
    /// Whether `GET_STATUS` seeded the snapshot.
    status_seed: bool,
    /// Whether `GET_CATALOGUE` completed.
    catalogue_seed: bool,
    /// The total catalogue entries from the first page.
    catalogue_total: Option<u8>,
    /// The entries gathered across pages.
    catalogue_entries: Vec<CatalogueProcedure>,
    /// The next catalogue page to request.
    catalogue_next_page: u8,
}

impl Session {
    /// Starts a Session at `now`.
    fn new(conn: Conn, started: Instant, config: &LinkConfig) -> Self {
        let now = Instant::now();
        Self {
            conn,
            started,
            last_robot_frame: now,
            next_tick: now,
            next_ping: now,
            bring_up_deadline: now + config.bring_up_budget,
            bring_up_active: true,
            bring_up_retried: false,
            bring_up_retry_at: None,
            pending_query: None,
            status_seed: false,
            catalogue_seed: false,
            catalogue_total: None,
            catalogue_entries: Vec::new(),
            catalogue_next_page: 0,
        }
    }

    /// Hands frames read before `HELLO_ACK` to the relay now that the Session is open.
    fn admit(&mut self, frames: Vec<RawFrame>, relay: &Mutex<Relay>) {
        let now_ms = millis(self.started.elapsed());
        for frame in frames {
            self.handle_frame(&frame, relay, now_ms);
        }
    }

    /// Runs the Session loop until the link stalls, the peer closes or the adapter is cancelled.
    async fn run(mut self, relay: &Mutex<Relay>, config: &LinkConfig, stop: &mut watch::Receiver<bool>) -> Attempt {
        let mut buffer = [0u8; READ_CHUNK];
        loop {
            if *stop.borrow() {
                return Attempt::Stopped;
            }
            let now = Instant::now();
            let now_ms = millis(self.started.elapsed());
            self.advance_timers(relay, config, now, now_ms);

            if now.duration_since(self.last_robot_frame) >= config.stall {
                return retry();
            }
            if self.conn.flush().await.is_err() {
                return retry();
            }

            let wake = self.next_wake(config, now);
            let sleep = tokio::time::sleep_until(tokio::time::Instant::from_std(wake));
            tokio::select! {
                _ = stop.changed() => {}
                () = sleep => {}
                result = self.conn.stream.read(&mut buffer) => match result {
                    Ok(0) | Err(_) => return retry(),
                    Ok(count) => {
                        let stamp = u32::try_from(now_ms).unwrap_or(u32::MAX);
                        let frames = decode_frames(&mut self.conn.parser, &buffer[..count], stamp);
                        for frame in frames {
                            self.handle_frame(&frame, relay, now_ms);
                        }
                    }
                },
            }
        }
    }

    /// Runs the relay tick, bring-up scheduling, command polling and PING.
    fn advance_timers(&mut self, relay: &Mutex<Relay>, config: &LinkConfig, now: Instant, now_ms: u64) {
        if now >= self.next_tick {
            lock(relay).tick(now_ms);
            self.next_tick = now + config.tick_interval;
        }

        if let Some(pending) = &self.pending_query
            && now.duration_since(pending.sent_at) >= config.query_timeout
        {
            self.pending_query = None;
        }

        self.schedule_bring_up(relay, config, now);
        self.poll_commands(relay, now_ms);

        if now >= self.next_ping {
            if lock(relay).wants_ping() {
                let seq = self.conn.next_control_seq();
                self.conn
                    .queue_frame(Class::Control, LinkFlags::default(), Control::Ping.wire_id(), seq, &[]);
            }
            self.next_ping = now + config.ping_interval;
        }
    }

    /// Schedules `GET_STATUS` and the paged `GET_CATALOGUE` within the bring-up budget.
    fn schedule_bring_up(&mut self, relay: &Mutex<Relay>, config: &LinkConfig, now: Instant) {
        if self.bring_up_active && now >= self.bring_up_deadline {
            self.bring_up_active = false;
            if !(self.bring_up_retried || (self.status_seed && self.catalogue_seed)) {
                self.bring_up_retried = true;
                self.bring_up_retry_at = Some(now + config.bring_up_budget);
            }
        } else if !self.bring_up_active
            && let Some(at) = self.bring_up_retry_at
            && now >= at
        {
            self.bring_up_active = true;
            self.bring_up_retry_at = None;
            self.bring_up_deadline = now + config.bring_up_budget;
            self.reset_missing_bring_up();
        }

        if !self.bring_up_active || self.pending_query.is_some() {
            return;
        }
        if !self.status_seed {
            if self.send_query(relay, Command::GetStatus, 0).is_some() {
                self.pending_query = Some(PendingQuery { sent_at: now });
            }
        } else if !self.catalogue_seed
            && self
                .send_query(relay, Command::GetCatalogue, self.catalogue_next_page)
                .is_some()
        {
            self.pending_query = Some(PendingQuery { sent_at: now });
        }
    }

    /// Clears the bring-up state that must restart after a failed first attempt.
    fn reset_missing_bring_up(&mut self) {
        if !self.catalogue_seed {
            self.catalogue_total = None;
            self.catalogue_entries.clear();
            self.catalogue_next_page = 0;
        }
    }

    /// Sends one daemon-owned query when the app command lane is idle.
    fn send_query(&mut self, relay: &Mutex<Relay>, command: Command, page: u8) -> Option<u16> {
        let seq = {
            let mut guard = lock(relay);
            if guard.has_pending_commands() {
                return None;
            }
            guard.allocate_query_seq()
        };
        let message = match command {
            Command::GetStatus => CommandMessage::GetStatus(GetStatus),
            Command::GetCatalogue => CommandMessage::GetCatalogue(GetCatalogue { page }),
            _ => return None,
        };
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let Ok(length) = encode_command(&message, &mut payload) else {
            return None;
        };
        self.conn.queue_frame(
            Class::Command,
            LinkFlags::ACK_REQUESTED,
            command.wire_id(),
            seq,
            &payload[..length],
        );
        Some(seq)
    }

    /// Drains one frame from the relay while no query is in flight.
    fn poll_commands(&mut self, relay: &Mutex<Relay>, now_ms: u64) {
        if self.pending_query.is_some() {
            return;
        }
        let outbound = lock(relay).poll_link(now_ms);
        let Some(outbound) = outbound else {
            return;
        };
        self.send_command(&outbound);
    }

    /// Encodes and queues one relay command, adding `RETRY` when it is a retransmission.
    fn send_command(&mut self, outbound: &OutboundCommand) {
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let Ok(length) = encode_command(&outbound.message, &mut payload) else {
            return;
        };
        let mut flags = LinkFlags::ACK_REQUESTED;
        if outbound.retry {
            flags |= LinkFlags::RETRY;
        }
        self.conn.queue_frame(
            Class::Command,
            flags,
            outbound.command.wire_id(),
            outbound.wire_seq,
            &payload[..length],
        );
    }

    /// The instant the session next needs a timer wake-up.
    fn next_wake(&self, config: &LinkConfig, now: Instant) -> Instant {
        let mut wake = now + config.tick_interval;
        if let Some(pending) = &self.pending_query {
            wake = wake.min(pending.sent_at + config.query_timeout);
        }
        if let Some(at) = self.bring_up_retry_at {
            wake = wake.min(at);
        }
        if self.bring_up_active {
            wake = wake.min(self.bring_up_deadline);
        }
        wake
    }

    /// Dispatches one decoded robot frame.
    fn handle_frame(&mut self, frame: &RawFrame, relay: &Mutex<Relay>, now_ms: u64) {
        if !is_bridge_telemetry(frame) {
            self.last_robot_frame = Instant::now();
        }
        match frame.class {
            Class::Stream => Self::handle_stream(frame, relay, now_ms),
            Class::Event => self.handle_event(frame, relay, now_ms),
            Class::Command => self.handle_command(frame, relay, now_ms),
            Class::Control => Self::handle_control(frame),
        }
    }

    /// Feeds one stream sample into the relay's latest-wins slot.
    fn handle_stream(frame: &RawFrame, relay: &Mutex<Relay>, now_ms: u64) {
        if BridgeStream::from_wire(frame.id) == Some(BridgeStream::Status) {
            if let Ok(status) = BridgeStatus::decode(&frame.payload) {
                lock(relay).on_bridge_status(&status, now_ms);
            }
            return;
        }
        let Some(stream) = Stream::from_wire(frame.id) else {
            return;
        };
        let Ok(sample) = decode_stream(stream, &frame.payload) else {
            return;
        };
        let mut guard = lock(relay);
        match sample {
            StreamSample::Power(power) => {
                let _ = guard.on_power(&power, now_ms);
            }
            StreamSample::Motion(motion) => {
                let _ = guard.on_motion(&motion, now_ms);
            }
            StreamSample::Activity(activity) => {
                let _ = guard.on_activity(&activity, now_ms);
            }
            StreamSample::Perception(perception) => {
                let _ = guard.on_perception(perception, now_ms);
            }
            StreamSample::LidarCloud(lidar) => {
                let _ = guard.on_lidar(&lidar, now_ms);
            }
            StreamSample::Calibration(calibration) => {
                let _ = guard.on_calibration(&calibration, now_ms);
            }
        }
    }

    /// Stores one robot event and only then acknowledges it (ADR-0039).
    fn handle_event(&mut self, frame: &RawFrame, relay: &Mutex<Relay>, now_ms: u64) {
        if !frame.flags.contains(LinkFlags::ACK_REQUESTED) || frame.flags.contains(LinkFlags::IS_ACK) {
            return;
        }
        let Some(event) = Event::from_wire(frame.id) else {
            return;
        };
        let Ok(message) = decode_event(event, false, &frame.payload) else {
            return;
        };
        {
            let mut guard = lock(relay);
            match message {
                EventMessage::Obstacle(obstacle) => guard.on_obstacle(frame.seq, &obstacle, now_ms),
                EventMessage::FloorDrop(floor_drop) => guard.on_floor_drop(frame.seq, &floor_drop, now_ms),
                EventMessage::ProcedureCompleted(completed) => {
                    guard.on_procedure_completed(frame.seq, &completed, now_ms);
                }
                EventMessage::DriveDistanceCompleted(completed) => {
                    guard.on_drive_completed(frame.seq, &completed, now_ms);
                }
                EventMessage::RotateCompleted(completed) => {
                    guard.on_rotate_completed(frame.seq, &completed, now_ms);
                }
                EventMessage::CalibrationLoaded(loaded) => guard.on_calibration_loaded(frame.seq, &loaded, now_ms),
                EventMessage::EventOverflow(overflow) => guard.on_robot_event_overflow(frame.seq, &overflow, now_ms),
                EventMessage::Ack(_) => return,
            }
        }

        // The relay stored the event; now it is safe to acknowledge it.
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let ack = EventMessage::Ack(EventAck { status: Status::Ok });
        if let Ok(length) = encode_event(&ack, &mut payload) {
            self.conn
                .queue_frame(Class::Event, LinkFlags::IS_ACK, frame.id, frame.seq, &payload[..length]);
        }
    }

    /// Routes a command acknowledgement: typed query responses seed state, others settle a command.
    fn handle_command(&mut self, frame: &RawFrame, relay: &Mutex<Relay>, now_ms: u64) {
        if !frame.flags.contains(LinkFlags::IS_ACK) {
            return;
        }
        let Some(command) = Command::from_wire(frame.id) else {
            return;
        };
        match command {
            Command::GetStatus => {
                if let Ok(CommandMessage::StatusSnapshot(snapshot)) = decode_command(command, true, &frame.payload) {
                    lock(relay).on_status_snapshot(&snapshot, now_ms);
                    self.status_seed = true;
                    self.pending_query = None;
                }
            }
            Command::GetCatalogue => {
                if let Ok(CommandMessage::CataloguePage(page)) = decode_command(command, true, &frame.payload) {
                    self.accept_catalogue_page(&page, relay, now_ms);
                }
            }
            _ => {
                if let Ok(CommandMessage::Ack(ack)) = decode_command(command, true, &frame.payload) {
                    lock(relay).on_command_ack(frame.seq, ack.status, now_ms);
                }
            }
        }
    }

    /// Appends one catalogue page, seeding the relay only when every entry has arrived.
    fn accept_catalogue_page(&mut self, page: &CataloguePage<'_>, relay: &Mutex<Relay>, now_ms: u64) {
        self.pending_query = None;
        self.catalogue_total = Some(page.total);
        for entry in page.entries() {
            self.catalogue_entries.push(CatalogueProcedure {
                id: entry.procedure_id.get(),
                label: entry.label.to_owned(),
                available: entry.flags.contains(CatalogueFlags::AVAILABLE),
                interactive: entry.flags.contains(CatalogueFlags::INTERACTIVE),
                calibration: entry.flags.contains(CatalogueFlags::CALIBRATION),
                takes_params: entry.flags.contains(CatalogueFlags::TAKES_PARAMS),
            });
        }
        if self.catalogue_entries.len() >= usize::from(page.total) {
            let procedures = std::mem::take(&mut self.catalogue_entries);
            lock(relay).on_catalogue(Catalogue { procedures }, now_ms);
            self.catalogue_seed = true;
        } else {
            self.catalogue_next_page = page.page.saturating_add(1);
        }
    }

    /// Honours `THROTTLE` by logging only (ADR-0039).
    fn handle_control(frame: &RawFrame) {
        if frame.flags.contains(LinkFlags::IS_ACK) {
            return;
        }
        if frame.id != Control::Throttle.wire_id() {
            return;
        }
        if let Ok(ControlMessage::Throttle(hint)) = decode_control(Control::Throttle, false, &frame.payload) {
            eprintln!(
                "THROTTLE stream {} {:?} rate {} chz ttl {} ms",
                hint.stream_id, hint.action, hint.rate_chz, hint.ttl_ms
            );
        }
    }
}

/// Locks the relay, recovering a guard if another thread panicked while holding it.
fn lock(relay: &Mutex<Relay>) -> MutexGuard<'_, Relay> {
    relay.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Sleeps for `delay`, returning `false` when the adapter is cancelled first.
async fn sleep_or_stop(delay: Duration, stop: &mut watch::Receiver<bool>) -> bool {
    tokio::select! {
        () = tokio::time::sleep(delay) => true,
        _ = stop.changed() => false,
    }
}

/// Converts a duration to whole milliseconds, saturating.
fn millis(duration: Duration) -> u64 {
    u64::try_from(duration.as_millis()).unwrap_or(u64::MAX)
}

/// Draws a jitter sample from the operating system's random source, with a mid-range fallback.
fn random_u16() -> u16 {
    let mut bytes = [0u8; 2];
    match getrandom::fill(&mut bytes) {
        Ok(()) => u16::from_le_bytes(bytes),
        Err(_) => u16::MAX / 2,
    }
}

#[cfg(test)]
mod tests {
    //! The link's pure encoders, proven without a Bridge.

    use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};

    use link_protocol::{
        BridgeStream, Caps, Class, Flags as LinkFlags, Hello, MAX_PC_TO_ROBOT_FRAME, PayloadCodec, RateEntry, Stream,
    };

    use super::{BASELINE_RATE_CHZ, LinkConfig, RawFrame, hello_payload, is_bridge_telemetry};

    /// Extracts a value, failing the test rather than unwrapping (the workspace denies `unwrap`).
    fn must<T>(value: Option<T>, what: &str) -> T {
        value.unwrap_or_else(|| {
            assert_eq!(what.len(), usize::MAX, "missing {what}");
            std::process::abort()
        })
    }

    /// `HELLO` carries the configured epoch, the motion/activity baseline and no capabilities.
    #[test]
    fn the_hello_carries_the_epoch_and_baseline_rates() {
        let endpoint = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 9_000));
        let config = LinkConfig::with_epoch(endpoint, b"token".to_vec(), 0xDEAD_BEEF);
        let mut payload = [0u8; MAX_PC_TO_ROBOT_FRAME];
        let length = must(hello_payload(&config, &mut payload), "the HELLO encodes");
        let hello = must(Hello::decode(&payload[..length]).ok(), "the HELLO decodes");

        assert_eq!(hello.epoch, 0xDEAD_BEEF, "the once-per-process epoch is carried");
        assert_eq!(hello.major, link_protocol::PROTOCOL_MAJOR);
        assert_eq!(hello.minor, link_protocol::PROTOCOL_MINOR);
        assert_eq!(hello.caps, Caps::from_bits(0), "v1 requests no optional capability");
        let entries: Vec<RateEntry> = hello.entries().collect();
        assert_eq!(entries.len(), 2, "the two baseline streams are requested");
        assert_eq!(
            entries.first(),
            Some(&RateEntry::enabled(Stream::Motion.wire_id(), BASELINE_RATE_CHZ))
        );
        assert_eq!(
            entries.get(1),
            Some(&RateEntry::enabled(Stream::Activity.wire_id(), BASELINE_RATE_CHZ))
        );
    }

    /// Bridge telemetry must not refresh the robot's stall timer.
    #[test]
    fn bridge_telemetry_is_not_a_robot_frame() {
        let frame = |class: Class, id: u16| RawFrame {
            class,
            flags: LinkFlags::default(),
            id,
            seq: 1,
            payload: Vec::new(),
        };

        assert!(
            is_bridge_telemetry(&frame(Class::Stream, BridgeStream::Status.wire_id())),
            "BRIDGE_STATUS is Bridge telemetry"
        );
        assert!(
            !is_bridge_telemetry(&frame(Class::Stream, Stream::Power.wire_id())),
            "a robot stream is a robot frame"
        );
        assert!(
            !is_bridge_telemetry(&frame(Class::Event, 1)),
            "an event is a robot frame"
        );
        assert!(
            !is_bridge_telemetry(&frame(Class::Stream, 0x00FF)),
            "an unassigned stream id is not Bridge telemetry"
        );
    }
}
