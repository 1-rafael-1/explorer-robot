//! The stub's servers: the preamble's Link listener and the Beacon broadcaster.
//!
//! The listener is plain blocking `std::net` on its own threads, so the library spins up from a
//! plain `std` test without an async runtime, and a tokio daemon can import it later without
//! friction. Each connection runs the preamble on the auth deadline, then the scripted robot
//! traffic; the Beacon thread broadcasts the scenario's datagram at its configured interval.

use std::{
    io::{self, Read, Write},
    net::{Ipv4Addr, SocketAddr, TcpListener, TcpStream, UdpSocket},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use bridge_net::{beacon::LINK_MAJOR, preamble as net_preamble};
use remote_protocol::{
    AckOutcome, AckTracker, Caps, Class, Command, CommandMessage, Control, ControlMessage, Direction, Event,
    Flags as LinkFlags, Frame, MAX_ROBOT_TO_PC_FRAME, PROTOCOL_MAJOR, PROTOCOL_MINOR, PayloadCodec, Ping, PingAck,
    ProcedureId, RateEntry, RateFlags, Seq, Status, Stream, StreamSample, encode_command, encode_control,
    encode_hello_ack, encode_stream,
    parser::PcToRobotParser,
    payload::{
        command::{AckPayload, CatalogueEntry, GetCatalogue, StatusSnapshot, encode_catalogue_page},
        event::EVENT_PAYLOAD_MAX,
        stream::{
            Activity, ActivityStage, Calibration, CalibrationStatus, LIDAR_SLOTS, LidarCloud, Motion, Perception, Power,
        },
    },
};

use crate::{
    discovery::Beacon,
    preamble::{
        DenyReason, HEADER_LEN, MAX_PAYLOAD_LEN, Message, NONCE_LEN, Received, bridge_proof, client_proof,
        proofs_match, try_decode, write_message,
    },
    script::{Deny, Scenario, ScriptEvent},
};

/// How often the accept loop polls the listener between connections.
const ACCEPT_POLL_MS: u64 = 10;

/// How often the Beacon loop wakes to check the clock and the stop signal.
const BEACON_WAKE_MS: u64 = 25;

/// How often the traffic loop wakes to schedule streams and events.
const TRAFFIC_READ_MS: u64 = 25;

/// How long a single preamble read may block before the deadline is re-checked.
const PREAMBLE_READ_SLICE: Duration = Duration::from_millis(100);

/// How long a write to a stalled peer may block before the connection is dropped.
const WRITE_TIMEOUT: Duration = Duration::from_millis(500);

/// The largest single read chunk.
const READ_CHUNK: usize = 4096;

/// A busy denial's default retry hint (ADR-0039's 4 s).
const BUSY_RETRY_MS: u32 = 4_000;

/// ADR-0033's baud; the parser derives its assembly timeout from it.
const BAUD: u32 = 921_600;

/// A running stub: one Link listener and one Beacon broadcaster, on their own threads.
///
/// Dropping the handle stops both servers and waits for their threads to exit.
pub struct Stub {
    /// The address the Link listener bound.
    link_addr: SocketAddr,
    /// The address the Beacon datagrams are sent to.
    beacon_target: SocketAddr,
    /// The shared stop signal.
    shutdown: Arc<AtomicBool>,
    /// The threads to join on stop; the accept thread registers connection threads here.
    joins: Arc<Mutex<Vec<JoinHandle<()>>>>,
}

impl Stub {
    /// Starts serving `scenario`, returning once the listener and the Beacon socket are bound.
    ///
    /// # Errors
    ///
    /// Returns the bind errors the sockets produced.
    pub fn start(scenario: Scenario) -> io::Result<Self> {
        let scenario = Arc::new(scenario);
        let link_port = scenario.link_port.unwrap_or(0);
        let listener = TcpListener::bind((scenario.bind, link_port))?;
        let link_addr = listener.local_addr()?;
        listener.set_nonblocking(true)?;
        let beacon_socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;
        if is_broadcast(scenario.beacon.target) {
            beacon_socket.set_broadcast(true)?;
        }

        let shutdown = Arc::new(AtomicBool::new(false));
        let live = Arc::new(AtomicBool::new(false));
        let joins: Arc<Mutex<Vec<JoinHandle<()>>>> = Arc::new(Mutex::new(Vec::new()));
        let stub = Self {
            link_addr,
            beacon_target: scenario.beacon.target,
            shutdown: Arc::clone(&shutdown),
            joins: Arc::clone(&joins),
        };

        let beacon_scenario = Arc::clone(&scenario);
        let beacon_shutdown = Arc::clone(&shutdown);
        let beacon_port = link_addr.port();
        let beacon_thread =
            thread::spawn(move || broadcast_beacons(&beacon_socket, &beacon_scenario, beacon_port, &beacon_shutdown));
        if let Ok(mut registered) = joins.lock() {
            registered.push(beacon_thread);
        }

        let accept_scenario = Arc::clone(&scenario);
        let accept_shutdown = Arc::clone(&shutdown);
        let accept_live = Arc::clone(&live);
        let accept_joins = Arc::clone(&joins);
        let accept_thread = thread::spawn(move || {
            accept_connections(
                &listener,
                &accept_scenario,
                &accept_live,
                &accept_shutdown,
                &accept_joins,
            );
        });
        if let Ok(mut registered) = joins.lock() {
            registered.push(accept_thread);
        }
        Ok(stub)
    }

    /// The address the Link listener bound.
    #[must_use]
    pub const fn link_addr(&self) -> SocketAddr {
        self.link_addr
    }

    /// The address the Beacon datagrams are sent to.
    #[must_use]
    pub const fn beacon_target(&self) -> SocketAddr {
        self.beacon_target
    }

    /// Stops the servers and waits for their threads to exit.
    ///
    /// Stopping is idempotent; the connection threads notice the signal within one read slice.
    pub fn stop(&self) {
        self.shutdown.store(true, Ordering::Relaxed);
        if let Ok(mut joins) = self.joins.lock() {
            for handle in joins.drain(..) {
                let _ = handle.join();
            }
        }
    }
}

impl Drop for Stub {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Accepts connections until the stop signal, serving each on its own thread.
fn accept_connections(
    listener: &TcpListener,
    scenario: &Arc<Scenario>,
    live: &Arc<AtomicBool>,
    shutdown: &Arc<AtomicBool>,
    joins: &Arc<Mutex<Vec<JoinHandle<()>>>>,
) {
    loop {
        if shutdown.load(Ordering::Relaxed) {
            return;
        }
        match listener.accept() {
            Ok((stream, _)) => {
                let scenario = Arc::clone(scenario);
                let live = Arc::clone(live);
                let shutdown = Arc::clone(shutdown);
                let handle = thread::spawn(move || serve_connection(stream, &scenario, &live, &shutdown));
                if let Ok(mut registered) = joins.lock() {
                    registered.push(handle);
                }
            }
            Err(error) if matches!(error.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {
                thread::sleep(Duration::from_millis(ACCEPT_POLL_MS));
            }
            Err(_) => return,
        }
    }
}

/// Broadcasts the scenario's Beacon until the stop signal.
fn broadcast_beacons(socket: &UdpSocket, scenario: &Arc<Scenario>, link_port: u16, shutdown: &Arc<AtomicBool>) {
    let beacon = Beacon {
        major: LINK_MAJOR,
        device_id: scenario.identity.device_id,
        name: scenario.identity.name,
        tcp_port: scenario.beacon.tcp_port.unwrap_or(link_port),
        flags: scenario.identity.flags,
    };
    let mut datagram = [0u8; crate::discovery::ENCODED_LEN];
    let Ok(len) = beacon.encode_into(&mut datagram) else {
        return;
    };
    let interval = Duration::from_millis(scenario.beacon.interval_ms);
    let mut next = Instant::now();
    loop {
        if shutdown.load(Ordering::Relaxed) {
            return;
        }
        let now = Instant::now();
        if now >= next {
            // A broadcast can fail on an interface-less host; the next interval retries.
            let _ = socket.send_to(&datagram[..len], scenario.beacon.target);
            next = now + interval;
        }
        thread::sleep(Duration::from_millis(BEACON_WAKE_MS));
    }
}

/// Serves one connection: the preamble, then the scripted traffic.
fn serve_connection(
    mut stream: TcpStream,
    scenario: &Arc<Scenario>,
    live: &Arc<AtomicBool>,
    shutdown: &Arc<AtomicBool>,
) {
    let _ = stream.set_nodelay(true);
    let deadline = Instant::now() + Duration::from_millis(scenario.auth_deadline_ms);
    let Ok(received) = receive_message(&mut stream, deadline, shutdown) else {
        return;
    };
    let Message::ClientHello { nonce } = received.message else {
        return;
    };

    // A scripted denial is the operator's scenario and wins over the natural gates.
    if let Some(deny) = scenario.deny {
        let _ = write_denial(&mut stream, &deny);
        return;
    }
    if received.major != net_preamble::MAJOR {
        let _ = write_denial(
            &mut stream,
            &Deny {
                reason: DenyReason::Version,
                retry_ms: None,
            },
        );
        return;
    }
    // One authenticated client at a time (ADR-0037): a second connection while one is live is busy.
    if live.load(Ordering::Relaxed) {
        let _ = write_denial(
            &mut stream,
            &Deny {
                reason: DenyReason::Busy,
                retry_ms: Some(BUSY_RETRY_MS),
            },
        );
        return;
    }

    let bridge_nonce = fresh_nonce();
    let proof = bridge_proof(&scenario.token, &nonce, &bridge_nonce, &scenario.identity);
    let challenge = Message::BridgeChallenge {
        bridge_proof: proof,
        bridge_nonce,
        identity: scenario.identity,
    };
    if write_message(&mut stream, net_preamble::MAJOR, &challenge).is_err() {
        return;
    }
    let Ok(received) = receive_message(&mut stream, deadline, shutdown) else {
        return;
    };
    let Message::ClientAuth { proof } = received.message else {
        return;
    };
    let expected = client_proof(&scenario.token, &nonce, &bridge_nonce);
    if !proofs_match(&expected, &proof) {
        let _ = write_denial(
            &mut stream,
            &Deny {
                reason: DenyReason::Unauthorized,
                retry_ms: None,
            },
        );
        return;
    }

    // The client is authenticated before it is told so, so a second connection cannot slip in
    // between the verdict and the flag.
    live.store(true, Ordering::Relaxed);
    if write_message(&mut stream, net_preamble::MAJOR, &Message::Accept).is_err() {
        live.store(false, Ordering::Relaxed);
        return;
    }
    serve_traffic(&mut stream, scenario, shutdown);
    live.store(false, Ordering::Relaxed);
}

/// Writes one denial; the caller closes the connection by returning.
fn write_denial(stream: &mut TcpStream, deny: &Deny) -> io::Result<()> {
    write_message(
        stream,
        net_preamble::MAJOR,
        &Message::Deny {
            reason: deny.reason,
            retry_ms: deny.retry_ms,
        },
    )
}

/// A fresh bridge nonce; the hardware RNG's stand-in.
fn fresh_nonce() -> [u8; NONCE_LEN] {
    let mut nonce = [0u8; NONCE_LEN];
    if getrandom::fill(&mut nonce).is_err() {
        // A bench stub can proceed with a time-derived nonce; the proof still binds it.
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0, |since| since.as_nanos());
        nonce[..16].copy_from_slice(&nanos.to_le_bytes());
    }
    nonce
}

/// Reads one preamble message, bounded by the auth deadline and the stop signal.
fn receive_message(stream: &mut TcpStream, deadline: Instant, shutdown: &AtomicBool) -> io::Result<Received> {
    let mut buffer = [0u8; HEADER_LEN + MAX_PAYLOAD_LEN];
    let mut filled = 0usize;
    loop {
        match try_decode(&buffer[..filled]) {
            Ok(Some(received)) => return Ok(received),
            Ok(None) => {}
            Err(error) => return Err(io::Error::new(io::ErrorKind::InvalidData, error)),
        }
        if shutdown.load(Ordering::Relaxed) {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "the stub is stopping"));
        }
        let now = Instant::now();
        if now >= deadline {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "the auth deadline elapsed"));
        }
        let slice = deadline.saturating_duration_since(now).min(PREAMBLE_READ_SLICE);
        stream.set_read_timeout(Some(slice))?;
        match stream.read(&mut buffer[filled..]) {
            Ok(0) => return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "closed mid-preamble")),
            Ok(count) => filled += count,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut | io::ErrorKind::Interrupted
                ) => {}
            Err(error) => return Err(error),
        }
    }
}

/// The traffic loop's per-connection state.
struct TrafficState {
    /// The per-stream sequence numbers, indexed by [`Stream::index`].
    stream_seq: [u16; Stream::COUNT],
    /// The per-stream next-due timestamps in milliseconds.
    next_due: [u64; Stream::COUNT],
    /// The event sequence, per Boot Id.
    event_seq: u16,
    /// The events still to send.
    events: std::vec::IntoIter<ScriptEvent>,
    /// The event in flight, with its encoded payload.
    pending: Option<PendingEvent>,
    /// The retransmission tracker for the event in flight.
    tracker: AckTracker,
}

impl TrafficState {
    /// Builds the state for a scenario's traffic.
    fn new(scenario: &Scenario) -> Self {
        Self {
            stream_seq: [0; Stream::COUNT],
            next_due: [0; Stream::COUNT],
            event_seq: 0,
            events: scenario.traffic.events.clone().into_iter(),
            pending: None,
            tracker: AckTracker::new(scenario.traffic.event_retry),
        }
    }
}

/// The event in flight, with its encoded payload.
struct PendingEvent {
    /// The event id.
    event: Event,
    /// The event's sequence.
    seq: u16,
    /// The encoded payload.
    payload: [u8; EVENT_PAYLOAD_MAX],
    /// The payload's length.
    len: usize,
}

/// Serves the scripted traffic behind `ACCEPT` until the peer or the stop signal ends it.
fn serve_traffic(stream: &mut TcpStream, scenario: &Scenario, shutdown: &AtomicBool) {
    if stream.set_write_timeout(Some(WRITE_TIMEOUT)).is_err() {
        return;
    }
    if stream
        .set_read_timeout(Some(Duration::from_millis(TRAFFIC_READ_MS)))
        .is_err()
    {
        return;
    }
    let mut parser = PcToRobotParser::new(Direction::PcToRobot, BAUD);
    let start = Instant::now();
    let mut state = TrafficState::new(scenario);
    let mut chunk = [0u8; READ_CHUNK];
    loop {
        if shutdown.load(Ordering::Relaxed) {
            return;
        }
        let now = elapsed_ms(start);
        send_due_streams(stream, scenario, &mut state, now);
        advance_events(stream, scenario, &mut state, now);
        match stream.read(&mut chunk) {
            Ok(0) => return,
            Ok(count) => {
                let stamp = u32::try_from(now).unwrap_or(u32::MAX);
                parser.feed(&chunk[..count], stamp, |frame| {
                    handle_frame(stream, scenario, &mut state, frame);
                });
            }
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut | io::ErrorKind::Interrupted
                ) => {}
            Err(_) => return,
        }
    }
}

/// Sends every stream sample whose rate interval elapsed.
fn send_due_streams(stream: &mut TcpStream, scenario: &Scenario, state: &mut TrafficState, now: u64) {
    for rate in &scenario.traffic.streams {
        if rate.rate_chz == 0 {
            continue;
        }
        let slot = rate.stream.index();
        if now < state.next_due[slot] {
            continue;
        }
        let seq = state.stream_seq[slot].wrapping_add(1);
        state.stream_seq[slot] = seq;
        let mut payload = [0u8; MAX_ROBOT_TO_PC_FRAME];
        let sample = sample_for(rate.stream, seq);
        let Ok(len) = encode_stream(&sample, &mut payload) else {
            continue;
        };
        send_frame(
            stream,
            Class::Stream,
            LinkFlags::default(),
            rate.stream.wire_id(),
            seq,
            &payload[..len],
        );
        state.next_due[slot] = now + stream_period_ms(rate.rate_chz);
    }
}

/// The sample period of a rate, in milliseconds.
fn stream_period_ms(rate_chz: u16) -> u64 {
    100_000 / u64::from(rate_chz.max(1))
}

/// Synthesizes the stream's `seq`-th sample: a plausible, deterministic robot.
fn sample_for(stream: Stream, seq: u16) -> StreamSample<'static> {
    match stream {
        Stream::Power => StreamSample::Power(Power {
            level: u8::try_from(50 + seq % 51).unwrap_or(100),
            voltage_mv: 7_400,
        }),
        Stream::Motion => StreamSample::Motion(Motion {
            left: i8::try_from(i32::from(seq % 201) - 100).unwrap_or(0),
            right: i8::try_from(i32::from(seq.wrapping_mul(7) % 201) - 100).unwrap_or(0),
        }),
        Stream::Activity => StreamSample::Activity(Activity {
            procedure_id: ProcedureId::IDLE,
            stage: ActivityStage::Running,
            percent: None,
            detail: "",
        }),
        Stream::Perception => StreamSample::Perception(Perception::from_bits(u8::try_from(seq % 4).unwrap_or(0))),
        Stream::LidarCloud => StreamSample::LidarCloud(LidarCloud {
            slots: [LidarCloud::NO_RETURN; LIDAR_SLOTS],
        }),
        Stream::Calibration => StreamSample::Calibration(Calibration {
            motor: CalibrationStatus::Loaded,
            imu: CalibrationStatus::Loaded,
            mag: CalibrationStatus::Loaded,
            distance: CalibrationStatus::Loaded,
        }),
    }
}

/// Starts the next scripted event, or retransmits the one in flight.
fn advance_events(stream: &mut TcpStream, scenario: &Scenario, state: &mut TrafficState, now: u64) {
    if state.tracker.is_in_flight() {
        match state.tracker.poll(now) {
            AckOutcome::Retransmit { .. } => send_pending(stream, state, true),
            AckOutcome::Indeterminate { .. } => state.pending = None,
            AckOutcome::Idle | AckOutcome::DiscardedOnBootChange { .. } => {}
        }
        return;
    }
    let Some(event) = state.events.next() else {
        return;
    };
    let seq = state.event_seq.wrapping_add(1);
    state.event_seq = seq;
    let mut payload = [0u8; EVENT_PAYLOAD_MAX];
    let Ok(len) = event.encode_into(&mut payload) else {
        return;
    };
    state.pending = Some(PendingEvent {
        event: event.event(),
        seq,
        payload,
        len,
    });
    if !state.tracker.begin(Seq::new(seq), scenario.traffic.boot_id, now) {
        return;
    }
    send_pending(stream, state, false);
}

/// Sends the pending event, flagged as a retransmission when `retry` is set.
fn send_pending(stream: &mut TcpStream, state: &TrafficState, retry: bool) {
    let Some(pending) = &state.pending else {
        return;
    };
    let flags = if retry {
        LinkFlags::ACK_REQUESTED | LinkFlags::RETRY
    } else {
        LinkFlags::ACK_REQUESTED
    };
    send_frame(
        stream,
        Class::Event,
        flags,
        pending.event.wire_id(),
        pending.seq,
        &pending.payload[..pending.len],
    );
}

/// Applies one decoded PC-to-robot frame, scripting the replies.
///
/// An event ACK retires the event in flight whatever its diagnostic status; an ACK for another
/// sequence is a duplicate the tracker already ignored, and falls through to nothing.
fn handle_frame(stream: &mut TcpStream, scenario: &Scenario, state: &mut TrafficState, frame: Frame<'_>) {
    match frame.header.class {
        Class::Control if !frame.header.flags.contains(LinkFlags::IS_ACK) => handle_control(stream, scenario, frame),
        Class::Command
            if frame.header.flags.contains(LinkFlags::ACK_REQUESTED)
                && !frame.header.flags.contains(LinkFlags::IS_ACK) =>
        {
            handle_command(stream, scenario, frame);
        }
        Class::Event if frame.header.flags.contains(LinkFlags::IS_ACK) && state.tracker.on_ack(frame.header.seq) => {
            state.pending = None;
        }
        _ => {}
    }
}

/// Replies to `HELLO` and `PING` as the robot would: `IS_ACK` on the same id and sequence.
fn handle_control(stream: &mut TcpStream, scenario: &Scenario, frame: Frame<'_>) {
    match Control::from_wire(frame.header.id) {
        Some(Control::Hello) => {
            if remote_protocol::decode_control(Control::Hello, false, frame.payload).is_err() {
                return;
            }
            let entries = scenario.traffic.streams.iter().map(|rate| RateEntry {
                stream_id: rate.stream.wire_id(),
                rate_chz: rate.rate_chz,
                flags: RateFlags::ENABLED,
            });
            let mut payload = [0u8; MAX_ROBOT_TO_PC_FRAME];
            let Ok(len) = encode_hello_ack(
                PROTOCOL_MAJOR,
                PROTOCOL_MINOR,
                Caps::from_bits(0),
                scenario.traffic.boot_id,
                Status::Ok,
                entries,
                &mut payload,
            ) else {
                return;
            };
            send_frame(
                stream,
                Class::Control,
                LinkFlags::IS_ACK,
                Control::Hello.wire_id(),
                frame.header.seq.get(),
                &payload[..len],
            );
        }
        Some(Control::Ping) => {
            if Ping::decode(frame.payload).is_err() {
                return;
            }
            let mut payload = [0u8; MAX_ROBOT_TO_PC_FRAME];
            let Ok(len) = encode_control(&ControlMessage::PingAck(PingAck { status: Status::Ok }), &mut payload) else {
                return;
            };
            send_frame(
                stream,
                Class::Control,
                LinkFlags::IS_ACK,
                Control::Ping.wire_id(),
                frame.header.seq.get(),
                &payload[..len],
            );
        }
        _ => {}
    }
}

/// Replies to one command: the configured status, or the typed query responses.
fn handle_command(stream: &mut TcpStream, scenario: &Scenario, frame: Frame<'_>) {
    let Some(command) = Command::from_wire(frame.header.id) else {
        return;
    };
    let mut payload = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let len = match command {
        Command::GetStatus => encode_command(&CommandMessage::StatusSnapshot(status_snapshot()), &mut payload),
        Command::GetCatalogue => {
            let page = GetCatalogue::decode(frame.payload).map_or(0, |request| request.page);
            encode_catalogue_page(0, page, catalog_entries(), &mut payload)
        }
        _ => encode_command(
            &CommandMessage::Ack(AckPayload {
                status: scenario.traffic.ack_status(command),
                result: &[],
            }),
            &mut payload,
        ),
    };
    let Ok(len) = len else {
        return;
    };
    send_frame(
        stream,
        Class::Command,
        LinkFlags::IS_ACK,
        frame.header.id,
        frame.header.seq.get(),
        &payload[..len],
    );
}

/// The fixed, plausible snapshot the stub answers `GET_STATUS` with.
fn status_snapshot() -> StatusSnapshot<'static> {
    StatusSnapshot {
        level: 87,
        voltage_mv: 7_400,
        left: 0,
        right: 0,
        procedure_id: ProcedureId::IDLE,
        stage: ActivityStage::Running,
        percent: None,
        detail: "",
        flags: Perception::default(),
        calibration: Calibration {
            motor: CalibrationStatus::Loaded,
            imu: CalibrationStatus::Loaded,
            mag: CalibrationStatus::Loaded,
            distance: CalibrationStatus::Loaded,
        },
    }
}

/// The empty catalogue the stub answers `GET_CATALOGUE` with: no startable Procedures.
fn catalog_entries() -> impl Iterator<Item = CatalogueEntry<'static>> {
    core::iter::empty()
}

/// Encodes and writes one Link frame.
fn send_frame(stream: &mut TcpStream, class: Class, flags: LinkFlags, id: u16, seq: u16, payload: &[u8]) {
    let Ok(frame) = Frame::new(PROTOCOL_MAJOR, class, flags, id, Seq::new(seq), payload) else {
        return;
    };
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let Ok(len) = frame.encode_into(&mut buffer) else {
        return;
    };
    let _ = stream.write_all(&buffer[..len]);
}

/// The milliseconds since `start`.
fn elapsed_ms(start: Instant) -> u64 {
    u64::try_from(start.elapsed().as_millis()).unwrap_or(u64::MAX)
}

/// Returns `true` when `addr` is the IPv4 broadcast address.
const fn is_broadcast(addr: SocketAddr) -> bool {
    match addr {
        SocketAddr::V4(v4) => v4.ip().is_broadcast(),
        SocketAddr::V6(_) => false,
    }
}
