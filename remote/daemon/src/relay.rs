//! The app-facing relay and robot Session state (ADR-0039).
//!
//! The relay is the daemon's deep module. It owns the latest-wins stream slots, the bounded event
//! ring with its per-app cursors and completion attribution, `last_terminal`, the Catalogue and the
//! command map. It is transport agnostic: the `link` adapter feeds decoded `remote-protocol`
//! payloads in through the `on_*` methods and drains [`Relay::poll_link`], while the `app` adapter
//! registers connections with [`Relay::connect_app`] and drains [`Relay::poll`].
//!
//! Like `remote-link`, every entry point takes a plain millisecond timestamp rather than reading a
//! clock, so the whole module is host-tested with a fake link and an in-memory app transport.
//!
//! # The three app lanes
//!
//! [`Relay::poll`] drains one app connection in the order ADR-0039 fixes: a pending snapshot
//! first, then the command results addressed to that connection, then the latest-wins stream
//! slots it has not seen, then the shared event ring from its own cursor. A cursor that has fallen
//! behind the ring's oldest entry receives one `event_overflow{dropped}` notice and jumps. There is
//! no unbounded per-app queue; a slow app only ever sees the newest sample of each stream.
//!
//! # Unit and sign conversion
//!
//! The wire's distances are already centimetres. Angles are the one place the sign flips: the
//! wire's `ROTATE_BY` and `ROTATE_COMPLETED` are clockwise, while the App Boundary is
//! counter-clockwise positive, so every degree value is negated once at this boundary (ADR-0038,
//! ADR-0039). Unknown wire values become `null`, never a sentinel.

use std::collections::{BTreeMap, VecDeque};

use remote_protocol::{
    BridgeStation as WireBridgeStation, BridgeStatus as WireBridgeStatus, BridgeStatusFlags as WireBridgeFlags,
    Command, CommandMessage, CompletionStatus as WireCompletionStatus, OBSTACLE_SOURCE_LIDAR, ProcedureId,
    Status as WireStatus,
    payload::{
        command::{DriveDistance, RotateBy, StartProcedure, StatusSnapshot, StopMotion, StopProcedure},
        event::{
            CalibrationKind, CalibrationLoadStatus, CalibrationLoaded, DriveDistanceCompleted, EventOverflow,
            FloorDrop, Obstacle, ProcedureCompleted, RotateCompleted,
        },
        stream::{
            Activity, ActivityStage, Calibration, CalibrationStatus as WireCalibrationStatus, LidarCloud, Motion,
            Perception as WirePerception, Power,
        },
    },
};

use crate::schema::{
    BridgeStation, BridgeStatus, CalibrationStatus, Catalogue, CommandFrame, CommandResult, CompletionStatus,
    Connection, EventEntry, EventName, EventValue, MANUAL_DRIVE_ID, Outcome, PROTOCOL, ServerMessage, Snapshot, Stage,
    Status, StreamEntry, StreamName, StreamValue,
};

/// A relay connection identifier.
///
/// The app adapter allocates one per WebSocket connection. Identifiers are relay-owned and never
/// reused within a process.
pub type AppId = u64;

/// The number of events the shared replay ring retains (ADR-0039).
pub const EVENT_RING_CAPACITY: usize = 300;

/// The number of command records the command map retains (ADR-0039).
pub const COMMAND_MAP_CAPACITY: usize = 64;

/// The silence after which an app is no longer live (ADR-0039).
pub const APP_LIVENESS_MS: u64 = 2_000;

/// The acknowledgement window for a volatile command (ADR-0039).
pub const VOLATILE_ACK_TIMEOUT_MS: u64 = 3_000;

/// The retry interval for an idempotent command (ADR-0039).
pub const IDEMPOTENT_RETRY_MS: u64 = 1_000;

/// The number of times an idempotent command is retried before it times out (ADR-0039).
pub const IDEMPOTENT_RETRIES: u8 = 3;

/// How long a completion record outlives its acknowledgement (ADR-0039).
pub const COMPLETION_RETENTION_MS: u64 = 60_000;

/// The number of stream slots: one per wire stream.
const STREAM_SLOTS: usize = 6;

/// The first wire command sequence, matching the protocol's convention that zero is skipped.
const FIRST_WIRE_SEQ: u16 = 1;

/// The slot index of a stream name.
///
/// The order matches `remote_protocol::Stream::ALL`, so a snapshot's streams arrive in wire order.
const fn slot_of(name: StreamName) -> usize {
    match name {
        StreamName::Power => 0,
        StreamName::Motion => 1,
        StreamName::Activity => 2,
        StreamName::Perception => 3,
        StreamName::Lidar => 4,
        StreamName::Calibration => 5,
    }
}

/// One app connection's relay state.
#[derive(Clone, Debug)]
struct App {
    /// The daemon receive time of the last heartbeat or connection.
    last_heartbeat_ms: u64,
    /// Whether the heartbeat was fresh at the last [`Relay::tick`].
    alive: bool,
    /// Whether the next [`Relay::poll`] must lead with a snapshot.
    pending_snapshot: bool,
    /// The daemon stream sequence each slot has already delivered to this app.
    stream_cursors: [u64; STREAM_SLOTS],
    /// The absolute ring position of the next event this app has not seen.
    event_cursor: u64,
    /// Command results addressed to this connection, drained before streams.
    results: VecDeque<CommandResult>,
    /// The connection-state sequence this app has already been sent.
    connection_cursor: u64,
    /// The Bridge-status sequence this app has already been sent.
    bridge_cursor: u64,
}

/// The app that originated a completion event, kept so the event can be attributed per tab.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CompletionOrigin {
    /// The originating app connection.
    app: AppId,
    /// The originating app's `command_seq`.
    command_seq: u64,
}

/// One entry in the shared replay ring.
#[derive(Clone, Debug)]
struct RingEntry {
    /// The event exactly as the app-facing schema stores it; completion sequences are filled per app.
    event: EventEntry,
    /// The originating app, for the two drive completions only.
    origin: Option<CompletionOrigin>,
}

impl RingEntry {
    /// Materialises this entry for `app`, attributing a completion to its originator only.
    fn for_app(&self, app: AppId) -> EventEntry {
        let mut event = self.event.clone();
        match &mut event.v {
            EventValue::DriveDistanceCompleted { command_seq, .. }
            | EventValue::RotateCompleted { command_seq, .. } => {
                *command_seq = match self.origin {
                    Some(origin) if origin.app == app => Some(origin.command_seq),
                    _ => None,
                };
            }
            _ => {}
        }
        event
    }
}

/// Whether a command is sent once or retried with the `RETRY` flag (ADR-0039).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CommandKind {
    /// A relative, non-idempotent command: sent once and never retried.
    Volatile,
    /// A Procedure or stop command: retried with `RETRY` before timing out.
    Idempotent,
}

/// The relay's record of a claim it started, used only by the app-frozen rule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DaemonClaim {
    /// A Manual Drive the relay issued (`drive_distance` or `rotate_by`).
    Manual,
    /// The Procedure id a `start_procedure` command started.
    Procedure(u16),
}

/// What an acknowledged command does to the daemon's claim record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ClaimAction {
    /// The command starts a claim the daemon owns.
    Set(DaemonClaim),
    /// The command stops the named Procedure's claim, if the daemon owns it.
    ClearProcedure(u16),
    /// The command stops any daemon-owned claim.
    ClearAll,
}

/// One record in the command map: a wire sequence mapped back to its originating app.
#[derive(Clone, Debug)]
struct CommandRecord {
    /// The originating app connection.
    app: AppId,
    /// The originating app's `command_seq`.
    app_seq: u64,
    /// The command name the app sent, echoed in the result.
    name: String,
    /// Whether the command is still queued or in flight, as opposed to a completion record.
    active: bool,
    /// When the command was submitted, used to evict the oldest stale record.
    created_ms: u64,
    /// When a completion record stops being useful, set once the flight ends.
    expires_ms: Option<u64>,
}

/// A command waiting for the single in-flight slot.
#[derive(Clone, Debug)]
struct QueuedCommand {
    /// The daemon-allocated wire sequence.
    wire_seq: u16,
    /// The originating app, or [`None`] for a daemon-originated safety stop.
    app: Option<AppId>,
    /// The originating app's `command_seq`, unused for a daemon command.
    app_seq: u64,
    /// The command name, echoed only to an app.
    name: String,
    /// The wire command id.
    command: Command,
    /// The typed payload the link adapter encodes.
    payload: CommandMessage<'static>,
    /// Whether the command is volatile or idempotent.
    kind: CommandKind,
    /// The claim change an acknowledgement applies.
    claim_action: Option<ClaimAction>,
}

/// The one command currently on the wire.
#[derive(Clone, Debug)]
struct InFlight {
    /// The daemon-allocated wire sequence.
    wire_seq: u16,
    /// The originating app, or [`None`] for a daemon-originated safety stop.
    app: Option<AppId>,
    /// The originating app's `command_seq`, unused for a daemon command.
    app_seq: u64,
    /// The command name, echoed only to an app.
    name: String,
    /// The wire command id, repeated on every retry.
    command: Command,
    /// The typed payload, repeated on every retry.
    payload: CommandMessage<'static>,
    /// Whether the command is volatile or idempotent.
    kind: CommandKind,
    /// The claim change an acknowledgement applies.
    claim_action: Option<ClaimAction>,
    /// When the frame was first sent, which anchors the volatile timeout.
    first_sent_ms: u64,
    /// When the frame was last sent, which anchors the idempotent retry interval.
    last_sent_ms: u64,
    /// How many `RETRY` transmissions have been made.
    retries: u8,
}

/// A parsed app command ready to queue.
#[derive(Clone, Debug)]
struct PreparedCommand {
    /// The command name the app sent.
    name: String,
    /// The wire command id.
    command: Command,
    /// The typed payload the link adapter encodes.
    payload: CommandMessage<'static>,
    /// Whether the command is volatile or idempotent.
    kind: CommandKind,
    /// The claim change an acknowledgement applies.
    claim_action: Option<ClaimAction>,
}

/// An immediate command rejection: a bad payload or a stale peer.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Rejection {
    /// The app-facing status to report.
    status: Status,
    /// The operator-facing reason.
    detail: String,
}

/// The app-facing relay and Session state (ADR-0039).
#[derive(Clone, Debug)]
pub struct Relay {
    /// The next app connection identifier.
    next_app: AppId,
    /// The live app connections.
    apps: BTreeMap<AppId, App>,
    /// Whether the robot Session is open.
    connected: bool,
    /// The last Boot Id learned from `HELLO_ACK`; kept through a reconnect.
    boot_id: Option<u32>,
    /// The latest-wins slot per stream, indexed by [`slot_of`].
    slots: [Option<StreamEntry>; STREAM_SLOTS],
    /// The daemon-owned monotonic sequence per stream, never reset.
    stream_seqs: [u64; STREAM_SLOTS],
    /// The last terminal Activity sample, retained until the next terminal or a Boot Id change.
    last_terminal: Option<StreamEntry>,
    /// The shared replay ring, oldest first.
    events: VecDeque<RingEntry>,
    /// The absolute number of events ever pushed, including those dropped from the ring.
    events_pushed: u64,
    /// The robot's Catalogue for the current Session.
    catalogue: Option<Catalogue>,
    /// The bounded command map: wire sequence to its originating app.
    commands: BTreeMap<u16, CommandRecord>,
    /// The next wire command sequence.
    next_wire_seq: u16,
    /// The command awaiting its acknowledgement, if any.
    in_flight: Option<InFlight>,
    /// Commands waiting behind the single in-flight slot.
    queue: VecDeque<QueuedCommand>,
    /// The live claim the daemon started, if any.
    claim: Option<DaemonClaim>,
    /// Whether the app-frozen timer has fired for the current silence.
    frozen: bool,
    /// The last app heartbeat (or connection) time; the app-frozen timer measures from here.
    ///
    /// [`Relay::tick`] never advances this: a tick only observes liveness. Refreshing it on every
    /// tick while the liveness window is still open would delay the freeze to roughly four seconds
    /// after the last heartbeat.
    last_heartbeat_ms: u64,
    /// Whether any app was live at the last connection change or [`Relay::tick`].
    app_live: bool,
    /// The latest app-facing connection state, shared by every tab (latest wins).
    connection: Option<Connection>,
    /// A sequence bumped on every connection update, so each tab sees one copy.
    connection_seq: u64,
    /// The Bridge's latest status; independent of the robot Boot Id.
    bridge: Option<BridgeStatus>,
    /// A sequence bumped on every Bridge-status update, so each tab sees one copy.
    bridge_seq: u64,
}

impl Relay {
    /// Creates an empty relay.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            next_app: 1,
            apps: BTreeMap::new(),
            connected: false,
            boot_id: None,
            slots: [const { None }; STREAM_SLOTS],
            stream_seqs: [0; STREAM_SLOTS],
            last_terminal: None,
            events: VecDeque::new(),
            events_pushed: 0,
            catalogue: None,
            commands: BTreeMap::new(),
            next_wire_seq: FIRST_WIRE_SEQ,
            in_flight: None,
            queue: VecDeque::new(),
            claim: None,
            frozen: false,
            last_heartbeat_ms: 0,
            app_live: false,
            connection: None,
            connection_seq: 0,
            bridge: None,
            bridge_seq: 0,
        }
    }

    /// Registers an app connection and returns its identifier.
    ///
    /// The connection starts with a pending snapshot and an event cursor at the ring tail, so a
    /// new tab receives the retained state and only the events that arrive afterwards.
    pub fn connect_app(&mut self, now_ms: u64) -> AppId {
        let app = self.next_app;
        self.next_app = self.next_app.saturating_add(1);
        self.apps.insert(
            app,
            App {
                last_heartbeat_ms: now_ms,
                alive: true,
                pending_snapshot: true,
                stream_cursors: [0; STREAM_SLOTS],
                event_cursor: self.events_pushed,
                results: VecDeque::new(),
                connection_cursor: 0,
                bridge_cursor: 0,
            },
        );
        self.app_live = true;
        self.last_heartbeat_ms = now_ms;
        if self.frozen {
            self.frozen = false;
            for state in self.apps.values_mut() {
                state.pending_snapshot = true;
            }
        }
        app
    }

    /// Removes an app connection; other tabs share the Session and are untouched.
    pub fn disconnect_app(&mut self, app: AppId) {
        let Some(removed) = self.apps.remove(&app) else {
            return;
        };
        if self.apps.is_empty() {
            self.last_heartbeat_ms = self.last_heartbeat_ms.max(removed.last_heartbeat_ms);
        }
        self.app_live = self.apps.values().any(|state| state.alive);
    }

    /// Opens a robot Session at `boot_id` and requests a fresh snapshot for every app.
    ///
    /// A same-Boot-Id reconnect keeps the slots, the ring, `last_terminal`, the Catalogue, the
    /// command map and the drive plan; a Boot Id change clears them and marks the outstanding
    /// command indeterminate (ADR-0039).
    pub fn session_up(&mut self, boot_id: u32, now_ms: u64) {
        if matches!(self.boot_id, Some(previous) if previous != boot_id) {
            self.reset_for_boot_change(now_ms);
        }
        self.connected = true;
        self.boot_id = Some(boot_id);
        for state in self.apps.values_mut() {
            state.pending_snapshot = true;
        }
        if let Some(flight) = &mut self.in_flight {
            flight.first_sent_ms = now_ms;
            flight.last_sent_ms = now_ms;
        }
        if self.apps.is_empty() {
            self.last_heartbeat_ms = now_ms;
        }
    }

    /// Closes the robot Session, keeping its retained state for a same-Boot-Id reconnect.
    pub const fn session_down(&mut self) {
        self.connected = false;
    }

    /// Publishes the latest app-facing connection state to every tab (ADR-0039).
    ///
    /// The relay keeps one latest-wins slot rather than a queue: a tab that is slow to drain sees
    /// the newest reachability state, never a replay of intermediates. A tab that connects later
    /// is sent the current state on its first [`Relay::poll`], before its snapshot.
    pub fn set_connection(&mut self, connection: Connection) {
        self.connection = Some(connection);
        self.connection_seq = self.connection_seq.saturating_add(1);
    }

    /// Returns the last Boot Id learned from the robot, `None` before the first Session.
    #[must_use]
    pub const fn boot_id(&self) -> Option<u32> {
        self.boot_id
    }

    /// Returns whether the robot Session is currently open.
    #[must_use]
    pub const fn is_connected(&self) -> bool {
        self.connected
    }

    /// Records a `POWER` sample and returns its daemon-sequenced stream entry.
    pub fn on_power(&mut self, power: &Power, now_ms: u64) -> StreamEntry {
        let value = StreamValue::Power {
            level: unknown_sentinel(power.level, remote_protocol::payload::stream::LEVEL_UNKNOWN),
            voltage_mv: unknown_sentinel(
                u32::from(power.voltage_mv),
                u32::from(remote_protocol::payload::stream::VOLTAGE_UNKNOWN_MV),
            ),
        };
        self.publish(StreamName::Power, value, now_ms)
    }

    /// Records a `MOTION` sample and returns its daemon-sequenced stream entry.
    pub fn on_motion(&mut self, motion: &Motion, now_ms: u64) -> StreamEntry {
        let value = StreamValue::Motion {
            left: motion.left,
            right: motion.right,
        };
        self.publish(StreamName::Motion, value, now_ms)
    }

    /// Records an `ACTIVITY` sample and returns its daemon-sequenced stream entry.
    ///
    /// A terminal stage becomes `last_terminal`, retained until the next terminal sample or a
    /// Boot Id change.
    pub fn on_activity(&mut self, activity: &Activity<'_>, now_ms: u64) -> StreamEntry {
        let value = activity_value(activity);
        self.publish(StreamName::Activity, value, now_ms)
    }

    /// Records a `PERCEPTION` sample and returns its daemon-sequenced stream entry.
    pub fn on_perception(&mut self, perception: WirePerception, now_ms: u64) -> StreamEntry {
        let value = perception_value(perception);
        self.publish(StreamName::Perception, value, now_ms)
    }

    /// Records a `LIDAR_CLOUD` sample and returns its daemon-sequenced stream entry.
    pub fn on_lidar(&mut self, lidar: &LidarCloud, now_ms: u64) -> StreamEntry {
        let value = StreamValue::Lidar {
            ranges_cm: lidar
                .slots
                .iter()
                .map(|slot| unknown_sentinel(*slot, LidarCloud::NO_RETURN))
                .collect(),
            first_deg: 0,
            step_deg: 1,
            clockwise: true,
        };
        self.publish(StreamName::Lidar, value, now_ms)
    }

    /// Records a `CALIBRATION` sample and returns its daemon-sequenced stream entry.
    pub fn on_calibration(&mut self, calibration: &Calibration, now_ms: u64) -> StreamEntry {
        let value = calibration_value(calibration);
        self.publish(StreamName::Calibration, value, now_ms)
    }

    /// Seeds the five streams a `GET_STATUS` response carries.
    ///
    /// `GET_STATUS` mirrors `POWER`, `MOTION`, `ACTIVITY`, `PERCEPTION` and `CALIBRATION`; the
    /// `LiDAR` cloud is not part of it and stays unseen until a scan arrives.
    pub fn on_status_snapshot(&mut self, status: &StatusSnapshot<'_>, now_ms: u64) {
        let power = StreamValue::Power {
            level: unknown_sentinel(status.level, remote_protocol::payload::stream::LEVEL_UNKNOWN),
            voltage_mv: unknown_sentinel(
                u32::from(status.voltage_mv),
                u32::from(remote_protocol::payload::stream::VOLTAGE_UNKNOWN_MV),
            ),
        };
        let _ = self.publish(StreamName::Power, power, now_ms);
        let motion = StreamValue::Motion {
            left: status.left,
            right: status.right,
        };
        let _ = self.publish(StreamName::Motion, motion, now_ms);
        let activity = StreamValue::Activity {
            id: status.procedure_id.get(),
            stage: activity_stage(status.stage),
            percent: status.percent,
            detail: status.detail.to_owned(),
        };
        let _ = self.publish(StreamName::Activity, activity, now_ms);
        let perception = perception_value(status.flags);
        let _ = self.publish(StreamName::Perception, perception, now_ms);
        let calibration = calibration_value(&status.calibration);
        let _ = self.publish(StreamName::Calibration, calibration, now_ms);
    }

    /// Stores an `OBSTACLE` event with the wire's `(boot_id, seq)`.
    pub fn on_obstacle(&mut self, seq: u16, obstacle: &Obstacle, now_ms: u64) {
        let source = if obstacle.source == OBSTACLE_SOURCE_LIDAR {
            "lidar"
        } else {
            "unknown"
        };
        self.push_event(
            seq,
            now_ms,
            EventName::Obstacle,
            EventValue::Obstacle {
                detected: obstacle.detected != 0,
                source: source.to_owned(),
            },
            None,
        );
    }

    /// Stores a `FLOOR_DROP` event with the wire's `(boot_id, seq)`.
    pub fn on_floor_drop(&mut self, seq: u16, floor_drop: &FloorDrop, now_ms: u64) {
        self.push_event(
            seq,
            now_ms,
            EventName::FloorDrop,
            EventValue::FloorDrop {
                detected: floor_drop.detected != 0,
            },
            None,
        );
    }

    /// Stores a `PROCEDURE_COMPLETED` event with the wire's `(boot_id, seq)`.
    pub fn on_procedure_completed(&mut self, seq: u16, completed: &ProcedureCompleted<'_>, now_ms: u64) {
        if let Some(DaemonClaim::Procedure(id)) = self.claim
            && id == completed.procedure_id.get()
        {
            self.claim = None;
        }
        self.push_event(
            seq,
            now_ms,
            EventName::ProcedureCompleted,
            EventValue::ProcedureCompleted {
                id: completed.procedure_id.get(),
                stage: activity_stage(completed.stage),
                detail: completed.detail.to_owned(),
            },
            None,
        );
    }

    /// Stores a `DRIVE_DISTANCE_COMPLETED` event and attributes it to the originating app.
    pub fn on_drive_completed(&mut self, seq: u16, completed: &DriveDistanceCompleted<'_>, now_ms: u64) {
        let origin = self.take_completion_origin(completed.command_seq, "drive_distance", now_ms);
        if matches!(self.claim, Some(DaemonClaim::Manual)) {
            self.claim = None;
        }
        self.push_event(
            seq,
            now_ms,
            EventName::DriveDistanceCompleted,
            EventValue::DriveDistanceCompleted {
                command_seq: None,
                status: completion_status(completed.status),
                target_cm: i32::from(completed.target_cm),
                actual_cm: completed.distance_cm,
                duration_ms: completed.duration_ms,
                detail: completed.detail.to_owned(),
            },
            origin,
        );
    }

    /// Stores a `ROTATE_COMPLETED` event and attributes it to the originating app.
    ///
    /// The wire's angles are clockwise; both the target and the actual-minus-target error are
    /// negated once here to become the App Boundary's counter-clockwise degrees.
    pub fn on_rotate_completed(&mut self, seq: u16, completed: &RotateCompleted<'_>, now_ms: u64) {
        let origin = self.take_completion_origin(completed.command_seq, "rotate_by", now_ms);
        if matches!(self.claim, Some(DaemonClaim::Manual)) {
            self.claim = None;
        }
        self.push_event(
            seq,
            now_ms,
            EventName::RotateCompleted,
            EventValue::RotateCompleted {
                command_seq: None,
                status: completion_status(completed.status),
                target_deg: -f64::from(completed.target_cdeg) / 100.0,
                error_deg: -f64::from(completed.error_cdeg) / 100.0,
                duration_ms: completed.duration_ms,
                detail: completed.detail.to_owned(),
            },
            origin,
        );
    }

    /// Stores a `CALIBRATION_LOADED` event with the wire's `(boot_id, seq)`.
    pub fn on_calibration_loaded(&mut self, seq: u16, loaded: &CalibrationLoaded, now_ms: u64) {
        self.push_event(
            seq,
            now_ms,
            EventName::CalibrationLoaded,
            EventValue::CalibrationLoaded {
                kind: calibration_kind(loaded.kind).to_owned(),
                status: calibration_load_status(loaded.status).to_owned(),
            },
            None,
        );
    }

    /// Stores the robot's own `EVENT_OVERFLOW` notice with the wire's `(boot_id, seq)`.
    ///
    /// This is distinct from the relay's synthetic overflow notice, which replays no wire event
    /// and therefore carries `null` in both sequence fields.
    pub fn on_robot_event_overflow(&mut self, seq: u16, overflow: &EventOverflow, now_ms: u64) {
        self.push_event(
            seq,
            now_ms,
            EventName::EventOverflow,
            EventValue::EventOverflow {
                dropped: u64::from(overflow.dropped),
            },
            None,
        );
    }

    /// Replaces the stored Catalogue and requests a fresh snapshot for every app.
    ///
    /// The snapshot is the truth: a catalogue retry does not touch the streams, the ring or any
    /// in-flight command.
    pub fn on_catalogue(&mut self, catalogue: Catalogue, _now_ms: u64) {
        self.catalogue = Some(catalogue);
        for state in self.apps.values_mut() {
            state.pending_snapshot = true;
        }
    }

    /// Records the Bridge's latest status and makes it available to every tab.
    ///
    /// The status is Bridge/network state, independent of the robot's Boot Id, so it survives a
    /// Session close and a Boot Id change; its `ts` exposes staleness. The caller only feeds it
    /// from within an open Session, so this method does not require [`Relay::is_connected`].
    pub const fn on_bridge_status(&mut self, status: &WireBridgeStatus, now_ms: u64) {
        self.bridge = Some(BridgeStatus {
            ts: now_ms,
            rssi_dbm: status.rssi_dbm,
            channel: status.channel,
            station: match status.station {
                WireBridgeStation::Down => BridgeStation::Down,
                WireBridgeStation::Connecting => BridgeStation::Connecting,
                WireBridgeStation::Associated => BridgeStation::Associated,
            },
            provisioned: status.flags.contains(WireBridgeFlags::PROVISIONED),
            setup_open: status.flags.contains(WireBridgeFlags::SETUP_OPEN),
            link_ready: status.flags.contains(WireBridgeFlags::LINK_READY),
            uart_dropped_bytes: status.uart_dropped_bytes,
            dma_stalls: status.dma_stalls,
            crc_failures: status.crc_failures,
            resyncs: status.resyncs,
            wdt_feeds: status.wdt_feeds,
            wdt_withheld: status.wdt_withheld,
        });
        self.bridge_seq = self.bridge_seq.wrapping_add(1);
    }

    /// Assembles a snapshot for `app` at any time, and marks it delivered.
    ///
    /// Returns `None` for an unknown connection. Calling this advances the app's cursors past the
    /// state the snapshot carries, so a following [`Relay::poll`] does not repeat it.
    pub fn snapshot(&mut self, app: AppId, now_ms: u64) -> Option<Snapshot> {
        let snapshot = self.build_snapshot(app, now_ms);
        let state = self.apps.get_mut(&app)?;
        state.pending_snapshot = false;
        state.event_cursor = self.events_pushed;
        for (index, slot) in self.slots.iter().enumerate() {
            if let Some(entry) = slot {
                state.stream_cursors[index] = state.stream_cursors[index].max(entry.seq);
            }
        }
        state.bridge_cursor = self.bridge_seq;
        Some(snapshot)
    }

    /// Drains one app connection's three lanes in ADR-0039's order.
    ///
    /// The lanes are a pending snapshot, the command results addressed to this connection, the
    /// latest-wins stream slots it has not seen, and the shared event ring from its own cursor.
    /// The Bridge status rides alongside the streams as its own latest-wins message.
    pub fn poll(&mut self, app: AppId, now_ms: u64) -> Vec<ServerMessage> {
        let mut messages = Vec::new();
        if !self.apps.contains_key(&app) {
            return messages;
        }
        let pending_connection = if self
            .apps
            .get(&app)
            .is_some_and(|state| state.connection_cursor < self.connection_seq)
        {
            self.connection.clone()
        } else {
            None
        };
        if let Some(connection) = pending_connection {
            messages.push(ServerMessage::Connection(connection));
            let sequence = self.connection_seq;
            if let Some(state) = self.apps.get_mut(&app) {
                state.connection_cursor = sequence;
            }
        }
        if self.apps.get(&app).is_some_and(|state| state.pending_snapshot)
            && let Some(snapshot) = self.snapshot(app, now_ms)
        {
            messages.push(ServerMessage::Snapshot(snapshot));
        }
        let pending_bridge = if self
            .apps
            .get(&app)
            .is_some_and(|state| state.bridge_cursor < self.bridge_seq)
        {
            self.bridge.clone()
        } else {
            None
        };
        let Some(state) = self.apps.get_mut(&app) else {
            return messages;
        };
        if let Some(bridge) = pending_bridge {
            messages.push(ServerMessage::Bridge(bridge));
            state.bridge_cursor = self.bridge_seq;
        }
        while let Some(result) = state.results.pop_front() {
            messages.push(ServerMessage::CommandResult(result));
        }
        for (index, slot) in self.slots.iter().enumerate() {
            if let Some(entry) = slot
                && entry.seq > state.stream_cursors[index]
            {
                state.stream_cursors[index] = entry.seq;
                messages.push(ServerMessage::Stream(entry.clone()));
            }
        }

        let oldest = self.events_pushed.saturating_sub(self.events.len() as u64);
        if state.event_cursor < oldest {
            let dropped = oldest - state.event_cursor;
            messages.push(ServerMessage::Event(EventEntry {
                boot_id: None,
                seq: None,
                ts: now_ms,
                name: EventName::EventOverflow,
                v: EventValue::EventOverflow { dropped },
            }));
            state.event_cursor = oldest;
        }
        let skip = match usize::try_from(state.event_cursor.saturating_sub(oldest)) {
            Ok(skip) => skip,
            Err(_) => self.events.len(),
        };
        for entry in self.events.iter().skip(skip) {
            messages.push(ServerMessage::Event(entry.for_app(app)));
        }
        state.event_cursor = self.events_pushed;
        messages
    }

    /// Accepts an app command, returning an immediate rejection only.
    ///
    /// A stale connection (no fresh heartbeat) or a closed Session is rejected `stale` and never
    /// queued. A malformed payload is rejected `bad_argument`; an unknown command name is
    /// `unknown_id`. An accepted command is queued and its result arrives through [`Relay::poll`].
    pub fn submit_command(&mut self, app: AppId, frame: CommandFrame, now_ms: u64) -> Option<CommandResult> {
        if !self.apps.contains_key(&app) {
            return None;
        }
        let CommandFrame {
            command_seq,
            command,
            args,
        } = frame;
        if !self.connected {
            return Some(rejection(
                command_seq,
                &command,
                Status::Stale,
                "Robot link lost",
                now_ms,
            ));
        }
        if self
            .apps
            .get(&app)
            .is_some_and(|state| now_ms.saturating_sub(state.last_heartbeat_ms) > APP_LIVENESS_MS)
        {
            return Some(rejection(
                command_seq,
                &command,
                Status::Stale,
                "App heartbeat is stale",
                now_ms,
            ));
        }
        let prepared = match prepare(&command, &args) {
            Ok(prepared) => prepared,
            Err(reason) => return Some(rejection(command_seq, &command, reason.status, &reason.detail, now_ms)),
        };
        if !self.make_room() {
            return Some(rejection(
                command_seq,
                &command,
                Status::Busy,
                "Too many commands in flight",
                now_ms,
            ));
        }

        let wire_seq = self.allocate_wire_seq();
        self.commands.insert(
            wire_seq,
            CommandRecord {
                app,
                app_seq: command_seq,
                name: command,
                active: true,
                created_ms: now_ms,
                expires_ms: None,
            },
        );
        let queued = QueuedCommand {
            wire_seq,
            app: Some(app),
            app_seq: command_seq,
            name: prepared.name,
            command: prepared.command,
            payload: prepared.payload,
            kind: prepared.kind,
            claim_action: prepared.claim_action,
        };
        if prepared.command == Command::StopMotion {
            self.queue.push_front(queued);
        } else {
            self.queue.push_back(queued);
        }
        None
    }

    /// Yields the next frame the link adapter should encode and send.
    ///
    /// At most one frame is produced per call: a due `RETRY` for the in-flight command, or the
    /// next queued command when nothing is in flight. `stop_motion` is queued at the front but
    /// never preempts the frame already on the wire (ADR-0039).
    pub fn poll_link(&mut self, now_ms: u64) -> Option<OutboundCommand> {
        if !self.connected {
            return None;
        }
        self.expire_in_flight(now_ms);
        if let Some(flight) = &mut self.in_flight {
            if flight.kind == CommandKind::Idempotent
                && now_ms.saturating_sub(flight.last_sent_ms) >= IDEMPOTENT_RETRY_MS
                && flight.retries < IDEMPOTENT_RETRIES
            {
                flight.retries += 1;
                flight.last_sent_ms = now_ms;
                return Some(OutboundCommand {
                    wire_seq: flight.wire_seq,
                    command: flight.command,
                    retry: true,
                    message: flight.payload,
                });
            }
            return None;
        }
        let queued = self.queue.pop_front()?;
        let outbound = OutboundCommand {
            wire_seq: queued.wire_seq,
            command: queued.command,
            retry: false,
            message: queued.payload,
        };
        self.in_flight = Some(InFlight {
            wire_seq: queued.wire_seq,
            app: queued.app,
            app_seq: queued.app_seq,
            name: queued.name,
            command: queued.command,
            payload: queued.payload,
            kind: queued.kind,
            claim_action: queued.claim_action,
            first_sent_ms: now_ms,
            last_sent_ms: now_ms,
            retries: 0,
        });
        Some(outbound)
    }

    /// Returns whether any command is queued or awaiting its acknowledgement.
    ///
    /// The link adapter uses this before sending a daemon-owned `GET_STATUS`/`GET_CATALOGUE`
    /// query, so the query's wire sequence cannot make a queued app command stale (ADR-0039).
    #[must_use]
    pub fn has_pending_commands(&self) -> bool {
        self.in_flight.is_some() || !self.queue.is_empty()
    }

    /// Allocates a wire sequence for a daemon-owned query frame.
    ///
    /// The daemon's own queries never enter the command map and never produce an app
    /// `command_result`, but they share the robot's single sequence space so the next app command
    /// is still `last + 1`. Call only while [`Relay::has_pending_commands`] is false.
    pub fn allocate_query_seq(&mut self) -> u16 {
        self.allocate_wire_seq()
    }

    /// Restarts the wire command sequence for a new robot Boot Id.
    ///
    /// The robot resets its own dedupe window on a boot, so its next accepted command is sequence
    /// `1`. The link adapter calls this when `HELLO_ACK` reports a different Boot Id, keeping the
    /// daemon's sequence and the robot's window in step (ADR-0039).
    pub const fn restart_wire_sequence(&mut self) {
        self.next_wire_seq = FIRST_WIRE_SEQ;
    }

    /// Completes the in-flight command with the robot's acknowledgement.
    ///
    /// An `Ok` ACK is `acked`; every other wire status is `rejected`, with `VERSION_MISMATCH`
    /// carried as `rejected` plus a detail because the app's status set has no separate value.
    pub fn on_command_ack(&mut self, wire_seq: u16, status: WireStatus, now_ms: u64) {
        let Some(flight) = self.in_flight.as_ref() else {
            return;
        };
        if flight.wire_seq != wire_seq {
            return;
        }
        let (app_status, detail) = ack_status(status);
        let outcome = if status == WireStatus::Ok {
            Outcome::Acked
        } else {
            Outcome::Rejected
        };
        self.finish_flight(outcome, app_status, detail.map(str::to_owned), now_ms);
    }

    /// Marks the app live; a heartbeat also resumes a frozen Session.
    pub fn heartbeat(&mut self, app: AppId, now_ms: u64) {
        let Some(state) = self.apps.get_mut(&app) else {
            return;
        };
        state.last_heartbeat_ms = now_ms;
        state.alive = true;
        self.app_live = true;
        self.last_heartbeat_ms = now_ms;
        if self.frozen {
            self.frozen = false;
            for state in self.apps.values_mut() {
                state.pending_snapshot = true;
            }
        }
    }

    /// Advances retries, timeouts, the app-frozen timer and completion eviction.
    ///
    /// The adapters call this at their timer cadence (roughly 100 ms to 1 s). Command timeouts
    /// pause while the robot link is down, so a same-Boot-Id reconnect resumes the ACK window
    /// instead of voiding the command.
    pub fn tick(&mut self, now_ms: u64) {
        let mut any_alive = false;
        for state in self.apps.values_mut() {
            state.alive = now_ms.saturating_sub(state.last_heartbeat_ms) <= APP_LIVENESS_MS;
            any_alive |= state.alive;
        }

        if any_alive {
            self.app_live = true;
            if self.frozen {
                self.frozen = false;
                for state in self.apps.values_mut() {
                    state.pending_snapshot = true;
                }
            }
        } else {
            self.app_live = false;
            if !self.frozen && now_ms.saturating_sub(self.last_heartbeat_ms) > APP_LIVENESS_MS {
                self.freeze();
            }
        }

        if self.connected {
            self.expire_in_flight(now_ms);
        }
        self.evict_completions(now_ms);
    }

    /// Returns whether the link adapter should send `PING` at 1 Hz.
    ///
    /// PING stops when no app is live, including after the app-frozen timer fires, and whenever
    /// the robot Session is down.
    #[must_use]
    pub const fn wants_ping(&self) -> bool {
        self.connected && self.app_live
    }

    /// Returns how many app connections were live at the last liveness update.
    ///
    /// A connection, a heartbeat and [`Relay::tick`] each refresh the per-app liveness flag.
    #[must_use]
    pub fn apps_live(&self) -> usize {
        self.apps.values().filter(|state| state.alive).count()
    }

    /// Stores one stream sample, assigning the daemon's monotonic sequence and receive time.
    fn publish(&mut self, name: StreamName, value: StreamValue, now_ms: u64) -> StreamEntry {
        let index = slot_of(name);
        self.stream_seqs[index] = self.stream_seqs[index].saturating_add(1);
        let entry = StreamEntry {
            name,
            seq: self.stream_seqs[index],
            ts: now_ms,
            v: value,
        };
        self.slots[index] = Some(entry.clone());
        if let StreamValue::Activity { id, stage, .. } = &entry.v {
            self.observe_activity(*id, *stage);
            if *stage != Stage::Running {
                self.last_terminal = Some(entry.clone());
            }
        }
        entry
    }

    /// Clears a daemon claim a newer Activity sample has superseded.
    const fn observe_activity(&mut self, id: u16, stage: Stage) {
        match stage {
            Stage::Running => match self.claim {
                Some(DaemonClaim::Manual) if id != MANUAL_DRIVE_ID => self.claim = None,
                Some(DaemonClaim::Procedure(procedure)) if id != procedure => self.claim = None,
                _ => {}
            },
            Stage::Complete | Stage::Failed => self.claim = None,
        }
    }

    /// Appends one wire event to the ring, keeping the wire's `(boot_id, seq)`.
    fn push_event(
        &mut self,
        seq: u16,
        now_ms: u64,
        name: EventName,
        value: EventValue,
        origin: Option<CompletionOrigin>,
    ) {
        if !self.connected {
            return;
        }
        let Some(boot_id) = self.boot_id else {
            return;
        };
        if self.events.len() == EVENT_RING_CAPACITY {
            self.events.pop_front();
        }
        self.events.push_back(RingEntry {
            event: EventEntry {
                boot_id: Some(boot_id),
                seq: Some(seq),
                ts: now_ms,
                name,
                v: value,
            },
            origin,
        });
        self.events_pushed = self.events_pushed.saturating_add(1);
    }

    /// Looks up and evicts the command a completion event belongs to.
    ///
    /// A completion that overtakes its ACK also settles the in-flight command as `acked`: the
    /// drive demonstrably ran, and a later ACK is ignored.
    fn take_completion_origin(&mut self, wire_seq: u16, expected: &str, now_ms: u64) -> Option<CompletionOrigin> {
        let origin = self
            .commands
            .get(&wire_seq)
            .filter(|record| record.name == expected)
            .map(|record| CompletionOrigin {
                app: record.app,
                command_seq: record.app_seq,
            });
        if self
            .in_flight
            .as_ref()
            .is_some_and(|flight| flight.wire_seq == wire_seq && flight.name == expected)
        {
            self.finish_flight(Outcome::Acked, Status::Ok, None, now_ms);
        }
        if origin.is_some() {
            self.commands.remove(&wire_seq);
        }
        origin
    }

    /// Ends the in-flight command, reporting to its app and settling its map record.
    fn finish_flight(&mut self, outcome: Outcome, status: Status, detail: Option<String>, now_ms: u64) {
        let Some(flight) = self.in_flight.take() else {
            return;
        };
        if outcome == Outcome::Acked {
            match flight.claim_action {
                Some(ClaimAction::Set(claim)) => self.claim = Some(claim),
                Some(ClaimAction::ClearProcedure(id)) => {
                    if self.claim == Some(DaemonClaim::Procedure(id)) {
                        self.claim = None;
                    }
                }
                Some(ClaimAction::ClearAll) => self.claim = None,
                None => {}
            }
        }
        if let Some(app) = flight.app
            && let Some(state) = self.apps.get_mut(&app)
        {
            state.results.push_back(CommandResult {
                command_seq: flight.app_seq,
                command: flight.name,
                outcome,
                status,
                detail,
                ts: now_ms,
            });
        }
        let keep = flight.kind == CommandKind::Volatile
            && matches!(outcome, Outcome::Acked | Outcome::Timeout)
            && flight.app.is_some();
        match self.commands.get_mut(&flight.wire_seq) {
            Some(record) if keep => {
                record.active = false;
                record.expires_ms = Some(now_ms.saturating_add(COMPLETION_RETENTION_MS));
            }
            _ => {
                self.commands.remove(&flight.wire_seq);
            }
        }
    }

    /// Expires the in-flight command when its ACK window has elapsed.
    fn expire_in_flight(&mut self, now_ms: u64) {
        let Some(flight) = self.in_flight.as_ref() else {
            return;
        };
        let expired = match flight.kind {
            CommandKind::Volatile => now_ms.saturating_sub(flight.first_sent_ms) >= VOLATILE_ACK_TIMEOUT_MS,
            CommandKind::Idempotent => {
                flight.retries >= IDEMPOTENT_RETRIES
                    && now_ms.saturating_sub(flight.last_sent_ms) >= IDEMPOTENT_RETRY_MS
            }
        };
        if expired {
            self.finish_flight(
                Outcome::Timeout,
                Status::Unknown,
                Some("No acknowledgement from the robot".to_owned()),
                now_ms,
            );
        }
    }

    /// Fires the app-frozen rule: a conditional stop, then no more PING.
    fn freeze(&mut self) {
        self.frozen = true;
        if self.daemon_started_claim() {
            let wire_seq = self.allocate_wire_seq();
            self.queue.push_front(QueuedCommand {
                wire_seq,
                app: None,
                app_seq: 0,
                name: "stop_motion".to_owned(),
                command: Command::StopMotion,
                payload: CommandMessage::StopMotion(StopMotion {
                    brake: StopMotion::BRAKE,
                }),
                kind: CommandKind::Idempotent,
                claim_action: Some(ClaimAction::ClearAll),
            });
        }
    }

    /// Returns whether the running Activity is a claim the daemon started.
    fn daemon_started_claim(&self) -> bool {
        let Some(entry) = self.slots[slot_of(StreamName::Activity)].as_ref() else {
            return false;
        };
        let StreamValue::Activity { id, stage, .. } = &entry.v else {
            return false;
        };
        if *stage != Stage::Running || *id == 0 {
            return false;
        }
        match self.claim {
            Some(DaemonClaim::Manual) => *id == MANUAL_DRIVE_ID,
            Some(DaemonClaim::Procedure(procedure)) => *id == procedure,
            None => false,
        }
    }

    /// Evicts completion records past their retention.
    fn evict_completions(&mut self, now_ms: u64) {
        let expired: Vec<u16> = self
            .commands
            .iter()
            .filter(|(_, record)| !record.active && record.expires_ms.is_some_and(|expiry| expiry <= now_ms))
            .map(|(wire_seq, _)| *wire_seq)
            .collect();
        for wire_seq in expired {
            self.commands.remove(&wire_seq);
        }
    }

    /// Ensures the command map has room, evicting an inactive completion record if needed.
    fn make_room(&mut self) -> bool {
        if self.commands.len() < COMMAND_MAP_CAPACITY {
            return true;
        }
        let victim = self
            .commands
            .iter()
            .filter(|(_, record)| !record.active)
            .min_by_key(|(_, record)| record.created_ms)
            .map(|(wire_seq, _)| *wire_seq);
        match victim {
            Some(wire_seq) => {
                self.commands.remove(&wire_seq);
                true
            }
            None => false,
        }
    }

    /// Allocates the next wire sequence that is not in the command map, skipping zero.
    fn allocate_wire_seq(&mut self) -> u16 {
        loop {
            let wire_seq = self.next_wire_seq;
            self.next_wire_seq = match wire_seq.wrapping_add(1) {
                0 => FIRST_WIRE_SEQ,
                next => next,
            };
            if !self.commands.contains_key(&wire_seq) {
                return wire_seq;
            }
        }
    }

    /// Clears the retained state after a Boot Id change and voids the outstanding commands.
    fn reset_for_boot_change(&mut self, now_ms: u64) {
        self.slots = std::array::from_fn(|_| None);
        self.last_terminal = None;
        self.events.clear();
        self.events_pushed = 0;
        self.catalogue = None;
        self.commands.clear();
        self.claim = None;
        for state in self.apps.values_mut() {
            state.event_cursor = 0;
            state.pending_snapshot = true;
        }
        if let Some(flight) = self.in_flight.take()
            && let Some(app) = flight.app
            && let Some(state) = self.apps.get_mut(&app)
        {
            state.results.push_back(CommandResult {
                command_seq: flight.app_seq,
                command: flight.name,
                outcome: Outcome::Indeterminate,
                status: Status::Unknown,
                detail: Some("Robot restarted".to_owned()),
                ts: now_ms,
            });
        }
        while let Some(queued) = self.queue.pop_front() {
            if let Some(app) = queued.app
                && let Some(state) = self.apps.get_mut(&app)
            {
                state.results.push_back(CommandResult {
                    command_seq: queued.app_seq,
                    command: queued.name,
                    outcome: Outcome::Rejected,
                    status: Status::Stale,
                    detail: Some("Robot restarted".to_owned()),
                    ts: now_ms,
                });
            }
        }
    }

    /// Builds a snapshot without touching any cursor.
    fn build_snapshot(&self, app: AppId, now_ms: u64) -> Snapshot {
        Snapshot {
            protocol: PROTOCOL,
            boot_id: self.boot_id,
            ts: now_ms,
            streams: self.slots.iter().flatten().cloned().collect(),
            last_terminal: self.last_terminal.clone(),
            events: self.events.iter().map(|entry| entry.for_app(app)).collect(),
            event_cursor: self.events.back().and_then(|entry| entry.event.seq),
            catalogue: self.catalogue.clone(),
            bridge: self.bridge.clone(),
        }
    }
}

impl Default for Relay {
    fn default() -> Self {
        Self::new()
    }
}

/// The frame a link adapter should encode and send (ADR-0039).
///
/// The adapter adds the class, the frame flags (including `RETRY` when [`Self::retry`] is set) and
/// the protocol major; the relay supplies the wire sequence, the command id and the typed payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OutboundCommand {
    /// The daemon-allocated wire command sequence.
    pub wire_seq: u16,
    /// The wire command id.
    pub command: Command,
    /// Whether this frame is a retransmission and must carry the `RETRY` flag.
    pub retry: bool,
    /// The typed payload the adapter encodes with `remote_protocol::encode_command`.
    pub message: CommandMessage<'static>,
}

/// Returns `Some(value)` unless it is the wire's unknown sentinel.
fn unknown_sentinel<T: Copy + PartialEq>(value: T, sentinel: T) -> Option<T> {
    if value == sentinel { None } else { Some(value) }
}

/// Converts a wire Activity stage into the app-facing stage.
const fn activity_stage(stage: ActivityStage) -> Stage {
    match stage {
        ActivityStage::Running => Stage::Running,
        ActivityStage::Complete => Stage::Complete,
        ActivityStage::Failed => Stage::Failed,
    }
}

/// Converts a wire calibration status into the app-facing status.
const fn wire_calibration_status(status: WireCalibrationStatus) -> CalibrationStatus {
    match status {
        WireCalibrationStatus::Unknown => CalibrationStatus::Unknown,
        WireCalibrationStatus::Loaded => CalibrationStatus::Loaded,
        WireCalibrationStatus::Missing => CalibrationStatus::Missing,
    }
}

/// Builds the app-facing Activity value.
fn activity_value(activity: &Activity<'_>) -> StreamValue {
    StreamValue::Activity {
        id: activity.procedure_id.get(),
        stage: activity_stage(activity.stage),
        percent: activity.percent,
        detail: activity.detail.to_owned(),
    }
}

/// Builds the app-facing Perception value from the wire flags.
const fn perception_value(perception: WirePerception) -> StreamValue {
    StreamValue::Perception {
        obstacle: perception.contains(WirePerception::OBSTACLE),
        floor_drop: perception.contains(WirePerception::FLOOR_DROP),
    }
}

/// Builds the app-facing Calibration value from the wire statuses.
#[allow(
    clippy::trivially_copy_pass_by_ref,
    reason = "it keeps the same by-reference call shape as the other stream converters"
)]
const fn calibration_value(calibration: &Calibration) -> StreamValue {
    StreamValue::Calibration {
        motor: wire_calibration_status(calibration.motor),
        imu: wire_calibration_status(calibration.imu),
        mag: wire_calibration_status(calibration.mag),
        distance: wire_calibration_status(calibration.distance),
    }
}

/// Converts a wire drive completion status into the app-facing status.
const fn completion_status(status: WireCompletionStatus) -> CompletionStatus {
    match status {
        WireCompletionStatus::Success => CompletionStatus::Ok,
        WireCompletionStatus::Cancelled => CompletionStatus::Cancelled,
        WireCompletionStatus::Failed => CompletionStatus::Failed,
    }
}

/// Names the calibration data an event reports.
const fn calibration_kind(kind: CalibrationKind) -> &'static str {
    match kind {
        CalibrationKind::Motor => "motor",
        CalibrationKind::ImuFlags => "imu",
        CalibrationKind::Distance => "distance",
    }
}

/// Names the calibration load outcome an event reports.
const fn calibration_load_status(status: CalibrationLoadStatus) -> &'static str {
    match status {
        CalibrationLoadStatus::Loaded => "loaded",
        CalibrationLoadStatus::Missing => "missing",
    }
}

/// Maps a wire ACK status onto the app's status vocabulary and an optional detail.
const fn ack_status(status: WireStatus) -> (Status, Option<&'static str>) {
    match status {
        WireStatus::Ok => (Status::Ok, None),
        WireStatus::Rejected => (Status::Rejected, None),
        WireStatus::UnknownId => (Status::UnknownId, None),
        WireStatus::BadArgument => (Status::BadArgument, None),
        WireStatus::Busy => (Status::Busy, None),
        WireStatus::VersionMismatch => (Status::Rejected, Some("Robot protocol version mismatch")),
        WireStatus::BadLength => (Status::BadLength, None),
        WireStatus::BadFlags => (Status::Rejected, Some("Robot rejected the frame flags")),
        WireStatus::Stale => (Status::Stale, None),
        WireStatus::Internal => (Status::Rejected, Some("Robot internal error")),
    }
}

/// Builds an immediate rejection result.
fn rejection(command_seq: u64, command: &str, status: Status, detail: &str, now_ms: u64) -> CommandResult {
    CommandResult {
        command_seq,
        command: command.to_owned(),
        outcome: Outcome::Rejected,
        status,
        detail: Some(detail.to_owned()),
        ts: now_ms,
    }
}

/// Parses and validates one app command frame.
fn prepare(command: &str, args: &serde_json::Value) -> Result<PreparedCommand, Rejection> {
    match command {
        "drive_distance" => prepare_drive(args),
        "rotate_by" => prepare_rotate(args),
        "stop_motion" => prepare_stop(args),
        "start_procedure" => prepare_start(args),
        "stop_procedure" => prepare_stop_procedure(args),
        other => Err(Rejection {
            status: Status::UnknownId,
            detail: format!("Unknown command: {other}"),
        }),
    }
}

/// Parses `drive_distance {distance_cm, speed}`.
fn prepare_drive(args: &serde_json::Value) -> Result<PreparedCommand, Rejection> {
    let object = args
        .as_object()
        .ok_or_else(|| bad_argument("drive_distance args must be an object"))?;
    let distance_cm = signed_field(object, "distance_cm")?;
    let distance_cm = i16::try_from(distance_cm)
        .map_err(|_| bad_argument("distance_cm must fit a signed 16-bit centimetre value"))?;
    let speed = speed_field(object)?;
    Ok(PreparedCommand {
        name: "drive_distance".to_owned(),
        command: Command::DriveDistance,
        payload: CommandMessage::DriveDistance(DriveDistance { distance_cm, speed }),
        kind: CommandKind::Volatile,
        claim_action: Some(ClaimAction::Set(DaemonClaim::Manual)),
    })
}

/// Parses `rotate_by {angle_deg, speed}`, validating the centidegree range before the wire.
fn prepare_rotate(args: &serde_json::Value) -> Result<PreparedCommand, Rejection> {
    let object = args
        .as_object()
        .ok_or_else(|| bad_argument("rotate_by args must be an object"))?;
    let requested_deg = object
        .get("angle_deg")
        .and_then(serde_json::Value::as_f64)
        .ok_or_else(|| bad_argument("angle_deg must be a number"))?;
    let scaled = -requested_deg * 100.0;
    if !scaled.is_finite() {
        return Err(bad_argument("angle_deg must be finite"));
    }
    let rounded = scaled.round();
    if !(f64::from(remote_protocol::payload::command::ANGLE_MIN_CDEG)
        ..=f64::from(remote_protocol::payload::command::ANGLE_MAX_CDEG))
        .contains(&rounded)
    {
        return Err(bad_argument("angle_deg must be within [-180, 180] degrees"));
    }
    // The range check above proves the cast fits `i16`; the allow records why it is not a bug.
    #[allow(clippy::cast_possible_truncation)]
    let wire_angle_cdeg = rounded as i16;
    let speed = speed_field(object)?;
    Ok(PreparedCommand {
        name: "rotate_by".to_owned(),
        command: Command::RotateBy,
        payload: CommandMessage::RotateBy(RotateBy {
            angle_cdeg: wire_angle_cdeg,
            speed,
        }),
        kind: CommandKind::Volatile,
        claim_action: Some(ClaimAction::Set(DaemonClaim::Manual)),
    })
}

/// Parses `stop_motion {brake}`.
fn prepare_stop(args: &serde_json::Value) -> Result<PreparedCommand, Rejection> {
    let object = args
        .as_object()
        .ok_or_else(|| bad_argument("stop_motion args must be an object"))?;
    let brake = object
        .get("brake")
        .and_then(serde_json::Value::as_bool)
        .ok_or_else(|| bad_argument("brake must be a boolean"))?;
    Ok(PreparedCommand {
        name: "stop_motion".to_owned(),
        command: Command::StopMotion,
        payload: CommandMessage::StopMotion(StopMotion {
            brake: if brake { StopMotion::BRAKE } else { StopMotion::COAST },
        }),
        kind: CommandKind::Idempotent,
        claim_action: Some(ClaimAction::ClearAll),
    })
}

/// Parses `start_procedure {id, params?}`; v1 rejects any `params` field.
fn prepare_start(args: &serde_json::Value) -> Result<PreparedCommand, Rejection> {
    let object = args
        .as_object()
        .ok_or_else(|| bad_argument("start_procedure args must be an object"))?;
    if object.contains_key("params") {
        return Err(bad_argument("start_procedure takes no params in v1"));
    }
    let id = procedure_field(object)?;
    Ok(PreparedCommand {
        name: "start_procedure".to_owned(),
        command: Command::StartProcedure,
        payload: CommandMessage::StartProcedure(StartProcedure {
            procedure_id: ProcedureId::new(id),
            params: &[],
        }),
        kind: CommandKind::Idempotent,
        claim_action: Some(ClaimAction::Set(DaemonClaim::Procedure(id))),
    })
}

/// Parses `stop_procedure {id}`.
fn prepare_stop_procedure(args: &serde_json::Value) -> Result<PreparedCommand, Rejection> {
    let object = args
        .as_object()
        .ok_or_else(|| bad_argument("stop_procedure args must be an object"))?;
    let id = procedure_field(object)?;
    Ok(PreparedCommand {
        name: "stop_procedure".to_owned(),
        command: Command::StopProcedure,
        payload: CommandMessage::StopProcedure(StopProcedure {
            procedure_id: ProcedureId::new(id),
        }),
        kind: CommandKind::Idempotent,
        claim_action: Some(ClaimAction::ClearProcedure(id)),
    })
}

/// Reads an integer argument.
fn signed_field(object: &serde_json::Map<String, serde_json::Value>, key: &str) -> Result<i64, Rejection> {
    object
        .get(key)
        .and_then(serde_json::Value::as_i64)
        .ok_or_else(|| Rejection {
            status: Status::BadArgument,
            detail: format!("{key} must be an integer"),
        })
}

/// Reads a speed argument in the firmware's documented `0..=100` percentage.
fn speed_field(object: &serde_json::Map<String, serde_json::Value>) -> Result<u8, Rejection> {
    let raw = signed_field(object, "speed")?;
    match u8::try_from(raw) {
        Ok(speed) if speed <= remote_protocol::payload::command::SPEED_MAX => Ok(speed),
        _ => Err(bad_argument("speed must be a percentage in [0, 100]")),
    }
}

/// Reads a Procedure id argument.
fn procedure_field(object: &serde_json::Map<String, serde_json::Value>) -> Result<u16, Rejection> {
    let raw = signed_field(object, "id")?;
    u16::try_from(raw).map_err(|_| bad_argument("id must fit an unsigned 16-bit Procedure id"))
}

/// Builds a `bad_argument` rejection.
fn bad_argument(detail: &str) -> Rejection {
    Rejection {
        status: Status::BadArgument,
        detail: detail.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    //! The relay's pure helpers, proven without an app or a link.

    use remote_protocol::Status as WireStatus;

    use super::{Relay, ack_status, unknown_sentinel};
    use crate::schema::{CommandFrame, Status};

    /// Extracts a value, failing the test rather than unwrapping (the workspace denies `unwrap`).
    fn must<T>(value: Option<T>, what: &str) -> T {
        value.unwrap_or_else(|| {
            assert_eq!(what.len(), usize::MAX, "missing {what}");
            std::process::abort()
        })
    }

    /// The unknown sentinel maps to `null` and every other value survives.
    #[test]
    fn a_sentinel_becomes_none() {
        assert_eq!(unknown_sentinel(0xFF, 0xFF), None);
        assert_eq!(unknown_sentinel(80, 0xFF), Some(80));
        assert_eq!(unknown_sentinel(0xFFFF_u16, 0xFFFF), None);
        assert_eq!(unknown_sentinel(7_400_u16, 0xFFFF), Some(7_400));
    }

    /// A version mismatch is `rejected` with a detail, not a status the app does not have.
    #[test]
    fn a_version_mismatch_maps_to_rejected_with_a_detail() {
        assert_eq!(
            ack_status(WireStatus::VersionMismatch),
            (Status::Rejected, Some("Robot protocol version mismatch"))
        );
    }

    /// A daemon-owned query shares the app command's wire sequence space but not its map.
    #[test]
    fn a_query_sequence_shares_the_app_command_space() {
        let mut relay = Relay::new();
        relay.session_up(7, 0);
        let app = relay.connect_app(0);
        assert!(!relay.has_pending_commands(), "a fresh Session has nothing pending");

        assert_eq!(relay.allocate_query_seq(), 1, "the first wire sequence is one");
        assert!(!relay.has_pending_commands(), "a query is not an app command");

        let frame = CommandFrame {
            command_seq: 1,
            command: "stop_motion".to_owned(),
            args: serde_json::json!({"brake": true}),
        };
        assert!(relay.submit_command(app, frame, 0).is_none());
        assert!(relay.has_pending_commands(), "a queued command counts as pending");
        let outbound = must(relay.poll_link(0), "app command");
        assert_eq!(outbound.wire_seq, 2, "the app command follows the query sequence");
        assert!(relay.has_pending_commands(), "the command is in flight");
        relay.on_command_ack(outbound.wire_seq, WireStatus::Ok, 1);
        assert!(!relay.has_pending_commands(), "the ACK clears the in-flight slot");
    }

    /// A new robot Boot Id restarts the wire sequence at the robot's expected `1`.
    #[test]
    fn a_boot_restart_restarts_the_wire_sequence() {
        let mut relay = Relay::new();
        relay.session_up(7, 0);
        assert_eq!(relay.allocate_query_seq(), 1);
        assert_eq!(relay.allocate_query_seq(), 2);
        relay.restart_wire_sequence();
        assert_eq!(relay.allocate_query_seq(), 1, "a new boot expects sequence one");
    }
}
