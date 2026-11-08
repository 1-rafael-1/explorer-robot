//! The App Boundary's JSON schema (ADR-0038, ADR-0039).
//!
//! One WebSocket, text frames, one JSON object per frame. The schema is additive and versioned:
//! unknown message types, stream names, event names and extra fields are ignored, no field is
//! removed or repurposed, and the version lives in `hello.protocol`, never in the path. Distances
//! are integer centimetres, angles are degrees counter-clockwise positive, an unknown value is
//! `null` never a sentinel, and `ts` is the daemon's receive time in epoch milliseconds.
//!
//! The relay and the app adapter speak these types; the app itself never sees the binary wire.

use serde::{Deserialize, Serialize};

/// The App Boundary protocol version (ADR-0038).
pub const PROTOCOL: u8 = 1;

/// The stable service name in the server's `hello`; it names no process (ADR-0039).
pub const SERVICE_NAME: &str = "explorer-remote";

/// The Activity-only Manual Drive id (ADR-0031); the app owns its label.
pub const MANUAL_DRIVE_ID: u16 = 0x000D;

/// A daemon-to-app message.
///
/// The `type` field is the envelope tag; the variants are the named messages of ADR-0038's table.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerMessage {
    /// The protocol version and names; the server adds `auth_required`.
    Hello(Hello),
    /// Reachability and failure state.
    Connection(Connection),
    /// Reload-safe state after connect, and whenever the robot Session is re-established.
    Snapshot(Snapshot),
    /// One latest-wins sample.
    Stream(StreamEntry),
    /// One reliable, replayable edge.
    Event(EventEntry),
    /// The ACK or rejection of an app command, never the drive outcome.
    CommandResult(CommandResult),
    /// The Bridge's latest link and resilience status; latest-wins and independent of the robot.
    Bridge(BridgeStatus),
}

/// An app-to-daemon message.
///
/// `auth`, `pair` and `connect` are consumed by the app adapter (the auth gate and the manual
/// endpoint); the relay itself only consumes `heartbeat`, `command` and `disconnect`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ClientMessage {
    /// The app's protocol version.
    Hello(ClientHello),
    /// The pairing token, sent first when `hello.auth_required`.
    Auth(Auth),
    /// A new pairing code, which also stores it.
    Pair(Pair),
    /// A manual `host:port` endpoint.
    Connect(Connect),
    /// Releases the shared robot Session for all tabs; the stored token is kept.
    Disconnect,
    /// App liveness; the app sends it at 1 Hz.
    Heartbeat,
    /// A robot command.
    Command(CommandFrame),
}

/// The server's `hello`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Hello {
    /// The App Boundary protocol version.
    pub protocol: u8,
    /// The stable service identity.
    pub server: ServerName,
    /// The robot's advertised model, `null` unless the Bridge says otherwise.
    pub robot: RobotModel,
    /// Whether this peer must authenticate before anything else.
    pub auth_required: bool,
}

/// The stable service name in `hello.server`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ServerName {
    /// The service name, never a process name.
    pub name: String,
}

/// The robot's model in `hello.robot`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RobotModel {
    /// The advertised model, `null` when unknown.
    pub model: Option<String>,
}

/// The app's `hello`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClientHello {
    /// The app's protocol version.
    pub protocol: u8,
}

/// The `auth` message: a stored pairing token.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Auth {
    /// The pairing token.
    pub code: String,
}

/// The `pair` message: a new pairing code, which also stores it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Pair {
    /// The pairing code as twelve base32 characters.
    pub code: String,
}

/// The `connect` message: a manual endpoint for networks that block broadcast.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Connect {
    /// The Bridge's host.
    pub host: String,
    /// The Link Protocol port.
    pub port: u16,
}

/// A `connection` message's state (ADR-0038).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionState {
    /// No beacon yet.
    Searching,
    /// A Session bring-up is in progress.
    Connecting,
    /// The robot Session is live.
    Connected,
    /// The robot link stalled; retrying with backoff.
    Reconnecting,
    /// No Beacon was heard.
    NoRobots,
    /// A robot was found but no token is stored.
    PairingRequired,
    /// The pairing token was refused; the operator re-pairs.
    Unauthorized,
    /// Another client holds the Bridge's single slot.
    Busy,
    /// The Bridge is provisioning.
    SetupMode,
}

/// A `connection` message.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Connection {
    /// The reachability state.
    pub state: ConnectionState,
    /// The robot's identity, `null` until the daemon knows one.
    pub robot: Option<String>,
    /// A human-facing detail, `null` when the state says it all.
    pub detail: Option<String>,
    /// The backoff hint while reconnecting, `null` otherwise.
    pub retry_in_ms: Option<u64>,
    /// The current Boot Id, `null` before the first Session.
    pub boot_id: Option<u32>,
    /// The daemon's receive time.
    pub ts: u64,
}

/// A stream's name (ADR-0038's stream table).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StreamName {
    /// `power`: battery level and pack voltage.
    Power,
    /// `motion`: left and right motor output.
    Motion,
    /// `activity`: the current Procedure and its progress.
    Activity,
    /// `perception`: obstacle and floor-drop flags.
    Perception,
    /// `lidar`: the 360-slot range grid.
    Lidar,
    /// `calibration`: the four calibration statuses.
    Calibration,
}

impl StreamName {
    /// Returns the name of the wire stream it mirrors.
    ///
    /// The wire's `LIDAR_CLOUD` is the app's `lidar`; every other name matches its stream.
    #[must_use]
    pub const fn from_wire(stream: remote_protocol::Stream) -> Self {
        match stream {
            remote_protocol::Stream::Power => Self::Power,
            remote_protocol::Stream::Motion => Self::Motion,
            remote_protocol::Stream::Activity => Self::Activity,
            remote_protocol::Stream::Perception => Self::Perception,
            remote_protocol::Stream::LidarCloud => Self::Lidar,
            remote_protocol::Stream::Calibration => Self::Calibration,
        }
    }
}

/// An Activity stage (ADR-0038).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Stage {
    /// The Procedure is running.
    Running,
    /// The Procedure completed successfully.
    Complete,
    /// The Procedure failed.
    Failed,
}

/// A calibration status (ADR-0038).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationStatus {
    /// Not known yet.
    Unknown,
    /// The calibration data loaded.
    Loaded,
    /// The calibration data is missing.
    Missing,
}

/// A drive completion status (ADR-0038): it mirrors the drive's `CompletionStatus`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CompletionStatus {
    /// The move finished as requested.
    Ok,
    /// The move was cancelled before it finished.
    Cancelled,
    /// The move failed.
    Failed,
}

/// One stream sample: the body of a `stream` message and one entry of a snapshot's `streams`.
///
/// `seq` is the daemon's monotonic per-stream sequence, never the wire's (ADR-0039); the app
/// deduplicates by `(name, seq)`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct StreamEntry {
    /// The stream's name.
    pub name: StreamName,
    /// The daemon's monotonic per-stream sequence.
    pub seq: u64,
    /// The daemon's receive time.
    pub ts: u64,
    /// The stream's value.
    pub v: StreamValue,
}

/// A stream's value (ADR-0038's stream table).
///
/// The variants are untagged because the `name` sibling field discriminates them. Every variant
/// except `Power` has a required field, so the all-optional `Power` is matched last and can never
/// swallow another variant's shape.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum StreamValue {
    /// `motion`: the left and right motor output, `-100..100`.
    Motion {
        /// The left motor output.
        left: i8,
        /// The right motor output.
        right: i8,
    },
    /// `activity`: the current Procedure and its progress.
    Activity {
        /// The Procedure id.
        id: u16,
        /// The Procedure's stage.
        stage: Stage,
        /// The completion percentage, `null` when not reported.
        percent: Option<u8>,
        /// The operator-facing detail.
        detail: String,
    },
    /// `perception`: the obstacle and floor-drop flags.
    Perception {
        /// Whether an obstacle is present.
        obstacle: bool,
        /// Whether a floor drop is present.
        floor_drop: bool,
    },
    /// `lidar`: the 360-slot range grid, slot 0 ahead and clockwise.
    Lidar {
        /// The ranges in centimetres; `null` is no return.
        ranges_cm: Vec<Option<u16>>,
        /// The first slot's angle; v1 is always `0`.
        first_deg: u8,
        /// The angular step between slots; v1 is always `1`.
        step_deg: u8,
        /// Whether the slots run clockwise; v1 is always `true`.
        clockwise: bool,
    },
    /// `calibration`: the four calibration statuses.
    Calibration {
        /// Motor calibration status.
        motor: CalibrationStatus,
        /// IMU calibration status.
        imu: CalibrationStatus,
        /// Magnetometer calibration status.
        mag: CalibrationStatus,
        /// Distance calibration status.
        distance: CalibrationStatus,
    },
    /// `power`: the sentinels become `null`.
    Power {
        /// The battery level percentage, `null` when unknown.
        level: Option<u8>,
        /// The pack voltage in millivolts, `null` when unknown.
        voltage_mv: Option<u32>,
    },
}

/// An event's name (ADR-0038's event table).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventName {
    /// The obstacle flag changed.
    Obstacle,
    /// The floor-drop flag changed.
    FloorDrop,
    /// A Procedure reached a terminal outcome.
    ProcedureCompleted,
    /// A `drive_distance` command finished.
    DriveDistanceCompleted,
    /// A `rotate_by` command finished.
    RotateCompleted,
    /// A boot-time calibration load finished.
    CalibrationLoaded,
    /// Events were dropped before they could be delivered.
    EventOverflow,
}

/// One event: the body of an `event` message and one entry of a snapshot's `events`.
///
/// The wire's `(boot_id, seq)` is kept because the app needs the Boot Id to decide "outcome
/// unknown". Both are `null` only on the daemon's own overflow notice, which replays no wire event.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EventEntry {
    /// The wire Boot Id the event was sequenced under, `null` on a daemon notice.
    pub boot_id: Option<u32>,
    /// The wire's event sequence, `null` on a daemon notice.
    pub seq: Option<u16>,
    /// The daemon's receive time.
    pub ts: u64,
    /// The event's name.
    pub name: EventName,
    /// The event's value.
    pub v: EventValue,
}

/// An event's value (ADR-0038's event table).
///
/// The variants are untagged because the `name` sibling field discriminates them. Every variant
/// has a required field, and `obstacle` is ordered before `floor_drop` because its field set
/// contains `floor_drop`'s; the order is the untagged match order and must not be reshuffled.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum EventValue {
    /// The obstacle flag changed.
    Obstacle {
        /// Whether an obstacle is present.
        detected: bool,
        /// The detector: `"lidar"` or `"unknown"`.
        source: String,
    },
    /// The floor-drop flag changed.
    FloorDrop {
        /// Whether a floor drop is present.
        detected: bool,
    },
    /// A Procedure reached a terminal outcome.
    ProcedureCompleted {
        /// The Procedure that finished.
        id: u16,
        /// The terminal stage.
        stage: Stage,
        /// The operator-facing reason.
        detail: String,
    },
    /// A `drive_distance` command finished; the completion event, not the ACK, carries the outcome.
    DriveDistanceCompleted {
        /// The originating app's `command_seq`, `null` for every other tab.
        command_seq: Option<u64>,
        /// The drive outcome.
        status: CompletionStatus,
        /// The requested distance in centimetres, forward-positive.
        target_cm: i32,
        /// The distance actually travelled in centimetres.
        actual_cm: i32,
        /// The move's duration in milliseconds.
        duration_ms: u32,
        /// The operator-facing reason.
        detail: String,
    },
    /// A `rotate_by` command finished; angles are counter-clockwise degrees.
    RotateCompleted {
        /// The originating app's `command_seq`, `null` for every other tab.
        command_seq: Option<u64>,
        /// The rotation outcome.
        status: CompletionStatus,
        /// The requested angle in counter-clockwise degrees.
        target_deg: f64,
        /// The achieved minus requested angle in counter-clockwise degrees.
        error_deg: f64,
        /// The rotation's duration in milliseconds.
        duration_ms: u32,
        /// The operator-facing reason.
        detail: String,
    },
    /// A boot-time calibration load finished.
    CalibrationLoaded {
        /// Which calibration data: `"motor"`, `"imu"` or `"distance"`.
        kind: String,
        /// Whether it loaded or was missing.
        status: String,
    },
    /// Events were dropped; `dropped` says how many.
    EventOverflow {
        /// How many events were dropped.
        dropped: u64,
    },
}

/// The Bridge's Wi-Fi association state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BridgeStation {
    /// The station interface is down.
    Down,
    /// The station is connecting.
    Connecting,
    /// The station is associated.
    Associated,
}

/// A `bridge` message: the Bridge's link and resilience snapshot (ADR-0031).
///
/// The status is Bridge/network state, independent of the robot's Boot Id; its `ts` exposes
/// staleness. Unknown RSSI and channel values are `null`, never a sentinel.
#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BridgeStatus {
    /// The daemon's receive time.
    pub ts: u64,
    /// The station RSSI in dBm, `null` when unknown.
    pub rssi_dbm: Option<i8>,
    /// The station channel, `null` when unknown.
    pub channel: Option<u8>,
    /// The station association state.
    pub station: BridgeStation,
    /// Whether the Bridge is provisioned.
    pub provisioned: bool,
    /// Whether the Bridge's setup mode is open.
    pub setup_open: bool,
    /// Whether the Bridge's link is ready.
    pub link_ready: bool,
    /// Bytes dropped by the Bridge's UART receive path.
    pub uart_dropped_bytes: u32,
    /// DMA stalls observed by the Bridge.
    pub dma_stalls: u32,
    /// Frame CRC failures observed by the Bridge.
    pub crc_failures: u32,
    /// Resynchronisations after a framing error.
    pub resyncs: u32,
    /// Watchdog feeds.
    pub wdt_feeds: u32,
    /// Watchdog feeds withheld because a task was late.
    pub wdt_withheld: u32,
}

/// A `snapshot` message (ADR-0038).
///
/// A snapshot may arrive at any time: on connect, on a robot reconnect, on a Boot Id change and
/// after a catalogue retry. It replaces the streams, Boot Id, `last_terminal` and catalogue; its
/// events are merged by `(boot_id, seq)`. Only a Boot Id change voids an in-flight command.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Snapshot {
    /// The App Boundary protocol version.
    pub protocol: u8,
    /// The current Boot Id, `null` before the first Session.
    pub boot_id: Option<u32>,
    /// The daemon's assembly time.
    pub ts: u64,
    /// The latest sample of every stream; an unseen stream is omitted.
    pub streams: Vec<StreamEntry>,
    /// The last terminal Activity sample, retained until the next one or a Boot Id change.
    pub last_terminal: Option<StreamEntry>,
    /// A bounded replay of recent events.
    pub events: Vec<EventEntry>,
    /// The wire sequence of the last replayed event, `null` when the replay is empty.
    pub event_cursor: Option<u16>,
    /// The robot's `GET_CATALOGUE` response, `null` while unknown.
    pub catalogue: Option<Catalogue>,
    /// The Bridge's latest status, `null` before the first one; it survives a robot Boot Id change.
    pub bridge: Option<BridgeStatus>,
}

/// The robot's startable Procedures (ADR-0031's `GET_CATALOGUE` response).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Catalogue {
    /// The startable Procedures, in catalogue order across every page.
    pub procedures: Vec<CatalogueProcedure>,
}

/// One startable Procedure.
#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CatalogueProcedure {
    /// The Procedure id.
    pub id: u16,
    /// The operator-facing label.
    pub label: String,
    /// Whether the Procedure is startable from the Remote now.
    pub available: bool,
    /// Whether the Procedure is interactive.
    pub interactive: bool,
    /// Whether the Procedure is a calibration.
    pub calibration: bool,
    /// Whether the Procedure takes parameters.
    pub takes_params: bool,
}

/// The `command_result` outcome (ADR-0038).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Outcome {
    /// The robot acknowledged the command.
    Acked,
    /// The robot or the daemon refused it.
    Rejected,
    /// The outcome is unknown; a Boot Id change voided it.
    Indeterminate,
    /// No acknowledgement arrived in time.
    Timeout,
}

/// The `command_result` status (ADR-0038).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Status {
    /// The command succeeded.
    Ok,
    /// The robot is busy with another activity.
    Busy,
    /// The command was understood but refused.
    Rejected,
    /// The id is not known.
    UnknownId,
    /// The app's heartbeat is stale or the robot link is lost.
    Stale,
    /// A payload value is malformed or out of range.
    BadArgument,
    /// A payload has the wrong structural length.
    BadLength,
    /// No status is known.
    Unknown,
}

/// A `command_result` message: the ACK or rejection, never the drive outcome (ADR-0038).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CommandResult {
    /// The originating app's `command_seq`.
    pub command_seq: u64,
    /// The command's name, as the app sent it.
    pub command: String,
    /// Whether the command was acked, rejected, indeterminate or timed out.
    pub outcome: Outcome,
    /// The wire status, mapped onto the app's vocabulary.
    pub status: Status,
    /// A human-facing detail, `null` when the status says it all.
    pub detail: Option<String>,
    /// The daemon's receive time.
    pub ts: u64,
}

/// An app `command` frame: `{command_seq, command, args}`.
///
/// `args` is kept raw: the relay validates it per command, so an additive extra field is ignored
/// and a missing or malformed one is `bad_argument`, and `start_procedure`'s `params` stays
/// rejectable until the catalogue carries a parameter schema (ADR-0039).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CommandFrame {
    /// The app-local command sequence, echoed by the `command_result`.
    pub command_seq: u64,
    /// The command's name.
    pub command: String,
    /// The command's arguments.
    pub args: serde_json::Value,
}
