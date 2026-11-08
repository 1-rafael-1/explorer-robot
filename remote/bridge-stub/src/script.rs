//! The stub's scenario: everything a test or a bench run can script.
//!
//! A [`Scenario`] is plain data with the ADR's defaults: the token the stub verifies, the identity
//! served in the challenge and the Beacon, the preamble's verdict knobs and the robot traffic
//! served behind `ACCEPT`. The daemon's tests build one per case; the binary builds one from its
//! command line.

use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4};

use bridge_net::{beacon, ports, preamble as net_preamble};
use remote_protocol::{
    Command, EncodeError, Event, EventMessage, ProcedureId, RetryPolicy, Status, Stream, encode_event,
    payload::stream::ActivityStage,
};

use crate::{
    identity::{DeviceId, Flags, Identity, Name},
    preamble::DenyReason,
};

/// The whole stub: identity, token, preamble behaviour and scripted traffic.
#[derive(Clone, Debug)]
pub struct Scenario {
    /// The pairing token's bytes; a real token is 60 bits, the stub scripts what it is given.
    pub token: Vec<u8>,
    /// The identity served in the challenge and the Beacon.
    pub identity: Identity,
    /// The address the Link listener binds; loopback by default.
    pub bind: IpAddr,
    /// The TCP port to bind; [`None`] picks an ephemeral port.
    pub link_port: Option<u16>,
    /// The Beacon's script.
    pub beacon: BeaconScript,
    /// When set, every connection is denied with this reason, proofs aside.
    pub deny: Option<Deny>,
    /// How long a connection has to complete the preamble; ADR-0037's 5 s.
    pub auth_deadline_ms: u64,
    /// The robot traffic served behind `ACCEPT`.
    pub traffic: Traffic,
}

impl Scenario {
    /// A scenario with `token` and the documented defaults: a bench identity, the 3 s Beacon, the
    /// 5 s auth deadline and a 1 Hz motion and activity baseline.
    #[must_use]
    pub fn new(token: &[u8]) -> Self {
        Self {
            token: token.to_vec(),
            identity: Identity {
                device_id: DeviceId::new(*b"stub01"),
                // The literal is thirteen bytes; the fallback exists because `new` is fallible.
                name: Name::new("Explorer-STUB").unwrap_or_else(|| Name::from_padded([0u8; Name::ENCODED_LEN])),
                flags: Flags::PROVISIONED | Flags::LINK_READY,
            },
            bind: IpAddr::V4(Ipv4Addr::LOCALHOST),
            link_port: None,
            beacon: BeaconScript {
                tcp_port: None,
                interval_ms: beacon::INTERVAL_MS,
                target: SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::BROADCAST, ports::BEACON_UDP)),
            },
            deny: None,
            auth_deadline_ms: net_preamble::AUTH_DEADLINE_MS,
            traffic: Traffic::baseline(),
        }
    }
}

/// The Beacon's script.
#[derive(Clone, Copy, Debug)]
pub struct BeaconScript {
    /// The advertised Link port; [`None`] mirrors the port the stub actually bound, [`Some`] pins
    /// one — including [`beacon::PORTAL_ONLY`] to advertise a portal-only Bridge.
    pub tcp_port: Option<u16>,
    /// The broadcast interval in milliseconds; ADR-0037's 3 s.
    pub interval_ms: u64,
    /// The datagram's destination; the LAN broadcast address by default.
    pub target: SocketAddr,
}

/// A scripted denial, served to every connection regardless of its proof.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Deny {
    /// The deny reason.
    pub reason: DenyReason,
    /// The retry hint, when the reason carries one.
    pub retry_ms: Option<u32>,
}

/// The scripted robot traffic behind `ACCEPT`.
#[derive(Clone, Debug)]
pub struct Traffic {
    /// The Boot Id served in `HELLO_ACK` and bound into the event sequence.
    pub boot_id: u32,
    /// The streams and their rates, in centihertz.
    pub streams: Vec<StreamRate>,
    /// The events, sent in order, each retried until acknowledged.
    pub events: Vec<ScriptEvent>,
    /// The event retransmission schedule; the protocol's event default.
    pub event_retry: RetryPolicy,
    /// Per-command ACK statuses; unlisted commands are acknowledged `OK`.
    pub command_acks: Vec<(Command, Status)>,
}

impl Traffic {
    /// The 1 Hz motion and activity baseline an idle robot streams (ADR-0039).
    #[must_use]
    pub fn baseline() -> Self {
        Self {
            boot_id: 1,
            streams: vec![
                StreamRate {
                    stream: Stream::Motion,
                    rate_chz: 100,
                },
                StreamRate {
                    stream: Stream::Activity,
                    rate_chz: 100,
                },
            ],
            events: Vec::new(),
            event_retry: RetryPolicy::EVENT,
            command_acks: Vec::new(),
        }
    }

    /// Returns the configured ACK status for `command`, defaulting to `OK`.
    #[must_use]
    pub fn ack_status(&self, command: Command) -> Status {
        self.command_acks
            .iter()
            .find(|(listed, _)| *listed == command)
            .map_or(Status::Ok, |(_, status)| *status)
    }
}

/// One scripted stream and its rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StreamRate {
    /// The stream to sample.
    pub stream: Stream,
    /// The rate in centihertz, the Requested Rate vocabulary; zero disables the stream.
    pub rate_chz: u16,
}

/// One scripted event, with owned detail fields.
///
/// The variants mirror the protocol's event payloads; the stub encodes them with the shared
/// codecs, so it never invents event wire semantics.
#[derive(Clone, Debug)]
pub enum ScriptEvent {
    /// An `OBSTACLE` event.
    Obstacle {
        /// The obstacle flag, as the firmware reported it.
        detected: u8,
        /// The detector's wire id.
        source: u8,
    },
    /// A `FLOOR_DROP` event.
    FloorDrop {
        /// The floor-drop flag, as the firmware reported it.
        detected: u8,
    },
    /// A `PROCEDURE_COMPLETED` event.
    ProcedureCompleted {
        /// The Procedure that finished.
        procedure_id: ProcedureId,
        /// The terminal stage.
        stage: ActivityStage,
        /// The operator-facing reason.
        detail: String,
    },
    /// A `DRIVE_DISTANCE_COMPLETED` event.
    DriveDistanceCompleted {
        /// The originating command's sequence number.
        command_seq: u16,
        /// The drive outcome.
        status: remote_protocol::CompletionStatus,
        /// The requested distance in centimetres.
        target_cm: i16,
        /// The distance actually travelled in centimetres.
        distance_cm: i32,
        /// The move's duration in milliseconds.
        duration_ms: u32,
        /// The operator-facing reason.
        detail: String,
    },
    /// A `ROTATE_COMPLETED` event.
    RotateCompleted {
        /// The originating command's sequence number.
        command_seq: u16,
        /// The rotation outcome.
        status: remote_protocol::CompletionStatus,
        /// The requested angle in centidegrees.
        target_cdeg: i16,
        /// The signed angle actually achieved, in centidegrees.
        error_cdeg: i32,
        /// The rotation's duration in milliseconds.
        duration_ms: u32,
        /// The operator-facing reason.
        detail: String,
    },
    /// A `CALIBRATION_LOADED` event.
    CalibrationLoaded {
        /// Which calibration data the event concerns.
        kind: remote_protocol::CalibrationKind,
        /// Whether it loaded or was missing.
        status: remote_protocol::CalibrationLoadStatus,
    },
    /// An `EVENT_OVERFLOW` event.
    EventOverflow {
        /// How many events were dropped.
        dropped: u16,
    },
}

impl ScriptEvent {
    /// Returns the event id.
    #[must_use]
    pub const fn event(&self) -> Event {
        match self {
            Self::Obstacle { .. } => Event::Obstacle,
            Self::FloorDrop { .. } => Event::FloorDrop,
            Self::ProcedureCompleted { .. } => Event::ProcedureCompleted,
            Self::DriveDistanceCompleted { .. } => Event::DriveDistanceCompleted,
            Self::RotateCompleted { .. } => Event::RotateCompleted,
            Self::CalibrationLoaded { .. } => Event::CalibrationLoaded,
            Self::EventOverflow { .. } => Event::EventOverflow,
        }
    }

    /// Encodes the event's payload into `out` with the shared codec.
    ///
    /// # Errors
    ///
    /// Returns [`EncodeError`] when `out` cannot hold the payload or a field cannot be
    /// represented.
    pub fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let message = match self {
            Self::Obstacle { detected, source } => EventMessage::Obstacle(remote_protocol::Obstacle {
                detected: *detected,
                source: *source,
            }),
            Self::FloorDrop { detected } => EventMessage::FloorDrop(remote_protocol::FloorDrop { detected: *detected }),
            Self::ProcedureCompleted {
                procedure_id,
                stage,
                detail,
            } => EventMessage::ProcedureCompleted(remote_protocol::ProcedureCompleted {
                procedure_id: *procedure_id,
                stage: *stage,
                detail,
            }),
            Self::DriveDistanceCompleted {
                command_seq,
                status,
                target_cm,
                distance_cm,
                duration_ms,
                detail,
            } => EventMessage::DriveDistanceCompleted(remote_protocol::DriveDistanceCompleted {
                command_seq: *command_seq,
                status: *status,
                target_cm: *target_cm,
                distance_cm: *distance_cm,
                duration_ms: *duration_ms,
                detail,
            }),
            Self::RotateCompleted {
                command_seq,
                status,
                target_cdeg,
                error_cdeg,
                duration_ms,
                detail,
            } => EventMessage::RotateCompleted(remote_protocol::RotateCompleted {
                command_seq: *command_seq,
                status: *status,
                target_cdeg: *target_cdeg,
                error_cdeg: *error_cdeg,
                duration_ms: *duration_ms,
                detail,
            }),
            Self::CalibrationLoaded { kind, status } => {
                EventMessage::CalibrationLoaded(remote_protocol::CalibrationLoaded {
                    kind: *kind,
                    status: *status,
                })
            }
            Self::EventOverflow { dropped } => {
                EventMessage::EventOverflow(remote_protocol::EventOverflow { dropped: *dropped })
            }
        };
        encode_event(&message, out)
    }
}
