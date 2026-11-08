//! The shared Link Protocol contract (ADR-0030, ADR-0031, ADR-0040).
//!
//! This crate is `no_std` and transport agnostic: it carries the frame envelope, the shared
//! identifiers and the payload codecs that the robot, the Bridge and the daemon all speak. It has
//! no UART, clock, allocator or logging. Enable the `std` feature only for host tests and the
//! simulated lossy link; the feature is a no-op on `no_std` targets so `--all-features` stays
//! valid for the firmware.
//!
//! The crate version mirrors the wire version: changing [`PROTOCOL_MAJOR`] or [`PROTOCOL_MINOR`]
//! changes the crate's major or minor version, and every consumer states a `version = "1"`
//! requirement so a major bump cannot pass unnoticed (ADR-0040).

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

/// The `std` feature is host-only: on a `no_std` target it compiles to nothing, so the workspace's
/// `--all-features` clippy pass stays valid for the firmware.
#[cfg(all(feature = "std", not(target_os = "none")))]
extern crate std;

pub mod codec;
pub mod crc;
pub mod error;
pub mod frame;
pub mod ids;
pub mod parser;
pub mod payload;
pub mod reliability;
#[cfg(all(feature = "std", not(target_os = "none")))]
pub mod sim;

pub use codec::{
    CommandMessage, ControlMessage, EventMessage, PayloadCodec, StreamSample, decode_command, decode_control,
    decode_event, decode_stream, encode_command, encode_control, encode_event, encode_stream,
};
pub use crc::{Crc16, crc16};
pub use error::{CodecError, EncodeError, FrameError};
pub use frame::{Class, Direction, Flags, Frame, FrameHeader, HEADER_LEN, MAGIC, Seq, Status};
pub use ids::{BridgeStream, Caps, Command, Control, Event, ProcedureId, Stream};
pub use parser::{MAX_PC_TO_ROBOT_FRAME, MAX_ROBOT_TO_PC_FRAME, Parser, PcToRobotParser, RobotToPcParser};
pub use payload::{
    bridge::{BridgeStation, BridgeStatus, BridgeStatusFlags, CHANNEL_UNKNOWN, RSSI_UNKNOWN_DBM},
    control::{
        BridgeControl, BridgeControlAction, BridgeControlReply, Hello, HelloAck, PAIRING_CODE_LEN, PairingCode, Ping,
        PingAck, RateEntries, RateEntry, RateFlags, SetupCredentials, Throttle as ThrottleHint, ThrottleAction,
        decode_bridge_control_ack, encode_bridge_control_ack, encode_hello, encode_hello_ack,
    },
    event::{
        CalibrationKind, CalibrationLoadStatus, CalibrationLoaded, CompletionStatus, DriveDistanceCompleted,
        EVENT_DETAIL_MAX, EVENT_PAYLOAD_MAX, EventAck, EventOverflow, FloorDrop, OBSTACLE_SOURCE_LIDAR, Obstacle,
        ProcedureCompleted, RotateCompleted,
    },
};
pub use reliability::{
    AckOutcome, AckTracker, CommandVerdict, CommandWindow, ReplayWindow, RetryPolicy, Throttle, ThrottleState,
    Timestamp,
};
#[cfg(all(feature = "std", not(target_os = "none")))]
pub use sim::{FakeRemote, Faults, Wire};

/// The wire major version carried in every frame header.
pub const PROTOCOL_MAJOR: u8 = 1;

/// The wire minor version exchanged in the session handshake.
pub const PROTOCOL_MINOR: u8 = 1;
