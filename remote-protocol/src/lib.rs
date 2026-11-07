//! The shared Link Protocol contract (ADR-0030, ADR-0031, ADR-0040).
//!
//! This crate is `no_std` and transport agnostic: it carries the frame envelope, the shared
//! identifiers and the payload codecs that the robot, the Bridge and the daemon all speak. It has
//! no UART, clock, allocator or logging. Enable the `std` feature only for host tests and the
//! simulated lossy link.
//!
//! The crate version mirrors the wire version: changing [`PROTOCOL_MAJOR`] or [`PROTOCOL_MINOR`]
//! changes the crate's major or minor version, and every consumer states a `version = "1"`
//! requirement so a major bump cannot pass unnoticed (ADR-0040).

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(feature = "std")]
extern crate std;

pub mod codec;
pub mod crc;
pub mod error;
pub mod frame;
pub mod ids;
pub mod parser;
pub mod payload;
pub mod reliability;
#[cfg(feature = "std")]
pub mod sim;

pub use codec::{
    CommandMessage, EventMessage, PayloadCodec, StreamSample, decode_command, decode_event, decode_stream,
    encode_command, encode_event, encode_stream,
};
pub use crc::{Crc16, crc16};
pub use error::{CodecError, EncodeError, FrameError};
pub use frame::{Class, Direction, Flags, Frame, FrameHeader, HEADER_LEN, MAGIC, Seq, Status};
pub use ids::{Caps, Command, Control, Event, ProcedureId, Stream};
pub use parser::{MAX_PC_TO_ROBOT_FRAME, MAX_ROBOT_TO_PC_FRAME, Parser, PcToRobotParser, RobotToPcParser};
pub use payload::{
    control::{Hello, HelloAck, Ping, PingAck, RateEntries, RateEntry, RateFlags, encode_hello, encode_hello_ack},
    event::{
        CalibrationKind, CalibrationLoadStatus, CalibrationLoaded, CompletionStatus, DriveDistanceCompleted,
        EVENT_DETAIL_MAX, EVENT_PAYLOAD_MAX, EventAck, EventOverflow, FloorDrop, OBSTACLE_SOURCE_LIDAR, Obstacle,
        ProcedureCompleted, RotateCompleted,
    },
};
pub use reliability::{AckOutcome, AckTracker, ReplayWindow, RetryPolicy};
#[cfg(feature = "std")]
pub use sim::{FakeRemote, Faults, Wire};

/// The wire major version carried in every frame header.
pub const PROTOCOL_MAJOR: u8 = 1;

/// The wire minor version exchanged in the session handshake.
pub const PROTOCOL_MINOR: u8 = 0;
