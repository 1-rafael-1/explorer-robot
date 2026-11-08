//! The typed v1 payloads (ADR-0031).
//!
//! Every payload is little-endian and follows ADR-0031's byte layout exactly. Decoding is strict:
//! a structural mismatch is [`CodecError::Length`](crate::error::CodecError::Length), an
//! out-of-range value is [`CodecError::Value`](crate::error::CodecError::Value), and invalid text
//! is [`CodecError::Utf8`](crate::error::CodecError::Utf8). Sentinels are ordinary wire values and
//! survive a round trip unchanged.
//!
//! Streams live in [`stream`]; commands and their query responses live in [`command`]; the
//! reliable events live in [`event`]; the session and Bridge control payloads live in [`control`].
//! The Bridge-originated status snapshot lives in [`bridge`].

pub mod bridge;
pub mod command;
pub mod control;
pub mod event;
pub mod stream;

pub use bridge::{BridgeStation, BridgeStatus, BridgeStatusFlags, CHANNEL_UNKNOWN, RSSI_UNKNOWN_DBM};
pub use command::{
    AckPayload, CatalogueEntries, CatalogueEntry, CatalogueFlags, CataloguePage, DriveDistance, GetCatalogue,
    GetStatus, RotateBy, StartProcedure, StatusSnapshot, StopMotion, StopProcedure,
};
pub use control::{
    BridgeControl, BridgeControlAction, BridgeControlReply, Hello, HelloAck, PAIRING_CODE_LEN, PairingCode, Ping,
    PingAck, RateEntries, RateEntry, RateFlags, SetupCredentials, Throttle, ThrottleAction, decode_bridge_control_ack,
    encode_bridge_control_ack, encode_hello, encode_hello_ack,
};
pub use event::{
    CalibrationKind, CalibrationLoadStatus, CalibrationLoaded, CompletionStatus, DriveDistanceCompleted,
    EVENT_DETAIL_MAX, EVENT_PAYLOAD_MAX, EventAck, EventOverflow, FloorDrop, OBSTACLE_SOURCE_LIDAR, Obstacle,
    ProcedureCompleted, RotateCompleted,
};
pub use stream::{
    Activity, ActivityStage, Calibration, CalibrationStatus, LidarCloud, Motion, Perception, PerceptionFlags, Power,
};
