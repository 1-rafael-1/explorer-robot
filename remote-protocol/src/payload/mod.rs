//! The typed v1 payloads (ADR-0031).
//!
//! Every payload is little-endian and follows ADR-0031's byte layout exactly. Decoding is strict:
//! a structural mismatch is [`CodecError::Length`](crate::error::CodecError::Length), an
//! out-of-range value is [`CodecError::Value`](crate::error::CodecError::Value), and invalid text
//! is [`CodecError::Utf8`](crate::error::CodecError::Utf8). Sentinels are ordinary wire values and
//! survive a round trip unchanged.
//!
//! Streams live in [`stream`]; commands and their query responses live in [`command`]; the
//! reliable events live in [`event`]; the session control payloads live in [`control`].

pub mod command;
pub mod control;
pub mod event;
pub mod stream;

pub use command::{
    AckPayload, CatalogueEntries, CatalogueEntry, CatalogueFlags, CataloguePage, DriveDistance, GetCatalogue,
    GetStatus, RotateBy, StartProcedure, StatusSnapshot, StopMotion, StopProcedure,
};
pub use control::{
    Hello, HelloAck, Ping, PingAck, RateEntries, RateEntry, RateFlags, Throttle, ThrottleAction, encode_hello,
    encode_hello_ack,
};
pub use event::{
    CalibrationKind, CalibrationLoadStatus, CalibrationLoaded, CompletionStatus, DriveDistanceCompleted,
    EVENT_DETAIL_MAX, EVENT_PAYLOAD_MAX, EventAck, EventOverflow, FloorDrop, OBSTACLE_SOURCE_LIDAR, Obstacle,
    ProcedureCompleted, RotateCompleted,
};
pub use stream::{
    Activity, ActivityStage, Calibration, CalibrationStatus, LidarCloud, Motion, Perception, PerceptionFlags, Power,
};
