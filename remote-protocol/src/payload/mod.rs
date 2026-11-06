//! The typed v1 payloads (ADR-0031).
//!
//! Every payload is little-endian and follows ADR-0031's byte layout exactly. Decoding is strict:
//! a structural mismatch is [`CodecError::Length`](crate::error::CodecError::Length), an
//! out-of-range value is [`CodecError::Value`](crate::error::CodecError::Value), and invalid text
//! is [`CodecError::Utf8`](crate::error::CodecError::Utf8). Sentinels are ordinary wire values and
//! survive a round trip unchanged.
//!
//! Streams live in [`stream`]; commands and their query responses live in [`command`].

pub mod command;
pub mod stream;

pub use command::{
    AckPayload, CatalogueEntries, CatalogueEntry, CatalogueFlags, CataloguePage, DriveDistance, GetCatalogue,
    GetStatus, RotateBy, StartProcedure, StatusSnapshot, StopMotion, StopProcedure,
};
pub use stream::{
    Activity, ActivityStage, Calibration, CalibrationStatus, LidarCloud, Motion, Perception, PerceptionFlags, Power,
};
