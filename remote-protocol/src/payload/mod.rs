//! The typed v1 telemetry payloads (ADR-0031).
//!
//! Every payload is little-endian and follows ADR-0031's byte layout exactly. Decoding is strict:
//! a structural mismatch is [`CodecError::Length`], an out-of-range value is
//! [`CodecError::Value`], and invalid text is [`CodecError::Utf8`]. Sentinels are ordinary wire
//! values and survive a round trip unchanged.

pub mod stream;

pub use stream::{
    Activity, ActivityStage, Calibration, CalibrationStatus, LidarCloud, Motion, Perception, PerceptionFlags, Power,
};
