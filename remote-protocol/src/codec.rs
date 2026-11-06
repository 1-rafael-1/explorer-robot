//! Typed payload codecs and the stream-keyed dispatch (ADR-0031).
//!
//! Each payload is a typed struct implementing [`PayloadCodec`]. The id dispatch is the exhaustive
//! [`Stream`] match in [`decode_stream`] and [`encode_stream`]; the codecs themselves never repeat
//! an id constant.
//!
//! Decoding borrows from the input. That matters for `ACTIVITY`, whose detail is a `&str` into the
//! frame payload, and it means no codec allocates.

use crate::{
    error::{CodecError, EncodeError},
    ids::Stream,
    payload::stream::{Activity, Calibration, LidarCloud, Motion, Perception, Power},
};

/// A typed payload that can be encoded into, and decoded from, a frame payload.
///
/// The lifetime parameter lets a borrowed payload such as [`Activity`] decode into a value that
/// borrows the input slice.
pub trait PayloadCodec<'a>: Sized {
    /// Decodes a payload from its wire bytes.
    ///
    /// # Errors
    ///
    /// [`CodecError::Length`] for a structural length mismatch, [`CodecError::Value`] for an
    /// out-of-range value and [`CodecError::Utf8`] for invalid text.
    fn decode(bytes: &'a [u8]) -> Result<Self, CodecError>;

    /// Encodes the payload into the start of `out`, returning the number of bytes written.
    ///
    /// # Errors
    ///
    /// [`EncodeError::BufferTooSmall`] when `out` is too short and [`EncodeError::Value`] when a
    /// field cannot be represented in the wire layout.
    fn encode_into(&self, out: &mut [u8]) -> Result<usize, EncodeError>;
}

/// One telemetry sample tagged with the stream it belongs to.
///
/// The borrow is only used by `ACTIVITY`'s detail; every other variant owns its data. The cloud
/// variant is 720 bytes by design: boxing it would require an allocator and break `Copy`, and the
/// sample is passed by reference into the robot's `publish_stream`.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamSample<'a> {
    /// A `POWER` sample.
    Power(Power),
    /// A `MOTION` sample.
    Motion(Motion),
    /// An `ACTIVITY` sample with a borrowed detail.
    Activity(Activity<'a>),
    /// A `PERCEPTION` sample.
    Perception(Perception),
    /// A `LIDAR_CLOUD` sample.
    LidarCloud(LidarCloud),
    /// A `CALIBRATION` sample.
    Calibration(Calibration),
}

impl StreamSample<'_> {
    /// Returns the stream this sample belongs to.
    #[must_use]
    pub const fn stream(&self) -> Stream {
        match self {
            Self::Power(_) => Stream::Power,
            Self::Motion(_) => Stream::Motion,
            Self::Activity(_) => Stream::Activity,
            Self::Perception(_) => Stream::Perception,
            Self::LidarCloud(_) => Stream::LidarCloud,
            Self::Calibration(_) => Stream::Calibration,
        }
    }
}

/// Decodes one payload of `stream` from the frame's payload bytes.
///
/// # Errors
///
/// Returns the codec's [`CodecError`] when the payload does not follow ADR-0031's layout.
pub fn decode_stream(stream: Stream, bytes: &[u8]) -> Result<StreamSample<'_>, CodecError> {
    match stream {
        Stream::Power => Power::decode(bytes).map(StreamSample::Power),
        Stream::Motion => Motion::decode(bytes).map(StreamSample::Motion),
        Stream::Activity => Activity::decode(bytes).map(StreamSample::Activity),
        Stream::Perception => Perception::decode(bytes).map(StreamSample::Perception),
        Stream::LidarCloud => LidarCloud::decode(bytes).map(StreamSample::LidarCloud),
        Stream::Calibration => Calibration::decode(bytes).map(StreamSample::Calibration),
    }
}

/// Encodes one stream sample into `out`, returning the number of payload bytes written.
///
/// # Errors
///
/// Returns [`EncodeError::BufferTooSmall`] when `out` cannot hold the payload and
/// [`EncodeError::Value`] when a field cannot be represented.
pub fn encode_stream(sample: &StreamSample<'_>, out: &mut [u8]) -> Result<usize, EncodeError> {
    match sample {
        StreamSample::Power(power) => power.encode_into(out),
        StreamSample::Motion(motion) => motion.encode_into(out),
        StreamSample::Activity(activity) => activity.encode_into(out),
        StreamSample::Perception(perception) => perception.encode_into(out),
        StreamSample::LidarCloud(cloud) => cloud.encode_into(out),
        StreamSample::Calibration(calibration) => calibration.encode_into(out),
    }
}

/// Reads a little-endian `u16` at `offset`.
///
/// The caller checks the length first; every codec validates its layout before calling this.
pub(crate) fn read_u16_le(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

/// Writes a little-endian `u16` at `offset`.
///
/// The caller checks the length first; every codec checks its output buffer before calling this.
pub(crate) fn write_u16_le(out: &mut [u8], offset: usize, value: u16) {
    let [low, high] = value.to_le_bytes();
    out[offset] = low;
    out[offset + 1] = high;
}
