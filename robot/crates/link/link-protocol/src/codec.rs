//! Typed payload codecs and the class-keyed dispatch (ADR-0031).
//!
//! Each payload is a typed struct implementing [`PayloadCodec`]. The id dispatch is the exhaustive
//! [`Stream`] match in [`decode_stream`] and [`encode_stream`] for telemetry, and the exhaustive
//! [`Command`] match in [`decode_command`] and [`encode_command`] for commands; the codecs
//! themselves never repeat an id constant.
//!
//! Decoding borrows from the input. That matters for `ACTIVITY`, whose detail is a `&str` into the
//! frame payload, and for the catalogue page and status snapshot, and it means no codec allocates.

use crate::{
    error::{CodecError, EncodeError},
    ids::{Command, Control, Event, Stream},
    payload::{
        bridge::BridgeStatus,
        command::{
            AckPayload, CataloguePage, DriveDistance, GetCatalogue, GetStatus, RotateBy, StartProcedure,
            StatusSnapshot, StopMotion, StopProcedure,
        },
        control::{BridgeControl, Hello, HelloAck, Ping, PingAck, Throttle},
        event::{
            CalibrationLoaded, DriveDistanceCompleted, EventAck, EventOverflow, FloorDrop, Obstacle,
            ProcedureCompleted, RotateCompleted,
        },
        stream::{Activity, Calibration, LidarCloud, Motion, Perception, Power},
    },
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

/// One typed command payload, tagged with the command it belongs to.
///
/// Requests are the seven PC-to-robot payloads. Acknowledgements are the generic
/// [`AckPayload`] for ordinary commands and the two typed query responses for `GET_CATALOGUE` and
/// `GET_STATUS`. The borrow is used by the opaque `START_PROCEDURE` parameters, the catalogue
/// labels and the status detail.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandMessage<'a> {
    /// A `START_PROCEDURE` request.
    StartProcedure(StartProcedure<'a>),
    /// A `STOP_PROCEDURE` request.
    StopProcedure(StopProcedure),
    /// A `DRIVE_DISTANCE` request.
    DriveDistance(DriveDistance),
    /// A `ROTATE_BY` request.
    RotateBy(RotateBy),
    /// A `STOP_MOTION` request.
    StopMotion(StopMotion),
    /// A `GET_CATALOGUE` request.
    GetCatalogue(GetCatalogue),
    /// A `GET_STATUS` request.
    GetStatus(GetStatus),
    /// A `GET_CATALOGUE` acknowledgement.
    CataloguePage(CataloguePage<'a>),
    /// A `GET_STATUS` acknowledgement.
    StatusSnapshot(StatusSnapshot<'a>),
    /// A generic command acknowledgement: a status plus optional result bytes.
    Ack(AckPayload<'a>),
}

impl CommandMessage<'_> {
    /// Returns the command this message belongs to.
    ///
    /// The generic [`CommandMessage::Ack`] carries no id, so it returns [`None`]; the caller pairs
    /// it with the frame's id, which ADR-0030 already mirrors from the request.
    #[must_use]
    pub const fn command(&self) -> Option<Command> {
        match self {
            Self::StartProcedure(_) => Some(Command::StartProcedure),
            Self::StopProcedure(_) => Some(Command::StopProcedure),
            Self::DriveDistance(_) => Some(Command::DriveDistance),
            Self::RotateBy(_) => Some(Command::RotateBy),
            Self::StopMotion(_) => Some(Command::StopMotion),
            Self::GetCatalogue(_) | Self::CataloguePage(_) => Some(Command::GetCatalogue),
            Self::GetStatus(_) | Self::StatusSnapshot(_) => Some(Command::GetStatus),
            Self::Ack(_) => None,
        }
    }
}

/// Decodes one command payload, branching on `IS_ACK` (ADR-0031).
///
/// A request decodes as its request type. An acknowledgement decodes as the typed response for
/// the two query commands and as the generic [`AckPayload`] for every other command.
///
/// # Errors
///
/// Returns [`CodecError::Length`] for a structural mismatch, [`CodecError::Value`] for an
/// out-of-range value (including an unassigned ACK status) and [`CodecError::Utf8`] for invalid
/// text.
pub fn decode_command(command: Command, is_ack: bool, bytes: &[u8]) -> Result<CommandMessage<'_>, CodecError> {
    if is_ack {
        return match command {
            Command::GetCatalogue => CataloguePage::decode(bytes).map(CommandMessage::CataloguePage),
            Command::GetStatus => StatusSnapshot::decode(bytes).map(CommandMessage::StatusSnapshot),
            Command::StartProcedure
            | Command::StopProcedure
            | Command::DriveDistance
            | Command::RotateBy
            | Command::StopMotion => AckPayload::decode(bytes).map(CommandMessage::Ack),
        };
    }
    match command {
        Command::StartProcedure => StartProcedure::decode(bytes).map(CommandMessage::StartProcedure),
        Command::StopProcedure => StopProcedure::decode(bytes).map(CommandMessage::StopProcedure),
        Command::DriveDistance => DriveDistance::decode(bytes).map(CommandMessage::DriveDistance),
        Command::RotateBy => RotateBy::decode(bytes).map(CommandMessage::RotateBy),
        Command::StopMotion => StopMotion::decode(bytes).map(CommandMessage::StopMotion),
        Command::GetCatalogue => GetCatalogue::decode(bytes).map(CommandMessage::GetCatalogue),
        Command::GetStatus => GetStatus::decode(bytes).map(CommandMessage::GetStatus),
    }
}

/// Encodes one typed command payload into `out`, returning the number of bytes written.
///
/// # Errors
///
/// Returns [`EncodeError::BufferTooSmall`] when `out` cannot hold the payload and
/// [`EncodeError::Value`] when a field cannot be represented in the wire layout.
pub fn encode_command(message: &CommandMessage<'_>, out: &mut [u8]) -> Result<usize, EncodeError> {
    match message {
        CommandMessage::StartProcedure(value) => value.encode_into(out),
        CommandMessage::StopProcedure(value) => value.encode_into(out),
        CommandMessage::DriveDistance(value) => value.encode_into(out),
        CommandMessage::RotateBy(value) => value.encode_into(out),
        CommandMessage::StopMotion(value) => value.encode_into(out),
        CommandMessage::GetCatalogue(value) => value.encode_into(out),
        CommandMessage::GetStatus(value) => value.encode_into(out),
        CommandMessage::CataloguePage(value) => value.encode_into(out),
        CommandMessage::StatusSnapshot(value) => value.encode_into(out),
        CommandMessage::Ack(value) => value.encode_into(out),
    }
}

/// Returns the longest prefix of `text` that fits in `max` bytes without splitting a character.
pub(crate) fn truncate_utf8(text: &str, max: usize) -> &str {
    if text.len() <= max {
        return text;
    }
    let mut end = max;
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    &text[..end]
}

/// One typed event payload, tagged with the event it belongs to.
///
/// The two drive completions and the Procedure completion borrow their detail from the payload on
/// decode. The generic [`EventMessage::Ack`] is the peer's acknowledgement and carries the shared
/// diagnostic status rather than an event id.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventMessage<'a> {
    /// An `OBSTACLE` event.
    Obstacle(Obstacle),
    /// A `FLOOR_DROP` event.
    FloorDrop(FloorDrop),
    /// A `PROCEDURE_COMPLETED` event.
    ProcedureCompleted(ProcedureCompleted<'a>),
    /// A `DRIVE_DISTANCE_COMPLETED` event.
    DriveDistanceCompleted(DriveDistanceCompleted<'a>),
    /// A `ROTATE_COMPLETED` event.
    RotateCompleted(RotateCompleted<'a>),
    /// A `CALIBRATION_LOADED` event.
    CalibrationLoaded(CalibrationLoaded),
    /// An `EVENT_OVERFLOW` event.
    EventOverflow(EventOverflow),
    /// An acknowledgement of an event.
    Ack(EventAck),
}

impl EventMessage<'_> {
    /// Returns the event this message belongs to.
    ///
    /// The generic [`EventMessage::Ack`] mirrors the frame's id rather than carrying one, so it
    /// returns [`None`].
    #[must_use]
    pub const fn event(&self) -> Option<Event> {
        match self {
            Self::Obstacle(_) => Some(Event::Obstacle),
            Self::FloorDrop(_) => Some(Event::FloorDrop),
            Self::ProcedureCompleted(_) => Some(Event::ProcedureCompleted),
            Self::DriveDistanceCompleted(_) => Some(Event::DriveDistanceCompleted),
            Self::RotateCompleted(_) => Some(Event::RotateCompleted),
            Self::CalibrationLoaded(_) => Some(Event::CalibrationLoaded),
            Self::EventOverflow(_) => Some(Event::EventOverflow),
            Self::Ack(_) => None,
        }
    }
}

/// Decodes one event payload, branching on `IS_ACK` (ADR-0031).
///
/// An event decodes as its payload type. An acknowledgement decodes as the shared [`EventAck`].
///
/// # Errors
///
/// Returns [`CodecError::Length`] for a structural mismatch, [`CodecError::Value`] for an
/// out-of-range value (including an unassigned completion status or ACK status) and
/// [`CodecError::Utf8`] for invalid text.
pub fn decode_event(event: Event, is_ack: bool, bytes: &[u8]) -> Result<EventMessage<'_>, CodecError> {
    if is_ack {
        return EventAck::decode(bytes).map(EventMessage::Ack);
    }
    match event {
        Event::Obstacle => Obstacle::decode(bytes).map(EventMessage::Obstacle),
        Event::FloorDrop => FloorDrop::decode(bytes).map(EventMessage::FloorDrop),
        Event::ProcedureCompleted => ProcedureCompleted::decode(bytes).map(EventMessage::ProcedureCompleted),
        Event::DriveDistanceCompleted => {
            DriveDistanceCompleted::decode(bytes).map(EventMessage::DriveDistanceCompleted)
        }
        Event::RotateCompleted => RotateCompleted::decode(bytes).map(EventMessage::RotateCompleted),
        Event::CalibrationLoaded => CalibrationLoaded::decode(bytes).map(EventMessage::CalibrationLoaded),
        Event::EventOverflow => EventOverflow::decode(bytes).map(EventMessage::EventOverflow),
    }
}

/// Encodes one typed event payload into `out`, returning the number of bytes written.
///
/// # Errors
///
/// Returns [`EncodeError::BufferTooSmall`] when `out` cannot hold the payload and
/// [`EncodeError::Value`] when a field cannot be represented in the wire layout.
pub fn encode_event(message: &EventMessage<'_>, out: &mut [u8]) -> Result<usize, EncodeError> {
    match message {
        EventMessage::Obstacle(value) => value.encode_into(out),
        EventMessage::FloorDrop(value) => value.encode_into(out),
        EventMessage::ProcedureCompleted(value) => value.encode_into(out),
        EventMessage::DriveDistanceCompleted(value) => value.encode_into(out),
        EventMessage::RotateCompleted(value) => value.encode_into(out),
        EventMessage::CalibrationLoaded(value) => value.encode_into(out),
        EventMessage::EventOverflow(value) => value.encode_into(out),
        EventMessage::Ack(value) => value.encode_into(out),
    }
}

/// One typed control payload, tagged with the control message it belongs to.
///
/// `HELLO` and `PING` branch on `IS_ACK`; `THROTTLE` has no acknowledgement, so it is the same
/// type in both directions and [`decode_control`] ignores `IS_ACK` for it. `BRIDGE_CONTROL` has a
/// one-byte request here; its acknowledgement is action-dependent and is decoded with
/// [`decode_bridge_control_ack`](crate::payload::control::decode_bridge_control_ack), because the
/// wire acknowledgement does not repeat the action.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlMessage<'a> {
    /// A `HELLO` request.
    Hello(Hello<'a>),
    /// A `HELLO_ACK` acknowledgement.
    HelloAck(HelloAck<'a>),
    /// A `PING` request.
    Ping(Ping),
    /// A `PING` acknowledgement.
    PingAck(PingAck),
    /// A `THROTTLE` hint.
    Throttle(Throttle),
    /// A `BRIDGE_CONTROL` request.
    BridgeControl(BridgeControl),
    /// A `BRIDGE_STATUS` snapshot.
    BridgeStatus(BridgeStatus),
}

impl ControlMessage<'_> {
    /// Returns the control id this message belongs to.
    #[must_use]
    pub const fn control(&self) -> Control {
        match self {
            Self::Hello(_) | Self::HelloAck(_) => Control::Hello,
            Self::Ping(_) | Self::PingAck(_) => Control::Ping,
            Self::Throttle(_) => Control::Throttle,
            Self::BridgeControl(_) => Control::BridgeControl,
            Self::BridgeStatus(_) => Control::BridgeStatus,
        }
    }
}

/// Decodes one control payload, branching on `IS_ACK` for the acknowledged controls.
///
/// `THROTTLE` is decoded identically whether or not `IS_ACK` is set, because it has no
/// acknowledgement. A `BRIDGE_CONTROL` request decodes from its one action byte; its
/// acknowledgement cannot be decoded without the outstanding request's action, so this function
/// reports [`CodecError::Length`] for it. Use
/// [`decode_bridge_control_ack`](crate::payload::control::decode_bridge_control_ack) for that
/// frame.
///
/// # Errors
///
/// Returns [`CodecError::Length`] for a structural mismatch, [`CodecError::Value`] for an
/// out-of-range value (including an unassigned `THROTTLE` or `BRIDGE_CONTROL` action or ACK
/// status) and [`CodecError::Utf8`] for invalid text.
pub fn decode_control(control: Control, is_ack: bool, bytes: &[u8]) -> Result<ControlMessage<'_>, CodecError> {
    match control {
        Control::Hello if is_ack => HelloAck::decode(bytes).map(ControlMessage::HelloAck),
        Control::Hello => Hello::decode(bytes).map(ControlMessage::Hello),
        Control::Ping if is_ack => PingAck::decode(bytes).map(ControlMessage::PingAck),
        Control::Ping => Ping::decode(bytes).map(ControlMessage::Ping),
        Control::Throttle => Throttle::decode(bytes).map(ControlMessage::Throttle),
        Control::BridgeControl | Control::BridgeStatus if is_ack => Err(CodecError::Length),
        Control::BridgeControl => BridgeControl::decode(bytes).map(ControlMessage::BridgeControl),
        Control::BridgeStatus => BridgeStatus::decode(bytes).map(ControlMessage::BridgeStatus),
    }
}

/// Encodes one typed control payload into `out`, returning the number of bytes written.
///
/// # Errors
///
/// Returns [`EncodeError::BufferTooSmall`] when `out` cannot hold the payload and
/// [`EncodeError::Value`] when a field cannot be represented in the wire layout.
pub fn encode_control(message: &ControlMessage<'_>, out: &mut [u8]) -> Result<usize, EncodeError> {
    match message {
        ControlMessage::Hello(value) => value.encode_into(out),
        ControlMessage::HelloAck(value) => value.encode_into(out),
        ControlMessage::Ping(value) => value.encode_into(out),
        ControlMessage::PingAck(value) => value.encode_into(out),
        ControlMessage::Throttle(value) => value.encode_into(out),
        ControlMessage::BridgeControl(value) => value.encode_into(out),
        ControlMessage::BridgeStatus(value) => value.encode_into(out),
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

/// Reads a little-endian `u32` at `offset`.
///
/// The caller checks the length first; every codec validates its layout before calling this.
pub(crate) fn read_u32_le(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([bytes[offset], bytes[offset + 1], bytes[offset + 2], bytes[offset + 3]])
}

/// Writes a little-endian `u32` at `offset`.
///
/// The caller checks the length first; every codec checks its output buffer before calling this.
pub(crate) fn write_u32_le(out: &mut [u8], offset: usize, value: u32) {
    let [b0, b1, b2, b3] = value.to_le_bytes();
    out[offset] = b0;
    out[offset + 1] = b1;
    out[offset + 2] = b2;
    out[offset + 3] = b3;
}
