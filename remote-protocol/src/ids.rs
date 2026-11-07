//! Shared wire identifiers (ADR-0030, ADR-0031).
//!
//! The enums in this module have explicit `#[repr(u16)]` discriminants that **are** the wire ids,
//! so `wire_id` is a cast and `from_wire` is the only place that knows how a raw id maps back to a
//! variant. Only implemented ids become variants; the reserved ids stay in the tables below and
//! stay out of [`Stream::ALL`] until a later major or minor implements them.
//!
//! Reserved stream ids (never variants until implemented):
//!
//! | id | stream | meaning |
//! |---|---|---|
//! | `0x0010` | `POSE` | reserved for odometry |
//! | `0x0011` | `POSE_TRAIL` | reserved |
//! | `0x0012` | `MAP` | reserved |
//! | `0x0013` | `CAMERA_DETECTIONS` | reserved |
//! | `0x0014` | `LOG` | reserved |
//!
//! Reserved command ids (never variants until implemented):
//!
//! | id | command | meaning |
//! |---|---|---|
//! | `0x0010` | `DRIVE_TO_POSE` | absolute |
//! | `0x0011` | `DRIVE_TO_BEARING` | absolute |
//! | `0x0012` | `SET_DIFFERENTIAL` | joystick/teleop |
//! | `0x0013` | `GET_POSE` | reserved |
//! | `0x0014` | `GET_MAP` | reserved |
//! | `0x0015` | `GET_CAMERA` | reserved |
//!
//! Reserved event ids (never variants until implemented):
//!
//! | id | event | meaning |
//! |---|---|---|
//! | `0x0008–0x000F` | — | low battery, map and landmark updates |
//!
//! Ids are namespaced by class, so a stream, command, event and control may each use `0x0001`.
//! `0x0007–0x000F` stay free for future core streams, and ids are never reassigned.

/// A telemetry stream id (robot to PC, lossy and latest-wins).
///
/// The discriminants are the ADR-0031 wire ids. This enum is deliberately not `#[non_exhaustive]`:
/// a new stream is a reviewed change to the catalogue.
#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Stream {
    /// `POWER`: battery level and pack voltage.
    Power = 0x0001,
    /// `MOTION`: left and right motor output.
    Motion = 0x0002,
    /// `ACTIVITY`: the current Procedure and its progress.
    Activity = 0x0003,
    /// `PERCEPTION`: obstacle and floor-drop flags.
    Perception = 0x0004,
    /// `LIDAR_CLOUD`: the 360-slot range grid in whole centimetres.
    LidarCloud = 0x0005,
    /// `CALIBRATION`: the four calibration status bytes.
    Calibration = 0x0006,
}

impl Stream {
    /// Every implemented stream, in wire order.
    pub const ALL: [Self; 6] = [
        Self::Power,
        Self::Motion,
        Self::Activity,
        Self::Perception,
        Self::LidarCloud,
        Self::Calibration,
    ];

    /// The number of implemented streams.
    pub const COUNT: usize = Self::ALL.len();

    /// Returns the wire id of this stream.
    #[must_use]
    pub const fn wire_id(self) -> u16 {
        self as u16
    }

    /// Returns the position of this stream in [`Stream::ALL`].
    ///
    /// The index is the slot and sequence-counter position in `remote-link`.
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::Power => 0,
            Self::Motion => 1,
            Self::Activity => 2,
            Self::Perception => 3,
            Self::LidarCloud => 4,
            Self::Calibration => 5,
        }
    }

    /// Builds a stream from its wire id, returning [`None`] for an unimplemented or reserved id.
    ///
    /// The match is exhaustive over the implemented variants; every other id is unknown to this
    /// version of the protocol and is ignored by the receiver.
    #[must_use]
    pub const fn from_wire(id: u16) -> Option<Self> {
        match id {
            0x0001 => Some(Self::Power),
            0x0002 => Some(Self::Motion),
            0x0003 => Some(Self::Activity),
            0x0004 => Some(Self::Perception),
            0x0005 => Some(Self::LidarCloud),
            0x0006 => Some(Self::Calibration),
            _ => None,
        }
    }
}

/// A command id (PC to robot, reliable and at-most-once).
///
/// The discriminants are the ADR-0031 wire ids. This enum is deliberately not
/// `#[non_exhaustive]`: a new command is a reviewed change to the catalogue. Reserved command ids
/// stay out of [`Command::ALL`] until a later minor implements them.
#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Command {
    /// `START_PROCEDURE`: start a Procedure with opaque parameters.
    StartProcedure = 0x0001,
    /// `STOP_PROCEDURE`: stop a Procedure by id.
    StopProcedure = 0x0002,
    /// `DRIVE_DISTANCE`: drive a relative distance.
    DriveDistance = 0x0003,
    /// `ROTATE_BY`: rotate by a relative angle.
    RotateBy = 0x0004,
    /// `STOP_MOTION`: coast or brake immediately.
    StopMotion = 0x0005,
    /// `GET_CATALOGUE`: read one page of the Procedure catalogue.
    GetCatalogue = 0x0006,
    /// `GET_STATUS`: read one snapshot of the robot's status.
    GetStatus = 0x0007,
}

impl Command {
    /// Every implemented command, in wire order.
    pub const ALL: [Self; 7] = [
        Self::StartProcedure,
        Self::StopProcedure,
        Self::DriveDistance,
        Self::RotateBy,
        Self::StopMotion,
        Self::GetCatalogue,
        Self::GetStatus,
    ];

    /// The number of implemented commands.
    pub const COUNT: usize = Self::ALL.len();

    /// Returns the wire id of this command.
    #[must_use]
    pub const fn wire_id(self) -> u16 {
        self as u16
    }

    /// Returns the position of this command in [`Command::ALL`].
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::StartProcedure => 0,
            Self::StopProcedure => 1,
            Self::DriveDistance => 2,
            Self::RotateBy => 3,
            Self::StopMotion => 4,
            Self::GetCatalogue => 5,
            Self::GetStatus => 6,
        }
    }

    /// Builds a command from its wire id, returning [`None`] for an unimplemented or reserved id.
    ///
    /// The match is exhaustive over the implemented variants; every other id is unknown to this
    /// version of the protocol and is answered with `UNKNOWN_ID`.
    #[must_use]
    pub const fn from_wire(id: u16) -> Option<Self> {
        match id {
            0x0001 => Some(Self::StartProcedure),
            0x0002 => Some(Self::StopProcedure),
            0x0003 => Some(Self::DriveDistance),
            0x0004 => Some(Self::RotateBy),
            0x0005 => Some(Self::StopMotion),
            0x0006 => Some(Self::GetCatalogue),
            0x0007 => Some(Self::GetStatus),
            _ => None,
        }
    }
}

/// A control-frame id (ADR-0031).
///
/// The discriminants are the ADR-0031 wire ids. This enum is deliberately not
/// `#[non_exhaustive]`: a new control message is a reviewed change to the catalogue. `THROTTLE`
/// and `BRIDGE_CONTROL` are part of the vocabulary before their payload codecs exist; a receiver
/// ignores a control id it has no payload for.
#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Control {
    /// `HELLO`: the session handshake request and its acknowledgement.
    Hello = 0x0001,
    /// `PING`: the liveness probe and its acknowledgement.
    Ping = 0x0002,
    /// `THROTTLE`: the advisory flow-control hint from the Bridge or the daemon.
    Throttle = 0x0003,
    /// `BRIDGE_CONTROL`: a robot-to-Bridge request, acknowledged by the Bridge.
    BridgeControl = 0x0004,
}

impl Control {
    /// Every control id in this version, in wire order.
    pub const ALL: [Self; 4] = [Self::Hello, Self::Ping, Self::Throttle, Self::BridgeControl];

    /// The number of control ids in this version.
    pub const COUNT: usize = Self::ALL.len();

    /// Returns the wire id of this control message.
    #[must_use]
    pub const fn wire_id(self) -> u16 {
        self as u16
    }

    /// Returns the position of this control message in [`Control::ALL`].
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::Hello => 0,
            Self::Ping => 1,
            Self::Throttle => 2,
            Self::BridgeControl => 3,
        }
    }

    /// Builds a control message from its wire id, returning [`None`] for a reserved id.
    ///
    /// The match is exhaustive over the implemented variants; every other id is unknown to this
    /// version of the protocol and is ignored by the receiver.
    #[must_use]
    pub const fn from_wire(id: u16) -> Option<Self> {
        match id {
            0x0001 => Some(Self::Hello),
            0x0002 => Some(Self::Ping),
            0x0003 => Some(Self::Throttle),
            0x0004 => Some(Self::BridgeControl),
            _ => None,
        }
    }
}

/// An event id (robot to PC, reliable and at-least-once).
///
/// The discriminants are the ADR-0031 wire ids. This enum is deliberately not
/// `#[non_exhaustive]`: a new event is a reviewed change to the catalogue. The reserved event ids
/// stay out of [`Event::ALL`] until a later minor implements them.
#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Event {
    /// `OBSTACLE`: the obstacle flag changed.
    Obstacle = 0x0001,
    /// `FLOOR_DROP`: the floor-drop flag changed.
    FloorDrop = 0x0002,
    /// `PROCEDURE_COMPLETED`: a Procedure reached a terminal outcome.
    ProcedureCompleted = 0x0003,
    /// `DRIVE_DISTANCE_COMPLETED`: a `DRIVE_DISTANCE` command finished.
    DriveDistanceCompleted = 0x0004,
    /// `ROTATE_COMPLETED`: a `ROTATE_BY` command finished.
    RotateCompleted = 0x0005,
    /// `CALIBRATION_LOADED`: a boot-time calibration load finished.
    CalibrationLoaded = 0x0006,
    /// `EVENT_OVERFLOW`: the event ring dropped one or more events.
    EventOverflow = 0x0007,
}

impl Event {
    /// Every implemented event, in wire order.
    pub const ALL: [Self; 7] = [
        Self::Obstacle,
        Self::FloorDrop,
        Self::ProcedureCompleted,
        Self::DriveDistanceCompleted,
        Self::RotateCompleted,
        Self::CalibrationLoaded,
        Self::EventOverflow,
    ];

    /// The number of implemented events.
    pub const COUNT: usize = Self::ALL.len();

    /// Returns the wire id of this event.
    #[must_use]
    pub const fn wire_id(self) -> u16 {
        self as u16
    }

    /// Returns the position of this event in [`Event::ALL`].
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::Obstacle => 0,
            Self::FloorDrop => 1,
            Self::ProcedureCompleted => 2,
            Self::DriveDistanceCompleted => 3,
            Self::RotateCompleted => 4,
            Self::CalibrationLoaded => 5,
            Self::EventOverflow => 6,
        }
    }

    /// Builds an event from its wire id, returning [`None`] for an unimplemented or reserved id.
    ///
    /// The match is exhaustive over the implemented variants; every other id is unknown to this
    /// version of the protocol and is ignored by the receiver.
    #[must_use]
    pub const fn from_wire(id: u16) -> Option<Self> {
        match id {
            0x0001 => Some(Self::Obstacle),
            0x0002 => Some(Self::FloorDrop),
            0x0003 => Some(Self::ProcedureCompleted),
            0x0004 => Some(Self::DriveDistanceCompleted),
            0x0005 => Some(Self::RotateCompleted),
            0x0006 => Some(Self::CalibrationLoaded),
            0x0007 => Some(Self::EventOverflow),
            _ => None,
        }
    }
}

/// A Procedure id carried by `ACTIVITY` and by Procedure commands (ADR-0031).
///
/// This is a `u16` newtype rather than an enum because the wire must tolerate a Procedure id that
/// a peer does not know. The exhaustive "is this procedure exposed" decision belongs to the
/// firmware's `Procedure -> ProcedureId` mapping, not to this type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ProcedureId(u16);

impl ProcedureId {
    /// Idle, Activity-only and never startable.
    pub const IDLE: Self = Self(0x0000);
    /// Boot, Activity-only and never startable.
    pub const BOOT: Self = Self(0x0001);
    /// Motor calibration.
    pub const MOTOR_CALIBRATION: Self = Self(0x0002);
    /// Magnetometer calibration.
    pub const MAG_CALIBRATION: Self = Self(0x0003);
    /// Distance calibration.
    pub const DISTANCE_CALIBRATION: Self = Self(0x0004);
    /// Coast and avoid.
    pub const COAST_AND_AVOID: Self = Self(0x0005);
    /// Attempt straight; the only Procedure with parameters.
    pub const ATTEMPT_STRAIGHT: Self = Self(0x0006);
    /// Basic motor.
    pub const BASIC_MOTOR: Self = Self(0x0007);
    /// Turns.
    pub const TURNS: Self = Self(0x0008);
    /// Straight drive.
    pub const STRAIGHT_DRIVE: Self = Self(0x0009);
    /// Arc drive.
    pub const ARC_DRIVE: Self = Self(0x000A);
    /// Six-axis IMU test.
    pub const IMU_6_AXIS: Self = Self(0x000B);
    /// Nine-axis IMU test.
    pub const IMU_9_AXIS: Self = Self(0x000C);

    /// Wraps a raw wire value, including one this version does not know.
    #[must_use]
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    /// Returns the raw wire value.
    #[must_use]
    pub const fn get(self) -> u16 {
        self.0
    }
}

impl From<u16> for ProcedureId {
    fn from(value: u16) -> Self {
        Self(value)
    }
}

/// The capability mode bitset exchanged in the session handshake (ADR-0030).
///
/// Only behavioural modes are negotiated, never individual streams, events or commands. Unknown
/// bits are preserved by [`Caps::from_bits`] so a newer peer's modes survive a round trip and are
/// simply ignored by this version.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Caps(u32);

impl Caps {
    /// The peer supports delta `LiDAR` clouds.
    pub const DELTA_CLOUD: Self = Self(0x0000_0001);
    /// The peer supports pose trails.
    pub const POSE_TRAIL: Self = Self(0x0000_0002);
    /// The peer supports map transfers.
    pub const MAP: Self = Self(0x0000_0004);
    /// The peer understands `THROTTLE` hints.
    pub const THROTTLE_SUPPORT: Self = Self(0x0000_0008);
    /// The peer supports fragmentation (reserved in v1).
    pub const FRAGMENTATION: Self = Self(0x0000_0010);

    /// Every mode bit defined in this version.
    pub const KNOWN: Self =
        Self(Self::DELTA_CLOUD.0 | Self::POSE_TRAIL.0 | Self::MAP.0 | Self::THROTTLE_SUPPORT.0 | Self::FRAGMENTATION.0);

    /// Wraps a raw bitset, keeping unknown bits so they can be ignored rather than lost.
    #[must_use]
    pub const fn from_bits(bits: u32) -> Self {
        Self(bits)
    }

    /// Returns the raw bitset.
    #[must_use]
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// Returns `true` when every bit in `other` is set.
    #[must_use]
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Returns `true` when no mode bit is set.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Returns the modes both peers support.
    #[must_use]
    pub const fn intersection(self, other: Self) -> Self {
        Self(self.0 & other.0)
    }
}

impl core::ops::BitOr for Caps {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitOrAssign for Caps {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}
