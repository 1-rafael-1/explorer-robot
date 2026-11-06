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
