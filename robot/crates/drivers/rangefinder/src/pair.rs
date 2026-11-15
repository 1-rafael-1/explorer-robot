//! Pair lifecycle and the drive gate policy.
//!
//! The rangefinder task owns both sensors and derives one [`PairState`] from
//! them. The drive dispatch consults [`gate`] before sending a motor command:
//! motion needs both sensors armed and a clear floor-drop flag.

/// The state of one sensor within the pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorState {
    /// Enabled, but not yet reporting a valid reading.
    Waiting,
    /// Reporting valid readings.
    Armed,
    /// Recovering after a fault.
    Recovering,
    /// Latched failed for this enable.
    Failed,
}

/// The state of the rangefinder pair.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PairState {
    /// The pair is disabled.
    Off = 0,
    /// Enabled and waiting for both sensors to arm.
    Arming = 1,
    /// Both sensors are ready.
    Armed = 2,
    /// At least one sensor is not ready.
    Degraded = 3,
    /// Both sensors failed.
    Failed = 4,
}

impl PairState {
    /// The encoded value used by lock-free status cells.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Decode a lock-free status cell value; unknown values mean `Off`.
    #[must_use]
    pub const fn from_u8(value: u8) -> Self {
        match value {
            1 => Self::Arming,
            2 => Self::Armed,
            3 => Self::Degraded,
            4 => Self::Failed,
            _ => Self::Off,
        }
    }
}

/// Derive the pair state from the enable flag and both sensor states.
///
/// `Arming` covers the normal bring-up where one sensor has produced its first
/// valid reading and the other has not. `Degraded` means a sensor is recovering
/// or has failed, or one sensor is armed while the other has failed.
#[must_use]
pub const fn pair_state(enabled: bool, front: SensorState, rear: SensorState) -> PairState {
    if !enabled {
        return PairState::Off;
    }
    match (front, rear) {
        (SensorState::Armed, SensorState::Armed) => PairState::Armed,
        (SensorState::Failed, SensorState::Failed) => PairState::Failed,
        (SensorState::Waiting | SensorState::Armed, SensorState::Waiting)
        | (SensorState::Waiting, SensorState::Armed) => PairState::Arming,
        _ => PairState::Degraded,
    }
}

/// Why the drive gate refused a movement action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateRefusal {
    /// The pair is not armed.
    NotReady,
    /// A sensor reports a floor drop.
    FloorDrop,
}

impl GateRefusal {
    /// The exact failure reason reported to the caller.
    #[must_use]
    pub const fn reason(self) -> &'static str {
        match self {
            Self::NotReady => "floor-drop sensors not ready",
            Self::FloorDrop => "floor drop detected",
        }
    }
}

/// Decide whether a movement action may proceed.
///
/// A set floor-drop flag is refused as [`GateRefusal::FloorDrop`] even when the
/// pair is not armed, because a drop already present is the more specific
/// safety cause. Otherwise only [`PairState::Armed`] passes; every other state
/// is refused as [`GateRefusal::NotReady`].
///
/// # Errors
///
/// Returns [`GateRefusal::FloorDrop`] when the flag is set, and
/// [`GateRefusal::NotReady`] when the pair is not armed.
pub const fn gate(state: PairState, floor_drop: bool) -> Result<(), GateRefusal> {
    if floor_drop {
        return Err(GateRefusal::FloorDrop);
    }
    if matches!(state, PairState::Armed) {
        Ok(())
    } else {
        Err(GateRefusal::NotReady)
    }
}
