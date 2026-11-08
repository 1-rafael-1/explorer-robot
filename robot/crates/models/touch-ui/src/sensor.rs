//! The sensor's lifecycle state, as the UI names it.
//!
//! A neutral type the firmware maps its own `LiDAR` lifecycle onto, so the touch
//! UI never depends on firmware state. The four states are distinguishable by
//! their [`SensorState::label`] and are the caption the Room Scan screen draws.

/// A sensor's lifecycle state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorState {
    /// Powered down; no mode needs the sensor.
    Off,
    /// Powered on and waiting for the rotor to settle.
    Warming,
    /// Warmed up and producing scans.
    Streaming,
    /// A lifecycle attempt failed; the sensor stays powered down.
    Failed,
}

impl SensorState {
    /// The short caption that names this state on screen.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Off => "LiDAR off",
            Self::Warming => "LiDAR warming",
            Self::Streaming => "LiDAR streaming",
            Self::Failed => "LiDAR failed",
        }
    }
}
