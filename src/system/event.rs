//! Event System
//!
//! Provides a centralized event handling system for inter-task communication.
//! Uses an async channel to coordinate events between different parts of the system.
//!
//! # Event Flow
//! 1. Tasks generate events (e.g., sensor readings, button presses)
//! 2. Events are sent through the channel
//! 3. The orchestrator task processes events and updates system state
//! 4. State changes trigger corresponding actions in other tasks
//!
//! # Channel Design
//! - Multi-producer: Any task can send events
//! - Single-consumer: Orchestrator task processes all events
//! - Bounded capacity: 64 events maximum to prevent memory exhaustion
//! - Async operation: Non-blocking event handling
//!
//! # Usage Example
//! ```rust
//! // Sending an event
//! raise_event(Events::RotaryButtonPressed).await;
//!
//! // Receiving an event (in orchestrator)
//! let event = wait().await;
//! ```

use defmt::Format;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

use crate::task::io::display::MAX_LINE_LEN;

/// Multi-producer, single-consumer event channel.
///
/// Capacity of 64 events provides headroom for multiple producers raising
/// events concurrently (e.g. button presses arriving while calibration
/// status updates are still in flight). The orchestrator drains the channel
/// sequentially, so the queue only grows during transient bursts.
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, Events, 64> = Channel::new();

/// Sends an event to the system channel.
///
/// Events are queued if channel is full. If multiple events
/// occur simultaneously, they are processed in order of arrival.
pub async fn raise_event(event: Events) {
    EVENT_CHANNEL.sender().send(event).await;
}

/// Receives the next event from the system channel.
///
/// Called by the orchestrator task to process events sequentially.
/// Waits asynchronously if no events are available.
pub async fn wait() -> Events {
    EVENT_CHANNEL.receiver().receive().await
}

// ── Supporting types ────────────────────────────────────────────────────────────

/// Source of obstacle detection events.
#[derive(Debug, Clone, Copy, Format, PartialEq, Eq)]
pub enum ObstacleSource {
    /// Obstacle detected by spinning `LiDAR` (forward arc).
    Lidar,
}

/// Rotary encoder direction.
#[derive(Debug, Clone, Copy, Format, Eq, PartialEq)]
pub enum RotaryDirection {
    /// Encoder turned clockwise.
    Clockwise,
    /// Encoder turned counter-clockwise.
    CounterClockwise,
}

// ── System events ───────────────────────────────────────────────────────────────

/// System-wide events that can occur during robot operation.
#[derive(Debug, Clone)]
pub enum Events {
    /// System initialization requested.
    /// - Triggered at startup or after reset.
    /// - Coordinates initial setup across tasks.
    Initialize,

    /// Calibration data loaded from flash storage.
    /// - Triggered when flash storage completes reading calibration data.
    /// - Carries Option: Some(data) if found, None if not found in flash.
    /// - Allows orchestrator to update system state accordingly.
    CalibrationDataLoaded(
        crate::task::io::flash_storage::CalibrationKind,
        Option<crate::task::io::flash_storage::CalibrationDataKind>,
    ),

    /// Obstacle detection status changed.
    /// - source: which sensor reported the change.
    /// - detected: true if obstacle within threshold, false if clear.
    ObstacleDetected {
        /// Sensor source reporting the change.
        source: ObstacleSource,
        /// true: Obstacle detected within threshold.
        /// false: Path is clear.
        detected: bool,
    },

    /// Floor-drop detection status changed (stairs, ledges).
    ///
    /// Raised by the front-down VL53L0X rangefinder when the distance
    /// reading crosses the drop threshold. This is *not* an obstacle —
    /// the floor opening up (e.g. top of stairs) looks like a sudden
    /// distance increase beyond range.
    ///
    /// In `CoastAndAvoid` mode a floor drop triggers the same
    /// stop/back-up/turn response as an obstacle. In other drive modes
    /// it is logged but does not interrupt motion.
    FloorDropDetected {
        /// true: Floor drop detected (distance exceeds threshold).
        /// false: Floor is solid again.
        detected: bool,
    },

    /// Obstacle avoidance maneuver completed.
    /// - Triggered after attempting to navigate around obstacle.
    /// - Used to coordinate next movement decision.
    ObstacleAvoidanceAttempted,

    /// `LiDAR` buffered scan completed (360° point-cloud pass).
    /// The point cloud in perception state is now fully populated.
    LidarScanCompleted,

    /// Battery measurement (level percentage and raw voltage).
    /// - level: 0-100 percent, triggers LED color updates.
    /// - voltage: raw voltage in volts, used for motor driver voltage compensation.
    /// - Single event reduces event channel load.
    BatteryMeasured {
        /// Battery charge level (0-100%).
        level: u8,
        /// Battery voltage in volts.
        voltage: f32,
    },

    /// Rotary encoder turn.
    /// - Clockwise = increment, `CounterClockwise` = decrement.
    RotaryTurned(RotaryDirection),

    /// Rotary encoder button press.
    /// - Short press.
    RotaryButtonPressed,

    /// Rotary encoder button hold initiated.
    RotaryButtonHoldStart,

    /// Rotary encoder button hold released.
    RotaryButtonHoldEnd,

    /// Testing sequence finished.
    TestingCompleted,

    /// Calibration status update.
    /// - Triggered during calibration procedures to update display.
    /// - Contains optional header (line 0) and up to 3 status lines (lines 1-3).
    CalibrationStatus {
        /// Optional header text (line 0).
        header: Option<heapless::String<MAX_LINE_LEN>>,
        /// Optional status line 1.
        line1: Option<heapless::String<MAX_LINE_LEN>>,
        /// Optional status line 2.
        line2: Option<heapless::String<MAX_LINE_LEN>>,
        /// Optional status line 3.
        line3: Option<heapless::String<MAX_LINE_LEN>>,
    },

    /// Calibration procedure finished (success or failure).
    CalibrationCompleted,
}
