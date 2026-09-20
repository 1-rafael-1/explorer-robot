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
//! Progress text does not travel on this bus: producers publish their phase,
//! percent and result through [`crate::system::state::activity`] and the UI
//! renders it. The bus carries transitions only.
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
//! raise_event(Events::TestingCompleted).await;
//!
//! // Receiving an event (in orchestrator)
//! let event = wait().await;
//! ```

use defmt::Format;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

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

    /// The obstacle flag changed.
    ///
    /// This is an edge, not a state copy: the payload names which sensor
    /// changed the flag, and a handler reads the flag for the current state
    /// rather than trusting a value carried here. Raised by the `LiDAR` task
    /// when its Front Sector test flips the flag.
    ObstacleDetected {
        /// Sensor source reporting the change.
        source: ObstacleSource,
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

    /// Testing sequence finished.
    TestingCompleted,

    /// Calibration procedure finished (success or failure).
    ///
    /// The result itself is not carried here: the procedure records its phase,
    /// percent and outcome in [`crate::system::state::activity`], and the UI
    /// reads that to decide what to show.
    CalibrationCompleted,
}
