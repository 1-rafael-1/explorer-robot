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
//! raise_event(Events::Initialize).await;
//!
//! // Receiving an event (in orchestrator)
//! let event = wait().await;
//! ```

use defmt::Format;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use touch_ui::Procedure;

use crate::system::state::activity::Stage;

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
    /// This is an edge, not a periodic state copy: it is raised on both the
    /// detection and the clear transition. The payload names which sensor
    /// changed the flag and carries the value the producer measured, so the
    /// Link Task can translate it without reading the live flag (ADR-0031). A
    /// handler still reads the flag for the current state when the safety
    /// decision must not trust a queued edge. Raised by the `LiDAR` task when
    /// its Front Sector test flips the flag and when a disable clears it.
    ObstacleDetected {
        /// Sensor source reporting the change.
        source: ObstacleSource,
        /// The flag's new value: `true` on detection, `false` on the clear edge.
        detected: bool,
    },

    /// Floor-drop detection status changed (stairs, ledges).
    ///
    /// Raised by the front/rear VL53L0X rangefinders when the distance
    /// reading crosses the drop threshold. The event currently carries a
    /// single flag and does not distinguish which sensor raised it. This is
    /// *not* an obstacle — the floor opening up (e.g. top of stairs) looks
    /// like a sudden distance increase beyond range.
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

    /// Battery measurement (level percentage and pack voltage).
    /// - level: 0-100 percent, triggers LED color updates.
    /// - voltage: pack-terminal voltage in volts, for the panel and Link telemetry.
    ///   The motor driver reads the separately stored rail voltage.
    /// - Single event reduces event channel load.
    BatteryMeasured {
        /// Battery charge level (0-100%).
        level: u8,
        /// Pack-terminal voltage in volts.
        voltage: f32,
    },

    /// A Procedure reached a terminal outcome.
    ///
    /// Raised by the Active Claim that owns the run for both success and
    /// failure, carrying the identity, stage and detail directly: reading
    /// Activity State after the claim has been released would race with the
    /// next run (ADR-0032). A normal stop raises nothing. The UI still reads
    /// Activity for the landing screen, and the Link Task translates this event
    /// onto the wire.
    ProcedureFinished {
        /// The Procedure that finished.
        procedure: Procedure,
        /// Whether it completed or failed.
        stage: Stage,
        /// The result line recorded for the run.
        detail: &'static str,
        /// The claim generation that owned the run.
        ///
        /// A terminal event is raised after the claim releases the slot, so a
        /// later run can start before the UI handles it. The generation lets the
        /// panel dismiss only the run it actually finished.
        generation: u32,
    },
}
