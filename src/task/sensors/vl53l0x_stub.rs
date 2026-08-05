//! VL53L0X rangefinder stub task — emits synthetic distance readings on a timer.
//!
//! This is a **stub for the VL53L0X Time-of-Flight sensor array**. It produces
//! fixed distance readings to exercise the perception, obstacle-avoidance, and
//! event pipelines without requiring physical hardware.
//!
//! # Architecture
//!
//! - Runs on **core0** as an embassy task.
//! - Signal channel (`VL53L0X_SIGNAL_CHANNEL`) for timestamped boolean obstacle
//!   signals per sensor.
//! - Edge-triggered obstacle detection: tracks `last_obstacle_detected` per

#![allow(clippy::missing_docs_in_private_items)]
//!   sensor, only raises `ObstacleDetected { source: Rangefinder }` on state
//!   change for any sensor.
//! - Writes readings to `perception::update_rangefinder_readings()` each cycle.
//! - Obstacle state written to `perception::set_rangefinder_obstacle()` on
//!   any state change.
//!
//! # Sensor positions
//!
//! | Sensor        | Position            | Purpose                       |
//! |---------------|---------------------|-------------------------------|
//! | `FrontLeft`   | Front-left corner   | Side collision detection      |
//! | `FrontCenter` | Front center        | Forward collision detection   |
//! | `FrontDown`   | Front, angled down  | Stair / drop-off detection    |
//! | `Rear`        | Rear center         | Rear collision detection      |
//!
//! # Default readings
//!
//! All four sensors report 200 cm clear by default, 500 ms interval.

#![allow(dead_code)]

use defmt::info;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};

use crate::system::{
    event::{Events, ObstacleSource, raise_event},
    state::perception::{self, RangefinderReadings},
};

// ── Public types ──────────────────────────────────────────────────────────────

/// Which VL53L0X sensor reported a state change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Vl53l0xSensorId {
    /// Front-left corner sensor (side collision detection).
    FrontLeft,
    /// Front-center sensor (forward collision detection).
    FrontCenter,
    /// Front-down sensor (stair/drop detection, angled downward).
    FrontDown,
    /// Rear-center sensor (rear collision detection).
    Rear,
}

// ── Constants ─────────────────────────────────────────────────────────────────

/// Debounce delay to filter out noise.
const DEBOUNCE_DELAY: Duration = Duration::from_millis(100);

/// Obstacle detection threshold in cm.
const OBSTACLE_THRESHOLD_CM: f32 = 20.0;

/// Scan interval in milliseconds.
const SCAN_INTERVAL_MS: u64 = 500;

// ── Signal channel ────────────────────────────────────────────────────────────

/// Signal channel for obstacle state changes per sensor.
///
/// Each message is a `(Vl53l0xSensorId, bool)` tuple — which sensor and whether
/// an obstacle is detected (`true`) or cleared (`false`).
static VL53L0X_SIGNAL_CHANNEL: Channel<CriticalSectionRawMutex, (Vl53l0xSensorId, bool), 16> = Channel::new();

// ── Public API ────────────────────────────────────────────────────────────────

/// Signal an obstacle state change from the VL53L0X sensor array.
pub async fn signal_vl53l0x_obstacle(sensor: Vl53l0xSensorId, state: bool) {
    VL53L0X_SIGNAL_CHANNEL.sender().send((sensor, state)).await;
}

// ── Internal state ────────────────────────────────────────────────────────────

/// Per-sensor obstacle tracking for edge-triggered detection.
struct SensorState {
    last_detected: Option<bool>,
    reading_cm: f32,
}

impl SensorState {
    const fn new(reading_cm: f32) -> Self {
        Self {
            last_detected: None,
            reading_cm,
        }
    }
}

/// Internal mutable state for the stub task.
struct StubState {
    front_left: SensorState,
    front_center: SensorState,
    front_down: SensorState,
    rear: SensorState,
}

impl StubState {
    const fn new() -> Self {
        Self {
            front_left: SensorState::new(200.0),
            front_center: SensorState::new(200.0),
            front_down: SensorState::new(200.0),
            rear: SensorState::new(200.0),
        }
    }

    const fn get_mut(&mut self, sensor: Vl53l0xSensorId) -> &mut SensorState {
        match sensor {
            Vl53l0xSensorId::FrontLeft => &mut self.front_left,
            Vl53l0xSensorId::FrontCenter => &mut self.front_center,
            Vl53l0xSensorId::FrontDown => &mut self.front_down,
            Vl53l0xSensorId::Rear => &mut self.rear,
        }
    }

    const fn get(&self, sensor: Vl53l0xSensorId) -> &SensorState {
        match sensor {
            Vl53l0xSensorId::FrontLeft => &self.front_left,
            Vl53l0xSensorId::FrontCenter => &self.front_center,
            Vl53l0xSensorId::FrontDown => &self.front_down,
            Vl53l0xSensorId::Rear => &self.rear,
        }
    }

    /// Returns true if any sensor currently detects an obstacle.
    fn any_obstacle(&self) -> bool {
        self.front_left.last_detected == Some(true)
            || self.front_center.last_detected == Some(true)
            || self.front_down.last_detected == Some(true)
            || self.rear.last_detected == Some(true)
    }

    /// Build `RangefinderReadings` from current sensor values.
    const fn to_readings(&self) -> RangefinderReadings {
        RangefinderReadings {
            front_left: Some(self.front_left.reading_cm),
            front_center: Some(self.front_center.reading_cm),
            front_down: Some(self.front_down.reading_cm),
            rear: Some(self.rear.reading_cm),
        }
    }
}

// ── Embassy task ──────────────────────────────────────────────────────────────

/// VL53L0X rangefinder stub embassy task — generates periodic readings and
/// runs the obstacle detection loop.
///
/// The signal channel receives `(Vl53l0xSensorId, bool)` tuples.
/// - Edge-triggered: only raises `ObstacleDetected { source: Rangefinder }`
///   when obstacle state changes for any sensor.
/// - A timer-driven loop emits fixed distance readings to perception.
///
/// Runs on **core0**.
#[embassy_executor::task]
pub async fn vl53l0x_stub_task() {
    info!("[vl53l0x_stub] booted on core0");

    let mut state = StubState::new();
    // Combined rangefinder obstacle flag for edge-triggered events.
    let mut last_combined_obstacle: Option<bool> = None;

    loop {
        // Process any pending obstacle signals (non-blocking drain).
        while let Ok((sensor, obstacle_detected)) = VL53L0X_SIGNAL_CHANNEL.receiver().try_receive() {
            let s = state.get_mut(sensor);
            if s.last_detected != Some(obstacle_detected) {
                info!("[vl53l0x_stub] {:?} obstacle: {}", sensor, obstacle_detected);
                s.last_detected = Some(obstacle_detected);
            }
        }

        // Debounce: wait before publishing.
        Timer::after(DEBOUNCE_DELAY).await;

        // Publish rangefinder readings to perception.
        let readings = state.to_readings();
        perception::update_rangefinder_readings(readings).await;

        // Edge-triggered combined obstacle detection.
        let combined = state.any_obstacle();
        if last_combined_obstacle != Some(combined) {
            raise_event(Events::ObstacleDetected {
                source: ObstacleSource::Rangefinder,
                detected: combined,
            })
            .await;
            perception::set_rangefinder_obstacle(combined).await;
            last_combined_obstacle = Some(combined);
        }

        #[cfg(feature = "telemetry_logs")]
        info!(
            "[vl53l0x_stub] readings: FL={} FC={} FD={} R={} (obstacle: {})",
            state.front_left.reading_cm,
            state.front_center.reading_cm,
            state.front_down.reading_cm,
            state.rear.reading_cm,
            combined
        );

        // Wait for the next scan interval.
        Timer::after(Duration::from_millis(SCAN_INTERVAL_MS)).await;
    }
}
