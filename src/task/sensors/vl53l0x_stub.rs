//! VL53L0X rangefinder stub task — emits synthetic distance readings on a timer.
//!
//! This is a **stub for the VL53L0X Time-of-Flight sensor**. It produces
//! fixed distance readings to exercise the perception, floor-drop-detection, and
//! event pipelines without requiring physical hardware.
//!
//! # Architecture
//!
//! - Runs on **core0** as an embassy task.
//! - Single front-down sensor (angled downward for stair/drop-off detection).
//!   The 360° COIN-D6 `LiDAR` covers forward, lateral, and rear arcs, so only
//!   the downward-facing sensor is retained — it sees what the planar `LiDAR`
//!   cannot.
//! - Edge-triggered floor-drop detection: tracks `last_drop_detected` and
//!   only raises `FloorDropDetected { detected }` on state change.
//! - Writes readings to `perception::update_rangefinder_readings()` each cycle.
//! - Floor-drop state written to `perception::set_floor_drop()` on
//!   any state change.
//!
//! # Default reading
//!
//! Alternates between 15 cm (solid floor) and 200 cm (floor drop)
//! every 500 ms to exercise the full floor-drop detection pipeline.

#![allow(dead_code)]

use defmt::info;
use embassy_time::{Duration, Timer};

use crate::system::{
    event::{Events, raise_event},
    state::perception::{self, RangefinderReadings},
};

// ── Constants ─────────────────────────────────────────────────────────────────

/// Floor-drop detection threshold in cm. Readings above this indicate the floor
/// has dropped away (e.g. top of stairs, ledge).
const FLOOR_DROP_THRESHOLD_CM: f32 = 40.0;

/// Scan interval in milliseconds.
const SCAN_INTERVAL_MS: u64 = 500;

// ── Embassy task ──────────────────────────────────────────────────────────────

/// VL53L0X rangefinder stub embassy task — generates periodic readings and
/// runs the floor-drop detection loop.
///
/// Alternates between a solid-floor reading (15 cm) and a floor-drop reading
/// (200 cm) each cycle to exercise both detection and clear transitions.
///
/// Runs on **core0**.
#[embassy_executor::task]
pub async fn vl53l0x_stub_task() {
    info!("[vl53l0x_stub] booted on core0");

    let mut toggle: bool = false;
    let mut last_detected: Option<bool> = None;

    loop {
        // Alternate between solid floor (15 cm) and floor drop (200 cm).
        let reading_cm: f32 = if toggle { 15.0 } else { 200.0 };
        toggle = !toggle;

        let detected = reading_cm > FLOOR_DROP_THRESHOLD_CM;

        // Publish rangefinder reading to perception.
        let readings = RangefinderReadings {
            front_down: Some(reading_cm),
        };
        perception::update_rangefinder_readings(readings).await;

        // Edge-triggered floor-drop detection.
        if last_detected != Some(detected) {
            raise_event(Events::FloorDropDetected { detected }).await;
            perception::set_floor_drop(detected).await;
            last_detected = Some(detected);
        }

        #[cfg(feature = "telemetry_logs")]
        info!(
            "[vl53l0x_stub] reading: {:.1} cm (floor_drop: {})",
            reading_cm, detected
        );

        // Wait for the next scan interval.
        Timer::after(Duration::from_millis(SCAN_INTERVAL_MS)).await;
    }
}
