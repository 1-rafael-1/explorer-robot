//! `LiDAR` stub task — emits synthetic 360° point clouds on a timer.
//!
//! This is a **stub for the COIN-D6 `LiDAR`**, matching the ultrasonic sensor's
//! event contract from v2. It produces fixed distance data to exercise the
//! perception, obstacle-avoidance, and event pipelines without requiring
//! physical hardware.
//!
//! # Architecture
//!
//! - Runs on **core1** as an embassy task.
//! - Command channel (`LIDAR_STUB_CONTROL`) for mode control (like v2's
//!   `US_SWEEP_CONTROL`).
//! - Edge-triggered obstacle detection: tracks `last_obstacle_detected` and
//!   only raises `ObstacleDetected { source: Lidar }` on state change.
//! - Writes point cloud to `perception::update_lidar_points()` each cycle.
//! - Raises `LidarScanCompleted` event when each scan finishes (like v2's
//!   `UltrasonicSweepCompleted`).
//!
//! # Default pattern
//!
//! 360° scan with 200 cm clear in all directions, walls at ±90° at 50 cm.

#![allow(dead_code)]

use defmt::info;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};

use crate::system::{
    event::{Events, ObstacleSource, raise_event},
    state::perception::{self, LidarPointCloud},
};

// ── Public types ──────────────────────────────────────────────────────────────

/// Commands for `LiDAR` stub mode control (matches v2 ultrasonic command pattern).
#[derive(Debug, Clone, Copy)]
pub enum LidarStubCommand {
    /// Start continuous 360° scanning.
    StartScanning,
    /// Stop scanning and go idle.
    Stop,
}

// ── Command channel ───────────────────────────────────────────────────────────

/// Control channel for `LiDAR` stub commands (capacity 4).
static LIDAR_STUB_CONTROL: Channel<CriticalSectionRawMutex, LidarStubCommand, 4> = Channel::new();

// ── Public API ────────────────────────────────────────────────────────────────

/// Start continuous `LiDAR` scanning.
pub async fn start_lidar_scanning() {
    LIDAR_STUB_CONTROL.send(LidarStubCommand::StartScanning).await;
}

/// Stop `LiDAR` scanning.
pub async fn stop_lidar() {
    LIDAR_STUB_CONTROL.send(LidarStubCommand::Stop).await;
}

// ── Pattern generation ────────────────────────────────────────────────────────

/// Generate a default 360° point cloud:
/// - 200 cm clear in all directions
/// - 50 cm walls at ±90° (left/right)
const fn generate_default_cloud(sequence: u64) -> LidarPointCloud {
    let mut distances = [200.0_f32; 360];
    // Walls at ±90°
    distances[90] = 50.0;
    distances[270] = 50.0;

    LidarPointCloud { distances, sequence }
}

/// Obstacle detection threshold in cm for the forward cone.
const OBSTACLE_THRESHOLD_CM: f32 = 20.0;
/// Forward cone width in degrees (±45°).
const CONE_WIDTH_DEG: u16 = 90;

/// Check whether the generated cloud has an obstacle in the forward cone.
fn cloud_has_obstacle(cloud: &LidarPointCloud) -> bool {
    cloud.is_obstacle_ahead(OBSTACLE_THRESHOLD_CM, CONE_WIDTH_DEG)
}

// ── Embassy task ──────────────────────────────────────────────────────────────

/// `LiDAR` stub embassy task — command loop + scanning loop.
///
/// Pattern matches v2's `ultrasonic_sweep` task: an outer command loop waits
/// for `StartScanning`/`Stop`, and an inner loop generates point clouds on
/// a timer, raising `LidarScanCompleted` each cycle.
///
/// Runs on **core1**.
#[embassy_executor::task]
pub async fn lidar_stub_task() {
    info!("[lidar_stub] booted on core1");

    let mut scanning = false;
    let mut sequence: u64 = 0;
    let mut last_obstacle_detected: Option<bool> = None;

    loop {
        info!("[lidar_stub] waiting for command (scanning={})", scanning);

        // Wait for a command (blocking — task idles until commanded).
        match LIDAR_STUB_CONTROL.receive().await {
            LidarStubCommand::StartScanning => {
                info!("[lidar_stub] starting continuous scan");
                // Reset obstacle tracking on mode change.
                if last_obstacle_detected == Some(true) {
                    raise_event(Events::ObstacleDetected {
                        source: ObstacleSource::Lidar,
                        detected: false,
                    })
                    .await;
                    perception::set_lidar_obstacle(false).await;
                }
                last_obstacle_detected = None;
                scanning = true;
            }
            LidarStubCommand::Stop => {
                info!("[lidar_stub] stopping scan");
                // Clear obstacle state on stop.
                if last_obstacle_detected == Some(true) {
                    raise_event(Events::ObstacleDetected {
                        source: ObstacleSource::Lidar,
                        detected: false,
                    })
                    .await;
                    perception::set_lidar_obstacle(false).await;
                }
                last_obstacle_detected = None;
                scanning = false;
            }
        }

        // Inner scanning loop: generate point clouds on a timer.
        while scanning {
            // Check for incoming commands without blocking the scan cycle.
            if let Ok(cmd) = LIDAR_STUB_CONTROL.receiver().try_receive() {
                match cmd {
                    LidarStubCommand::Stop => {
                        info!("[lidar_stub] stop received during scan");
                        if last_obstacle_detected == Some(true) {
                            raise_event(Events::ObstacleDetected {
                                source: ObstacleSource::Lidar,
                                detected: false,
                            })
                            .await;
                            perception::set_lidar_obstacle(false).await;
                        }
                        last_obstacle_detected = None;
                        scanning = false;
                        break;
                    }
                    LidarStubCommand::StartScanning => {
                        // Already scanning — restart obstacle tracking.
                        if last_obstacle_detected == Some(true) {
                            raise_event(Events::ObstacleDetected {
                                source: ObstacleSource::Lidar,
                                detected: false,
                            })
                            .await;
                            perception::set_lidar_obstacle(false).await;
                        }
                        last_obstacle_detected = None;
                    }
                }
            }

            // Generate and publish the point cloud.
            sequence = sequence.wrapping_add(1);
            let cloud = generate_default_cloud(sequence);

            perception::update_lidar_points(cloud.clone()).await;

            // Edge-triggered obstacle detection.
            let detected = cloud_has_obstacle(&cloud);
            if last_obstacle_detected != Some(detected) {
                raise_event(Events::ObstacleDetected {
                    source: ObstacleSource::Lidar,
                    detected,
                })
                .await;
                perception::set_lidar_obstacle(detected).await;
                last_obstacle_detected = Some(detected);
            }

            // Raise scan-completed event (like v2's UltrasonicSweepCompleted).
            raise_event(Events::LidarScanCompleted).await;

            #[cfg(feature = "telemetry_logs")]
            info!("[lidar_stub] scan {} completed (obstacle: {})", sequence, detected);

            // Scan interval: 200 ms.
            Timer::after(Duration::from_millis(200)).await;
        }
    }
}
