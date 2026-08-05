//! Perception state module.
//!
//! Owns `LiDAR` point cloud and VL53L0X rangefinder readings behind a single
//! interface. Obstacle detection flags are exposed via lock-free atomics
//! (fast path) while full sensor data sits behind a mutex (rich path).
//!
//! Obstacle perception is driven by a 360° `LiDAR` point cloud. The front-down
//! VL53L0X rangefinder is a floor-drop detector (stairs, ledges) — it is **not**
//! an obstacle sensor.
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) power state mutex (use power module accessors)
//! 2) `CALIBRATION_STATE`
//! 3) perception mutex (private — use accessor functions)
//! 4) `MOTION_STATE`

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

// ── LiDAR point cloud ────────────────────────────────────────────────────────

/// A full 360° `LiDAR` scan, one distance per degree.
///
/// Index 0 = 0° (straight ahead), index 90 = 90° (left), etc.
/// A distance of `0.0` means no valid return at that angle (out of range,
/// reflective surface, or measurement error).
#[derive(Debug, Clone)]
pub struct LidarPointCloud {
    /// Distance in cm per degree. `0.0` = no return.
    pub distances: [f32; 360],
    /// Monotonically increasing scan sequence number.
    #[allow(dead_code)]
    pub sequence: u64,
}

impl LidarPointCloud {
    /// Check whether an obstacle lies within a forward cone.
    ///
    /// The cone is centered on 0° (forward) and extends `cone_width_deg/2`
    /// degrees to either side. Only non-zero distances are considered.
    ///
    /// # Arguments
    /// * `threshold_cm` — distances ≤ this value count as obstacles.
    /// * `cone_width_deg` — total cone width in degrees (e.g. 60 = ±30°).
    pub fn is_obstacle_ahead(&self, threshold_cm: f32, cone_width_deg: u16) -> bool {
        let half = cone_width_deg as usize / 2;

        // Right side of the cone: 0° .. half°
        for i in 0..=half {
            let d = self.distances[i];
            if d > 0.0 && d <= threshold_cm {
                return true;
            }
        }

        // Left side of the cone: (360 - half)° .. 359°
        let start = 360usize.saturating_sub(half);
        for i in start..360 {
            let d = self.distances[i];
            if d > 0.0 && d <= threshold_cm {
                return true;
            }
        }

        false
    }
}

// ── Rangefinder readings ─────────────────────────────────────────────────────

/// VL53L0X rangefinder readings from all four sensors.
///
/// Each field is an `Option<f32>` — `None` means no valid reading (e.g. sensor
/// not yet initialized, out of range, or measurement error).
#[derive(Debug, Clone, Copy, Default)]
pub struct RangefinderReadings {
    /// Front-down distance in cm (stair/drop detection, angled downward).
    pub front_down: Option<f32>,
}

impl RangefinderReadings {
    /// Returns `true` if the reading is valid and above the threshold,
    /// indicating that the floor has dropped away (e.g. top of stairs,
    /// ledge). A normal flat floor returns a consistent short distance;
    /// a floor drop causes the distance to jump beyond range.
    #[allow(dead_code)]
    pub fn is_floor_drop(&self, threshold_cm: f32) -> bool {
        self.front_down.is_some_and(|d| d > threshold_cm)
    }
}

// ── Atomics (lock-free fast path for reads) ────────────────────────────────────

/// `LiDAR` obstacle detection flag — updated by `set_lidar_obstacle`, read lock-free.
static LIDAR_OBSTACLE: AtomicBool = AtomicBool::new(false);
/// Floor-drop detection flag — updated by `set_floor_drop`, read lock-free.
static FLOOR_DROP: AtomicBool = AtomicBool::new(false);

// ── Mutex (full picture: point cloud, rangefinder readings, flags) ─────────────

/// Mutex-guarded state holding `LiDAR` and rangefinder data with flags.
static PERCEPTION_STATE: Mutex<CriticalSectionRawMutex, PerceptionState> = Mutex::new(PerceptionState {
    lidar: None,
    rangefinder: None,
    lidar_obstacle: false,
    floor_drop: false,
});

/// Internal state behind the mutex — holds both `LiDAR` and rangefinder data.
struct PerceptionState {
    /// Latest `LiDAR` point cloud, if available.
    lidar: Option<LidarPointCloud>,
    /// Latest rangefinder readings, if available.
    rangefinder: Option<RangefinderReadings>,
    /// `LiDAR` obstacle flag snapshot in the mutex.
    lidar_obstacle: bool,
    /// Floor-drop flag snapshot in the mutex.
    floor_drop: bool,
}

// ── Change detection ──────────────────────────────────────────────────────────

/// Outcome of a setter that may change the obstacle flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChangeDetected {
    /// The obstacle flag did not change.
    NoChange,
    /// The obstacle flag transitioned from false → true.
    ChangedToDetected,
    /// The obstacle flag transitioned from true → false.
    ChangedToCleared,
}

// ── Public accessors (lock-free) ──────────────────────────────────────────────

/// Set the `LiDAR` obstacle flag.
///
/// Also stores the flag in the mutex for rich-path consumers.
/// Returns whether the obstacle state changed.
pub async fn set_lidar_obstacle(detected: bool) -> ChangeDetected {
    let old = LIDAR_OBSTACLE.load(Ordering::Relaxed);
    LIDAR_OBSTACLE.store(detected, Ordering::Relaxed);

    let mut state = PERCEPTION_STATE.lock().await;
    state.lidar_obstacle = detected;
    drop(state);

    change_detected(old, detected)
}

/// Set the floor-drop flag.
///
/// Also stores the flag in the mutex for rich-path consumers.
pub async fn set_floor_drop(detected: bool) {
    FLOOR_DROP.store(detected, Ordering::Relaxed);

    let mut state = PERCEPTION_STATE.lock().await;
    state.floor_drop = detected;
}

/// Returns `true` if the `LiDAR` sensor reports an obstacle.
/// Lock-free — safe to call from any context.
pub fn is_obstacle_detected() -> bool {
    LIDAR_OBSTACLE.load(Ordering::Relaxed)
}

/// Returns `true` if the `LiDAR` sensor reports an obstacle.
/// Lock-free — safe to call from any context.
#[allow(dead_code)]
pub fn is_lidar_obstacle_detected() -> bool {
    LIDAR_OBSTACLE.load(Ordering::Relaxed)
}

/// Returns `true` if a floor drop is currently detected.
/// Lock-free — safe to call from any context.
#[allow(dead_code)]
pub fn is_floor_drop_detected() -> bool {
    FLOOR_DROP.load(Ordering::Relaxed)
}

/// Async read of the `LiDAR` obstacle flag from the mutex.
pub async fn lidar_obstacle() -> bool {
    PERCEPTION_STATE.lock().await.lidar_obstacle
}

/// Async read of the floor-drop flag from the mutex.
#[allow(dead_code)]
pub async fn floor_drop() -> bool {
    PERCEPTION_STATE.lock().await.floor_drop
}

// ── Public accessors (async — touch the mutex) ────────────────────────────────

/// Replace the stored `LiDAR` point cloud with a new scan.
pub async fn update_lidar_points(cloud: LidarPointCloud) {
    let mut state = PERCEPTION_STATE.lock().await;
    state.lidar = Some(cloud);
}

/// Replace the stored rangefinder readings.
pub async fn update_rangefinder_readings(readings: RangefinderReadings) {
    let mut state = PERCEPTION_STATE.lock().await;
    state.rangefinder = Some(readings);
}

/// Return a clone of the current `LiDAR` point cloud, if available.
pub async fn get_lidar_snapshot() -> Option<LidarPointCloud> {
    PERCEPTION_STATE.lock().await.lidar.clone()
}

/// Return a copy of the current rangefinder readings, if available.
#[allow(dead_code)]
pub async fn get_rangefinder_snapshot() -> Option<RangefinderReadings> {
    PERCEPTION_STATE.lock().await.rangefinder
}

/// Reset all obstacle flags, floor-drop flag, and sensor data to defaults.
#[allow(dead_code)]
pub async fn reset_all() {
    LIDAR_OBSTACLE.store(false, Ordering::Relaxed);
    FLOOR_DROP.store(false, Ordering::Relaxed);

    let mut state = PERCEPTION_STATE.lock().await;
    state.lidar = None;
    state.rangefinder = None;
    state.lidar_obstacle = false;
    state.floor_drop = false;
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Map (old, new) obstacle flag states to a `ChangeDetected` variant.
const fn change_detected(old: bool, new: bool) -> ChangeDetected {
    match (old, new) {
        (false, true) => ChangeDetected::ChangedToDetected,
        (true, false) => ChangeDetected::ChangedToCleared,
        _ => ChangeDetected::NoChange,
    }
}
