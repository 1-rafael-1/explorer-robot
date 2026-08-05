//! Perception state module.
//!
//! Owns `LiDAR` point cloud and VL53L0X rangefinder readings behind a single
//! interface. Obstacle detection flags are exposed via lock-free atomics
//! (fast path) while full sensor data sits behind a mutex (rich path).
//!
//! Obstacle perception is driven by a 360° `LiDAR` point cloud and four
//! VL53L0X rangefinder readings.
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
    /// Front-left distance in cm (side collision detection).
    #[allow(dead_code)]
    pub front_left: Option<f32>,
    /// Front-center distance in cm (forward collision detection).
    #[allow(dead_code)]
    pub front_center: Option<f32>,
    /// Front-down distance in cm (stair/drop detection, angled downward).
    #[allow(dead_code)]
    pub front_down: Option<f32>,
    /// Rear distance in cm (rear collision detection).
    #[allow(dead_code)]
    pub rear: Option<f32>,
}

impl RangefinderReadings {
    /// Returns `true` if any valid reading is at or below the threshold.
    #[allow(dead_code)]
    pub fn any_below(&self, threshold_cm: f32) -> bool {
        self.front_left.is_some_and(|d| d <= threshold_cm)
            || self.front_center.is_some_and(|d| d <= threshold_cm)
            || self.front_down.is_some_and(|d| d <= threshold_cm)
            || self.rear.is_some_and(|d| d <= threshold_cm)
    }
}

// ── Atomics (lock-free fast path for reads) ────────────────────────────────────

/// `LiDAR` obstacle detection flag — updated by `set_lidar_obstacle`, read lock-free.
static LIDAR_OBSTACLE: AtomicBool = AtomicBool::new(false);
/// Rangefinder obstacle detection flag — updated by `set_rangefinder_obstacle`, read lock-free.
static RANGEFINDER_OBSTACLE: AtomicBool = AtomicBool::new(false);
/// Combined obstacle flag — `LIDAR_OBSTACLE || RANGEFINDER_OBSTACLE`.
/// Recomputed atomically by every mutating accessor, read lock-free.
static COMBINED_OBSTACLE: AtomicBool = AtomicBool::new(false);

// ── Mutex (full picture: point cloud, rangefinder readings, flags) ─────────────

/// Mutex-guarded state holding `LiDAR` and rangefinder data with obstacle flags.
static PERCEPTION_STATE: Mutex<CriticalSectionRawMutex, PerceptionState> = Mutex::new(PerceptionState {
    lidar: None,
    rangefinder: None,
    lidar_obstacle: false,
    rangefinder_obstacle: false,
});

/// Internal state behind the mutex — holds both `LiDAR` and rangefinder data.
struct PerceptionState {
    /// Latest `LiDAR` point cloud, if available.
    lidar: Option<LidarPointCloud>,
    /// Latest rangefinder readings, if available.
    rangefinder: Option<RangefinderReadings>,
    /// `LiDAR` obstacle flag snapshot in the mutex.
    lidar_obstacle: bool,
    /// Rangefinder obstacle flag snapshot in the mutex.
    rangefinder_obstacle: bool,
}

// ── Change detection ──────────────────────────────────────────────────────────

/// Outcome of a setter that may change the combined obstacle flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChangeDetected {
    /// The obstacle flag did not change.
    NoChange,
    /// The combined obstacle flag transitioned from false → true.
    ChangedToDetected,
    /// The combined obstacle flag transitioned from true → false.
    ChangedToCleared,
}

// ── Public accessors (lock-free) ──────────────────────────────────────────────

/// Set the `LiDAR` obstacle flag and update the combined flag.
///
/// Also stores the flag in the mutex for rich-path consumers.
/// Returns whether the combined obstacle state changed.
pub async fn set_lidar_obstacle(detected: bool) -> ChangeDetected {
    let old_combined = COMBINED_OBSTACLE.load(Ordering::Relaxed);
    LIDAR_OBSTACLE.store(detected, Ordering::Relaxed);
    recompute_combined();
    let new_combined = COMBINED_OBSTACLE.load(Ordering::Relaxed);

    let mut state = PERCEPTION_STATE.lock().await;
    state.lidar_obstacle = detected;
    drop(state);

    change_detected(old_combined, new_combined)
}

/// Set the rangefinder obstacle flag and update the combined flag.
///
/// Also stores the flag in the mutex for rich-path consumers.
/// Returns whether the combined obstacle state changed.
pub async fn set_rangefinder_obstacle(detected: bool) -> ChangeDetected {
    let old_combined = COMBINED_OBSTACLE.load(Ordering::Relaxed);
    RANGEFINDER_OBSTACLE.store(detected, Ordering::Relaxed);
    recompute_combined();
    let new_combined = COMBINED_OBSTACLE.load(Ordering::Relaxed);

    let mut state = PERCEPTION_STATE.lock().await;
    state.rangefinder_obstacle = detected;
    drop(state);

    change_detected(old_combined, new_combined)
}

/// Returns `true` if any obstacle sensor (`LiDAR` or rangefinder) reports an obstacle.
/// Lock-free — safe to call from any context.
pub fn is_obstacle_detected() -> bool {
    COMBINED_OBSTACLE.load(Ordering::Relaxed)
}

/// Returns `true` if the `LiDAR` sensor reports an obstacle.
/// Lock-free — safe to call from any context.
#[allow(dead_code)]
pub fn is_lidar_obstacle_detected() -> bool {
    LIDAR_OBSTACLE.load(Ordering::Relaxed)
}

/// Returns `true` if a rangefinder sensor reports an obstacle.
/// Lock-free — safe to call from any context.
#[allow(dead_code)]
pub fn is_rangefinder_obstacle_detected() -> bool {
    RANGEFINDER_OBSTACLE.load(Ordering::Relaxed)
}

/// Async read of the `LiDAR` obstacle flag from the mutex.
pub async fn lidar_obstacle() -> bool {
    PERCEPTION_STATE.lock().await.lidar_obstacle
}

/// Async read of the rangefinder obstacle flag from the mutex.
pub async fn rangefinder_obstacle() -> bool {
    PERCEPTION_STATE.lock().await.rangefinder_obstacle
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

/// Reset all obstacle flags and sensor data to defaults.
#[allow(dead_code)]
pub async fn reset_all() {
    LIDAR_OBSTACLE.store(false, Ordering::Relaxed);
    RANGEFINDER_OBSTACLE.store(false, Ordering::Relaxed);
    COMBINED_OBSTACLE.store(false, Ordering::Relaxed);

    let mut state = PERCEPTION_STATE.lock().await;
    state.lidar = None;
    state.rangefinder = None;
    state.lidar_obstacle = false;
    state.rangefinder_obstacle = false;
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Recompute the combined obstacle flag from `LiDAR` and rangefinder atomics.
fn recompute_combined() {
    let combined = LIDAR_OBSTACLE.load(Ordering::Relaxed) || RANGEFINDER_OBSTACLE.load(Ordering::Relaxed);
    COMBINED_OBSTACLE.store(combined, Ordering::Relaxed);
}

/// Map (old, new) combined flag states to a `ChangeDetected` variant.
const fn change_detected(old_combined: bool, new_combined: bool) -> ChangeDetected {
    match (old_combined, new_combined) {
        (false, true) => ChangeDetected::ChangedToDetected,
        (true, false) => ChangeDetected::ChangedToCleared,
        _ => ChangeDetected::NoChange,
    }
}
