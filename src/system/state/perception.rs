//! Perception state module.
//!
//! Owns the `LiDAR` point cloud behind a single interface. Obstacle and
//! floor-drop detection flags are exposed via lock-free atomics (fast path)
//! while the point cloud sits behind a mutex (rich path). The `LiDAR` task is
//! the only writer of the obstacle flag.
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
//! 5) `ACTIVITY_STATE` (use accessor functions, see `activity` module)

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use lidar_cloud::Cloud;

// ── LiDAR point cloud ────────────────────────────────────────────────────────
//
// The cloud type is the `lidar-cloud` crate's [`Cloud`]: 360 one-degree slots of
// an optional distance in centimetres, slot 0 dead ahead, increasing slots
// clockwise on the glass (ADR-0012). A missing return is `None`, never a zero
// distance.

// ── Atomics (lock-free fast path for reads) ────────────────────────────────────

/// `LiDAR` obstacle detection flag — written by `set_lidar_obstacle`, read lock-free.
static LIDAR_OBSTACLE: AtomicBool = AtomicBool::new(false);
/// Floor-drop detection flag — written by `set_floor_drop`, read lock-free.
static FLOOR_DROP: AtomicBool = AtomicBool::new(false);

// ── Mutex (full picture: point cloud) ─────────────────────────────────────────

/// Mutex-guarded state holding the `LiDAR` point cloud.
static PERCEPTION_STATE: Mutex<CriticalSectionRawMutex, PerceptionState> = Mutex::new(PerceptionState { lidar: None });

/// Internal state behind the mutex — holds the `LiDAR` point cloud.
struct PerceptionState {
    /// Latest `LiDAR` point cloud, if available.
    lidar: Option<Cloud>,
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
/// Returns whether the obstacle state changed. The flag lives only in the
/// lock-free atomic; the `LiDAR` task is its sole writer, so nothing else
/// mirrors the value back into this module.
pub fn set_lidar_obstacle(detected: bool) -> ChangeDetected {
    let old = LIDAR_OBSTACLE.load(Ordering::Relaxed);
    LIDAR_OBSTACLE.store(detected, Ordering::Relaxed);
    change_detected(old, detected)
}

/// Set the floor-drop flag.
pub fn set_floor_drop(detected: bool) {
    FLOOR_DROP.store(detected, Ordering::Relaxed);
}

/// Returns `true` if the `LiDAR` sensor reports an obstacle.
/// Lock-free — safe to call from any context.
pub fn is_obstacle_detected() -> bool {
    LIDAR_OBSTACLE.load(Ordering::Relaxed)
}

/// Returns `true` if a floor drop is currently detected.
/// Lock-free — safe to call from any context.
pub fn is_floor_drop_detected() -> bool {
    FLOOR_DROP.load(Ordering::Relaxed)
}

// ── Public accessors (async — touch the mutex) ────────────────────────────────

/// Replace the stored `LiDAR` point cloud with a new scan.
pub async fn update_lidar_points(cloud: Cloud) {
    let mut state = PERCEPTION_STATE.lock().await;
    state.lidar = Some(cloud);
}

/// Return a clone of the current `LiDAR` point cloud, if available.
pub async fn get_lidar_snapshot() -> Option<Cloud> {
    PERCEPTION_STATE.lock().await.lidar.clone()
}

/// Run `f` with the stored `LiDAR` cloud borrowed under the lock.
///
/// A caller that only reads the cloud — the Room Scan refresh, which hands the
/// slots to the UI model under the mutex — uses this instead of
/// [`get_lidar_snapshot`] so the 360-slot array is never cloned. The closure
/// runs while the perception mutex is held, so it must not take another lock:
/// perception is lock-order step 3 (see the module docs). The borrowed cloud is
/// `None` when the sensor has published nothing.
pub async fn with_lidar<R>(f: impl FnOnce(Option<&Cloud>) -> R) -> R {
    let state = PERCEPTION_STATE.lock().await;
    f(state.lidar.as_ref())
}

/// Clear the stored `LiDAR` cloud and its obstacle flag.
///
/// Called by the `LiDAR` task's release and terminal-failure paths so stale data
/// cannot outlive a powered-down sensor: the cloud becomes absent and the
/// obstacle flag is cleared in the lock-free snapshot.
///
/// Returns [`ChangeDetected::ChangedToCleared`] when the obstacle flag had been
/// set, so the caller can raise the matching cleared edge. Takes only the
/// perception mutex (lock-order step 3) and holds no other lock.
pub async fn clear_lidar_state() -> ChangeDetected {
    let old = LIDAR_OBSTACLE.swap(false, Ordering::Relaxed);

    let mut state = PERCEPTION_STATE.lock().await;
    state.lidar = None;
    drop(state);

    change_detected(old, false)
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
