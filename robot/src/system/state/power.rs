//! Power state module.
//!
//! Holds battery status in a private mutex, accessed through public functions.
//! All fields start as `None` — populated on first battery measurement.
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) power state mutex (private — use accessor functions)
//! 2) `CALIBRATION_STATE`
//! 3) perception mutex (use accessor functions, see `perception` module)
//! 4) `MOTION_STATE`
//! 5) `ACTIVITY_STATE` (use accessor functions, see `activity` module)
//! 6) `IMU_TEST_STATE` — independent leaf; never taken while another state mutex is held
//!
//! Callers that need both Power and Calibration data should call Power
//! accessors before locking `CALIBRATION_STATE`.
//! Calling power accessors while already holding another state lock
//! violates this order and can deadlock.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Instant;

/// Global power state — private, use accessor functions.
static POWER_STATE: Mutex<CriticalSectionRawMutex, PowerState> = Mutex::new(PowerState {
    battery_level: None,
    battery_voltage: None,
    rail_voltage: None,
    rail_sampled_at: None,
});

/// Power-related state shared across the system.
struct PowerState {
    /// Battery level percentage (0-100), if available.
    battery_level: Option<u8>,
    /// Pack-terminal voltage in volts, for the panel and Link telemetry.
    battery_voltage: Option<f32>,
    /// Measured `+8V` motor rail in volts, for voltage compensation.
    rail_voltage: Option<f32>,
    /// When `rail_voltage` was measured. The motor driver uses this to reject
    /// samples taken before it became idle.
    rail_sampled_at: Option<Instant>,
}

/// Write a complete battery measurement in one atomic update.
///
/// `rail_sampled_at` is the ADC sample time; the motor driver compares it with
/// the moment it last became idle so it can reject a loaded sample.
pub async fn set_measurement(level: u8, battery_voltage: f32, rail_voltage: f32, rail_sampled_at: Instant) {
    let mut state = POWER_STATE.lock().await;
    state.battery_level = Some(level);
    state.battery_voltage = Some(battery_voltage);
    state.rail_voltage = Some(rail_voltage);
    state.rail_sampled_at = Some(rail_sampled_at);
}

/// Refresh the operator-facing level and pack voltage.
///
/// Leaves the measured rail voltage untouched so the motor driver keeps the
/// voltage it compensates against.
pub async fn set_battery(level: u8, voltage: f32) {
    let mut state = POWER_STATE.lock().await;
    state.battery_level = Some(level);
    state.battery_voltage = Some(voltage);
}

/// Read battery level (0-100). Returns `None` before first measurement.
pub async fn get_battery_level() -> Option<u8> {
    POWER_STATE.lock().await.battery_level
}

/// A consistent snapshot of battery state, read under a single lock.
#[derive(Clone, Copy, Debug)]
pub struct BatterySnapshot {
    /// Battery level percentage (0-100), if available.
    pub level: Option<u8>,
    /// Pack-terminal voltage in volts, if available.
    pub voltage: Option<f32>,
}

/// Read both battery level and voltage under a single lock,
/// returning a consistent snapshot even if `set_battery()` is
/// called concurrently.
pub async fn get_battery_snapshot() -> BatterySnapshot {
    let state = POWER_STATE.lock().await;
    BatterySnapshot {
        level: state.battery_level,
        voltage: state.battery_voltage,
    }
}

/// Non-blocking motor-rail sample for the motor control loop.
///
/// Returns the measured rail voltage and when it was sampled. The motor driver
/// compares the timestamp with the moment it last became idle, so a sample
/// taken while the motors were loaded is never used for compensation.
pub fn try_get_rail_sample() -> Option<(f32, Instant)> {
    let state = POWER_STATE.try_lock().ok()?;
    Some((state.rail_voltage?, state.rail_sampled_at?))
}
