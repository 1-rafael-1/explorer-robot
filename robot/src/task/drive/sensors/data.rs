//! Sensor feedback channels and measurement forwarding for the drive task.
//!
//! This module owns all static sensor data channels and the functions that
//! write into or read from them. It is the *data plane* of the sensor
//! infrastructure: measurements flow in directly from sensor tasks and are
//! stored here so that drive control loops can read them on demand.
//!
//! # Channel design
//!
//! - **Encoder measurements** are stored in a mutex-guarded `Option` so the
//!   drive task always reads the latest value rather than draining a queue.
//! - **IMU measurements** are queued (capacity 16) to buffer bursts during
//!   calibration sequences at 100 Hz sampling.
//!
//! # Visibility
//!
//! All items in this module are `pub` — access is naturally limited because
//! this module itself is private within the `drive` module tree. Items called
//! by the orchestrator or IMU task are re-exported from `drive/mod.rs` where
//! needed; items used only within the drive task remain reachable only through
//! the private module path.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};

use crate::task::sensors::{encoders::EncoderMeasurement, imu::ImuMeasurement};

// ── Static channels ───────────────────────────────────────────────────────────

/// Latest encoder measurement.
///
/// The encoder task forwards measurements here. Drive control loops
/// read the latest value on demand rather than draining a queue, ensuring
/// they always see fresh data even if they miss intermediate samples.
pub static LATEST_ENCODER_MEASUREMENT: Mutex<CriticalSectionRawMutex, Option<EncoderMeasurement>> = Mutex::new(None);

/// Channel for receiving IMU measurements from the IMU task.
///
/// Capacity of 16 buffers bursts during calibration sequences at 100 Hz
/// sampling. The rotation control loop drains this channel each tick.
const IMU_FEEDBACK_QUEUE_SIZE: usize = 16;

/// IMU feedback channel shared between the orchestrator and drive control loops.
pub static IMU_FEEDBACK_CHANNEL: Channel<CriticalSectionRawMutex, ImuMeasurement, IMU_FEEDBACK_QUEUE_SIZE> =
    Channel::new();

// ── Public write functions (called by encoder / IMU task) ────────────────

/// Try to update the latest encoder measurement without blocking.
///
/// Returns `true` if the update succeeded, `false` if the mutex is currently
/// held by the drive task. Used by the encoder task to avoid blocking when
/// the drive task holds the mutex during calibration.
pub fn try_send_encoder_measurement(measurement: EncoderMeasurement) -> bool {
    LATEST_ENCODER_MEASUREMENT.try_lock().is_ok_and(|mut latest| {
        *latest = Some(measurement);
        true
    })
}

/// Try to send an IMU measurement to the drive task without blocking.
///
/// Returns `true` if sent, `false` if the channel is full. Used by the
/// IMU task to avoid blocking; IMU measurements arrive at 100 Hz so
/// dropping occasional readings during heavy load is acceptable.
pub fn try_send_imu_measurement(measurement: ImuMeasurement) -> bool {
    IMU_FEEDBACK_CHANNEL.sender().try_send(measurement).is_ok()
}

/// Clear any queued IMU measurements.
///
/// Called before starting a new rotation to avoid stale samples.
/// Bounded to the known queue capacity so the loop always terminates even if
/// the IMU producer is active concurrently.
pub fn clear_imu_measurements() {
    for _ in 0..IMU_FEEDBACK_QUEUE_SIZE {
        if IMU_FEEDBACK_CHANNEL.receiver().try_receive().is_err() {
            break;
        }
    }
}

// ── Internal read / clear functions (used by drive submodules) ────────────────

/// Get the latest encoder measurement.
///
/// Returns the most recent encoder measurement, or `None` if no measurement
/// is available yet (e.g., before the encoder task has started sampling).
pub async fn get_latest_encoder_measurement() -> Option<EncoderMeasurement> {
    let latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest
}

/// Clear the latest encoder measurement.
///
/// Called before starting a new distance drive or calibration step to ensure
/// the next read waits for a fresh sample after the encoder counters are reset.
pub async fn clear_encoder_measurement() {
    let mut latest = LATEST_ENCODER_MEASUREMENT.lock().await;
    *latest = None;
}

/// Wait for a fresh encoder measurement with a timeout.
///
/// Polls `get_latest_encoder_measurement` at 10 ms intervals until a value
/// is available or `timeout_ms` elapses. Used during motor calibration to
/// wait for the first sample after a counter reset.
pub async fn wait_for_encoder_event_timeout(timeout_ms: u64) -> Option<EncoderMeasurement> {
    use embassy_time::{Duration, Instant, Timer};

    let start = Instant::now();
    let timeout_duration = Duration::from_millis(timeout_ms);

    loop {
        if let Some(measurement) = get_latest_encoder_measurement().await {
            return Some(measurement);
        }

        if start.elapsed() >= timeout_duration {
            return None;
        }

        Timer::after(Duration::from_millis(10)).await;
    }
}
