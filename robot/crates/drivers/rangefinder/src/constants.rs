//! Timing, threshold and lifecycle constants for the rangefinder pair.
//!
//! This is the floor-drop policy surface. The measurement-profile constants
//! alias the `vl53l1x` driver so the sensor's inter-measurement period and the
//! pair task's ticker cannot drift apart; the remaining values are the policy
//! numbers recorded in ADR-0048.

/// A valid reading strictly above this distance means the floor is gone.
pub const DROP_THRESHOLD_MM: u16 = 400;

/// Inter-measurement period for continuous timed ranging.
///
/// Aliases the driver's commanded period so the pair task ticker and the sensor
/// agree.
pub const RANGING_PERIOD_MS: u32 = vl53l1x::INTERMEASUREMENT_PERIOD_MS;

/// Measurement timing budget the driver applies, in microseconds.
///
/// Aliases the driver's short-distance-mode budget so the policy and the device
/// configuration agree.
pub const TIMING_BUDGET_US: u32 = vl53l1x::SHORT_TIMING_BUDGET_US;

/// How long an invalid or no-target streak may hold the last state before the
/// detector fails safe to [`Drop`](crate::FloorState::Drop).
pub const INVALID_HOLD_MS: u64 = 200;

/// A gap of at least this long without a reading marks the sensor stale.
pub const STALE_TIMEOUT_MS: u64 = 150;

/// Consecutive valid floor readings required to clear a drop.
pub const CLEAR_SAMPLES: u8 = 2;

/// Reset attempts allowed per enable before the sensor latches as failed.
pub const RESET_ATTEMPTS: u8 = 3;

/// How long the drive gate waits for the pair to become armed.
pub const GATE_TIMEOUT_MS: u64 = 1_000;

/// Publishing rate of the panel test-mode readout.
pub const TEST_PUBLISH_HZ: u8 = 10;
