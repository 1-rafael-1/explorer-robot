//! Timing, threshold and lifecycle constants for the rangefinder pair.
//!
//! These are the single source of truth for the numbers the firmware and the
//! floor-drop detector share. They mirror the values recorded in ADR-0048 and
//! ADR-0049. The sensor's measurement profile aliases the driver constants, so
//! the inter-measurement period and the task ticker cannot drift apart.

/// A valid reading strictly above this distance means the floor is gone.
pub const DROP_THRESHOLD_MM: u16 = 400;

/// Inter-measurement period for continuous timed ranging.
///
/// Aliases the driver's commanded period so the pair task ticker and the sensor
/// agree.
pub const RANGING_PERIOD_MS: u32 = vl53l1x::INTERMEASUREMENT_PERIOD_MS;

/// Measurement timing budget handed to the sensor after initialisation.
///
/// Aliases the driver's short-distance-mode budget.
pub const TIMING_BUDGET_US: u32 = vl53l1x::SHORT_TIMING_BUDGET_US;

/// How long an invalid or no-target streak may hold the last state before the
/// detector fails safe to [`Drop`](crate::FloorState::Drop).
pub const INVALID_HOLD_MS: u64 = 200;

/// A gap of at least this long without a sample marks the sensor stale.
pub const STALE_TIMEOUT_MS: u64 = 150;

/// Consecutive valid floor readings required to clear a drop.
pub const CLEAR_SAMPLES: u8 = 2;

/// Reset attempts allowed per enable before the sensor latches as failed.
pub const RESET_ATTEMPTS: u8 = 3;

/// How long the drive gate waits for the pair to become armed.
pub const GATE_TIMEOUT_MS: u64 = 1_000;

/// Publishing rate of the panel test-mode readout.
pub const TEST_PUBLISH_HZ: u8 = 10;
