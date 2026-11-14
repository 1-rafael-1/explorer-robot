//! Async VL53L0X rangefinder driver with floor-drop detection.
//!
//! The robot carries two VL53L0X time-of-flight modules, one per dedicated I2C
//! bus, as a floor-drop safety sensor (ADR-0048). This crate provides:
//!
//! - [`Rangefinder`], an async wrapper that owns the I2C device, the open-drain
//!   `XSHUT` output and the `INT` input, and exposes `init`, `read`,
//!   `wait_ready`, `reset` and `standby`.
//! - [`RangefinderReading`] and [`RangeStatus`], which keep the sensor's raw
//!   4-bit range status instead of discarding it.
//! - [`FloorDropDetector`], a pure per-sensor state machine with immediate drop,
//!   two-sample recovery, a 200 ms invalid hold and a 150 ms stale watchdog.
//! - [`pair_state`] and [`gate`], the pair lifecycle and drive-gate policy.
//!
//! The crate is `#![no_std]`. Enable the `std` feature to build and run the
//! host-side integration tests.
//!
//! # Wiring
//!
//! `XSHUT` is open-drain: the wrapper drives it low to hold the module off and
//! releases it high so the module's own pull-up can bring it up. It never drives
//! the line push-pull high. `INT` is active-low, so [`Rangefinder::wait_ready`]
//! waits for a low level.

#![no_std]
#![warn(missing_docs)]

pub mod constants;
pub mod detector;
pub mod driver;
pub mod lifecycle;
pub mod pair;
pub mod types;

pub use constants::{
    CLEAR_SAMPLES, DROP_THRESHOLD_MM, GATE_TIMEOUT_MS, INVALID_HOLD_MS, OUT_OF_RANGE_MILLIMETERS, RANGING_PERIOD_MS,
    RESET_ATTEMPTS, STALE_TIMEOUT_MS, TEST_PUBLISH_HZ, TIMING_BUDGET_US,
};
pub use detector::{FloorDropDetector, FloorState, ReadingHealth};
pub use driver::{Error, Rangefinder};
pub use lifecycle::{RecoveryCause, SensorLifecycle};
pub use pair::{GateRefusal, PairState, SensorState, gate, pair_state};
pub use types::{RangeStatus, RangefinderReading, Reading};
