//! VL53L1X floor-drop policy facade.
//!
//! The robot carries two TOF0400C/VL53L1X time-of-flight modules, one per
//! dedicated I2C bus, as a floor-drop safety sensor (ADR-0026, ADR-0048,
//! ADR-0049). The register-level device logic lives in the `vl53l1x` crate;
//! this crate is the policy built on top of it:
//!
//! - [`Rangefinder`], [`RangefinderReading`], [`RangeStatus`] and the other
//!   reading types are re-exported from the driver so the firmware's imports stay
//!   stable.
//! - [`FloorDropDetector`], a pure per-sensor state machine with immediate drop,
//!   two-sample recovery, a 200 ms invalid hold and a 150 ms stale watchdog.
//! - [`SensorLifecycle`], the per-sensor arming and recovery state machine. Only
//!   a **Rangefinder Fault** status drives invalid-streak recovery.
//! - [`pair_state`] and [`gate`], the pair lifecycle and drive-gate policy.
//!
//! The crate is `#![no_std]`. Enable the `std` feature to build and run the
//! host-side integration tests.

#![no_std]
#![warn(missing_docs)]

pub mod constants;
pub mod detector;
pub mod lifecycle;
pub mod pair;

pub use constants::{
    CLEAR_SAMPLES, DROP_THRESHOLD_MM, GATE_TIMEOUT_MS, INVALID_HOLD_MS, RANGING_PERIOD_MS, RESET_ATTEMPTS,
    STALE_TIMEOUT_MS, TEST_PUBLISH_HZ, TIMING_BUDGET_US,
};
pub use detector::{FloorDropDetector, FloorState, ReadingHealth};
pub use lifecycle::{RecoveryCause, SensorLifecycle};
pub use pair::{GateRefusal, PairState, SensorState, gate, pair_state};
pub use vl53l1x::{Error, RangeDiagnostics, RangeStatus, Rangefinder, RangefinderReading, Reading};
