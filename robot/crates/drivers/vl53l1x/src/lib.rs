//! Async VL53L1X time-of-flight rangefinder driver.
//!
//! The robot carries two TOF0400C/VL53L1X modules, one per dedicated I2C bus,
//! as a floor-drop safety sensor (ADR-0026, ADR-0049). This crate is the
//! register-level device logic:
//!
//! - [`Rangefinder`] owns the async I2C device, the open-drain `XSHUT` output
//!   and the active-low `INT` input, and exposes `new`, `init`, `read`,
//!   `wait_ready`, `reset` and `standby`.
//! - [`RangefinderReading`] carries the classified [`Reading`], the decoded
//!   [`RangeStatus`], the raw 8-bit status code and the [`RangeDiagnostics`].
//! - [`Error`] separates bus failures from identification, boot, budget,
//!   shutdown and interrupt failures, and reports a spurious wake as
//!   `NotReady`.
//!
//! The crate is `#![no_std]`. Enable the `std` feature to build and run the
//! host-side integration tests.
//!
//! # Wiring
//!
//! `XSHUT` is open-drain: the driver drives it low to hold the module off and
//! releases it high so the module's own pull-up can bring it up. It never drives
//! the line push-pull high. `INT` is active-low, so
//! [`Rangefinder::wait_ready`] waits for a low level.
//!
//! # Provenance
//!
//! The initialisation and read sequences are ported from `vl53l1x-simple`
//! 0.2.0 (MIT OR Apache-2.0) and adapted to this async, 16-bit-register API.
//! The port adds the explicit `GPIO1` interrupt configuration, the 8-bit status
//! and diagnostics exposure, boot-NACK tolerance and the one-time VHV/phasecal
//! calibration placement that the poll-only reference lacks. See
//! `THIRD_PARTY_LICENSES.md` at the repository root and ADR-0049.
//!
//! # Measurement profile
//!
//! `init` applies short distance mode (about 1.36 m, the best ambient-light
//! tolerance), a 33 ms timing budget and a 50 ms inter-measurement period. The
//! floor-drop threshold itself is policy and lives in the `rangefinder` crate.

#![no_std]
#![warn(missing_docs)]

pub mod driver;
pub mod registers;
pub mod types;

pub use driver::{Error, Rangefinder};
pub use types::{RangeDiagnostics, RangeStatus, RangefinderReading, Reading};
