//! Async driver for the COIN-D6 360° spinning dToF `LiDAR`.
//!
//! The COIN-D6 streams a continuous point cloud over UART at a native 0.9°
//! resolution — 400 points per revolution. This crate splits that pipeline into three
//! stages, one per module:
//!
//! - [`decoder`] reassembles raw UART bytes into individual [`Point`]s, resynchronising
//!   on the sensor's per-frame checksum.
//! - [`post_processing`] aggregates several revolutions into a single stable
//!   [`Scan`] according to an [`AggregationConfig`].
//! - [`driver`] owns the async UART and the power-enable pin, and wires the other two
//!   stages together into a ready-to-consume scan stream.
//!
//! The crate is `#![no_std]` so it can run on the RP2350's second core. Enable the
//! `std` feature to build and run the host-side integration tests.

#![no_std]
#![warn(missing_docs)]

pub mod decoder;
pub mod driver;
pub mod post_processing;
pub mod types;
pub mod warmup;

pub use decoder::{Decode, Decoder};
pub use driver::CoinD6;
pub use post_processing::{ANGLE_CORRECTION_COEFF, ANGLE_CORRECTION_ZERO_MM, aggregate, angle_correction_deg};
pub use types::{
    AggregationConfig, AggregationMethod, Config, Error, NATIVE_POINTS, Point, Scan, WarmupConfig, WarmupOutcome,
};
pub use warmup::Warmup;
