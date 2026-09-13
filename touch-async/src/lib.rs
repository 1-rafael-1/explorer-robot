//! Async driver for an XPT2046/TSC2046-class resistive touch controller.
//!
//! This crate implements the SPI register protocol shared by
//! XPT2046/TSC2046-class controllers: each conversion is requested with an
//! 8-bit command byte selecting a 12-bit, single-ended channel with the
//! controller powered down between conversions, and answered with a 2-byte
//! big-endian frame whose upper 12 bits are the sample. The exact framing
//! implemented here was verified against the Waveshare Pico-ResTouch wiring.
//!
//! The driver is HAL-agnostic: it needs only an
//! [`embedded_hal_async::spi::SpiDevice`] for the panel and, for
//! [`TouchPanel::wait_for_touch`], an active-low
//! [`embedded_hal_async::digital::Wait`] for the `PENIRQ` line. It is
//! `#![no_std]`; enable the `std` feature to build the host-side tests.
//!
//! # Hardware status
//!
//! The touch controller's IC markings have been sanded off, so the exact part
//! number remains unconfirmed. The assumed XPT2046/TSC2046-class protocol was
//! nevertheless confirmed empirically on the bench — raw X/Y track the finger
//! and both Z channels respond — so the framing above is no longer only an
//! assumption.
//!
//! # Calibration
//!
//! Raw counts are mapped to pixels by a pure [`Calibration`], kept separate from
//! the bus so it can be re-measured without touching the driver. Two constants
//! ship for a 320 × 240 area: [`Calibration::REFERENCE`] holds uncalibrated
//! vendor values for first bring-up, and [`Calibration::MEASURED`] holds this
//! project's measured panel.
//!
//! To (re-)measure, render a target at a known pixel coordinate, touch it, and
//! read the raw sample; do this at two points per axis and solve the linear
//! `raw -> pixel` endpoints. The `hardware-tests` `touch_coexistence` example
//! draws such targets at the framebuffer corners, and the `hardware-tests`
//! README documents the full procedure.

#![no_std]
#![warn(missing_docs)]

pub mod calibration;
pub mod driver;
pub mod pressure;
pub mod types;

pub use calibration::{Calibration, CalibrationError};
pub use driver::TouchPanel;
pub use pressure::pressure;
pub use types::{Error, TouchSample};
