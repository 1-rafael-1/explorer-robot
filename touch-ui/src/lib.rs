//! Host-testable touch UI model for the explorer-robot's graphics panel.
//!
//! This crate is the pure UI half of the robot's touch interface: the screen
//! tree, navigation, gesture tracking, hit-testing, layout geometry, the
//! palette, the widgets, the value-entry flow, and the Room Scan radar. It is
//! `#![no_std]` and HAL-free, drawing against a generic
//! [`embedded_graphics`] target, so the same model runs on the robot and under
//! host tests. The `std` feature only gates the host test targets; the library
//! itself never links `std`.
//!
//! It is a port of the bench feeler
//! (`hardware-tests/examples/touch_menu.rs`), which established the screen
//! model, navigation, tap-versus-drag thresholds, hit-testing, geometry,
//! palette, and widgets. The radar widget ports the drawing from
//! `hardware-tests/examples/lidar_tft_radar.rs`.
//!
//! # What lives here
//!
//! - [`screens`] holds the robot's menu tree and where Back returns.
//! - [`ui`] holds [`Ui`], the state model: which screen is shown, the active
//!   press, scrolling, hit-testing, and the value-entry value.
//! - [`geometry`] holds every on-screen rectangle, shared by hit-testing and
//!   rendering so the two cannot disagree.
//! - [`palette`] holds the colours and fonts.
//! - [`widgets`] holds the drawing functions.
//! - [`system_info`] holds the neutral System Info snapshot the firmware maps
//!   onto.
//! - [`radar`] holds the Room Scan radar widget, fed by a neutral 360-slot
//!   input.
//! - [`hit`] names the interactive regions and the header action button.
//!
//! # What does not live here
//!
//! No firmware type, no driver, no HAL, and no `LiDAR` type crosses this
//! boundary: System Info takes a neutral snapshot and the radar takes a plain
//! 360-slot array of optional distances in centimetres.
//!
//! Rendering is full-frame on state change and throttled during a drag by the
//! caller, which flushes the framebuffer after each [`Ui::render`] call.

#![no_std]
#![warn(missing_docs)]
// Pixel geometry converts between `usize`, `u32`, `i32`, `i64` and `f32`. Every
// value is a small positive coordinate well inside the 320x240 framebuffer, so
// the conversions cannot truncate, wrap, or lose sign; the trigonometric
// plotting is the one place `f32` is used, and its precision loss is
// sub-pixel. This mirrors the bench feeler, which allows the same casts.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]

pub mod buffer;
pub mod geometry;
pub mod hit;
pub mod palette;
pub mod radar;
pub mod screens;
pub mod system_info;
pub mod ui;
pub mod widgets;

pub use embedded_graphics;
pub use hit::{HeaderAction, Hit};
pub use screens::{PlaceholderKind, Screen, StatusView, ValueFlow};
pub use system_info::{CalibrationStatus, SystemInfo};
pub use ui::{DRAG_RENDER_MS, TAP_MAX_MOVE, TAP_MIN_DURATION_MS, TICK_MS, Ui};
