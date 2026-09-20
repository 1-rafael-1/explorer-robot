//! Core system components for robot operation
//!
//! - `event` — typed event system for inter-task communication
//! - `state` — domain state modules (power, calibration, perception, motion, activity)

pub mod event;
pub mod state;
