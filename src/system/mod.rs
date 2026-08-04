//! Core system components for robot operation
//!
//! - `event` — typed event system for inter-task communication
//! - `helper` — utility functions (string formatting, math helpers)
//! - `state` — domain state modules (power, motion, perception, calibration)

pub mod event;
pub mod helper;
pub mod state;
