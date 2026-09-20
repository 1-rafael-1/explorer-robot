//! Task modules for robot operation
//!
//! - `motor_driver` — TB6612FNG motor control (PWM + direct GPIO direction)
//! - `drive` — drive command model, calibration, distance tracking
//! - `orchestrate` — central event loop dispatching to behavior handlers
//! - `battery_charge_read` — ADC battery voltage monitoring
//! - `indicators` — RGB LED status indication
//! - `io` — graphics panel (TFT + touch), flash storage
//! - `sensors` — encoder reader, IMU, `LiDAR` driver, VL53L0X stub
//! - `ui` — touch-driven menu controller (panel owner)
//! - `autonomous_mode` — coast-and-avoid, attempt-straight-line behaviors
//! - `behavior` — event-driven behavior handlers (battery, obstacle, input)
//! - `initialization` — boot-time calibration loading coordination
//! - `testmode` — on-demand test mode tasks (motor, turns, IMU, drive)
//! - `startup` — fires Initialize event at boot

pub mod autonomous_mode;
pub mod battery_charge_read;
/// Behavior handlers for system events.
pub mod behavior;
pub mod drive;
/// LED and other visual indicators.
pub mod indicators;
pub mod initialization;
pub mod io;
pub mod motor_driver;
pub mod orchestrate;
/// Sensor tasks (`IMU`, encoders, `LiDAR` driver, `VL53L0X` stub).
pub mod sensors;
pub mod startup;
pub mod testmode;
pub mod ui;
