//! System State Management
//!
//! Manages the robot's global state including:
//! - Shared enums and cross-cutting state modules
//!
//! State is compartmentalized into domain-specific modules, each with its own
//! synchronization. Shared enums live here; domain state lives in dedicated
//! submodules (e.g. `power`, `motion`, `perception`, `calibration`).

use defmt::Format;

pub mod calibration;
pub mod motion;
pub mod perception;
pub mod power;

/// Calibration data status.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum CalibrationStatus {
    /// Calibration data has not been queried yet.
    NotLoaded,
    /// Calibration data was loaded from flash.
    Loaded,
    /// No calibration data exists in flash (needs calibration run).
    NotAvailable,
}

/// Main menu selections.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum MenuSelection {
    /// Show system info screen.
    SystemInfo,
    /// Enter calibration submenu.
    Calibrate,
    /// Enter drive mode submenu.
    DriveMode,
    /// Enter test submenu.
    TestMode,
}

/// Autonomous drive mode selection.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum DriveMode {
    /// Coast until obstacle detected, then back up and turn randomly.
    CoastAndAvoid,
    /// Attempt to travel toward a user-defined target distance by
    /// sweeping, finding gaps, and correcting drift.
    AttemptStraightLine,
}

/// Test submenu selections (v3 — LiDAR/rangefinder based, no IR/ultrasonic).
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum TestSelection {
    /// Run left/right track at configurable speed via menu.
    BasicMotor,
    /// In-place turn accuracy test at multiple speeds.
    Turns,
    /// Straight-line encoder-based distance test (forward + backward).
    StraightDrive,
    /// Curve arc drive test (360° circle at 1 m radius).
    ArcDrive,
    /// IMU live display test (6-axis: accel + gyro).
    Imu6Axis,
    /// IMU live display test (9-axis: accel + gyro + mag).
    Imu9Axis,
}

/// Calibration submenu selections.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum CalibrationSelection {
    /// Motor speed calibration.
    Motor,
    /// Magnetometer calibration.
    Mag,
    /// Distance factor calibration.
    Distance,
}
