//! System State Management
//!
//! Manages the robot's global state including:
//! - Shared enums and cross-cutting state modules
//!
//! State is compartmentalized into domain-specific modules, each with its own
//! synchronization. Shared enums live here; domain state lives in dedicated
//! submodules (e.g. `power`, `motion`, `perception`, `calibration`, `activity`).

use defmt::Format;

pub mod activity;
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
