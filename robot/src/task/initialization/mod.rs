//! System initialization and calibration coordination.
//!
//! Orchestrates boot-time setup, calibration loading, and the UI's handover to
//! the main menu.
//!
//! Progress is reported through [`crate::system::state::activity`] — the boot
//! flow takes the one Active Claim while it loads and records what it found — and
//! this module formats no display text. The claim is held in [`BOOT_CLAIM`] and
//! abandoned when the main menu is ready, because boot is not a Procedure and has
//! no Result Report. When another claim already holds the slot, boot loads
//! silently: it never writes over, or clears, a Procedure's Activity.

use defmt::{info, warn};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use crate::{
    system::state::{CalibrationStatus, calibration},
    task::{
        io::flash_storage::{self, CalibrationDataKind, CalibrationKind, ImuCalibrationFlags},
        motor_driver::{self, MotorCommand},
        procedure::{self, Claim, ClaimKind, ClaimOrigin},
        sensors::imu,
        ui,
    },
};

/// The boot flow's Active Claim, held for the duration of the calibration load.
///
/// `None` when another claim already held the slot, and again once boot has
/// abandoned its own; both mean boot writes no Activity.
static BOOT_CLAIM: Mutex<CriticalSectionRawMutex, Option<Claim>> = Mutex::new(None);

/// Handle system initialization.
pub async fn handle_initialize() {
    info!("System initializing");

    // Take the claim if it is free. A Procedure that already owns the slot keeps
    // it: the boot flow then reports progress only to the log.
    let claim = procedure::claim(ClaimKind::Boot, ClaimOrigin::Panel).await;
    if claim.is_none() {
        info!("Boot flow: another claim is held, reporting progress only to the log");
    }
    *BOOT_CLAIM.lock().await = claim;

    // Request motor calibration from flash.
    info!("Requesting motor calibration from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::Motor,
    ))
    .await;

    // Request IMU calibration from flash.
    info!("Requesting IMU calibration from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::ImuFlags,
    ))
    .await;

    // Request distance calibration from flash.
    info!("Requesting distance calibration from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::Distance,
    ))
    .await;

    // Request the DMP Bias Snapshot from flash. Its response applies the snapshot
    // to the DMP and settles `dmp_cal_status`, which the boot gate waits for.
    info!("Requesting DMP bias snapshot from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::DmpBias,
    ))
    .await;

    // Note: initialization completes when calibration data arrives via events.
}

/// Publish a boot phase through the held claim, if there is one.
///
/// A late event from an older boot is inert: [`Claim::phase`] is a no-op once the
/// claim's generation no longer holds the slot.
async fn publish_phase(detail: &'static str, percent: Option<u8>) {
    let claim = *BOOT_CLAIM.lock().await;
    if let Some(claim) = claim {
        claim.phase(detail, percent).await;
    }
}

/// Handle calibration data loaded from flash storage.
pub async fn handle_calibration_data_loaded(kind: CalibrationKind, data: Option<CalibrationDataKind>) {
    match kind {
        CalibrationKind::Motor => load_motor_calibration(data).await,
        CalibrationKind::Distance => load_distance_calibration(data).await,
        CalibrationKind::ImuFlags => {
            let flags = data.as_ref().and_then(|d| match d {
                CalibrationDataKind::ImuFlags(flags) => Some(*flags),
                _ => None,
            });
            handle_imu_calibration_flags_loaded(flags).await;
        }
        CalibrationKind::DmpBias => {
            let words = data.and_then(|data| match data {
                CalibrationDataKind::DmpBias(words) => Some(words),
                _ => None,
            });
            load_dmp_calibration(words).await;
        }
    }

    check_initialization_complete().await;
}

/// Apply the DMP Bias Snapshot to the DMP, or record that none is available.
///
/// `Loaded` is published only when the nine words were actually written to the
/// DMP. An absent, corrupt or wrong-version snapshot arrives as `None` (flash
/// validation rejects it), and a failed DMP write leaves the robot uncalibrated;
/// both are `NotAvailable` so System Info reports `Missing` and the boot gate is
/// not held shut once the query has settled.
async fn load_dmp_calibration(words: Option<flash_storage::DmpBiasWords>) {
    let status = if let Some(words) = words {
        info!("DMP bias snapshot found — applying to the DMP");
        match imu::apply_dmp_bias(words.into()).await {
            Ok(()) => {
                info!("DMP bias snapshot applied to the DMP");
                CalibrationStatus::Loaded
            }
            Err(error) => {
                warn!("DMP bias snapshot apply failed: {:?}", error);
                CalibrationStatus::NotAvailable
            }
        }
    } else {
        warn!("No valid DMP bias snapshot found — DMP calibration missing");
        CalibrationStatus::NotAvailable
    };

    // Never hold the state lock across the DMP write; it is taken only to store
    // the settled result. `check_initialization_complete` reads it next.
    calibration::set_dmp_cal_status(status).await;
}

/// Apply the motor calibration, or record that none was found.
async fn load_motor_calibration(data: Option<CalibrationDataKind>) {
    if let Some(CalibrationDataKind::Motor(motor_cal)) = data {
        info!(
            "Motor calibration loaded: left_factor={} right_factor={}",
            motor_cal.left_factor, motor_cal.right_factor
        );

        {
            let mut state = calibration::CALIBRATION_STATE.lock().await;
            state.motor_cal_status = CalibrationStatus::Loaded;
        }

        motor_driver::send_motor_command(MotorCommand::LoadCalibration(motor_driver::MotorCalibration::new(
            motor_cal.left_factor,
            motor_cal.right_factor,
        )))
        .await;

        publish_phase("Calibration loaded", None).await;
    } else {
        info!("No motor calibration found - using defaults");

        {
            let mut state = calibration::CALIBRATION_STATE.lock().await;
            state.motor_cal_status = CalibrationStatus::NotAvailable;
        }

        publish_phase("Need calibration", None).await;
    }
}

/// Apply the distance calibration factor, or record that none was found.
async fn load_distance_calibration(data: Option<CalibrationDataKind>) {
    if let Some(CalibrationDataKind::Distance(factor)) = data {
        info!("Distance calibration loaded: factor={}", factor);
        {
            let mut state = calibration::CALIBRATION_STATE.lock().await;
            state.distance_cal_status = CalibrationStatus::Loaded;
            state.distance_factor = factor;
        }
    } else {
        info!("No distance calibration found - using default 1.0");
        {
            let mut state = calibration::CALIBRATION_STATE.lock().await;
            state.distance_cal_status = CalibrationStatus::NotAvailable;
        }
        publish_phase("Need calibration", None).await;
    }
}

/// Handle IMU calibration flags loaded from flash.
///
/// This result settles both IMU statuses, because nothing else ever queries
/// them. The aggregate status records that the query completed: a board whose
/// flash holds no flags yet is `NotAvailable`, not left as `NotLoaded` — a
/// `NotLoaded` status would hold `is_initialized` false forever, keeping the
/// boot gate shut and the panel dark.
async fn handle_imu_calibration_flags_loaded(flags: Option<ImuCalibrationFlags>) {
    {
        let mut state = calibration::CALIBRATION_STATE.lock().await;
        state.mag_cal_status = if flags.is_some_and(|flags| flags.mag) {
            CalibrationStatus::Loaded
        } else {
            CalibrationStatus::NotAvailable
        };
        state.imu_cal_status = if flags.is_some() {
            CalibrationStatus::Loaded
        } else {
            CalibrationStatus::NotAvailable
        };
    }

    if ui::ui_initialized().await {
        ui::show_main_menu().await;
    }

    check_initialization_complete().await;
}

/// Check if initialization is complete and hand the panel to the main menu.
async fn check_initialization_complete() {
    let should_show_menu = {
        let state = calibration::CALIBRATION_STATE.lock().await;

        if state.is_initialized() {
            info!("System initialization complete");
            info!("  Motor calibration: {:?}", state.motor_cal_status);
            info!("  IMU calibration: {:?}", state.imu_cal_status);
            info!("  Distance calibration: {:?}", state.distance_cal_status);
            info!("  DMP calibration: {:?}", state.dmp_cal_status);
            true
        } else {
            false
        }
    };

    if should_show_menu {
        // Boot is not a Result Report: it forgets its Activity and frees the slot.
        // Taking the claim means a repeated completion check abandons nothing.
        let boot_claim = BOOT_CLAIM.lock().await.take();
        if let Some(claim) = boot_claim {
            claim.abandon().await;
        }

        if !ui::ui_is_calibrating().await {
            ui::show_main_menu().await;
        }
    }
}
