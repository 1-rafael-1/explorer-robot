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
        io::flash_storage::{self, CalibrationDataKind, CalibrationKind},
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
        CalibrationKind::Motor => report_motor_calibration(data).await,
        CalibrationKind::Distance => load_distance_calibration(data).await,
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

/// Report the boot phase for the Motor Calibration Curve the flash task
/// already decoded and published.
///
/// The flash task owns the record bytes and publishes the derived curve to the
/// calibration state leaf in the same step, so there is no second copy of what
/// is applied here. This function only names the phase: a valid record loaded,
/// or the identity curve is in force.
async fn report_motor_calibration(data: Option<CalibrationDataKind>) {
    if let Some(CalibrationDataKind::Motor(summary)) = data {
        info!(
            "Motor calibration curve loaded: run={} floor={}%",
            summary.run_id, summary.floor_percent
        );
        publish_phase("Calibration loaded", None).await;
    } else {
        info!("No motor calibration curve found - identity curve in force");
        publish_phase("Need calibration", None).await;
    }
}

/// Report the Distance Calibration scale the flash task already published.
///
/// The scale lives in the one Motor Calibration record, so the flash task read it
/// with the record and published it to the calibration leaf. This function only
/// names the phase: a measured scale loaded, or the nominal fallback is in force.
async fn load_distance_calibration(data: Option<CalibrationDataKind>) {
    if let Some(CalibrationDataKind::Distance(counts_per_cm)) = data {
        info!("Distance calibration loaded: counts_per_cm={}", counts_per_cm);
    } else {
        info!("No distance calibration found - nominal scale in force");
        publish_phase("Need calibration", None).await;
    }
}

/// Check if initialization is complete and hand the panel to the main menu.
async fn check_initialization_complete() {
    let should_show_menu = {
        let state = calibration::CALIBRATION_STATE.lock().await;

        if state.is_initialized() {
            info!("System initialization complete");
            info!("  Motor calibration: {:?}", state.motor_cal_status);
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
