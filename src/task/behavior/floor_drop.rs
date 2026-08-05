//! Floor-drop detection behavior handlers.

use defmt::info;

use crate::{
    system::state::perception,
    task::{
        drive::{InterruptKind, send_drive_interrupt},
        indicators::rgb_led_indicate::update_floor_drop_indicator,
    },
};

/// Handle floor-drop detection status changes.
///
/// Updates the `FLOOR_DROP` atomic in perception state. When a floor drop
/// is detected (e.g. top of stairs, ledge), sends an `EmergencyBrake`
/// interrupt to the drive task — this is the same safety response as an
/// obstacle, stopping the robot before it can drive off the edge.
pub async fn handle_floor_drop_detected(detected: bool) {
    let state_label = if detected { "DETECTED" } else { "CLEAR" };
    info!("Floor drop status: {}", state_label);

    perception::set_floor_drop(detected).await;
    update_floor_drop_indicator(detected);

    if detected {
        send_drive_interrupt(InterruptKind::EmergencyBrake);
    }
}
