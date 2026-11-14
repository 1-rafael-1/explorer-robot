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
/// The rangefinder task is the sole writer of the `FLOOR_DROP` flag, so this
/// handler reads the live value rather than writing the event payload back. A
/// set flag updates the indicator and sends an `EmergencyBrake` interrupt to the
/// drive task — the same safety response as an obstacle, stopping the robot
/// before it can drive off the edge. A floor drop brakes in every mode.
pub fn handle_floor_drop_detected(_detected: bool) {
    let detected = perception::is_floor_drop_detected();
    let state_label = if detected { "DETECTED" } else { "CLEAR" };
    info!("Floor drop status: {}", state_label);

    update_floor_drop_indicator(detected);

    if detected {
        send_drive_interrupt(InterruptKind::EmergencyBrake);
    }
}
