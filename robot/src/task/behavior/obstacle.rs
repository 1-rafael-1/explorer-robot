//! Obstacle-related behavior handlers.

use defmt::info;

use crate::{
    system::state::perception,
    task::{
        drive::{InterruptKind, send_drive_interrupt},
        indicators::rgb_led_indicate::update_obstacle_indicator,
    },
};

/// Handle a change of the obstacle flag.
///
/// The event is an edge, so the handler reads the flag from the perception
/// state module rather than a copy in the payload. When the flag is set it
/// unconditionally sends an `EmergencyBrake` interrupt to the drive task. This
/// is a system-wide safety invariant — sensors are armed in all modes
/// (autonomous, testing). The interrupt brakes motors, bumps the command epoch,
/// and drains queued commands.
pub fn handle_obstacle_detected(source: crate::system::event::ObstacleSource) {
    let obstacle = perception::is_obstacle_detected();
    info!(
        "Obstacle detection status changed: source={:?} detected={}",
        source, obstacle
    );

    update_obstacle_indicator(obstacle);
    if obstacle {
        send_drive_interrupt(InterruptKind::EmergencyBrake);
    }
}

/// Handle obstacle avoidance completion.
pub fn handle_obstacle_avoidance_attempted() {
    info!("Obstacle avoidance attempted");
    update_obstacle_indicator(false);
}
