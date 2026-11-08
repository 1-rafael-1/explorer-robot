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
/// The event is an edge and carries the value the producer measured, which is
/// logged. The brake decision still reads the live perception flag: the safety
/// edge must reflect the current state, not a queued edge whose successor has
/// already cleared it. When the flag is set the handler unconditionally sends an
/// `EmergencyBrake` interrupt to the drive task. This is a system-wide safety
/// invariant — sensors are armed in all modes (autonomous, testing). The
/// interrupt brakes motors, bumps the command epoch, and drains queued commands.
pub fn handle_obstacle_detected(source: crate::system::event::ObstacleSource, detected: bool) {
    let obstacle = perception::is_obstacle_detected();
    info!(
        "Obstacle detection status changed: source={:?} detected={} live={}",
        source, detected, obstacle
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
