//! Obstacle-related behavior handlers.

use defmt::info;

use crate::{
    system::state::perception,
    task::{
        drive::{InterruptKind, send_drive_interrupt},
        indicators::rgb_led_indicate::update_obstacle_indicator,
    },
};

/// Reset obstacle detection state and clear all perception data.
pub async fn reset_obstacle_state() {
    perception::set_lidar_obstacle(false).await;
    update_obstacle_indicator(false);
}

/// Handle obstacle detection status changes.
///
/// Updates perception atomics and unconditionally sends an `EmergencyBrake`
/// interrupt to the drive task. This is a system-wide safety invariant — sensors
/// are armed in all modes (autonomous, testing). The interrupt brakes motors,
/// bumps the command epoch, and drains queued commands.
pub async fn handle_obstacle_detected(source: crate::system::event::ObstacleSource, detected: bool) {
    info!(
        "Obstacle detection status changed: source={:?} detected={}",
        source, detected
    );

    perception::set_lidar_obstacle(detected).await;

    let obstacle = perception::lidar_obstacle().await;
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
