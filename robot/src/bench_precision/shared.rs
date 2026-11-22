//! Boot-phase helpers shared by the calibrate and measure bench phases.

use embassy_time::{Duration, Timer};

use crate::system::state::calibration;

/// Wait for the boot calibration load to publish its first result.
pub(super) async fn wait_for_initialization() {
    while !calibration::is_initialized().await {
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Let the boot flow release its claim and the panel reach the menu.
pub(super) async fn settle_at_menu() {
    Timer::after(Duration::from_secs(3)).await;
}

/// Sleep forever so the finished bench boot keeps its final screen.
pub(super) async fn hold() -> ! {
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}
