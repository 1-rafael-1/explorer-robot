//! Battery-related behavior handlers.

use defmt::debug;

use crate::{system::state::power, task::indicators::rgb_led_indicate};

/// Handle battery measurement (level and voltage).
pub async fn handle_battery_measured(level: u8, voltage: f32) {
    debug!("Battery measured: level={}%, pack={}V", level, voltage);

    // The battery task already wrote the full measurement, including the rail
    // voltage. This refresh only carries the operator-facing values.
    power::set_battery(level, voltage).await;

    rgb_led_indicate::update_indicator(false);
}
