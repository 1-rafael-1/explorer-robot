//! Battery charge monitoring
//!
//! Monitors 2S Li-Ion battery pack voltage (two 18650 cells in series).
//!
//! # Battery Specifications
//! - Configuration: 2S (two cells in series)
//! - Voltage range: 8.4V (fully charged) to 6.0V (cutoff)
//! - Cell chemistry: Li-Ion 18650
//!
//! # Measurement Strategy
//! - Reads voltage through ADC every 5 seconds (faster for motor control responsiveness)
//! - Uses voltage divider to scale battery voltage to ADC range
//! - Initial 500ms delay ensures system stabilization
//!
//! # Hardware Configuration
//! ```text
//! +8V (protected) ----[33kΩ]----+----[10kΩ]---- GND
//!                               |
//!                        GPIO40 (ADC0)
//!
//! Voltage Divider Ratio: 10k/(33k+10k) = 0.2326
//! Source Impedance: 33k||10k = ~7.7kΩ (suitable for ADC)
//! At 8.4V battery: ADC sees 1.95V (within safe range)
//! At 6.0V battery: ADC sees 1.40V
//! Current draw: ~195µA at 8.4V
//! ```
//!
//! # Pin Connection
//! - ADC Pin: GPIO40 (ADC0) — RP2350B variant ADC pin
//! - Voltage Source: Battery pack before voltage regulator (6V-8.4V range)
//! - The voltage divider scales battery voltage to ADC input range (< 3.3V)
//!
//! # Voltage Calculations
//! ```text
//! Battery Voltage = (ADC Value * 3.3V) / (4096 * 0.2326)
//! Where:
//! - 3.3V is ADC reference voltage
//! - 0.2326 is the voltage divider ratio (low/(high+low))
//! - 4096 is ADC resolution (12-bit)
//! ```
//!
//! # Charge Level Calculation
//! - Maps voltage range 6.0V-8.4V to 0%-99%
//! - Linear interpolation between min/max voltages
//! - Caps at 99% to indicate charging might still occur
//! - Reports 0% at or below 6.0V (warning level)
//!
//! # Motor Driver Integration
//! - Battery voltage is shared with motor driver for voltage compensation
//! - Motor driver uses this to maintain consistent 6V output as battery drains

use embassy_rp::adc::{Adc, Async as AdcAsync, Channel};
use embassy_time::{Duration, Timer};

use crate::system::{event, state::power};

/// Time between voltage measurements (5s provides fast response for motor control)
const MEASUREMENT_INTERVAL: Duration = Duration::from_secs(5);

/// Minimum battery voltage for 2S Li-Ion pack (6.0V is safe cutoff)
const BATTERY_VOLTAGE_LOWER: f32 = 6.0;

/// Maximum battery voltage for 2S Li-Ion pack (8.4V is full charge, 4.2V per cell)
const BATTERY_VOLTAGE_UPPER: f32 = 8.4;

/// ADC reference voltage (3.3V is RP2350's reference)
const REF_VOLTAGE: f32 = 3.3;

/// Higher resistor value for voltage divider
const DIVIDER_R_HIGH_OHMS: f32 = 33_000.0;
/// Lower resistor value for voltage divider
const DIVIDER_R_LOW_OHMS: f32 = 10_000.0;
/// Voltage divider ratio calculation
const V_DIVIDER_RATIO: f32 = DIVIDER_R_LOW_OHMS / (DIVIDER_R_HIGH_OHMS + DIVIDER_R_LOW_OHMS);

/// ADC resolution (12-bit = 4096 steps)
const ADC_RANGE: f32 = 4096.0;

/// Voltage calibration offset (V)
/// Empirically determined with the previous (20 kΩ/10 kΩ) divider; re-measure
/// after the divider change rather than assuming this value still applies.
const VOLTAGE_CALIBRATION_OFFSET: f32 = -0.02;

/// Battery monitoring task that continuously measures voltage and
/// reports charge level as a percentage.
///
/// Also writes the measurement to the power state module so the motor
/// driver can read it for voltage compensation without going through
/// the event system.
#[embassy_executor::task]
pub async fn battery_charge_read(mut adc: Adc<'static, AdcAsync>, mut channel: Channel<'static>) {
    // Initial delay to ensure system stabilization before first reading
    Timer::after(Duration::from_millis(500)).await;

    loop {
        // Read ADC value and convert to battery voltage
        // Formula: (adc_value * reference_voltage) / (adc_resolution * voltage_divider_ratio)
        // The voltage divider brings battery voltage down, so we divide by the ratio to get actual voltage
        let adc_raw = adc.read(&mut channel).await.unwrap_or(0);
        let voltage = f32::from(adc_raw) * REF_VOLTAGE / (ADC_RANGE * V_DIVIDER_RATIO) + VOLTAGE_CALIBRATION_OFFSET;

        // Calculate battery charge percentage:
        // - 99% if voltage >= max voltage (avoiding 100% to indicate charging might still occur)
        // - 0% if voltage <= min voltage
        // - Linear interpolation between min and max otherwise
        let battery_level = if voltage >= BATTERY_VOLTAGE_UPPER {
            0.99
        } else if voltage <= BATTERY_VOLTAGE_LOWER {
            0.0
        } else {
            (voltage - BATTERY_VOLTAGE_LOWER) / (BATTERY_VOLTAGE_UPPER - BATTERY_VOLTAGE_LOWER)
        };

        let battery_percent_f32: f32 = (battery_level * 100.0).clamp(0.0, 100.0);

        // Convert to integer percentage for events (0..=100).
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let battery_percent: u8 = battery_percent_f32 as u8;

        defmt::debug!(
            "Battery: ADC raw={}, voltage={}V, level={}%",
            adc_raw,
            voltage,
            battery_percent_f32
        );

        // Write to power state so the motor driver can read voltage without events
        power::set_battery(battery_percent, voltage).await;

        // Send consolidated battery measurement event
        // Battery monitoring is critical for safety - must not drop events
        event::raise_event(event::Events::BatteryMeasured {
            level: battery_percent,
            voltage,
        })
        .await;

        // Wait for next measurement interval
        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
