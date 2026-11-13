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
//! - Reads the protected `+8V` rail through the ADC every 5 seconds.
//! - The 33k/10k divider scales the rail into the ADC range.
//! - A bench-calibrated scale corrects the divider/reference gain error.
//! - The pack-terminal voltage is derived from the rail with an idle-load
//!   drop model.
//! - Initial 500ms delay ensures system stabilization.
//!
//! # Hardware Configuration
//! ```text
//! +8V (protected) ----[33kΩ]----+----[10kΩ]---- GND
//!                               |
//!                        GPIO40 (ADC0)
//!                               |
//!                            [100nF]
//!                               |
//!                              GND
//!
//! Voltage Divider Ratio: 10k/(33k+10k) = 0.2326
//! Source Impedance: 33k||10k = ~7.7kΩ (suitable for ADC)
//! At 8.4V rail: ADC sees ~1.95V (within safe range)
//! At 6.0V rail: ADC sees ~1.40V
//! Current draw: ~195µA at 8.4V
//! ```
//!
//! # Pin Connection
//! - ADC Pin: GPIO40 (ADC0) — RP2350B variant ADC pin
//! - Voltage Source: protected `+8V` rail (TP12), the TB6612FNG `VM` node
//! - The voltage divider scales the rail to the ADC input range (< 3.3V)
//!
//! # Voltage Calculations
//! ```text
//! Rail = (ADC Value * 3.3V) / (4096 * 0.2326) * 1.0036
//! Pack = Rail + 0.372 / Rail
//! Where:
//! - 3.3V is the nominal ADC reference (measured 3.295V on the bench)
//! - 0.2326 is the nominal voltage divider ratio (low/(high+low))
//! - 1.0036 is the bench-measured gain correction
//! - 0.372 V² is the idle-load cable + reverse-polarity FET drop coefficient
//! - 4096 is ADC resolution (12-bit)
//! ```
//!
//! The scale and drop constant were measured at 6.0V, 8.0V and 8.4V against
//! TP12 and the pack terminals; they reproduce all three points within ~2mV.
//! The drop term is a constant-power model (`I = P / Rail`), not a fixed
//! offset. Motor current adds more drop, so the displayed pack voltage is the
//! loaded pack voltage rather than an open-circuit estimate.
//!
//! # Charge Level Calculation
//! - Maps pack voltage range 6.0V-8.4V to 0%-99%
//! - Linear interpolation between min/max voltages
//! - Caps at 99% to indicate charging might still occur
//! - Reports 0% at or below 6.0V (warning level)
//!
//! # Consumers
//! - The motor driver uses the rail voltage for 6V compensation.
//! - The panel and Link telemetry use the pack-terminal voltage.
//! - The charge percentage uses the pack-terminal voltage.

use embassy_rp::{
    adc::{Adc, Channel},
    mode::Async as AdcAsync,
};
use embassy_time::{Duration, Timer};

use crate::system::{event, state::power};

/// Time between voltage measurements (5s provides fast response for motor control)
const MEASUREMENT_INTERVAL: Duration = Duration::from_secs(5);

/// Minimum battery voltage for 2S Li-Ion pack (6.0V is safe cutoff)
const BATTERY_VOLTAGE_LOWER: f32 = 6.0;

/// Maximum battery voltage for 2S Li-Ion pack (8.4V is full charge, 4.2V per cell)
const BATTERY_VOLTAGE_UPPER: f32 = 8.4;

/// ADC reference voltage (nominal 3.3V; the calibration scale absorbs the
/// measured 3.295V rail and divider tolerance).
const REF_VOLTAGE: f32 = 3.3;

/// Higher resistor value for voltage divider
const DIVIDER_R_HIGH_OHMS: f32 = 33_000.0;
/// Lower resistor value for voltage divider
const DIVIDER_R_LOW_OHMS: f32 = 10_000.0;
/// Voltage divider ratio calculation
const V_DIVIDER_RATIO: f32 = DIVIDER_R_LOW_OHMS / (DIVIDER_R_HIGH_OHMS + DIVIDER_R_LOW_OHMS);

/// ADC resolution (12-bit = 4096 steps)
const ADC_RANGE: f32 = 4096.0;

/// Gain calibration for the divider and ADC reference.
///
/// Bench-measured against TP12 (`+8V`) at 6.0V, 8.0V and 8.4V with the panel
/// and `LiDAR` powered: the actual conversion is 3.4769 mV per ADC count against
/// the nominal 3.4644 mV. The divider is about 0.52% low; the 3.295V reference
/// partly offsets that. This is a per-board bench value; a production unit
/// needs its own calibration or tighter divider/reference tolerances.
const VOLTAGE_CALIBRATION_SCALE: f32 = 1.0036;

/// Constant-power coefficient for the pack-terminal drop (`V²`).
///
/// The pack terminals sit above the `+8V` rail by the cable and
/// reverse-polarity FET drops. With the motors idle the board load is roughly
/// constant power, so the drop is `I * R = P * R / Rail`; `P * R` is this
/// coefficient. Bench points (pack - rail): 63mV at a 5.937V rail, 48mV at
/// 7.955V, and 43mV at 8.357V. `0.372 / Rail` reproduces all three within ~2mV.
const PACK_DROP_K: f32 = 0.372;

/// Battery monitoring task that continuously measures voltage and
/// reports charge level as a percentage.
///
/// Writes the pack and rail voltages to the power state module: the motor
/// driver reads the rail, while the panel and Link telemetry read the pack
/// voltage.
#[embassy_executor::task]
pub async fn battery_charge_read(mut adc: Adc<'static, AdcAsync>, mut channel: Channel<'static>) {
    // Initial delay to ensure system stabilization before first reading
    Timer::after(Duration::from_millis(500)).await;

    loop {
        // Read the ADC and convert it to the protected `+8V` rail voltage.
        // Formula: (adc_value * reference_voltage) / (adc_resolution * divider_ratio)
        // The divider brings the rail down, so we divide by the ratio to
        // recover the actual rail voltage.
        let adc_raw = adc.read(&mut channel).await.unwrap_or(0);
        let rail_voltage = f32::from(adc_raw) * REF_VOLTAGE / (ADC_RANGE * V_DIVIDER_RATIO) * VOLTAGE_CALIBRATION_SCALE;

        // The pack terminals sit above the rail by the cable and reverse-
        // polarity FET drops. The idle load is roughly constant power, so the
        // drop is K / rail. Guard the division so a failed ADC read (raw = 0)
        // cannot produce infinity.
        let pack_drop = if rail_voltage > 0.0 {
            PACK_DROP_K / rail_voltage
        } else {
            0.0
        };
        let battery_voltage = rail_voltage + pack_drop;

        // Calculate battery charge percentage:
        // - 99% if voltage >= max voltage (avoiding 100% to indicate charging might still occur)
        // - 0% if voltage <= min voltage
        // - Linear interpolation between min and max otherwise
        let battery_level = if battery_voltage >= BATTERY_VOLTAGE_UPPER {
            0.99
        } else if battery_voltage <= BATTERY_VOLTAGE_LOWER {
            0.0
        } else {
            (battery_voltage - BATTERY_VOLTAGE_LOWER) / (BATTERY_VOLTAGE_UPPER - BATTERY_VOLTAGE_LOWER)
        };

        let battery_percent_f32: f32 = (battery_level * 100.0).clamp(0.0, 100.0);

        // Convert to integer percentage for events (0..=100).
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let battery_percent: u8 = battery_percent_f32 as u8;

        defmt::debug!(
            "Battery: ADC raw={}, rail={}V, pack={}V, level={}%",
            adc_raw,
            rail_voltage,
            battery_voltage,
            battery_percent_f32
        );

        // Write the full measurement so the motor driver can read the rail
        // without going through the event system.
        power::set_measurement(battery_percent, battery_voltage, rail_voltage).await;

        // Send consolidated battery measurement event
        // Battery monitoring is critical for safety - must not drop events
        event::raise_event(event::Events::BatteryMeasured {
            level: battery_percent,
            voltage: battery_voltage,
        })
        .await;

        // Wait for next measurement interval
        Timer::after(MEASUREMENT_INTERVAL).await;
    }
}
