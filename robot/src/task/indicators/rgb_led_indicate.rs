//! RGB LED indication for system status
//!
//! Provides visual feedback about system state through an RGB LED:
//!
//! # Battery Indication
//! - Solid color indicating pack voltage
//!   - Green at or above 7.7 V, yellow at 6.7 V, red at or below 6.2 V
//!   - Green holds above the green anchor, then shifts continuously; the blend
//!     has no step at either end
//!   - A normalized mix bias compensates for green being perceptually brighter
//!   - The curve lives in the `indicator-led` crate so its anchors are
//!     host-tested
//!
//! # State Change Indication
//! - Quick alternating red/green blink sequence (5 blinks)
//! - 30ms interval for noticeable but brief feedback
//!
//! # Obstacle Indication
//! - Solid red blink pattern when an obstacle is detected
//!
//! # Floor-Drop Indication
//! - Amber blink pattern when a floor drop / ledge is detected
//!
//! # PWM Control
//! - 100Hz PWM frequency for flicker-free operation
//! - Independent control of red, green, and blue channels via PIO PWM
//! - Duty cycle derived from pack voltage, with a perceptual red/green mix
//! - Smooth transitions between states

use core::time::Duration as CoreDuration;

use embassy_rp::{peripherals::PIO1, pio_programs::pwm::PioPwm};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use indicator_led::battery_color;

use crate::system::state::power;

/// Indicator events used to trigger LED state updates
pub enum IndicatorEvent {
    /// Trigger state change confirmation blink sequence
    Affirm(bool),
    /// Obstacle status update (true = obstacle detected)
    Obstacle(bool),
    /// Floor-drop status update (true = floor drop / ledge detected)
    FloorDrop(bool),
}

/// Signal for triggering LED state updates
pub static INDICATOR_CHANGED: Signal<CriticalSectionRawMutex, IndicatorEvent> = Signal::new();

/// Triggers an LED indicator state update
///
/// - affirm: true to show state change confirmation blink sequence
pub fn update_indicator(affirm: bool) {
    INDICATOR_CHANGED.signal(IndicatorEvent::Affirm(affirm));
}

/// Triggers an LED obstacle status update
///
/// - detected: true for obstacle present, false for clear
pub fn update_obstacle_indicator(detected: bool) {
    INDICATOR_CHANGED.signal(IndicatorEvent::Obstacle(detected));
}

/// Triggers an LED floor-drop status update
///
/// - detected: true for floor drop / ledge present, false for clear
pub fn update_floor_drop_indicator(detected: bool) {
    INDICATOR_CHANGED.signal(IndicatorEvent::FloorDrop(detected));
}

/// Waits for next indicator state change signal
async fn wait() -> IndicatorEvent {
    INDICATOR_CHANGED.wait().await
}

/// Interval for state change confirmation blinks (30ms is quick but noticeable)
const AFFIRM_BLINK_INTERVAL: Duration = Duration::from_millis(30);
/// Interval for obstacle status blinks (short burst)
const OBSTACLE_BLINK_INTERVAL: Duration = Duration::from_millis(60);

/// PWM period for the PIO-driven RGB LED (100 Hz).
const PWM_PERIOD: CoreDuration = CoreDuration::from_millis(10);

/// Sets a PIO PWM channel to a duty cycle percentage.
#[allow(clippy::cast_possible_truncation)]
fn set_pwm_percent<const SM: usize>(pwm: &mut PioPwm<'static, PIO1, SM>, percent: u8) {
    let clamped = u128::from(percent.min(100));
    let high_us = PWM_PERIOD.as_micros() * clamped / 100;
    let high = CoreDuration::from_micros(high_us as u64);
    pwm.write(high);
}

/// Sets all RGB channels in one call.
fn set_rgb(
    pwm_red: &mut PioPwm<'static, PIO1, 0>,
    pwm_green: &mut PioPwm<'static, PIO1, 1>,
    pwm_blue: &mut PioPwm<'static, PIO1, 2>,
    red: u8,
    green: u8,
    blue: u8,
) {
    set_pwm_percent(pwm_red, red);
    set_pwm_percent(pwm_green, green);
    set_pwm_percent(pwm_blue, blue);
}

/// Main LED control task that manages visual feedback based on system state
///
/// Uses PIO PWM to control LED brightness and color:
/// - Red channel indicates low battery
/// - Green channel indicates high battery
/// - Blue channel reserved for future status modes
/// - Mixing creates intermediate colors
#[allow(clippy::cast_lossless, clippy::cast_possible_truncation)]
#[embassy_executor::task]
pub async fn rgb_led_indicate(
    mut pwm_red: PioPwm<'static, PIO1, 0>,
    mut pwm_green: PioPwm<'static, PIO1, 1>,
    mut pwm_blue: PioPwm<'static, PIO1, 2>,
) {
    pwm_red.set_period(PWM_PERIOD);
    pwm_green.set_period(PWM_PERIOD);
    pwm_blue.set_period(PWM_PERIOD);

    pwm_red.start();
    pwm_green.start();
    pwm_blue.start();

    // Start with LED off
    let mut led_on = false;
    set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 0, 0);

    loop {
        let event = wait().await;

        let mut affirm = false;

        match event {
            IndicatorEvent::Affirm(value) => {
                affirm = value;
            }
            IndicatorEvent::Obstacle(detected) => {
                let (base_red, base_green, base_blue) = if detected { (100, 0, 0) } else { (0, 0, 100) };

                for _ in 0..5 {
                    if led_on {
                        set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 100, 100, 100);
                    } else {
                        set_rgb(
                            &mut pwm_red,
                            &mut pwm_green,
                            &mut pwm_blue,
                            base_red,
                            base_green,
                            base_blue,
                        );
                    }

                    led_on = !led_on;
                    Timer::after(OBSTACLE_BLINK_INTERVAL).await;
                }

                led_on = false;
            }
            IndicatorEvent::FloorDrop(detected) => {
                if detected {
                    // Amber blink: alternate between amber (red + green) and off.
                    for _ in 0..5 {
                        if led_on {
                            set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 100, 50, 0);
                        } else {
                            set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 0, 0);
                        }

                        led_on = !led_on;
                        Timer::after(OBSTACLE_BLINK_INTERVAL).await;
                    }
                } else {
                    // Floor drop cleared: turn LED off.
                    set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 0, 0);
                }

                led_on = false;
            }
        }

        // Show state change confirmation sequence if requested
        if affirm {
            for _ in 0..5 {
                if led_on {
                    set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 100, 0);
                } else {
                    set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 100, 0, 0);
                }
                led_on = !led_on;
                Timer::after(AFFIRM_BLINK_INTERVAL).await;
            }
            led_on = false;
            set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 0, 0);
        }

        // Get the current pack voltage. Level and voltage are published
        // together, so a missing or non-finite voltage means no usable battery
        // info; show the same blue as the pre-measurement state.
        let battery = power::get_battery_snapshot().await;
        let Some(voltage) = battery.voltage.filter(|voltage| voltage.is_finite()) else {
            // No usable battery info: solid blue
            set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, 0, 0, 100);
            continue;
        };

        let color = battery_color(voltage);
        set_rgb(&mut pwm_red, &mut pwm_green, &mut pwm_blue, color.red, color.green, 0);
    }
}
