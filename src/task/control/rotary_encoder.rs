//! EC11 rotary encoder handling (PIO quadrature + button input)
//!
//! Produces events for:
//! - Rotary turn increments/decrements
//! - Button press / hold start / hold end
//!
//! Pin plan:
//! - GPIO22: Encoder A
//! - GPIO23: Encoder B
//! - Button: direct GPIO Input with pull-up (pin assigned by caller in main.rs)
//!
//! # v3 changes from v2
//! - Button moved from port expander (signal-based) to direct GPIO Input with `Pull::Up`.
//! - Debounce pattern uses `wait_for_low` / `wait_for_high` on the GPIO pin directly.
//! - Same press-vs-hold detection logic and event types preserved.

use defmt::debug;
use embassy_futures::select::{Either, select};
use embassy_rp::{
    gpio::Input,
    pio_programs::rotary_encoder::{Direction as PioDirection, PioEncoder},
};
use embassy_time::{Duration, Instant, Timer};

use crate::system::event::{Events, RotaryDirection, raise_event};

/// Button hold threshold (ms)
const HOLD_DURATION: Duration = Duration::from_millis(700);

/// Rotary turn debounce window (ms)
const TURN_DEBOUNCE_MS: u64 = 50;

/// Button debounce delay (ms)
const DEBOUNCE_DURATION: Duration = Duration::from_millis(30);

/// Task that reads quadrature turns and emits increment/decrement events.
#[embassy_executor::task]
pub async fn rotary_encoder_turns(mut encoder: PioEncoder<'static, embassy_rp::peripherals::PIO1, 3>) {
    let mut last_turn_ms: u64 = 0;

    loop {
        let dir = encoder.read().await;
        let now_ms = Instant::now().as_millis();
        if now_ms.saturating_sub(last_turn_ms) < TURN_DEBOUNCE_MS {
            continue;
        }
        last_turn_ms = now_ms;

        let event = match dir {
            PioDirection::Clockwise => {
                debug!("Rotary encoder turned clockwise");
                Events::RotaryTurned(RotaryDirection::Clockwise)
            }
            PioDirection::CounterClockwise => {
                debug!("Rotary encoder turned counter-clockwise");
                Events::RotaryTurned(RotaryDirection::CounterClockwise)
            }
        };
        raise_event(event).await;
    }
}

/// Task that handles the EC11 button (active-low, pull-up enabled).
///
/// The button pin must be configured as `Input` with `Pull::Up` by the caller.
#[embassy_executor::task]
pub async fn rotary_encoder_button(mut button: Input<'static>) {
    loop {
        // Wait for button press (active-low: pin pulled to ground).
        button.wait_for_low().await;

        // Debounce: short delay then verify stable low.
        Timer::after(DEBOUNCE_DURATION).await;
        if button.is_high() {
            // Glitch — ignore and wait for next press.
            continue;
        }

        debug!("Rotary encoder button pressed");

        // Race: hold-duration timer vs button release.
        match select(Timer::after(HOLD_DURATION), button.wait_for_high()).await {
            Either::First(()) => {
                // Hold detected — button still low after HOLD_DURATION.
                debug!("Rotary encoder button hold start");
                raise_event(Events::RotaryButtonHoldStart).await;

                // Wait for release.
                button.wait_for_high().await;

                debug!("Rotary encoder button hold end");
                raise_event(Events::RotaryButtonHoldEnd).await;
            }
            Either::Second(()) => {
                // Released before hold duration → short press.
                debug!("Rotary encoder button short press");
                raise_event(Events::RotaryButtonPressed).await;
            }
        }
    }
}
