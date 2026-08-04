//! Input-related behavior handlers.
//!
//! Rotary encoder events are dispatched to the UI subsystem for navigation
//! (menu selection, button press, hold actions). RC button events were
//! removed in v3 — the only rotary encoder is the UI control surface.

#![allow(dead_code)]

use defmt::info;

use crate::{
    system::event::RotaryDirection,
    task::ui::{self, UiEvent},
};

/// Handle rotary encoder turn: update UI selection.
pub async fn handle_rotary_turned(dir: RotaryDirection) {
    info!("Rotary turned: {:?}", dir);
    ui::send_ui_event(UiEvent::RotaryTurned(dir)).await;
}

/// Handle rotary encoder button press: dispatch menu press.
pub async fn handle_rotary_button_pressed() {
    info!("Rotary button pressed");
    ui::send_ui_event(UiEvent::RotaryButtonPressed).await;
}

/// Handle rotary encoder button hold start.
pub async fn handle_rotary_button_hold_start() {
    info!("Rotary button hold started");
    ui::send_ui_event(UiEvent::RotaryButtonHoldStart).await;
}

/// Handle rotary encoder button hold end.
pub async fn handle_rotary_button_hold_end() {
    info!("Rotary button hold ended");
    ui::send_ui_event(UiEvent::RotaryButtonHoldEnd).await;
}
