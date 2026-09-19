//! Legacy text-display drain shim.
//!
//! The dedicated write-only `SPI1` display bus and the 240 × 240 text contract
//! are gone; the panel is now a graphics device owned by
//! [`crate::task::io::panel`]. This module exists only so the producers that
//! still publish text actions — the initialization coordinator, the test-mode
//! runners, and the rotary text UI — keep compiling while the graphics UI is
//! built. [`legacy_display_drain`] receives and discards every action; nothing
//! here draws.
//!
//! Ticket 10 deletes this shim along with the text path.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use heapless::String;

/// Text style for display rendering.
#[derive(Clone, Copy)]
pub enum TextStyle {
    /// Normal weight text.
    Normal,
    /// Bold text.
    Bold,
}

/// Display actions that can be requested by other tasks.
///
/// The payloads are never read while this shim exists: every action is drained
/// and discarded, which is why the fields are marked dead until ticket 10
/// deletes the text path.
#[allow(dead_code, clippy::large_enum_variant)]
pub enum DisplayAction {
    /// Show a text message on the display (bold style).
    ShowText(String<MAX_LINE_LEN>, u8),
    /// Show a text message with an explicit style.
    ShowTextStyled(String<MAX_LINE_LEN>, u8, TextStyle),
    /// Show all text lines in a single update.
    ShowLines([String<MAX_LINE_LEN>; DISPLAY_LINES]),
    /// Clear the entire display.
    Clear,
}

/// Control channel for display update requests.
static DISPLAY_CHANNEL: Channel<CriticalSectionRawMutex, DisplayAction, 16> = Channel::new();

/// Number of text lines the retired display could show.
pub const DISPLAY_LINES: usize = 14;

/// Maximum line length in characters supported by the retired text contract.
pub const MAX_LINE_LEN: usize = 26;

/// Request a display update — blocks until the action is queued.
pub async fn display_update(display_action: DisplayAction) {
    DISPLAY_CHANNEL.send(display_action).await;
}

/// Try to request a display update without blocking.
/// Returns true if the update was queued.
pub fn display_try_update(display_action: DisplayAction) -> bool {
    DISPLAY_CHANNEL.sender().try_send(display_action).is_ok()
}

/// Receive and discard legacy text-display actions.
///
/// Draining keeps the channel from filling, so the producers that still call
/// [`display_update`] never block on a panel that no longer renders text.
#[embassy_executor::task]
pub async fn legacy_display_drain() {
    loop {
        let _ = DISPLAY_CHANNEL.receive().await;
    }
}
