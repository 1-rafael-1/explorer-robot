//! The named interactive regions and the header action button.
//!
//! [`Hit`] is what hit-testing returns and what the interaction model acts on.
//! [`HeaderAction`] is the header's single left button, which is Back on the
//! list-like screens, Cancel on the value screens, and Stop on a running
//! screen.

use embedded_graphics::primitives::Rectangle;

use crate::{
    geometry::{back_button_rect, cancel_button_rect},
    screens::Item,
};

/// An interactive region named by hit-testing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hit {
    /// The header Back button (present on every list-like screen except the
    /// Main Menu).
    Back,
    /// The list item this region names on the current screen, by identity.
    MenuItem(Item),
    /// The header Cancel button on a value-entry screen.
    Cancel,
    /// The header Stop button on a running/status screen.
    Stop,
    /// The footer Save button on a value-entry screen.
    Save,
    /// The draggable slider track (and its thumb) on a value-entry screen.
    Slider,
    /// The fine-adjust decrement button on a value-entry screen.
    NudgeMinus,
    /// The fine-adjust increment button on a value-entry screen.
    NudgePlus,
}

/// The header's single left action button.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeaderAction {
    /// Return to the parent screen.
    Back,
    /// Abandon the value screen without saving.
    Cancel,
    /// Stop the running procedure and return to its parent.
    Stop,
}

impl HeaderAction {
    /// The button's label.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Back => "Back",
            Self::Cancel => "Cancel",
            Self::Stop => "Stop",
        }
    }

    /// The button's on-screen rectangle.
    #[must_use]
    pub const fn rect(self) -> Rectangle {
        match self {
            Self::Cancel => cancel_button_rect(),
            Self::Back | Self::Stop => back_button_rect(),
        }
    }

    /// The [`Hit`] a tap inside [`HeaderAction::rect`] reports.
    #[must_use]
    pub const fn hit(self) -> Hit {
        match self {
            Self::Back => Hit::Back,
            Self::Cancel => Hit::Cancel,
            Self::Stop => Hit::Stop,
        }
    }
}
