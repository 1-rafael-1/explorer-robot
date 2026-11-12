//! The named interactive regions and the header action button.
//!
//! [`Hit`] is what hit-testing returns and what the interaction model acts on.
//! [`HeaderAction`] is the header's single left button, which is Back on the
//! list-like screens, Cancel on the value screens, and Stop on a running
//! screen.

use embedded_graphics::{prelude::Point, primitives::Rectangle};

use crate::{
    geometry::{back_button_rect, cancel_button_rect, link_action_rect, prompt_done_rect, prompt_redo_rect},
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
    /// One of the Link screen's Bridge Control action buttons.
    LinkAction(LinkAction),
    /// One of a prompt screen's Done/Redo action buttons.
    PromptAction(PromptAction),
}

/// A Prompt screen action offered to the operator.
///
/// The actions are neutral: the panel names what the operator asked for and the
/// firmware decides what it means for the running Procedure. Activating one is
/// reported through [`crate::Ui::last_activation`] and does not itself
/// navigate, like [`LinkAction`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PromptAction {
    /// Accept the current step and advance.
    Done,
    /// Restart the procedure's movement sequence.
    Redo,
}

impl PromptAction {
    /// Every action, in screen order.
    pub const ALL: [Self; 2] = [Self::Done, Self::Redo];

    /// The button label for this action.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Done => "Done",
            Self::Redo => "Redo",
        }
    }

    /// The button's on-screen rectangle.
    #[must_use]
    pub const fn rect(self) -> Rectangle {
        match self {
            Self::Done => prompt_done_rect(),
            Self::Redo => prompt_redo_rect(),
        }
    }
}

/// The prompt action whose rectangle contains `p`, if that action is available.
///
/// A disabled action (the movement steps have no Redo) is not hittable, so a tap
/// on its rectangle reports nothing. The rectangles come from [`crate::geometry`],
/// so this and the drawing pass cannot disagree about where an action is.
#[must_use]
pub fn prompt_action_hit(p: Point, done: bool, redo: bool) -> Option<PromptAction> {
    PromptAction::ALL.into_iter().find(|action| match action {
        PromptAction::Done => done && prompt_done_rect().contains(p),
        PromptAction::Redo => redo && prompt_redo_rect().contains(p),
    })
}

/// A Bridge Control action offered on the Link screen.
///
/// The actions are neutral: the panel names what the operator asked for and the
/// firmware maps it onto the wire, so this type carries no protocol value.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LinkAction {
    /// Forget the stored network and raise Setup Mode.
    ForgetNetwork,
    /// Show the existing pairing code without rotating it.
    ShowPairingCode,
    /// Rotate the pairing code and return the new one.
    RotatePairingCode,
}

impl LinkAction {
    /// Every action, in screen order.
    pub const ALL: [Self; 3] = [Self::ForgetNetwork, Self::ShowPairingCode, Self::RotatePairingCode];

    /// The button label for this action.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::ForgetNetwork => "Forget Wi-Fi & set up",
            Self::ShowPairingCode => "Show pairing code",
            Self::RotatePairingCode => "Rotate pairing code",
        }
    }
}

/// The Link screen's action whose rectangle contains `p`, if any.
///
/// The rectangles come from [`crate::geometry`], so this and the drawing pass
/// cannot disagree about where an action is.
#[must_use]
pub fn link_action_hit(p: Point) -> Option<LinkAction> {
    LinkAction::ALL
        .into_iter()
        .enumerate()
        .find(|(index, _)| link_action_rect(*index).contains(p))
        .map(|(_, action)| action)
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
