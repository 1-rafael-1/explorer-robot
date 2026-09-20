//! On-screen layout geometry, shared by hit-testing and rendering.
//!
//! Every rectangle a tap can land on is defined here once, so the hit test and
//! the drawing pass cannot disagree. Coordinates are framebuffer pixels in the
//! panel's landscape orientation.
//!
//! The layout is a port of the bench feeler: a tall titled header, a column of
//! wide buttons inset from the edges, a right-edge scroll indicator, and a
//! value screen with a readout, a slider, fine-adjust buttons, and a footer.

use embedded_graphics::{
    prelude::{Point, Size},
    primitives::Rectangle,
};

use crate::screens::ValueFlow;

/// Framebuffer width in pixels (landscape after the orientation transform).
pub const FB_W: usize = 320;

/// Framebuffer height in pixels.
pub const FB_H: usize = 240;

/// Height of the title header at the top of the screen, in pixels.
///
/// Deliberately tall: it holds a comfortably sized Back/Cancel target below the
/// panel's bezel-adjacent top edge.
pub const HEADER_H: u32 = 48;

/// Height of one list button, in pixels.
pub const BUTTON_H: u32 = 38;

/// Vertical gap between adjacent buttons, in pixels (also the gap below the
/// header).
pub const BUTTON_GAP: u32 = 8;

/// Horizontal inset of the button column from the screen edges, in pixels.
///
/// Wide enough to leave a clear gutter for the 25 px scroll indicator.
pub const SIDE_MARGIN: u32 = 32;

/// Inner horizontal padding between a button's edge and its label, in pixels.
pub const BUTTON_PAD: u32 = 12;

/// Stroke width of a button's outline, in pixels.
pub const BUTTON_BORDER_W: u32 = 1;

/// Height of the header divider line, in pixels.
pub const DIVIDER_H: u32 = 1;

/// Width of the header Back (and Stop) button, in pixels.
pub const BACK_W: u32 = 84;

/// Vertical inset of the header action button from the header's top and bottom,
/// in pixels.
pub const BACK_MARGIN: u32 = 12;

/// Horizontal gap between the header action button and the centered title, in
/// pixels.
pub const HEADER_ACTION_GAP: u32 = 8;

/// Horizontal inset of the placeholder and status panels from the screen edges,
/// in pixels.
pub const PANEL_MARGIN: u32 = 24;

/// Height of the status panel's progress bar, in pixels.
pub const PROGRESS_H: u32 = 14;

/// Inset of the status panel's progress bar from the panel's side and bottom
/// edges, in pixels.
pub const PROGRESS_INSET: u32 = 24;

/// Stroke width of the placeholder panel's border, in pixels.
pub const PANEL_BORDER: u32 = 2;

/// Height of one System Info row, in pixels.
pub const INFO_ROW_H: u32 = 26;

/// Width of the scroll-indicator track, in pixels.
pub const SCROLL_W: u32 = 25;

/// Right-margin inset of the scroll indicator from the screen edge, in pixels.
pub const SCROLL_MARGIN: u32 = 4;

/// Minimum height of the scroll indicator's thumb, in pixels.
pub const SCROLL_MIN_THUMB_H: u32 = 20;

/// Width of the header Cancel button on value screens, in pixels.
pub const CANCEL_W: u32 = 96;

/// Height of the value-entry readout area, in pixels.
pub const READOUT_H: u32 = 56;

/// Horizontal inset of the slider from the screen edges, in pixels.
pub const SLIDER_MARGIN: u32 = 32;

/// Thickness of the slider track, in pixels.
pub const SLIDER_H: u32 = 10;

/// Vertical offset of the slider track from the top of the screen, in pixels.
pub const SLIDER_Y: u32 = 104;

/// Width of the slider thumb, in pixels.
pub const SLIDER_THUMB_W: u32 = 16;

/// Height of the slider thumb and its touch band, in pixels.
pub const SLIDER_THUMB_H: u32 = 28;

/// Width of a fine-adjust button, in pixels.
pub const NUDGE_W: u32 = 64;

/// Height of a fine-adjust button, in pixels.
pub const NUDGE_H: u32 = 44;

/// Vertical offset of the fine-adjust row from the top of the screen, in
/// pixels.
pub const NUDGE_Y: u32 = 132;

/// Height of the footer at the bottom of the value screen, in pixels.
pub const FOOTER_H: u32 = 36;

/// Vertical inset of the footer Save button from the footer's top and bottom,
/// in pixels.
pub const FOOTER_PAD: u32 = 6;

/// Width of the footer Save button, in pixels.
pub const SAVE_W: u32 = 96;

/// Height of the Room Scan screen's sensor-state caption band, in pixels.
///
/// The band sits along the screen's bottom edge, over the radar's lowest ring.
pub const CAPTION_H: u32 = 20;

/// Vertical offset of the first list item below the header, in pixels.
#[must_use]
pub const fn list_top() -> u32 {
    HEADER_H + BUTTON_GAP
}

/// Height of the list view below the header, in pixels.
#[must_use]
pub const fn list_view_height() -> u32 {
    FB_H as u32 - list_top()
}

/// Total height of a list of `count` buttons and the gaps between them, in
/// pixels.
#[must_use]
pub const fn list_content_height(count: usize) -> u32 {
    count as u32 * BUTTON_H + (count as u32).saturating_sub(1) * BUTTON_GAP
}

/// The largest scroll offset for a list of `count` items, or `0` when it fits.
#[must_use]
pub const fn max_scroll_for(count: usize) -> u32 {
    list_content_height(count).saturating_sub(list_view_height())
}

/// The on-screen rectangle of list item `index` at scroll offset `scroll`.
///
/// Every menu shares this geometry, and it is used by both rendering and
/// hit-testing so the two can never disagree.
#[must_use]
pub const fn menu_item_rect(index: usize, scroll: u32) -> Rectangle {
    let x = SIDE_MARGIN;
    let y = (list_top() + index as u32 * (BUTTON_H + BUTTON_GAP)).saturating_sub(scroll);
    let width = FB_W as u32 - 2 * SIDE_MARGIN;
    Rectangle::new(Point::new(x as i32, y as i32), Size::new(width, BUTTON_H))
}

/// The on-screen rectangle of a header action button of the given width, inset
/// from the header's top-left.
#[must_use]
pub const fn header_action_rect(width: u32) -> Rectangle {
    Rectangle::new(
        Point::new(SIDE_MARGIN as i32, BACK_MARGIN as i32),
        Size::new(width, HEADER_H - 2 * BACK_MARGIN),
    )
}

/// The on-screen rectangle of the header Back button.
#[must_use]
pub const fn back_button_rect() -> Rectangle {
    header_action_rect(BACK_W)
}

/// The on-screen rectangle of the header Cancel button on value screens.
#[must_use]
pub const fn cancel_button_rect() -> Rectangle {
    header_action_rect(CANCEL_W)
}

/// The on-screen rectangle of the scroll-indicator track, in the right margin
/// beside the list.
#[must_use]
pub const fn scroll_track_rect() -> Rectangle {
    Rectangle::new(
        Point::new((FB_W as u32 - SCROLL_MARGIN - SCROLL_W) as i32, list_top() as i32),
        Size::new(SCROLL_W, list_view_height()),
    )
}

/// The on-screen rectangle of the placeholder/status panel, inset from the
/// screen edges and below the header.
#[must_use]
pub const fn panel_rect() -> Rectangle {
    Rectangle::new(
        Point::new(PANEL_MARGIN as i32, HEADER_H as i32 + PANEL_MARGIN as i32),
        Size::new(
            FB_W as u32 - 2 * PANEL_MARGIN,
            FB_H as u32 - HEADER_H - 2 * PANEL_MARGIN,
        ),
    )
}

/// The on-screen rectangle of the status panel's progress bar, inset from the
/// panel's side and bottom edges.
#[must_use]
pub const fn progress_bar_rect() -> Rectangle {
    let panel = panel_rect();
    Rectangle::new(
        Point::new(
            panel.top_left.x + PROGRESS_INSET as i32,
            panel.top_left.y + panel.size.height as i32 - PROGRESS_INSET as i32 - PROGRESS_H as i32,
        ),
        Size::new(panel.size.width - 2 * PROGRESS_INSET, PROGRESS_H),
    )
}

/// The on-screen rectangle of the value-entry readout area.
#[must_use]
pub const fn readout_rect() -> Rectangle {
    Rectangle::new(
        Point::new(SIDE_MARGIN as i32, HEADER_H as i32),
        Size::new(FB_W as u32 - 2 * SIDE_MARGIN, READOUT_H),
    )
}

/// The on-screen rectangle of the slider track.
#[must_use]
pub const fn slider_track_rect() -> Rectangle {
    Rectangle::new(
        Point::new(SLIDER_MARGIN as i32, SLIDER_Y as i32),
        Size::new(FB_W as u32 - 2 * SLIDER_MARGIN, SLIDER_H),
    )
}

/// The on-screen rectangle of the slider's touch band: the track height widened
/// to the thumb height so the slider is comfortable to grab.
#[must_use]
pub const fn slider_touch_rect() -> Rectangle {
    let y = SLIDER_Y + SLIDER_H / 2 - SLIDER_THUMB_H / 2;
    Rectangle::new(
        Point::new(SLIDER_MARGIN as i32, y as i32),
        Size::new(FB_W as u32 - 2 * SLIDER_MARGIN, SLIDER_THUMB_H),
    )
}

/// The on-screen rectangle of the fine-adjust decrement button.
#[must_use]
pub const fn nudge_minus_rect() -> Rectangle {
    Rectangle::new(
        Point::new(SIDE_MARGIN as i32, NUDGE_Y as i32),
        Size::new(NUDGE_W, NUDGE_H),
    )
}

/// The on-screen rectangle of the fine-adjust increment button.
#[must_use]
pub const fn nudge_plus_rect() -> Rectangle {
    Rectangle::new(
        Point::new((FB_W as u32 - SIDE_MARGIN - NUDGE_W) as i32, NUDGE_Y as i32),
        Size::new(NUDGE_W, NUDGE_H),
    )
}

/// The on-screen rectangle of the value screen's footer band.
#[must_use]
pub const fn footer_rect() -> Rectangle {
    Rectangle::new(
        Point::new(0, (FB_H as u32 - FOOTER_H) as i32),
        Size::new(FB_W as u32, FOOTER_H),
    )
}

/// The on-screen rectangle of the footer Save button, centered in the footer.
#[must_use]
pub const fn save_button_rect() -> Rectangle {
    Rectangle::new(
        Point::new(
            ((FB_W as u32 - SAVE_W) / 2) as i32,
            (FB_H as u32 - FOOTER_H + FOOTER_PAD) as i32,
        ),
        Size::new(SAVE_W, FOOTER_H - 2 * FOOTER_PAD),
    )
}

/// The on-screen rectangle of the Room Scan screen's sensor-state caption.
///
/// A full-width band along the bottom edge: the radar's outer ring reaches the
/// screen edge, so the caption carries its own background to stay legible.
#[must_use]
pub const fn room_scan_caption_rect() -> Rectangle {
    Rectangle::new(
        Point::new(0, (FB_H as u32 - CAPTION_H) as i32),
        Size::new(FB_W as u32, CAPTION_H),
    )
}

/// The on-screen rectangle of System Info row `index`.
#[must_use]
pub const fn info_row_rect(index: usize) -> Rectangle {
    Rectangle::new(
        Point::new(
            SIDE_MARGIN as i32,
            HEADER_H as i32 + BUTTON_GAP as i32 + index as i32 * INFO_ROW_H as i32,
        ),
        Size::new(FB_W as u32 - 2 * SIDE_MARGIN, INFO_ROW_H),
    )
}

/// The on-screen rectangle of the slider thumb for `value`.
#[must_use]
pub fn slider_thumb_rect(flow: ValueFlow, value: i32) -> Rectangle {
    let left = SLIDER_MARGIN as i32;
    let travel = FB_W as i32 - 2 * SLIDER_MARGIN as i32 - SLIDER_THUMB_W as i32;
    let span = flow.max() - flow.min();
    let ratio = (value - flow.min()).clamp(0, span);
    let x = left + (i64::from(travel) * i64::from(ratio) / i64::from(span)) as i32;
    let y = SLIDER_Y as i32 + SLIDER_H as i32 / 2 - SLIDER_THUMB_H as i32 / 2;
    Rectangle::new(Point::new(x, y), Size::new(SLIDER_THUMB_W, SLIDER_THUMB_H))
}

/// The value nearest to pointer `x` on the slider, snapped to `flow`'s step and
/// clamped to its range.
#[must_use]
pub fn slider_value(flow: ValueFlow, x: i32) -> i32 {
    let left = SLIDER_MARGIN as i32;
    let width = FB_W as i32 - 2 * SLIDER_MARGIN as i32;
    let span = i64::from(flow.max()) - i64::from(flow.min());
    let offset = i64::from((x - left).clamp(0, width));
    let raw = i64::from(flow.min()) + offset * span / i64::from(width);
    let increment = i64::from(flow.step());
    let count = (raw - i64::from(flow.min()) + increment / 2) / increment;
    let snapped = i64::from(flow.min()) + count * increment;
    let clamped = snapped.clamp(i64::from(flow.min()), i64::from(flow.max()));
    i32::try_from(clamped).unwrap_or_else(|_| flow.min())
}
