//! The drawing functions.
//!
//! These are pure views over the model in [`crate::ui`]: every rectangle comes
//! from [`crate::geometry`], so rendering and hit-testing share one layout.
//! Each function draws into a generic [`DrawTarget`] and reports the target's
//! error rather than panicking; the caller flushes the framebuffer afterwards.

use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{MonoFont, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text, renderer::TextRenderer},
};

use crate::{
    buffer::TextBuf,
    geometry::{
        BUTTON_BORDER_W, BUTTON_PAD, DIVIDER_H, FB_W, HEADER_ACTION_GAP, HEADER_H, PANEL_BORDER, SCROLL_MIN_THUMB_H,
        SCROLL_W, footer_rect, info_row_rect, list_view_height, nudge_minus_rect, nudge_plus_rect, panel_rect,
        progress_bar_rect, readout_rect, room_scan_caption_rect, save_button_rect, scroll_track_rect,
        slider_thumb_rect, slider_track_rect,
    },
    hit::{HeaderAction, Hit},
    palette::{ACCENT, BG, BUTTON_BG, BUTTON_BORDER, LABEL_FONT, MUTED, READOUT_FONT, SMALL_FONT, TEXT, TITLE_FONT},
    radar,
    screens::{StatusView, ValueFlow},
    sensor::SensorState,
    system_info::SystemInfo,
};

/// Draw the title header with a muted divider along its bottom edge.
///
/// `action` is the header's left button, if any: [`HeaderAction::Cancel`] on
/// the value screens, [`HeaderAction::Back`] on the other list-like screens,
/// and [`HeaderAction::Stop`] on a running screen; the Main Menu passes `None`.
/// `action_pressed` selects its accent fill. The title is centered in the space
/// the action leaves free, so a wide button and a long title cannot overlap.
///
/// # Errors
///
/// Returns the draw target's error if a primitive fails to draw.
pub fn draw_header<D>(
    d: &mut D,
    title: &str,
    action: Option<HeaderAction>,
    action_pressed: bool,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Fill the header background so list items scrolled up behind it are
    // hidden when the body is drawn first.
    Rectangle::new(Point::zero(), Size::new(FB_W as u32, HEADER_H))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(d)?;

    let title_left = if let Some(action) = action {
        let rect = action.rect();
        draw_button(d, rect, action.label(), LABEL_FONT, action_pressed, true)?;
        rect.top_left.x + rect.size.width as i32 + HEADER_ACTION_GAP as i32
    } else {
        0
    };

    let area = Rectangle::new(
        Point::new(title_left, 0),
        Size::new(FB_W as u32 - title_left as u32, HEADER_H),
    );
    draw_centered_text(d, area, title, MonoTextStyle::new(TITLE_FONT, TEXT))?;

    Rectangle::new(
        Point::new(0, HEADER_H as i32 - DIVIDER_H as i32),
        Size::new(FB_W as u32, DIVIDER_H),
    )
    .into_styled(PrimitiveStyle::with_fill(MUTED))
    .draw(d)?;
    Ok(())
}

/// Draw one wide button.
///
/// A pressed button is filled with [`ACCENT`] and its label inverts to
/// [`BG`]; an idle button uses [`BUTTON_BG`] with a [`BUTTON_BORDER`] outline.
/// `center` centers the label in the button (the header action and the
/// fine-adjust buttons use it); otherwise the label is left-aligned with
/// [`BUTTON_PAD`].
///
/// # Errors
///
/// Returns the draw target's error if a primitive fails to draw.
pub fn draw_button<D>(
    d: &mut D,
    rect: Rectangle,
    label: &str,
    font: &'static MonoFont<'static>,
    pressed: bool,
    center: bool,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let (fill, border, text_color) = if pressed {
        (ACCENT, ACCENT, BG)
    } else {
        (BUTTON_BG, BUTTON_BORDER, TEXT)
    };
    rect.into_styled(PrimitiveStyle::with_fill(fill)).draw(d)?;
    rect.into_styled(PrimitiveStyle::with_stroke(border, BUTTON_BORDER_W))
        .draw(d)?;
    let style = MonoTextStyle::new(font, text_color);
    if center {
        draw_centered_text(d, rect, label, style)
    } else {
        draw_left_text(d, rect, label, style)
    }
}

/// Draw `text` horizontally and vertically centered inside `area`.
///
/// # Errors
///
/// Returns the draw target's error if the text fails to draw.
pub fn draw_centered_text<D>(
    d: &mut D,
    area: Rectangle,
    text: &str,
    style: MonoTextStyle<'_, Rgb565>,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let size = style
        .measure_string(text, Point::zero(), Baseline::Top)
        .bounding_box
        .size;
    let position = Point::new(
        area.top_left.x + (area.size.width as i32 - size.width as i32) / 2,
        area.top_left.y + (area.size.height as i32 - size.height as i32) / 2,
    );
    Text::with_baseline(text, position, style, Baseline::Top).draw(d)?;
    Ok(())
}

/// Draw `text` left-aligned with [`BUTTON_PAD`] of inset and vertically centered
/// inside `area`.
///
/// # Errors
///
/// Returns the draw target's error if the text fails to draw.
pub fn draw_left_text<D>(
    d: &mut D,
    area: Rectangle,
    text: &str,
    style: MonoTextStyle<'_, Rgb565>,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let size = style
        .measure_string(text, Point::zero(), Baseline::Top)
        .bounding_box
        .size;
    let position = Point::new(
        area.top_left.x + BUTTON_PAD as i32,
        area.top_left.y + (area.size.height as i32 - size.height as i32) / 2,
    );
    Text::with_baseline(text, position, style, Baseline::Top).draw(d)?;
    Ok(())
}

/// Draw a running/status screen: an accent-bordered panel carrying the body
/// line.
///
/// When the view reports progress the body sits above a bar across the panel's
/// lower edge. The header carries the dynamic title and the Stop button.
///
/// # Errors
///
/// Returns the draw target's error if a primitive fails to draw.
pub fn draw_status<D>(d: &mut D, view: &StatusView) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let panel = panel_rect();
    panel.into_styled(PrimitiveStyle::with_fill(BUTTON_BG)).draw(d)?;
    panel
        .into_styled(PrimitiveStyle::with_stroke(ACCENT, PANEL_BORDER))
        .draw(d)?;

    let track = progress_bar_rect();
    // With a bar, the body sits in the space above it so the two cannot overlap.
    let body_area = if view.progress.is_some() {
        Rectangle::new(
            panel.top_left,
            Size::new(panel.size.width, (track.top_left.y - panel.top_left.y) as u32),
        )
    } else {
        panel
    };
    draw_centered_text(d, body_area, view.body, MonoTextStyle::new(SMALL_FONT, TEXT))?;

    if let Some(percent) = view.progress {
        track.into_styled(PrimitiveStyle::with_fill(BUTTON_BORDER)).draw(d)?;
        let filled = track.size.width * u32::from(percent.min(100)) / 100;
        if filled > 0 {
            Rectangle::new(track.top_left, Size::new(filled, track.size.height))
                .into_styled(PrimitiveStyle::with_fill(ACCENT))
                .draw(d)?;
        }
    }
    Ok(())
}

/// Draw the System Info rows, one per line in [`SMALL_FONT`].
///
/// # Errors
///
/// Returns the draw target's error if the text fails to draw.
pub fn draw_system_info<D>(d: &mut D, info: &SystemInfo) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let style = MonoTextStyle::new(SMALL_FONT, TEXT);
    for (index, row) in info.rows().iter().enumerate() {
        draw_left_text(d, info_row_rect(index), row.as_str(), style)?;
    }
    Ok(())
}

/// Draw the scroll indicator: a muted track over the list view with an accent
/// thumb whose height is the visible fraction and whose position follows the
/// finger.
///
/// The thumb rests at the bottom of the track when the list is at the top and
/// rises as the list scrolls down, so it moves the same way the finger does
/// (the direction chosen on the bench, inverted from a desktop scrollbar).
///
/// # Errors
///
/// Returns the draw target's error if a primitive fails to draw.
pub fn draw_scroll_indicator<D>(d: &mut D, scroll: u32, max_scroll: u32) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let track = scroll_track_rect();
    track.into_styled(PrimitiveStyle::with_fill(MUTED)).draw(d)?;

    // `max_scroll == content_height - view_height`, so the content height is
    // recovered without a second helper.
    let view_height = list_view_height();
    let content_height = view_height + max_scroll;
    let thumb_height = (track.size.height * view_height / content_height).clamp(SCROLL_MIN_THUMB_H, track.size.height);
    let travel = track.size.height - thumb_height;
    let thumb_y = track.top_left.y + (travel * (max_scroll - scroll) / max_scroll) as i32;
    let thumb = Rectangle::new(Point::new(track.top_left.x, thumb_y), Size::new(SCROLL_W, thumb_height));
    thumb.into_styled(PrimitiveStyle::with_fill(ACCENT)).draw(d)?;
    Ok(())
}

/// Draw the Room Scan screen: the sensor's live spins as a radar, with a caption
/// naming the sensor's lifecycle state.
///
/// The radar is fed the neutral 360-slot frame; an absent frame (`None`) draws
/// as "No data" rather than as an empty room. The caption band carries its own
/// background because the radar's outer ring reaches the screen edge.
///
/// # Errors
///
/// Returns the draw target's error if a primitive or text fails to draw.
pub fn draw_room_scan<D>(d: &mut D, slots: Option<&radar::Slots>, state: SensorState) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    radar::draw(d, slots)?;

    let caption = room_scan_caption_rect();
    caption.into_styled(PrimitiveStyle::with_fill(BG)).draw(d)?;
    draw_centered_text(d, caption, state.label(), MonoTextStyle::new(SMALL_FONT, TEXT))
}

/// Draw the value-entry screen: the live readout, the slider, the fine-adjust
/// buttons, and the footer Save button.
///
/// # Errors
///
/// Returns the draw target's error if a primitive fails to draw.
pub fn draw_value_entry<D>(d: &mut D, flow: ValueFlow, value: i32, pressed: Option<Hit>) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut readout = TextBuf::new();
    readout.set(value, flow.unit());
    draw_centered_text(
        d,
        readout_rect(),
        readout.as_str(),
        MonoTextStyle::new(READOUT_FONT, TEXT),
    )?;

    let track = slider_track_rect();
    track.into_styled(PrimitiveStyle::with_fill(BUTTON_BORDER)).draw(d)?;
    slider_thumb_rect(flow, value)
        .into_styled(PrimitiveStyle::with_fill(ACCENT))
        .draw(d)?;

    draw_button(
        d,
        nudge_minus_rect(),
        "-",
        LABEL_FONT,
        pressed == Some(Hit::NudgeMinus),
        true,
    )?;
    draw_button(
        d,
        nudge_plus_rect(),
        "+",
        LABEL_FONT,
        pressed == Some(Hit::NudgePlus),
        true,
    )?;

    footer_rect()
        .into_styled(PrimitiveStyle::with_fill(BUTTON_BG))
        .draw(d)?;
    draw_button(
        d,
        save_button_rect(),
        "Save",
        LABEL_FONT,
        pressed == Some(Hit::Save),
        true,
    )
}
