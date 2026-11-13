//! The drawing functions.
//!
//! These are pure views over the model in [`crate::ui`]: every rectangle comes
//! from [`crate::geometry`], so rendering and hit-testing share one layout.
//! Each function draws into a generic [`DrawTarget`] and reports the target's
//! error rather than panicking; the caller flushes the framebuffer afterwards.

use embedded_graphics::{
    draw_target::{DrawTarget, DrawTargetExt},
    mono_font::{MonoFont, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text, renderer::TextRenderer},
};
use link_state::{ControlOutcome, ControlReply, ControlStatus, LinkPhase, Snapshot};

use crate::{
    buffer::TextBuf,
    geometry::{
        BUTTON_BORDER_W, BUTTON_PAD, DIVIDER_H, FB_W, HEADER_H, PANEL_BORDER, SCROLL_MIN_THUMB_H, SCROLL_W,
        footer_rect, header_status_rect, header_title_rect, info_row_rect, link_action_rect, link_info_cell_rect,
        link_result_line_rect, link_result_rect, list_view_height, nudge_minus_rect, nudge_plus_rect, panel_rect,
        progress_bar_rect, prompt_instruction_rect, prompt_progress_rect, prompt_status_rect, readout_area,
        readout_rect, readout_row_rect, room_scan_caption_rect, save_button_rect, scroll_track_rect, slider_thumb_rect,
        slider_track_rect,
    },
    hit::{HeaderAction, Hit, LinkAction, PromptAction},
    imu::ImuReadout,
    palette::{ACCENT, BG, BUTTON_BG, BUTTON_BORDER, LABEL_FONT, MUTED, READOUT_FONT, SMALL_FONT, TEXT, TITLE_FONT},
    radar,
    screens::{PromptView, StatusView, ValueFlow},
    sensor::SensorState,
    system_info::SystemInfo,
    ui::HeaderStatus,
};

/// Draw the title header with a muted divider along its bottom edge.
///
/// `action` is the header's left button, if any: [`HeaderAction::Cancel`] on
/// the value screens, [`HeaderAction::Back`] on the other list-like screens,
/// and [`HeaderAction::Stop`] on a running screen; the Main Menu passes `None`.
/// `action_pressed` selects its accent fill. `status` is drawn in the fixed slot
/// on the right on every screen, so a missing or changing value never moves the
/// title or the action. The title is centered between the action and that slot.
///
/// # Errors
///
/// Returns the draw target's error if a primitive fails to draw.
pub fn draw_header<D>(
    d: &mut D,
    title: &str,
    action: Option<HeaderAction>,
    action_pressed: bool,
    status: HeaderStatus,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Fill the header background so list items scrolled up behind it are
    // hidden when the body is drawn first.
    Rectangle::new(Point::zero(), Size::new(FB_W as u32, HEADER_H))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(d)?;

    let action_width = if let Some(action) = action {
        let rect = action.rect();
        draw_button(d, rect, action.label(), LABEL_FONT, action_pressed, true)?;
        Some(rect.size.width)
    } else {
        None
    };

    let title_area = header_title_rect(action_width);
    let title_font = header_title_font(title, title_area);
    // The fixed status slot leaves a narrow title area. Fall back to the small
    // font for a long title, and clip as a last resort so the action button on
    // the left and the status slot on the right can never be overpainted.
    let mut clipped = d.clipped(&title_area);
    draw_centered_text(&mut clipped, title_area, title, MonoTextStyle::new(title_font, TEXT))?;
    draw_header_status(d, status)?;

    Rectangle::new(
        Point::new(0, HEADER_H as i32 - DIVIDER_H as i32),
        Size::new(FB_W as u32, DIVIDER_H),
    )
    .into_styled(PrimitiveStyle::with_fill(MUTED))
    .draw(d)?;
    Ok(())
}

/// The font the header uses for `title` in `area`.
///
/// A title that does not fit the title font falls back to the small font. The
/// caller still clips to `area`, so even a title too long for the small font
/// cannot paint over the header action or the fixed status slot.
#[must_use]
pub fn header_title_font(title: &str, area: Rectangle) -> &'static MonoFont<'static> {
    let style = MonoTextStyle::new(TITLE_FONT, TEXT);
    let width = style
        .measure_string(title, Point::zero(), Baseline::Top)
        .bounding_box
        .size
        .width;
    if width <= area.size.width {
        TITLE_FONT
    } else {
        SMALL_FONT
    }
}

/// Draw the fixed header status slot: the compact link phase over the battery.
///
/// The slot paints its own background, so a title long enough to reach it is
/// clipped rather than drawn over the indicator.
fn draw_header_status<D>(d: &mut D, status: HeaderStatus) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let slot = header_status_rect();
    slot.into_styled(PrimitiveStyle::with_fill(BG)).draw(d)?;

    let half = slot.size.height / 2;
    let top = Rectangle::new(slot.top_left, Size::new(slot.size.width, half));
    let bottom = Rectangle::new(
        Point::new(slot.top_left.x, slot.top_left.y + half as i32),
        Size::new(slot.size.width, slot.size.height - half),
    );
    let style = MonoTextStyle::new(SMALL_FONT, TEXT);
    draw_centered_text(d, top, compact_link_label(status.link), style)?;

    let mut battery = TextBuf::new();
    match status.battery {
        Some(level) => battery.fill(format_args!("{level}%")),
        None => battery.fill(format_args!("--%")),
    }
    draw_centered_text(d, bottom, battery.as_str(), style)
}

/// The compact link phase the fixed header slot shows.
const fn compact_link_label(phase: LinkPhase) -> &'static str {
    match phase {
        LinkPhase::Off => "Off",
        LinkPhase::WaitingForBoard => "Wait",
        LinkPhase::ReadyNoPeer => "NoPeer",
        LinkPhase::PeerLive => "Live",
    }
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

/// Lines the prompt draws for one text block.
///
/// The instruction and the status line each get a two-line block inside their
/// existing 24-pixel row, so a long instruction wraps instead of being clipped
/// at both ends.
pub const PROMPT_TEXT_LINES: usize = 2;

/// Split `text` into at most `lines.len()` lines of at most `max_chars`
/// characters, breaking at spaces.
///
/// Returns the number of lines written into `lines`. A word longer than a whole
/// line is hard-cut at the limit, so the split always makes progress.
pub fn wrap_text<'a>(text: &'a str, max_chars: usize, lines: &mut [&'a str]) -> usize {
    let limit = max_chars.max(1);
    let mut written = 0;
    let mut rest = text.trim();

    while !rest.is_empty() && written < lines.len() {
        let mut end = rest.len();
        let mut last_space = None;
        for (width, (offset, ch)) in rest.char_indices().enumerate() {
            if width == limit {
                end = offset;
                break;
            }
            if ch == ' ' {
                last_space = Some(offset);
            }
        }

        // Prefer the last space inside the line; hard-cut when there is none.
        let cut = match last_space {
            Some(space) if end != rest.len() && space > 0 => space,
            _ => end,
        };
        lines[written] = rest[..cut].trim_end();
        rest = rest[cut..].trim_start();
        written += 1;
    }
    written
}

/// Draw `text` wrapped to at most [`PROMPT_TEXT_LINES`] lines, centered inside
/// `area`.
///
/// The style's first glyph gives the character width, so this assumes the
/// monospace style the prompt uses.
///
/// # Errors
///
/// Returns the draw target's error if the text fails to draw.
fn draw_wrapped_centered_text<D>(
    d: &mut D,
    area: Rectangle,
    text: &str,
    style: MonoTextStyle<'_, Rgb565>,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let glyph_width = style
        .measure_string("M", Point::zero(), Baseline::Top)
        .bounding_box
        .size
        .width
        .max(1);
    let max_chars = (area.size.width / glyph_width) as usize;

    let mut lines = [""; PROMPT_TEXT_LINES];
    let count = wrap_text(text, max_chars, &mut lines);
    let pitch = style.line_height() + 2;
    let block_height = pitch * count as u32;
    let top = area.top_left.y + (area.size.height as i32 - block_height as i32) / 2;

    for (index, line) in lines[..count].iter().enumerate() {
        let size = style
            .measure_string(line, Point::zero(), Baseline::Top)
            .bounding_box
            .size;
        let position = Point::new(
            area.top_left.x + (area.size.width as i32 - size.width as i32) / 2,
            top + (index as u32 * pitch) as i32,
        );
        Text::with_baseline(line, position, style, Baseline::Top).draw(d)?;
    }
    Ok(())
}

/// Draw a running/status screen: an accent-bordered panel carrying the body
/// line.
///
/// When the view reports progress the body sits above a bar across the panel's
/// lower edge. When the view carries an IMU Test Readout, the rows replace both
/// the body line and the progress bar. The header carries the dynamic title and
/// the Stop button.
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

    if let Some(readout) = view.readout() {
        return draw_imu_readout(d, readout);
    }

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
    draw_centered_text(d, body_area, view.body_text(), MonoTextStyle::new(SMALL_FONT, TEXT))?;

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

/// Draw a guided prompt screen.
///
/// The status panel carries the instruction and the dynamic status line, an
/// optional progress bar, and the available Done/Redo actions. The header
/// carries the dynamic title and the Stop button.
///
/// A disabled action is not drawn, so a movement step shows only Done while the
/// final Compass Checks show Done and Redo. Both text lines are wrapped to two
/// lines and clipped to their own rectangles, so a long instruction stays fully
/// readable and cannot paint over the panel border or the action row.
///
/// # Errors
///
/// Returns the draw target's error if a primitive or text fails to draw.
pub fn draw_prompt<D>(d: &mut D, view: &PromptView, pressed: Option<Hit>) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let panel = panel_rect();
    panel.into_styled(PrimitiveStyle::with_fill(BUTTON_BG)).draw(d)?;
    panel
        .into_styled(PrimitiveStyle::with_stroke(ACCENT, PANEL_BORDER))
        .draw(d)?;

    let style = MonoTextStyle::new(SMALL_FONT, TEXT);
    draw_wrapped_centered_text(
        &mut d.clipped(&prompt_instruction_rect()),
        prompt_instruction_rect(),
        view.instruction,
        style,
    )?;
    draw_wrapped_centered_text(
        &mut d.clipped(&prompt_status_rect()),
        prompt_status_rect(),
        view.status_text(),
        style,
    )?;

    if let Some(percent) = view.progress {
        let track = prompt_progress_rect();
        track.into_styled(PrimitiveStyle::with_fill(BUTTON_BORDER)).draw(d)?;
        let filled = track.size.width * u32::from(percent.min(100)) / 100;
        if filled > 0 {
            Rectangle::new(track.top_left, Size::new(filled, track.size.height))
                .into_styled(PrimitiveStyle::with_fill(ACCENT))
                .draw(d)?;
        }
    }

    for action in PromptAction::ALL {
        let available = match action {
            PromptAction::Done => view.done,
            PromptAction::Redo => view.redo,
        };
        if available {
            draw_button(
                d,
                action.rect(),
                action.label(),
                LABEL_FONT,
                pressed == Some(Hit::PromptAction(action)),
                true,
            )?;
        }
    }
    Ok(())
}

/// Draw the IMU Test Readout rows inside the status panel.
///
/// The rows come from the readout model, so labels, units, precision and
/// placeholders stay in one place. Each line starts at its row's left edge (not
/// inset by [`BUTTON_PAD`], which would clip the widest rows) and is vertically
/// centred on a ten-pixel glyph. Drawing is clipped to the readout area so a
/// hostile over-long value cannot paint over the panel border.
///
/// # Errors
///
/// Returns the draw target's error if the text fails to draw.
pub fn draw_imu_readout<D>(d: &mut D, readout: &ImuReadout) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let area = readout_area();
    let mut clipped = d.clipped(&area);
    let style = MonoTextStyle::new(SMALL_FONT, TEXT);
    let row_count = readout.row_count();

    for (index, row) in readout.rows().iter().take(row_count).enumerate() {
        let rect = readout_row_rect(index, row_count);
        let size = style
            .measure_string(row.as_str(), Point::zero(), Baseline::Top)
            .bounding_box
            .size;
        let position = Point::new(
            rect.top_left.x,
            rect.top_left.y + (rect.size.height as i32 - size.height as i32) / 2,
        );
        Text::with_baseline(row.as_str(), position, style, Baseline::Top).draw(&mut clipped)?;
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

/// Draw the Link screen: a compact info block, the three Bridge Control action
/// buttons, and the result band.
///
/// The info block keeps Bridge presence, Peer, RSSI, channel, station and setup
/// state visible. The actions are drawn and hittable even when no Bridge is
/// present, so the firmware — not the panel — decides what to do with them. The
/// result band shows the latest Bridge Control outcome: pending, timed out, or
/// the Bridge's credentials or pairing code. `pressed` selects an action's
/// accent fill while the pen is on it.
///
/// # Errors
///
/// Returns the draw target's error if a primitive or text fails to draw.
pub fn draw_link<D>(d: &mut D, snapshot: &Snapshot, pressed: Option<Hit>) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let style = MonoTextStyle::new(SMALL_FONT, TEXT);

    for (row, cells) in link_info_rows(snapshot).iter().enumerate() {
        for (column, text) in cells.iter().enumerate() {
            if !text.as_str().is_empty() {
                draw_left_text(d, link_info_cell_rect(row, column), text.as_str(), style)?;
            }
        }
    }

    for (index, action) in LinkAction::ALL.into_iter().enumerate() {
        draw_button(
            d,
            link_action_rect(index),
            action.label(),
            LABEL_FONT,
            pressed == Some(Hit::LinkAction(action)),
            false,
        )?;
    }

    link_result_rect()
        .into_styled(PrimitiveStyle::with_fill(BUTTON_BG))
        .draw(d)?;

    match &snapshot.control {
        ControlOutcome::Idle => draw_left_text(d, link_result_line_rect(0), "No request", style)?,
        ControlOutcome::Pending => draw_left_text(d, link_result_line_rect(0), "Sending...", style)?,
        ControlOutcome::TimedOut => draw_left_text(d, link_result_line_rect(0), "Timed out", style)?,
        ControlOutcome::Answered(reply) => {
            let status = reply.status();
            draw_left_text(d, link_result_line_rect(0), status.label(), style)?;
            match reply {
                ControlReply::Setup { status, credentials } => {
                    if *status == ControlStatus::Ok {
                        draw_labelled_value(d, link_result_line_rect(1), "SSID: ", credentials.ssid.as_str(), style)?;
                        draw_labelled_value(
                            d,
                            link_result_line_rect(2),
                            "Pass: ",
                            credentials.passphrase.as_str(),
                            style,
                        )?;
                    }
                }
                ControlReply::PairingCode { code, .. } => {
                    draw_labelled_value(d, link_result_line_rect(1), "Code: ", code.code.as_str(), style)?;
                }
            }
        }
    }

    Ok(())
}

/// Draw `label` and then `value` on one line, left-aligned in `area`.
///
/// The value is drawn from its own slice, not through [`TextBuf`], so a setup
/// SSID or passphrase longer than [`crate::buffer::TEXT_BUF_LEN`] is not
/// truncated. A string wider than the screen still clips at the panel edge, but
/// no byte of it is dropped by this crate.
fn draw_labelled_value<D>(
    d: &mut D,
    area: Rectangle,
    label: &str,
    value: &str,
    style: MonoTextStyle<'_, Rgb565>,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let start = Point::new(area.top_left.x + BUTTON_PAD as i32, area.top_left.y);
    draw_text_line(d, start, area.size.height, label, style)?;
    let label_width = style
        .measure_string(label, Point::zero(), Baseline::Top)
        .bounding_box
        .size
        .width;
    let value_start = Point::new(start.x + label_width as i32, start.y);
    draw_text_line(d, value_start, area.size.height, value, style)
}

/// Draw one text line whose top-left is `start`, vertically centered in
/// `height`.
fn draw_text_line<D>(
    d: &mut D,
    start: Point,
    height: u32,
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
    let position = Point::new(start.x, start.y + (height as i32 - size.height as i32) / 2);
    Text::with_baseline(text, position, style, Baseline::Top).draw(d)?;
    Ok(())
}

/// The Link info block, row by row: `Bridge`/`Peer`, `RSSI`/`Channel`,
/// `Station`/`Setup`, and `Status` on its own.
fn link_info_rows(snapshot: &Snapshot) -> [[TextBuf; 2]; 4] {
    let bridge = snapshot.bridge;

    let mut presence = TextBuf::new();
    presence.fill(format_args!("Bridge: {}", snapshot.presence.label()));

    let mut peer = TextBuf::new();
    peer.fill(format_args!("Peer: {}", snapshot.peer.label()));

    let mut rssi = TextBuf::new();
    match bridge.and_then(|status| status.rssi_dbm) {
        Some(dbm) => rssi.fill(format_args!("RSSI: {dbm} dBm")),
        None => rssi.fill(format_args!("RSSI: --")),
    }

    let mut channel = TextBuf::new();
    match bridge.and_then(|status| status.channel) {
        Some(value) => channel.fill(format_args!("Channel: {value}")),
        None => channel.fill(format_args!("Channel: --")),
    }

    let mut station = TextBuf::new();
    match bridge {
        Some(status) => station.fill(format_args!("Station: {}", status.station.label())),
        None => station.fill(format_args!("Station: --")),
    }

    let mut setup = TextBuf::new();
    match bridge {
        Some(status) => setup.fill(format_args!(
            "Setup: {}",
            if status.flags.setup_open { "Open" } else { "Closed" }
        )),
        None => setup.fill(format_args!("Setup: --")),
    }

    let mut status = TextBuf::new();
    let label = match (bridge, snapshot.bridge_stale) {
        (None, _) => "No status",
        (Some(_), true) => "Stale",
        (Some(_), false) => "Fresh",
    };
    status.fill(format_args!("Status: {label}"));

    [
        [presence, peer],
        [rssi, channel],
        [station, setup],
        [status, TextBuf::new()],
    ]
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
