//! The Room Scan radar widget.
//!
//! The widget takes a **neutral** 360-slot input of optional distances in
//! centimetres, not a firmware or driver type, so the UI never depends on
//! firmware state. Slot `0` is dead ahead and increasing slots run
//! counter-clockwise, matching the robot's world-frame yaw convention.
//!
//! It draws range rings at one-metre intervals out to five metres, a crosshair,
//! and one mark per valid return at its angle and range. The plotting reuses
//! the transform the bench radar example established. A missing snapshot
//! ([`None`]) renders as "no data", never as an empty room: an all-`None` slot
//! array is a real measurement of no returns and draws as such, while an absent
//! snapshot is labelled explicitly.

use embedded_graphics::{
    Pixel,
    draw_target::DrawTarget,
    mono_font::MonoTextStyle,
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
};

use crate::{
    palette::{RADAR_RETURN, RADAR_RING, SMALL_FONT, TEXT},
    widgets::draw_centered_text,
};

/// Number of one-degree slots in a radar frame.
pub const SLOTS: usize = 360;

/// A neutral radar frame: distance in centimetres per one-degree slot, or
/// `None` for no return.
pub type Slots = [Option<f32>; SLOTS];

/// Radar centre on screen, X (pixels).
const CENTER_X: i32 = 160;
/// Radar centre on screen, Y (pixels).
const CENTER_Y: i32 = 120;
/// Pixels per metre: the 5 m range maps onto the 120 px radius.
const PX_PER_M: f32 = 24.0;
/// Radius of one range ring (1 m) in pixels.
const RING_STEP_PX: i32 = 24;
/// Number of range rings (1 m … 5 m).
const RING_COUNT: i32 = 5;
/// Outer ring radius in pixels (5 m).
const MAX_RADIUS_PX: i32 = 120;
/// Height of the "no data" label area, in pixels.
const NO_DATA_H: u32 = 24;
/// Half-width of the "no data" label area, in pixels.
const NO_DATA_HALF_W: i32 = 100;

/// Draw the radar into `d`, or the "no data" label when no snapshot is
/// available.
///
/// The caller clears the background first (the UI's full-frame render does).
///
/// # Errors
///
/// Returns the draw target's error if a primitive or text fails to draw.
pub fn draw<D>(d: &mut D, slots: Option<&Slots>) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let center = Point::new(CENTER_X, CENTER_Y);

    // White range rings at 1 m intervals, out to 5 m.
    for ring in 1..=RING_COUNT {
        let radius = ring * RING_STEP_PX;
        Circle::with_center(center, (radius * 2) as u32)
            .into_styled(PrimitiveStyle::with_stroke(RADAR_RING, 1))
            .draw(d)?;
    }

    // Crosshair through the centre: the 0°/90°/180°/270° reference.
    Line::new(
        Point::new(CENTER_X - MAX_RADIUS_PX, CENTER_Y),
        Point::new(CENTER_X + MAX_RADIUS_PX, CENTER_Y),
    )
    .into_styled(PrimitiveStyle::with_stroke(RADAR_RING, 1))
    .draw(d)?;
    Line::new(
        Point::new(CENTER_X, CENTER_Y - MAX_RADIUS_PX),
        Point::new(CENTER_X, CENTER_Y + MAX_RADIUS_PX),
    )
    .into_styled(PrimitiveStyle::with_stroke(RADAR_RING, 1))
    .draw(d)?;

    // Centre marker: the sensor position.
    draw_cross(d, CENTER_X, CENTER_Y, RADAR_RING)?;

    match slots {
        Some(slots) => {
            for (slot, distance) in slots.iter().enumerate() {
                let Some(distance_cm) = *distance else {
                    continue;
                };
                let mark = mark_point(slot, distance_cm);
                draw_cross(d, mark.x, mark.y, RADAR_RETURN)?;
            }
        }
        None => draw_centered_text(d, radar_area(), "No data", MonoTextStyle::new(SMALL_FONT, TEXT))?,
    }

    Ok(())
}

/// The screen point for a return in `slot` at `distance_cm`, clamped to the
/// outer ring.
///
/// The sensor's mounting reverses both axes relative to the panel, so the
/// plotted offsets are negated — equivalent to rotating the returns 180° while
/// `0°` still points forward and increasing angles stay clockwise. The angle
/// comes from the slot index at one degree per slot.
#[must_use]
pub fn mark_point(slot: usize, distance_cm: f32) -> Point {
    let metres = distance_cm / 100.0;
    let radius = (metres * PX_PER_M).min(MAX_RADIUS_PX as f32);
    let angle_rad = (slot as f32).to_radians();
    let x = CENTER_X - (radius * libm::sinf(angle_rad)) as i32;
    let y = CENTER_Y + (radius * libm::cosf(angle_rad)) as i32;
    Point::new(x, y)
}

/// The centred label area used when no radar snapshot is available.
const fn radar_area() -> Rectangle {
    Rectangle::new(
        Point::new(CENTER_X - NO_DATA_HALF_W, CENTER_Y - NO_DATA_H as i32 / 2),
        Size::new((NO_DATA_HALF_W * 2) as u32, NO_DATA_H),
    )
}

/// Draw one cross-shaped mark (centre plus four orthogonal neighbours) at
/// `(x, y)`. Out-of-bounds pixels are clipped by the draw target.
fn draw_cross<D>(d: &mut D, x: i32, y: i32, color: Rgb565) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    for (offset_x, offset_y) in [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)] {
        Pixel(Point::new(x + offset_x, y + offset_y), color).draw(d)?;
    }
    Ok(())
}
