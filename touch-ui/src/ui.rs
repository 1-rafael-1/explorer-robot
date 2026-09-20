//! The screen/state model: navigation, gestures, scrolling, and hit-testing.
//!
//! [`Ui`] holds which screen is shown, the active press if any, the list scroll
//! offset, the value-entry value, the System Info snapshot, and the running
//! screen's content. The event loop above it translates panel samples into
//! [`Ui::pointer_down`], [`Ui::pointer_move`], and [`Ui::pointer_up`] calls and
//! redisplays when a call reports that the screen needs redrawing. Rendering is
//! full-frame on state change and throttled during a drag by [`DRAG_RENDER_MS`].
//!
//! Gesture policy lives here: a press becomes a drag once it moves further than
//! [`TAP_MAX_MOVE`], a release is a tap only if it also lasted at least
//! [`TAP_MIN_DURATION_MS`] and landed on the same region the press began on,
//! and a tap only then activates.

use embedded_graphics::{draw_target::DrawTarget, pixelcolor::Rgb565, prelude::Point};

use crate::{
    geometry::{
        cancel_button_rect, list_top, max_scroll_for, menu_item_rect, nudge_minus_rect, nudge_plus_rect,
        save_button_rect, slider_touch_rect, slider_value,
    },
    hit::{HeaderAction, Hit},
    palette, radar,
    screens::{Screen, StatusView, ValueFlow},
    sensor::SensorState,
    system_info::SystemInfo,
    widgets,
};

/// Poll period, in milliseconds, while the pen is down.
///
/// 20 ms (50 Hz) tracks a finger closely without spending more time on SPI
/// reads than the panel needs.
pub const TICK_MS: u64 = 20;

/// Maximum total pointer movement, in pixels, that still counts as a tap.
///
/// A fingertip rolls several pixels when it presses, and the first panel sample
/// after the pen-down edge can be noisier still; 20 px absorbs that. It stays
/// well below the 38 px button height, and a release must also land on the same
/// region the press began on, so an intended tap cannot activate a neighbour
/// across the 8 px gap.
pub const TAP_MAX_MOVE: u32 = 20;

/// Minimum press duration, in milliseconds, for a release to count as a tap.
///
/// Set just below [`TICK_MS`] so it rejects contact bounce without rejecting a
/// quick deliberate tap: the poll loop can only observe a release on a tick, so
/// a floor above the tick period would drop fast taps.
pub const TAP_MIN_DURATION_MS: u64 = 20;

/// Minimum interval, in milliseconds, between drag redraws.
///
/// A full-frame flush at 64 MHz costs a few milliseconds, so 50 ms (20 fps)
/// keeps drag-scroll smooth without saturating the shared bus; a release always
/// renders the final position immediately.
pub const DRAG_RENDER_MS: u64 = 50;

/// An in-progress press, from pen-down to release.
#[derive(Clone, Copy, Debug)]
struct Press {
    /// Pixel where the pen first went down.
    start: Point,
    /// Most recent pointer pixel (equals `start` until the pointer moves).
    last: Point,
    /// Uptime in milliseconds at pen-down, for the tap-duration check.
    started_ms: u64,
    /// Whether the press has moved past the tap threshold and become a drag.
    dragging: bool,
    /// Whether the press began on the value screen's slider, which it drives
    /// directly instead of going through the tap path.
    on_slider: bool,
    /// The region the press began on, so a release only activates the region it
    /// started from.
    target: Option<Hit>,
}

/// The screen/state model.
///
/// Create one with [`Ui::new`], feed it pointer samples, and call [`Ui::render`]
/// into the display when a call reports a redraw is needed.
#[derive(Clone, Copy, Debug)]
pub struct Ui {
    /// The screen currently rendered.
    screen: Screen,
    /// The in-progress press, or `None` when the pen is up.
    press: Option<Press>,
    /// Vertical scroll offset of the current list, in pixels; `0` when it fits.
    scroll: u32,
    /// Current value on a value-entry screen, in the flow's unit; `0` otherwise.
    value: i32,
    /// The System Info snapshot shown by [`Screen::SystemInfo`].
    system_info: SystemInfo,
    /// The content shown by [`Screen::Status`].
    status: StatusView,
    /// The radar frame shown by [`Screen::RoomScan`], or `None` when no snapshot
    /// has been supplied yet.
    radar: Option<radar::Slots>,
    /// The sensor-state caption shown by [`Screen::RoomScan`].
    sensor_state: SensorState,
    /// The region the most recent completed tap activated, or `None` when the
    /// last gesture was a drag, a release, or a slider drag.
    activated: Option<Hit>,
}

impl Ui {
    /// Create a UI showing the Main Menu with no press, scroll, or value.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            screen: Screen::MainMenu,
            press: None,
            scroll: 0,
            value: 0,
            system_info: SystemInfo::new(),
            status: StatusView::new("Running", "", Screen::MainMenu),
            radar: None,
            sensor_state: SensorState::Off,
            activated: None,
        }
    }

    /// The screen currently shown.
    #[must_use]
    pub const fn screen(&self) -> Screen {
        self.screen
    }

    /// The header title for the current screen.
    ///
    /// A running screen supplies its own title through the active
    /// [`StatusView`]; every other screen uses its [`Screen::title`].
    #[must_use]
    pub const fn title(&self) -> &'static str {
        match self.screen {
            Screen::Status => self.status.title,
            other => other.title(),
        }
    }

    /// The current value on a value-entry screen, in the flow's unit.
    #[must_use]
    pub const fn value(&self) -> i32 {
        self.value
    }

    /// The current list scroll offset, in pixels.
    #[must_use]
    pub const fn scroll(&self) -> u32 {
        self.scroll
    }

    /// The largest valid scroll offset for the current list (`0` when it fits).
    #[must_use]
    pub const fn max_scroll(&self) -> u32 {
        max_scroll_for(self.screen.items().len())
    }

    /// The System Info snapshot the status screen renders.
    #[must_use]
    pub const fn system_info(&self) -> &SystemInfo {
        &self.system_info
    }

    /// The radar frame the Room Scan screen renders, if one was supplied.
    ///
    /// `None` means no snapshot is available, which the screen draws as "No
    /// data" rather than as an empty room.
    #[must_use]
    pub const fn radar(&self) -> Option<&radar::Slots> {
        self.radar.as_ref()
    }

    /// The sensor-state caption the Room Scan screen renders.
    #[must_use]
    pub const fn sensor_state(&self) -> SensorState {
        self.sensor_state
    }

    /// The active running screen's content, or `None` on any other screen.
    #[must_use]
    pub const fn status_view(&self) -> Option<StatusView> {
        match self.screen {
            Screen::Status => Some(self.status),
            _ => None,
        }
    }

    /// The region activated by the most recent completed tap, if any.
    ///
    /// The firmware reads this after a gesture to decide what to start, stop, or
    /// save: the model owns the tap-versus-drag decision, so the host has one
    /// authority for what counts as an activation. A drag, a drag that began on
    /// the slider, and a release that did not activate all clear it.
    #[must_use]
    pub const fn last_activation(&self) -> Option<Hit> {
        self.activated
    }

    /// Replace the System Info snapshot, reporting whether it changed.
    pub fn set_system_info(&mut self, info: SystemInfo) -> bool {
        if info == self.system_info {
            return false;
        }
        self.system_info = info;
        true
    }

    /// Replace the Room Scan radar frame, reporting whether it changed.
    ///
    /// A no-op on any screen other than [`Screen::RoomScan`]. `None` means no
    /// snapshot is available and the screen draws "No data".
    pub fn set_radar(&mut self, slots: Option<radar::Slots>) -> bool {
        if self.screen != Screen::RoomScan || self.radar == slots {
            return false;
        }
        self.radar = slots;
        true
    }

    /// Replace the Room Scan sensor-state caption, reporting whether it changed.
    ///
    /// A no-op on any screen other than [`Screen::RoomScan`].
    pub fn set_sensor_state(&mut self, state: SensorState) -> bool {
        if self.screen != Screen::RoomScan || self.sensor_state == state {
            return false;
        }
        self.sensor_state = state;
        true
    }

    /// Open a running/status screen, resetting the list scroll and value.
    ///
    /// Always reports a redraw: entering the screen is itself a state change.
    pub const fn show_status(&mut self, view: StatusView) -> bool {
        self.screen = Screen::Status;
        self.status = view;
        self.scroll = 0;
        self.value = 0;
        true
    }

    /// Replace the running screen's content, reporting whether it changed.
    ///
    /// Unlike [`Ui::show_status`] this neither changes the screen nor resets the
    /// scroll and value, so it is the update call for a producer reporting
    /// progress. A no-op on any screen other than [`Screen::Status`], and when
    /// the view is already what was supplied.
    pub fn set_status(&mut self, view: StatusView) -> bool {
        if self.screen != Screen::Status || self.status == view {
            return false;
        }
        self.status = view;
        true
    }

    /// Open the value-entry screen for `flow`, resetting the value to its preset.
    ///
    /// Always reports a redraw. Used by the firmware when a flow that ran before
    /// the entry step (distance calibration's drive) finishes.
    pub const fn show_value_entry(&mut self, flow: ValueFlow) -> bool {
        self.navigate_to(Screen::ValueEntry(flow))
    }

    /// Leave the current screen for its parent, as the header Back button does.
    ///
    /// Reports whether the screen changed; the Main Menu has no parent, so this
    /// is a no-op there. Used by the firmware to abandon a screen whose entry it
    /// refuses.
    pub fn back(&mut self) -> bool {
        self.go_back()
    }

    /// Replace the running screen's body line, reporting whether it changed.
    ///
    /// A no-op on any screen other than [`Screen::Status`], and when the body is
    /// already what was supplied.
    pub fn set_status_body(&mut self, body: &'static str) -> bool {
        if self.screen != Screen::Status || self.status.body == body {
            return false;
        }
        self.status.body = body;
        true
    }

    /// Return the interactive region under `p`, if any.
    #[must_use]
    pub fn hit_test(&self, p: Point) -> Option<Hit> {
        if matches!(self.screen, Screen::ValueEntry(_)) {
            return value_entry_hit(p);
        }
        if let Some(action) = self.header_action()
            && action.rect().contains(p)
        {
            return Some(action.hit());
        }
        // Items scrolled above the list top must not be reachable through the
        // header region.
        if p.y < list_top() as i32 {
            return None;
        }
        let items = self.screen.items();
        (0..items.len())
            .find(|&index| menu_item_rect(index, self.scroll).contains(p))
            .map(Hit::MenuItem)
    }

    /// Record a pen-down at `p` and report whether the screen needs redrawing.
    ///
    /// A redraw is needed when the pen landed on a button, because that starts
    /// the pressed indication, or on the value screen's slider, which takes
    /// effect immediately.
    #[must_use]
    pub fn pointer_down(&mut self, p: Point, now_ms: u64) -> bool {
        let hit = self.hit_test(p);
        let on_slider = hit == Some(Hit::Slider);
        self.activated = None;
        self.press = Some(Press {
            start: p,
            last: p,
            started_ms: now_ms,
            dragging: false,
            on_slider,
            target: hit,
        });
        if on_slider {
            // The slider drives the value directly, bypassing the tap path.
            return self.set_slider_value(p.x);
        }
        hit.is_some()
    }

    /// Update the pointer to `p` and report whether the screen needs redrawing.
    ///
    /// A press that began on the slider maps its `x` to the value immediately. A
    /// list press becomes a drag once the total movement from the press start
    /// exceeds [`TAP_MAX_MOVE`]: any pressed highlight clears and, on an
    /// overflowing list started within the list view, the vertical movement
    /// scrolls it. A smaller movement keeps the press a candidate tap and moves
    /// the highlight. `now_ms` is accepted for symmetry with the other pointer
    /// calls; the drag decision measures movement, not time.
    #[must_use]
    pub fn pointer_move(&mut self, p: Point, _now_ms: u64) -> bool {
        let Some(mut press) = self.press else {
            return false;
        };

        if press.on_slider {
            press.last = p;
            self.press = Some(press);
            return self.set_slider_value(p.x);
        }

        let was = self.hit_test(press.last);
        let step_y = p.y - press.last.y;
        press.last = p;

        let move_x = i64::from(p.x) - i64::from(press.start.x);
        let move_y = i64::from(p.y) - i64::from(press.start.y);
        let max_move = i64::from(TAP_MAX_MOVE);
        let became_drag = !press.dragging && move_x * move_x + move_y * move_y > max_move * max_move;
        press.dragging |= became_drag;

        let now = self.hit_test(p);
        let mut redraw = was != now || became_drag;

        if press.dragging && self.press_scrolls(press.start) {
            redraw |= self.scroll_by(-step_y);
        }

        self.press = Some(press);
        redraw
    }

    /// End the active press at the stored release position and report whether
    /// the screen needs redrawing.
    ///
    /// A release is a tap when the press never became a drag or a slider drag,
    /// moved no further than [`TAP_MAX_MOVE`], lasted at least
    /// [`TAP_MIN_DURATION_MS`], and landed on the same region the press began
    /// on; a tap on a region activates it. A drag never activates, but any press
    /// that began on a region still redraws on release: the throttled move
    /// redraws may not have cleared the pressed highlight or rendered the final
    /// scroll position or slider value.
    #[must_use]
    pub fn pointer_up(&mut self, now_ms: u64) -> bool {
        let Some(press) = self.press.take() else {
            return false;
        };

        let released = self.hit_test(press.last);
        let tap_candidate = !press.dragging && !press.on_slider;
        let same_region = released.is_some() && released == press.target;
        let move_x = i64::from(press.last.x) - i64::from(press.start.x);
        let move_y = i64::from(press.last.y) - i64::from(press.start.y);
        let max_move = i64::from(TAP_MAX_MOVE);
        let moved_sq = move_x * move_x + move_y * move_y;
        let elapsed_ms = now_ms.saturating_sub(press.started_ms);
        let is_tap =
            tap_candidate && same_region && moved_sq <= max_move * max_move && elapsed_ms >= TAP_MIN_DURATION_MS;

        let released = if is_tap { released } else { None };
        self.activated = released;
        let activated = released.is_some_and(|hit| self.activate(hit));

        activated || press.target.is_some() || press.dragging
    }

    /// Draw the current screen into `d`.
    ///
    /// Rendering is full-frame: the caller flushes the whole framebuffer after
    /// this returns.
    ///
    /// # Errors
    ///
    /// Returns the draw target's error if a primitive or text fails to draw.
    pub fn render<D>(&self, d: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        d.clear(palette::BG)?;

        // A drag clears the pressed highlight, and a press only marks the region
        // it started on while the pointer is still on it.
        let pressed = self
            .press
            .as_ref()
            .filter(|p| !p.dragging)
            .and_then(|p| p.target.filter(|target| self.hit_test(p.last) == Some(*target)));

        // The body is drawn before the header so items scrolled above the list
        // top are painted over by the header background.
        match self.screen {
            Screen::Placeholder(kind) => widgets::draw_placeholder(d, kind)?,
            Screen::SystemInfo => widgets::draw_system_info(d, &self.system_info)?,
            Screen::ValueEntry(flow) => widgets::draw_value_entry(d, flow, self.value, pressed)?,
            Screen::Status => widgets::draw_status(d, &self.status)?,
            Screen::RoomScan => widgets::draw_room_scan(d, self.radar.as_ref(), self.sensor_state)?,
            Screen::MainMenu | Screen::Calibrate | Screen::DriveMode | Screen::TestMode => {
                self.render_list(d, pressed)?;
            }
        }

        let action = self.header_action();
        let action_pressed = action.is_some_and(|a| pressed == Some(a.hit()));
        widgets::draw_header(d, self.title(), action, action_pressed)
    }

    /// The header action for the current screen, or `None` on the Main Menu.
    const fn header_action(&self) -> Option<HeaderAction> {
        match self.screen {
            Screen::MainMenu => None,
            Screen::ValueEntry(_) => Some(HeaderAction::Cancel),
            Screen::Status => Some(HeaderAction::Stop),
            _ => Some(HeaderAction::Back),
        }
    }

    /// The screen Back/Stop returns to.
    ///
    /// A running screen's parent lives in its [`StatusView`]; every other screen
    /// answers through [`Screen::parent`].
    const fn parent(&self) -> Option<Screen> {
        match self.screen {
            Screen::Status => Some(self.status.parent),
            other => other.parent(),
        }
    }

    /// Whether a press starting at `start` may scroll: the current list
    /// overflows and the press began within the list view below the header.
    const fn press_scrolls(&self, start: Point) -> bool {
        self.max_scroll() > 0 && start.y >= list_top() as i32
    }

    /// Draw the current screen's list of buttons, offset by the scroll, plus the
    /// scroll indicator when the list overflows.
    fn render_list<D>(&self, d: &mut D, pressed: Option<Hit>) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        for (index, item) in self.screen.items().iter().enumerate() {
            let is_pressed = pressed == Some(Hit::MenuItem(index));
            widgets::draw_button(
                d,
                menu_item_rect(index, self.scroll),
                item.label(),
                palette::LABEL_FONT,
                is_pressed,
                false,
            )?;
        }

        let max_scroll = self.max_scroll();
        if max_scroll > 0 {
            widgets::draw_scroll_indicator(d, self.scroll, max_scroll)?;
        }
        Ok(())
    }

    /// Run the action for a completed tap on `hit`, reporting whether the screen
    /// changed.
    fn activate(&mut self, hit: Hit) -> bool {
        match hit {
            // Back and Stop return to the parent, as do a value screen's Cancel
            // and Save; saving is wired in later tickets.
            Hit::Back | Hit::Stop | Hit::Cancel | Hit::Save => self.go_back(),
            Hit::MenuItem(index) => self.open_item(index),
            Hit::NudgeMinus => self.nudge(-1),
            Hit::NudgePlus => self.nudge(1),
            Hit::Slider => false,
        }
    }

    /// Return to the current screen's parent, if it has one.
    fn go_back(&mut self) -> bool {
        self.parent().is_some_and(|parent| self.navigate_to(parent))
    }

    /// Open the item at `index` on the current screen.
    ///
    /// The entry supplies its destination through
    /// [`crate::screens::Item::destination`], so the order the screen draws and the
    /// screen a tap resolves to come from the same enumeration. An index past the
    /// end of the list reports no change, as does any index on a screen with no
    /// entries.
    fn open_item(&mut self, index: usize) -> bool {
        self.screen
            .items()
            .get(index)
            .is_some_and(|item| self.navigate_to(item.destination()))
    }

    /// Switch to `screen`, reset the list scroll and the value-entry value, and
    /// report a needed redraw.
    ///
    /// Entering a value screen sets the value to that flow's preset; leaving one
    /// clears it. Entering Room Scan clears any radar frame and caption from a
    /// previous visit, so a fresh entry starts at "No data".
    const fn navigate_to(&mut self, screen: Screen) -> bool {
        self.screen = screen;
        self.scroll = 0;
        self.value = match screen {
            Screen::ValueEntry(flow) => flow.preset(),
            _ => 0,
        };
        if matches!(screen, Screen::RoomScan) {
            self.radar = None;
            self.sensor_state = SensorState::Off;
        }
        true
    }

    /// Add `delta` pixels to the scroll offset, clamped to `[0, max_scroll]`,
    /// and report whether the offset changed.
    fn scroll_by(&mut self, delta: i32) -> bool {
        let max = i64::from(self.max_scroll());
        let next = (i64::from(self.scroll) + i64::from(delta)).clamp(0, max);
        let next = u32::try_from(next).unwrap_or(self.scroll);
        if next == self.scroll {
            return false;
        }
        self.scroll = next;
        true
    }

    /// Step the value by `steps` of the current flow's step size, clamped to its
    /// range, and report whether it changed.
    fn nudge(&mut self, steps: i32) -> bool {
        let Screen::ValueEntry(flow) = self.screen else {
            return false;
        };
        let next = (self.value + steps * flow.step()).clamp(flow.min(), flow.max());
        if next == self.value {
            return false;
        }
        self.value = next;
        true
    }

    /// Move the value to the slider position for `x`, reporting whether it
    /// changed.
    fn set_slider_value(&mut self, x: i32) -> bool {
        let Screen::ValueEntry(flow) = self.screen else {
            return false;
        };
        let next = slider_value(flow, x);
        if next == self.value {
            return false;
        }
        self.value = next;
        true
    }
}

impl Default for Ui {
    /// Equivalent to [`Ui::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// The interactive region under `p` on a value-entry screen.
fn value_entry_hit(p: Point) -> Option<Hit> {
    for (rect, hit) in [
        (cancel_button_rect(), Hit::Cancel),
        (save_button_rect(), Hit::Save),
        (nudge_minus_rect(), Hit::NudgeMinus),
        (nudge_plus_rect(), Hit::NudgePlus),
        (slider_touch_rect(), Hit::Slider),
    ] {
        if rect.contains(p) {
            return Some(hit);
        }
    }
    None
}
