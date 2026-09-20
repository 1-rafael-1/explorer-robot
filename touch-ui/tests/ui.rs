//! Host-side integration tests for the `touch-ui` crate.
//!
//! These exercise the model's external behaviour: navigation into and back out
//! of every branch, hit-testing each named region, slider and nudge clamping and
//! stepping, the tap-versus-drag decision at its thresholds, and the radar's
//! plotting transform. Drawing is not asserted; it is validated on glass.

use embedded_graphics::{prelude::*, primitives::Rectangle};
use touch_ui::{
    Hit, SensorState, TAP_MAX_MOVE, TAP_MIN_DURATION_MS, Ui,
    geometry::{
        back_button_rect, cancel_button_rect, info_row_rect, menu_item_rect, nudge_minus_rect, nudge_plus_rect,
        panel_rect, progress_bar_rect, readout_rect, room_scan_caption_rect, save_button_rect, scroll_track_rect,
        slider_touch_rect,
    },
    radar,
    screens::{CALIBRATE_MENU, DRIVE_MODE_MENU, MAIN_MENU, PlaceholderKind, Screen, StatusView, TEST_MENU, ValueFlow},
    system_info::{CalibrationStatus, SystemInfo},
};

/// The placeholder each Test Mode index opens, in menu order.
const TEST_LEAVES: [PlaceholderKind; 6] = [
    PlaceholderKind::BasicMotor,
    PlaceholderKind::Turns,
    PlaceholderKind::StraightDrive,
    PlaceholderKind::ArcDrive,
    PlaceholderKind::Imu6Axis,
    PlaceholderKind::Imu9Axis,
];

/// Every placeholder and the submenu it belongs to.
const ALL_PLACEHOLDERS: [(PlaceholderKind, Screen); 9] = [
    (PlaceholderKind::Motor, Screen::Calibrate),
    (PlaceholderKind::Mag, Screen::Calibrate),
    (PlaceholderKind::CoastAndAvoid, Screen::DriveMode),
    (PlaceholderKind::BasicMotor, Screen::TestMode),
    (PlaceholderKind::Turns, Screen::TestMode),
    (PlaceholderKind::StraightDrive, Screen::TestMode),
    (PlaceholderKind::ArcDrive, Screen::TestMode),
    (PlaceholderKind::Imu6Axis, Screen::TestMode),
    (PlaceholderKind::Imu9Axis, Screen::TestMode),
];

/// [`TAP_MAX_MOVE`] as the pixels the pointer moves by.
fn tap_max_move_px() -> i32 {
    i32::try_from(TAP_MAX_MOVE).unwrap_or(i32::MAX)
}

/// The rectangle's inclusive bottom-right corner.
///
/// The rectangles under test are never empty, but embedded-graphics models the
/// corner as an `Option`, so this computes it from the top-left and size.
fn bottom_right(rect: Rectangle) -> Point {
    Point::new(
        rect.top_left.x + i32::try_from(rect.size.width).unwrap_or(i32::MAX) - 1,
        rect.top_left.y + i32::try_from(rect.size.height).unwrap_or(i32::MAX) - 1,
    )
}

/// A tap at `p`: pen-down, no movement, release exactly at the duration floor.
fn tap(ui: &mut Ui, p: Point) {
    let _ = ui.pointer_down(p, 0);
    let _ = ui.pointer_up(TAP_MIN_DURATION_MS);
}

/// Tap the list item at `index` on the current list screen.
fn tap_item(ui: &mut Ui, index: usize) {
    tap(ui, menu_item_rect(index, ui.scroll()).center());
}

/// A point inside the list view, used to start a scroll drag.
const fn in_list() -> Point {
    Point::new(160, 120)
}

/// Drag the visible list upwards until it reaches its maximum scroll.
fn drag_scroll_to_bottom(ui: &mut Ui) {
    let start = in_list();
    let _ = ui.pointer_down(start, 0);
    let _ = ui.pointer_move(Point::new(start.x, start.y - 10_000), 0);
    let _ = ui.pointer_up(0);
}

/// Navigate from the Main Menu into `flow`'s value-entry screen.
fn open_value_entry(flow: ValueFlow) -> Ui {
    let mut ui = Ui::new();
    match flow {
        ValueFlow::DistanceCalibration => {
            tap_item(&mut ui, 1);
            tap_item(&mut ui, 2);
        }
        ValueFlow::AttemptStraight => {
            tap_item(&mut ui, 2);
            tap_item(&mut ui, 1);
        }
    }
    assert_eq!(ui.screen(), Screen::ValueEntry(flow));
    ui
}

/// Test that the UI opens on the Main Menu with nothing scrolled or selected.
#[test]
fn starts_on_the_main_menu() {
    let ui = Ui::new();
    assert_eq!(ui.screen(), Screen::MainMenu);
    assert_eq!(ui.title(), "Main Menu");
    assert_eq!(ui.value(), 0);
    assert_eq!(ui.scroll(), 0);
    assert_eq!(ui.max_scroll(), 0);
}

/// Test that the menu tree matches the robot's labels.
#[test]
fn the_menu_tree_matches_the_robot() {
    assert_eq!(Screen::MainMenu.items(), MAIN_MENU.as_slice());
    assert_eq!(Screen::Calibrate.items(), CALIBRATE_MENU.as_slice());
    assert_eq!(Screen::DriveMode.items(), DRIVE_MODE_MENU.as_slice());
    assert_eq!(Screen::TestMode.items(), TEST_MENU.as_slice());
    assert!(Screen::SystemInfo.items().is_empty());
    assert!(Screen::Placeholder(PlaceholderKind::Motor).items().is_empty());
    assert!(Screen::ValueEntry(ValueFlow::DistanceCalibration).items().is_empty());
}

/// Test that the Main Menu's branches open and Back returns.
#[test]
fn main_menu_branches_open_and_back_returns() {
    for (index, target) in [
        (0, Screen::SystemInfo),
        (1, Screen::Calibrate),
        (2, Screen::DriveMode),
        (3, Screen::TestMode),
    ] {
        let mut ui = Ui::new();
        tap_item(&mut ui, index);
        assert_eq!(ui.screen(), target);
        tap(&mut ui, back_button_rect().center());
        assert_eq!(ui.screen(), Screen::MainMenu);
    }
}

/// Test that the Main Menu has no Back button and no parent.
#[test]
fn the_main_menu_is_the_root() {
    let ui = Ui::new();
    assert_eq!(Screen::MainMenu.parent(), None);
    assert_eq!(ui.hit_test(back_button_rect().center()), None);
}

/// Test that System Info opens and Back returns.
#[test]
fn system_info_opens_and_back_returns() {
    let mut ui = Ui::new();
    tap_item(&mut ui, 0);
    assert_eq!(ui.screen(), Screen::SystemInfo);
    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Back));
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::MainMenu);
}

/// Test that every Calibrate leaf opens and Back returns.
#[test]
fn calibrate_leaves_open_and_back_returns() {
    let mut ui = Ui::new();
    tap_item(&mut ui, 1);
    assert_eq!(ui.screen(), Screen::Calibrate);

    tap_item(&mut ui, 0);
    assert_eq!(ui.screen(), Screen::Placeholder(PlaceholderKind::Motor));
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::Calibrate);

    tap_item(&mut ui, 1);
    assert_eq!(ui.screen(), Screen::Placeholder(PlaceholderKind::Mag));
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::Calibrate);

    tap_item(&mut ui, 2);
    assert_eq!(ui.screen(), Screen::ValueEntry(ValueFlow::DistanceCalibration));
    tap(&mut ui, cancel_button_rect().center());
    assert_eq!(ui.screen(), Screen::Calibrate);

    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::MainMenu);
}

/// Test that every Drive Mode leaf opens and Back returns.
#[test]
fn drive_mode_leaves_open_and_back_returns() {
    let mut ui = Ui::new();
    tap_item(&mut ui, 2);
    assert_eq!(ui.screen(), Screen::DriveMode);

    tap_item(&mut ui, 0);
    assert_eq!(ui.screen(), Screen::Placeholder(PlaceholderKind::CoastAndAvoid));
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::DriveMode);

    tap_item(&mut ui, 1);
    assert_eq!(ui.screen(), Screen::ValueEntry(ValueFlow::AttemptStraight));
    tap(&mut ui, save_button_rect().center());
    assert_eq!(ui.screen(), Screen::DriveMode);

    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::MainMenu);
}

/// Test that every Test Mode leaf opens and Back returns, scrolling the later
/// leaves into reach.
#[test]
fn test_mode_leaves_open_and_back_returns() {
    for (index, kind) in TEST_LEAVES.iter().enumerate() {
        let mut ui = Ui::new();
        tap_item(&mut ui, 3);
        assert_eq!(ui.screen(), Screen::TestMode);

        if index >= 3 {
            drag_scroll_to_bottom(&mut ui);
        }
        tap_item(&mut ui, index);
        assert_eq!(ui.screen(), Screen::Placeholder(*kind));

        tap(&mut ui, back_button_rect().center());
        assert_eq!(ui.screen(), Screen::TestMode);
        tap(&mut ui, back_button_rect().center());
        assert_eq!(ui.screen(), Screen::MainMenu);
    }
}

/// Test that every placeholder names the action it would run and knows its
/// parent.
#[test]
fn placeholders_name_the_action_they_would_run() {
    for (kind, parent) in ALL_PLACEHOLDERS {
        assert!(kind.body().starts_with("Would run:"));
        assert!(!kind.title().is_empty());
        assert_eq!(kind.parent(), parent);
    }
}

/// Test hit-testing on a list screen: items, Back, and the empty gap between.
#[test]
fn list_screens_report_their_regions() {
    let mut ui = Ui::new();
    assert_eq!(ui.hit_test(menu_item_rect(0, 0).center()), Some(Hit::MenuItem(0)));
    assert_eq!(ui.hit_test(menu_item_rect(3, 0).center()), Some(Hit::MenuItem(3)));
    // Below the last item, and in the gap between two items, is no region.
    assert_eq!(ui.hit_test(Point::new(160, 98)), None);
    assert_eq!(ui.hit_test(menu_item_rect(4, 0).center()), None);

    tap_item(&mut ui, 1);
    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Back));
}

/// Test hit-testing on the value screen: every named region is reachable and
/// the readout is not.
#[test]
fn value_screen_reports_its_regions() {
    let ui = open_value_entry(ValueFlow::DistanceCalibration);
    assert_eq!(ui.hit_test(cancel_button_rect().center()), Some(Hit::Cancel));
    assert_eq!(ui.hit_test(save_button_rect().center()), Some(Hit::Save));
    assert_eq!(ui.hit_test(nudge_minus_rect().center()), Some(Hit::NudgeMinus));
    assert_eq!(ui.hit_test(nudge_plus_rect().center()), Some(Hit::NudgePlus));
    assert_eq!(ui.hit_test(slider_touch_rect().center()), Some(Hit::Slider));
    assert_eq!(ui.hit_test(readout_rect().center()), None);
    // The header's wider Cancel button covers the Back rectangle.
    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Cancel));
}

/// Test hit-testing on a running screen: the header offers Stop.
#[test]
fn a_running_screen_reports_stop() {
    let mut ui = Ui::new();
    assert!(ui.show_status(StatusView::new("Basic Motor Test", "Running", Screen::TestMode)));
    assert_eq!(ui.screen(), Screen::Status);
    assert_eq!(ui.title(), "Basic Motor Test");
    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Stop));
    assert_eq!(ui.hit_test(info_row_rect(0).center()), None);
}

/// Test that a running screen updates its body and Stop returns to its parent.
#[test]
fn a_running_screen_stops_back_to_its_parent() {
    let mut ui = Ui::new();
    tap_item(&mut ui, 3);
    assert!(ui.show_status(StatusView::new("Basic Motor Test", "Running", Screen::TestMode)));
    assert_eq!(ui.status_view().map(|view| view.body), Some("Running"));

    assert!(ui.set_status_body("Stopping"));
    assert!(!ui.set_status_body("Stopping"));
    assert_eq!(ui.status_view().map(|view| view.body), Some("Stopping"));

    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::TestMode);
}

/// Test that the running screen's content is replaced wholesale, that progress
/// rides along with it, and that the update reports only real changes.
#[test]
fn a_running_screen_reports_its_progress() {
    let mut ui = Ui::new();
    let view = StatusView::new("Turns Test", "Turning 90 deg", Screen::TestMode).with_progress(25);
    assert!(ui.show_status(view));
    assert_eq!(ui.status_view().map(|view| view.progress), Some(Some(25)));

    assert!(ui.set_status(view.with_progress(50)));
    assert_eq!(ui.status_view().map(|view| view.progress), Some(Some(50)));
    assert!(!ui.set_status(view.with_progress(50)));
    assert_eq!(ui.status_view().and_then(|view| view.progress), Some(50));

    // Leaving the running screen makes the update a no-op.
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::TestMode);
    assert!(!ui.set_status(view));
}

/// Test that the progress bar is drawn inside the status panel and nowhere else.
#[test]
fn the_progress_bar_sits_inside_the_status_panel() {
    let bar = progress_bar_rect();
    let panel = panel_rect();
    assert!(bar.top_left.x > panel.top_left.x);
    assert!(bar.size.width < panel.size.width);
    assert!(bottom_right(bar).y < bottom_right(panel).y);
}

/// Test that a completed tap is reported as the region it activated, and that a
/// drag, a slider drag, and a release that does not activate report nothing.
#[test]
fn the_ui_reports_the_region_a_tap_activated() {
    let mut ui = Ui::new();
    assert_eq!(ui.last_activation(), None);

    tap_item(&mut ui, 1);
    assert_eq!(ui.last_activation(), Some(Hit::MenuItem(1)));
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.last_activation(), Some(Hit::Back));

    // A drag never activates.
    let start = menu_item_rect(0, 0).center();
    let _ = ui.pointer_down(start, 0);
    let _ = ui.pointer_move(Point::new(start.x, start.y + tap_max_move_px() + 1), 0);
    let _ = ui.pointer_up(TAP_MIN_DURATION_MS);
    assert_eq!(ui.last_activation(), None);

    // Neither does a slider drag.
    let ui = open_value_entry(ValueFlow::DistanceCalibration);
    let track = slider_touch_rect();
    let y = track.center().y;
    let mut ui = ui;
    let _ = ui.pointer_down(Point::new(track.center().x, y), 0);
    let _ = ui.pointer_move(Point::new(bottom_right(track).x, y), 0);
    let _ = ui.pointer_up(TAP_MIN_DURATION_MS);
    assert_eq!(ui.last_activation(), None);
}

/// Test that a flow with a step before its value screen can open that screen
/// with its preset.
#[test]
fn a_flow_can_open_its_value_screen_after_a_first_step() {
    let mut ui = Ui::new();
    assert!(ui.show_value_entry(ValueFlow::DistanceCalibration));
    assert_eq!(ui.screen(), Screen::ValueEntry(ValueFlow::DistanceCalibration));
    assert_eq!(ui.value(), 150);
    tap(&mut ui, cancel_button_rect().center());
    assert_eq!(ui.screen(), Screen::Calibrate);
}

/// Test that Cancel and Save both leave the value screen for its parent.
#[test]
fn cancel_and_save_leave_the_value_screen() {
    let mut ui = open_value_entry(ValueFlow::DistanceCalibration);
    tap(&mut ui, cancel_button_rect().center());
    assert_eq!(ui.screen(), Screen::Calibrate);

    let mut ui = open_value_entry(ValueFlow::AttemptStraight);
    tap(&mut ui, save_button_rect().center());
    assert_eq!(ui.screen(), Screen::DriveMode);
}

/// Test distance calibration: preset, fine steps, and clamping at both ends,
/// with zero a legitimate value.
#[test]
fn distance_calibration_steps_and_clamps() {
    let mut ui = open_value_entry(ValueFlow::DistanceCalibration);
    assert_eq!(ui.value(), 150);

    tap(&mut ui, nudge_plus_rect().center());
    assert_eq!(ui.value(), 151);
    tap(&mut ui, nudge_minus_rect().center());
    tap(&mut ui, nudge_minus_rect().center());
    assert_eq!(ui.value(), 149);

    let track = slider_touch_rect();
    let y = track.center().y;

    // Grab the slider and drag past the left edge: zero is a legitimate value
    // and does not close the screen.
    let _ = ui.pointer_down(Point::new(track.center().x, y), 0);
    let _ = ui.pointer_move(Point::new(track.top_left.x - 5, y), 0);
    assert_eq!(ui.value(), 0);
    assert_eq!(ui.screen(), Screen::ValueEntry(ValueFlow::DistanceCalibration));
    let _ = ui.pointer_up(TAP_MIN_DURATION_MS);
    tap(&mut ui, nudge_minus_rect().center());
    assert_eq!(ui.value(), 0);
    tap(&mut ui, nudge_plus_rect().center());
    assert_eq!(ui.value(), 1);

    // Drag past the right edge to reach the maximum.
    let _ = ui.pointer_down(Point::new(track.center().x, y), 0);
    let _ = ui.pointer_move(Point::new(bottom_right(track).x + 5, y), 0);
    assert_eq!(ui.value(), 200);
    let _ = ui.pointer_up(TAP_MIN_DURATION_MS);
    tap(&mut ui, nudge_plus_rect().center());
    assert_eq!(ui.value(), 200);
}

/// Test attempt-straight distance: its ten-centimetre step and its 10–5000
/// range.
#[test]
fn attempt_straight_steps_by_ten_and_clamps() {
    let mut ui = open_value_entry(ValueFlow::AttemptStraight);
    assert_eq!(ui.value(), 100);

    tap(&mut ui, nudge_plus_rect().center());
    assert_eq!(ui.value(), 110);
    tap(&mut ui, nudge_minus_rect().center());
    tap(&mut ui, nudge_minus_rect().center());
    assert_eq!(ui.value(), 90);

    let track = slider_touch_rect();
    let y = track.center().y;

    // Grab the slider and drag past the left edge: the minimum is 10, not zero,
    // and decrementing holds there.
    let _ = ui.pointer_down(Point::new(track.center().x, y), 0);
    let _ = ui.pointer_move(Point::new(track.top_left.x - 5, y), 0);
    assert_eq!(ui.value(), 10);
    let _ = ui.pointer_up(TAP_MIN_DURATION_MS);
    tap(&mut ui, nudge_minus_rect().center());
    assert_eq!(ui.value(), 10);
    tap(&mut ui, nudge_plus_rect().center());
    assert_eq!(ui.value(), 20);

    // Drag past the right edge to reach 5000, and incrementing holds.
    let _ = ui.pointer_down(Point::new(track.center().x, y), 0);
    let _ = ui.pointer_move(Point::new(bottom_right(track).x + 5, y), 0);
    assert_eq!(ui.value(), 5000);
    let _ = ui.pointer_up(TAP_MIN_DURATION_MS);
    tap(&mut ui, nudge_plus_rect().center());
    assert_eq!(ui.value(), 5000);
}

/// Test that a slider press drives the value directly and never navigates.
#[test]
fn a_slider_drag_sets_the_value_without_activating() {
    let mut ui = open_value_entry(ValueFlow::DistanceCalibration);
    let track = slider_touch_rect();
    let y = track.center().y;

    let _ = ui.pointer_down(Point::new(track.center().x, y), 0);
    let moved = ui.pointer_move(Point::new(bottom_right(track).x + 1, y), 0);
    assert!(moved);
    assert_eq!(ui.value(), 200);
    let _ = ui.pointer_up(TAP_MIN_DURATION_MS);
    assert_eq!(ui.screen(), Screen::ValueEntry(ValueFlow::DistanceCalibration));
}

/// Test that a tap within the movement threshold activates.
#[test]
fn a_tap_within_the_move_threshold_activates() {
    let mut ui = Ui::new();
    let start = menu_item_rect(0, 0).center();
    let max = tap_max_move_px();

    let _ = ui.pointer_down(start, 0);
    let _ = ui.pointer_move(Point::new(start.x + max, start.y), 0);
    assert!(ui.pointer_up(TAP_MIN_DURATION_MS));
    assert_eq!(ui.screen(), Screen::SystemInfo);
}

/// Test that movement past the threshold is a drag and never activates.
#[test]
fn movement_past_the_threshold_is_a_drag() {
    let mut ui = Ui::new();
    let start = menu_item_rect(0, 0).center();
    let max = tap_max_move_px();

    let _ = ui.pointer_down(start, 0);
    let _ = ui.pointer_move(Point::new(start.x + max + 1, start.y), 0);
    // A drag still redraws on release (the pressed highlight must clear), but
    // it does not open the item.
    assert!(ui.pointer_up(TAP_MIN_DURATION_MS));
    assert_eq!(ui.screen(), Screen::MainMenu);
}

/// Test that a release before the duration floor is not a tap, and that exactly
/// at the floor it is.
#[test]
fn the_tap_duration_floor_is_inclusive() {
    let mut ui = Ui::new();
    let start = menu_item_rect(0, 0).center();

    let _ = ui.pointer_down(start, 1_000);
    let _ = ui.pointer_up(1_000 + TAP_MIN_DURATION_MS - 1);
    assert_eq!(ui.screen(), Screen::MainMenu);

    let _ = ui.pointer_down(start, 2_000);
    let _ = ui.pointer_up(2_000 + TAP_MIN_DURATION_MS);
    assert_eq!(ui.screen(), Screen::SystemInfo);
}

/// Test that a release on a different region than the press began on does not
/// activate.
#[test]
fn a_release_off_the_press_region_does_not_activate() {
    let mut ui = Ui::new();
    // Press on the last row of the first item and release inside the next item,
    // staying within the movement threshold so only the region change can
    // reject the tap.
    let start = Point::new(160, bottom_right(menu_item_rect(0, 0)).y);
    let released_at = Point::new(start.x, start.y + 9);
    let _ = ui.pointer_down(start, 0);
    let _ = ui.pointer_move(released_at, 0);
    assert_eq!(ui.hit_test(released_at), Some(Hit::MenuItem(1)));
    let _ = ui.pointer_up(TAP_MIN_DURATION_MS);
    assert_eq!(ui.screen(), Screen::MainMenu);
}

/// Test that dragging a long list scrolls within clamped bounds.
#[test]
fn dragging_a_long_list_clamps_its_scroll() {
    let mut ui = Ui::new();
    tap_item(&mut ui, 3);
    assert_eq!(ui.screen(), Screen::TestMode);
    assert!(ui.max_scroll() > 0);

    let start = in_list();
    let _ = ui.pointer_down(start, 0);
    let _ = ui.pointer_move(Point::new(start.x, start.y - 10_000), 0);
    let _ = ui.pointer_up(0);
    assert_eq!(ui.scroll(), ui.max_scroll());

    let _ = ui.pointer_down(start, 0);
    let _ = ui.pointer_move(Point::new(start.x, start.y + 10_000), 0);
    let _ = ui.pointer_up(0);
    assert_eq!(ui.scroll(), 0);
}

/// Test that the scroll indicator's track exists only over the list view.
#[test]
fn the_scroll_track_sits_beside_the_list() {
    let track = scroll_track_rect();
    let first = menu_item_rect(0, 0);
    assert!(track.top_left.x > bottom_right(first).x);
}

/// Test the neutral System Info snapshot's row rendering.
#[test]
fn system_info_renders_the_neutral_snapshot() {
    let info = SystemInfo {
        battery_level: Some(87),
        battery_voltage: Some(7.4),
        motor_calibration: CalibrationStatus::Loaded,
        mag_calibration: CalibrationStatus::Unknown,
        distance_calibration: CalibrationStatus::Missing,
    };
    let rows = info.rows();
    assert_eq!(rows[0].as_str(), "Batt  87%");
    assert_eq!(rows[1].as_str(), "Batt  7.4V");
    assert_eq!(rows[2].as_str(), "Motor: Loaded");
    assert_eq!(rows[3].as_str(), "Mag: Unknown");
    assert_eq!(rows[4].as_str(), "Dist: Missing");
}

/// Test that an unread snapshot reads as explicitly unknown, not as dummy
/// values.
#[test]
fn system_info_defaults_are_explicitly_unknown() {
    let rows = SystemInfo::new().rows();
    assert_eq!(rows[0].as_str(), "Batt --%");
    assert_eq!(rows[1].as_str(), "Batt --.-V");
    assert_eq!(rows[2].as_str(), "Motor: Unknown");
    assert_eq!(rows[3].as_str(), "Mag: Unknown");
    assert_eq!(rows[4].as_str(), "Dist: Unknown");
}

/// Test that the UI holds the System Info snapshot the firmware supplies.
#[test]
fn the_ui_reports_the_system_info_snapshot_it_was_given() {
    let mut ui = Ui::new();
    let info = SystemInfo {
        battery_level: Some(50),
        ..SystemInfo::new()
    };
    assert!(ui.set_system_info(info));
    assert!(!ui.set_system_info(info));
    assert_eq!(ui.system_info(), &info);
}

/// Open the Room Scan screen by tapping its entry in the Test Mode menu.
fn open_room_scan() -> Ui {
    let mut ui = Ui::new();
    tap_item(&mut ui, 3);
    drag_scroll_to_bottom(&mut ui);
    tap_item(&mut ui, ROOM_SCAN_INDEX);
    assert_eq!(ui.screen(), Screen::RoomScan);
    ui
}

/// The Room Scan entry's index in the Test Mode menu.
const ROOM_SCAN_INDEX: usize = 6;

/// Test that the Room Scan entry opens from the Test Mode menu and Back returns.
#[test]
fn room_scan_opens_from_test_mode_and_back_returns() {
    assert_eq!(TEST_MENU[ROOM_SCAN_INDEX], "Room Scan");
    assert_eq!(Screen::RoomScan.parent(), Some(Screen::TestMode));
    assert!(Screen::RoomScan.items().is_empty());

    let mut ui = open_room_scan();
    assert_eq!(ui.title(), "Room Scan");
    // The radar screen offers Back, not Stop.
    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Back));
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::TestMode);
}

/// Test that the model holds the radar frame and sensor caption it was given, and
/// that a fresh entry starts with no frame rather than a stale one.
#[test]
fn room_scan_holds_the_radar_frame_and_sensor_state() {
    let mut ui = open_room_scan();
    assert!(ui.radar().is_none());
    assert_eq!(ui.sensor_state(), SensorState::Off);

    let mut slots: radar::Slots = [None; radar::SLOTS];
    slots[0] = Some(120.0);
    assert!(ui.set_sensor_state(SensorState::Warming));
    assert!(!ui.set_sensor_state(SensorState::Warming));
    assert!(ui.set_radar(Some(slots)));
    assert!(!ui.set_radar(Some(slots)));
    assert_eq!(ui.radar(), Some(&slots));
    assert_eq!(ui.sensor_state(), SensorState::Warming);

    // Leaving the screen makes both updates no-ops, and re-entering clears them.
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::TestMode);
    assert!(!ui.set_radar(None));
    assert!(!ui.set_sensor_state(SensorState::Failed));

    drag_scroll_to_bottom(&mut ui);
    tap_item(&mut ui, ROOM_SCAN_INDEX);
    assert_eq!(ui.screen(), Screen::RoomScan);
    assert!(ui.radar().is_none());
    assert_eq!(ui.sensor_state(), SensorState::Off);
}

/// Test that the four sensor states are distinguishable and the caption sits at
/// the bottom of the radar screen.
#[test]
fn the_sensor_states_are_distinguishable() {
    let labels = [
        SensorState::Off.label(),
        SensorState::Warming.label(),
        SensorState::Streaming.label(),
        SensorState::Failed.label(),
    ];
    for (index, label) in labels.iter().enumerate() {
        assert!(!label.is_empty());
        assert!(!labels[..index].contains(label), "duplicate sensor label {label}");
    }

    let caption = room_scan_caption_rect();
    assert_eq!(caption.top_left.x, 0);
    assert_eq!(caption.size.width, 320);
    assert_eq!(
        caption.top_left.y + i32::try_from(caption.size.height).unwrap_or(i32::MAX),
        240
    );
}

/// Test that the model can leave a screen through its parent, as the firmware
/// does when it refuses an entry.
#[test]
fn back_leaves_for_the_parent() {
    let mut ui = open_room_scan();
    assert!(ui.back());
    assert_eq!(ui.screen(), Screen::TestMode);

    // The Main Menu is the root: there is nowhere to go back to.
    let mut ui = Ui::new();
    assert!(!ui.back());
    assert_eq!(ui.screen(), Screen::MainMenu);
}

/// Test that a missing snapshot is reported as no frame, not as an empty room.
#[test]
fn a_missing_snapshot_is_no_data() {
    let mut ui = open_room_scan();
    let empty: radar::Slots = [None; radar::SLOTS];

    // An all-`None` frame is a real measurement of no returns, distinct from an
    // absent snapshot.
    assert!(ui.set_radar(Some(empty)));
    assert_eq!(ui.radar(), Some(&empty));

    // Dropping back to an absent snapshot clears the frame rather than showing
    // the previous measurement.
    assert!(ui.set_radar(None));
    assert!(ui.radar().is_none());
}

/// Test the radar's plotting transform: angle, range, and the outer-ring clamp.
#[test]
fn radar_marks_plot_at_their_angle_and_range() {
    // Dead ahead at 4.8 m: 115 px up the screen from the centre.
    assert_eq!(radar::mark_point(0, 480.0), Point::new(160, 235));
    // A quarter turn counter-clockwise: to the left (the mount reverses both
    // axes).
    assert_eq!(radar::mark_point(90, 480.0), Point::new(45, 120));
    // Beyond the outer ring, the mark clamps onto it.
    assert_eq!(radar::mark_point(0, 1_000.0), Point::new(160, 240));
}

/// Test that the radar's neutral input has the documented slot count.
#[test]
fn the_radar_input_has_three_hundred_and_sixty_slots() {
    assert_eq!(radar::SLOTS, 360);
    let slots: radar::Slots = [None; radar::SLOTS];
    assert_eq!(slots.len(), radar::SLOTS);
}
