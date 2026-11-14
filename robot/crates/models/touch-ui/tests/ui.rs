//! Host-side integration tests for the `touch-ui` crate.
//!
//! These exercise the model's external behaviour: navigation into and back out
//! of every branch, hit-testing each named region, slider and nudge clamping and
//! stepping, the tap-versus-drag decision at its thresholds, and the radar's
//! plotting transform. The radar transform is pinned by a host test (ADR-0012)
//! so a reordering cannot rotate the radar unnoticed; other drawing is not
//! asserted and is validated on glass.

use dmp_cal::{CheckFailure, CheckOutcome, CompassStep};
use embedded_graphics::{
    Pixel, draw_target::DrawTarget, geometry::OriginDimensions, pixelcolor::Rgb565, prelude::*, primitives::Rectangle,
};
use link_state::{
    BridgeFlags, BridgeStatus, ControlOutcome, ControlReply, ControlStatus, LinkPhase, LinkState, PairingCode, Peer,
    Presence, STALE_AFTER_MS, SetupCredentials, Station,
};
use touch_ui::{
    DmpCalibrationSnapshot, DmpCheckFail, DmpCheckState, DmpHint, HeaderStatus, Hit, Item, LinkAction, Procedure,
    PromptAction, PromptView, ScreenEntry, SensorState, Submenu, TAP_MAX_MOVE, TAP_MIN_DURATION_MS, Ui,
    buffer::{TEXT_BUF_LEN, TextBuf},
    geometry::{
        BACK_MARGIN, BACK_W, FB_H, FB_W, HEADER_ACTION_GAP, HEADER_H, HEADER_STATUS_W, SCROLL_MARGIN, SIDE_MARGIN,
        back_button_rect, cancel_button_rect, header_status_rect, header_title_rect, info_row_rect, link_action_rect,
        link_result_rect, max_scroll_for, menu_item_rect, nudge_minus_rect, nudge_plus_rect, panel_rect,
        progress_bar_rect, prompt_action_rect, prompt_done_rect, prompt_instruction_rect, prompt_progress_rect,
        prompt_redo_rect, prompt_status_rect, readout_rect, readout_row_pitch, readout_row_rect,
        room_scan_caption_rect, save_button_rect, scroll_track_rect, slider_touch_rect,
    },
    imu::{Euler, ImuHealth, ImuReadout, ImuReadoutKind, MAX_IMU_ROWS, MagDiagnostics, Vector3},
    palette::{SMALL_FONT, TITLE_FONT},
    radar,
    screens::{Screen, StatusView, ValueFlow},
    system_info::{CalibrationStatus, SystemInfo},
    widgets::{header_title_font, wrap_text},
};

/// A draw target that discards every pixel, so a test can render a screen without
/// depending on the panel or asserting its pixels.
struct DiscardDisplay;

impl OriginDimensions for DiscardDisplay {
    fn size(&self) -> Size {
        Size::new(
            u32::try_from(FB_W).unwrap_or(u32::MAX),
            u32::try_from(FB_H).unwrap_or(u32::MAX),
        )
    }
}

impl DrawTarget for DiscardDisplay {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, _pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        Ok(())
    }
}

/// Render `ui` into [`DiscardDisplay`], asserting only that drawing succeeds.
fn render_ok(ui: &Ui) {
    assert!(ui.render(&mut DiscardDisplay).is_ok());
}

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

/// A title that does not fit the narrowed header falls back to the small font,
/// so it cannot overpaint the header action or the fixed status slot.
#[test]
fn a_long_header_title_falls_back_to_the_small_font() {
    let area = header_title_rect(Some(BACK_W));
    assert_eq!(
        header_title_font("Link", area).character_size,
        TITLE_FONT.character_size,
        "a short title keeps the title font"
    );
    assert_eq!(
        header_title_font("Basic Motor Test", area).character_size,
        SMALL_FONT.character_size,
        "a long title uses the small font"
    );
}

/// Test that the UI opens on the Main Menu with nothing scrolled or selected.
#[test]
fn starts_on_the_main_menu() {
    let ui = Ui::new();
    assert_eq!(ui.screen(), Screen::MainMenu);
    assert_eq!(ui.title(), "Main Menu");
    assert_eq!(ui.value(), 0);
    assert_eq!(ui.scroll(), 0);
    // Five entries do not fit the list view, so the Main Menu scrolls from
    // the top rather than assuming it all fits.
    assert_eq!(ui.max_scroll(), max_scroll_for(Screen::MainMenu.items().len()));
    assert!(ui.max_scroll() > 0, "five Main Menu entries overflow the list view");
}

/// The labels of a screen's entries, in display order.
fn labels(screen: Screen) -> Vec<&'static str> {
    screen.items().iter().map(|item| item.label()).collect()
}

/// Test that the menu tree matches the robot's labels: each list screen exposes
/// its entries in order, and every entry supplies its label.
#[test]
fn the_menu_tree_matches_the_robot() {
    assert_eq!(
        Screen::MainMenu.items(),
        [
            Item::ScreenEntry(ScreenEntry::SystemInfo),
            Item::Submenu(Submenu::Calibrate),
            Item::Submenu(Submenu::DriveMode),
            Item::Submenu(Submenu::TestMode),
            Item::ScreenEntry(ScreenEntry::Link),
        ]
        .as_slice()
    );
    assert_eq!(
        Screen::Calibrate.items(),
        [
            Item::Procedure(Procedure::MotorCalibration),
            Item::Procedure(Procedure::DmpCompassCalibration),
            Item::Procedure(Procedure::DistanceCalibration),
        ]
        .as_slice()
    );
    assert_eq!(
        Screen::DriveMode.items(),
        [
            Item::Procedure(Procedure::CoastAndAvoid),
            Item::Procedure(Procedure::AttemptStraight),
        ]
        .as_slice()
    );
    assert_eq!(
        Screen::TestMode.items(),
        [
            Item::Procedure(Procedure::BasicMotor),
            Item::Procedure(Procedure::Turns),
            Item::Procedure(Procedure::StraightDrive),
            Item::Procedure(Procedure::ArcDrive),
            Item::Procedure(Procedure::Imu6Axis),
            Item::Procedure(Procedure::Imu9Axis),
            Item::ScreenEntry(ScreenEntry::RoomScan),
        ]
        .as_slice()
    );

    // The labels are byte-identical to the strings the screens shipped.
    assert_eq!(
        labels(Screen::MainMenu),
        ["System Info", "Calibrate", "Drive Mode", "Test Mode", "Link"]
    );
    assert_eq!(labels(Screen::Calibrate), ["Motor", "DMP Mag", "Distance"]);
    assert_eq!(labels(Screen::DriveMode), ["Coast & Avoid", "Attempt Straight"]);
    assert_eq!(
        labels(Screen::TestMode),
        [
            "Basic Motor Test",
            "Turns Test",
            "Straight Drive",
            "Arc Drive",
            "IMU Test (6-axis)",
            "IMU Test (9-axis)",
            "Room Scan",
        ]
    );

    assert_eq!(Screen::SystemInfo.items(), []);
    assert_eq!(Screen::Link.items(), []);
    assert_eq!(Screen::ValueEntry(ValueFlow::DistanceCalibration).items(), []);
}

/// Test that every list screen's entries are distinct and each one is labelled.
#[test]
fn every_screens_items_are_distinct_and_labelled() {
    for screen in [Screen::MainMenu, Screen::Calibrate, Screen::DriveMode, Screen::TestMode] {
        let items = screen.items();
        assert!(!items.is_empty(), "{screen:?} has entries");
        for (index, item) in items.iter().enumerate() {
            assert!(!item.label().is_empty(), "{screen:?} entry {index} is labelled");
            assert!(
                !items[..index].contains(item),
                "{screen:?} entry {item:?} is duplicated"
            );
        }
    }
}

/// Test that one enumeration names every menu entry exactly once, so Room Scan
/// and every leaf is a real, labelled entry of a screen.
#[test]
fn every_menu_entry_is_a_listed_item() {
    let listed: Vec<Item> = [Screen::MainMenu, Screen::Calibrate, Screen::DriveMode, Screen::TestMode]
        .into_iter()
        .flat_map(|screen| screen.items().iter().copied())
        .collect();
    assert_eq!(
        listed,
        [
            Item::ScreenEntry(ScreenEntry::SystemInfo),
            Item::Submenu(Submenu::Calibrate),
            Item::Submenu(Submenu::DriveMode),
            Item::Submenu(Submenu::TestMode),
            Item::ScreenEntry(ScreenEntry::Link),
            Item::Procedure(Procedure::MotorCalibration),
            Item::Procedure(Procedure::DmpCompassCalibration),
            Item::Procedure(Procedure::DistanceCalibration),
            Item::Procedure(Procedure::CoastAndAvoid),
            Item::Procedure(Procedure::AttemptStraight),
            Item::Procedure(Procedure::BasicMotor),
            Item::Procedure(Procedure::Turns),
            Item::Procedure(Procedure::StraightDrive),
            Item::Procedure(Procedure::ArcDrive),
            Item::Procedure(Procedure::Imu6Axis),
            Item::Procedure(Procedure::Imu9Axis),
            Item::ScreenEntry(ScreenEntry::RoomScan),
        ]
    );

    // Room Scan and Link are present and labelled.
    assert!(listed.contains(&Item::ScreenEntry(ScreenEntry::RoomScan)));
    assert_eq!(Item::ScreenEntry(ScreenEntry::RoomScan).label(), "Room Scan");
    assert!(listed.contains(&Item::ScreenEntry(ScreenEntry::Link)));
    assert_eq!(Item::ScreenEntry(ScreenEntry::Link).label(), "Link");
}

/// Test that the menus list eleven Procedures: the ten the firmware runs today
/// plus the deferred attempt-straight, and no Procedure twice.
#[test]
fn the_menus_list_every_procedure_once() {
    let procedures: Vec<Procedure> = [Screen::MainMenu, Screen::Calibrate, Screen::DriveMode, Screen::TestMode]
        .into_iter()
        .flat_map(|screen| screen.items().iter().copied())
        .filter_map(Item::as_procedure)
        .collect();
    assert_eq!(procedures.len(), 11);
    for (index, procedure) in procedures.iter().enumerate() {
        assert!(!procedure.label().is_empty(), "{procedure:?} is labelled");
        assert!(!procedures[..index].contains(procedure), "{procedure:?} is listed once");
    }
    // Attempt-straight is modelled, but it is the one the firmware does not start.
    assert!(procedures.contains(&Procedure::AttemptStraight));
}

/// Test that an entry tells a submenu, a Procedure and a screen apart, so a
/// caller reaches the Procedure by identity rather than by position.
#[test]
fn each_entry_knows_whether_it_is_a_procedure() {
    assert_eq!(Item::Submenu(Submenu::DriveMode).as_procedure(), None);
    assert_eq!(Item::ScreenEntry(ScreenEntry::SystemInfo).as_procedure(), None);
    for procedure in [
        Procedure::MotorCalibration,
        Procedure::CoastAndAvoid,
        Procedure::AttemptStraight,
        Procedure::Imu9Axis,
    ] {
        assert_eq!(Item::Procedure(procedure).as_procedure(), Some(procedure));
    }
}

/// Test that Back returns from every submenu the Main Menu lists to the Main
/// Menu.
#[test]
fn back_returns_from_every_submenu() {
    let mut submenus = 0;
    for (index, item) in Screen::MainMenu.items().iter().enumerate() {
        let Item::Submenu(submenu) = item else {
            continue;
        };
        submenus += 1;

        let mut ui = Ui::new();
        tap_item(&mut ui, index);
        assert_eq!(ui.screen(), submenu.destination());
        tap(&mut ui, back_button_rect().center());
        assert_eq!(ui.screen(), Screen::MainMenu);
    }
    assert_eq!(submenus, 3);
}

/// Test that each entry carries its own header title and the screen Back
/// returns to, rather than either being restated by the screen that lists it.
///
/// The parent comes from the entry itself: a test mode is listed on Test Mode and
/// lands back there whether the operator stops it or a run-to-completion test
/// finishes on its own, a calibration lands on Calibrate, and a drive mode on
/// Drive Mode.
#[test]
fn every_entry_names_its_own_title_and_parent() {
    // Every entry the crate ships, with the parent it states for itself.
    let expected: [(Item, Screen); 17] = [
        (Item::ScreenEntry(ScreenEntry::SystemInfo), Screen::MainMenu),
        (Item::Submenu(Submenu::Calibrate), Screen::MainMenu),
        (Item::Submenu(Submenu::DriveMode), Screen::MainMenu),
        (Item::Submenu(Submenu::TestMode), Screen::MainMenu),
        (Item::ScreenEntry(ScreenEntry::Link), Screen::MainMenu),
        (Item::Procedure(Procedure::MotorCalibration), Screen::Calibrate),
        (Item::Procedure(Procedure::DmpCompassCalibration), Screen::Calibrate),
        (Item::Procedure(Procedure::DistanceCalibration), Screen::Calibrate),
        (Item::Procedure(Procedure::CoastAndAvoid), Screen::DriveMode),
        (Item::Procedure(Procedure::AttemptStraight), Screen::DriveMode),
        (Item::Procedure(Procedure::BasicMotor), Screen::TestMode),
        (Item::Procedure(Procedure::Turns), Screen::TestMode),
        (Item::Procedure(Procedure::StraightDrive), Screen::TestMode),
        (Item::Procedure(Procedure::ArcDrive), Screen::TestMode),
        (Item::Procedure(Procedure::Imu6Axis), Screen::TestMode),
        (Item::Procedure(Procedure::Imu9Axis), Screen::TestMode),
        (Item::ScreenEntry(ScreenEntry::RoomScan), Screen::TestMode),
    ];

    // The known table covers exactly the entries the list screens show.
    let listed: Vec<Item> = [Screen::MainMenu, Screen::Calibrate, Screen::DriveMode, Screen::TestMode]
        .into_iter()
        .flat_map(|screen| screen.items().iter().copied())
        .collect();
    assert_eq!(
        listed.len(),
        expected.len(),
        "the known table covers every listed entry"
    );

    for (entry, parent) in expected {
        assert!(listed.contains(&entry), "{entry:?} is listed on a list screen");
        assert_eq!(entry.parent(), Some(parent), "{entry:?} names its own parent");
        assert!(!entry.title().is_empty(), "{entry:?} carries a header title");

        // Every destination but the shared running screen is a screen the UI can
        // be sent to. The header vocabulary is pinned against literals by
        // `the_menu_tree_matches_the_robot`, so it is not compared to itself
        // here. The running screen is opened with `show_status`, not this call.
        if entry.destination() != Screen::Status {
            let mut ui = Ui::new();
            assert!(
                ui.show_screen(entry.destination()),
                "{entry:?} opens a screen the UI can show"
            );
        }
    }
}

/// Test that a caller moves the panel to a screen by naming it, with no pointer
/// sample, rectangle or duration.
#[test]
fn a_caller_shows_a_screen_by_naming_it() {
    let mut ui = Ui::new();
    assert!(ui.show_screen(Screen::SystemInfo));
    assert_eq!(ui.screen(), Screen::SystemInfo);
    assert_eq!(ui.title(), "System Info");

    // Entering a value screen adopts its preset and clears the scroll.
    assert!(ui.show_screen(Screen::ValueEntry(ValueFlow::DistanceCalibration)));
    assert_eq!(ui.value(), 150);
    assert_eq!(ui.scroll(), 0);

    // Entering Room Scan starts from no frame rather than a stale one.
    assert!(ui.show_screen(Screen::RoomScan));
    assert!(ui.radar().is_none());
    assert_eq!(ui.sensor_state(), SensorState::Off);

    assert!(ui.show_screen(Screen::MainMenu));
    assert_eq!(ui.screen(), Screen::MainMenu);
}

/// Test that the interactive flag matches the producers: the Procedures that end
/// only on an explicit stop are interactive, the calibrations and the deferred
/// attempt-straight are not, and a non-Procedure entry never is.
#[test]
fn interactive_procedures_match_their_producers() {
    for procedure in [
        Procedure::CoastAndAvoid,
        Procedure::BasicMotor,
        Procedure::Imu6Axis,
        Procedure::Imu9Axis,
    ] {
        assert!(procedure.interactive(), "{procedure:?} ends only on a stop");
        assert!(Item::Procedure(procedure).interactive());
    }
    for procedure in [
        Procedure::MotorCalibration,
        Procedure::DmpCompassCalibration,
        Procedure::DistanceCalibration,
        Procedure::AttemptStraight,
        Procedure::Turns,
        Procedure::StraightDrive,
        Procedure::ArcDrive,
    ] {
        assert!(!procedure.interactive(), "{procedure:?} ends on its own");
    }

    assert!(!Item::Submenu(Submenu::Calibrate).interactive());
    assert!(!Item::ScreenEntry(ScreenEntry::RoomScan).interactive());
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

/// Test hit-testing on a list screen: items, Back, and the empty gap between.
#[test]
fn list_screens_report_their_regions() {
    let mut ui = Ui::new();
    assert_eq!(
        ui.hit_test(menu_item_rect(0, 0).center()),
        Some(Hit::MenuItem(Item::ScreenEntry(ScreenEntry::SystemInfo)))
    );
    assert_eq!(
        ui.hit_test(menu_item_rect(3, 0).center()),
        Some(Hit::MenuItem(Item::Submenu(Submenu::TestMode)))
    );
    // Below the last item, and in the gap between two items, is no region.
    assert_eq!(ui.hit_test(Point::new(160, 98)), None);
    assert_eq!(ui.hit_test(menu_item_rect(5, 0).center()), None);

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

/// Test that a running Procedure's view takes its title and parent from the
/// entry itself, so a test mode stops back to the Test Mode menu, a calibration
/// to Calibrate and a drive mode to Drive Mode.
#[test]
fn a_procedure_supplies_its_running_views_title_and_parent() {
    for (procedure, parent) in [
        (Procedure::MotorCalibration, Screen::Calibrate),
        (Procedure::DistanceCalibration, Screen::Calibrate),
        (Procedure::CoastAndAvoid, Screen::DriveMode),
        (Procedure::AttemptStraight, Screen::DriveMode),
        (Procedure::BasicMotor, Screen::TestMode),
        (Procedure::Turns, Screen::TestMode),
        (Procedure::StraightDrive, Screen::TestMode),
        (Procedure::ArcDrive, Screen::TestMode),
        (Procedure::Imu6Axis, Screen::TestMode),
        (Procedure::Imu9Axis, Screen::TestMode),
    ] {
        let view = StatusView::for_procedure(procedure, "Running");
        assert_eq!(view.parent, parent, "{procedure:?} states where Stop returns");
    }

    // The calibrations are the ones a calibration lifecycle belongs to.
    for procedure in [
        Procedure::MotorCalibration,
        Procedure::DmpCompassCalibration,
        Procedure::DistanceCalibration,
    ] {
        assert!(procedure.is_calibration(), "{procedure:?} is a calibration");
    }
    for procedure in [
        Procedure::CoastAndAvoid,
        Procedure::AttemptStraight,
        Procedure::BasicMotor,
        Procedure::Turns,
        Procedure::StraightDrive,
        Procedure::ArcDrive,
        Procedure::Imu6Axis,
        Procedure::Imu9Axis,
    ] {
        assert!(!procedure.is_calibration(), "{procedure:?} is not a calibration");
    }
}

/// Test hit-testing on a finished Result Report: the header offers Back, while a
/// screen with a live procedure offers Stop.
#[test]
fn a_finished_report_offers_back() {
    let mut ui = Ui::new();
    let finished = StatusView::new("Motor", "Zero encoder — check wiring", Screen::Calibrate).finished();
    assert!(ui.show_status(finished));
    assert_eq!(ui.screen(), Screen::Status);
    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Back));

    // A live procedure still offers the Touch Stop.
    assert!(ui.show_status(StatusView::new("Basic Motor Test", "Running", Screen::TestMode)));
    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Stop));
}

/// Test that a Manual Drive's running view is a live screen, not a Result
/// Report: it shows the command label as its body and the header offers Stop.
#[test]
fn a_manual_drive_runs_with_stop_and_a_label() {
    let mut label = TextBuf::new();
    label.fill(format_args!("Drive 150 cm"));
    let view = StatusView::new("Manual Drive", "", Screen::MainMenu).with_text_body(label);

    let mut ui = Ui::new();
    assert!(ui.show_status(view));
    assert_eq!(ui.screen(), Screen::Status);
    assert_eq!(ui.title(), "Manual Drive");

    let shown = ui.status_view();
    assert_eq!(shown.as_ref().map(StatusView::body_text), Some("Drive 150 cm"));
    assert_eq!(shown.map(|view| view.finished), Some(false));
    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Stop));
}

/// Test that a runtime-formatted body is part of the view's identity, so a new
/// label reports a change, and that a finished view keeps the body but offers
/// Back instead of Stop.
#[test]
fn a_runtime_body_changes_with_the_view() {
    let mut first = TextBuf::new();
    first.fill(format_args!("Rotate 90 deg"));
    let first = StatusView::new("Manual Drive", "", Screen::MainMenu).with_text_body(first);

    let mut second = TextBuf::new();
    second.fill(format_args!("Rotate -45 deg"));
    let second = StatusView::new("Manual Drive", "", Screen::MainMenu).with_text_body(second);

    let mut ui = Ui::new();
    assert!(ui.show_status(first));
    assert!(ui.set_status(second));
    assert!(!ui.set_status(second));
    let shown = ui.status_view();
    assert_eq!(shown.as_ref().map(StatusView::body_text), Some("Rotate -45 deg"));

    assert!(ui.set_status(second.finished()));
    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Back));
    let shown = ui.status_view();
    assert_eq!(shown.as_ref().map(StatusView::body_text), Some("Rotate -45 deg"));
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
    assert_eq!(
        ui.last_activation(),
        Some(Hit::MenuItem(Item::Submenu(Submenu::Calibrate)))
    );
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
    assert_eq!(
        ui.hit_test(released_at),
        Some(Hit::MenuItem(Item::Submenu(Submenu::Calibrate)))
    );
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
        distance_calibration: CalibrationStatus::Missing,
        dmp_calibration: CalibrationStatus::Loaded,
    };
    let rows = info.rows();
    assert_eq!(rows[0].as_str(), "Batt  87%");
    assert_eq!(rows[1].as_str(), "Batt  7.4V");
    assert_eq!(rows[2].as_str(), "Motor: Loaded");
    assert_eq!(rows[3].as_str(), "Dist: Missing");
    assert_eq!(rows[4].as_str(), "DMP Cal: Loaded");
    assert_eq!(rows.len(), 5);

    // The fifth row must still fit the existing System Info layout.
    let last = info_row_rect(rows.len() - 1);
    assert!(bottom_right(last).y < i32::try_from(FB_H).unwrap_or(i32::MAX));
}

/// Test that an unread snapshot reads as explicitly unknown, not as dummy
/// values.
#[test]
fn system_info_defaults_are_explicitly_unknown() {
    let rows = SystemInfo::new().rows();
    assert_eq!(rows[0].as_str(), "Batt --%");
    assert_eq!(rows[1].as_str(), "Batt --.-V");
    assert_eq!(rows[2].as_str(), "Motor: Unknown");
    assert_eq!(rows[3].as_str(), "Dist: Unknown");
    assert_eq!(rows[4].as_str(), "DMP Cal: Unknown");
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

/// A representative Bridge Status for the Link tests.
const fn sample_bridge_status() -> BridgeStatus {
    BridgeStatus {
        rssi_dbm: Some(-57),
        channel: Some(6),
        station: Station::Associated,
        flags: BridgeFlags {
            provisioned: true,
            setup_open: false,
            link_ready: true,
        },
        uart_dropped_bytes: 1,
        dma_stalls: 2,
        crc_failures: 3,
        resyncs: 4,
        wdt_feeds: 5,
        wdt_withheld: 6,
    }
}

/// The Link entry's index in the Main Menu.
const LINK_INDEX: usize = 4;

/// Open the Link screen by tapping its entry in the Main Menu.
fn open_link() -> Ui {
    let mut ui = Ui::new();
    drag_scroll_to_bottom(&mut ui);
    tap_item(&mut ui, LINK_INDEX);
    ui
}

/// A link state with a live Bridge and a control outcome, ready for a snapshot.
fn link_with_control(control: ControlOutcome) -> LinkState {
    let mut state = LinkState::new();
    state.set_presence(Presence::Ready);
    state.set_peer(Peer::PeerLive);
    state.set_control(control);
    state
}

/// Test that Link opens from the Main Menu and Back returns to the Main Menu.
#[test]
fn link_opens_from_the_main_menu_and_back_returns() {
    assert_eq!(
        Screen::MainMenu.items()[LINK_INDEX],
        Item::ScreenEntry(ScreenEntry::Link)
    );
    assert_eq!(Item::ScreenEntry(ScreenEntry::Link).label(), "Link");
    assert_eq!(Screen::Link.parent(), Some(Screen::MainMenu));
    assert_eq!(Screen::Link.items(), []);

    let mut ui = Ui::new();
    drag_scroll_to_bottom(&mut ui);
    tap_item(&mut ui, LINK_INDEX);
    assert_eq!(ui.screen(), Screen::Link);
    assert_eq!(ui.title(), "Link");
    // The info rows are not tappable, and the header still offers Back.
    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Back));
    assert_eq!(ui.hit_test(info_row_rect(0).center()), None);
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::MainMenu);
}

/// Test that the three Link actions each have a distinct, on-screen region that
/// hit-tests to the action it names.
#[test]
fn the_link_actions_are_distinct_and_hittable() {
    let ui = open_link();
    assert_eq!(ui.screen(), Screen::Link);
    assert_eq!(LinkAction::ALL.len(), 3);

    let rects: Vec<Rectangle> = (0..LinkAction::ALL.len()).map(link_action_rect).collect();
    for (index, rect) in rects.iter().enumerate() {
        assert!(
            rect.top_left.x >= 0 && rect.top_left.y >= 0,
            "action {index} starts on screen"
        );
        let corner = bottom_right(*rect);
        assert!(
            u32::try_from(corner.x).unwrap_or(u32::MAX) < u32::try_from(FB_W).unwrap_or(u32::MAX)
                && u32::try_from(corner.y).unwrap_or(u32::MAX) < u32::try_from(FB_H).unwrap_or(u32::MAX),
            "action {index} ends on screen"
        );
        assert_eq!(
            ui.hit_test(rect.center()),
            Some(Hit::LinkAction(LinkAction::ALL[index]))
        );
        for other in &rects[index + 1..] {
            assert_ne!(rect, other, "action rectangles must stay distinct");
            assert!(!rect.contains(other.center()), "action rectangles must not overlap");
        }
    }

    let result = link_result_rect();
    assert!(
        u32::try_from(bottom_right(result).x).unwrap_or(u32::MAX) < u32::try_from(FB_W).unwrap_or(u32::MAX)
            && u32::try_from(bottom_right(result).y).unwrap_or(u32::MAX) < u32::try_from(FB_H).unwrap_or(u32::MAX),
        "the result band stays on screen"
    );
    for rect in &rects {
        assert!(!rect.contains(result.center()), "actions sit above the result band");
    }
}

/// Test that tapping a Link action reports it as the last activation without
/// leaving the screen (the firmware owns the request).
#[test]
fn tapping_a_link_action_reports_it_and_keeps_the_screen() {
    for (index, action) in LinkAction::ALL.into_iter().enumerate() {
        let mut ui = open_link();
        tap(&mut ui, link_action_rect(index).center());
        assert_eq!(ui.screen(), Screen::Link);
        assert_eq!(ui.last_activation(), Some(Hit::LinkAction(action)));
    }
}

/// Test that a pending control outcome is held and renders.
#[test]
fn a_pending_control_outcome_is_held_and_drawn() {
    let mut ui = open_link();
    assert!(ui.set_link(link_with_control(ControlOutcome::Pending).snapshot(0)));
    assert!(ui.link().control.is_pending());
    assert_eq!(ui.link().control.label(), "Pending");
    render_ok(&ui);
}

/// Test that an answered setup reply keeps the SSID and passphrase, and renders.
#[test]
fn an_answered_setup_reply_keeps_its_credentials() {
    let Some(credentials) = SetupCredentials::new("ExplorerRobot-1A2B", "explorer-setup-passphrase") else {
        return;
    };
    let expected = ControlOutcome::Answered(ControlReply::Setup {
        status: ControlStatus::Ok,
        credentials,
    });

    let mut ui = open_link();
    assert!(ui.set_link(link_with_control(expected.clone()).snapshot(0)));
    assert_eq!(ui.link().control, expected);
    if let ControlOutcome::Answered(ControlReply::Setup { credentials, .. }) = &ui.link().control {
        assert_eq!(credentials.ssid.as_str(), "ExplorerRobot-1A2B");
        assert_eq!(credentials.passphrase.as_str(), "explorer-setup-passphrase");
    }
    render_ok(&ui);
}

/// Test that an answered pairing-code reply keeps the twelve-character code and
/// renders.
#[test]
fn an_answered_pairing_code_reply_keeps_its_code() {
    let Some(code) = PairingCode::new("ABCDEFGHIJKL") else {
        return;
    };
    let expected = ControlOutcome::Answered(ControlReply::PairingCode {
        status: ControlStatus::Ok,
        code,
    });

    let mut ui = open_link();
    assert!(ui.set_link(link_with_control(expected.clone()).snapshot(0)));
    assert_eq!(ui.link().control, expected);
    if let ControlOutcome::Answered(ControlReply::PairingCode { code, .. }) = &ui.link().control {
        assert_eq!(code.code.as_str(), "ABCDEFGHIJKL");
    }
    render_ok(&ui);
}

/// Test that a timed-out control outcome is held and renders.
#[test]
fn a_timed_out_control_outcome_is_held_and_drawn() {
    let mut ui = open_link();
    assert!(ui.set_link(link_with_control(ControlOutcome::TimedOut).snapshot(0)));
    assert_eq!(ui.link().control, ControlOutcome::TimedOut);
    render_ok(&ui);
}

/// Test that an absent Bridge keeps the Link screen, its rows and the three
/// actions usable.
#[test]
fn an_absent_bridge_keeps_the_link_actions_hittable() {
    for presence in [Presence::Off, Presence::WaitingForBoard] {
        let mut state = LinkState::new();
        state.set_presence(presence);
        let mut ui = open_link();
        // The disabled baseline is already held, so offering it again is no change.
        ui.set_link(state.snapshot(0));
        assert!(ui.link().bridge.is_none(), "no Bridge Status is retained");
        assert_eq!(ui.screen(), Screen::Link);

        for (index, action) in LinkAction::ALL.into_iter().enumerate() {
            assert_eq!(
                ui.hit_test(link_action_rect(index).center()),
                Some(Hit::LinkAction(action))
            );
        }
        render_ok(&ui);

        tap(&mut ui, back_button_rect().center());
        assert_eq!(ui.screen(), Screen::MainMenu);
    }
}

/// Test that the model stores the link snapshot and battery it is given and
/// reports only real changes.
#[test]
fn set_link_and_set_battery_report_changes() {
    let mut ui = Ui::new();

    // A fresh UI already holds the disabled baseline, so offering it again is
    // not a change.
    assert!(!ui.set_link(LinkState::new().snapshot(0)));

    let mut state = LinkState::new();
    state.set_presence(Presence::Ready);
    state.set_peer(Peer::PeerLive);
    let live = state.snapshot(0);
    assert!(ui.set_link(live.clone()));
    assert!(!ui.set_link(live));
    assert_eq!(ui.link().phase, LinkPhase::PeerLive);
    assert_eq!(
        ui.header_status(),
        HeaderStatus {
            link: LinkPhase::PeerLive,
            battery: None,
        }
    );

    assert!(ui.set_battery(Some(87)));
    assert!(!ui.set_battery(Some(87)));
    assert_eq!(ui.header_status().battery, Some(87));
    assert!(ui.set_battery(None));
    assert_eq!(ui.header_status().battery, None);
}

/// Test that a fresh UI starts Off with no battery reading.
#[test]
fn a_fresh_ui_has_an_off_link_and_no_battery() {
    let ui = Ui::new();
    assert_eq!(
        ui.header_status(),
        HeaderStatus {
            link: LinkPhase::Off,
            battery: None,
        }
    );
    assert_eq!(ui.link().phase, LinkPhase::Off);
    assert!(ui.link().bridge.is_none());
    assert!(!ui.link().bridge_stale);
}

/// Test that a missing battery stays explicitly unread (`None`) and never
/// changes the screen.
#[test]
fn a_missing_battery_stays_unread_and_leaves_the_screen_alone() {
    let mut ui = Ui::new();
    assert_eq!(ui.header_status().battery, None);
    assert!(!ui.set_battery(None));
    assert_eq!(ui.screen(), Screen::MainMenu);

    // A later reading is a real change, and losing it returns to the unread
    // state without disturbing navigation.
    assert!(ui.set_battery(Some(42)));
    assert_eq!(ui.header_status().battery, Some(42));
    assert!(ui.set_battery(None));
    assert_eq!(ui.header_status().battery, None);
    assert_eq!(ui.screen(), Screen::MainMenu);
}

/// Test that the four link phases stay distinct in the snapshot the model holds.
#[test]
fn the_four_link_phases_are_distinct_in_the_stored_snapshot() {
    let mut ui = Ui::new();
    let mut state = LinkState::new();

    // A fresh UI already holds the disabled baseline, so this is not a change.
    assert!(!ui.set_link(state.snapshot(0)));
    assert_eq!(ui.link().phase, LinkPhase::Off);

    state.set_presence(Presence::WaitingForBoard);
    assert!(ui.set_link(state.snapshot(0)));
    assert_eq!(ui.link().phase, LinkPhase::WaitingForBoard);

    state.set_presence(Presence::Ready);
    assert!(ui.set_link(state.snapshot(0)));
    assert_eq!(ui.link().phase, LinkPhase::ReadyNoPeer);

    state.set_peer(Peer::PeerLive);
    assert!(ui.set_link(state.snapshot(0)));
    assert_eq!(ui.link().phase, LinkPhase::PeerLive);

    let phases = [
        LinkPhase::Off,
        LinkPhase::WaitingForBoard,
        LinkPhase::ReadyNoPeer,
        LinkPhase::PeerLive,
    ];
    for (index, phase) in phases.iter().enumerate() {
        for other in &phases[index + 1..] {
            assert_ne!(phase, other, "link phases must stay distinct");
        }
    }
}

/// Test that an absent Bridge leaves every other screen reachable and usable.
#[test]
fn an_absent_bridge_leaves_every_other_screen_reachable() {
    let mut ui = Ui::new();
    let mut state = LinkState::new();
    state.set_presence(Presence::WaitingForBoard);
    let absent = state.snapshot(0);
    assert!(absent.bridge.is_none());
    assert!(!absent.bridge_stale);
    assert!(ui.set_link(absent));

    for screen in [
        Screen::MainMenu,
        Screen::Calibrate,
        Screen::DriveMode,
        Screen::TestMode,
        Screen::SystemInfo,
        Screen::RoomScan,
        Screen::Link,
        Screen::ValueEntry(ValueFlow::DistanceCalibration),
        Screen::ValueEntry(ValueFlow::AttemptStraight),
    ] {
        assert!(ui.show_screen(screen), "{screen:?} stays reachable");
        assert_eq!(ui.screen(), screen);
    }

    // The running screen is reached by its content, not by `show_screen`.
    assert!(ui.show_status(StatusView::new("Basic Motor Test", "Running", Screen::TestMode)));
    assert_eq!(ui.screen(), Screen::Status);

    // The Link screen still reports the absent Bridge as no status, not stale.
    assert!(ui.show_screen(Screen::Link));
    assert!(ui.link().bridge.is_none());
    assert!(!ui.link().bridge_stale);
}

/// Test that a retained Bridge Status survives a no-Remote period, stays
/// readable when stale, and keeps every field.
#[test]
fn a_retained_stale_bridge_status_is_still_readable() {
    let mut ui = Ui::new();
    let mut state = LinkState::new();
    state.set_presence(Presence::Ready);
    state.set_peer(Peer::PeerLive);
    state.set_bridge_status(sample_bridge_status(), 1_000);

    let fresh = state.snapshot(1_000 + STALE_AFTER_MS);
    assert!(fresh.bridge.is_some(), "the status stays readable");
    assert!(!fresh.bridge_stale, "exactly two seconds old is still fresh");
    assert!(ui.set_link(fresh));

    state.set_peer(Peer::NoPeer);
    let retained = state.snapshot(1_001 + STALE_AFTER_MS);
    assert_eq!(retained.phase, LinkPhase::ReadyNoPeer);
    assert!(retained.bridge.is_some(), "a stale status is still present");
    assert!(retained.bridge_stale, "older than two seconds is stale");
    assert!(ui.set_link(retained));
    assert!(ui.link().bridge_stale);

    let Some(bridge) = ui.link().bridge else {
        return;
    };
    assert_eq!(bridge.rssi_dbm, Some(-57));
    assert_eq!(bridge.channel, Some(6));
    assert_eq!(bridge.station, Station::Associated);
    assert!(bridge.flags.provisioned);
    assert!(!bridge.flags.setup_open);
    assert!(bridge.flags.link_ready);
    assert_eq!(bridge.uart_dropped_bytes, 1);
    assert_eq!(bridge.dma_stalls, 2);
    assert_eq!(bridge.crc_failures, 3);
    assert_eq!(bridge.resyncs, 4);
    assert_eq!(bridge.wdt_feeds, 5);
    assert_eq!(bridge.wdt_withheld, 6);
}

/// Test that the header status slot is a constant rectangle and the title area
/// is bounded by it, so a missing or changing value cannot move the header.
#[test]
fn the_header_status_slot_is_fixed() {
    let slot = header_status_rect();
    assert_eq!(slot.size.width, HEADER_STATUS_W);
    assert_eq!(slot.top_left.y, i32::try_from(BACK_MARGIN).unwrap_or(i32::MAX));
    assert_eq!(slot.size.height, HEADER_H - 2 * BACK_MARGIN);
    assert_eq!(slot, header_status_rect(), "the slot geometry takes no state");
    let right_edge = u32::try_from(FB_W).unwrap_or(u32::MAX) - SCROLL_MARGIN - 1;
    assert_eq!(
        bottom_right(slot).x,
        i32::try_from(right_edge).unwrap_or(i32::MAX),
        "the slot is flush with the right margin"
    );

    assert_eq!(header_title_rect(None).top_left.x, 0);
    let with_action = header_title_rect(Some(BACK_W));
    assert_eq!(
        with_action.top_left.x,
        i32::try_from(SIDE_MARGIN + BACK_W + HEADER_ACTION_GAP).unwrap_or(i32::MAX),
        "the title starts after the action"
    );
    assert_eq!(
        with_action.top_left.x + i32::try_from(with_action.size.width).unwrap_or(i32::MAX),
        slot.top_left.x,
        "the title area ends at the fixed slot"
    );
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
    assert_eq!(
        Screen::TestMode.items()[ROOM_SCAN_INDEX],
        Item::ScreenEntry(ScreenEntry::RoomScan)
    );
    assert_eq!(Item::ScreenEntry(ScreenEntry::RoomScan).label(), "Room Scan");
    assert_eq!(Screen::RoomScan.parent(), Some(Screen::TestMode));
    assert_eq!(Screen::RoomScan.items(), []);

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
    assert!(ui.set_radar(Some((&slots, 1))));
    assert_eq!(ui.radar(), Some(&slots));
    assert_eq!(ui.sensor_state(), SensorState::Warming);

    // Leaving the screen makes both updates no-ops.
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.screen(), Screen::TestMode);
    assert!(!ui.set_radar(Some((&slots, 1))));
    assert!(!ui.set_sensor_state(SensorState::Failed));

    // Re-entering clears the frame, its sequence and the caption, so a fresh entry
    // starts at "No data" and the first frame reports a change even when it carries
    // a sequence the model saw on the previous visit.
    drag_scroll_to_bottom(&mut ui);
    tap_item(&mut ui, ROOM_SCAN_INDEX);
    assert_eq!(ui.screen(), Screen::RoomScan);
    assert!(ui.radar().is_none());
    assert_eq!(ui.sensor_state(), SensorState::Off);
    assert!(ui.set_radar(Some((&slots, 1))));
    assert_eq!(ui.radar(), Some(&slots));
}

/// Test that the cloud's sequence is the frame's change token: the frame the model
/// already holds reports no change, and a new sequence always reports one.
#[test]
fn the_radar_sequence_is_the_change_token() {
    let mut ui = open_room_scan();
    let mut slots: radar::Slots = [None; radar::SLOTS];
    slots[0] = Some(120.0);

    assert!(ui.set_radar(Some((&slots, 7))));
    // The same frame at the same sequence is the frame already drawn: no change,
    // and the slots are not copied again.
    assert!(!ui.set_radar(Some((&slots, 7))));

    // A new sequence reports a change and the model takes the frame that came
    // with it.
    let mut next: radar::Slots = [None; radar::SLOTS];
    next[0] = Some(90.0);
    assert!(ui.set_radar(Some((&next, 8))));
    assert_eq!(ui.radar().map(|frame| frame[0]), Some(Some(90.0)));

    // A sequence the model already holds is that frame, whatever slots accompany
    // it: the counter, not the array, decides.
    let mut other: radar::Slots = [None; radar::SLOTS];
    other[0] = Some(30.0);
    assert!(!ui.set_radar(Some((&other, 8))));
    assert_eq!(ui.radar().map(|frame| frame[0]), Some(Some(90.0)));
}

/// Test that the model owns the frame it draws: the caller's array is copied in,
/// so reusing that array afterwards cannot change what the model shows.
#[test]
fn the_model_owns_the_radar_frame_it_draws() {
    let mut ui = open_room_scan();
    let mut source: radar::Slots = [None; radar::SLOTS];
    source[0] = Some(120.0);

    assert!(ui.set_radar(Some((&source, 3))));
    // The caller reuses its buffer for the next scene.
    source[0] = Some(1.0);
    source[90] = Some(250.0);
    assert_eq!(source[0], Some(1.0));
    assert_eq!(source[90], Some(250.0));

    assert_eq!(ui.radar().map(|frame| frame[0]), Some(Some(120.0)));
    assert_eq!(ui.radar().map(|frame| frame[90]), Some(None));
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
        assert_ne!(*label, "");
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

/// Test that a missing snapshot is reported as no frame, not as an empty room, and
/// that an absent frame stays distinguishable from a measured frame with no
/// returns.
#[test]
fn a_missing_snapshot_is_no_data() {
    let mut ui = open_room_scan();

    // An absent snapshot is the state a fresh entry starts in, and offering it
    // again reports no change.
    assert!(ui.radar().is_none());
    assert!(!ui.set_radar(None));

    // An all-`None` frame is a real measurement of no returns, distinct from an
    // absent snapshot: it is stored and kept, not mistaken for one.
    let empty: radar::Slots = [None; radar::SLOTS];
    assert!(ui.set_radar(Some((&empty, 1))));
    assert_eq!(ui.radar(), Some(&empty));

    // Dropping back to an absent snapshot clears the frame rather than showing
    // the previous measurement, so the screen draws "No data" instead of the
    // empty room the old frame drew.
    assert!(ui.set_radar(None));
    assert!(ui.radar().is_none());
    // Already absent: there is nothing to clear, so nothing reported a change.
    assert!(!ui.set_radar(None));
}

/// Test the radar's plotting transform: angle, range, and the outer-ring clamp.
#[test]
fn radar_marks_plot_at_their_angle_and_range() {
    // Dead ahead at 4.8 m: 115 px up the screen from the centre.
    assert_eq!(radar::mark_point(0, 480.0), Point::new(160, 5));
    // A quarter turn clockwise: to the right, because slot zero is drawn at the
    // top and the slots advance clockwise on the glass (ADR-0012).
    assert_eq!(radar::mark_point(90, 480.0), Point::new(275, 120));
    // Beyond the outer ring, the mark clamps onto it.
    assert_eq!(radar::mark_point(0, 1_000.0), Point::new(160, 0));
}

/// Test that pins the radar's direction convention (ADR-0012): slot zero is
/// drawn at the top of the radar and the following slots advance clockwise on
/// the glass, so a future reordering cannot rotate the radar unnoticed.
#[test]
fn radar_slots_advance_clockwise_from_the_top() {
    // Slot zero is dead ahead: straight up from the centre, at the same X.
    let top = radar::mark_point(0, 480.0);
    assert_eq!(top.x, 160);
    assert!(top.y < 120, "slot 0 must plot above the centre, got {top:?}");

    // Slot 90 is a quarter turn clockwise of it: to the right, at the same Y.
    let right = radar::mark_point(90, 480.0);
    assert!(right.x > 160, "slot 90 must plot right of the centre, got {right:?}");
    assert_eq!(right.y, 120);
}

/// Test that the radar's neutral input has the documented slot count.
#[test]
fn the_radar_input_has_three_hundred_and_sixty_slots() {
    assert_eq!(radar::SLOTS, 360);
    let slots: radar::Slots = [None; radar::SLOTS];
    assert_eq!(slots.len(), radar::SLOTS);
}

/// A synthetic ASCII row exactly as wide as the buffer's documented minimum:
/// 48 characters. It is not a real rendered row, only a round-trip probe.
const FORTY_EIGHT_CHAR_ROW: &str = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789abcdefghijkl";

/// Test that a 48-character readout row formats and reads back intact, so the
/// IMU Test Readout can draw a row without truncation.
#[test]
fn a_forty_eight_character_readout_row_round_trips() {
    assert_eq!(FORTY_EIGHT_CHAR_ROW.len(), 48);
    assert_eq!(TEXT_BUF_LEN, 48);

    let mut row = TextBuf::new();
    row.fill(format_args!("{FORTY_EIGHT_CHAR_ROW}"));
    assert_eq!(row.as_str(), FORTY_EIGHT_CHAR_ROW);
}

/// Test that a row wider than the buffer still clears rather than panicking.
#[test]
fn an_over_long_row_clears_the_buffer_without_panicking() {
    let over_long = format_args!("{FORTY_EIGHT_CHAR_ROW}!");
    let mut row = TextBuf::new();
    row.fill(over_long);
    assert_eq!(row.as_str(), "");
}

/// Test that the widest existing value-entry and Link rows keep their text; the
/// System Info rows are pinned by
/// [`system_info_renders_the_neutral_snapshot`] and
/// [`system_info_defaults_are_explicitly_unknown`].
#[test]
fn the_widest_existing_value_entry_and_link_rows_are_unchanged() {
    let mut readout = TextBuf::new();
    readout.set(5000, "cm");
    assert_eq!(readout.as_str(), "5000 cm");

    let mut presence = TextBuf::new();
    presence.fill(format_args!("Bridge: Waiting for board"));
    assert_eq!(presence.as_str(), "Bridge: Waiting for board");

    let mut station = TextBuf::new();
    station.fill(format_args!("Station: Associated"));
    assert_eq!(station.as_str(), "Station: Associated");
}

/// A fully populated six-axis readout: Euler, accelerometer and both gyros.
const fn sample_readout() -> ImuReadout {
    ImuReadout::waiting(ImuReadoutKind::SixAxis)
        .with_health(ImuHealth::Live, 0)
        .with_orientation(Euler {
            yaw: 12.3,
            pitch: -45.6,
            roll: 78.9,
        })
        .with_raw_accel(Vector3 {
            x: 0.01,
            y: -0.02,
            z: 1.0,
        })
        .with_raw_gyro(Vector3 {
            x: 1.2,
            y: -3.4,
            z: 5.6,
        })
        .with_calibrated_gyro(Vector3 {
            x: -1.2,
            y: 3.4,
            z: -5.6,
        })
}

/// Test that the six-axis readout renders exactly five rows, in order, with the
/// agreed labels, precision and units.
#[test]
fn the_six_axis_readout_has_five_rows_in_order() {
    let readout = sample_readout();
    assert_eq!(readout.row_count(), 5);

    let rows = readout.rows();
    let expected = [
        "Euler yaw=12.3 pitch=-45.6 roll=78.9",
        "Accel x=0.01 y=-0.02 z=1.00 g",
        "Gyro raw x=1.2 y=-3.4 z=5.6 dps",
        "Gyro cal x=-1.2 y=3.4 z=-5.6 dps",
        "Live miss=0 dmpcal n/a",
    ];
    for (row, expected) in rows.iter().zip(expected) {
        assert_eq!(row.as_str(), expected);
    }
}

/// Test that before the first sample every value shows `--` and the health row
/// says Waiting, so zeros are never mistaken for readings.
#[test]
fn the_six_axis_readout_shows_placeholders_before_the_first_sample() {
    let readout = ImuReadout::waiting(ImuReadoutKind::SixAxis);

    let rows = readout.rows();
    let expected = [
        "Euler yaw=-- pitch=-- roll=--",
        "Accel x=-- y=-- z=-- g",
        "Gyro raw x=-- y=-- z=-- dps",
        "Gyro cal x=-- y=-- z=-- dps",
        "Waiting miss=0 dmpcal n/a",
    ];
    for (row, expected) in rows.iter().zip(expected) {
        assert_eq!(row.as_str(), expected);
    }
}

/// Test that the health row distinguishes Waiting, Live and a silent IMU, and
/// that the missing count is capped at four digits.
#[test]
fn the_health_row_names_waiting_live_and_not_responding() {
    for (health, missing, expected) in [
        (ImuHealth::Waiting, 0, "Waiting miss=0 dmpcal n/a"),
        (ImuHealth::Live, 7, "Live miss=7 dmpcal n/a"),
        (ImuHealth::NotResponding, 100, "IMU not responding miss=100 dmpcal n/a"),
        (
            ImuHealth::NotResponding,
            10_000,
            "IMU not responding miss=9999 dmpcal n/a",
        ),
    ] {
        let readout = ImuReadout::waiting(ImuReadoutKind::SixAxis).with_health(health, missing);
        assert_eq!(readout.rows()[4].as_str(), expected);
    }
}

/// Test that a same-value readout reports no change while a changed health or
/// value reports one, so the 10 Hz tick flushes only on a visible change.
#[test]
fn a_readout_redraws_only_when_a_displayed_value_changes() {
    let view = StatusView::for_procedure(Procedure::Imu6Axis, "6-axis streaming").with_readout(sample_readout());
    let mut ui = Ui::new();
    assert!(ui.show_status(view));
    assert!(!ui.set_status(view));

    let not_responding = view.with_readout(sample_readout().with_health(ImuHealth::NotResponding, 100));
    assert!(ui.set_status(not_responding));
    assert!(!ui.set_status(not_responding));

    let moved = view.with_readout(sample_readout().with_orientation(Euler {
        yaw: 13.3,
        pitch: -45.6,
        roll: 78.9,
    }));
    assert!(ui.set_status(moved));
}

/// Test that a `StatusView` carrying a readout renders into the existing discard
/// target.
#[test]
fn a_running_screen_with_a_readout_renders() {
    let view = StatusView::for_procedure(Procedure::Imu6Axis, "6-axis streaming").with_readout(sample_readout());
    let mut ui = Ui::new();
    assert!(ui.show_status(view));
    render_ok(&ui);
}

/// Test that the six-axis row block fits inside the status panel, with the last
/// row strictly inside its top and bottom edges, and that the pitch compresses
/// for a full twelve-row readout while the five-row block is capped and centred.
#[test]
fn the_readout_rows_fit_inside_the_status_panel() {
    let panel = panel_rect();

    let last = readout_row_rect(4, 5);
    assert!(last.top_left.y > panel.top_left.y);
    assert!(bottom_right(last).y < bottom_right(panel).y);

    assert_eq!(readout_row_pitch(5), 26);
    assert_eq!(readout_row_pitch(MAX_IMU_ROWS), 11);
    let last_full = readout_row_rect(MAX_IMU_ROWS - 1, MAX_IMU_ROWS);
    assert!(last_full.top_left.y > panel.top_left.y);
    assert!(bottom_right(last_full).y < bottom_right(panel).y);
}

/// Test that the readout kind names both IMU Procedures and leaves every other
/// Procedure without a readout.
#[test]
fn the_readout_kind_maps_both_imu_procedures() {
    assert_eq!(
        ImuReadoutKind::for_procedure(Procedure::Imu6Axis),
        Some(ImuReadoutKind::SixAxis)
    );
    assert_eq!(
        ImuReadoutKind::for_procedure(Procedure::Imu9Axis),
        Some(ImuReadoutKind::NineAxis)
    );
    assert_eq!(ImuReadoutKind::for_procedure(Procedure::BasicMotor), None);
    assert_eq!(ImuReadoutKind::for_procedure(Procedure::MotorCalibration), None);
}

/// The XY-plane remap headings the sample readout carries, including negative
/// and zero values and the full right-aligned width.
const SAMPLE_REMAP_XY: [f32; 8] = [-179.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 134.0];

/// The XZ-plane remap headings the sample readout carries, including negative
/// and zero values.
const SAMPLE_REMAP_XZ: [f32; 8] = [-180.0, -90.0, -1.0, 0.0, 1.0, 90.0, 179.0, 180.0];

/// The YZ-plane remap headings the sample readout carries, including negative
/// and zero values.
const SAMPLE_REMAP_YZ: [f32; 8] = [0.0, 10.0, -10.0, 100.0, -100.0, 45.0, -45.0, 90.0];

/// A fully populated nine-axis readout: the six-axis values plus both
/// magnetometer vectors, a loaded DMP calibration, a passing gate and the heading
/// and remap diagnostics.
const fn sample_nine_axis_readout() -> ImuReadout {
    ImuReadout::waiting(ImuReadoutKind::NineAxis)
        .with_health(ImuHealth::Live, 0)
        .with_orientation(Euler {
            yaw: 12.3,
            pitch: -45.6,
            roll: 78.9,
        })
        .with_raw_accel(Vector3 {
            x: 0.01,
            y: -0.02,
            z: 1.0,
        })
        .with_raw_gyro(Vector3 {
            x: 1.2,
            y: -3.4,
            z: 5.6,
        })
        .with_calibrated_gyro(Vector3 {
            x: -1.2,
            y: 3.4,
            z: -5.6,
        })
        .with_raw_mag(Vector3 {
            x: 12.3,
            y: -45.6,
            z: 78.9,
        })
        .with_calibrated_mag(Vector3 {
            x: -1.5,
            y: 2.5,
            z: -3.5,
        })
        .with_dmp_calibration(CalibrationStatus::Loaded)
        .with_mag_available(true)
        .with_mag(MagDiagnostics {
            norm: 50.0,
            use_mag: true,
            raw_heading: -179.0,
            tilt_heading: Some(45.6),
            remap_xy: SAMPLE_REMAP_XY,
            remap_xz: SAMPLE_REMAP_XZ,
            remap_yz: SAMPLE_REMAP_YZ,
        })
}

/// Test that the nine-axis readout renders exactly twelve rows, in order, with
/// the agreed labels, precision and units, and no unused slots.
#[test]
fn the_nine_axis_readout_has_twelve_rows_in_order() {
    let readout = sample_nine_axis_readout();
    assert_eq!(readout.row_count(), 12);
    assert_eq!(readout.row_count(), MAX_IMU_ROWS);

    let rows = readout.rows();
    let expected = [
        "Euler yaw=12.3 pitch=-45.6 roll=78.9",
        "Accel x=0.01 y=-0.02 z=1.00 g",
        "Gyro raw x=1.2 y=-3.4 z=5.6 dps",
        "Gyro cal x=-1.2 y=3.4 z=-5.6 dps",
        "M raw x=12.3 y=-45.6 z=78.9 uT",
        "M dmp x=-1.5 y=2.5 z=-3.5 uT",
        "Hdg raw=-179.0 tilt=45.6 dmp=12.3",
        "XY -179 -135  -90  -45    0   45   90  134",
        "XZ -180  -90   -1    0    1   90  179  180",
        "YZ    0   10  -10  100 -100   45  -45   90",
        "Live miss=0 dmpcal loaded",
        "|m|=50.0 uT use_mag=yes",
    ];
    for (row, expected) in rows.iter().zip(expected) {
        assert_eq!(row.as_str(), expected);
    }
}

/// Test that each plane's remap row prints all eight headings as integers,
/// right-aligned in five-character fields, so negatives and zero line up.
#[test]
fn the_remap_rows_pin_eight_right_aligned_integers_per_plane() {
    let readout = ImuReadout::waiting(ImuReadoutKind::NineAxis).with_mag(MagDiagnostics {
        norm: 50.0,
        use_mag: true,
        raw_heading: -179.0,
        tilt_heading: Some(0.0),
        remap_xy: SAMPLE_REMAP_XY,
        remap_xz: SAMPLE_REMAP_XZ,
        remap_yz: SAMPLE_REMAP_YZ,
    });

    let rows = readout.rows();
    assert_eq!(rows[7].as_str(), "XY -179 -135  -90  -45    0   45   90  134");
    assert_eq!(rows[8].as_str(), "XZ -180  -90   -1    0    1   90  179  180");
    assert_eq!(rows[9].as_str(), "YZ    0   10  -10  100 -100   45  -45   90");
    for row in &rows[7..10] {
        assert_eq!(row.as_str().len(), 42);
    }
}

/// Test that the heading row shows `dmp=--` when the magnetometer is present
/// but the fused orientation is not.
#[test]
fn the_heading_row_shows_a_dmp_placeholder_without_orientation() {
    let readout = ImuReadout::waiting(ImuReadoutKind::NineAxis).with_mag(MagDiagnostics {
        norm: 50.0,
        use_mag: true,
        raw_heading: -179.0,
        tilt_heading: Some(45.6),
        remap_xy: SAMPLE_REMAP_XY,
        remap_xz: SAMPLE_REMAP_XZ,
        remap_yz: SAMPLE_REMAP_YZ,
    });

    assert_eq!(readout.rows()[6].as_str(), "Hdg raw=-179.0 tilt=45.6 dmp=--");
}

/// Test that the heading row shows `tilt=--` when the magnetometer is present
/// but there was no accelerometer sample to tilt-compensate with.
#[test]
fn the_heading_row_shows_a_tilt_placeholder_without_an_accelerometer() {
    let readout = ImuReadout::waiting(ImuReadoutKind::NineAxis)
        .with_orientation(Euler {
            yaw: 12.3,
            pitch: -45.6,
            roll: 78.9,
        })
        .with_mag(MagDiagnostics {
            norm: 50.0,
            use_mag: true,
            raw_heading: -179.0,
            tilt_heading: None,
            remap_xy: SAMPLE_REMAP_XY,
            remap_xz: SAMPLE_REMAP_XZ,
            remap_yz: SAMPLE_REMAP_YZ,
        });

    assert_eq!(readout.rows()[6].as_str(), "Hdg raw=-179.0 tilt=-- dmp=12.3");
}

/// Test that the raw and calibrated magnetometer rows use one decimal place and
/// the ASCII `uT` unit.
#[test]
fn the_magnetometer_rows_use_one_decimal_and_ut() {
    let readout = ImuReadout::waiting(ImuReadoutKind::NineAxis)
        .with_raw_mag(Vector3 {
            x: 12.34,
            y: -45.67,
            z: 78.91,
        })
        .with_calibrated_mag(Vector3 {
            x: -1.23,
            y: 2.34,
            z: -3.45,
        });

    let rows = readout.rows();
    assert_eq!(rows[4].as_str(), "M raw x=12.3 y=-45.7 z=78.9 uT");
    assert_eq!(rows[5].as_str(), "M dmp x=-1.2 y=2.3 z=-3.5 uT");
}

/// Test that the magnetometer gate row shows the norm and the driver's verdict,
/// and a placeholder with `use_mag=no` when no verdict exists.
#[test]
fn the_magnetometer_gate_reports_the_use_mag_verdict() {
    let below = ImuReadout::waiting(ImuReadoutKind::NineAxis).with_mag(MagDiagnostics {
        norm: 5.0,
        use_mag: false,
        raw_heading: 0.0,
        tilt_heading: Some(0.0),
        remap_xy: [0.0; 8],
        remap_xz: [0.0; 8],
        remap_yz: [0.0; 8],
    });
    assert_eq!(below.rows()[11].as_str(), "|m|=5.0 uT use_mag=no");

    let in_range = ImuReadout::waiting(ImuReadoutKind::NineAxis).with_mag(MagDiagnostics {
        norm: 50.0,
        use_mag: true,
        raw_heading: 0.0,
        tilt_heading: Some(0.0),
        remap_xy: [0.0; 8],
        remap_xz: [0.0; 8],
        remap_yz: [0.0; 8],
    });
    assert_eq!(in_range.rows()[11].as_str(), "|m|=50.0 uT use_mag=yes");

    let absent = ImuReadout::waiting(ImuReadoutKind::NineAxis);
    assert_eq!(absent.rows()[11].as_str(), "|m|=-- uT use_mag=no");
}

/// Test that the nine-axis health row names Waiting, Live and a silent IMU, and
/// distinguishes a loaded, missing and unknown DMP calibration.
#[test]
fn the_nine_axis_health_row_names_the_dmp_calibration() {
    for (health, missing, calibration, expected) in [
        (
            ImuHealth::Waiting,
            0,
            CalibrationStatus::Unknown,
            "Waiting miss=0 dmpcal unknown",
        ),
        (
            ImuHealth::Live,
            7,
            CalibrationStatus::Loaded,
            "Live miss=7 dmpcal loaded",
        ),
        (
            ImuHealth::NotResponding,
            100,
            CalibrationStatus::Missing,
            "IMU not responding miss=100 dmpcal missing",
        ),
        (
            ImuHealth::Live,
            10_000,
            CalibrationStatus::Unknown,
            "Live miss=9999 dmpcal unknown",
        ),
    ] {
        let readout = ImuReadout::waiting(ImuReadoutKind::NineAxis)
            .with_health(health, missing)
            .with_dmp_calibration(calibration);
        assert_eq!(readout.rows()[10].as_str(), expected);
    }

    // A readout that has not learned the calibration status defaults to the
    // conservative `unknown`.
    assert_eq!(
        ImuReadout::waiting(ImuReadoutKind::NineAxis).rows()[10].as_str(),
        "Waiting miss=0 dmpcal unknown"
    );
}

/// Test that an unavailable magnetometer keeps the health line naming the state
/// and calibration, shows the six-axis fallback and `use_mag=no` on the gate
/// row, and renders `--` magnetometer, heading and remap values, while the Euler
/// row keeps the fallback yaw the fused orientation still provides.
#[test]
fn an_unavailable_magnetometer_falls_back_to_six_axis() {
    let readout = ImuReadout::waiting(ImuReadoutKind::NineAxis)
        .with_health(ImuHealth::Live, 0)
        .with_orientation(Euler {
            yaw: 12.3,
            pitch: -45.6,
            roll: 78.9,
        })
        .with_dmp_calibration(CalibrationStatus::Loaded)
        .with_mag_available(false);
    assert_eq!(readout.row_count(), 12);

    let rows = readout.rows();
    assert_eq!(rows[0].as_str(), "Euler yaw=12.3 pitch=-45.6 roll=78.9");
    assert_eq!(rows[1].as_str(), "Accel x=-- y=-- z=-- g");
    assert_eq!(rows[2].as_str(), "Gyro raw x=-- y=-- z=-- dps");
    assert_eq!(rows[3].as_str(), "Gyro cal x=-- y=-- z=-- dps");
    assert_eq!(rows[4].as_str(), "M raw x=-- y=-- z=-- uT");
    assert_eq!(rows[5].as_str(), "M dmp x=-- y=-- z=-- uT");
    assert_eq!(rows[6].as_str(), "Hdg --");
    assert_eq!(rows[7].as_str(), "XY --");
    assert_eq!(rows[8].as_str(), "XZ --");
    assert_eq!(rows[9].as_str(), "YZ --");
    assert_eq!(rows[10].as_str(), "Live miss=0 dmpcal loaded");
    assert_eq!(rows[11].as_str(), "mag unavailable - 6-axis fallback use_mag=no");
}

/// Test that the last of the twelve nine-axis rows fits inside the status panel
/// at the compressed pitch, so the full page needs no paging or scrolling.
#[test]
fn the_nine_axis_last_row_fits_inside_the_status_panel() {
    let panel = panel_rect();

    assert_eq!(readout_row_pitch(12), 11);
    let last = readout_row_rect(11, 12);
    assert!(last.top_left.y > panel.top_left.y);
    assert!(bottom_right(last).y < bottom_right(panel).y);
}

/// Every Procedure the crate models.
const ALL_PROCEDURES: [Procedure; 11] = [
    Procedure::MotorCalibration,
    Procedure::DmpCompassCalibration,
    Procedure::DistanceCalibration,
    Procedure::CoastAndAvoid,
    Procedure::AttemptStraight,
    Procedure::BasicMotor,
    Procedure::Turns,
    Procedure::StraightDrive,
    Procedure::ArcDrive,
    Procedure::Imu6Axis,
    Procedure::Imu9Axis,
];

/// Build the prompt for the DMP compass calibration: the Procedure supplies the
/// title and the screen Stop returns to.
const fn dmp_prompt(instruction: &'static str) -> PromptView {
    PromptView::for_procedure(Procedure::DmpCompassCalibration, instruction)
}

/// Test the DMP compass calibration identity: its label, parent, calibration
/// flag, destination, hands-on metadata, and that it is the Calibrate submenu's
/// only magnetometer entry.
#[test]
fn the_dmp_compass_calibration_is_hands_on_and_listed_on_calibrate() {
    let procedure = Procedure::DmpCompassCalibration;
    assert_eq!(procedure.label(), "DMP Mag");
    assert_eq!(procedure.parent(), Some(Screen::Calibrate));
    assert_eq!(procedure.landing_screen(), Screen::Calibrate);
    assert!(
        procedure.is_calibration(),
        "the DMP compass calibration is a calibration"
    );
    assert_eq!(procedure.destination(), Screen::Prompt);
    assert!(procedure.is_hands_on(), "an operator must hold the robot while it runs");
    assert_eq!(Item::Procedure(procedure).interactive(), procedure.interactive());

    // The Prompt screen has no Menu Entry of its own; its parent and title come
    // from the active view like the shared running screen.
    assert_eq!(Screen::Prompt.entry(), None);
    assert_eq!(Screen::Prompt.parent(), None);
    assert_eq!(Screen::Prompt.items(), []);
    assert_eq!(Screen::Prompt.title(), "Prompt");

    // Calibrate lists Motor, DMP Mag and Distance; DMP Mag is listed nowhere else.
    assert_eq!(
        Screen::Calibrate.items(),
        [
            Item::Procedure(Procedure::MotorCalibration),
            Item::Procedure(Procedure::DmpCompassCalibration),
            Item::Procedure(Procedure::DistanceCalibration),
        ]
        .as_slice()
    );
    for screen in [Screen::MainMenu, Screen::DriveMode, Screen::TestMode] {
        assert!(
            screen.items().iter().all(|item| item.as_procedure() != Some(procedure)),
            "{procedure:?} is not listed on {screen:?}"
        );
    }
}

/// A neutral DMP snapshot formats a placeholder status line and the first step's
/// Prompt without any measured values.
#[test]
fn a_dmp_snapshot_starts_on_the_first_step_with_placeholders() {
    let snapshot = DmpCalibrationSnapshot::starting();
    assert_eq!(snapshot.step, CompassStep::FlatHold);
    assert_eq!(snapshot.step_index(), 0);
    assert_eq!(snapshot.step_count(), CompassStep::ALL.len());
    assert_eq!(snapshot.instruction(), "Place flat. Hold still.");
    assert_eq!(snapshot.status_text().as_str(), "Cal unknown Hdg -- Acc --");
    assert_eq!(snapshot.progress(), 0);

    let view = snapshot.prompt_view();
    assert_eq!(view.title, "DMP Mag");
    assert_eq!(view.parent, Screen::Calibrate);
    assert_eq!(view.instruction, "Place flat. Hold still.");
    assert_eq!(view.status_text(), "Cal unknown Hdg -- Acc --");
    assert_eq!(view.progress, Some(0));
    assert!(view.done);
    assert!(!view.redo);

    let mut ui = Ui::new();
    assert!(ui.show_prompt(view));
    assert_eq!(ui.title(), "DMP Mag");
    render_ok(&ui);
}

/// Every DMP instruction fits the prompt's instruction row when wrapped to two
/// lines, and uses only glyphs the ASCII font can draw.
#[test]
fn every_dmp_instruction_wraps_into_the_prompt_row() {
    // The instruction row is 240 px wide in the 6x10 font, so 40 characters fit.
    let max_chars = (prompt_instruction_rect().size.width / 6) as usize;
    assert_eq!(max_chars, 40);

    for step in CompassStep::ALL {
        let instruction = step.instruction();
        assert!(instruction.is_ascii(), "{instruction:?} needs a non-ASCII glyph");

        let mut lines = [""; 2];
        let count = wrap_text(instruction, max_chars, &mut lines);
        assert!((1..=2).contains(&count), "{instruction:?} wrapped to {count} lines");
        for line in &lines[..count] {
            assert!(
                line.chars().count() <= max_chars,
                "{line:?} is longer than the prompt row"
            );
        }
    }

    // The longest instruction, the Compass Check, breaks before the final hold.
    let mut lines = [""; 2];
    assert_eq!(
        wrap_text(CompassStep::NorthCheck.instruction(), max_chars, &mut lines),
        2
    );
    assert_eq!(lines, ["Point front at magnetic north (0 deg).", "Hold still."]);
}

/// A movement step formats the heading, accuracy and span hint on the one status
/// line, and its Prompt carries the step's instruction and progress.
#[test]
fn a_dmp_movement_snapshot_formats_heading_and_span() {
    let snapshot = DmpCalibrationSnapshot {
        step: CompassStep::FlatSpin,
        heading_deg: Some(123.4),
        target_heading_deg: None,
        accuracy: Some(3),
        hint: DmpHint::YawSpanDeg(45.2),
        check: DmpCheckState::NotChecking,
        redo: false,
        prior_status: CalibrationStatus::Loaded,
    };

    assert_eq!(snapshot.status_text().as_str(), "Hdg 123 Acc 3 Span 45");
    assert!(snapshot.status_text().as_str().len() <= TEXT_BUF_LEN);
    assert_eq!(snapshot.prompt_view().instruction, CompassStep::FlatSpin.instruction());
    assert_eq!(snapshot.prompt_view().progress, Some(12));
    assert!(!snapshot.prompt_view().redo);
}

/// A hold step formats elapsed stillness in whole seconds, and a Compass Check
/// formats its target and state, offering Redo as well as Done.
#[test]
fn a_dmp_hold_and_check_snapshot_format_their_hints() {
    let hold = DmpCalibrationSnapshot {
        step: CompassStep::FlatHold,
        heading_deg: Some(12.0),
        target_heading_deg: None,
        accuracy: Some(3),
        hint: DmpHint::StillMillis(2_500),
        check: DmpCheckState::NotChecking,
        redo: false,
        prior_status: CalibrationStatus::Loaded,
    };
    assert_eq!(hold.status_text().as_str(), "Hdg 12 Acc 3 Still 2s");

    let check = DmpCalibrationSnapshot {
        step: CompassStep::NorthCheck,
        heading_deg: Some(5.0),
        target_heading_deg: Some(0.0),
        accuracy: Some(3),
        hint: DmpHint::None,
        check: DmpCheckState::Waiting,
        redo: true,
        prior_status: CalibrationStatus::Loaded,
    };
    assert_eq!(check.status_text().as_str(), "Hdg 5 Tgt 0 Acc 3 Check Waiting");
    assert!(check.status_text().as_str().len() <= TEXT_BUF_LEN);
    assert!(check.prompt_view().redo);
    assert_eq!(check.prompt_view().progress, Some(75));

    let mut ui = Ui::new();
    assert!(ui.show_prompt(check.prompt_view()));
    assert_eq!(
        ui.hit_test(prompt_redo_rect().center()),
        Some(Hit::PromptAction(PromptAction::Redo))
    );
    render_ok(&ui);
}

/// The first Prompt frame says when the starting DMP calibration is missing, so
/// an operator without a saved snapshot is not asked to move blindly.
#[test]
fn a_dmp_snapshot_says_when_the_starting_calibration_is_missing() {
    let mut snapshot = DmpCalibrationSnapshot::starting();
    assert_eq!(snapshot.status_text().as_str(), "Cal unknown Hdg -- Acc --");

    snapshot.prior_status = CalibrationStatus::Missing;
    assert_eq!(snapshot.status_text().as_str(), "Cal missing Hdg -- Acc --");

    snapshot.prior_status = CalibrationStatus::Loaded;
    assert_eq!(snapshot.status_text().as_str(), "Hdg -- Acc --");
    assert!(snapshot.status_text().as_str().len() <= TEXT_BUF_LEN);
}

/// A failed Compass Check names the rule it broke, keeps the measured heading,
/// target and accuracy on the same line, and still offers Redo.
#[test]
fn a_dmp_check_failure_names_its_reason() {
    assert_eq!(
        DmpCheckState::from(CheckOutcome::Fail(CheckFailure::Accuracy { measured: 1 })),
        DmpCheckState::Fail(DmpCheckFail::Accuracy)
    );
    assert_eq!(
        DmpCheckState::from(CheckOutcome::Fail(CheckFailure::Heading {
            measured_deg: 30.0,
            target_deg: 0.0,
        })),
        DmpCheckState::Fail(DmpCheckFail::Heading)
    );
    assert_eq!(DmpCheckState::from(CheckOutcome::Waiting), DmpCheckState::Waiting);
    assert_eq!(DmpCheckState::from(CheckOutcome::Unstable), DmpCheckState::Unstable);
    assert_eq!(DmpCheckState::from(CheckOutcome::Pass), DmpCheckState::Pass);

    let accuracy = DmpCalibrationSnapshot {
        step: CompassStep::NorthCheck,
        heading_deg: Some(5.0),
        target_heading_deg: Some(0.0),
        accuracy: Some(1),
        hint: DmpHint::None,
        check: DmpCheckState::Fail(DmpCheckFail::Accuracy),
        redo: true,
        prior_status: CalibrationStatus::Loaded,
    };
    assert_eq!(accuracy.status_text().as_str(), "Hdg 5 Tgt 0 Acc 1 Check Acc");
    assert!(accuracy.status_text().as_str().len() <= TEXT_BUF_LEN);
    assert!(accuracy.prompt_view().redo);

    let heading = DmpCalibrationSnapshot {
        step: CompassStep::EastCheck,
        heading_deg: Some(130.0),
        target_heading_deg: Some(90.0),
        accuracy: Some(3),
        hint: DmpHint::None,
        check: DmpCheckState::Fail(DmpCheckFail::Heading),
        redo: true,
        prior_status: CalibrationStatus::Loaded,
    };
    assert_eq!(heading.status_text().as_str(), "Hdg 130 Tgt 90 Acc 3 Check Hdg");
    assert!(heading.status_text().as_str().len() <= TEXT_BUF_LEN);
}

/// Test that the model's hands-on set is the firmware's Remote-availability rule
/// and nothing else, so a future edit cannot introduce a second contradicting
/// rule.
#[test]
fn the_hands_on_set_mirrors_the_firmware_availability_rule() {
    let hands_on = [
        Procedure::MotorCalibration,
        Procedure::DmpCompassCalibration,
        Procedure::DistanceCalibration,
        Procedure::BasicMotor,
        Procedure::AttemptStraight,
    ];
    for procedure in ALL_PROCEDURES {
        assert_eq!(
            procedure.is_hands_on(),
            hands_on.contains(&procedure),
            "{procedure:?} hands-on membership"
        );
    }
    // The Remote-startable Procedures stay startable.
    for procedure in [
        Procedure::CoastAndAvoid,
        Procedure::Turns,
        Procedure::StraightDrive,
        Procedure::ArcDrive,
        Procedure::Imu6Axis,
        Procedure::Imu9Axis,
    ] {
        assert!(!procedure.is_hands_on(), "{procedure:?} may be started from the Remote");
    }
}

/// Test that a prompt carries the Procedure title, instruction, dynamic status
/// line, action availability and parent, and that the UI shows it with its own
/// title.
#[test]
fn a_prompt_carries_its_title_instruction_status_and_parent() {
    let mut status_line = TextBuf::new();
    status_line.fill(format_args!("Hdg 12 target 0 acc 2"));
    let view = dmp_prompt("Place flat. Hold still.").with_status(status_line);

    assert_eq!(view.title, "DMP Mag");
    assert_eq!(view.parent, Screen::Calibrate);
    assert_eq!(view.instruction, "Place flat. Hold still.");
    assert_eq!(view.status_text(), "Hdg 12 target 0 acc 2");
    assert_eq!(view.progress, None);
    assert!(view.done, "Done is offered on every prompt");
    assert!(!view.redo, "a movement step offers no Redo");

    let mut ui = Ui::new();
    assert!(ui.show_prompt(view));
    assert_eq!(ui.screen(), Screen::Prompt);
    assert_eq!(ui.title(), "DMP Mag");
    assert_eq!(ui.prompt_view(), Some(view));
    assert_eq!(ui.status_view(), None);
    render_ok(&ui);
}

/// Test that a prompt update reports only real changes, and is a no-op away from
/// the prompt screen.
#[test]
fn a_prompt_reports_only_real_changes() {
    let mut first = TextBuf::new();
    first.fill(format_args!("Waiting for motion"));
    let view = dmp_prompt("Place flat. Hold still.").with_status(first);

    let mut ui = Ui::new();
    assert!(ui.show_prompt(view));
    assert!(!ui.set_prompt(view), "the view is already shown");

    let mut second = TextBuf::new();
    second.fill(format_args!("Hdg 0 acc 3"));
    let changed = view.with_status(second).with_progress(25).with_redo();
    assert!(ui.set_prompt(changed));
    assert_eq!(ui.prompt_view(), Some(changed));
    assert!(!ui.set_prompt(changed));
    assert_eq!(ui.prompt_view().map(|view| view.progress), Some(Some(25)));

    let mut other = Ui::new();
    assert!(!other.set_prompt(changed), "no prompt screen is shown");
    assert_eq!(other.prompt_view(), None);
}

/// Test that Done is hittable only inside its drawn rectangle, and that tapping
/// it is reported through `last_activation` without navigating.
#[test]
fn a_prompt_done_is_hittable_and_reported_without_navigating() {
    let mut ui = Ui::new();
    assert!(ui.show_prompt(dmp_prompt("Place flat. Hold still.")));

    let done = prompt_done_rect().center();
    assert_eq!(ui.hit_test(done), Some(Hit::PromptAction(PromptAction::Done)));
    tap(&mut ui, done);
    assert_eq!(ui.last_activation(), Some(Hit::PromptAction(PromptAction::Done)));
    assert_eq!(ui.screen(), Screen::Prompt, "the firmware decides what Done means");
}

/// Test that Redo is drawn and hittable only when the prompt offers it, and that
/// a disabled Redo is not hittable.
#[test]
fn a_prompt_redo_is_hittable_only_when_available() {
    let mut ui = Ui::new();
    assert!(ui.show_prompt(dmp_prompt("Front at magnetic north.").with_redo()));
    let redo = prompt_redo_rect().center();
    assert_eq!(ui.hit_test(redo), Some(Hit::PromptAction(PromptAction::Redo)));
    tap(&mut ui, redo);
    assert_eq!(ui.last_activation(), Some(Hit::PromptAction(PromptAction::Redo)));
    assert_eq!(ui.screen(), Screen::Prompt);

    let mut ui = Ui::new();
    assert!(ui.show_prompt(dmp_prompt("Tilt nose up and down 90 deg.")));
    assert_eq!(ui.hit_test(prompt_redo_rect().center()), None);
    tap(&mut ui, prompt_redo_rect().center());
    assert_eq!(ui.last_activation(), None, "a disabled Redo activates nothing");
    assert_eq!(
        ui.hit_test(prompt_done_rect().center()),
        Some(Hit::PromptAction(PromptAction::Done)),
        "Done stays available"
    );
}

/// Test that the header Stop and the prompt's Done/Redo never report each
/// other, and that only the action rectangles are interactive.
#[test]
fn prompt_actions_and_header_stop_never_swap() {
    let mut ui = Ui::new();
    assert!(ui.show_prompt(dmp_prompt("Front at magnetic east.").with_redo()));

    assert_eq!(ui.hit_test(back_button_rect().center()), Some(Hit::Stop));
    assert_eq!(
        ui.hit_test(prompt_done_rect().center()),
        Some(Hit::PromptAction(PromptAction::Done))
    );
    assert_eq!(
        ui.hit_test(prompt_redo_rect().center()),
        Some(Hit::PromptAction(PromptAction::Redo))
    );
    assert_eq!(ui.hit_test(prompt_instruction_rect().center()), None);
    assert_eq!(ui.hit_test(prompt_status_rect().center()), None);

    // The regions stay disjoint.
    assert!(!prompt_done_rect().contains(prompt_redo_rect().center()));
    assert!(!prompt_redo_rect().contains(prompt_done_rect().center()));
    assert!(!prompt_done_rect().contains(back_button_rect().center()));
    assert!(!back_button_rect().contains(prompt_done_rect().center()));
    assert_eq!(
        ui.hit_test(prompt_done_rect().top_left),
        Some(Hit::PromptAction(PromptAction::Done))
    );
    assert_eq!(
        ui.hit_test(prompt_redo_rect().top_left),
        Some(Hit::PromptAction(PromptAction::Redo))
    );
}

/// Test that the prompt's actions sit inside the panel, below both text lines,
/// and are reported by the shared geometry.
#[test]
fn the_prompt_actions_sit_inside_the_panel_below_the_text() {
    let panel = panel_rect();
    let done = prompt_done_rect();
    let redo = prompt_redo_rect();

    for rect in [done, redo] {
        assert!(rect.top_left.x >= panel.top_left.x);
        assert!(rect.top_left.y >= panel.top_left.y);
        assert!(bottom_right(rect).x < bottom_right(panel).x);
        assert!(bottom_right(rect).y < bottom_right(panel).y);
    }
    assert!(bottom_right(prompt_instruction_rect()).y <= prompt_status_rect().top_left.y);
    assert!(bottom_right(prompt_status_rect()).y < done.top_left.y);
    assert!(bottom_right(done).x < redo.top_left.x);
    assert_eq!(prompt_action_rect(0), done);
    assert_eq!(prompt_action_rect(1), redo);
    assert_eq!(PromptAction::ALL, [PromptAction::Done, PromptAction::Redo]);
    assert_eq!(PromptAction::Done.rect(), done);
    assert_eq!(PromptAction::Redo.rect(), redo);
}

/// Test that a prompt renders with and without progress, and that the progress
/// bar sits between the status line and the action row.
#[test]
fn a_prompt_with_and_without_progress_renders() {
    let mut ui = Ui::new();
    assert!(ui.show_prompt(dmp_prompt("On the table, spin 360 deg both ways.")));
    render_ok(&ui);

    let with_progress = dmp_prompt("Flip wheels up, spin 360 deg both ways.")
        .with_progress(50)
        .with_redo();
    assert!(ui.set_prompt(with_progress));
    render_ok(&ui);

    let bar = prompt_progress_rect();
    assert!(bar.top_left.y > prompt_status_rect().top_left.y);
    assert!(bottom_right(bar).y < prompt_done_rect().top_left.y);
    assert!(bar.top_left.x >= panel_rect().top_left.x);
    assert!(bottom_right(bar).x < bottom_right(panel_rect()).x);
}

/// Test that Stop from a prompt returns to the prompt's parent, the Calibrate
/// submenu for the DMP compass calibration.
#[test]
fn a_prompt_stops_back_to_the_calibrate_submenu() {
    let mut ui = Ui::new();
    assert!(ui.show_prompt(dmp_prompt("Place flat. Hold still.")));
    tap(&mut ui, back_button_rect().center());
    assert_eq!(ui.last_activation(), Some(Hit::Stop));
    assert_eq!(ui.screen(), Screen::Calibrate);
    assert_eq!(ui.prompt_view(), None);

    // The firmware can also land the Procedure through the command-level seam.
    let mut ui = Ui::new();
    assert!(ui.show_prompt(dmp_prompt("Place flat. Hold still.")));
    assert!(ui.show_screen(Procedure::DmpCompassCalibration.landing_screen()));
    assert_eq!(ui.screen(), Screen::Calibrate);
}
