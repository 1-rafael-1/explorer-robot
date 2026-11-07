//! Host tests for the bounded Setup Mode window (ADR-0036).
//!
//! The window is modelled with plain millisecond timestamps: an unprovisioned boot, five
//! continuous minutes without a station link, or a panel request raises it; it lasts ten minutes
//! and closes early thirty seconds after a stable station association. The outage rule fires once
//! per boot, and an open window consumes that one opportunity so closing it does not immediately
//! raise another.

use bridge_core::{CloseReason, RaiseReason, SetupEvent, SetupWindow, setup};

/// Boots unprovisioned at `now_ms` and returns the open window.
fn unprovisioned_boot(now_ms: u32) -> SetupWindow {
    let (window, event) = SetupWindow::boot(false, now_ms);
    assert_eq!(
        event,
        Some(SetupEvent::Opened(RaiseReason::Unprovisioned)),
        "an unprovisioned boot opens the window"
    );
    window
}

/// The ADR-0036 timings are the firmware's constants.
#[test]
fn the_window_constants_match_adr_0036() {
    assert_eq!(setup::WINDOW_MS, 600_000);
    assert_eq!(setup::STABLE_ASSOCIATION_MS, 30_000);
    assert_eq!(setup::OUTAGE_MS, 300_000);
}

/// An unprovisioned boot opens immediately with the unprovisioned reason.
#[test]
fn an_unprovisioned_boot_opens_the_window() {
    let (window, event) = SetupWindow::boot(false, 0);

    assert_eq!(event, Some(SetupEvent::Opened(RaiseReason::Unprovisioned)));
    assert!(window.is_open());
    assert_eq!(window.reason(), Some(RaiseReason::Unprovisioned));
    assert_eq!(window.remaining_ms(0), setup::WINDOW_MS);
}

/// A provisioned boot starts closed, with no window timer running.
#[test]
fn a_provisioned_boot_starts_closed() {
    let (window, event) = SetupWindow::boot(true, 42);

    assert_eq!(event, None);
    assert!(!window.is_open());
    assert_eq!(window.reason(), None);
    assert_eq!(window.remaining_ms(42), 0);
}

/// The remaining time counts down from the full window and clamps at zero.
#[test]
fn remaining_ms_counts_down_and_clamps_at_zero() {
    let mut window = unprovisioned_boot(7);

    assert_eq!(window.remaining_ms(7), setup::WINDOW_MS);
    assert_eq!(window.remaining_ms(8), setup::WINDOW_MS - 1);
    assert_eq!(window.remaining_ms(600_006), 1);
    assert_eq!(window.remaining_ms(600_007), 0);

    assert_eq!(
        window.tick(600_007, false),
        Some(SetupEvent::Closed(CloseReason::Expired))
    );
    assert_eq!(window.remaining_ms(600_008), 0, "a closed window has no time left");
}

/// The window expires exactly ten minutes after it opened, not one millisecond earlier.
#[test]
fn the_window_expires_exactly_ten_minutes_after_it_opens() {
    let mut window = unprovisioned_boot(1_000);

    assert_eq!(window.remaining_ms(600_999), 1);
    assert_eq!(window.tick(600_999, false), None);
    assert!(
        window.is_open(),
        "one millisecond before the deadline the window is still open"
    );
    assert_eq!(
        window.tick(601_000, false),
        Some(SetupEvent::Closed(CloseReason::Expired))
    );
    assert!(!window.is_open());
    assert_eq!(window.reason(), None);
}

/// A stable association closes the window exactly thirty seconds after the rising edge.
#[test]
fn a_stable_association_closes_exactly_thirty_seconds_after_the_rise() {
    let mut window = unprovisioned_boot(0);

    assert_eq!(window.tick(10_000, true), None, "the rise arms the close timer");
    assert_eq!(
        window.tick(39_999, true),
        None,
        "29_999 ms after the rise is not yet stable"
    );
    assert_eq!(
        window.tick(40_000, true),
        Some(SetupEvent::Closed(CloseReason::StationStable))
    );
    assert!(!window.is_open());
    assert_eq!(window.reason(), None);
}

/// A drop before the stable deadline cancels the close; the next rise arms it again.
#[test]
fn a_drop_before_thirty_seconds_cancels_the_close_and_the_next_rise_rearms_it() {
    let mut window = unprovisioned_boot(0);

    assert_eq!(window.tick(10_000, true), None);
    assert_eq!(
        window.tick(39_999, false),
        None,
        "the station dropping before the deadline cancels the close"
    );
    assert_eq!(window.tick(40_000, false), None, "the cancelled deadline does not fire");
    assert!(window.is_open(), "the burst of association did not close the window");

    assert_eq!(window.tick(50_000, true), None, "the next rise arms a new deadline");
    assert_eq!(window.tick(79_999, true), None);
    assert_eq!(
        window.tick(80_000, true),
        Some(SetupEvent::Closed(CloseReason::StationStable))
    );
}

/// The close timer is relative to the rise, not to the window's opening time.
#[test]
fn the_stable_close_is_relative_to_the_rise() {
    let (mut window, _) = SetupWindow::boot(true, 0);

    assert_eq!(
        window.panel_request(0, false),
        Some(SetupEvent::Opened(RaiseReason::PanelRequest))
    );
    assert_eq!(window.tick(500_000, true), None);
    assert_eq!(window.tick(529_999, true), None);
    assert_eq!(
        window.tick(530_000, true),
        Some(SetupEvent::Closed(CloseReason::StationStable))
    );
}

/// A rising edge while the window is closed does not arm a close for a later window.
#[test]
fn a_rise_outside_an_open_window_does_not_arm_the_stable_close() {
    let (mut window, _) = SetupWindow::boot(true, 0);

    assert_eq!(window.tick(10_000, true), None, "the rise happens while closed");
    assert_eq!(
        window.panel_request(20_000, true),
        Some(SetupEvent::Opened(RaiseReason::PanelRequest))
    );
    assert_eq!(
        window.tick(50_000, true),
        None,
        "thirty seconds after opening on an already-up station is not stable"
    );
    assert_eq!(
        window.tick(620_000, true),
        Some(SetupEvent::Closed(CloseReason::Expired))
    );
}

/// A panel-requested window on a healthy link stays past thirty seconds and waits for expiry.
#[test]
fn a_panel_request_while_up_stays_past_thirty_seconds_until_expiry() {
    let (mut window, _) = SetupWindow::boot(true, 0);

    assert_eq!(
        window.panel_request(1_000, true),
        Some(SetupEvent::Opened(RaiseReason::PanelRequest))
    );
    assert_eq!(window.tick(30_999, true), None);
    assert_eq!(window.tick(31_000, true), None, "the window is still open");
    assert_eq!(
        window.tick(601_000, true),
        Some(SetupEvent::Closed(CloseReason::Expired))
    );
}

/// A rise too late in the window arms a close that the ten-minute expiry beats.
#[test]
fn a_late_rise_loses_to_the_expiry() {
    let mut window = unprovisioned_boot(0);

    assert_eq!(window.tick(599_999, true), None);
    assert_eq!(
        window.tick(600_000, true),
        Some(SetupEvent::Closed(CloseReason::Expired)),
        "expiry wins over the newly armed stable close"
    );
}

/// A station down since boot raises the window at exactly five minutes.
#[test]
fn the_outage_rule_raises_at_exactly_five_minutes() {
    let (mut window, _) = SetupWindow::boot(true, 10_000);

    assert_eq!(window.tick(309_999, false), None);
    assert_eq!(
        window.tick(310_000, false),
        Some(SetupEvent::Opened(RaiseReason::StationOutage))
    );
    assert!(window.is_open());
    assert_eq!(window.reason(), Some(RaiseReason::StationOutage));
    assert_eq!(window.remaining_ms(310_000), setup::WINDOW_MS);
    assert_eq!(window.tick(310_001, false), None);
}

/// The outage clock restarts whenever the station comes up.
#[test]
fn the_outage_clock_restarts_whenever_the_station_is_up() {
    let (mut window, _) = SetupWindow::boot(true, 0);

    assert_eq!(window.tick(100_000, true), None);
    assert_eq!(window.tick(150_000, false), None, "the outage clock restarts here");
    assert_eq!(
        window.tick(299_999, false),
        None,
        "the original five-minute mark has passed but not the restarted one"
    );
    assert_eq!(window.tick(449_999, false), None);
    assert_eq!(
        window.tick(450_000, false),
        Some(SetupEvent::Opened(RaiseReason::StationOutage))
    );
}

/// The outage rule never fires twice in one boot, even after the raised window closes.
#[test]
fn the_outage_rule_raises_only_once_per_boot() {
    let (mut window, _) = SetupWindow::boot(true, 0);

    assert_eq!(
        window.tick(300_000, false),
        Some(SetupEvent::Opened(RaiseReason::StationOutage))
    );
    assert_eq!(
        window.tick(900_000, false),
        Some(SetupEvent::Closed(CloseReason::Expired))
    );
    assert_eq!(
        window.tick(900_001, false),
        None,
        "the once-per-boot outage opportunity is spent"
    );
    assert_eq!(window.tick(1_500_000, false), None);
}

/// An open window consumes the outage opportunity so closing it does not re-raise immediately.
#[test]
fn an_open_window_consumes_the_outage_opportunity() {
    let mut window = unprovisioned_boot(0);

    assert_eq!(window.tick(299_999, false), None);
    assert_eq!(
        window.tick(300_000, false),
        None,
        "the window is already open, so the outage only consumes the opportunity"
    );
    assert_eq!(window.reason(), Some(RaiseReason::Unprovisioned));
    assert_eq!(
        window.tick(600_000, false),
        Some(SetupEvent::Closed(CloseReason::Expired))
    );
    assert_eq!(
        window.tick(600_001, false),
        None,
        "closing does not immediately re-raise the outage it consumed"
    );
    assert_eq!(window.tick(900_001, false), None);
}

/// A panel request during an outage consumes the opportunity once the five minutes have elapsed.
#[test]
fn a_late_panel_request_during_an_outage_consumes_the_opportunity() {
    let (mut window, _) = SetupWindow::boot(true, 0);

    assert_eq!(
        window.panel_request(400_000, false),
        Some(SetupEvent::Opened(RaiseReason::PanelRequest))
    );
    assert_eq!(
        window.tick(400_000, false),
        None,
        "the already-elapsed outage is consumed, not raised again"
    );
    assert_eq!(
        window.tick(1_000_000, false),
        Some(SetupEvent::Closed(CloseReason::Expired))
    );
    assert_eq!(window.tick(1_000_001, false), None);
}

/// When a close and an outage raise are both due, the close wins and the tick returns one event.
#[test]
fn a_close_wins_when_the_outage_also_matures_on_the_same_tick() {
    let mut window = unprovisioned_boot(0);

    // Short association bursts never reach the stable deadline; the last one drops at 300_000.
    assert_eq!(window.tick(100_000, true), None);
    assert_eq!(window.tick(110_000, false), None);
    assert_eq!(window.tick(200_000, true), None);
    assert_eq!(window.tick(210_000, false), None);
    assert_eq!(window.tick(290_000, true), None);
    assert_eq!(window.tick(300_000, false), None);

    // The expiry and the restarted outage clock (300 s from 300_000) coincide at 600_000.
    assert_eq!(window.tick(599_999, false), None);
    assert_eq!(
        window.tick(600_000, false),
        Some(SetupEvent::Closed(CloseReason::Expired))
    );
    assert_eq!(
        window.tick(600_001, false),
        None,
        "the consumed outage does not open on the next tick"
    );
}

/// A panel request opens a closed window with its own reason.
#[test]
fn a_panel_request_while_closed_opens_with_the_panel_reason() {
    let (mut window, _) = SetupWindow::boot(true, 0);

    assert_eq!(
        window.panel_request(5_000, false),
        Some(SetupEvent::Opened(RaiseReason::PanelRequest))
    );
    assert!(window.is_open());
    assert_eq!(window.reason(), Some(RaiseReason::PanelRequest));
    assert_eq!(window.remaining_ms(5_000), setup::WINDOW_MS);
}

/// A panel request while the window is open changes nothing.
#[test]
fn a_panel_request_while_open_changes_nothing() {
    let mut window = unprovisioned_boot(0);

    assert_eq!(window.panel_request(10_000, true), None);
    assert_eq!(
        window.reason(),
        Some(RaiseReason::Unprovisioned),
        "the original reason survives"
    );
    assert_eq!(
        window.remaining_ms(10_000),
        setup::WINDOW_MS - 10_000,
        "the original timer survives"
    );
}

/// Forgetting from a provisioned state raises a fresh window with the forgotten reason.
#[test]
fn forget_from_provisioned_opens_a_fresh_window() {
    let (mut window, _) = SetupWindow::boot(true, 0);

    assert_eq!(
        window.forget(2_000, true),
        Some(SetupEvent::Opened(RaiseReason::Forgotten))
    );
    assert!(window.is_open());
    assert_eq!(window.reason(), Some(RaiseReason::Forgotten));
    assert_eq!(window.remaining_ms(2_000), setup::WINDOW_MS);
}

/// Forgetting while the window is open keeps the open window and its reason.
#[test]
fn forget_while_open_keeps_the_window() {
    let mut window = unprovisioned_boot(0);

    assert_eq!(window.forget(1_000, true), None);
    assert_eq!(window.reason(), Some(RaiseReason::Unprovisioned));
    assert_eq!(window.remaining_ms(1_000), setup::WINDOW_MS - 1_000);
}

/// Setting the provisioned flag false opens a closed window; setting it true never closes one.
#[test]
fn set_provisioned_false_opens_and_true_never_closes() {
    let (mut window, _) = SetupWindow::boot(true, 0);

    assert_eq!(window.set_provisioned(true, 1), None);
    assert!(!window.is_open());

    assert_eq!(
        window.set_provisioned(false, 2),
        Some(SetupEvent::Opened(RaiseReason::Forgotten))
    );
    assert!(window.is_open());

    assert_eq!(
        window.set_provisioned(true, 3),
        None,
        "provisioning again does not close the window"
    );
    assert!(window.is_open());
    assert_eq!(window.reason(), Some(RaiseReason::Forgotten));

    assert_eq!(
        window.set_provisioned(false, 4),
        None,
        "clearing an already-set flag does not raise a second window"
    );
    assert_eq!(window.remaining_ms(4), setup::WINDOW_MS - 2);
}

/// Opening resets the ten-minute timer; closing clears the reason and the stable timer.
#[test]
fn opening_resets_the_timer_and_closing_clears_the_reason() {
    let mut window = unprovisioned_boot(0);

    assert_eq!(window.tick(10_000, true), None);
    assert_eq!(
        window.tick(40_000, true),
        Some(SetupEvent::Closed(CloseReason::StationStable))
    );
    assert_eq!(window.reason(), None);
    assert_eq!(window.remaining_ms(40_000), 0);

    assert_eq!(
        window.panel_request(50_000, true),
        Some(SetupEvent::Opened(RaiseReason::PanelRequest))
    );
    assert_eq!(window.reason(), Some(RaiseReason::PanelRequest));
    assert_eq!(window.remaining_ms(50_000), setup::WINDOW_MS);
    assert_eq!(
        window.tick(80_000, true),
        None,
        "the closed window's stable timer does not leak into the new one"
    );
}
