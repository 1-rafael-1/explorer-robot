//! Host tests for the watchdog's task-liveness policy.
//!
//! The policy owns no clock and no timer: the firmware passes a monotonic millisecond stamp to
//! every beat and to the watchdog's check. These tests pin the two rules the firmware relies on —
//! only covered tasks are checked, and a task whose last beat is older than the maximum age is
//! named as stale — including across the `u32` millisecond wrap.

use bridge_core::heartbeat::{CoveredTask, HeartbeatSet};

/// The maximum silence used throughout these tests.
const MAX_AGE_MS: u32 = 5_000;

/// With nothing covered the watchdog has nothing to check.
#[test]
fn an_empty_set_is_alive() {
    let set = HeartbeatSet::new();

    assert!(set.all_alive(10_000_000, MAX_AGE_MS));
    assert_eq!(set.stale(10_000_000, MAX_AGE_MS), None);
}

/// A covered task is alive while its last beat is inside the maximum age.
#[test]
fn a_covered_task_is_alive_inside_the_window() {
    let mut set = HeartbeatSet::new();
    set.seed(1_000);
    set.cover(CoveredTask::UartRx);

    assert!(set.all_alive(1_000, MAX_AGE_MS));
    assert!(set.all_alive(1_000 + MAX_AGE_MS, MAX_AGE_MS));
    assert_eq!(set.stale(1_000 + MAX_AGE_MS, MAX_AGE_MS), None);
}

/// A covered task that misses its window is named as stale.
#[test]
fn a_silent_task_becomes_stale_after_the_maximum_age() {
    let mut set = HeartbeatSet::new();
    set.seed(1_000);
    set.cover(CoveredTask::UartRx);

    assert_eq!(set.stale(1_000 + MAX_AGE_MS + 1, MAX_AGE_MS), Some(CoveredTask::UartRx));
}

/// A beat moves the task's deadline forward.
#[test]
fn a_beat_refreshes_the_deadline() {
    let mut set = HeartbeatSet::new();
    set.seed(0);
    set.cover(CoveredTask::UartTx);

    set.beat(CoveredTask::UartTx, 4_000);
    assert_eq!(set.stale(8_999, MAX_AGE_MS), None, "9 s − 4 s is inside the window");
    assert_eq!(
        set.stale(9_001, MAX_AGE_MS),
        Some(CoveredTask::UartTx),
        "5 s of silence is the edge; past it the task is stale"
    );
}

/// Coverage is what turns a slot into a requirement; an uncovered slot never goes stale.
#[test]
fn uncovered_tasks_are_not_checked() {
    let mut set = HeartbeatSet::new();
    set.seed(0);
    set.cover(CoveredTask::UartRx);
    set.beat(CoveredTask::UartRx, 9_999);

    assert!(set.all_alive(10_000, MAX_AGE_MS));
    assert_eq!(set.stale(10_000, MAX_AGE_MS), None);
}

/// Only the stalled task is reported, even when several are covered.
#[test]
fn the_stalled_task_is_named_among_the_others() {
    let mut set = HeartbeatSet::new();
    set.seed(0);
    for task in [
        CoveredTask::UartRx,
        CoveredTask::UartTx,
        CoveredTask::NetStation,
        CoveredTask::Beacon,
    ] {
        set.cover(task);
    }

    let now = 6_000;
    set.beat(CoveredTask::UartRx, now);
    set.beat(CoveredTask::UartTx, now);
    set.beat(CoveredTask::NetStation, now);
    // Beacon stayed at 0, now more than the maximum age in the past.

    assert_eq!(set.stale(now, MAX_AGE_MS), Some(CoveredTask::Beacon));
}

/// Every slot is independent: beating one never refreshes another.
#[test]
fn every_slot_is_independent() {
    let mut set = HeartbeatSet::new();
    set.seed(0);
    for task in CoveredTask::ALL {
        set.cover(task);
    }

    let now = 6_000;
    for task in CoveredTask::ALL {
        if task != CoveredTask::Client2 {
            set.beat(task, now);
        }
    }

    assert_eq!(set.stale(now, MAX_AGE_MS), Some(CoveredTask::Client2));
}

/// The millisecond clock wraps at `u32::MAX`; the age arithmetic must wrap with it.
#[test]
fn the_millisecond_clock_wraps() {
    let mut set = HeartbeatSet::new();
    let before_wrap = u32::MAX - 100;
    set.seed(before_wrap);
    set.cover(CoveredTask::UartRx);
    set.beat(CoveredTask::UartRx, before_wrap);

    let after_wrap = 99u32; // 200 ms later.
    assert_eq!(set.stale(after_wrap, MAX_AGE_MS), None, "a wrapped stamp is fresh");

    let past_wrap = after_wrap + MAX_AGE_MS + 1;
    assert_eq!(set.stale(past_wrap, MAX_AGE_MS), Some(CoveredTask::UartRx));
}

/// `seed` gives every slot, covered or not, the same starting stamp.
#[test]
fn seed_initialises_every_slot() {
    let mut set = HeartbeatSet::new();
    set.seed(123);
    set.cover(CoveredTask::NetAccessPoint);

    assert_eq!(set.last_beat(CoveredTask::UartRx), 123);
    assert_eq!(set.last_beat(CoveredTask::NetAccessPoint), 123);
    assert!(set.all_alive(123 + MAX_AGE_MS, MAX_AGE_MS));
}

/// Every variant appears once, in index order, so a duplicate or misplaced `ALL` entry cannot
/// make a slot beat against another task's timestamp.
#[test]
fn every_variant_owns_its_indexed_slot() {
    for (index, task) in CoveredTask::ALL.into_iter().enumerate() {
        assert_eq!(task.index(), index, "{} is out of order", task.label());
    }
}

/// The labels are stable for the firmware's log line and the watchdog coverage table.
#[test]
fn the_task_labels_are_stable() {
    assert_eq!(CoveredTask::UartRx.label(), "uart-rx");
    assert_eq!(CoveredTask::UartTx.label(), "uart-tx");
    assert_eq!(CoveredTask::NetStation.label(), "net-station");
    assert_eq!(CoveredTask::NetAccessPoint.label(), "net-ap");
    assert_eq!(CoveredTask::Beacon.label(), "beacon");
    assert_eq!(CoveredTask::Client0.label(), "client-0");
    assert_eq!(CoveredTask::Client1.label(), "client-1");
    assert_eq!(CoveredTask::Client2.label(), "client-2");
}
