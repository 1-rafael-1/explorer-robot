//! Touch UI controller: the panel's single owner and the menu's sole owner.
//!
//! One task on core0 owns the display, the touch controller, and the
//! [`touch_ui::Ui`] model. It brings the panel up with retry, samples the
//! calibrated touch stream, feeds pointer events into the model, renders a full
//! frame on every state change (throttled during a drag), and flushes it. No
//! other task draws.
//!
//! # Loop
//!
//! Idle, the task selects over three sources:
//! 1. the [`UiEvent`] channel — lifecycle events from the orchestrator, the boot
//!    flow, and the calibrations;
//! 2. a periodic tick — refreshes the running screen from the activity state, the
//!    System Info snapshot while that screen is shown, and the fixed header's
//!    link and battery indicators on every screen;
//! 3. the pen-down edge — starts a gesture, which is then polled to pen-up.
//!
//! # Starting procedures
//!
//! The model owns the tap-versus-drag decision, so it is the authority on what a
//! gesture activated: after a release the controller reads the region the model
//! activated and the screen and value that were showing when the press began, and
//! starts the procedure, records the value, or stops the running one. Nothing
//! here formats display text — producers publish phases, percents and results
//! through [`crate::system::state::activity`], and this task renders them.
//!
//! # Rendering
//!
//! The UI is the only thing that draws: producers publish state (the activity
//! state module), and the retired text display contract no longer exists.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{debug, error, info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};
use link_state::LinkPhase;
use touch_ui::{
    CalibrationStatus as UiCalibrationStatus, DRAG_RENDER_MS, Hit, ImuReadout, ImuReadoutKind, Item, Procedure, Screen,
    ScreenEntry, SensorState, StatusView, SystemInfo, TICK_MS, Ui, ValueFlow,
};

use crate::{
    resources::PanelResources,
    system::state::{
        CalibrationStatus,
        activity::{self, Activity, Snapshot, Stage},
        calibration, imu_test, link, perception, power,
    },
    task::{
        autonomous_mode::{attempt_straight_line, coast_obstacle_avoid},
        drive::{
            InterruptKind,
            calibration::{self as drive_calibration, distance::DistanceDecision},
        },
        io::panel::Panel,
        procedure::{self, Claim, ClaimKind, ClaimOrigin, ScreenKind, StartOutcome, StopOutcome},
        remote_link,
        sensors::lidar::{self, LidarStatus},
    },
};

// ── UI event channel ────────────────────────────────────────────────────────────

/// Events delivered to the UI controller task from the orchestrator, the boot
/// flow, and the calibrations.
#[derive(Debug, Clone, Copy)]
pub enum UiEvent {
    /// Request to show the main menu (from initialisation or the boot gate).
    ShowMainMenu,
    /// Testing sequence finished; the landing screen comes from the finished
    /// Procedure's identity, and `generation` identifies the run that finished.
    TestingCompleted {
        /// The Procedure that finished.
        procedure: Procedure,
        /// The claim generation that owned the run.
        generation: u32,
    },
    /// Calibration procedure finished; the outcome is in the activity state.
    CalibrationCompleted {
        /// The claim generation that owned the run.
        generation: u32,
    },
    /// The distance calibration's drive step finished; the value screen follows
    /// from the activity state.
    DistanceDriveFinished,
}

/// Channel carrying [`UiEvent`]s into the UI controller task.
///
/// Capacity 64 ensures the orchestrator never blocks on UI delivery.
static UI_EVENT_CHANNEL: Channel<CriticalSectionRawMutex, UiEvent, 64> = Channel::new();

/// Send an event to the UI controller task.
pub async fn send_ui_event(event: UiEvent) {
    UI_EVENT_CHANNEL.sender().send(event).await;
}

/// Requests that the coast-and-avoid drive mode start, carrying the claim it runs
/// under.
///
/// The start publishes "Enabling `LiDAR`", then hands the claim to [`coast_avoid_task`]
/// through this channel. Capacity one: a start is dispatched only after the previous
/// claim was received, and the one Active Claim already prevents two starts.
static COAST_AVOID_REQUEST: Channel<CriticalSectionRawMutex, Claim, 1> = Channel::new();

/// Latched intent of the Room Scan screen: `true` while the radar screen wants
/// the sensor, `false` once it has left.
///
/// The screen records its latest intent here and signals, so the lifecycle task
/// converges on the current state instead of replaying queued requests and can
/// never lose the final disable to a full queue.
static ROOM_SCAN_WANTED: AtomicBool = AtomicBool::new(false);

/// Wakes [`room_scan_task`] whenever [`ROOM_SCAN_WANTED`] changes.
///
/// [`Signal`] coalesces — a signal stays set until the task consumes it — so a
/// store-and-signal that lands just before a `wait()` makes that `wait()` return
/// immediately and no state change can be lost.
static ROOM_SCAN_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The Room Scan claim, held while the radar screen is open.
///
/// The radar is not a Procedure, so its claim publishes no Activity; it exists so
/// a Procedure cannot start behind the radar and the radar cannot open behind a
/// Procedure. `None` while the radar is closed, and the stored claim is the only
/// one there is: entering again while it is held is refused by the claim slot.
static ROOM_SCAN_CLAIM: Mutex<CriticalSectionRawMutex, Option<Claim>> = Mutex::new(None);

// ── Timing ────────────────────────────────────────────────────────────────────

/// Poll interval, in milliseconds, while the boot gate waits for initialisation.
const BOOT_POLL_MS: u64 = 100;

/// Idle tick period, in milliseconds.
///
/// The IMU Test Readout updates at 10 Hz, so the tick is 100 ms. Equality checks
/// on the status view keep the other screens from flushing a full frame unless a
/// displayed value changes; only an IMU test redraws at this rate.
const TICK_INTERVAL_MS: u64 = 100;

/// Continuous no-contact window, in milliseconds, that ends a gesture whose
/// interrupt line is stuck low.
///
/// A healthy controller reports no contact only while the pen lifts, and
/// `PENIRQ` goes high at the same moment, so the line ends the gesture first. A
/// stuck-low line instead reports no contact indefinitely while never lifting.
/// 500 ms — 25 gesture ticks at [`TICK_MS`] — is long enough that no deliberate
/// hold on a running screen reads as a fault, and short enough that the screen's
/// Touch Stop stays reachable.
const NO_CONTACT_LIMIT_MS: u64 = 500;

/// Last-resort deadline, in milliseconds, on a single gesture.
///
/// The complement of [`NO_CONTACT_LIMIT_MS`]: the line is stuck low *and* the
/// controller keeps reporting contact, so neither the line nor the sample stream
/// ever signals a release. No operator holds a tap or a drag for 30 s, so the
/// deadline cannot cut a deliberate press short, but it bounds the gesture when
/// no other signal will.
const GESTURE_DEADLINE_MS: u64 = 30_000;

/// Backoff before retrying an offline panel, in milliseconds.
const REINIT_BACKOFF_MS: u64 = 2_000;

/// Interval between `LiDAR` status polls while the coast-and-avoid helper waits
/// for the sensor to warm up, in milliseconds.
const LIDAR_STATUS_POLL_MS: u64 = 50;

// ── Initialisation ──────────────────────────────────────────────────────────────

/// Build the panel, spawn the UI controller task, and spawn the helper tasks it
/// delegates to.
///
/// The controller owns the panel's display, touch controller, and framebuffer
/// for the lifetime of the firmware. The helper tasks own the procedures that
/// cannot run on the UI task without freezing the panel: the coast-and-avoid
/// mode's `LiDAR` enable, which may take several seconds.
#[allow(clippy::unwrap_used)]
pub fn init_ui(spawner: Spawner, pins: PanelResources) {
    spawner.spawn(ui_task(Panel::new(pins)).unwrap());
    spawner.spawn(coast_avoid_task(spawner).unwrap());
    spawner.spawn(room_scan_task().unwrap());
}

/// Return true once calibration data has been queried.
pub async fn ui_initialized() -> bool {
    calibration::is_initialized().await
}

/// Return true if a calibration is currently under way.
///
/// True from the moment a calibration procedure starts until the operator saves,
/// cancels or dismisses it, so the boot flow does not pull the panel to the main
/// menu out from under a calibration.
pub async fn ui_is_calibrating() -> bool {
    activity::is_calibrating().await
}

/// Request that the UI show the main menu.
pub async fn show_main_menu() {
    send_ui_event(UiEvent::ShowMainMenu).await;
}

/// Publish "Enabling `LiDAR`" and hand `claim` to [`coast_avoid_task`].
///
/// This is the coast-and-avoid arm of `procedure::dispatch`: the claim is already
/// taken and the starting Activity published, so this only replaces the phase
/// line with the warm-up message and hands the claim over. The enable can take
/// seconds, so it runs on the helper task and this returns at once.
///
/// Returns `false` when the hand-off channel refused the claim; the claim is
/// abandoned in that case so work that cannot run leaves no Activity behind.
pub async fn request_coast_and_avoid(claim: Claim) -> bool {
    claim.phase("Enabling LiDAR", None).await;
    if COAST_AVOID_REQUEST.try_send(claim).is_err() {
        claim.abandon().await;
        return false;
    }
    true
}

/// Warm up the `LiDAR` for coast-and-avoid, off the UI task, then hand the mode the
/// claim.
///
/// The start already published "Enabling `LiDAR`" and took the one Active Claim, so
/// the running screen is up from the first frame. Enabling can take seconds, so the
/// UI task must not await it: it sends the claim here and returns. This task polls
/// the sensor's lock-free status and makes the terminal claim call itself:
///
/// - a stop during the warm-up disables the sensor and finishes the claim;
/// - a failed bring-up disables the sensor and fails the claim with
///   "`LiDAR` unavailable";
/// - once streaming it spawns the mode task with the claim. A spawn failure
///   disables the sensor and abandons the claim so nothing keeps running.
#[embassy_executor::task]
async fn coast_avoid_task(spawner: Spawner) {
    loop {
        let claim = COAST_AVOID_REQUEST.receive().await;

        // [`lidar::enable`] releases a failure latched by a previous run before it
        // enables, so this run's own bring-up decides the outcome.
        lidar::enable().await;

        loop {
            if claim.is_stop_requested() {
                lidar::disable().await;
                claim.finish().await;
                info!("[ui] coast-and-avoid: stopped during LiDAR warm-up");
                break;
            }

            match lidar::status() {
                LidarStatus::Streaming => {
                    if !coast_obstacle_avoid::spawn(spawner, claim) {
                        lidar::disable().await;
                        claim.abandon().await;
                        error!("[ui] coast-and-avoid: mode task spawn failed");
                    }
                    break;
                }
                LidarStatus::Failed => {
                    // The sensor is powered down and will not retry until a fresh
                    // disable-then-enable, so clear it and refuse to drive.
                    lidar::disable().await;
                    claim.fail("LiDAR unavailable").await;
                    warn!("[ui] coast-and-avoid: LiDAR unavailable");
                    break;
                }
                LidarStatus::Off | LidarStatus::Warming => {
                    Timer::after(Duration::from_millis(LIDAR_STATUS_POLL_MS)).await;
                }
            }
        }
    }
}

/// Drive the Room Scan screen's `LiDAR` need, off the UI task.
///
/// The task converges on the screen's latest intent, latched in
/// [`ROOM_SCAN_WANTED`], rather than consuming queued requests, so a rapid
/// leave/re-enter cannot drop the final change. Entering enables the sensor and
/// leaving disables it; the sensor's own task owns the bring-up, so this task
/// never blocks on the multi-second warm-up. A failed bring-up leaves `Failed`
/// for the screen to show; re-entering enables again, and [`lidar::enable`]
/// releases the latched failure first, so the converged re-enter recovers rather
/// than wedging.
#[embassy_executor::task]
async fn room_scan_task() {
    loop {
        ROOM_SCAN_SIGNAL.wait().await;
        if ROOM_SCAN_WANTED.load(Ordering::Acquire) {
            lidar::enable().await;
        } else {
            lidar::disable().await;
        }
    }
}

// ── Controller task ─────────────────────────────────────────────────────────────

/// The touch UI task: owns the panel and the model, and drives both.
#[embassy_executor::task]
async fn ui_task(mut panel: Panel) {
    debug!("[ui] touch controller started");

    // Boot gate: do not let the operator navigate before calibration status is
    // known. The model starts on the Main Menu.
    while !ui_initialized().await {
        Timer::after(Duration::from_millis(BOOT_POLL_MS)).await;
    }

    let mut ui = Ui::new();
    let mut last_activity = Activity::Idle;
    let mut ticker = Ticker::every(Duration::from_millis(TICK_INTERVAL_MS));

    loop {
        if !panel.bring_up().await {
            warn!("panel offline; retrying initialisation");
            Timer::after(Duration::from_millis(REINIT_BACKOFF_MS)).await;
            continue;
        }
        if run_ui(&mut panel, &mut ui, &mut last_activity, &mut ticker)
            .await
            .is_err()
        {
            warn!("panel went offline; re-initialising");
        }
        Timer::after(Duration::from_millis(REINIT_BACKOFF_MS)).await;
    }
}

/// Drive the model while the panel is online, until a draw or flush fails.
///
/// Returns `Err(())` when the panel went offline, so the caller can re-run
/// bring-up.
async fn run_ui(panel: &mut Panel, ui: &mut Ui, last_activity: &mut Activity, ticker: &mut Ticker) -> Result<(), ()> {
    // A bring-up (or a re-bring-up) always repaints the current screen.
    render_and_flush(panel, ui).await?;

    loop {
        match select3(
            UI_EVENT_CHANNEL.receiver().receive(),
            ticker.next(),
            panel.wait_for_pen_down(),
        )
        .await
        {
            Either3::First(event) => {
                handle_event(ui, event).await;
                render_and_flush(panel, ui).await?;
            }
            Either3::Second(()) => {
                if refresh_screens(ui, last_activity).await {
                    render_and_flush(panel, ui).await?;
                }
            }
            Either3::Third(Some(point)) => {
                let screen_before = ui.screen();
                let value_before = ui.value();
                let now_ms = Instant::now().as_millis();
                if ui.pointer_down(point, now_ms) {
                    render_and_flush(panel, ui).await?;
                }
                run_gesture(panel, ui, now_ms, screen_before, value_before).await?;
            }
            Either3::Third(None) => {
                // A failed pen-down wait: yield a tick before retrying so a
                // persistently erroring touch controller cannot spin the loop.
                Timer::after(Duration::from_millis(TICK_MS)).await;
            }
        }
    }
}

/// Poll the pen until it lifts, feeding moves into the model, then act on what
/// the release activated.
///
/// Drag redraws are throttled to one per [`DRAG_RENDER_MS`]; the release always
/// renders once the activation has been applied, because starting or stopping a
/// procedure changes the screen.
///
/// `PENIRQ` is the primary pen-up signal, but it is not trusted without bound: a
/// line stuck low would otherwise pin this loop to the Panel forever and hide
/// every running screen's Touch Stop. Two bounds end the gesture exactly as a
/// real release does. [`NO_CONTACT_LIMIT_MS`] ends it when the controller keeps
/// reporting no contact while the line stays low — a sustained disagreement
/// between the sample stream and the line. [`GESTURE_DEADLINE_MS`] ends it when
/// contact reads nonzero for the whole gesture. A transient `Err(())` is a bus
/// error, not a no-contact report, so it neither advances the no-contact window
/// nor clears it. Each bound logs one warning naming which one fired, so a
/// genuine line fault is visible on the bench.
async fn run_gesture(
    panel: &mut Panel,
    ui: &mut Ui,
    started_ms: u64,
    screen_before: Screen,
    value_before: i32,
) -> Result<(), ()> {
    let mut last_drag_render = started_ms;
    // Start of the current run of ticks that reported no contact while the line
    // stayed low; `None` when the last valid sample reported contact.
    let mut no_contact_since_ms: Option<u64> = None;
    loop {
        Timer::after(Duration::from_millis(TICK_MS)).await;
        let now_ms = Instant::now().as_millis();
        // The failure bound, if any, that ends this gesture; `None` keeps it
        // running. A real pen-up never sets it, because it is an ordinary end.
        let mut bound: Option<(&str, u64)> = None;

        let sample = panel.read_touch().await;
        let pen_up = panel.pen_up();
        match sample {
            Ok(Some(point)) => {
                no_contact_since_ms = None;
                if ui.pointer_move(point, now_ms) && now_ms.saturating_sub(last_drag_render) >= DRAG_RENDER_MS {
                    render_and_flush(panel, ui).await?;
                    last_drag_render = now_ms;
                }
            }
            Ok(None) => {
                if !pen_up {
                    let since_ms = *no_contact_since_ms.get_or_insert(now_ms);
                    if now_ms.saturating_sub(since_ms) >= NO_CONTACT_LIMIT_MS {
                        bound = Some(("no-contact window with PENIRQ stuck low", NO_CONTACT_LIMIT_MS));
                    }
                }
            }
            // A transient bus error: retry, without counting it as reported
            // no-contact or clearing the window.
            Err(()) => {}
        }

        // The last-resort deadline, reached only when reads keep reporting
        // contact, so a genuine release is never attributed to it.
        if bound.is_none() && !pen_up && now_ms.saturating_sub(started_ms) >= GESTURE_DEADLINE_MS {
            bound = Some(("gesture deadline", GESTURE_DEADLINE_MS));
        }

        if let Some((reason, limit_ms)) = bound {
            warn!("touch gesture ended by {} ({} ms)", reason, limit_ms);
        }

        if pen_up || bound.is_some() {
            let _ = ui.pointer_up(now_ms);
            let activated = ui.last_activation();
            handle_activation(ui, screen_before, value_before, activated).await;
            refresh_system_info_on_entry(ui).await;
            render_and_flush(panel, ui).await?;
            return Ok(());
        }
    }
}

/// Render the current screen into the framebuffer and flush it.
async fn render_and_flush(panel: &mut Panel, ui: &Ui) -> Result<(), ()> {
    ui.render(panel.display_mut()).map_err(|_| ())?;
    panel.flush().await
}

/// Whether a newer claim replaced the run that raised a terminal event.
///
/// The claim releases the slot before the event reaches the UI, so a Remote can
/// start a Manual Drive in that window. Dismissing the terminal record blindly
/// would clear the newer run's Activity and screen; this guard leaves it alone.
fn stale_completion(generation: u32) -> bool {
    procedure::live_generation().is_some_and(|live| live.get() != generation)
}

/// Apply a lifecycle event to the model.
async fn handle_event(ui: &mut Ui, event: UiEvent) {
    match event {
        UiEvent::ShowMainMenu => *ui = Ui::new(),
        UiEvent::TestingCompleted { procedure, generation } => {
            if stale_completion(generation) {
                return;
            }
            let landing = procedure.landing_screen();
            activity::clear().await;
            *ui = Ui::new();
            ui.show_screen(landing);
        }
        UiEvent::CalibrationCompleted { generation } => hold_or_leave_calibration(ui, generation).await,
        UiEvent::DistanceDriveFinished => enter_distance_entry(ui).await,
    }
}

// ── Activation handling ─────────────────────────────────────────────────────────

/// Act on the region the model activated, given the screen and value the press
/// began on.
///
/// The model has already performed its own navigation (opening a submenu, going
/// back from a value screen), so this only starts, stops or saves things, and
/// replaces the screen when a running procedure takes over from the leaf's own
/// destination.
async fn handle_activation(ui: &mut Ui, screen_before: Screen, value_before: i32, activated: Option<Hit>) {
    match (screen_before, activated) {
        // A menu entry carries its identity, so what it starts does not depend on
        // the screen it was tapped on: the item names its own screen.
        (_, Some(Hit::MenuItem(item))) => handle_menu_item(ui, item).await,
        (Screen::RoomScan, Some(Hit::Back)) => leave_room_scan().await,
        (Screen::ValueEntry(ValueFlow::DistanceCalibration), Some(Hit::Save)) => {
            save_distance_calibration(value_before).await;
        }
        (Screen::ValueEntry(ValueFlow::DistanceCalibration), Some(Hit::Cancel)) => {
            cancel_distance_calibration().await;
        }
        (Screen::ValueEntry(ValueFlow::AttemptStraight), Some(Hit::Save)) => {
            record_attempt_straight(value_before).await;
        }
        (Screen::Status, Some(Hit::Back)) => activity::clear().await,
        (Screen::Status, Some(Hit::Stop)) => stop_running(),
        (Screen::Link, Some(Hit::LinkAction(action))) => request_link_action(action).await,
        _ => {}
    }
}

/// Ask the Link Task to send one Bridge Control request.
///
/// One request is in flight at a time: a pending outcome refuses the tap rather
/// than queueing it behind the first. A Bridge that is off or still waiting for
/// the board is also refused, so the action cannot become `Pending` with no
/// frame to time out (and a no-op Forget cannot fire the failsafe). The request
/// itself is a non-blocking `try_send` to the Link Task, so the panel never waits
/// on the link; the next [`refresh_screens`] tick reads the pending outcome and
/// then the reply or timeout back out of [`link`]. The Link screen is a
/// `ScreenEntry`, so this takes no Active Claim and stops no local work; only
/// `ForgetNetwork` fires the Remote failsafe, and the Link Task does that.
async fn request_link_action(action: touch_ui::LinkAction) {
    let snapshot = link::snapshot(Instant::now().as_millis()).await;
    if snapshot.control.is_pending() {
        debug!("[ui] link action refused: a bridge control request is already in flight");
        return;
    }
    if !matches!(snapshot.phase, LinkPhase::ReadyNoPeer | LinkPhase::PeerLive) {
        debug!("[ui] link action refused: the bridge is not ready");
        return;
    }

    if !remote_link::request_control(action).await {
        warn!("[ui] link action dropped: the link task command channel is full");
    }
}

/// Act on the menu entry the model activated.
///
/// The match is exhaustive over the crate's [`Item`], so adding an entry — or
/// moving one to another screen — is a compile-time question rather than a
/// silent change of which procedure a tap starts. Every startable Procedure
/// goes through the one `procedure::dispatch` seam that the Link Task will also
/// call; Attempt Straight is a value-entry flow, so tapping it starts nothing.
async fn handle_menu_item(ui: &mut Ui, item: Item) {
    match item {
        Item::ScreenEntry(ScreenEntry::RoomScan) => enter_room_scan(ui).await,
        // Attempt Straight is a value-entry flow, and the submenus and System Info
        // are opened by the model on its own: nothing starts here.
        Item::Procedure(Procedure::AttemptStraight)
        | Item::Submenu(_)
        | Item::ScreenEntry(ScreenEntry::SystemInfo | ScreenEntry::Link) => {}
        Item::Procedure(procedure) => start_procedure(ui, procedure).await,
    }
}

/// Start `procedure` through the shared dispatch seam and show its running view.
///
/// The model has already navigated to the entry's destination. A `Started`
/// dispatch has taken the one Active Claim and routed it to its controller, so
/// the running screen follows from Activity State. A `Busy` or `Rejected` start
/// goes back to the entry's landing screen: leaving the running screen up would
/// draw a procedure with nothing behind it. The landing screen comes from the
/// entry's identity, not from the stale status view the model still holds from
/// whatever ran last.
async fn start_procedure(ui: &mut Ui, procedure: Procedure) {
    match procedure::dispatch(procedure, ClaimOrigin::Panel).await {
        StartOutcome::Started(_) => {
            let snapshot = activity::snapshot().await;
            ui.show_status(status_view(&snapshot, None));
        }
        StartOutcome::Busy | StartOutcome::Rejected => {
            ui.show_screen(procedure.landing_screen());
        }
    }
}

/// Enter the Room Scan screen: claim the one Active Claim and enable the sensor.
///
/// The model has already navigated to the radar screen. If another claim holds the
/// slot the entry is refused and the screen leaves again, so the radar never opens
/// alongside a running Procedure and the `LiDAR` is left alone. The claim publishes
/// no Activity; it is stored in [`ROOM_SCAN_CLAIM`] until the screen leaves. The
/// intent is latched for [`room_scan_task`] to pick up, so this returns immediately
/// and the screen shows the sensor warming.
async fn enter_room_scan(ui: &mut Ui) {
    let Some(claim) = procedure::claim(ClaimKind::Screen(ScreenKind::RoomScan), ClaimOrigin::Panel).await else {
        warn!("[ui] room scan refused — another claim is held");
        let _ = ui.back();
        return;
    };

    *ROOM_SCAN_CLAIM.lock().await = Some(claim);
    ROOM_SCAN_WANTED.store(true, Ordering::Release);
    ROOM_SCAN_SIGNAL.signal(());
}

/// Leave the Room Scan screen: abandon the claim and clear the `LiDAR` need it
/// holds.
///
/// The claim is taken from [`ROOM_SCAN_CLAIM`], so one leave releases the slot even
/// if the screen leaves twice, and a late leave cannot free a newer claim: `abandon`
/// checks the generation and is otherwise a no-op. The intent is latched rather
/// than awaited, so the panel never blocks; the lifecycle task acts on it even if
/// the leave lands while the sensor is still warming. Called on every exit path,
/// including a failed bring-up, it clears only the need this screen actually holds,
/// so a failed bring-up powers down nothing and cannot stop a sensor another mode
/// owns.
async fn leave_room_scan() {
    let claim = ROOM_SCAN_CLAIM.lock().await.take();
    if let Some(claim) = claim {
        claim.abandon().await;
    }

    ROOM_SCAN_WANTED.store(false, Ordering::Release);
    ROOM_SCAN_SIGNAL.signal(());
}

/// Record the attempt-straight distance the operator set, without starting it.
///
/// Starting that mode is out of scope, so the entry only stores the value in the
/// deferred mode's display state and returns to the Drive Mode menu.
async fn record_attempt_straight(value_cm: i32) {
    let target_cm = u16::try_from(value_cm).unwrap_or_else(|_| attempt_straight_line::target_preset_cm());
    attempt_straight_line::set_requested_target_cm(target_cm).await;
    debug!("[ui] attempt-straight recorded {} cm", target_cm);
}

/// Send the measured distance to the calibration task, which stores the factor.
///
/// The task holds the claim across the value screen and makes the terminal call, so
/// the UI sends only the decision: it never writes the factor or clears Activity.
async fn save_distance_calibration(measured_cm: i32) {
    drive_calibration::distance::send_decision(DistanceDecision::Commit(measured_cm)).await;
}

/// Tell the calibration task to restore the factor in force before the run.
///
/// The task restores it and abandons the claim; the UI's Cancel has already left the
/// value screen, and nothing here touches Activity.
async fn cancel_distance_calibration() {
    drive_calibration::distance::send_decision(DistanceDecision::Cancel).await;
}

/// Stop whatever the running screen is showing.
///
/// `procedure::stop` latches the stop for whichever claim holds the slot and
/// interrupts a drive in flight. The claim's owner observes the latch, cleans up
/// and records the terminal outcome through `finish`, so nothing here clears the
/// Activity. A panel Stop is an operator stop; it coasts, as it always has.
fn stop_running() {
    procedure::stop(StopOutcome::Operator, InterruptKind::Stop);
}

/// React to a finished calibration: leave a completed one, hold a failed one's
/// reason on screen until the operator dismisses it.
async fn hold_or_leave_calibration(ui: &mut Ui, generation: u32) {
    if stale_completion(generation) {
        return;
    }

    let snapshot = activity::snapshot().await;

    match snapshot.activity {
        Activity::Procedure(procedure) if procedure.is_calibration() => {
            if snapshot.stage == Stage::Failed {
                ui.set_status(status_view(&snapshot, None));
            } else {
                let landing = procedure.landing_screen();
                activity::clear().await;
                *ui = Ui::new();
                ui.show_screen(landing);
            }
        }
        Activity::Procedure(_) | Activity::ManualDrive(_) | Activity::Booting | Activity::Idle => {
            // A stopped or already-dismissed calibration: the panel has moved on.
        }
    }
}

/// Show the distance value screen once the drive step has finished.
///
/// The calibration task publishes "Enter measured distance" before it sends the
/// event, so the snapshot is running with that phase and the value screen takes
/// over. The guards are defensive: the event is sent only for a completed drive, so
/// an idle snapshot means the operator left the running screen while the event was
/// in flight — the task is still waiting on the decision and holds the claim, so it
/// is told to cancel rather than left holding the slot.
async fn enter_distance_entry(ui: &mut Ui) {
    let snapshot = activity::snapshot().await;

    if snapshot.activity == Activity::Idle {
        // The panel already left and cleared Activity; the task still owns the
        // claim and waits for the decision, so cancel it to restore the factor and
        // release.
        drive_calibration::distance::send_decision(DistanceDecision::Cancel).await;
        return;
    }

    if snapshot.stage == Stage::Failed {
        ui.set_status(status_view(&snapshot, None));
    } else {
        ui.show_value_entry(ValueFlow::DistanceCalibration);
    }
}

// ── Running screen ─────────────────────────────────────────────────────────────

/// The running screen's content for a snapshot.
///
/// A drive mode's body line is the `LiDAR`'s lifecycle state, read lock-free on
/// each tick so the screen reports the sensor warming up while the bring-up
/// runs, then streaming once it settles, without the controller formatting text
/// of its own.
///
/// `readout` is the IMU Test Telemetry snapshot. An IMU Procedure attaches it to
/// the view; every other Procedure ignores it. When no snapshot has been
/// published yet the view carries an empty Waiting readout, so the panel shows
/// placeholders from the moment the test starts.
fn status_view(snapshot: &Snapshot, readout: Option<ImuReadout>) -> StatusView {
    // A running Procedure names itself: its title and where Stop returns both come
    // from the Menu Entry that started it. The boot flow and the idle state have no
    // entry, so they fall back to their own title and the Main Menu.
    let body = status_body(snapshot);
    let view = snapshot.procedure().map_or_else(
        || {
            // A Manual Drive has no Menu Entry; its dynamic command label is the
            // body line, and Back returns to the Main Menu.
            let base = StatusView::new(snapshot.title(), body, Screen::MainMenu);
            snapshot
                .manual_drive_label()
                .map_or(base, |label| base.with_text_body(label))
        },
        |procedure| StatusView::for_procedure(procedure, body),
    );
    let view = snapshot
        .procedure()
        .and_then(ImuReadoutKind::for_procedure)
        .map_or(view, |kind| {
            view.with_readout(readout.unwrap_or_else(|| ImuReadout::waiting(kind)))
        });
    let view = snapshot.percent.map_or(view, |percent| view.with_progress(percent));
    if matches!(snapshot.stage, Stage::Complete | Stage::Failed) {
        view.finished()
    } else {
        view
    }
}

/// The body line a running screen shows for a snapshot.
///
/// A drive mode reports the sensor's state while it runs; once its start has
/// failed, the reason the producer recorded through the claim's `fail` takes over
/// the line.
fn status_body(snapshot: &Snapshot) -> &'static str {
    match snapshot.activity {
        Activity::Procedure(Procedure::CoastAndAvoid) if snapshot.stage != Stage::Failed => {
            map_sensor_state(lidar::status()).label()
        }
        Activity::Idle | Activity::Booting | Activity::ManualDrive(_) | Activity::Procedure(_) => snapshot.detail,
    }
}

/// Map the firmware's `LiDAR` lifecycle onto the UI's neutral sensor state.
const fn map_sensor_state(status: LidarStatus) -> SensorState {
    match status {
        LidarStatus::Off => SensorState::Off,
        LidarStatus::Warming => SensorState::Warming,
        LidarStatus::Streaming => SensorState::Streaming,
        LidarStatus::Failed => SensorState::Failed,
    }
}

// ── System Info ─────────────────────────────────────────────────────────────────

/// Refresh the tick's screens and follow a newly started Activity.
///
/// The running screen is refreshed from the one Activity snapshot read here, so
/// a remote-started Procedure or Manual Drive appears without the operator
/// navigating there. Following is gated by `last_activity`: only a transition
/// from Idle pulls the panel onto the running screen, so leaving it is not undone
/// by the next tick. When a followed Activity goes Idle the panel leaves the
/// running screen for its parent. Booting is deliberately not followed.
///
/// The header indicator is refreshed on every tick from `state::link` and
/// `state::power` alone, so an absent Link Task leaves the panel fully usable.
///
/// Reports whether anything on screen changed.
async fn refresh_screens(ui: &mut Ui, last_activity: &mut Activity) -> bool {
    let snapshot = activity::snapshot().await;
    let previous = *last_activity;
    let mut redraw = refresh_running_screen(ui, &snapshot).await;

    // The header indicator reads only the link state and power, never the Link
    // Task or the port power, so its values cannot block the panel.
    redraw |= ui.set_link(link::snapshot(Instant::now().as_millis()).await);
    redraw |= ui.set_battery(power::get_battery_snapshot().await.level);

    if ui.screen() != Screen::Status {
        if previous == Activity::Idle && matches!(snapshot.activity, Activity::Procedure(_) | Activity::ManualDrive(_))
        {
            redraw |= ui.show_status(status_view(&snapshot, None));
        }
    } else if snapshot.activity == Activity::Idle
        && matches!(previous, Activity::Procedure(_) | Activity::ManualDrive(_))
    {
        redraw |= ui.back();
    }

    *last_activity = snapshot.activity;

    if ui.screen() == Screen::SystemInfo {
        redraw |= ui.set_system_info(read_system_info().await);
    }

    if ui.screen() == Screen::RoomScan {
        redraw |= refresh_room_scan(ui).await;
    }

    redraw
}

/// Refresh the Room Scan radar frame and sensor caption, reporting a change.
///
/// The radar comes from the perception snapshot and is `None` while no snapshot
/// exists — before the sensor warms or after it is disabled — which the screen
/// draws as "No data", never as an empty room. The cloud crosses owned with its
/// sequence as the change token, so an unchanged sequence costs one counter
/// comparison and no copy of the slot array (ADR-0016). The caption comes from
/// the sensor's lock-free lifecycle status.
async fn refresh_room_scan(ui: &mut Ui) -> bool {
    let mut redraw = ui.set_sensor_state(map_sensor_state(lidar::status()));
    redraw |= perception::with_lidar(|cloud| ui.set_radar(cloud.map(|c| (c.slots(), c.sequence())))).await;
    redraw
}

/// Re-render the running screen from `snapshot`, reporting a change.
///
/// A no-op off the running screen, and while nothing is recorded — a producer
/// that has not published its first phase yet leaves the screen as it was. The
/// caller has already read the snapshot, so the follow rule and this refresh
/// share one Activity read.
///
/// For an IMU Procedure the IMU Test Telemetry snapshot is read on the same
/// tick, so the readout updates at the tick rate. Other screens never take that
/// lock.
async fn refresh_running_screen(ui: &mut Ui, snapshot: &Snapshot) -> bool {
    if ui.screen() != Screen::Status || snapshot.activity == Activity::Idle {
        return false;
    }

    let readout = match snapshot.procedure() {
        Some(procedure) if ImuReadoutKind::for_procedure(procedure).is_some() => imu_test::snapshot().await,
        _ => None,
    };
    ui.set_status(status_view(snapshot, readout))
}

/// Refresh the System Info snapshot when the model is showing that screen.
async fn refresh_system_info_on_entry(ui: &mut Ui) {
    if ui.screen() == Screen::SystemInfo {
        let info = read_system_info().await;
        ui.set_system_info(info);
    }
}

/// Read the battery and the three calibration statuses into a neutral snapshot.
///
/// The lock order documented by the state modules is respected: the power
/// accessor is called before `CALIBRATION_STATE` is locked.
async fn read_system_info() -> SystemInfo {
    let battery = power::get_battery_snapshot().await;
    let (motor, mag, distance) = {
        let state = calibration::CALIBRATION_STATE.lock().await;
        (state.motor_cal_status, state.mag_cal_status, state.distance_cal_status)
    };

    SystemInfo {
        battery_level: battery.level,
        battery_voltage: battery.voltage,
        motor_calibration: map_calibration_status(motor),
        mag_calibration: map_calibration_status(mag),
        distance_calibration: map_calibration_status(distance),
    }
}

/// Map the firmware's calibration status onto the UI's neutral one.
///
/// A never-queried status is *unknown*, not missing: only flash reporting that
/// no calibration exists is *missing*.
const fn map_calibration_status(status: CalibrationStatus) -> UiCalibrationStatus {
    match status {
        CalibrationStatus::Loaded => UiCalibrationStatus::Loaded,
        CalibrationStatus::NotLoaded => UiCalibrationStatus::Unknown,
        CalibrationStatus::NotAvailable => UiCalibrationStatus::Missing,
    }
}
