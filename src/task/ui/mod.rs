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
//!    flow, and the distance-calibration drive the UI spawns;
//! 2. a periodic tick — refreshes the running screen from the activity state and
//!    the System Info snapshot while that screen is shown;
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

use defmt::{debug, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Ticker, Timer};
use touch_ui::{
    CalibrationStatus as UiCalibrationStatus, DRAG_RENDER_MS, Hit, Item, Procedure, Screen, ScreenEntry, SensorState,
    StatusView, SystemInfo, TICK_MS, Ui, ValueFlow,
};

use crate::{
    system::state::{
        CalibrationStatus,
        activity::{self, Activity, Snapshot, Stage},
        calibration, perception, power,
    },
    task::{
        autonomous_mode::{attempt_straight_line, coast_obstacle_avoid},
        drive::{
            self, DriveCommand, ImuCalibrationKind,
            calibration::{self as drive_calibration, distance::DriveOutcome},
        },
        io::panel::{Panel, PanelPins},
        sensors::lidar::{self, LidarStatus},
        testmode,
    },
};

// ── UI event channel ────────────────────────────────────────────────────────────

/// Events delivered to the UI controller task from the orchestrator, the boot
/// flow, and the UI's own distance-calibration drive task.
#[derive(Debug, Clone, Copy)]
pub enum UiEvent {
    /// Request to show the main menu (from initialisation or the boot gate).
    ShowMainMenu,
    /// Testing sequence finished; the landing screen comes from the finished
    /// Procedure's identity.
    TestingCompleted,
    /// Calibration procedure finished; the outcome is in the activity state.
    CalibrationCompleted,
    /// The distance calibration's drive step finished or failed; the value screen
    /// or the failure reason follows from the activity state.
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

/// Requests that the distance calibration's drive step run.
static DISTANCE_DRIVE_REQUEST: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

/// Requests that the coast-and-avoid drive mode start.
static COAST_AVOID_REQUEST: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

/// What the UI asks of the Room Scan sensor lifecycle task.
#[derive(Clone, Copy)]
enum RoomScanRequest {
    /// Power on the sensor and wait until it is streaming.
    Acquire,
    /// Stop the sensor, drop its power, and clear its stale state.
    Release,
}

/// Requests that the Room Scan screen's sensor lifecycle runs.
static ROOM_SCAN_REQUEST: Channel<CriticalSectionRawMutex, RoomScanRequest, 4> = Channel::new();

// ── Timing ────────────────────────────────────────────────────────────────────

/// Poll interval, in milliseconds, while the boot gate waits for initialisation.
const BOOT_POLL_MS: u64 = 100;

/// Idle tick period, in milliseconds.
///
/// Battery and calibration state move slowly, and a running screen's phase and
/// progress advance by hand or by stage; 250 ms keeps both current without
/// reading the state locks or flushing a full frame more often than either can
/// change.
const TICK_INTERVAL_MS: u64 = 250;

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

// ── Initialisation ──────────────────────────────────────────────────────────────

/// Build the panel, spawn the UI controller task, and spawn the helper tasks it
/// delegates to.
///
/// The controller owns the panel's display, touch controller, and framebuffer
/// for the lifetime of the firmware. The helper tasks own the procedures that
/// cannot run on the UI task without freezing the panel: the distance
/// calibration's fixed-distance drive, and the coast-and-avoid mode's `LiDAR`
/// acquisition, which may take several seconds.
#[allow(clippy::unwrap_used)]
pub fn init_ui(spawner: Spawner, pins: PanelPins) {
    spawner.spawn(ui_task(Panel::new(pins)).unwrap());
    spawner.spawn(distance_calibration_drive_task().unwrap());
    spawner.spawn(coast_avoid_task().unwrap());
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

/// Run the distance calibration's fixed-distance drive step and report back.
///
/// The step is one-shot per request: the UI asks for it when the operator opens
/// the Distance entry, and the reply arrives as a [`UiEvent`] so the UI task is
/// never blocked while the robot drives.
#[embassy_executor::task]
async fn distance_calibration_drive_task() {
    loop {
        DISTANCE_DRIVE_REQUEST.receive().await;
        if drive_calibration::distance::run_drive_step().await != DriveOutcome::Stopped {
            send_ui_event(UiEvent::DistanceDriveFinished).await;
        }
    }
}

/// Start coast-and-avoid when the panel asks, off the UI task.
///
/// Acquiring the `LiDAR` can take several seconds, so the UI task must not await
/// it: it dispatches a request and polls the activity state and the sensor's
/// lock-free status while this task runs the acquisition and start. On failure
/// the mode records the reason, which the running screen shows, and never drives.
#[embassy_executor::task]
async fn coast_avoid_task() {
    loop {
        COAST_AVOID_REQUEST.receive().await;
        if let Err(error) = coast_obstacle_avoid::start().await {
            warn!("[ui] coast-and-avoid start failed: {}", error.label());
            activity::fail(error.label()).await;
        }
    }
}

/// Run the Room Scan screen's `LiDAR` lease, off the UI task.
///
/// Acquiring can take several seconds, so the UI task dispatches a request and
/// polls the sensor's lock-free status and the perception snapshot while this
/// task owns the lifecycle. The lease is held across iterations and handed back
/// only on release, so leaving the screen after a failed acquisition releases
/// nothing and cannot power down a sensor another mode owns. Requests are
/// handled in order: a release queued while an acquire is still running waits
/// for it.
#[embassy_executor::task]
async fn room_scan_task() {
    let mut lease: Option<lidar::Lease> = None;
    loop {
        match ROOM_SCAN_REQUEST.receive().await {
            RoomScanRequest::Acquire => {
                if lease.is_none() {
                    match lidar::acquire().await {
                        Ok(acquired) => lease = Some(acquired),
                        Err(lidar::AcquireError::Busy) => {
                            warn!("[ui] room scan: LiDAR acquisition already in flight");
                        }
                        Err(lidar::AcquireError::Failed) => {
                            warn!("[ui] room scan: LiDAR acquisition failed");
                        }
                    }
                }
            }
            RoomScanRequest::Release => {
                // A no-op when no lease is held — the whole point of the lease:
                // leaving after a failed acquisition releases nothing.
                let held = lease.take();
                if let Some(lease) = held {
                    lidar::release(lease).await;
                }
            }
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
    let mut ticker = Ticker::every(Duration::from_millis(TICK_INTERVAL_MS));

    loop {
        if !panel.bring_up().await {
            warn!("panel offline; retrying initialisation");
            Timer::after(Duration::from_millis(REINIT_BACKOFF_MS)).await;
            continue;
        }
        if run_ui(&mut panel, &mut ui, &mut ticker).await.is_err() {
            warn!("panel went offline; re-initialising");
        }
        Timer::after(Duration::from_millis(REINIT_BACKOFF_MS)).await;
    }
}

/// Drive the model while the panel is online, until a draw or flush fails.
///
/// Returns `Err(())` when the panel went offline, so the caller can re-run
/// bring-up.
async fn run_ui(panel: &mut Panel, ui: &mut Ui, ticker: &mut Ticker) -> Result<(), ()> {
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
                if refresh_screens(ui).await {
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

/// The screen a just-finished Procedure lands on, read from its identity.
///
/// The Activity State still names the Procedure when this is called, so the
/// landing screen comes from the Menu Entry that started it rather than from a
/// literal. A completion with no recorded Procedure lands on the Main Menu.
async fn finished_landing() -> Screen {
    activity::snapshot()
        .await
        .procedure()
        .map_or(Screen::MainMenu, Procedure::landing_screen)
}

/// Apply a lifecycle event to the model.
async fn handle_event(ui: &mut Ui, event: UiEvent) {
    match event {
        UiEvent::ShowMainMenu => *ui = Ui::new(),
        UiEvent::TestingCompleted => {
            let landing = finished_landing().await;
            activity::clear().await;
            *ui = Ui::new();
            ui.show_screen(landing);
        }
        UiEvent::CalibrationCompleted => hold_or_leave_calibration(ui).await,
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
        (Screen::RoomScan, Some(Hit::Back)) => leave_room_scan(),
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
        (Screen::Status, Some(Hit::Stop)) => stop_running().await,
        _ => {}
    }
}

/// Act on the menu entry the model activated.
///
/// The match is exhaustive over the crate's [`Item`], so adding an entry — or
/// moving one to another screen — is a compile-time question rather than a
/// silent change of which procedure a tap starts.
async fn handle_menu_item(ui: &mut Ui, item: Item) {
    match item {
        Item::ScreenEntry(ScreenEntry::RoomScan) => enter_room_scan(ui),
        Item::Procedure(procedure) => match procedure {
            Procedure::BasicMotor
            | Procedure::Turns
            | Procedure::StraightDrive
            | Procedure::ArcDrive
            | Procedure::Imu6Axis
            | Procedure::Imu9Axis => start_test(ui, procedure).await,
            Procedure::MotorCalibration | Procedure::MagCalibration | Procedure::DistanceCalibration => {
                start_calibration(ui, procedure).await;
            }
            Procedure::CoastAndAvoid => start_coast_and_avoid(ui).await,
            // The attempt-straight value only feeds the deferred mode: nothing
            // starts here.
            Procedure::AttemptStraight => {}
        },
        // The entries the model opens on its own: nothing starts here.
        Item::Submenu(_) | Item::ScreenEntry(ScreenEntry::SystemInfo) => {}
    }
}

/// Start the test-mode `procedure`.
///
/// The producer records the identity in the activity state, so the running screen
/// names it from the entry rather than from a parallel kind enumeration.
async fn start_test(ui: &mut Ui, procedure: Procedure) {
    testmode::start(procedure).await;

    let snapshot = activity::snapshot().await;
    ui.show_status(status_view(&snapshot));
}

/// Enter the Room Scan screen: claim the single-active test-mode slot and start
/// the sensor's acquisition.
///
/// The model has already navigated to the radar screen. If another test owns the
/// slot the entry is refused and the screen leaves again, so the radar never
/// opens alongside a running test. Acquisition runs on [`room_scan_task`], so
/// this returns immediately and the screen shows the sensor warming.
fn enter_room_scan(ui: &mut Ui) {
    if !testmode::claim_room_scan() {
        warn!("[ui] room scan refused — another test is active");
        let _ = ui.back();
        return;
    }
    ROOM_SCAN_REQUEST.try_send(RoomScanRequest::Acquire).ok();
}

/// Leave the Room Scan screen: release the single-active slot and the `LiDAR`
/// lease it holds.
///
/// The release is dispatched rather than awaited, so the panel never blocks; the
/// lifecycle task handles it after any in-flight acquisition finishes. Called on
/// every exit path, including a failed acquisition, it hands back only the lease
/// this screen actually holds, so a failed acquisition releases nothing and
/// cannot power down a sensor another mode owns.
fn leave_room_scan() {
    testmode::release_room_scan();
    ROOM_SCAN_REQUEST.try_send(RoomScanRequest::Release).ok();
}

/// Start the calibration `procedure`.
///
/// The identity maps to the concrete calibration flow, and the running view takes
/// its title and parent from that identity. Every other Procedure belongs to
/// another screen and is not a calibration, so it starts nothing.
async fn start_calibration(ui: &mut Ui, procedure: Procedure) {
    match procedure {
        Procedure::MotorCalibration => {
            drive::send_drive_command(DriveCommand::RunMotorCalibration).await;
            ui.show_status(StatusView::for_procedure(procedure, "Starting"));
        }
        Procedure::MagCalibration => {
            drive::send_drive_command(DriveCommand::RunImuCalibration(ImuCalibrationKind::Mag)).await;
            ui.show_status(StatusView::for_procedure(procedure, "Starting"));
        }
        Procedure::DistanceCalibration => start_distance_calibration(ui).await,
        Procedure::CoastAndAvoid
        | Procedure::AttemptStraight
        | Procedure::BasicMotor
        | Procedure::Turns
        | Procedure::StraightDrive
        | Procedure::ArcDrive
        | Procedure::Imu6Axis
        | Procedure::Imu9Axis => {}
    }
}

/// Start coast-and-avoid from the panel without blocking on the acquisition.
///
/// The activity is recorded first so the running screen appears immediately and
/// names the mode; the [`coast_avoid_task`] helper then acquires the `LiDAR`,
/// which the running screen tracks through [`lidar::status`]. If the sensor
/// cannot be brought up, the helper records the failure reason and the screen
/// holds it until the operator dismisses it back to the Drive Mode menu.
async fn start_coast_and_avoid(ui: &mut Ui) {
    activity::begin(Activity::Procedure(Procedure::CoastAndAvoid), "Acquiring LiDAR").await;
    let snapshot = activity::snapshot().await;
    ui.show_status(status_view(&snapshot));
    COAST_AVOID_REQUEST.try_send(()).ok();
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

/// Back up the distance factor and start the calibration's drive step.
///
/// The step runs on its own task and reports back through the activity state and
/// a [`UiEvent`], so the panel stays live while the robot drives.
async fn start_distance_calibration(ui: &mut Ui) {
    ui.show_status(StatusView::for_procedure(Procedure::DistanceCalibration, "Starting"));

    drive_calibration::distance::begin().await;
    DISTANCE_DRIVE_REQUEST.send(()).await;
}

/// Commit the factor the operator measured, and forget the calibration.
async fn save_distance_calibration(measured_cm: i32) {
    let factor = drive_calibration::distance::commit(measured_cm).await;
    debug!("[ui] distance calibration saved: factor {}", factor);
    activity::clear().await;
}

/// Restore the factor in force before the calibration, and forget it.
async fn cancel_distance_calibration() {
    drive_calibration::distance::abort().await;
    activity::clear().await;
}

/// Stop whatever the running screen is showing, and forget it.
///
/// Every producer latch is set, plus the drive interrupt that cancels a leg in
/// flight. The panel cannot tell which producer is mid-start — a procedure
/// requested microseconds ago may not have published itself yet — and a latch the
/// wrong producer holds is cleared by that producer's own re-arm when its next run
/// starts, so setting them all is safe and leaves no running procedure
/// unstoppable. Stopping coast-and-avoid clears its active flag; its task then
/// brakes, releases the `LiDAR`, and exits on its own. The activity state is
/// cleared here because the panel is what moved on.
async fn stop_running() {
    testmode::stop();
    drive_calibration::request_stop();
    coast_obstacle_avoid::stop();
    activity::clear().await;
}

/// React to a finished calibration: leave a completed one, hold a failed one's
/// reason on screen until the operator dismisses it.
async fn hold_or_leave_calibration(ui: &mut Ui) {
    let snapshot = activity::snapshot().await;

    match snapshot.activity {
        Activity::Procedure(procedure) if procedure.is_calibration() => {
            if snapshot.stage == Stage::Failed {
                ui.set_status(status_view(&snapshot));
            } else {
                let landing = procedure.landing_screen();
                activity::clear().await;
                *ui = Ui::new();
                ui.show_screen(landing);
            }
        }
        Activity::Procedure(_) | Activity::Booting | Activity::Idle => {
            // A stopped or already-dismissed calibration: the panel has moved on.
        }
    }
}

/// Show the distance value screen once the drive step has finished, or hold the
/// step's failure reason on screen.
async fn enter_distance_entry(ui: &mut Ui) {
    let snapshot = activity::snapshot().await;

    if snapshot.activity == Activity::Idle {
        // The operator stopped the step; the panel already left its screen.
        return;
    }

    if snapshot.stage == Stage::Failed {
        ui.set_status(status_view(&snapshot));
    } else {
        ui.show_value_entry(ValueFlow::DistanceCalibration);
    }
}

// ── Running screen ─────────────────────────────────────────────────────────────

/// The running screen's content for a snapshot.
///
/// A drive mode's body line is the `LiDAR`'s lifecycle state, read lock-free on
/// each tick so the screen reports the sensor warming up while the acquisition
/// runs, then streaming once it settles, without the controller formatting text
/// of its own.
fn status_view(snapshot: &Snapshot) -> StatusView {
    // A running Procedure names itself: its title and where Stop returns both come
    // from the Menu Entry that started it. The boot flow and the idle state have no
    // entry, so they fall back to their own title and the Main Menu.
    let body = status_body(snapshot);
    let view = snapshot.procedure().map_or_else(
        || StatusView::new(snapshot.title(), body, Screen::MainMenu),
        |procedure| StatusView::for_procedure(procedure, body),
    );
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
/// failed, the reason the producer recorded (the [`coast_obstacle_avoid::StartError`]
/// label) takes over the line.
fn status_body(snapshot: &Snapshot) -> &'static str {
    match snapshot.activity {
        Activity::Procedure(Procedure::CoastAndAvoid) if snapshot.stage != Stage::Failed => {
            map_sensor_state(lidar::status()).label()
        }
        Activity::Idle | Activity::Booting | Activity::Procedure(_) => snapshot.detail,
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

/// Refresh the tick's screens: the running screen from the activity state, the
/// System Info snapshot while that screen is shown, and the Room Scan radar and
/// sensor caption while the radar screen is shown.
///
/// Reports whether anything on screen changed.
async fn refresh_screens(ui: &mut Ui) -> bool {
    let mut redraw = refresh_running_screen(ui).await;

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
/// exists — before the sensor warms or after it is released — which the screen
/// draws as "No data", never as an empty room. The cloud crosses owned with its
/// sequence as the change token, so an unchanged sequence costs one counter
/// comparison and no copy of the slot array (ADR-0016). The caption comes from
/// the sensor's lock-free lifecycle status.
async fn refresh_room_scan(ui: &mut Ui) -> bool {
    let mut redraw = ui.set_sensor_state(map_sensor_state(lidar::status()));
    redraw |= perception::with_lidar(|cloud| ui.set_radar(cloud.map(|c| (c.slots(), c.sequence())))).await;
    redraw
}

/// Re-render the running screen from the activity state, reporting a change.
///
/// A no-op off the running screen, and while nothing is recorded — a producer
/// that has not published its first phase yet leaves the screen as it was.
async fn refresh_running_screen(ui: &mut Ui) -> bool {
    if ui.screen() != Screen::Status {
        return false;
    }

    let snapshot = activity::snapshot().await;
    if snapshot.activity == Activity::Idle {
        return false;
    }

    ui.set_status(status_view(&snapshot))
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
