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
    CalibrationStatus as UiCalibrationStatus, DRAG_RENDER_MS, Hit, Screen, SensorState, StatusView, SystemInfo,
    TAP_MIN_DURATION_MS, TICK_MS, Ui, ValueFlow, geometry,
};

use crate::{
    system::state::{
        CalibrationStatus,
        activity::{self, Activity, CalibrationKind, DriveModeKind, Snapshot, Stage, TestKind},
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
    /// Request to show the Test Mode submenu.
    ///
    /// Kept for the same reason as [`show_test_menu`]: the request path stays
    /// compiling whether or not a caller uses it.
    #[allow(dead_code)]
    ShowTestMenu,
    /// Testing sequence finished — show the main menu.
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

/// Backoff before retrying an offline panel, in milliseconds.
const REINIT_BACKOFF_MS: u64 = 2_000;

/// Index of the Calibrate entry on the Main Menu (`touch_ui::screens::MAIN_MENU`).
const CALIBRATE_INDEX: usize = 1;

/// Index of the Coast & Avoid entry on the Drive Mode menu
/// (`touch_ui::screens::DRIVE_MODE_MENU`).
const COAST_AVOID_INDEX: usize = 0;

/// Index of the Test Mode entry on the Main Menu.
const TEST_MODE_INDEX: usize = 3;

/// Index of the Room Scan entry on the Test Mode menu
/// (`touch_ui::screens::TEST_MENU`).
const ROOM_SCAN_INDEX: usize = 6;

/// The test-mode leaves, in `touch_ui::screens::TEST_MENU` order.
const TEST_KINDS: [TestKind; 6] = [
    TestKind::BasicMotor,
    TestKind::Turns,
    TestKind::StraightDrive,
    TestKind::ArcDrive,
    TestKind::Imu6Axis,
    TestKind::Imu9Axis,
];

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

/// Request that the UI show the Test Mode submenu.
///
/// Part of the controller's published surface alongside [`show_main_menu`]: the
/// panel's own Stop paths reach the test menu through the model's navigation, so
/// nothing in the firmware calls this yet.
#[allow(dead_code)]
pub async fn show_test_menu() {
    send_ui_event(UiEvent::ShowTestMenu).await;
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

/// Run the Room Scan screen's `LiDAR` lifecycle, off the UI task.
///
/// Acquiring can take several seconds, so the UI task dispatches a request and
/// polls the sensor's lock-free status and the perception snapshot while this
/// task owns the lifecycle. Requests are handled in order: a release queued
/// while an acquire is still running waits for it, so leaving the screen always
/// powers the sensor back down.
#[embassy_executor::task]
async fn room_scan_task() {
    loop {
        match ROOM_SCAN_REQUEST.receive().await {
            RoomScanRequest::Acquire => {
                if lidar::acquire().await.is_err() {
                    warn!("[ui] room scan: LiDAR acquire failed");
                }
            }
            RoomScanRequest::Release => lidar::release().await,
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
async fn run_gesture(
    panel: &mut Panel,
    ui: &mut Ui,
    started_ms: u64,
    screen_before: Screen,
    value_before: i32,
) -> Result<(), ()> {
    let mut last_drag_render = started_ms;
    loop {
        Timer::after(Duration::from_millis(TICK_MS)).await;
        let now_ms = Instant::now().as_millis();
        match panel.read_touch().await {
            Ok(Some(point)) => {
                if ui.pointer_move(point, now_ms) && now_ms.saturating_sub(last_drag_render) >= DRAG_RENDER_MS {
                    render_and_flush(panel, ui).await?;
                    last_drag_render = now_ms;
                }
            }
            Ok(None) | Err(()) => {
                if panel.pen_up() {
                    let _ = ui.pointer_up(now_ms);
                    let activated = ui.last_activation();
                    handle_activation(ui, screen_before, value_before, activated).await;
                    refresh_system_info_on_entry(ui).await;
                    render_and_flush(panel, ui).await?;
                    return Ok(());
                }
            }
        }
    }
}

/// Render the current screen into the framebuffer and flush it.
async fn render_and_flush(panel: &mut Panel, ui: &Ui) -> Result<(), ()> {
    ui.render(panel.display_mut()).map_err(|_| ())?;
    panel.flush().await
}

/// Apply a lifecycle event to the model.
async fn handle_event(ui: &mut Ui, event: UiEvent) {
    match event {
        UiEvent::ShowMainMenu => *ui = Ui::new(),
        UiEvent::TestingCompleted => {
            activity::clear().await;
            *ui = Ui::new();
        }
        UiEvent::ShowTestMenu => {
            *ui = Ui::new();
            // The model has no "open this screen" call in its public surface, so
            // reuse the tap path and the shared geometry to land on Test Mode.
            tap_main_menu_item(ui, TEST_MODE_INDEX);
        }
        UiEvent::CalibrationCompleted => hold_or_leave_calibration(ui).await,
        UiEvent::DistanceDriveFinished => enter_distance_entry(ui).await,
    }
}

/// Open a Main Menu entry by feeding the model a synthetic tap at its centre.
fn tap_main_menu_item(ui: &mut Ui, index: usize) {
    let target = geometry::menu_item_rect(index, 0).center();
    let now_ms = Instant::now().as_millis();
    let _ = ui.pointer_down(target, now_ms);
    let _ = ui.pointer_up(now_ms + TAP_MIN_DURATION_MS);
}

// ── Activation handling ─────────────────────────────────────────────────────────

/// Act on the region the model activated, given the screen and value the press
/// began on.
///
/// The model has already performed its own navigation (opening a submenu, going
/// back from a value screen), so this only starts, stops or saves things, and
/// replaces the screen when a running procedure takes over from a placeholder.
async fn handle_activation(ui: &mut Ui, screen_before: Screen, value_before: i32, activated: Option<Hit>) {
    match (screen_before, activated) {
        (Screen::TestMode, Some(Hit::MenuItem(ROOM_SCAN_INDEX))) => enter_room_scan(ui),
        (Screen::TestMode, Some(Hit::MenuItem(index))) => start_test(ui, index).await,
        (Screen::RoomScan, Some(Hit::Back)) => leave_room_scan(),
        (Screen::Calibrate, Some(Hit::MenuItem(index))) => start_calibration(ui, index).await,
        (Screen::DriveMode, Some(Hit::MenuItem(COAST_AVOID_INDEX))) => start_coast_and_avoid(ui).await,
        (Screen::ValueEntry(ValueFlow::DistanceCalibration), Some(Hit::Save)) => {
            save_distance_calibration(value_before).await;
        }
        (Screen::ValueEntry(ValueFlow::DistanceCalibration), Some(Hit::Cancel)) => {
            cancel_distance_calibration().await;
        }
        (Screen::ValueEntry(ValueFlow::AttemptStraight), Some(Hit::Save)) => {
            record_attempt_straight(value_before).await;
        }
        (Screen::Status, Some(Hit::Stop | Hit::Back)) => stop_running().await,
        _ => {}
    }
}

/// Start the test-mode procedure behind Test Mode entry `index`.
async fn start_test(ui: &mut Ui, index: usize) {
    let Some(kind) = TEST_KINDS.get(index).copied() else {
        return;
    };

    testmode::start(kind).await;

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

/// Leave the Room Scan screen: release the single-active slot and the sensor.
///
/// The release is dispatched rather than awaited, so the panel never blocks; the
/// lifecycle task handles it after any in-flight acquisition finishes. Called on
/// every exit path, including a failed acquisition, so the sensor is never left
/// powered on behind the menu.
fn leave_room_scan() {
    testmode::release_room_scan();
    ROOM_SCAN_REQUEST.try_send(RoomScanRequest::Release).ok();
}

/// Start the calibration procedure behind Calibrate entry `index`.
async fn start_calibration(ui: &mut Ui, index: usize) {
    match index {
        0 => {
            drive::send_drive_command(DriveCommand::RunMotorCalibration).await;
            ui.show_status(StatusView::new(
                CalibrationKind::Motor.title(),
                "Starting",
                Screen::Calibrate,
            ));
        }
        1 => {
            drive::send_drive_command(DriveCommand::RunImuCalibration(ImuCalibrationKind::Mag)).await;
            ui.show_status(StatusView::new(
                CalibrationKind::Mag.title(),
                "Starting",
                Screen::Calibrate,
            ));
        }
        2 => start_distance_calibration(ui).await,
        _ => {}
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
    activity::begin(
        Activity::DriveMode(DriveModeKind::CoastAndAvoid),
        "Acquiring LiDAR",
        true,
    )
    .await;
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
    ui.show_status(StatusView::new(
        CalibrationKind::Distance.title(),
        "Starting",
        Screen::Calibrate,
    ));

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
        Activity::Calibration(_) if snapshot.stage == Stage::Failed => {
            ui.set_status(status_view(&snapshot));
        }
        Activity::Calibration(_) => {
            activity::clear().await;
            *ui = Ui::new();
            tap_main_menu_item(ui, CALIBRATE_INDEX);
        }
        Activity::Booting | Activity::Test(_) | Activity::Idle | Activity::DriveMode(_) => {
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
    let view = StatusView::new(
        snapshot.title(),
        status_body(snapshot),
        status_parent(snapshot.activity),
    );
    snapshot.percent.map_or(view, |percent| view.with_progress(percent))
}

/// The body line a running screen shows for a snapshot.
///
/// A drive mode reports the sensor's state while it runs; once its start has
/// failed, the reason the producer recorded (the [`coast_obstacle_avoid::StartError`]
/// label) takes over the line.
fn status_body(snapshot: &Snapshot) -> &'static str {
    match snapshot.activity {
        Activity::DriveMode(_) if snapshot.stage != Stage::Failed => sensor_label(lidar::status()),
        Activity::Booting | Activity::Test(_) | Activity::Calibration(_) | Activity::Idle | Activity::DriveMode(_) => {
            snapshot.detail
        }
    }
}

/// The body line naming a sensor's lifecycle state.
const fn sensor_label(status: LidarStatus) -> &'static str {
    map_sensor_state(status).label()
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

/// The screen a running screen's Stop returns to.
///
/// A run-to-completion test ends on the main menu, and its Stop goes there too. An
/// interactive test runs until stopped, so Stop returns to the test menu it was
/// opened from; a drive mode returns to the Drive Mode menu, and every calibration
/// to the Calibrate submenu.
const fn status_parent(activity: Activity) -> Screen {
    match activity {
        Activity::Test(kind) if kind.is_interactive() => Screen::TestMode,
        Activity::Calibration(_) => Screen::Calibrate,
        Activity::DriveMode(_) => Screen::DriveMode,
        Activity::Booting | Activity::Test(_) | Activity::Idle => Screen::MainMenu,
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
/// draws as "No data", never as an empty room. The caption comes from the
/// sensor's lock-free lifecycle status.
async fn refresh_room_scan(ui: &mut Ui) -> bool {
    let mut redraw = ui.set_sensor_state(map_sensor_state(lidar::status()));
    let slots = perception::get_lidar_snapshot().await.map(|cloud| *cloud.slots());
    redraw |= ui.set_radar(slots);
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
