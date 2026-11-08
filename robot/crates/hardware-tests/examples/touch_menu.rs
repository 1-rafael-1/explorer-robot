//! Touch-driven mock menu for the 2.8″ ST7789 panel — a bench feeler.
//!
//! Brings up the panel's Display and Touch Panel on one shared full-duplex SPI0
//! bus — the same wiring and per-device configuration as `touch_coexistence` —
//! and draws the robot's menus as columns of wide buttons under a titled header.
//! A caller-side 5-sample moving-median filter smooths the raw `x`/`y` counts
//! before this example's `CALIBRATION` maps them to a pixel; taps, drags, and
//! slider positions are hit-tested against the on-screen regions and logged.
//!
//! The whole flow is navigable by touch alone: the Main Menu opens the Calibrate,
//! Drive Mode, and Test Mode submenus and the mocked System Info screen; leaf
//! entries open non-navigable placeholders naming the action they would run; the
//! two distance flows open a value-entry screen with a live readout, a draggable
//! slider, fine-adjust `-`/`+`, a header Cancel, and a footer Save. Long lists
//! (Test Mode) drag-scroll with a right-edge indicator. Headers carry Back, or
//! Cancel on the value screens.
//!
//! No hardware action is performed: placeholders and System Info are mocks, and
//! Cancel/Save only return to the parent. This example is bench-only and does not
//! touch the robot firmware or its dedicated display bus.
//!
//! Rendering is full-frame (clear → redraw → flush), immediate on pen-down, on
//! release, and for slider/`-`/`+` changes, and throttled during movement so a
//! drag stays responsive.
//!
//! The menu labels and System Info rows are a **transcribed snapshot** of the
//! `touch-ui` crate's `screens` module — the source of truth for the labels the
//! UI ships — and may drift from it.
//!
//! Run from the repository root with:
//!
//! ```sh
//! cargo run -p hardware-tests --example touch_menu --release
//! ```
//!
//! This is an embassy binary validated by compilation plus manual bench
//! observation; it has no automated tests (the `touch_coexistence` example is the
//! prior art). The `TICK_MS`, `TAP_MAX_MOVE`, `TAP_MIN_DURATION_MS`, and
//! `DRAG_RENDER_MS` constants are the ones to re-tune on glass.

#![no_std]
#![no_main]
// Demo code: `.unwrap()` on driver/GPIO/draw results is intentional, and the
// `main` future is large because it holds both device drivers and the shared bus.
#![allow(
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::cast_precision_loss,
    clippy::used_underscore_binding,
    clippy::large_futures
)]

use core::fmt::Write as _;

use defmt::info;
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH6, DMA_CH7, SPI0},
    spi::{self, Spi},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{
        MonoFont, MonoTextStyle,
        ascii::{FONT_6X10, FONT_9X15, FONT_9X15_BOLD, FONT_10X20},
    },
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text, renderer::TextRenderer},
};
use moving_median::MovingMedian;
use panic_probe as _;
use st7789_async::{ColorOrder, Config as DisplayConfig, Orientation, Rotation, St7789};
use static_cell::{ConstStaticCell, StaticCell};
use touch_async::{Calibration, TouchPanel, TouchSample};

// Full-duplex SPI0 needs one DMA channel per direction: `DMA_IRQ_0` is bound to
// both the TX (`DMA_CH6`) and RX (`DMA_CH7`) handlers.
bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH6>, embassy_rp::dma::InterruptHandler<DMA_CH7>;
});

// ── Bus and framebuffer ───────────────────────────────────────────────────────

/// Display SPI clock frequency in Hz (64 MHz): fast enough to flush a full frame.
const DISPLAY_FREQ: u32 = 64_000_000;

/// Touch SPI clock frequency in Hz (200 kHz): the low speed the XPT2046-class
/// controller is specified for.
const TOUCH_FREQ: u32 = 200_000;

/// Panel hardware-reset low pulse, in milliseconds.
const PANEL_RESET_LOW_MS: u64 = 10;

/// Panel settle after the reset is released, in milliseconds.
const PANEL_RESET_HIGH_MS: u64 = 120;

/// Framebuffer width in pixels (landscape after the orientation transform).
const FB_W: usize = 320;
/// Framebuffer height in pixels.
const FB_H: usize = 240;

/// Length of the moving-median window, in samples, for the raw `x`/`y` counts.
const MEDIAN_WINDOW: usize = 5;

// ── Panel calibration ─────────────────────────────────────────────────────────

/// Touch calibration for this example's readable landscape orientation.
///
/// This example displays with `Orientation::Deg90` alone, the same readable
/// landscape orientation as `lidar_tft_radar`. [`Calibration::MEASURED`] was
/// measured in `touch_coexistence`'s orientation, which is that orientation
/// plus a horizontal mirror; a horizontally mirrored display flips the raw X
/// axis, so [`Calibration::mirrored_x`] swaps the X endpoints and leaves the Y
/// endpoints unchanged, deriving from `MEASURED` so a re-measurement there flows
/// through. Using `MEASURED` directly with this orientation mirrors touch
/// horizontally: the header Back, the value screen's `-`/`+` and Cancel, and the
/// slider all respond as if the panel were flipped.
const CALIBRATION: Calibration = Calibration::MEASURED.mirrored_x();

/// The caller-allocated framebuffer (big-endian RGB565 bytes).
type Fb = [u8; FB_W * FB_H * 2];

/// Statically-allocated framebuffer (`153_600` bytes of zeroed `.bss`).
static FB: ConstStaticCell<Fb> = ConstStaticCell::new([0; FB_W * FB_H * 2]);

/// Mutable SPI bus shared by the display and the touch controller.
type SpiBus = Mutex<NoopRawMutex, Spi<'static, SPI0, spi::Async>>;

/// Statically-allocated home for the shared SPI bus.
static SPI_BUS: StaticCell<SpiBus> = StaticCell::new();

/// Build the display's bus configuration: Mode 3 at [`DISPLAY_FREQ`].
///
/// The ST7789 and the touch controller share Mode 3 and differ only in clock
/// speed, so the per-device [`SpiDeviceWithConfig`] reconfigures the bus on each
/// transaction.
fn display_spi_config() -> spi::Config {
    let mut config = spi::Config::default();
    config.frequency = DISPLAY_FREQ;
    config.phase = spi::Phase::CaptureOnSecondTransition;
    config.polarity = spi::Polarity::IdleHigh;
    config
}

/// Build the touch controller's bus configuration: Mode 3 at [`TOUCH_FREQ`].
fn touch_spi_config() -> spi::Config {
    let mut config = spi::Config::default();
    config.frequency = TOUCH_FREQ;
    config.phase = spi::Phase::CaptureOnSecondTransition;
    config.polarity = spi::Polarity::IdleHigh;
    config
}

// ── Theme ─────────────────────────────────────────────────────────────────────
//
// Every colour, font, and layout number lives here, so the whole look can be
// restyled on glass without touching layout or interaction code. Colours are
// built with the panel's RGB subpixel order.

// Palette

/// Screen background: a near-black blue-grey.
const BG: Rgb565 = Rgb565::new(2, 4, 6);

/// Unpressed button fill: a dark slate that reads as a raised surface.
const BUTTON_BG: Rgb565 = Rgb565::new(8, 12, 16);

/// Button outline: a subtle step up from [`BUTTON_BG`].
const BUTTON_BORDER: Rgb565 = Rgb565::new(18, 24, 30);

/// Primary text (titles and labels): near-white.
const TEXT: Rgb565 = Rgb565::new(30, 60, 30);

/// Muted chrome (the header divider): a mid grey, dimmer than [`TEXT`].
const MUTED: Rgb565 = Rgb565::new(14, 28, 14);

/// Accent used for the pressed/selected state: a saturated teal.
const ACCENT: Rgb565 = Rgb565::new(3, 40, 31);

/// Warning/amber accent, reserved for non-navigable or error states (the
/// placeholder panel border).
const WARN: Rgb565 = Rgb565::new(31, 44, 0);

// Typography

/// Bold font used for the header title.
const TITLE_FONT: &MonoFont<'static> = &FONT_9X15_BOLD;

/// Regular font used for button labels.
const LABEL_FONT: &MonoFont<'static> = &FONT_9X15;

/// Small font used for the System Info rows and placeholder body text.
const SMALL_FONT: &MonoFont<'static> = &FONT_6X10;

/// Large font used for the value-entry readout.
const READOUT_FONT: &MonoFont<'static> = &FONT_10X20;

// Geometry

/// Height of the title header at the top of the screen, in pixels.
///
/// Deliberately tall: it holds a comfortably sized Back/Cancel target below the
/// panel's bezel-adjacent top edge (the region the calibration targets avoided).
const HEADER_H: u32 = 48;

/// Height of one list button, in pixels.
const BUTTON_H: u32 = 38;

/// Vertical gap between adjacent buttons, in pixels (also the gap below the header).
const BUTTON_GAP: u32 = 8;

/// Horizontal inset of the button column from the screen edges, in pixels.
///
/// Wide enough to leave a clear gutter for the 25 px scroll indicator on the
/// right.
const SIDE_MARGIN: u32 = 32;

/// Inner horizontal padding between a button's edge and its label, in pixels.
const BUTTON_PAD: u32 = 12;

/// Stroke width of a button's outline, in pixels.
const BUTTON_BORDER_W: u32 = 1;

/// Height of the header divider line, in pixels.
const DIVIDER_H: u32 = 1;

/// Width of the header Back button, in pixels.
const BACK_W: u32 = 84;

/// Vertical inset of the header action button from the header's top and bottom,
/// in pixels.
///
/// Kept at the same 12 px inset the calibration targets use, so the button sits
/// clear of the panel's bezel-adjacent top edge instead of in it.
const BACK_MARGIN: u32 = 12;

/// Horizontal gap between the header action button and the centered title, in
/// pixels.
const HEADER_ACTION_GAP: u32 = 8;

/// Inset of the placeholder panel from the screen edges, in pixels.
const PANEL_MARGIN: u32 = 24;

/// Stroke width of the placeholder panel's border, in pixels.
const PANEL_BORDER: u32 = 2;

/// Height of one System Info row, in pixels.
const INFO_ROW_H: u32 = 26;

/// Width of the scroll-indicator track, in pixels.
const SCROLL_W: u32 = 25;

/// Right-margin inset of the scroll indicator from the screen edge, in pixels.
const SCROLL_MARGIN: u32 = 4;

/// Minimum height of the scroll indicator's thumb, in pixels.
const SCROLL_MIN_THUMB_H: u32 = 20;

/// Width of the header Cancel button on value screens, in pixels.
const CANCEL_W: u32 = 96;

/// Height of the value-entry readout area, in pixels.
const READOUT_H: u32 = 56;

/// Horizontal inset of the slider from the screen edges, in pixels.
const SLIDER_MARGIN: u32 = 32;

/// Thickness of the slider track, in pixels.
const SLIDER_H: u32 = 10;

/// Vertical offset of the slider track from the top of the screen, in pixels.
const SLIDER_Y: u32 = 104;

/// Width of the slider thumb, in pixels.
const SLIDER_THUMB_W: u32 = 16;

/// Height of the slider thumb and its touch band, in pixels.
const SLIDER_THUMB_H: u32 = 28;

/// Width of a fine-adjust button, in pixels.
const NUDGE_W: u32 = 64;

/// Height of a fine-adjust button, in pixels.
const NUDGE_H: u32 = 44;

/// Vertical offset of the fine-adjust row from the top of the screen, in pixels.
const NUDGE_Y: u32 = 132;

/// Height of the footer at the bottom of the value screen, in pixels.
const FOOTER_H: u32 = 36;

/// Vertical inset of the footer Save button from the footer's top and bottom, in
/// pixels.
const FOOTER_PAD: u32 = 6;

/// Width of the footer Save button, in pixels.
const SAVE_W: u32 = 96;

// ── Interaction tuning (tunable) ──────────────────────────────────────────────
//
// These shape the gesture feel on glass and are the first things to re-tune. They
// are chosen directly, not derived from one another.

/// Poll period, in milliseconds, while the pen is down.
///
/// 20 ms (50 Hz) tracks a finger closely without spending more time on SPI reads
/// than the panel needs.
const TICK_MS: u64 = 20;

/// Maximum total pointer movement, in pixels, that still counts as a tap.
///
/// A fingertip rolls several pixels when it presses, and the first panel sample
/// after the pen-down edge can be noisier still; 20 px absorbs that. It stays well
/// below the 38 px button height, and a release must also land on the same region
/// the press began on, so an intended tap cannot activate a neighbour across the
/// 8 px gap.
const TAP_MAX_MOVE: u32 = 20;

/// Minimum press duration, in milliseconds, for a release to count as a tap.
///
/// Set just below [`TICK_MS`] so it rejects contact bounce without rejecting a
/// quick deliberate tap: the poll loop can only observe a release on a tick, so a
/// floor above the tick period would drop fast taps.
const TAP_MIN_DURATION_MS: u64 = 20;

/// Minimum interval, in milliseconds, between drag redraws.
///
/// A full-frame flush at 64 MHz costs a few milliseconds, so 50 ms (20 fps) keeps
/// drag-scroll smooth without saturating the shared bus; a release always renders
/// the final position immediately.
const DRAG_RENDER_MS: u64 = 50;

// ── Menu data ─────────────────────────────────────────────────────────────────

/// The robot's Main Menu labels, transcribed from the `touch-ui` crate's
/// `screens` module.
///
/// This is a snapshot and may drift from that crate, which is the source of
/// truth for the labels the UI ships. Its in-list Back entries are dropped: the
/// header Back button is the only way back.
const MAIN_MENU: [&str; 4] = ["System Info", "Calibrate", "Drive Mode", "Test Mode"];

/// The Calibrate submenu labels, transcribed from the firmware (no in-list Back).
const CALIBRATE_MENU: [&str; 3] = ["Motor", "Mag", "Distance"];

/// The Drive Mode submenu labels, transcribed from the firmware (no in-list Back).
const DRIVE_MODE_MENU: [&str; 2] = ["Coast & Avoid", "Attempt Straight"];

/// The Test Mode submenu labels, transcribed from the firmware (no in-list Back).
const TEST_MENU: [&str; 6] = [
    "Basic Motor Test",
    "Turns Test",
    "Straight Drive",
    "Arc Drive",
    "IMU Test (6-axis)",
    "IMU Test (9-axis)",
];

// ── System Info data ──────────────────────────────────────────────────────────

/// Dummy System Info values, mirroring the firmware's `SystemInfoData` shape.
///
/// No sensors are read: the rows are pre-formatted exactly as the firmware's
/// `format_battery_level_line`, `format_battery_voltage_line`, and
/// `format_cal_line` would render these values. The three calibration rows show
/// every status wording — `Loaded`, `Unknown`, and `Missing`. Like the menu
/// labels, this is a transcribed snapshot that may drift from the firmware.
struct SystemInfoData {
    /// Battery level row.
    battery_level: &'static str,
    /// Battery voltage row.
    battery_voltage: &'static str,
    /// Motor calibration status row.
    motor_calibration: &'static str,
    /// Magnetometer calibration status row.
    mag_calibration: &'static str,
    /// Distance calibration status row.
    distance_calibration: &'static str,
}

impl SystemInfoData {
    /// This instance's rows, in display order.
    const fn rows(&self) -> [&'static str; 5] {
        [
            self.battery_level,
            self.battery_voltage,
            self.motor_calibration,
            self.mag_calibration,
            self.distance_calibration,
        ]
    }
}

/// The dummy System Info values shown on the System Info screen.
const SYSTEM_INFO: SystemInfoData = SystemInfoData {
    battery_level: "Batt  87%",
    battery_voltage: "Batt  7.4V",
    motor_calibration: "Motor: Loaded",
    mag_calibration: "Mag: Unknown",
    distance_calibration: "Dist: Missing",
};

// ── Screen and interaction model ──────────────────────────────────────────────

/// Which menu screen the UI is showing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Screen {
    /// The robot's Main Menu.
    MainMenu,
    /// The Calibrate submenu.
    Calibrate,
    /// The Drive Mode submenu.
    DriveMode,
    /// The Test Mode submenu.
    TestMode,
    /// A non-navigable placeholder for a leaf, naming the action it would run.
    Placeholder(PlaceholderKind),
    /// The mocked System Info status screen.
    SystemInfo,
    /// A draggable value-entry screen for a distance flow.
    ValueEntry(ValueFlow),
}

impl Screen {
    /// The title shown in the header for this screen.
    const fn title(self) -> &'static str {
        match self {
            Self::MainMenu => "Main Menu",
            Self::Calibrate => "Calibrate",
            Self::DriveMode => "Drive Mode",
            Self::TestMode => "Test Mode",
            Self::Placeholder(kind) => kind.title(),
            Self::SystemInfo => "System Info",
            Self::ValueEntry(flow) => flow.title(),
        }
    }

    /// The labels shown as list buttons on this screen, in order.
    ///
    /// Placeholder, System Info, and value-entry screens are not lists, so they
    /// have none.
    const fn items(self) -> &'static [&'static str] {
        match self {
            Self::MainMenu => &MAIN_MENU,
            Self::Calibrate => &CALIBRATE_MENU,
            Self::DriveMode => &DRIVE_MODE_MENU,
            Self::TestMode => &TEST_MENU,
            Self::Placeholder(_) | Self::SystemInfo | Self::ValueEntry(_) => &[],
        }
    }

    /// The screen Back returns to, or `None` on the Main Menu (which has no Back).
    const fn parent(self) -> Option<Self> {
        match self {
            Self::MainMenu => None,
            Self::Calibrate | Self::DriveMode | Self::TestMode | Self::SystemInfo => Some(Self::MainMenu),
            Self::Placeholder(kind) => Some(kind.parent()),
            Self::ValueEntry(flow) => Some(flow.parent()),
        }
    }
}

/// A non-navigable leaf: a placeholder screen naming the action it would run.
///
/// These exist so tapping a leaf is visibly acknowledged without performing any
/// motor, turn, arc, or IMU action.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PlaceholderKind {
    /// Motor calibration.
    Motor,
    /// Magnetometer calibration.
    Mag,
    /// Coast-and-avoid drive mode.
    CoastAndAvoid,
    /// Basic motor test.
    BasicMotor,
    /// Turns test.
    Turns,
    /// Straight drive test.
    StraightDrive,
    /// Arc drive test.
    ArcDrive,
    /// Six-axis IMU test.
    Imu6Axis,
    /// Nine-axis IMU test.
    Imu9Axis,
}

impl PlaceholderKind {
    /// The header title, reusing the menu label that opened this placeholder.
    const fn title(self) -> &'static str {
        match self {
            Self::Motor => "Motor",
            Self::Mag => "Mag",
            Self::CoastAndAvoid => "Coast & Avoid",
            Self::BasicMotor => "Basic Motor Test",
            Self::Turns => "Turns Test",
            Self::StraightDrive => "Straight Drive",
            Self::ArcDrive => "Arc Drive",
            Self::Imu6Axis => "IMU Test (6-axis)",
            Self::Imu9Axis => "IMU Test (9-axis)",
        }
    }

    /// A short description of the action that would run.
    const fn body(self) -> &'static str {
        match self {
            Self::Motor => "Would run: Motor calibration",
            Self::Mag => "Would run: Magnetometer calibration",
            Self::CoastAndAvoid => "Would run: Coast & avoid drive",
            Self::BasicMotor => "Would run: Basic motor test",
            Self::Turns => "Would run: Turns test",
            Self::StraightDrive => "Would run: Straight drive",
            Self::ArcDrive => "Would run: Arc drive",
            Self::Imu6Axis => "Would run: IMU test (6-axis)",
            Self::Imu9Axis => "Would run: IMU test (9-axis)",
        }
    }

    /// The submenu this placeholder was opened from.
    const fn parent(self) -> Screen {
        match self {
            Self::Motor | Self::Mag => Screen::Calibrate,
            Self::CoastAndAvoid => Screen::DriveMode,
            Self::BasicMotor | Self::Turns | Self::StraightDrive | Self::ArcDrive | Self::Imu6Axis | Self::Imu9Axis => {
                Screen::TestMode
            }
        }
    }
}

/// A value-entry flow: the range, step, and preset for one distance setting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ValueFlow {
    /// Distance calibration: 0–200 cm in 1 cm steps.
    DistanceCalibration,
    /// Attempt-straight-line distance: 10–5000 cm in 10 cm steps.
    AttemptStraight,
}

impl ValueFlow {
    /// The header title, matching the menu label that opened this flow.
    const fn title(self) -> &'static str {
        match self {
            Self::DistanceCalibration => "Distance",
            Self::AttemptStraight => "Attempt Straight",
        }
    }

    /// The inclusive minimum value.
    const fn min(self) -> i32 {
        match self {
            Self::DistanceCalibration => 0,
            Self::AttemptStraight => 10,
        }
    }

    /// The inclusive maximum value.
    const fn max(self) -> i32 {
        match self {
            Self::DistanceCalibration => 200,
            Self::AttemptStraight => 5000,
        }
    }

    /// The fine-adjust step.
    const fn step(self) -> i32 {
        match self {
            Self::DistanceCalibration => 1,
            Self::AttemptStraight => 10,
        }
    }

    /// The value the screen opens at.
    const fn preset(self) -> i32 {
        match self {
            Self::DistanceCalibration => 150,
            Self::AttemptStraight => 100,
        }
    }

    /// The unit shown after the readout (both flows are centimetres).
    const fn unit(self) -> &'static str {
        match self {
            Self::DistanceCalibration | Self::AttemptStraight => "cm",
        }
    }

    /// The submenu this flow returns to.
    const fn parent(self) -> Screen {
        match self {
            Self::DistanceCalibration => Screen::Calibrate,
            Self::AttemptStraight => Screen::DriveMode,
        }
    }
}

/// An interactive region named by [`Ui::hit_test`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Hit {
    /// The header Back button (present on every list-like screen except the Main
    /// Menu).
    Back,
    /// The list item at this index on the current screen.
    MenuItem(usize),
    /// The header Cancel button on a value-entry screen.
    Cancel,
    /// The footer Save button on a value-entry screen.
    Save,
    /// The draggable slider track (and its thumb) on a value-entry screen.
    Slider,
    /// The fine-adjust decrement button on a value-entry screen.
    NudgeMinus,
    /// The fine-adjust increment button on a value-entry screen.
    NudgePlus,
}

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

/// The screen/state model: which screen is shown, the active press if any, the
/// current list scroll offset, and the current value-entry value.
///
/// Gesture policy (tap versus drag), scrolling, the slider, and hit-testing live
/// here, so the event loop above it only translates panel samples into
/// `pointer_*` calls and redisplays when a call reports that the screen changed.
struct Ui {
    /// The screen currently rendered.
    screen: Screen,
    /// The in-progress press, or `None` when the pen is up.
    press: Option<Press>,
    /// Vertical scroll offset of the current list, in pixels; `0` when it fits.
    scroll: u32,
    /// Current value on a value-entry screen, in the flow's unit; `0` otherwise.
    value: i32,
}

impl Ui {
    /// Create a UI showing the Main Menu with no press, scroll, or value.
    const fn new() -> Self {
        Self {
            screen: Screen::MainMenu,
            press: None,
            scroll: 0,
            value: 0,
        }
    }

    /// Record a pen-down at `p` and report whether the screen needs redrawing.
    ///
    /// A redraw is needed when the pen landed on a button, because that starts
    /// the pressed indication, or on the value screen's slider, which takes
    /// effect immediately.
    fn pointer_down(&mut self, p: Point, now_ms: u64) -> bool {
        let hit = self.hit_test(p);
        let on_slider = hit == Some(Hit::Slider);
        self.press = Some(Press {
            start: p,
            last: p,
            started_ms: now_ms,
            dragging: false,
            on_slider,
            target: hit,
        });
        info!(
            "touch down: pixel=({}, {}) region={}",
            p.x,
            p.y,
            hit_label(self.screen, hit)
        );
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
    /// the highlight.
    fn pointer_move(&mut self, p: Point, _now_ms: u64) -> bool {
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

        let dx = i64::from(p.x) - i64::from(press.start.x);
        let dy = i64::from(p.y) - i64::from(press.start.y);
        let max_move = i64::from(TAP_MAX_MOVE);
        let became_drag = !press.dragging && dx * dx + dy * dy > max_move * max_move;
        press.dragging |= became_drag;

        let now = self.hit_test(p);
        let mut redraw = was != now || became_drag;

        if press.dragging && self.press_scrolls(press.start) {
            redraw |= self.scroll_by(-step_y);
        }

        self.press = Some(press);
        redraw
    }

    /// End the active press at the stored release position and report whether the
    /// screen needs redrawing.
    ///
    /// A release is a tap when the press never became a drag or a slider drag,
    /// the total movement is below [`TAP_MAX_MOVE`], the press lasted at least
    /// [`TAP_MIN_DURATION_MS`], and the release landed on the same region the
    /// press began on; a tap on a region activates it. A drag never activates,
    /// but any press that began on a region still redraws on release: the
    /// throttled move redraws may not have cleared the pressed highlight or
    /// rendered the final scroll position or slider value.
    fn pointer_up(&mut self, now_ms: u64) -> bool {
        let Some(press) = self.press.take() else {
            return false;
        };

        let released = self.hit_test(press.last);
        let tap_candidate = !press.dragging && !press.on_slider;
        let same_region = released.is_some() && released == press.target;
        let dx = i64::from(press.last.x) - i64::from(press.start.x);
        let dy = i64::from(press.last.y) - i64::from(press.start.y);
        let max_move = i64::from(TAP_MAX_MOVE);
        let moved_sq = dx * dx + dy * dy;
        let elapsed_ms = now_ms.saturating_sub(press.started_ms);
        let is_tap =
            tap_candidate && same_region && moved_sq <= max_move * max_move && elapsed_ms >= TAP_MIN_DURATION_MS;

        let released = if is_tap { released } else { None };
        if let Some(hit) = released {
            info!(
                "tap: pixel=({}, {}) region={}",
                press.last.x,
                press.last.y,
                hit_label(self.screen, Some(hit))
            );
        }
        let activated = released.is_some_and(|hit| self.activate(hit));

        activated || press.target.is_some() || press.dragging
    }

    /// Run the action for a completed tap on `hit`, reporting whether the screen
    /// changed.
    fn activate(&mut self, hit: Hit) -> bool {
        match hit {
            Hit::Back => self.go_back(),
            Hit::MenuItem(index) => self.open_item(index),
            Hit::Cancel => self.leave_value_screen("cancel"),
            Hit::Save => self.leave_value_screen("save"),
            Hit::NudgeMinus => self.nudge(-1),
            Hit::NudgePlus => self.nudge(1),
            Hit::Slider => false,
        }
    }

    /// Leave the value screen for its parent, logging whether the exit was a
    /// cancel or a save. The mock performs no real action either way.
    fn leave_value_screen(&mut self, action: &'static str) -> bool {
        if let Screen::ValueEntry(flow) = self.screen {
            info!("value entry {}: {} = {} cm", action, flow.title(), self.value);
        }
        self.go_back()
    }

    /// Step the value by `steps` of the current flow's step size, clamped to its
    /// range, and report whether it changed.
    fn nudge(&mut self, steps: i32) -> bool {
        let Screen::ValueEntry(flow) = self.screen else {
            return false;
        };
        let next = (self.value + steps * flow.step()).clamp(flow.min(), flow.max());
        let changed = next != self.value;
        self.value = next;
        changed
    }

    /// Move the value to the slider position for `x`, reporting whether it
    /// changed.
    fn set_slider_value(&mut self, x: i32) -> bool {
        let Screen::ValueEntry(flow) = self.screen else {
            return false;
        };
        let next = slider_value(flow, x);
        let changed = next != self.value;
        self.value = next;
        changed
    }

    /// Return to the current screen's parent, if it has one.
    fn go_back(&mut self) -> bool {
        self.screen.parent().is_some_and(|parent| self.navigate_to(parent))
    }

    /// Open the item at `index` on the current screen.
    ///
    /// Most items open a submenu, a placeholder, System Info, or the value-entry
    /// screen for a distance flow.
    fn open_item(&mut self, index: usize) -> bool {
        let target = match (self.screen, index) {
            (Screen::MainMenu, 0) => Screen::SystemInfo,
            (Screen::MainMenu, 1) => Screen::Calibrate,
            (Screen::MainMenu, 2) => Screen::DriveMode,
            (Screen::MainMenu, 3) => Screen::TestMode,
            (Screen::Calibrate, 0) => Screen::Placeholder(PlaceholderKind::Motor),
            (Screen::Calibrate, 1) => Screen::Placeholder(PlaceholderKind::Mag),
            (Screen::Calibrate, 2) => Screen::ValueEntry(ValueFlow::DistanceCalibration),
            (Screen::DriveMode, 0) => Screen::Placeholder(PlaceholderKind::CoastAndAvoid),
            (Screen::DriveMode, 1) => Screen::ValueEntry(ValueFlow::AttemptStraight),
            (Screen::TestMode, 0) => Screen::Placeholder(PlaceholderKind::BasicMotor),
            (Screen::TestMode, 1) => Screen::Placeholder(PlaceholderKind::Turns),
            (Screen::TestMode, 2) => Screen::Placeholder(PlaceholderKind::StraightDrive),
            (Screen::TestMode, 3) => Screen::Placeholder(PlaceholderKind::ArcDrive),
            (Screen::TestMode, 4) => Screen::Placeholder(PlaceholderKind::Imu6Axis),
            (Screen::TestMode, 5) => Screen::Placeholder(PlaceholderKind::Imu9Axis),
            _ => {
                info!(
                    "activate: {} (no destination yet)",
                    hit_label(self.screen, Some(Hit::MenuItem(index)))
                );
                return false;
            }
        };
        self.navigate_to(target)
    }

    /// Switch to `screen`, reset the list scroll and the value-entry value, log
    /// the transition, and report a needed redraw.
    ///
    /// Entering a value screen sets the value to that flow's preset; leaving one
    /// clears it.
    fn navigate_to(&mut self, screen: Screen) -> bool {
        info!("navigate: {} -> {}", self.screen.title(), screen.title());
        self.screen = screen;
        self.scroll = 0;
        self.value = match screen {
            Screen::ValueEntry(flow) => flow.preset(),
            _ => 0,
        };
        true
    }

    /// The largest valid scroll offset for the current list (0 when it fits).
    const fn max_scroll(&self) -> u32 {
        max_scroll_for(self.screen.items().len())
    }

    /// Whether a press starting at `start` may scroll: the current list
    /// overflows and the press began within the list view below the header.
    const fn press_scrolls(&self, start: Point) -> bool {
        self.max_scroll() > 0 && start.y >= list_top() as i32
    }

    /// Add `delta` pixels to the scroll offset, clamped to `[0, max_scroll]`, and
    /// report whether the offset changed.
    fn scroll_by(&mut self, delta: i32) -> bool {
        let max = i64::from(self.max_scroll());
        let next = (i64::from(self.scroll) + i64::from(delta)).clamp(0, max);
        let next = u32::try_from(next).unwrap_or(self.scroll);
        let changed = next != self.scroll;
        self.scroll = next;
        changed
    }

    /// Return the interactive region under `p`, if any.
    fn hit_test(&self, p: Point) -> Option<Hit> {
        if let Screen::ValueEntry(_) = self.screen {
            return value_entry_hit(p);
        }
        if self.screen != Screen::MainMenu && back_button_rect().contains(p) {
            return Some(Hit::Back);
        }
        // Items scrolled above the list top must not be reachable through the
        // header/Back region.
        if p.y < list_top() as i32 {
            return None;
        }
        let items = self.screen.items();
        (0..items.len())
            .find(|&index| menu_item_rect(index, self.scroll).contains(p))
            .map(Hit::MenuItem)
    }

    /// Draw the current screen into `d`.
    ///
    /// Rendering is full-frame: the caller flushes the whole framebuffer after
    /// this returns.
    fn render<D>(&self, d: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        d.clear(BG)?;

        // A drag clears the pressed highlight, and a press only marks the region
        // it started on while the pointer is still on it.
        let pressed = self
            .press
            .as_ref()
            .filter(|press| !press.dragging)
            .and_then(|press| press.target.filter(|target| self.hit_test(press.last) == Some(*target)));

        // The body is drawn before the header so items scrolled above the list
        // top are painted over by the header background.
        match self.screen {
            Screen::Placeholder(kind) => draw_placeholder(d, kind)?,
            Screen::SystemInfo => draw_system_info(d)?,
            Screen::ValueEntry(flow) => draw_value_entry(d, flow, self.value, pressed)?,
            Screen::MainMenu | Screen::Calibrate | Screen::DriveMode | Screen::TestMode => {
                self.render_list(d, pressed)?;
            }
        }

        // Value screens replace the header Back with Cancel; the Main Menu has no
        // header action.
        let (action, action_pressed) = match self.screen {
            Screen::MainMenu => (None, false),
            Screen::ValueEntry(_) => (Some(Hit::Cancel), pressed == Some(Hit::Cancel)),
            _ => (Some(Hit::Back), pressed == Some(Hit::Back)),
        };
        draw_header(d, self.screen.title(), action, action_pressed)?;
        Ok(())
    }

    /// Draw the current screen's list of buttons, offset by the scroll, plus the
    /// scroll indicator when the list overflows.
    fn render_list<D>(&self, d: &mut D, pressed: Option<Hit>) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb565>,
    {
        for (index, label) in self.screen.items().iter().enumerate() {
            let is_pressed = pressed == Some(Hit::MenuItem(index));
            draw_button(
                d,
                menu_item_rect(index, self.scroll),
                label,
                LABEL_FONT,
                is_pressed,
                false,
            )?;
        }

        let max_scroll = self.max_scroll();
        if max_scroll > 0 {
            draw_scroll_indicator(d, self.scroll, max_scroll)?;
        }
        Ok(())
    }
}

// ── Layout ────────────────────────────────────────────────────────────────────

/// Vertical offset of the first list item below the header, in pixels.
const fn list_top() -> u32 {
    HEADER_H + BUTTON_GAP
}

/// Height of the list view below the header, in pixels.
const fn list_view_height() -> u32 {
    FB_H as u32 - list_top()
}

/// Total height of a list of `count` buttons and the gaps between them, in pixels.
const fn list_content_height(count: usize) -> u32 {
    count as u32 * BUTTON_H + (count as u32).saturating_sub(1) * BUTTON_GAP
}

/// The largest scroll offset for a list of `count` items, or `0` when it fits.
const fn max_scroll_for(count: usize) -> u32 {
    list_content_height(count).saturating_sub(list_view_height())
}

/// The on-screen rectangle of list item `index` at scroll offset `scroll`.
///
/// Every menu shares this geometry, and it is used by both rendering and
/// hit-testing so the two can never disagree.
const fn menu_item_rect(index: usize, scroll: u32) -> Rectangle {
    let x = SIDE_MARGIN;
    let y = (list_top() + index as u32 * (BUTTON_H + BUTTON_GAP)).saturating_sub(scroll);
    let width = FB_W as u32 - 2 * SIDE_MARGIN;
    Rectangle::new(Point::new(x as i32, y as i32), Size::new(width, BUTTON_H))
}

/// The on-screen rectangle of a header action button of the given width, inset
/// from the header's top-left.
const fn header_action_rect(width: u32) -> Rectangle {
    Rectangle::new(
        Point::new(SIDE_MARGIN as i32, BACK_MARGIN as i32),
        Size::new(width, HEADER_H - 2 * BACK_MARGIN),
    )
}

/// The on-screen rectangle of the header Back button (present on every list-like
/// screen except the Main Menu).
const fn back_button_rect() -> Rectangle {
    header_action_rect(BACK_W)
}

/// The on-screen rectangle of the header Cancel button on value screens.
const fn cancel_button_rect() -> Rectangle {
    header_action_rect(CANCEL_W)
}

/// The on-screen rectangle of the scroll-indicator track, in the right margin
/// beside the list.
const fn scroll_track_rect() -> Rectangle {
    Rectangle::new(
        Point::new((FB_W as u32 - SCROLL_MARGIN - SCROLL_W) as i32, list_top() as i32),
        Size::new(SCROLL_W, list_view_height()),
    )
}

/// The on-screen rectangle of the placeholder panel, inset from the screen edges
/// and below the header.
const fn placeholder_panel_rect() -> Rectangle {
    Rectangle::new(
        Point::new(PANEL_MARGIN as i32, HEADER_H as i32 + PANEL_MARGIN as i32),
        Size::new(
            FB_W as u32 - 2 * PANEL_MARGIN,
            FB_H as u32 - HEADER_H - 2 * PANEL_MARGIN,
        ),
    )
}

/// The on-screen rectangle of the value-entry readout area.
const fn readout_rect() -> Rectangle {
    Rectangle::new(
        Point::new(SIDE_MARGIN as i32, HEADER_H as i32),
        Size::new(FB_W as u32 - 2 * SIDE_MARGIN, READOUT_H),
    )
}

/// The on-screen rectangle of the slider track.
const fn slider_track_rect() -> Rectangle {
    Rectangle::new(
        Point::new(SLIDER_MARGIN as i32, SLIDER_Y as i32),
        Size::new(FB_W as u32 - 2 * SLIDER_MARGIN, SLIDER_H),
    )
}

/// The on-screen rectangle of the slider's touch band: the track height widened
/// to the thumb height so the slider is comfortable to grab.
const fn slider_touch_rect() -> Rectangle {
    let y = SLIDER_Y + SLIDER_H / 2 - SLIDER_THUMB_H / 2;
    Rectangle::new(
        Point::new(SLIDER_MARGIN as i32, y as i32),
        Size::new(FB_W as u32 - 2 * SLIDER_MARGIN, SLIDER_THUMB_H),
    )
}

/// The on-screen rectangle of the fine-adjust decrement button.
const fn nudge_minus_rect() -> Rectangle {
    Rectangle::new(
        Point::new(SIDE_MARGIN as i32, NUDGE_Y as i32),
        Size::new(NUDGE_W, NUDGE_H),
    )
}

/// The on-screen rectangle of the fine-adjust increment button.
const fn nudge_plus_rect() -> Rectangle {
    Rectangle::new(
        Point::new((FB_W as u32 - SIDE_MARGIN - NUDGE_W) as i32, NUDGE_Y as i32),
        Size::new(NUDGE_W, NUDGE_H),
    )
}

/// The on-screen rectangle of the value screen's footer band.
const fn footer_rect() -> Rectangle {
    Rectangle::new(
        Point::new(0, (FB_H as u32 - FOOTER_H) as i32),
        Size::new(FB_W as u32, FOOTER_H),
    )
}

/// The on-screen rectangle of the footer Save button, centered in the footer.
const fn save_button_rect() -> Rectangle {
    Rectangle::new(
        Point::new(
            ((FB_W as u32 - SAVE_W) / 2) as i32,
            (FB_H as u32 - FOOTER_H + FOOTER_PAD) as i32,
        ),
        Size::new(SAVE_W, FOOTER_H - 2 * FOOTER_PAD),
    )
}

/// The on-screen rectangle of the slider thumb for `value`.
fn slider_thumb_rect(flow: ValueFlow, value: i32) -> Rectangle {
    let left = SLIDER_MARGIN as i32;
    let travel = FB_W as i32 - 2 * SLIDER_MARGIN as i32 - SLIDER_THUMB_W as i32;
    let span = flow.max() - flow.min();
    let ratio = (value - flow.min()).clamp(0, span);
    let x = left + (i64::from(travel) * i64::from(ratio) / i64::from(span)) as i32;
    let y = SLIDER_Y as i32 + SLIDER_H as i32 / 2 - SLIDER_THUMB_H as i32 / 2;
    Rectangle::new(Point::new(x, y), Size::new(SLIDER_THUMB_W, SLIDER_THUMB_H))
}

/// The value nearest to pointer `x` on the slider, snapped to `flow`'s step and
/// clamped to its range.
fn slider_value(flow: ValueFlow, x: i32) -> i32 {
    let left = SLIDER_MARGIN as i32;
    let width = FB_W as i32 - 2 * SLIDER_MARGIN as i32;
    let span = i64::from(flow.max()) - i64::from(flow.min());
    let offset = i64::from((x - left).clamp(0, width));
    let raw = i64::from(flow.min()) + offset * span / i64::from(width);
    let step = i64::from(flow.step());
    let steps = (raw - i64::from(flow.min()) + step / 2) / step;
    let snapped = i64::from(flow.min()) + steps * step;
    let clamped = snapped.clamp(i64::from(flow.min()), i64::from(flow.max()));
    i32::try_from(clamped).unwrap_or_else(|_| flow.min())
}

/// The interactive region under `p` on a value-entry screen.
fn value_entry_hit(p: Point) -> Option<Hit> {
    if cancel_button_rect().contains(p) {
        return Some(Hit::Cancel);
    }
    if save_button_rect().contains(p) {
        return Some(Hit::Save);
    }
    if nudge_minus_rect().contains(p) {
        return Some(Hit::NudgeMinus);
    }
    if nudge_plus_rect().contains(p) {
        return Some(Hit::NudgePlus);
    }
    if slider_touch_rect().contains(p) {
        return Some(Hit::Slider);
    }
    None
}

/// The human-readable region name for a hit on `screen`, or `"none"`.
fn hit_label(screen: Screen, hit: Option<Hit>) -> &'static str {
    match hit {
        Some(Hit::Back) => "Back",
        Some(Hit::Cancel) => "Cancel",
        Some(Hit::Save) => "Save",
        Some(Hit::Slider) => "Slider",
        Some(Hit::NudgeMinus) => "-",
        Some(Hit::NudgePlus) => "+",
        Some(Hit::MenuItem(index)) => screen.items().get(index).copied().unwrap_or("?"),
        None => "none",
    }
}

// ── Widgets ───────────────────────────────────────────────────────────────────

/// Draw the title header with a muted divider along its bottom edge.
///
/// `action` is the header's left button, if any: [`Hit::Cancel`] on the value
/// screens and [`Hit::Back`] on the other list-like screens; the Main Menu
/// passes `None`. `action_pressed` selects its accent fill.
fn draw_header<D>(d: &mut D, title: &str, action: Option<Hit>, action_pressed: bool) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Fill the header background so list items scrolled up behind it are hidden.
    Rectangle::new(Point::zero(), Size::new(FB_W as u32, HEADER_H))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(d)?;

    // The title is centered in the space the header action leaves free, so a wide
    // Back/Cancel button and a long title can never overlap.
    let title_left = if let Some(action) = action {
        let (label, rect) = match action {
            Hit::Cancel => ("Cancel", cancel_button_rect()),
            _ => ("Back", back_button_rect()),
        };
        draw_button(d, rect, label, LABEL_FONT, action_pressed, true)?;
        rect.top_left.x + rect.size.width as i32 + HEADER_ACTION_GAP as i32
    } else {
        0
    };

    let area = Rectangle::new(
        Point::new(title_left, 0),
        Size::new(FB_W as u32 - title_left as u32, HEADER_H),
    );
    draw_centered_text(d, area, title, MonoTextStyle::new(TITLE_FONT, TEXT))?;

    Rectangle::new(
        Point::new(0, HEADER_H as i32 - DIVIDER_H as i32),
        Size::new(FB_W as u32, DIVIDER_H),
    )
    .into_styled(PrimitiveStyle::with_fill(MUTED))
    .draw(d)?;
    Ok(())
}

/// Draw one wide list button.
///
/// A pressed button is filled with [`ACCENT`] and its label inverts to [`BG`];
/// an idle button uses [`BUTTON_BG`] with a [`BUTTON_BORDER`] outline. `center`
/// centers the label in the button (the header Back uses it); otherwise the
/// label is left-aligned with [`BUTTON_PAD`].
fn draw_button<D>(
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
        draw_centered_text(d, rect, label, style)?;
    } else {
        draw_left_text(d, rect, label, style)?;
    }
    Ok(())
}

/// Draw `text` horizontally centered and vertically centered inside `area`.
fn draw_centered_text<D>(
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
fn draw_left_text<D>(d: &mut D, area: Rectangle, text: &str, style: MonoTextStyle<'_, Rgb565>) -> Result<(), D::Error>
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

/// Draw a non-navigable placeholder: a warning-bordered panel carrying the
/// leaf's title and a short "would run" body, with no list buttons.
fn draw_placeholder<D>(d: &mut D, kind: PlaceholderKind) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let panel = placeholder_panel_rect();
    panel.into_styled(PrimitiveStyle::with_fill(BUTTON_BG)).draw(d)?;
    panel
        .into_styled(PrimitiveStyle::with_stroke(WARN, PANEL_BORDER))
        .draw(d)?;

    let half = panel.size.height / 2;
    let title_area = Rectangle::new(panel.top_left, Size::new(panel.size.width, half));
    let body_area = Rectangle::new(
        Point::new(panel.top_left.x, panel.top_left.y + half as i32),
        Size::new(panel.size.width, panel.size.height - half),
    );
    draw_centered_text(d, title_area, kind.title(), MonoTextStyle::new(LABEL_FONT, TEXT))?;
    draw_centered_text(d, body_area, kind.body(), MonoTextStyle::new(SMALL_FONT, MUTED))?;
    Ok(())
}

/// Draw the mocked System Info rows, one per line in [`SMALL_FONT`].
fn draw_system_info<D>(d: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let style = MonoTextStyle::new(SMALL_FONT, TEXT);
    for (index, row) in SYSTEM_INFO.rows().iter().enumerate() {
        let area = Rectangle::new(
            Point::new(
                SIDE_MARGIN as i32,
                HEADER_H as i32 + BUTTON_GAP as i32 + index as i32 * INFO_ROW_H as i32,
            ),
            Size::new(FB_W as u32 - 2 * SIDE_MARGIN, INFO_ROW_H),
        );
        draw_left_text(d, area, row, style)?;
    }
    Ok(())
}

/// Draw the scroll indicator: a muted track over the list view with an accent
/// thumb whose height is the visible fraction and whose position follows the
/// finger.
///
/// The thumb rests at the bottom of the track when the list is at the top and
/// rises as the list scrolls down, so it moves the same way the finger does.
/// (This is the direction chosen on the bench: it is inverted from a desktop
/// scrollbar, where the thumb descends as you scroll down.) Swap the
/// `max_scroll - scroll` term for `scroll` to get the desktop direction.
///
/// Only called when `max_scroll > 0`, so the list overflows and the track sits
/// in the right margin without overlapping the buttons.
fn draw_scroll_indicator<D>(d: &mut D, scroll: u32, max_scroll: u32) -> Result<(), D::Error>
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

/// Draw the value-entry screen: the live readout, the slider, the fine-adjust
/// buttons, and the footer Save button.
fn draw_value_entry<D>(d: &mut D, flow: ValueFlow, value: i32, pressed: Option<Hit>) -> Result<(), D::Error>
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
    )?;
    Ok(())
}

// ── Value formatting ──────────────────────────────────────────────────────────

/// Capacity of [`TextBuf`], in bytes: enough for any value these flows produce
/// plus its unit.
const TEXT_BUF_LEN: usize = 16;

/// A tiny fixed-capacity ASCII buffer for formatting the value readout.
///
/// `heapless`/`alloc` are unavailable in this crate, and the readout is the only
/// text needing runtime formatting, so a small stack buffer implementing
/// [`core::fmt::Write`] is enough.
struct TextBuf {
    /// Backing bytes; only the first `len` are valid.
    bytes: [u8; TEXT_BUF_LEN],
    /// Number of valid bytes in `bytes`.
    len: usize,
}

impl TextBuf {
    /// Create an empty buffer.
    const fn new() -> Self {
        Self {
            bytes: [0; TEXT_BUF_LEN],
            len: 0,
        }
    }

    /// Overwrite the buffer with `value` and `unit`, e.g. `150 cm`.
    ///
    /// The buffer is sized for the widest value these flows produce, so the
    /// write cannot overflow; if it somehow did, the readout falls back to empty.
    fn set(&mut self, value: i32, unit: &str) {
        self.len = 0;
        if write!(self, "{value} {unit}").is_err() {
            self.len = 0;
        }
    }

    /// The written contents as a string (empty if not valid UTF-8, which cannot
    /// happen for the ASCII written here).
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}

impl core::fmt::Write for TextBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.bytes.len() - self.len;
        let take = bytes.len().min(remaining);
        self.bytes[self.len..self.len + take].copy_from_slice(&bytes[..take]);
        self.len += take;
        if take < bytes.len() {
            return Err(core::fmt::Error);
        }
        Ok(())
    }
}

// ── Touch sampling ────────────────────────────────────────────────────────────

/// Push `raw` through the caller-side median filters and map the median through
/// `calibration` to a screen pixel.
fn filter_and_calibrate(
    x_filter: &mut MovingMedian<u16, MEDIAN_WINDOW>,
    y_filter: &mut MovingMedian<u16, MEDIAN_WINDOW>,
    calibration: Calibration,
    raw: TouchSample,
) -> Point {
    // `add_value` only fails on NaN, which `u16` cannot produce.
    x_filter.add_value(raw.x).unwrap();
    y_filter.add_value(raw.y).unwrap();
    let median_x = x_filter.median().unwrap_or(raw.x);
    let median_y = y_filter.median().unwrap_or(raw.y);
    calibration.to_pixels(TouchSample {
        x: median_x,
        y: median_y,
        z1: raw.z1,
        z2: raw.z2,
    })
}

// ── Entry point ───────────────────────────────────────────────────────────────

/// The embassy entry point: bring up the shared bus, display, and touch panel,
/// then run the menu's pen-down / poll / pen-up loop over the calibrated touch
/// stream.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(embassy_rp::config::Config::default());
    info!("touch -> tft menu");

    // ── Display: ST7789 over the shared bus ─────────────────────────────────
    // Backlight on.
    let _backlight = Output::new(p.PIN_20, Level::High);

    // Hardware reset: RST low, settle, RST high, settle.
    let mut rst = Output::new(p.PIN_27, Level::Low);
    rst.set_low();
    Timer::after(Duration::from_millis(PANEL_RESET_LOW_MS)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(PANEL_RESET_HIGH_MS)).await;

    // Full-duplex SPI0 with per-transaction DMA. The bus is initialised with the
    // touch config; `SpiDeviceWithConfig` reconfigures it before each use.
    let spi = Spi::new(
        p.SPI0,
        p.PIN_18,
        p.PIN_19,
        p.PIN_16,
        p.DMA_CH6,
        p.DMA_CH7,
        Irqs,
        touch_spi_config(),
    );
    let spi_bus: &'static SpiBus = SPI_BUS.init(Mutex::new(spi));

    // Both devices arbitrate on the shared bus; their CS pins select which is
    // listening. CS idles high so the deselected device stays quiet.
    let display_dev = SpiDeviceWithConfig::new(spi_bus, Output::new(p.PIN_17, Level::High), display_spi_config());
    let touch_dev = SpiDeviceWithConfig::new(spi_bus, Output::new(p.PIN_21, Level::High), touch_spi_config());

    let dcx = Output::new(p.PIN_26, Level::Low);
    let fb = FB.take();
    let mut display = St7789::new(display_dev, dcx, fb, FB_W as u16, FB_H as u16);

    let config = DisplayConfig {
        // This panel is RGB-ordered: with `Bgr` the red and blue channels swap.
        color_order: ColorOrder::Rgb,
        // Readable landscape, matching `lidar_tft_radar`: rotate 90° only.
        // `touch_coexistence` adds a flip and a 180° rotation, which stays
        // self-consistent for its shapes but mirrors the image horizontally;
        // `CALIBRATION` is adjusted for the orientation used here.
        orientation: Orientation::new().rotate(Rotation::Deg90),
        invert_colors: false,
    };
    display.init(&config, &mut Delay).await.unwrap();
    info!("display initialised");

    // ── Touch: XPT2046/TSC2046-class controller on the same bus ─────────────
    // PENIRQ is active-low, so pull it up and treat a low level as pen-down.
    let mut irq = Input::new(p.PIN_22, Pull::Up);
    let mut panel = TouchPanel::new(touch_dev);

    // Caller-side smoothing: the driver stays raw, the example owns the filter.
    let mut x_filter = MovingMedian::<u16, MEDIAN_WINDOW>::new();
    let mut y_filter = MovingMedian::<u16, MEDIAN_WINDOW>::new();
    // Measured on this panel during bring-up, mirrored onto this example's
    // readable landscape orientation (see `CALIBRATION`).
    let calibration = CALIBRATION;

    let mut ui = Ui::new();
    ui.render(&mut display).unwrap();
    display.flush().await.unwrap();
    info!("touch a button to navigate; Back returns to the parent menu");

    loop {
        // `PENIRQ` is the pen-down authority; the filtered first sample starts
        // the press.
        let raw = panel.wait_for_touch(&mut irq).await.unwrap();
        // Start each gesture with fresh filters. The moving median must not carry
        // samples over from the previous touch: otherwise the first (and, for a
        // quick tap, effectively the only) calibrated point is pulled toward where
        // the finger last was, so the hit-test targets the wrong region and the
        // header Back button in particular misses.
        x_filter.clear();
        y_filter.clear();
        let point = filter_and_calibrate(&mut x_filter, &mut y_filter, calibration, raw);
        let now_ms = Instant::now().as_millis();
        if ui.pointer_down(point, now_ms) {
            ui.render(&mut display).unwrap();
            display.flush().await.unwrap();
        }
        let mut last_drag_render = now_ms;

        // Poll while the pen is down so continuous drag positions are available.
        // `PENIRQ` is the pen-up authority: `read` can return `Ok(None)` on noise
        // or `Err` on a transient bus error while a finger is still down, so the
        // gesture ends only once the IRQ line is high, not on a single bad read.
        loop {
            Timer::after(Duration::from_millis(TICK_MS)).await;
            let now_ms = Instant::now().as_millis();
            match panel.read().await {
                Ok(Some(raw)) => {
                    let point = filter_and_calibrate(&mut x_filter, &mut y_filter, calibration, raw);
                    if ui.pointer_move(point, now_ms) && now_ms.saturating_sub(last_drag_render) >= DRAG_RENDER_MS {
                        ui.render(&mut display).unwrap();
                        display.flush().await.unwrap();
                        last_drag_render = now_ms;
                    }
                }
                Ok(None) | Err(_) => {
                    if irq.is_high() {
                        if ui.pointer_up(now_ms) {
                            ui.render(&mut display).unwrap();
                            display.flush().await.unwrap();
                        }
                        break;
                    }
                }
            }
        }
    }
}
