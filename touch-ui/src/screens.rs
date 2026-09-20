//! The robot's menu tree: the screens, their entries, and where Back returns.
//!
//! [`Item`] names every entry of the Main Menu and its Calibrate, Drive Mode and
//! Test Mode submenus, in one enumeration; [`Screen::items`] returns each list
//! screen's entries in display order and [`Item::label`] is the text the button
//! draws. This crate is the source of truth for the labels it ships: the entries a
//! caller sees and the labels it acts on are the same data, so there is no second
//! ordering to keep in step.
//!
//! The in-list Back entries are dropped: the header Back button is the only way
//! back.

/// One entry in a list screen.
///
/// This is the single enumeration of every menu entry the crate ships, across the
/// Main Menu and its Calibrate, Drive Mode and Test Mode submenus. Each variant
/// supplies its [`Item::label`] and the [`Item::destination`] it opens, so the
/// label a caller acts on and the entry it sees are the same data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Item {
    /// Main Menu: the System Info status screen.
    SystemInfo,
    /// Main Menu: the Calibrate submenu.
    Calibrate,
    /// Main Menu: the Drive Mode submenu.
    DriveMode,
    /// Main Menu: the Test Mode submenu.
    TestMode,
    /// Calibrate: motor calibration, a placeholder leaf.
    Motor,
    /// Calibrate: magnetometer calibration, a placeholder leaf.
    Mag,
    /// Calibrate: the distance-calibration value screen.
    Distance,
    /// Drive Mode: coast-and-avoid, a placeholder leaf.
    CoastAndAvoid,
    /// Drive Mode: the attempt-straight value screen.
    AttemptStraight,
    /// Test Mode: the basic motor test, a placeholder leaf.
    BasicMotor,
    /// Test Mode: the turns test, a placeholder leaf.
    Turns,
    /// Test Mode: the straight drive test, a placeholder leaf.
    StraightDrive,
    /// Test Mode: the arc drive test, a placeholder leaf.
    ArcDrive,
    /// Test Mode: the six-axis IMU test, a placeholder leaf.
    Imu6Axis,
    /// Test Mode: the nine-axis IMU test, a placeholder leaf.
    Imu9Axis,
    /// Test Mode: the Room Scan radar screen.
    RoomScan,
}

impl Item {
    /// The label the list button draws for this entry.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::SystemInfo => "System Info",
            Self::Calibrate => "Calibrate",
            Self::DriveMode => "Drive Mode",
            Self::TestMode => "Test Mode",
            Self::Motor => "Motor",
            Self::Mag => "Mag",
            Self::Distance => "Distance",
            Self::CoastAndAvoid => "Coast & Avoid",
            Self::AttemptStraight => "Attempt Straight",
            Self::BasicMotor => "Basic Motor Test",
            Self::Turns => "Turns Test",
            Self::StraightDrive => "Straight Drive",
            Self::ArcDrive => "Arc Drive",
            Self::Imu6Axis => "IMU Test (6-axis)",
            Self::Imu9Axis => "IMU Test (9-axis)",
            Self::RoomScan => "Room Scan",
        }
    }

    /// The screen this entry opens.
    #[must_use]
    pub const fn destination(self) -> Screen {
        match self {
            Self::SystemInfo => Screen::SystemInfo,
            Self::Calibrate => Screen::Calibrate,
            Self::DriveMode => Screen::DriveMode,
            Self::TestMode => Screen::TestMode,
            Self::Motor => Screen::Placeholder(PlaceholderKind::Motor),
            Self::Mag => Screen::Placeholder(PlaceholderKind::Mag),
            Self::Distance => Screen::ValueEntry(ValueFlow::DistanceCalibration),
            Self::CoastAndAvoid => Screen::Placeholder(PlaceholderKind::CoastAndAvoid),
            Self::AttemptStraight => Screen::ValueEntry(ValueFlow::AttemptStraight),
            Self::BasicMotor => Screen::Placeholder(PlaceholderKind::BasicMotor),
            Self::Turns => Screen::Placeholder(PlaceholderKind::Turns),
            Self::StraightDrive => Screen::Placeholder(PlaceholderKind::StraightDrive),
            Self::ArcDrive => Screen::Placeholder(PlaceholderKind::ArcDrive),
            Self::Imu6Axis => Screen::Placeholder(PlaceholderKind::Imu6Axis),
            Self::Imu9Axis => Screen::Placeholder(PlaceholderKind::Imu9Axis),
            Self::RoomScan => Screen::RoomScan,
        }
    }
}

/// The Main Menu's entries, in display order.
const MAIN_MENU_ITEMS: [Item; 4] = [Item::SystemInfo, Item::Calibrate, Item::DriveMode, Item::TestMode];

/// The Calibrate submenu's entries, in display order.
const CALIBRATE_ITEMS: [Item; 3] = [Item::Motor, Item::Mag, Item::Distance];

/// The Drive Mode submenu's entries, in display order.
const DRIVE_MODE_ITEMS: [Item; 2] = [Item::CoastAndAvoid, Item::AttemptStraight];

/// The Test Mode submenu's entries, in display order.
///
/// Room Scan is the last entry: it opens the sensor's live radar rather than
/// running a test-mode task, but it shares the menu and its single-active guard.
const TEST_MODE_ITEMS: [Item; 7] = [
    Item::BasicMotor,
    Item::Turns,
    Item::StraightDrive,
    Item::ArcDrive,
    Item::Imu6Axis,
    Item::Imu9Axis,
    Item::RoomScan,
];

/// Which menu screen the UI is showing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Screen {
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
    /// The System Info status screen, rendered from the neutral snapshot.
    SystemInfo,
    /// A draggable value-entry screen for a distance flow.
    ValueEntry(ValueFlow),
    /// A running/status screen, whose content is supplied by [`StatusView`].
    ///
    /// Later tickets use this for a procedure's progress and sensor state; it
    /// offers a touch Stop in the header.
    Status,
    /// The Room Scan screen: the sensor's live spins drawn as a radar.
    ///
    /// It is not a running procedure: its header offers Back, and the firmware
    /// acquires the `LiDAR` on entry and releases it on exit.
    RoomScan,
}

impl Screen {
    /// The title shown in the header for this screen.
    ///
    /// [`Screen::Status`] has no title of its own — [`crate::Ui::title`]
    /// returns the dynamic title from the active [`StatusView`] instead; this
    /// fallback is only used if the enum is matched on directly.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::MainMenu => "Main Menu",
            Self::Calibrate => "Calibrate",
            Self::DriveMode => "Drive Mode",
            Self::TestMode => "Test Mode",
            Self::Placeholder(kind) => kind.title(),
            Self::SystemInfo => "System Info",
            Self::ValueEntry(flow) => flow.title(),
            Self::Status => "Running",
            Self::RoomScan => "Room Scan",
        }
    }

    /// The entries shown as list buttons on this screen, in order.
    ///
    /// Each entry supplies its own [`Item::label`], so this ordered slice is both
    /// what the screen draws and what a caller resolves through
    /// [`Item::destination`].
    ///
    /// Placeholder, System Info, status, and value-entry screens are not lists,
    /// so they have none.
    #[must_use]
    pub const fn items(self) -> &'static [Item] {
        match self {
            Self::MainMenu => &MAIN_MENU_ITEMS,
            Self::Calibrate => &CALIBRATE_ITEMS,
            Self::DriveMode => &DRIVE_MODE_ITEMS,
            Self::TestMode => &TEST_MODE_ITEMS,
            Self::Placeholder(_) | Self::SystemInfo | Self::ValueEntry(_) | Self::Status | Self::RoomScan => &[],
        }
    }

    /// The screen Back returns to, or `None` on the Main Menu.
    ///
    /// [`Screen::Status`]'s parent is held in the active [`StatusView`] and is
    /// resolved by [`crate::Ui`] rather than here, so this returns `None` for
    /// it. Room Scan returns to the Test Mode menu it is opened from.
    #[must_use]
    pub const fn parent(self) -> Option<Self> {
        match self {
            Self::MainMenu | Self::Status => None,
            Self::Calibrate | Self::DriveMode | Self::TestMode | Self::SystemInfo => Some(Self::MainMenu),
            Self::Placeholder(kind) => Some(kind.parent()),
            Self::ValueEntry(flow) => Some(flow.parent()),
            Self::RoomScan => Some(Self::TestMode),
        }
    }
}

/// A non-navigable leaf: a placeholder screen naming the action it would run.
///
/// These exist so tapping a leaf is visibly acknowledged without performing any
/// motor, turn, arc, or IMU action.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlaceholderKind {
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
    #[must_use]
    pub const fn title(self) -> &'static str {
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
    #[must_use]
    pub const fn body(self) -> &'static str {
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
    #[must_use]
    pub const fn parent(self) -> Screen {
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
pub enum ValueFlow {
    /// Distance calibration: 0–200 cm in 1 cm steps.
    DistanceCalibration,
    /// Attempt-straight-line distance: 10–5000 cm in 10 cm steps.
    AttemptStraight,
}

impl ValueFlow {
    /// The header title, matching the menu label that opened this flow.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::DistanceCalibration => "Distance",
            Self::AttemptStraight => "Attempt Straight",
        }
    }

    /// The inclusive minimum value.
    #[must_use]
    pub const fn min(self) -> i32 {
        match self {
            Self::DistanceCalibration => 0,
            Self::AttemptStraight => 10,
        }
    }

    /// The inclusive maximum value.
    #[must_use]
    pub const fn max(self) -> i32 {
        match self {
            Self::DistanceCalibration => 200,
            Self::AttemptStraight => 5000,
        }
    }

    /// The fine-adjust step.
    #[must_use]
    pub const fn step(self) -> i32 {
        match self {
            Self::DistanceCalibration => 1,
            Self::AttemptStraight => 10,
        }
    }

    /// The value the screen opens at.
    #[must_use]
    pub const fn preset(self) -> i32 {
        match self {
            Self::DistanceCalibration => 150,
            Self::AttemptStraight => 100,
        }
    }

    /// The unit shown after the readout (both flows are centimetres).
    #[must_use]
    pub const fn unit(self) -> &'static str {
        match self {
            Self::DistanceCalibration | Self::AttemptStraight => "cm",
        }
    }

    /// The submenu this flow returns to.
    #[must_use]
    pub const fn parent(self) -> Screen {
        match self {
            Self::DistanceCalibration => Screen::Calibrate,
            Self::AttemptStraight => Screen::DriveMode,
        }
    }
}

/// The neutral content of a running/status screen.
///
/// The firmware maps its activity state onto this and calls
/// [`crate::Ui::show_status`] on entry and [`crate::Ui::set_status`] as progress
/// advances. `parent` is where the header Stop returns to, and `progress` is the
/// percent the widget draws as a bar when the procedure can report one — the
/// model never formats a number, so no dynamic text crosses this boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StatusView {
    /// The header title.
    pub title: &'static str,
    /// The body line: the procedure's phase or the sensor's state.
    pub body: &'static str,
    /// Percent progress (0–100), drawn as a bar; `None` to draw no bar.
    pub progress: Option<u8>,
    /// The screen Stop returns to.
    pub parent: Screen,
}

impl StatusView {
    /// Build a status view with no progress bar.
    #[must_use]
    pub const fn new(title: &'static str, body: &'static str, parent: Screen) -> Self {
        Self {
            title,
            body,
            progress: None,
            parent,
        }
    }

    /// The same view carrying a percent progress bar.
    #[must_use]
    pub const fn with_progress(self, percent: u8) -> Self {
        Self {
            progress: Some(percent),
            ..self
        }
    }
}
