//! The robot's menu tree: the screens, their labels, and where Back returns.
//!
//! The labels are transcribed from the firmware's `src/task/ui/screens.rs` (a
//! snapshot that the firmware remains the source of truth for). The firmware's
//! in-list Back entries are dropped: the header Back button is the only way
//! back.

/// The Main Menu labels.
pub const MAIN_MENU: [&str; 4] = ["System Info", "Calibrate", "Drive Mode", "Test Mode"];

/// The Calibrate submenu labels (no in-list Back).
pub const CALIBRATE_MENU: [&str; 3] = ["Motor", "Mag", "Distance"];

/// The Drive Mode submenu labels (no in-list Back).
pub const DRIVE_MODE_MENU: [&str; 2] = ["Coast & Avoid", "Attempt Straight"];

/// The Test Mode submenu labels (no in-list Back).
pub const TEST_MENU: [&str; 6] = [
    "Basic Motor Test",
    "Turns Test",
    "Straight Drive",
    "Arc Drive",
    "IMU Test (6-axis)",
    "IMU Test (9-axis)",
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
        }
    }

    /// The labels shown as list buttons on this screen, in order.
    ///
    /// Placeholder, System Info, status, and value-entry screens are not lists,
    /// so they have none.
    #[must_use]
    pub const fn items(self) -> &'static [&'static str] {
        match self {
            Self::MainMenu => &MAIN_MENU,
            Self::Calibrate => &CALIBRATE_MENU,
            Self::DriveMode => &DRIVE_MODE_MENU,
            Self::TestMode => &TEST_MENU,
            Self::Placeholder(_) | Self::SystemInfo | Self::ValueEntry(_) | Self::Status => &[],
        }
    }

    /// The screen Back returns to, or `None` on the Main Menu.
    ///
    /// [`Screen::Status`]'s parent is held in the active [`StatusView`] and is
    /// resolved by [`crate::Ui`] rather than here, so this returns `None` for
    /// it.
    #[must_use]
    pub const fn parent(self) -> Option<Self> {
        match self {
            Self::MainMenu | Self::Status => None,
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
/// [`crate::Ui::show_status`] on entry and [`crate::Ui::set_status_body`] as
/// progress advances. `parent` is where the header Stop returns to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StatusView {
    /// The header title.
    pub title: &'static str,
    /// The body line: the procedure's progress or the sensor's state.
    pub body: &'static str,
    /// The screen Stop returns to.
    pub parent: Screen,
}

impl StatusView {
    /// Build a status view.
    #[must_use]
    pub const fn new(title: &'static str, body: &'static str, parent: Screen) -> Self {
        Self { title, body, parent }
    }
}
