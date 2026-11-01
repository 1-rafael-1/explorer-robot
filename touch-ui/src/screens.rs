//! The robot's menu tree: the screens, their entries, and where Back returns.
//!
//! [`Item`] names every entry of the Main Menu and its Calibrate, Drive Mode and
//! Test Mode submenus, in one enumeration; [`Screen::items`] returns each list
//! screen's entries in display order and [`Item::label`] is the text the button
//! draws. Each entry carries its own label, the screen it opens, the screen Back
//! returns to, and whether it ends only on an explicit stop, so no other table in
//! the crate states a label or a parent relation.
//!
//! An entry is a [`Submenu`], a [`Procedure`] or a [`ScreenEntry`], so the domain
//! distinction is in the type rather than in a guard.
//!
//! The in-list Back entries are dropped: the header Back button is the only way
//! back.

/// One entry in a list screen.
///
/// This is the single enumeration of every menu entry the crate ships, across the
/// Main Menu and its Calibrate, Drive Mode and Test Mode submenus. Each variant
/// supplies its [`Item::label`], its [`Item::destination`], the [`Item::parent`]
/// Back returns to, and its [`Item::interactive`] flag, so the label a caller
/// acts on and the entry it sees are the same data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Item {
    /// A submenu: Calibrate, Drive Mode, or Test Mode.
    Submenu(Submenu),
    /// A unit of work the Panel can start.
    Procedure(Procedure),
    /// A screen that is not a unit of work.
    ScreenEntry(ScreenEntry),
}

/// The facts every Menu Entry supplies, resolved from its variant.
///
/// [`Item::entry`] is the enumeration's one dispatch: it matches the variant once
/// and projects it into these fields, so each accessor below reads a field
/// instead of repeating the same three-way match.
#[derive(Clone, Copy)]
struct EntryFacts {
    /// The entry's one name: the list button's text and the header it opens.
    label: &'static str,
    /// The screen the entry opens.
    destination: Screen,
    /// The screen Back returns to from `destination`.
    parent: Option<Screen>,
    /// Whether the entry's Procedure ends only on an explicit stop.
    interactive: bool,
    /// The Procedure the entry starts, or `None` for a submenu or a screen.
    procedure: Option<Procedure>,
}

impl Item {
    /// Project this entry's variant into its facts.
    ///
    /// This is the only place [`Item`] matches its three variants; the accessors
    /// below all read this one value.
    const fn entry(self) -> EntryFacts {
        match self {
            Self::Submenu(submenu) => EntryFacts {
                label: submenu.label(),
                destination: submenu.destination(),
                parent: submenu.parent(),
                interactive: false,
                procedure: None,
            },
            Self::Procedure(procedure) => EntryFacts {
                label: procedure.label(),
                destination: procedure.destination(),
                parent: procedure.parent(),
                interactive: procedure.interactive(),
                procedure: Some(procedure),
            },
            Self::ScreenEntry(entry) => EntryFacts {
                label: entry.label(),
                destination: entry.destination(),
                parent: entry.parent(),
                interactive: false,
                procedure: None,
            },
        }
    }

    /// The label the list button draws for this entry.
    #[must_use]
    pub const fn label(self) -> &'static str {
        self.entry().label
    }

    /// The header title of the screen this entry opens.
    ///
    /// An entry has one name (ADR-0013), so this is its [`Item::label`]; it is
    /// kept as a distinct accessor for the running screen, whose header reads the
    /// entry rather than restating it.
    #[must_use]
    pub const fn title(self) -> &'static str {
        self.entry().label
    }

    /// The screen this entry opens.
    #[must_use]
    pub const fn destination(self) -> Screen {
        self.entry().destination
    }

    /// The screen Back returns to from this entry's destination.
    #[must_use]
    pub const fn parent(self) -> Option<Screen> {
        self.entry().parent
    }

    /// Whether the Procedure this entry starts ends only on an explicit stop.
    ///
    /// A submenu and a screen are not Procedures, so they are never interactive.
    #[must_use]
    pub const fn interactive(self) -> bool {
        self.entry().interactive
    }

    /// The Procedure this entry starts, or `None` for a submenu or a screen.
    ///
    /// A caller obtains the Procedure's identity through this rather than
    /// inferring it from the label or the position.
    #[must_use]
    pub const fn as_procedure(self) -> Option<Procedure> {
        self.entry().procedure
    }
}

/// A submenu of the Main Menu, whose entry opens further entries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Submenu {
    /// The Calibrate submenu.
    Calibrate,
    /// The Drive Mode submenu.
    DriveMode,
    /// The Test Mode submenu.
    TestMode,
}

impl Submenu {
    /// The label the Main Menu button draws for this submenu.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Calibrate => "Calibrate",
            Self::DriveMode => "Drive Mode",
            Self::TestMode => "Test Mode",
        }
    }

    /// The submenu screen this entry opens.
    #[must_use]
    pub const fn destination(self) -> Screen {
        match self {
            Self::Calibrate => Screen::Calibrate,
            Self::DriveMode => Screen::DriveMode,
            Self::TestMode => Screen::TestMode,
        }
    }

    /// The screen Back returns to, which is the Main Menu for every submenu.
    #[must_use]
    pub const fn parent(self) -> Option<Screen> {
        match self {
            Self::Calibrate | Self::DriveMode | Self::TestMode => Some(Screen::MainMenu),
        }
    }
}

/// A unit of work the Panel can start.
///
/// Ten run today: motor, magnetometer and distance calibration; coast-and-avoid;
/// and the six test modes. Attempt-straight is modelled the same way but the
/// firmware does not start it yet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Procedure {
    /// Calibrate: motor calibration, opening the running screen.
    MotorCalibration,
    /// Calibrate: magnetometer calibration, opening the running screen.
    MagCalibration,
    /// Calibrate: the distance-calibration value screen, after its drive step.
    DistanceCalibration,
    /// Drive Mode: coast-and-avoid, opening the running screen.
    CoastAndAvoid,
    /// Drive Mode: the attempt-straight value screen, not started yet.
    AttemptStraight,
    /// Test Mode: the basic motor test, opening the running screen.
    BasicMotor,
    /// Test Mode: the turns test, opening the running screen.
    Turns,
    /// Test Mode: the straight drive test, opening the running screen.
    StraightDrive,
    /// Test Mode: the arc drive test, opening the running screen.
    ArcDrive,
    /// Test Mode: the six-axis IMU test, opening the running screen.
    Imu6Axis,
    /// Test Mode: the nine-axis IMU test, opening the running screen.
    Imu9Axis,
}

impl Procedure {
    /// The label the list button draws for this Procedure.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::MotorCalibration => "Motor",
            Self::MagCalibration => "Mag",
            Self::DistanceCalibration => "Distance",
            Self::CoastAndAvoid => "Coast & Avoid",
            Self::AttemptStraight => "Attempt Straight",
            Self::BasicMotor => "Basic Motor Test",
            Self::Turns => "Turns Test",
            Self::StraightDrive => "Straight Drive",
            Self::ArcDrive => "Arc Drive",
            Self::Imu6Axis => "IMU Test (6-axis)",
            Self::Imu9Axis => "IMU Test (9-axis)",
        }
    }

    /// The screen this Procedure opens.
    ///
    /// Distance calibration finishes its drive step before its value-entry screen
    /// opens, and the deferred attempt-straight is only a value screen; every
    /// other Procedure opens the running [`Screen::Status`].
    #[must_use]
    pub const fn destination(self) -> Screen {
        match self {
            Self::DistanceCalibration => Screen::ValueEntry(ValueFlow::DistanceCalibration),
            Self::AttemptStraight => Screen::ValueEntry(ValueFlow::AttemptStraight),
            Self::MotorCalibration
            | Self::MagCalibration
            | Self::CoastAndAvoid
            | Self::BasicMotor
            | Self::Turns
            | Self::StraightDrive
            | Self::ArcDrive
            | Self::Imu6Axis
            | Self::Imu9Axis => Screen::Status,
        }
    }

    /// The screen Back returns to when this Procedure finishes.
    ///
    /// A test mode returns to the Test Mode menu it was opened from, both when
    /// the operator stops it and when a run-to-completion test finishes on its
    /// own. A calibration returns to the Calibrate submenu and a drive mode to
    /// the Drive Mode submenu.
    #[must_use]
    pub const fn parent(self) -> Option<Screen> {
        match self {
            Self::MotorCalibration | Self::MagCalibration | Self::DistanceCalibration => Some(Screen::Calibrate),
            Self::CoastAndAvoid | Self::AttemptStraight => Some(Screen::DriveMode),
            Self::BasicMotor | Self::Turns | Self::StraightDrive | Self::ArcDrive | Self::Imu6Axis | Self::Imu9Axis => {
                Some(Screen::TestMode)
            }
        }
    }

    /// The screen this Procedure lands on when it finishes.
    ///
    /// This is its parent, or the Main Menu for a Procedure that names none. The
    /// firmware resolves a finished Procedure's landing screen through this
    /// rather than naming a screen, so where a finished Procedure lands stays a
    /// fact of its identity (ADR-0013).
    #[must_use]
    pub const fn landing_screen(self) -> Screen {
        match self.parent() {
            Some(screen) => screen,
            None => Screen::MainMenu,
        }
    }

    /// Whether this Procedure is one of the three calibrations.
    #[must_use]
    pub const fn is_calibration(self) -> bool {
        matches!(
            self,
            Self::MotorCalibration | Self::MagCalibration | Self::DistanceCalibration
        )
    }

    /// Whether this Procedure ends only on an explicit stop.
    ///
    /// The flag is entry data: a run-to-completion test finishes on its own, while
    /// an interactive one and coast-and-avoid run until the operator stops them.
    /// The basic-motor, six-axis and nine-axis tests and coast-and-avoid are
    /// interactive; the calibrations, the remaining tests and the deferred
    /// attempt-straight are not.
    #[must_use]
    pub const fn interactive(self) -> bool {
        matches!(
            self,
            Self::CoastAndAvoid | Self::BasicMotor | Self::Imu6Axis | Self::Imu9Axis
        )
    }
}

/// A screen that is a Menu Entry but not a unit of work.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScreenEntry {
    /// Main Menu: the System Info status screen.
    SystemInfo,
    /// Test Mode: the Room Scan radar screen.
    RoomScan,
}

impl ScreenEntry {
    /// The label the list button draws for this entry.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::SystemInfo => "System Info",
            Self::RoomScan => "Room Scan",
        }
    }

    /// The screen this entry opens.
    #[must_use]
    pub const fn destination(self) -> Screen {
        match self {
            Self::SystemInfo => Screen::SystemInfo,
            Self::RoomScan => Screen::RoomScan,
        }
    }

    /// The screen Back returns to.
    #[must_use]
    pub const fn parent(self) -> Option<Screen> {
        match self {
            Self::SystemInfo => Some(Screen::MainMenu),
            Self::RoomScan => Some(Screen::TestMode),
        }
    }
}

/// The Main Menu's entries, in display order.
const MAIN_MENU_ITEMS: [Item; 4] = [
    Item::ScreenEntry(ScreenEntry::SystemInfo),
    Item::Submenu(Submenu::Calibrate),
    Item::Submenu(Submenu::DriveMode),
    Item::Submenu(Submenu::TestMode),
];

/// The Calibrate submenu's entries, in display order.
const CALIBRATE_ITEMS: [Item; 3] = [
    Item::Procedure(Procedure::MotorCalibration),
    Item::Procedure(Procedure::MagCalibration),
    Item::Procedure(Procedure::DistanceCalibration),
];

/// The Drive Mode submenu's entries, in display order.
const DRIVE_MODE_ITEMS: [Item; 2] = [
    Item::Procedure(Procedure::CoastAndAvoid),
    Item::Procedure(Procedure::AttemptStraight),
];

/// The Test Mode submenu's entries, in display order.
///
/// Room Scan is the last entry: it opens the sensor's live radar rather than
/// running a test-mode task, but it shares the menu and its single-active guard.
const TEST_MODE_ITEMS: [Item; 7] = [
    Item::Procedure(Procedure::BasicMotor),
    Item::Procedure(Procedure::Turns),
    Item::Procedure(Procedure::StraightDrive),
    Item::Procedure(Procedure::ArcDrive),
    Item::Procedure(Procedure::Imu6Axis),
    Item::Procedure(Procedure::Imu9Axis),
    Item::ScreenEntry(ScreenEntry::RoomScan),
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
    /// The System Info status screen, rendered from the neutral snapshot.
    SystemInfo,
    /// A draggable value-entry screen for a distance flow.
    ValueEntry(ValueFlow),
    /// A running/status screen, whose content is supplied by [`StatusView`].
    ///
    /// Later tickets use this for a procedure's progress and sensor state. A
    /// live procedure offers a touch Stop in the header; a finished Result
    /// Report offers Back instead.
    Status,
    /// The Room Scan screen: the sensor's live spins drawn as a radar.
    ///
    /// It is not a running procedure: its header offers Back, and the firmware
    /// enables the `LiDAR` on entry and disables it on exit.
    RoomScan,
}

impl Screen {
    /// The Menu Entry whose destination this screen is, if any.
    ///
    /// Every screen but the Main Menu and the running screen is a Menu Entry's
    /// destination. The Main Menu is the root, and the running screen is shared by
    /// many Procedures, so its parent is held in the active [`StatusView`] and
    /// resolved by [`crate::Ui`].
    #[must_use]
    pub const fn entry(self) -> Option<Item> {
        match self {
            Self::MainMenu | Self::Status => None,
            Self::Calibrate => Some(Item::Submenu(Submenu::Calibrate)),
            Self::DriveMode => Some(Item::Submenu(Submenu::DriveMode)),
            Self::TestMode => Some(Item::Submenu(Submenu::TestMode)),
            Self::SystemInfo => Some(Item::ScreenEntry(ScreenEntry::SystemInfo)),
            Self::RoomScan => Some(Item::ScreenEntry(ScreenEntry::RoomScan)),
            Self::ValueEntry(ValueFlow::DistanceCalibration) => Some(Item::Procedure(Procedure::DistanceCalibration)),
            Self::ValueEntry(ValueFlow::AttemptStraight) => Some(Item::Procedure(Procedure::AttemptStraight)),
        }
    }

    /// The title shown in the header for this screen.
    ///
    /// Every screen that is a Menu Entry's destination takes its title from that
    /// entry, so the header and the button that opened it are one string.
    /// [`Screen::MainMenu`] is the root, so it names itself; [`Screen::Status`]
    /// has no title of its own — [`crate::Ui::title`] returns the dynamic title
    /// from the active [`StatusView`] instead, and this fallback is only used if
    /// the enum is matched on directly.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self.entry() {
            Some(entry) => entry.title(),
            // The two screens that are not a Menu Entry's destination name
            // themselves: the root menu, and the running screen, whose title
            // comes from its own content instead.
            None => match self {
                Self::MainMenu => "Main Menu",
                _ => "Running",
            },
        }
    }

    /// The entries shown as list buttons on this screen, in order.
    ///
    /// Each entry supplies its own [`Item::label`], so this ordered slice is both
    /// what the screen draws and what a caller resolves through
    /// [`Item::destination`].
    ///
    /// System Info, status, and value-entry screens are not lists, so they have
    /// none.
    #[must_use]
    pub const fn items(self) -> &'static [Item] {
        match self {
            Self::MainMenu => &MAIN_MENU_ITEMS,
            Self::Calibrate => &CALIBRATE_ITEMS,
            Self::DriveMode => &DRIVE_MODE_ITEMS,
            Self::TestMode => &TEST_MODE_ITEMS,
            Self::SystemInfo | Self::ValueEntry(_) | Self::Status | Self::RoomScan => &[],
        }
    }

    /// The screen Back returns to, or `None` on the Main Menu.
    ///
    /// Every screen that is an entry's destination takes its parent from that
    /// entry, so the tree has one parent relation rather than a second table.
    /// [`Screen::Status`] is shared by many Procedures, so its parent is held in
    /// the active [`StatusView`] and resolved by [`crate::Ui`]; this returns
    /// `None` for it.
    #[must_use]
    pub const fn parent(self) -> Option<Self> {
        match self.entry() {
            Some(entry) => entry.parent(),
            None => None,
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
}

/// The neutral content of a running/status screen.
///
/// The firmware maps its activity state onto this and calls
/// [`crate::Ui::show_status`] on entry and [`crate::Ui::set_status`] as progress
/// advances. `parent` is where the header action returns, `progress` is the
/// percent the widget draws as a bar when the procedure can report one, and
/// `finished` marks the view as a Result Report — a procedure that has already
/// completed or failed and only waits to be dismissed, rather than one still
/// running. The model never formats a number, so no dynamic text crosses this
/// boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StatusView {
    /// The header title.
    pub title: &'static str,
    /// The body line: the procedure's phase or the sensor's state.
    pub body: &'static str,
    /// Percent progress (0–100), drawn as a bar; `None` to draw no bar.
    pub progress: Option<u8>,
    /// The screen the header action returns to.
    pub parent: Screen,
    /// Whether this is a finished Result Report, whose header offers Back rather
    /// than Stop.
    pub finished: bool,
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
            finished: false,
        }
    }

    /// Build a status view for `procedure`, taking its title and the screen the
    /// header action returns to from the entry's own identity.
    ///
    /// This is the constructor the firmware uses while a Procedure runs, so it
    /// never restates either a title or a parent. A Procedure with no parent is
    /// rooted at the Main Menu.
    #[must_use]
    pub const fn for_procedure(procedure: Procedure, body: &'static str) -> Self {
        Self {
            title: procedure.label(),
            body,
            progress: None,
            parent: procedure.landing_screen(),
            finished: false,
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

    /// Mark the view as a finished Result Report, whose header offers Back.
    #[must_use]
    pub const fn finished(self) -> Self {
        Self { finished: true, ..self }
    }
}
