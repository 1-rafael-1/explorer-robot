//! The bounded Setup Mode window (ADR-0036).
//!
//! The Bridge raises a Setup Mode `SoftAP` with a captive portal when the credential record is
//! missing at boot, when the station link has been down for five continuous minutes since boot,
//! or when the robot's panel asks. A window lasts ten minutes and closes early thirty seconds
//! after a stable station association.
//!
//! The outage rule fires at most once per boot. An open window consumes that one opportunity once
//! the station has stayed down long enough, so closing the window does not immediately raise a
//! new one; exposure stays bounded. A boot that is provisioned while the unprovisioned window is
//! open therefore cannot later raise an outage window in that same boot — the panel's
//! `BRIDGE_CONTROL` action 0 remains the on-demand recovery path (ADR-0037).
//!
//! The window owns no clock, no I/O and no radio: the caller passes a monotonic millisecond
//! timestamp and reports whether the station link is up (associated and holding a DHCP address).

/// The lifetime of one Setup Mode window (ADR-0036's 10 minutes).
pub const WINDOW_MS: u32 = 600_000;

/// How long a stable station association keeps the window open (ADR-0036's 30 s).
pub const STABLE_ASSOCIATION_MS: u32 = 30_000;

/// How long the station link must stay down to raise the window (ADR-0036's 5 minutes).
pub const OUTAGE_MS: u32 = 300_000;

/// Half a `u32`'s range, used to compare timestamps across a wrap.
const HALF_RANGE: u32 = 1 << 31;

/// Why the Setup Mode window was raised.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RaiseReason {
    /// The credential record was missing or unusable at boot.
    Unprovisioned,
    /// The station link had been down for five continuous minutes.
    StationOutage,
    /// The robot's panel (or the portal's Forget button) asked for a window.
    PanelRequest,
    /// The stored credentials were forgotten.
    Forgotten,
}

/// Why the Setup Mode window closed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CloseReason {
    /// The ten-minute window elapsed.
    Expired,
    /// The station associated and stayed up for thirty seconds.
    StationStable,
}

/// A transition the caller may log or react to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetupEvent {
    /// The window opened.
    Opened(
        /// Why it opened.
        RaiseReason,
    ),
    /// The window closed.
    Closed(
        /// Why it closed.
        CloseReason,
    ),
}

/// The bounded Setup Mode window.
#[derive(Clone, Copy, Debug)]
pub struct SetupWindow {
    /// Whether the credential record is currently usable.
    provisioned: bool,
    /// The reason the window is open, or `None` while it is closed.
    reason: Option<RaiseReason>,
    /// When the current window opened, in the caller's millisecond scale.
    opened_at: u32,
    /// The last observed station link state.
    station_up: bool,
    /// When an open window must close after a stable association, if a rise armed it.
    stable_close_at: Option<u32>,
    /// When the station was last seen going down; `None` while it has been observed up.
    down_since: Option<u32>,
    /// Whether the once-per-boot outage raise has already been used.
    outage_consumed: bool,
}

/// Whether `now_ms` has reached `deadline_ms`, tolerating a `u32` wrap.
const fn reached(now_ms: u32, deadline_ms: u32) -> bool {
    now_ms.wrapping_sub(deadline_ms) < HALF_RANGE
}

impl SetupWindow {
    /// Creates the state at boot. Returns the event when the boot is unprovisioned.
    ///
    /// The window opens immediately with [`RaiseReason::Unprovisioned`] when `provisioned` is
    /// false. The station link is assumed down until the first [`SetupWindow::tick`] reports it
    /// up, so the outage clock starts at boot.
    #[must_use]
    pub const fn boot(provisioned: bool, now_ms: u32) -> (Self, Option<SetupEvent>) {
        let mut window = Self {
            provisioned,
            reason: None,
            opened_at: now_ms,
            station_up: false,
            stable_close_at: None,
            down_since: Some(now_ms),
            outage_consumed: false,
        };
        if provisioned {
            (window, None)
        } else {
            window.open(RaiseReason::Unprovisioned, now_ms);
            (window, Some(SetupEvent::Opened(RaiseReason::Unprovisioned)))
        }
    }

    /// Returns whether the window is open.
    #[must_use]
    pub const fn is_open(&self) -> bool {
        self.reason.is_some()
    }

    /// Returns why the window is open, or `None` while it is closed.
    #[must_use]
    pub const fn reason(&self) -> Option<RaiseReason> {
        self.reason
    }

    /// Returns the clamped time left in the ten-minute window at `now_ms`.
    ///
    /// Returns zero while the window is closed.
    #[must_use]
    pub const fn remaining_ms(&self, now_ms: u32) -> u32 {
        if self.reason.is_none() {
            return 0;
        }
        let elapsed = now_ms.wrapping_sub(self.opened_at);
        WINDOW_MS.saturating_sub(elapsed)
    }

    /// Updates the provisioned flag.
    ///
    /// Setting it false opens the window with [`RaiseReason::Forgotten`] when it is closed.
    /// Setting it true only records the flag and never closes an open window.
    pub const fn set_provisioned(&mut self, provisioned: bool, now_ms: u32) -> Option<SetupEvent> {
        self.provisioned = provisioned;
        if !self.provisioned && self.reason.is_none() {
            self.open(RaiseReason::Forgotten, now_ms);
            return Some(SetupEvent::Opened(RaiseReason::Forgotten));
        }
        None
    }

    /// Asks for a window on behalf of the panel or the portal's Forget button.
    ///
    /// When the window is closed it opens with [`RaiseReason::PanelRequest`]. When it is already
    /// open this changes nothing and returns `None`.
    pub const fn panel_request(&mut self, now_ms: u32, station_up: bool) -> Option<SetupEvent> {
        if self.reason.is_some() {
            return None;
        }
        self.observe_station(now_ms, station_up);
        self.open(RaiseReason::PanelRequest, now_ms);
        Some(SetupEvent::Opened(RaiseReason::PanelRequest))
    }

    /// Forgets the credentials and raises a window when it is closed.
    ///
    /// The provisioned flag is cleared either way; an already-open window keeps its reason and
    /// its timer, and the call returns `None`.
    pub const fn forget(&mut self, now_ms: u32, station_up: bool) -> Option<SetupEvent> {
        self.provisioned = false;
        if self.reason.is_some() {
            return None;
        }
        self.observe_station(now_ms, station_up);
        self.open(RaiseReason::Forgotten, now_ms);
        Some(SetupEvent::Opened(RaiseReason::Forgotten))
    }

    /// Advances the window's timers.
    ///
    /// `station_up` means the station is associated and holds a DHCP address. The call returns at
    /// most one event: a close wins over a raise when both are due, and the outage opportunity is
    /// preserved for the next call.
    pub const fn tick(&mut self, now_ms: u32, station_up: bool) -> Option<SetupEvent> {
        self.observe_station(now_ms, station_up);

        if self.reason.is_some() {
            self.consume_outage(now_ms);
            if let Some(deadline_ms) = self.stable_close_at
                && reached(now_ms, deadline_ms)
            {
                return Some(self.close(CloseReason::StationStable));
            }
            if reached(now_ms, self.opened_at.wrapping_add(WINDOW_MS)) {
                return Some(self.close(CloseReason::Expired));
            }
            return None;
        }

        self.maybe_raise_outage(now_ms)
    }

    /// Opens the window at `now_ms`, restarting the ten-minute timer.
    const fn open(&mut self, reason: RaiseReason, now_ms: u32) {
        self.reason = Some(reason);
        self.opened_at = now_ms;
        self.stable_close_at = None;
    }

    /// Closes the window, clearing its reason and stable-association timer.
    const fn close(&mut self, reason: CloseReason) -> SetupEvent {
        self.reason = None;
        self.stable_close_at = None;
        SetupEvent::Closed(reason)
    }

    /// Records a change in the station link, arming or cancelling the stable-association close.
    ///
    /// A rising edge arms the close only while the window is open, so a window opened while the
    /// station is already up relies on the ten-minute expiry instead.
    const fn observe_station(&mut self, now_ms: u32, station_up: bool) {
        match (self.station_up, station_up) {
            (false, true) => {
                if self.reason.is_some() {
                    self.stable_close_at = Some(now_ms.wrapping_add(STABLE_ASSOCIATION_MS));
                }
                self.down_since = None;
            }
            (true, false) => {
                self.stable_close_at = None;
                self.down_since = Some(now_ms);
            }
            _ => {}
        }
        self.station_up = station_up;
    }

    /// Marks the once-per-boot outage raise as used once the station has stayed down long enough
    /// while the window is open.
    const fn consume_outage(&mut self, now_ms: u32) {
        if self.outage_consumed {
            return;
        }
        let Some(down_since_ms) = self.down_since else {
            return;
        };
        if reached(now_ms, down_since_ms.wrapping_add(OUTAGE_MS)) {
            self.outage_consumed = true;
        }
    }

    /// Raises the window when the station has been down for five continuous minutes.
    const fn maybe_raise_outage(&mut self, now_ms: u32) -> Option<SetupEvent> {
        if self.outage_consumed {
            return None;
        }
        let Some(down_since_ms) = self.down_since else {
            return None;
        };
        if reached(now_ms, down_since_ms.wrapping_add(OUTAGE_MS)) {
            self.outage_consumed = true;
            self.open(RaiseReason::StationOutage, now_ms);
            return Some(SetupEvent::Opened(RaiseReason::StationOutage));
        }
        None
    }
}
