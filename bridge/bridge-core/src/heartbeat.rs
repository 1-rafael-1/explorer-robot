//! The task-liveness policy behind the Bridge's watchdog.
//!
//! The hardware watchdog is fed only when every covered task has checked in inside a bounded
//! window. A task that panics, spins or awaits forever stops beating; the feed then stops and the
//! RTC watchdog resets the board. This module owns the rules only: no clock, no timer and no
//! hardware. The firmware owns the shared set, the millisecond stamps and the `Rwdt`.
//!
//! The set is deliberately small and fixed. Each covered task owns one slot, so a stall cannot be
//! masked by a sibling worker that happens to share the same task function.

/// One task the Bridge's watchdog covers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CoveredTask {
    /// The UHCI RX DMA task: robot to PC.
    UartRx = 0,
    /// The UART TX task: PC to robot.
    UartTx = 1,
    /// The station `embassy-net` stack runner.
    NetStation = 2,
    /// The setup AP `embassy-net` stack runner.
    NetAccessPoint = 3,
    /// The Discovery Beacon task.
    Beacon = 4,
    /// The first Link Protocol connection worker.
    Client0 = 5,
    /// The second Link Protocol connection worker.
    Client1 = 6,
    /// The third Link Protocol connection worker.
    Client2 = 7,
}

impl CoveredTask {
    /// The number of slots.
    pub const COUNT: usize = 8;

    /// Every slot, in index order.
    pub const ALL: [Self; Self::COUNT] = [
        Self::UartRx,
        Self::UartTx,
        Self::NetStation,
        Self::NetAccessPoint,
        Self::Beacon,
        Self::Client0,
        Self::Client1,
        Self::Client2,
    ];

    /// Returns the slot's index into [`HeartbeatSet`]'s arrays.
    #[must_use]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// Returns the coverage bit for this task.
    const fn mask(self) -> u16 {
        1u16 << (self as u16)
    }

    /// Returns a short, stable label for the firmware's log lines and the watchdog coverage table.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::UartRx => "uart-rx",
            Self::UartTx => "uart-tx",
            Self::NetStation => "net-station",
            Self::NetAccessPoint => "net-ap",
            Self::Beacon => "beacon",
            Self::Client0 => "client-0",
            Self::Client1 => "client-1",
            Self::Client2 => "client-2",
        }
    }
}

/// The coverage mask and the last beat of every covered task.
///
/// The caller owns the clock: [`HeartbeatSet::beat`] and [`HeartbeatSet::stale`] take monotonic
/// millisecond stamps in whatever wrapping scale the firmware uses. Wrapping subtraction makes the
/// set correct across the `u32` millisecond boundary.
#[derive(Clone, Copy, Debug)]
pub struct HeartbeatSet {
    /// Bit `n` is set when the matching [`CoveredTask`] must beat.
    covered: u16,
    /// The last beat per slot, in the caller's millisecond scale.
    beats: [u32; CoveredTask::COUNT],
}

impl HeartbeatSet {
    /// An empty set: nothing is covered and every slot is at zero.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            covered: 0,
            beats: [0; CoveredTask::COUNT],
        }
    }

    /// Starts the liveness window: every slot's last beat becomes `now_ms`.
    ///
    /// The firmware seeds the set once, after spawning the covered tasks, so a slow boot cannot
    /// make them look stale before they have had a chance to run.
    pub const fn seed(&mut self, now_ms: u32) {
        self.beats = [now_ms; CoveredTask::COUNT];
    }

    /// Adds `task` to the set that must beat.
    pub const fn cover(&mut self, task: CoveredTask) {
        self.covered |= task.mask();
    }

    /// Returns whether `task` must beat.
    #[must_use]
    pub const fn is_covered(&self, task: CoveredTask) -> bool {
        self.covered & task.mask() != 0
    }

    /// Records a beat for `task` at `now_ms`.
    pub const fn beat(&mut self, task: CoveredTask, now_ms: u32) {
        self.beats[task.index()] = now_ms;
    }

    /// Returns the last beat recorded for `task`.
    #[must_use]
    pub const fn last_beat(&self, task: CoveredTask) -> u32 {
        self.beats[task.index()]
    }

    /// Returns the first covered task whose last beat is older than `max_age_ms`.
    ///
    /// A task is alive while its silence is at most `max_age_ms`; it is stale on the first
    /// millisecond beyond that.
    #[must_use]
    pub fn stale(&self, now_ms: u32, max_age_ms: u32) -> Option<CoveredTask> {
        CoveredTask::ALL
            .into_iter()
            .find(|task| self.is_covered(*task) && now_ms.wrapping_sub(self.last_beat(*task)) > max_age_ms)
    }

    /// Returns whether every covered task has beaten inside `max_age_ms`.
    #[must_use]
    pub fn all_alive(&self, now_ms: u32, max_age_ms: u32) -> bool {
        self.stale(now_ms, max_age_ms).is_none()
    }
}

impl Default for HeartbeatSet {
    fn default() -> Self {
        Self::new()
    }
}
