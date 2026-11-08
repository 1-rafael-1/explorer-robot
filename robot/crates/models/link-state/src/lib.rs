//! Neutral operator-facing link state for the robot's Link Integration.
//!
//! [`LinkState`] combines the Link Port's Bridge presence, the Remote [`Peer`], the latest
//! Bridge Status and the latest Bridge Control outcome. It derives the four operator-visible
//! link phases and decides whether a retained Bridge Status is stale, so the touch panel and,
//! later, the daemon can share one projection.
//!
//! The crate is `#![no_std]` and depends only on `heapless` for its fixed-capacity strings. It
//! deliberately names no wire, graphics or HAL type: the Link Task and the panel are adapters
//! that translate their own types into these neutral values. The `std` feature exists only for
//! the host test harness.

#![no_std]
#![warn(missing_docs)]

use heapless::String;

/// A Bridge Status older than this many milliseconds is stale.
pub const STALE_AFTER_MS: u64 = 2_000;

/// The maximum setup SSID length in bytes.
pub const SSID_CAPACITY: usize = 32;

/// The maximum setup passphrase length in bytes.
pub const PASSPHRASE_CAPACITY: usize = 63;

/// The number of characters in a pairing code (`ADR-0037`).
pub const PAIRING_CODE_CAPACITY: usize = 12;

/// The Link Port's Bridge-presence lifecycle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Presence {
    /// The Link Port is powered down or disabled.
    #[default]
    Off,
    /// The port is enabled but the Bridge has not asserted presence yet.
    WaitingForBoard,
    /// The Bridge is attached and its UART is ready.
    Ready,
}

impl Presence {
    /// Returns the panel label for this presence.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::WaitingForBoard => "Waiting for board",
            Self::Ready => "Ready",
        }
    }
}

/// Whether a Remote is currently the robot's peer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Peer {
    /// No valid PC-to-robot frame has arrived since the link was enabled.
    #[default]
    NoPeer,
    /// A valid frame arrived within the Deadman window.
    PeerLive,
}

impl Peer {
    /// Returns the panel label for this peer state.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::NoPeer => "No peer",
            Self::PeerLive => "Peer live",
        }
    }
}

/// The four operator-visible link states derived from presence and peer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LinkPhase {
    /// The link is disabled.
    Off,
    /// The port is enabled and the Bridge has not appeared.
    WaitingForBoard,
    /// The Bridge is present but no Remote has sent a valid frame.
    ReadyNoPeer,
    /// The Bridge is present and the Remote is live.
    PeerLive,
}

impl LinkPhase {
    /// Derives the phase from presence and peer, with presence taking precedence.
    #[must_use]
    pub const fn derive(presence: Presence, peer: Peer) -> Self {
        match presence {
            Presence::Off => Self::Off,
            Presence::WaitingForBoard => Self::WaitingForBoard,
            Presence::Ready => match peer {
                Peer::NoPeer => Self::ReadyNoPeer,
                Peer::PeerLive => Self::PeerLive,
            },
        }
    }

    /// Returns the panel label for this phase.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::WaitingForBoard => "Waiting for board",
            Self::ReadyNoPeer => "Ready",
            Self::PeerLive => "Peer live",
        }
    }
}

/// The Bridge's station association state.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Station {
    /// The station is not associated.
    #[default]
    Down,
    /// The station is attempting to associate.
    Connecting,
    /// The station is associated.
    Associated,
}

impl Station {
    /// Returns the panel label for this station state.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Down => "Down",
            Self::Connecting => "Connecting",
            Self::Associated => "Associated",
        }
    }
}

/// The three status flags the Bridge publishes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct BridgeFlags {
    /// The Bridge has stored usable station credentials.
    pub provisioned: bool,
    /// The Bridge's setup access point is open.
    pub setup_open: bool,
    /// The Bridge's station link is ready.
    pub link_ready: bool,
}

/// A neutral Bridge Status snapshot (`ADR-0042`).
///
/// The values mirror the Bridge's 1 Hz status payload without naming its wire ids.
/// [`BridgeStatus::rssi_dbm`] and [`BridgeStatus::channel`] are [`None`] when the Bridge
/// reports its unknown sentinel. The six counters are cumulative since the Bridge's boot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct BridgeStatus {
    /// The station `RSSI` in dBm, or [`None`] for unknown.
    pub rssi_dbm: Option<i8>,
    /// The station channel, or [`None`] for unknown.
    pub channel: Option<u8>,
    /// The station association state.
    pub station: Station,
    /// The provisioned, setup-open and link-ready flags.
    pub flags: BridgeFlags,
    /// Bytes dropped by the Bridge's UART receive path.
    pub uart_dropped_bytes: u32,
    /// DMA stalls observed by the Bridge.
    pub dma_stalls: u32,
    /// Frame CRC failures observed by the Bridge.
    pub crc_failures: u32,
    /// Resynchronisations after a framing error.
    pub resyncs: u32,
    /// Watchdog feeds.
    pub wdt_feeds: u32,
    /// Watchdog feeds withheld because a task was late.
    pub wdt_withheld: u32,
}

/// The Bridge's verdict on a Bridge Control request.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ControlStatus {
    /// The Bridge accepted and performed the action.
    Ok,
    /// The Bridge refused the action.
    Refused,
    /// The Bridge attempted the action and failed.
    Failed,
}

impl ControlStatus {
    /// Returns the panel label for this status.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Ok => "OK",
            Self::Refused => "Refused",
            Self::Failed => "Failed",
        }
    }
}

/// The setup access point's `SSID` and public passphrase.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SetupCredentials {
    /// The setup `SSID`, at most [`SSID_CAPACITY`] bytes.
    pub ssid: String<SSID_CAPACITY>,
    /// The setup passphrase, at most [`PASSPHRASE_CAPACITY`] bytes.
    pub passphrase: String<PASSPHRASE_CAPACITY>,
}

impl SetupCredentials {
    /// Builds the credentials, or returns [`None`] when either string exceeds its capacity.
    #[must_use]
    pub fn new(ssid: &str, passphrase: &str) -> Option<Self> {
        let Ok(ssid) = String::<SSID_CAPACITY>::try_from(ssid) else {
            return None;
        };
        let Ok(passphrase) = String::<PASSPHRASE_CAPACITY>::try_from(passphrase) else {
            return None;
        };
        Some(Self { ssid, passphrase })
    }
}

/// A twelve-character pairing code (`ADR-0037`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PairingCode {
    /// The code, at most [`PAIRING_CODE_CAPACITY`] characters.
    pub code: String<PAIRING_CODE_CAPACITY>,
}

impl PairingCode {
    /// Builds the code, or returns [`None`] when it exceeds [`PAIRING_CODE_CAPACITY`].
    #[must_use]
    pub fn new(code: &str) -> Option<Self> {
        let Ok(code) = String::<PAIRING_CODE_CAPACITY>::try_from(code) else {
            return None;
        };
        Some(Self { code })
    }
}

/// The detail of an answered Bridge Control request.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ControlReply {
    /// A forget-network reply carrying the raised setup access point's credentials.
    Setup {
        /// The Bridge's verdict.
        status: ControlStatus,
        /// The setup `SSID` and public passphrase.
        credentials: SetupCredentials,
    },
    /// A show-or-rotate pairing code reply.
    PairingCode {
        /// The Bridge's verdict.
        status: ControlStatus,
        /// The displayed pairing code.
        code: PairingCode,
    },
}

impl ControlReply {
    /// Returns the status the Bridge reported.
    #[must_use]
    pub const fn status(&self) -> ControlStatus {
        match self {
            Self::Setup { status, .. } | Self::PairingCode { status, .. } => *status,
        }
    }
}

/// The latest Bridge Control request's outcome.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum ControlOutcome {
    /// No request has been made since the last clear.
    #[default]
    Idle,
    /// A request is in flight.
    Pending,
    /// The Bridge answered the request.
    Answered(ControlReply),
    /// No acknowledgement arrived before the timeout.
    TimedOut,
}

impl ControlOutcome {
    /// Returns `true` while a request is in flight.
    #[must_use]
    pub const fn is_pending(&self) -> bool {
        matches!(self, Self::Pending)
    }

    /// Returns the panel label for this outcome.
    #[must_use]
    pub const fn label(&self) -> &'static str {
        match self {
            Self::Idle => "Idle",
            Self::Pending => "Pending",
            Self::Answered(_) => "Answered",
            Self::TimedOut => "Timed out",
        }
    }
}

/// A copy-on-read projection of [`LinkState`] at a given time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Snapshot {
    /// The derived link phase.
    pub phase: LinkPhase,
    /// The Bridge-presence lifecycle, before the peer is folded into the phase.
    pub presence: Presence,
    /// The current peer state.
    pub peer: Peer,
    /// The latest Bridge Status, even when stale.
    pub bridge: Option<BridgeStatus>,
    /// Whether the Bridge Status is older than [`STALE_AFTER_MS`].
    pub bridge_stale: bool,
    /// The latest Bridge Control outcome.
    pub control: ControlOutcome,
}

impl Snapshot {
    /// The disabled-link snapshot: `Off`, `NoPeer`, no Bridge Status and `Idle` control.
    ///
    /// This is the constant `Ui::new` starts from, so the panel has a valid neutral
    /// projection before the Link Task publishes anything.
    #[must_use]
    pub const fn off() -> Self {
        Self {
            phase: LinkPhase::Off,
            presence: Presence::Off,
            peer: Peer::NoPeer,
            bridge: None,
            bridge_stale: false,
            control: ControlOutcome::Idle,
        }
    }
}

/// The operator-facing link state: presence, peer, Bridge Status and control outcome.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct LinkState {
    /// The Link Port's Bridge-presence lifecycle.
    presence: Presence,
    /// Whether a Remote is live.
    peer: Peer,
    /// The latest Bridge Status, retained across no-peer periods.
    bridge: Option<BridgeStatus>,
    /// The monotonic time at which `bridge` was received.
    bridge_received_at_ms: Option<u64>,
    /// The latest Bridge Control outcome.
    control: ControlOutcome,
}

impl LinkState {
    /// Creates the disabled state: `Off`, `NoPeer`, no Bridge Status and `Idle` control.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            presence: Presence::Off,
            peer: Peer::NoPeer,
            bridge: None,
            bridge_received_at_ms: None,
            control: ControlOutcome::Idle,
        }
    }

    /// Sets the Bridge-presence lifecycle.
    pub const fn set_presence(&mut self, presence: Presence) {
        self.presence = presence;
    }

    /// Sets the peer state.
    pub const fn set_peer(&mut self, peer: Peer) {
        self.peer = peer;
    }

    /// Stores a Bridge Status and the monotonic time it was received.
    pub const fn set_bridge_status(&mut self, status: BridgeStatus, now_ms: u64) {
        self.bridge = Some(status);
        self.bridge_received_at_ms = Some(now_ms);
    }

    /// Replaces the Bridge Control outcome.
    pub fn set_control(&mut self, outcome: ControlOutcome) {
        self.control = outcome;
    }

    /// Clears everything for the Link Port's Disable path.
    ///
    /// The state returns to `Off` / `NoPeer`, the Bridge Status is dropped and the control
    /// outcome is `Idle`.
    pub fn clear(&mut self) {
        *self = Self::new();
    }

    /// Returns the current Bridge-presence lifecycle.
    #[must_use]
    pub const fn presence(&self) -> Presence {
        self.presence
    }

    /// Returns the current peer state.
    #[must_use]
    pub const fn peer(&self) -> Peer {
        self.peer
    }

    /// Returns the current Bridge Control outcome.
    #[must_use]
    pub const fn control(&self) -> &ControlOutcome {
        &self.control
    }

    /// Returns the phase derived from presence and peer.
    #[must_use]
    pub const fn phase(&self) -> LinkPhase {
        LinkPhase::derive(self.presence, self.peer)
    }

    /// Returns a copy-on-read snapshot evaluated at `now_ms`.
    ///
    /// A retained Bridge Status is stale when its age is greater than [`STALE_AFTER_MS`];
    /// exactly that age is still fresh. A stale status is still returned in
    /// [`Snapshot::bridge`].
    #[must_use]
    pub fn snapshot(&self, now_ms: u64) -> Snapshot {
        let bridge_stale = match (self.bridge, self.bridge_received_at_ms) {
            (Some(_), Some(received_at_ms)) => now_ms.saturating_sub(received_at_ms) > STALE_AFTER_MS,
            (Some(_), None) => true,
            (None, _) => false,
        };
        Snapshot {
            phase: self.phase(),
            presence: self.presence,
            peer: self.peer,
            bridge: self.bridge,
            bridge_stale,
            control: self.control.clone(),
        }
    }
}
