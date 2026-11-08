//! Link state module.
//!
//! Holds the operator-facing link projection in a private mutex, written by the
//! Link Task on change and at the Bridge's 1 Hz cadence and read by the panel
//! through a copy-on-read snapshot.
//!
//! # Lock order
//!
//! The link-state mutex is **outside** the sampler's state lock order by
//! construction: it is never taken while another state mutex is held, and the
//! Link Task never holds it across the UART or port-power awaits. A caller that
//! already holds another state lock must not call any accessor here.
//!
//! # Lifecycle
//!
//! [`clear`] is the Link Port's Disable path and the start of every enabled
//! period, so the panel returns to `Off` / `NoPeer` with no Bridge Status. A
//! Bridge reset is not a Disable: the Link Task only updates presence and peer,
//! so the retained Bridge Status and control outcome survive until the next
//! [`clear`].

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use link_state::{BridgeStatus, ControlOutcome, LinkState, Peer, Presence, Snapshot};

/// Global operator-facing link state, private; use the accessor functions.
static LINK_STATE: Mutex<CriticalSectionRawMutex, LinkState> = Mutex::new(LinkState::new());

/// Set the Link Port's Bridge-presence lifecycle.
pub async fn set_presence(presence: Presence) {
    LINK_STATE.lock().await.set_presence(presence);
}

/// Set whether a Remote is currently live.
pub async fn set_peer(peer: Peer) {
    LINK_STATE.lock().await.set_peer(peer);
}

/// Store the latest Bridge Status and the time it was received.
///
/// `now_ms` is the same monotonic uptime the snapshot reader uses, so the
/// status goes stale exactly [`link_state::STALE_AFTER_MS`] after it arrived.
pub async fn set_bridge_status(status: BridgeStatus, now_ms: u64) {
    LINK_STATE.lock().await.set_bridge_status(status, now_ms);
}

/// Replace the latest Bridge Control outcome.
///
/// The three panel actions publish `Pending` here and the Link Task publishes
/// `Answered` or `TimedOut` when the Bridge replies or the request expires.
pub async fn set_control(outcome: ControlOutcome) {
    LINK_STATE.lock().await.set_control(outcome);
}

/// Clear everything for the Link Port's Disable path and every enabled period.
pub async fn clear() {
    LINK_STATE.lock().await.clear();
}

/// Read a copy-on-read snapshot evaluated at `now_ms`.
pub async fn snapshot(now_ms: u64) -> Snapshot {
    LINK_STATE.lock().await.snapshot(now_ms)
}
