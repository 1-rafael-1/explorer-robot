//! The Discovery Beacon listener and the endpoint cache (ADR-0037, ADR-0039's `discovery` module).
//!
//! The Bridge broadcasts a small UDP datagram every 3 s; this module decodes it, deduplicates by
//! device id, caches the last endpoint per robot and selects the one robot v1 talks to. The
//! selection rule is ADR-0039's: the last-used device id when it is still seen, otherwise the only
//! Beacon, otherwise a deterministic pick named in the app's `detail`.
//!
//! A manual `host:port` wins over every Beacon, for networks that block broadcast. A Beacon with
//! a zero Link port or the Setup Mode flag is reported as Setup Mode and never connected to
//! (ADR-0037's amendment). Discovery is IPv4 only, matching the Beacon.

use std::{
    collections::BTreeMap,
    fmt, io,
    net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket},
    sync::{Arc, Mutex, MutexGuard, PoisonError},
};

use bridge_net::{
    beacon::{self, Beacon},
    identity::{DeviceId, Flags, Name},
    ports,
};
use tokio::{net::UdpSocket as TokioUdpSocket, sync::watch, task::JoinHandle};

use crate::clock;

/// The fixed UDP port the daemon listens on for the Discovery Beacon (ADR-0037).
pub const BEACON_PORT: u16 = ports::BEACON_UDP;

/// The Link Protocol TCP port used when an endpoint is entered by hand (ADR-0037).
pub const LINK_PORT: u16 = ports::LINK_TCP_DEFAULT;

/// How long a Beacon stays selectable after it was last heard.
///
/// The Bridge broadcasts every 3 s, so this is three missed Beacons plus a margin: an address
/// change is picked up within one window, and a silent Bridge stops being selected shortly after.
pub const BEACON_TTL_MS: u64 = 10_000;

/// The datagram receive buffer; larger than [`beacon::ENCODED_LEN`] so a later minor can extend it.
const RECEIVE_BUFFER: usize = 256;

/// The Beacon listener's configuration.
#[derive(Clone, Copy, Debug)]
pub struct DiscoveryConfig {
    /// The local address to bind; the unspecified IPv4 address on a LAN.
    pub bind: IpAddr,
    /// The local UDP port; zero is ephemeral and is what tests use.
    pub port: u16,
    /// The device id the operator last used, from the config store.
    pub last_device: Option<DeviceId>,
}

impl Default for DiscoveryConfig {
    fn default() -> Self {
        Self {
            bind: IpAddr::V4(Ipv4Addr::UNSPECIFIED),
            port: BEACON_PORT,
            last_device: None,
        }
    }
}

/// One Beacon heard recently, with the endpoint it advertises.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BeaconEntry {
    /// The MAC-derived device id.
    pub device_id: DeviceId,
    /// The display name.
    pub name: Name,
    /// The flag bits: provisioned, link-ready, setup-mode.
    pub flags: Flags,
    /// The advertised Link port; zero is the portal-only advertisement.
    pub tcp_port: u16,
    /// The endpoint to connect to: the datagram's source address and the advertised port.
    pub endpoint: SocketAddr,
    /// The daemon's receive time, in monotonic milliseconds.
    pub seen_ms: u64,
}

impl BeaconEntry {
    /// Returns whether the Beacon advertises Setup Mode rather than a connectable Link.
    ///
    /// ADR-0037's amendment defines `tcp_port = 0` as a portal-only Bridge; the Setup Mode flag
    /// covers the APSTA case. Either means the daemon must not connect.
    #[must_use]
    pub const fn setup_mode(&self) -> bool {
        self.tcp_port == beacon::PORTAL_ONLY || self.flags.contains(Flags::SETUP_MODE)
    }

    /// Returns the display name, or the hex device id when the name is empty.
    #[must_use]
    pub fn label(&self) -> String {
        let name = self.name.as_str();
        if name.is_empty() {
            hex(self.device_id)
        } else {
            name.to_owned()
        }
    }
}

/// The robot discovery selected, or why none was selected.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Choice {
    /// No fresh Beacon and no manual endpoint.
    None,
    /// A manual `host:port` endpoint; it covers networks that block broadcast and wins over a
    /// Beacon.
    Manual {
        /// The endpoint the operator supplied.
        endpoint: SocketAddr,
    },
    /// A fresh Beacon for the selected robot.
    Beacon {
        /// The cached Beacon entry, including the endpoint to connect to.
        entry: BeaconEntry,
        /// Why this one was picked; `None` for the last-used device or the only Beacon.
        detail: Option<String>,
    },
}

/// The Beacon cache and selection state.
struct DiscoveryState {
    /// The last entry per device id, keyed by its six bytes for a deterministic order.
    entries: BTreeMap<[u8; 6], BeaconEntry>,
    /// The last-used device id, from the config store.
    last_device: Option<DeviceId>,
    /// An explicit `host:port`, which wins over every Beacon.
    manual: Option<SocketAddr>,
    /// How long an entry stays selectable.
    ttl_ms: u64,
}

impl DiscoveryState {
    /// Records or replaces one Beacon, deduplicating by device id.
    fn observe(&mut self, entry: BeaconEntry) {
        self.entries.insert(entry.device_id.bytes(), entry);
    }

    /// Expires stale entries and applies ADR-0039's one-robot selection rule.
    fn choose(&mut self, now_ms: u64) -> Choice {
        if let Some(endpoint) = self.manual {
            return Choice::Manual { endpoint };
        }
        self.entries
            .retain(|_, entry| now_ms.saturating_sub(entry.seen_ms) <= self.ttl_ms);
        if self.entries.is_empty() {
            return Choice::None;
        }
        if let Some(device) = self.last_device
            && let Some(entry) = self.entries.get(&device.bytes()).copied()
        {
            return Choice::Beacon { entry, detail: None };
        }
        if self.entries.len() == 1 {
            return self
                .entries
                .values()
                .copied()
                .next()
                .map_or(Choice::None, |entry| Choice::Beacon { entry, detail: None });
        }
        // Several robots and no history: prefer a connectable one, then pick deterministically and
        // name it in the app's detail. The map's device-id order makes the pick reproducible.
        let chosen = self
            .entries
            .values()
            .copied()
            .find(|entry| !entry.setup_mode())
            .or_else(|| self.entries.values().copied().next());
        chosen.map_or(Choice::None, |entry| {
            let detail = format!("Several robots found; using {}.", entry.label());
            Choice::Beacon {
                entry,
                detail: Some(detail),
            }
        })
    }
}

/// A pre-bound Beacon listener; bind it before starting a Bridge Stub in tests so the stub can
/// target its ephemeral port.
pub struct Discovery {
    /// The standard socket; it stays synchronous until [`Discovery::spawn`].
    socket: UdpSocket,
    /// The configuration, including the last-used device id.
    config: DiscoveryConfig,
}

impl Discovery {
    /// Binds the listener without starting it.
    ///
    /// # Errors
    ///
    /// Returns the bind or non-blocking error from the socket.
    pub fn bind(config: DiscoveryConfig) -> io::Result<Self> {
        let socket = UdpSocket::bind(SocketAddr::new(config.bind, config.port))?;
        socket.set_nonblocking(true)?;
        Ok(Self { socket, config })
    }

    /// Returns the local address the listener bound.
    ///
    /// # Errors
    ///
    /// Returns the socket's `local_addr` error.
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// Spawns the receive loop on the current tokio runtime.
    ///
    /// # Errors
    ///
    /// Returns the socket's local-address or runtime-registration error.
    pub fn spawn(self) -> io::Result<DiscoveryHandle> {
        let local_addr = self.socket.local_addr()?;
        let socket = TokioUdpSocket::from_std(self.socket)?;
        let state = Arc::new(Mutex::new(DiscoveryState {
            entries: BTreeMap::new(),
            last_device: self.config.last_device,
            manual: None,
            ttl_ms: BEACON_TTL_MS,
        }));
        let listener_state = Arc::clone(&state);
        let (stop, mut stopped) = watch::channel(false);
        let join = tokio::spawn(async move {
            let mut buffer = [0u8; RECEIVE_BUFFER];
            loop {
                tokio::select! {
                    _ = stopped.changed() => return,
                    received = socket.recv_from(&mut buffer) => {
                        let Ok((len, from)) = received else {
                            continue;
                        };
                        if !from.ip().is_ipv4() {
                            continue;
                        }
                        let Ok(beacon) = Beacon::decode(&buffer[..len]) else {
                            continue;
                        };
                        // ADR-0037: the Beacon's major lets a daemon filter a future layout rather
                        // than misparse it. Ignore it entirely: do not cache and do not connect.
                        if beacon.major != remote_protocol::PROTOCOL_MAJOR {
                            continue;
                        }
                        let entry = BeaconEntry {
                            device_id: beacon.device_id,
                            name: beacon.name,
                            flags: beacon.flags,
                            tcp_port: beacon.tcp_port,
                            endpoint: SocketAddr::new(from.ip(), beacon.tcp_port),
                            seen_ms: clock::now_ms(),
                        };
                        if let Ok(mut state) = listener_state.lock() {
                            state.observe(entry);
                        }
                    }
                }
            }
        });
        Ok(DiscoveryHandle {
            inner: Arc::new(DiscoveryInner {
                state: Arc::clone(&state),
                local_addr,
                stop,
                join: Mutex::new(Some(join)),
            }),
        })
    }
}

/// A cloneable handle to a running Beacon listener.
#[derive(Clone)]
pub struct DiscoveryHandle {
    /// The shared listener state and task.
    inner: Arc<DiscoveryInner>,
}

/// The state one handle shares with its clones and the receive loop.
struct DiscoveryInner {
    /// The Beacon cache.
    state: Arc<Mutex<DiscoveryState>>,
    /// The bound address, kept so tests can target the listener.
    local_addr: SocketAddr,
    /// The stop signal.
    stop: watch::Sender<bool>,
    /// The receive task, taken by [`DiscoveryHandle::wait`].
    join: Mutex<Option<JoinHandle<()>>>,
}

impl DiscoveryHandle {
    /// Returns the local address the listener bound.
    #[must_use]
    pub fn local_addr(&self) -> SocketAddr {
        self.inner.local_addr
    }

    /// Applies ADR-0039's selection rule at `now_ms`.
    #[must_use]
    pub fn choose(&self, now_ms: u64) -> Choice {
        self.state().choose(now_ms)
    }

    /// Returns every fresh Beacon, for tracing and tests.
    #[must_use]
    pub fn entries(&self, now_ms: u64) -> Vec<BeaconEntry> {
        let state = self.state();
        let ttl = state.ttl_ms;
        state
            .entries
            .values()
            .filter(|entry| now_ms.saturating_sub(entry.seen_ms) <= ttl)
            .copied()
            .collect()
    }

    /// Sets or clears the manual `host:port` endpoint.
    pub fn set_manual(&self, endpoint: Option<SocketAddr>) {
        self.state().manual = endpoint;
    }

    /// Sets or clears the last-used device id the selection prefers.
    pub fn set_last_device(&self, device: Option<DeviceId>) {
        self.state().last_device = device;
    }

    /// Asks the receive loop to stop.
    pub fn stop(&self) {
        let _ = self.inner.stop.send(true);
    }

    /// Waits for the receive loop to end.
    pub async fn wait(&self) {
        let join = self.inner.join.lock().ok().and_then(|mut join| join.take());
        if let Some(join) = join {
            let _ = join.await;
        }
    }

    /// Locks the shared discovery state.
    fn state(&self) -> MutexGuard<'_, DiscoveryState> {
        self.inner.state.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

impl fmt::Debug for DiscoveryHandle {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("DiscoveryHandle")
            .field("local_addr", &self.inner.local_addr)
            .finish_non_exhaustive()
    }
}

/// Returns the lower-case hex encoding of a device id.
fn hex(device: DeviceId) -> String {
    let mut text = String::with_capacity(12);
    for byte in device.bytes() {
        use fmt::Write as _;
        let _ = write!(text, "{byte:02x}");
    }
    text
}

#[cfg(test)]
mod tests {
    use super::{BEACON_PORT, BEACON_TTL_MS, LINK_PORT};

    /// Pins ADR-0037's shared port constants.
    #[test]
    fn the_ports_are_the_adr_defaults() {
        assert_eq!(BEACON_PORT, 9001);
        assert_eq!(LINK_PORT, 9000);
        assert_eq!(BEACON_TTL_MS, 10_000);
    }
}
