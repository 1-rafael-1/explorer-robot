//! The process wiring the daemon binary and the end-to-end test share: config → discovery → app
//! server, and the `AppHooks` that open and release the Bridge Session (ADR-0039).
//!
//! [`Host::start`] is the library entry point the binary and the end-to-end test share. It binds
//! the app listener, starts the Beacon listener, installs the [`AppHooks`] that ask the link to
//! connect on the first app authentication, and returns a [`HostHandle`] carrying the URL and the
//! graceful-shutdown handle. The manager keeps the ADR-0039 rules: the Session opens only while an
//! app is authenticated, the last disconnect holds it for the 5 s post-app grace, and a manual
//! `host:port` wins over discovery.

use std::{
    fmt, io,
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::{Arc, Mutex},
    time::Duration,
};

use tokio::{
    net::{TcpListener, lookup_host},
    sync::{mpsc, watch},
    task::JoinHandle,
    time::{self, Instant},
};

use crate::{
    app::{App, AppHooks, Assets, ShutdownHandle, TokenGate},
    clock,
    config::{ConfigError, Store},
    discovery::{Choice, Discovery, DiscoveryHandle},
    link::{Link, LinkConfig, LinkError, LinkHandle, LinkState},
    relay::Relay,
    schema::{Connection, ConnectionState},
};

/// The app-facing HTTP port ADR-0039 serves by default.
pub const DEFAULT_HTTP_PORT: u16 = 8765;

/// The post-app grace before the Bridge Session is released (ADR-0039's 5 s).
pub const POST_APP_GRACE: Duration = Duration::from_secs(5);

/// How long a wanted Session waits for a Beacon before the app is told `no_robots`.
pub const NO_ROBOTS_AFTER_MS: u64 = 3_000;

/// How often the manager revisits discovery, the link state and its timers.
const MANAGER_POLL: Duration = Duration::from_millis(200);

/// Why the host could not start.
#[derive(Debug)]
pub enum HostError {
    /// The app listener, the Beacon listener or the config file failed.
    Io(io::Error),
    /// The token store could not be read or written.
    Config(ConfigError),
    /// The once-per-process Session epoch could not be generated.
    Link(LinkError),
}

impl fmt::Display for HostError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "the daemon could not start: {error}"),
            Self::Config(error) => write!(formatter, "{error}"),
            Self::Link(error) => write!(formatter, "{error}"),
        }
    }
}

impl std::error::Error for HostError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Config(error) => Some(error),
            Self::Link(error) => Some(error),
        }
    }
}

impl From<io::Error> for HostError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

impl From<ConfigError> for HostError {
    fn from(error: ConfigError) -> Self {
        Self::Config(error)
    }
}

impl From<LinkError> for HostError {
    fn from(error: LinkError) -> Self {
        Self::Link(error)
    }
}

/// The wired pieces the binary supplies to [`Host::start`].
pub struct Host {
    /// The pre-bound Beacon listener; binding it first lets a test point the Bridge Stub at it.
    pub discovery: Discovery,
    /// The per-user token store and endpoint cache.
    pub store: Arc<Store>,
    /// The SPA's bytes: the embedded folder by default, a disk folder for `--assets`.
    pub assets: Assets,
    /// Whether the browser test suite under `tests/` may be served (`--serve-tests`).
    pub serve_tests: bool,
    /// The app-facing bind address; loopback unless the operator opts into the LAN.
    pub bind: IpAddr,
    /// The app-facing port; `None` tries [`DEFAULT_HTTP_PORT`] and falls back to an ephemeral port.
    pub port: Option<u16>,
    /// Whether to open the default browser once the URL is known.
    pub open_browser: bool,
    /// Whether to trace discovery and Session changes to stderr.
    pub verbose: bool,
}

impl Host {
    /// Starts the discovery listener, the app server and the Session manager.
    ///
    /// # Errors
    ///
    /// Returns [`HostError`] when a socket cannot be bound, the config cannot be used or the
    /// process epoch cannot be generated.
    pub async fn start(self) -> Result<HostHandle, HostError> {
        let Self {
            discovery,
            store,
            assets,
            serve_tests,
            bind,
            port,
            open_browser,
            verbose,
        } = self;

        let discovery = discovery.spawn()?;
        let epoch = crate::link::random_epoch()?;
        let relay = Arc::new(Mutex::new(Relay::new()));

        let (control, commands) = mpsc::unbounded_channel();
        let (manager_stop, manager_stopped) = watch::channel(false);
        let manager = Manager {
            relay: Arc::clone(&relay),
            store: Arc::clone(&store),
            discovery: discovery.clone(),
            epoch,
            verbose,
            control: commands,
            link: None,
            manual: None,
            cached: store.last_endpoint(),
            active: false,
            auto_connect: true,
            acquired_ms: None,
            grace_until: None,
            blocked: None,
            retry_blocked: false,
            opened: None,
            detail: None,
            robot: None,
            used_cached: false,
            last_applied: None,
            last_connection: None,
            retired: Vec::new(),
        };
        let manager_task = tokio::spawn(manager.run(manager_stopped));

        let gate: Arc<dyn TokenGate> = store;
        let app = App::new(Arc::clone(&relay), gate)
            .with_hooks(Arc::new(SessionHooks { control }))
            .with_assets(assets)
            .with_tests(serve_tests);
        let listener = bind_listener(bind, port).await?;
        let addr = listener.local_addr()?;
        let app_shutdown = app.shutdown_handle();
        let app_task = tokio::spawn(app.serve_with_shutdown(listener, std::future::pending::<()>()));
        let url = format!("http://{}/", authority(addr));

        if open_browser && let Err(error) = open::that(&url) {
            eprintln!("remote-daemon: the browser could not be opened: {error}");
        }

        Ok(HostHandle {
            addr,
            url,
            app_shutdown,
            manager_stop,
            discovery,
            manager: manager_task,
            app: app_task,
        })
    }
}

/// A running daemon: its address, URL and the handles that stop it.
pub struct HostHandle {
    /// The address the app listener bound.
    addr: SocketAddr,
    /// The URL a browser opens.
    url: String,
    /// The app server's graceful-shutdown handle (close code 1001).
    app_shutdown: ShutdownHandle,
    /// The Session manager's stop signal.
    manager_stop: watch::Sender<bool>,
    /// The Beacon listener.
    discovery: DiscoveryHandle,
    /// The manager task.
    manager: JoinHandle<()>,
    /// The app server task.
    app: JoinHandle<io::Result<()>>,
}

impl HostHandle {
    /// Returns the address the app listener bound.
    #[must_use]
    pub const fn addr(&self) -> SocketAddr {
        self.addr
    }

    /// Returns the URL the app is served at.
    #[must_use]
    pub fn url(&self) -> &str {
        &self.url
    }

    /// Stops the daemon gracefully and waits for its tasks.
    ///
    /// The app server closes every WebSocket with **1001**, the manager releases the Bridge
    /// Session and the Beacon listener stops. The app's result is returned.
    ///
    /// # Errors
    ///
    /// Returns the app server's error, if it failed while serving.
    pub async fn stop(self) -> io::Result<()> {
        self.app_shutdown.shutdown();
        let _ = self.manager_stop.send(true);
        self.discovery.stop();
        let result = match self.app.await {
            Ok(result) => result,
            Err(error) => Err(io::Error::other(error)),
        };
        self.discovery.wait().await;
        let _ = self.manager.await;
        result
    }
}

/// Starts the daemon, prints its URL and serves until Ctrl+C.
///
/// # Errors
///
/// Returns [`HostError`] when startup fails or the app server fails while serving.
pub async fn run(host: Host) -> Result<(), HostError> {
    let handle = host.start().await?;
    println!("{}", handle.url());
    if let Err(error) = tokio::signal::ctrl_c().await {
        eprintln!("remote-daemon: the interrupt handler failed: {error}");
    }
    handle.stop().await?;
    Ok(())
}

/// Binds the app listener, applying ADR-0039's ephemeral fallback for the default port.
async fn bind_listener(bind: IpAddr, port: Option<u16>) -> io::Result<TcpListener> {
    match port {
        Some(port) => TcpListener::bind((bind, port)).await,
        None => match TcpListener::bind((bind, DEFAULT_HTTP_PORT)).await {
            Ok(listener) => Ok(listener),
            Err(error) if error.kind() == io::ErrorKind::AddrInUse => TcpListener::bind((bind, 0)).await,
            Err(error) => Err(error),
        },
    }
}

/// Returns the authority a browser can open for `addr` (an unspecified bind means localhost).
fn authority(addr: SocketAddr) -> String {
    match addr.ip() {
        IpAddr::V4(ip) if ip.is_unspecified() => format!("127.0.0.1:{}", addr.port()),
        IpAddr::V6(ip) if ip.is_unspecified() => format!("[::1]:{}", addr.port()),
        IpAddr::V6(ip) => format!("[{ip}]:{}", addr.port()),
        IpAddr::V4(ip) => format!("{ip}:{}", addr.port()),
    }
}

/// The Session manager's control messages from the app hooks.
enum Control {
    /// The first app connection authenticated; a Session is wanted.
    Acquire,
    /// The last app connection closed; hold the Session through the grace.
    Release,
    /// An app asked to disconnect; release the Session and stop auto-connect.
    Disconnect,
    /// An app supplied a manual `host:port`.
    Manual(String, u16),
    /// An app submitted a new pairing code: retry a fatal state with it.
    Repair,
}

/// The [`AppHooks`] the app adapter calls; they forward to the Session manager.
struct SessionHooks {
    /// The manager's control channel.
    control: mpsc::UnboundedSender<Control>,
}

impl AppHooks for SessionHooks {
    fn on_first_auth(&self) {
        let _ = self.control.send(Control::Acquire);
    }

    fn on_last_disconnect(&self) {
        let _ = self.control.send(Control::Release);
    }

    fn on_disconnect(&self) {
        let _ = self.control.send(Control::Disconnect);
    }

    fn on_manual_endpoint(&self, host: &str, port: u16) {
        let _ = self.control.send(Control::Manual(host.to_owned(), port));
    }

    fn on_pair(&self) {
        let _ = self.control.send(Control::Repair);
    }
}

/// What the manager is about to open a Session against.
struct Opening {
    /// The Bridge endpoint.
    endpoint: SocketAddr,
    /// The token to authenticate with, if one is stored.
    token: Option<String>,
    /// The discovery choice that produced the endpoint, for the fatal-state block.
    choice: Option<Choice>,
    /// The app-facing detail to attach to the Session.
    detail: Option<String>,
    /// The robot's label, when a Beacon named it.
    robot: Option<String>,
    /// Whether this is the cached endpoint rather than a fresh Beacon.
    cached: bool,
}

/// Owns the robot Session and the timers ADR-0039 fixes around it.
// The flags are the ADR's independent facts (an app is live, discovery may auto-connect, the
// endpoint is the cached one, tracing is on); a state enum would hide rather than clarify them.
#[allow(clippy::struct_excessive_bools)]
struct Manager {
    /// The relay both adapters share.
    relay: Arc<Mutex<Relay>>,
    /// The token store and endpoint cache.
    store: Arc<Store>,
    /// The Beacon listener and endpoint cache.
    discovery: DiscoveryHandle,
    /// The Session epoch, chosen once per process.
    epoch: u32,
    /// Whether to trace state changes.
    verbose: bool,
    /// The control channel from the app hooks.
    control: mpsc::UnboundedReceiver<Control>,
    /// The running link, when a Session is wanted.
    link: Option<LinkHandle>,
    /// An app-supplied endpoint, which wins over discovery.
    manual: Option<SocketAddr>,
    /// The endpoint cached from the previous process run, tried before waiting for a Beacon.
    cached: Option<SocketAddr>,
    /// Whether at least one app is authenticated.
    active: bool,
    /// Whether discovery may open a Session automatically.
    auto_connect: bool,
    /// When the current acquisition began, for the `no_robots` deadline.
    acquired_ms: Option<u64>,
    /// When the post-app grace expires.
    grace_until: Option<Instant>,
    /// The identity of a choice whose fatal state must not be retried unchanged.
    blocked: Option<ChoiceKey>,
    /// Whether a fatal state blocked the current intent until the operator acts or the world changes.
    retry_blocked: bool,
    /// The choice the running link was opened for.
    opened: Option<Choice>,
    /// The detail attached to the running Session.
    detail: Option<String>,
    /// The label of the robot the running Session targets.
    robot: Option<String>,
    /// Whether the running link is the cached endpoint rather than a fresh Beacon.
    used_cached: bool,
    /// The last link state applied to the relay.
    last_applied: Option<LinkState>,
    /// The last connection message published, so an unchanged state is not repeated.
    last_connection: Option<Connection>,
    /// Link waiters retained so shutdown can join them.
    retired: Vec<JoinHandle<()>>,
}

impl Manager {
    /// Runs the control loop until shutdown.
    async fn run(mut self, mut stop: watch::Receiver<bool>) {
        let mut interval = time::interval(MANAGER_POLL);
        interval.set_missed_tick_behavior(time::MissedTickBehavior::Delay);
        loop {
            tokio::select! {
                _ = stop.changed() => break,
                command = self.control.recv() => {
                    let Some(command) = command else {
                        break;
                    };
                    if !self.handle_command(command).await {
                        break;
                    }
                }
                _ = interval.tick() => {
                    let now_ms = clock::now_ms();
                    self.observe_link(now_ms);
                    self.tick(now_ms);
                }
            }
        }
        self.release_now();
        for task in self.retired.drain(..) {
            let _ = task.await;
        }
    }

    /// Applies one control message; returns `false` when the manager should stop.
    async fn handle_command(&mut self, command: Control) -> bool {
        match command {
            Control::Acquire => {
                self.active = true;
                self.auto_connect = true;
                self.grace_until = None;
                self.acquired_ms = Some(clock::now_ms());
                // A new authenticated tab is fresh operator intent: a previously fatal state may
                // be retried (for example after the operator re-paired through the LAN gate).
                self.blocked = None;
                self.retry_blocked = false;
                self.trace("an app authenticated; a Session is wanted");
            }
            Control::Release => {
                self.active = false;
                self.grace_until = Some(Instant::now() + POST_APP_GRACE);
                self.trace("the last app closed; holding the Session through the grace");
            }
            Control::Disconnect => {
                self.active = false;
                self.auto_connect = false;
                self.grace_until = None;
                self.stop_link();
                self.publish(ConnectionState::Searching, clock::now_ms(), None, None, None);
                self.trace("an app disconnected; the Session is released");
            }
            Control::Manual(host, port) => {
                self.active = true;
                self.auto_connect = true;
                self.blocked = None;
                self.retry_blocked = false;
                self.set_manual(&host, port).await;
            }
            Control::Repair => {
                self.active = true;
                self.auto_connect = true;
                self.acquired_ms = Some(clock::now_ms());
                // A new code is explicit operator intent: it clears the fatal-state block and
                // replaces a link that was opened with the old token.
                self.blocked = None;
                self.retry_blocked = false;
                self.stop_link();
                self.trace("an app re-paired; retrying the Session");
            }
        }
        true
    }

    /// Advances the grace timer and opens a Session when one is wanted and missing.
    fn tick(&mut self, now_ms: u64) {
        if !self.active && self.grace_until.is_some_and(|deadline| Instant::now() >= deadline) {
            self.grace_until = None;
            self.stop_link();
            // Do not leave a stale `connected` in the relay's lane for the next tab.
            self.publish(ConnectionState::Searching, now_ms, None, None, None);
            self.trace("the post-app grace elapsed; the Session is released");
        }
        if self.active && self.auto_connect && self.link.is_none() {
            self.try_open(now_ms);
        }
    }

    /// Reads the link's published state each tick and applies state changes once.
    fn observe_link(&mut self, now_ms: u64) {
        let state = self.link.as_ref().map(LinkHandle::state);
        if let Some(state) = state
            && self.last_applied.as_ref() != Some(&state)
        {
            self.apply(state, now_ms);
        }
        if self.link.as_ref().is_some_and(LinkHandle::is_finished) {
            let finished = self.link.take();
            if let Some(link) = finished {
                let state = link.state();
                if self.last_applied.as_ref() != Some(&state) {
                    self.apply(state, now_ms);
                }
                self.trace("the link task finished");
            }
        }
    }

    /// Chooses an endpoint — manual, cached or Beacon — and opens the Session.
    fn try_open(&mut self, now_ms: u64) {
        if let Some(endpoint) = self.manual {
            if self.retry_blocked {
                return;
            }
            let token = self.store.token_for(None);
            self.open(
                Opening {
                    endpoint,
                    token,
                    choice: None,
                    detail: Some(String::from("Manual endpoint")),
                    robot: None,
                    cached: false,
                },
                now_ms,
            );
            return;
        }
        let choice = self.discovery.choose(now_ms);
        if self.retry_blocked {
            // A fatal state stays surfaced until the operator acts. A different Beacon (the Setup
            // Mode flag clearing, or another robot appearing) is a changed world and may retry.
            if matches!(choice, Choice::Beacon { .. }) && self.blocked != choice_key(&choice) {
                self.blocked = None;
                self.retry_blocked = false;
            } else {
                return;
            }
        }
        match &choice {
            Choice::None => {
                if let Some(endpoint) = self.cached {
                    let token = self.store.token_for(self.store.last_device());
                    self.open(
                        Opening {
                            endpoint,
                            token,
                            choice: None,
                            detail: None,
                            robot: None,
                            cached: true,
                        },
                        now_ms,
                    );
                } else {
                    self.publish_waiting(now_ms);
                }
            }
            Choice::Manual { endpoint } => {
                let token = self.store.token_for(None);
                self.open(
                    Opening {
                        endpoint: *endpoint,
                        token,
                        choice: None,
                        detail: Some(String::from("Manual endpoint")),
                        robot: None,
                        cached: false,
                    },
                    now_ms,
                );
            }
            Choice::Beacon { entry, detail } => {
                if self.blocked == choice_key(&choice) {
                    return;
                }
                if entry.setup_mode() {
                    self.publish(
                        ConnectionState::SetupMode,
                        now_ms,
                        Some(String::from(
                            "The robot is in Setup Mode; connect it to the home network first.",
                        )),
                        None,
                        None,
                    );
                    self.blocked = choice_key(&choice);
                    return;
                }
                let token = self.store.token_for(Some(entry.device_id));
                if token.is_none() && self.store.last_device() != Some(entry.device_id) {
                    // Remember which robot the operator is being asked to pair with, even before a
                    // token exists, so the app's `pair` stores the code under this device.
                    let _ = self.store.set_last(entry.endpoint, Some(entry.device_id));
                }
                let endpoint = entry.endpoint;
                let robot = entry.label();
                let detail = detail.clone();
                self.discovery.set_last_device(Some(entry.device_id));
                self.open(
                    Opening {
                        endpoint,
                        token,
                        choice: Some(choice.clone()),
                        detail,
                        robot: Some(robot),
                        cached: false,
                    },
                    now_ms,
                );
            }
        }
    }

    /// Spawns a link for `opening`, or reports why it cannot.
    fn open(&mut self, opening: Opening, now_ms: u64) {
        if self.link.is_some() {
            return;
        }
        let Opening {
            endpoint,
            token,
            choice,
            detail,
            robot,
            cached,
        } = opening;
        let Some(token) = token else {
            self.publish(
                ConnectionState::PairingRequired,
                now_ms,
                Some(String::from(
                    "No pairing token is stored; pair from the panel's Link screen.",
                )),
                None,
                None,
            );
            return;
        };
        let device = choice.as_ref().and_then(device_of);
        let _ = self.store.set_last(endpoint, device);
        let config = LinkConfig::with_epoch(endpoint, token, self.epoch);
        self.link = Some(Link::new(config, Arc::clone(&self.relay)).spawn());
        self.opened = choice;
        self.detail = detail;
        self.robot = robot;
        self.used_cached = cached;
        self.blocked = None;
        self.retry_blocked = false;
        self.last_applied = None;
        self.trace(&format!("opening a Session to {endpoint}"));
        self.publish(ConnectionState::Connecting, now_ms, self.detail.clone(), None, None);
    }

    /// Maps a [`LinkState`] onto the app's `connection` message and the manager's own bookkeeping.
    fn apply(&mut self, state: LinkState, now_ms: u64) {
        self.last_applied = Some(state.clone());
        match state {
            LinkState::Starting => {
                self.publish(ConnectionState::Searching, now_ms, self.detail.clone(), None, None);
            }
            LinkState::Connecting => {
                self.publish(ConnectionState::Connecting, now_ms, self.detail.clone(), None, None);
            }
            LinkState::Connected { boot_id } => {
                self.publish(
                    ConnectionState::Connected,
                    now_ms,
                    self.detail.clone(),
                    None,
                    Some(boot_id),
                );
            }
            LinkState::Reconnecting { retry_in_ms } => {
                if self.used_cached {
                    self.stop_link();
                    self.cached = None;
                    self.trace("the cached endpoint did not answer; waiting for a Beacon");
                    self.publish(ConnectionState::Searching, now_ms, None, None, None);
                } else {
                    self.publish(
                        ConnectionState::Reconnecting,
                        now_ms,
                        self.detail.clone(),
                        Some(retry_in_ms),
                        None,
                    );
                }
            }
            LinkState::Busy { retry_in_ms } => {
                self.publish(
                    ConnectionState::Busy,
                    now_ms,
                    self.detail.clone(),
                    Some(retry_in_ms),
                    None,
                );
            }
            LinkState::Unauthorized { detail } => {
                self.blocked = self.opened.as_ref().and_then(choice_key);
                self.retry_blocked = true;
                self.cached = None;
                self.publish(ConnectionState::Unauthorized, now_ms, Some(detail), None, None);
            }
            LinkState::SetupMode { detail } => {
                self.blocked = self.opened.as_ref().and_then(choice_key);
                self.retry_blocked = true;
                self.cached = None;
                self.publish(ConnectionState::SetupMode, now_ms, Some(detail), None, None);
            }
            LinkState::VersionMismatch { detail } => {
                // ADR-0038 has no version state; ADR-0039 surfaces it as `unauthorized` + detail.
                self.blocked = self.opened.as_ref().and_then(choice_key);
                self.retry_blocked = true;
                self.cached = None;
                self.publish(ConnectionState::Unauthorized, now_ms, Some(detail), None, None);
            }
            LinkState::Stopped => {}
        }
    }

    /// Publishes the honest waiting state: `searching`, then `no_robots` after the deadline.
    fn publish_waiting(&mut self, now_ms: u64) {
        let elapsed = self.acquired_ms.map_or(0, |start| now_ms.saturating_sub(start));
        if elapsed >= NO_ROBOTS_AFTER_MS {
            self.publish(
                ConnectionState::NoRobots,
                now_ms,
                Some(String::from("No robots found; enter a host:port.")),
                None,
                None,
            );
        } else {
            self.publish(ConnectionState::Searching, now_ms, None, None, None);
        }
    }

    /// Sends `state` to the relay unless it is unchanged from the last message.
    fn publish(
        &mut self,
        state: ConnectionState,
        now_ms: u64,
        detail: Option<String>,
        retry_in_ms: Option<u64>,
        boot_id: Option<u32>,
    ) {
        let connection = Connection {
            state,
            robot: self.robot.clone(),
            detail,
            retry_in_ms,
            boot_id,
            ts: now_ms,
        };
        if self
            .last_connection
            .as_ref()
            .is_some_and(|last| same_connection(last, &connection))
        {
            return;
        }
        if self.verbose {
            eprintln!("remote-daemon: connection {state:?}");
        }
        if let Ok(mut relay) = self.relay.lock() {
            relay.set_connection(connection.clone());
        }
        self.last_connection = Some(connection);
    }

    /// Resolves and adopts a manual `host:port`, replacing any current Session.
    async fn set_manual(&mut self, host: &str, port: u16) {
        let Some(endpoint) = resolve(host, port).await else {
            self.publish(
                ConnectionState::NoRobots,
                clock::now_ms(),
                Some(format!("Could not resolve {host}:{port}.")),
                None,
                None,
            );
            return;
        };
        self.manual = Some(endpoint);
        self.cached = None;
        self.blocked = None;
        self.stop_link();
        let _ = self.store.set_last(endpoint, None);
        self.trace(&format!("manual endpoint {endpoint}"));
        let now_ms = clock::now_ms();
        let token = self.store.token_for(None);
        self.open(
            Opening {
                endpoint,
                token,
                choice: None,
                detail: Some(String::from("Manual endpoint")),
                robot: None,
                cached: false,
            },
            now_ms,
        );
    }

    /// Asks the running link to stop and retires its waiter.
    fn stop_link(&mut self) {
        if let Some(link) = self.link.take() {
            link.stop();
            self.retired.push(tokio::spawn(async move { link.wait().await }));
        }
        self.last_applied = None;
        self.opened = None;
        self.detail = None;
        self.robot = None;
        self.used_cached = false;
    }

    /// Releases the Session unconditionally (shutdown or an app `disconnect`).
    fn release_now(&mut self) {
        self.active = false;
        self.grace_until = None;
        self.stop_link();
    }

    /// Traces one line when `--verbose` is set.
    fn trace(&self, message: &str) {
        if self.verbose {
            eprintln!("remote-daemon: {message}");
        }
    }
}

/// The stable identity of a Beacon choice: the device and whether it is in Setup Mode.
///
/// The cached `seen_ms` and `name` change as Beacons arrive, so a fatal-state block must not key
/// on the whole entry or a still-wrong robot would be retried on every datagram.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ChoiceKey {
    /// The MAC-derived device id.
    device: [u8; 6],
    /// Whether the Beacon advertises Setup Mode.
    setup_mode: bool,
}

/// Returns the stable identity of a Beacon choice.
const fn choice_key(choice: &Choice) -> Option<ChoiceKey> {
    match choice {
        Choice::Beacon { entry, .. } => Some(ChoiceKey {
            device: entry.device_id.bytes(),
            setup_mode: entry.setup_mode(),
        }),
        Choice::None | Choice::Manual { .. } => None,
    }
}

/// Returns the device id a Beacon choice names.
const fn device_of(choice: &Choice) -> Option<bridge_net::identity::DeviceId> {
    match choice {
        Choice::Beacon { entry, .. } => Some(entry.device_id),
        Choice::None | Choice::Manual { .. } => None,
    }
}

/// Compares two connection messages without their timestamp.
fn same_connection(left: &Connection, right: &Connection) -> bool {
    left.state == right.state
        && left.robot == right.robot
        && left.detail == right.detail
        && left.retry_in_ms == right.retry_in_ms
        && left.boot_id == right.boot_id
}

/// Resolves a manual `host:port`, accepting an IP literal without a DNS lookup.
async fn resolve(host: &str, port: u16) -> Option<SocketAddr> {
    if let Ok(ip) = host.parse::<IpAddr>() {
        return Some(SocketAddr::new(ip, port));
    }
    lookup_host((host, port)).await.ok()?.next()
}

/// Pins the loopback default used by the binary.
pub const DEFAULT_HOST: IpAddr = IpAddr::V4(Ipv4Addr::LOCALHOST);

#[cfg(test)]
mod tests {
    use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4};

    use super::{DEFAULT_HOST, DEFAULT_HTTP_PORT, POST_APP_GRACE, authority};

    /// The ADR's defaults and grace are literals, not approximations.
    #[test]
    fn the_defaults_are_the_adrs() {
        assert_eq!(DEFAULT_HTTP_PORT, 8765);
        assert_eq!(DEFAULT_HOST, IpAddr::V4(Ipv4Addr::LOCALHOST));
        assert_eq!(POST_APP_GRACE, std::time::Duration::from_secs(5));
    }

    /// An unspecified bind is printed as the loopback URL a browser can open.
    #[test]
    fn an_unspecified_bind_prints_loopback() {
        assert_eq!(
            authority(SocketAddr::from((Ipv4Addr::UNSPECIFIED, 8765))),
            "127.0.0.1:8765"
        );
        assert_eq!(
            authority(SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(192, 0, 2, 1), 9000))),
            "192.0.2.1:9000"
        );
    }
}
