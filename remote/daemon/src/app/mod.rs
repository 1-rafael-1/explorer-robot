//! The app adapter: one `axum` WebSocket at `/ws`, the static SPA and the auth and `Origin`
//! gates (ADR-0039's `app` module).
//!
//! The adapter is a library so the daemon's process host can own the process. It shares the relay
//! through the same `Arc<Mutex<Relay>>` the link adapter uses: [`App::new`] registers a connection on
//! authentication, its WebSocket task drains [`Relay::poll`] and ticks the relay's timers, and a
//! [`TokenGate`] holds the pairing token the LAN gate compares in constant time. The process host
//! supplies the gate over its config store, wires the [`AppHooks`] to open and release the Bridge
//! Session, and binds the listener [`App::serve`] consumes.
//!
//! The server's first frame is always `hello`; `auth_required` is true exactly for a peer address
//! the [`PeerPolicy`] gates. A loopback peer needs no token, and a non-loopback peer must send
//! `auth` or `pair` first. The client's `hello.protocol` must be [`PROTOCOL`], else the socket
//! closes **4426**; auth failures close **4401**; shutdown closes **1001**.

mod auth;
mod static_files;
mod ws;

use std::{
    future::Future,
    net::SocketAddr,
    sync::{
        Arc, Mutex,
        atomic::{AtomicUsize, Ordering},
    },
    time::Duration,
};

pub use auth::{AUTH_FAILURE_LIMIT, AUTH_FAILURE_WINDOW, AUTH_REFUSAL};
use axum::{Router, routing::get};
pub use static_files::{Assets, CSP};
use tokio::{net::TcpListener, sync::watch, task::JoinHandle};

use crate::relay::Relay;

/// How long a non-loopback peer has to send `auth` or `pair` (ADR-0037's 5 s).
pub const AUTH_DEADLINE: Duration = Duration::from_secs(5);

/// How long an authenticated client has to send its `hello` before the socket closes 4426.
pub const HELLO_DEADLINE: Duration = Duration::from_secs(5);

/// The close code for a peer that did not authenticate (ADR-0039).
pub const CLOSE_UNAUTHORIZED: u16 = 4401;

/// The close code for a protocol mismatch or a missing/invalid `hello` (ADR-0039).
pub const CLOSE_PROTOCOL: u16 = 4426;

/// The close code for a server shutdown (ADR-0039).
pub const CLOSE_SHUTDOWN: u16 = 1001;

/// How often one connection drains the relay's snapshot, result, stream and event lanes.
const POLL_INTERVAL: Duration = Duration::from_millis(50);

/// How often the adapter ticks the relay's command, liveness and eviction timers.
const TICK_INTERVAL: Duration = Duration::from_millis(100);

/// The pairing token store the LAN gate consults.
///
/// The process host implements this over the per-user config store; tests use a fake. The trait
/// exposes the stored token rather than a `verify(&str)` answer so the adapter performs the one
/// constant-time comparison, keeping the timing property in a single place.
pub trait TokenGate: Send + Sync {
    /// Returns the stored pairing token, or `None` when none has been stored.
    #[must_use]
    fn token(&self) -> Option<String>;

    /// Stores `code` as the pairing token (the `pair` message also authenticates its sender).
    fn pair(&self, code: &str);
}

/// The process hooks the host wires to the Bridge Session.
///
/// The adapter never awaits the link: these are short, synchronous callbacks. A `disconnect`
/// message releases the shared Session for every tab but keeps the stored token (ADR-0039), and a
/// manual `connect` supplies an endpoint for networks that block discovery broadcast.
pub trait AppHooks: Send + Sync {
    /// Called when the first app connection authenticates; the robot Session may open.
    fn on_first_auth(&self) {}

    /// Called when the last authenticated app connection closes; the Session may be released.
    fn on_last_disconnect(&self) {}

    /// Called for an app `disconnect`; every tab's Session is released, the token is kept.
    fn on_disconnect(&self) {}

    /// Called for a manual `host:port` endpoint from an app `connect` message.
    fn on_manual_endpoint(&self, _host: &str, _port: u16) {}

    /// Called when an already-authenticated app sends a new `pair` code. This is explicit
    /// operator intent to retry a Session a fatal state had blocked (ADR-0037, ADR-0039).
    fn on_pair(&self) {}
}

/// The do-nothing [`AppHooks`] used when no Session wiring is installed.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoHooks;

impl AppHooks for NoHooks {}

/// Decides whether a peer address must authenticate before using the app boundary.
///
/// The rule is per peer address, not per bind (ADR-0039). [`LoopbackFree`] is the production
/// implementation; tests inject a policy that gates a loopback test client so the LAN paths can be
/// exercised without a second network interface.
pub trait PeerPolicy: Send + Sync {
    /// Returns whether `peer` must send `auth` or `pair` before anything else.
    fn requires_auth(&self, peer: SocketAddr) -> bool;
}

/// ADR-0039's implementation: a loopback peer is free, every other peer is gated.
#[derive(Clone, Copy, Debug, Default)]
pub struct LoopbackFree;

impl PeerPolicy for LoopbackFree {
    fn requires_auth(&self, peer: SocketAddr) -> bool {
        !peer.ip().is_loopback()
    }
}

/// The shared state of one server; cloned into every connection task.
#[derive(Clone)]
struct AppState {
    /// The relay both adapters share.
    relay: Arc<Mutex<Relay>>,
    /// The pairing-token store.
    gate: Arc<dyn TokenGate>,
    /// The Session hooks the host supplies.
    hooks: Arc<dyn AppHooks>,
    /// The per-peer-address auth rule.
    policy: Arc<dyn PeerPolicy>,
    /// The per-IP failure limiter.
    limiter: Arc<auth::AuthLimiter>,
    /// The SPA's bytes.
    assets: Assets,
    /// Whether the browser test pages under `tests/` may be served.
    serve_tests: bool,
    /// The robot model advertised in `hello`, when known.
    robot_model: Option<String>,
    /// How long the auth gate waits for the first message.
    auth_deadline: Duration,
    /// How long the protocol handshake waits for the client's `hello`.
    hello_deadline: Duration,
    /// How often a connection drains the relay.
    poll_interval: Duration,
    /// How often the relay's timers advance.
    tick_interval: Duration,
    /// How many connections have authenticated and not yet closed.
    live: Arc<AtomicUsize>,
    /// The shutdown signal every connection watches.
    shutdown: watch::Receiver<bool>,
}

impl AppState {
    /// Registers one authenticated connection and fires the first-auth hook on `0 -> 1`.
    fn admit(&self) -> ConnectionTicket {
        let live = self.live.fetch_add(1, Ordering::SeqCst) + 1;
        if live == 1 {
            self.hooks.on_first_auth();
        }
        ConnectionTicket {
            live: Arc::clone(&self.live),
            hooks: Arc::clone(&self.hooks),
        }
    }
}

/// Keeps the adapter's authenticated-connection count and fires the last-disconnect hook.
struct ConnectionTicket {
    /// The shared connection count.
    live: Arc<AtomicUsize>,
    /// The hooks that opened the Session.
    hooks: Arc<dyn AppHooks>,
}

impl Drop for ConnectionTicket {
    fn drop(&mut self) {
        if self.live.fetch_sub(1, Ordering::SeqCst) == 1 {
            self.hooks.on_last_disconnect();
        }
    }
}

/// One handle to ask a running server to close its sockets with 1001 and stop accepting.
#[derive(Clone, Debug)]
pub struct ShutdownHandle {
    /// The shutdown signal.
    sender: watch::Sender<bool>,
}

impl ShutdownHandle {
    /// Requests a graceful shutdown; every open WebSocket closes with 1001.
    pub fn shutdown(&self) {
        let _ = self.sender.send(true);
    }
}

/// The app adapter: the `/ws` endpoint and the static SPA over one shared relay.
pub struct App {
    /// The shared connection state.
    state: AppState,
    /// The shutdown signal's sending half.
    shutdown: watch::Sender<bool>,
}

impl App {
    /// Builds an adapter over `relay` with [`LoopbackFree`], [`NoHooks`] and the embedded app.
    #[must_use]
    pub fn new(relay: Arc<Mutex<Relay>>, gate: Arc<dyn TokenGate>) -> Self {
        let (shutdown, receiver) = watch::channel(false);
        Self {
            state: AppState {
                relay,
                gate,
                hooks: Arc::new(NoHooks),
                policy: Arc::new(LoopbackFree),
                limiter: Arc::new(auth::AuthLimiter::default()),
                assets: Assets::Embedded,
                serve_tests: false,
                robot_model: None,
                auth_deadline: AUTH_DEADLINE,
                hello_deadline: HELLO_DEADLINE,
                poll_interval: POLL_INTERVAL,
                tick_interval: TICK_INTERVAL,
                live: Arc::new(AtomicUsize::new(0)),
                shutdown: receiver,
            },
            shutdown,
        }
    }

    /// Installs the Session hooks the host supplies.
    #[must_use]
    pub fn with_hooks(mut self, hooks: Arc<dyn AppHooks>) -> Self {
        self.state.hooks = hooks;
        self
    }

    /// Serves the SPA from `assets` instead of the compile-time embed (`--assets`).
    #[must_use]
    pub fn with_assets(mut self, assets: Assets) -> Self {
        self.state.assets = assets;
        self
    }

    /// Allows the browser suite under `tests/` to be served (`--serve-tests`).
    #[must_use]
    pub const fn with_tests(mut self, serve_tests: bool) -> Self {
        self.state.serve_tests = serve_tests;
        self
    }

    /// Advertises `model` in `hello.robot.model` (`null` when unknown).
    #[must_use]
    pub fn with_robot_model(mut self, model: Option<String>) -> Self {
        self.state.robot_model = model;
        self
    }

    /// Replaces the per-peer-address auth rule (tests use this; production keeps [`LoopbackFree`]).
    #[must_use]
    pub fn with_peer_policy(mut self, policy: Arc<dyn PeerPolicy>) -> Self {
        self.state.policy = policy;
        self
    }

    /// Returns a handle that asks [`App::serve`] to shut down.
    #[must_use]
    pub fn shutdown_handle(&self) -> ShutdownHandle {
        ShutdownHandle {
            sender: self.shutdown.clone(),
        }
    }

    /// Builds the router; serve it with connect info so the auth gate sees the peer address.
    pub fn router(&self) -> Router {
        Router::new()
            .route("/ws", get(ws::ws_handler))
            .fallback(static_files::static_handler)
            .with_state(self.state.clone())
    }

    /// Serves until [`ShutdownHandle::shutdown`] is called.
    ///
    /// # Errors
    ///
    /// Returns the listener or connection error from `axum::serve`.
    pub async fn serve(self, listener: TcpListener) -> std::io::Result<()> {
        self.serve_with_shutdown(listener, std::future::pending::<()>()).await
    }

    /// Serves until `shutdown` resolves or [`ShutdownHandle::shutdown`] is called.
    ///
    /// Every open WebSocket observes the same signal and closes with **1001** before the server
    /// stops. The relay ticker is scoped to this future and stops with it.
    ///
    /// # Errors
    ///
    /// Returns the listener or connection error from `axum::serve`.
    pub async fn serve_with_shutdown<F>(self, listener: TcpListener, shutdown: F) -> std::io::Result<()>
    where
        F: Future<Output = ()> + Send + 'static,
    {
        let _ticker = Ticker(tokio::spawn(tick_loop(self.state.clone())));
        let mut receiver = self.state.shutdown.clone();
        let sender = self.shutdown.clone();
        let signalled = async move {
            tokio::select! {
                () = shutdown => {}
                _ = receiver.changed() => {}
            }
            let _ = sender.send(true);
        };
        let router = self.router();
        axum::serve(listener, router.into_make_service_with_connect_info::<SocketAddr>())
            .with_graceful_shutdown(signalled)
            .await
    }
}

/// Aborts the relay ticker when the server future is dropped.
struct Ticker(JoinHandle<()>);

impl Drop for Ticker {
    fn drop(&mut self) {
        self.0.abort();
    }
}

/// Advances the relay's command, liveness and eviction timers at [`TICK_INTERVAL`].
async fn tick_loop(state: AppState) {
    let mut interval = tokio::time::interval(state.tick_interval);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        interval.tick().await;
        let now = crate::clock::now_ms();
        if let Ok(mut relay) = state.relay.lock() {
            relay.tick(now);
        }
    }
}
