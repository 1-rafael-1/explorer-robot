//! Shared fixtures for the app adapter's headless tests.
//!
//! Each test runs a real `axum` server on an ephemeral loopback port and drives it with
//! `tokio-tungstenite`, so the seam is a real HTTP upgrade and real JSON text frames. Helpers fail
//! a test rather than unwrapping, because the workspace denies `unwrap` and `expect` in tests too.

#![allow(dead_code)]

use std::{
    collections::BTreeMap,
    fmt::Display,
    io,
    net::SocketAddr,
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex,
        atomic::{AtomicUsize, Ordering},
    },
    time::Duration,
};

use futures_util::{SinkExt, StreamExt};
use remote_daemon::{
    app::{App, AppHooks, Assets, PeerPolicy, ShutdownHandle, TokenGate},
    clock,
    relay::{OutboundCommand, Relay},
};
use remote_protocol::payload::stream::Power;
use serde_json::{Value, json};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    task::JoinHandle,
};
use tokio_tungstenite::{
    MaybeTlsStream, WebSocketStream, connect_async,
    tungstenite::{Error as WsError, Message, client::IntoClientRequest, http::HeaderValue},
};

/// Extracts a value, failing the test if it is absent.
pub fn must<T, E: Display>(result: Result<T, E>, what: &str) -> T {
    result.unwrap_or_else(|error| {
        eprintln!("{what}: {error}");
        std::process::abort()
    })
}

/// Extracts an optional value, failing the test if it is absent.
pub fn must_some<T>(value: Option<T>, what: &str) -> T {
    value.unwrap_or_else(|| {
        eprintln!("missing {what}");
        std::process::abort()
    })
}

/// A fake token store: the adapter's constant-time comparison is the real one.
#[derive(Debug, Default)]
pub struct FakeGate {
    /// The stored token, if any.
    token: Mutex<Option<String>>,
    /// Every code the adapter stored through `pair`.
    pairs: Mutex<Vec<String>>,
}

impl FakeGate {
    /// A gate with no stored token.
    #[must_use]
    pub fn empty() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// A gate that already holds `code`.
    #[must_use]
    pub fn with_token(code: &str) -> Arc<Self> {
        Arc::new(Self {
            token: Mutex::new(Some(code.to_owned())),
            pairs: Mutex::new(Vec::new()),
        })
    }

    /// Returns every code stored through `pair`.
    #[must_use]
    pub fn pairs(&self) -> Vec<String> {
        self.pairs.lock().map(|pairs| pairs.clone()).unwrap_or_default()
    }
}

impl TokenGate for FakeGate {
    fn token(&self) -> Option<String> {
        self.token.lock().ok().and_then(|token| token.clone())
    }

    fn pair(&self, code: &str) {
        if let Ok(mut token) = self.token.lock() {
            *token = Some(code.to_owned());
        }
        if let Ok(mut pairs) = self.pairs.lock() {
            pairs.push(code.to_owned());
        }
    }
}

/// Records the Session hooks ticket 13 will wire.
#[derive(Debug, Default)]
pub struct RecordingHooks {
    /// How many times the first-auth hook fired.
    first: AtomicUsize,
    /// How many times the last-disconnect hook fired.
    last: AtomicUsize,
    /// How many `disconnect` messages arrived.
    disconnects: AtomicUsize,
    /// Every manual `host:port` endpoint.
    endpoints: Mutex<Vec<(String, u16)>>,
}

impl RecordingHooks {
    /// Returns the first-auth count.
    #[must_use]
    pub fn first_auths(&self) -> usize {
        self.first.load(Ordering::SeqCst)
    }

    /// Returns the last-disconnect count.
    #[must_use]
    pub fn last_disconnects(&self) -> usize {
        self.last.load(Ordering::SeqCst)
    }

    /// Returns the app `disconnect` count.
    #[must_use]
    pub fn disconnects(&self) -> usize {
        self.disconnects.load(Ordering::SeqCst)
    }

    /// Returns every manual endpoint.
    #[must_use]
    pub fn endpoints(&self) -> Vec<(String, u16)> {
        self.endpoints
            .lock()
            .map(|endpoints| endpoints.clone())
            .unwrap_or_default()
    }
}

impl AppHooks for RecordingHooks {
    fn on_first_auth(&self) {
        self.first.fetch_add(1, Ordering::SeqCst);
    }

    fn on_last_disconnect(&self) {
        self.last.fetch_add(1, Ordering::SeqCst);
    }

    fn on_disconnect(&self) {
        self.disconnects.fetch_add(1, Ordering::SeqCst);
    }

    fn on_manual_endpoint(&self, host: &str, port: u16) {
        if let Ok(mut endpoints) = self.endpoints.lock() {
            endpoints.push((host.to_owned(), port));
        }
    }
}

/// Gates every peer, so a loopback test client exercises the LAN auth paths.
#[derive(Clone, Copy, Debug, Default)]
pub struct GateAll;

impl PeerPolicy for GateAll {
    fn requires_auth(&self, _peer: SocketAddr) -> bool {
        true
    }
}

/// Returns the gating test policy.
#[must_use]
pub fn gate_all() -> Arc<dyn PeerPolicy> {
    Arc::new(GateAll)
}

/// Returns a relay with one powered Session, as the link adapter would leave it.
#[must_use]
pub fn relay_with_power(boot_id: u32, level: u8, voltage_mv: u16) -> Arc<Mutex<Relay>> {
    let now = clock::now_ms();
    let mut relay = Relay::new();
    relay.session_up(boot_id, now);
    relay.on_power(&Power { level, voltage_mv }, now);
    Arc::new(Mutex::new(relay))
}

/// Returns a relay with an open Session and no samples.
#[must_use]
pub fn relay_with_session(boot_id: u32) -> Arc<Mutex<Relay>> {
    let mut relay = Relay::new();
    relay.session_up(boot_id, clock::now_ms());
    Arc::new(Mutex::new(relay))
}

/// A running app-adapter server on an ephemeral loopback port.
pub struct TestServer {
    /// The bound address.
    addr: SocketAddr,
    /// The shutdown signal.
    shutdown: ShutdownHandle,
    /// The serving task.
    join: JoinHandle<io::Result<()>>,
}

impl TestServer {
    /// Starts a server with ADR-0039's defaults.
    pub async fn start(relay: Arc<Mutex<Relay>>, gate: Arc<dyn TokenGate>) -> io::Result<Self> {
        Self::start_with(
            relay,
            gate,
            Arc::new(remote_daemon::app::LoopbackFree),
            Assets::embedded(),
        )
        .await
    }

    /// Starts a server with a policy and assets.
    pub async fn start_with(
        relay: Arc<Mutex<Relay>>,
        gate: Arc<dyn TokenGate>,
        policy: Arc<dyn PeerPolicy>,
        assets: Assets,
    ) -> io::Result<Self> {
        Self::start_full(relay, gate, policy, assets, Arc::new(remote_daemon::app::NoHooks)).await
    }

    /// Starts a server with hooks too.
    pub async fn start_full(
        relay: Arc<Mutex<Relay>>,
        gate: Arc<dyn TokenGate>,
        policy: Arc<dyn PeerPolicy>,
        assets: Assets,
        hooks: Arc<dyn AppHooks>,
    ) -> io::Result<Self> {
        let listener = TcpListener::bind(("127.0.0.1", 0)).await?;
        let addr = listener.local_addr()?;
        let app = App::new(relay, gate)
            .with_peer_policy(policy)
            .with_assets(assets)
            .with_hooks(hooks);
        let shutdown = app.shutdown_handle();
        let join = tokio::spawn(app.serve(listener));
        Ok(Self { addr, shutdown, join })
    }

    /// Returns the bound address.
    #[must_use]
    pub const fn addr(&self) -> SocketAddr {
        self.addr
    }

    /// Asks the server to close every socket with 1001.
    pub fn shutdown(&self) {
        self.shutdown.shutdown();
    }
}

impl Drop for TestServer {
    fn drop(&mut self) {
        self.shutdown.shutdown();
        self.join.abort();
    }
}

/// The headless WebSocket client.
pub type Client = WebSocketStream<MaybeTlsStream<TcpStream>>;

/// Connects to `/ws` with no `Origin` header.
pub async fn connect(addr: SocketAddr) -> Result<Client, WsError> {
    let (client, _response) = connect_async(format!("ws://{addr}/ws")).await?;
    Ok(client)
}

/// Connects to `/ws` with the supplied `Origin`.
pub async fn connect_with_origin(addr: SocketAddr, origin: &str) -> Result<Client, WsError> {
    let mut request = format!("ws://{addr}/ws").into_client_request()?;
    let value = HeaderValue::from_str(origin)
        .map_err(|_| WsError::Io(io::Error::new(io::ErrorKind::InvalidInput, "bad origin")))?;
    request.headers_mut().insert("origin", value);
    let (client, _response) = connect_async(request).await?;
    Ok(client)
}

/// One frame the server sent.
pub enum Frame {
    /// A JSON text frame.
    Json(Value),
    /// A close frame, with its code when present.
    Close(Option<u16>),
    /// A control frame the tests ignore.
    Other,
}

/// Reads the next frame, returning `None` on a timeout or a transport error.
pub async fn next_frame(client: &mut Client, timeout: Duration) -> Option<Frame> {
    let message = tokio::time::timeout(timeout, client.next()).await.ok()??;
    match message.ok()? {
        Message::Text(text) => serde_json::from_str(&text).ok().map(Frame::Json),
        Message::Close(frame) => Some(Frame::Close(frame.map(|frame| u16::from(frame.code)))),
        _ => Some(Frame::Other),
    }
}

/// Reads the next frame with a three-second budget.
pub async fn next_frame_default(client: &mut Client) -> Option<Frame> {
    next_frame(client, Duration::from_secs(3)).await
}

/// Reads frames until one has the given `type`.
pub async fn json_of_type(client: &mut Client, kind: &str) -> Value {
    for _ in 0..50 {
        match next_frame_default(client).await {
            Some(Frame::Json(value)) if value["type"] == kind => return value,
            Some(Frame::Json(_) | Frame::Other) => {}
            Some(Frame::Close(code)) => {
                eprintln!("closed before {kind}: {code:?}");
                std::process::abort();
            }
            None => {
                eprintln!("closed before {kind}");
                std::process::abort();
            }
        }
    }
    eprintln!("no {kind} frame");
    std::process::abort();
}

/// Reads until a close frame and returns its code (0 when the peer sent none).
pub async fn close_code(client: &mut Client) -> u16 {
    close_code_within(client, Duration::from_secs(3)).await
}

/// Reads until a close frame with an explicit per-frame budget.
pub async fn close_code_within(client: &mut Client, timeout: Duration) -> u16 {
    for _ in 0..50 {
        match next_frame(client, timeout).await {
            Some(Frame::Close(Some(code))) => return code,
            Some(Frame::Close(None)) => return 0,
            Some(_) => {}
            None => {
                eprintln!("no close frame within {timeout:?}");
                std::process::abort();
            }
        }
    }
    eprintln!("no close frame");
    std::process::abort();
}

/// Sends one JSON text frame.
pub async fn send_json(client: &mut Client, value: &Value) {
    let text = must(serde_json::to_string(value), "serialise");
    if let Err(error) = client.send(Message::Text(text.into())).await {
        eprintln!("send failed: {error}");
        std::process::abort();
    }
}

/// Waits for the relay to yield a command frame, polling as the link adapter would.
pub async fn wait_for_outbound(relay: &Mutex<Relay>) -> Option<OutboundCommand> {
    for _ in 0..200 {
        let outbound = { relay.lock().ok().and_then(|mut relay| relay.poll_link(clock::now_ms())) };
        if outbound.is_some() {
            return outbound;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    None
}

/// Waits for a counter to reach `wanted`, yielding to the runtime between checks.
pub async fn wait_for_count(counter: impl Fn() -> usize, wanted: usize) -> bool {
    for _ in 0..200 {
        if counter() >= wanted {
            return true;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    false
}

/// A temporary disk folder for the `--assets` override.
pub struct TempDir {
    /// The folder's path.
    path: PathBuf,
}

impl TempDir {
    /// Creates a uniquely named folder under the system temporary directory.
    pub fn new(label: &str) -> io::Result<Self> {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let counter = NEXT.fetch_add(1, Ordering::SeqCst);
        let path = std::env::temp_dir().join(format!("remote-daemon-{label}-{}-{counter}", std::process::id()));
        std::fs::create_dir_all(&path)?;
        Ok(Self { path })
    }

    /// Writes one file, creating parent folders.
    pub fn write(&self, name: &str, body: &str) -> io::Result<()> {
        let path = self.path.join(name);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, body)
    }

    /// Returns the folder's path.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.path);
    }
}

/// A parsed HTTP response from the static endpoint.
pub struct HttpResponse {
    /// The status code.
    pub status: u16,
    /// The lower-cased header map.
    pub headers: BTreeMap<String, String>,
    /// The body as text.
    pub body: String,
}

impl HttpResponse {
    /// Returns one header value.
    #[must_use]
    pub fn header(&self, name: &str) -> Option<&str> {
        self.headers.get(name).map(String::as_str)
    }
}

/// Sends one HTTP/1.1 GET with `Connection: close` and parses the response.
pub async fn http_get(addr: SocketAddr, path: &str) -> io::Result<HttpResponse> {
    let mut stream = TcpStream::connect(addr).await?;
    let request = format!("GET {path} HTTP/1.1\r\nHost: {addr}\r\nConnection: close\r\n\r\n");
    stream.write_all(request.as_bytes()).await?;
    let mut bytes = Vec::new();
    stream.read_to_end(&mut bytes).await?;
    parse_response(&bytes)
}

/// Parses an HTTP/1.1 response with a known `Content-Length`.
fn parse_response(bytes: &[u8]) -> io::Result<HttpResponse> {
    let split = bytes
        .windows(4)
        .position(|window| window == b"\r\n\r\n")
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "no header terminator"))?;
    let head = std::str::from_utf8(&bytes[..split])
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "headers are not UTF-8"))?;
    let mut lines = head.split("\r\n");
    let status = lines
        .next()
        .and_then(|line| line.split_whitespace().nth(1))
        .and_then(|code| code.parse::<u16>().ok())
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "no status code"))?;
    let mut headers = BTreeMap::new();
    for line in lines {
        if let Some((name, value)) = line.split_once(':') {
            headers.insert(name.trim().to_ascii_lowercase(), value.trim().to_owned());
        }
    }
    let body = String::from_utf8_lossy(&bytes[split + 4..]).into_owned();
    Ok(HttpResponse { status, headers, body })
}

/// The literal header values ADR-0039 requires on every static response.
pub fn assert_static_headers(response: &HttpResponse) {
    assert_eq!(response.header("x-content-type-options"), Some("nosniff"));
    assert_eq!(response.header("referrer-policy"), Some("no-referrer"));
    assert_eq!(response.header("cache-control"), Some("no-store"));
    assert_eq!(
        response.header("content-security-policy"),
        Some(remote_daemon::app::CSP)
    );
}

/// The JSON `hello` a client sends after the server's greeting.
#[must_use]
pub fn a_client_hello() -> Value {
    json!({"type": "hello", "protocol": 1})
}
