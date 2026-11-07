//! The `/ws` endpoint: the `Origin` gate, the per-peer auth gate and the versioned JSON loop
//! (ADR-0038, ADR-0039).

use std::{
    net::{IpAddr, SocketAddr},
    sync::{Arc, Mutex},
};

use axum::{
    extract::{
        ConnectInfo, State,
        ws::{CloseFrame, Message, WebSocket, WebSocketUpgrade},
    },
    http::{HeaderMap, StatusCode, header},
    response::{IntoResponse, Response},
};
use tokio::time::{self, MissedTickBehavior};

use super::{AppState, CLOSE_PROTOCOL, CLOSE_SHUTDOWN, CLOSE_UNAUTHORIZED, auth::tokens_match};
use crate::{
    clock,
    relay::{AppId, Relay},
    schema::{
        Auth, ClientHello, ClientMessage, Connection, ConnectionState, Hello, PROTOCOL, Pair, RobotModel, SERVICE_NAME,
        ServerMessage, ServerName,
    },
};

/// The close reason for an unauthenticated peer.
const UNAUTHORIZED: &str = "unauthorized";

/// The close reason for a server shutdown.
const SHUTDOWN: &str = "server shutting down";

/// The close reason for a protocol mismatch.
const PROTOCOL_MISMATCH: &str = "protocol mismatch";

/// The close reason for a missing client `hello`.
const MISSING_HELLO: &str = "missing hello";

/// The close reason for a malformed known message.
const INVALID_MESSAGE: &str = "invalid message";

/// The close reason for a binary frame.
const TEXT_ONLY: &str = "text frames only";

/// Rejects a cross-origin upgrade before the WebSocket is accepted, then hands the socket to
/// [`session`].
pub async fn ws_handler(
    ws: WebSocketUpgrade,
    State(state): State<AppState>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    headers: HeaderMap,
) -> Response {
    if !origin_allowed(&headers) {
        return (StatusCode::FORBIDDEN, "Origin not allowed").into_response();
    }
    ws.on_upgrade(move |socket| session(socket, state, peer))
}

/// ADR-0039's `Origin` rule.
///
/// A present `Origin` must be the request's own `Host` under an `http`/`https` scheme, or one of
/// the Tauri shell's origins. A missing `Origin` is allowed: browsers always send one, so its
/// absence identifies a local non-browser tool, and on the LAN the auth gate still applies.
fn origin_allowed(headers: &HeaderMap) -> bool {
    let Some(value) = headers.get(header::ORIGIN) else {
        return true;
    };
    let Ok(origin) = value.to_str() else {
        return false;
    };
    if origin.eq_ignore_ascii_case("tauri://localhost") || origin.eq_ignore_ascii_case("http://tauri.localhost") {
        return true;
    }
    let Some(host) = headers.get(header::HOST).and_then(|value| value.to_str().ok()) else {
        return false;
    };
    ["http://", "https://"].iter().any(|scheme| {
        origin
            .strip_prefix(scheme)
            .is_some_and(|authority| authority.eq_ignore_ascii_case(host))
    })
}

/// Runs one WebSocket from the server's `hello` to the relay's disconnect.
async fn session(mut socket: WebSocket, state: AppState, peer: SocketAddr) {
    let auth_required = state.policy.requires_auth(peer);
    let hello = ServerMessage::Hello(Hello {
        protocol: PROTOCOL,
        server: ServerName {
            name: SERVICE_NAME.to_owned(),
        },
        robot: RobotModel {
            model: state.robot_model.clone(),
        },
        auth_required,
    });
    if send(&mut socket, &hello).await.is_err() {
        return;
    }
    if *state.shutdown.borrow() {
        close(&mut socket, CLOSE_SHUTDOWN, SHUTDOWN).await;
        return;
    }
    if auth_required && !authenticate(&mut socket, &state, peer.ip()).await {
        let notice = ServerMessage::Connection(Connection {
            state: ConnectionState::Unauthorized,
            robot: None,
            detail: Some(String::from("Authentication required")),
            retry_in_ms: None,
            boot_id: None,
            ts: clock::now_ms(),
        });
        let _ = send(&mut socket, &notice).await;
        close(&mut socket, CLOSE_UNAUTHORIZED, UNAUTHORIZED).await;
        return;
    }

    let Some(app) = state
        .relay
        .lock()
        .ok()
        .map(|mut relay| relay.connect_app(clock::now_ms()))
    else {
        return;
    };
    // The relay guard must drop before the ticket so the last-disconnect hook sees the relay
    // without this connection (drop order is the reverse of declaration order).
    let _ticket = state.admit();
    let _connection = RelayConnection {
        relay: Arc::clone(&state.relay),
        app,
    };

    for message in poll(&state, app) {
        if send(&mut socket, &message).await.is_err() {
            return;
        }
    }
    run(&mut socket, &state, app).await;
}

/// Performs the auth gate: `auth` against the stored token, or `pair`, within the 5 s deadline.
///
/// Anything else, a wrong code, a refusal window or silence returns `false`. A success clears the
/// peer's failure count (ADR-0037).
async fn authenticate(socket: &mut WebSocket, state: &AppState, ip: IpAddr) -> bool {
    if state.limiter.is_refused(ip) {
        return false;
    }
    let deadline = time::sleep(state.auth_deadline);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            () = &mut deadline => return false,
            frame = socket.recv() => {
                match frame {
                    Some(Ok(Message::Text(text))) => match parse_client(&text) {
                        Ok(Some(ClientMessage::Auth(Auth { code }))) => {
                            let accepted = state
                                .gate
                                .token()
                                .is_some_and(|stored| tokens_match(&stored, &code));
                            if accepted {
                                state.limiter.clear(ip);
                                return true;
                            }
                            state.limiter.record_failure(ip);
                            return false;
                        }
                        Ok(Some(ClientMessage::Pair(Pair { code }))) => {
                            state.gate.pair(&code);
                            state.limiter.clear(ip);
                            return true;
                        }
                        _ => return false,
                    },
                    Some(Ok(Message::Ping(payload))) => {
                        let _ = socket.send(Message::Pong(payload)).await;
                    }
                    _ => return false,
                }
            }
        }
    }
}

/// The authenticated loop: the relay's lanes, heartbeats, commands, the hooks and the timers.
async fn run(socket: &mut WebSocket, state: &AppState, app: AppId) {
    let mut hello_seen = false;
    let hello_deadline = time::sleep(state.hello_deadline);
    tokio::pin!(hello_deadline);
    let mut poll_interval = time::interval(state.poll_interval);
    poll_interval.set_missed_tick_behavior(MissedTickBehavior::Delay);
    let mut shutdown = state.shutdown.clone();
    loop {
        tokio::select! {
            _ = shutdown.changed() => {
                close(socket, CLOSE_SHUTDOWN, SHUTDOWN).await;
                return;
            }
            () = &mut hello_deadline, if !hello_seen => {
                close(socket, CLOSE_PROTOCOL, MISSING_HELLO).await;
                return;
            }
            _ = poll_interval.tick() => {
                for message in poll(state, app) {
                    if send(socket, &message).await.is_err() {
                        return;
                    }
                }
            }
            frame = socket.recv() => {
                let Some(frame) = frame else {
                    return;
                };
                let Ok(message) = frame else {
                    return;
                };
                match message {
                    Message::Text(text) => {
                        match parse_client(&text) {
                            Ok(Some(ClientMessage::Hello(ClientHello { protocol }))) => {
                                if protocol != PROTOCOL {
                                    close(socket, CLOSE_PROTOCOL, PROTOCOL_MISMATCH).await;
                                    return;
                                }
                                hello_seen = true;
                            }
                            Ok(Some(_)) if !hello_seen => {
                                close(socket, CLOSE_PROTOCOL, MISSING_HELLO).await;
                                return;
                            }
                            Ok(Some(ClientMessage::Heartbeat)) => {
                                if let Ok(mut relay) = state.relay.lock() {
                                    relay.heartbeat(app, clock::now_ms());
                                }
                            }
                            Ok(Some(ClientMessage::Command(frame))) => {
                                let result = state
                                    .relay
                                    .lock()
                                    .ok()
                                    .and_then(|mut relay| {
                                        relay.submit_command(app, frame, clock::now_ms())
                                    });
                                if let Some(result) = result
                                    && send(socket, &ServerMessage::CommandResult(result)).await.is_err()
                                {
                                    return;
                                }
                            }
                            Ok(Some(ClientMessage::Disconnect)) => state.hooks.on_disconnect(),
                            Ok(Some(ClientMessage::Connect(endpoint))) => {
                                state.hooks.on_manual_endpoint(&endpoint.host, endpoint.port);
                            }
                            Ok(Some(ClientMessage::Auth(_) | ClientMessage::Pair(_)) | None) => {}
                            Err(()) => {
                                close(socket, CLOSE_PROTOCOL, INVALID_MESSAGE).await;
                                return;
                            }
                        }
                    }
                    Message::Ping(payload) => {
                        let _ = socket.send(Message::Pong(payload)).await;
                    }
                    Message::Pong(_) => {}
                    Message::Close(_) => return,
                    Message::Binary(_) => {
                        close(socket, CLOSE_PROTOCOL, TEXT_ONLY).await;
                        return;
                    }
                }
            }
        }
    }
}

/// Drains the relay's lanes for one connection with the shared clock.
fn poll(state: &AppState, app: AppId) -> Vec<ServerMessage> {
    let now = clock::now_ms();
    state
        .relay
        .lock()
        .ok()
        .map_or_else(Vec::new, |mut relay| relay.poll(app, now))
}

/// Parses one text frame, ignoring unknown message types (ADR-0038's additive schema).
///
/// Returns `Ok(None)` for an unknown `type`, `Err(())` for malformed JSON or a known type with
/// missing fields.
fn parse_client(text: &str) -> Result<Option<ClientMessage>, ()> {
    let value: serde_json::Value = serde_json::from_str(text).map_err(|_| ())?;
    let kind = value.get("type").and_then(serde_json::Value::as_str).ok_or(())?;
    match kind {
        "hello" | "auth" | "pair" | "connect" | "disconnect" | "heartbeat" | "command" => {
            serde_json::from_value(value).map(Some).map_err(|_| ())
        }
        _ => Ok(None),
    }
}

/// Serialises and sends one server message.
async fn send(socket: &mut WebSocket, message: &ServerMessage) -> Result<(), ()> {
    let text = serde_json::to_string(message).map_err(|_| ())?;
    socket.send(Message::Text(text.into())).await.map_err(|_| ())
}

/// Sends a close frame; the peer's own close handshake is best-effort.
async fn close(socket: &mut WebSocket, code: u16, reason: &'static str) {
    let _ = socket
        .send(Message::Close(Some(CloseFrame {
            code,
            reason: reason.into(),
        })))
        .await;
}

/// Disconnects the relay app when the socket task ends.
struct RelayConnection {
    /// The shared relay.
    relay: Arc<Mutex<Relay>>,
    /// This connection's relay identifier.
    app: AppId,
}

impl Drop for RelayConnection {
    fn drop(&mut self) {
        if let Ok(mut relay) = self.relay.lock() {
            relay.disconnect_app(self.app);
        }
    }
}
