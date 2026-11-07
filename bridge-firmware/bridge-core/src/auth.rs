//! The authentication and single-client gate (ADR-0037).
//!
//! `bridge-net` owns the HMAC-SHA256 preamble and hands this gate an opaque 32-byte proof; the
//! gate owns only the admission rules. A connection has five seconds to authenticate, a new
//! unauthenticated connection supersedes an older one so a probe cannot lock the owner out, and
//! exactly one authenticated client holds the slot. Five failed proofs inside a minute close a
//! one-minute refusal window; a success resets the failure counter. Comparison is constant time,
//! so a wrong proof cannot be found byte by byte.
//!
//! The gate has no I/O and no clock: the caller passes a monotonic millisecond timestamp.

pub use bridge_net::preamble::PROOF_LEN;
use bridge_net::preamble::proofs_match;

/// How long a connection has to authenticate (ADR-0037's 5 s).
pub const DEADLINE_MS: u32 = 5_000;

/// How many failed proofs close the gate (ADR-0037's five).
pub const FAILURE_LIMIT: u32 = 5;

/// The window that counts failures (ADR-0037's 60 s).
pub const FAILURE_WINDOW_MS: u32 = 60_000;

/// How long the gate refuses new connections (ADR-0037's 60 s).
pub const REFUSAL_MS: u32 = 60_000;

/// The number of failure timestamps the gate remembers; equal to [`FAILURE_LIMIT`].
const FAILURE_SLOTS: usize = 5;

/// Half a `u32`'s range, used to compare timestamps across a wrap.
const HALF_RANGE: u32 = 1 << 31;

/// An opaque proof of knowledge of the pairing secret.
///
/// The gate never interprets the bytes and never logs them. `bridge-net` computes the HMAC; the
/// gate only compares two proofs in constant time.
#[derive(Clone, Copy)]
pub struct Proof(
    /// The opaque proof bytes.
    [u8; PROOF_LEN],
);

impl Proof {
    /// Wraps 32 opaque bytes.
    #[must_use]
    pub const fn from_bytes(bytes: [u8; PROOF_LEN]) -> Self {
        Self(bytes)
    }

    /// Borrows the opaque bytes.
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; PROOF_LEN] {
        &self.0
    }

    /// Compares two proofs without an early exit.
    #[must_use]
    pub fn matches(&self, other: &Self) -> bool {
        proofs_match(&self.0, &other.0)
    }
}

/// The gate's answer to a new connection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnectOutcome {
    /// The connection may present a proof until its deadline.
    Challenge {
        /// The new connection's id.
        connection: u32,
    },
    /// An authenticated client already holds the slot.
    Busy,
    /// The failure limit closed the gate until this timestamp.
    Refused {
        /// When the refusal window ends, in the caller's millisecond scale.
        until_ms: u32,
    },
}

/// The gate's answer to a proof.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProofOutcome {
    /// The proof matched and the connection is authenticated.
    Accepted,
    /// The proof did not match; the connection may try again inside its deadline.
    Rejected {
        /// How many failures are inside the current window, including this one.
        failures: u32,
    },
    /// This failure reached the limit and closed the gate.
    Refused {
        /// When the refusal window ends.
        until_ms: u32,
        /// The failure count, equal to [`FAILURE_LIMIT`].
        failures: u32,
    },
    /// The connection's five-second deadline had passed.
    DeadlineExpired,
    /// The connection id is no longer the current one.
    Superseded,
}

/// A timer-driven transition the caller may log or react to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerEvent {
    /// An unauthenticated connection reached its deadline and was dropped.
    AuthenticationExpired {
        /// The dropped connection's id.
        connection: u32,
    },
    /// The refusal window ended.
    RefusalExpired,
}

/// The gate's current connection state.
#[derive(Clone, Copy, Debug)]
enum State {
    /// No connection.
    Idle,
    /// A connection is proving the pairing secret.
    Authenticating {
        /// The connection's id.
        connection: u32,
        /// When the connection arrived, in the caller's millisecond scale.
        started_at: u32,
    },
    /// A connection has proven the pairing secret.
    Authenticated {
        /// The connection's id.
        connection: u32,
    },
}

/// The last five failure timestamps, oldest first.
#[derive(Clone, Copy, Debug)]
struct Failures {
    /// The timestamps, with the oldest at index zero while `len` entries are valid.
    at_ms: [u32; FAILURE_SLOTS],
    /// The number of valid timestamps.
    len: usize,
}

impl Failures {
    /// Creates an empty failure log.
    const fn new() -> Self {
        Self {
            at_ms: [0; FAILURE_SLOTS],
            len: 0,
        }
    }

    /// Records one failure, dropping the oldest when the log is full.
    fn record(&mut self, now_ms: u32) {
        if self.len < FAILURE_SLOTS {
            self.at_ms[self.len] = now_ms;
            self.len += 1;
        } else {
            self.at_ms.copy_within(1..FAILURE_SLOTS, 0);
            self.at_ms[FAILURE_SLOTS - 1] = now_ms;
        }
    }

    /// Counts the recorded failures inside the window ending at `now_ms`.
    fn within_window(&self, now_ms: u32) -> u32 {
        let mut count = 0u32;
        for at_ms in &self.at_ms[..self.len] {
            if now_ms.wrapping_sub(*at_ms) < FAILURE_WINDOW_MS {
                count += 1;
            }
        }
        count
    }

    /// Forgets every recorded failure.
    const fn clear(&mut self) {
        self.len = 0;
    }
}

/// Whether `now_ms` has reached `deadline_ms`, tolerating a `u32` wrap.
const fn reached(now_ms: u32, deadline_ms: u32) -> bool {
    now_ms.wrapping_sub(deadline_ms) < HALF_RANGE
}

/// The authentication and single-client gate.
#[derive(Clone, Copy, Debug)]
pub struct ClientAuth {
    /// The current connection state.
    state: State,
    /// The end of an active refusal window.
    refusal_until: Option<u32>,
    /// The failure timestamps feeding the refusal rule.
    failures: Failures,
    /// The id handed to the next connection.
    next_connection: u32,
}

impl ClientAuth {
    /// Creates a gate with no connection and no refusal.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            refusal_until: None,
            failures: Failures::new(),
            next_connection: 0,
        }
    }

    /// Offers a new connection to the gate.
    ///
    /// A new unauthenticated connection supersedes an older unauthenticated one. An authenticated
    /// client makes the gate report [`ConnectOutcome::Busy`], and an active refusal window makes it
    /// report [`ConnectOutcome::Refused`].
    pub fn connect(&mut self, now_ms: u32) -> ConnectOutcome {
        if let Some(until_ms) = self.refusal(now_ms) {
            return ConnectOutcome::Refused { until_ms };
        }
        if matches!(self.state, State::Authenticated { .. }) {
            return ConnectOutcome::Busy;
        }

        let connection = self.next_connection;
        self.next_connection = self.next_connection.wrapping_add(1);
        self.state = State::Authenticating {
            connection,
            started_at: now_ms,
        };
        ConnectOutcome::Challenge { connection }
    }

    /// Presents a proof for `connection`.
    ///
    /// A proof from a superseded connection is ignored. A failure is recorded only while the
    /// connection is inside its five-second deadline; the fifth failure inside the window closes
    /// the gate for one minute and drops the connection.
    pub fn authenticate(&mut self, connection: u32, proof: Proof, expected: &Proof, now_ms: u32) -> ProofOutcome {
        match self.state {
            State::Authenticating {
                connection: current,
                started_at,
            } if current == connection => {
                if reached(now_ms, started_at.wrapping_add(DEADLINE_MS)) {
                    self.state = State::Idle;
                    return ProofOutcome::DeadlineExpired;
                }
                if proof.matches(expected) {
                    self.failures.clear();
                    self.state = State::Authenticated { connection };
                    ProofOutcome::Accepted
                } else {
                    self.failures.record(now_ms);
                    let failures = self.failures.within_window(now_ms);
                    if failures >= FAILURE_LIMIT {
                        let until_ms = now_ms.wrapping_add(REFUSAL_MS);
                        self.refusal_until = Some(until_ms);
                        self.state = State::Idle;
                        ProofOutcome::Refused { until_ms, failures }
                    } else {
                        ProofOutcome::Rejected { failures }
                    }
                }
            }
            _ => ProofOutcome::Superseded,
        }
    }

    /// Advances the gate's timers.
    ///
    /// Expires an authentication deadline and clears an elapsed refusal window. A deadline takes
    /// precedence over a refusal expiry when both happen on the same tick.
    pub const fn tick(&mut self, now_ms: u32) -> Option<TimerEvent> {
        let event = if let Some(until_ms) = self.refusal_until
            && reached(now_ms, until_ms)
        {
            self.refusal_until = None;
            Some(TimerEvent::RefusalExpired)
        } else {
            None
        };
        if let State::Authenticating { connection, started_at } = self.state
            && reached(now_ms, started_at.wrapping_add(DEADLINE_MS))
        {
            self.state = State::Idle;
            return Some(TimerEvent::AuthenticationExpired { connection });
        }
        event
    }

    /// Drops `connection` if it is the current one, returning whether it was.
    pub fn disconnect(&mut self, connection: u32) -> bool {
        let current = match self.state {
            State::Authenticating {
                connection: current, ..
            }
            | State::Authenticated { connection: current } => Some(current),
            State::Idle => None,
        };
        if current == Some(connection) {
            self.state = State::Idle;
            true
        } else {
            false
        }
    }

    /// Returns whether a client is authenticated.
    #[must_use]
    pub const fn is_authenticated(&self) -> bool {
        matches!(self.state, State::Authenticated { .. })
    }

    /// Returns whether a connection is still proving the pairing secret.
    #[must_use]
    pub const fn is_authenticating(&self) -> bool {
        matches!(self.state, State::Authenticating { .. })
    }

    /// Returns the authenticated connection's id, if any.
    #[must_use]
    pub const fn authenticated_connection(&self) -> Option<u32> {
        match self.state {
            State::Authenticated { connection } => Some(connection),
            _ => None,
        }
    }

    /// Returns the current connection's id, whether or not it is authenticated.
    #[must_use]
    pub const fn current_connection(&self) -> Option<u32> {
        match self.state {
            State::Authenticating { connection, .. } | State::Authenticated { connection } => Some(connection),
            State::Idle => None,
        }
    }

    /// Returns how many failures are inside the window ending at `now_ms`.
    #[must_use]
    pub fn failures_within_window(&self, now_ms: u32) -> u32 {
        self.failures.within_window(now_ms)
    }

    /// Returns the active refusal deadline, clearing it when it has elapsed.
    fn refusal(&mut self, now_ms: u32) -> Option<u32> {
        let until_ms = self.refusal_until?;
        if reached(now_ms, until_ms) {
            self.refusal_until = None;
            None
        } else {
            Some(until_ms)
        }
    }
}

impl Default for ClientAuth {
    fn default() -> Self {
        Self::new()
    }
}
