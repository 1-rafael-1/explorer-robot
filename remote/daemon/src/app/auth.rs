//! The app boundary's authentication rules: constant-time token comparison and the per-IP
//! failure limiter (ADR-0037's numbers, applied to the app WebSocket by ADR-0039).

use std::{
    collections::{HashMap, VecDeque},
    net::IpAddr,
    sync::Mutex,
    time::{Duration, Instant},
};

/// The window in which failed `auth` attempts are counted (ADR-0037's 60 s).
pub const AUTH_FAILURE_WINDOW: Duration = Duration::from_mins(1);

/// How many failed `auth` attempts close a peer's gate (ADR-0037's five).
pub const AUTH_FAILURE_LIMIT: usize = 5;

/// How long a peer address is refused after reaching the limit (ADR-0037's 60 s).
pub const AUTH_REFUSAL: Duration = Duration::from_mins(1);

/// Compares a stored token with a candidate without an early exit.
///
/// The length difference is folded into the accumulator first, so a different-length candidate
/// still walks the longer of the two strings. `black_box` keeps the optimiser from turning the
/// fold into a short-circuiting comparison (the same shape as `bridge-core`'s proof comparison).
#[must_use]
pub fn tokens_match(stored: &str, candidate: &str) -> bool {
    let stored = stored.as_bytes();
    let candidate = candidate.as_bytes();
    let mut difference = stored.len() ^ candidate.len();
    let longest = stored.len().max(candidate.len());
    for index in 0..longest {
        let left = stored.get(index).copied().unwrap_or(0);
        let right = candidate.get(index).copied().unwrap_or(0);
        difference |= usize::from(left ^ right);
    }
    core::hint::black_box(difference) == 0
}

/// One peer address's failed attempts and refusal window.
#[derive(Debug, Default)]
struct PeerFailures {
    /// The failure timestamps inside the counting window, oldest first.
    failures: VecDeque<Instant>,
    /// The end of the refusal window, while one is open.
    refused_until: Option<Instant>,
}

/// The per-peer-address auth failure limiter.
///
/// A loopback peer never reaches this gate. A non-loopback peer that fails five `auth` attempts
/// inside a minute is refused for a minute, and a later success starts the count again
/// (ADR-0037). The limiter is shared by every connection of the process, so a probe cannot mint
/// itself a fresh budget by reconnecting.
#[derive(Debug, Default)]
pub struct AuthLimiter {
    /// The per-address state.
    peers: Mutex<HashMap<IpAddr, PeerFailures>>,
}

impl AuthLimiter {
    /// Returns whether `peer` is inside a refusal window.
    pub(crate) fn is_refused(&self, peer: IpAddr) -> bool {
        let Ok(mut peers) = self.peers.lock() else {
            // A poisoned limiter cannot prove a peer is refused; the caller still enforces the
            // token check, so fail open here rather than locking every LAN user out forever.
            return false;
        };
        let now = Instant::now();
        let Some(state) = peers.get_mut(&peer) else {
            return false;
        };
        match state.refused_until {
            Some(until) if now < until => true,
            Some(_) => {
                state.refused_until = None;
                state.failures.clear();
                false
            }
            None => false,
        }
    }

    /// Records one failed `auth` attempt and opens the refusal window at the limit.
    pub(crate) fn record_failure(&self, peer: IpAddr) {
        let Ok(mut peers) = self.peers.lock() else {
            return;
        };
        let now = Instant::now();
        let state = peers.entry(peer).or_default();
        if state.refused_until.is_some_and(|until| now < until) {
            return;
        }
        state.refused_until = None;
        while state
            .failures
            .front()
            .is_some_and(|at| now.saturating_duration_since(*at) >= AUTH_FAILURE_WINDOW)
        {
            state.failures.pop_front();
        }
        state.failures.push_back(now);
        if state.failures.len() >= AUTH_FAILURE_LIMIT {
            state.failures.clear();
            state.refused_until = Some(now + AUTH_REFUSAL);
        }
    }

    /// Forgets a peer's failures after a successful authentication (ADR-0037).
    pub(crate) fn clear(&self, peer: IpAddr) {
        let Ok(mut peers) = self.peers.lock() else {
            return;
        };
        peers.remove(&peer);
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{AUTH_FAILURE_LIMIT, AUTH_FAILURE_WINDOW, AUTH_REFUSAL, tokens_match};

    /// A different token never matches, including a length difference.
    #[test]
    fn a_different_token_does_not_match() {
        assert!(!tokens_match("abc", "abd"));
        assert!(!tokens_match("abc", "abcd"));
        assert!(!tokens_match("", "a"));
        assert!(tokens_match("abc", "abc"));
    }

    /// Pins ADR-0037's documented failure limit and windows.
    #[test]
    fn the_failure_limit_is_five() {
        assert_eq!(AUTH_FAILURE_LIMIT, 5);
        assert_eq!(AUTH_FAILURE_WINDOW, Duration::from_mins(1));
        assert_eq!(AUTH_REFUSAL, Duration::from_mins(1));
    }
}
