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

/// How many failed `auth` attempts close an address's gate (ADR-0037's five).
pub const AUTH_FAILURE_LIMIT: usize = 5;

/// How long an address is refused after reaching the limit (ADR-0037's 60 s).
pub const AUTH_REFUSAL: Duration = Duration::from_mins(1);

/// How many addresses the limiter tracks at once.
///
/// A connection can rotate its source address — a LAN host holding a whole IPv6 prefix can mint them
/// without limit — and every failed attempt would otherwise leave a record behind forever. The map
/// is capped: expired records are reaped on every failure, and if every record is still live the
/// least recently active one is evicted, so the limiter's memory stays bounded while a new address
/// is still counted.
const AUTH_TRACKED_ADDRESSES: usize = 1024;

/// Compares a stored token with a candidate without an early exit.
///
/// The primitive is `bridge-net`'s shared [`bridge_net::constant_time_eq`], so the app gate and the
/// preamble's proof check cannot drift into different comparison shapes. The length difference is
/// folded in first, and the optimiser barrier stays inside that one implementation.
#[must_use]
pub fn tokens_match(stored: &str, candidate: &str) -> bool {
    bridge_net::constant_time_eq(stored.as_bytes(), candidate.as_bytes())
}

/// One address's failed attempts and refusal window.
#[derive(Debug, Default)]
struct AddressFailures {
    /// The failure timestamps inside the counting window, oldest first.
    failures: VecDeque<Instant>,
    /// The end of the refusal window, while one is open.
    refused_until: Option<Instant>,
    /// When this address last failed, which orders eviction when the map is full.
    last_seen: Option<Instant>,
}

impl AddressFailures {
    /// Returns whether the refusal window is still open at `now`.
    fn refused(&self, now: Instant) -> bool {
        self.refused_until.is_some_and(|until| now < until)
    }

    /// Returns whether the failure at `at` still counts at `now` (ADR-0037's 60 s window).
    fn in_window(at: Instant, now: Instant) -> bool {
        now.saturating_duration_since(at) < AUTH_FAILURE_WINDOW
    }

    /// Ends a refusal whose window has closed and drops the failures that left the counting window,
    /// returning whether the address is still refused.
    fn settle(&mut self, now: Instant) -> bool {
        if self.refused(now) {
            return true;
        }
        // A closed refusal starts the count again (ADR-0037), and an aged failure no longer counts.
        self.refused_until = None;
        self.failures.retain(|at| Self::in_window(*at, now));
        false
    }

    /// Returns whether the record can no longer decide anything at `now`, so it is only memory.
    fn is_expired(&self, now: Instant) -> bool {
        !self.refused(now) && self.failures.iter().all(|at| !Self::in_window(*at, now))
    }
}

/// The per-address auth failure limiter.
///
/// A connection from a loopback address never reaches this gate. One from a non-loopback address
/// that fails five `auth` attempts inside a minute is refused for a minute, and a later success
/// starts the count again (ADR-0037). The limiter is shared by every connection of the process, so
/// a probe cannot mint itself a fresh budget by reconnecting. It tracks at most
/// `AUTH_TRACKED_ADDRESSES` addresses: expired records are reaped, and if every record is still live
/// the least recently active one is evicted.
#[derive(Debug, Default)]
pub struct AuthLimiter {
    /// The per-address state.
    addresses: Mutex<HashMap<IpAddr, AddressFailures>>,
}

impl AuthLimiter {
    /// Returns whether `address` is inside a refusal window.
    pub(crate) fn is_refused(&self, address: IpAddr) -> bool {
        let Ok(mut addresses) = self.addresses.lock() else {
            // A poisoned limiter cannot prove an address is refused; the caller still enforces the
            // token check, so fail open here rather than locking every LAN user out forever.
            return false;
        };
        let now = Instant::now();
        let Some(state) = addresses.get_mut(&address) else {
            return false;
        };
        let refused = state.settle(now);
        if state.is_expired(now) {
            addresses.remove(&address);
        }
        refused
    }

    /// Records one failed `auth` attempt from `address` and opens the refusal window at the limit.
    pub(crate) fn record_failure(&self, address: IpAddr) {
        let Ok(mut addresses) = self.addresses.lock() else {
            return;
        };
        let now = Instant::now();
        // Reap the records that can no longer decide anything, so a stale entry does not linger and
        // an address-rotating probe cannot grow the map.
        Self::prune_expired(&mut addresses, now);
        if !addresses.contains_key(&address) && addresses.len() >= AUTH_TRACKED_ADDRESSES {
            // Every record is still live: drop the least recently active one, so the map stays
            // bounded and the new address is still counted instead of escaping the limit.
            Self::evict_stalest(&mut addresses);
        }
        let state = addresses.entry(address).or_default();
        state.last_seen = Some(now);
        if state.settle(now) {
            return;
        }
        state.failures.push_back(now);
        if state.failures.len() >= AUTH_FAILURE_LIMIT {
            state.failures.clear();
            state.refused_until = Some(now + AUTH_REFUSAL);
        }
    }

    /// Forgets an address's failures after a successful authentication (ADR-0037).
    pub(crate) fn clear(&self, address: IpAddr) {
        let Ok(mut addresses) = self.addresses.lock() else {
            return;
        };
        addresses.remove(&address);
    }

    /// Drops every record that can no longer decide anything at `now`.
    fn prune_expired(addresses: &mut HashMap<IpAddr, AddressFailures>, now: Instant) {
        addresses.retain(|_, state| !state.is_expired(now));
    }

    /// Drops the record that has gone longest without a failure.
    fn evict_stalest(addresses: &mut HashMap<IpAddr, AddressFailures>) {
        let stalest = addresses
            .iter()
            .min_by_key(|(_, state)| state.last_seen)
            .map(|(address, _)| *address);
        if let Some(stalest) = stalest {
            addresses.remove(&stalest);
        }
    }
}

#[cfg(test)]
impl AuthLimiter {
    /// The number of address records held, for the retention tests.
    fn tracked(&self) -> usize {
        self.addresses.lock().map_or(0, |addresses| addresses.len())
    }

    /// Whether the limiter holds a record for `address`.
    fn tracks(&self, address: IpAddr) -> bool {
        self.addresses
            .lock()
            .is_ok_and(|addresses| addresses.contains_key(&address))
    }
}

#[cfg(test)]
mod tests {
    use std::{
        collections::{HashMap, VecDeque},
        net::{IpAddr, Ipv4Addr},
        time::{Duration, Instant},
    };

    use super::{
        AUTH_FAILURE_LIMIT, AUTH_FAILURE_WINDOW, AUTH_REFUSAL, AUTH_TRACKED_ADDRESSES, AddressFailures, AuthLimiter,
        tokens_match,
    };

    /// The `index`-th distinct address, for enumerating a map's worth of them.
    fn address(index: usize) -> IpAddr {
        IpAddr::V4(Ipv4Addr::from(u32::try_from(index).unwrap_or(u32::MAX)))
    }

    /// A different token never matches, including a length difference.
    #[test]
    fn a_different_token_does_not_match() {
        assert!(!tokens_match("abc", "abd"));
        assert!(!tokens_match("abc", "abcd"));
        assert!(!tokens_match("", "a"));
        assert!(tokens_match("abc", "abc"));
    }

    /// Pins ADR-0037's documented failure limit and windows, and ADR-0039's tracked-address cap.
    #[test]
    fn the_limits_are_the_adrs() {
        assert_eq!(AUTH_FAILURE_LIMIT, 5);
        assert_eq!(AUTH_FAILURE_WINDOW, Duration::from_mins(1));
        assert_eq!(AUTH_REFUSAL, Duration::from_mins(1));
        assert_eq!(AUTH_TRACKED_ADDRESSES, 1024);
    }

    /// A connection that rotates its source address cannot grow the tracked-address map past its
    /// cap.
    #[test]
    fn the_tracked_address_map_is_bounded() {
        let limiter = AuthLimiter::default();
        for index in 0..AUTH_TRACKED_ADDRESSES * 2 {
            limiter.record_failure(address(index));
        }
        assert_eq!(limiter.tracked(), AUTH_TRACKED_ADDRESSES);
    }

    /// A record whose windows have both closed is dropped the next time its address reaches the
    /// gate, even while the map is below its cap.
    #[test]
    fn an_expired_record_is_dropped_when_its_address_asks_again() {
        let limiter = AuthLimiter::default();
        if let Ok(mut addresses) = limiter.addresses.lock() {
            addresses.insert(
                address(1),
                AddressFailures {
                    // A refusal window that ends immediately.
                    refused_until: Some(Instant::now()),
                    ..AddressFailures::default()
                },
            );
        }

        assert!(!limiter.is_refused(address(1)));
        assert!(!limiter.tracks(address(1)), "the dead record is not kept");
    }

    /// Reaping drops records whose refusal and failure windows have both expired, and keeps a live
    /// refusal window.
    #[test]
    fn reaping_drops_expired_records_and_keeps_live_refusals() {
        let Some(now) = Instant::now().checked_add(Duration::from_mins(2)) else {
            return;
        };
        let Some(live_until) = now.checked_add(Duration::from_secs(1)) else {
            return;
        };

        let mut addresses = HashMap::new();
        // The refusal never opened, and the last failure is older than the counting window.
        addresses.insert(
            address(1),
            AddressFailures {
                failures: VecDeque::from([Instant::now()]),
                ..AddressFailures::default()
            },
        );
        // The refusal window has ended, and the lazily-cleared record no longer decides anything.
        addresses.insert(
            address(2),
            AddressFailures {
                refused_until: Some(now),
                ..AddressFailures::default()
            },
        );
        // The refusal window is still open.
        addresses.insert(
            address(3),
            AddressFailures {
                refused_until: Some(live_until),
                ..AddressFailures::default()
            },
        );

        AuthLimiter::prune_expired(&mut addresses, now);

        assert!(!addresses.contains_key(&address(1)), "an aged failure record is reaped");
        assert!(!addresses.contains_key(&address(2)), "an ended refusal is reaped");
        assert!(addresses.contains_key(&address(3)), "a live refusal window is kept");
    }

    /// A saturated map evicts its stalest record, and the new address still reaches the limit
    /// instead of escaping the per-address throttle.
    #[test]
    fn a_saturated_map_evicts_the_stalest_record() {
        let limiter = AuthLimiter::default();
        for index in 0..AUTH_TRACKED_ADDRESSES {
            for _ in 0..AUTH_FAILURE_LIMIT {
                limiter.record_failure(address(index));
            }
        }
        assert_eq!(limiter.tracked(), AUTH_TRACKED_ADDRESSES);

        for _ in 0..AUTH_FAILURE_LIMIT {
            limiter.record_failure(address(AUTH_TRACKED_ADDRESSES));
        }

        assert_eq!(limiter.tracked(), AUTH_TRACKED_ADDRESSES);
        assert!(
            limiter.is_refused(address(AUTH_TRACKED_ADDRESSES)),
            "the per-address limit still holds under saturation"
        );
    }
}
