//! Command reliability mechanisms shared by every peer (ADR-0041).
//!
//! The daemon keeps one command in flight, retransmits it on a fixed schedule with the same
//! sequence and the `RETRY` flag, and reports an exhausted command as **indeterminate** rather
//! than failed or successful. These types are pure: time arrives as a `u64` millisecond
//! parameter, there is no clock, I/O, allocator or logging, and the caller owns the wire.
//!
//! The schedule is anchored to each transmission. For a policy with `n` attempts the tracker
//! sends once, retransmits `n - 1` times, and then waits one further (capped) backoff interval
//! before declaring the outcome indeterminate. [`RetryPolicy::COMMAND`] therefore runs:
//!
//! | event | time |
//! |---|---|
//! | initial send | `t0` |
//! | retransmission 1 | `t0 + 300 ms` |
//! | retransmission 2 | `t0 + 900 ms` |
//! | indeterminate outcome | `t0 + 1900 ms` |
//!
//! [`RetryPolicy::EVENT`] runs at `t0`, `t0 + 500`, `t0 + 1500`, `t0 + 3500` and
//! `t0 + 5500 ms`, with the outcome indeterminate at `t0 + 7500 ms`.
//!
//! A Boot Id change is not a retry trigger. It discards the in-flight command and yields
//! [`AckOutcome::DiscardedOnBootChange`]; the caller reconciles by querying robot state and never
//! blindly re-sends a volatile command such as `DRIVE_DISTANCE` or `ROTATE_BY` (ADR-0031).
//!
//! [`ReplayWindow`] is the peer-side event mechanism: events are at-least-once, so the receiver
//! ignores a `(boot_id, seq)` it has already seen. It is bounded and allocation-free, and a new
//! Boot Id starts a clean window.
//!
//! [`Throttle`] is the flow-control mechanism: it merges advisory rate and pause hints from the
//! Bridge or the daemon, keeps the most restrictive result per stream, and expires each hint
//! against injected time. It is also bounded and allocation-free.

use crate::{
    frame::Seq,
    ids::Stream,
    payload::control::{Throttle as ThrottleHint, ThrottleAction},
};

/// The largest forward distance for which a wrapping deadline comparison is unambiguous.
const HALF_RANGE: u64 = u64::MAX / 2;

/// Returns `true` when `now_ms` has reached `deadline_ms`, comparing wrap-safely.
///
/// The two values are assumed to be within half the `u64` range of each other, which holds for
/// any millisecond interval a link can produce.
const fn reached(now_ms: u64, deadline_ms: u64) -> bool {
    now_ms.wrapping_sub(deadline_ms) <= HALF_RANGE
}

/// A retransmission schedule for one reliable command or event.
///
/// A retransmission delay doubles each time up to [`RetryPolicy::max_delay_ms`]; `attempts`
/// counts the initial transmission as the first attempt. The contract's named defaults are
/// [`RetryPolicy::COMMAND`] and [`RetryPolicy::EVENT`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RetryPolicy {
    /// The delay before the first retransmission, in milliseconds.
    first_delay_ms: u64,
    /// The multiplier applied to the previous delay for each retransmission.
    factor: u64,
    /// The maximum delay before any retransmission or the final outcome, in milliseconds.
    max_delay_ms: u64,
    /// The total number of transmissions, including the initial send; at least one.
    attempts: u8,
}

impl RetryPolicy {
    /// Commands: 300 ms, doubling to a 1 s cap, three attempts.
    ///
    /// Send at `t0`, retransmit at `t0 + 300 ms` and `t0 + 900 ms`, then declare the outcome
    /// indeterminate at `t0 + 1900 ms`, one further capped backoff interval after the final send.
    pub const COMMAND: Self = Self::new(300, 2, 1_000, 3);

    /// Events: 500 ms, doubling to a 2 s cap, five attempts.
    ///
    /// Send at `t0`, retransmit at `t0 + 500`, `t0 + 1500`, `t0 + 3500` and `t0 + 5500 ms`, then
    /// declare the event exhausted at `t0 + 7500 ms`.
    pub const EVENT: Self = Self::new(500, 2, 2_000, 5);

    /// Builds a policy, clamping the inputs into a usable range.
    ///
    /// `attempts` is at least one, `factor` is at least one, and `first_delay_ms` is capped to
    /// `max_delay_ms`.
    #[must_use]
    pub const fn new(first_delay_ms: u64, factor: u64, max_delay_ms: u64, attempts: u8) -> Self {
        Self {
            first_delay_ms: if first_delay_ms > max_delay_ms {
                max_delay_ms
            } else {
                first_delay_ms
            },
            factor: if factor == 0 { 1 } else { factor },
            max_delay_ms,
            attempts: if attempts == 0 { 1 } else { attempts },
        }
    }

    /// Returns the delay before the first retransmission, in milliseconds.
    #[must_use]
    pub const fn first_delay_ms(&self) -> u64 {
        self.first_delay_ms
    }

    /// Returns the backoff multiplier.
    #[must_use]
    pub const fn factor(&self) -> u64 {
        self.factor
    }

    /// Returns the delay cap, in milliseconds.
    #[must_use]
    pub const fn max_delay_ms(&self) -> u64 {
        self.max_delay_ms
    }

    /// Returns the total number of transmissions, the initial send included.
    #[must_use]
    pub const fn attempts(&self) -> u8 {
        self.attempts
    }

    /// Returns the delay after the `transmissions`-th transmission (one-based).
    ///
    /// The first transmission is followed by `first_delay_ms`, and each later one doubles the
    /// previous delay up to `max_delay_ms`. Zero returns the first delay as a defensive default;
    /// a live schedule never passes zero.
    #[must_use]
    pub const fn delay_after(&self, transmissions: u8) -> u64 {
        if transmissions <= 1 {
            return self.first_delay_ms;
        }
        let mut delay = self.first_delay_ms;
        let mut step = 1u8;
        while step < transmissions {
            if delay >= self.max_delay_ms {
                return self.max_delay_ms;
            }
            delay = delay.saturating_mul(self.factor);
            step += 1;
        }
        if delay > self.max_delay_ms {
            self.max_delay_ms
        } else {
            delay
        }
    }
}

/// The next action the sender should take for an in-flight command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AckOutcome {
    /// No action is due: either nothing is in flight or the next retry is still in the future.
    Idle,
    /// The retry interval elapsed. Retransmit [`AckOutcome::Retransmit::seq`] with
    /// `RETRY` set; the sequence is the original one.
    Retransmit {
        /// The original sequence, unchanged by retransmission.
        seq: Seq,
        /// The Boot Id the command was sent under.
        boot_id: u32,
    },
    /// The attempts are exhausted and the outcome is unknown.
    ///
    /// This is never success and never failure: the caller reconciles by querying robot state.
    Indeterminate {
        /// The sequence whose outcome is unknown.
        seq: Seq,
        /// The Boot Id the command was sent under.
        boot_id: u32,
    },
    /// A Boot Id change discarded the in-flight command.
    ///
    /// Do not automatically re-send it. Volatile commands such as `DRIVE_DISTANCE` and
    /// `ROTATE_BY` are not idempotent; reconcile by querying robot state instead.
    DiscardedOnBootChange {
        /// The sequence that was discarded.
        seq: Seq,
    },
}

impl AckOutcome {
    /// Returns the sequence this outcome refers to, if any.
    #[must_use]
    pub const fn seq(self) -> Option<Seq> {
        match self {
            Self::Idle => None,
            Self::Retransmit { seq, .. } | Self::Indeterminate { seq, .. } | Self::DiscardedOnBootChange { seq } => {
                Some(seq)
            }
        }
    }
}

/// The tracker's in-flight state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// Nothing is in flight; the sender may begin a command.
    Idle,
    /// Exactly one command is in flight.
    InFlight {
        /// The tracked command's sequence.
        seq: Seq,
        /// The Boot Id the command was sent under.
        boot_id: u32,
        /// The number of transmissions made, including the initial send.
        transmissions: u8,
        /// The millisecond at which the next retransmission or the final outcome is due.
        next_at_ms: u64,
        /// Whether the next due deadline declares indeterminacy instead of retransmitting.
        awaiting_outcome: bool,
    },
}

/// Tracks the one command in flight and when to retransmit or report it indeterminate.
///
/// The tracker enforces one command in flight, schedules retransmissions without a clock, and
/// treats exhaustion as an unknown outcome. It never decides success or failure: only a matching
/// [`AckTracker::on_ack`] retires a command successfully.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AckTracker {
    /// The retransmission schedule.
    policy: RetryPolicy,
    /// The current in-flight command, if any.
    state: State,
}

impl AckTracker {
    /// Creates a tracker for `policy` with nothing in flight.
    #[must_use]
    pub const fn new(policy: RetryPolicy) -> Self {
        Self {
            policy,
            state: State::Idle,
        }
    }

    /// Returns the tracker's retransmission schedule.
    #[must_use]
    pub const fn policy(&self) -> RetryPolicy {
        self.policy
    }

    /// Returns `true` while a command is in flight.
    #[must_use]
    pub const fn is_in_flight(&self) -> bool {
        matches!(self.state, State::InFlight { .. })
    }

    /// Returns the in-flight sequence, if any.
    #[must_use]
    pub const fn in_flight_seq(&self) -> Option<Seq> {
        match self.state {
            State::InFlight { seq, .. } => Some(seq),
            State::Idle => None,
        }
    }

    /// Begins tracking `seq` under `boot_id` at `now_ms`.
    ///
    /// Returns `false` and leaves the current command untouched when one is already in flight, so
    /// the sender can never have two commands outstanding. A successful begin schedules the first
    /// retransmission at `now_ms + first_delay_ms`.
    #[must_use]
    pub const fn begin(&mut self, seq: Seq, boot_id: u32, now_ms: u64) -> bool {
        if self.is_in_flight() {
            return false;
        }
        let transmissions = 1u8;
        self.state = State::InFlight {
            seq,
            boot_id,
            transmissions,
            next_at_ms: now_ms.wrapping_add(self.policy.delay_after(transmissions)),
            awaiting_outcome: transmissions >= self.policy.attempts(),
        };
        true
    }

    /// Retires the in-flight command when `seq` matches, returning whether it was retired.
    ///
    /// An ACK for any other sequence is ignored, and a command is only ever retired here or
    /// discarded by a Boot Id change.
    #[must_use]
    pub fn on_ack(&mut self, seq: Seq) -> bool {
        match self.state {
            State::InFlight { seq: in_flight, .. } => {
                if in_flight == seq {
                    self.state = State::Idle;
                    true
                } else {
                    false
                }
            }
            State::Idle => false,
        }
    }

    /// Discards the in-flight command when the robot's `boot_id` changed.
    ///
    /// Returns the explicit "do not retry" outcome once, or [`None`] when no Boot Id change
    /// discarded anything. Observing the same Boot Id is a no-op. After a change the tracker is
    /// idle, so [`AckTracker::poll`] cannot produce an automatic retransmission.
    #[must_use]
    pub const fn on_boot_id(&mut self, boot_id: u32) -> Option<AckOutcome> {
        match self.state {
            State::InFlight {
                seq,
                boot_id: in_flight_boot,
                ..
            } => {
                if in_flight_boot == boot_id {
                    None
                } else {
                    self.state = State::Idle;
                    Some(AckOutcome::DiscardedOnBootChange { seq })
                }
            }
            State::Idle => None,
        }
    }

    /// Advances the schedule at `now_ms`, reporting the next action.
    ///
    /// Returns [`AckOutcome::Retransmit`] when a retransmission is due, or
    /// [`AckOutcome::Indeterminate`] once the attempts and the final grace interval are
    /// exhausted. The tracker owns no I/O; the caller integrates the action onto the wire.
    #[must_use]
    pub const fn poll(&mut self, now_ms: u64) -> AckOutcome {
        let State::InFlight {
            seq,
            boot_id,
            transmissions,
            next_at_ms,
            awaiting_outcome,
        } = self.state
        else {
            return AckOutcome::Idle;
        };

        if !reached(now_ms, next_at_ms) {
            return AckOutcome::Idle;
        }

        if awaiting_outcome {
            self.state = State::Idle;
            return AckOutcome::Indeterminate { seq, boot_id };
        }

        let transmissions = transmissions.saturating_add(1);
        self.state = State::InFlight {
            seq,
            boot_id,
            transmissions,
            next_at_ms: now_ms.wrapping_add(self.policy.delay_after(transmissions)),
            awaiting_outcome: transmissions >= self.policy.attempts(),
        };
        AckOutcome::Retransmit { seq, boot_id }
    }
}

/// A bounded replay window that deduplicates events keyed by `(boot_id, seq)`.
///
/// Events are at-least-once: the robot retransmits until the peer ACKs, so the peer sees the same
/// sequence more than once when an ACK is lost. A retransmit of an already-seen `(boot_id, seq)`
/// is ignored; a new Boot Id starts a clean window because sequences restart at zero.
///
/// The window holds the last [`ReplayWindow::CAPACITY`] accepted sequences in a ring. It is fixed
/// size and allocates nothing. A window this small is sufficient because a sender retries at most
/// five times before it stops, so a sequence older than 64 accepted events is outside any live
/// retransmission.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReplayWindow {
    /// The Boot Id the window currently belongs to, if any sequence has been observed.
    boot_id: Option<u32>,
    /// The accepted sequences in insertion order; only `seqs[..len]` is meaningful.
    seqs: [u16; Self::CAPACITY],
    /// The number of accepted sequences, capped at [`ReplayWindow::CAPACITY`].
    len: usize,
    /// The slot the next sequence overwrites once the window is full.
    next: usize,
}

impl ReplayWindow {
    /// The number of remembered sequences.
    pub const CAPACITY: usize = 64;

    /// Creates an empty window that has not observed a Boot Id yet.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            boot_id: None,
            seqs: [0; Self::CAPACITY],
            len: 0,
            next: 0,
        }
    }

    /// Returns the Boot Id the window belongs to, or [`None`] before the first sequence.
    #[must_use]
    pub const fn boot_id(&self) -> Option<u32> {
        self.boot_id
    }

    /// Returns the number of remembered sequences.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` when no sequence is remembered.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns `true` when `(boot_id, seq)` is already in the window.
    ///
    /// A different Boot Id is never a duplicate: it belongs to a window that has not started.
    #[must_use]
    pub fn contains(&self, boot_id: u32, seq: Seq) -> bool {
        self.boot_id == Some(boot_id) && self.seqs[..self.len].contains(&seq.get())
    }

    /// Starts a clean window when `boot_id` differs from the current one.
    ///
    /// Returns `true` when the window was reset. Observing the same Boot Id is a no-op.
    pub fn observe_boot_id(&mut self, boot_id: u32) -> bool {
        if self.boot_id == Some(boot_id) {
            return false;
        }
        self.boot_id = Some(boot_id);
        self.len = 0;
        self.next = 0;
        true
    }

    /// Records `(boot_id, seq)` and returns `true` when it is new.
    ///
    /// A Boot Id change clears the window first. A sequence already present returns `false` and
    /// changes nothing; a new sequence evicts the oldest one once the window is full.
    pub fn accept(&mut self, boot_id: u32, seq: Seq) -> bool {
        self.observe_boot_id(boot_id);
        if self.seqs[..self.len].contains(&seq.get()) {
            return false;
        }
        if self.len < Self::CAPACITY {
            self.seqs[self.len] = seq.get();
            self.len += 1;
        } else {
            self.seqs[self.next] = seq.get();
            self.next = (self.next + 1) % Self::CAPACITY;
        }
        true
    }

    /// Forgets every sequence while keeping the current Boot Id.
    pub const fn clear(&mut self) {
        self.len = 0;
        self.next = 0;
    }
}

impl Default for ReplayWindow {
    fn default() -> Self {
        Self::new()
    }
}

/// The effective restriction a [`Throttle`] applies to one stream.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ThrottleState {
    /// No hint is active; the Requested Rate or the configured baseline applies unchanged.
    #[default]
    Unrestricted,
    /// The active hints cap emission at this rate in centi-Hz; the value is never zero.
    Limited(u16),
    /// Emission is paused; a pause beats every rate.
    Paused,
}

/// The merged rate and pause state for one scope: the global `0xFFFF` scope or one stream.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct ThrottleSlot {
    /// The active maximum rate in centi-Hz, if any.
    rate_chz: Option<u16>,
    /// The time at which the rate hint expires.
    rate_expires_ms: u64,
    /// Whether a pause hint is active.
    paused: bool,
    /// The time at which the pause hint expires.
    pause_expires_ms: u64,
}

impl ThrottleSlot {
    /// Returns the rate hint if it has not expired at `now_ms`.
    const fn rate(&self, now_ms: u64) -> Option<u16> {
        match self.rate_chz {
            Some(rate) if !reached(now_ms, self.rate_expires_ms) => Some(rate),
            _ => None,
        }
    }

    /// Returns `true` when a pause hint has not expired at `now_ms`.
    const fn is_paused(&self, now_ms: u64) -> bool {
        self.paused && !reached(now_ms, self.pause_expires_ms)
    }

    /// Forgets every hint that has expired at `now_ms`.
    const fn expire(&mut self, now_ms: u64) {
        if self.rate_chz.is_some() && reached(now_ms, self.rate_expires_ms) {
            self.rate_chz = None;
        }
        if self.paused && reached(now_ms, self.pause_expires_ms) {
            self.paused = false;
            self.pause_expires_ms = 0;
        }
    }

    /// Clears a pause without touching the rate hint.
    const fn resume(&mut self) {
        self.paused = false;
        self.pause_expires_ms = 0;
    }
}

/// The merged `THROTTLE` state for every stream (ADR-0030, ADR-0041).
///
/// The mechanism holds one slot for the global `0xFFFF` scope and one for each implemented
/// stream. It merges the active hints by taking the most restrictive result: a pause beats any
/// rate, and a rate is a maximum, so the lowest active rate wins. Each hint carries a TTL that is
/// checked against the injected `now_ms`; an expired hint is ignored immediately and is forgotten
/// by [`Throttle::tick`]. It has no clock, I/O or allocation.
///
/// A `SET_RATE` of zero cannot mean "emit at zero"; it is stored as a pause, the most restrictive
/// state the protocol can express. A `RESUME` for `0xFFFF` clears the global pause and every
/// stream pause; a `RESUME` for one stream clears only that stream's pause, so a global pause
/// still wins.
///
/// A TTL of zero expires immediately, so it has no effect.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Throttle {
    /// The hint state addressed by `0xFFFF`.
    global: ThrottleSlot,
    /// The per-stream hint state, indexed by [`Stream::index`].
    streams: [ThrottleSlot; Stream::COUNT],
}

impl Throttle {
    /// Creates a throttle with no active hints.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            global: ThrottleSlot {
                rate_chz: None,
                rate_expires_ms: 0,
                paused: false,
                pause_expires_ms: 0,
            },
            streams: [ThrottleSlot {
                rate_chz: None,
                rate_expires_ms: 0,
                paused: false,
                pause_expires_ms: 0,
            }; Stream::COUNT],
        }
    }

    /// Applies one hint at `now_ms`, returning `false` when it addressed no implemented stream.
    ///
    /// A `SET_RATE` with a zero rate is applied as a pause. A `RESUME` is immediate and is not
    /// stored; a later `PAUSE` re-applies the pause. An unknown stream id is ignored.
    pub fn apply(&mut self, hint: &ThrottleHint, now_ms: u64) -> bool {
        let expires_ms = now_ms.saturating_add(u64::from(hint.ttl_ms));
        match hint.action {
            ThrottleAction::Resume => {
                if hint.stream_id == ThrottleHint::ALL_STREAMS {
                    self.global.resume();
                    for slot in &mut self.streams {
                        slot.resume();
                    }
                } else {
                    let Some(slot) = Self::slot_mut(&mut self.streams, hint.stream_id) else {
                        return false;
                    };
                    slot.resume();
                }
                true
            }
            ThrottleAction::Pause => {
                let Some(slot) = Self::slot_mut_global(self, hint.stream_id) else {
                    return false;
                };
                slot.paused = true;
                slot.pause_expires_ms = expires_ms;
                true
            }
            ThrottleAction::SetRate if hint.rate_chz == 0 => {
                let Some(slot) = Self::slot_mut_global(self, hint.stream_id) else {
                    return false;
                };
                slot.paused = true;
                slot.pause_expires_ms = expires_ms;
                true
            }
            ThrottleAction::SetRate => {
                let Some(slot) = Self::slot_mut_global(self, hint.stream_id) else {
                    return false;
                };
                slot.rate_chz = Some(hint.rate_chz);
                slot.rate_expires_ms = expires_ms;
                true
            }
        }
    }

    /// Forgets every hint that has expired at `now_ms`.
    ///
    /// [`Throttle::effective`] already ignores expired hints, so this only reclaims their slots.
    pub fn tick(&mut self, now_ms: u64) {
        self.global.expire(now_ms);
        for slot in &mut self.streams {
            slot.expire(now_ms);
        }
    }

    /// Returns the most restrictive active state for `stream` at `now_ms`.
    #[must_use]
    pub fn effective(&self, stream: Stream, now_ms: u64) -> ThrottleState {
        let local = &self.streams[stream.index()];
        if self.global.is_paused(now_ms) || local.is_paused(now_ms) {
            return ThrottleState::Paused;
        }
        let global_rate = self.global.rate(now_ms);
        let local_rate = local.rate(now_ms);
        match (global_rate, local_rate) {
            (Some(global), Some(local)) => ThrottleState::Limited(global.min(local)),
            (Some(rate), None) | (None, Some(rate)) => ThrottleState::Limited(rate),
            (None, None) => ThrottleState::Unrestricted,
        }
    }

    /// Returns the per-stream slot for `stream_id`, or [`None`] for an unknown id.
    fn slot_mut(streams: &mut [ThrottleSlot; Stream::COUNT], stream_id: u16) -> Option<&mut ThrottleSlot> {
        let stream = Stream::from_wire(stream_id)?;
        Some(&mut streams[stream.index()])
    }

    /// Returns the slot addressed by `stream_id`, including the global `0xFFFF` slot.
    fn slot_mut_global(&mut self, stream_id: u16) -> Option<&mut ThrottleSlot> {
        if stream_id == ThrottleHint::ALL_STREAMS {
            Some(&mut self.global)
        } else {
            Self::slot_mut(&mut self.streams, stream_id)
        }
    }
}
