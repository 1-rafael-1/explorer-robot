//! The link's timing contract as named constants (ADR-0030, ADR-0033, ADR-0041).
//!
//! The values are the spec's: a 5 s lease, a 2 s Deadman, 1 s `PING`, `HELLO` every 1 s up to five
//! attempts, command retries at 300 ms doubling to a 1 s cap with three attempts, event retries at
//! 500 ms doubling to a 2 s cap with five attempts, an 8-entry event ring, `THROTTLE` repetition
//! every 500 ms and a 50 ms frame timeout.
//!
//! [`Config`] carries those defaults so a host test can compress time; the firmware passes a
//! `Config` into [`crate::LinkCore::new`] rather than editing these constants.

use remote_protocol::{Caps, ids::Stream};

/// The ADR-0033 link baud.
pub const BAUD: u32 = 921_600;

/// The lease silence after which the peer is expired.
pub const LEASE_MS: u64 = 5_000;

/// The peer silence after which the one-shot `peer_lost` edge fires.
pub const DEADMAN_MS: u64 = 2_000;

/// The `PING` interval while a Session is open.
pub const PING_INTERVAL_MS: u64 = 1_000;

/// The `HELLO` retry interval.
pub const HELLO_INTERVAL_MS: u64 = 1_000;

/// The number of `HELLO` attempts before the daemon gives up.
pub const HELLO_ATTEMPTS: u8 = 5;

/// The first command retry delay.
pub const COMMAND_RETRY_MS: u64 = 300;

/// The command retry backoff factor.
pub const COMMAND_RETRY_FACTOR: u32 = 2;

/// The command retry delay cap.
pub const COMMAND_RETRY_CAP_MS: u64 = 1_000;

/// The number of command attempts before the outcome is indeterminate.
pub const COMMAND_ATTEMPTS: u8 = 3;

/// The first event retry delay.
pub const EVENT_RETRY_MS: u64 = 500;

/// The event retry backoff factor.
pub const EVENT_RETRY_FACTOR: u32 = 2;

/// The event retry delay cap.
pub const EVENT_RETRY_CAP_MS: u64 = 2_000;

/// The number of event attempts before the sender stops retrying.
///
/// The event stays in the ring, so a later re-attach gives it a fresh schedule; it is never
/// silently discarded after exhaustion.
pub const EVENT_ATTEMPTS: u8 = 5;

/// The number of entries in the event ring.
pub const EVENT_RING: usize = 8;

/// The interval at which an active `THROTTLE` hint is repeated.
pub const THROTTLE_REPEAT_MS: u64 = 500;

/// The frame assembly timeout.
pub const FRAME_TIMEOUT_MS: u32 = 50;

/// The robot's baseline cadence per stream, in centi-Hz, indexed by [`Stream::index`].
///
/// The values follow ADR-0031: `POWER` and `CALIBRATION` report on change at least every 5 s,
/// `MOTION` idles at 1 Hz, and `ACTIVITY` reports at 1 Hz while `PERCEPTION` and `LIDAR_CLOUD`
/// run at 10 Hz. A Session's Requested Rate overrides these once ticket 08 applies it; until then
/// they are only what `HELLO_ACK` advertises.
pub const DEFAULT_RATES_CHZ: [u16; Stream::COUNT] = [
    20,    // POWER: 5 s heartbeat
    100,   // MOTION: 1 Hz idle
    100,   // ACTIVITY: 1 Hz
    1_000, // PERCEPTION: 10 Hz
    1_000, // LIDAR_CLOUD: 10 Hz
    20,    // CALIBRATION: 5 s heartbeat
];

/// The timing values `LinkCore` runs with.
///
/// Every field defaults to the matching constant so host tests can override just the ones they
/// need to compress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// The lease silence after which the peer is expired.
    pub lease_ms: u64,
    /// The peer silence after which the one-shot `peer_lost` edge fires.
    pub deadman_ms: u64,
    /// The `PING` interval while a Session is open.
    pub ping_interval_ms: u64,
    /// The `HELLO` retry interval.
    pub hello_interval_ms: u64,
    /// The number of `HELLO` attempts.
    pub hello_attempts: u8,
    /// The first command retry delay.
    pub command_retry_ms: u64,
    /// The command retry backoff factor.
    pub command_retry_factor: u32,
    /// The command retry delay cap.
    pub command_retry_cap_ms: u64,
    /// The number of command attempts.
    pub command_attempts: u8,
    /// The first event retry delay.
    pub event_retry_ms: u64,
    /// The event retry backoff factor.
    pub event_retry_factor: u32,
    /// The event retry delay cap.
    pub event_retry_cap_ms: u64,
    /// The number of event attempts.
    pub event_attempts: u8,
    /// The number of entries in the event ring.
    ///
    /// The ring's backing array is [`EVENT_RING`] entries; a host test may set this lower to shrink
    /// the usable capacity, but it can never exceed the compile-time bound.
    pub event_ring: usize,
    /// The interval at which an active `THROTTLE` hint is repeated.
    pub throttle_repeat_ms: u64,
    /// The frame assembly timeout.
    pub frame_timeout_ms: u32,
    /// The capabilities the robot advertises and intersects with a peer's `HELLO`.
    pub robot_caps: Caps,
    /// The baseline cadence per stream, in centi-Hz, indexed by [`Stream::index`].
    ///
    /// A host test overrides this to make `HELLO_ACK` deterministic; the firmware leaves it at
    /// [`DEFAULT_RATES_CHZ`].
    pub default_rates_chz: [u16; Stream::COUNT],
}

impl Config {
    /// The contract defaults; identical to [`Config::default`].
    pub const DEFAULT: Self = Self {
        lease_ms: LEASE_MS,
        deadman_ms: DEADMAN_MS,
        ping_interval_ms: PING_INTERVAL_MS,
        hello_interval_ms: HELLO_INTERVAL_MS,
        hello_attempts: HELLO_ATTEMPTS,
        command_retry_ms: COMMAND_RETRY_MS,
        command_retry_factor: COMMAND_RETRY_FACTOR,
        command_retry_cap_ms: COMMAND_RETRY_CAP_MS,
        command_attempts: COMMAND_ATTEMPTS,
        event_retry_ms: EVENT_RETRY_MS,
        event_retry_factor: EVENT_RETRY_FACTOR,
        event_retry_cap_ms: EVENT_RETRY_CAP_MS,
        event_attempts: EVENT_ATTEMPTS,
        event_ring: EVENT_RING,
        throttle_repeat_ms: THROTTLE_REPEAT_MS,
        frame_timeout_ms: FRAME_TIMEOUT_MS,
        robot_caps: Caps::from_bits(0),
        default_rates_chz: DEFAULT_RATES_CHZ,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self::DEFAULT
    }
}
