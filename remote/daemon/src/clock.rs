//! The process-wide monotonic millisecond clock the two adapters share (ADR-0039).
//!
//! The relay takes plain millisecond timestamps instead of reading a clock, so every adapter that
//! feeds it must use the same scale: an app heartbeat stamped by the app adapter is compared with
//! the app-frozen timer advanced by the link adapter. A single process origin keeps the
//! differences meaningful. It is monotonic, not the wall clock, because the relay only ever
//! subtracts two of its own timestamps.

use std::{sync::OnceLock, time::Instant};

/// The monotonic origin shared by every adapter in the process.
static ORIGIN: OnceLock<Instant> = OnceLock::new();

/// Returns the process's monotonic origin.
///
/// The link adapter uses it for its relay timestamps and the app adapter uses it through
/// [`now_ms`], so both adapters speak one millisecond scale (ADR-0039).
pub(crate) fn origin() -> Instant {
    *ORIGIN.get_or_init(Instant::now)
}

/// Returns the monotonic milliseconds elapsed since the process origin.
///
/// The value saturates at [`u64::MAX`] rather than wrapping; a process would have to run for
/// hundreds of millions of years to reach it.
#[must_use]
pub fn now_ms() -> u64 {
    u64::try_from(origin().elapsed().as_millis()).unwrap_or(u64::MAX)
}
