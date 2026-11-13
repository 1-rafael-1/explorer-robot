//! A tiny fixed-capacity text buffer for the handful of runtime-formatted
//! strings the UI produces.
//!
//! `alloc` and `heapless` are unavailable here, and the only strings that are
//! not compile-time constants are the value-entry readout, the System Info,
//! Link and IMU Test Readout rows, so a small stack buffer implementing
//! [`core::fmt::Write`] is enough.

use core::fmt::{self, Write as _};

/// Capacity of [`TextBuf`], in bytes.
///
/// The longest real IMU Test Readout rows are about 43–44 ASCII characters
/// (the nine-axis health line `IMU not responding miss=9999 dmpcal unknown` and
/// the fallback gate line `mag unavailable - 6-axis fallback use_mag=no`). The
/// capacity is 48 because ticket 01 requires a 48-character readout row to
/// round-trip intact; it also holds the widest value-entry readout (a five-digit
/// attempt-straight value plus its unit, e.g. `5000 cm`), the widest System Info
/// row (e.g. `Motor: Unknown`), and the widest Link row (e.g.
/// `Bridge: Waiting for board` and `Station: Associated`).
pub const TEXT_BUF_LEN: usize = 48;

/// A tiny fixed-capacity ASCII buffer for formatting short strings.
///
/// Writing beyond the capacity truncates and then clears the buffer rather than
/// panicking, and every string these helpers format fits comfortably.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TextBuf {
    /// Backing bytes; only the first `len` are valid.
    bytes: [u8; TEXT_BUF_LEN],
    /// Number of valid bytes in `bytes`.
    len: usize,
}

impl TextBuf {
    /// Create an empty buffer.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            bytes: [0; TEXT_BUF_LEN],
            len: 0,
        }
    }

    /// Overwrite the buffer with `value` and `unit`, e.g. `150 cm`.
    pub fn set(&mut self, value: i32, unit: &str) {
        self.fill(format_args!("{value} {unit}"));
    }

    /// Overwrite the buffer with `args`, falling back to empty on overflow.
    ///
    /// This is the single place overflow is absorbed, so callers can format
    /// freely knowing a too-long string can never panic.
    pub fn fill(&mut self, args: fmt::Arguments<'_>) {
        self.len = 0;
        if self.write_fmt(args).is_err() {
            self.len = 0;
        }
    }

    /// The written contents as a string slice.
    ///
    /// Only ASCII is ever written here, so the conversion cannot fail; a
    /// corrupted buffer reads as empty rather than panicking.
    #[must_use]
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}

impl Default for TextBuf {
    /// An empty buffer.
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Write for TextBuf {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let incoming = s.as_bytes();
        let remaining = self.bytes.len() - self.len;
        let take = incoming.len().min(remaining);
        self.bytes[self.len..self.len + take].copy_from_slice(&incoming[..take]);
        self.len += take;
        if take < incoming.len() {
            return Err(fmt::Error);
        }
        Ok(())
    }
}
