//! A tiny fixed-capacity text buffer for the couple of runtime-formatted
//! strings the UI produces.
//!
//! `alloc` and `heapless` are unavailable here, and the only strings that are
//! not compile-time constants are the value-entry readout and the System Info
//! rows, so a small stack buffer implementing [`core::fmt::Write`] is enough.

use core::fmt::{self, Write as _};

/// Capacity of [`TextBuf`], in bytes.
///
/// Wide enough for the widest value-entry readout (a five-digit
/// attempt-straight value plus its unit, e.g. `5000 cm`) and the widest System
/// Info row (e.g. `Motor: Unknown`).
pub const TEXT_BUF_LEN: usize = 16;

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
