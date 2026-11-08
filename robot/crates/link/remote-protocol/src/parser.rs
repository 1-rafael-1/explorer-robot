//! The incremental, resynchronising frame parser (ADR-0030).
//!
//! The parser is a three-state machine: it scans for the magic pair, collects the twelve header
//! bytes, then collects exactly the declared payload bytes. A bad magic byte, an out-of-range
//! length or a failed CRC discards the candidate and advances one byte from its start before
//! rescanning. A declared payload is never scanned for magic; the CRC and the assembly timeout
//! are the only judges.
//!
//! The candidate buffer is an array whose length is a const generic, so the parser allocates
//! nothing. Use [`RobotToPcParser`] or [`PcToRobotParser`] for a buffer sized to one direction.

use crate::{
    crc::Crc16,
    frame::{Class, Direction, Flags, Frame, FrameHeader, HEADER_LEN, MAGIC, Seq},
};

/// The number of bits the class value is shifted left in the class byte.
const CLASS_SHIFT: u8 = 6;

/// The floor of the derived assembly timeout, in milliseconds.
const MIN_TIMEOUT_MS: u32 = 50;

/// The number of bits on the wire per byte for 8N1: one start bit, eight data bits, one stop bit.
const BITS_PER_BYTE: u64 = 10;

/// The number of milliseconds in a second.
const MS_PER_SECOND: u64 = 1_000;

/// The factor applied to the largest frame's wire time to derive the timeout.
const TIMEOUT_FACTOR: u64 = 4;

/// The largest robot-to-PC frame, header included.
pub const MAX_ROBOT_TO_PC_FRAME: usize = Direction::RobotToPc.max_frame();

/// The largest PC-to-robot frame, header included.
pub const MAX_PC_TO_ROBOT_FRAME: usize = Direction::PcToRobot.max_frame();

/// A [`Parser`] whose candidate buffer holds the largest robot-to-PC frame.
pub type RobotToPcParser = Parser<MAX_ROBOT_TO_PC_FRAME>;

/// A [`Parser`] whose candidate buffer holds the largest PC-to-robot frame.
pub type PcToRobotParser = Parser<MAX_PC_TO_ROBOT_FRAME>;

/// The parser's position in the three-state assembly machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// Scanning for the two magic bytes.
    Sync,
    /// Collecting the twelve header bytes.
    Header,
    /// Collecting the declared payload bytes.
    Payload,
}

/// An incremental frame parser that recovers from arbitrary byte-stream damage.
///
/// The parser owns a candidate buffer of `CAP` bytes. It yields a [`Frame`] borrowed from that
/// buffer, so the caller must finish with a frame before feeding the parser again. Feed several
/// frames in one chunk with [`Parser::feed`], or one byte at a time with [`Parser::push`].
///
/// `CAP` must be at least the direction's largest frame; use [`RobotToPcParser`] or
/// [`PcToRobotParser`] rather than spelling the constant out. A short `CAP` rejects frames that
/// do not fit instead of panicking.
pub struct Parser<const CAP: usize> {
    /// The direction whose payload limit and frame timeout apply.
    direction: Direction,
    /// The assembly timeout in milliseconds.
    timeout_ms: u32,
    /// The current assembly state.
    state: State,
    /// The candidate bytes, starting at the first magic byte.
    buffer: [u8; CAP],
    /// The number of valid bytes in `buffer`.
    filled: usize,
    /// The timestamp of the current candidate's first byte.
    started_at: u32,
    /// Whether a completed frame was returned and must be released before more input.
    delivered: bool,
    /// How many candidates were rejected because the CRC field did not match.
    crc_failures: u32,
    /// How many candidates were discarded and the scan restarted.
    resyncs: u32,
    /// How many bytes were discarded while resynchronising.
    discarded_bytes: u32,
}

impl<const CAP: usize> Parser<CAP> {
    /// Creates a parser for `direction` at `baud`.
    ///
    /// The assembly timeout is four times the wire time of the direction's largest frame with a
    /// 50 ms floor. At 921600 both directions hit the floor. `CAP` should match the direction;
    /// the [`RobotToPcParser`] and [`PcToRobotParser`] aliases do that.
    #[must_use]
    pub fn new(direction: Direction, baud: u32) -> Self {
        Self {
            direction,
            timeout_ms: assembly_timeout_ms(direction, baud),
            state: State::Sync,
            buffer: [0; CAP],
            filled: 0,
            started_at: 0,
            delivered: false,
            crc_failures: 0,
            resyncs: 0,
            discarded_bytes: 0,
        }
    }

    /// Returns the direction this parser was built for.
    #[must_use]
    pub const fn direction(&self) -> Direction {
        self.direction
    }

    /// Returns the current assembly timeout in milliseconds.
    #[must_use]
    pub const fn timeout_ms(&self) -> u32 {
        self.timeout_ms
    }

    /// Overrides the assembly timeout, for tests that should not wait for the derived value.
    pub const fn set_timeout_ms(&mut self, timeout_ms: u32) {
        self.timeout_ms = timeout_ms;
    }

    /// Returns how many candidates were rejected because the CRC field did not match.
    ///
    /// The counter is cumulative for the parser's lifetime; [`Parser::reset`] does not clear it.
    #[must_use]
    pub const fn crc_failures(&self) -> u32 {
        self.crc_failures
    }

    /// Returns how many candidates were discarded and the scan restarted.
    ///
    /// A candidate is discarded for a failed CRC, an out-of-range length or a declared frame that
    /// does not fit the buffer. The counter is cumulative; [`Parser::reset`] does not clear it.
    #[must_use]
    pub const fn resyncs(&self) -> u32 {
        self.resyncs
    }

    /// Returns how many bytes were dropped from the stream while resynchronising.
    ///
    /// This is the damage the parser discarded: header bytes of a rejected candidate and the
    /// payload that came with it. The counter is cumulative; [`Parser::reset`] does not clear it.
    #[must_use]
    pub const fn discarded_bytes(&self) -> u32 {
        self.discarded_bytes
    }

    /// Discards any partial candidate and returns to scanning for magic.
    pub const fn reset(&mut self) {
        self.state = State::Sync;
        self.filled = 0;
        self.started_at = 0;
        self.delivered = false;
    }

    /// Feeds one byte received at `now_ms`, returning a frame when one completes.
    ///
    /// The returned frame borrows the parser, so it must be dropped before the next call. The
    /// byte that completes a frame is consumed; the next call discards that frame and resumes
    /// scanning.
    pub fn push(&mut self, byte: u8, now_ms: u32) -> Option<Frame<'_>> {
        if CAP < HEADER_LEN {
            return None;
        }
        if self.delivered {
            self.reset();
        }
        if self.filled > 0 && now_ms.wrapping_sub(self.started_at) >= self.timeout_ms {
            self.reset();
        }
        self.accept(byte, now_ms)
    }

    /// Feeds a chunk, calling `on_frame` for each frame that completes in order.
    ///
    /// A frame borrows the parser, so `on_frame` must finish with it before the next byte is
    /// processed.
    pub fn feed(&mut self, bytes: &[u8], now_ms: u32, mut on_frame: impl FnMut(Frame<'_>)) {
        for &byte in bytes {
            if let Some(frame) = self.push(byte, now_ms) {
                on_frame(frame);
            }
        }
    }

    /// Advances the state machine by one byte.
    fn accept(&mut self, byte: u8, now_ms: u32) -> Option<Frame<'_>> {
        match self.state {
            State::Sync => {
                self.accept_sync_byte(byte, now_ms);
                None
            }
            State::Header | State::Payload => {
                self.buffer[self.filled] = byte;
                self.filled += 1;
                self.settle(now_ms)
            }
        }
    }

    /// Handles one byte while scanning for the magic pair.
    const fn accept_sync_byte(&mut self, byte: u8, now_ms: u32) {
        if self.filled == 0 {
            if byte == MAGIC[0] {
                self.buffer[0] = byte;
                self.filled = 1;
                self.started_at = now_ms;
            }
        } else if byte == MAGIC[1] {
            self.buffer[1] = byte;
            self.filled = 2;
            self.state = State::Header;
        } else if byte == MAGIC[0] {
            self.buffer[0] = byte;
            self.filled = 1;
            self.started_at = now_ms;
        } else {
            self.filled = 0;
        }
    }

    /// Evaluates the buffered candidate, yielding a frame when it is complete and valid.
    fn settle(&mut self, now_ms: u32) -> Option<Frame<'_>> {
        loop {
            match self.state {
                State::Sync => return None,
                State::Header => {
                    if self.filled < HEADER_LEN {
                        return None;
                    }
                    let len = usize::from(self.declared_len());
                    if len > self.direction.max_payload() || HEADER_LEN + len > CAP {
                        self.resync(now_ms);
                        continue;
                    }
                    if len == 0 {
                        if self.crc_matches(0) {
                            self.delivered = true;
                            return Some(self.frame());
                        }
                        self.crc_failures = self.crc_failures.wrapping_add(1);
                        self.resync(now_ms);
                        continue;
                    }
                    self.state = State::Payload;
                }
                State::Payload => {
                    let len = usize::from(self.declared_len());
                    let total = HEADER_LEN + len;
                    if self.filled < total {
                        return None;
                    }
                    if self.crc_matches(len) {
                        self.delivered = true;
                        return Some(self.frame());
                    }
                    self.crc_failures = self.crc_failures.wrapping_add(1);
                    self.resync(now_ms);
                }
            }
        }
    }

    /// Returns the payload length declared in the buffered header.
    #[must_use]
    const fn declared_len(&self) -> u16 {
        u16::from_le_bytes([self.buffer[8], self.buffer[9]])
    }

    /// Returns whether the header and the first `payload_len` payload bytes match the CRC field.
    #[must_use]
    fn crc_matches(&self, payload_len: usize) -> bool {
        let mut crc = Crc16::new();
        crc.update(&self.buffer[..10]);
        crc.update(&self.buffer[HEADER_LEN..HEADER_LEN + payload_len]);
        crc.finish() == u16::from_le_bytes([self.buffer[10], self.buffer[11]])
    }

    /// Borrows the buffered candidate as a complete frame.
    #[must_use]
    fn frame(&self) -> Frame<'_> {
        let len = self.declared_len();
        let payload_len = usize::from(len);
        Frame {
            header: FrameHeader {
                major: self.buffer[2],
                class: Class::from_byte(self.buffer[3] >> CLASS_SHIFT).unwrap_or(Class::Stream),
                flags: Flags::from_bits(self.buffer[3]),
                id: u16::from_le_bytes([self.buffer[4], self.buffer[5]]),
                seq: Seq::new(u16::from_le_bytes([self.buffer[6], self.buffer[7]])),
                len,
            },
            payload: &self.buffer[HEADER_LEN..HEADER_LEN + payload_len],
        }
    }

    /// Discards the candidate's first byte and rescans the rest for the next magic pair.
    fn resync(&mut self, now_ms: u32) {
        self.resyncs = self.resyncs.wrapping_add(1);

        let len = self.filled;
        if len == 0 {
            self.state = State::Sync;
            return;
        }

        let start = (1..len.saturating_sub(1))
            .find(|&index| self.buffer[index] == MAGIC[0] && self.buffer[index + 1] == MAGIC[1]);

        let discarded = if let Some(start) = start {
            self.buffer.copy_within(start..len, 0);
            self.filled = len - start;
            self.state = State::Header;
            start
        } else if self.buffer[len - 1] == MAGIC[0] {
            self.buffer[0] = MAGIC[0];
            self.filled = 1;
            self.state = State::Sync;
            len - 1
        } else {
            self.filled = 0;
            self.state = State::Sync;
            len
        };
        self.discarded_bytes = self
            .discarded_bytes
            .wrapping_add(u32::try_from(discarded).unwrap_or(u32::MAX));
        self.started_at = now_ms;
    }
}

/// Derives the assembly timeout from the direction's largest frame and the baud.
fn assembly_timeout_ms(direction: Direction, baud: u32) -> u32 {
    if baud == 0 {
        return MIN_TIMEOUT_MS;
    }
    let baud = u64::from(baud);
    let frame = u64::try_from(direction.max_frame()).unwrap_or(u64::MAX);
    let numerator = frame
        .saturating_mul(BITS_PER_BYTE)
        .saturating_mul(MS_PER_SECOND)
        .saturating_mul(TIMEOUT_FACTOR);
    let timeout = numerator.div_ceil(baud);
    u32::try_from(timeout).unwrap_or(u32::MAX).max(MIN_TIMEOUT_MS)
}
