//! The robot side of the Link Protocol (ADR-0032).
//!
//! `LinkCore` owns the RX parser, the pending stream slots, the event ring, the lease, command
//! dedupe and TX priority. It is driven with plain millisecond timestamps rather than a clock so it
//! can be host-tested; the firmware keeps the UART, the port power, the sampling and the mapping.
//!
//! This crate is `no_std` and allocates nothing. The `std` feature exists only for host tests and
//! for the simulated link in `remote-protocol`.
//!
//! This ticket lands the stream half of the core: the latest-wins slots, per-stream sequences and
//! the round-robin stream emitter. Commands, events, the session lease and the Deadman arrive in
//! later tickets; their timing values are already in [`timing`].

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(feature = "std")]
extern crate std;

pub mod timing;

use remote_protocol::{
    Class, Direction, EncodeError, Flags, Frame, PROTOCOL_MAJOR, Seq,
    codec::{PayloadCodec, StreamSample},
    ids::Stream,
    parser::PcToRobotParser,
    payload::stream::{Activity, Calibration, LidarCloud, Motion, Perception, Power},
};
pub use timing::Config;

/// The total encoded payload staging the six latest-wins slots reserve.
///
/// ADR-0030 allows roughly 1.1 KiB of staging on the robot; the v1 catalogue sums to 767 bytes,
/// dominated by the 720-byte `LiDAR` cloud.
pub const STREAM_STAGING_BYTES: usize = Power::ENCODED_LEN
    + Motion::ENCODED_LEN
    + Activity::ENCODED_LEN
    + Perception::ENCODED_LEN
    + LidarCloud::ENCODED_LEN
    + Calibration::ENCODED_LEN;

/// The staging budget ADR-0030 allows for the pending stream slots.
pub const STREAM_STAGING_BUDGET_BYTES: usize = 1024;

/// A compile-time guard that the stream catalogue fits the staging budget.
const _: () = assert!(STREAM_STAGING_BYTES <= STREAM_STAGING_BUDGET_BYTES);

/// Whether a Remote is currently attached to the link.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PeerState {
    /// No valid PC-to-robot frame has arrived in this powered link period, or the lease expired.
    #[default]
    NoPeer,
    /// A valid PC-to-robot frame arrived and the lease has not expired.
    PeerLive,
}

/// The latest encoded payload of each stream.
///
/// Each slot is sized from the ADR-0031 catalogue and holds one encoded payload. A publish
/// replaces the slot's contents; nothing is queued.
struct StreamSlots {
    /// The pending `POWER` payload.
    power: heapless::Vec<u8, { Power::ENCODED_LEN }>,
    /// The pending `MOTION` payload.
    motion: heapless::Vec<u8, { Motion::ENCODED_LEN }>,
    /// The pending `ACTIVITY` payload.
    activity: heapless::Vec<u8, { Activity::ENCODED_LEN }>,
    /// The pending `PERCEPTION` payload.
    perception: heapless::Vec<u8, { Perception::ENCODED_LEN }>,
    /// The pending `LIDAR_CLOUD` payload.
    lidar_cloud: heapless::Vec<u8, { LidarCloud::ENCODED_LEN }>,
    /// The pending `CALIBRATION` payload.
    calibration: heapless::Vec<u8, { Calibration::ENCODED_LEN }>,
}

impl StreamSlots {
    /// Creates six empty slots.
    const fn new() -> Self {
        Self {
            power: heapless::Vec::new(),
            motion: heapless::Vec::new(),
            activity: heapless::Vec::new(),
            perception: heapless::Vec::new(),
            lidar_cloud: heapless::Vec::new(),
            calibration: heapless::Vec::new(),
        }
    }

    /// Returns the encoded payload currently stored for `stream`.
    fn payload(&self, stream: Stream) -> &[u8] {
        match stream {
            Stream::Power => &self.power,
            Stream::Motion => &self.motion,
            Stream::Activity => &self.activity,
            Stream::Perception => &self.perception,
            Stream::LidarCloud => &self.lidar_cloud,
            Stream::Calibration => &self.calibration,
        }
    }
}

/// The robot's link state and the composition point for the protocol mechanisms.
///
/// Construct it with the firmware's Boot Id and feed it bytes, time and samples. It never blocks,
/// never allocates and never holds an incoming frame past the call that produced it.
pub struct LinkCore {
    /// The timing configuration; later tickets consume its values.
    config: Config,
    /// The Boot Id the firmware minted for this powered link period.
    boot_id: u32,
    /// The inbound parser for PC-to-robot frames.
    parser: PcToRobotParser,
    /// The latest-wins encoded payload slots.
    slots: StreamSlots,
    /// One sequence counter per stream, incremented when a frame is emitted.
    sequences: [u16; Stream::COUNT],
    /// The pending bitmap, indexed by [`Stream::index`].
    pending: u8,
    /// The stream index to try first on the next emission, for round-robin fairness.
    cursor: usize,
    /// The number of unknown inbound stream ids ignored so far.
    unknown_streams: u32,
}

impl LinkCore {
    /// Creates a link core with `config`, the firmware's `boot_id` and the current time.
    #[must_use]
    pub fn new(config: Config, boot_id: u32, _now_ms: u64) -> Self {
        let mut parser = PcToRobotParser::new(Direction::PcToRobot, timing::BAUD);
        parser.set_timeout_ms(config.frame_timeout_ms);
        Self {
            config,
            boot_id,
            parser,
            slots: StreamSlots::new(),
            sequences: [0; Stream::COUNT],
            pending: 0,
            cursor: 0,
            unknown_streams: 0,
        }
    }

    /// Returns the timing configuration this core was built with.
    #[must_use]
    pub const fn config(&self) -> Config {
        self.config
    }

    /// Returns the Boot Id the firmware minted for this link period.
    #[must_use]
    pub const fn boot_id(&self) -> u32 {
        self.boot_id
    }

    /// Returns how many unknown inbound stream ids have been ignored.
    #[must_use]
    pub const fn unknown_streams(&self) -> u32 {
        self.unknown_streams
    }

    /// Publishes one telemetry sample into its per-stream latest-wins slot.
    ///
    /// The sample is encoded immediately; a newer publish replaces the pending one, so there is no
    /// queue and a coalesced sample is never retransmitted. `requested_rate` and `now_ms` are
    /// accepted for ticket 08's emission schedule and ignored until then.
    pub fn publish_stream(&mut self, sample: &StreamSample<'_>, _requested_rate: u32, _now_ms: u64) {
        let stored = match sample {
            StreamSample::Power(power) => store_fixed(&mut self.slots.power, |out| power.encode_into(out)),
            StreamSample::Motion(motion) => store_fixed(&mut self.slots.motion, |out| motion.encode_into(out)),
            StreamSample::Activity(activity) => self.store_activity(activity),
            StreamSample::Perception(perception) => {
                store_fixed(&mut self.slots.perception, |out| perception.encode_into(out))
            }
            StreamSample::LidarCloud(cloud) => store_fixed(&mut self.slots.lidar_cloud, |out| cloud.encode_into(out)),
            StreamSample::Calibration(calibration) => {
                store_fixed(&mut self.slots.calibration, |out| calibration.encode_into(out))
            }
        };
        self.set_pending(sample.stream(), stored);
    }

    /// Emits the next pending stream frame into `out`, returning the number of bytes written.
    ///
    /// Pending streams are served round-robin. Control and event priority arrive in later
    /// tickets, so only streams can be pending today. A frame that does not fit returns [`None`]
    /// without consuming the pending sample or advancing its sequence.
    pub fn next_frame(&mut self, out: &mut [u8]) -> Option<usize> {
        for offset in 0..Stream::COUNT {
            let index = (self.cursor + offset) % Stream::COUNT;
            if self.pending & (1u8 << index) == 0 {
                continue;
            }
            let stream = Stream::ALL[index];
            let frame = Frame::new(
                PROTOCOL_MAJOR,
                Class::Stream,
                Flags::default(),
                stream.wire_id(),
                Seq::new(self.sequences[index]),
                self.slots.payload(stream),
            )
            .ok()?;
            let written = frame.encode_into(out).ok()?;
            self.sequences[index] = self.sequences[index].wrapping_add(1);
            self.pending &= !(1u8 << index);
            self.cursor = (index + 1) % Stream::COUNT;
            return Some(written);
        }
        None
    }

    /// Advances the link's timers to `now_ms`.
    ///
    /// The lease, the Deadman, retries and the emission schedule land in later tickets, so this is
    /// a no-op for now.
    pub const fn tick(&mut self, _now_ms: u64) {}

    /// Returns the current peer state.
    ///
    /// The session lease arrives in ticket 06; until then no peer is ever reported live.
    #[must_use]
    pub const fn peer(&self) -> PeerState {
        PeerState::NoPeer
    }

    /// Takes the one-shot `peer_lost` edge.
    ///
    /// The Deadman arrives in ticket 06; until then the edge never fires.
    #[must_use]
    pub const fn take_peer_lost(&mut self) -> bool {
        false
    }

    /// Feeds PC-to-robot bytes through the parser.
    ///
    /// Command, event and control handling land in later tickets. Stream ids are validated here so
    /// that an unknown id is counted and ignored rather than panicking or being mistaken for a
    /// known stream.
    pub fn on_rx(&mut self, bytes: &[u8], now_ms: u64) {
        let now = u32::try_from(now_ms).unwrap_or(u32::MAX);
        let mut unknown = 0u32;
        self.parser.feed(bytes, now, |frame| {
            if frame.header.class == Class::Stream && Stream::from_wire(frame.header.id).is_none() {
                unknown = unknown.saturating_add(1);
            }
        });
        self.unknown_streams = self.unknown_streams.saturating_add(unknown);
    }

    /// Encodes and stores an `ACTIVITY` payload, whose encoded length varies with the detail.
    fn store_activity(&mut self, activity: &Activity<'_>) -> bool {
        let mut buffer = [0u8; Activity::ENCODED_LEN];
        self.slots.activity.clear();
        if let Ok(len) = activity.encode_into(&mut buffer) {
            self.slots.activity.extend_from_slice(&buffer[..len]).is_ok()
        } else {
            false
        }
    }

    /// Sets or clears `stream`'s pending bit after a publish attempt.
    const fn set_pending(&mut self, stream: Stream, pending: bool) {
        let bit = 1u8 << stream.index();
        if pending {
            self.pending |= bit;
        } else {
            self.pending &= !bit;
        }
    }
}

/// Encodes a fixed-size payload into its slot, clearing the slot if encoding fails.
///
/// The slot's capacity is the payload's exact encoded length, so a successful encode fills it.
fn store_fixed<const N: usize>(
    slot: &mut heapless::Vec<u8, N>,
    encode: impl FnOnce(&mut [u8]) -> Result<usize, EncodeError>,
) -> bool {
    slot.clear();
    if slot.resize(N, 0).is_err() {
        return false;
    }
    match encode(&mut slot[..]) {
        Ok(len) if len <= N => {
            slot.truncate(len);
            true
        }
        _ => {
            slot.clear();
            false
        }
    }
}
