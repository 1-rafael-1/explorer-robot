//! The frame-level pump: the ring, the router, the watermark policy and the client gate composed.
//!
//! The binary hands this type parsed frames from either side and acts on its decisions. No frame
//! is queued for the client until one is authenticated, so a connect never has stale stream
//! history to flush; the ring holds the current coalesced state, not a log. The UART side keeps
//! being parsed regardless, which is what lets a `BRIDGE_CONTROL` and the Beacon's link flag stay
//! truthful with no client attached.

use remote_protocol::{BridgeStream, Class, Flags, Frame, Seq, ThrottleHint};

use crate::{
    auth::{ClientAuth, ConnectOutcome, Proof, ProofOutcome, TimerEvent},
    coalescing::{Entry, PushOutcome, Ring},
    flow::WatermarkPolicy,
    routing::{BridgeControlRequest, Destination, Route, Router, Side},
};

/// What the pump decided for one frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    /// The frame was queued for the authenticated client.
    Queued,
    /// A robot-to-client frame was dropped because no client is authenticated.
    DroppedNoClient,
    /// A stream or reliable frame could not fit the ring.
    DroppedFull,
    /// The robot asked for a `BRIDGE_CONTROL` answer.
    BridgeControl(BridgeControlRequest),
    /// A client frame must be written to the robot-facing UART.
    ForwardToRobot,
    /// A client-side `BRIDGE_CONTROL` was dropped and counted.
    DroppedBridgeControl,
    /// No rule applies; the frame is ignored.
    Ignored,
}

/// The Bridge's pump rules and their counters.
pub struct BridgePump {
    /// The client-bound coalescing ring.
    ring: Ring,
    /// The class-rule routing filter.
    router: Router,
    /// The ring-occupancy to `THROTTLE` policy.
    flow: WatermarkPolicy,
    /// The authentication and single-client gate.
    auth: ClientAuth,
    /// How many robot-to-client frames were dropped without a client.
    dropped_without_client: u32,
}

impl BridgePump {
    /// Creates an idle pump: no client, an empty ring and no hint history.
    #[must_use]
    #[allow(
        clippy::large_stack_frames,
        reason = "the pump owns the 8 KiB ring for its whole lifetime"
    )]
    pub const fn new() -> Self {
        Self {
            ring: Ring::new(),
            router: Router::new(),
            flow: WatermarkPolicy::new(),
            auth: ClientAuth::new(),
            dropped_without_client: 0,
        }
    }

    /// Offers a new connection to the client gate.
    pub fn connect(&mut self, now_ms: u32) -> ConnectOutcome {
        self.auth.connect(now_ms)
    }

    /// Presents a proof for `connection`.
    pub fn authenticate(&mut self, connection: u32, proof: Proof, expected: &Proof, now_ms: u32) -> ProofOutcome {
        self.auth.authenticate(connection, proof, expected, now_ms)
    }

    /// Advances the gate's timers.
    pub const fn tick(&mut self, now_ms: u32) -> Option<TimerEvent> {
        self.auth.tick(now_ms)
    }

    /// Drops `connection` if it is the current one.
    pub fn disconnect(&mut self, connection: u32) -> bool {
        self.auth.disconnect(connection)
    }

    /// Returns whether a client is authenticated.
    #[must_use]
    pub const fn is_authenticated(&self) -> bool {
        self.auth.is_authenticated()
    }

    /// Returns the authenticated connection's id, if any.
    #[must_use]
    pub const fn authenticated_connection(&self) -> Option<u32> {
        self.auth.authenticated_connection()
    }

    /// Returns the current connection's id, whether or not it is authenticated.
    ///
    /// A rotation disconnects the current connection, so this is the id the supervisory loop
    /// closes when the pairing code changes.
    #[must_use]
    pub const fn current_connection(&self) -> Option<u32> {
        self.auth.current_connection()
    }

    /// Handles one frame parsed from the robot-facing UART.
    ///
    /// `BRIDGE_CONTROL` is consumed whether or not a client is attached. Everything else that is
    /// client-bound is queued only for an authenticated client; without one the frame is dropped
    /// and counted, never buffered.
    pub fn on_robot_frame(&mut self, frame: &Frame<'_>) -> Decision {
        match self.router.route(Side::Robot, frame) {
            Route::BridgeControl(request) => Decision::BridgeControl(request),
            Route::Forward(Destination::Client) => self.queue_for_client(frame),
            Route::Forward(Destination::Robot) | Route::DroppedBridgeControl | Route::Ignore => Decision::Ignored,
        }
    }

    /// Handles one frame parsed from the authenticated client's TCP connection.
    pub fn on_client_frame(&mut self, frame: &Frame<'_>) -> Decision {
        match self.router.route(Side::Client, frame) {
            Route::Forward(Destination::Robot) => Decision::ForwardToRobot,
            Route::DroppedBridgeControl => Decision::DroppedBridgeControl,
            Route::Forward(Destination::Client) | Route::BridgeControl(_) | Route::Ignore => Decision::Ignored,
        }
    }

    /// Observes the ring's occupancy and returns the `THROTTLE` hint to emit, if any.
    #[must_use]
    pub const fn poll_throttle(&mut self, now_ms: u32) -> Option<ThrottleHint> {
        self.flow.observe(self.ring.used_bytes(), self.ring.entries(), now_ms)
    }

    /// Queues a Bridge-originated status snapshot for the authenticated client.
    ///
    /// The snapshot is coalesced by its stream id in the same latest-wins ring as robot streams.
    /// With no authenticated client it is skipped silently (it must not pollute the
    /// `dropped_without_client` counter, which counts robot frames).
    pub fn offer_bridge_status(&mut self, seq: Seq, payload: &[u8]) -> Decision {
        if !self.auth.is_authenticated() {
            return Decision::Ignored;
        }

        match self
            .ring
            .push_stream(BridgeStream::Status.wire_id(), seq, Flags::default(), payload)
        {
            PushOutcome::Stored | PushOutcome::Replaced => Decision::Queued,
            PushOutcome::Dropped => Decision::DroppedFull,
        }
    }

    /// Removes and returns the oldest client-bound entry.
    pub fn pop(&mut self) -> Option<Entry<'_>> {
        self.ring.pop()
    }

    /// Drains every client-bound entry in arrival order, calling `sink` for each.
    pub fn drain(&mut self, sink: impl FnMut(Entry<'_>)) {
        self.ring.drain(sink);
    }

    /// Returns the ring's used bytes.
    #[must_use]
    pub const fn used_bytes(&self) -> usize {
        self.ring.used_bytes()
    }

    /// Returns the number of queued client-bound entries.
    #[must_use]
    pub const fn entries(&self) -> usize {
        self.ring.entries()
    }

    /// Returns the ring's reserved bytes.
    #[must_use]
    pub const fn reserved_bytes(&self) -> usize {
        self.ring.reserved_bytes()
    }

    /// Returns how many client-bound frames were dropped without a client.
    #[must_use]
    pub const fn dropped_without_client(&self) -> u32 {
        self.dropped_without_client
    }

    /// Returns how many client-side `BRIDGE_CONTROL` frames were dropped.
    #[must_use]
    pub const fn dropped_bridge_controls(&self) -> u32 {
        self.router.dropped_bridge_controls()
    }

    /// Returns how many stream samples the ring discarded.
    #[must_use]
    pub const fn dropped_streams(&self) -> u32 {
        self.ring.dropped_streams()
    }

    /// Returns how many reliable entries the ring discarded.
    #[must_use]
    pub const fn dropped_reliable(&self) -> u32 {
        self.ring.dropped_reliable()
    }

    /// Queues a robot-to-client frame for the authenticated client.
    fn queue_for_client(&mut self, frame: &Frame<'_>) -> Decision {
        if !self.auth.is_authenticated() {
            self.dropped_without_client = self.dropped_without_client.wrapping_add(1);
            return Decision::DroppedNoClient;
        }

        let queued = match frame.header.class {
            Class::Stream => {
                match self
                    .ring
                    .push_stream(frame.header.id, frame.header.seq, frame.header.flags, frame.payload)
                {
                    PushOutcome::Stored | PushOutcome::Replaced => true,
                    PushOutcome::Dropped => false,
                }
            }
            Class::Event => self
                .ring
                .push_event(frame.header.id, frame.header.seq, frame.header.flags, frame.payload)
                .is_ok(),
            Class::Command | Class::Control => self
                .ring
                .push_reliable(
                    frame.header.class,
                    frame.header.id,
                    frame.header.seq,
                    frame.header.flags,
                    frame.payload,
                )
                .is_ok(),
        };
        if queued {
            Decision::Queued
        } else {
            Decision::DroppedFull
        }
    }
}

impl Default for BridgePump {
    #[allow(
        clippy::large_stack_frames,
        reason = "the pump owns the 8 KiB ring for its whole lifetime"
    )]
    fn default() -> Self {
        Self::new()
    }
}
