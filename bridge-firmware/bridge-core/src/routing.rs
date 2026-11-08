//! The routing filter over the Link Protocol's class rules (ADR-0030, ADR-0036, ADR-0037, ADR-0042).
//!
//! The filter has exactly two forwarding exceptions. `THROTTLE` is the Bridge's own hint and
//! travels out to the robot. `BRIDGE_CONTROL` travels in: the Bridge consumes it, never forwards
//! it, and answers action 0 with the setup credentials or actions 1 and 2 with the pairing code. A
//! `BRIDGE_CONTROL` from the TCP side is dropped and counted; the Remote cannot take the Bridge
//! off its own network.
//!
//! Everything else crosses by class: streams and events to the client, commands to the robot, and
//! acknowledgements in the direction their `IS_ACK` flag reverses. A frame travelling the wrong
//! way is ignored rather than forwarded. The Bridge-originated `BRIDGE_STATUS` stream is never
//! accepted from a peer: a stream carrying its id is ignored from either side, and the router also
//! ignores the `BRIDGE_STATUS` control message. The Bridge injects its snapshot into the pump's
//! ring directly instead.

use remote_protocol::{
    BridgeControlAction, BridgeStream, Class, Control, ControlMessage, Flags, Frame, Seq, decode_control,
};

/// The side a frame was received from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    /// The robot-facing UART.
    Robot,
    /// The authenticated TCP client.
    Client,
}

/// The side a forwarded frame leaves for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Destination {
    /// The robot-facing UART.
    Robot,
    /// The authenticated TCP client.
    Client,
}

/// A `BRIDGE_CONTROL` request the firmware must answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BridgeControlRequest {
    /// The action byte the robot asked for.
    pub action: BridgeControlAction,
    /// The request's sequence number, which the acknowledgement must echo.
    pub seq: Seq,
}

impl BridgeControlRequest {
    /// Returns which reply layout the ADR's action byte selects.
    ///
    /// Action 0 answers with the setup credentials; actions 1 and 2 answer with the pairing code.
    /// The match is exhaustive, so a new action cannot be added without choosing its reply.
    #[must_use]
    pub const fn reply(self) -> BridgeReply {
        match self.action {
            BridgeControlAction::ForgetNetwork => BridgeReply::SetupCredentials,
            BridgeControlAction::ShowPairingCode | BridgeControlAction::RotatePairingCode => BridgeReply::PairingCode,
        }
    }
}

/// The reply payload layout a `BRIDGE_CONTROL` action selects (ADR-0036, ADR-0037).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BridgeReply {
    /// Action 0: the setup SSID and passphrase.
    SetupCredentials,
    /// Actions 1 and 2: the twelve-character pairing code.
    PairingCode,
}

/// What the routing filter decided for one frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Route {
    /// Forward the frame unchanged to this side.
    Forward(Destination),
    /// Consume the frame and answer the robot's `BRIDGE_CONTROL` request.
    BridgeControl(BridgeControlRequest),
    /// A client-side `BRIDGE_CONTROL`: dropped and counted, never forwarded or acknowledged.
    DroppedBridgeControl,
    /// No route applies; the frame is ignored.
    Ignore,
}

/// The routing filter and its counters.
#[derive(Clone, Copy, Debug, Default)]
pub struct Router {
    /// How many client-side `BRIDGE_CONTROL` frames have been dropped.
    dropped_bridge_controls: u32,
}

impl Router {
    /// Creates a router with zero counters.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            dropped_bridge_controls: 0,
        }
    }

    /// Returns how many client-side `BRIDGE_CONTROL` frames have been dropped.
    #[must_use]
    pub const fn dropped_bridge_controls(&self) -> u32 {
        self.dropped_bridge_controls
    }

    /// Routes one frame received from `side`.
    pub fn route(&mut self, side: Side, frame: &Frame<'_>) -> Route {
        match frame.header.class {
            Class::Stream => {
                if side == Side::Robot
                    && !frame.header.flags.contains(Flags::IS_ACK)
                    && BridgeStream::from_wire(frame.header.id).is_none()
                {
                    Route::Forward(Destination::Client)
                } else {
                    Route::Ignore
                }
            }
            Class::Event => Self::route_reliable(
                side,
                frame.header.flags,
                (Side::Robot, Destination::Client),
                (Side::Client, Destination::Robot),
            ),
            Class::Command => Self::route_reliable(
                side,
                frame.header.flags,
                (Side::Client, Destination::Robot),
                (Side::Robot, Destination::Client),
            ),
            Class::Control => self.route_control(side, frame),
        }
    }

    /// Routes a class whose normal and acknowledged directions are given as `(side, destination)`.
    fn route_reliable(
        side: Side,
        flags: Flags,
        normal: (Side, Destination),
        acknowledged: (Side, Destination),
    ) -> Route {
        let (expected, destination) = if flags.contains(Flags::IS_ACK) {
            acknowledged
        } else {
            normal
        };
        if side == expected {
            Route::Forward(destination)
        } else {
            Route::Ignore
        }
    }

    /// Routes a control frame, applying the two control exceptions.
    fn route_control(&mut self, side: Side, frame: &Frame<'_>) -> Route {
        match Control::from_wire(frame.header.id) {
            Some(Control::BridgeControl) => self.route_bridge_control(side, frame),
            Some(Control::Throttle) => {
                if side == Side::Client {
                    Route::Forward(Destination::Robot)
                } else {
                    Route::Ignore
                }
            }
            Some(Control::Hello | Control::Ping) => Self::route_reliable(
                side,
                frame.header.flags,
                (Side::Client, Destination::Robot),
                (Side::Robot, Destination::Client),
            ),
            // `BRIDGE_STATUS` is Bridge-originated, so the router never accepts it from a peer;
            // unknown control ids are ignored as well.
            Some(Control::BridgeStatus) | None => Route::Ignore,
        }
    }

    /// Consumes a robot-side `BRIDGE_CONTROL` and drops a client-side one.
    fn route_bridge_control(&mut self, side: Side, frame: &Frame<'_>) -> Route {
        if side == Side::Client {
            self.dropped_bridge_controls = self.dropped_bridge_controls.wrapping_add(1);
            return Route::DroppedBridgeControl;
        }

        let is_ack = frame.header.flags.contains(Flags::IS_ACK);
        match decode_control(Control::BridgeControl, is_ack, frame.payload) {
            Ok(ControlMessage::BridgeControl(request)) => Route::BridgeControl(BridgeControlRequest {
                action: request.action,
                seq: frame.header.seq,
            }),
            _ => Route::Ignore,
        }
    }
}
