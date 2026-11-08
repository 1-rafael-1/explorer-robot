//! Host test for the worst-case persona (ticket 02): ADR-0033's ~10.3 KB/s sustained budget.
//!
//! The budget's stream lines price every change-based stream at the 10 Hz sampling tick; its
//! event line (1,856 B/s) was priced with the 250 ms retry placeholder, while the shipped retry
//! constant is 500 ms, so the persona paces fresh publications of the catalogue's largest event
//! to sustain the same event bytes. The test measures raw framed bytes per second crossing the
//! seam and holds them against the ADR's total.

mod support;

use link_protocol::{Caps, Class, Control};
use robot_stub::{Persona, worst_case_config};
use support::Bench;

/// The Boot Id the stub mints for this powered link period.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// ADR-0033's worst-case total: 150 + 140 + 490 + 130 + 7,320 + 160 stream bytes, 1,856 event
/// bytes and 100 control bytes per second.
const BUDGET_BYTES_PER_SEC: usize = 10_346;

/// The persona may overshoot the budget slightly (its event pacing is honest but coarse); it may
/// not saturate the wire, which would make the soak mean something else than the budget.
const BUDGET_TOLERANCE_BYTES_PER_SEC: usize = 11_000;

/// The worst-case persona sustains ADR-0033's budget: measured over a two-second window with a
/// Session attached, the framed bytes crossing the seam per second reach the ADR's total.
#[test]
fn worst_case_persona_sustains_the_adr_budget() {
    let mut bench = Bench::new(worst_case_config(), BOOT_ID, Persona::WorstCase);
    bench.run_until(100);
    let _ = bench.peer.send_hello(7, Caps::from_bits(0), &[]);
    bench.step_now();

    let before = bench.peer.bytes_read();
    bench.run_until(2_100);
    let window_bytes = bench.peer.bytes_read() - before;
    let measured = window_bytes / 2;

    assert!(
        measured >= BUDGET_BYTES_PER_SEC,
        "the persona must sustain the {BUDGET_BYTES_PER_SEC} B/s budget, measured {measured} B/s"
    );
    assert!(
        measured <= BUDGET_TOLERANCE_BYTES_PER_SEC,
        "the persona targets the budget, not saturation: measured {measured} B/s"
    );

    // The budget's stream side is the point of the soak: every stream must be on the wire, not
    // just a flood of events, and the handshake stays answered while under load.
    let window = (Some(1_000u64), 2_100u64);
    for id in [
        link_protocol::Stream::Power.wire_id(),
        link_protocol::Stream::Motion.wire_id(),
        link_protocol::Stream::Activity.wire_id(),
        link_protocol::Stream::Perception.wire_id(),
        link_protocol::Stream::LidarCloud.wire_id(),
        link_protocol::Stream::Calibration.wire_id(),
    ] {
        let count = bench.peer.frames_of(Class::Stream, id, window.0, window.1).len();
        assert!(count > 0, "stream {id:#06x} must flow under worst-case load");
    }
    let replies = bench
        .peer
        .frames_of(Class::Control, Control::Hello.wire_id(), None, bench.at());
    assert_eq!(replies.len(), 1, "the HELLO is answered while under load");
}
