//! Host tests for the fault knobs (ticket 02): induced drop, delay and corruption of bytes on the
//! serial side, observable at the byte seam and switchable on demand.
//!
//! The knobs act on the stub's outbound bytes, which is what the Bridge's parser receives; the
//! observations are raw byte counts and the frames the peer can and cannot decode.

mod support;

use remote_protocol::{Class, Control};
use robot_stub::{Config, FaultKnobs, Persona};
use support::Bench;

/// The Boot Id the stub mints for this powered link period.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// The drop knob swallows every outbound byte at `drop_every = 1` — nothing crosses the seam —
/// and switching it off on demand restores the exchange.
#[test]
fn drop_knob_removes_outbound_bytes_and_switches_off_on_demand() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::Baseline);
    bench.run_until(100);

    bench.stub.set_faults(FaultKnobs {
        drop_every: 1,
        ..FaultKnobs::default()
    });
    let before = bench.peer.bytes_read();
    let _ = bench.peer.send_ping();
    bench.run_until(200);
    assert_eq!(
        bench.peer.bytes_read() - before,
        0,
        "with every byte dropped, nothing crosses the seam"
    );
    assert!(
        bench
            .peer
            .frames_of(Class::Control, Control::Ping.wire_id(), None, bench.at())
            .is_empty(),
        "no PING reply can be decoded"
    );

    // On demand, the knob is switched off and the same exchange is answered again.
    bench.stub.set_faults(FaultKnobs::default());
    let _ = bench.peer.send_ping();
    bench.run_until(300);
    let replies = bench
        .peer
        .frames_of(Class::Control, Control::Ping.wire_id(), Some(200), bench.at());
    assert_eq!(replies.len(), 1, "with the knob off, the PING is answered again");
}

/// A partial drop (`drop_every = 2`) removes every second byte: the seam still carries the
/// baseline streams' bytes, but the reply's framing is destroyed and no PING reply decodes.
#[test]
fn partial_drops_break_the_reply_framing() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::Baseline);
    bench.run_until(100);

    bench.stub.set_faults(FaultKnobs {
        drop_every: 2,
        ..FaultKnobs::default()
    });
    let before = bench.peer.bytes_read();
    let _ = bench.peer.send_ping();
    bench.run_until(200);
    let read = bench.peer.bytes_read() - before;
    assert!(read > 0, "some bytes still cross the seam");
    assert!(
        bench
            .peer
            .frames_of(Class::Control, Control::Ping.wire_id(), None, bench.at())
            .is_empty(),
        "a partially dropped reply cannot decode"
    );
}

/// The corrupt knob flips bytes without removing them: the seam carries the full byte count, but
/// the reply no longer decodes.
#[test]
fn corrupt_knob_breaks_the_outbound_frames() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::Baseline);
    bench.run_until(100);

    bench.stub.set_faults(FaultKnobs {
        corrupt_every: 1,
        ..FaultKnobs::default()
    });
    let before = bench.peer.bytes_read();
    let _ = bench.peer.send_ping();
    bench.run_until(200);
    let read = bench.peer.bytes_read() - before;
    assert!(read > 0, "corruption does not remove bytes from the seam");
    assert!(
        bench
            .peer
            .frames_of(Class::Control, Control::Ping.wire_id(), None, bench.at())
            .is_empty(),
        "a fully corrupted reply cannot decode"
    );

    // Switched off on demand, the same exchange decodes again.
    bench.stub.set_faults(FaultKnobs::default());
    let _ = bench.peer.send_ping();
    bench.run_until(300);
    let replies = bench
        .peer
        .frames_of(Class::Control, Control::Ping.wire_id(), Some(200), bench.at());
    assert_eq!(replies.len(), 1, "with the knob off, the PING is answered again");
}

/// The delay knob holds outbound bytes until their time has passed; the bytes arrive intact.
#[test]
fn delay_knob_holds_outbound_bytes_until_their_time() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::Baseline);
    bench.run_until(100);

    bench.stub.set_faults(FaultKnobs {
        delay_ms: 100,
        ..FaultKnobs::default()
    });
    let _ = bench.peer.send_ping();
    bench.run_until(150);
    assert!(
        bench
            .peer
            .frames_of(Class::Control, Control::Ping.wire_id(), None, bench.at())
            .is_empty(),
        "the reply is held before its release time"
    );

    bench.run_until(205);
    let replies = bench
        .peer
        .frames_of(Class::Control, Control::Ping.wire_id(), Some(150), bench.at());
    assert_eq!(replies.len(), 1, "the held reply is released after the delay");
    assert_eq!(replies[0].payload.len(), 1, "the delayed bytes arrive intact");
}
