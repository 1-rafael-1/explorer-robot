//! Host tests for the `BRIDGE_CONTROL` persona (ticket 02, ADR-0036, ADR-0037).
//!
//! The stub sends the actions and reports the Bridge's reply — or its timeout. The peer plays the
//! Bridge's UART side: it consumes the request and acknowledges with the setup credentials or the
//! pairing code, exactly as the Bridge's firmware will.

mod support;

use remote_link::timing::CONTROL_TIMEOUT_MS;
use remote_protocol::{
    BridgeControlAction, BridgeControlReply, Class, Control, Flags, Status,
    payload::control::{PairingCode, SetupCredentials},
};
use robot_stub::{BridgeControlReport, Config, OwnedReply, Persona};
use support::Bench;

/// The Boot Id the stub mints for this powered link period.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// The setup credentials the Bridge under test reports (the setup passphrase is public by
/// design, ADR-0036).
const SETUP_SSID: &str = "ExplorerRobot-ABCD";
const SETUP_PASSPHRASE: &str = "setup-mode";

/// The twelve-character pairing code the Bridge under test reports (ADR-0037).
const PAIRING_CODE: &str = "ABCD2345EFGH";

/// Action 0 leaves at control priority, and the reply surfaces the Bridge's setup SSID and
/// passphrase. An unknown action is refused with `BAD_ARGUMENT` before anything is sent.
#[test]
fn action_zero_returns_the_setup_credentials() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::BridgeControl);

    let refused = bench.stub.request_bridge_control(3, 0);
    assert_eq!(
        refused.err(),
        Some(Status::BadArgument),
        "an unknown action is refused with the robot's own rule"
    );

    let requested = bench
        .stub
        .request_bridge_control(BridgeControlAction::ForgetNetwork.bits(), 0);
    assert!(requested.is_ok(), "action 0 is accepted");
    let seq = requested.unwrap_or(0);
    bench.step_now();

    let requests = bench
        .peer
        .frames_of(Class::Control, Control::BridgeControl.wire_id(), None, bench.at());
    assert_eq!(requests.len(), 1, "the request is sent once");
    let request = requests[0];
    assert_eq!(request.seq, seq, "the wire carries the returned sequence");
    assert!(
        request.flags.contains(Flags::ACK_REQUESTED),
        "the request asks for its acknowledgement"
    );
    assert_eq!(
        request.payload,
        &[BridgeControlAction::ForgetNetwork.bits()],
        "the payload is the action byte"
    );

    let reply = BridgeControlReply::Setup(SetupCredentials {
        status: Status::Ok,
        ssid: SETUP_SSID,
        passphrase: SETUP_PASSPHRASE,
    });
    bench.peer.ack_bridge_control(seq, &reply);
    bench.run_until(50);

    let report = bench.stub.take_bridge_control_report();
    assert!(report.is_some(), "the reply is reported");
    assert!(
        matches!(report, Some(BridgeControlReport::Replied { .. })),
        "the report is a reply, got {report:?}"
    );
    if let Some(BridgeControlReport::Replied {
        action,
        seq: reply_seq,
        reply,
    }) = report
    {
        assert_eq!(action, 0);
        assert_eq!(reply_seq, seq);
        assert_eq!(
            reply,
            OwnedReply::Setup {
                status: Status::Ok,
                ssid: String::from(SETUP_SSID),
                passphrase: String::from(SETUP_PASSPHRASE),
            },
            "the report carries the Bridge's setup credentials"
        );
    }
}

/// Actions 1 and 2 both return the Bridge's pairing code.
#[test]
fn actions_one_and_two_return_the_pairing_code() {
    for action in [
        BridgeControlAction::ShowPairingCode,
        BridgeControlAction::RotatePairingCode,
    ] {
        let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::BridgeControl);

        let requested = bench.stub.request_bridge_control(action.bits(), 0);
        assert!(requested.is_ok(), "action {} is accepted", action.bits());
        let seq = requested.unwrap_or(0);
        bench.step_now();

        let requests = bench
            .peer
            .frames_of(Class::Control, Control::BridgeControl.wire_id(), None, bench.at());
        assert_eq!(requests.len(), 1, "the request is sent once");
        assert_eq!(requests[0].payload, &[action.bits()], "the payload is the action byte");

        let reply = BridgeControlReply::PairingCode(PairingCode {
            status: Status::Ok,
            code: PAIRING_CODE,
        });
        bench.peer.ack_bridge_control(seq, &reply);
        bench.run_until(50);

        let report = bench.stub.take_bridge_control_report();
        assert!(report.is_some(), "the reply is reported");
        assert!(
            matches!(report, Some(BridgeControlReport::Replied { .. })),
            "the report is a reply, got {report:?}"
        );
        if let Some(BridgeControlReport::Replied {
            action: reported_action,
            seq: reply_seq,
            reply,
        }) = report
        {
            assert_eq!(reported_action, action.bits());
            assert_eq!(reply_seq, seq);
            assert_eq!(
                reply,
                OwnedReply::PairingCode {
                    status: Status::Ok,
                    code: String::from(PAIRING_CODE),
                },
                "the report carries the Bridge's pairing code"
            );
        }
    }
}

/// A request the Bridge never acknowledges is reported as a timeout on the shipped schedule
/// (ADR-0030's control timeout, [`remote_link::timing::CONTROL_TIMEOUT_MS`]).
#[test]
fn an_unacknowledged_request_is_reported_as_a_timeout() {
    let mut bench = Bench::new(Config::default(), BOOT_ID, Persona::BridgeControl);

    let requested = bench
        .stub
        .request_bridge_control(BridgeControlAction::ShowPairingCode.bits(), 0);
    assert!(requested.is_ok(), "the request is accepted");
    let seq = requested.unwrap_or(0);
    bench.step_now();

    // The Bridge stays silent: no report until the control timeout has elapsed.
    bench.run_until(CONTROL_TIMEOUT_MS - 5);
    assert!(
        bench.stub.take_bridge_control_report().is_none(),
        "no report before the control timeout"
    );

    bench.run_until(CONTROL_TIMEOUT_MS + 5);
    let report = bench.stub.take_bridge_control_report();
    assert!(report.is_some(), "the timeout is reported");
    assert!(
        matches!(report, Some(BridgeControlReport::TimedOut { .. })),
        "the report is a timeout, got {report:?}"
    );
    if let Some(BridgeControlReport::TimedOut {
        action,
        seq: timed_out_seq,
    }) = report
    {
        assert_eq!(action, BridgeControlAction::ShowPairingCode.bits());
        assert_eq!(timed_out_seq, seq);
    }
}
