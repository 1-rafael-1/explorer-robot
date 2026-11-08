//! The binary's command line: hand-rolled parsing, no CLI dependency.
//!
//! The knobs are the scenario's: the token, the ports, the identity flags, the deny script and
//! the traffic. [`parse`] and [`scenario`] are exported so the library's tests exercise exactly
//! what the binary runs.

use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4},
    thread,
    time::Duration,
};

use bridge_net::{beacon, ports};
use remote_protocol::{
    CalibrationKind, CalibrationLoadStatus, CompletionStatus, OBSTACLE_SOURCE_LIDAR, ProcedureId, Stream,
    payload::stream::ActivityStage,
};

use crate::{
    identity::{Flags, Name},
    preamble::DenyReason,
    script::{BeaconScript, Deny, Scenario, ScriptEvent, StreamRate},
    serve::Stub,
};

/// The usage text, printed by `--help` and on a parse error.
pub const USAGE: &str = "\
bridge-stub: the daemon's bench peer (ticket 09)
usage: bridge-stub [options]

options:
  --token <hex>            the pairing token's bytes; default 62656e63682d746f6b656e
  --name <text>            the display name; default Explorer-STUB
  --bind <ip>              the Link listener's address; default 127.0.0.1
  --link-port <port>        the Link Protocol port; default 9000, 0 picks an ephemeral port
  --beacon-addr <ip:port>  the Beacon datagram's destination; default 255.255.255.255:9001
  --beacon-interval <ms>   the Beacon interval; default 3000
  --setup-mode             serve the setup-mode flag and deny with setup_mode
  --portal-only            advertise tcp_port = 0: a portal-only Bridge
  --deny <reason>          deny every connection: unauthorized, busy, setup_mode or version
  --retry-ms <ms>          the deny's retry hint
  --stream <name>:<chz>    script a stream at a rate; repeatable
  --event <name>           script an event; repeatable

streams: power motion activity perception lidar_cloud calibration
events:  obstacle floor_drop procedure_completed drive_distance_completed
         rotate_completed calibration_loaded event_overflow
";

/// What the command line asked for.
#[derive(Clone, Debug)]
pub enum Invocation {
    /// Serve the parsed options.
    Run(Options),
    /// Print the usage and exit successfully.
    Help,
}

/// The binary's parsed options.
#[derive(Clone, Debug)]
pub struct Options {
    /// The pairing token's bytes.
    pub token: Vec<u8>,
    /// The display name.
    pub name: String,
    /// The Link listener's bind address.
    pub bind: IpAddr,
    /// The Link Protocol port; zero picks an ephemeral port.
    pub link_port: u16,
    /// The Beacon datagram's destination.
    pub beacon_addr: SocketAddr,
    /// The Beacon interval in milliseconds.
    pub beacon_interval_ms: u64,
    /// Serve the Setup Mode flag and deny with `setup_mode`.
    pub setup_mode: bool,
    /// Advertise `tcp_port = 0`: a portal-only Bridge.
    pub portal_only: bool,
    /// Deny every connection with this reason.
    pub deny: Option<DenyReason>,
    /// The deny's `retry_ms`.
    pub deny_retry_ms: Option<u32>,
    /// The scripted streams, as `name:rate_chz`.
    pub streams: Vec<StreamRate>,
    /// The scripted events, by name.
    pub events: Vec<ScriptEvent>,
}

/// The documented default options.
#[must_use]
pub fn bench_defaults() -> Options {
    Options {
        token: b"bench-token".to_vec(),
        name: String::from("Explorer-STUB"),
        bind: IpAddr::V4(Ipv4Addr::LOCALHOST),
        link_port: ports::LINK_TCP_DEFAULT,
        beacon_addr: SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::BROADCAST, ports::BEACON_UDP)),
        beacon_interval_ms: beacon::INTERVAL_MS,
        setup_mode: false,
        portal_only: false,
        deny: None,
        deny_retry_ms: None,
        streams: Vec::new(),
        events: Vec::new(),
    }
}

/// Parses the command line.
///
/// # Errors
///
/// Returns a message naming the offending argument.
pub fn parse(args: &[String]) -> Result<Invocation, String> {
    let mut options = bench_defaults();
    let mut index = 0;
    while index < args.len() {
        let flag = args[index].as_str();
        index += 1;
        match flag {
            "--help" | "-h" => return Ok(Invocation::Help),
            "--setup-mode" => options.setup_mode = true,
            "--portal-only" => options.portal_only = true,
            "--token" => options.token = parse_hex(value(args, &mut index, flag)?)?,
            "--name" => options.name = String::from(value(args, &mut index, flag)?),
            "--bind" => {
                options.bind = parse_ip(value(args, &mut index, flag)?, flag)?;
            }
            "--link-port" => {
                options.link_port = parse_number(value(args, &mut index, flag)?, flag)?;
            }
            "--beacon-addr" => {
                options.beacon_addr = parse_socket(value(args, &mut index, flag)?, flag)?;
            }
            "--beacon-interval" => {
                options.beacon_interval_ms = parse_number(value(args, &mut index, flag)?, flag)?;
            }
            "--deny" => {
                options.deny = Some(parse_deny(value(args, &mut index, flag)?)?);
            }
            "--retry-ms" => {
                options.deny_retry_ms = Some(parse_number(value(args, &mut index, flag)?, flag)?);
            }
            "--stream" => {
                options.streams.push(parse_stream(value(args, &mut index, flag)?)?);
            }
            "--event" => {
                options.events.push(parse_event(value(args, &mut index, flag)?)?);
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(Invocation::Run(options))
}

/// Builds the scenario the options describe.
///
/// # Errors
///
/// Returns a message when a value cannot be represented on the wire, such as a name that does
/// not fit its field.
pub fn scenario(options: &Options) -> Result<Scenario, String> {
    let mut scenario = Scenario::new(&options.token);
    match Name::new(&options.name) {
        Some(name) => scenario.identity.name = name,
        None => return Err(format!("the name {} does not fit the wire field", options.name)),
    }
    scenario.identity.flags = if options.setup_mode {
        Flags::PROVISIONED | Flags::SETUP_MODE
    } else {
        Flags::PROVISIONED | Flags::LINK_READY
    };
    scenario.bind = options.bind;
    scenario.link_port = if options.link_port == 0 {
        None
    } else {
        Some(options.link_port)
    };
    scenario.beacon = BeaconScript {
        tcp_port: if options.portal_only {
            Some(beacon::PORTAL_ONLY)
        } else {
            None
        },
        interval_ms: options.beacon_interval_ms,
        target: options.beacon_addr,
    };
    scenario.deny = match options.deny {
        Some(reason) => Some(Deny {
            reason,
            retry_ms: options.deny_retry_ms,
        }),
        None if options.setup_mode => Some(Deny {
            reason: DenyReason::SetupMode,
            retry_ms: options.deny_retry_ms,
        }),
        None => None,
    };
    if !options.streams.is_empty() {
        scenario.traffic.streams.clone_from(&options.streams);
    }
    scenario.traffic.events.clone_from(&options.events);
    Ok(scenario)
}

/// Serves the options' scenario until the process is killed.
///
/// # Errors
///
/// Returns a message when the scenario could not be built or serving could not start.
pub fn run(options: &Options) -> Result<(), String> {
    let interval_ms = options.beacon_interval_ms;
    let scenario = scenario(options)?;
    let stub = Stub::start(scenario).map_err(|error| error.to_string())?;
    println!("bridge-stub listening on {}", stub.link_addr());
    println!("bridge-stub beacon to {} every {interval_ms}ms", stub.beacon_target());
    loop {
        thread::sleep(Duration::from_secs(1));
    }
}

/// Takes the value that follows `flag`.
fn value<'a>(args: &'a [String], index: &mut usize, flag: &str) -> Result<&'a str, String> {
    let Some(value) = args.get(*index) else {
        return Err(format!("{flag} needs a value"));
    };
    *index += 1;
    Ok(value.as_str())
}

/// Parses a hex byte string.
fn parse_hex(text: &str) -> Result<Vec<u8>, String> {
    if !text.len().is_multiple_of(2) {
        return Err(format!("the token {text} is not a whole number of bytes"));
    }
    let mut token = Vec::with_capacity(text.len() / 2);
    for pair in text.as_bytes().chunks(2) {
        let high = hex_digit(pair[0], text)?;
        let low = hex_digit(pair[1], text)?;
        token.push((high << 4) | low);
    }
    Ok(token)
}

/// Parses one hex digit.
fn hex_digit(byte: u8, text: &str) -> Result<u8, String> {
    match byte {
        b'0'..=b'9' => Ok(byte - b'0'),
        b'a'..=b'f' => Ok(byte - b'a' + 10),
        b'A'..=b'F' => Ok(byte - b'A' + 10),
        _ => Err(format!("the token {text} is not hexadecimal")),
    }
}

/// Parses a number for `flag`.
fn parse_number<T: std::str::FromStr>(text: &str, flag: &str) -> Result<T, String> {
    text.parse().map_err(|_| format!("{flag} wants a number, got {text}"))
}

/// Parses an IP address for `flag`.
fn parse_ip(text: &str, flag: &str) -> Result<IpAddr, String> {
    text.parse()
        .map_err(|_| format!("{flag} wants an IP address, got {text}"))
}

/// Parses a socket address for `flag`.
fn parse_socket(text: &str, flag: &str) -> Result<SocketAddr, String> {
    text.parse().map_err(|_| format!("{flag} wants an ip:port, got {text}"))
}

/// Parses a deny reason name.
fn parse_deny(text: &str) -> Result<DenyReason, String> {
    match text {
        "unauthorized" => Ok(DenyReason::Unauthorized),
        "busy" => Ok(DenyReason::Busy),
        "setup_mode" => Ok(DenyReason::SetupMode),
        "version" => Ok(DenyReason::Version),
        other => Err(format!("unknown deny reason {other}")),
    }
}

/// Parses a `name:rate` stream script.
fn parse_stream(text: &str) -> Result<StreamRate, String> {
    let Some((name, rate)) = text.split_once(':') else {
        return Err(format!("--stream wants name:chz, got {text}"));
    };
    let stream = parse_stream_name(name)?;
    let rate_chz = parse_number(rate, "--stream")?;
    Ok(StreamRate { stream, rate_chz })
}

/// Parses a stream name.
fn parse_stream_name(name: &str) -> Result<Stream, String> {
    match name {
        "power" => Ok(Stream::Power),
        "motion" => Ok(Stream::Motion),
        "activity" => Ok(Stream::Activity),
        "perception" => Ok(Stream::Perception),
        "lidar_cloud" => Ok(Stream::LidarCloud),
        "calibration" => Ok(Stream::Calibration),
        other => Err(format!("unknown stream {other}")),
    }
}

/// Parses one event name into its fixed scripted payload.
fn parse_event(text: &str) -> Result<ScriptEvent, String> {
    match text {
        "obstacle" => Ok(ScriptEvent::Obstacle {
            detected: 1,
            source: OBSTACLE_SOURCE_LIDAR,
        }),
        "floor_drop" => Ok(ScriptEvent::FloorDrop { detected: 1 }),
        "procedure_completed" => Ok(ScriptEvent::ProcedureCompleted {
            procedure_id: ProcedureId::COAST_AND_AVOID,
            stage: ActivityStage::Complete,
            detail: String::from("done"),
        }),
        "drive_distance_completed" => Ok(ScriptEvent::DriveDistanceCompleted {
            command_seq: 1,
            status: CompletionStatus::Success,
            target_cm: 50,
            distance_cm: 50,
            duration_ms: 2_500,
            detail: String::from("done"),
        }),
        "rotate_completed" => Ok(ScriptEvent::RotateCompleted {
            command_seq: 1,
            status: CompletionStatus::Success,
            target_cdeg: 9_000,
            error_cdeg: 0,
            duration_ms: 1_800,
            detail: String::from("done"),
        }),
        "calibration_loaded" => Ok(ScriptEvent::CalibrationLoaded {
            kind: CalibrationKind::Motor,
            status: CalibrationLoadStatus::Loaded,
        }),
        "event_overflow" => Ok(ScriptEvent::EventOverflow { dropped: 1 }),
        other => Err(format!("unknown event {other}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Parses `args` as a run invocation.
    fn run(args: &[&str]) -> Result<Options, String> {
        let owned: Vec<String> = args.iter().map(ToString::to_string).collect();
        match parse(&owned)? {
            Invocation::Run(options) => Ok(options),
            Invocation::Help => Err(String::from("expected a run, got --help")),
        }
    }

    /// The defaults are the documented bench knobs.
    #[test]
    fn the_defaults_are_the_documented_bench() -> Result<(), String> {
        let options = run(&[])?;
        assert_eq!(options.token, b"bench-token".to_vec());
        assert_eq!(options.name, "Explorer-STUB");
        assert_eq!(options.bind, IpAddr::V4(Ipv4Addr::LOCALHOST));
        assert_eq!(options.link_port, ports::LINK_TCP_DEFAULT);
        assert_eq!(
            options.beacon_addr,
            SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::BROADCAST, ports::BEACON_UDP))
        );
        assert_eq!(options.beacon_interval_ms, beacon::INTERVAL_MS);
        assert!(!options.setup_mode && !options.portal_only);
        assert_eq!(options.deny, None);
        Ok(())
    }

    /// `--help` short-circuits the parse.
    #[test]
    fn help_short_circuits() {
        let owned = vec![String::from("--help")];
        assert!(matches!(parse(&owned), Ok(Invocation::Help)));
    }

    /// The token parses as hex bytes.
    #[test]
    fn the_token_parses_as_hex() -> Result<(), String> {
        let options = run(&["--token", "706169696e"])?;
        assert_eq!(options.token, vec![0x70, 0x61, 0x69, 0x69, 0x6e]);
        assert!(
            parse(&[String::from("--token"), String::from("706")]).is_err(),
            "an odd-length token is rejected"
        );
        Ok(())
    }

    /// The deny knob parses its reason and retry hint.
    #[test]
    fn the_deny_knob_parses() -> Result<(), String> {
        let options = run(&["--deny", "busy", "--retry-ms", "4000"])?;
        assert_eq!(options.deny, Some(DenyReason::Busy));
        assert_eq!(options.deny_retry_ms, Some(4_000));
        assert!(parse(&[String::from("--deny"), String::from("nope")]).is_err());
        Ok(())
    }

    /// The stream and event knobs parse their names.
    #[test]
    fn the_traffic_knobs_parse() -> Result<(), String> {
        let options = run(&["--stream", "motion:500", "--event", "obstacle"])?;
        assert_eq!(
            options.streams,
            vec![StreamRate {
                stream: Stream::Motion,
                rate_chz: 500
            }]
        );
        assert!(matches!(options.events[0], ScriptEvent::Obstacle { .. }));
        assert!(parse(&[String::from("--stream"), String::from("motion")]).is_err());
        Ok(())
    }

    /// An unknown argument or a bad value is rejected with a message.
    #[test]
    fn bad_arguments_are_rejected() {
        assert!(parse(&[String::from("--nope")]).is_err());
        assert!(
            parse(&[String::from("--link-port"), String::from("many")]).is_err(),
            "a non-numeric port is rejected"
        );
        assert!(
            parse(&[String::from("--link-port")]).is_err(),
            "a missing value is rejected"
        );
    }

    /// The scenario wires the knobs: setup mode flags and denies, portal-only zeroes the port.
    #[test]
    fn the_scenario_wires_the_knobs() -> Result<(), String> {
        let options = run(&["--setup-mode", "--portal-only"])?;
        let built = scenario(&options)?;
        assert!(built.identity.flags.contains(Flags::SETUP_MODE));
        assert!(!built.identity.flags.contains(Flags::LINK_READY));
        assert_eq!(built.deny.map(|deny| deny.reason), Some(DenyReason::SetupMode));
        assert_eq!(built.beacon.tcp_port, Some(beacon::PORTAL_ONLY));

        let options = run(&["--deny", "version", "--retry-ms", "250"])?;
        let built = scenario(&options)?;
        assert_eq!(
            built.deny,
            Some(Deny {
                reason: DenyReason::Version,
                retry_ms: Some(250)
            })
        );
        Ok(())
    }
}
