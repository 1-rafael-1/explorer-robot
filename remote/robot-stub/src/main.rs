//! The Robot Stub's bench binary (ticket 02): the library's personas behind a real serial port.
//!
//! The port opens at ADR-0033's 921600 8N1 — the baud all three projects share — and the stub
//! steps on the wall clock. The persona, the Boot Id and the fault knobs come from the command
//! line; hand-rolled parsing keeps the bench binary dependency-light beyond the port library.
//!
//! Usage:
//!
//! ```text
//! robot-stub --port /dev/ttyUSB0 [--persona baseline|worst-case|bridge-control]
//!            [--action 0|1|2] [--boot-id N]
//!            [--drop-every N] [--corrupt-every N] [--delay-ms N]
//! ```

use std::{
    io::{Read, Write},
    process::ExitCode,
    thread,
    time::{Duration, Instant},
};

use remote_link::timing::BAUD;
use robot_stub::{BridgeControlReport, Config, FaultKnobs, Persona, Stub, Transport, worst_case_config};
use serialport::{DataBits, Parity, SerialPort, StopBits};

/// The stub's step cadence on the wall clock, matching the firmware loop's granularity.
const STEP_MS: u64 = 10;

/// How often the binary prints a one-line link summary.
const STATS_MS: u64 = 5_000;

/// The Boot Id the stub mints when the operator does not choose one.
const DEFAULT_BOOT_ID: u32 = 0x5354_5542;

/// The parsed command line.
struct Args {
    /// The serial port's device path.
    port: String,
    /// The load persona.
    persona: Persona,
    /// The `BRIDGE_CONTROL` action to send once, if any.
    action: Option<u8>,
    /// The Boot Id to mint for this powered link period.
    boot_id: u32,
    /// The fault knobs.
    faults: FaultKnobs,
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("robot-stub: {message}");
            ExitCode::FAILURE
        }
    }
}

/// Opens the port, builds the stub and runs the bench loop until interrupted.
fn run() -> Result<(), String> {
    let args = parse_args()?;
    let config = match args.persona {
        Persona::WorstCase => worst_case_config(),
        Persona::Baseline | Persona::BridgeControl => Config::default(),
    };
    let port = serialport::new(&args.port, BAUD)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(1))
        .open()
        .map_err(|error| format!("opening {} at {BAUD} 8N1: {error}", args.port))?;
    let mut stub = Stub::new(config, args.boot_id, args.persona, SerialTransport { port });
    stub.set_faults(args.faults);

    println!(
        "robot-stub: persona {:?} on {} at {BAUD} 8N1, boot id {:#010x}, faults {:?}",
        args.persona, args.port, args.boot_id, args.faults
    );

    if let Some(action) = args.action {
        match stub.request_bridge_control(action, 0) {
            Ok(seq) => println!("robot-stub: BRIDGE_CONTROL action {action} queued as seq {seq}"),
            Err(status) => return Err(format!("BRIDGE_CONTROL action {action} refused: {status:?}")),
        }
    }

    let start = Instant::now();
    let mut printed_executions = 0;
    let mut next_stats = STATS_MS;
    loop {
        thread::sleep(Duration::from_millis(STEP_MS));
        let now_ms = u64::try_from(start.elapsed().as_millis()).unwrap_or(u64::MAX);
        stub.step(now_ms);

        // The execution log is a bounded ring; if it wrapped, start printing from the oldest.
        let executions = stub.executions().count();
        if executions < printed_executions {
            printed_executions = 0;
        }
        for execution in stub.executions().skip(printed_executions) {
            println!(
                "robot-stub: executed {:?} seq {} -> {:?}",
                execution.command, execution.seq, execution.status
            );
        }
        printed_executions = executions;

        while let Some(report) = stub.take_bridge_control_report() {
            match report {
                BridgeControlReport::Replied { action, seq, reply } => {
                    println!("robot-stub: BRIDGE_CONTROL action {action} seq {seq} replied: {reply:?}");
                }
                BridgeControlReport::TimedOut { action, seq } => {
                    println!("robot-stub: BRIDGE_CONTROL action {action} seq {seq} timed out");
                }
            }
        }

        if now_ms >= next_stats {
            println!(
                "robot-stub: {now_ms} ms, peer {:?}, {} events dropped",
                stub.peer(),
                stub.dropped_events()
            );
            next_stats += STATS_MS;
        }
    }
}

/// Parses the command line, naming the flag in every error.
fn parse_args() -> Result<Args, String> {
    let flags: Vec<String> = std::env::args_os()
        .map(std::ffi::OsString::into_string)
        .collect::<Result<_, _>>()
        .map_err(|_| String::from("arguments must be valid UTF-8"))?;

    let mut args = Args {
        port: String::new(),
        persona: Persona::Baseline,
        action: None,
        boot_id: DEFAULT_BOOT_ID,
        faults: FaultKnobs::default(),
    };
    let mut index = 1;
    while index < flags.len() {
        let flag = flags[index].as_str();
        let value = flags.get(index + 1).ok_or_else(|| format!("{flag} needs a value"))?;
        match flag {
            "--port" => args.port.clone_from(value),
            "--persona" => {
                args.persona = match value.as_str() {
                    "baseline" => Persona::Baseline,
                    "worst-case" => Persona::WorstCase,
                    "bridge-control" => Persona::BridgeControl,
                    other => return Err(format!("unknown persona: {other}")),
                };
            }
            "--action" => args.action = Some(parse_number(flag, value)?),
            "--boot-id" => args.boot_id = parse_number(flag, value)?,
            "--drop-every" => args.faults.drop_every = parse_number(flag, value)?,
            "--corrupt-every" => args.faults.corrupt_every = parse_number(flag, value)?,
            "--delay-ms" => args.faults.delay_ms = parse_number(flag, value)?,
            other => return Err(format!("unknown argument: {other}")),
        }
        index += 2;
    }
    if args.port.is_empty() {
        return Err(String::from("--port is required (e.g. --port /dev/ttyUSB0)"));
    }
    if args.action.is_some() && args.persona != Persona::BridgeControl {
        return Err(String::from("--action needs --persona bridge-control"));
    }
    Ok(args)
}

/// Parses one numeric flag value, naming the flag in the error.
fn parse_number<T>(flag: &str, value: &str) -> Result<T, String>
where
    T: std::str::FromStr,
    <T as std::str::FromStr>::Err: std::fmt::Display,
{
    value.parse().map_err(|error| format!("{flag} needs a number: {error}"))
}

/// A real serial port behind the library's transport seam.
struct SerialTransport {
    /// The open port.
    port: Box<dyn SerialPort>,
}

impl Transport for SerialTransport {
    fn read(&mut self, out: &mut [u8]) -> usize {
        // A read timeout means nothing new arrived; the next step drains again.
        self.port.read(out).unwrap_or(0)
    }

    fn write(&mut self, bytes: &[u8]) {
        if let Err(error) = self.port.write_all(bytes) {
            eprintln!("robot-stub: serial write failed: {error}");
        }
    }
}
