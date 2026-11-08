//! The standalone Bridge Stub binary (ticket 09): manual integration runs and the joint
//! milestone's early rehearsals. Everything it serves is the library's scenario, parsed from the
//! command line by [`bridge_stub::cli`].

use bridge_stub::cli;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match cli::parse(&args) {
        Ok(cli::Invocation::Help) => {
            print!("{}", cli::USAGE);
        }
        Ok(cli::Invocation::Run(options)) => {
            if let Err(message) = cli::run(&options) {
                eprintln!("bridge-stub: {message}");
                std::process::exit(1);
            }
        }
        Err(message) => {
            eprintln!("bridge-stub: {message}");
            eprint!("{}", cli::USAGE);
            std::process::exit(2);
        }
    }
}
