//! Copies `memory.x` into the linker search path and applies the RP2350
//! link arguments needed for the bare-metal example binaries.

use std::{env, fs::File, io::Write, path::PathBuf};

#[allow(clippy::unwrap_used)]
fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory.x");

    // The `--nmagic`/`-Tlink.x`/`-Tdefmt.x` linker flags are ARM-bare-metal
    // specific. Emitting them on host targets (where `cargo test` builds the
    // auto-discovered example) makes the host linker reject the command line.
    let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap();
    let os = env::var("CARGO_CFG_TARGET_OS").unwrap();
    if arch == "arm" && os == "none" {
        println!("cargo:rustc-link-arg-examples=--nmagic");
        println!("cargo:rustc-link-arg-examples=-Tlink.x");
        println!("cargo:rustc-link-arg-examples=-Tdefmt.x");
    }
}
