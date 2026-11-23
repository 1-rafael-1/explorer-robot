#![no_std]

//! The HAL-bound half of the Bridge firmware, shared by the `bridge` binary and the `stack-soak`
//! regression soak.
//!
//! Everything that can be decided without the radio lives in `bridge-core`; this crate holds the
//! pieces that must call `esp-radio` and are therefore untestable on the host. The modules here
//! are deliberately small: [`wifi`] is the join and diagnostic-scan helper both binaries execute
//! when the Radio Session returns a join action (ADR-0058).

pub mod wifi;
