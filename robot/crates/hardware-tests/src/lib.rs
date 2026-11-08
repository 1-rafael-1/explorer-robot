//! Standalone, breadboard-scale hardware tests for the explorer-robot sensors.
//!
//! This crate is a workspace member that hosts small, focused hardware checks —
//! each a self-contained embassy binary under [`examples`](examples) that wires
//! one or two sensors directly to the RP2350, independently of the main robot
//! firmware. Individual tests live in `examples/`; shared helper code (e.g. TFT
//! or `LiDAR` bring-up) can move here as more tests are added.

#![no_std]
