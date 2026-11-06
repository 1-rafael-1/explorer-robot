# remote — the PC side

The host workspace: `remote-daemon`, the shared `bridge-net` crate and the static app in `app/`.

Cargo reads `.cargo/config.toml` from the current directory **upward**, and the repository root
sets the robot's `thumbv8m.main-none-eabihf` default. Run host-workspace commands from this
directory (or below); `cargo build --manifest-path remote/Cargo.toml` from the root is not
equivalent and targets the robot. See ADR-0040.

```sh
cargo run -p remote-daemon
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
```
