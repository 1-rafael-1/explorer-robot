# bridge-net

The Bridge↔daemon network edge (ADR-0037, ADR-0039, ADR-0040): the Discovery Beacon, the
pre-Link `EXPH` HMAC-SHA256 preamble and the fixed ports. It is `no_std` and allocation-free;
sockets, tasks and timers stay in the Bridge firmware and the daemon, its only consumers. The
robot never depends on it.

The crate version mirrors the preamble version (`1.0.0` = preamble `1.0`); a test enforces it.

## Beacon

One UDP datagram, broadcast every **3 s** (`beacon::INTERVAL_MS`) on the Bridge's station
interface:

| field | size |
|---|---|
| magic `EXPR` | 4 B |
| Link Protocol major | 1 B |
| device id | 6 B |
| name, zero-padded UTF-8 | 16 B |
| Link port, little-endian | 2 B |
| flags | 1 B |

Flags are `PROVISIONED` (bit 0), `LINK_READY` (bit 1) and `SETUP_MODE` (bit 2); reserved bits
are cleared on decode. `LINK_READY` has a pinned meaning: a valid Link Protocol frame arrived
from the robot within the last **5 seconds**, so the Beacon never claims a silent robot is
ready. A `tcp_port` of **0** (`beacon::PORTAL_ONLY`) advertises a portal-only Bridge and is
treated as Setup Mode.

`Beacon { major, device_id, name, tcp_port, flags }` encodes with `encode_into` and decodes
with `decode`; trailing bytes are ignored so a later minor can extend the datagram.

## Preamble

Every message is `magic "EXPH"`, preamble major, type, little-endian `u16` payload length and
payload (`HEADER_LEN = 8`). The exchange is:

| type | message | payload |
|---|---|---|
| 1 | `CLIENT_HELLO` | 32-byte client nonce |
| 2 | `BRIDGE_CHALLENGE` | 32-byte bridge proof, 32-byte bridge nonce, `Identity` |
| 3 | `CLIENT_AUTH` | 32-byte client proof |
| 4 | `ACCEPT` | empty; the Link Protocol starts |
| 5 | `DENY` | reason byte, optional `retry_ms` |

`DenyReason` is `Unauthorized`, `Busy`, `SetupMode` or `Version`. `AUTH_DEADLINE_MS` is
**5000**. `encode_into` frames a `Message` and `Message::encode_payload_into` encodes just its
payload; `try_decode` and `Message::decode_payload` are the read side, and `frame_len` returns
the framed length without decoding.

Proofs are HMAC-SHA256 with domain-separated labels (`BRIDGE_PROOF_LABEL`,
`CLIENT_PROOF_LABEL`). `bridge_proof` binds both nonces and the served `Identity`;
`client_proof` binds both nonces. `proofs_match` and the crate's `constant_time_eq` compare
without a short-circuit on the first difference.

`Identity` is the device id (6 B), flags (1 B) and zero-padded name (16 B), shared by the
Beacon and the challenge.

## Ports

| constant | value | owner |
|---|---|---|
| `ports::BEACON_UDP` | 9001 | the Bridge broadcasts, the daemon listens |
| `ports::LINK_TCP_DEFAULT` | 9000 | manual endpoint default; a Beacon advertises its own |

The app-facing HTTP port (8765) belongs to the daemon and is not part of this contract.

## Using it

Both sides call the same functions; there is no feature gate and no `std` feature. The
consumer owns the socket; this crate owns the bytes.

```rust
use bridge_net::beacon::{self, Beacon};

const BEACON_BYTES: usize = beacon::ENCODED_LEN;

fn forward(bytes: &[u8]) -> Result<(), bridge_net::error::DecodeError> {
    let beacon = Beacon::decode(bytes)?;
    let _ = (beacon.tcp_port, beacon.flags);
    Ok(())
}

fn encode(beacon: &Beacon, out: &mut [u8]) -> Result<usize, remote_protocol::EncodeError> {
    beacon.encode_into(out)
}
```

## Tests

From `remote/`:

```sh
cargo test -p bridge-net
```

`beacon` pins the wire layout and flag semantics, `preamble` pins the framing, message
round-trips and the proof formula, and `version` guards the version mirror.

## Validation

The `bridge` CI job also compiles this crate for `riscv32imc-unknown-none-elf`: the firmware
path-depends on it, so building the Bridge is the cross-target gate (ADR-0040). Locally, from
`remote/`:

```sh
cargo +nightly fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
```

## Known limits

- No sockets, scheduling or retries: the consumers own the I/O loops.
- The HMAC/SHA-256 code size on the ESP32-C3 remains an ADR-0039 open item.
- The token is a byte slice; the 60-bit pairing-token policy lives in the Bridge and daemon.
