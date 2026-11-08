# Reliability mechanisms live in the shared protocol crate

The Link Protocol's reliability rules — command sequencing and dedupe, ACK tracking and retry,
event dedupe, throttle merge and TTL — are needed by every peer, not only the robot. ADR-0030
puts "the reliability primitives (sequence, ACK tracking, dedupe, throttle)" in
`remote-protocol`. ADR-0032 gives `remote-link::LinkCore` the event ring, lease expiry, command
dedupe and TX priority. ADR-0040 then makes `remote-link` invisible to the Bridge and the host
workspace, so the daemon cannot reuse anything that lives there. Read literally, the three
decisions leave the daemon hand-writing the same mechanisms, which is the drift the shared crate
exists to prevent.

**Status:** accepted. The split is by **mechanism versus policy**.

- **`remote-protocol` owns the mechanisms**: wrap-safe sequence ordering, ACK tracking with the
  retry schedule, dedupe windows keyed by `(epoch, seq)` and `(boot_id, seq)`, and `Throttle`
  merge/TTL. They are pure and take time or sequence numbers as parameters. The crate still has no
  clock, I/O, allocator or logging.
- **`remote-link::LinkCore` owns robot-only policy** and composes those mechanisms with the parser,
  pending stream slots, event ring, lease, Peer/Deadman and TX priority.
- **The daemon** composes its own relay state from the same mechanisms.
- **The Bridge** uses only the envelope, CRC and control types.

**Considered Options**

- **All reliability in `remote-link`.** Rejected. ADR-0040 keeps `remote-link` out of the host
  workspace, so the daemon would have to duplicate the mechanisms — exactly the drift the shared
  crate exists to prevent.
- **Duplicate the mechanisms in the daemon.** Rejected. Command sequencing, retry and event dedupe
  are where the bugs are; two implementations would diverge on the first change, and only one side
  would have the host tests.
- **Move `LinkCore`'s policy into `remote-protocol`.** Rejected. The Bridge and the daemon do not
  need the robot's parser, stream slots, lease or Deadman, and the protocol crate would grow a
  robot-shaped state machine it cannot host-test without the robot's configuration.
- **A `reliability` feature on the protocol crate.** Rejected for v1. The mechanisms are small and
  `no_std`; a feature split buys code size that link-time optimisation already removes, at the cost
  of a second compile configuration to test.

**Decision**

`remote-protocol` exposes the mechanisms as small, time-parameterised types. `remote-link` does not
reimplement them; it composes them. The observable rules are unchanged: commands are
`last + 1` / duplicate / `STALE`, events use a dedupe window rather than a single last sequence,
and the robot applies the most restrictive of its Requested Rate and any active `Throttle`.

**Consequences**

- ADR-0030's "reliability primitives" means mechanism; ADR-0032's `LinkCore` means composition.
- A new reliability rule is a protocol-crate change, so the daemon and the robot move together
  instead of one of them silently lagging.
- The protocol crate must stay free of I/O and time sources; every mechanism takes `now` or a
  sequence value as an argument.
- Host tests can drive both ends from the shared simulation; `remote-link` tests exercise the robot
  policy on top of the same mechanisms.
- The Bridge remains envelope-only and gains no dependency on the mechanisms.
