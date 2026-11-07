#!/usr/bin/env python3
"""A dependency-free Link Protocol client for the Bridge bench.

The probe connects to the Bridge's raw TCP listener, sends one valid ``HELLO`` control frame,
and prints every robot frame it receives (streams, events, commands and controls) while counting
them. It is the ticket-04 "TCP client": a successful run shows a full ``HELLO``/``HELLO_ACK``
round trip through the Bridge and the Robot Stub.

Usage:
    python3 tools/link-probe.py 192.168.1.42
    python3 tools/link-probe.py 192.168.1.42 --duration 60 --quiet
    python3 tools/link-probe.py 192.168.1.42 --count 100
    python3 tools/link-probe.py 192.168.1.42 --loopback   # GPIO6↔GPIO7 jumper, no robot

Exit status:
    0  a ``HELLO_ACK`` was decoded and the requested count (if any) was reached;
       with ``--loopback``, the ``HELLO`` was sent (the evidence is on the Bridge)
    1  the idle timeout expired, the peer closed, or the expected frame did not arrive

With UART1's TX and RX jumpered instead of wired to a robot, the Bridge forwards the probe's
``HELLO`` out of the UART and the jumper returns it. The router then **ignores** it by design: a
robot-side ``HELLO`` without ``IS_ACK`` is not a legal robot frame, so nothing can come back over
the socket. The ``--loopback`` run therefore sends the frame and exits 0; the proof is the
Bridge's status line, where ``tcp_rx``, ``uart_tx``, ``uart_rx`` and ``good`` must rise while
``crc``, ``resync``, ``bad`` and ``dma_stalls`` stay zero.

The frame envelope is ADR-0030: magic ``0xA5 0x5A``, a twelve-byte header and the
``CRC-16/CCITT-FALSE`` of header bytes 0..10 plus the payload. The probe deliberately does not
acknowledge events; the daemon does that, and the ticket-04 probe only observes.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Final

MAGIC: Final = b"\xA5\x5A"
HEADER_LEN: Final = 12

CLASS_STREAM: Final = 0
CLASS_COMMAND: Final = 1
CLASS_EVENT: Final = 2
CLASS_CONTROL: Final = 3

FLAG_ACK_REQUESTED: Final = 0x01
FLAG_IS_ACK: Final = 0x02
FLAG_RETRY: Final = 0x04

CONTROL_HELLO: Final = 1
CONTROL_PING: Final = 2
CONTROL_THROTTLE: Final = 3
CONTROL_BRIDGE_CONTROL: Final = 4

# A loopback run only has to get the HELLO out; the jumper echo is ignored by the router.
LOOPBACK_GRACE: Final = 1.0

CAPS_THROTTLE_SUPPORT: Final = 0x0000_0008

STREAM_NAMES: Final = {
    0x0001: "POWER",
    0x0002: "MOTION",
    0x0003: "ACTIVITY",
    0x0004: "PERCEPTION",
    0x0005: "LIDAR_CLOUD",
    0x0006: "CALIBRATION",
}

EVENT_NAMES: Final = {
    0x0001: "OBSTACLE",
    0x0002: "FLOOR_DROP",
    0x0003: "PROCEDURE_COMPLETED",
    0x0004: "DRIVE_DISTANCE_COMPLETED",
    0x0005: "ROTATE_COMPLETED",
    0x0006: "CALIBRATION_LOADED",
    0x0007: "EVENT_OVERFLOW",
}

COMMAND_NAMES: Final = {
    0x0001: "START_PROCEDURE",
    0x0002: "STOP_PROCEDURE",
    0x0003: "DRIVE_DISTANCE",
    0x0004: "ROTATE_BY",
    0x0005: "STOP_MOTION",
    0x0006: "GET_CATALOGUE",
    0x0007: "GET_STATUS",
}

STATUS_NAMES: Final = {
    0: "OK",
    1: "REJECTED",
    2: "UNKNOWN_ID",
    3: "BAD_ARGUMENT",
    4: "BUSY",
    5: "VERSION_MISMATCH",
    6: "BAD_LENGTH",
    7: "BAD_FLAGS",
    8: "STALE",
    9: "INTERNAL",
}

THROTTLE_ACTIONS: Final = {
    0: "SET_RATE",
    1: "PAUSE",
    2: "RESUME",
}

BRIDGE_CONTROL_ACTIONS: Final = {
    0: "FORGET_NETWORK",
    1: "SHOW_PAIRING_CODE",
    2: "ROTATE_PAIRING_CODE",
}

RATE_FLAG_ENABLED: Final = 0x01


@dataclass(frozen=True)
class Frame:
    """One decoded Link Protocol frame, with a borrowed-free payload copy."""

    major: int
    class_: int
    flags: int
    frame_id: int
    seq: int
    payload: bytes


def crc16_ccitt_false(data: bytes) -> int:
    """Return the CRC-16/CCITT-FALSE of ``data`` (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(
    major: int,
    class_: int,
    flags: int,
    frame_id: int,
    seq: int,
    payload: bytes,
) -> bytes:
    """Encode one frame in the ADR-0030 envelope."""
    header = bytearray()
    header += MAGIC
    header.append(major)
    header.append((class_ << 6) | (flags & 0x3F))
    header += struct.pack("<HHH", frame_id, seq, len(payload))
    header += struct.pack("<H", crc16_ccitt_false(bytes(header) + payload))
    return bytes(header) + payload


def hello_payload(epoch: int, rate_chz: int) -> bytes:
    """Build a ``HELLO`` request: versions, caps, epoch and one enabled rate per stream."""
    payload = bytearray()
    payload += struct.pack("<BBII", 1, 0, CAPS_THROTTLE_SUPPORT, epoch)
    for stream_id in STREAM_NAMES:
        payload += struct.pack("<HHB", stream_id, rate_chz, RATE_FLAG_ENABLED)
    return bytes(payload)


def parse_frames(buffer: bytearray) -> list[Frame]:
    """Extract every complete, valid frame from ``buffer``, resynchronising on damage."""
    frames: list[Frame] = []
    while len(buffer) >= HEADER_LEN:
        if buffer[0:2] != MAGIC:
            del buffer[0]
            continue

        major, class_byte, frame_id, seq, length = struct.unpack_from("<BBHHH", buffer, 2)
        if length > 1024:
            del buffer[0]
            continue

        total = HEADER_LEN + length
        if len(buffer) < total:
            break

        expected = struct.unpack_from("<H", buffer, 10)[0]
        covered = bytes(buffer[0:10]) + bytes(buffer[HEADER_LEN:total])
        if crc16_ccitt_false(covered) != expected:
            del buffer[0]
            continue

        frames.append(
            Frame(
                major=major,
                class_=class_byte >> 6,
                flags=class_byte & 0x3F,
                frame_id=frame_id,
                seq=seq,
                payload=bytes(buffer[HEADER_LEN:total]),
            )
        )
        del buffer[:total]
    return frames


def flag_names(flags: int) -> str:
    """Render the flag bits as a compact list."""
    names = []
    if flags & FLAG_ACK_REQUESTED:
        names.append("ACK_REQUESTED")
    if flags & FLAG_IS_ACK:
        names.append("IS_ACK")
    if flags & FLAG_RETRY:
        names.append("RETRY")
    return ",".join(names) if names else "-"


def format_rates(payload: bytes, offset: int) -> str:
    """Render the rate entries at ``offset`` as ``STREAM:RATE/FLAG`` triples."""
    entries = []
    index = offset
    while index + 5 <= len(payload):
        stream_id, rate_chz, flags = struct.unpack_from("<HHB", payload, index)
        name = STREAM_NAMES.get(stream_id, f"0x{stream_id:04x}")
        state = "enabled" if flags & RATE_FLAG_ENABLED else "disabled"
        entries.append(f"{name}:{rate_chz / 100:g}Hz/{state}")
        index += 5
    return " ".join(entries) if entries else "-"


def payload_preview(payload: bytes, limit: int = 16) -> str:
    """Render a short hexadecimal payload preview."""
    if len(payload) <= limit:
        return payload.hex()
    return payload[:limit].hex() + "..."


def describe(frame: Frame) -> str:
    """Render one frame as a single human-readable line."""
    prefix = (
        f"class={frame.class_} id=0x{frame.frame_id:04x} seq={frame.seq} "
        f"flags={flag_names(frame.flags)} len={len(frame.payload)}"
    )

    if frame.class_ == CLASS_STREAM:
        name = STREAM_NAMES.get(frame.frame_id, f"0x{frame.frame_id:04x}")
        return f"stream {name} {prefix} payload={payload_preview(frame.payload)}"

    if frame.class_ == CLASS_EVENT:
        name = EVENT_NAMES.get(frame.frame_id, f"0x{frame.frame_id:04x}")
        return f"event {name} {prefix} payload={payload_preview(frame.payload)}"

    if frame.class_ == CLASS_COMMAND:
        name = COMMAND_NAMES.get(frame.frame_id, f"0x{frame.frame_id:04x}")
        return f"command {name} {prefix} payload={payload_preview(frame.payload)}"

    if frame.class_ != CLASS_CONTROL:
        return f"unknown-class {frame.class_} {prefix}"

    is_ack = bool(frame.flags & FLAG_IS_ACK)
    payload = frame.payload

    if frame.frame_id == CONTROL_HELLO:
        if is_ack:
            return describe_hello_ack(prefix, payload)
        if len(payload) >= 10:
            major, minor, caps, epoch = struct.unpack_from("<BBII", payload, 0)
            rates = format_rates(payload, 10)
            return (
                f"control HELLO {prefix} major={major} minor={minor} "
                f"caps=0x{caps:08x} epoch={epoch} rates={rates}"
            )
        return f"control HELLO {prefix} payload={payload_preview(payload)}"

    if frame.frame_id == CONTROL_PING:
        if is_ack and payload:
            status = STATUS_NAMES.get(payload[0], str(payload[0]))
            return f"control PING_ACK {prefix} status={status}"
        return f"control PING {prefix}"

    if frame.frame_id == CONTROL_THROTTLE:
        if len(payload) >= 7:
            stream_id, action, rate_chz, ttl_ms = struct.unpack_from("<HBHH", payload, 0)
            stream = "ALL" if stream_id == 0xFFFF else STREAM_NAMES.get(stream_id, f"0x{stream_id:04x}")
            action_name = THROTTLE_ACTIONS.get(action, str(action))
            return (
                f"control THROTTLE {prefix} stream={stream} action={action_name} "
                f"rate={rate_chz / 100:g}Hz ttl={ttl_ms}ms"
            )
        return f"control THROTTLE {prefix} payload={payload_preview(payload)}"

    if frame.frame_id == CONTROL_BRIDGE_CONTROL:
        if is_ack:
            return f"control BRIDGE_CONTROL_ACK {prefix} payload={payload_preview(payload)}"
        if payload:
            action = BRIDGE_CONTROL_ACTIONS.get(payload[0], str(payload[0]))
            return f"control BRIDGE_CONTROL {prefix} action={action}"
        return f"control BRIDGE_CONTROL {prefix}"

    return f"control 0x{frame.frame_id:04x} {prefix} payload={payload_preview(payload)}"


def describe_hello_ack(prefix: str, payload: bytes) -> str:
    """Render a ``HELLO_ACK`` payload."""
    if len(payload) < 11:
        return f"control HELLO_ACK {prefix} payload={payload_preview(payload)}"
    major, minor, caps, boot_id = struct.unpack_from("<BBII", payload, 0)
    status = STATUS_NAMES.get(payload[10], str(payload[10]))
    rates = format_rates(payload, 11)
    return (
        f"control HELLO_ACK {prefix} major={major} minor={minor} "
        f"caps=0x{caps:08x} boot=0x{boot_id:08x} status={status} rates={rates}"
    )


def run(args: argparse.Namespace) -> int:
    """Connect, send ``HELLO``, print frames and return the process exit status."""
    hello = build_frame(
        1,
        CLASS_CONTROL,
        0,
        CONTROL_HELLO,
        1,
        hello_payload(args.epoch, args.rate),
    )

    try:
        sock = socket.create_connection((args.host, args.port), timeout=args.timeout)
    except OSError as error:
        print(f"link-probe: connect failed: {error}", file=sys.stderr)
        return 1

    started = time.monotonic()
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # Short socket timeouts keep the duration and idle checks responsive.
        sock.settimeout(min(0.5, args.timeout))
        try:
            sock.sendall(hello)
        except OSError as error:
            print(f"link-probe: send failed: {error}", file=sys.stderr)
            return 1
        print(
            f"link-probe: HELLO sent to {args.host}:{args.port} "
            f"(epoch={args.epoch}, rate={args.rate / 100:g}Hz)",
            flush=True,
        )

        buffer = bytearray()
        last_activity = started
        frames = 0
        hello_ack = False
        loopback_echo = False

        while True:
            now = time.monotonic()
            if args.loopback and now - started >= LOOPBACK_GRACE:
                break
            if args.duration > 0 and now - started >= args.duration:
                break
            if args.count > 0 and frames >= args.count:
                break
            if now - last_activity >= args.timeout:
                break

            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            except OSError as error:
                print(f"link-probe: receive failed: {error}", file=sys.stderr)
                break

            if not chunk:
                print("link-probe: peer closed the connection", flush=True)
                break

            last_activity = time.monotonic()
            buffer.extend(chunk)
            for frame in parse_frames(buffer):
                frames += 1
                if (
                    frame.class_ == CLASS_CONTROL
                    and frame.frame_id == CONTROL_HELLO
                    and frame.flags & FLAG_IS_ACK
                ):
                    hello_ack = True
                if (
                    frame.class_ == CLASS_CONTROL
                    and frame.frame_id == CONTROL_HELLO
                    and not frame.flags & FLAG_IS_ACK
                ):
                    loopback_echo = True
                if not args.quiet:
                    print(describe(frame), flush=True)
                if args.count > 0 and frames >= args.count:
                    break
    finally:
        sock.close()

    elapsed = time.monotonic() - started
    loopback = f" loopback={'echo' if loopback_echo else 'sent'}" if args.loopback else ""
    print(
        f"link-probe: done after {elapsed:.1f}s frames={frames} "
        f"hello_ack={'yes' if hello_ack else 'no'}{loopback}",
        flush=True,
    )

    if args.loopback:
        print(
            "link-probe: loopback HELLO sent; watch the Bridge status line "
            "(tcp_rx, uart_tx, uart_rx and good must rise; crc, resync, bad, dma_stalls stay 0)",
            flush=True,
        )
        return 0
    if not hello_ack:
        print("link-probe: timed out without a HELLO_ACK", file=sys.stderr)
        return 1
    if args.count > 0 and frames < args.count:
        print(
            f"link-probe: timed out before the requested {args.count} frames",
            file=sys.stderr,
        )
        return 1
    return 0


def main() -> int:
    """Parse the command line and run the probe."""
    parser = argparse.ArgumentParser(
        description="Send a HELLO to the Bridge Link listener and decode robot frames.",
    )
    parser.add_argument("host", help="the Bridge's station address (e.g. 192.168.1.42)")
    parser.add_argument(
        "--port",
        type=int,
        default=9000,
        help="the Link Protocol TCP port (default: 9000)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="idle timeout in seconds before giving up (default: 5)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="total run time in seconds; 0 runs until the idle timeout or count",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="stop after this many frames; 0 means no limit",
    )
    parser.add_argument(
        "--epoch",
        type=int,
        default=0,
        help="the Session epoch to advertise (default: 0)",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=1000,
        help="the Requested Rate per stream in centi-Hz (default: 1000 = 10 Hz)",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="print only the final count, not every frame",
    )
    parser.add_argument(
        "--loopback",
        action="store_true",
        help="accept the echoed HELLO as success (GPIO6↔GPIO7 jumper instead of a robot)",
    )
    args = parser.parse_args()

    if args.port < 1 or args.port > 65535:
        parser.error("--port must be between 1 and 65535")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.duration < 0:
        parser.error("--duration must not be negative")
    if args.count < 0:
        parser.error("--count must not be negative")
    if args.epoch < 0 or args.epoch > 0xFFFF_FFFF:
        parser.error("--epoch must fit a u32")
    if args.rate < 1 or args.rate > 0xFFFF:
        parser.error("--rate must be between 1 and 65535 centi-Hz")

    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
