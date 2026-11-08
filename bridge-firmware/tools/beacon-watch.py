#!/usr/bin/env python3
"""Watch the Bridge's Discovery Beacon on UDP 9001.

The Bridge broadcasts one `bridge-net` Beacon every 3 s on its station interface; this listener
decodes the datagram with no dependencies so the ticket-05 flags can be read on the bench:

    python3 tools/beacon-watch.py --count 3
    python3 tools/beacon-watch.py --duration 15

`link_ready` is true only for 5 s after a valid robot frame, so run a `link-probe.py --loopback`
(GPIO6↔GPIO7 jumper) while this listener is watching and the flag should flip for one or two
Beacons. Exit status is 0 when at least one Beacon was decoded.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from typing import Final

MAGIC: Final = b"EXPR"
ENCODED_LEN: Final = 4 + 1 + 6 + 16 + 2 + 1

FLAG_PROVISIONED: Final = 0x01
FLAG_LINK_READY: Final = 0x02
FLAG_SETUP_MODE: Final = 0x04

BEACON_PORT: Final = 9001


def decode(datagram: bytes) -> tuple[str, int, str, str, int, int] | None:
    """Decodes one Beacon as `(device_id, major, name, flags_text, flags, tcp_port)`."""
    if len(datagram) < ENCODED_LEN or datagram[:4] != MAGIC:
        return None
    major = datagram[4]
    device_id = datagram[5:11].hex()
    name = datagram[11:27].rstrip(b"\x00").decode("utf-8", errors="replace")
    tcp_port = struct.unpack_from("<H", datagram, 27)[0]
    flags = datagram[29]
    names = [
        label
        for bit, label in (
            (FLAG_PROVISIONED, "provisioned"),
            (FLAG_LINK_READY, "link-ready"),
            (FLAG_SETUP_MODE, "setup_mode"),
        )
        if flags & bit
    ]
    flags_text = ",".join(names) if names else "-"
    return device_id, major, name, flags_text, flags, tcp_port


def main() -> int:
    """Parse the command line and print decoded Beacons."""
    parser = argparse.ArgumentParser(description="Watch the Bridge's Discovery Beacon.")
    parser.add_argument("--bind", default="0.0.0.0", help="the local address (default: 0.0.0.0)")
    parser.add_argument(
        "--port",
        type=int,
        default=BEACON_PORT,
        help=f"the UDP listen port (default: {BEACON_PORT})",
    )
    parser.add_argument("--count", type=int, default=0, help="stop after this many Beacons")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="stop after this many seconds; 0 waits for --count or forever",
    )
    args = parser.parse_args()

    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.count < 0:
        parser.error("--count must not be negative")
    if args.duration < 0:
        parser.error("--duration must not be negative")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((args.bind, args.port))
    except OSError as error:
        print(f"beacon-watch: bind failed: {error}", file=sys.stderr)
        return 1

    started = time.monotonic()
    seen = 0
    while args.count == 0 or seen < args.count:
        if args.duration > 0 and time.monotonic() - started >= args.duration:
            break
        sock.settimeout(0.5 if args.duration > 0 else None)
        try:
            datagram, source = sock.recvfrom(256)
        except socket.timeout:
            continue
        except OSError as error:
            print(f"beacon-watch: receive failed: {error}", file=sys.stderr)
            return 1

        decoded = decode(datagram)
        if decoded is None:
            print(f"beacon-watch: ignored {len(datagram)} undecodable bytes from {source[0]}")
            continue
        device_id, major, name, flags_text, flags, tcp_port = decoded
        seen += 1
        print(
            f"beacon from={source[0]} device={device_id} name={name!r} major={major} "
            f"tcp_port={tcp_port} flags=0x{flags:02x} ({flags_text})",
            flush=True,
        )

    if seen == 0:
        print(
            "beacon-watch: no Beacon heard; check that the Bridge has a station address "
            "(sta: address=...) and that UDP 9001 is not blocked",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
