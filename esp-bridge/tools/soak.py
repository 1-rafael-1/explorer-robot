#!/usr/bin/env python3
"""TCP soak client for expansion-remote ticket 09.

The spike firmware echoes bytes over TCP on port 9000. This client sends a sequence of
self-identifying frames and requires each one back unchanged, so a wedged stack shows up as a
stalled connection rather than a quiet log.

Usage:
    python3 tools/soak.py <host> [port] [duration-seconds] [frame-bytes]

Examples:
    # station interface, the address logged by the firmware
    python3 tools/soak.py 192.168.1.42 9000 600

    # setup AP: associate to explorer-spike, set the client to 192.168.4.2/24
    python3 tools/soak.py 192.168.4.1 9000 600

The firmware forces a station disconnect every two minutes after the first ten. Run this client
over the station interface to observe the reconnect; run it over the setup AP to confirm the AP
survives.
"""

import argparse
import socket
import struct
import time


MAGIC = 0x55AA55AA
HEADER = struct.Struct("<II")


def make_frame(sequence: int, size: int) -> bytes:
    """Build a frame whose payload is cheap to generate and easy to check."""
    if size < HEADER.size:
        raise ValueError("frame size is smaller than the header")
    header = HEADER.pack(MAGIC, sequence)
    return header + bytes([sequence & 0xFF]) * (size - HEADER.size)


def recv_exact(sock: socket.socket, length: int) -> bytes:
    """Read exactly `length` bytes, or raise ConnectionError when the peer closes."""
    chunks = []
    missing = length
    while missing:
        chunk = sock.recv(missing)
        if not chunk:
            raise ConnectionError("peer closed")
        chunks.append(chunk)
        missing -= len(chunk)
    return b"".join(chunks)


def run(host: str, port: int, duration: float, frame_size: int) -> int:
    started = time.monotonic()
    deadline = started + duration

    frames = 0
    reconnects = 0
    mismatches = 0
    bytes_sent = 0
    max_gap = 0.0
    last_report = started
    last_bytes = 0
    last_exchange = None

    sequence = 0
    sock = None

    def connect() -> socket.socket:
        connection = socket.create_connection((host, port), timeout=5.0)
        connection.settimeout(10.0)
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return connection

    try:
        while time.monotonic() < deadline:
            if sock is None:
                try:
                    sock = connect()
                    print(f"connected to {host}:{port}", flush=True)
                except OSError as exc:
                    print(f"connect failed: {exc}; retrying", flush=True)
                    time.sleep(1.0)
                    continue

            frame = make_frame(sequence, frame_size)
            try:
                sock.sendall(frame)
                echoed = recv_exact(sock, len(frame))
            except (OSError, ConnectionError) as exc:
                reconnects += 1
                print(f"link lost after {frames} frames: {exc}", flush=True)
                try:
                    sock.close()
                except OSError:
                    pass
                sock = None
                time.sleep(1.0)
                continue

            if echoed != frame:
                mismatches += 1
                print(f"echo mismatch at frame {sequence}", flush=True)

            now = time.monotonic()
            if last_exchange is not None:
                max_gap = max(max_gap, now - last_exchange)
            last_exchange = now

            frames += 1
            bytes_sent += len(frame)
            sequence = (sequence + 1) & 0xFFFFFFFF

            if now - last_report >= 5.0:
                elapsed = now - started
                delta = bytes_sent - last_bytes
                kbps = delta * 8 / (now - last_report) / 1000
                print(
                    f"t={elapsed:6.1f}s rate={kbps:7.1f}kb/s "
                    f"frames={frames} reconnects={reconnects} mismatches={mismatches} "
                    f"max_gap={max_gap * 1000:.0f}ms",
                    flush=True,
                )
                last_report = now
                last_bytes = bytes_sent

            # Small voluntary yield so a fast LAN does not turn the client into the bottleneck.
            time.sleep(0.001)
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    elapsed = time.monotonic() - started
    print(
        f"done: {elapsed:.1f}s frames={frames} bytes={bytes_sent} "
        f"reconnects={reconnects} mismatches={mismatches} max_gap={max_gap * 1000:.0f}ms",
        flush=True,
    )
    return 0 if mismatches == 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="TCP soak client for the C3 bridge spike")
    parser.add_argument("host", help="station address or 192.168.4.1 on the setup AP")
    parser.add_argument("port", nargs="?", type=int, default=9000)
    parser.add_argument("duration", nargs="?", type=float, default=600.0)
    parser.add_argument("frame_size", nargs="?", type=int, default=1024)
    args = parser.parse_args()

    return run(args.host, args.port, args.duration, args.frame_size)


if __name__ == "__main__":
    raise SystemExit(main())
