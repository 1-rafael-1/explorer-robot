#!/usr/bin/env python3
"""The ticket-05 authentication matrix for the Bridge's `bridge-net` gate.

Runs the preamble against a live Bridge with a dependency-free client (see `bridge_preamble.py`)
and demonstrates ADR-0037's rules on the bench:

- ``valid``      the right token reaches ``ACCEPT``;
- ``wrong``      a wrong proof gets ``DENY(unauthorized)``;
- ``silent``     a connection that sends nothing is dropped at the 5 s deadline;
- ``supersede``  a new client authenticates promptly while a silent one is pending;
- ``busy``       a second client is refused while the first is authenticated;
- ``ratelimit``  failed proofs close a 60 s refusal window, which outranks a valid proof;
- ``matrix``     all of the above, in an order that respects the failure counter.

The token is the text the firmware compiled in as `BRIDGE_BENCH_TOKEN`; its UTF-8 bytes are the
HMAC key. Put it on the command line or export it; the probe never takes it as an argv flag:

    BRIDGE_BENCH_TOKEN='bench05' python3 tools/auth-probe.py 192.168.1.42
    python3 tools/auth-probe.py 192.168.1.42 --token-file ~/.config/explorer-bench-token

Exit status is 0 only when every requested check passed.

Note: the matrix's ``wrong`` check spends one of the five failures in the 60 s window, so the
``ratelimit`` check needs four more. A Bridge that was already refusing connections outranks the
ordering; reboot it (or wait out the window) for a clean matrix.
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from contextlib import closing

from bridge_preamble import (
    DENY_BUSY,
    DENY_UNAUTHORIZED,
    KIND_BRIDGE_CHALLENGE,
    KIND_CLIENT_AUTH,
    KIND_DENY,
    PROOF_LEN,
    Denied,
    PreambleError,
    encode_message,
    load_token,
    parse_deny,
    perform_preamble,
    read_message,
)

# `bridge-core::auth::REFUSAL_MS`.
REFUSAL_RETRY_MS = 60_000
# `preamble::AUTH_DEADLINE_MS`.
AUTH_DEADLINE = 5.0

# A client must not wait behind a silent, unauthenticated connection's 5 s deadline.
SUPERSEDE_LIMIT = 2.0


def connect(args: argparse.Namespace) -> socket.socket:
    """Opens one client connection with the probe's socket options."""
    sock = socket.create_connection((args.host, args.port), timeout=args.timeout)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(args.timeout)
    return sock


def send_hello(sock: socket.socket) -> tuple[int, int, bytes]:
    """Sends a CLIENT_HELLO and reads the first reply.

    A `DENY(busy)` or a refusal may be written by the Bridge before it reads anything, so a send
    failure is not fatal: the reply is still read.
    """
    try:
        sock.sendall(encode_message(1, os.urandom(32)))
    except OSError:
        pass
    return read_message(sock)


def check_valid(args: argparse.Namespace, token: bytes) -> list[str]:
    """The right token reaches ACCEPT."""
    try:
        with closing(connect(args)) as sock:
            identity = perform_preamble(sock, token)
    except (OSError, PreambleError) as error:
        return [f"the valid preamble failed: {error}"]
    print(
        f"  accepted device={identity['device_id']} name={identity['name']!r} "
        f"flags=0x{identity['flags']:02x}"
    )
    return []


def check_wrong(args: argparse.Namespace, token: bytes) -> list[str]:
    """A wrong proof is denied as unauthorized."""
    try:
        with closing(connect(args)) as sock:
            _major, kind, payload = send_hello(sock)
            if kind == KIND_DENY:
                # A stale refusal window; still an unauthorized denial, but not this check.
                return [f"expected a challenge, got a denial ({parse_deny(payload)}); is the window open?"]
            if kind != KIND_BRIDGE_CHALLENGE:
                return [f"expected BRIDGE_CHALLENGE, got message {kind}"]
            # Any 32 bytes are a wrong proof; the Bridge compares them in constant time.
            sock.sendall(encode_message(KIND_CLIENT_AUTH, bytes(PROOF_LEN)))
            _major, kind, payload = read_message(sock)
    except (OSError, PreambleError) as error:
        return [f"the wrong-proof exchange failed: {error}"]

    if kind != KIND_DENY:
        return [f"a wrong proof expected DENY, got message {kind}"]
    denied = parse_deny(payload)
    if denied.reason != DENY_UNAUTHORIZED:
        return [f"a wrong proof denied with {denied}, expected unauthorized"]
    print(f"  denied {denied}")
    return []


def check_silent(args: argparse.Namespace) -> list[str]:
    """A connection that sends nothing is dropped at the 5 s deadline."""
    started = time.monotonic()
    try:
        with closing(connect(args)) as sock:
            try:
                early = sock.recv(1)
            except (ConnectionResetError, ConnectionAbortedError):
                early = b""
            except TimeoutError:
                return [f"the silent connection was still open after {args.timeout:g}s"]
            if early:
                return ["the Bridge sent a preamble byte to a connection that sent nothing"]
    except OSError as error:
        return [f"the silent connection failed: {error}"]
    elapsed = time.monotonic() - started
    if elapsed < AUTH_DEADLINE - 1.0 or elapsed > AUTH_DEADLINE + 2.0:
        return [f"the silent connection closed after {elapsed:.1f}s, expected about {AUTH_DEADLINE:g}s"]
    print(f"  dropped after {elapsed:.1f}s")
    return []


def check_supersede(args: argparse.Namespace, token: bytes) -> list[str]:
    """A new client authenticates promptly while a silent one is pending."""
    try:
        silent = connect(args)
    except OSError as error:
        return [f"the silent connection failed: {error}"]
    with closing(silent):
        # Give the Bridge a moment to accept the silent connection into a worker.
        time.sleep(0.2)
        started = time.monotonic()
        try:
            with closing(connect(args)) as second:
                identity = perform_preamble(second, token)
        except (OSError, PreambleError) as error:
            return [f"the new client was blocked or refused: {error}"]
        elapsed = time.monotonic() - started
    if elapsed > SUPERSEDE_LIMIT:
        return [
            (
                f"the new client took {elapsed:.1f}s while a silent one was pending; "
                f"it waited behind the {AUTH_DEADLINE:g}s deadline"
            )
        ]
    print(
        f"  new client {identity['device_id']} authenticated in {elapsed:.2f}s "
        f"while a silent one was pending"
    )
    return []


def check_busy(args: argparse.Namespace, token: bytes) -> list[str]:
    """A second authenticated client is refused while the first is live."""
    try:
        holder = connect(args)
    except OSError as error:
        return [f"the first connection failed: {error}"]
    with closing(holder):
        try:
            perform_preamble(holder, token)
        except (OSError, PreambleError) as error:
            return [f"the first preamble failed: {error}"]

        try:
            with closing(connect(args)) as second:
                try:
                    perform_preamble(second, token)
                except Denied as denied:
                    if denied.reason != DENY_BUSY:
                        return [f"the second client was denied {denied}, expected busy"]
                    if denied.retry_ms != 4_000:
                        return [f"busy carried retry_ms={denied.retry_ms}, expected 4000"]
                    print(f"  second client refused: {denied}")
                    return []
                except (OSError, PreambleError) as error:
                    return [f"the second preamble failed without a busy denial: {error}"]
                return ["the second client was accepted while the first was live"]
        except OSError as error:
            return [f"the second connection failed: {error}"]


def check_rate_limit(args: argparse.Namespace, token: bytes) -> list[str]:
    """Failed proofs close a 60 s window that outranks a valid proof."""
    attempts = 0
    opened = False
    for _attempt in range(1, 6):
        attempts += 1
        try:
            with closing(connect(args)) as sock:
                _major, kind, payload = send_hello(sock)
                if kind == KIND_DENY:
                    denied = parse_deny(payload)
                    if denied.retry_ms == REFUSAL_RETRY_MS:
                        opened = True
                        break
                    return [f"a refusal without the 60 s hint: {denied}"]
                if kind != KIND_BRIDGE_CHALLENGE:
                    return [f"expected BRIDGE_CHALLENGE, got message {kind}"]
                sock.sendall(encode_message(KIND_CLIENT_AUTH, bytes(PROOF_LEN)))
                _major, kind, payload = read_message(sock)
        except TimeoutError:
            return [f"attempt {attempts} timed out instead of being denied"]
        except (OSError, PreambleError) as error:
            return [f"attempt {attempts} failed: {error}"]
        if kind != KIND_DENY:
            return [f"attempt {attempts} expected DENY, got message {kind}"]
        time.sleep(0.1)

    if not opened:
        return [f"{attempts} failed proofs did not open a refusal window"]

    try:
        with closing(connect(args)) as sock:
            try:
                perform_preamble(sock, token)
            except Denied as denied:
                if denied.reason != DENY_UNAUTHORIZED or denied.retry_ms != REFUSAL_RETRY_MS:
                    return [f"the valid proof was refused with {denied}, expected unauthorized/60000"]
                print(f"  refusal window opened after {attempts} failures: {denied}")
                return []
            except (OSError, PreambleError) as error:
                return [f"the valid proof failed without a refusal denial: {error}"]
            return ["a valid proof was accepted inside the refusal window"]
    except OSError as error:
        return [f"the post-window connection failed: {error}"]


CHECKS = {
    "valid": check_valid,
    "wrong": check_wrong,
    "silent": check_silent,
    "supersede": check_supersede,
    "busy": check_busy,
    "ratelimit": check_rate_limit,
}

MATRIX_ORDER = ("valid", "busy", "silent", "supersede", "wrong", "ratelimit")

# How long to let a closed client's gate state settle before the next check.
MODE_SETTLE = 0.5


def main() -> int:
    """Parse the command line and run the requested checks."""
    parser = argparse.ArgumentParser(
        description="Run the Bridge's bridge-net authentication matrix.",
    )
    parser.add_argument("host", help="the Bridge's station address (e.g. 192.168.1.42)")
    parser.add_argument("--port", type=int, default=9000, help="the Link TCP port (default: 9000)")
    parser.add_argument(
        "--mode",
        choices=(*CHECKS, "matrix"),
        default="matrix",
        help="the check to run (default: matrix)",
    )
    parser.add_argument(
        "--token-file",
        default=None,
        help="read the pairing token from this file instead of the environment",
    )
    parser.add_argument(
        "--token-env",
        default="BRIDGE_BENCH_TOKEN",
        help="the environment variable holding the token (default: BRIDGE_BENCH_TOKEN)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="socket timeout in seconds (default: 10, above the 5 s deadline)",
    )
    args = parser.parse_args()

    if args.port < 1 or args.port > 65535:
        parser.error("--port must be between 1 and 65535")
    if args.timeout <= AUTH_DEADLINE:
        parser.error("--timeout must exceed the 5 s authentication deadline")

    try:
        token = load_token(args.token_file, args.token_env)
    except PreambleError as error:
        parser.error(str(error))

    modes = MATRIX_ORDER if args.mode == "matrix" else (args.mode,)
    failures = 0
    for mode in modes:
        print(f"auth-probe: {mode}", flush=True)
        problems = check_silent(args) if mode == "silent" else CHECKS[mode](args, token)
        if problems:
            failures += 1
            for problem in problems:
                print(f"  FAIL {problem}", file=sys.stderr)
        else:
            print("  PASS", flush=True)
        if mode != modes[-1]:
            time.sleep(MODE_SETTLE)

    if failures:
        print(f"auth-probe: {failures} check(s) failed", file=sys.stderr)
        return 1
    print("auth-probe: all checks passed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
