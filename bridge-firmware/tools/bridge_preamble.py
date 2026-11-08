"""The `bridge-net` `EXPH` preamble, for the dependency-free bench probes.

The Rust Bridge and daemon compose `bridge-net`, so neither implements this exchange itself. The
Python bench tools cannot link the crate, so this module mirrors the frozen wire contract for
ticket 05's bench matrix only: 8-byte framing (`EXPH`, major, type, little-endian payload length),
32-byte nonces, HMAC-SHA256 proofs with the crate's domain-separated labels, and the identity the
challenge serves. Keep it in step with `remote/bridge-net/src/preamble.rs`.

The token is always the **text's UTF-8 bytes**, matching the firmware's `str::as_bytes` and the
daemon's config string. The tools read it from a file or the environment, never from argv, so it
does not land in shell history.
"""

from __future__ import annotations

import hashlib
import hmac
import os
import socket
import struct
from typing import Final

MAGIC: Final = b"EXPH"
MAJOR: Final = 1

KIND_CLIENT_HELLO: Final = 1
KIND_BRIDGE_CHALLENGE: Final = 2
KIND_CLIENT_AUTH: Final = 3
KIND_ACCEPT: Final = 4
KIND_DENY: Final = 5

DENY_UNAUTHORIZED: Final = 0
DENY_BUSY: Final = 1
DENY_SETUP_MODE: Final = 2
DENY_VERSION: Final = 3

DENY_NAMES: Final = {
    DENY_UNAUTHORIZED: "unauthorized",
    DENY_BUSY: "busy",
    DENY_SETUP_MODE: "setup_mode",
    DENY_VERSION: "version",
}

HEADER_LEN: Final = 8
NONCE_LEN: Final = 32
PROOF_LEN: Final = 32
IDENTITY_LEN: Final = 6 + 1 + 16

BRIDGE_PROOF_LABEL: Final = b"explorer-link-v1 bridge\x00"
CLIENT_PROOF_LABEL: Final = b"explorer-link-v1 client\x00"


class PreambleError(Exception):
    """The preamble did not reach `ACCEPT`."""


class Denied(PreambleError):
    """The Bridge sent a `DENY`, with its reason and optional retry hint."""

    def __init__(self, reason: int, retry_ms: int | None) -> None:
        name = DENY_NAMES.get(reason, f"reason {reason}")
        suffix = "" if retry_ms is None else f", retry_ms={retry_ms}"
        super().__init__(f"denied: {name}{suffix}")
        self.reason = reason
        self.retry_ms = retry_ms


def load_token(path: str | None = None, env: str = "BRIDGE_BENCH_TOKEN") -> bytes:
    """Returns the token text's UTF-8 bytes from `path` or the environment.

    Only trailing newlines are removed, so the key is exactly the text the daemon stores and the
    firmware compiled in. An unset or empty token is an error, never an empty HMAC key.
    """
    if path is not None:
        try:
            with open(path, encoding="utf-8") as source:
                token = source.read().rstrip("\r\n")
        except OSError as error:
            raise PreambleError(f"the token file could not be read: {error}") from error
    else:
        token = os.environ.get(env, "")

    if not token:
        where = path if path is not None else f"${env}"
        raise PreambleError(
            f"no token at {where}; set it on the command line "
            f"(e.g. {env}='...' python3 ...), export it, or pass --token-file"
        )
    if "\x00" in token:
        raise PreambleError("the token contains a NUL byte; it cannot be a C string key")
    return token.encode("utf-8")


def encode_message(kind: int, payload: bytes = b"") -> bytes:
    """Frames one preamble message."""
    return MAGIC + bytes((MAJOR, kind)) + struct.pack("<H", len(payload)) + payload


def recv_exact(sock: socket.socket, length: int) -> bytes:
    """Reads exactly `length` bytes or raises."""
    data = bytearray()
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk:
            raise PreambleError("the peer closed mid-preamble")
        data.extend(chunk)
    return bytes(data)


def read_message(sock: socket.socket) -> tuple[int, int, bytes]:
    """Reads one framed message as `(major, kind, payload)`."""
    header = recv_exact(sock, HEADER_LEN)
    if header[:4] != MAGIC:
        raise PreambleError("the preamble magic is not EXPH")
    length = struct.unpack_from("<H", header, 6)[0]
    if length > 88:
        raise PreambleError(f"the preamble payload is too long ({length})")
    payload = recv_exact(sock, length) if length else b""
    return header[4], header[5], payload


def bridge_proof(token: bytes, client_nonce: bytes, bridge_nonce: bytes, identity: bytes) -> bytes:
    """HMAC-SHA256 over the Bridge's label, both nonces and the served identity."""
    message = BRIDGE_PROOF_LABEL + client_nonce + bridge_nonce + identity
    return hmac.new(token, message, hashlib.sha256).digest()


def client_proof(token: bytes, client_nonce: bytes, bridge_nonce: bytes) -> bytes:
    """HMAC-SHA256 over the client's label and both nonces."""
    message = CLIENT_PROOF_LABEL + client_nonce + bridge_nonce
    return hmac.new(token, message, hashlib.sha256).digest()


def decode_identity(identity: bytes) -> dict[str, object]:
    """Decodes the challenge's identity field."""
    if len(identity) != IDENTITY_LEN:
        raise PreambleError(f"the identity is {len(identity)} bytes, expected {IDENTITY_LEN}")
    device_id = identity[:6]
    name = identity[7:].rstrip(b"\x00").decode("utf-8", errors="replace")
    return {
        "device_id": device_id.hex(),
        "flags": identity[6],
        "name": name,
    }


def perform_preamble(sock: socket.socket, token: bytes) -> dict[str, object]:
    """Runs the client side of the preamble and returns the Bridge's served identity.

    The Bridge's proof is verified **before** `CLIENT_AUTH` is written, exactly as the daemon's
    `handshake` does: a Beacon spoofer can make the probe connect, but cannot make it reveal a
    token-derived byte.
    """
    client_nonce = os.urandom(NONCE_LEN)
    sock.sendall(encode_message(KIND_CLIENT_HELLO, client_nonce))

    major, kind, payload = read_message(sock)
    if kind == KIND_DENY:
        raise parse_deny(payload)
    if major != MAJOR:
        raise Denied(DENY_VERSION, None)
    if kind != KIND_BRIDGE_CHALLENGE:
        raise PreambleError(f"expected BRIDGE_CHALLENGE, got message {kind}")
    if len(payload) != PROOF_LEN + NONCE_LEN + IDENTITY_LEN:
        raise PreambleError(f"the challenge is {len(payload)} bytes, expected 87")

    served_proof = payload[:PROOF_LEN]
    bridge_nonce = payload[PROOF_LEN : PROOF_LEN + NONCE_LEN]
    identity = payload[PROOF_LEN + NONCE_LEN :]

    expected = bridge_proof(token, client_nonce, bridge_nonce, identity)
    if not hmac.compare_digest(expected, served_proof):
        raise PreambleError("the Bridge did not prove the pairing token")

    sock.sendall(encode_message(KIND_CLIENT_AUTH, client_proof(token, client_nonce, bridge_nonce)))

    _major, kind, payload = read_message(sock)
    if kind == KIND_DENY:
        raise parse_deny(payload)
    if kind != KIND_ACCEPT:
        raise PreambleError(f"expected ACCEPT, got message {kind}")
    return decode_identity(identity)


def parse_deny(payload: bytes) -> Denied:
    """Decodes a `DENY` payload."""
    if len(payload) not in (1, 5):
        raise PreambleError(f"the DENY payload is {len(payload)} bytes")
    reason = payload[0]
    retry_ms = struct.unpack_from("<I", payload, 1)[0] if len(payload) == 5 else None
    return Denied(reason, retry_ms)
