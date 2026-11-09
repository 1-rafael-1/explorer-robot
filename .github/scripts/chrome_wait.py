#!/usr/bin/env python3
"""Poll a headless Chrome page over the DevTools pipe until an attribute settles.

`chrome --headless --dump-dom` dumps at the document's load event, but the
real-chain page finishes asynchronously over a real WebSocket: the module starts,
the awaited WebSocket traffic arrives, and only then does the page flip
`data-result` to `pass` or `fail`. A top-level `await` does not hold Chrome 157's
dump. This driver starts Chrome with `--remote-debugging-pipe` (no port, no
third-party Python package), attaches to the page target, and polls one
`data-*` attribute with `Runtime.evaluate`. It prints the page's `#results`
text and title on failure.

The pipe protocol is Chrome's: the browser reads NUL-terminated JSON commands
on file descriptor 3 and writes responses and events on descriptor 4.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time


class DevToolsPipe:
    """A minimal DevTools client over Chrome's `--remote-debugging-pipe`."""

    def __init__(self, write_fd: int, read_fd: int) -> None:
        self._write_fd = write_fd
        self._read_fd = read_fd
        self._buffer = b""
        self._messages: queue.Queue[dict | None] = queue.Queue()
        self._next_id = 0
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        while True:
            try:
                chunk = os.read(self._read_fd, 65536)
            except OSError:
                self._messages.put(None)
                return
            if not chunk:
                self._messages.put(None)
                return
            self._buffer += chunk
            while b"\0" in self._buffer:
                raw, self._buffer = self._buffer.split(b"\0", 1)
                if not raw:
                    continue
                try:
                    self._messages.put(json.loads(raw.decode("utf-8")))
                except json.JSONDecodeError:
                    continue

    def send(self, message: dict) -> None:
        os.write(self._write_fd, json.dumps(message).encode("utf-8") + b"\0")

    def call(
        self,
        method: str,
        params: dict | None = None,
        session_id: str | None = None,
        timeout: float = 10.0,
    ) -> dict:
        self._next_id += 1
        message_id = self._next_id
        message = {"id": message_id, "method": method, "params": params or {}}
        if session_id is not None:
            message["sessionId"] = session_id
        self.send(message)

        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(f"{method} timed out")
            try:
                item = self._messages.get(timeout=remaining)
            except queue.Empty:
                raise RuntimeError(f"{method} timed out") from None
            if item is None:
                raise RuntimeError("Chrome closed the DevTools pipe")
            if item.get("id") != message_id:
                continue
            if "error" in item:
                raise RuntimeError(f"{method}: {item['error']}")
            return item.get("result", {})

    def evaluate(self, session_id: str, expression: str) -> object:
        response = self.call(
            "Runtime.evaluate",
            {"expression": expression, "returnByValue": True},
            session_id,
        )
        # A page exception must fail the run immediately, not look like an
        # attribute that never settles. `Runtime.evaluate` reports it beside
        # the result, so surface the description with the expression.
        exception = response.get("exceptionDetails")
        if exception is not None:
            description = (
                (exception.get("exception") or {}).get("description")
                or exception.get("text")
                or "unknown page exception"
            )
            raise RuntimeError(f"page exception evaluating {expression!r}: {description}")
        return response.get("result", {}).get("value")


def start_chrome(chrome: str, user_data_dir: str) -> tuple[subprocess.Popen, int, int]:
    """Starts Chrome and returns it with the parent's write and read fds.

    Chrome reads commands on descriptor 3 and writes responses on descriptor 4.
    The descriptors are set up in the parent before `Popen`, so `pass_fds` keeps
    exactly the two child ends open through `exec` (a `preexec_fn` runs too late
    for `close_fds` on this platform).
    """
    command_read, command_write = os.pipe()
    response_read, response_write = os.pipe()
    # Keep the parent's ends on private descriptors first: the two child ends are
    # about to take 3 and 4, which `os.dup2` may overwrite in this process too.
    parent_write = os.dup(command_write)
    parent_read = os.dup(response_read)
    os.dup2(command_read, 3)
    os.dup2(response_write, 4)
    for fd in (command_read, command_write, response_read, response_write):
        if fd not in (3, 4):
            os.close(fd)

    process = subprocess.Popen(  # noqa: S603 - the caller supplies the browser
        [
            chrome,
            "--headless=new",
            "--no-sandbox",
            "--disable-gpu",
            "--disable-dev-shm-usage",
            "--remote-debugging-pipe",
            f"--user-data-dir={user_data_dir}",
            "about:blank",
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        pass_fds=(3, 4),
        close_fds=True,
        start_new_session=True,
    )
    os.close(3)
    os.close(4)
    return process, parent_write, parent_read


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chrome", required=True, help="the Chrome or Chromium binary")
    parser.add_argument("--url", required=True, help="the page to load")
    parser.add_argument("--attribute", default="data-result", help="the data-* attribute to poll")
    parser.add_argument("--expect", default="pass", help="the value that means success")
    parser.add_argument("--timeout", type=float, default=60.0, help="seconds before failing")
    parser.add_argument("--label", default="page", help="the name printed on failure")
    args = parser.parse_args()

    attribute = args.attribute
    if not attribute.startswith("data-"):
        parser.error("--attribute must start with data-")
    key = attribute[len("data-") :]

    with tempfile.TemporaryDirectory(
        prefix="chrome-wait-", ignore_cleanup_errors=True
    ) as user_data_dir:
        process, write_fd, read_fd = start_chrome(args.chrome, user_data_dir)
        transport = DevToolsPipe(write_fd, read_fd)
        target_id = None
        try:
            target_id = transport.call("Target.createTarget", {"url": args.url})["targetId"]
            session_id = transport.call(
                "Target.attachToTarget",
                {"targetId": target_id, "flatten": True},
            )["sessionId"]
            transport.call("Runtime.enable", {}, session_id)

            expression = f"document.documentElement.dataset[{json.dumps(key)}] ?? null"
            deadline = time.monotonic() + args.timeout
            value = None
            while time.monotonic() < deadline:
                value = transport.evaluate(session_id, expression)
                if value == args.expect:
                    print(f"  PASS {args.label}")
                    return 0
                if value == "fail":
                    break
                time.sleep(0.1)

            title = transport.evaluate(session_id, "document.title")
            status = transport.evaluate(session_id, "document.getElementById('status')?.textContent ?? ''")
            results = transport.evaluate(
                session_id, "document.getElementById('results')?.textContent ?? ''"
            )
            print(f"  FAIL {args.label} ({attribute}={value!r}, title={title!r})", file=sys.stderr)
            if status:
                print(f"  status: {status}", file=sys.stderr)
            if results:
                print(results, file=sys.stderr)
            return 1
        finally:
            if target_id is not None:
                try:
                    transport.call("Target.closeTarget", {"targetId": target_id}, timeout=2.0)
                except (RuntimeError, OSError):
                    pass
            # Chrome spawns helper processes that may still write under the
            # user-data dir. The process group lets the driver stop all of them
            # before the temporary directory is removed.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
