#!/usr/bin/env python3
"""Serve the Remote App folder on an ephemeral loopback port.

The headless browser suite imports ES modules, which `file://` blocks, so the
pages are served same-origin over HTTP. The port is printed on stdout for the
shell runner to read; requests are not logged.
"""

import http.server
import socketserver
import sys
from functools import partial


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, _format, *_args):
        """Suppress the per-request log line."""


def main() -> int:
    root = sys.argv[1] if len(sys.argv) > 1 else "remote/app"
    handler = partial(QuietHandler, directory=root)
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("127.0.0.1", 0), handler) as server:
        print(server.server_address[1], flush=True)
        server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
