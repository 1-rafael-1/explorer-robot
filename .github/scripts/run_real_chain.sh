#!/usr/bin/env bash
# Run the Remote App against the real Relay and Bridge Stub.
#
# Unlike run_web_tests.sh, this page talks to the daemon's real `/ws`: the
# daemon discovers the Bridge Stub on loopback, opens the Session, and speaks
# the App Boundary the app ships. The runner builds the two binaries, starts the
# stub with scripted traffic, starts the daemon with the stub's token in a
# temporary config, waits for `GET /`, then renders the page under headless
# Chrome. No Node, npm or bundler is involved (ADR-0038, ADR-0039, ADR-0043).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
remote="$root/remote"

chrome_bin="${CHROME:-}"
if [[ -z "$chrome_bin" ]]; then
  for candidate in google-chrome google-chrome-stable chromium chromium-browser; do
    if command -v "$candidate" >/dev/null 2>&1; then
      chrome_bin="$candidate"
      break
    fi
  done
fi
if [[ -z "$chrome_bin" ]]; then
  echo "error: no Chrome or Chromium found (set CHROME)" >&2
  exit 1
fi

work="$(mktemp -d)"
config="$work/bench.toml"
stub_log="$work/bridge-stub.log"
daemon_log="$work/remote-daemon.log"
stub_pid=""
daemon_pid=""

cleanup() {
  if [[ -n "$daemon_pid" ]]; then
    kill "$daemon_pid" 2>/dev/null || true
    wait "$daemon_pid" 2>/dev/null || true
  fi
  if [[ -n "$stub_pid" ]]; then
    kill "$stub_pid" 2>/dev/null || true
    wait "$stub_pid" 2>/dev/null || true
  fi
  rm -rf "$work"
}
trap cleanup EXIT

# Surface the daemon and stub logs as annotations when the run fails; the job
# log itself is not readable without admin rights.
on_error() {
  local status=$?
  echo "::error::real-chain run failed (exit $status)"
  if [[ -f "$daemon_log" ]]; then
    tail -20 "$daemon_log" 2>/dev/null | sed 's/^/::error::daemon: /' >&2 || true
  fi
  if [[ -f "$stub_log" ]]; then
    tail -20 "$stub_log" 2>/dev/null | sed 's/^/::error::stub: /' >&2 || true
  fi
  exit "$status"
}
trap on_error ERR

echo "building remote-daemon and bridge-stub"
(
  cd "$remote"
  cargo build -p remote-daemon -p bridge-stub
)

stub_bin="$remote/target/x86_64-unknown-linux-gnu/debug/bridge-stub"
daemon_bin="$remote/target/x86_64-unknown-linux-gnu/debug/remote-daemon"
if [[ ! -x "$stub_bin" || ! -x "$daemon_bin" ]]; then
  echo "error: the built binaries are missing under $remote/target" >&2
  exit 1
fi

# The Bridge Stub's default device id is the hex of "stub01"; its default token
# is "bench-token". The daemon has no token flag, so the config store is the
# only source (ADR-0039).
cat >"$config" <<'EOF'
[devices."737475623031"]
token = "bench-token"
EOF

# Fast streams make the page's bounded waits short; the events prove the
# scripted Event lane. `--link-port 0` picks an ephemeral Link port that the
# stub advertises in its Beacon, so a busy 9000 cannot fail the run.
echo "starting bridge-stub"
"$stub_bin" \
  --beacon-addr 127.0.0.1:9001 \
  --beacon-interval 200 \
  --link-port 0 \
  --stream motion:1000 \
  --stream activity:100 \
  --event obstacle \
  --event floor_drop \
  --event procedure_completed \
  >"$stub_log" 2>&1 &
stub_pid=$!

echo "starting remote-daemon"
(
  cd "$remote"
  exec "$daemon_bin" --no-open --assets app --serve-tests --port 0 --config "$config"
) >"$daemon_log" 2>&1 &
daemon_pid=$!

url=""
for _ in $(seq 1 200); do
  url="$(grep -m1 -E '^http://[^ ]+/$' "$daemon_log" || true)"
  if [[ -n "$url" ]]; then
    break
  fi
  if ! kill -0 "$daemon_pid" 2>/dev/null; then
    echo "error: remote-daemon exited before printing its URL" >&2
    cat "$daemon_log" >&2 || true
    exit 1
  fi
  if ! kill -0 "$stub_pid" 2>/dev/null; then
    echo "error: bridge-stub exited before the daemon was ready" >&2
    cat "$stub_log" >&2 || true
    exit 1
  fi
  sleep 0.05
done
if [[ -z "$url" ]]; then
  echo "error: remote-daemon did not print its URL" >&2
  cat "$daemon_log" >&2 || true
  exit 1
fi

# Wait for the app surface and its CSP header, not just the TCP port.
ready=0
for _ in $(seq 1 200); do
  headers="$(curl -sS -D - -o /dev/null --max-time 2 "$url" || true)"
  if printf '%s' "$headers" | grep -q '^HTTP/.* 200' \
    && printf '%s' "$headers" | grep -qi '^content-security-policy:.*script-src'; then
    ready=1
    break
  fi
  sleep 0.05
done
if [[ "$ready" -ne 1 ]]; then
  echo "error: $url did not answer GET / with the CSP header" >&2
  cat "$daemon_log" >&2 || true
  exit 1
fi

# `--dump-dom` dumps at the load event, but the page finishes over a real
# WebSocket after that. `chrome_wait.py` drives Chrome's DevTools pipe and polls
# the page's `data-*` attribute until the bounded waits settle.
echo "running the real-chain page"
python3 "$root/.github/scripts/chrome_wait.py" \
  --chrome "$chrome_bin" \
  --url "$url/tests/real.html" \
  --attribute data-result \
  --expect pass \
  --timeout 90 \
  --label "real-chain page"

# The same daemon serves the real app at `/`. This is the boot assertion the
# ADR-0039 `frame-ancestors 'none'` CSP forbids the page from making in an iframe.
echo "running the app boot smoke at /"
python3 "$root/.github/scripts/chrome_wait.py" \
  --chrome "$chrome_bin" \
  --url "$url/" \
  --attribute data-boot \
  --expect ok \
  --timeout 30 \
  --label "app boot at /"
