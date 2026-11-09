#!/usr/bin/env bash
# Run the Remote App's headless browser suite.
#
# Every `remote/app/tests/*.html` page reports `data-result="pass"` on success.
# The pages use ES modules, so a small Python server serves the app folder
# same-origin; Chrome only has to render and dump the DOM. No Node, npm or
# bundler is involved (ADR-0038, ADR-0043).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
app="$root/remote/app"

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

port_file="$(mktemp)"
server_log="$(mktemp)"
python3 "$root/.github/scripts/serve_app.py" "$app" >"$port_file" 2>"$server_log" &
server_pid=$!
cleanup() {
  kill "$server_pid" 2>/dev/null || true
  wait "$server_pid" 2>/dev/null || true
  rm -f "$port_file" "$server_log"
}
trap cleanup EXIT

for _ in $(seq 1 100); do
  [[ -s "$port_file" ]] && break
  sleep 0.1
done
if [[ ! -s "$port_file" ]]; then
  echo "error: the static server did not start" >&2
  cat "$server_log" >&2 || true
  exit 1
fi
port="$(head -n 1 "$port_file")"
base="http://127.0.0.1:${port}"

shopt -s nullglob
pages=("$app"/tests/*.html)
if [[ ${#pages[@]} -eq 0 ]]; then
  echo "error: no test pages under remote/app/tests" >&2
  exit 1
fi

# `WEB_TESTS_FILTER` narrows the run to pages whose name contains the value,
# so a ticket can run its own suite while other pages are being written.
filter="${WEB_TESTS_FILTER:-}"
ran=0

status=0

# The app itself must boot all of its modules; a load-time error leaves
# `data-boot` at `loading` or flips it to `error`.
echo "running app smoke"
if "$chrome_bin" \
  --headless=new \
  --no-sandbox \
  --disable-gpu \
  --disable-dev-shm-usage \
  --hide-scrollbars \
  --virtual-time-budget=3000 \
  --dump-dom "$base/" 2>/dev/null | grep -q 'data-boot="ok"'; then
  echo "  PASS app smoke"
else
  echo "  FAIL app smoke" >&2
  status=1
fi

for page in "${pages[@]}"; do
  name="$(basename "$page")"
  if [[ -n "$filter" && "$name" != *"$filter"* ]]; then
    continue
  fi
  ran=$((ran + 1))
  url="${base}/tests/${name}"
  echo "running ${name}"
  output="$(
    "$chrome_bin" \
      --headless=new \
      --no-sandbox \
      --disable-gpu \
      --disable-dev-shm-usage \
      --hide-scrollbars \
      --virtual-time-budget=20000 \
      --dump-dom "$url" 2>/dev/null || true
  )"
  if printf '%s' "$output" | grep -q 'data-result="pass"'; then
    echo "  PASS ${name}"
  else
    echo "  FAIL ${name}" >&2
    printf '%s\n' "$output" | sed -n '1,60p' >&2
    status=1
  fi
done

if [[ $ran -eq 0 ]]; then
  echo "error: no test page matched ${filter}" >&2
  exit 1
fi

exit "$status"
