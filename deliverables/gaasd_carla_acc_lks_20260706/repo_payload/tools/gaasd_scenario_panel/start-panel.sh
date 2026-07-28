#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PORT="${GAASD_PANEL_PORT:-8765}"
HOST="127.0.0.1"
URL="http://${HOST}:${PORT}"
BASE_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}"
if [ ! -d "$BASE_RUNTIME_DIR" ] || [ ! -w "$BASE_RUNTIME_DIR" ]; then
  BASE_RUNTIME_DIR="/tmp"
fi
RUNTIME_DIR="$BASE_RUNTIME_DIR/gaasd-scenario-panel"
PID_FILE="$RUNTIME_DIR/panel.pid"
LOG_FILE="$RUNTIME_DIR/panel.log"

mkdir -p "$RUNTIME_DIR"

is_port_ready() {
  python3 - "$HOST" "$PORT" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(0.5)
try:
    sock.connect((host, port))
except OSError:
    sys.exit(1)
finally:
    sock.close()
PY
}

start_server() {
  cd "$REPO_ROOT"
  setsid python3 tools/gaasd_scenario_panel/app.py >"$LOG_FILE" 2>&1 < /dev/null &
  echo "$!" > "$PID_FILE"
}

wait_server() {
  local index
  index=0
  while [ "$index" -lt 60 ]; do
    if is_port_ready; then
      return 0
    fi
    sleep 0.2
    index=$((index + 1))
  done
  echo "GAASD-CARLA Scenario Panel did not start within 12s. Log: $LOG_FILE" >&2
  return 1
}

if ! is_port_ready; then
  start_server
  wait_server
fi

if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "$URL" >/dev/null 2>&1 &
else
  echo "$URL"
fi
