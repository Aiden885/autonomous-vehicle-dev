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
  nohup setsid python3 tools/gaasd_scenario_panel/app.py >"$LOG_FILE" 2>&1 < /dev/null &
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

panel_sources_newer_than_process() {
  [[ -f "$PID_FILE" ]] || return 1
  find "$SCRIPT_DIR" -maxdepth 2 -type f \
    \( -name '*.py' -o -name '*.js' -o -name '*.css' -o -name '*.html' \) \
    -newer "$PID_FILE" -print -quit | grep -q .
}

restart_stale_server() {
  local pid
  local listener_pid
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$pid" ]]; then
    kill -TERM "$pid" >/dev/null 2>&1 || true
  fi

  # The PID file can outlive the Flask process after an abnormal restart.
  # Stop the actual listener so a desktop launch never keeps serving stale code.
  if is_port_ready; then
    while IFS= read -r listener_pid; do
      [[ -n "$listener_pid" ]] || continue
      kill -TERM "$listener_pid" >/dev/null 2>&1 || true
    done < <(lsof -t -nP -iTCP:"$PORT" -sTCP:LISTEN 2>/dev/null | sort -u)
  fi
  for _ in $(seq 1 40); do
    is_port_ready || break
    sleep 0.1
  done
  rm -f "$PID_FILE"

  ! is_port_ready
}

if is_port_ready && panel_sources_newer_than_process; then
  if ! restart_stale_server; then
    echo "Cannot stop stale GAASD-CARLA panel listener on ${HOST}:${PORT}." >&2
    exit 1
  fi
fi
if ! is_port_ready; then
  start_server
  wait_server
fi

if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "$URL" >/dev/null 2>&1 &
else
  echo "$URL"
fi
