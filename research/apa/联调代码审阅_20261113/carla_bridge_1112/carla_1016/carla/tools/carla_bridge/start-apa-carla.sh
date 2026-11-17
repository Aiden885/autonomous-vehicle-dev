#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${CARLA_ROOT:-}" ]; then
  if [ -x "/home/nl/carla/CarlaUE4.sh" ]; then
    CARLA_ROOT="/home/nl/carla"
  else
    CARLA_ROOT="/opt/carla"
  fi
fi
CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2000}"
PYTHON_BIN="${PYTHON_BIN:-python3.8}"
CONFIG_PATH="${CONFIG_PATH:-${SCRIPT_DIR}/../../scenarios/apa/bridge_config.json}"
LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/apa-pangu-carla/carla}"
WAIT_CARLA_SEC="${WAIT_CARLA_SEC:-90}"
WAIT_BRIDGE_SEC="${WAIT_BRIDGE_SEC:-60}"
CARLA_EXTRA_ARGS="${CARLA_EXTRA_ARGS:--windowed -Resx=1280 -Resy=720 -fps=20 -nosound}"
CARLA_FORCE_NVIDIA_VULKAN="${CARLA_FORCE_NVIDIA_VULKAN:-1}"
RESTART_BRIDGE="${RESTART_BRIDGE:-1}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

log() {
  printf '[APA-CARLA] %s\n' "$*"
}

fail() {
  printf '[APA-CARLA] ERROR: %s\n' "$*" >&2
  exit 1
}

tcp_open() {
  local host="$1"
  local port="$2"
  "${PYTHON_BIN}" - "$host" "$port" <<'PY' >/dev/null 2>&1
import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(0.5)
try:
    sock.connect((sys.argv[1], int(sys.argv[2])))
except OSError:
    sys.exit(1)
finally:
    sock.close()
sys.exit(0)
PY
}

carla_api_ready() {
  local host="$1"
  local port="$2"
  local root="$3"
  "${PYTHON_BIN}" - "$host" "$port" "$root" <<'PY' >/dev/null 2>&1
import glob
import os
import sys

host = sys.argv[1]
port = int(sys.argv[2])
root = sys.argv[3]
paths = [os.path.join(root, "PythonAPI", "carla")]
paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
for path in reversed(paths):
    if path and os.path.exists(path) and path not in sys.path:
        sys.path.insert(0, path)
import carla
client = carla.Client(host, port)
client.set_timeout(2.0)
world = client.get_world()
_ = world.get_map().name
PY
}

wait_tcp() {
  local name="$1"
  local host="$2"
  local port="$3"
  local timeout_sec="$4"
  local start_sec
  start_sec="$(date +%s)"
  while true; do
    if tcp_open "$host" "$port"; then
      log "${name} ready at ${host}:${port}"
      return 0
    fi
    if [ $(( $(date +%s) - start_sec )) -ge "$timeout_sec" ]; then
      return 1
    fi
    sleep 1
  done
}

pid_running() {
  local pid="$1"
  local stat=""
  [ -n "$pid" ] || return 1
  stat="$(ps -p "$pid" -o stat= 2>/dev/null | awk '{print $1}')"
  [ -n "$stat" ] || return 1
  case "$stat" in
    Z*) return 1 ;;
    *) return 0 ;;
  esac
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --carla-root) CARLA_ROOT="$2"; shift 2 ;;
    --host) CARLA_HOST="$2"; shift 2 ;;
    --port) CARLA_PORT="$2"; shift 2 ;;
    --python) PYTHON_BIN="$2"; shift 2 ;;
    --config) CONFIG_PATH="$2"; shift 2 ;;
    --log-dir) LOG_DIR="$2"; shift 2 ;;
    --reuse-bridge) RESTART_BRIDGE="0"; shift ;;
    --restart-bridge) RESTART_BRIDGE="1"; shift ;;
    -h|--help)
      echo "Usage: start-apa-carla.sh [--carla-root PATH] [--config PATH] [--reuse-bridge]"
      exit 0
      ;;
    *) fail "unknown arg: $1" ;;
  esac
done

[ -f "$CONFIG_PATH" ] || fail "config not found: $CONFIG_PATH"
[ -d "$CARLA_ROOT" ] || fail "CARLA root not found: $CARLA_ROOT"
command -v "$PYTHON_BIN" >/dev/null 2>&1 || fail "python not found: $PYTHON_BIN"

mkdir -p "$LOG_DIR"
export CARLA_ROOT CARLA_HOST CARLA_PORT PYTHON_BIN CONFIG_PATH CARLA_EXTRA_ARGS CARLA_FORCE_NVIDIA_VULKAN
export CARLA_PID_FILE="${CARLA_PID_FILE:-${LOG_DIR}/carla.pid}"
export CARLA_LOG_FILE="${CARLA_LOG_FILE:-${LOG_DIR}/carla.log}"
export BRIDGE_PID_FILE="${BRIDGE_PID_FILE:-${LOG_DIR}/bridge_apa.pid}"
export BRIDGE_LOG_FILE="${BRIDGE_LOG_FILE:-${LOG_DIR}/bridge_apa.log}"

if ! carla_api_ready "$CARLA_HOST" "$CARLA_PORT" "$CARLA_ROOT"; then
  log "starting CARLA from ${CARLA_ROOT}"
  "${SCRIPT_DIR}/start-carla.sh" --background --root "$CARLA_ROOT" --log "$CARLA_LOG_FILE" --pid-file "$CARLA_PID_FILE"
fi

start_sec="$(date +%s)"
while ! carla_api_ready "$CARLA_HOST" "$CARLA_PORT" "$CARLA_ROOT"; do
  if [ $(( $(date +%s) - start_sec )) -ge "$WAIT_CARLA_SEC" ]; then
    tail -120 "$CARLA_LOG_FILE" 2>/dev/null || true
    fail "CARLA Python API did not become ready within ${WAIT_CARLA_SEC}s"
  fi
  sleep 1
done
log "CARLA Python API ready at ${CARLA_HOST}:${CARLA_PORT}"

if [ "$RESTART_BRIDGE" = "1" ]; then
  if [ -f "$BRIDGE_PID_FILE" ]; then
    old_pid="$(cat "$BRIDGE_PID_FILE" 2>/dev/null || true)"
    if pid_running "$old_pid"; then
      kill "$old_pid" >/dev/null 2>&1 || true
    fi
    rm -f "$BRIDGE_PID_FILE"
  fi
fi

if [ -f "$BRIDGE_PID_FILE" ] && pid_running "$(cat "$BRIDGE_PID_FILE")"; then
  log "APA bridge already running pid=$(cat "$BRIDGE_PID_FILE")"
else
  log "starting APA bridge"
  nohup setsid "$PYTHON_BIN" "${SCRIPT_DIR}/carla_bridge_apa.py" --config "$CONFIG_PATH" >"$BRIDGE_LOG_FILE" 2>&1 </dev/null &
  echo "$!" >"$BRIDGE_PID_FILE"
  log "APA bridge started pid=$(cat "$BRIDGE_PID_FILE") log=${BRIDGE_LOG_FILE}"
fi

sleep 1
if ! pid_running "$(cat "$BRIDGE_PID_FILE")"; then
  tail -120 "$BRIDGE_LOG_FILE" 2>/dev/null || true
  fail "APA bridge exited during startup"
fi

wait_tcp "Bridge PUB" "127.0.0.1" "5701" "$WAIT_BRIDGE_SEC" || fail "Bridge PUB port 5701 not ready"
log "Bridge CONTROL subscriber configured for Pangu at 127.0.0.1:5702"
if ! pid_running "$(cat "$BRIDGE_PID_FILE")"; then
  tail -120 "$BRIDGE_LOG_FILE" 2>/dev/null || true
  fail "APA bridge exited after configuring ZMQ"
fi
log "APA stack is ready"
