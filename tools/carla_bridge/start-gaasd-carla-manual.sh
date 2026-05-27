#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
CARLA_PORT="${CARLA_PORT:-2000}"
PYTHON_BIN="${PYTHON_BIN:-python3.8}"
CONFIG_PATH="${CONFIG_PATH:-${PROJECT_ROOT}/tools/carla_bridge/config.phase2.json}"
LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/gaasd-carla-manual}"
WAIT_CARLA_SEC="${WAIT_CARLA_SEC:-90}"
WAIT_BRIDGE_SEC="${WAIT_BRIDGE_SEC:-20}"
WAIT_EGO_SEC="${WAIT_EGO_SEC:-15}"
EGO_SPAWN_INDEX="${EGO_SPAWN_INDEX:-198}"
LEAD_DISTANCE_M="${LEAD_DISTANCE_M:-25}"
LEAD_SPEED_MPS="${LEAD_SPEED_MPS:-2}"
LEAD_PLACEMENT="${LEAD_PLACEMENT:-ego_forward}"
LEAD_BEHAVIOR="${LEAD_BEHAVIOR:-traffic_manager}"
TRAFFIC_MANAGER_PORT="${TRAFFIC_MANAGER_PORT:-8000}"
RESTART_BRIDGE="${RESTART_BRIDGE:-1}"
FOLLOW_SPECTATOR="1"
WATCH_CAMERA="${WATCH_CAMERA:-0}"
SPECTATOR_BACK_M="${SPECTATOR_BACK_M:-8}"
SPECTATOR_UP_M="${SPECTATOR_UP_M:-6}"
SPECTATOR_PITCH_DEG="${SPECTATOR_PITCH_DEG:--25}"
RESET_SCENE="1"
SPAWN_LEAD="1"
PROBE_BRIDGE="1"
QUALITY_OPT=""

usage() {
    cat <<'EOF'
Usage:
  tools/carla_bridge/start-gaasd-carla-manual.sh [options]

Options:
  --carla-root PATH       CARLA package root.
  --host HOST             CARLA host, default 127.0.0.1.
  --port PORT             CARLA port, default 2000.
  --python PATH           Python executable for CARLA API, default python3.8.
  --config PATH           Bridge config JSON.
  --log-dir PATH          Log directory, default /tmp/gaasd-carla-manual.
  --wait-carla SEC        Max seconds to wait for CARLA TCP/API readiness.
  --wait-bridge SEC       Max seconds to wait for Bridge ZMQ ports.
  --ego-spawn-index INDEX Straight-road ego spawn index, default 198.
  --lead-distance M       Lead vehicle initial distance, default 25.
  --lead-speed MPS        Lead vehicle constant speed, default 2.
  --lead-placement MODE   ego_forward or lane_waypoint, default ego_forward.
  --lead-behavior MODE    traffic_manager or constant_velocity, default traffic_manager.
  --tm-port PORT          CARLA Traffic Manager port, default 8000.
  --restart-bridge        Restart Bridge before each visual test, default on.
  --reuse-bridge          Reuse existing Bridge if ports are already open.
  --follow-spectator      Keep CARLA spectator behind ego for visual tests, default on.
  --no-follow-spectator   Do not start the CARLA spectator follow process.
  --spectator-back M      Spectator follow distance behind ego, default 8.
  --spectator-up M        Spectator follow height above ego, default 6.
  --spectator-pitch DEG   Spectator follow pitch angle, default -25.
  --watch-camera          Open a pygame window with RGB camera and HUD (needs display).
  --no-reset-scene        Do not reset ego/lead to the straight-road ACC scene.
  --no-lead               Do not spawn/replace lead vehicle.
  --no-probe              Do not probe Bridge PUB messages.
  --high                  Start CARLA in high quality mode.
  -h, --help              Show this help.

After this script succeeds, open GAASD and start the oscilloscope simulation.
EOF
}

log() {
    printf '[GAASD-CARLA] %s\n' "$*"
}

fail() {
    printf '[GAASD-CARLA] ERROR: %s\n' "$*" >&2
    exit 1
}

stop_pid_file_if_running() {
    local name="$1"
    local pid_file="$2"
    local pid=""

    if [ ! -f "$pid_file" ]; then
        return 0
    fi

    pid="$(cat "$pid_file" 2>/dev/null || true)"
    if [ -z "$pid" ]; then
        rm -f "$pid_file"
        return 0
    fi

    if kill "$pid" 2>/dev/null; then
        log "stopped stale ${name} pid=${pid}"
        sleep 0.2
    fi
    rm -f "$pid_file"
}

stop_stale_spectator_follow() {
    local pids

    stop_pid_file_if_running "spectator follow" "$SPECTATOR_PID_FILE"

    pids="$(pgrep -f "set-spectator-follow.py" || true)"
    if [ -z "$pids" ]; then
        return 0
    fi

    for pid in $pids; do
        if [ "$pid" != "$$" ]; then
            if kill "$pid" 2>/dev/null; then
                log "stopped stale spectator follow pid=${pid}"
                sleep 0.2
            fi
        fi
    done
}

stop_stale_bridge() {
    local pids

    "${SCRIPT_DIR}/stop-bridge.sh" || true

    pids="$(pgrep -f "carla_bridge.py --config ${CONFIG_PATH}" || true)"
    if [ -z "$pids" ]; then
        return 0
    fi

    for pid in $pids; do
        if [ "$pid" != "$$" ]; then
            if kill "$pid" 2>/dev/null; then
                log "stopped stale Bridge pid=${pid}"
                sleep 0.2
            fi
        fi
    done
}

start_spectator_follow() {
    if [ "$FOLLOW_SPECTATOR" != "1" ]; then
        return 0
    fi

    stop_stale_spectator_follow
    log "starting spectator follow camera back=${SPECTATOR_BACK_M}m up=${SPECTATOR_UP_M}m pitch=${SPECTATOR_PITCH_DEG}deg"
    nohup setsid "$PYTHON_BIN" "${SCRIPT_DIR}/set-spectator-follow.py" \
        --carla-root "$CARLA_ROOT" \
        --host "$CARLA_HOST" \
        --port "$CARLA_PORT" \
        --ego-role-name hero \
        --back-m "$SPECTATOR_BACK_M" \
        --up-m "$SPECTATOR_UP_M" \
        --pitch-deg "$SPECTATOR_PITCH_DEG" \
        --fallback-spawn-index "$EGO_SPAWN_INDEX" \
        >"$SPECTATOR_LOG_FILE" 2>&1 </dev/null &
    printf '%s\n' "$!" >"$SPECTATOR_PID_FILE"
    log "spectator follow started pid=$(cat "$SPECTATOR_PID_FILE") log=${SPECTATOR_LOG_FILE}"
}

apply_spectator_once() {
    if [ "$FOLLOW_SPECTATOR" != "1" ]; then
        return 0
    fi

    if ! "$PYTHON_BIN" "${SCRIPT_DIR}/set-spectator-follow.py" \
        --carla-root "$CARLA_ROOT" \
        --host "$CARLA_HOST" \
        --port "$CARLA_PORT" \
        --ego-role-name hero \
        --back-m "$SPECTATOR_BACK_M" \
        --up-m "$SPECTATOR_UP_M" \
        --pitch-deg "$SPECTATOR_PITCH_DEG" \
        --fallback-spawn-index "$EGO_SPAWN_INDEX" \
        --once; then
        log "spectator one-shot update skipped; background follow will retry when ego is available"
    fi
}

tcp_open() {
    local host="$1"
    local port="$2"
    "${PYTHON_BIN}" - "$host" "$port" <<'PY' >/dev/null 2>&1
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

import carla  # type: ignore

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
    local now_sec

    start_sec="$(date +%s)"
    while true; do
        if tcp_open "$host" "$port"; then
            log "${name} ready at ${host}:${port}"
            return 0
        fi

        now_sec="$(date +%s)"
        if [ $((now_sec - start_sec)) -ge "$timeout_sec" ]; then
            return 1
        fi
        sleep 1
    done
}

wait_carla_api() {
    local timeout_sec="$1"
    local start_sec
    local now_sec

    start_sec="$(date +%s)"
    while true; do
        if carla_api_ready "$CARLA_HOST" "$CARLA_PORT" "$CARLA_ROOT"; then
            log "CARLA Python API ready at ${CARLA_HOST}:${CARLA_PORT}"
            return 0
        fi

        now_sec="$(date +%s)"
        if [ $((now_sec - start_sec)) -ge "$timeout_sec" ]; then
            return 1
        fi
        sleep 1
    done
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --carla-root)
            CARLA_ROOT="$2"
            shift 2
            ;;
        --host)
            CARLA_HOST="$2"
            shift 2
            ;;
        --port)
            CARLA_PORT="$2"
            shift 2
            ;;
        --python)
            PYTHON_BIN="$2"
            shift 2
            ;;
        --config)
            CONFIG_PATH="$2"
            shift 2
            ;;
        --log-dir)
            LOG_DIR="$2"
            shift 2
            ;;
        --wait-carla)
            WAIT_CARLA_SEC="$2"
            shift 2
            ;;
        --wait-bridge)
            WAIT_BRIDGE_SEC="$2"
            shift 2
            ;;
        --lead-distance)
            LEAD_DISTANCE_M="$2"
            shift 2
            ;;
        --ego-spawn-index)
            EGO_SPAWN_INDEX="$2"
            shift 2
            ;;
        --lead-speed)
            LEAD_SPEED_MPS="$2"
            shift 2
            ;;
        --lead-placement)
            LEAD_PLACEMENT="$2"
            shift 2
            ;;
        --lead-behavior)
            LEAD_BEHAVIOR="$2"
            shift 2
            ;;
        --tm-port)
            TRAFFIC_MANAGER_PORT="$2"
            shift 2
            ;;
        --restart-bridge)
            RESTART_BRIDGE="1"
            shift
            ;;
        --reuse-bridge)
            RESTART_BRIDGE="0"
            shift
            ;;
        --follow-spectator)
            FOLLOW_SPECTATOR="1"
            shift
            ;;
        --no-follow-spectator)
            FOLLOW_SPECTATOR="0"
            shift
            ;;
        --spectator-back)
            SPECTATOR_BACK_M="$2"
            shift 2
            ;;
        --spectator-up)
            SPECTATOR_UP_M="$2"
            shift 2
            ;;
        --spectator-pitch)
            SPECTATOR_PITCH_DEG="$2"
            shift 2
            ;;
        --watch-camera)
            WATCH_CAMERA="1"
            shift
            ;;
        --no-reset-scene)
            RESET_SCENE="0"
            shift
            ;;
        --no-lead)
            SPAWN_LEAD="0"
            RESET_SCENE="0"
            shift
            ;;
        --no-probe)
            PROBE_BRIDGE="0"
            shift
            ;;
        --high)
            QUALITY_OPT="--high"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            fail "unknown arg: $1"
            ;;
    esac
done

[ -x "${SCRIPT_DIR}/start-carla.sh" ] || fail "missing executable: ${SCRIPT_DIR}/start-carla.sh"
[ -x "${SCRIPT_DIR}/start-bridge.sh" ] || fail "missing executable: ${SCRIPT_DIR}/start-bridge.sh"
[ -f "$CONFIG_PATH" ] || fail "config not found: $CONFIG_PATH"
[ -d "$CARLA_ROOT" ] || fail "CARLA root not found: $CARLA_ROOT"
command -v "$PYTHON_BIN" >/dev/null 2>&1 || fail "python not found: $PYTHON_BIN"

mkdir -p "$LOG_DIR"

export CARLA_ROOT
export CARLA_HOST
export CARLA_PORT
export PYTHON_BIN
export CONFIG_PATH
export CARLA_PID_FILE="${CARLA_PID_FILE:-${LOG_DIR}/carla.pid}"
export CARLA_LOG_FILE="${CARLA_LOG_FILE:-${LOG_DIR}/carla.log}"
export BRIDGE_PID_FILE="${BRIDGE_PID_FILE:-${LOG_DIR}/bridge.pid}"
export BRIDGE_LOG_FILE="${BRIDGE_LOG_FILE:-${LOG_DIR}/bridge.log}"
export SPECTATOR_PID_FILE="${SPECTATOR_PID_FILE:-${LOG_DIR}/spectator.pid}"
export SPECTATOR_LOG_FILE="${SPECTATOR_LOG_FILE:-${LOG_DIR}/spectator.log}"
export WATCH_CAMERA_PID_FILE="${WATCH_CAMERA_PID_FILE:-${LOG_DIR}/watch-camera.pid}"
export WATCH_CAMERA_LOG_FILE="${WATCH_CAMERA_LOG_FILE:-${LOG_DIR}/watch-camera.log}"

log "logs: ${LOG_DIR}"

if carla_api_ready "$CARLA_HOST" "$CARLA_PORT" "$CARLA_ROOT"; then
    log "CARLA Python API already ready at ${CARLA_HOST}:${CARLA_PORT}"
else
    log "starting CARLA from ${CARLA_ROOT}"
    "${SCRIPT_DIR}/start-carla.sh" --background --root "$CARLA_ROOT" --log "$CARLA_LOG_FILE" --pid-file "$CARLA_PID_FILE" ${QUALITY_OPT}
fi

if ! wait_carla_api "$WAIT_CARLA_SEC"; then
    tail -120 "$CARLA_LOG_FILE" 2>/dev/null || true
    fail "CARLA Python API did not become ready within ${WAIT_CARLA_SEC}s"
fi

if tcp_open "127.0.0.1" "5701" && tcp_open "127.0.0.1" "5702"; then
    log "Bridge already reachable at 127.0.0.1:5701/5702"
else
    if [ "$RESTART_BRIDGE" = "1" ]; then
        log "restarting Bridge for clean CARLA test state"
        stop_stale_bridge
    fi
    log "starting Bridge"
    "${SCRIPT_DIR}/start-bridge.sh"
fi

if ! wait_tcp "Bridge PUB" "127.0.0.1" "5701" "$WAIT_BRIDGE_SEC"; then
    tail -120 "$BRIDGE_LOG_FILE" 2>/dev/null || true
    fail "Bridge PUB port 5701 did not become ready within ${WAIT_BRIDGE_SEC}s"
fi

if ! wait_tcp "Bridge CONTROL" "127.0.0.1" "5702" "$WAIT_BRIDGE_SEC"; then
    tail -120 "$BRIDGE_LOG_FILE" 2>/dev/null || true
    fail "Bridge CONTROL port 5702 did not become ready within ${WAIT_BRIDGE_SEC}s"
fi

if [ "$RESET_SCENE" = "1" ]; then
    log "resetting straight-road scene ego_spawn_index=${EGO_SPAWN_INDEX} lead_distance=${LEAD_DISTANCE_M}m lead_speed=${LEAD_SPEED_MPS}m/s"
    "$PYTHON_BIN" "${SCRIPT_DIR}/reset-acc-straight-scene.py" \
        --carla-root "$CARLA_ROOT" \
        --host "$CARLA_HOST" \
        --port "$CARLA_PORT" \
        --ego-spawn-index "$EGO_SPAWN_INDEX" \
        --lead-distance-m "$LEAD_DISTANCE_M" \
        --lead-speed-mps "$LEAD_SPEED_MPS" \
        --spectator-back-m "$SPECTATOR_BACK_M" \
        --spectator-up-m "$SPECTATOR_UP_M" \
        --spectator-pitch-deg "$SPECTATOR_PITCH_DEG"
elif [ "$SPAWN_LEAD" = "1" ]; then
    log "spawning lead vehicle distance=${LEAD_DISTANCE_M}m speed=${LEAD_SPEED_MPS}m/s placement=${LEAD_PLACEMENT} behavior=${LEAD_BEHAVIOR}"
    "$PYTHON_BIN" "${SCRIPT_DIR}/spawn-lead-vehicle.py" \
        --carla-root "$CARLA_ROOT" \
        --host "$CARLA_HOST" \
        --port "$CARLA_PORT" \
        --wait-ego-sec "$WAIT_EGO_SEC" \
        --replace \
        --distance-m "$LEAD_DISTANCE_M" \
        --speed-mps "$LEAD_SPEED_MPS" \
        --placement "$LEAD_PLACEMENT" \
        --behavior "$LEAD_BEHAVIOR" \
        --traffic-manager-port "$TRAFFIC_MANAGER_PORT"
fi

# Pin spectator once (blocking) before starting background follow loop.
# set-spectator-follow.py --once does not call world.tick(); it only reads
# the ego transform and writes the spectator position, so it is safe while
# Bridge is ticking.
if [ "$FOLLOW_SPECTATOR" = "1" ]; then
    log "pinning spectator to ego (one-shot)"
    "$PYTHON_BIN" "${SCRIPT_DIR}/set-spectator-follow.py" \
        --once \
        --carla-root "$CARLA_ROOT" \
        --host "$CARLA_HOST" \
        --port "$CARLA_PORT" \
        --ego-role-name hero \
        --back-m "$SPECTATOR_BACK_M" \
        --up-m "$SPECTATOR_UP_M" \
        --pitch-deg "$SPECTATOR_PITCH_DEG" \
        --fallback-spawn-index "$EGO_SPAWN_INDEX" \
        --timeout-sec 10 || log "spectator one-shot skipped; background follow will retry"
fi

start_spectator_follow

if [ "$WATCH_CAMERA" = "1" ]; then
    log "starting pygame camera window (watch-carla.py)"
    nohup setsid "$PYTHON_BIN" "${SCRIPT_DIR}/watch-carla.py" \
        --carla-root "$CARLA_ROOT" \
        --host "$CARLA_HOST" \
        --port "$CARLA_PORT" \
        >"$WATCH_CAMERA_LOG_FILE" 2>&1 </dev/null &
    printf '%s\n' "$!" >"$WATCH_CAMERA_PID_FILE"
    log "camera window started pid=$(cat "$WATCH_CAMERA_PID_FILE") log=${WATCH_CAMERA_LOG_FILE}"
fi

if [ "$PROBE_BRIDGE" = "1" ]; then
    log "probing Bridge PUB messages"
    "$PYTHON_BIN" "${SCRIPT_DIR}/probe-pub.py" --duration 5 --min-messages 3
fi

log "manual stack is ready"
log "next: in GAASD, start the oscilloscope simulation for project/carla"
log "recommended signals: egoV, leadV, distance, targetSpeed"
log "stop commands:"
log "  BRIDGE_PID_FILE=${BRIDGE_PID_FILE} tools/carla_bridge/stop-bridge.sh"
log "  CARLA_PID_FILE=${CARLA_PID_FILE} CARLA_ROOT=${CARLA_ROOT} tools/carla_bridge/stop-carla.sh"
