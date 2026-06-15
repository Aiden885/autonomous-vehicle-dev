#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/gaasd-carla-manual}"
STOP_GAASD_SIM="1"

usage() {
    cat <<'EOF'
Usage:
  tools/carla_bridge/stop-gaasd-carla-manual.sh [options]

Options:
  --log-dir PATH        Log directory used by start script.
  --carla-root PATH     CARLA package root.
  --keep-gaasd-sim      Do not stop GAASD module_sim containers/processes.
  -h, --help            Show this help.
EOF
}

log() {
    printf '[GAASD-CARLA] %s\n' "$*"
}

stop_pid_file() {
    local name="$1"
    local pid_file="$2"
    local pid=""

    if [ ! -f "$pid_file" ]; then
        log "${name} pid file not found: ${pid_file}"
        return 0
    fi

    pid="$(cat "$pid_file" 2>/dev/null || true)"
    if [ -z "$pid" ]; then
        rm -f "$pid_file"
        log "${name} empty pid file removed"
        return 0
    fi

    if kill "$pid" 2>/dev/null; then
        log "${name} stopped pid=${pid}"
        sleep 0.2
    else
        log "${name} already stopped pid=${pid}"
    fi
    rm -f "$pid_file"
}

stop_by_pattern() {
    local name="$1"
    local pattern="$2"
    local pids

    pids="$(pgrep -f "$pattern" || true)"
    if [ -z "$pids" ]; then
        log "${name} no matching process"
        return 0
    fi

    for pid in $pids; do
        if [ "$pid" != "$$" ]; then
            if kill "$pid" 2>/dev/null; then
                log "${name} stopped pid=${pid}"
                sleep 0.2
            fi
        fi
    done
}

stop_module_sim_containers() {
    if ! command -v docker >/dev/null 2>&1; then
        log "docker not found, skip container cleanup"
        return 0
    fi

    local names
    names="$(docker ps --format '{{.Names}}' 2>/dev/null | grep -E '^(module_sim_container_|codex_carla_)' || true)"
    if [ -z "$names" ]; then
        log "no GAASD module_sim containers"
        return 0
    fi

    for name in $names; do
        if docker stop "$name" >/dev/null 2>&1; then
            log "stopped container ${name}"
        else
            log "container already stopped or inaccessible: ${name}"
        fi
    done
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --log-dir)
            LOG_DIR="$2"
            shift 2
            ;;
        --carla-root)
            CARLA_ROOT="$2"
            shift 2
            ;;
        --keep-gaasd-sim)
            STOP_GAASD_SIM="0"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf '[GAASD-CARLA] ERROR: unknown arg: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

BRIDGE_PID_FILE="${BRIDGE_PID_FILE:-${LOG_DIR}/bridge.pid}"
CARLA_PID_FILE="${CARLA_PID_FILE:-${LOG_DIR}/carla.pid}"
SPECTATOR_PID_FILE="${SPECTATOR_PID_FILE:-${LOG_DIR}/spectator.pid}"
WATCH_CAMERA_PID_FILE="${WATCH_CAMERA_PID_FILE:-${LOG_DIR}/watch-camera.pid}"

log "stopping manual stack, log dir: ${LOG_DIR}"

stop_pid_file "Camera window" "$WATCH_CAMERA_PID_FILE"
stop_by_pattern "Camera window fallback" "tools/carla_bridge/watch-carla.py"

stop_pid_file "Spectator follow" "$SPECTATOR_PID_FILE"
stop_by_pattern "Spectator follow fallback" "tools/carla_bridge/set-spectator-follow.py"

stop_pid_file "Bridge" "$BRIDGE_PID_FILE"
stop_by_pattern "Bridge fallback" "tools/carla_bridge/carla_bridge.py"

if [ "$STOP_GAASD_SIM" = "1" ]; then
    stop_by_pattern "module_sim process" "/tmp/module_sim_build/module_sim"
    stop_module_sim_containers
fi

stop_pid_file "CARLA" "$CARLA_PID_FILE"
stop_by_pattern "CARLA fallback" "${CARLA_ROOT}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
stop_by_pattern "CARLA launcher fallback" "${CARLA_ROOT}/CarlaUE4.sh"

log "manual stack stopped"
