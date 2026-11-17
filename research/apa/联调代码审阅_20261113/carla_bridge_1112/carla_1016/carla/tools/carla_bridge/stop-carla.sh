#!/usr/bin/env bash
set -euo pipefail

PID_FILE="${CARLA_PID_FILE:-/tmp/gaasd-carla.pid}"
CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
BINARY_PATH="$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"

stop_pid() {
    local pid="$1"

    if [ -n "$pid" ] && kill "$pid" 2>/dev/null; then
        echo "[CARLA] stopped pid=$pid"
        return 0
    fi

    return 1
}

PID=""
if [ -f "$PID_FILE" ]; then
    PID="$(cat "$PID_FILE")"
fi

if stop_pid "$PID"; then
    rm -f "$PID_FILE"
    exit 0
fi

ACTUAL_PID="$(pgrep -f "$BINARY_PATH" | tail -n 1 || true)"
if stop_pid "$ACTUAL_PID"; then
    rm -f "$PID_FILE"
    exit 0
fi

rm -f "$PID_FILE"
echo "[CARLA] process already stopped"
