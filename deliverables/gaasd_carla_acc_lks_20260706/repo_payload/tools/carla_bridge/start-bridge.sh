#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="${BRIDGE_PID_FILE:-/tmp/gaasd-carla-bridge.pid}"
LOG_FILE="${BRIDGE_LOG_FILE:-/tmp/gaasd-carla-bridge.log}"

if [ -f "$PID_FILE" ]; then
    OLD_PID="$(cat "$PID_FILE")"
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        echo "[Bridge] already running pid=$OLD_PID log=$LOG_FILE"
        exit 0
    fi
fi

nohup setsid "${SCRIPT_DIR}/run-bridge.sh" "$@" >"$LOG_FILE" 2>&1 </dev/null &
echo "$!" >"$PID_FILE"
echo "[Bridge] started pid=$(cat "$PID_FILE") log=$LOG_FILE"
