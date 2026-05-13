#!/usr/bin/env bash
set -euo pipefail

PID_FILE="${BRIDGE_PID_FILE:-/tmp/gaasd-carla-bridge.pid}"

if [ ! -f "$PID_FILE" ]; then
    echo "[Bridge] pid file not found: $PID_FILE"
    exit 0
fi

PID="$(cat "$PID_FILE")"
if [ -z "$PID" ]; then
    rm -f "$PID_FILE"
    echo "[Bridge] empty pid file removed"
    exit 0
fi

if kill "$PID" 2>/dev/null; then
    rm -f "$PID_FILE"
    echo "[Bridge] stopped pid=$PID"
else
    rm -f "$PID_FILE"
    echo "[Bridge] process already stopped pid=$PID"
fi
