#!/usr/bin/env bash
set -euo pipefail

CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
PID_FILE="${CARLA_PID_FILE:-/tmp/gaasd-carla.pid}"
LOG_FILE="${CARLA_LOG_FILE:-/tmp/gaasd-carla.log}"
QUALITY_ARG="-quality-level=Low"
BACKGROUND="0"

usage() {
    cat <<'EOF'
Usage: start-carla.sh [--root PATH] [--high] [--background] [--log PATH] [--pid-file PATH] [-- extra args]

Defaults:
  --root       /home/aiden/snap/code/app/carla-package
  quality      Low
  foreground   yes
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --root)
            CARLA_ROOT="$2"
            shift 2
            ;;
        --high)
            QUALITY_ARG=""
            shift
            ;;
        --background)
            BACKGROUND="1"
            shift
            ;;
        --log)
            LOG_FILE="$2"
            shift 2
            ;;
        --pid-file)
            PID_FILE="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

if [ ! -d "$CARLA_ROOT" ]; then
    echo "[CARLA] root not found: $CARLA_ROOT" >&2
    exit 1
fi

cd "$CARLA_ROOT"

if [ -x "./start-carla.sh" ]; then
    CMD=(./start-carla.sh)
    if [ -z "$QUALITY_ARG" ]; then
        CMD+=(--high)
    fi
else
    if [ ! -x "./CarlaUE4.sh" ]; then
        echo "[CARLA] neither ./start-carla.sh nor ./CarlaUE4.sh is executable in $CARLA_ROOT" >&2
        exit 1
    fi
    CMD=(./CarlaUE4.sh)
    if [ -n "$QUALITY_ARG" ]; then
        CMD+=("$QUALITY_ARG")
    fi
fi

CMD+=("$@")

export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

if [ "$BACKGROUND" = "1" ]; then
    nohup setsid "${CMD[@]}" >"$LOG_FILE" 2>&1 </dev/null &
    LAUNCHER_PID="$!"
    BINARY_PATH="$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
    sleep 1
    ACTUAL_PID="$(pgrep -f "$BINARY_PATH" | tail -n 1 || true)"
    if [ -n "$ACTUAL_PID" ]; then
        echo "$ACTUAL_PID" >"$PID_FILE"
    else
        echo "$LAUNCHER_PID" >"$PID_FILE"
    fi
    echo "[CARLA] started in background pid=$(cat "$PID_FILE") log=$LOG_FILE"
else
    echo "[CARLA] starting in foreground from $CARLA_ROOT"
    exec "${CMD[@]}"
fi
