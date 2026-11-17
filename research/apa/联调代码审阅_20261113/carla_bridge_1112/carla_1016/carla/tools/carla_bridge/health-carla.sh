#!/usr/bin/env bash
set -euo pipefail

HOST="${CARLA_HOST:-127.0.0.1}"
PORT="${CARLA_PORT:-2000}"
TIMEOUT="${CARLA_HEALTH_TIMEOUT:-2.0}"
CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
PYTHON_BIN="${PYTHON_BIN:-python3.8}"

usage() {
    cat <<'EOF'
Usage: health-carla.sh [--host HOST] [--port PORT] [--timeout SEC] [--carla-root PATH] [--python PATH]
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --host)
            HOST="$2"
            shift 2
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        --carla-root)
            CARLA_ROOT="$2"
            shift 2
            ;;
        --python)
            PYTHON_BIN="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "unknown arg: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

"$PYTHON_BIN" - "$HOST" "$PORT" "$TIMEOUT" "$CARLA_ROOT" <<'PY'
import glob
import os
import sys

host = sys.argv[1]
port = int(sys.argv[2])
timeout = float(sys.argv[3])
root = sys.argv[4]

paths = [os.path.join(root, "PythonAPI", "carla")]
paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
for path in reversed(paths):
    if path and os.path.exists(path) and path not in sys.path:
        sys.path.insert(0, path)

try:
    import carla  # type: ignore
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    map_name = world.get_map().name
except Exception as exc:
    print(f"[CARLA] unhealthy {host}:{port} ({exc})")
    sys.exit(1)

print(f"[CARLA] healthy {host}:{port} map={map_name}")
PY
