#!/usr/bin/env bash
set -euo pipefail

HOST="${CARLA_HOST:-127.0.0.1}"
PORT="${CARLA_PORT:-2000}"
TIMEOUT="${CARLA_HEALTH_TIMEOUT:-2.0}"

usage() {
    cat <<'EOF'
Usage: health-carla.sh [--host HOST] [--port PORT] [--timeout SEC]
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

python3 - "$HOST" "$PORT" "$TIMEOUT" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
timeout = float(sys.argv[3])

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(timeout)

try:
    sock.connect((host, port))
except OSError as exc:
    print(f"[CARLA] unhealthy {host}:{port} ({exc})")
    sys.exit(1)
finally:
    sock.close()

print(f"[CARLA] healthy {host}:{port}")
PY
