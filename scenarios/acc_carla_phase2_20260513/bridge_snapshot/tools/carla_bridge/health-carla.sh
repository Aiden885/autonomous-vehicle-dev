#!/usr/bin/env bash
# Check CARLA health by probing Bridge ZMQ ports.
# Direct bare-TCP to CARLA:2000 (RPC port) crashes CARLA 0.9.x on connect/close.
# Bridge exits when CARLA disconnects, so Bridge ports being reachable implies CARLA is up.
set -euo pipefail

PUB_PORT="${BRIDGE_PUB_PORT:-5701}"
CONTROL_PORT="${BRIDGE_CONTROL_PORT:-5702}"
HOST="${CARLA_HOST:-127.0.0.1}"
TIMEOUT="${CARLA_HEALTH_TIMEOUT:-2.0}"

usage() {
    cat <<'EOF'
Usage: health-carla.sh [--host HOST] [--pub-port PORT] [--control-port PORT] [--timeout SEC]
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --host)
            HOST="$2"
            shift 2
            ;;
        --pub-port)
            PUB_PORT="$2"
            shift 2
            ;;
        --control-port)
            CONTROL_PORT="$2"
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

python3 - "$HOST" "$PUB_PORT" "$CONTROL_PORT" "$TIMEOUT" <<'PY'
import socket
import sys

host = sys.argv[1]
pub_port = int(sys.argv[2])
control_port = int(sys.argv[3])
timeout = float(sys.argv[4])

def port_open(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
        return True
    except OSError:
        return False
    finally:
        sock.close()

pub_ok = port_open(pub_port)
control_ok = port_open(control_port)
carla_ok = pub_ok and control_ok

if carla_ok:
    print(f"[CARLA] healthy (inferred via Bridge {host}:{pub_port}/{control_port})")
    sys.exit(0)
else:
    print(f"[CARLA] unhealthy: Bridge PUB={pub_ok} CONTROL={control_ok}")
    sys.exit(1)
PY
