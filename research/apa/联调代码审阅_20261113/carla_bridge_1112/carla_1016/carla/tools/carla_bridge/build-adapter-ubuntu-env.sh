#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ADAPTER_DIR="${SCRIPT_DIR}/adapter"
OUTPUT_DIR="${ADAPTER_OUTPUT_DIR:-${ADAPTER_DIR}/dist/ubuntu_env}"
IMAGE="${GAASD_RUNTIME_IMAGE:-ubuntu:env}"

usage() {
    cat <<'EOF'
Usage: tools/carla_bridge/build-adapter-ubuntu-env.sh [--output PATH] [--image IMAGE]

Build libcarla_gaasd_adapter.so in the same container ABI used by GAASD simulation.
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --image)
            IMAGE="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'unknown argument: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd)"

docker run --rm \
    -v "${ADAPTER_DIR}:/work/adapter:ro" \
    -v "${OUTPUT_DIR}:/out" \
    "$IMAGE" bash -lc '
        set -euo pipefail
        rm -rf /tmp/carla_adapter_build
        cmake -S /work/adapter -B /tmp/carla_adapter_build -DCMAKE_BUILD_TYPE=Release
        cmake --build /tmp/carla_adapter_build -- -j2
        LD_LIBRARY_PATH=/tmp/carla_adapter_build /tmp/carla_adapter_build/carla_gaasd_adapter_mock_loop
        cp /tmp/carla_adapter_build/libcarla_gaasd_adapter.so /out/libcarla_gaasd_adapter.so
    '

printf '[Adapter] output: %s/libcarla_gaasd_adapter.so\n' "$OUTPUT_DIR"
