#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/gaasd-carla-lkspro1}"
exec "${REPO_ROOT}/tools/carla_bridge/stop-gaasd-carla-manual.sh" "$@"
