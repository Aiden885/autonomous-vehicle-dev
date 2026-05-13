#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
BRIDGE_DIR="${SCENARIO_DIR}/bridge_snapshot/tools/carla_bridge"

export CONFIG_PATH="${SCENARIO_DIR}/bridge_config.json"
export CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"

cd "$REPO_ROOT"

echo "[Scenario] using bridge snapshot: ${BRIDGE_DIR}"
echo "[Scenario] using config: ${CONFIG_PATH}"

"${BRIDGE_DIR}/start-gaasd-carla-manual.sh" \
  --lead-placement lane_waypoint \
  --lead-behavior traffic_manager \
  --no-probe

echo "[Scenario] CARLA + Bridge ready. Open GAASD and run the oscilloscope simulation."
