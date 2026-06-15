#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
BRIDGE_DIR="${REPO_ROOT}/tools/carla_bridge"

export CONFIG_PATH="${SCENARIO_DIR}/bridge_config.json"
export CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"

cd "$REPO_ROOT"

echo "[Scenario] newaccpro2 keyboard ACC decision and lane keeping"
echo "[Scenario] config: ${CONFIG_PATH}"

"${BRIDGE_DIR}/start-gaasd-carla-manual.sh" \
  --ego-spawn-index 198 \
  --lead-distance 25 \
  --lead-speed 2 \
  --lead-placement lane_waypoint \
  --lead-behavior traffic_manager \
  --follow-spectator \
  --spectator-back 8 \
  --spectator-up 6 \
  --spectator-pitch -25 \
  --watch-camera \
  --no-probe

echo "[Scenario] CARLA + Bridge + keyboard window ready."
echo "[Scenario] Open project/newaccpro2 in GAASD and start the oscilloscope simulation."
