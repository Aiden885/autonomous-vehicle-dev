#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
BRIDGE_DIR="${REPO_ROOT}/scenarios/acc_carla_phase2_20260513/bridge_snapshot/tools/carla_bridge"

export CONFIG_PATH="${SCENARIO_DIR}/bridge_config.json"
export CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"

cd "$REPO_ROOT"

echo "[Scenario] accpro2 basic-module ACC"
echo "[Scenario] using bridge snapshot: ${BRIDGE_DIR}"
echo "[Scenario] using config: ${CONFIG_PATH}"

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
  --no-probe

echo "[Scenario] CARLA + Bridge ready. Open GAASD project/accpro2 and run the oscilloscope simulation."
