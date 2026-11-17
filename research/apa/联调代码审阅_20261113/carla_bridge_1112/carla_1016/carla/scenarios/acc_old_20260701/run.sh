#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

export CARLA_ROOT="${CARLA_ROOT:-/opt/carla}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"
export CONFIG_PATH="${CONFIG_PATH:-${SCENARIO_DIR}/bridge_config.json}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/newaccpro3-pangu-carla/carla}"
export GAASD_PANEL_PORT="${GAASD_PANEL_PORT:-8088}"

EGO_SPAWN_INDEX="${EGO_SPAWN_INDEX:-198}"
LEAD_DISTANCE="${LEAD_DISTANCE:-25}"
LEAD_SPEED="${LEAD_SPEED:-2}"
LEAD_PLACEMENT="${LEAD_PLACEMENT:-lane_waypoint}"
LEAD_BEHAVIOR="${LEAD_BEHAVIOR:-constant_velocity}"
SPECTATOR_BACK="${SPECTATOR_BACK:-8}"
SPECTATOR_UP="${SPECTATOR_UP:-6}"
SPECTATOR_PITCH="${SPECTATOR_PITCH:--25}"
WATCH_CAMERA="${WATCH_CAMERA:-0}"
PROBE_BRIDGE="${PROBE_BRIDGE:-0}"

args=(
  --config "${CONFIG_PATH}"
  --ego-spawn-index "${EGO_SPAWN_INDEX}"
  --lead-distance "${LEAD_DISTANCE}"
  --lead-speed "${LEAD_SPEED}"
  --lead-placement "${LEAD_PLACEMENT}"
  --lead-behavior "${LEAD_BEHAVIOR}"
  --follow-spectator
  --spectator-back "${SPECTATOR_BACK}"
  --spectator-up "${SPECTATOR_UP}"
  --spectator-pitch "${SPECTATOR_PITCH}"
)

if [[ "${WATCH_CAMERA}" == "1" ]]; then
  args+=(--watch-camera)
fi

if [[ "${PROBE_BRIDGE}" != "1" ]]; then
  args+=(--no-probe)
fi

cd "${REPO_ROOT}"
echo "[ACC bridge-only] CARLA_ROOT=${CARLA_ROOT}"
echo "[ACC bridge-only] CONFIG_PATH=${CONFIG_PATH}"
echo "[ACC bridge-only] logs=${GAASD_CARLA_LOG_DIR}"
"${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh" "${args[@]}"

echo "[ACC bridge-only] starting scenario panel"
bash "${REPO_ROOT}/tools/gaasd_scenario_panel/start-panel.sh"
echo "[ACC bridge-only] ready: http://127.0.0.1:${GAASD_PANEL_PORT}"
