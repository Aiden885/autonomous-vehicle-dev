#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

export CARLA_ROOT="${CARLA_ROOT:-/opt/carla}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"
BASE_CONFIG_PATH="${SCENARIO_DIR}/bridge_config.json"
export CONFIG_PATH="${CONFIG_PATH:-${GAASD_CARLA_LOG_DIR:-/tmp/lks2-pangu-carla/carla}/bridge_config.runtime.json}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/lks2-pangu-carla/carla}"
export GAASD_PANEL_PORT="${GAASD_PANEL_PORT:-8088}"
export CARLA_FORCE_NVIDIA_VULKAN="${CARLA_FORCE_NVIDIA_VULKAN:-1}"
export CARLA_EXTRA_ARGS="${CARLA_EXTRA_ARGS:--windowed -Resx=800 -Resy=600 -fps=20 -nosound -graphicsadapter=0 -norhithread}"

EGO_SPAWN_INDEX="${EGO_SPAWN_INDEX:-0}"
SPECTATOR_BACK="${SPECTATOR_BACK:-8}"
SPECTATOR_UP="${SPECTATOR_UP:-6}"
SPECTATOR_PITCH="${SPECTATOR_PITCH:--25}"
LKS_MAP_NAME="${LKS_MAP_NAME:-Town04}"
LKS_REFERENCE_X="${LKS_REFERENCE_X:--511.738}"
LKS_REFERENCE_Y="${LKS_REFERENCE_Y:-242.657}"
LKS_REFERENCE_Z="${LKS_REFERENCE_Z:-0.5}"
LKS_EGO_X="${LKS_EGO_X:-${LKS_REFERENCE_X}}"
LKS_EGO_Y="${LKS_EGO_Y:-${LKS_REFERENCE_Y}}"
LKS_EGO_Z="${LKS_EGO_Z:-${LKS_REFERENCE_Z}}"
LKS_INITIAL_OFFSET_M="${LKS_INITIAL_OFFSET_M:-0.0}"
LKS_FOLLOW_SPECTATOR="${LKS_FOLLOW_SPECTATOR:-1}"
LKS_WATCH_CAMERA="${LKS_WATCH_CAMERA:-0}"
PROBE_BRIDGE="${PROBE_BRIDGE:-0}"

mkdir -p "${GAASD_CARLA_LOG_DIR}"
BASE_CONFIG_PATH="${BASE_CONFIG_PATH}" CONFIG_PATH="${CONFIG_PATH}" LKS_MAP_NAME="${LKS_MAP_NAME}" \
  "${PYTHON_BIN}" - <<'PY'
import json
import os
from pathlib import Path

base_path = Path(os.environ["BASE_CONFIG_PATH"])
config_path = Path(os.environ["CONFIG_PATH"])
data = json.loads(base_path.read_text(encoding="utf-8"))
data.setdefault("carla", {})["map_name"] = os.environ["LKS_MAP_NAME"]
config_path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY

args=(
  --config "${CONFIG_PATH}"
  --ego-spawn-index "${EGO_SPAWN_INDEX}"
  --no-lead
  --spectator-back "${SPECTATOR_BACK}"
  --spectator-up "${SPECTATOR_UP}"
  --spectator-pitch "${SPECTATOR_PITCH}"
)

if [[ "${LKS_FOLLOW_SPECTATOR}" == "1" ]]; then
  args+=(--follow-spectator)
else
  args+=(--no-follow-spectator)
fi

if [[ "${LKS_WATCH_CAMERA}" == "1" ]]; then
  args+=(--watch-camera)
fi

if [[ "${PROBE_BRIDGE}" != "1" ]]; then
  args+=(--no-probe)
fi

cd "${REPO_ROOT}"
echo "[LKS bridge-only] CARLA_ROOT=${CARLA_ROOT}"
echo "[LKS bridge-only] CONFIG_PATH=${CONFIG_PATH}"
echo "[LKS bridge-only] logs=${GAASD_CARLA_LOG_DIR}"
echo "[LKS bridge-only] CARLA_EXTRA_ARGS=${CARLA_EXTRA_ARGS}"
"${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh" "${args[@]}"

"${PYTHON_BIN}" "${REPO_ROOT}/tools/carla_bridge/reset-lks-reference-scene.py" \
  --carla-root "${CARLA_ROOT}" \
  --expected-map "${LKS_MAP_NAME}" \
  --reference-x "${LKS_REFERENCE_X}" \
  --reference-y "${LKS_REFERENCE_Y}" \
  --ego-x "${LKS_EGO_X}" \
  --ego-y "${LKS_EGO_Y}" \
  --ego-z "${LKS_EGO_Z}" \
  --lateral-offset-m "${LKS_INITIAL_OFFSET_M}"

echo "[LKS bridge-only] starting scenario panel"
bash "${REPO_ROOT}/tools/gaasd_scenario_panel/start-panel.sh"
echo "[LKS bridge-only] ready: http://127.0.0.1:${GAASD_PANEL_PORT}"
