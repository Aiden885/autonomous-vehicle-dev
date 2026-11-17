#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

if [ -z "${CARLA_ROOT:-}" ]; then
  if [ -x "/home/nl/carla/CarlaUE4.sh" ]; then
    export CARLA_ROOT="/home/nl/carla"
  else
    export CARLA_ROOT="/opt/carla"
  fi
fi
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/apa-pangu-carla/carla}"
export GAASD_PANEL_PORT="${GAASD_PANEL_PORT:-8088}"
export CARLA_FORCE_NVIDIA_VULKAN="${CARLA_FORCE_NVIDIA_VULKAN:-1}"
export CARLA_EXTRA_ARGS="${CARLA_EXTRA_ARGS:--windowed -Resx=1280 -Resy=720 -fps=20 -nosound}"

MAP="${MAP:-Town04}"
SLOT_ID="${SLOT_ID:-}"
BASE_CONFIG_PATH="${SCENARIO_DIR}/bridge_config.json"
export CONFIG_PATH="${CONFIG_PATH:-${GAASD_CARLA_LOG_DIR}/bridge_config.runtime.json}"

usage() {
  cat <<'EOF'
Usage:
  bash scenarios/apa/run.sh [Town04|Town15] [slot_id]
  MAP=Town15 SLOT_ID=15 bash scenarios/apa/run.sh
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    Town04|Town15)
      MAP="$1"
      shift
      if [ "${1:-}" != "" ] && [[ "${1}" =~ ^[0-9]+$ ]]; then
        SLOT_ID="$1"
        shift
      fi
      ;;
    --map)
      MAP="$2"
      shift 2
      ;;
    --slot)
      SLOT_ID="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[APA] unknown arg: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

mkdir -p "${GAASD_CARLA_LOG_DIR}"
BASE_CONFIG_PATH="${BASE_CONFIG_PATH}" CONFIG_PATH="${CONFIG_PATH}" MAP="${MAP}" SLOT_ID="${SLOT_ID}" SCENARIO_DIR="${SCENARIO_DIR}" \
  "${PYTHON_BIN}" - <<'PY'
import json
import os
import sys
from pathlib import Path

scenario_dir = Path(os.environ["SCENARIO_DIR"])
sys.path.insert(0, str(scenario_dir.parents[1] / "tools" / "carla_bridge"))
from apa_parking_map import parking_map_preset

base_path = Path(os.environ["BASE_CONFIG_PATH"])
config_path = Path(os.environ["CONFIG_PATH"])
data = json.loads(base_path.read_text(encoding="utf-8"))
map_name = os.environ["MAP"]
_, slot_file, default_slot_id = parking_map_preset(map_name)
slot_id = int(os.environ["SLOT_ID"]) if os.environ["SLOT_ID"] else default_slot_id
data.setdefault("carla", {})["map_name"] = map_name
data.setdefault("carla", {})["root"] = os.environ.get("CARLA_ROOT", "/opt/carla")
data.setdefault("apa", {})["slot_id"] = slot_id
data["apa"]["slot_file"] = str(scenario_dir / slot_file)
config_path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY

cd "${REPO_ROOT}"
echo "[APA bridge-only] CARLA_ROOT=${CARLA_ROOT}"
echo "[APA bridge-only] CONFIG_PATH=${CONFIG_PATH}"
echo "[APA bridge-only] logs=${GAASD_CARLA_LOG_DIR}"
bash "${REPO_ROOT}/tools/carla_bridge/start-apa-carla.sh" --config "${CONFIG_PATH}"

echo "[APA bridge-only] starting scenario panel"
bash "${REPO_ROOT}/tools/gaasd_scenario_panel/start-panel.sh"
echo "[APA bridge-only] ready: http://127.0.0.1:${GAASD_PANEL_PORT}"
