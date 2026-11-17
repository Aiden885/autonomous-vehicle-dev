#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

export CARLA_ROOT="${CARLA_ROOT:-/opt/carla}"
export CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
export CARLA_PORT="${CARLA_PORT:-2000}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"
export BRIDGE_SCRIPT="${BRIDGE_SCRIPT:-carla_bridge_lks.py}"
BASE_CONFIG_PATH="${CONFIG_PATH:-${SCENARIO_DIR}/bridge_config.json}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/lks3-codegen-carla/carla}"
export GAASD_PANEL_PORT="${GAASD_PANEL_PORT:-8088}"

LKS_MAP_NAME="${LKS_MAP_NAME:-Town04}"
LKS_REFERENCE_X="${LKS_REFERENCE_X:--511.738}"
LKS_REFERENCE_Y="${LKS_REFERENCE_Y:-242.657}"
LKS_REFERENCE_Z="${LKS_REFERENCE_Z:-0.5}"
LKS_INITIAL_OFFSET_M="${LKS_INITIAL_OFFSET_M:-0.0}"
LKS_RECORD_DURATION_SEC="${LKS_RECORD_DURATION_SEC:-180}"
LKS_RESULT_DIR="${LKS_RESULT_DIR:-/tmp/lks3-codegen-carla/results}"

SPECTATOR_BACK="${SPECTATOR_BACK:-8}"
SPECTATOR_UP="${SPECTATOR_UP:-6}"
SPECTATOR_PITCH="${SPECTATOR_PITCH:--25}"
WATCH_CAMERA="${WATCH_CAMERA:-0}"
PROBE_BRIDGE="${PROBE_BRIDGE:-0}"
START_PANEL="${START_PANEL:-1}"
START_RECORDER="${START_RECORDER:-1}"
RESTART_BRIDGE="${RESTART_BRIDGE:-1}"
PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/lks3-codegen-carla/pangu}"
export WAIT_EGO_SEC="${WAIT_EGO_SEC:-90}"
RUNTIME_CONFIG_PATH="${GAASD_CARLA_LOG_DIR}/bridge_config.runtime.json"

require_file() {
  if [[ ! -e "$1" ]]; then
    echo "[LKS bridge-only] missing: $1" >&2
    exit 1
  fi
}

check_carla_ready() {
  "${PYTHON_BIN}" - "${CARLA_ROOT}" "${CARLA_HOST}" "${CARLA_PORT}" <<'PY'
import glob
import os
import sys

root = sys.argv[1]
host = sys.argv[2]
port = int(sys.argv[3])
paths = [os.path.join(root, "PythonAPI", "carla")]
paths.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*-py3*.egg")))
for path in reversed(paths):
    if path and os.path.exists(path) and path not in sys.path:
        sys.path.insert(0, path)

import carla  # type: ignore

client = carla.Client(host, port)
client.set_timeout(10.0)
world = client.get_world()
_ = world.get_map().name
PY
}

stop_pid_file() {
  local label="$1"
  local pid_file="$2"
  local pid=""
  if [[ ! -f "${pid_file}" ]]; then
    return 0
  fi
  pid="$(cat "${pid_file}" 2>/dev/null || true)"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    echo "[LKS bridge-only] stopped stale ${label} pid=${pid}"
  fi
  rm -f "${pid_file}"
}

stop_by_pattern() {
  local label="$1"
  local pattern="$2"
  local pids
  pids="$(pgrep -f "${pattern}" || true)"
  for pid in ${pids}; do
    if [[ "${pid}" != "$$" ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
      echo "[LKS bridge-only] stopped stale ${label} pid=${pid}"
    fi
  done
}

stop_bridge_stack_only() {
  stop_pid_file "recorder" "${PANGU_LOG_DIR}/recorder.pid"
  stop_pid_file "watch camera" "${GAASD_CARLA_LOG_DIR}/watch-camera.pid"
  stop_by_pattern "watch camera" "tools/carla_bridge/watch-carla.py"
  stop_pid_file "spectator follow" "${GAASD_CARLA_LOG_DIR}/spectator.pid"
  stop_by_pattern "spectator follow" "tools/carla_bridge/set-spectator-follow.py"
  stop_pid_file "bridge" "${GAASD_CARLA_LOG_DIR}/bridge.pid"
  stop_by_pattern "bridge" "tools/carla_bridge/carla_bridge_lks.py"
  stop_by_pattern "legacy bridge" "tools/carla_bridge/carla_bridge.py"
}

prepare_runtime_bridge_config() {
  mkdir -p "${GAASD_CARLA_LOG_DIR}"
  "${PYTHON_BIN}" - \
    "${BASE_CONFIG_PATH}" \
    "${RUNTIME_CONFIG_PATH}" \
    "${CARLA_ROOT}" \
    "${LKS_MAP_NAME}" \
    "${LKS_REFERENCE_X}" "${LKS_REFERENCE_Y}" "${LKS_REFERENCE_Z}" \
    "${LKS_INITIAL_OFFSET_M}" <<'PY'
import json
import sys
from pathlib import Path

source_path = Path(sys.argv[1])
output_path = Path(sys.argv[2])
carla_root = str(Path(sys.argv[3]).expanduser())
map_name = sys.argv[4]
config = json.loads(source_path.read_text(encoding="utf-8"))
carla = config.setdefault("carla", {})
carla["root"] = carla_root
carla["python_api_paths"] = [str(Path(carla_root) / "PythonAPI/carla")]
carla["map_name"] = map_name
carla["ego_reference_start"] = {
    "x": float(sys.argv[5]), "y": float(sys.argv[6]), "z": float(sys.argv[7]),
    "lateral_offset_m": float(sys.argv[8]),
}
output_path.write_text(
    json.dumps(config, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
PY
  export CONFIG_PATH="${RUNTIME_CONFIG_PATH}"
}

start_recorder() {
  if [[ "${START_RECORDER}" != "1" ]]; then
    return 0
  fi
  mkdir -p "${PANGU_LOG_DIR}" "${LKS_RESULT_DIR}"
  stop_pid_file "recorder" "${PANGU_LOG_DIR}/recorder.pid"
  nohup setsid "${PYTHON_BIN}" "${REPO_ROOT}/tools/pangu_lks_closed_loop/record_lks_run.py" \
    --endpoint tcp://127.0.0.1:5701 \
    --duration-sec "${LKS_RECORD_DURATION_SEC}" \
    --output-dir "${LKS_RESULT_DIR}" \
    --carla-root "${CARLA_ROOT}" \
    >"${PANGU_LOG_DIR}/recorder.log" 2>&1 </dev/null &
  printf '%s\n' "$!" >"${PANGU_LOG_DIR}/recorder.pid"
  echo "[LKS bridge-only] recorder started pid=$(cat "${PANGU_LOG_DIR}/recorder.pid")"
}

require_file "${BASE_CONFIG_PATH}"
require_file "${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh"
require_file "${REPO_ROOT}/tools/gaasd_scenario_panel/start-panel.sh"
command -v "${PYTHON_BIN}" >/dev/null 2>&1 || {
  echo "[LKS bridge-only] python not found: ${PYTHON_BIN}" >&2
  exit 1
}
[[ -d "${CARLA_ROOT}" ]] || {
  echo "[LKS bridge-only] CARLA root not found: ${CARLA_ROOT}" >&2
  exit 1
}

if ! check_carla_ready; then
  echo "[LKS bridge-only] CARLA is not ready at ${CARLA_HOST}:${CARLA_PORT}; start CARLA first." >&2
  exit 1
fi

prepare_runtime_bridge_config

if [[ "${RESTART_BRIDGE}" == "1" ]]; then
  stop_bridge_stack_only
fi

args=(
  --config "${CONFIG_PATH}"
  --carla-root "${CARLA_ROOT}"
  --host "${CARLA_HOST}"
  --port "${CARLA_PORT}"
  --python "${PYTHON_BIN}"
  --ego-spawn-index 0
  --no-lead
  --follow-spectator
  --spectator-back "${SPECTATOR_BACK}"
  --spectator-up "${SPECTATOR_UP}"
  --spectator-pitch "${SPECTATOR_PITCH}"
)

if [[ "${RESTART_BRIDGE}" != "1" ]]; then
  args+=(--reuse-bridge)
fi

if [[ "${WATCH_CAMERA}" == "1" ]]; then
  args+=(--watch-camera)
fi

if [[ "${PROBE_BRIDGE}" != "1" ]]; then
  args+=(--no-probe)
fi

cd "${REPO_ROOT}"
echo "[LKS bridge-only] CARLA_ROOT=${CARLA_ROOT}"
echo "[LKS bridge-only] CONFIG_PATH=${CONFIG_PATH}"
echo "[LKS bridge-only] BRIDGE_SCRIPT=${BRIDGE_SCRIPT}"
echo "[LKS bridge-only] logs=${GAASD_CARLA_LOG_DIR}"
bash "${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh" "${args[@]}"

start_recorder

if [[ "${START_PANEL}" == "1" ]]; then
  echo "[LKS bridge-only] starting scenario panel"
  bash "${REPO_ROOT}/tools/gaasd_scenario_panel/start-panel.sh"
  echo "[LKS bridge-only] panel: http://127.0.0.1:${GAASD_PANEL_PORT}"
fi

echo "[LKS bridge-only] ready"
