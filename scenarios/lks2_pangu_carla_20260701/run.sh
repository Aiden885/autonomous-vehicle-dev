#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
export CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"
BASE_CONFIG_PATH="${SCENARIO_DIR}/bridge_config.json"
export CONFIG_PATH="${GAASD_CARLA_LOG_DIR:-/tmp/lks2-pangu-carla/carla}/bridge_config.runtime.json"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/lks2-pangu-carla/carla}"
export CARLA_FORCE_NVIDIA_VULKAN="${CARLA_FORCE_NVIDIA_VULKAN:-1}"
export CARLA_EXTRA_ARGS="${CARLA_EXTRA_ARGS:--windowed -Resx=800 -Resy=600 -fps=20 -nosound -graphicsadapter=0 -norhithread}"
CARLA_RESET_RENDER_SETTINGS="${CARLA_RESET_RENDER_SETTINGS:-1}"
LKS_FOLLOW_SPECTATOR="${LKS_FOLLOW_SPECTATOR:-1}"
LKS_WATCH_CAMERA="${LKS_WATCH_CAMERA:-0}"

PANGU_BUILD_ROOT="${PANGU_BUILD_ROOT:-${HOME}/.cache/gaasd-pangu/lks2_codegen_build}"
PANGU_INSTALL_DIR="${PANGU_INSTALL_DIR:-${PANGU_BUILD_ROOT}/install}"
PANGU_IMAGE="${PANGU_DOCKER_IMAGE:-docker.cbdes.cn:8080/cbdes/x86:pangu-2.0.5}"
PANGU_CONTAINER_NAME="${PANGU_CONTAINER_NAME:-lks2_pangu_carla}"
PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/lks2-pangu-carla/pangu}"
LKS_INITIAL_OFFSET_M="${LKS_INITIAL_OFFSET_M:-0.0}"
LKS_INITIAL_HEADING_ERROR_DEG="${LKS_INITIAL_HEADING_ERROR_DEG:-0.0}"
LKS_MAP_NAME="${LKS_MAP_NAME:-Town04}"
LKS_REFERENCE_X="${LKS_REFERENCE_X:--511.738}"
LKS_REFERENCE_Y="${LKS_REFERENCE_Y:-242.657}"
LKS_REFERENCE_Z="${LKS_REFERENCE_Z:-0.5}"
LKS_EGO_X="${LKS_EGO_X:-${LKS_REFERENCE_X}}"
LKS_EGO_Y="${LKS_EGO_Y:-${LKS_REFERENCE_Y}}"
LKS_EGO_Z="${LKS_EGO_Z:-${LKS_REFERENCE_Z}}"
LKS_TARGET_SPEED_MPS="${LKS_TARGET_SPEED_MPS:-4.0}"
LKS_BOOST_SPEED_MPS="${LKS_BOOST_SPEED_MPS:-1.5}"
LKS_USE_BOOST="${LKS_USE_BOOST:-1}"
LKS_RECORD_DURATION_SEC="${LKS_RECORD_DURATION_SEC:-180}"
LKS_PARAM_L0="${LKS_PARAM_L0:-5.0}"
LKS_PARAM_RT="${LKS_PARAM_RT:-0.5}"
LKS_PARAM_R_ALPHA="${LKS_PARAM_R_ALPHA:-0.6666667}"
LKS_PARAM_CURVATURE_THRESHOLD="${LKS_PARAM_CURVATURE_THRESHOLD:-0.001}"
LKS_PARAM_NEAR_PREVIEW_DISTANCE="${LKS_PARAM_NEAR_PREVIEW_DISTANCE:-0.5}"
LKS_PARAM_W1="${LKS_PARAM_W1:-0.2}"
LKS_PARAM_W2="${LKS_PARAM_W2:-0.3}"
LKS_PARAM_W3="${LKS_PARAM_W3:-0.5}"
LKS_PARAM_KP="${LKS_PARAM_KP:-0.09}"
LKS_PARAM_STEER_SCALE="${LKS_PARAM_STEER_SCALE:-0.6}"
LKS_PARAM_AY_MAX="${LKS_PARAM_AY_MAX:-3.0}"
LKS_PARAM_V_MIN="${LKS_PARAM_V_MIN:-1.0}"
LKS_PARAM_DRIVER_STEER_THRESHOLD="${LKS_PARAM_DRIVER_STEER_THRESHOLD:-0.1}"
LKS_RESULT_DIR="${LKS_RESULT_DIR:-/tmp/lks2-pangu-carla/results}"
LKS_RECORDER_PID_FILE="${PANGU_LOG_DIR}/recorder.pid"
LKS_RECORDER_LOG_FILE="${PANGU_LOG_DIR}/recorder.log"

mkdir -p "${PANGU_LOG_DIR}"
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
if [[ "${CLEAN_START:-1}" == "1" ]]; then
  bash "${SCENARIO_DIR}/stop.sh" >/dev/null 2>&1 || true
fi
if [[ "${CARLA_RESET_RENDER_SETTINGS}" == "1" ]]; then
  CARLA_USER_SETTINGS="${HOME}/.config/Epic/CarlaUE4/Saved/Config/LinuxNoEditor/GameUserSettings.ini"
  if [[ -f "${CARLA_USER_SETTINGS}" ]]; then
    mkdir -p "${GAASD_CARLA_LOG_DIR}"
    cp "${CARLA_USER_SETTINGS}" "${GAASD_CARLA_LOG_DIR}/GameUserSettings.ini.bak.$(date +%Y%m%d_%H%M%S)" || true
    rm -f "${CARLA_USER_SETTINGS}"
    echo "[LKS scenario] reset CARLA GameUserSettings.ini; backup saved under ${GAASD_CARLA_LOG_DIR}"
  fi
fi
if [[ ! -f "${PANGU_INSTALL_DIR}/lib/libLKSModule.so" ]]; then
  echo "[LKS scenario] Pangu module missing; building it first"
  PANGU_BUILD_ROOT="${PANGU_BUILD_ROOT}" \
    bash "${REPO_ROOT}/tools/pangu_lks_closed_loop/build_pangu_module.sh"
fi

echo "[LKS scenario] starting CARLA and Bridge"
echo "[LKS scenario] CARLA_EXTRA_ARGS=${CARLA_EXTRA_ARGS}"
echo "[LKS scenario] CARLA_FORCE_NVIDIA_VULKAN=${CARLA_FORCE_NVIDIA_VULKAN} VK_ICD_FILENAMES=${VK_ICD_FILENAMES:-auto}"
CARLA_START_ARGS=(
  --config "${CONFIG_PATH}"
  --ego-spawn-index 0
  --no-lead
  --spectator-back 8
  --spectator-up 6
  --spectator-pitch -25
  --no-probe
)
if [[ "${LKS_FOLLOW_SPECTATOR}" == "1" ]]; then
  CARLA_START_ARGS+=(--follow-spectator)
else
  CARLA_START_ARGS+=(--no-follow-spectator)
fi
if [[ "${LKS_WATCH_CAMERA}" == "1" ]]; then
  CARLA_START_ARGS+=(--watch-camera)
fi
bash "${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh" "${CARLA_START_ARGS[@]}"

"${PYTHON_BIN}" "${REPO_ROOT}/tools/carla_bridge/reset-lks-reference-scene.py" \
  --carla-root "${CARLA_ROOT}" \
  --expected-map "${LKS_MAP_NAME}" \
  --reference-x "${LKS_REFERENCE_X}" \
  --reference-y "${LKS_REFERENCE_Y}" \
  --ego-x "${LKS_EGO_X}" \
  --ego-y "${LKS_EGO_Y}" \
  --ego-z "${LKS_EGO_Z}" \
  --lateral-offset-m "${LKS_INITIAL_OFFSET_M}"

mkdir -p "${LKS_RESULT_DIR}"
nohup setsid "${PYTHON_BIN}" "${REPO_ROOT}/tools/pangu_lks_closed_loop/record_lks_run.py" \
  --endpoint tcp://127.0.0.1:5701 \
  --duration-sec "${LKS_RECORD_DURATION_SEC}" \
  --output-dir "${LKS_RESULT_DIR}" \
  --carla-root "${CARLA_ROOT}" \
  >"${LKS_RECORDER_LOG_FILE}" 2>&1 </dev/null &
printf '%s\n' "$!" >"${LKS_RECORDER_PID_FILE}"
echo "[LKS scenario] recorder started pid=$! duration=${LKS_RECORD_DURATION_SEC}s result=${LKS_RESULT_DIR}"

docker rm -f "${PANGU_CONTAINER_NAME}" >/dev/null 2>&1 || true
docker run -d \
  --name "${PANGU_CONTAINER_NAME}" \
  --privileged \
  --net=host \
  --shm-size=8193m \
  -v /home:/home -v /data:/data -v /tmp:/tmp \
  -e LKS_TARGET_SPEED_MPS="${LKS_TARGET_SPEED_MPS}" \
  -e LKS_PARAM_L0="${LKS_PARAM_L0}" \
  -e LKS_PARAM_RT="${LKS_PARAM_RT}" \
  -e LKS_PARAM_R_ALPHA="${LKS_PARAM_R_ALPHA}" \
  -e LKS_PARAM_CURVATURE_THRESHOLD="${LKS_PARAM_CURVATURE_THRESHOLD}" \
  -e LKS_PARAM_NEAR_PREVIEW_DISTANCE="${LKS_PARAM_NEAR_PREVIEW_DISTANCE}" \
  -e LKS_PARAM_W1="${LKS_PARAM_W1}" \
  -e LKS_PARAM_W2="${LKS_PARAM_W2}" \
  -e LKS_PARAM_W3="${LKS_PARAM_W3}" \
  -e LKS_PARAM_KP="${LKS_PARAM_KP}" \
  -e LKS_PARAM_STEER_SCALE="${LKS_PARAM_STEER_SCALE}" \
  -e LKS_PARAM_AY_MAX="${LKS_PARAM_AY_MAX}" \
  -e LKS_PARAM_V_MIN="${LKS_PARAM_V_MIN}" \
  -e LKS_PARAM_DRIVER_STEER_THRESHOLD="${LKS_PARAM_DRIVER_STEER_THRESHOLD}" \
  "${PANGU_IMAGE}" /bin/bash -lc "
    set -e
    ln -sfn '${PANGU_INSTALL_DIR}/image/conf' '${PANGU_INSTALL_DIR}/conf'
    source '${PANGU_BUILD_ROOT}/dependencies/thirdparty/X86/setup.bash' >/dev/null 2>&1 || true
    source '${PANGU_INSTALL_DIR}/setup.bash' >/dev/null 2>&1 || true
    sed -i 's/target_speed_mps: [0-9.]*/target_speed_mps: ${LKS_TARGET_SPEED_MPS}/' \
      '${PANGU_INSTALL_DIR}/conf/node_module/LKSModule/ZmqBridgeModule.pt'
    bash '${PANGU_INSTALL_DIR}/run.sh' app_empty LKSModule ZmqBridgeModule -nohup
    tail -f /dev/null
  " >"${PANGU_LOG_DIR}/container_id.txt"

deadline=$((SECONDS + 35))
while (( SECONDS < deadline )); do
  if [[ "$(docker inspect -f '{{.State.Running}}' "${PANGU_CONTAINER_NAME}" 2>/dev/null || true)" != "true" ]]; then
    echo "[LKS scenario] Pangu container exited before processes became ready" >&2
    break
  fi
  if docker exec "${PANGU_CONTAINER_NAME}" bash -lc \
    "ps -eo args | grep '^dataflow_runner .*--process_name=LKSModule' >/dev/null && ps -eo args | grep '^dataflow_runner .*--process_name=ZmqBridgeModule' >/dev/null" \
    2>/dev/null; then
    echo "[LKS scenario] Pangu LKSModule and ZmqBridgeModule are running"
    break
  fi
  sleep 0.5
done
docker logs "${PANGU_CONTAINER_NAME}" >"${PANGU_LOG_DIR}/docker_start.log" 2>&1 || true
if ! docker ps --format '{{.Names}}' | grep -Fxq "${PANGU_CONTAINER_NAME}"; then
  tail -120 "${PANGU_LOG_DIR}/docker_start.log" || true
  echo "[LKS scenario] Pangu container exited" >&2
  exit 1
fi
if ! docker exec "${PANGU_CONTAINER_NAME}" bash -lc \
  "ps -eo args | grep '^dataflow_runner .*--process_name=LKSModule' >/dev/null && ps -eo args | grep '^dataflow_runner .*--process_name=ZmqBridgeModule' >/dev/null"; then
  tail -120 "${PANGU_LOG_DIR}/docker_start.log" || true
  echo "[LKS scenario] Pangu processes did not become ready" >&2
  exit 1
fi

if [[ "${LKS_USE_BOOST}" == "1" ]]; then
  echo "[LKS scenario] applying optional ego speed boost"
  "${PYTHON_BIN}" "${REPO_ROOT}/tools/carla_bridge/boost-ego-speed.py" \
    --carla-root "${CARLA_ROOT}" \
    --speed-mps "${LKS_BOOST_SPEED_MPS}" \
    --duration-sec 3 \
    >"${PANGU_LOG_DIR}/boost_ego.log" 2>&1
fi

echo "[LKS scenario] parameters: target=${LKS_TARGET_SPEED_MPS}m/s offset=${LKS_INITIAL_OFFSET_M}m duration=${LKS_RECORD_DURATION_SEC}s"
echo "[LKS scenario] tuning: l0=${LKS_PARAM_L0} rt=${LKS_PARAM_RT} rAlpha=${LKS_PARAM_R_ALPHA} Kp=${LKS_PARAM_KP} steerScale=${LKS_PARAM_STEER_SCALE} ayMax=${LKS_PARAM_AY_MAX}"
echo "[LKS scenario] ready: recorder covers straight and curve sections"
