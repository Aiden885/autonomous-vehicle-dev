#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
export CARLA_ROOT="${CARLA_ROOT:-${HOME}/snap/code/app/carla-package}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"
BASE_CONFIG_PATH="${SCENARIO_DIR}/bridge_config.json"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/lks3-codegen-carla/carla}"
export CONFIG_PATH="${GAASD_CARLA_LOG_DIR}/bridge_config.runtime.json"
export CARLA_FORCE_NVIDIA_VULKAN="${CARLA_FORCE_NVIDIA_VULKAN:-1}"
export CARLA_EXTRA_ARGS="${CARLA_EXTRA_ARGS:--windowed -Resx=800 -Resy=600 -fps=20 -nosound -graphicsadapter=0 -norhithread}"

PANGU_BUILD_ROOT="${PANGU_BUILD_ROOT:-${HOME}/.cache/gaasd-pangu/lks3_codegen_build}"
PANGU_INSTALL_DIR="${PANGU_INSTALL_DIR:-${PANGU_BUILD_ROOT}/install}"
PANGU_THIRD_SETUP="${PANGU_THIRD_SETUP:-${PANGU_BUILD_ROOT}/dependencies/thirdparty/X86/setup.bash}"
PANGU_DOCKER_IMAGE="${PANGU_DOCKER_IMAGE:-docker.cbdes.cn:8080/cbdes/x86:latest}"
PANGU_CONTAINER_NAME="${PANGU_CONTAINER_NAME:-lks3_codegen_carla}"
PANGU_APP_NAME="${PANGU_APP_NAME:-app_empty}"
PANGU_MAIN_PROCESS="process_empty_6f47e11e"
PANGU_INPUT_PROCESS="LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d"
PANGU_OUTPUT_PROCESS="LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d"
PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/lks3-codegen-carla/pangu}"
RUNTIME_PROJECT_SOURCE="${SCENARIO_DIR}/runtime_project_src"

LKS_MAP_NAME="${LKS_MAP_NAME:-Town04}"
LKS_REFERENCE_X="${LKS_REFERENCE_X:--511.738}"
LKS_REFERENCE_Y="${LKS_REFERENCE_Y:-242.657}"
LKS_REFERENCE_Z="${LKS_REFERENCE_Z:-0.5}"
LKS_INITIAL_OFFSET_M="${LKS_INITIAL_OFFSET_M:-0.0}"
LKS_TARGET_SPEED_MPS="${LKS_TARGET_SPEED_MPS:-4.0}"
LKS_BOOST_SPEED_MPS="${LKS_BOOST_SPEED_MPS:-1.5}"
LKS_BOOST_DURATION_SEC="${LKS_BOOST_DURATION_SEC:-3.0}"
LKS_RECORD_DURATION_SEC="${LKS_RECORD_DURATION_SEC:-180}"
LKS_RESULT_DIR="${LKS_RESULT_DIR:-/tmp/lks3-codegen-carla/results}"
PANGU_READY_TIMEOUT_SEC="${PANGU_READY_TIMEOUT_SEC:-30}"

LKS_PARAM_L0="${LKS_PARAM_L0:-5.0}"
LKS_PARAM_RT="${LKS_PARAM_RT:-0.5}"
LKS_PARAM_R_ALPHA="${LKS_PARAM_R_ALPHA:-0.666667}"
LKS_PARAM_CURVATURE_THRESHOLD="${LKS_PARAM_CURVATURE_THRESHOLD:-0.0001}"
LKS_PARAM_NEAR_PREVIEW_DISTANCE="${LKS_PARAM_NEAR_PREVIEW_DISTANCE:-0.5}"
LKS_PARAM_W1="${LKS_PARAM_W1:-0.2}"
LKS_PARAM_W2="${LKS_PARAM_W2:-0.3}"
LKS_PARAM_W3="${LKS_PARAM_W3:-0.5}"
LKS_PARAM_KP="${LKS_PARAM_KP:-0.09}"
LKS_PARAM_STEER_SCALE="${LKS_PARAM_STEER_SCALE:-0.6}"
LKS_PARAM_AY_MAX="${LKS_PARAM_AY_MAX:-3.0}"
LKS_PARAM_V_MIN="${LKS_PARAM_V_MIN:-1.0}"
LKS_PARAM_DRIVER_STEER_THRESHOLD="${LKS_PARAM_DRIVER_STEER_THRESHOLD:-0.1}"
LKS_PARAM_WHEEL_BASE="${LKS_PARAM_WHEEL_BASE:-2.9}"
LKS_PARAM_FRONT_WHEEL_MAX_RAD="${LKS_PARAM_FRONT_WHEEL_MAX_RAD:-0.5236}"
PANGU_ENV_FILE="${PANGU_LOG_DIR}/lks_runtime.env"

require_file() {
  if [[ ! -e "$1" ]]; then
    echo "[LKS scenario] missing: $1" >&2
    exit 1
  fi
}

ensure_build() {
  local missing=0
  local library
  for library in \
    "libmodule_empty_a807407f_21e6_442d_a6e4_13a788ec666d_core.so" \
    "libmodule_empty_a807407f_21e6_442d_a6e4_13a788ec666d.so" \
    "libLksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d.so" \
    "libLksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d.so"; do
    [[ -f "${PANGU_INSTALL_DIR}/lib/${library}" ]] || missing=1
  done
  if [[ -f "${PANGU_INSTALL_DIR}/lib/libmodule_empty_a807407f_21e6_442d_a6e4_13a788ec666d_core.so" ]] &&
     ! grep -a -q "LKS_PARAM_FRONT_WHEEL_MAX_RAD" "${PANGU_INSTALL_DIR}/lib/libmodule_empty_a807407f_21e6_442d_a6e4_13a788ec666d_core.so"; then
    missing=1
  fi
  if [[ "${missing}" == "1" ]]; then
    echo "[LKS scenario] latest lks3 runtime missing; building it first"
    PANGU_PROJECT_SOURCE="${RUNTIME_PROJECT_SOURCE}" \
      PANGU_BUILD_ROOT="${PANGU_BUILD_ROOT}" PANGU_DOCKER_IMAGE="${PANGU_DOCKER_IMAGE}" \
      bash "${REPO_ROOT}/tools/pangu_generated_closed_loop/build_generated_project.sh" lks3
  fi
}

ensure_local_soc_name() {
  local app_cfg="${PANGU_INSTALL_DIR}/conf/app_module/${PANGU_APP_NAME}.pt"
  local machine_cfg="${PANGU_INSTALL_DIR}/conf/global_conf/icvos_machine.pt"
  local local_soc
  local_soc="$(sed -n 's/.*local_machine_name: "\([^"]*\)".*/\1/p' "${machine_cfg}" | head -1)"
  [[ -n "${local_soc}" ]] || { echo "[LKS scenario] missing local_machine_name" >&2; exit 1; }
  sed -i "s/soc_name: \"[^\"]*\"/soc_name: \"${local_soc}\"/" "${app_cfg}"
}

wait_processes() {
  local deadline=$((SECONDS + PANGU_READY_TIMEOUT_SEC))
  local process
  while (( SECONDS < deadline )); do
    local ready=1
    for process in "${PANGU_MAIN_PROCESS}" "${PANGU_INPUT_PROCESS}" "${PANGU_OUTPUT_PROCESS}"; do
      docker exec "${PANGU_CONTAINER_NAME}" bash -lc \
        "pgrep -f 'dataflow_runner.*--process_name=${process}' >/dev/null" 2>/dev/null || ready=0
    done
    if [[ "${ready}" == "1" ]]; then
      echo "[LKS scenario] all three lks3 Pangu processes are running"
      return 0
    fi
    sleep 0.5
  done
  docker logs "${PANGU_CONTAINER_NAME}" 2>&1 | tail -120 || true
  echo "[LKS scenario] lks3 Pangu processes did not become ready" >&2
  return 1
}

mkdir -p "${PANGU_LOG_DIR}" "${GAASD_CARLA_LOG_DIR}" "${LKS_RESULT_DIR}"
BASE_CONFIG_PATH="${BASE_CONFIG_PATH}" CONFIG_PATH="${CONFIG_PATH}" \
  LKS_MAP_NAME="${LKS_MAP_NAME}" CARLA_ROOT="${CARLA_ROOT}" \
  "${PYTHON_BIN}" -c 'import json, os; from pathlib import Path; data=json.loads(Path(os.environ["BASE_CONFIG_PATH"]).read_text()); root=str(Path(os.environ["CARLA_ROOT"]).expanduser()); data["carla"]["root"]=root; data["carla"]["python_api_paths"]=[str(Path(root) / "PythonAPI/carla")]; data["carla"]["map_name"]=os.environ["LKS_MAP_NAME"]; Path(os.environ["CONFIG_PATH"]).write_text(json.dumps(data, indent=2)+"\n")'

ensure_build
require_file "${PANGU_INSTALL_DIR}/setup.bash"
require_file "${PANGU_INSTALL_DIR}/run.sh"

if [[ "${CLEAN_START:-1}" == "1" ]]; then
  bash "${SCENARIO_DIR}/stop.sh" >/dev/null 2>&1 || true
fi

CARLA_USER_SETTINGS="${HOME}/.config/Epic/CarlaUE4/Saved/Config/LinuxNoEditor/GameUserSettings.ini"
if [[ -f "${CARLA_USER_SETTINGS}" ]]; then
  cp "${CARLA_USER_SETTINGS}" "${GAASD_CARLA_LOG_DIR}/GameUserSettings.ini.bak" || true
  rm -f "${CARLA_USER_SETTINGS}"
fi

echo "[LKS scenario] starting CARLA and Bridge for latest lks3 generated code"
bash "${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh" \
  --config "${CONFIG_PATH}" \
  --ego-spawn-index 0 \
  --no-lead \
  --follow-spectator \
  --spectator-back 8 \
  --spectator-up 6 \
  --spectator-pitch -25 \
  --no-probe

"${PYTHON_BIN}" "${REPO_ROOT}/tools/carla_bridge/reset-lks-reference-scene.py" \
  --carla-root "${CARLA_ROOT}" \
  --expected-map "${LKS_MAP_NAME}" \
  --reference-x "${LKS_REFERENCE_X}" \
  --reference-y "${LKS_REFERENCE_Y}" \
  --ego-x "${LKS_REFERENCE_X}" \
  --ego-y "${LKS_REFERENCE_Y}" \
  --ego-z "${LKS_REFERENCE_Z}" \
  --lateral-offset-m "${LKS_INITIAL_OFFSET_M}"

nohup setsid "${PYTHON_BIN}" "${REPO_ROOT}/tools/pangu_lks_closed_loop/record_lks_run.py" \
  --endpoint tcp://127.0.0.1:5701 \
  --duration-sec "${LKS_RECORD_DURATION_SEC}" \
  --output-dir "${LKS_RESULT_DIR}" \
  --carla-root "${CARLA_ROOT}" \
  >"${PANGU_LOG_DIR}/recorder.log" 2>&1 </dev/null &
printf '%s\n' "$!" >"${PANGU_LOG_DIR}/recorder.pid"

ensure_local_soc_name
sed -i "s/target_speed_mps: [0-9.]*/target_speed_mps: ${LKS_TARGET_SPEED_MPS}/" \
  "${PANGU_INSTALL_DIR}/conf/node_module/module_empty_a807407f_21e6_442d_a6e4_13a788ec666d/${PANGU_OUTPUT_PROCESS}.pt"

printf '%s\n' \
  "LKS_PARAM_L0=${LKS_PARAM_L0}" \
  "LKS_PARAM_RT=${LKS_PARAM_RT}" \
  "LKS_PARAM_R_ALPHA=${LKS_PARAM_R_ALPHA}" \
  "LKS_PARAM_CURVATURE_THRESHOLD=${LKS_PARAM_CURVATURE_THRESHOLD}" \
  "LKS_PARAM_NEAR_PREVIEW_DISTANCE=${LKS_PARAM_NEAR_PREVIEW_DISTANCE}" \
  "LKS_PARAM_W1=${LKS_PARAM_W1}" \
  "LKS_PARAM_W2=${LKS_PARAM_W2}" \
  "LKS_PARAM_W3=${LKS_PARAM_W3}" \
  "LKS_PARAM_KP=${LKS_PARAM_KP}" \
  "LKS_PARAM_STEER_SCALE=${LKS_PARAM_STEER_SCALE}" \
  "LKS_PARAM_AY_MAX=${LKS_PARAM_AY_MAX}" \
  "LKS_PARAM_V_MIN=${LKS_PARAM_V_MIN}" \
  "LKS_PARAM_DRIVER_STEER_THRESHOLD=${LKS_PARAM_DRIVER_STEER_THRESHOLD}" \
  "LKS_PARAM_WHEEL_BASE=${LKS_PARAM_WHEEL_BASE}" \
  "LKS_PARAM_FRONT_WHEEL_MAX_RAD=${LKS_PARAM_FRONT_WHEEL_MAX_RAD}" \
  >"${PANGU_ENV_FILE}"

echo "[LKS scenario] scene parameters: targetSpeed=${LKS_TARGET_SPEED_MPS} offset=${LKS_INITIAL_OFFSET_M} record=${LKS_RECORD_DURATION_SEC}s boost=${LKS_BOOST_SPEED_MPS}m/s@${LKS_BOOST_DURATION_SEC}s"
echo "[LKS scenario] preview parameters: l0=${LKS_PARAM_L0} rt=${LKS_PARAM_RT} rAlpha=${LKS_PARAM_R_ALPHA} curvatureThreshold=${LKS_PARAM_CURVATURE_THRESHOLD} near=${LKS_PARAM_NEAR_PREVIEW_DISTANCE}"
echo "[LKS scenario] control parameters: weights=${LKS_PARAM_W1},${LKS_PARAM_W2},${LKS_PARAM_W3} Kp=${LKS_PARAM_KP} steerScale=${LKS_PARAM_STEER_SCALE} ayMax=${LKS_PARAM_AY_MAX}"
echo "[LKS scenario] decision/calibration: vMin=${LKS_PARAM_V_MIN} driverThreshold=${LKS_PARAM_DRIVER_STEER_THRESHOLD} wheelBase=${LKS_PARAM_WHEEL_BASE} frontWheelMaxRad=${LKS_PARAM_FRONT_WHEEL_MAX_RAD}"

docker rm -f "${PANGU_CONTAINER_NAME}" >/dev/null 2>&1 || true
docker run -d \
  --name "${PANGU_CONTAINER_NAME}" \
  --privileged \
  --net=host \
  --shm-size=8193m \
  --env-file "${PANGU_ENV_FILE}" \
  -v /home:/home \
  -v /data:/data \
  -v /tmp:/tmp \
  "${PANGU_DOCKER_IMAGE}" \
  /bin/bash -lc "
    set -e
    source '${PANGU_THIRD_SETUP}' >/dev/null 2>&1 || true
    source '${PANGU_INSTALL_DIR}/setup.bash' >/dev/null 2>&1 || true
    bash '${PANGU_INSTALL_DIR}/run.sh' '${PANGU_APP_NAME}' \
      '${PANGU_MAIN_PROCESS}' '${PANGU_INPUT_PROCESS}' '${PANGU_OUTPUT_PROCESS}' -nohup
    tail -f /dev/null
  " >"${PANGU_LOG_DIR}/container_id.txt"

wait_processes
echo "[LKS scenario] boosting ego above generated lks_vMin=1"
"${PYTHON_BIN}" "${REPO_ROOT}/tools/carla_bridge/boost-ego-speed.py" \
  --carla-root "${CARLA_ROOT}" \
  --speed-mps "${LKS_BOOST_SPEED_MPS}" \
  --duration-sec "${LKS_BOOST_DURATION_SEC}" \
  >"${PANGU_LOG_DIR}/boost_ego.log" 2>&1

echo "[LKS scenario] ready: latest lks3 generated-code LKS is connected"
