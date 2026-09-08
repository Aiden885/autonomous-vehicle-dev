#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

export CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"
export CONFIG_PATH="${SCENARIO_DIR}/bridge_config.json"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/newaccpro4-codegen-carla/carla}"

PANGU_BUILD_ROOT="${PANGU_BUILD_ROOT:-${HOME}/.cache/gaasd-pangu/newaccpro4_codegen_build}"
PANGU_INSTALL_DIR="${PANGU_INSTALL_DIR:-${PANGU_BUILD_ROOT}/install}"
PANGU_THIRD_SETUP="${PANGU_THIRD_SETUP:-${PANGU_BUILD_ROOT}/dependencies/thirdparty/X86/setup.bash}"
PANGU_DOCKER_IMAGE="${PANGU_DOCKER_IMAGE:-docker.cbdes.cn:8080/cbdes/x86:latest}"
PANGU_CONTAINER_NAME="${PANGU_CONTAINER_NAME:-newaccpro4_codegen_carla}"
PANGU_APP_NAME="${PANGU_APP_NAME:-app_empty}"
PANGU_MAIN_PROCESS="process_empty_99756268"
PANGU_INPUT_PROCESS="AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43"
PANGU_OUTPUT_PROCESS="AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43"
PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/newaccpro4-codegen-carla/pangu}"

ACC_RESULTS_DIR="${ACC_RESULTS_DIR:-/tmp/newaccpro4-codegen-carla/results}"
ACC_RECORD_DURATION_SEC="${ACC_RECORD_DURATION_SEC:-180}"
BOOST_SPEED_MPS="${BOOST_SPEED_MPS:-2.0}"
BOOST_DURATION_SEC="${BOOST_DURATION_SEC:-4.0}"
AUTO_START_COMMAND="${AUTO_START_COMMAND:-e}"
PANGU_READY_TIMEOUT_SEC="${PANGU_READY_TIMEOUT_SEC:-30}"

require_file() {
  if [[ ! -e "$1" ]]; then
    echo "[Scenario] missing: $1" >&2
    exit 1
  fi
}

ensure_build() {
  local missing=0
  local library
  for library in \
    "libmodule_empty_b6d54e1b_2577_4492_bcea_dfe707509e43.so" \
    "libAccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.so" \
    "libAccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.so"; do
    [[ -f "${PANGU_INSTALL_DIR}/lib/${library}" ]] || missing=1
  done
  if [[ "${missing}" == "1" ]]; then
    echo "[Scenario] latest newaccpro4 runtime missing; building it first"
    PANGU_BUILD_ROOT="${PANGU_BUILD_ROOT}" PANGU_DOCKER_IMAGE="${PANGU_DOCKER_IMAGE}" \
      bash "${REPO_ROOT}/tools/pangu_generated_closed_loop/build_generated_project.sh" newaccpro4
  fi
}

ensure_local_soc_name() {
  local app_cfg="${PANGU_INSTALL_DIR}/conf/app_module/${PANGU_APP_NAME}.pt"
  local machine_cfg="${PANGU_INSTALL_DIR}/conf/global_conf/icvos_machine.pt"
  local local_soc
  local_soc="$(sed -n 's/.*local_machine_name: "\([^"]*\)".*/\1/p' "${machine_cfg}" | head -1)"
  [[ -n "${local_soc}" ]] || { echo "[Scenario] missing local_machine_name" >&2; exit 1; }
  sed -i "s/soc_name: \"[^\"]*\"/soc_name: \"${local_soc}\"/" "${app_cfg}"
  # Keep generated-code inputs and outputs available for closed-loop diagnosis.
  sed -i 's/to_file: OFF/to_file: ON/g' "${app_cfg}"
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
      echo "[Scenario] all three newaccpro4 Pangu processes are running"
      return 0
    fi
    sleep 0.5
  done
  docker logs "${PANGU_CONTAINER_NAME}" 2>&1 | tail -120 || true
  echo "[Scenario] newaccpro4 Pangu processes did not become ready" >&2
  return 1
}

mkdir -p "${PANGU_LOG_DIR}" "${ACC_RESULTS_DIR}"
require_file "${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh"
require_file "${REPO_ROOT}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py"
require_file "${REPO_ROOT}/tools/pangu_acc_closed_loop/record_acc_run.py"
command -v docker >/dev/null 2>&1 || { echo "[Scenario] docker is required" >&2; exit 1; }

ensure_build
require_file "${PANGU_INSTALL_DIR}/setup.bash"
require_file "${PANGU_INSTALL_DIR}/run.sh"

if [[ "${CLEAN_START:-1}" == "1" ]]; then
  bash "${SCENARIO_DIR}/stop.sh" >/dev/null 2>&1 || true
fi

echo "[Scenario] starting CARLA + Bridge for latest newaccpro4 generated code"
bash "${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh" \
  --config "${CONFIG_PATH}" \
  --ego-spawn-index 198 \
  --lead-distance 25 \
  --lead-speed 2 \
  --lead-placement lane_waypoint \
  --lead-behavior waypoint_pid \
  --follow-spectator \
  --spectator-back 8 \
  --spectator-up 6 \
  --spectator-pitch -25 \
  --no-probe

nohup setsid python3 "${REPO_ROOT}/tools/pangu_acc_closed_loop/record_acc_run.py" \
  --duration "${ACC_RECORD_DURATION_SEC}" \
  --output-dir "${ACC_RESULTS_DIR}" \
  >"${PANGU_LOG_DIR}/recorder.log" 2>&1 </dev/null &
printf '%s\n' "$!" >"${PANGU_LOG_DIR}/recorder.pid"

ensure_local_soc_name
docker rm -f "${PANGU_CONTAINER_NAME}" >/dev/null 2>&1 || true
docker run -d \
  --name "${PANGU_CONTAINER_NAME}" \
  --privileged \
  --net=host \
  --shm-size=8193m \
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
sleep 2

echo "[Scenario] boosting ego above generated minimumOperatingSpeedMps=1"
"${PYTHON_BIN}" "${REPO_ROOT}/tools/carla_bridge/boost-ego-speed.py" \
  --carla-root "${CARLA_ROOT}" \
  --speed-mps "${BOOST_SPEED_MPS}" \
  --duration-sec "${BOOST_DURATION_SEC}" \
  >"${PANGU_LOG_DIR}/boost_ego.log" 2>&1

if [[ -n "${AUTO_START_COMMAND}" ]]; then
  echo "[Scenario] sending ACC engage after ego boost has completed"
  python3 "${REPO_ROOT}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py" \
    --once "${AUTO_START_COMMAND}" >"${PANGU_LOG_DIR}/startup_command.log" 2>&1
fi

echo "[Scenario] ready: latest newaccpro4 generated-code ACC is connected"
