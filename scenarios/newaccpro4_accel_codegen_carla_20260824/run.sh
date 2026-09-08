#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

export CARLA_ROOT="${CARLA_ROOT:-${HOME}/snap/code/app/carla-package}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"
BASE_CONFIG_PATH="${CONFIG_PATH:-${SCENARIO_DIR}/bridge_config.json}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/newaccpro4-accel-codegen-carla/carla}"

PANGU_BUILD_ROOT="${PANGU_BUILD_ROOT:-${HOME}/.cache/gaasd-pangu/newaccpro4_accel_codegen_build}"
PANGU_INSTALL_DIR="${PANGU_INSTALL_DIR:-${PANGU_BUILD_ROOT}/install}"
PANGU_THIRD_SETUP="${PANGU_THIRD_SETUP:-${PANGU_BUILD_ROOT}/dependencies/thirdparty/X86/setup.bash}"
PANGU_DOCKER_IMAGE="${PANGU_DOCKER_IMAGE:-docker.cbdes.cn:8080/cbdes/x86:latest}"
PANGU_CONTAINER_NAME="${PANGU_CONTAINER_NAME:-newaccpro4_accel_codegen_carla}"
PANGU_APP_NAME="${PANGU_APP_NAME:-app_empty}"
PANGU_MAIN_PROCESS="AccProcess"
PANGU_INPUT_PROCESS="AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43"
PANGU_OUTPUT_PROCESS="AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43"
PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/newaccpro4-accel-codegen-carla/pangu}"

ACC_RESULTS_DIR="${ACC_RESULTS_DIR:-/tmp/newaccpro4-accel-codegen-carla/results}"
ACC_RECORD_DURATION_SEC="${ACC_RECORD_DURATION_SEC:-180}"
ACC_LEAD_SPEED_MPS="${ACC_LEAD_SPEED_MPS:-2.0}"
ACC_LEAD_DISTANCE_M="${ACC_LEAD_DISTANCE_M:-30.0}"
ACC_LEAD_STEP_SPEED_MPS="${ACC_LEAD_STEP_SPEED_MPS:-$ACC_LEAD_SPEED_MPS}"
ACC_LEAD_STEP_TIME_SEC="${ACC_LEAD_STEP_TIME_SEC:-0.0}"
ACC_LEAD_MAX_THROTTLE="${ACC_LEAD_MAX_THROTTLE:-0.8}"
ACC_MAX_ACCEL_MPS2="${ACC_MAX_ACCEL_MPS2:-1.5}"
ACC_MAX_BRAKE_MPS2="${ACC_MAX_BRAKE_MPS2:-6.0}"
ACC_MAX_THROTTLE="${ACC_MAX_THROTTLE:-0.35}"
ACC_MAX_BRAKE="${ACC_MAX_BRAKE:-0.7}"
BOOST_SPEED_MPS="${BOOST_SPEED_MPS:-2.0}"
BOOST_DURATION_SEC="${BOOST_DURATION_SEC:-4.0}"
AUTO_START_COMMAND="${AUTO_START_COMMAND:-}"
PANGU_READY_TIMEOUT_SEC="${PANGU_READY_TIMEOUT_SEC:-30}"

RUNTIME_PROJECT_SOURCE="${SCENARIO_DIR}/runtime_project_src"
MODULE_ROOT="${RUNTIME_PROJECT_SOURCE}/modules/module_empty/module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43"
RUNTIME_CONFIG_PATH="${PANGU_LOG_DIR}/bridge_runtime_config.json"

require_file() {
  if [[ ! -e "$1" ]]; then
    echo "[Scenario] missing: $1" >&2
    exit 1
  fi
}

verify_acceleration_boundary() {
  local bridge_source="${MODULE_ROOT}/module_lib/acc_zmq_bridge.cpp"
  local output_source="${MODULE_ROOT}/src/AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.cc"
  local input_config="${MODULE_ROOT}/conf/AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.pt"
  local output_config="${MODULE_ROOT}/conf/AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.pt"
  require_file "${bridge_source}"
  require_file "${output_source}"
  require_file "${input_config}"
  require_file "${output_config}"
  grep -q 'target_accel_mps2' "${bridge_source}" || {
    echo "[Scenario] acceleration ZMQ boundary is missing; code generation may have overwritten the local patch" >&2
    exit 1
  }
  grep -q 'PublishAccelerationControl' "${output_source}" || {
    echo "[Scenario] generated output boundary is not mapped to acceleration control" >&2
    exit 1
  }
  grep -q 'channel: "acc_input"' "${input_config}" || {
    echo "[Scenario] acc_input publisher configuration is missing" >&2
    exit 1
  }
  grep -q 'channel: "acc_output"' "${output_config}" || {
    echo "[Scenario] acc_output subscriber configuration is missing" >&2
    exit 1
  }
}

prepare_runtime_bridge_config() {
  python3 - \
    "${BASE_CONFIG_PATH}" \
    "${RUNTIME_CONFIG_PATH}" \
    "${ACC_MAX_ACCEL_MPS2}" \
    "${ACC_MAX_BRAKE_MPS2}" \
    "${ACC_MAX_THROTTLE}" \
    "${ACC_MAX_BRAKE}" \
    "${CARLA_ROOT}" <<'PY'
import json
import sys
from pathlib import Path

source_path = Path(sys.argv[1])
output_path = Path(sys.argv[2])
config = json.loads(source_path.read_text(encoding="utf-8"))
carla_root = str(Path(sys.argv[7]).expanduser())
carla = config.setdefault("carla", {})
carla["root"] = carla_root
carla["python_api_paths"] = [str(Path(carla_root) / "PythonAPI/carla")]
control = config.setdefault("control", {})
control["max_accel_mps2"] = float(sys.argv[3])
control["max_brake_mps2"] = float(sys.argv[4])
control["max_throttle"] = float(sys.argv[5])
control["max_brake"] = float(sys.argv[6])
control["override_command_accel_limits"] = True
output_path.write_text(
    json.dumps(config, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
PY
  export CONFIG_PATH="${RUNTIME_CONFIG_PATH}"
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
    echo "[Scenario] acceleration runtime missing; building latest newaccpro4 code"
    PANGU_PROJECT_SOURCE="${RUNTIME_PROJECT_SOURCE}" \
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
  sed -i 's/to_file: OFF/to_file: ON/g' "${app_cfg}"
}

wait_processes() {
  local deadline=$((SECONDS + PANGU_READY_TIMEOUT_SEC))
  local process
  while (( SECONDS < deadline )); do
    local ready=1
    for process in "${PANGU_MAIN_PROCESS}" "${PANGU_INPUT_PROCESS}" "${PANGU_OUTPUT_PROCESS}"; do
      docker exec "${PANGU_CONTAINER_NAME}" pgrep -f \
        "^dataflow_runner .*--process_name=${process}" >/dev/null 2>&1 || ready=0
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

prepare_runtime_bridge_config
verify_acceleration_boundary
ensure_build
require_file "${PANGU_INSTALL_DIR}/setup.bash"
require_file "${PANGU_INSTALL_DIR}/run.sh"

if [[ "${CLEAN_START:-1}" == "1" ]]; then
  bash "${SCENARIO_DIR}/stop.sh" >/dev/null 2>&1 || true
fi

echo "[Scenario] starting CARLA + Bridge for newaccpro4 acceleration output"
LEAD_WAYPOINT_STEP_SPEED_MPS="${ACC_LEAD_STEP_SPEED_MPS}" \
LEAD_WAYPOINT_STEP_AT_SEC="${ACC_LEAD_STEP_TIME_SEC}" \
LEAD_WAYPOINT_MAX_THROTTLE="${ACC_LEAD_MAX_THROTTLE}" \
bash "${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh" \
  --config "${CONFIG_PATH}" \
  --ego-spawn-index 198 \
  --lead-distance "${ACC_LEAD_DISTANCE_M}" \
  --lead-speed "${ACC_LEAD_SPEED_MPS}" \
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
  --analysis-start-sec "${ACC_LEAD_STEP_TIME_SEC}" \
  --max-accel-mps2 "${ACC_MAX_ACCEL_MPS2}" \
  --max-brake-mps2 "${ACC_MAX_BRAKE_MPS2}" \
  --speed-tolerance-mps 0.25 \
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

if [[ -n "${AUTO_START_COMMAND}" ]]; then
  echo "[Scenario] boosting ego above minimumOperatingSpeedMps=1"
  "${PYTHON_BIN}" "${REPO_ROOT}/tools/carla_bridge/boost-ego-speed.py" \
    --carla-root "${CARLA_ROOT}" \
    --speed-mps "${BOOST_SPEED_MPS}" \
    --duration-sec "${BOOST_DURATION_SEC}" \
    >"${PANGU_LOG_DIR}/boost_ego.log" 2>&1
  echo "[Scenario] sending ACC engage after ego boost has completed"
  python3 "${REPO_ROOT}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py" \
    --once "${AUTO_START_COMMAND}" >"${PANGU_LOG_DIR}/startup_command.log" 2>&1
  echo "[Scenario] ready: ACC automatic startup command has been sent"
else
  echo "[Scenario] ready: ACC is in standby; hold W to accelerate, then press E to engage"
fi
