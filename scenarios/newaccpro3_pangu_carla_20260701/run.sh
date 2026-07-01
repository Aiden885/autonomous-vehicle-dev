#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

export CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"
export CONFIG_PATH="${SCENARIO_DIR}/bridge_config.json"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/newaccpro3-pangu-carla/carla}"

PANGU_BUILD_ROOT="${PANGU_BUILD_ROOT:-/tmp/newaccpro3_pangu_codegen_build}"
PANGU_INSTALL_DIR="${PANGU_INSTALL_DIR:-${PANGU_BUILD_ROOT}/install}"
PANGU_THIRD_SETUP="${PANGU_THIRD_SETUP:-${PANGU_BUILD_ROOT}/dependencies/thirdparty/X86/setup.bash}"
PANGU_APP_NAME="${PANGU_APP_NAME:-app_empty}"
PANGU_PROCESS_NAME="${PANGU_PROCESS_NAME:-ZmqBridgeModule}"
PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/newaccpro3-pangu-carla/pangu}"
PANGU_DOCKER_IMAGE="${PANGU_DOCKER_IMAGE:-docker.cbdes.cn:8080/cbdes/x86:pangu-2.0.5}"
PANGU_CONTAINER_NAME="${PANGU_CONTAINER_NAME:-newaccpro3_pangu_carla}"
ACC_DECISION_VMIN="${ACC_DECISION_VMIN:-0.0}"
ACC_ZMQ_DEBUG_LOG="${ACC_ZMQ_DEBUG_LOG:-/tmp/newaccpro3_zmq_bridge_debug.log}"
AUTO_START_COMMAND="${AUTO_START_COMMAND:-e}"
BOOST_SPEED_MPS="${BOOST_SPEED_MPS:-2.0}"
BOOST_DURATION_SEC="${BOOST_DURATION_SEC:-4.0}"
PANGU_READY_TIMEOUT_SEC="${PANGU_READY_TIMEOUT_SEC:-20}"
PANGU_COMMAND_READY_DELAY_SEC="${PANGU_COMMAND_READY_DELAY_SEC:-2.0}"

mkdir -p "${PANGU_LOG_DIR}"
: >"${ACC_ZMQ_DEBUG_LOG}"

require_file() {
  local path="$1"
  if [[ ! -e "${path}" ]]; then
    echo "[Scenario] missing: ${path}" >&2
    exit 1
  fi
}

load_pangu_env() {
  set +u
  # shellcheck disable=SC1090
  source "${PANGU_THIRD_SETUP}" >/dev/null 2>&1 || true
  # shellcheck disable=SC1090
  source "${PANGU_INSTALL_DIR}/setup.bash" >/dev/null 2>&1 || true
  set -u
}

ensure_local_soc_name() {
  local app_cfg="${PANGU_INSTALL_DIR}/conf/app_module/${PANGU_APP_NAME}.pt"
  local machine_cfg="${PANGU_INSTALL_DIR}/conf/global_conf/icvos_machine.pt"
  local local_soc
  require_file "${app_cfg}"
  require_file "${machine_cfg}"
  local_soc="$(sed -n 's/.*local_machine_name: "\([^"]*\)".*/\1/p' "${machine_cfg}" | head -1)"
  if [[ -z "${local_soc}" ]]; then
    echo "[Scenario] failed to read local_machine_name from ${machine_cfg}" >&2
    exit 1
  fi
  sed -i "s/soc_name: \"[^\"]*\"/soc_name: \"${local_soc}\"/" "${app_cfg}"
  echo "[Scenario] app ${PANGU_APP_NAME} soc_name=${local_soc}"
}

wait_pangu_process_ready() {
  local deadline
  deadline=$((SECONDS + PANGU_READY_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if docker exec "${PANGU_CONTAINER_NAME}" bash -lc \
      "pgrep -f 'dataflow_runner.*--process_name=${PANGU_PROCESS_NAME}' >/dev/null 2>&1"; then
      echo "[Scenario] Pangu process ${PANGU_PROCESS_NAME} is running"
      return 0
    fi
    sleep 0.5
  done
  echo "[Scenario] Pangu process ${PANGU_PROCESS_NAME} did not become ready within ${PANGU_READY_TIMEOUT_SEC}s" >&2
  docker logs "${PANGU_CONTAINER_NAME}" >"${PANGU_LOG_DIR}/docker_start.log" 2>&1 || true
  tail -120 "${PANGU_LOG_DIR}/docker_start.log" || true
  return 1
}

require_file "${PANGU_INSTALL_DIR}/setup.bash"
require_file "${PANGU_INSTALL_DIR}/run.sh"
require_file "${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh"
require_file "${REPO_ROOT}/tools/carla_bridge/boost-ego-speed.py"
require_file "${REPO_ROOT}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py"
command -v docker >/dev/null 2>&1 || {
  echo "[Scenario] docker is required to run Pangu ${PANGU_DOCKER_IMAGE}" >&2
  exit 1
}

cd "${REPO_ROOT}"

echo "[Scenario] starting CARLA + Bridge for newaccpro3 generated-code ACC"
"${REPO_ROOT}/tools/carla_bridge/start-gaasd-carla-manual.sh" \
  --config "${CONFIG_PATH}" \
  --ego-spawn-index 198 \
  --lead-distance 25 \
  --lead-speed 2 \
  --lead-placement lane_waypoint \
  --lead-behavior constant_velocity \
  --follow-spectator \
  --spectator-back 8 \
  --spectator-up 6 \
  --spectator-pitch -25 \
  --no-probe

echo "[Scenario] starting Pangu process ${PANGU_PROCESS_NAME} via run.sh direct mode"
ensure_local_soc_name

docker rm -f "${PANGU_CONTAINER_NAME}" >/dev/null 2>&1 || true
docker run -d \
  --name "${PANGU_CONTAINER_NAME}" \
  --net=host \
  --shm-size=8193m \
  -v /home:/home \
  -v /data:/data \
  -v /tmp:/tmp \
  -e PANGU_BUILD_ROOT="${PANGU_BUILD_ROOT}" \
  -e PANGU_INSTALL_DIR="${PANGU_INSTALL_DIR}" \
  -e PANGU_THIRD_SETUP="${PANGU_THIRD_SETUP}" \
  -e PANGU_APP_NAME="${PANGU_APP_NAME}" \
  -e PANGU_PROCESS_NAME="${PANGU_PROCESS_NAME}" \
  -e ACC_DECISION_VMIN="${ACC_DECISION_VMIN}" \
  -e ACC_ZMQ_DEBUG_LOG="${ACC_ZMQ_DEBUG_LOG}" \
  "${PANGU_DOCKER_IMAGE}" \
  /bin/bash -lc '
    set -e
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="${PYTHONPATH:-}"
    export PATH="${PATH:-/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin}"
    source "${PANGU_THIRD_SETUP}" >/dev/null 2>&1 || true
    source "${PANGU_INSTALL_DIR}/setup.bash" >/dev/null 2>&1 || true
    bash "${PANGU_INSTALL_DIR}/stop_all.sh" "${PANGU_APP_NAME}" >/dev/null 2>&1 || true
    bash "${PANGU_INSTALL_DIR}/run.sh" "${PANGU_APP_NAME}" "${PANGU_PROCESS_NAME}" -nohup
    tail -f /dev/null
  ' >"${PANGU_LOG_DIR}/docker_container_id.txt"

echo "[Scenario] Pangu container: ${PANGU_CONTAINER_NAME}"
sleep 2
docker logs "${PANGU_CONTAINER_NAME}" >"${PANGU_LOG_DIR}/docker_start.log" 2>&1 || true
tail -80 "${PANGU_LOG_DIR}/docker_start.log" || true
if ! docker ps --format '{{.Names}}' | grep -Fxq "${PANGU_CONTAINER_NAME}"; then
  echo "[Scenario] Pangu container exited unexpectedly; see ${PANGU_LOG_DIR}/docker_start.log" >&2
  exit 1
fi

wait_pangu_process_ready
echo "[Scenario] waiting ${PANGU_COMMAND_READY_DELAY_SEC}s for Pangu ZMQ subscriber to settle"
sleep "${PANGU_COMMAND_READY_DELAY_SEC}"

echo "[Scenario] boosting ego speed for visual ACC startup speed=${BOOST_SPEED_MPS}m/s"
"${PYTHON_BIN}" "${REPO_ROOT}/tools/carla_bridge/boost-ego-speed.py" \
  --carla-root "${CARLA_ROOT}" \
  --speed-mps "${BOOST_SPEED_MPS}" \
  --duration-sec "${BOOST_DURATION_SEC}" \
  >"${PANGU_LOG_DIR}/boost_ego.log" 2>&1 &
BOOST_PID="$!"
sleep 0.3

if [[ -n "${AUTO_START_COMMAND}" ]]; then
  echo "[Scenario] sending startup driver command: ${AUTO_START_COMMAND}"
  python3 "${REPO_ROOT}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py" \
    --once "${AUTO_START_COMMAND}" >"${PANGU_LOG_DIR}/startup_command.log" 2>&1
fi

wait "${BOOST_PID}" || true

echo "[Scenario] newaccpro3 Pangu-CARLA stack ready"
echo "[Scenario] CARLA window should show ego following the 2m/s lead vehicle."
echo "[Scenario] stop from UI or run: bash ${SCENARIO_DIR}/stop.sh"
