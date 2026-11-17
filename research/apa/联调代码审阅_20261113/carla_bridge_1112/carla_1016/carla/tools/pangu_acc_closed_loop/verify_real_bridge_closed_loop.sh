#!/usr/bin/env bash
set -euo pipefail

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${PYTHONPATH:-}"
export PATH="${PATH:-/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_ROOT="${PANGU_BUILD_ROOT:-/tmp/pangu_acc_closed_loop_build_20260629_202705}"
INSTALL_DIR="${PANGU_INSTALL_DIR:-${BUILD_ROOT}/install}"
THIRD_SETUP="${PANGU_THIRD_SETUP:-${BUILD_ROOT}/dependencies/thirdparty/X86/setup.bash}"
APP_NAME="${PANGU_APP_NAME:-app_empty}"

GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/gaasd-carla-manual}"
LOG_DIR="${LOG_DIR:-/tmp/pangu_acc_closed_loop_runtime}"
RUN_LOG="${LOG_DIR}/real_bridge_run_all.log"
KEY_LOG="${LOG_DIR}/real_bridge_keyboard.log"
START_LOG="${LOG_DIR}/real_bridge_start_stack.log"
STATUS_LOG="${LOG_DIR}/real_bridge_status_probe.log"
KEEP_STACK="${KEEP_STACK:-0}"
REAL_COMMAND_SEQUENCE="${REAL_COMMAND_SEQUENCE:-e:5,t:2,r:2,q:2,c:2,e:3,s:2,0:1}"

mkdir -p "${LOG_DIR}"
: >"${RUN_LOG}"
: >"${KEY_LOG}"
: >"${START_LOG}"
: >"${STATUS_LOG}"

require_file() {
  local path="$1"
  if [[ ! -e "${path}" ]]; then
    echo "[real-verify] missing: ${path}" >&2
    exit 1
  fi
}

load_pangu_env() {
  set +u
  # shellcheck disable=SC1090
  source "${THIRD_SETUP}" >/dev/null 2>&1 || true
  # shellcheck disable=SC1090
  source "${INSTALL_DIR}/setup.bash" >/dev/null 2>&1 || true
  set -u
}

ensure_local_soc_name() {
  local app_cfg="${INSTALL_DIR}/conf/app_module/${APP_NAME}.pt"
  local machine_cfg="${INSTALL_DIR}/conf/global_conf/icvos_machine.pt"
  local local_soc
  require_file "${app_cfg}"
  require_file "${machine_cfg}"
  local_soc="$(sed -n 's/.*local_machine_name: "\([^"]*\)".*/\1/p' "${machine_cfg}" | head -1)"
  if [[ -z "${local_soc}" ]]; then
    echo "[real-verify] failed to read local_machine_name from ${machine_cfg}" >&2
    exit 1
  fi
  sed -i "s/soc_name: \"[^\"]*\"/soc_name: \"${local_soc}\"/" "${app_cfg}"
  echo "[real-verify] app ${APP_NAME} soc_name=${local_soc}"
}

stop_pangu() {
  if [[ -f "${INSTALL_DIR}/setup.bash" && -f "${INSTALL_DIR}/stop_all.sh" ]]; then
    load_pangu_env
    bash "${INSTALL_DIR}/stop_all.sh" "${APP_NAME}" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  stop_pangu
  if [[ "${KEEP_STACK}" != "1" ]]; then
    GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR}" \
      "${ROOT_DIR}/tools/carla_bridge/stop-gaasd-carla-manual.sh" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

send_key() {
  local key="$1"
  local wait_sec="$2"
  python3 "${ROOT_DIR}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py" --once "${key}" >>"${KEY_LOG}" 2>&1
  echo "[real-verify] sent ${key}, wait ${wait_sec}s"
  sleep "${wait_sec}"
}

run_command_sequence() {
  local item
  local key
  local wait_sec
  IFS=',' read -ra items <<<"${REAL_COMMAND_SEQUENCE}"
  for item in "${items[@]}"; do
    key="${item%%:*}"
    wait_sec="${item#*:}"
    if [[ "${key}" == "${wait_sec}" ]]; then
      wait_sec="2"
    fi
    send_key "${key}" "${wait_sec}"
  done
}

require_file "${THIRD_SETUP}"
require_file "${INSTALL_DIR}/setup.bash"
require_file "${INSTALL_DIR}/run_all.sh"
require_file "${ROOT_DIR}/tools/carla_bridge/start-gaasd-carla-manual.sh"
require_file "${ROOT_DIR}/tools/carla_bridge/stop-gaasd-carla-manual.sh"
require_file "${ROOT_DIR}/tools/carla_bridge/probe-pub.py"
require_file "${ROOT_DIR}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py"

echo "[real-verify] build root: ${BUILD_ROOT}"
echo "[real-verify] install dir: ${INSTALL_DIR}"
echo "[real-verify] pangu log dir: ${LOG_DIR}"
echo "[real-verify] carla log dir: ${GAASD_CARLA_LOG_DIR}"

stop_pangu
ensure_local_soc_name

GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR}" \
  "${ROOT_DIR}/tools/carla_bridge/start-gaasd-carla-manual.sh" \
    --lead-placement lane_waypoint \
    --lead-behavior constant_velocity \
    --no-probe >"${START_LOG}" 2>&1
echo "[real-verify] CARLA/Bridge stack ready"

load_pangu_env
bash "${INSTALL_DIR}/run_all.sh" "${APP_NAME}" >"${RUN_LOG}" 2>&1
echo "[real-verify] pangu run_all returned"
sleep 3

echo "[real-verify] command sequence: ${REAL_COMMAND_SEQUENCE}"
run_command_sequence

python3 "${ROOT_DIR}/tools/carla_bridge/probe-pub.py" \
  --topic-prefix gaasd.carla.bridge_status.v1 \
  --duration 3 \
  --min-messages 1 \
  --max-print 5 \
  --show-payload >"${STATUS_LOG}" 2>&1 || true

echo "[real-verify] key log:"
tail -20 "${KEY_LOG}" || true

echo "[real-verify] bridge status probe:"
cat "${STATUS_LOG}" || true

echo "[real-verify] bridge log key lines:"
tail -160 "${GAASD_CARLA_LOG_DIR}/bridge.log" 2>/dev/null || true

echo "[real-verify] pangu run log key lines:"
grep -E "ACCModule|ZmqBridgeModule|run_all success|FAILED|ERROR|failed|error" "${RUN_LOG}" || true

echo "[real-verify] stack start log key lines:"
grep -E "CARLA ready|Bridge .*ready|manual stack is ready|ERROR|failed|failed" "${START_LOG}" || true

echo "[real-verify] done"
