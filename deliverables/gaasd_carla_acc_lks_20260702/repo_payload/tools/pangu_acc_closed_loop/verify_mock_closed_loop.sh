#!/usr/bin/env bash
set -euo pipefail

# Pangu/colcon setup scripts assume these variables already exist. Keep strict
# mode in this script, but initialize optional environment variables first.
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${PYTHONPATH:-}"
export PATH="${PATH:-/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_ROOT="${PANGU_BUILD_ROOT:-/tmp/pangu_acc_closed_loop_build_20260629_202705}"
INSTALL_DIR="${PANGU_INSTALL_DIR:-${BUILD_ROOT}/install}"
THIRD_SETUP="${PANGU_THIRD_SETUP:-${BUILD_ROOT}/dependencies/thirdparty/X86/setup.bash}"
LOG_DIR="${LOG_DIR:-/tmp/pangu_acc_closed_loop_runtime}"
APP_NAME="${PANGU_APP_NAME:-app_empty}"

MOCK_LOG="${LOG_DIR}/verify_mock_bridge.log"
RUN_LOG="${LOG_DIR}/verify_run_all.log"
KEY_LOG="${LOG_DIR}/verify_keyboard.log"
MOCK_PID_FILE="${LOG_DIR}/verify_mock_bridge.pid"
LEGACY_MOCK_PID_FILE="${LOG_DIR}/mock_bridge.pid"

mkdir -p "${LOG_DIR}"
: >"${KEY_LOG}"
: >"${MOCK_LOG}"
: >"${RUN_LOG}"

require_file() {
  local path="$1"
  if [[ ! -e "${path}" ]]; then
    echo "[verify] missing: ${path}" >&2
    exit 1
  fi
}

load_pangu_env() {
  # colcon-generated setup scripts can reference optional unset variables.
  # Temporarily disable nounset only while sourcing them.
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
    echo "[verify] failed to read local_machine_name from ${machine_cfg}" >&2
    exit 1
  fi
  sed -i "s/soc_name: \"[^\"]*\"/soc_name: \"${local_soc}\"/" "${app_cfg}"
  echo "[verify] app ${APP_NAME} soc_name=${local_soc}"
}

stop_pangu() {
  if [[ -f "${INSTALL_DIR}/setup.bash" && -f "${INSTALL_DIR}/stop_all.sh" ]]; then
    load_pangu_env
    bash "${INSTALL_DIR}/stop_all.sh" "${APP_NAME}" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  stop_pangu
  for pid_file in "${MOCK_PID_FILE}" "${LEGACY_MOCK_PID_FILE}"; do
    if [[ ! -f "${pid_file}" ]]; then
      continue
    fi
    local pid
    pid="$(cat "${pid_file}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}

trap cleanup EXIT

require_file "${THIRD_SETUP}"
require_file "${INSTALL_DIR}/setup.bash"
require_file "${INSTALL_DIR}/run_all.sh"
require_file "${ROOT_DIR}/tools/pangu_acc_closed_loop/mock_carla_bridge.py"
require_file "${ROOT_DIR}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py"

echo "[verify] build root: ${BUILD_ROOT}"
echo "[verify] install dir: ${INSTALL_DIR}"
echo "[verify] log dir: ${LOG_DIR}"

stop_pangu
cleanup
ensure_local_soc_name

python3 -u "${ROOT_DIR}/tools/pangu_acc_closed_loop/mock_carla_bridge.py" >"${MOCK_LOG}" 2>&1 &
echo "$!" >"${MOCK_PID_FILE}"
echo "[verify] mock bridge pid=$(cat "${MOCK_PID_FILE}")"
sleep 1

load_pangu_env

bash "${INSTALL_DIR}/run_all.sh" "${APP_NAME}" >"${RUN_LOG}" 2>&1
echo "[verify] pangu run_all returned"
sleep 3

python3 "${ROOT_DIR}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py" --once e >>"${KEY_LOG}" 2>&1
echo "[verify] sent commandType=1 (E)"
sleep 3

python3 "${ROOT_DIR}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py" --once c >>"${KEY_LOG}" 2>&1
echo "[verify] sent commandType=7 (C)"
sleep 1

echo "[verify] key log:"
tail -20 "${KEY_LOG}" || true

echo "[verify] mock bridge log:"
tail -80 "${MOCK_LOG}" || true

echo "[verify] pangu run log key lines:"
grep -E "ACCModule|ZmqBridgeModule|run_all success|FAILED|ERROR|failed|error" "${RUN_LOG}" || true

echo "[verify] done"
