#!/usr/bin/env bash
set -euo pipefail

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${PYTHONPATH:-}"
export PATH="${PATH:-/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_ROOT="${PANGU_BUILD_ROOT:-/tmp/pangu_acc_closed_loop_build_20260629_202705}"
INSTALL_DIR="${PANGU_INSTALL_DIR:-${BUILD_ROOT}/install}"
THIRD_SETUP="${PANGU_THIRD_SETUP:-${BUILD_ROOT}/dependencies/thirdparty/X86/setup.bash}"
LOG_DIR="${LOG_DIR:-/tmp/pangu_acc_closed_loop_runtime}"
APP_NAME="${PANGU_APP_NAME:-app_empty}"
START_MODE="${PANGU_START_MODE:-topo}"
PROCESS_NAME="${PANGU_PROCESS_NAME:-ZmqBridgeModule}"
MOCK_EGO_SPEED="${MOCK_EGO_SPEED:-0}"
MOCK_COAST_DECEL="${MOCK_COAST_DECEL:-0.4}"

MOCK_LOG="${LOG_DIR}/mock_sequence_bridge.log"
RUN_LOG="${LOG_DIR}/mock_sequence_run_all.log"
KEY_LOG="${LOG_DIR}/mock_sequence_keyboard.log"
MOCK_PID_FILE="${LOG_DIR}/mock_sequence_bridge.pid"

mkdir -p "${LOG_DIR}"
: >"${MOCK_LOG}"
: >"${RUN_LOG}"
: >"${KEY_LOG}"

require_file() {
  local path="$1"
  if [[ ! -e "${path}" ]]; then
    echo "[mock-seq] missing: ${path}" >&2
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
    echo "[mock-seq] failed to read local_machine_name from ${machine_cfg}" >&2
    exit 1
  fi
  sed -i "s/soc_name: \"[^\"]*\"/soc_name: \"${local_soc}\"/" "${app_cfg}"
  echo "[mock-seq] app ${APP_NAME} soc_name=${local_soc}"
}

stop_pangu() {
  if [[ -f "${INSTALL_DIR}/setup.bash" && -f "${INSTALL_DIR}/stop_all.sh" ]]; then
    load_pangu_env
    bash "${INSTALL_DIR}/stop_all.sh" "${APP_NAME}" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  stop_pangu
  if [[ -f "${MOCK_PID_FILE}" ]]; then
    local pid
    pid="$(cat "${MOCK_PID_FILE}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  fi
}

send_key() {
  local key="$1"
  local wait_sec="$2"
  python3 "${ROOT_DIR}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py" --once "${key}" >>"${KEY_LOG}" 2>&1
  echo "[mock-seq] sent ${key}"
  sleep "${wait_sec}"
}

assert_log() {
  local pattern="$1"
  local message="$2"
  if ! grep -Eq "${pattern}" "${MOCK_LOG}"; then
    echo "[mock-seq] FAIL: ${message}" >&2
    tail -120 "${MOCK_LOG}" >&2 || true
    exit 1
  fi
}

trap cleanup EXIT

require_file "${THIRD_SETUP}"
require_file "${INSTALL_DIR}/setup.bash"
require_file "${INSTALL_DIR}/run_all.sh"
require_file "${ROOT_DIR}/tools/pangu_acc_closed_loop/mock_carla_bridge.py"
require_file "${ROOT_DIR}/tools/pangu_acc_closed_loop/keyboard_command_publisher.py"

stop_pangu
ensure_local_soc_name

python3 -u "${ROOT_DIR}/tools/pangu_acc_closed_loop/mock_carla_bridge.py" \
  --ego-speed "${MOCK_EGO_SPEED}" \
  --coast-decel "${MOCK_COAST_DECEL}" \
  --distance 12 \
  --lead-speed 2 \
  >"${MOCK_LOG}" 2>&1 &
echo "$!" >"${MOCK_PID_FILE}"
echo "[mock-seq] mock bridge pid=$(cat "${MOCK_PID_FILE}")"
sleep 1

load_pangu_env
if [[ "${START_MODE}" == "direct" ]]; then
  bash "${INSTALL_DIR}/run.sh" "${APP_NAME}" "${PROCESS_NAME}" -nohup >"${RUN_LOG}" 2>&1
  echo "[mock-seq] pangu run.sh returned process=${PROCESS_NAME}"
else
  bash "${INSTALL_DIR}/run_all.sh" "${APP_NAME}" >"${RUN_LOG}" 2>&1
  echo "[mock-seq] pangu run_all returned"
fi
sleep 2

send_key e 2
send_key t 1
send_key r 1
send_key q 1
send_key c 1
send_key e 2
send_key s 1
send_key 0 1

assert_log "driver cmd=1" "E command did not reach mock bridge"
assert_log "driver cmd=3" "T command did not reach mock bridge"
assert_log "driver cmd=4" "R command did not reach mock bridge"
assert_log "driver cmd=2" "Q command did not reach mock bridge"
assert_log "driver cmd=7" "C command did not reach mock bridge"
assert_log "driver cmd=6" "S command did not reach mock bridge"
assert_log "driver cmd=0" "release command did not reach mock bridge"
assert_log "control target=.* enable=1" "enabled control command was not observed"
assert_log "control target=.* enable=0" "disabled control command was not observed"

echo "[mock-seq] key log:"
cat "${KEY_LOG}"
echo "[mock-seq] mock bridge tail:"
tail -120 "${MOCK_LOG}"
echo "[mock-seq] PASS"
