#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

export CARLA_ROOT="${CARLA_ROOT:-/opt/carla}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/apa-pangu-carla/carla}"
PANEL_RUNTIME_DIR="${GAASD_PANEL_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-/tmp}/gaasd-scenario-panel}"
PANEL_PID_FILE="${PANEL_PID_FILE:-${PANEL_RUNTIME_DIR}/panel.pid}"

stop_pid_file() {
  local label="$1"
  local pid_file="$2"
  local pid=""
  local stat=""
  if [ ! -f "${pid_file}" ]; then
    echo "[APA stop] ${label} pid file not found: ${pid_file}"
    return 0
  fi
  pid="$(cat "${pid_file}" 2>/dev/null || true)"
  stat="$(ps -p "${pid}" -o stat= 2>/dev/null | awk '{print $1}')"
  if [ -n "${pid}" ] && [ -n "${stat}" ] && [[ "${stat}" != Z* ]]; then
    kill "${pid}" >/dev/null 2>&1 || true
    sleep 0.3
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -KILL "${pid}" >/dev/null 2>&1 || true
    fi
    echo "[APA stop] stopped ${label} pid=${pid}"
  else
    echo "[APA stop] ${label} already stopped pid=${pid}"
  fi
  rm -f "${pid_file}"
}

stop_pid_file "scenario panel" "${PANEL_PID_FILE}"
stop_pid_file "APA bridge" "${GAASD_CARLA_LOG_DIR}/bridge_apa.pid"
stop_pid_file "CARLA" "${GAASD_CARLA_LOG_DIR}/carla.pid"

if [ -x "${REPO_ROOT}/tools/carla_bridge/stop-carla.sh" ]; then
  CARLA_PID_FILE="${GAASD_CARLA_LOG_DIR}/carla.pid" CARLA_ROOT="${CARLA_ROOT}" \
    bash "${REPO_ROOT}/tools/carla_bridge/stop-carla.sh" || true
fi

echo "[APA stop] stopped"
