#!/usr/bin/env bash
set -euo pipefail

PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/lks3-codegen-carla/pangu}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/lks3-codegen-carla/carla}"
PANEL_RUNTIME_DIR="${GAASD_PANEL_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-/tmp}/gaasd-scenario-panel}"
PANEL_PID_FILE="${PANEL_PID_FILE:-${PANEL_RUNTIME_DIR}/panel.pid}"

stop_pid_file() {
  local label="$1"
  local pid_file="$2"
  local pid=""
  if [[ ! -f "${pid_file}" ]]; then
    echo "[LKS bridge-only stop] ${label} pid file not found: ${pid_file}"
    return 0
  fi
  pid="$(cat "${pid_file}" 2>/dev/null || true)"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    echo "[LKS bridge-only stop] stopped ${label} pid=${pid}"
  else
    echo "[LKS bridge-only stop] ${label} already stopped pid=${pid}"
  fi
  rm -f "${pid_file}"
}

stop_by_pattern() {
  local label="$1"
  local pattern="$2"
  local pids
  pids="$(pgrep -f "${pattern}" || true)"
  if [[ -z "${pids}" ]]; then
    echo "[LKS bridge-only stop] ${label} no matching process"
    return 0
  fi
  for pid in ${pids}; do
    if [[ "${pid}" != "$$" ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
      echo "[LKS bridge-only stop] stopped ${label} pid=${pid}"
    fi
  done
}

stop_pid_file "recorder" "${PANGU_LOG_DIR}/recorder.pid"

stop_pid_file "scenario panel" "${PANEL_PID_FILE}"
stop_by_pattern "scenario panel" "tools/gaasd_scenario_panel/app.py"

stop_pid_file "watch camera" "${GAASD_CARLA_LOG_DIR}/watch-camera.pid"
stop_by_pattern "watch camera" "tools/carla_bridge/watch-carla.py"

stop_pid_file "spectator follow" "${GAASD_CARLA_LOG_DIR}/spectator.pid"
stop_by_pattern "spectator follow" "tools/carla_bridge/set-spectator-follow.py"

stop_pid_file "bridge" "${GAASD_CARLA_LOG_DIR}/bridge.pid"
stop_by_pattern "bridge" "tools/carla_bridge/carla_bridge_lks.py"
stop_by_pattern "legacy bridge" "tools/carla_bridge/carla_bridge.py"

echo "[LKS bridge-only stop] stopped bridge/panel helpers; CARLA and Pangu were left running"
