#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

export CARLA_ROOT="${CARLA_ROOT:-/opt/carla}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/lks2-pangu-carla/carla}"
PANGU_CONTAINER_NAME="${PANGU_CONTAINER_NAME:-lks2_pangu_carla}"
PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/lks2-pangu-carla/pangu}"
RECORDER_PID_FILE="${RECORDER_PID_FILE:-${PANGU_LOG_DIR}/recorder.pid}"
PANEL_RUNTIME_DIR="${GAASD_PANEL_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-/tmp}/gaasd-scenario-panel}"
PANEL_PID_FILE="${PANEL_PID_FILE:-${PANEL_RUNTIME_DIR}/panel.pid}"
PANEL_PORT="${GAASD_PANEL_PORT:-8088}"

stop_pid_file() {
  local label="$1"
  local pid_file="$2"
  local pid=""
  if [[ ! -f "${pid_file}" ]]; then
    echo "[LKS stop] ${label} pid file not found: ${pid_file}"
    return 0
  fi
  pid="$(cat "${pid_file}" 2>/dev/null || true)"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    sleep 0.3
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -TERM "${pid}" >/dev/null 2>&1 || true
      sleep 0.5
    fi
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -KILL "${pid}" >/dev/null 2>&1 || true
    fi
    echo "[LKS stop] stopped ${label} pid=${pid}"
  else
    echo "[LKS stop] ${label} already stopped pid=${pid}"
  fi
  rm -f "${pid_file}"
}

stop_panel() {
  echo "[LKS stop] stopping scenario panel"
  stop_pid_file "scenario panel" "${PANEL_PID_FILE}"
  local pids
  pids="$(pgrep -f "tools/gaasd_scenario_panel/app.py" || true)"
  for pid in ${pids}; do
    if [[ "${pid}" != "$$" ]]; then
      kill "${pid}" >/dev/null 2>&1 || true
      echo "[LKS stop] stopped scenario panel fallback pid=${pid}"
    fi
  done
}

stop_panel
stop_pid_file "LKS recorder" "${RECORDER_PID_FILE}"

if command -v docker >/dev/null 2>&1; then
  echo "[LKS stop] stopping Pangu container ${PANGU_CONTAINER_NAME} if present"
  docker rm -f "${PANGU_CONTAINER_NAME}" >/dev/null 2>&1 || true
fi

echo "[LKS stop] stopping CARLA + Bridge"
bash "${REPO_ROOT}/tools/carla_bridge/stop-gaasd-carla-manual.sh" \
  --log-dir "${GAASD_CARLA_LOG_DIR}" \
  --carla-root "${CARLA_ROOT}" \
  --keep-gaasd-sim

echo "[LKS stop] stopped"
