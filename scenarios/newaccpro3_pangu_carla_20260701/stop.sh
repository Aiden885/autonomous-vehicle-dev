#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"

PANGU_BUILD_ROOT="${PANGU_BUILD_ROOT:-${HOME}/.cache/gaasd-pangu/newaccpro3_codegen_build}"
PANGU_INSTALL_DIR="${PANGU_INSTALL_DIR:-${PANGU_BUILD_ROOT}/install}"
PANGU_THIRD_SETUP="${PANGU_THIRD_SETUP:-${PANGU_BUILD_ROOT}/dependencies/thirdparty/X86/setup.bash}"
PANGU_APP_NAME="${PANGU_APP_NAME:-app_empty}"
PANGU_CONTAINER_NAME="${PANGU_CONTAINER_NAME:-newaccpro3_pangu_carla}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/newaccpro3-pangu-carla/carla}"
PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/newaccpro3-pangu-carla/pangu}"

if [[ -f "${PANGU_LOG_DIR}/acc_recorder.pid" ]]; then
  recorder_pid="$(cat "${PANGU_LOG_DIR}/acc_recorder.pid" 2>/dev/null || true)"
  if [[ -n "${recorder_pid}" ]]; then
    echo "[Scenario] stopping ACC recorder pid=${recorder_pid}"
    kill "${recorder_pid}" >/dev/null 2>&1 || true
  fi
  rm -f "${PANGU_LOG_DIR}/acc_recorder.pid"
fi

echo "[Scenario] stopping Pangu container ${PANGU_CONTAINER_NAME}"
if command -v docker >/dev/null 2>&1; then
  docker rm -f "${PANGU_CONTAINER_NAME}" >/dev/null 2>&1 || true
fi

echo "[Scenario] stopping CARLA + Bridge"
"${REPO_ROOT}/tools/carla_bridge/stop-gaasd-carla-manual.sh" \
  --log-dir "${GAASD_CARLA_LOG_DIR}" \
  --carla-root "${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}" \
  --keep-gaasd-sim

echo "[Scenario] stopped"
