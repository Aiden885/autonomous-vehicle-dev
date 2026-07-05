#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
PANGU_CONTAINER_NAME="${PANGU_CONTAINER_NAME:-lks2_pangu_carla}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/lks2-pangu-carla/carla}"
PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/lks2-pangu-carla/pangu}"
RECORDER_PID_FILE="${PANGU_LOG_DIR}/recorder.pid"

if [[ -f "${RECORDER_PID_FILE}" ]]; then
  RECORDER_PID="$(cat "${RECORDER_PID_FILE}" 2>/dev/null || true)"
  if [[ -n "${RECORDER_PID}" ]]; then
    kill -TERM "${RECORDER_PID}" >/dev/null 2>&1 || true
    sleep 1
  fi
  rm -f "${RECORDER_PID_FILE}"
fi

docker rm -f "${PANGU_CONTAINER_NAME}" >/dev/null 2>&1 || true
bash "${REPO_ROOT}/tools/carla_bridge/stop-gaasd-carla-manual.sh" \
  --log-dir "${GAASD_CARLA_LOG_DIR}" \
  --carla-root "${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}" \
  --keep-gaasd-sim
