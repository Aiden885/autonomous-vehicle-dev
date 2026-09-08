#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
PANGU_CONTAINER_NAME="${PANGU_CONTAINER_NAME:-newaccpro4_codegen_carla}"
PANGU_LOG_DIR="${PANGU_LOG_DIR:-/tmp/newaccpro4-codegen-carla/pangu}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/newaccpro4-codegen-carla/carla}"

if [[ -f "${PANGU_LOG_DIR}/recorder.pid" ]]; then
  recorder_pid="$(cat "${PANGU_LOG_DIR}/recorder.pid" 2>/dev/null || true)"
  [[ -z "${recorder_pid}" ]] || kill -TERM "${recorder_pid}" >/dev/null 2>&1 || true
  rm -f "${PANGU_LOG_DIR}/recorder.pid"
fi

docker rm -f "${PANGU_CONTAINER_NAME}" >/dev/null 2>&1 || true
bash "${REPO_ROOT}/tools/carla_bridge/stop-gaasd-carla-manual.sh" \
  --log-dir "${GAASD_CARLA_LOG_DIR}" \
  --carla-root "${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}" \
  --keep-gaasd-sim
