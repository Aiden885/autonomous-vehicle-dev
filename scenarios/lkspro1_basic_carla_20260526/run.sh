#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
BRIDGE_DIR="${REPO_ROOT}/tools/carla_bridge"

export CONFIG_PATH="${SCENARIO_DIR}/bridge_config.json"
export CARLA_ROOT="${CARLA_ROOT:-/home/aiden/snap/code/app/carla-package}"
export PYTHON_BIN="${PYTHON_BIN:-python3.8}"
export GAASD_CARLA_LOG_DIR="${GAASD_CARLA_LOG_DIR:-/tmp/gaasd-carla-lkspro1}"

cd "$REPO_ROOT"

echo "[Scenario] lkspro1 single-ego lane keeping"
echo "[Scenario] using bridge: ${BRIDGE_DIR}"
echo "[Scenario] using config: ${CONFIG_PATH}"

"${BRIDGE_DIR}/start-gaasd-carla-manual.sh" \
  --ego-spawn-index 198 \
  --no-lead \
  --follow-spectator \
  --spectator-back 8 \
  --spectator-up 6 \
  --spectator-pitch -25 \
  --no-probe

"$PYTHON_BIN" "${BRIDGE_DIR}/reset-lks-straight-scene.py" \
  --carla-root "$CARLA_ROOT" \
  --host 127.0.0.1 \
  --port 2000 \
  --ego-spawn-index 198 \
  --lateral-offset-m 0.8 \
  --heading-error-deg 5.0

"$PYTHON_BIN" "${BRIDGE_DIR}/set-spectator-follow.py" \
  --once \
  --carla-root "$CARLA_ROOT" \
  --host 127.0.0.1 \
  --port 2000 \
  --ego-role-name hero \
  --back-m 8 \
  --up-m 6 \
  --pitch-deg -25 \
  --fallback-spawn-index 198 \
  --timeout-sec 10

echo "[Scenario] CARLA + Bridge ready. Open the LKS GAASD canvas and start the oscilloscope simulation."
echo "[Scenario] recommended LKS signals: lateralOffset, headingError, steerRad, egoV"
