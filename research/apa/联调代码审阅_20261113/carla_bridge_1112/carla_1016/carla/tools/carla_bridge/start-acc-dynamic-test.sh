#!/usr/bin/env bash
# ACC dynamic-lead test launcher.
#
# This script intentionally delegates to start-gaasd-carla-manual.sh so the
# CARLA/Bridge startup, readiness checks, logging, and stop flow stay unified.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

CONFIG_PATH="${CONFIG_PATH:-${PROJECT_ROOT}/tools/carla_bridge/config.phase2.json}"
LEAD_DISTANCE_M="${LEAD_DISTANCE_M:-${LEAD_DISTANCE:-25}}"
LEAD_SPEED_MPS="${LEAD_SPEED_MPS:-${LEAD_SPEED:-3}}"
LEAD_PLACEMENT="${LEAD_PLACEMENT:-lane_waypoint}"
LEAD_BEHAVIOR="${LEAD_BEHAVIOR:-waypoint_pid}"

echo "[GAASD-CARLA] ACC dynamic-lead launcher"
echo "[GAASD-CARLA] config=${CONFIG_PATH}"
echo "[GAASD-CARLA] lead distance=${LEAD_DISTANCE_M}m speed=${LEAD_SPEED_MPS}m/s placement=${LEAD_PLACEMENT} behavior=${LEAD_BEHAVIOR}"

exec "${SCRIPT_DIR}/start-gaasd-carla-manual.sh" \
    --config "$CONFIG_PATH" \
    --lead-distance "$LEAD_DISTANCE_M" \
    --lead-speed "$LEAD_SPEED_MPS" \
    --lead-placement "$LEAD_PLACEMENT" \
    --lead-behavior "$LEAD_BEHAVIOR" \
    --watch-camera \
    "$@"
