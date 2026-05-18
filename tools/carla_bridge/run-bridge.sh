#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PYTHON_BIN="${PYTHON_BIN:-python3.8}"
CONFIG_PATH="${CONFIG_PATH:-${PROJECT_ROOT}/tools/carla_bridge/config.example.json}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
    echo "[Bridge] ${PYTHON_BIN} not found. Set PYTHON_BIN=python3.7 or install python3.8." >&2
    exit 1
fi

export PYTHON_EGG_CACHE="${PYTHON_EGG_CACHE:-/tmp/gaasd-carla-python-eggs}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

exec "${PYTHON_BIN}" "${SCRIPT_DIR}/carla_bridge.py" --config "${CONFIG_PATH}" "$@"
