#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="/home/aiden/gaasd_versions/gaasd-2.7.0.5"
APP_DIR="${BASE_DIR}/app"
PROFILE_DIR="${BASE_DIR}/home"

mkdir -p \
  "${PROFILE_DIR}/.config" \
  "${PROFILE_DIR}/.cache" \
  "${PROFILE_DIR}/.local/share" \
  "${PROFILE_DIR}/electron-user-data"

export HOME="${PROFILE_DIR}"
export XDG_CONFIG_HOME="${PROFILE_DIR}/.config"
export XDG_CACHE_HOME="${PROFILE_DIR}/.cache"
export XDG_DATA_HOME="${PROFILE_DIR}/.local/share"
export GAASD_SIDE_BY_SIDE_VERSION="2.7.0.5"
export PATH="${APP_DIR}:${PATH}"

exec "${APP_DIR}/gaasd" \
  --user-data-dir="${PROFILE_DIR}/electron-user-data" \
  "$@"
