#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
SOURCE="${SCENARIO_DIR}/gaasd_project_snapshot/newaccpro2"
TARGET="${REPO_ROOT}/project/newaccpro2"
FORCE="0"

if [ "${1:-}" = "--force" ]; then
  FORCE="1"
elif [ "$#" -gt 0 ]; then
  echo "[Restore] unknown argument: $1" >&2
  exit 2
fi

if [ ! -d "$SOURCE" ]; then
  echo "[Restore] snapshot not found: $SOURCE" >&2
  exit 1
fi

if [ -e "$TARGET" ] && [ "$FORCE" != "1" ]; then
  echo "[Restore] target exists, skip: $TARGET"
  echo "[Restore] enable overwrite in the UI or rerun with --force"
  exit 0
fi

if [ -e "$TARGET" ]; then
  BACKUP="${TARGET}_before_restore_$(date +%Y%m%d_%H%M%S)"
  mv "$TARGET" "$BACKUP"
  echo "[Restore] existing project moved to: $BACKUP"
fi

mkdir -p "$(dirname "$TARGET")"
rsync -a "$SOURCE/" "$TARGET/"
echo "[Restore] restored: $TARGET"
echo "[Restore] open project/newaccpro2 in GAASD and regenerate code before simulation"
