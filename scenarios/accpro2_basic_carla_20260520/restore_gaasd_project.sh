#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
FORCE="0"

usage() {
  cat <<'EOF'
Usage:
  scenarios/accpro2_basic_carla_20260520/restore_gaasd_project.sh [--force]

Restores the saved GAASD project snapshot to:
  project/accpro2

Use --force to replace the existing restored directory.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --force)
      FORCE="1"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[Restore] unknown arg: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

SRC="${SCENARIO_DIR}/gaasd_project_snapshot/accpro2"
DST="${REPO_ROOT}/project/accpro2"

if [ -e "$DST" ] && [ "$FORCE" != "1" ]; then
  echo "[Restore] target exists, skip: $DST"
  echo "[Restore] rerun with --force to replace it"
  exit 0
fi

if [ -e "$DST" ]; then
  rm -rf "$DST"
fi

mkdir -p "$(dirname "$DST")"
rsync -a "$SRC/" "$DST/"
echo "[Restore] restored: $DST"
echo "[Restore] done. Open project/accpro2 in GAASD for the basic-module ACC canvas."
