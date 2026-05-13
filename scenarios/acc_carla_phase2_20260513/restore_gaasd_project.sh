#!/usr/bin/env bash
set -euo pipefail

SCENARIO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCENARIO_DIR}/../.." && pwd)"
FORCE="0"

usage() {
  cat <<'EOF'
Usage:
  scenarios/acc_carla_phase2_20260513/restore_gaasd_project.sh [--force]

Restores saved GAASD project snapshots to:
  project/carla_restored_20260513
  project/accpro1_reference_20260513

Use --force to replace existing restored directories.
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

restore_one() {
  local src="$1"
  local dst="$2"

  if [ -e "$dst" ] && [ "$FORCE" != "1" ]; then
    echo "[Restore] target exists, skip: $dst"
    echo "[Restore] rerun with --force to replace it"
    return 0
  fi

  if [ -e "$dst" ]; then
    rm -rf "$dst"
  fi

  mkdir -p "$(dirname "$dst")"
  rsync -a "$src/" "$dst/"
  echo "[Restore] restored: $dst"
}

restore_one \
  "${SCENARIO_DIR}/gaasd_project_snapshot/carla" \
  "${REPO_ROOT}/project/carla_restored_20260513"

restore_one \
  "${SCENARIO_DIR}/gaasd_project_snapshot/accpro1_reference" \
  "${REPO_ROOT}/project/accpro1_reference_20260513"

echo "[Restore] done. Open project/carla_restored_20260513 in GAASD for the CARLA closed-loop canvas."
