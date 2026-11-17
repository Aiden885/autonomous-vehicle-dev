#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCHER="$SCRIPT_DIR/start-panel.sh"
ICON="$SCRIPT_DIR/assets/gaasd-carla-panel.svg"
APP_DIR="${HOME}/.local/share/applications"
APP_FILE="$APP_DIR/gaasd-carla-panel.desktop"

desktop_dir() {
  if command -v xdg-user-dir >/dev/null 2>&1; then
    xdg-user-dir DESKTOP
  else
    echo "${HOME}/桌面"
  fi
}

write_desktop_file() {
  local target
  target="$1"
  cat > "$target" <<EOF
[Desktop Entry]
Type=Application
Name=GAASD-CARLA 场景面板
Comment=打开 GAASD-CARLA 联调场景管理面板
Exec=$LAUNCHER
Icon=$ICON
Terminal=false
Categories=Development;Simulation;
StartupNotify=true
EOF
  chmod +x "$target"
}

mkdir -p "$APP_DIR"
chmod +x "$LAUNCHER"
write_desktop_file "$APP_FILE"

DESKTOP_DIR="$(desktop_dir)"
if [ -d "$DESKTOP_DIR" ]; then
  DESKTOP_FILE="$DESKTOP_DIR/GAASD-CARLA 场景面板.desktop"
  write_desktop_file "$DESKTOP_FILE"
  if command -v gio >/dev/null 2>&1; then
    gio set "$DESKTOP_FILE" metadata::trusted true >/dev/null 2>&1 || true
  fi
  echo "Desktop launcher: $DESKTOP_FILE"
fi

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "$APP_DIR" >/dev/null 2>&1 || true
fi

echo "Application launcher: $APP_FILE"

