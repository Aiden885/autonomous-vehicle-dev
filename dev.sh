#!/bin/bash
# 开发环境快捷脚本
# 用法：
#   ./dev.sh build   — 构建镜像（第一次或更新依赖时用）
#   ./dev.sh start   — 进入开发容器
#   ./dev.sh stop    — 停止容器
#   ./dev.sh status  — 查看容器状态

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

case "$1" in
  build)
    echo ">>> 构建 Docker 镜像（首次约需 10-20 分钟）..."
    sudo docker-compose build
    echo ">>> 镜像构建完成"
    ;;
  start)
    echo ">>> 启动开发容器..."
    # 允许 Docker 访问主机 X11 显示器（GUI支持）
    xhost +local:docker 2>/dev/null || true
    # 如果容器不存在则创建，存在则直接进入
    if sudo docker ps -a --format '{{.Names}}' | grep -q '^modularization-dev$'; then
      if ! sudo docker ps --format '{{.Names}}' | grep -q '^modularization-dev$'; then
        sudo docker start modularization-dev
      fi
      sudo docker exec -it modularization-dev /bin/bash
    else
      sudo docker-compose run --rm dev
    fi
    ;;
  stop)
    echo ">>> 停止容器..."
    sudo docker stop modularization-dev 2>/dev/null || true
    ;;
  status)
    sudo docker ps -a | grep modularization || echo "容器不存在，请先运行 ./dev.sh build"
    ;;
  *)
    echo "用法: $0 {build|start|stop|status}"
    echo ""
    echo "  build   构建 Docker 镜像"
    echo "  start   进入开发容器（自动挂载项目目录到 /workspace）"
    echo "  stop    停止容器"
    echo "  status  查看容器状态"
    exit 1
    ;;
esac
