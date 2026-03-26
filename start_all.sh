#!/bin/bash
# ============================================================
# 启动脚本：定位 + 规划 + 控制 联合运行
#
# ZMQ 数据流：
#   strThreadDemo  →[5003 IMU]→   planning
#   strThreadDemo  →[3151 底盘]→  planning
#   planning       →[5010 轨迹]→  controller
#   controller     →[5003 状态]→  (读取IMU)
#   controller     →[3171 控制]→  strThreadDemo (闭环)
#
# 用法：
#   ./start_all.sh          启动所有模块
#   ./start_all.sh stop     停止所有模块
#   ./start_all.sh status   查看运行状态
# ============================================================

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

PLANNING_BIN="$ROOT_DIR/huanyuan1/thicv-pilot/planningFigure/build/planning"
PLANNING_DIR="$ROOT_DIR/huanyuan1/thicv-pilot/planningFigure/build"

LOCALIZATION_BIN="$ROOT_DIR/Thread/build/strThreadDemo"
LOCALIZATION_DIR="$ROOT_DIR/Thread/build"

CONTROLLER_BIN="$ROOT_DIR/Controller/build/controller"
CONTROLLER_DIR="$ROOT_DIR/Controller/build"

LOG_DIR="$ROOT_DIR/logs"
PID_FILE="$ROOT_DIR/.pids"

start_all() {
    mkdir -p "$LOG_DIR"
    # Bug修复②：每次启动前清空旧PID文件，避免残留过期PID
    > "$PID_FILE"

    # 检查二进制文件存在
    for bin in "$PLANNING_BIN" "$LOCALIZATION_BIN" "$CONTROLLER_BIN"; do
        if [ ! -f "$bin" ]; then
            echo "❌ 找不到: $bin"
            echo "   请先在 CLion 中编译对应模块"
            exit 1
        fi
    done

    echo ">>> 启动定位模块 (strThreadDemo)..."
    cd "$LOCALIZATION_DIR"
    nohup ./strThreadDemo > "$LOG_DIR/localization.log" 2>&1 &
    echo $! >> "$PID_FILE"
    echo "    PID: $!  日志: logs/localization.log"

    sleep 1  # 等定位模块先绑定端口

    echo ">>> 启动规划模块 (planning)..."
    cd "$PLANNING_DIR"
    nohup env LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libxml2.so.2 ./planning > "$LOG_DIR/planning.log" 2>&1 &
    echo $! >> "$PID_FILE"
    echo "    PID: $!  日志: logs/planning.log"

    sleep 1

    echo ">>> 启动控制模块 (controller)..."
    cd "$CONTROLLER_DIR"
    nohup env LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libxml2.so.2 ./controller > "$LOG_DIR/controller.log" 2>&1 &
    echo $! >> "$PID_FILE"
    echo "    PID: $!  日志: logs/controller.log"

    echo ""
    echo "✅ 所有模块已启动，查看日志："
    echo "   tail -f $LOG_DIR/localization.log"
    echo "   tail -f $LOG_DIR/planning.log"
    echo "   tail -f $LOG_DIR/controller.log"
    echo ""
    echo "停止所有模块: ./start_all.sh stop"
}

stop_all() {
    if [ ! -f "$PID_FILE" ]; then
        echo "没有找到运行中的模块（$PID_FILE 不存在）"
        return
    fi

    echo ">>> 停止所有模块..."
    while read pid; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid"
            echo "    已停止 PID $pid"
        fi
    done < "$PID_FILE"

    rm -f "$PID_FILE"
    echo "✅ 所有模块已停止"
}

status_all() {
    echo "=== 模块运行状态 ==="
    for name in "strThreadDemo" "planning" "controller"; do
        # Bug修复①：pgrep 返回非0时不能让脚本退出，用 || true 屏蔽
        pid=$(pgrep -x "$name" 2>/dev/null || true)
        if [ -n "$pid" ]; then
            echo "  ✅ $name  (PID: $pid)"
        else
            echo "  ❌ $name  (未运行)"
        fi
    done
}

case "${1:-start}" in
    start)  start_all ;;
    stop)   stop_all ;;
    status) status_all ;;
    *)
        echo "用法: $0 {start|stop|status}"
        exit 1
        ;;
esac
