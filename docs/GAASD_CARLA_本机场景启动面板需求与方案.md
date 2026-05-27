# GAASD-CARLA 本机场景启动面板需求与方案

## 1. 目标

本工具用于在本机快速复现和调试 GAASD-CARLA 联调场景。它只负责管理已保存的 `scenarios/` 快照，并把常用命令封装成 UI 按钮，方便恢复工程、启动环境、停止环境和查看日志。

工具不替代 GAASD 的画布、示波器、仿真运行和结果分析能力。

## 2. 使用边界

本工具提供：

1. 扫描 `scenarios/` 下的测试快照。
2. 显示场景名称、说明、CARLA 配置和 GAASD 工程恢复路径。
3. 调用场景内的 `restore_gaasd_project.sh` 恢复 GAASD 工程。
4. 调用场景内的 `run.sh` 启动 CARLA、Bridge 和对应测试工况。
5. 调用场景内的停止脚本清理 CARLA、Bridge、视角跟随和 GAASD 仿真进程。
6. 显示实时执行日志。
7. 检查 CARLA 与 Bridge 端口状态。
8. 提供复制 GAASD 工程路径和查看场景说明的入口。

本工具不提供：

1. GAASD 画布编辑。
2. 示波器替代功能。
3. 测试结果曲线分析。
4. 参数调优平台。
5. 远程部署管理。

## 3. 数据来源

每个场景以一个目录表示，例如：

```text
scenarios/acc_carla_phase2_20260513/
├── scenario.yaml
├── run.sh
├── restore_gaasd_project.sh
├── bridge_config.json
├── bridge_snapshot/
├── gaasd_project_snapshot/
└── README.md
```

启动面板以 `scenario.yaml` 为主索引，按目录自动扫描，不单独维护一份重复的项目注册表。

## 4. 第一版页面

页面分为三块：

1. 左侧场景列表：展示所有可用快照。
2. 右侧场景卡片：展示当前场景的关键配置和操作按钮。
3. 底部日志窗口：显示恢复、启动、停止和健康检查输出。

主要按钮：

1. 恢复 GAASD 工程
2. 启动联调环境
3. 停止环境
4. 健康检查
5. 复制工程路径
6. 查看说明

执行恢复、启动或停止时，页面按钮应进入禁用状态，避免用户连续点击造成重复操作。

## 5. 实现方式

采用 `Python + Flask + 浏览器页面`：

1. Flask 后端负责扫描场景、执行脚本、检查端口、返回日志。
2. 前端通过 `fetch` 调用后端接口。
3. 日志采用轮询方式刷新，避免引入复杂的长连接状态管理。
4. 页面绑定到 `0.0.0.0:8765`，便于本机浏览器或 Web Preview 打开。
5. 场景 ID 只允许使用字母、数字、点、下划线和短横线，并且后端会校验场景目录必须位于 `scenarios/` 内。

## 6. 启动方式

```bash
cd ~/文档/Modularization
python3 tools/gaasd_scenario_panel/app.py
```

浏览器打开：

```text
http://127.0.0.1:8765
```

## 7. 当前默认场景

当前已保存的可复现场景：

```text
scenarios/acc_carla_phase2_20260513
```

该场景用于复现 ACC 跟车 CARLA-GAASD 联合仿真。
