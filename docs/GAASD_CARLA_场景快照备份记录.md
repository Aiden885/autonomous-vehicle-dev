# GAASD-CARLA 场景快照备份记录

更新时间：2026-05-18

## 备份目标

保留当前已确认可快速复现的 ACC 跟车 CARLA-GAASD 联合仿真场景，后续可直接通过本机 UI 面板启动联调环境，用于演示、回归测试和继续调参。

## 当前可用入口

本机 UI 面板：

```bash
cd ~/文档/Modularization
python3 tools/gaasd_scenario_panel/app.py
```

浏览器打开：

```text
http://127.0.0.1:8765/
```

选择 `ACC 跟车 CARLA-GAASD 联合仿真快照`，点击“启动环境”。

## 已确认状态

2026-05-18 已确认：通过 UI 面板启动联调环境后，CARLA 页面会进入已确认的跟随视角，前车和自车会出现在设计位置，场景可用于后续 GAASD 示波器运行 ACC 闭环。

启动环境只负责 CARLA、Bridge、前车和视角。GAASD 算法仍通过 GAASD 软件中的示波器“开始”运行。

## 已保存内容

场景快照：

- `scenarios/acc_carla_phase2_20260513/scenario.yaml`
- `scenarios/acc_carla_phase2_20260513/run.sh`
- `scenarios/acc_carla_phase2_20260513/bridge_config.json`
- `scenarios/acc_carla_phase2_20260513/bridge_snapshot/tools/carla_bridge/`
- `scenarios/acc_carla_phase2_20260513/gaasd_project_snapshot/carla/`
- `scenarios/acc_carla_phase2_20260513/prebuilt/ubuntu-env/`

本机 UI：

- `tools/gaasd_scenario_panel/app.py`
- `tools/gaasd_scenario_panel/templates/index.html`
- `tools/gaasd_scenario_panel/static/style.css`
- `tools/gaasd_scenario_panel/static/app.js`

根目录工作版 CARLA 工具：

- `tools/carla_bridge/`

## 快照关键修复

- `reset-acc-straight-scene.py`：`ego_spawn_index=198` 超出 Town01 `spawn_points` 范围时，不再退出，改为使用 Bridge 已放置好的 ego 当前位姿。
- `set-spectator-follow.py`：后台视角跟随使用 `time.sleep()`，不再通过 `world.wait_for_tick()` 和 Bridge 抢 CARLA tick。
- `start-gaasd-carla-manual.sh`：后台跟随启动前先执行一次 `set-spectator-follow.py --once`，确保第一帧视角正确。
- GAASD 快照工程中保留了可编译的 `carla.h` 兼容补丁，避免示波器工程因 `protobuf-c` 头文件和交通灯枚举缺失而失败。

## 后续使用顺序

1. 启动 UI 面板并点击“启动环境”。
2. 等待日志显示 CARLA、Bridge、场景初始化完成。
3. 打开 GAASD 中恢复后的工程。
4. 在示波器中运行仿真，建议周期 `0.1s`。
5. 观测 `targetSpeed`、`egoV`、`leadV`、`distance`。

如果 GAASD 重新保存或重新生成代码后出现 `protobuf-c/protobuf-c.h` 缺失、`Infopack__TrafficLight__State` 未定义等编译错误，可以在 UI 面板中点击“修复生成代码”。该按钮会修复当前场景对应 GAASD 工程里的 `functions/carla.h` 和 `oscilloscopeFunctions/carla.h`。

如需恢复 GAASD 工程快照：

```bash
scenarios/acc_carla_phase2_20260513/restore_gaasd_project.sh
```
