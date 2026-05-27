# LKS 基础模块版 CARLA-GAASD 单车联合仿真场景

本场景用于验证 GAASD 画布中的车道保持控制链路。测试仅包含自车，不生成前车，也不同时验证 ACC。

## 已准备内容

- Bridge 发布 `gaasd.carla.lane_tracking.v1`，包含 `lateral_offset_m`、`heading_error_rad` 和 `valid`。
- adapter 提供 `carla_adapter_read_lane_tracking()` 和联合控制发布接口。
- 场景启动后将自车置于直道路段，初始横向偏差为 `0.8 m`，初始航向误差为 `5 deg`。
- `bridge_config.json` 已关闭 `lane_keep_enabled`，转向必须由 GAASD 画布输出。
- adapter 构建命令为 `tools/carla_bridge/build-adapter-ubuntu-env.sh`；生成工程后将输出库放入该工程的 `objectCode/total/DLL/`。

## GAASD 画布设计

使用 `tools/carla_bridge/gaasd_lks_components/` 中的四个组件：

| 组件 | 用途 |
|---|---|
| `CARLALKSLateralOffset` | 输出横向偏差 `lateralOffset` |
| `CARLALKSHeadingError` | 输出航向误差 `headingError` |
| `CARLALKSValid` | 输出数据有效标志 `valid` |
| `CARLALKSControlCmd` | 接收目标速度、转向角和使能，发送车辆控制命令 |

第一版基础模块控制公式：

```text
steerRad = limit(0.072 * lateralOffset + 0.48 * headingError, -0.15, 0.15)
targetSpeed = 2.0
enable = valid
```

建议示波器观测 `lateralOffset`、`headingError`、`steerRad` 和自车速度。

## 启动场景

通过本地场景面板选择本场景并点击启动，或直接运行：

```bash
scenarios/lkspro1_basic_carla_20260526/run.sh
```

环境就绪后，在 GAASD 中打开新建的 LKS 画布工程并启动示波器仿真。

停止环境：

```bash
scenarios/lkspro1_basic_carla_20260526/stop.sh
```
