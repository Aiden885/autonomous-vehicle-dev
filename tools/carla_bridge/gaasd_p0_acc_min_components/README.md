# GAASD-CARLA ACC 最小闭环组件包

本组件包用于阶段二 ACC 最小闭环验证，目标是适配当前 GAASD 代码生成器“只给已连线端口传参”的行为。

## 组件列表

| 组件 | 输出/输入 | 用途 |
|---|---|---|
| `CARLAACCEgoSpeed` | 输出 `egoV` | 读取自车纵向速度 |
| `CARLAACCLeadSpeed` | 输出 `leadV` | 读取前车纵向速度 |
| `CARLAACCLeadDistance` | 输出 `distance` | 读取自车到前车净距离 |
| `CARLAACCComputeTargetSpeed` | 输入 `egoV`、`leadV`、`distance`，输出 `targetSpeed` | 计算 ACC 目标速度 |
| `CARLAACCLongitudinalCmd` | 输入 `speed`、`enable` | 发布 CARLA 纵向速度控制命令 |

## ACC 画布连线

| 来源端口 | 目标端口 |
|---|---|
| `CARLAACCEgoSpeed.egoV` | `CARLAACCComputeTargetSpeed.egoV` |
| `CARLAACCLeadSpeed.leadV` | `CARLAACCComputeTargetSpeed.leadV` |
| `CARLAACCLeadDistance.distance` | `CARLAACCComputeTargetSpeed.distance` |
| `CARLAACCComputeTargetSpeed.targetSpeed` | `CARLAACCLongitudinalCmd.speed` |
| 常量 `1` | `CARLAACCLongitudinalCmd.enable` |

## 说明

完整协议组件包仍保留在 `tools/carla_bridge/gaasd_p0_components/`。本包只用于优先跑通 ACC 闭环，不覆盖完整 CARLA 协议组件设计。
