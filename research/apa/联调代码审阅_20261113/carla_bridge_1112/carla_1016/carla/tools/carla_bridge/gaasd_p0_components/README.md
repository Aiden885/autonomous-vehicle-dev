# GAASD CARLA P0 组件包

本目录是阶段二 ACC 最小闭环用的 CARLA 组件包，由 `tools/carla_bridge/gaasd_p0_components_src/carlaP0Components.c` 通过 GAASD `codescan` 生成。

## 组件清单

| 组件 | 方向 | 端口 |
|------|------|------|
| `CARLAEgoState` | CARLA 输入 | `egoV`、`egoX`、`egoY`、`egoYawRad`、`egoAcc`、`valid` |
| `CARLALeadVehicle` | CARLA 输入 | `leadV`、`distance`、`relativeSpeed`、`ttc`、`valid` |
| `CARLALongitudinalCmd` | CARLA 输出 | 输入 `speed`、`enable`，输出 `status` |
| `CARLAChassisFeedback` | CARLA 输入/观测 | `speed`、`steerRad`、`mode`、`valid` |

## ACC 最小闭环连线

| 来源 | 目标 |
|------|------|
| `CARLAEgoState.egoV` | `accComputeTargetSpeed.egoV` |
| `CARLALeadVehicle.leadV` | `accComputeTargetSpeed.leadV` |
| `CARLALeadVehicle.distance` | `accComputeTargetSpeed.distance` |
| `accComputeTargetSpeed.targetSpeed1` | `CARLALongitudinalCmd.speed` |
| 常量 `1` | `CARLALongitudinalCmd.enable` |

`CARLALongitudinalCmd.status` 为 `0` 表示命令发布成功；非 `0` 时优先检查 Bridge 是否已启动、`libcarla_gaasd_adapter.so` 是否被工程链接。

## 运行依赖

生成工程运行时需要链接：

```text
objectCode/total/DLL/libcarla_gaasd_adapter.so
```

该动态库由 `tools/carla_bridge/adapter/` 构建，已经验证可在 `ubuntu:env` 容器内被 `run_simulation.sh` 加载。

## GAASD 页面操作边界

本目录只准备组件包文件。导入组件、画布拖拽、连线、启动仿真需要在 GAASD 软件页面中完成。
