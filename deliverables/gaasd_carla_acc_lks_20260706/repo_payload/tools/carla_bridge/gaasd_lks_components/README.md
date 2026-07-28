# GAASD-CARLA LKS 单车闭环组件包

本组件包用于在 GAASD 画布中搭建单车车道保持验证链路。组件采用单输出或无输出包装形式，避免当前代码生成器对多输出端口处理不完整的问题。

## 组件

| 组件 | 方向 | 作用 |
|---|---|---|
| `CARLALKSLateralOffset` | CARLA 输入 | 输出横向偏差，正值表示车道中心位于自车右侧 |
| `CARLALKSHeadingError` | CARLA 输入 | 输出航向误差，正值对应向右修正 |
| `CARLALKSValid` | CARLA 输入 | 输出车道跟踪数据有效标志 |
| `CARLALKSControlCmd` | CARLA 输出 | 输入目标速度、转向角和使能，发布横纵向联合控制 |

## 推荐连线

```text
steerRad = limit(0.072 * lateralOffset + 0.48 * headingError, -0.15, 0.15)
CARLALKSControlCmd(targetSpeed=2.0, steerRad=steerRad, enable=valid)
```

运行依赖新版 `libcarla_gaasd_adapter.so`，该库需包含 `carla_adapter_read_lane_tracking` 符号。
