# GAASD CARLA P1 组件包

本目录是 CARLA-GAASD 后续算法扩展用的 P1 组件包，放置位置与 P0 组件包并列。P1 目标是从 ACC 最小闭环扩展到通用障碍物输入、横向控制和横纵向联合控制。

## 组件清单

| 组件 | 方向 | 端口 |
|---|---|---|
| `CARLAObjectList` | CARLA 输入 | `maxObjects`，输出 `objectCount`、`objectId`、`objectType`、`objectX`、`objectY`、`objectYawRad`、`objectV`、`objectVx`、`objectVy`、`objectLength`、`objectWidth`、`objectHeight`、`valid` |
| `CARLALateralCmd` | CARLA 输出 | 输入 `steerRad`、`enable`，输出 `status` |
| `CARLAControlCmd` | CARLA 输出 | 输入 `targetSpeed`、`targetAccel`、`steerRad`、`enable`，输出 `status` |

## 使用边界

- `CARLAObjectList` 用于后续规划、避障、预测等算法读取通用障碍物，不替代 ACC 专用的 `CARLAACCLeadDistance` / `CARLAACCLeadSpeed`。
- `CARLALateralCmd` 和 `CARLAControlCmd` 最终都发布到 `gaasd.carla.control_cmd.v1`。同一时刻只应启用一个控制输出组件，避免 Bridge 收到多路控制命令。
- 当前 GAASD 代码生成器对数组端口和多输出端口的支持需要实际页面导入后继续验证；若只做 ACC 闭环，优先继续使用 `gaasd_p0_acc_min_components`。

## 运行依赖

生成工程运行时需要链接：

```text
objectCode/total/DLL/libcarla_gaasd_adapter.so
```

P1 组件依赖 adapter 中新增的以下 C ABI：

```text
carla_adapter_read_object_list
carla_adapter_publish_lateral_cmd
carla_adapter_publish_control_cmd
```
