# GAASD-CARLA LKS Pangu 旁路闭环执行方案

## 1. 测试目标

在 GAASD 代码生成和运行入口仍不稳定时，保留 `lks2` 画布算法结构，修复其生成代码的
确定性缺陷，并通过 Pangu 节点完成 CARLA 单车横向闭环。该测试验证的接口和算法结构可
直接用于后续正式集成，不使用 Bridge 内置车道保持替代 LKS。

## 2. 数据链路

```text
CARLA 路网和自车状态
  -> Python Bridge (ZMQ 5701)
  -> Pangu ZmqBridgeModule
  -> lks_input channel
  -> LKSModule (修复后的 lks2 生成代码)
  -> lks_output channel
  -> Pangu ZmqBridgeModule (ZMQ 5702)
  -> Python Bridge
  -> CARLA VehicleControl
```

Bridge 在 LKS 场景中配置 `lane_keep_enabled=false`。因此 CARLA 的转向响应只来自
`LKSModule` 输出的 `steerRad`。

## 3. 固定接口

输入：`egoV`、`c0`、`c1`、`c2`、`c3`、`curvature`、`brakePressed`、
`driverSteerNorm`。

输出：`steerRad`、`controlEnabled`。

没有物理方向盘和制动踏板时，测试平台通过 `driver_state_cmd` 发送驾驶员输入，Bridge
再发布 `driver_state`。车辆实际转角不得作为 `driverSteerNorm`，否则会把 LKS 自己的输出
误判为驾驶员接管。

## 4. 生成代码修复范围

`project/lks2/icvos/src/temp_codegen_output` 的修复只处理代码生成缺陷：

1. 恢复缺失的 LKS 全局参数。
2. 将复合组件实现名统一为头文件声明的 `run()`。
3. 补齐 `cmath` 数学函数声明。
4. 把顶层常量输入改为正式 LKS 输入结构。
5. 补齐三个预瞄点：`x1=0.5`、`x2=previewDistance/2`、`x3=previewDistance`。
6. 暴露 `lksSteerRad/controlEnabled/previewDistance/weightedError` 输出。

这些补丁不会改变画布公式。离线基准输入 `egoV=5 m/s、c0=0.8 m` 的结果为：

```text
previewDistance = 7.5 m
weightedError = 0.8 m
steerRad = 0.0384 rad
controlEnabled = 1
```

## 5. 场景设计

- 地图：CARLA 官方地图 `Town05`。
- 起点：`CarlaAcc` Git 历史提交 `5667b6d`、`dd934d8` 使用的固定道路点
  `(0.663731, -203.651886, 0.5)`。CARLA 0.9.15 实测投影到 `road=37、lane=-2`，
  航向约 `179.758deg`；场景脚本不硬编码航向，而是读取 waypoint 姿态。
- 路线条件：从该点沿当前车道向前至少 600 m 无分叉，包含连续缓弯，适合 LKS 验证。
- 自车：单车，无前车。
- 初始横向偏差：0 m，首轮只验证基本连续弯道能力。
- 初始航向误差：0 deg。
- 纵向目标速度：6.0 m/s，用于较快覆盖参考项目的连续道路和缓弯。
- LKS 最低横向启控速度：1 m/s；低于该速度只关闭 LKS 转向，场景纵向速度控制保持启用。
- 预期：横向偏差和航向误差逐步收敛，车辆保持在当前车道。

测试自动记录 `c0~c3`、曲率、横向偏差、道路编号、实际转角和碰撞事件，结果位于
`/tmp/lks2-pangu-carla/results/latest_summary.json`。

## 6. 执行顺序

先验证修复后的生成算法：

```bash
tools/pangu_lks_closed_loop/verify_generated_lks.sh
```

再构建 Pangu 模块：

```bash
tools/pangu_lks_closed_loop/build_pangu_module.sh
```

最后从调试页面选择 `lks2 生成代码修复版 LKS Pangu-CARLA 单车闭环测试`，或直接运行：

```bash
scenarios/lks2_pangu_carla_20260701/run.sh
```

停止：

```bash
scenarios/lks2_pangu_carla_20260701/stop.sh
```

调试页面中：`A/D` 分别模拟驾驶员向左/向右接管，`B` 模拟制动退出，按住生效、松开
归零；“测试结果”读取最新自动记录摘要。

## 7. 验收标准

1. Bridge 连续发布带 `c0~c3/curvature` 的 `lane_tracking`。
2. Pangu 的 `LKSModule` 和 `ZmqBridgeModule` 均保持运行。
3. Bridge 持续收到带非零 `steerRad` 的 `control_cmd`。
4. 初始偏差方向改变时，首个有效转角方向随之反转。
5. 车辆横向偏差收敛；驾驶员 A/D 接管时 LKS 转向贡献清零并由驾驶员转向接管车辆，
   制动时 LKS 退出且底盘产生有效制动。
6. 必须采集不少于 20 个有效弯道样本，弯道最大横向偏差不超过 0.8 m，且无碰撞。

## 8. 当前状态

离线生成代码、ZMQ 适配源码和 Pangu 动态库均已编译通过。CARLA 官方 `Town05` 基线
闭环已通过：最高车速 `5.1489 m/s`，681 个弯道样本，全程最大横向偏差 `0.30394 m`，
RMS 横向偏差 `0.17159 m`，无路口、无碰撞。

驾驶接管链路已经通过真实 CARLA 反馈验证：A/D 分别得到约 `-0.30/+0.30` 的底盘
`steer_norm`，B 得到 `brake=1.0`，释放后驾驶输入归零且车辆重新起步。Pangu 场景容器
需要 `--net=host --privileged`；缺少 `--privileged` 时 Pangu 配置路由失败并退出。

Town01 路线测试在路口急转处失败，该路线包含路径选择和近 90 度道路转换，不作为 LKS
算法验收工况。自定义地图 `acc_30km_new` 只保留为历史测试依据，不再作为交接包默认
环境。原 Simulink 没有独立曲率前馈，曲率只用于预瞄距离切换；当前保持三点预瞄控制
结构不变。
