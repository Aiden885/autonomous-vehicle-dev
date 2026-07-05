# GAASD-CARLA ACC/LKS 开发集成说明

## 1. 集成目标

本说明用于把已经验证的 CARLA Bridge、驾驶输入、Pangu 节点和场景脚本集成到 GAASD
运行环境。算法画布和 CARLA 之间只通过固定接口交换数据，场景辅助速度、测试按键和相机
不作为算法接口。

## 2. 链路和端口

```text
CARLA
  -> Python Bridge PUB :5701
  -> Pangu ZmqBridgeModule
  -> acc_input / lks_input channel
  -> ACCModule / LKSModule
  -> acc_output / lks_output channel
  -> Pangu ZmqBridgeModule
  -> Python Bridge CONTROL :5702
  -> CARLA VehicleControl
```

- `5701`：Bridge 发布 CARLA 状态和经 Bridge 归一化后的驾驶输入。
- `5702`：Pangu 发布车辆控制命令；测试键盘也向该端口发送驾驶输入命令，再由 Bridge
  转发到 `5701`，保证 Pangu 只订阅一条输入总线。
- 容器必须使用 `--net=host`。当前 Pangu `run.sh` 还会配置主机路由，因此测试容器需要
  `--privileged`，否则会出现 `SIOCADDRT: Operation not permitted` 并退出。

## 3. ACC 固定接口

### 3.1 算法输入

| 字段 | 类型 | 单位 | 来源 |
| --- | --- | --- | --- |
| `egoV` | `double` | m/s | `ego_state` |
| `leadV` | `double` | m/s | `lead_vehicle` |
| `distance` | `double` | m | `lead_vehicle.clearance_m` |
| `commandType` | `int` | 1 | `driver_command` |

### 3.2 算法输出

| 字段 | 类型 | 单位 | 去向 |
| --- | --- | --- | --- |
| `targetSpeed` / `speed` | `double` | m/s | `control_cmd.target_speed_mps` |
| `enable` | `int` | 1 | `control_cmd.enable` |

### 3.3 ACC 按键

`commandType` 是单周期事件：有指令时输出一次 `1..7`，下一周期恢复 `0`。

| 按键 | `commandType` | 含义 |
| --- | ---: | --- |
| E | 1 | 降低设定速度；待命无历史时按当前速度启控 |
| Q | 2 | 提高设定速度；待命有历史时继承参数启控 |
| T | 3 | 减小时距 |
| R | 4 | 增大时距 |
| W | 5 | 驾驶员油门事件 |
| S | 6 | 驾驶员制动并退出 ACC |
| C | 7 | 取消 ACC |

## 4. LKS 固定接口

### 4.1 算法输入

| 字段 | 类型 | 单位 | 来源 |
| --- | --- | --- | --- |
| `egoV` | `double` | m/s | `ego_state` |
| `c0` | `double` | m | `lane_tracking.c0_m` |
| `c1` | `double` | 1 | `lane_tracking.c1` |
| `c2` | `double` | 1/m | `lane_tracking.c2_per_m` |
| `c3` | `double` | 1/m^2 | `lane_tracking.c3_per_m2` |
| `curvature` | `double` | 1/m | `lane_tracking.curvature_per_m` |
| `brakePressed` | `int` | 1 | `driver_state.brake_pressed` |
| `driverSteerNorm` | `double` | 1 | `driver_state.driver_steer_norm` |

`driverSteerNorm` 必须是驾驶员方向盘输入，不能回填车辆实际转角。否则 LKS 自身产生的
转向会被误判为驾驶员接管。

### 4.2 算法输出

| 字段 | 类型 | 单位 | 去向 |
| --- | --- | --- | --- |
| `steerRad` / `lksSteerRad` | `double` | rad | `control_cmd.steer_rad` |
| `controlEnabled` | `int` | 1 | 横向控制选择 |

LKS 只负责横向控制。当前单车场景由 `ZmqBridgeModule` 提供固定纵向目标速度，该速度不属于
LKS 算法输出。

### 4.3 LKS 驾驶接管按键

当前 LKS 无总开关：`egoV >= 1m/s`、未制动且无驾驶员大转角时自动在控。

| 按键 | 按住时输入 | 作用 |
| --- | --- | --- |
| A | `driverSteerNorm=-0.3` | 驾驶员向左接管，LKS 横向输出退出 |
| D | `driverSteerNorm=+0.3` | 驾驶员向右接管，LKS 横向输出退出 |
| B | `brakePressed=1` | 制动并退出 LKS |
| 松键 | 两字段归零 | 允许 LKS 自动恢复 |

A/D/B 是持续电平，不是事件脉冲。发布周期建议 `20Hz`；Bridge 超过 `0.75s` 未收到新输入
时自动清零。

## 5. ZMQ topic

| 方向 | topic | 用途 |
| --- | --- | --- |
| Bridge -> Pangu | `gaasd.carla.ego_state.v1` | 自车状态 |
| Bridge -> Pangu | `gaasd.carla.lead_vehicle.v1` | ACC 前车状态 |
| Bridge -> Pangu | `gaasd.carla.lane_tracking.v1` | LKS 车道多项式和曲率 |
| Bridge -> Pangu | `gaasd.carla.driver_command.v1` | ACC 驾驶指令 |
| Bridge -> Pangu | `gaasd.carla.driver_state.v1` | LKS 驾驶员转向和制动输入 |
| Pangu -> Bridge | `gaasd.carla.control_cmd.v1` | 纵向目标速度和横向转角 |
| 测试端 -> Bridge | `gaasd.carla.driver_state_cmd.v1` | LKS 键盘测试输入 |

消息体采用 JSON，协议版本为 `0.3.0`。正式字段定义以
`docs/GAASD_CARLA_ACC_LKS_接口变量清单.md` 和 Bridge 源码为准。

## 6. Docker 放置建议

```text
/opt/carla/carla_tools/
  tools/carla_bridge/          Python Bridge 和 CARLA 场景辅助脚本
  scenario/acc/                ACC 场景脚本与配置
  scenario/lks/                LKS 场景脚本与配置
  driver_input/                ACC/LKS 键盘发布器

Pangu 源码树/modules/
  ACCModule/                   ACCModule + ZmqBridgeModule
  LKSModule/                   LKSModule + ZmqBridgeModule
```

Pangu 动态库必须在目标 Pangu 镜像或 ABI 等价环境内编译，不能直接复用其他 Ubuntu 环境
生成的 `.so`。LKS 构建还需要包内 `project/lks2/icvos/src/temp_codegen_output` 的修复版
GAASD 生成代码。

## 7. 启动顺序

1. 启动 CARLA，等待 Python API 可连接。
2. 启动 Python Bridge，确认 `5701/5702` 就绪。
3. 重置 ACC 或 LKS 场景。
4. 启动 Pangu 容器和业务节点。
5. 确认 `ZmqBridgeModule` 已订阅后再发送 ACC 单周期启控指令。
6. 执行闭环测试并保存消息计数、车辆反馈和碰撞记录。

## 8. 已验证结果

- ACC：CARLA、Bridge、Pangu 生成代码修复版闭环可运行，驾驶指令序列 mock 测试通过。
- LKS：CARLA 官方 `Town05` 连续弯道基线通过，最大横向偏差约 `0.304m`、RMS 约
  `0.172m`，无碰撞；默认场景不依赖自定义地图。
- LKS 接管：A/D 实测底盘 `steer_norm=-0.30/+0.30`；B 实测 `brake=1.0`；释放后
  输入归零并恢复行驶。
