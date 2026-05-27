# GAASD-CARLA ACC 闭环测试方案

版本：v0.1  
日期：2026-04-28  
关联文档：

- `docs/carla_gaasd_bridge_protocol.md`
- `docs/GAASD_CARLA_软件开发团队配合事项.md`

## 1. 目标

当前目标不是一次性完成 GAASD 软件产品化改造，而是先证明以下链路可行：

```text
CARLA 仿真环境
  -> Python Bridge
  -> ACC 算法
  -> Python Bridge
  -> CARLA ego 车辆控制
```

测试分两个阶段：

| 阶段 | 名称 | 是否依赖 GAASD 软件改造 | 目标 |
|---|---|---|---|
| 阶段一 | 独立 ACC Runner 闭环测试 | 否 | 验证 CARLA、Bridge、ACC 输入输出语义和控制链路可行 |
| 阶段二 | GAASD 组件包闭环测试 | 原则上否 | 验证 CARLA 适配组件能扫描进 GAASD，并在生成工程中跑 ACC 闭环 |

阶段一跑通后，说明算法链路和 Bridge 协议可行。阶段二跑通后，说明在不修改 GAASD 软件本体的情况下，可以通过组件包和适配库完成最小闭环验证。

## 2. 固定测试配置

第一轮测试只在当前机器上跑，配置可以先写死，避免把问题复杂化。

| 配置项 | 固定值 |
|---|---|
| CARLA 根目录 | `/home/aiden/snap/code/app/carla-package` |
| CARLA 版本 | `0.9.15` |
| CARLA host | `127.0.0.1` |
| CARLA port | `2000` |
| Bridge PUB | `tcp://127.0.0.1:5701` |
| Bridge SUB | `tcp://127.0.0.1:5702` |
| 协议 | `gaasd_carla_bridge` |
| 协议版本 | `0.3.0` |
| 仿真模式 | 同步模式，`fixed_delta_seconds=0.05` |
| Python | `python3.8` |

使用的 Bridge 工具目录：

```text
tools/carla_bridge/
```

## 3. 阶段一：独立 ACC Runner 闭环测试

### 3.1 测试目的

不经过 GAASD 软件，先验证 CARLA Bridge 和 ACC 算法本身能否闭环。

阶段一链路：

```text
CARLA
  -> carla_bridge.py
  -> acc_runner
  -> carla_bridge.py
  -> CARLA ego vehicle
```

### 3.2 运行时消息

ACC Runner 订阅 Bridge 发布的数据：

| topic | 用途 |
|---|---|
| `gaasd.carla.ego_state.v1` | 获取自车速度、位置、姿态 |
| `gaasd.carla.lead_vehicle.v1` | 获取前车速度、间距、相对速度 |
| `gaasd.carla.chassis_feedback.v1` | 可选，用于观察实际控制反馈 |

ACC Runner 发布控制命令：

| topic | 用途 |
|---|---|
| `gaasd.carla.control_cmd.v1` | 输出 ACC 目标速度或目标加速度 |

### 3.3 ACC Runner 实现要求

ACC Runner 可以先用 Python 实现，目标是快速度验证，不要求接入 GAASD。

最小输入：

| 字段 | 来源 | 单位 | 说明 |
|---|---|---|---|
| `egoV` | `ego_state.velocity.speed_mps` | m/s | 自车速度 |
| `leadV` | `lead_vehicle.lead_speed_mps` | m/s | 前车速度 |
| `distance` | `lead_vehicle.clearance_m` | m | 保险杠间距 |
| `valid` | `lead_vehicle.valid` | 1 | 是否存在前车 |

最小输出：

| 字段 | 发布到 | 单位 | 说明 |
|---|---|---|---|
| `target_speed_mps` | `control_cmd.target.target_speed_mps` | m/s | ACC 输出目标速度 |
| `enable` | `control_cmd.payload.enable` | 1 | 是否启用控制 |

无前车时：

```text
target_speed_mps = cruise_speed_mps
```

有前车时：

```text
target_speed_mps = ACC 算法根据 egoV、leadV、distance 计算
```

### 3.4 阶段一测试步骤

1. 启动 CARLA：

```bash
tools/carla_bridge/start-carla.sh --background
```

2. 检查 CARLA：

```bash
tools/carla_bridge/health-carla.sh --host 127.0.0.1 --port 2000
```

期望输出：

```text
[CARLA] healthy 127.0.0.1:2000
```

3. 启动 Bridge：

```bash
tools/carla_bridge/start-bridge.sh
```

4. 检查 Bridge 是否发布消息：

```bash
python3.8 tools/carla_bridge/probe-pub.py --duration 2 --min-messages 3 --max-print 8
```

期望能看到：

```text
gaasd.carla.ego_state.v1
gaasd.carla.lead_vehicle.v1
gaasd.carla.chassis_feedback.v1
```

5. 启动 ACC Runner。

6. 观察控制反馈：

```bash
python3.8 tools/carla_bridge/probe-pub.py \
  --topic-prefix gaasd.carla.chassis_feedback \
  --duration 3 \
  --min-messages 1 \
  --show-payload
```

7. 测试结束后停止：

```bash
tools/carla_bridge/stop-bridge.sh
tools/carla_bridge/stop-carla.sh
```

### 3.5 阶段一验收标准

| 验收项 | 通过条件 |
|---|---|
| CARLA 连接 | `health-carla.sh` 返回 healthy |
| Bridge 数据输入 | 能持续收到 `ego_state`、`lead_vehicle` |
| ACC 输出 | Runner 能持续发布 `control_cmd` |
| Bridge 控制接收 | `bridge_status.publish_counts.control_cmd_received` 持续增加 |
| 车辆控制 | `chassis_feedback.last_command_id` 更新，`throttle/brake` 有响应 |
| 安全停止 | 停止 Runner 后控制超时，Bridge 不继续使用旧命令 |

阶段一通过后，可以确认：

```text
CARLA + Bridge + ACC 算法链路可行。
```

## 4. 阶段二：GAASD 组件包闭环测试

### 4.1 测试目的

验证 CARLA 输入/输出组件能否以 GAASD 组件形式使用，并在 GAASD 生成工程中跑通 ACC 闭环。

阶段二链路：

```text
CARLA
  -> carla_bridge.py
  -> CARLA 输入组件
  -> GAASD ACC 算法组件
  -> CARLA 输出组件
  -> carla_bridge.py
  -> CARLA ego vehicle
```

### 4.2 判断依据

已经确认 GAASD 当前仿真生成工程具有以下特征：

1. 生成仿真主程序通过周期调用 `FuncStep()` 运行。
2. 没有明确的组件级 `init()` / `destroy()` 生命周期接口。
3. 生成工程 CMake 已支持 `nlohmann_json`。
4. `run_simulation.sh` 支持自动链接 `objectCode/total/DLL/*.so`。
5. `run_simulation.sh` 会启动 Docker 容器运行仿真，默认镜像为 `ubuntu:env`，并使用 `--network host`。
6. 因此可以通过外部动态库实现 ZMQ/JSON 适配层，但适配库实际会在 `ubuntu:env` 容器内被加载和运行。

结论：

```text
不建议让每个 GAASD 组件直接实现复杂 ZMQ/JSON。
建议用 carla_gaasd_adapter.so 封装 ZMQ/JSON，对 GAASD 暴露简单 C 函数。
```

重要前提：

```text
libcarla_gaasd_adapter.so 必须在与 ubuntu:env ABI 兼容的环境中编译。
最稳妥方式是在 ubuntu:env 容器内编译。
适配库依赖的动态库（例如 libzmq.so）也必须在 ubuntu:env 容器内可用。
```

### 4.3 阶段二实现结构

建议实现：

```text
GAASD 组件包装函数
  -> carla_gaasd_adapter.h
  -> carla_gaasd_adapter.so
  -> ZMQ/JSON
  -> carla_bridge.py
```

适配库位置建议：

```text
objectCode/total/DLL/libcarla_gaasd_adapter.so
```

GAASD 生成仿真时会通过 `PROJECT_EXTRA_LIBS` 自动链接该 `.so`。

适配库构建前必须先验证容器环境：

```bash
docker run --rm ubuntu:env ldconfig -p | grep zmq
```

如果容器内没有 `libzmq.so`，阶段二不能直接使用动态链接 ZMQ 的适配库。处理方式二选一：

```text
方案 A：在 ubuntu:env 中安装 libzmq3-dev，并基于更新后的镜像构建/运行。
方案 B：将 ZMQ 静态链接进 libcarla_gaasd_adapter.so，避免运行时依赖容器内 libzmq.so。
```

### 4.4 P0 组件范围

第一轮只做 ACC 最小闭环组件。

| 组件 | 类型 | 作用 |
|---|---|---|
| `CARLAEgoState` | 输入组件 | 订阅 `ego_state`，输出自车状态 |
| `CARLALeadVehicle` | 输入组件 | 订阅 `lead_vehicle`，输出前车状态 |
| `CARLALongitudinalCmd` | 输出组件 | 发布 `control_cmd`，输出目标速度 |
| `CARLAChassisFeedback` | 输入组件 | 订阅 `chassis_feedback`，观察控制反馈 |

P0 阶段暂不做：

```text
CARLAObjectList
CARLAControlCmd
CARLALateralCmd
CARLATrafficLightList
CARLARoute
CARLATrajectoryCmd
```

### 4.5 适配库 C 接口建议

适配库内部用 C++ 实现 ZMQ/JSON，对外暴露 C ABI。

示例接口：

```c
#ifdef __cplusplus
extern "C" {
#endif

int carla_adapter_init(void);

int carla_adapter_read_ego_state(
    double *egoV,
    double *egoX,
    double *egoY,
    double *egoYawRad,
    double *egoAcc,
    int *valid);

int carla_adapter_read_lead_vehicle(
    double *leadV,
    double *distance,
    double *relativeSpeed,
    double *ttc,
    int *valid);

int carla_adapter_read_chassis_feedback(
    double *speed,
    double *steer,
    int *mode,
    int *valid);

int carla_adapter_publish_longitudinal_cmd(
    double targetSpeed,
    int enable);

#ifdef __cplusplus
}
#endif
```

内部初始化策略：

```text
第一次调用任意接口时 lazy init。
进程退出时用 atexit() 做 socket/context 清理。
```

### 4.6 GAASD 组件包装函数建议

组件函数保持简单，只调用适配库。

示例：

```c
void CARLAEgoState(
    double *egoV,
    double *egoX,
    double *egoY,
    double *egoYawRad,
    double *egoAcc,
    int *valid)
{
    carla_adapter_read_ego_state(
        egoV,
        egoX,
        egoY,
        egoYawRad,
        egoAcc,
        valid);
}
```

```c
void CARLALongitudinalCmd(double speed, int enable)
{
    carla_adapter_publish_longitudinal_cmd(speed, enable);
}
```

### 4.7 ACC 画布连接

最小闭环连接：

| 来源组件 | 端口 | 接到 |
|---|---|---|
| `CARLAEgoState` | `egoV` | `accComputeTargetSpeed.egoV` |
| `CARLALeadVehicle` | `leadV` | `accComputeTargetSpeed.leadV` |
| `CARLALeadVehicle` | `distance` | `accComputeTargetSpeed.distance` |
| `accComputeTargetSpeed` | `targetSpeed` | `CARLALongitudinalCmd.speed` |

注意：

```text
distance 使用 lead_vehicle.clearance_m，即保险杠间距，不是中心点距离。
```

### 4.8 阶段二测试步骤

1. 生成或准备 CARLA P0 组件包。

组件包应包含：

```text
CARLAEgoState
CARLALeadVehicle
CARLALongitudinalCmd
CARLAChassisFeedback
```

2. 将组件包导入或扫描进 GAASD。

3. 准备适配库：

```text
objectCode/total/DLL/libcarla_gaasd_adapter.so
```

注意：

```text
libcarla_gaasd_adapter.so 须在 ubuntu:env 容器内，或 ABI 等价的系统上编译。
建议先执行 docker run --rm ubuntu:env ldconfig -p | grep zmq，
确认容器内 ZMQ 运行时库可用，再构建适配库。
```

4. 在 GAASD 画布搭建 ACC 最小闭环。

5. 生成工程。

6. 启动 CARLA 和 Bridge。

7. 运行 GAASD 生成工程。

8. 观察：

```text
bridge_status.publish_counts.control_cmd_received
chassis_feedback.last_command_id
CARLA ego 车辆速度变化
GAASD 画布/示波器中的 egoV、leadV、distance、targetSpeed
```

9. 停止 Bridge 和 CARLA。

### 4.9 阶段二验收标准

| 验收项 | 通过条件 |
|---|---|
| 组件导入 | GAASD 组件库中能看到 P0 CARLA 组件 |
| 画布连线 | ACC 最小闭环端口能正常连接 |
| 工程生成 | 生成工程无报错 |
| 动态库链接 | 生成工程能链接 `libcarla_gaasd_adapter.so` |
| 输入组件 | `CARLAEgoState`、`CARLALeadVehicle` 有实时输出 |
| 输出组件 | `CARLALongitudinalCmd` 能发布 `control_cmd` |
| Bridge 计数 | `control_cmd_received` 持续增加 |
| CARLA 控制 | ego 车辆对目标速度有响应 |

阶段二通过后，可以确认：

```text
GAASD 不改软件本体，也可以通过组件包 + 适配库完成 CARLA ACC 最小闭环。
```

## 5. 两阶段责任边界

### 5.1 我们当前可以自行完成

| 工作 | 说明 |
|---|---|
| CARLA 启停 | 使用 `tools/carla_bridge/start-carla.sh` |
| Bridge 启停 | 使用 `tools/carla_bridge/start-bridge.sh` |
| Bridge 协议验证 | 使用 `probe-pub.py`、`send-control.py` |
| 阶段一 ACC Runner | 自己实现，直接连 Bridge |
| 阶段二适配库 | 自己实现 `libcarla_gaasd_adapter.so` |
| 阶段二 P0 组件包 | 自己按 GAASD 组件包格式生成或扫描 |
| ACC 最小闭环测试 | 自己在当前机器完成 |

### 5.2 后续需要 GAASD 团队产品化配合

| 工作 | 说明 |
|---|---|
| 仿真管理界面 | 支持本地 CARLA、服务器 SSH、外部已启动三种模式 |
| 配置落盘 | 保存 CARLA 路径、Bridge endpoint、topic、协议版本 |
| 正式组件库 | 将 CARLA 组件内置到官方组件库 |
| 代码生成模板 | 正式支持 ZMQ/JSON 依赖和组件生命周期 |
| 多算法扩展 | 支持 `object_list`、`route`、`trajectory_cmd` 等通用规划控制组件 |

## 6. 当前关键风险

| 风险 | 影响 | 应对 |
|---|---|---|
| GAASD 组件包格式细节不完整 | 组件导入失败 | 先参考 THICV 组件包生成最小组件 JSON |
| 生成工程无法找到 `.so` | 阶段二链接失败 | 按 `objectCode/total/DLL/*.so` 放置，并检查 `PROJECT_EXTRA_LIBS` |
| `.so` 与 `ubuntu:env` ABI 不兼容 | 容器内加载失败 | 在 `ubuntu:env` 内编译适配库，或使用 ABI 等价环境 |
| 容器内缺少 `libzmq.so` | 适配库运行时加载失败 | 安装 `libzmq3-dev` 更新镜像，或静态链接 ZMQ |
| 组件无生命周期 | ZMQ 初始化/清理困难 | 用 lazy init + `atexit()` |
| JSON 解析放在 C 组件里复杂 | 实现成本高 | 用 C++ 适配库封装，对外暴露 C ABI |
| ACC 场景没有前车 | 无法验证跟车 | 阶段一先验证巡航，再补 NPC 前车场景 |
| 多控制组件同时发布 | Bridge 行为不确定 | 阶段二只启用 `CARLALongitudinalCmd` |

## 7. 后续执行顺序

建议按以下顺序推进：

1. 实现阶段一 `acc_runner`。
2. 使用当前 Bridge 跑通 `CARLA -> Bridge -> acc_runner -> Bridge -> CARLA`。
3. 记录测试日志和关键截图。
4. 实现 `libcarla_gaasd_adapter.so`。
5. 生成 P0 CARLA 组件包。
6. 导入 GAASD，搭建 ACC 最小闭环。
7. 生成工程并验证 `.so` 链接。
8. 跑通 GAASD 组件包闭环。
9. 将可行性结果整理给 GAASD 团队，作为后续软件产品化改造依据。

## 8. 阶段完成判据

最终可行性结论需要同时满足：

```text
阶段一通过：
  ACC Runner 能控制 CARLA ego 车辆。

阶段二通过：
  GAASD 画布中的 ACC 最小闭环能通过 CARLA 组件包控制 CARLA ego 车辆。
```

如果阶段一通过但阶段二失败，说明算法和 Bridge 可行，问题集中在 GAASD 组件包、代码生成或链接机制。

如果阶段一失败，应先修 Bridge 协议、ACC 输入输出语义或 CARLA 控制映射，不进入阶段二。

## 9. 阶段一测试记录

### 9.1 2026-04-28 本机 Python ACC Runner 闭环

测试环境：

| 项目 | 结果 |
|---|---|
| CARLA | `127.0.0.1:2000`，`0.9.15` |
| 地图 | `Carla/Maps/Town10HD_Opt` |
| Bridge PUB | `tcp://127.0.0.1:5701` |
| Bridge SUB | `tcp://127.0.0.1:5702` |
| Bridge tick | `fixed_delta_seconds=0.05`，约 20Hz |
| ego | `role_name=hero` |
| 前车 | `role_name=gaasd_lead` |

执行命令：

```bash
tools/carla_bridge/start-carla.sh --background
tools/carla_bridge/start-bridge.sh
python3.8 tools/carla_bridge/spawn-lead-vehicle.py --replace --distance-m 25 --speed-mps 2
python3.8 tools/carla_bridge/acc-runner.py --duration-sec 20 --cruise-speed-mps 5 --min-distance-m 8 --time-gap-sec 1.5
```

说明：`spawn-lead-vehicle.py --distance-m` 表示生成瞬间相对 ego 的初始距离。前车生成后会继续运动，后续采样到的 `lead_vehicle.clearance_m` 或 `longitudinal_distance_m` 大于初始距离属于正常现象，不影响前车识别链路验证。

关键结果：

| 检查项 | 结果 |
|---|---|
| Bridge 发布协议消息 | 通过，持续发布 `ego_state`、`object_list`、`lead_vehicle`、`chassis_feedback` |
| 前车识别 | 通过，`lead_vehicle.valid=true` |
| 控制命令接收 | 通过，`control_cmd_received=401` |
| ego 控制响应 | 通过，速度从 0 提升到约 3.6m/s |
| 跟车调速 | 通过，距离缩短后 ACC target speed 从 5m/s 降到约 4.2m/s |
| 控制超时制动 | 通过，ACC 停止后 `mode=0`、`throttle=0.0`、`brake=1.0` |

本轮测试结论：

```text
阶段一最小闭环通过：
CARLA -> Bridge -> Python ACC Runner -> Bridge -> CARLA 可以跑通。
```

本轮测试中修正的问题：

| 问题 | 修正 |
|---|---|
| Bridge 同步 tick 未按真实时间限速，仿真时间跑得过快 | 新增 `carla.realtime_pacing=true`，按 `fixed_delta_seconds` 限速 |
| 前车脚本按 waypoint 生成时可能落到 ego 侧向 | 默认改为 `ego_forward` 方式放置前车，保留 `lane_waypoint` 可选项 |
| ACC 停止后 CARLA 保留上一条 throttle | 新增 `control.timeout_brake=1.0`，控制超时后主动下发刹车 |

下一步：

```text
进入阶段二：实现 GAASD 组件包 + libcarla_gaasd_adapter.so，并在 GAASD 生成工程内复现同一条闭环。
```

### 9.2 2026-04-28 本机 Python ACC Runner 60 秒加固测试

测试目的：

```text
在阶段一最小闭环已跑通后，验证 Bridge 20Hz 发布、ACC 控制命令收发、前车识别和控制超时制动在 60 秒窗口内是否稳定。
```

执行命令：

```bash
tools/carla_bridge/start-carla.sh --background
tools/carla_bridge/start-bridge.sh
python3.8 tools/carla_bridge/spawn-lead-vehicle.py --replace --distance-m 25 --speed-mps 2
python3.8 tools/carla_bridge/acc-runner.py --duration-sec 60 --cruise-speed-mps 5 --min-distance-m 8 --time-gap-sec 1.5
```

测试环境：

| 项目 | 结果 |
|---|---|
| CARLA | `127.0.0.1:2000`，`0.9.15` |
| 地图 | `Carla/Maps/Town10HD_Opt` |
| Bridge tick | `fixed_delta_seconds=0.05`，约 20Hz |
| ego | `role_name=hero` |
| 前车 | `role_name=gaasd_lead` |

关键结果：

| 检查项 | 结果 |
|---|---|
| ACC Runner 命令数 | `commands=1201` |
| Bridge 接收命令数 | `control_cmd_received=1201` |
| Bridge 状态 | `connected=true`、`ego_spawned=true`、`last_error=""` |
| Bridge 发布频率 | 65 秒监控内 `sim_clock=1296`，约 19.9Hz |
| 前车有效性 | 65 秒监控内 `lead_vehicle.valid=1295/1295` |
| topic 序列跳变 | 未发现 `sequence_gaps` |
| ego 速度响应 | `chassis_speed_mps` 范围约 `[0.0, 3.66]` |
| 前车横向偏差 | `abs(lateral_distance_m)` 最大约 `0.29m` |
| 跟车距离变化 | `clearance_m` 从约 `181.53m` 收敛到约 `10.37m` |
| 跟车调速 | 距离接近后 ACC target speed 从 `5.0m/s` 降至约 `0.62m/s` |
| 控制超时制动 | ACC 停止后 `mode=0`、`throttle=0.0`、`brake=1.0` |

测试结论：

```text
阶段一 60 秒加固测试通过。
CARLA + Bridge + Python ACC Runner 在短时稳定性、命令计数一致性、前车有效性和超时制动上均满足进入阶段二的要求。
```

保留说明：

| 现象 | 判断 |
|---|---|
| 前车生成命令为 `--speed-mps 2`，测试中后段 `lead_speed_mps` 接近 0 | 由 CARLA 车辆物理/碰撞/阻力等仿真行为导致，阶段一关注的是 Bridge 是否持续识别并输出前车，不以 NPC 匀速保持作为验收项 |
| 前车生成命令为 `--distance-m 25`，采样距离大于 25m | 前车生成后继续运动，采样距离大于初始距离正常 |
| 60 秒测试只覆盖纵向 ACC | 横向控制、路线、交通灯、多目标复杂场景留到阶段二之后扩展 |

下一步：

```text
进入阶段二前置验证：
1. 确认 `ubuntu:env` Docker 镜像可用。
2. 确认容器内是否存在 `libzmq.so`。
3. 在容器内编译最小 `libcarla_gaasd_adapter.so` 或决定静态链接 ZMQ。
```

## 10. 阶段二前置验证记录

### 10.1 2026-04-28 Docker 与容器依赖验证

验证目的：

```text
确认 GAASD 生成工程的 Docker 运行环境是否能加载 CARLA Bridge 阶段二需要的外部适配库。
阶段二的真实目标是：GAASD 组件代码调用 libcarla_gaasd_adapter.so，再由适配库通过 ZMQ/JSON 与 carla_bridge.py 通信。
```

验证结论：

| 检查项 | 结果 |
|---|---|
| Docker 服务 | 可用，Docker Server `20.10.21` |
| GAASD 仿真镜像 | 已准备 `ubuntu:env`，同时保留标签 `gaasd-ubuntu-env-zmq:latest` |
| 当前镜像 ID | `sha256:97cf522cd1bdb113b58e382e8a7753276fc656ad9b07b237f35d312845830671` |
| glibc | `2.31` |
| 编译工具 | `gcc/g++/cmake/pkg-config` 可用 |
| JSON 依赖 | `nlohmann/json` 可用 |
| ZMQ 依赖 | `libzmq.so.5`、`libzmq.so`、`zmq.h` 可用 |

关键发现：

| 发现 | 处理 |
|---|---|
| 原始 `ubuntuenv.tar:env` 镜像没有 `libzmq.so` 和 `zmq.h` | 已基于该镜像构建派生镜像，并打上 `ubuntu:env` 标签 |
| 动态链接的最小适配库在原始镜像中会 `dlopen failed: libzmq.so.5` | 已在新 `ubuntu:env` 中安装 `libzmq3-dev` 后验证通过 |
| 尝试静态链接 ZMQ 时缺少 `gssapi_krb5/krb5/k5crypto/krb5support` 等静态依赖 | 阶段二优先采用动态链接方案，不走 ZMQ 静态链接 |

当前 `ubuntu:env` 的构建方式：

```dockerfile
FROM ubuntuenv.tar:env
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        libzmq3-dev \
        pkg-config \
    && rm -rf /var/lib/apt/lists/*
```

### 10.2 最小适配库 ABI 与加载验证

验证内容：

```text
在容器 ABI 环境中编译一个最小 libcarla_gaasd_adapter_probe.so。
该库同时使用 ZMQ 与 nlohmann::json，并导出 C ABI 函数 gaasd_adapter_probe。
再通过 dlopen/dlsym 验证动态库能被正常加载和调用。
```

验证结果：

| 检查项 | 结果 |
|---|---|
| 最小适配库编译 | 通过 |
| `ldd` 依赖检查 | 能解析到容器内 `libzmq.so.5` |
| `dlopen/dlsym` 调用 | 通过 |
| 返回 payload | `{"probe":"ok","protocol":"gaasd_carla_bridge","zmq_version":40302}` |

结论：

```text
libcarla_gaasd_adapter.so 必须在 `ubuntu:env` 容器内，或 ABI 等价的 Ubuntu 20.04/glibc 2.31 环境中编译。
不能默认在宿主机编译后直接放入 GAASD 工程，否则存在 glibc 或动态库依赖不匹配风险。
```

### 10.3 GAASD 生成工程外部库链接验证

验证内容：

```text
使用 GAASD 当前 dist/CMakeLists.txt 编译一个最小 FuncStep.c。
FuncStep.c 调用外部库中的 gaasd_adapter_probe。
通过 PROJECT_EXTRA_LIB_DIR 和 PROJECT_EXTRA_LIBS 模拟 run_simulation.sh 自动收集 objectCode/total/DLL/*.so 后的链接行为。
```

验证依据：

| 文件 | 已确认行为 |
|---|---|
| `/home/aiden/gaasd_server/codeTools/dist/run_simulation.sh` | 默认镜像为 `ubuntu:env`，会扫描 `objectCode/total/DLL/*.so` 并传入 `PROJECT_EXTRA_LIBS` |
| `/home/aiden/gaasd_server/codeTools/dist/CMakeLists.txt` | 会通过 `target_link_directories` 与 `target_link_libraries` 链接外部 `.so` |

验证结果：

| 检查项 | 结果 |
|---|---|
| CMake 配置 | 通过 |
| `module_sim` 编译 | 通过 |
| `module_sim` 链接外部适配库 | 通过 |
| 运行时依赖解析 | `libcarla_gaasd_adapter_probe.so => /work/preflight/libcarla_gaasd_adapter_probe.so` |
| ZMQ 运行时依赖解析 | `libzmq.so.5 => /lib/x86_64-linux-gnu/libzmq.so.5` |

阶段二前置验证结论：

```text
阶段二硬性环境前提已验证通过。
GAASD 生成工程可以在 `ubuntu:env` 容器内链接并解析外部适配库。
容器内 ZMQ/JSON/编译依赖已满足。
```

阶段二后续实际工作：

| 优先级 | 工作 |
|---|---|
| P0 | 实现真实 `libcarla_gaasd_adapter.so`，提供 `gaasd_carla_read_*` 与 `gaasd_carla_write_*` C ABI 接口 |
| P0 | 生成/整理 ACC 闭环最小组件包：`CARLAEgoState`、`CARLALeadVehicle`、`CARLALongitudinalCmd` |
| P0 | 将适配库放入 GAASD 工程 `objectCode/total/DLL/`，用 `run_simulation.sh` 跑通 GAASD 内部阶段二闭环 |
| P1 | 再扩展 `object_list`、`chassis_feedback`、`route`、`traffic_light_list` 等通用输入输出组件 |

## 11. 阶段二 P0 适配库测试记录

### 11.1 2026-04-29 真实适配库编译与 ABI 检查

测试目的：

```text
验证真实 libcarla_gaasd_adapter.so 能在 GAASD 目标容器 ubuntu:env 内编译，
并暴露 GAASD 组件可调用的 C ABI 接口。
```

执行环境：

| 项目 | 结果 |
|---|---|
| Docker 镜像 | `ubuntu:env` |
| ZMQ | `libzmq 4.3.2` |
| 适配库源码 | `tools/carla_bridge/adapter/` |
| 构建系统 | CMake |

已验证导出符号：

```text
carla_adapter_get_status
carla_adapter_init
carla_adapter_poll
carla_adapter_publish_longitudinal_cmd
carla_adapter_read_chassis_feedback
carla_adapter_read_ego_state
carla_adapter_read_lead_vehicle
carla_adapter_shutdown
```

动态依赖检查：

```text
libzmq.so.5 => /lib/x86_64-linux-gnu/libzmq.so.5
libstdc++.so.6
libgcc_s.so.1
```

结论：

```text
真实适配库可在 ubuntu:env 内编译，C ABI 符号满足 P0 组件包装函数调用需求。
```

### 11.2 2026-04-29 Smoke 与 Mock ZMQ 通信测试

Smoke 测试结果：

```text
read_ego_rc=0 valid=0 egoV=0.000 egoX=0.000 egoY=0.000 yaw=0.000 acc=0.000
publish_rc=0
```

说明：

```text
没有 Bridge 对端时，适配库初始化、读接口安全返回 valid=0，发布接口不崩溃。
```

Mock ZMQ 通信测试结果：

```text
mock_loop_ok
```

Mock 测试覆盖内容：

| 方向 | 覆盖 |
|---|---|
| 输入 | mock 发布 `ego_state`，适配库正确解析 `velocity.speed_mps`、`pose.x_m/y_m/yaw_rad`、`acceleration.longitudinal_mps2` |
| 输入 | mock 发布 `lead_vehicle`，适配库正确解析 `lead_speed_mps`、`clearance_m`、`relative_speed_mps`、`ttc_sec`、`valid` |
| 输入 | mock 发布 `chassis_feedback`，适配库正确解析 `speed_mps`、`steer_rad`、`mode` |
| 输出 | 适配库发布 `control_cmd`，mock 能收到 `target.target_speed_mps` 和 `enable` |

结论：

```text
适配库的 topic、JSON message、payload 字段映射通过隔离测试。
```

### 11.3 2026-04-29 连接真实 CARLA Bridge 短测

测试目的：

```text
不进入 GAASD 画布，直接用容器内适配库连接真实 Bridge，
验证 libcarla_gaasd_adapter.so <-> carla_bridge.py <-> CARLA 的读写链路。
```

测试准备：

```bash
tools/carla_bridge/start-carla.sh --background
tools/carla_bridge/start-bridge.sh
python3.8 tools/carla_bridge/spawn-lead-vehicle.py --replace --distance-m 25 --speed-mps 2
```

执行测试：

```bash
docker run --rm --network host \
  -v /home/aiden/文档/Modularization/tools/carla_bridge/adapter:/work/adapter \
  ubuntu:env bash -lc '
    rm -rf /tmp/carla_adapter_build
    cmake -S /work/adapter -B /tmp/carla_adapter_build -DCMAKE_BUILD_TYPE=Release
    cmake --build /tmp/carla_adapter_build -- -j2
    LD_LIBRARY_PATH=/tmp/carla_adapter_build \
      /tmp/carla_adapter_build/carla_gaasd_adapter_bridge_live 8 1.5
  '
```

关键结果：

| 检查项 | 结果 |
|---|---|
| Bridge 发布探测 | 通过，`ego_state`、`object_list`、`lead_vehicle`、`chassis_feedback` 持续发布 |
| 前车生成 | 通过，`gaasd_lead` id=`25`，初始距离 `25m`，速度 `2m/s` |
| 适配库 live 测试 | `live_bridge_result ego_valid=202 lead_valid=202 chassis_valid=202 commands=122` |
| Bridge 接收命令 | `bridge_status.publish_counts.control_cmd_received=122` |
| 控制反馈 | `chassis_feedback.last_command_id=122` |
| 超时安全 | 测试结束后 Bridge 进入 `mode=0`、`brake=1.0` |

结论：

```text
阶段二 P0 的外部通信半链路通过：
容器内 libcarla_gaasd_adapter.so 可以连接真实 Bridge，
读取 CARLA 输入消息，并向 Bridge 发布 control_cmd。
```

下一步：

```text
实现 GAASD P0 组件包装函数：
CARLAEgoState
CARLALeadVehicle
CARLALongitudinalCmd

然后将适配库放入 objectCode/total/DLL/，
用 run_simulation.sh 验证 GAASD 生成工程内的最小 FuncStep 闭环。
```

## 12. 阶段二 P0 最小 FuncStep 加载测试记录

### 12.1 Claude 审核后修复项

Claude 对第 11 节测试的审核结论：

```text
认可“阶段二 P0 适配库通信链路已通过”，但边界是“适配库独立通信”，不是“GAASD FuncStep/画布闭环已通过”。
进入下一步前需要验证真实 run_simulation.sh 能加载并调用适配库。
```

已处理的问题：

| 问题 | 处理 |
|---|---|
| `libcarla_gaasd_adapter.so` 设置 `VERSION/SOVERSION` 后可能产生 `libcarla_gaasd_adapter.so.0` 运行时依赖 | 已去掉 CMake `VERSION/SOVERSION`，现在只生成单个 `libcarla_gaasd_adapter.so`，ELF SONAME 也是 `libcarla_gaasd_adapter.so` |
| 适配库读取 `chassis_feedback.steer_rad`，但 Bridge 实际发布 `steering_angle_rad` | 已修正为读取 `steering_angle_rad`，mock 测试同步更新 |

修复后验证：

```text
readelf -d libcarla_gaasd_adapter.so | grep SONAME
Library soname: [libcarla_gaasd_adapter.so]

adapter_smoke_test: 通过
adapter_mock_loop_test: mock_loop_ok
```

### 12.2 最小 GAASD FuncStep 测试工程

测试工程：

```text
tools/carla_bridge/gaasd_p0_smoke_project/
```

工程内容：

| 文件 | 作用 |
|---|---|
| `FuncStep.c` | 直接调用 `carla_adapter_init`、`carla_adapter_read_ego_state`、`carla_adapter_read_lead_vehicle`、`carla_adapter_publish_longitudinal_cmd` |
| `icvos/blocks/functions/dictionaryData/struct.json` | 满足 `run_simulation.sh` 当前工程检查 |
| `objectCode/total/DLL/libcarla_gaasd_adapter.so` | 阶段二真实适配库 |

说明：

```text
该工程不是 GAASD 画布导出的完整工程，只用于验证 GAASD 运行框架能否真实加载并调用适配库。
```

### 12.3 无 Bridge 场景加载验证

测试目的：

```text
不启动 CARLA/Bridge，只验证 run_simulation.sh 能链接 libcarla_gaasd_adapter.so，
并执行 FuncStep。此时输入 valid 应为 0，发布接口不应崩溃。
```

执行命令：

```bash
/home/aiden/gaasd_server/codeTools/dist/run_simulation.sh \
  -i ubuntu:env \
  -c gaasd_p0_smoke_container \
  /home/aiden/文档/Modularization/tools/carla_bridge/gaasd_p0_smoke_project \
  0.05 0.2 39091
```

由于 `scope_push_init_and_wait` 会等待 WebSocket 客户端，需要用一个临时 WebSocket 客户端连接端口 `39091` 后仿真才会开始。

关键结果：

| 检查项 | 结果 |
|---|---|
| `objectCode/total/DLL` 扫描 | 通过，附加库数量 `1` |
| `module_sim` 编译链接 | 通过 |
| `FuncStep` 执行 | 通过，共 `5` 步 |
| 适配库初始化 | `init rc=0` |
| 无 Bridge 输入 | `egoValid=0`、`leadValid=0`，符合预期 |
| 仿真结束 | `module_sim 正常结束` |

结论：

```text
GAASD run_simulation.sh 可以真实链接并调用 libcarla_gaasd_adapter.so。
无 Bridge 时适配库安全降级，FuncStep 不崩溃。
```

### 12.4 真实 CARLA/Bridge 场景 FuncStep 联通验证

测试目的：

```text
启动 CARLA 和 Bridge 后，用同一个最小 FuncStep 验证：
GAASD module_sim -> libcarla_gaasd_adapter.so -> Bridge -> CARLA
这条链路可以读到真实输入并发出 control_cmd。
```

测试准备：

```bash
tools/carla_bridge/start-carla.sh --background
tools/carla_bridge/start-bridge.sh
python3.8 tools/carla_bridge/spawn-lead-vehicle.py --replace --distance-m 25 --speed-mps 2
```

执行命令：

```bash
/home/aiden/gaasd_server/codeTools/dist/run_simulation.sh \
  -i ubuntu:env \
  -c gaasd_p0_smoke_container_live \
  /home/aiden/文档/Modularization/tools/carla_bridge/gaasd_p0_smoke_project \
  0.05 1.0 39092
```

同样需要临时 WebSocket 客户端连接端口 `39092`，触发 `module_sim` 开始执行。

关键结果：

| 检查项 | 结果 |
|---|---|
| Bridge 发布探测 | 通过，`ego_state` / `object_list` 等消息持续发布 |
| 前车生成 | 通过，`gaasd_lead` id=`25` |
| `module_sim` 编译链接 | 通过，附加库数量 `1` |
| `FuncStep` 执行 | 通过，共 `21` 步 |
| 自车输入 | 每步 `egoValid=1`，`egoV=0.000` |
| 前车输入 | 每步 `leadValid=1`，`leadV=0.023`，`distance=181.445` |
| 控制输出 | 每步发布 `target=1.000`、`enable=1` |
| Bridge 接收命令 | `bridge_status.publish_counts.control_cmd_received=21` |
| Bridge 控制反馈 | `chassis_feedback.last_command_id=21` |
| 仿真结束 | `module_sim 正常结束` |

结论：

```text
阶段二 P0 的 GAASD 运行框架联通验证通过。
最小 FuncStep 可以在 run_simulation.sh 启动的 module_sim 内调用适配库，
读取真实 Bridge 输入，并向 Bridge 发布 control_cmd。
```

当前边界：

```text
这一步仍不是 GAASD 画布组件闭环。
它验证的是“生成工程运行框架 + FuncStep + 适配库 + Bridge”的最小链路。
下一步才是把 FuncStep 中的直接调用拆成 GAASD 组件包装函数并通过画布连线执行。
```

下一步：

```text
已实现 P0 组件包装函数并生成扫描组件包。
随后用 GAASD 导入这些组件，在画布中连接 ACC 最小闭环。
```

## 13. 阶段二 P0 组件包准备记录

### 13.1 目标

把最小 FuncStep 中直接调用适配库的逻辑拆成 GAASD 可导入、可连线的 CARLA 组件。

### 13.2 产物

| 产物 | 路径 |
|---|---|
| 组件源码 | `tools/carla_bridge/gaasd_p0_components_src/carlaP0Components.c` |
| 扫描组件包 | `tools/carla_bridge/gaasd_p0_components/` |
| 组件包压缩文件 | `tools/carla_bridge/gaasd_carla_p0_components.tar.gz` |
| 页面操作步骤 | `docs/GAASD_CARLA_阶段二画布测试操作步骤.md` |

### 13.3 组件清单

| 组件 | 端口 |
|---|---|
| `CARLAEgoState` | 输出 `egoV`、`egoX`、`egoY`、`egoYawRad`、`egoAcc`、`valid` |
| `CARLALeadVehicle` | 输出 `leadV`、`distance`、`relativeSpeed`、`ttc`、`valid` |
| `CARLALongitudinalCmd` | 输入 `speed`、`enable`，输出 `status` |
| `CARLAChassisFeedback` | 输出 `speed`、`steerRad`、`mode`、`valid` |

### 13.4 命令行验证

验证项：

| 项目 | 结果 |
|---|---|
| `codescan` 扫描 | 通过，生成 4 个组件 JSON 和 `cbdes.db` |
| C 源码编译 | 通过，`cc -std=c99 -Wall -Wextra -fPIC -c` 无警告 |
| JSON 格式检查 | 通过 |
| 压缩包检查 | 通过，包含 `component/`、`dictionaryData/`、`cbdes.db` |

当前边界：

```text
组件包已经准备完成，但还没有经过 GAASD 页面导入、画布连线和真实画布闭环运行。
下一步需要在 GAASD 软件中导入组件包并搭建 ACC 最小闭环。
```
