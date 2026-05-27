# GAASD-CARLA ACC 阶段一测试 Claude 审阅材料

审阅日期：2026-04-28

## 1. 审阅目标

请基于本材料审阅本轮阶段一测试是否足以支持以下结论：

```text
CARLA -> Bridge -> Python ACC Runner -> Bridge -> CARLA 最小闭环已经跑通。
```

请重点检查：

| 关注点 | 需要判断的问题 |
|---|---|
| 测试链路 | 是否真的形成闭环，而不是只验证了单向通信 |
| Bridge 改动 | `realtime_pacing` 和控制超时刹车是否合理 |
| ACC Runner | 简化 ACC 公式是否足以作为阶段一验证工具 |
| 前车场景 | `ego_forward` 放置前车是否会掩盖真实路网/车道问题 |
| 验收结论 | 当前证据是否支持进入阶段二 GAASD 组件包测试 |

## 2. 当前文件状态

本轮相关文件在根仓库中目前仍是未跟踪状态，尚未提交：

```text
?? docs/GAASD_CARLA_ACC_闭环测试方案.md
?? tools/carla_bridge/
```

涉及的主要文件：

| 文件 | 作用 |
|---|---|
| `tools/carla_bridge/carla_bridge.py` | CARLA 与 GAASD JSON/ZMQ 协议 Bridge |
| `tools/carla_bridge/acc-runner.py` | 阶段一独立 Python ACC Runner |
| `tools/carla_bridge/spawn-lead-vehicle.py` | 在 ego 前方生成测试前车 |
| `tools/carla_bridge/config.example.json` | Bridge 配置样例 |
| `tools/carla_bridge/README.md` | 工具使用说明 |
| `docs/GAASD_CARLA_ACC_闭环测试方案.md` | 两阶段测试方案和本轮测试记录 |

## 3. 测试环境

| 项目 | 结果 |
|---|---|
| CARLA | `0.9.15` |
| CARLA 地址 | `127.0.0.1:2000` |
| CARLA 路径 | `/home/aiden/snap/code/app/carla-package` |
| 地图 | `Carla/Maps/Town10HD_Opt` |
| Python | `python3.8` |
| Bridge PUB | `tcp://127.0.0.1:5701` |
| Bridge SUB | `tcp://127.0.0.1:5702` |
| 协议版本 | `0.3.0` |
| Bridge tick | `fixed_delta_seconds=0.05`，约 20Hz |
| ego role | `hero` |
| lead role | `gaasd_lead` |

## 4. 本轮执行命令

```bash
tools/carla_bridge/start-carla.sh --background
tools/carla_bridge/start-bridge.sh
python3.8 tools/carla_bridge/spawn-lead-vehicle.py --replace --distance-m 25 --speed-mps 2
python3.8 tools/carla_bridge/acc-runner.py --duration-sec 20 --cruise-speed-mps 5 --min-distance-m 8 --time-gap-sec 1.5
```

测试后清理：

```bash
tools/carla_bridge/stop-bridge.sh
tools/carla_bridge/stop-carla.sh
```

## 5. 阶段一关键结果

### 5.1 Bridge 发布检查

Bridge 能持续发布：

| topic | 结果 |
|---|---|
| `gaasd.carla.ego_state.v1` | 正常 |
| `gaasd.carla.object_list.v1` | 正常 |
| `gaasd.carla.lead_vehicle.v1` | 正常 |
| `gaasd.carla.chassis_feedback.v1` | 正常 |
| `gaasd.carla.bridge_status.v1` | 正常 |

### 5.2 tick 频率检查

启用 `realtime_pacing` 后，`sim_clock` 约 2 秒收到 40 条消息：

```text
fixed_delta_seconds=0.05
2 秒约 40 tick
```

这说明 Bridge 没有再让同步模式下的 CARLA 仿真时间无限快跑。

### 5.3 前车识别检查

`spawn-lead-vehicle.py --placement ego_forward` 后，Bridge 能识别前车：

```json
{
  "valid": true,
  "object_id": 27,
  "selection_rule": "same_lane_nearest_front",
  "type": "vehicle",
  "lead_speed_mps": 2.006,
  "ego_speed_mps": 0.0,
  "longitudinal_distance_m": 35.497,
  "clearance_m": 30.705,
  "lateral_distance_m": -0.015
}
```

说明 `lead_vehicle` 选择链路有效。

需要你审阅的细节：命令中设置 `--distance-m 25`，但检测时前车距离约 35m。原因推断是前车以 2m/s 向前运动，且 Bridge/探针检查发生在生成后一段时间。请判断这是否需要在测试方案中显式要求“生成前车后立即采样”或“生成后暂停前车”。

### 5.4 ACC Runner 输出

20 秒 ACC Runner 日志摘要：

```text
[ACC] started cruise=5.00m/s min_distance=8.00m time_gap=1.50s
[ACC] cmd=1 enable=1 ego=0.00 target=5.00 lead_valid=0 distance=1000000.00
[ACC] cmd=21 enable=1 ego=3.00 target=5.00 lead_valid=1 distance=50.46 desired=12.50 rel=-1.00
[ACC] cmd=184 enable=1 ego=3.66 target=5.00 lead_valid=1 distance=37.27 desired=13.49 rel=-1.66
[ACC] cmd=328 enable=1 ego=3.66 target=5.00 lead_valid=1 distance=25.33 desired=13.49 rel=-1.66
[ACC] cmd=349 enable=1 ego=3.57 target=4.67 lead_valid=1 distance=23.70 desired=13.36 rel=-1.58
[ACC] cmd=390 enable=1 ego=3.14 target=4.21 lead_valid=1 distance=20.94 desired=12.70 rel=-1.13
[ACC] finished commands=401
```

可见：

| 现象 | 判断 |
|---|---|
| 起步时 target=5m/s | 巡航逻辑有效 |
| 跟近前车后 target 从 5 降到 4.67、4.21 | ACC 跟车调速逻辑触发 |
| `commands=401` | 20 秒、周期 0.05 秒，数量合理 |

### 5.5 Bridge 接收控制命令

测试后 `bridge_status`：

```json
{
  "state": "running",
  "connected": true,
  "ego_spawned": true,
  "last_error": "",
  "publish_counts": {
    "ego_state": 1384,
    "object_list": 1384,
    "control_cmd_received": 401
  }
}
```

这说明 ACC Runner 发送的 401 条 `control_cmd` 被 Bridge 接收并计数。

### 5.6 底盘反馈

测试中 `chassis_feedback` 示例：

```json
{
  "speed_mps": 3.657,
  "throttle": 0.201,
  "brake": 0.0,
  "mode": 1,
  "last_command_id": 241,
  "last_command_age_sec": 0.004
}
```

说明 Bridge 收到控制命令后，ego 车辆速度和 throttle 有响应。

测试结束后，超时制动验证：

```json
{
  "speed_mps": 0.0004,
  "throttle": 0.0,
  "brake": 1.0,
  "mode": 0,
  "last_command_id": 61,
  "last_command_age_sec": 7.03
}
```

说明 ACC Runner 停止后，Bridge 已不再保留最后一次 throttle，而是主动刹车。

## 6. 本轮修正的问题

### 6.1 Bridge 同步模式仿真跑太快

现象：

```text
CARLA 同步模式由 Bridge 连续 world.tick() 驱动，原先没有 sleep。
结果是仿真时间远快于真实时间，ACC Runner 真实时间 20Hz 发命令跟不上仿真推进。
```

修正：

```python
carla.realtime_pacing = True
fixed_delta_seconds = 0.05
```

Bridge 每轮 tick 后按 `fixed_delta_seconds` 补 sleep。

请审阅：

| 问题 | 请判断 |
|---|---|
| `realtime_pacing` 是否应该默认开启 | 当前默认开启 |
| pacing 是否应该基于 wall clock | 当前使用 `time.monotonic()` |
| 如果后续做加速仿真，是否应允许关闭 | 当前可以通过配置关闭 |

### 6.2 前车 waypoint 生成不稳定

现象：

```text
原先按 CARLA waypoint.next(distance) 生成前车，某次前车落到 ego 侧向约 19m。
Bridge object_list 显示 lateral_distance_m 约 19m，因此 lead_vehicle.valid=false。
```

修正：

```text
默认 --placement ego_forward，直接沿 ego 当前朝向放置前车。
保留 --placement lane_waypoint，后续需要路网一致性时再用。
```

请审阅：

| 问题 | 请判断 |
|---|---|
| 阶段一使用 `ego_forward` 是否合理 | 用于通信/控制闭环验证，可能不代表真实交通场景 |
| 阶段二是否必须切回 lane waypoint 或场景配置 | 需要你判断 |

### 6.3 控制超时后保留 throttle

现象：

```text
ACC Runner 停止后，Bridge mode=0，但 CARLA 仍保留最后一次 throttle。
```

修正：

```python
control.timeout_brake = 1.0
```

超过 `control.timeout_sec` 后，Bridge 主动下发：

```text
throttle=0.0
brake=1.0
steer=0.0
```

请审阅：

| 问题 | 请判断 |
|---|---|
| 超时后 full brake 是否过激 | 当前用于测试安全 |
| 是否应该改为 throttle=0、brake=0 的 coast | 真实车辆可能需要区分 |
| 是否应该保持最后 steering | 当前置 0 |

## 7. 已知需要你重点挑战的风险

| 风险 | 背景 |
|---|---|
| 20 秒测试时间偏短 | 只验证最小闭环，未验证长时间稳定性 |
| 前车位置不是标准场景生成 | `ego_forward` 可能不完全符合 CARLA 路网 |
| 短超时验证前未重置场景 | 后续 3 秒短测试出现过前车距离为负的日志，用途仅是验证超时刹车，不作为 ACC 跟车结论 |
| ACC Runner 是简化算法 | 只用于链路验证，不代表最终 GAASD 算法质量 |
| 只测了纵向控制 | 横向控制、路线、交通灯、复杂 object_list 未测 |
| Phase 2 仍有 Docker/ABI/ZMQ 依赖风险 | GAASD 生成工程运行在 `ubuntu:env` Docker 内 |

## 8. 希望 Claude 给出的结论格式

请按以下结构输出审阅结论：

```text
1. 是否认可“阶段一最小闭环已跑通”的结论
2. 发现的高优先级问题
3. 发现的中优先级问题
4. 发现的低优先级问题
5. 是否建议进入阶段二
6. 进入阶段二前必须补测的项目
```

如果你认为当前证据不足以进入阶段二，请明确指出缺失的最小补测项，不要泛泛而谈。
