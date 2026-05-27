# GAASD-CARLA Bridge 数据协议 v0.1

生成日期：2026-04-24

## 1. 定位

本文档定义你和 GAASD 软件开发人员之间的交接协议。

协议目标不是只跑 ACC，而是让后续规划、控制、ACC、预测、感知验证都可以复用同一个 CARLA Bridge 数据契约。

本协议分两层：

1. 标准层：JSON 消息字段契约，供 Python Bridge 和 GAASD 组件库对接。
2. 兼容层：映射到现有 GAASD/Modularization 的 ZMQ + Protobuf 字段，方便复用旧算法或旧进程。

## 2. 依据

### 2.1 GAASD 与 Modularization 依据

- GAASD 当前 CARLA 启动逻辑写死 `/home/aicc/CARLA_${version}/CarlaUE4.sh`，见 `/data/gaasd/resources/app/src/config/config.ts:43`。
- GAASD 当前场景逻辑依赖 `suc_node`、`suc_leadboard` 容器，见 `/data/gaasd/resources/app/src/config/config.ts:55`。
- GAASD 当前仿真器列表只含 `carla_0.9.12` 和 `carla_0.9.14`，见 `/data/gaasd/resources/app/src/Action/Scene.ts:5`。
- GAASD 当前启动接口 `/startSimulator` 使用本地或 SSH 命令字符串执行，见 `/data/gaasd/resources/app/src/Action/WsServer.ts:1720`。
- 当前组件映射说明 THICV 预置库来自 Modularization C 代码扫描，见 `/home/aiden/.claude/projects/-data-aiden----Modularization/memory/gaasd_component_mapping.md`。
- 现有 IMU Protobuf 字段见 `/data/aiden/文档/Modularization/proto/Imu/imu.proto:4`。
- 现有障碍物 Protobuf 字段见 `/data/aiden/文档/Modularization/proto/perception/perception.proto:4`。
- 现有控制 Protobuf 字段见 `/data/aiden/文档/Modularization/proto/controlData/control.proto:4`。
- 现有规划轨迹 Protobuf 字段见 `/data/aiden/文档/Modularization/proto/planningMsg/PlanningMsg.proto:8`。
- 现有定位发布端口 `5003`、障碍物端口 `5009`、规划输出端口 `5010`、底盘反馈端口 `3151`、控制命令端口 `3171` 见 `planningFigure`、`driver/coms`、`Thread` 源码。

### 2.2 CARLA 依据

- 本机 CARLA 包版本为 `0.9.15`，见 `/home/aiden/snap/code/app/carla-package/VERSION:1`。
- 本机 CARLA 官方包说明使用 `./CarlaUE4.sh` 启动仿真器，见 `/home/aiden/snap/code/app/carla-package/README:10`。
- 本机 PythonAPI `test_connection.py` 使用 `carla.Client(host, port)`，默认 `127.0.0.1:2000`，见 `/home/aiden/snap/code/app/carla-package/PythonAPI/util/test_connection.py:31`。
- CARLA 官方 Python API 文档：https://carla.readthedocs.io/en/0.9.15/python_api/
- CARLA 官方同步模式与固定步长文档：https://carla.readthedocs.io/en/0.9.15/adv_synchrony_timestep/
- CARLA 官方传感器文档：https://carla.readthedocs.io/en/0.9.15/ref_sensors/
- CARLA 官方快速启动文档：https://carla.readthedocs.io/en/0.9.15/start_quickstart/

## 3. 总体架构

推荐数据链路如下：

```text
CARLA Server
    ^
    | carla.Client(host, port)
    v
carla_bridge.py
    | publish JSON: clock, ego, objects, traffic lights, route, chassis feedback
    | subscribe JSON: control command, scene command
    v
GAASD CARLA 组件
    v
GAASD 算法画布和生成工程
```

兼容旧进程时，`carla_bridge.py` 可以额外发布或订阅旧 ZMQ + Protobuf 端口：

| 方向 | 端口 | 旧消息 | 用途 |
|---|---:|---|---|
| Bridge -> 算法 | `tcp://*:5003` | `IMU.Imu` | 自车定位/速度 |
| Bridge -> 算法 | `tcp://*:5009` | `perception.ObjectList` 或 `prediction.ObjectList` | 障碍物/预测 |
| 算法 -> Bridge | `tcp://127.0.0.1:3171` | `controlData.ControlCMD` | 控制命令 |
| Bridge -> 算法 | `tcp://*:3151` | `controlData.ChassisInfo` | 底盘状态反馈 |
| 规划 -> 其他模块 | `tcp://*:5010` | `Planning.TrajectoryPointVec` | 规划轨迹 |

## 4. 传输约定

### 4.1 标准传输

标准协议使用 JSON。默认推荐 ZMQ PUB/SUB，消息体为 UTF-8 JSON。

推荐端口：

| 方向 | 地址 | 说明 |
|---|---|---|
| Bridge PUB | `tcp://127.0.0.1:5701` | Bridge 发布仿真数据 |
| Bridge SUB | `tcp://127.0.0.1:5702` | Bridge 接收 GAASD 控制命令 |
| Bridge REP | `tcp://127.0.0.1:5703` | 可选，同步服务命令，如 reset/load_map |

推荐 ZMQ multipart 格式：

```text
[topic: utf8 string][payload: utf8 json]
```

topic 使用 `message_type`，例如 `gaasd.carla.ego_state.v1`。

### 4.2 频率

| 消息 | 默认频率 | 说明 |
|---|---:|---|
| `sim_clock` | 每个 CARLA tick | 主时钟 |
| `ego_state` | 每个 CARLA tick | 控制、规划、ACC 必需 |
| `object_list` | 每个 CARLA tick 或 20 Hz | ground truth 障碍物 |
| `lead_vehicle` | 每个 CARLA tick | ACC 便捷输入 |
| `traffic_light_list` | 10 Hz 或状态变化时 | 规划/决策使用 |
| `route` | 场景加载或路线变化时 | 规划使用 |
| `control_cmd` | 20 Hz 到 50 Hz | GAASD 到 CARLA |
| `chassis_feedback` | 每个 CARLA tick | 控制闭环反馈 |

第一阶段建议 CARLA 使用同步模式，`fixed_delta_seconds = 0.05`，即 20 Hz。控制需要更高频时再切到 `0.02`，即 50 Hz。

## 5. 坐标系与单位

### 5.1 标准 GAASD 坐标系

Bridge 对外发布的标准坐标必须是 GAASD 归一化坐标，不允许让算法直接依赖 CARLA 原始坐标。

| 字段 | 含义 | 单位 |
|---|---|---|
| `x_m` | 东向坐标 East，地图平面 x | m |
| `y_m` | 北向坐标 North，地图平面 y | m |
| `z_m` | 上向坐标 Up | m |
| `yaw_rad` | 以 `+x_m` 为 0，逆时针为正的数学航向角 | rad |
| `heading_deg` | 旧链路兼容航向角，北向为 0，顺时针为正 | deg |

`yaw_rad` 用于新 GAASD 组件和控制组件。

`heading_deg` 仅用于旧 Protobuf 兼容，例如旧 `IMU.Imu.yaw` 和旧 `ObjectsProto.yaw`。

### 5.2 CARLA 到 GAASD 的默认转换

默认采用 CARLA 本地地图坐标转 GAASD 本地 ENU 平面：

```text
x_m = carla_x - origin_carla_x
y_m = -(carla_y - origin_carla_y)
z_m = carla_z - origin_carla_z
yaw_rad = normalize_rad(-deg2rad(carla_yaw) + heading_offset_rad)
heading_deg = normalize_deg(90.0 - rad2deg(yaw_rad))
```

说明：

- `origin_carla_x/y/z` 由场景配置给出，默认取 ego 初始位置或地图指定原点。
- `heading_offset_rad` 默认 `0`，需要和外部地图对齐时由场景配置给出。
- CARLA 原始字段只允许出现在 `debug.carla_raw` 中，不能作为算法主输入。

### 5.3 高斯坐标兼容

现有项目约定：

```text
IMU.gaussX = Northing = y_m
IMU.gaussY = Easting  = x_m
```

因此写旧 `IMU.Imu` 时必须：

```text
gaussX = y_m
gaussY = x_m
yaw = heading_deg
velocity = speed_mps
```

### 5.4 单位

| 类型 | 单位 |
|---|---|
| 距离 | m |
| 速度 | m/s |
| 加速度 | m/s^2 |
| 角速度 | rad/s |
| 标准角度 | rad |
| 旧链路航向角 | deg |
| 时间 | s 或 ms，字段名必须带单位 |

禁止在标准 JSON 中混用角度单位。除 `heading_deg`、`carla_yaw_deg` 外，所有角度字段均为 rad。

## 6. 通用消息头

所有标准 JSON 消息必须包含 `header`。

```json
{
  "header": {
    "protocol": "gaasd_carla_bridge",
    "protocol_version": "0.1.0",
    "message_type": "gaasd.carla.ego_state.v1",
    "frame_id": 12345,
    "sequence": 12345,
    "sim_time_sec": 12.35,
    "delta_time_sec": 0.05,
    "timestamp_unix_ms": 1777020000000,
    "source": "carla_bridge",
    "map_name": "Town04",
    "ego_role_name": "hero",
    "coordinate_frame": "gaasd_map"
  },
  "payload": {}
}
```

字段要求：

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `protocol` | string | 是 | 固定 `gaasd_carla_bridge` |
| `protocol_version` | string | 是 | 当前 `0.1.0` |
| `message_type` | string | 是 | 消息类型 |
| `frame_id` | uint64 | 是 | CARLA frame |
| `sequence` | uint64 | 是 | Bridge 发布序号 |
| `sim_time_sec` | double | 是 | CARLA 仿真 elapsed time |
| `delta_time_sec` | double | 是 | 当前 tick 步长 |
| `timestamp_unix_ms` | int64 | 是 | Bridge 主机墙钟 |
| `source` | string | 是 | `carla_bridge` 或算法名 |
| `map_name` | string | 是 | CARLA 地图名 |
| `ego_role_name` | string | 是 | ego 车辆 role name |
| `coordinate_frame` | string | 是 | 固定 `gaasd_map` |

## 7. SimClock

### 7.1 topic

```text
gaasd.carla.sim_clock.v1
```

### 7.2 payload

```json
{
  "synchronous_mode": true,
  "fixed_delta_seconds": 0.05,
  "paused": false,
  "real_time_factor": 0.98,
  "carla_server_version": "0.9.15"
}
```

字段：

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `synchronous_mode` | bool | 是 | 是否由 Bridge 驱动 tick |
| `fixed_delta_seconds` | double | 是 | 固定步长 |
| `paused` | bool | 是 | 是否暂停 |
| `real_time_factor` | double | 否 | 仿真时间/真实时间 |
| `carla_server_version` | string | 是 | CARLA 版本 |

## 8. EgoState

### 8.1 topic

```text
gaasd.carla.ego_state.v1
```

### 8.2 payload

```json
{
  "ego_id": 101,
  "role_name": "hero",
  "pose": {
    "x_m": 12.34,
    "y_m": 56.78,
    "z_m": 0.30,
    "yaw_rad": 1.5708,
    "pitch_rad": 0.0,
    "roll_rad": 0.0,
    "heading_deg": 0.0
  },
  "velocity": {
    "vx_mps": 0.0,
    "vy_mps": 8.3,
    "vz_mps": 0.0,
    "speed_mps": 8.3,
    "longitudinal_mps": 8.3,
    "lateral_mps": 0.0
  },
  "acceleration": {
    "ax_mps2": 0.0,
    "ay_mps2": 0.1,
    "az_mps2": 0.0,
    "longitudinal_mps2": 0.1,
    "lateral_mps2": 0.0
  },
  "angular_velocity": {
    "roll_rate_rps": 0.0,
    "pitch_rate_rps": 0.0,
    "yaw_rate_rps": 0.0
  },
  "geo": {
    "valid": true,
    "longitude_deg": 116.335587,
    "latitude_deg": 40.001787,
    "altitude_m": 0.0,
    "gauss_north_m": 4429000.0,
    "gauss_east_m": 445000.0
  },
  "vehicle": {
    "length_m": 4.6,
    "width_m": 1.9,
    "height_m": 1.6,
    "wheel_base_m": 2.8
  },
  "status": {
    "gps_valid": 1,
    "rtk_mode": 0,
    "autopilot_enabled": false,
    "collision": false
  },
  "debug": {
    "carla_raw": {
      "x": 12.34,
      "y": -56.78,
      "z": 0.30,
      "yaw_deg": -90.0
    }
  }
}
```

### 8.3 GAASD 映射

| 标准字段 | 新组件建议端口 | 旧 Protobuf 映射 |
|---|---|---|
| `pose.x_m` | `egoX` | `IMU.Imu.gaussY` |
| `pose.y_m` | `egoY` | `IMU.Imu.gaussX` |
| `pose.yaw_rad` | `egoYawRad` | 不直接写旧 `yaw` |
| `pose.heading_deg` | `egoHeadingDeg` | `IMU.Imu.yaw` |
| `velocity.speed_mps` | `egoV` | `IMU.Imu.velocity` |
| `acceleration.longitudinal_mps2` | `egoAcc` | `IMU.Imu.accX` |
| `geo.longitude_deg` | `longitude` | `IMU.Imu.longitude` |
| `geo.latitude_deg` | `latitude` | `IMU.Imu.latitude` |

## 9. ObjectList

### 9.1 topic

```text
gaasd.carla.object_list.v1
```

### 9.2 payload

```json
{
  "source_type": "ground_truth",
  "sensor_id": 0,
  "object_count": 2,
  "objects": [
    {
      "object_id": 201,
      "type": "vehicle",
      "type_code": 1,
      "subtype": "vehicle.tesla.model3",
      "dynamic": true,
      "valid": true,
      "pose": {
        "x_m": 20.0,
        "y_m": 58.0,
        "z_m": 0.3,
        "yaw_rad": 1.5708,
        "pitch_rad": 0.0,
        "roll_rad": 0.0,
        "heading_deg": 0.0
      },
      "velocity": {
        "vx_mps": 0.0,
        "vy_mps": 6.0,
        "vz_mps": 0.0,
        "speed_mps": 6.0
      },
      "acceleration": {
        "ax_mps2": 0.0,
        "ay_mps2": 0.0,
        "az_mps2": 0.0
      },
      "dimension": {
        "length_m": 4.6,
        "width_m": 1.9,
        "height_m": 1.6
      },
      "relative_to_ego": {
        "center_dx_m": 0.0,
        "center_dy_m": 15.0,
        "longitudinal_center_distance_m": 15.0,
        "lateral_distance_m": 0.0,
        "clearance_m": 10.4,
        "relative_speed_mps": -2.3,
        "ttc_sec": 4.52
      },
      "debug": {
        "carla_actor_id": 201
      }
    }
  ]
}
```

### 9.3 类型枚举

| `type_code` | `type` | CARLA 来源 |
|---:|---|---|
| 0 | `unknown` | 未分类 |
| 1 | `vehicle` | `vehicle.*` |
| 2 | `pedestrian` | `walker.pedestrian.*` |
| 3 | `cyclist` | 自行车/摩托车按场景配置映射 |
| 4 | `static` | 静态障碍物 |
| 5 | `traffic_light` | 交通灯 actor |

### 9.4 尺寸映射

CARLA actor bounding box 使用半长宽高。Bridge 必须输出完整尺寸：

```text
length_m = 2.0 * bounding_box.extent.x
width_m  = 2.0 * bounding_box.extent.y
height_m = 2.0 * bounding_box.extent.z
```

### 9.5 GAASD 映射

| 标准字段 | 新组件建议端口 | 旧 Protobuf 映射 |
|---|---|---|
| `pose.x_m` | `objectX` | `perception.Object.x` |
| `pose.y_m` | `objectY` | `perception.Object.y` |
| `pose.z_m` | `objectZ` | `perception.Object.z` |
| `dimension.width_m` | `objectWidth` | `perception.Object.w` |
| `dimension.height_m` | `objectHeight` | `perception.Object.h` |
| `dimension.length_m` | `objectLength` | `perception.Object.l` |
| `type_code` | `objectType` | `perception.Object.type` |
| `object_id` | `trackID` | `perception.Object.trackID` |
| `velocity.vx_mps` | `objectVx` | `perception.Object.vx` |
| `velocity.vy_mps` | `objectVy` | `perception.Object.vy` |

旧 `infopack.ObjectsProto` 兼容时：

```text
ObjectsProto.x = pose.x_m
ObjectsProto.y = pose.y_m
ObjectsProto.yaw = pose.heading_deg
ObjectsProto.velocity = velocity.speed_mps
ObjectsProto.len = round(dimension.length_m)
ObjectsProto.width = round(dimension.width_m)
ObjectsProto.height = round(dimension.height_m)
ObjectsProto.timestamp = timestamp_unix_ms
```

## 10. LeadVehicle

### 10.1 topic

```text
gaasd.carla.lead_vehicle.v1
```

### 10.2 payload

```json
{
  "valid": true,
  "object_id": 201,
  "selection_rule": "same_lane_nearest_front",
  "type": "vehicle",
  "lead_speed_mps": 6.0,
  "ego_speed_mps": 8.3,
  "relative_speed_mps": -2.3,
  "longitudinal_center_distance_m": 15.0,
  "clearance_m": 10.4,
  "lateral_distance_m": 0.0,
  "time_gap_sec": 1.25,
  "ttc_sec": 4.52
}
```

字段说明：

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `valid` | bool | 是 | 是否存在前车 |
| `object_id` | int64 | 否 | 前车 actor id |
| `selection_rule` | string | 是 | 默认同车道最近前车 |
| `lead_speed_mps` | double | 是 | 前车速度 |
| `ego_speed_mps` | double | 是 | 自车速度 |
| `relative_speed_mps` | double | 是 | 前车速度减自车速度 |
| `longitudinal_center_distance_m` | double | 是 | 中心点纵向距离 |
| `clearance_m` | double | 是 | 车头到前车车尾距离 |
| `lateral_distance_m` | double | 是 | 横向距离 |
| `time_gap_sec` | double | 否 | `clearance_m / max(ego_speed_mps, epsilon)` |
| `ttc_sec` | double | 否 | 自车接近前车时的 TTC |

ACC 推荐使用：

```text
egoV = ego_speed_mps
leadV = lead_speed_mps
distance = clearance_m
```

如果复用旧 `computeDistance1D` 示例，可用 `longitudinal_center_distance_m`，但安全控制优先使用 `clearance_m`。

## 11. TrafficLightList

### 11.1 topic

```text
gaasd.carla.traffic_light_list.v1
```

### 11.2 payload

```json
{
  "lights": [
    {
      "light_id": 301,
      "active": true,
      "state": "red",
      "state_code": 0,
      "remaining_time_sec": -1.0,
      "direction": "straight",
      "direction_code": 0,
      "affected_ego": true,
      "distance_to_stop_line_m": 35.2,
      "stop_line": {
        "x_m": 12.0,
        "y_m": 100.0,
        "z_m": 0.0,
        "yaw_rad": 1.5708
      }
    }
  ]
}
```

### 11.3 信号灯枚举

| `state_code` | `state` | 说明 |
|---:|---|---|
| 0 | `red` | 红灯 |
| 1 | `yellow` | 黄灯 |
| 2 | `green` | 绿灯 |
| 3 | `off` | 关闭 |
| 4 | `unknown` | 未知 |

旧 `TrafficLightForUnity` 只支持 0/1/2。映射旧消息时：

```text
red -> 0
yellow -> 1
green -> 2
off/unknown -> active=false 或按 unknown 单独处理
```

### 11.4 方向枚举

| `direction_code` | `direction` | 说明 |
|---:|---|---|
| 0 | `straight` | 直行 |
| 1 | `right` | 右转 |
| 2 | `left` | 左转 |
| 3 | `u_turn` | 掉头 |
| 4 | `unknown` | 未知 |

## 12. Route

### 12.1 topic

```text
gaasd.carla.route.v1
```

### 12.2 payload

```json
{
  "route_id": "town04_acc_demo_001",
  "route_source": "carla_waypoints",
  "start": {
    "x_m": 0.0,
    "y_m": 0.0,
    "z_m": 0.0,
    "yaw_rad": 0.0
  },
  "goal": {
    "x_m": 200.0,
    "y_m": 0.0,
    "z_m": 0.0,
    "yaw_rad": 0.0
  },
  "waypoints": [
    {
      "index": 0,
      "x_m": 0.0,
      "y_m": 0.0,
      "z_m": 0.0,
      "yaw_rad": 0.0,
      "speed_limit_mps": 13.89,
      "s_m": 0.0,
      "road_id": 1,
      "lane_id": -1,
      "junction_id": -1
    }
  ]
}
```

### 12.3 GAASD 映射

| 标准字段 | 新组件建议端口 | 旧 Protobuf 映射 |
|---|---|---|
| `waypoints[].x_m` | `routeX` | `Planning.TrajectoryPoint.X` |
| `waypoints[].y_m` | `routeY` | `Planning.TrajectoryPoint.Y` |
| `waypoints[].yaw_rad` | `routeYawRad` | `Planning.TrajectoryPoint.Theta` |
| `waypoints[].speed_limit_mps` | `routeSpeed` | `Planning.TrajectoryPoint.Speed` |
| `waypoints[].s_m` | `routeS` | `Planning.TrajectoryPoint.S` |

## 13. ControlCmd

### 13.1 topic

```text
gaasd.carla.control_cmd.v1
```

### 13.2 payload

```json
{
  "command_id": 9001,
  "source_algorithm": "gaasd_controller",
  "command_mode": "ackermann",
  "enable": true,
  "target": {
    "steer_rad": 0.05,
    "target_speed_mps": 8.0,
    "target_accel_mps2": 0.2,
    "target_jerk_mps3": 0.0
  },
  "raw_vehicle_control": {
    "valid": false,
    "throttle": 0.0,
    "brake": 0.0,
    "steer_norm": 0.0,
    "hand_brake": false,
    "reverse": false,
    "gear": 0
  },
  "safety": {
    "max_speed_mps": 20.0,
    "max_accel_mps2": 4.0,
    "max_decel_mps2": -6.0,
    "max_abs_steer_rad": 0.6,
    "timeout_sec": 0.2
  }
}
```

### 13.3 控制语义

Bridge 优先使用 CARLA Ackermann 控制：

```text
steer = target.steer_rad
speed = target.target_speed_mps
acceleration = target.target_accel_mps2
```

如果 CARLA 车辆或版本不支持 Ackermann 控制，则 Bridge 内部负责把 `target_speed_mps`、`target_accel_mps2`、`steer_rad` 转为 CARLA 低层 `VehicleControl` 的 `throttle`、`brake`、`steer`。

低层 `raw_vehicle_control` 只用于调试或特殊算法。普通 GAASD 控制组件不要直接输出 `throttle/brake/steer_norm`。

### 13.4 GAASD 映射

| 标准字段 | 新组件建议端口 | 旧 Protobuf 映射 |
|---|---|---|
| `target.steer_rad` | `steer` | `controlData.ControlCMD.targetSteeringAngle` |
| `target.target_speed_mps` | `speed` | `controlData.ControlCMD.targetSpeed` |
| `target.target_accel_mps2` | `acceleration` | 新组件字段，旧 proto 无 |
| `safety.timeout_sec` | `cmdTimeout` | 新组件字段 |
| `lead_vehicle.clearance_m` | `obstacleDistance` | `controlData.ControlCMD.obstacleDIS` |

旧 `controlData.ControlCMD` 字段来自 `/data/aiden/文档/Modularization/proto/controlData/control.proto:4`。

## 14. ChassisFeedback

### 14.1 topic

```text
gaasd.carla.chassis_feedback.v1
```

### 14.2 payload

```json
{
  "speed_mps": 8.2,
  "steering_angle_rad": 0.04,
  "steer_norm": 0.10,
  "throttle": 0.25,
  "brake": 0.0,
  "gear": 1,
  "mode": 0,
  "last_command_id": 9001,
  "last_command_age_sec": 0.03
}
```

旧 Protobuf 映射：

```text
ChassisInfo.speed = speed_mps
ChassisInfo.steeringAngle = steering_angle_rad
ChassisInfo.mode = mode
```

## 15. SceneCommand

### 15.1 topic

```text
gaasd.carla.scene_cmd.v1
```

### 15.2 payload

```json
{
  "command": "load_world",
  "map_name": "Town04",
  "ego": {
    "role_name": "hero",
    "blueprint_filter": "vehicle.*model3*",
    "spawn_point_index": 0
  },
  "traffic": {
    "vehicles": 20,
    "walkers": 0,
    "traffic_manager_port": 8000
  },
  "weather": {
    "preset": "ClearNoon"
  },
  "sync": {
    "synchronous_mode": true,
    "fixed_delta_seconds": 0.05
  }
}
```

可选 `command`：

| command | 说明 |
|---|---|
| `load_world` | 加载地图 |
| `reset` | 重置场景 |
| `spawn_ego` | 生成自车 |
| `spawn_traffic` | 生成 NPC |
| `destroy_traffic` | 清除 NPC |
| `pause` | 暂停 tick |
| `resume` | 恢复 tick |

## 16. 组件库建议

GAASD 开发人员应新增以下标准 CARLA 组件。

### 16.1 输入组件

| 组件名 | 输入 | 输出 | 说明 |
|---|---|---|---|
| `CARLASimClock` | `sim_clock` | `dt`, `frameId`, `simTime` | 全局时钟 |
| `CARLAEgoState` | `ego_state` | `egoX`, `egoY`, `egoYawRad`, `egoV`, `egoAcc` | 自车状态 |
| `CARLAObjectList` | `object_list` | object array 或拆分端口 | 障碍物列表 |
| `CARLALeadVehicle` | `lead_vehicle` | `leadV`, `distance`, `relativeSpeed`, `ttc` | ACC/跟车 |
| `CARLATrafficLightList` | `traffic_light_list` | light array 或当前灯 | 决策 |
| `CARLARoute` | `route` | waypoint array 或 `Traj` | 规划路线 |
| `CARLAChassisFeedback` | `chassis_feedback` | `speed`, `steer`, `mode` | 控制反馈 |

### 16.2 输出组件

| 组件名 | 输入 | 输出 | 说明 |
|---|---|---|---|
| `CARLAControlCmd` | `steer`, `speed`, `acceleration` | `control_cmd` JSON | 通用控制 |
| `CARLALongitudinalCmd` | `speed`, `acceleration` | `control_cmd.target` | 纵向控制 |
| `CARLALateralCmd` | `steer` | `control_cmd.target.steer_rad` | 横向控制 |
| `CARLASceneCmd` | 场景配置 | `scene_cmd` JSON | 场景控制 |

### 16.3 ACC 最小闭环端口

ACC 示例只需要以下字段：

| 来源 | 字段 | 接到现有组件 |
|---|---|---|
| `CARLAEgoState` | `egoV` | `accComputeTargetSpeed.egoV` |
| `CARLALeadVehicle` | `leadV` | `accComputeTargetSpeed.leadV` |
| `CARLALeadVehicle` | `distance` | `accComputeTargetSpeed.distance` |
| `accComputeTargetSpeed` | `targetSpeed` | 纵向控制或 `CARLAControlCmd.speed` |
| 横向控制 | `steer` | `CARLAControlCmd.steer` |

## 17. 旧 Protobuf 兼容表

### 17.1 IMU.Imu

| 旧字段 | 标准字段 | 说明 |
|---|---|---|
| `longitude` | `ego_state.geo.longitude_deg` | 经度 |
| `latitude` | `ego_state.geo.latitude_deg` | 纬度 |
| `gaussX` | `ego_state.pose.y_m` | Northing |
| `gaussY` | `ego_state.pose.x_m` | Easting |
| `gpsValid` | `ego_state.status.gps_valid` | 定位有效 |
| `time` | `header.sim_time_sec` | 仿真时间 |
| `velocity` | `ego_state.velocity.speed_mps` | 速度 |
| `yaw` | `ego_state.pose.heading_deg` | 旧链路航向角，单位 deg |
| `ltime` | `header.timestamp_unix_ms` | Unix ms |
| `velocityEast` | `ego_state.velocity.vx_mps` | 东向速度 |
| `velocityNorth` | `ego_state.velocity.vy_mps` | 北向速度 |
| `velocityUp` | `ego_state.velocity.vz_mps` | 上向速度 |
| `gyroZ` | `ego_state.angular_velocity.yaw_rate_rps` | yaw rate |
| `accX` | `ego_state.acceleration.longitudinal_mps2` | 纵向加速度 |

### 17.2 perception.ObjectList

| 旧字段 | 标准字段 |
|---|---|
| `Object.x` | `objects[].pose.x_m` |
| `Object.y` | `objects[].pose.y_m` |
| `Object.z` | `objects[].pose.z_m` |
| `Object.w` | `objects[].dimension.width_m` |
| `Object.h` | `objects[].dimension.height_m` |
| `Object.l` | `objects[].dimension.length_m` |
| `Object.type` | `objects[].type_code` |
| `Object.trackID` | `objects[].object_id` |
| `Object.vx` | `objects[].velocity.vx_mps` |
| `Object.vy` | `objects[].velocity.vy_mps` |

### 17.3 controlData.ControlCMD

| 旧字段 | 标准字段 |
|---|---|
| `targetSteeringAngle` | `control_cmd.target.steer_rad` |
| `targetSpeed` | `control_cmd.target.target_speed_mps` |
| `obstacleDIS` | `lead_vehicle.clearance_m` |

### 17.4 controlData.ChassisInfo

| 旧字段 | 标准字段 |
|---|---|
| `speed` | `chassis_feedback.speed_mps` |
| `steeringAngle` | `chassis_feedback.steering_angle_rad` |
| `mode` | `chassis_feedback.mode` |

## 18. 安全与容错

Bridge 必须实现以下容错：

| 条件 | 行为 |
|---|---|
| `control_cmd.enable=false` | 不下发控制，保持或刹停 |
| 超过 `timeout_sec` 未收到控制 | 进入 fail-safe，速度目标 0，制动 |
| `target_speed_mps < 0` | 钳位为 0，除非显式启用倒车 |
| `abs(steer_rad) > max_abs_steer_rad` | 钳位 |
| `target_accel_mps2` 超限 | 钳位 |
| JSON 字段缺失 | 拒绝该消息并记录错误 |
| 出现 NaN/Inf | 拒绝该消息并记录错误 |
| CARLA 连接断开 | 发布 bridge status error，停止下发旧数据 |

标准 JSON 中不使用 NaN/Inf。未知数值用 `valid=false` 加默认数值表示。

## 19. Bridge 状态消息

### 19.1 topic

```text
gaasd.carla.bridge_status.v1
```

### 19.2 payload

```json
{
  "state": "running",
  "connected": true,
  "carla_host": "127.0.0.1",
  "carla_port": 2000,
  "map_name": "Town04",
  "ego_spawned": true,
  "last_error": "",
  "publish_counts": {
    "ego_state": 1000,
    "object_list": 1000,
    "control_cmd_received": 998
  }
}
```

状态枚举：

| state | 说明 |
|---|---|
| `starting` | Bridge 启动中 |
| `connecting` | 连接 CARLA 中 |
| `running` | 正常运行 |
| `paused` | 暂停 |
| `error` | 错误 |
| `stopping` | 停止中 |

## 20. 最小验收用例

第一阶段验收只需要跑通以下闭环：

1. 启动 CARLA 0.9.15。
2. 启动 `carla_bridge.py`，连接 `127.0.0.1:2000`。
3. Bridge 发布 `sim_clock`、`ego_state`、`object_list`、`lead_vehicle`。
4. GAASD 画布读取 `egoV`、`leadV`、`distance`，计算 ACC 目标速度。
5. GAASD 通过 `control_cmd` 输出 `target_speed_mps` 和 `steer_rad`。
6. Bridge 将控制应用到 CARLA ego 车。
7. `chassis_feedback` 返回当前速度和转角。

ACC 最小闭环不要求相机、激光雷达、完整感知链路，也不要求 Pangu/leaderboard 容器。

## 21. 责任边界

你的交付：

- `start-carla.sh`、`stop-carla.sh`、`health-carla.sh`。
- 可选 `carla_bridge.py`。
- 本文档定义的数据协议。
- ACC 最小闭环 Bridge 示例。

GAASD 开发人员交付：

- 仿真管理界面支持本地 CARLA 0.9.15 和脚本路径配置。
- 组件库增加 `CARLAEgoState`、`CARLAObjectList`、`CARLALeadVehicle`、`CARLAControlCmd` 等组件。
- 组件端口、代码生成模板和工程落盘格式按本文档实现。
- 旧 Pangu/容器场景逻辑和官方 CARLA 本地模式拆分。
- 必要时把 JSON 契约映射成内部 Protobuf/ZMQ。

## 22. 待 GAASD 团队确认项

以下事项需要 GAASD 开发人员最终确认后冻结为 v1.0：

| 项 | 当前建议 | 原因 |
|---|---|---|
| 标准传输 | JSON over ZMQ | 与组件交接最简单 |
| 内部落盘 | 由 GAASD 开发人员决定 | 属于 GAASD 软件实现 |
| 新组件字段名 | 使用本文端口名 | 便于生成模板 |
| CARLA yaw 映射 | `yaw_rad = -carla_yaw_rad` | 消除左手/右手差异 |
| 旧 IMU yaw | `heading_deg` | 旧规划代码按度使用 |
| ACC 距离 | `clearance_m` | 安全语义比中心距更明确 |
| 旧 ACC 示例距离 | 可用 `longitudinal_center_distance_m` | 兼容旧 `computeDistance1D` |
| Ackermann 控制 | 优先 | 与现有 `steer/speed/acceleration` 组件一致 |

