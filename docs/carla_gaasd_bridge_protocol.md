# CARLA ↔ GAASD Bridge 数据协议

版本：v0.3  
日期：2026-04-24  
适用：CARLA 0.9.15 × GAASD accpro1 及后续所有算法模块；CARLA 可运行在本机或远程服务器

---

## 1. 定位

本文档定义 Python Bridge 与 GAASD 组件库之间的数据契约，目标是让 ACC、规划、控制、预测、感知验证等后续算法模块复用同一个 Bridge 接口。

协议分两层：

- **标准层**：JSON 消息字段契约，供 Python Bridge 和 GAASD 新组件对接。
- **兼容层**：映射到现有 Modularization 的 ZMQ + Protobuf 字段，便于复用旧算法进程。

---

## 2. 依据

### 2.1 GAASD 与 Modularization

- GAASD 已预留 SSH 连接仿真服务器的入口，但当前启动逻辑仍写死 `/home/aicc/CARLA_${version}/CarlaUE4.sh`，见 `/opt/gaasd/resources/app/dist/config/config.js`（编译产物）及源码 `/data/gaasd/resources/app/src/config/config.ts`。该 SSH 入口应保留，并补齐远程路径、工作目录、启动脚本、环境变量和端口配置。
- GAASD 场景依赖 `suc_node`、`suc_leadboard` 容器，见同文件。该逻辑可作为服务器/旧场景模式保留，但本地 CARLA 官方包模式不能强依赖这些容器。
- GAASD 仿真器列表只含 `carla_0.9.12` 和 `carla_0.9.14`，需新增 `0.9.15`。
- 现有 Protobuf：`proto/Imu/imu.proto`、`proto/perception/perception.proto`、`proto/controlData/control.proto`、`proto/planningMsg/PlanningMsg.proto`。
- 现有端口：定位发布 `5003`、障碍物 `5009`、规划输出 `5010`、底盘反馈 `3151`、控制命令 `3171`。
- GAASD 结构体定义见 `project/accpro1` 扫描包中的 `struct.json`（`VehicleState`、`GaussRoadPoint`、`TrajPoint`、`Prediction__Object`、`Infopack__TrafficLight`）。

### 2.2 CARLA

- 本机 CARLA 版本：`0.9.15`，见 `~/snap/code/app/carla-package/VERSION`。
- Python Client 默认连接：`carla.Client(carla_host, carla_port)`；本机模式默认 `127.0.0.1:2000`，服务器模式使用远程服务器 IP/域名和端口，必要时通过 SSH 隧道转发到本地端口。
- 官方文档：https://carla.readthedocs.io/en/0.9.15/python_api/

---

## 3. 总体架构

```
GAASD 仿真管理
    ├── 本地脚本模式：cd <carla_root> && ./start-carla.sh
    └── 服务器 SSH 模式：ssh <user>@<host> "cd <remote_carla_root> && <remote_start_script>"

CARLA Server (0.9.15，本机或服务器)
    ↑↓  carla.Client(carla_host, carla_port)
carla_bridge.py
    ├── PUB tcp://127.0.0.1:5701  →  GAASD 输入组件（sim_clock / ego_state / object_list / lead_vehicle / traffic_light_list / route）
    ├── SUB tcp://127.0.0.1:5702  ←  GAASD 输出组件（control_cmd / trajectory_cmd）
    └── REP tcp://127.0.0.1:5703  ←→  GAASD（可选，scene_cmd 同步服务）

兼容旧进程时额外端口：
    PUB tcp://*:5003   →  旧算法（IMU.Imu Protobuf）
    PUB tcp://*:5009   →  旧算法（perception.ObjectList Protobuf）
    SUB tcp://127.0.0.1:3171  ←  旧控制（controlData.ControlCMD Protobuf）
    PUB tcp://*:3151   →  旧控制（controlData.ChassisInfo Protobuf）
```

---

## 4. 传输约定

### 4.1 标准传输

标准协议使用 **JSON over ZMQ PUB/SUB**，消息体为 UTF-8 JSON。

ZMQ multipart 格式：

```
[topic: utf8 string][payload: utf8 json]
```

topic 等于 `message_type`，例如 `gaasd.carla.ego_state.v1`。

| 方向 | 地址 | 说明 |
|---|---|---|
| Bridge PUB | `tcp://127.0.0.1:5701` | Bridge 发布仿真数据，默认本机；跨机器部署时需配置 bind/connect 地址 |
| Bridge SUB | `tcp://127.0.0.1:5702` | Bridge 接收 GAASD 控制/轨迹输出，默认本机；跨机器部署时需配置 bind/connect 地址 |
| Bridge REP | `tcp://127.0.0.1:5703` | 可选，同步场景命令，默认本机；跨机器部署时需配置 bind/connect 地址 |

### 4.2 消息频率

| 消息 | 默认频率 | 说明 |
|---|---|---|
| `sim_clock` | 每 tick | 主时钟 |
| `ego_state` | 每 tick | 控制/规划/ACC 必需 |
| `object_list` | 每 tick | ground truth 障碍物 |
| `lead_vehicle` | 每 tick | ACC 便捷输入 |
| `traffic_light_list` | 10 Hz 或状态变化 | 决策使用 |
| `route` | 场景加载或路线变化 | 规划使用 |
| `control_cmd` | 20–50 Hz | GAASD → Bridge |
| `trajectory_cmd` | 10–20 Hz | GAASD 规划 → Bridge |
| `chassis_feedback` | 每 tick | 控制闭环反馈 |

第一阶段建议 CARLA 同步模式，`fixed_delta_seconds = 0.05`（20 Hz）。

### 4.3 CARLA 启动配置

GAASD 仿真管理应将 CARLA 启动方式作为工程配置保存，而不是写死为单一路径。推荐配置：

```json
{
  "carla": {
    "launch_mode": "local_script",
    "version": "0.9.15",
    "host": "127.0.0.1",
    "port": 2000,
    "local": {
      "carla_root": "/home/aiden/snap/code/app/carla-package",
      "start_script": "/home/aiden/snap/code/app/carla-package/start-carla.sh",
      "working_directory": "/home/aiden/snap/code/app/carla-package"
    },
    "remote_ssh": {
      "ssh_host": "",
      "ssh_port": 22,
      "ssh_user": "",
      "auth_type": "key_or_password",
      "remote_carla_root": "/home/aicc/CARLA_0.9.15",
      "remote_start_script": "./CarlaUE4.sh",
      "remote_working_directory": "/home/aicc/CARLA_0.9.15",
      "carla_host_for_bridge": "",
      "carla_port_for_bridge": 2000,
      "use_ssh_tunnel": false,
      "tunnel_local_port": 2000
    }
  }
}
```

`launch_mode` 取值：

| launch_mode | 说明 |
|---|---|
| `local_script` | 在 GAASD 所在机器通过本地脚本启动 CARLA |
| `remote_ssh` | 通过 SSH 在服务器启动 CARLA，保留现有远程仿真入口 |
| `external` | CARLA 已由外部系统启动，GAASD 只保存连接地址，Bridge 只负责连接 |

---

## 5. 坐标系与单位

### 5.1 坐标系约定

| | CARLA（UE4 左手系） | GAASD（ENU 右手系） |
|---|---|---|
| X 轴 | 向东（East），正 | 向东（East），正 |
| Y 轴 | 向南（South），正 | 向北（North），正 |
| Z 轴 | 向上 | 向上 |
| Yaw | 顺时针为正，单位 ° | 右手系，逆时针为正，单位 rad |

Bridge 对外发布的坐标**必须**是 GAASD ENU 坐标，算法不能直接依赖 CARLA 原始坐标。

### 5.2 CARLA → GAASD 转换

```python
# CARLA Python API 来源
transform    = vehicle.get_transform()     # location.x/y/z, rotation.yaw (deg)
velocity     = vehicle.get_velocity()      # Vector3D x/y/z (m/s)
acceleration = vehicle.get_acceleration()  # Vector3D x/y/z (m/s²)

# 坐标转换
x_m   = transform.location.x - origin_x   # East，默认 origin_x=0
y_m   = -(transform.location.y - origin_y) # South→North，取反
z_m   = transform.location.z - origin_z

# 航向角：CARLA 顺时针° → GAASD 右手系 rad（逆时针为正）
yaw_rad     = math.radians(-transform.rotation.yaw)
heading_deg = (90.0 - math.degrees(yaw_rad)) % 360.0  # 旧链路兼容，北向为0

# 速度
vx_mps   = velocity.x                     # East 分量
vy_mps   = -velocity.y                    # North 分量（取反）
speed_mps = math.sqrt(velocity.x**2 + velocity.y**2)  # 2D 模长

# 加速度（投影到 ego 航向，保留正负号：正=加速，负=制动）
ax_enu = acceleration.x           # East 分量（CARLA x 无需转换）
ay_enu = -acceleration.y          # North 分量（取反）
longitudinal_mps2 = ax_enu * math.cos(yaw_rad) + ay_enu * math.sin(yaw_rad)
```

`origin_x/y/z` 默认取 0（地图绝对坐标），需要与外部地图对齐时由场景配置给出。

### 5.3 高斯坐标兼容（旧链路）

现有项目约定 `gaussX = Northing = y_m`，`gaussY = Easting = x_m`，写旧 `IMU.Imu` 时必须互换：

```python
imu.gaussX = y_m   # Northing
imu.gaussY = x_m   # Easting
imu.yaw    = heading_deg
imu.velocity = speed_mps
```

### 5.4 单位规范

| 类型 | 单位 | 字段后缀 |
|---|---|---|
| 距离 | m | `_m` |
| 速度 | m/s | `_mps` |
| 加速度 | m/s² | `_mps2` |
| 角速度 | rad/s | `_rps` |
| 角度（标准） | rad | `_rad` |
| 角度（旧链路） | deg | `_deg` |
| 时间 | s | `_sec` |

所有角度字段均为 rad，除 `heading_deg` 和 `debug.carla_raw` 中的原始值外。

---

## 6. 通用消息头

所有 JSON 消息必须包含 `header`：

```json
{
  "header": {
    "protocol": "gaasd_carla_bridge",
    "protocol_version": "0.3.0",
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

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `protocol_version` | string | 是 | 当前 `0.3.0` |
| `message_type` | string | 是 | 消息类型 topic |
| `frame_id` | uint64 | 是 | CARLA world frame 编号 |
| `sequence` | uint64 | 是 | Bridge 本消息类型发布序号 |
| `sim_time_sec` | double | 是 | CARLA 仿真经过时间 |
| `delta_time_sec` | double | 是 | 当前 tick 步长 |
| `timestamp_unix_ms` | int64 | 是 | Bridge 主机墙钟 |
| `coordinate_frame` | string | 是 | 固定 `gaasd_map` |

---

## 7. SimClock

**topic**：`gaasd.carla.sim_clock.v1`

```json
{
  "synchronous_mode": true,
  "fixed_delta_seconds": 0.05,
  "paused": false,
  "real_time_factor": 0.98,
  "carla_server_version": "0.9.15"
}
```

**CARLA API 来源**：

```python
settings = world.get_settings()
# settings.synchronous_mode, settings.fixed_delta_seconds
snapshot = world.get_snapshot()
# snapshot.elapsed_seconds, snapshot.frame, snapshot.delta_seconds
```

---

## 8. EgoState

**topic**：`gaasd.carla.ego_state.v1`  
**对应 GAASD 结构体**：`VehicleState`（扩展版）

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
    "yaw_rate_rps": 0.0,
    "pitch_rate_rps": 0.0,
    "roll_rate_rps": 0.0
  },
  "road": {
    "lane_id": -1,
    "road_id": 1,
    "junction_id": -1,
    "s_m": 45.2
  },
  "vehicle": {
    "length_m": 4.6,
    "width_m": 1.9,
    "wheel_base_m": 2.8
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

**CARLA API 来源**：

```python
transform    = vehicle.get_transform()
velocity     = vehicle.get_velocity()
acceleration = vehicle.get_acceleration()
ang_vel      = vehicle.get_angular_velocity()   # Vector3D (deg/s)，注意是度不是弧度
# yaw_rate_rps = -math.radians(ang_vel.z)        # 取反：CARLA 左手系→右手系
waypoint     = map.get_waypoint(transform.location)
# waypoint.lane_id, waypoint.road_id, waypoint.junction_id, waypoint.s
```

**GAASD 结构体映射**：

| 标准字段 | GAASD `VehicleState` 字段 | 新组件端口 | 旧 Protobuf |
|---|---|---|---|
| `pose.x_m` | `x` | `egoX` | `IMU.Imu.gaussY` |
| `pose.y_m` | - | `egoY` | `IMU.Imu.gaussX` |
| `pose.yaw_rad` | - | `egoYawRad` | - |
| `pose.heading_deg` | - | `egoHeadingDeg` | `IMU.Imu.yaw` |
| `velocity.speed_mps` | `v` | `egoV` | `IMU.Imu.velocity` |
| `acceleration.longitudinal_mps2` | `a` | `egoAcc` | `IMU.Imu.accX` |
| `road.s_m` | `s` | `egoS` | - |

---

## 9. ObjectList

**topic**：`gaasd.carla.object_list.v1`  
**对应 GAASD 结构体**：`perception.ObjectList`（每个 Object 的当前状态字段 x/y/vx/vy/l/w/h）  
注：`Prediction__Object` 是预测轨迹结构，不含当前直接状态字段，不应混用。

```json
{
  "source_type": "ground_truth",
  "object_count": 2,
  "objects": [
    {
      "object_id": 201,
      "type": "vehicle",
      "type_code": 1,
      "subtype": "vehicle.tesla.model3",
      "pose": {
        "x_m": 20.0,
        "y_m": 58.0,
        "z_m": 0.3,
        "yaw_rad": 1.5708,
        "heading_deg": 0.0
      },
      "velocity": {
        "vx_mps": 0.0,
        "vy_mps": 6.0,
        "speed_mps": 6.0
      },
      "acceleration": {
        "longitudinal_mps2": 0.0
      },
      "dimension": {
        "length_m": 4.6,
        "width_m": 1.9,
        "height_m": 1.6
      },
      "relative_to_ego": {
        "longitudinal_distance_m": 15.0,
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

**说明**：

- `clearance_m`：**保险杠间距**（前车车尾到自车车头），安全控制优先使用此值。
- `longitudinal_distance_m`：中心点纵向距离，兼容旧 `computeDistance1D`。
- `ttc_sec = clearance_m / max(-relative_speed_mps, ε)`，仅在自车接近前车时有效。

**类型枚举**：

| `type_code` | `type` | CARLA 来源 |
|---:|---|---|
| 0 | `unknown` | 未分类 |
| 1 | `vehicle` | `vehicle.*` |
| 2 | `pedestrian` | `walker.pedestrian.*` |
| 3 | `cyclist` | 自行车/摩托车（按场景配置映射） |
| 4 | `static` | 静态障碍物 |

**CARLA API 来源**：

```python
actors = world.get_actors().filter('vehicle.*')
for actor in actors:
    t   = actor.get_transform()
    vel = actor.get_velocity()
    bb  = actor.bounding_box
    # length_m = 2.0 * bb.extent.x
    # width_m  = 2.0 * bb.extent.y
    # height_m = 2.0 * bb.extent.z
```

**GAASD 结构体映射**：

| 标准字段 | GAASD `perception.Object` 字段 | 新组件端口 | 旧 Protobuf |
|---|---|---|---|
| `object_id` | `trackID` | `trackID` | `perception.Object.trackID` |
| `pose.x_m` | `x` | `objectX` | `perception.Object.x` |
| `pose.y_m` | `y` | `objectY` | `perception.Object.y` |
| `pose.z_m` | `z` | `objectZ` | `perception.Object.z` |
| `velocity.speed_mps` | - | `objectV` | - |
| `velocity.vx_mps` | `vx` | `objectVx` | `perception.Object.vx` |
| `velocity.vy_mps` | `vy` | `objectVy` | `perception.Object.vy` |
| `dimension.length_m` | `l` | `objectLength` | `perception.Object.l` |
| `dimension.width_m` | `w` | `objectWidth` | `perception.Object.w` |
| `dimension.height_m` | `h` | `objectHeight` | `perception.Object.h` |
| `type_code` | `type` | `objectType` | `perception.Object.type` |

---

## 10. LeadVehicle

**topic**：`gaasd.carla.lead_vehicle.v1`

专为 ACC 等跟车算法提供的便捷消息，Bridge 根据 ObjectList 计算同车道最近前车后发布。

```json
{
  "valid": true,
  "object_id": 201,
  "selection_rule": "same_lane_nearest_front",
  "type": "vehicle",
  "lead_speed_mps": 6.0,
  "ego_speed_mps": 8.3,
  "relative_speed_mps": -2.3,
  "longitudinal_distance_m": 15.0,
  "clearance_m": 10.4,
  "lateral_distance_m": 0.0,
  "time_gap_sec": 1.25,
  "ttc_sec": 4.52
}
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `valid` | bool | 是否存在前车 |
| `clearance_m` | double | 保险杠间距，安全控制首选 |
| `longitudinal_distance_m` | double | 中心点纵向距离，兼容 `computeDistance1D` |
| `relative_speed_mps` | double | 前车速度 − 自车速度（负=接近） |
| `time_gap_sec` | double | `clearance_m / max(ego_speed_mps, ε)` |
| `ttc_sec` | double | 碰撞时间，仅接近时有效 |

**ACC 推荐使用**：

```
egoV     = ego_speed_mps
leadV    = lead_speed_mps
distance = clearance_m          # 对应 ACC_DESIRED_DIST=15.0m 的语义
```

---

## 11. TrafficLightList

**topic**：`gaasd.carla.traffic_light_list.v1`  
**对应 GAASD 结构体**：`Infopack__TrafficLight`

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
        "yaw_rad": 1.5708
      }
    }
  ]
}
```

**注意**：CARLA 0.9.15 不支持 `remaining_time_sec`，固定填 `-1`。

**信号灯状态枚举**：

| `state_code` | `state` | CARLA `TrafficLightState` |
|---:|---|---|
| 0 | `red` | `Red = 0` |
| 1 | `yellow` | `Yellow = 1` |
| 2 | `green` | `Green = 2` |
| 3 | `off` | `Off = 3` |
| 4 | `unknown` | `Unknown = 4` |

**方向枚举**：

| `direction_code` | `direction` |
|---:|---|
| 0 | `straight` |
| 1 | `right` |
| 2 | `left` |
| 3 | `u_turn` |
| 4 | `unknown` |

**CARLA API 来源**：

```python
vehicle.is_at_traffic_light()         # bool
vehicle.get_traffic_light_state()     # carla.TrafficLightState
tl = vehicle.get_traffic_light()      # carla.TrafficLight actor
```

**GAASD 结构体映射**：

| 标准字段 | GAASD `Infopack__TrafficLight` 字段 | 新组件端口 |
|---|---|---|
| `lights[0].active` | `active` | `tlActive` |
| `lights[0].state_code` | `state` | `tlState` |
| `lights[0].distance_to_stop_line_m` | - | `tlDistance` |

---

## 12. Route

**topic**：`gaasd.carla.route.v1`  
**对应 GAASD 结构体**：`GaussRoadPoint[]`

```json
{
  "route_id": "town04_acc_demo_001",
  "route_source": "carla_waypoints",
  "waypoints": [
    {
      "index": 0,
      "x_m": 0.0,
      "y_m": 0.0,
      "z_m": 0.0,
      "yaw_rad": 0.0,
      "curvature_1pm": 0.0,
      "s_m": 0.0,
      "speed_limit_mps": 13.89,
      "road_id": 1,
      "lane_id": -1,
      "junction_id": -1
    }
  ]
}
```

`curvature_1pm`（单位 1/m）：Bridge 用数值微分计算：

```python
# 相邻两点的航向差 / 弧长差
curvature = (yaw[i+1] - yaw[i]) / max(ds, 1e-6)
```

**CARLA API 来源**：

```python
waypoint = map.get_waypoint(location)
next_wp  = waypoint.next(step_distance)   # 下一个路径点
speed_limit_kmh = vehicle.get_speed_limit()  # CARLA 返回 km/h
speed_limit_mps = speed_limit_kmh / 3.6      # 协议字段统一使用 m/s
```

**GAASD 结构体映射**：

| 标准字段 | GAASD `GaussRoadPoint` 字段 | 新组件端口 | 旧 Protobuf |
|---|---|---|---|
| `waypoints[i].x_m` | `GaussY`（Easting=x_m） | `routeX` | `Planning.TrajectoryPoint.X` |
| `waypoints[i].y_m` | `GaussX`（Northing=y_m） | `routeY` | `Planning.TrajectoryPoint.Y` |
| `waypoints[i].yaw_rad` | `yaw` | `routeYawRad` | `Planning.TrajectoryPoint.Theta` |
| `waypoints[i].curvature_1pm` | `curvature` | `routeCurvature` | - |
| `waypoints[i].s_m` | `s` | `routeS` | `Planning.TrajectoryPoint.S` |
| `waypoints[i].speed_limit_mps` | `speedMax` | `routeSpeedLimit` | `Planning.TrajectoryPoint.Speed` |

---

## 13. ControlCmd（GAASD → Bridge）

**topic**：`gaasd.carla.control_cmd.v1`

```json
{
  "command_id": 9001,
  "source_algorithm": "gaasd_acc",
  "enable": true,
  "target": {
    "steer_rad": 0.05,
    "target_speed_mps": 8.0,
    "target_accel_mps2": 0.2
  },
  "safety": {
    "max_speed_mps": 20.0,
    "max_accel_mps2": 4.0,
    "max_decel_mps2": 6.0,
    "max_abs_steer_rad": 0.6,
    "timeout_sec": 0.2
  }
}
```

算法组件**不直接**输出 `throttle/brake`。协议同时支持两种纵向输出方式：

- `target_speed_mps`：ACC 等速度规划算法使用，Bridge 内部用速度 PID 或 CARLA Ackermann 控制转为执行指令。
- `target_accel_mps2`：纵向控制算法直接输出加速度，Bridge 按 throttle/brake 标定值转换。

`steer_rad` 由 Bridge 归一化后下发。转换公式（标定值需按实际车型校准）：

**Bridge 内部转换（`target_accel_mps2` 路径，标定值需按实际车型校准）**：

```python
MAX_ACCEL = 4.0   # m/s²
MAX_BRAKE = 8.0   # m/s²
MAX_STEER = 0.6   # rad（前轮转角）

accel = target_accel_mps2
if accel >= 0:
    throttle = min(accel / MAX_ACCEL, 1.0)
    brake    = 0.0
else:
    throttle = 0.0
    brake    = min(-accel / MAX_BRAKE, 1.0)

steer_norm = max(-1.0, min(1.0, steer_rad / MAX_STEER))

vehicle.apply_control(carla.VehicleControl(
    throttle=throttle,
    brake=brake,
    steer=steer_norm
))
```

**GAASD 组件端口**：

| 标准字段 | 新组件端口 | 旧 Protobuf |
|---|---|---|
| `target.steer_rad` | `steer` | `controlData.ControlCMD.targetSteeringAngle` |
| `target.target_speed_mps` | `speed` | `controlData.ControlCMD.targetSpeed` |
| `target.target_accel_mps2` | `acceleration` | 新字段，旧 proto 无 |

---

## 14. TrajectoryCmd（GAASD 规划 → Bridge）

**topic**：`gaasd.carla.trajectory_cmd.v1`  
**对应 GAASD 结构体**：`Traj`

规划算法输出完整轨迹时使用，Bridge 据此做轨迹跟踪控制。

```json
{
  "source_algorithm": "gaasd_planner",
  "size": 50,
  "points": [
    {
      "index": 0,
      "x_m": 10.0,
      "y_m": 0.0,
      "yaw_rad": 0.0,
      "speed_mps": 5.0,
      "accel_mps2": 0.5,
      "curvature_1pm": 0.01,
      "s_m": 10.0
    }
  ]
}
```

**GAASD 结构体映射**：

| 标准字段 | GAASD `TrajPoint` 字段 | 说明 |
|---|---|---|
| `points[i].x_m` | `x` | East，m |
| `points[i].y_m` | `y` | North，m |
| `points[i].yaw_rad` | `yaw` | 航向角，rad |
| `points[i].speed_mps` | `v` | 期望速度，m/s |
| `points[i].accel_mps2` | `acc` | 期望加速度，m/s² |
| `points[i].curvature_1pm` | `k` | 曲率，1/m |
| `points[i].s_m` | `s` | 累计弧长，m |

---

## 15. ChassisFeedback（Bridge → GAASD）

**topic**：`gaasd.carla.chassis_feedback.v1`

```json
{
  "speed_mps": 8.2,
  "steering_angle_rad": 0.04,
  "steer_norm": 0.07,
  "throttle": 0.25,
  "brake": 0.0,
  "mode": 0,
  "gear": 1,
  "last_command_id": 9001,
  "last_command_age_sec": 0.03
}
```

`mode`：底盘/控制模式（0=手动，1=自动，与旧 `ChassisInfo.mode` 语义一致），**不是档位**。`gear` 为可选调试字段。

**旧 Protobuf 映射**：

| 旧字段 | 标准字段 |
|---|---|
| `ChassisInfo.speed` | `speed_mps` |
| `ChassisInfo.steeringAngle` | `steering_angle_rad` |
| `ChassisInfo.mode` | `mode` |

---

## 16. SceneCommand（GAASD → Bridge，可选）

**topic**：`gaasd.carla.scene_cmd.v1`

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
    "walkers": 0
  },
  "sync": {
    "synchronous_mode": true,
    "fixed_delta_seconds": 0.05
  }
}
```

| command | 说明 |
|---|---|
| `load_world` | 加载地图 |
| `reset` | 重置场景 |
| `spawn_ego` | 生成自车 |
| `spawn_traffic` | 生成 NPC |
| `pause` / `resume` | 暂停/恢复 tick |

---

## 17. Bridge 状态消息

**topic**：`gaasd.carla.bridge_status.v1`

```json
{
  "state": "running",
  "connected": true,
  "carla_launch_mode": "local_script",
  "carla_host": "127.0.0.1",
  "carla_port": 2000,
  "ssh_host": "",
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

| state | 说明 |
|---|---|
| `starting` | Bridge 启动中 |
| `connecting` | 连接 CARLA 中 |
| `running` | 正常运行 |
| `paused` | 暂停 |
| `error` | 错误 |
| `stopping` | 停止中 |

`carla_launch_mode` 取值：

| launch_mode | 说明 |
|---|---|
| `local_script` | 本机脚本启动 CARLA |
| `remote_ssh` | 通过 SSH 在服务器启动 CARLA |
| `external` | CARLA 已由外部系统启动，GAASD/Bridge 只负责连接 |

---

## 18. GAASD 组件库建议

GAASD 开发人员应新增以下标准 CARLA 组件。

### 18.1 输入组件

| 组件名 | 订阅 topic | 输出端口 | 说明 |
|---|---|---|---|
| `CARLASimClock` | `sim_clock` | `dt`, `frameId`, `simTime` | 全局时钟 |
| `CARLAEgoState` | `ego_state` | `egoX`, `egoY`, `egoYawRad`, `egoV`, `egoAcc` | 自车状态 |
| `CARLAObjectList` | `object_list` | object 数组或拆分端口 | 障碍物列表 |
| `CARLALeadVehicle` | `lead_vehicle` | `leadV`, `distance`, `relativeSpeed`, `ttc` | ACC/跟车 |
| `CARLATrafficLightList` | `traffic_light_list` | `tlActive`, `tlState`, `tlDistance` | 决策 |
| `CARLARoute` | `route` | waypoint 数组或 `GaussRoadPoint[]` | 规划路线 |
| `CARLAChassisFeedback` | `chassis_feedback` | `speed`, `steer`, `mode`, `gear` | 控制反馈 |

### 18.2 输出组件

| 组件名 | 输入端口 | 发布 topic | 说明 |
|---|---|---|---|
| `CARLAControlCmd` | `steer`, `speed`, `acceleration` | `control_cmd` | 通用控制 |
| `CARLALongitudinalCmd` | `speed` 或 `acceleration` | `control_cmd` | 纯纵向；输入 `speed` 时 Bridge 内部用速度 PID 转 accel |
| `CARLALateralCmd` | `steer` | `control_cmd` | 纯横向 |
| `CARLATrajectoryCmd` | `Traj` | `trajectory_cmd` | 规划轨迹输出 |

### 18.3 ACC 最小闭环端口

| 来源组件 | 端口 | 接到 |
|---|---|---|
| `CARLAEgoState` | `egoV` | `accComputeTargetSpeed.egoV` |
| `CARLALeadVehicle` | `leadV` | `accComputeTargetSpeed.leadV` |
| `CARLALeadVehicle` | `distance`（`clearance_m`） | `accComputeTargetSpeed.distance` |
| `accComputeTargetSpeed` | `targetSpeed` | `CARLALongitudinalCmd.speed`（Bridge 内部接速度 PID 转 accel，或直接用 CARLA Ackermann 控制） |

---

## 19. 旧 Protobuf 兼容

### 19.1 IMU.Imu

| 旧字段 | 标准字段 | 备注 |
|---|---|---|
| `gaussX` | `ego_state.pose.y_m` | Northing |
| `gaussY` | `ego_state.pose.x_m` | Easting |
| `yaw` | `ego_state.pose.heading_deg` | 北向 0，顺时针，单位 deg |
| `velocity` | `ego_state.velocity.speed_mps` | |
| `accX` | `ego_state.acceleration.longitudinal_mps2` | |
| `velocityEast` | `ego_state.velocity.vx_mps` | |
| `velocityNorth` | `ego_state.velocity.vy_mps` | |
| `gyroZ` | `ego_state.angular_velocity.yaw_rate_rps` | |

### 19.2 perception.ObjectList

| 旧字段 | 标准字段 |
|---|---|
| `Object.trackID` | `objects[].object_id` |
| `Object.x` | `objects[].pose.x_m` |
| `Object.y` | `objects[].pose.y_m` |
| `Object.l` | `objects[].dimension.length_m` |
| `Object.w` | `objects[].dimension.width_m` |
| `Object.h` | `objects[].dimension.height_m` |
| `Object.vx` | `objects[].velocity.vx_mps` |
| `Object.vy` | `objects[].velocity.vy_mps` |
| `Object.type` | `objects[].type_code` |

### 19.3 controlData.ControlCMD / ChassisInfo

| 旧字段 | 标准字段 |
|---|---|
| `ControlCMD.targetSteeringAngle` | `control_cmd.target.steer_rad` |
| `ControlCMD.targetSpeed` | `control_cmd.target.target_speed_mps` |
| `ControlCMD.obstacleDIS` | `lead_vehicle.clearance_m` |
| `ChassisInfo.speed` | `chassis_feedback.speed_mps` |
| `ChassisInfo.steeringAngle` | `chassis_feedback.steering_angle_rad` |

---

## 20. 安全与容错

Bridge 必须实现以下容错：

| 条件 | 处理 |
|---|---|
| `enable=false` | 不下发控制，保持或刹停 |
| 超过 `timeout_sec` 未收到 `control_cmd` | fail-safe：速度目标 0，制动 |
| `target_speed_mps < 0` | 钳位为 0，除非显式启用倒车 |
| `abs(steer_rad) > max_abs_steer_rad` | 钳位 |
| `target_accel_mps2` 超限 | 钳位 |
| JSON 字段缺失 | 拒绝该消息，记录错误 |
| NaN / Inf | 拒绝该消息，记录错误 |
| CARLA 连接断开 | 发布 `bridge_status.state = error`，停止下发旧数据 |

---

## 21. 最小验收用例

第一阶段验收只需闭环以下流程：

1. 启动 CARLA 0.9.15：本机模式执行 `cd /home/aiden/snap/code/app/carla-package && ./start-carla.sh`；服务器模式通过 GAASD 现有 SSH 入口在远程服务器执行等价启动命令。脚本内部若用相对路径调用 `./CarlaUE4.sh`，必须设置工作目录为脚本目录，或由脚本自行切换工作目录。
2. 启动 `carla_bridge.py`，连接配置中的 `carla_host:carla_port`，本机默认 `127.0.0.1:2000`，服务器模式使用远程服务器地址或 SSH 隧道地址，开启同步模式。
3. Bridge 发布 `sim_clock`、`ego_state`、`object_list`、`lead_vehicle`。
4. GAASD 画布读取 `egoV`、`leadV`、`distance`，计算 ACC 目标速度。
5. GAASD 通过 `CARLALongitudinalCmd` 输出 `target_speed_mps`（ACC 最小闭环用速度输出）。
6. Bridge 用速度 PID 或 CARLA Ackermann 控制将目标速度应用到 ego 车。
7. `chassis_feedback` 回传当前速度，闭环验证速度收敛。

不要求相机/激光雷达/Pangu 容器/leaderboard。

---

## 22. 责任边界

**Bridge / CARLA 联调侧负责内容**：

- 本机 CARLA 启停脚本：`start-carla.sh` / `stop-carla.sh` / `health-carla.sh`
- `carla_bridge.py`（本协议实现，支持配置 `carla_host` / `carla_port` 连接本机或服务器 CARLA）
- ACC 最小闭环示例

**GAASD 软件侧负责内容**：

- 仿真管理界面同时支持本地脚本模式、服务器 SSH 模式和外部已启动模式
- 保留并补齐现有 SSH 接口：远程 IP、端口、用户名、认证方式、远程 CARLA 根目录、远程启动脚本、工作目录、环境变量和日志输出
- 组件库新增 §18 列出的标准 CARLA 组件
- 旧 Pangu/容器场景逻辑与本地 CARLA 官方包模式拆分；服务器模式可按配置继续使用容器逻辑
- 必要时将 JSON 契约映射为内部 Protobuf/ZMQ

---

## 23. 待 GAASD 团队确认项

| 项 | 当前建议 | 说明 |
|---|---|---|
| 标准传输 | JSON over ZMQ PUB/SUB | 与组件库对接最简 |
| Yaw 方向 | `yaw_rad = -radians(carla.rotation.yaw)` | 消除左手/右手差异 |
| 旧 IMU yaw | `heading_deg`，北向 0，顺时针 | 旧规划代码按度使用 |
| ACC 距离语义 | `clearance_m`（保险杠间距） | 比中心距安全语义更明确 |
| throttle/brake 标定 | `MAX_ACCEL=4.0`，`MAX_BRAKE=8.0` | 需按实际车型在 CARLA 中标定 |
| 新组件端口名 | 使用 §18 端口名 | 便于代码生成模板统一 |
| CARLA 启动模式 | `local_script` / `remote_ssh` / `external` | SSH 服务器模式保留，本地模式补齐 |
