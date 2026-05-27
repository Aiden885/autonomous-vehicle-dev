# GAASD 接入 CARLA 需要软件开发团队配合的事项

版本：v0.2  
日期：2026-04-24  
参考协议：`docs/carla_gaasd_bridge_protocol.md`

## 1. 背景和目标

当前目标不是只让 ACC 算法跑通，而是让 GAASD 后续的规划、控制、ACC、预测、感知验证等算法都能复用同一个 CARLA 仿真入口。

Python Bridge 负责连接 CARLA Python API，并按 `CARLA ↔ GAASD Bridge 数据协议` 向 GAASD 发布标准数据、接收算法控制输出。GAASD 软件侧需要提供可配置的仿真管理、标准 CARLA 组件库、代码生成模板和工程落盘支持。

## 2. 当前 GAASD 侧存在的问题


| 问题                       | 当前现象                                                              | 影响                                      |
| ------------------------ | ----------------------------------------------------------------- | --------------------------------------- |
| CARLA 启动路径和方式硬编码         | 当前启动逻辑倾向写死 `/home/aicc/CARLA_${version}/CarlaUE4.sh`              | 无法同时适配本机脚本和服务器 SSH 启动                   |
| 仿真器版本不完整                 | 仿真器列表只有 `carla_0.9.12`、`carla_0.9.14`                             | 无法选择本机 CARLA `0.9.15`                   |
| 启动模式没有清晰拆分               | 现有场景逻辑依赖 `suc_node`、`suc_leadboard`，同时又预留 SSH 入口                  | 服务器模式应保留 SSH/容器能力，本地 CARLA 官方包模式不应强依赖容器 |
| 组件库缺少 CARLA 标准输入输出组件     | 现有组件不能直接表达 `ego_state`、`object_list`、`lead_vehicle`、`control_cmd` | 画布无法以标准协议对接 Bridge                      |
| 代码生成模板缺少 CARLA Bridge 契约 | 组件端口和 JSON/ZMQ 消息没有统一生成规则                                         | 工程生成后无法稳定运行或复现                          |
| 工程落盘格式缺少 CARLA 场景配置      | 工程保存时没有固定记录 CARLA 路径、Bridge 端口、协议版本等                              | 换机器或重新打开工程后配置丢失                         |


## 3. 责任边界

### 3.1 Bridge / CARLA 联调侧负责内容


| 工作项           | 说明                                                                                                |
| ------------- | ------------------------------------------------------------------------------------------------- |
| 本机 CARLA 启停脚本 | `start-carla.sh`、`stop-carla.sh`、`health-carla.sh`                                                |
| Python Bridge | `carla_bridge.py`，实现 CARLA API 与 GAASD 标准协议之间的转换，并支持配置 `carla_host` / `carla_port` 连接本机或服务器 CARLA |
| 标准数据协议        | `docs/carla_gaasd_bridge_protocol.md`                                                             |
| ACC 最小闭环示例    | 用于验证 `ego_state`、`lead_vehicle`、`CARLALongitudinalCmd`、`chassis_feedback` 闭环                      |


### 3.2 GAASD 软件侧负责内容


| 工作项          | 说明                                           |
| ------------ | -------------------------------------------- |
| 仿真管理界面修改     | 支持本地脚本模式、服务器 SSH 模式、外部已启动模式、Bridge 配置和运行状态展示 |
| 标准 CARLA 组件库 | 新增 CARLA 输入组件、输出组件和状态组件                      |
| 代码生成模板修改     | 能按协议生成订阅、发布、端口绑定和数据结构转换代码                    |
| 工程落盘格式修改     | 保存 CARLA 场景配置、Bridge 配置、协议版本和组件端口信息          |
| 示例工程         | 提供一个可打开、可生成、可运行的 ACC 最小闭环工程                  |


## 4. 仿真管理界面需要修改

### 4.1 支持本地和服务器 CARLA 模式

GAASD 需要把 CARLA 启动位置拆分为三种模式。原有 SSH 接口不应删除，应作为服务器模式继续可用。


| 模式             | 说明                                                              |
| -------------- | --------------------------------------------------------------- |
| `local_script` | 使用本机 CARLA 官方包，通过本地脚本启动，不依赖 `suc_node`、`suc_leadboard`          |
| `remote_ssh`   | 通过 SSH 登录服务器启动 CARLA，保留原有远程仿真入口，可按配置继续使用 Pangu/leaderboard/容器逻辑 |
| `external`     | CARLA 已由外部系统启动，GAASD 不负责启动，只保存连接地址并启动 Bridge                    |


本地模式下，不应再强制使用 `/home/aicc/CARLA_${version}/CarlaUE4.sh`。服务器模式下，可以继续使用服务器上的 CARLA 路径，但必须通过界面或工程配置填写，不应写死。

### 4.2 新增 CARLA 0.9.15 和启动配置

仿真器类型或版本列表需要支持：

```text
carla_0.9.15
```

本机 CARLA 示例配置：

```text
carlaRoot = /home/aiden/snap/code/app/carla-package
startCommand = cd /home/aiden/snap/code/app/carla-package && ./start-carla.sh
carlaHost = 127.0.0.1
carlaPort = 2000
```

服务器 SSH 示例配置：

```text
sshHost = <server_ip>
sshPort = 22
sshUser = <user>
remoteCarlaRoot = /home/aicc/CARLA_0.9.15
remoteStartCommand = cd /home/aicc/CARLA_0.9.15 && ./CarlaUE4.sh
carlaHostForBridge = <server_ip> 或 127.0.0.1（使用 SSH 隧道时）
carlaPortForBridge = 2000
```

注意：`start-carla.sh` 脚本应自行 `cd "$(dirname "$0")"` 后再调用 `./CarlaUE4.sh`，或者由 GAASD 在启动脚本时显式设置 `scriptWorkingDirectory`。远程服务器脚本同理，需要配置远程工作目录，不能依赖 GAASD 当前进程的默认工作目录。

### 4.3 配置项建议

仿真管理界面建议至少提供以下配置：


| 配置项                      | 示例                                                        | 说明                          |
| ------------------------ | --------------------------------------------------------- | --------------------------- |
| `simulatorType`          | `carla`                                                   | 仿真器类型                       |
| `carlaVersion`           | `0.9.15`                                                  | CARLA 版本                    |
| `launchMode`             | `local_script` / `remote_ssh` / `external`                | CARLA 启动模式                  |
| `carlaRoot`              | `/home/aiden/snap/code/app/carla-package`                 | CARLA 安装目录                  |
| `startScript`            | `/home/aiden/snap/code/app/carla-package/start-carla.sh`  | 启动脚本                        |
| `stopScript`             | `/home/aiden/snap/code/app/carla-package/stop-carla.sh`   | 停止脚本，可选                     |
| `healthScript`           | `/home/aiden/snap/code/app/carla-package/health-carla.sh` | 健康检查脚本，可选                   |
| `scriptWorkingDirectory` | `/home/aiden/snap/code/app/carla-package`                 | 脚本执行工作目录                    |
| `sshHost`                | `192.168.x.x`                                             | 服务器模式下的 SSH 主机              |
| `sshPort`                | `22`                                                      | SSH 端口                      |
| `sshUser`                | `aicc`                                                    | SSH 用户名                     |
| `sshAuthType`            | `password` / `key`                                        | SSH 认证方式                    |
| `remoteCarlaRoot`        | `/home/aicc/CARLA_0.9.15`                                 | 远程 CARLA 安装目录               |
| `remoteStartScript`      | `./CarlaUE4.sh`                                           | 远程启动脚本                      |
| `remoteWorkingDirectory` | `/home/aicc/CARLA_0.9.15`                                 | 远程脚本工作目录                    |
| `useSshTunnel`           | `false`                                                   | 是否通过 SSH 隧道访问 CARLA 2000 端口 |
| `carlaHost`              | `127.0.0.1`                                               | CARLA Python Client 连接地址    |
| `carlaPort`              | `2000`                                                    | CARLA Python Client 默认端口    |
| `syncMode`               | `true`                                                    | 是否同步模式                      |
| `fixedDeltaSeconds`      | `0.05`                                                    | 仿真步长，20 Hz                  |
| `mapName`                | `Town04`                                                  | 默认地图                        |
| `quality`                | `Low`                                                     | 联调建议低画质                     |


### 4.4 运行状态展示

GAASD 界面建议展示 Bridge 状态消息：

```text
gaasd.carla.bridge_status.v1
```

至少展示：


| 字段            | 说明                                                            |
| ------------- | ------------------------------------------------------------- |
| `state`       | `starting`、`connecting`、`running`、`paused`、`error`、`stopping` |
| `connected`   | 是否连接 CARLA                                                    |
| `carla_host`  | CARLA 地址                                                      |
| `carla_port`  | CARLA 端口                                                      |
| `map_name`    | 当前地图                                                          |
| `ego_spawned` | 自车是否生成                                                        |
| `last_error`  | 最近错误                                                          |


## 5. 组件库需要新增的 CARLA 组件

### 5.1 组件实现原则

GAASD 组件库开发人员不需要直接调用 CARLA Python API。CARLA API 由 `carla_bridge.py` 调用，GAASD 组件只需要订阅或发布协议中的 ZMQ JSON 消息。

组件实现必须遵守以下约定：


| 约定    | 要求                                                                     |
| ----- | ---------------------------------------------------------------------- |
| 坐标系   | 组件看到的坐标已经由 Bridge 转成 GAASD ENU 坐标，`x=East`，`y=North`，不要再使用 CARLA 原始左手系 |
| 单位    | 距离 m，速度 m/s，加速度 m/s²，角度 rad，时间 s                                       |
| topic | 组件订阅或发布协议 topic，内部 UI 可显示短名，例如 `ego_state`                             |
| 时间戳   | 所有输入组件建议输出 `frameId`、`simTime`、`dt`，便于算法判断数据同步                         |
| 有效性   | 每个输入组件建议输出 `valid` 或 `hasData`，超时或字段缺失时置 false                         |
| 数组    | `object_list`、`route`、`traffic_light_list` 可先支持固定最大长度数组，后续再支持动态数组      |
| 错误处理  | JSON 字段缺失、类型错误、NaN/Inf 时不应崩溃，应输出默认安全值并记录错误                             |


### 5.2 组件优先级


| 优先级 | 组件                                                                                               | 目标              |
| --- | ------------------------------------------------------------------------------------------------ | --------------- |
| P0  | `CARLASimClock`、`CARLAEgoState`、`CARLALeadVehicle`、`CARLALongitudinalCmd`、`CARLAChassisFeedback` | 跑通 ACC 最小闭环     |
| P1  | `CARLAObjectList`、`CARLAControlCmd`、`CARLALateralCmd`                                            | 支持通用障碍物输入和横纵向控制 |
| P2  | `CARLATrafficLightList`、`CARLARoute`、`CARLATrajectoryCmd`                                        | 支持后续决策、规划和轨迹跟踪  |


### 5.3 输入组件总表


| 组件名                     | 订阅 topic                            | 默认频率        | 主要输出端口                                    | 用途     |
| ----------------------- | ----------------------------------- | ----------- | ----------------------------------------- | ------ |
| `CARLASimClock`         | `gaasd.carla.sim_clock.v1`          | 每 tick      | `dt`、`frameId`、`simTime`                  | 全局仿真时钟 |
| `CARLAEgoState`         | `gaasd.carla.ego_state.v1`          | 每 tick      | `egoX`、`egoY`、`egoYawRad`、`egoV`、`egoAcc` | 自车状态   |
| `CARLAObjectList`       | `gaasd.carla.object_list.v1`        | 每 tick      | object 数组或拆分端口                            | 障碍物列表  |
| `CARLALeadVehicle`      | `gaasd.carla.lead_vehicle.v1`       | 每 tick      | `leadV`、`distance`、`relativeSpeed`、`ttc`  | ACC/跟车 |
| `CARLATrafficLightList` | `gaasd.carla.traffic_light_list.v1` | 10 Hz 或状态变化 | `tlActive`、`tlState`、`tlDistance`         | 信号灯决策  |
| `CARLARoute`            | `gaasd.carla.route.v1`              | 场景加载或路线变化   | waypoint 数组或 `GaussRoadPoint[]`           | 规划路线   |
| `CARLAChassisFeedback`  | `gaasd.carla.chassis_feedback.v1`   | 每 tick      | `speed`、`steer`、`mode`、`gear`             | 控制反馈   |


### 5.4 `CARLASimClock`

作用：给 GAASD 画布提供 CARLA 仿真时间，避免算法使用真实墙钟导致仿真不同步。

Bridge 侧 CARLA 来源：

```python
settings = world.get_settings()
snapshot = world.get_snapshot()
```

组件输出端口：


| 端口         | 类型     | 单位  | 说明                   |
| ---------- | ------ | --- | -------------------- |
| `valid`    | bool   | 1   | 是否收到有效时钟             |
| `dt`       | double | s   | 当前 tick 步长，默认 0.05   |
| `frameId`  | uint64 | 1   | CARLA world frame 编号 |
| `simTime`  | double | s   | CARLA 仿真累计时间         |
| `syncMode` | bool   | 1   | 是否同步模式               |
| `paused`   | bool   | 1   | 仿真是否暂停               |


组件行为：


| 场景              | 处理                         |
| --------------- | -------------------------- |
| 未收到消息           | `valid=false`，`dt` 使用配置默认值 |
| `dt <= 0` 或 NaN | `valid=false`，算法不应推进状态     |
| frame 跳变        | 记录 warning，但继续输出最新帧        |


### 5.5 `CARLAEgoState`

作用：提供自车位置、姿态、速度、加速度和道路信息，是控制、规划、ACC 的基础输入。

Bridge 侧 CARLA 来源：

```python
transform = vehicle.get_transform()
velocity = vehicle.get_velocity()
acceleration = vehicle.get_acceleration()
ang_vel = vehicle.get_angular_velocity()
waypoint = map.get_waypoint(transform.location)
```

核心端口：


| 端口              | 类型     | 单位    | 说明                        |
| --------------- | ------ | ----- | ------------------------- |
| `valid`         | bool   | 1     | 自车状态是否有效                  |
| `egoX`          | double | m     | ENU East，协议字段 `pose.x_m`  |
| `egoY`          | double | m     | ENU North，协议字段 `pose.y_m` |
| `egoZ`          | double | m     | 高度，可选                     |
| `egoYawRad`     | double | rad   | ENU 航向角，逆时针为正             |
| `egoHeadingDeg` | double | deg   | 旧链路兼容，北向 0，顺时针            |
| `egoV`          | double | m/s   | 自车速度模长                    |
| `egoVx`         | double | m/s   | East 方向速度                 |
| `egoVy`         | double | m/s   | North 方向速度                |
| `egoAcc`        | double | m/s²  | 沿车身前向投影后的纵向加速度            |
| `egoYawRate`    | double | rad/s | yaw 角速度                   |
| `egoS`          | double | m     | 当前道路 s 坐标，可选              |
| `laneId`        | int    | 1     | CARLA lane_id             |
| `roadId`        | int    | 1     | CARLA road_id             |


关键注意事项：


| 项     | 正确做法                                   |
| ----- | -------------------------------------- |
| 坐标    | Bridge 已将 CARLA `y` 取反成 North，组件不要二次取反 |
| yaw   | Bridge 已将 CARLA 顺时针 yaw 转为 ENU 右手系 rad |
| 加速度   | `egoAcc` 是纵向投影值，不是 CARLA 原始三轴模长        |
| 旧 IMU | `gaussX = egoY`，`gaussY = egoX`，不能写反   |


### 5.6 `CARLAObjectList`

作用：提供 CARLA 场景中的车辆、行人、静态障碍物状态。第一阶段可只支持车辆，后续再扩展行人和静态物体。

Bridge 侧 CARLA 来源：

```python
actors = world.get_actors()
vehicles = actors.filter("vehicle.*")
walkers = actors.filter("walker.pedestrian.*")
```

建议组件参数：


| 参数                | 默认值   | 说明        |
| ----------------- | ----- | --------- |
| `maxObjects`      | 64    | 最大输出对象数   |
| `includeVehicles` | true  | 是否输出车辆    |
| `includeWalkers`  | false | 是否输出行人    |
| `includeStatic`   | false | 是否输出静态障碍物 |


输出端口建议：


| 端口               | 类型    | 单位  | 说明                                                  |
| ---------------- | ----- | --- | --------------------------------------------------- |
| `valid`          | bool  | 1   | object_list 是否有效                                    |
| `objectCount`    | int   | 1   | 实际对象数量                                              |
| `objectId[]`     | Array | 1   | CARLA actor id 或 trackID                            |
| `objectType[]`   | Array | 1   | 0 unknown，1 vehicle，2 pedestrian，3 cyclist，4 static |
| `objectX[]`      | Array | m   | ENU East                                            |
| `objectY[]`      | Array | m   | ENU North                                           |
| `objectYawRad[]` | Array | rad | 对象航向                                                |
| `objectV[]`      | Array | m/s | 速度模长                                                |
| `objectVx[]`     | Array | m/s | East 方向速度                                           |
| `objectVy[]`     | Array | m/s | North 方向速度                                          |
| `objectLength[]` | Array | m   | 物体长度，CARLA bounding box extent.x 的 2 倍              |
| `objectWidth[]`  | Array | m   | 物体宽度，CARLA bounding box extent.y 的 2 倍              |
| `objectHeight[]` | Array | m   | 物体高度，CARLA bounding box extent.z 的 2 倍              |


实现注意：


| 项      | 要求                                                  |
| ------ | --------------------------------------------------- |
| 当前状态结构 | 使用 `perception.ObjectList` / `perception.Object` 语义 |
| 预测结构   | 不要使用 `Prediction__Object` 表达当前障碍物状态                 |
| 数组超限   | 超过 `maxObjects` 时截断并记录 warning                      |
| 自车过滤   | 不应把 ego 车输出为障碍物                                     |
| 坐标     | 所有对象坐标均为 GAASD ENU 坐标                               |


### 5.7 `CARLALeadVehicle`

作用：专门给 ACC 或跟车算法提供最近前车，避免画布里每次都从 ObjectList 手动筛选。

Bridge 侧来源：由 Bridge 基于 `ego_state` 和 `object_list` 计算同车道最近前车。

输出端口：


| 端口                | 类型     | 单位  | 说明                        |
| ----------------- | ------ | --- | ------------------------- |
| `valid`           | bool   | 1   | 是否存在前车                    |
| `leadId`          | int    | 1   | 前车对象 id                   |
| `leadV`           | double | m/s | 前车速度                      |
| `egoV`            | double | m/s | 自车速度快照                    |
| `distance`        | double | m   | ACC 使用距离，等于 `clearance_m` |
| `centerDistance`  | double | m   | 中心点纵向距离                   |
| `relativeSpeed`   | double | m/s | 前车速度减自车速度，负值表示接近          |
| `lateralDistance` | double | m   | 横向距离                      |
| `timeGap`         | double | s   | `distance / max(egoV, ε)` |
| `ttc`             | double | s   | 碰撞时间，仅接近时有效               |


无前车时的默认值：


| 端口              | 默认值   |
| --------------- | ----- |
| `valid`         | false |
| `leadV`         | 0.0   |
| `distance`      | `1e6` |
| `relativeSpeed` | 0.0   |
| `ttc`           | `1e6` |


ACC 连接要求：


| ACC 输入     | 连接端口                        |
| ---------- | --------------------------- |
| `egoV`     | `CARLAEgoState.egoV`        |
| `leadV`    | `CARLALeadVehicle.leadV`    |
| `distance` | `CARLALeadVehicle.distance` |


### 5.8 `CARLATrafficLightList`

作用：为后续决策和规划提供信号灯状态。第一阶段 ACC 不强制实现。

Bridge 侧 CARLA 来源：

```python
vehicle.is_at_traffic_light()
vehicle.get_traffic_light_state()
vehicle.get_traffic_light()
```

输出端口：


| 端口                | 类型     | 单位  | 说明                                           |
| ----------------- | ------ | --- | -------------------------------------------- |
| `valid`           | bool   | 1   | 信号灯数据是否有效                                    |
| `tlActive`        | bool   | 1   | 当前是否受信号灯影响                                   |
| `tlState`         | int    | 1   | 0 red，1 yellow，2 green，3 off，4 unknown       |
| `tlDistance`      | double | m   | 到停止线或信号灯作用点距离                                |
| `tlRemainingTime` | double | s   | CARLA 0.9.15 不支持时固定 -1                       |
| `tlDirection`     | int    | 1   | 0 straight，1 right，2 left，3 u_turn，4 unknown |


无信号灯时：


| 端口                | 默认值   |
| ----------------- | ----- |
| `tlActive`        | false |
| `tlState`         | 4     |
| `tlDistance`      | `1e6` |
| `tlRemainingTime` | -1    |


### 5.9 `CARLARoute`

作用：为规划和轨迹跟踪提供路线点。路线可以来自 CARLA waypoint，也可以来自外部场景配置。

Bridge 侧 CARLA 来源：

```python
waypoint = map.get_waypoint(location)
next_wp = waypoint.next(step_distance)
speed_limit_mps = vehicle.get_speed_limit() / 3.6
```

输出端口建议：


| 端口                  | 类型    | 单位  | 说明                  |
| ------------------- | ----- | --- | ------------------- |
| `valid`             | bool  | 1   | route 是否有效          |
| `routeSize`         | int   | 1   | 路点数量                |
| `routeX[]`          | Array | m   | ENU East            |
| `routeY[]`          | Array | m   | ENU North           |
| `routeYawRad[]`     | Array | rad | 路点航向                |
| `routeCurvature[]`  | Array | 1/m | 曲率                  |
| `routeS[]`          | Array | m   | 累计弧长                |
| `routeSpeedLimit[]` | Array | m/s | 限速，CARLA km/h 转 m/s |
| `laneId[]`          | Array | 1   | lane_id             |
| `roadId[]`          | Array | 1   | road_id             |


旧结构兼容：


| 标准字段                           | `GaussRoadPoint` 字段 |
| ------------------------------ | ------------------- |
| `routeX[]` / `waypoints[].x_m` | `GaussY`，Easting    |
| `routeY[]` / `waypoints[].y_m` | `GaussX`，Northing   |


### 5.10 `CARLAChassisFeedback`

作用：把 CARLA 当前车辆执行状态回传给 GAASD 控制闭环和界面显示。

Bridge 侧来源：车辆当前速度、转角、最近一次控制命令、CARLA vehicle control 状态。

输出端口：


| 端口               | 类型     | 单位  | 说明                                      |
| ---------------- | ------ | --- | --------------------------------------- |
| `valid`          | bool   | 1   | 反馈是否有效                                  |
| `speed`          | double | m/s | 当前车速                                    |
| `steer`          | double | rad | 当前前轮转角或等效转角                             |
| `steerNorm`      | double | 1   | CARLA `VehicleControl.steer`，范围 [-1, 1] |
| `throttle`       | double | 1   | 油门，范围 [0, 1]                            |
| `brake`          | double | 1   | 制动，范围 [0, 1]                            |
| `mode`           | int    | 1   | 控制模式，0 手动，1 自动                          |
| `gear`           | int    | 1   | 档位，可选调试字段                               |
| `lastCommandAge` | double | s   | 最近控制命令距当前时间                             |


关键注意事项：


| 项      | 正确语义                                       |
| ------ | ------------------------------------------ |
| `mode` | 对应旧 `ChassisInfo.mode`，表示控制模式，不是档位         |
| `gear` | 可选调试字段，不应替代 `mode`                         |
| 超时     | `lastCommandAge` 超过阈值时应能触发 fail-safe 显示或报警 |


### 5.11 输出组件总表


| 组件名                    | 输入端口                           | 发布 topic                        | 用途     |
| ---------------------- | ------------------------------ | ------------------------------- | ------ |
| `CARLAControlCmd`      | `steer`、`speed`、`acceleration` | `gaasd.carla.control_cmd.v1`    | 通用控制   |
| `CARLALongitudinalCmd` | `speed` 或 `acceleration`       | `gaasd.carla.control_cmd.v1`    | 纯纵向控制  |
| `CARLALateralCmd`      | `steer`                        | `gaasd.carla.control_cmd.v1`    | 纯横向控制  |
| `CARLATrajectoryCmd`   | `Traj`                         | `gaasd.carla.trajectory_cmd.v1` | 规划轨迹输出 |


控制输出互斥规则：


| 场景            | 要求                                                                                             |
| ------------- | ---------------------------------------------------------------------------------------------- |
| 只跑 ACC 或纯纵向算法 | 只启用 `CARLALongitudinalCmd`                                                                     |
| 只跑横向控制算法      | 只启用 `CARLALateralCmd`，且 Bridge 必须有明确纵向控制来源                                                     |
| 横向和纵向控制同时存在   | 优先使用 `CARLAControlCmd` 合并发布，不允许 `CARLALongitudinalCmd` 和 `CARLALateralCmd` 分别同时向 Bridge 发布生效控制 |
| 多个控制组件同时连接    | 同一时刻只能有一个控制输出组件处于 active 状态，否则 Bridge 行为未定义                                                    |
| 后续需要多源控制      | 应新增 `CARLAControlMux` 或等价仲裁组件，按优先级、时间戳和 enable 状态选择唯一控制命令                                      |


### 5.12 `CARLAControlCmd`

作用：通用控制输出组件，适合横向和纵向控制算法同时输出。

输入端口：


| 端口                 | 类型     | 单位   | 说明                     |
| ------------------ | ------ | ---- | ---------------------- |
| `enable`           | bool   | 1    | 是否启用控制                 |
| `steer`            | double | rad  | 目标转角                   |
| `speed`            | double | m/s  | 目标速度                   |
| `acceleration`     | double | m/s² | 目标加速度                  |
| `longitudinalMode` | int    | 1    | 0 speed，1 acceleration |
| `maxSpeed`         | double | m/s  | 最大速度限制                 |
| `maxAccel`         | double | m/s² | 最大加速度                  |
| `maxDecel`         | double | m/s² | 最大制动减速度                |
| `timeout`          | double | s    | 控制命令超时时间               |


实现要求：


| 场景                   | 处理                                                        |
| -------------------- | --------------------------------------------------------- |
| `longitudinalMode=0` | 发布 `target_speed_mps`，Bridge 用速度 PID 或 CARLA Ackermann 控制 |
| `longitudinalMode=1` | 发布 `target_accel_mps2`，Bridge 转 throttle/brake            |
| `enable=false`       | Bridge 不下发控制或执行安全停车                                       |
| `steer` 超限           | 组件或 Bridge 应钳位到 `max_abs_steer_rad`                       |


说明：`longitudinalMode` 是 GAASD 组件内部选择端口，用于决定发布 `target_speed_mps` 还是 `target_accel_mps2`，不要求 Bridge 协议新增同名字段。

使用约束：当 `CARLAControlCmd` 处于 active 状态时，`CARLALongitudinalCmd` 和 `CARLALateralCmd` 不应同时作为生效控制输出接入 Bridge。

### 5.13 `CARLALongitudinalCmd`

作用：纯纵向输出组件，ACC 最小闭环优先使用该组件。

输入端口：


| 端口             | 类型     | 单位   | 说明                     |
| -------------- | ------ | ---- | ---------------------- |
| `enable`       | bool   | 1    | 是否启用                   |
| `speed`        | double | m/s  | 目标速度，ACC 使用            |
| `acceleration` | double | m/s² | 目标加速度，可选               |
| `mode`         | int    | 1    | 0 speed，1 acceleration |
| `maxSpeed`     | double | m/s  | 最大速度限制                 |
| `timeout`      | double | s    | 超时时间                   |


ACC 第一阶段使用：

```text
accComputeTargetSpeed.targetSpeed -> CARLALongitudinalCmd.speed
CARLALongitudinalCmd.mode = 0
```

说明：这里的 `mode` 是组件内部纵向输出模式，不是 `ChassisInfo.mode`。

使用约束：`CARLALongitudinalCmd` 与 `CARLAControlCmd`、`CARLALateralCmd` 共用 `gaasd.carla.control_cmd.v1`，同一时刻只能有一个控制输出组件处于 active 状态；ACC 最小闭环中应只启用 `CARLALongitudinalCmd`。

### 5.14 `CARLALateralCmd`

作用：纯横向输出组件，供 Pure Pursuit、LQR、MPC 等横向控制算法使用。

输入端口：


| 端口            | 类型     | 单位  | 说明     |
| ------------- | ------ | --- | ------ |
| `enable`      | bool   | 1   | 是否启用   |
| `steer`       | double | rad | 目标转角   |
| `maxAbsSteer` | double | rad | 最大绝对转角 |
| `timeout`     | double | s   | 超时时间   |


注意：如果只发布横向控制，Bridge 需要明确纵向控制来源。建议 GAASD 在工程中用 `CARLAControlCmd` 合并横纵向输出，或在 Bridge 中配置“横向保持最近纵向命令”的策略。

### 5.15 `CARLATrajectoryCmd`

作用：规划算法输出完整轨迹时使用，Bridge 可据此进行轨迹跟踪控制。

输入端口：


| 端口                | 类型    | 单位   | 说明        |
| ----------------- | ----- | ---- | --------- |
| `trajSize`        | int   | 1    | 轨迹点数量     |
| `trajX[]`         | Array | m    | ENU East  |
| `trajY[]`         | Array | m    | ENU North |
| `trajYawRad[]`    | Array | rad  | 轨迹点航向     |
| `trajV[]`         | Array | m/s  | 期望速度      |
| `trajAcc[]`       | Array | m/s² | 期望加速度     |
| `trajCurvature[]` | Array | 1/m  | 曲率        |
| `trajS[]`         | Array | m    | 累计弧长      |


实现要求：


| 场景      | 处理                        |
| ------- | ------------------------- |
| 空轨迹     | 不发布或发布 `enable=false`     |
| 轨迹点超过上限 | 截断并记录 warning             |
| 坐标      | 必须是 ENU 坐标，不允许 CARLA 原始坐标 |
| 速度      | 统一 m/s，不允许 km/h           |


### 5.16 ACC 最小闭环连接

第一阶段验收建议使用最小 ACC 闭环：


| 来源组件                    | 端口            | 接到                               |
| ----------------------- | ------------- | -------------------------------- |
| `CARLAEgoState`         | `egoV`        | `accComputeTargetSpeed.egoV`     |
| `CARLALeadVehicle`      | `leadV`       | `accComputeTargetSpeed.leadV`    |
| `CARLALeadVehicle`      | `distance`    | `accComputeTargetSpeed.distance` |
| `accComputeTargetSpeed` | `targetSpeed` | `CARLALongitudinalCmd.speed`     |


其中 `distance` 对应协议里的 `lead_vehicle.clearance_m`，即保险杠间距，不是中心点距离。

## 6. 代码生成模板需要支持的内容

### 6.1 JSON over ZMQ 组件生成

GAASD 代码生成器需要支持标准传输：

```text
JSON over ZMQ PUB/SUB
```

默认端口建议：


| 方向             | 地址                     | 说明                                           |
| -------------- | ---------------------- | -------------------------------------------- |
| Bridge → GAASD | `tcp://127.0.0.1:5701` | GAASD 输入组件订阅，默认本机；跨机器部署时需可配置 bind/connect 地址 |
| GAASD → Bridge | `tcp://127.0.0.1:5702` | GAASD 输出组件发布，默认本机；跨机器部署时需可配置 bind/connect 地址 |
| GAASD ↔ Bridge | `tcp://127.0.0.1:5703` | 可选场景命令服务，默认本机；跨机器部署时需可配置 bind/connect 地址     |


模板需要能生成：


| 能力        | 说明                                                 |
| --------- | -------------------------------------------------- |
| topic 订阅  | 按 `sim_clock`、`ego_state`、`object_list` 等 topic 解析 |
| JSON 字段解析 | 将 JSON 字段映射为组件输出端口                                 |
| JSON 字段发布 | 将组件输入端口封装为 `control_cmd`、`trajectory_cmd`          |
| 数组字段处理    | 支持 object 数组、waypoint 数组、traffic light 数组          |
| 缺省值处理     | 字段缺失时给出安全默认值或报错                                    |
| 错误日志      | JSON 解析失败、类型错误、NaN/Inf 要可定位                        |
| 协议版本      | 生成物记录 `gaasd_carla_bridge` 协议版本                    |


### 6.2 旧 Protobuf 兼容

如果 GAASD 内部仍需要复用旧算法进程，代码生成模板需要保留以下映射能力：


| 旧结构                       | 标准协议               |
| ------------------------- | ------------------ |
| `IMU.Imu`                 | `ego_state`        |
| `perception.ObjectList`   | `object_list`      |
| `controlData.ControlCMD`  | `control_cmd`      |
| `controlData.ChassisInfo` | `chassis_feedback` |


关键注意事项：


| 字段                   | 正确语义                             |
| -------------------- | -------------------------------- |
| `IMU.Imu.gaussX`     | Northing，对应 `ego_state.pose.y_m` |
| `IMU.Imu.gaussY`     | Easting，对应 `ego_state.pose.x_m`  |
| `ChassisInfo.mode`   | 底盘/控制模式，不是档位                     |
| `Prediction__Object` | 预测轨迹结构，不应用作当前障碍物状态               |
| `perception.Object`  | 当前障碍物状态，应该用于 `object_list`       |


### 6.3 控制输出语义

`CARLALongitudinalCmd` 必须同时支持两种纵向输入：


| 输入                                   | 使用场景         | Bridge 行为                             |
| ------------------------------------ | ------------ | ------------------------------------- |
| `speed` / `target_speed_mps`         | ACC、速度规划     | Bridge 内部用速度 PID 或 CARLA Ackermann 控制 |
| `acceleration` / `target_accel_mps2` | 纵向控制器直接输出加速度 | Bridge 转为 throttle/brake              |


ACC 最小闭环只要求输出 `target_speed_mps`。

## 7. 工程落盘格式需要保存的内容

GAASD 工程保存时，建议新增 `carla` 或 `simulation` 配置节点，至少包含：

```json
{
  "simulator": {
    "type": "carla",
    "version": "0.9.15",
    "launchMode": "local_script",
    "carlaRoot": "/home/aiden/snap/code/app/carla-package",
    "startScript": "/home/aiden/snap/code/app/carla-package/start-carla.sh",
    "scriptWorkingDirectory": "/home/aiden/snap/code/app/carla-package",
    "remoteSsh": {
      "sshHost": "",
      "sshPort": 22,
      "sshUser": "",
      "sshAuthType": "password_or_key",
      "remoteCarlaRoot": "/home/aicc/CARLA_0.9.15",
      "remoteStartScript": "./CarlaUE4.sh",
      "remoteWorkingDirectory": "/home/aicc/CARLA_0.9.15",
      "useSshTunnel": false,
      "tunnelLocalPort": 2000
    },
    "host": "127.0.0.1",
    "port": 2000,
    "syncMode": true,
    "fixedDeltaSeconds": 0.05,
    "mapName": "Town04"
  },
  "bridge": {
    "protocol": "gaasd_carla_bridge",
    "protocolVersion": "v0.3",
    "inputEndpoint": "tcp://127.0.0.1:5701",
    "outputEndpoint": "tcp://127.0.0.1:5702",
    "serviceEndpoint": "tcp://127.0.0.1:5703"
  }
}
```

落盘要求：


| 要求          | 说明                                            |
| ----------- | --------------------------------------------- |
| 不硬编码用户路径    | 路径必须从工程配置或环境变量读取                              |
| 保存启动模式      | 必须保存 `local_script`、`remote_ssh` 或 `external` |
| 保存 SSH 配置   | 服务器模式必须保存主机、端口、用户、认证方式、远程路径、工作目录和脚本           |
| 保存协议版本      | 便于后续协议升级时兼容                                   |
| 保存端口和 topic | 重新打开工程后能恢复 Bridge 连接                          |
| 保存组件端口映射    | 代码生成时不丢端口名和单位                                 |
| 保存场景配置      | 地图、自车生成点、NPC 数量、同步模式等应可复现                     |


## 8. 验收标准

第一阶段只验收最小闭环，不要求相机、激光雷达、Pangu 容器、leaderboard。本机模式和服务器 SSH 模式可以分阶段验收，但软件结构必须同时支持两种模式。

验收步骤：

1. GAASD 仿真管理界面选择 `carla_0.9.15`。
2. 选择 `local_script` 时，GAASD 通过配置调用 `/home/aiden/snap/code/app/carla-package/start-carla.sh`。
3. 选择 `remote_ssh` 时，GAASD 通过 SSH 在服务器执行远程 CARLA 启动命令，并保留服务器日志输出。
4. 启动 `carla_bridge.py`，连接配置中的 `carlaHost:carlaPort`；本机默认 `127.0.0.1:2000`，服务器模式使用服务器地址或 SSH 隧道地址。
5. Bridge 发布 `sim_clock`、`ego_state`、`object_list`、`lead_vehicle`。
6. GAASD 画布读取 `egoV`、`leadV`、`distance`。
7. GAASD 调用 `accComputeTargetSpeed` 计算 `targetSpeed`。
8. GAASD 通过 `CARLALongitudinalCmd.speed` 发布 `target_speed_mps`。
9. Bridge 将目标速度应用到 CARLA ego 车。
10. `chassis_feedback` 回传当前速度。
11. 画布中能观察到速度闭环收敛。

通过条件：


| 条件        | 标准                                                 |
| --------- | -------------------------------------------------- |
| CARLA 启动  | 页面能按 `local_script` 或 `remote_ssh` 启动 CARLA 0.9.15 |
| Bridge 连接 | `bridge_status.connected = true`                   |
| 输入组件      | `CARLAEgoState`、`CARLALeadVehicle` 有实时输出           |
| 输出组件      | `CARLALongitudinalCmd` 能发布目标速度                     |
| 控制闭环      | ego 车速度随目标速度变化，并有 `chassis_feedback`               |
| 工程复现      | 保存工程后重新打开，配置和连线不丢失                                 |


## 9. 不应采用的实现方式


| 不建议做法                                            | 原因                                |
| ------------------------------------------------ | --------------------------------- |
| 只保留硬编码 `/home/aicc/CARLA_${version}/CarlaUE4.sh` | 无法适配不同服务器路径和本地脚本                  |
| 删除或绕开现有 SSH 接口                                   | 服务器 CARLA 是合理部署方式，SSH 模式需要保留并补齐配置 |
| 本地 CARLA 模式仍强依赖 Pangu/leaderboard 容器             | 增加不必要依赖，阻塞最小闭环                    |
| 让算法直接使用 CARLA 原始坐标                               | CARLA 是 UE4 左手系，GAASD 使用 ENU 右手系  |
| 将 `Prediction__Object` 当作当前障碍物状态                 | 预测结构体和感知当前状态语义不同                  |
| 将 `ChassisInfo.mode` 当作 gear                     | `mode` 是控制模式，gear 只是可选调试字段        |
| 把 `vehicle.get_speed_limit()` 当成 m/s             | CARLA 0.9.15 返回 km/h，协议字段需转换为 m/s |


## 10. 需要 GAASD 团队确认的事项


| 事项               | 当前建议                                        |
| ---------------- | ------------------------------------------- |
| 标准传输格式           | JSON over ZMQ PUB/SUB                       |
| 标准端口             | `5701` 输入、`5702` 输出、`5703` 可选服务             |
| 组件端口名            | 使用本文第 5 节端口名                                |
| object 数组落盘方式    | 可用数组端口或拆分端口，但代码生成需稳定                        |
| route 表达方式       | 支持 waypoint 数组或 `GaussRoadPoint[]`          |
| 内部是否继续转 Protobuf | GAASD 内部决定，但外部 Bridge 契约按 JSON 协议冻结         |
| CARLA 启动模式 UI    | 同时支持 `local_script`、`remote_ssh`、`external` |
| SSH 远程配置         | 主机、端口、用户、认证方式、远程路径、工作目录、日志、隧道策略             |
| 工程配置格式           | 由 GAASD 团队确定最终 schema，但必须保存第 7 节字段          |


