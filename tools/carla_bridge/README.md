# CARLA Bridge 工具

这个目录是 Bridge / CARLA 联调侧的第一版可运行骨架，用来先把本机或服务器 CARLA 接入 GAASD 标准协议。

## 文件

| 文件 | 作用 |
|---|---|
| `config.example.json` | Bridge 配置样例 |
| `carla_bridge.py` | CARLA 到 GAASD 的 ZMQ JSON Bridge |
| `start-carla.sh` | 本机 CARLA 启动包装脚本 |
| `stop-carla.sh` | 只停止由本包装脚本后台启动的 CARLA |
| `run-bridge.sh` | 用兼容 CARLA egg 的 Python 启动 Bridge |
| `start-bridge.sh` | 后台启动 Bridge，写入 PID 和日志 |
| `stop-bridge.sh` | 停止后台启动的 Bridge |
| `start-gaasd-carla-manual.sh` | 阶段二联调用的一键启动脚本：CARLA + Bridge + 测试前车 |
| `stop-gaasd-carla-manual.sh` | 停止手动联调栈：Bridge + CARLA + 残留仿真进程/容器 |
| `health-carla.sh` | 通过 CARLA Python API 检查服务是否可用，避免裸连 RPC 端口 |
| `probe-pub.py` | 订阅 Bridge PUB 端口，检查协议消息是否发出 |
| `send-control.py` | 向 Bridge 发送一组测试控制命令 |
| `acc-runner.py` | 阶段一独立 ACC 闭环测试 Runner |
| `spawn-lead-vehicle.py` | 在 ego 前方生成一辆测试前车 |
| `watch-carla.py` | CARLA 摄像头观察窗口，并向 Bridge 发送 ACC 键盘驾驶指令 |
| `gaasd_p0_acc_min_components/` | ACC 最小闭环组件包，优先用于当前已跑通的 ACC 测试 |
| `gaasd_p1_components/` | P1 扩展组件包，提供 `ObjectList`、横向控制和横纵向联合控制 |
| `gaasd_carla_p1_components.tar.gz` | P1 组件包压缩文件，可用于 GAASD 导入 |
| `gaasd_lks_components/` | LKS 单车闭环组件包，提供车道偏差输入和联合控制输出 |
| `gaasd_carla_lks_components.tar.gz` | LKS 组件包压缩文件，可用于 GAASD 导入 |
| `reset-lks-straight-scene.py` | 设置单车车道保持测试的初始横向/航向偏差 |
| `build-adapter-ubuntu-env.sh` | 在 GAASD 运行镜像中构建并验证 adapter 动态库 |
| `adapter/` | `libcarla_gaasd_adapter.so` 源码，封装 GAASD C ABI 到 ZMQ JSON Bridge |

## 环境要求

本机 CARLA 0.9.15 包只提供 `py3.7` egg。当前 `python3` 是 3.12 时不要直接加载 CARLA egg，
会有段错误风险；本机验证可用的是 `python3.8`。

```bash
python3.8 -m pip install --user pyzmq
```

`carla` Python API 需要从 CARLA 包中加载。默认配置会尝试使用：

```text
/home/aiden/snap/code/app/carla-package/PythonAPI/carla
```

如果 `python3.8 -c 'import carla'` 失败，可以在 `config.example.json` 的 `carla.python_api_paths` 中补充 CARLA egg 或 PythonAPI 路径。

## 启动本机 CARLA

前台启动：

```bash
./tools/carla_bridge/start-carla.sh
```

后台启动：

```bash
./tools/carla_bridge/start-carla.sh --background
```

健康检查：

```bash
./tools/carla_bridge/health-carla.sh --host 127.0.0.1 --port 2000
```

停止后台启动的 CARLA：

```bash
./tools/carla_bridge/stop-carla.sh
```

## 启动 Bridge

```bash
tools/carla_bridge/run-bridge.sh
```

后台启动：

```bash
tools/carla_bridge/start-bridge.sh
```

停止后台 Bridge：

```bash
tools/carla_bridge/stop-bridge.sh
```

如果暂时没有 CARLA Python API，只检查 ZMQ 和配置入口：

```bash
tools/carla_bridge/run-bridge.sh --dry-run
```

检查 Bridge 是否正在发布协议消息：

```bash
python3.8 tools/carla_bridge/probe-pub.py --duration 5 --min-messages 3
```

发送一组测试控制命令：

```bash
python3.8 tools/carla_bridge/send-control.py --target-speed-mps 2.0 --repeat 20
```

阶段一 ACC Runner：

```bash
python3.8 tools/carla_bridge/acc-runner.py --duration-sec 20 --cruise-speed-mps 5 --min-distance-m 8 --time-gap-sec 1.5
```

生成测试前车：

```bash
python3.8 tools/carla_bridge/spawn-lead-vehicle.py --replace --distance-m 25 --speed-mps 2
```

默认前车放置方式是 `--placement ego_forward --behavior traffic_manager`，用于让前车稳定出现在 ego 正前方。需要尝试 CARLA 路网 waypoint 放置时，可使用 `--placement lane_waypoint --behavior traffic_manager`。

## 阶段二手动联调一键启动

```bash
tools/carla_bridge/start-gaasd-carla-manual.sh --no-probe
```

该脚本会依次后台启动本机 CARLA、启动 Bridge、等待 `5701/5702` 端口、重置到 Town01 长直道测试场景、生成测试前车，并打开默认第三人称跟随视角。脚本完成后，再到 GAASD 页面点击示波器“开始”。

默认测试视角为 `--spectator-back 8 --spectator-up 6 --spectator-pitch -25`。需要关闭跟随视角时追加 `--no-follow-spectator`。
需要同时打开带键盘输入的摄像头窗口时追加 `--watch-camera`。

常用参数：

```bash
tools/carla_bridge/start-gaasd-carla-manual.sh --lead-distance 25 --lead-speed 2
tools/carla_bridge/start-gaasd-carla-manual.sh --lead-placement lane_waypoint --lead-behavior traffic_manager
tools/carla_bridge/start-gaasd-carla-manual.sh --lead-placement ego_forward --lead-behavior constant_velocity
tools/carla_bridge/start-gaasd-carla-manual.sh --spectator-back 8 --spectator-up 6 --spectator-pitch -25
tools/carla_bridge/start-gaasd-carla-manual.sh --watch-camera
tools/carla_bridge/start-gaasd-carla-manual.sh --no-lead
tools/carla_bridge/start-gaasd-carla-manual.sh --high
```

摄像头窗口键盘映射：

| 按键 | `commandType` | 发送方式 |
|---|---:|---|
| E | 1 | 单周期脉冲：降低设定速度或按当前速度启控 |
| Q | 2 | 单周期脉冲：提高设定速度或继承参数启控 |
| T | 3 | 单周期脉冲：减小时距 |
| R | 4 | 单周期脉冲：增大时距 |
| W | 5 | 按下期间持续发送驾驶员油门指令 |
| S | 6 | 按下期间持续发送驾驶员制动指令，并使 ACC 退出控制 |
| C | 7 | 单周期脉冲：取消 ACC |

W 当前只向 GAASD 传输驾驶员油门指令类型，尚未实现驾驶员扭矩与 ACC 扭矩的物理仲裁。S 不走扭矩仲裁：`commandType=6` 由 ACC 决策画布直接清除控制使能，使 Bridge 切换到制动状态。

动态前车测试入口：

```bash
tools/carla_bridge/start-acc-dynamic-test.sh
```

该脚本复用 `start-gaasd-carla-manual.sh`，默认使用 `config.phase2.json`，并以 `lane_waypoint + traffic_manager` 方式尝试生成低速行驶前车。若当前地图或 spawn 点下 waypoint 放置不合适，可回退为：

```bash
LEAD_PLACEMENT=ego_forward LEAD_BEHAVIOR=traffic_manager tools/carla_bridge/start-acc-dynamic-test.sh
```

停止手动联调栈：

```bash
tools/carla_bridge/stop-gaasd-carla-manual.sh
```

如果只想保留正在运行的 GAASD 仿真进程/容器，仅停止 Bridge 和 CARLA：

```bash
tools/carla_bridge/stop-gaasd-carla-manual.sh --keep-gaasd-sim
```

## 当前支持的 topic

Bridge 发布：

| topic | 说明 |
|---|---|
| `gaasd.carla.bridge_status.v1` | Bridge 状态 |
| `gaasd.carla.sim_clock.v1` | 仿真时钟 |
| `gaasd.carla.ego_state.v1` | 自车状态 |
| `gaasd.carla.object_list.v1` | 障碍物列表 |
| `gaasd.carla.lead_vehicle.v1` | ACC 前车便捷消息 |
| `gaasd.carla.chassis_feedback.v1` | 底盘反馈 |
| `gaasd.carla.lane_tracking.v1` | LKS 车道偏差与航向误差 |
| `gaasd.carla.driver_command.v1` | 转发给 GAASD 的 ACC 驾驶指令 |

Bridge 订阅：

| topic | 说明 |
|---|---|
| `gaasd.carla.control_cmd.v1` | GAASD 控制输出 |
| `gaasd.carla.driver_command.v1` | 摄像头窗口或其他输入源发送的 ACC 驾驶指令 |

## 注意

- 这是第一版最小联调骨架，不包含完整场景管理 UI。
- 同步模式默认启用 `carla.realtime_pacing=true`，Bridge 会按 `fixed_delta_seconds` 控制 tick 频率，避免仿真时间快于控制器真实发送频率。
- `target_speed_mps` 会先经过 Bridge 速度 PID 转为期望加速度，再按 `max_accel_mps2` / `max_brake_mps2` 映射为 CARLA `throttle` / `brake`。
- 控制命令超时后默认执行 `control.timeout_brake=1.0`，Bridge 会主动下发 `throttle=0, brake=1`。
- `CARLALongitudinalCmd`、`CARLALateralCmd`、`CARLAControlCmd` 最终都映射到 `control_cmd`，同一时刻只能有一个控制输出组件 active。
- `control.lane_keep_mode=lookahead_pid` 时，Bridge 使用当前车道偏差和动态预瞄偏差的加权值进行 PID 横向控制；该模式用于 ACC 联调时的基础车道保持。
- `vehicle.get_speed_limit()` 在 CARLA 0.9.15 中返回 km/h，进入协议前必须除以 3.6 转成 m/s。
