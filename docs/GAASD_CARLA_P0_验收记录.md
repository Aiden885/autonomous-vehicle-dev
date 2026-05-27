# GAASD-CARLA P0 验收记录

## 1. 概述

| 项目 | 内容 |
|---|---|
| 验收日期 | 2026-05-09 |
| 验收版本 | P0（静止前车 ACC 纵向闭环） |
| 仿真方案 | CARLA + ZMQ Bridge + libcarla_gaasd_adapter.so + GAASD 画布 |
| 结论 | **通过** |

## 2. 系统配置

### 2.1 启动命令

**步骤一：启动 CARLA**

```bash
cd /home/aiden/CARLA_0.9.15
./CarlaUE4.sh -prefernvidia -RenderOffScreen
```

**步骤二：启动 Bridge**

```bash
cd /data/aiden/文档/Modularization/tools/carla_bridge
python3 carla_bridge.py --config config.phase2.json
```

**步骤三：GAASD 启动仿真**

在 GAASD 画布（项目：carlatest）点击"开始仿真"，仿真时长在 GAASD UI 中配置。

### 2.2 关键配置参数

| 参数 | 值 |
|---|---|
| CARLA 地址 | `127.0.0.1:2000` |
| 同步模式 | `sync_mode=true` |
| 仿真步长 | `fixed_delta_seconds=0.05s` |
| Bridge 推送端口 | `tcp://127.0.0.1:5701` |
| Bridge 接收端口 | `tcp://127.0.0.1:5702` |
| 前车筛选规则 | `preferred_role_name=gaasd_lead`，横向阈值 12m |
| 适配库路径 | `project/carla/objectCode/total/DLL/libcarla_gaasd_adapter.so` |
| GAASD 项目 | `project/carla`（项目名：carlatest） |
| 示波器步长 | 0.1s |

### 2.3 画布组件与连线

| 组件 | 作用 |
|---|---|
| `CARLAACCEgoSpeed` | 读取自车速度（调用 `carla_adapter_read_ego_state`） |
| `CARLAACCLeadSpeed` | 读取前车速度（调用 `carla_adapter_read_lead_vehicle`） |
| `CARLAACCLeadDistance` | 读取前车距离（调用 `carla_adapter_read_lead_vehicle`） |
| `CARLAACCComputeTargetSpeed` | ACC 目标速度计算 |
| `CARLAACCLongitudinalCmd` | 发布纵向控制命令（调用 `carla_adapter_publish_longitudinal_cmd`） |

连线：

```
CARLAACCEgoSpeed → CARLAACCComputeTargetSpeed.egoV
CARLAACCLeadSpeed → CARLAACCComputeTargetSpeed.leadV
CARLAACCLeadDistance → CARLAACCComputeTargetSpeed.distance
CARLAACCComputeTargetSpeed.targetSpeed → CARLAACCLongitudinalCmd.speed
常量 1 → CARLAACCLongitudinalCmd.enable
```

示波器观测：`targetSpeed`、`egoV`、`leadV`、`distance`

## 3. 测试场景

| 参数 | 值 |
|---|---|
| 场景描述 | 静止前车 ACC 纵向跟车 |
| 前车初始距离 | 约 20.2m |
| 前车速度 | 0 m/s（静止） |
| 自车初始速度 | 0 m/s |
| ACC 期望跟车距离 | 15m（`desiredDist=15.0`） |
| ACC 最大速度 | 12/3.6 = 3.33 m/s |

## 4. 测试结果

### 4.1 30 秒闭环测试

| 指标 | 结果 |
|---|---|
| 日志文件 | `run_simulation_20260509_112022.log` |
| 采样点数 | 301（0.0s ~ 30.0s，无缺失） |
| distance 异常点（>999999） | 0 |
| egoV 峰值 | 约 1.90 m/s |
| distance 收敛过程 | 20.21m → 15.64m |
| targetSpeed 变化 | 2.60 m/s → 0.29 m/s |
| 结论 | **通过** |

关键时刻数据：

| 时刻 | egoV (m/s) | distance (m) | targetSpeed (m/s) |
|---|---|---|---|
| t=0s | 0.0000 | 20.208 | 2.6041 |
| t=10s | 0.0735 | 16.171 | 0.5488 |
| t=20s | 0.0472 | 15.805 | 0.3788 |
| t=30s | 0.0352 | 15.618 | 0.2914 |

### 4.2 60 秒稳定性测试

| 指标 | 结果 |
|---|---|
| 日志文件 | `run_simulation_20260509_114228.log` |
| 采样点数 | 601（0.0s ~ 60.0s，无缺失） |
| distance 异常点（>999999） | 0 |
| ZMQ 断线 / Bridge 重连 | 无 |
| 后 30s egoV 范围 | 0.0240 ~ 0.0352 m/s（无振荡） |
| 后 30s distance 范围 | 15.433 ~ 15.618 m（单调收敛） |
| 结论 | **通过** |

关键时刻数据：

| 时刻 | egoV (m/s) | distance (m) | targetSpeed (m/s) |
|---|---|---|---|
| t=0s | 0.0000 | 20.208 | 2.6041 |
| t=10s | 0.0735 | 16.171 | 0.5488 |
| t=20s | 0.0472 | 15.805 | 0.3788 |
| t=30s | 0.0352 | 15.618 | 0.2914 |
| t=40s | 0.0291 | 15.518 | 0.2444 |
| t=50s | 0.0259 | 15.463 | 0.2187 |
| t=60s | 0.0240 | 15.433 | 0.2045 |

## 5. 验收现象确认

| 验收项 | 期望 | 实际 | 是否通过 |
|---|---|---|---|
| CARLA → Bridge → GAASD 数据链路 | `egoV` 持续刷新 | ✅ 全程有效 | ✅ |
| 前车有效 | `distance` 非 1000000 | ✅ 全程正常，无跳变 | ✅ |
| ACC 算法响应 | `targetSpeed` 随距离变化 | ✅ 随距离收敛同步下降 | ✅ |
| GAASD → Bridge → CARLA 控制链路 | `egoV` 对 `targetSpeed` 有响应 | ✅ 自车从 0 加速至 1.9 m/s 后减速 | ✅ |
| 60s 全程连接稳定 | 无断线、无异常跳变 | ✅ | ✅ |

## 6. 已知限制

| 项目 | 说明 |
|---|---|
| 前车为静止目标 | P0 场景前车 `leadV=0`，未验证动态跟车 |
| 收敛距离 | 因 ACC 控制参数（`kDist=0.5`，`kSpeed=0.5`），60s 内未完全到达 15m，末值约 15.43m，属正常 |
| 横向控制 | P0 未接入，仅验证纵向闭环 |
| 适配库为本地构建 | `libcarla_gaasd_adapter.so` 在当前机器编译，换机器部署需重新构建 |

## 7. 后续计划

1. 动态前车场景测试：前车以 2~5 m/s 匀速行驶，验证 distance 收敛与 targetSpeed 跟随
2. 与 GAASD 软件侧确认 `carla-sim-input/output` 组件运行时能力，评估是否切换至正式组件接口
3. 适配库构建流程固化，补充 Docker 容器内 ZMQ 依赖确认步骤
