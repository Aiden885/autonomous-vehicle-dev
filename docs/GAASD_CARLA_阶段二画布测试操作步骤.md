# GAASD-CARLA 阶段二画布测试操作步骤

本文档用于下一轮 GAASD 页面联调。命令行侧已经准备好 ACC 最小闭环组件包，页面侧需要完成组件导入、画布搭建和启动验证。

## 1. 已准备内容

| 内容 | 位置 |
|---|---|
| ACC 最小闭环组件源码 | `tools/carla_bridge/gaasd_p0_acc_min_components_src/carlaAccMinComponents.c` |
| ACC 最小闭环扫描组件包 | `tools/carla_bridge/gaasd_p0_acc_min_components/` |
| ACC 最小闭环组件包压缩文件 | `/data/aiden/文档/Modularization/tools/carla_bridge/gaasd_carla_p0_acc_min_components.tar.gz` |
| CARLA 完整 P0 扫描组件包 | `tools/carla_bridge/gaasd_p0_components/` |
| CARLA 适配动态库源码 | `tools/carla_bridge/adapter/` |

当前阶段优先使用 ACC 最小闭环组件，避免 GAASD 当前代码生成器对多输出组件的未连线端口传参不完整问题。

ACC 最小闭环组件包括：

| 组件 | 用途 |
|---|---|
| `CARLAACCEgoSpeed` | 输出自车纵向速度 `egoV` |
| `CARLAACCLeadSpeed` | 输出前车纵向速度 `leadV` |
| `CARLAACCLeadDistance` | 输出自车到前车净距离 `distance` |
| `CARLAACCComputeTargetSpeed` | 计算 ACC 目标速度 `targetSpeed` |
| `CARLAACCLongitudinalCmd` | 接收目标速度和使能信号，发布纵向控制命令 |

## 2. GAASD 页面操作

1. 打开 GAASD 的组件导入入口。
2. 如果入口是“组件包导入”，选择 `/data/aiden/文档/Modularization/tools/carla_bridge/gaasd_p0_acc_min_components/`，或选择 `/data/aiden/文档/Modularization/tools/carla_bridge/gaasd_carla_p0_acc_min_components.tar.gz`。
   如果导入窗口不显示 `.tar.gz` 文件，直接选择 `gaasd_p0_acc_min_components/` 目录。
3. 如果入口是“C 代码扫描”，不要选择 `gaasd_p0_acc_min_components/`，应选择源码目录 `/data/aiden/文档/Modularization/tools/carla_bridge/gaasd_p0_acc_min_components_src/`。
   `gaasd_p0_acc_min_components/` 是已经扫描后的组件包，里面主要是 JSON 和 `cbdes.db`；再次用代码扫描处理它不会生成组件。
4. 在组件库中确认能看到 `功能模块库 / CARLA联合仿真 / ACC最小闭环` 分类。
5. 确认能看到 `CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCComputeTargetSpeed`、`CARLAACCLongitudinalCmd`。
6. 新建或打开 ACC 测试画布。
7. 放置 `CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCComputeTargetSpeed`、`CARLAACCLongitudinalCmd`。
8. 按下表完成 ACC 控制闭环必需连线。

| 来源端口 | 目标端口 | 说明 |
|---|---|
| `CARLAACCEgoSpeed.egoV` | `CARLAACCComputeTargetSpeed.egoV` | 自车速度输入 |
| `CARLAACCLeadSpeed.leadV` | `CARLAACCComputeTargetSpeed.leadV` | 前车速度输入 |
| `CARLAACCLeadDistance.distance` | `CARLAACCComputeTargetSpeed.distance` | 前车净距离输入 |
| `CARLAACCComputeTargetSpeed.targetSpeed` | `CARLAACCLongitudinalCmd.speed` | ACC 目标速度输出 |
| 常量 `1` | `CARLAACCLongitudinalCmd.enable` | 第一轮联通测试固定使能 |

这套组件没有多余输出端口，生成代码时不需要为未连线输出端口补临时变量。

## 3. 启动前检查

当前 GAASD 页面还没有把 CARLA 和 Bridge 启动动作接入仿真管理。阶段二手动联调时，先在项目根目录执行：

```bash
tools/carla_bridge/start-gaasd-carla-manual.sh --no-probe
```

脚本会启动本机 CARLA、启动 Bridge、等待 `5701/5702` 端口、重置到 Town01 长直道测试场景、生成测试前车，并打开默认测试视角。脚本结束并提示 `manual stack is ready` 后，再回到 GAASD 页面点击示波器“开始”。

默认测试视角固定为第三人称跟随视角：`back=8m`、`up=6m`、`pitch=-25deg`。该视角能在 CARLA 页面中看到自车和前方道路，便于观察 ACC 跟车效果。如需临时调整，可追加 `--spectator-back`、`--spectator-up`、`--spectator-pitch` 参数；如需关闭跟随视角，可追加 `--no-follow-spectator`。

当前脚本默认使用 `ego_forward + traffic_manager` 生成前车：前车会放在 ego 正前方，并由 CARLA Traffic Manager 管理，禁止变道、忽略红绿灯。该组合已在当前地图出生点下验证，Bridge 可持续识别有效前车。需要尝试路网 waypoint 放置时，可手动使用：

```bash
tools/carla_bridge/start-gaasd-carla-manual.sh --lead-placement lane_waypoint --lead-behavior traffic_manager
```

如果脚本没有输出 `manual stack is ready`，不要继续启动 GAASD 仿真。仅看到 CARLA 页面或 `2000` 端口可连接还不够，Bridge 还需要等待 CARLA Python API 可用并成功发布消息。

阶段二手动脚本默认使用 `tools/carla_bridge/config.phase2.json`。该配置把控制命令超时时间设置为 `1.0s`，用于兼容当前手动联调阶段的 GAASD 示波器控制输出节奏。已验证 `0.1s` 周期可以稳定驱动车辆；如果周期过长，Bridge 默认 `0.2s` 超时会在两次 GAASD 控制命令之间进入制动状态，导致自车不动。

重复测试或结束测试时，先停止 GAASD 示波器，再执行：

```bash
tools/carla_bridge/stop-gaasd-carla-manual.sh
```

启动仿真前确认：

| 检查项 | 期望结果 |
|---|---|
| CARLA 仿真器 | 已启动，端口为 `2000` |
| Bridge | 已启动并连接 CARLA |
| 前车场景 | 已生成前车，或场景配置中会生成前车 |
| 适配库 | 生成工程中存在 `objectCode/total/DLL/libcarla_gaasd_adapter.so` |
| 代码生成输入 | `icvos/blocks/functions/c_process_*.json` 中包含 CARLA 和 ACC 子组件 |

如果 GAASD 当前页面不会自动放置适配库，需要在生成工程后确认该 `.so` 已存在。缺少该库时，车辆通常不会响应控制命令。

如果画布中已经能看到组件和连线，但 `c_process_*.json` 的 `childComponents` 仍为空，说明画布数据库尚未同步到代码生成 JSON。此时需要在 GAASD 页面中执行保存/生成代码/导出工程一类操作，直到 `icvos/blocks/functions/` 下的过程 JSON 包含实际画布节点后再启动仿真。

## 4. 验收现象

阶段二画布闭环通过时，应看到：

| 信号 | 期望现象 |
|---|---|
| `CARLAACCEgoSpeed.egoV` | 持续刷新，自车运动后数值变化 |
| `CARLAACCLeadDistance.distance` | 持续刷新，前车有效时为正距离 |
| `CARLAACCComputeTargetSpeed.targetSpeed` | 根据距离和速度差变化 |
| CARLA 自车 | 对目标速度有响应 |

如果组件能导入和连线，但车辆不动，优先检查 Bridge、适配库和 `enable` 是否正确。

## 5. 当前验证结果

截至 2026-05-09，阶段二 P0 方案已经完成一次 GAASD 画布真实闭环验证。

| 项目 | 结果 |
|---|---|
| 使用组件 | `CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCComputeTargetSpeed`、`CARLAACCLongitudinalCmd` |
| Bridge 配置 | `tools/carla_bridge/config.phase2.json` |
| GAASD 示波器周期 | `0.1s` |
| 仿真时长 | `30s` |
| 最新有效日志 | `/home/aiden/gaasd_server/codeTools/dist/logs/run_simulation_20260511_114220.log` |
| 采样点数 | `601` |
| Bridge 控制计数 | 约 `601` 条控制命令 |
| 自车速度 | `egoV` 从 `0` 上升到最高约 `1.86m/s`，末值约 `0.046m/s` |
| 前车距离 | `distance` 从约 `20.21m` 收敛到约 `15.38m`，未再跳到 `1000000` |
| 目标速度 | `targetSpeed` 从约 `2.60m/s` 下降到约 `0.17m/s` |

该结果说明以下链路已经成立：

```text
CARLA 状态 -> Bridge -> GAASD 组件 -> GAASD 控制输出 -> Bridge -> CARLA 自车
```

注意：仿真结束后 Bridge 会因为没有新的控制命令而进入超时制动，此时实时探测可能看到 `brake=1.0`、`mode=0`、`ego_speed=0`。这属于仿真停止后的安全状态，不代表运行过程中自车没有响应。

2026-05-11 动态前车 60 秒测试补充：

| 项目 | 结果 |
|---|---|
| 最新有效日志 | `/home/aiden/gaasd_server/codeTools/dist/logs/run_simulation_20260511_160037.log` |
| 前车速度 | `leadV` 稳定在约 `1.955m/s` |
| 自车速度 | `egoV` 从 `0` 上升到约 `2.306m/s` |
| 目标速度 | `targetSpeed` 全程为 `3.333m/s`，受旧版 `12km/h` 上限限制 |
| 前车距离 | `distance` 从约 `51.70m` 降到约 `37.04m` |
| 结论 | 动态前车链路已连通，但起始距离过大且旧速度上限偏低，不适合继续靠长时间跑到目标车距 |

后续短直路测试改用快速收敛参数：`desiredDist=15m` 不变，`maxSpeed=18km/h`，`kDist=0.35`，`kSpeed=0.8`。Bridge 纵向控制改为速度 PID 映射，阶段二默认参数为 `speed_kp=0.7`、`speed_ki=0.05`、`speed_kd=0.08`，并限制 `max_throttle=0.7`、`max_brake=0.35`。测试前应尽量在重置场景后立即启动 GAASD 仿真，避免前车先行导致起始距离被拉大。

## 6. 后续计划

后续继续沿用当前 P0 C 包装组件方案，不切换到仅支持显示和连线的 `CARLA-json.zip` 组件。

| 优先级 | 工作 | 目标 |
|---|---|---|
| P0 | 固定前车行为和位置 | 已通过，Bridge 持续选中 `role_name=gaasd_lead` 前车 |
| P0 | 重跑 30 秒 ACC 跟车测试 | 已通过，`distance` 全程有限，`targetSpeed` 随距离下降 |
| P0 | 补跑 60 秒稳定性测试 | 已通过，601 个采样点无缺失、无 `1000000` 距离跳变 |
| P1 | 整理 P0 验收记录 | 固化启动命令、示波器周期、日志路径、验收指标 |
| P1 | 动态前车场景测试 | 使用 `tools/carla_bridge/start-acc-dynamic-test.sh` 尝试前车低速行驶场景 |
| P1 | 与 GAASD 软件侧对齐正式组件方案 | 明确 `carla-sim-input/output` 未来需要补运行时 topic 订阅、字段映射和控制发布能力 |
| P2 | 扩展到更完整 CARLA 组件 | 在 P0 稳定后再考虑 `object_list`、`route`、`traffic_light_list`、`trajectory_cmd` |

当前进展：已把前车默认生成策略改为 `ego_forward + traffic_manager`，并在 Bridge 中优先识别 `role_name=gaasd_lead` 的测试前车。命令行侧 20 秒短测结果为 `lead_vehicle.valid=399/399`、`selection_rule=preferred_role_nearest_front`、`clearance_m≈20.21`、`lateral_distance≈0`。GAASD 30 秒和 60 秒静止前车测试均已通过。下一步进入动态前车场景测试。
