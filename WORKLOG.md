# 工作日志 WORKLOG

> **用途**：记录每次工作的修改内容、遗留问题和下一步任务。
> 新开会话或交给 Codex 前，先读这个文件。
> 每次工作结束后，更新"当前状态"和"下一步任务"两节。

---

## 当前状态（2026-05-18 更新）

**当前工作目标**：保留已跑通的 ACC 跟车 CARLA-GAASD 联合仿真快照，保证后续可通过本机 UI 快速恢复和演示。

**已确认状态**：
- `http://127.0.0.1:8765/` 本机场景启动面板可直接启动 `ACC 跟车 CARLA-GAASD 联合仿真快照`。
- 点击“启动环境”后，CARLA + Bridge 可启动，CARLA 视角自动进入已确认的跟随视角。
- 前车和自车位置符合当前设计，可作为后续 GAASD 示波器运行 ACC 闭环的基准场景。
- 根目录 `tools/carla_bridge/` 与场景快照 `scenarios/acc_carla_phase2_20260513/bridge_snapshot/tools/carla_bridge/` 的关键启动逻辑已同步：
  - `reset-acc-straight-scene.py` 支持 `ego_spawn_index=198` 越界时使用 Bridge 已放置 ego 位置。
  - `set-spectator-follow.py` 后台跟随不再等待 CARLA tick。
  - `start-gaasd-carla-manual.sh` 后台跟随前先执行一次 `--once` 固定初始视角。
- UI 页面代码保存在 `tools/gaasd_scenario_panel/`。
- 可复现快照记录见 `docs/GAASD_CARLA_场景快照备份记录.md`。
- GAASD 重新生成代码后的 `carla.h` 兼容修复已标准化：`tools/carla_bridge/fix-gaasd-generated-code.py` 可修复 `protobuf-c` 头文件缺失和 `Infopack__TrafficLight__State` 枚举缺失问题，并已接入 UI 面板“修复生成代码”按钮。

**下一步任务**：
- 将场景快照、UI 面板、根目录 CARLA 工具脚本提交并推送到远端备份。
- 后续继续基于该快照做动态前车跟车调参和稳定性验证。

---

## 当前状态（2026-05-09 更新）

**当前工作目标**：GAASD 接入 CARLA，完成 ACC 最小闭环的阶段二联合仿真验证。

**当前路线**：继续采用 P0 C 包装组件方案。`CARLA-json.zip` 中的 `carla-sim-input/output` 组件目前只能用于 GAASD 组件显示和连线关系，不能替代当前已跑通的 C 运行组件；真实闭环仍由 `carla_adapter_*` 适配库和 ACC 最小闭环组件完成。

**已完成验证**：
- 阶段一 Python `acc-runner.py` 闭环通过：CARLA -> Bridge -> Python ACC Runner -> Bridge -> CARLA。
- 阶段一 60 秒加固测试通过：Bridge 约 20Hz 发布，`control_cmd_received=1201`，前车持续有效，超时制动正常。
- 阶段二 Docker 前置验证通过：`ubuntu:env` 已补齐 `libzmq3-dev` / `pkg-config`，GAASD `dist/CMakeLists.txt` 可链接外部 `.so`。
- 阶段二 P0 适配库测试通过：
  - `libcarla_gaasd_adapter.so` 可在 `ubuntu:env` 内编译。
  - C ABI 符号 `carla_adapter_read_*` / `carla_adapter_publish_longitudinal_cmd` 已导出。
  - smoke test 通过：无 Bridge 时接口安全返回。
  - mock ZMQ 测试通过：`ego_state` / `lead_vehicle` / `chassis_feedback` / `control_cmd` 字段映射正确。
  - live Bridge 短测通过：`ego_valid=202`、`lead_valid=202`、`chassis_valid=202`、`commands=122`，Bridge 侧 `control_cmd_received=122`。

**Claude 审核结论（2026-04-29）**：
- 认可“阶段二 P0 适配库通信链路已通过”，但边界是“适配库独立通信”，不是“GAASD 画布闭环已通过”。
- 必须修复/验证：
  - `libcarla_gaasd_adapter.so` 的 SONAME 部署风险。
  - `chassis_feedback` 中 `steer_rad` 与 Bridge 实际字段 `steering_angle_rad` 不匹配。
  - 用 `run_simulation.sh` 跑最小 `FuncStep.c`，验证 GAASD 生成工程真实加载并调用适配库。

**下一步任务（CARLA 阶段二）**：
- [x] 修复适配库 SONAME 和 `steering_angle_rad` 字段映射。
- [x] 重跑 `ubuntu:env` 内编译、smoke、mock 测试。
- [x] 准备最小 GAASD FuncStep 测试工程，放置 `objectCode/total/DLL/libcarla_gaasd_adapter.so`。
- [x] 用 `run_simulation.sh` 验证真实 GAASD 运行框架能加载并调用适配库。
- [x] 实现 `CARLAEgoState`、`CARLALeadVehicle`、`CARLALongitudinalCmd` 三个 P0 组件包装函数，并额外提供 `CARLAChassisFeedback` 观测组件。
- [x] 在 GAASD 页面导入 P0 ACC 最小闭环组件包，搭建 ACC 画布闭环并运行真实联调。
- [x] 固定前车场景，完成稳定跟车效果和 60 秒长跑稳定性验证。
- [ ] 动态前车场景测试：前车低速行驶，验证 `leadV` 非零时的跟车响应。

**2026-04-29 至 2026-05-09 后续测试结果**：
- 修复后 `readelf` 显示 `Library soname: [libcarla_gaasd_adapter.so]`。
- `adapter_smoke_test` 通过。
- `adapter_mock_loop_test` 通过，输出 `mock_loop_ok`。
- 最小 `run_simulation.sh` 无 Bridge 测试通过：执行 5 步 `FuncStep`，`egoValid=0`、`leadValid=0`，安全降级。
- 最小 `run_simulation.sh` + 真实 CARLA/Bridge 测试通过：执行 21 步 `FuncStep`，每步 `egoValid=1`、`leadValid=1`，Bridge 侧 `control_cmd_received=21`，`chassis_feedback.last_command_id=21`。
- P0 组件包已生成：`tools/carla_bridge/gaasd_p0_components/`，压缩包为 `tools/carla_bridge/gaasd_carla_p0_components.tar.gz`。
- `codescan` 生成 4 个组件 JSON：`CARLAEgoState`、`CARLALeadVehicle`、`CARLALongitudinalCmd`、`CARLAChassisFeedback`。
- P0 组件源码 `cc -std=c99 -Wall -Wextra -fPIC -c` 编译无警告。
- GAASD 页面已导入 CARLA 组件包，`/home/aiden/.gaasd/imported/component/` 中已有 4 个 CARLA 组件 JSON。
- `project/carla/data/temp.db` 中已保存 ACC 最小闭环画布：8 个节点、5 条必需连线，连线关系正确。
- 已在 `ubuntu:env` 内重新构建 `libcarla_gaasd_adapter.so`，并放置到 `project/carla/objectCode/total/DLL/`；SONAME 为 `libcarla_gaasd_adapter.so`，依赖 `libzmq.so.5` 正常。
- GAASD 页面重新生成代码后，`project/carla/icvos/blocks/functions/c_process_*.json` 已包含画布里的 CARLA/ACC 子组件，`project/carla/icvos/src/functions/` 已生成对应 C 源码。
- 当前新阻塞点：生成的 C 工程暂不能编译。
  - `carla.h` 引用了 `Infopack__TrafficLight__State`，但未生成该枚举定义，导致组件源码编译失败。
  - `main.c` 对多输出 CARLA 组件只传入已连线端口，例如 `CARLALeadVehicle(&leadV, &distance)`，但函数原型要求完整输出参数列表 `leadV/distance/relativeSpeed/ttc/valid`，导致参数数量不匹配。
  - 结论：画布保存与代码生成链路已前进一大步，但当前 P0 多输出组件设计与 GAASD 代码生成器行为不完全匹配。下一步建议拆分/裁剪为 ACC 最小闭环专用单输出或少输出组件，再重新导入并生成代码。
- 已新增 ACC 最小闭环组件包：
  - 源码：`tools/carla_bridge/gaasd_p0_acc_min_components_src/carlaAccMinComponents.c`
  - 扫描包：`tools/carla_bridge/gaasd_p0_acc_min_components/`
  - 压缩包：`tools/carla_bridge/gaasd_carla_p0_acc_min_components.tar.gz`
  - 组件：`CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCLongitudinalCmd`
  - 验证：组件源码 `cc -std=c99 -Wall -Wextra -fPIC -c` 通过；`codescan --custom` 生成 4 个组件 JSON 和 `cbdes.db`。
  - 下一步：在 GAASD 页面导入该最小组件包，替换原画布中的 `CARLAEgoState`、`CARLALeadVehicle`、`CARLALongitudinalCmd`，重新生成代码后再检查编译。
- GAASD 页面已用 ACC 最小闭环组件重新搭建并生成代码：
  - `project/carla/icvos/blocks/functions/` 中已替换为 `CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCLongitudinalCmd`。
  - `project/carla/icvos/src/functions/c_process_*/main.c` 调用关系正确：`egoV -> leadV -> distance -> accComputeTargetSpeed -> CARLAACCLongitudinalCmd`。
  - 之前的多输出组件参数不完整问题已消失。
  - 剩余生成问题：
    - `carla.h` 仍漏生成 `Infopack__TrafficLight__State` 枚举。
    - 当前 `accComputeTargetSpeed` 组件仍是旧三入参版本，函数体引用 `ACC_DESIRED_DIST` / `ACC_MAX_SPEED`，但生成头文件没有对应宏。
  - 已对当前生成工程临时补 `carla.h` 枚举和 ACC 宏后验证：组件 `.o`、`libcarla_static.a`、`libcarla_shared.so` 均可生成。
  - 当前独立 CMake 可执行文件链接失败，原因是生成 CMake 未链接 `objectCode/total/DLL/libcarla_gaasd_adapter.so`；该适配库已存在，真实仿真需通过 GAASD/codeTools 运行流程把 `PROJECT_EXTRA_LIBS` 带入。
  - `project/carla` 当前未生成 `FuncStep.c`，直接用 `run_simulation.sh` 还缺少运行入口；下一步需确认 GAASD 页面“启动仿真”是否会生成/调用 `FuncStep.c`，或需要我们手动提供一个 `FuncStep.c` 包装当前画布逻辑。
- 已从源头修复 ACC 宏依赖问题：
  - `tools/carla_bridge/gaasd_p0_acc_min_components_src/carlaAccMinComponents.c` 新增 `CARLAACCComputeTargetSpeed`。
  - 该组件直接在函数体内使用局部常量 `desiredDist=15.0`、`maxSpeed=12.0/3.6`、`kDist=0.5`、`kSpeed=0.5`，不依赖 `ACC_DESIRED_DIST` / `ACC_MAX_SPEED` 宏。
  - 已重新扫描生成 5 个 ACC 最小闭环组件并更新压缩包。
  - 下一次画布应使用 `CARLAACCComputeTargetSpeed` 替代旧 `accComputeTargetSpeed`。
- GAASD 页面已重新导入并使用 `CARLAACCComputeTargetSpeed` 生成代码：
  - `main.c` 调用关系正确：`CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCComputeTargetSpeed`、`CARLAACCLongitudinalCmd`。
  - 旧 `accComputeTargetSpeed` 和 `ACC_DESIRED_DIST` / `ACC_MAX_SPEED` 宏依赖已从生成工程中消失。
  - 直接编译仍先失败于 `carla.h` 漏生成 `Infopack__TrafficLight__State`；`enum.json` 中存在该枚举，确认是 GAASD 代码生成器未把 enum 写入头文件。
  - 临时补 enum 后，所有组件对象文件、`libcarla_static.a`、`libcarla_shared.so` 可生成。
  - 独立 CMake 可执行文件仍因未链接 `libcarla_gaasd_adapter.so` 失败；手动加入 `project/carla/objectCode/total/DLL/libcarla_gaasd_adapter.so` 后链接成功，`ldd` 可解析 `libcarla_gaasd_adapter.so`、`libzmq.so.5`。
  - 结论：当前画布代码链路已可编译/可链接；剩余问题是 GAASD 生成器 enum bug 和正式仿真启动流程是否传递 `PROJECT_EXTRA_LIBS`。
- 阶段二 GAASD 画布真实闭环已跑通：
  - 手动启动脚本：`tools/carla_bridge/start-gaasd-carla-manual.sh`，默认使用 `tools/carla_bridge/config.phase2.json`。
  - GAASD 示波器周期从 `0.5s` 调整到 `0.1s` 后，30 秒仿真产生 `301` 个采样点。
  - 最新有效日志：`/home/aiden/gaasd_server/codeTools/dist/logs/run_simulation_20260508_143150.log`。
  - `egoV` 从 `0` 上升到最高约 `2.35 m/s`，结束时约 `2.31 m/s`。
  - Bridge 侧 `control_cmd_received=301`、`last_command_id=301`，说明 GAASD 控制命令已持续进入 Bridge。
  - 结论：链路 `CARLA -> Bridge -> GAASD 组件 -> GAASD 控制输出 -> Bridge -> CARLA 自车` 已成立。
  - 剩余现象：仿真后半段 `leadV=0`、`distance=1000000`，说明前车后续未被 Bridge 选为有效同车道前车；下一步需要固定前车行为和车道位置后验证 ACC 跟车收敛效果。
- 前车固定策略已调整，待下一轮 GAASD 页面重跑验证：
  - `tools/carla_bridge/spawn-lead-vehicle.py` 默认从 `ego_forward + constant_velocity` 改为 `ego_forward + traffic_manager`。
  - 前车默认放在 ego 正前方，由 CARLA Traffic Manager 管理，禁止变道、忽略红绿灯。
  - 当前地图出生点下 `lane_waypoint` 会把前车放到侧向约 16m 的位置，暂不作为默认策略。
  - Bridge 已增加 `role_name` 输出，并在阶段二配置中优先选择 `role_name=gaasd_lead` 的测试前车，允许该前车使用更宽的横向筛选阈值。
  - `tools/carla_bridge/start-gaasd-carla-manual.sh` 新增 `--lead-placement`、`--lead-behavior`、`--tm-port` 参数，并默认使用 `ego_forward` / `traffic_manager`。
  - 命令行侧 20 秒探测结果：`ego_forward + traffic_manager` 下 `lead_vehicle.valid=399/399`，`selection_rule=preferred_role_nearest_front`，`clearance_m≈20.21`，`lateral_distance≈0`。
  - 当前 P0 前车可视为静止目标，因此 `leadV=0` 属于预期；下一轮 30 秒日志重点确认 `distance` 不再跳到 `1000000`，`egoV` 和 `targetSpeed` 对静止前车产生响应。
- 固定前车后的 30 秒 GAASD 画布测试通过：
  - 最新有效日志：`/home/aiden/gaasd_server/codeTools/dist/logs/run_simulation_20260509_112022.log`。
  - 示波器周期 `0.1s`，30 秒产生 `301` 个采样点。
  - Bridge 侧 `control_cmd_received=301`、`last_command_id=301`。
  - `distance` 从约 `20.21m` 收敛到约 `15.64m`，全程未跳到 `1000000`。
  - `egoV` 从 `0` 上升到最高约 `1.90m/s`，末值约 `0.06m/s`；说明自车先靠近前车，随后因 ACC 目标速度下降而减速。
  - `targetSpeed` 从约 `2.60m/s` 下降到约 `0.29m/s`，符合接近静止前车时的降速行为。
  - Bridge 实时探测显示 `lead_vehicle.valid=true`、`selection_rule=preferred_role_nearest_front`、`clearance_m≈15.63`、`lateral_distance≈-0.07`。
  - 结论：P0 30 秒 ACC 静止前车闭环验证通过；下一步补跑 60 秒稳定性测试。
- 60 秒稳定性测试通过：
  - 最新有效日志：`/home/aiden/gaasd_server/codeTools/dist/logs/run_simulation_20260511_114220.log`。
  - 示波器周期 `0.1s`，60 秒产生 `601` 个采样点。
  - `distance` 从约 `20.21m` 收敛到约 `15.38m`，全程无 `1000000` 跳变。
  - `egoV` 从 `0` 上升到最高约 `1.86m/s`，末值约 `0.046m/s`。
  - `targetSpeed` 从约 `2.60m/s` 下降到约 `0.17m/s`。
  - 结论：P0 静止前车 60 秒稳定性验证通过；下一步进入动态前车场景。
- 软件团队提供的 `/data/aiden/下载/CARLA-json.zip` 已检查：
  - 包含 `CARLASimClock`、`CARLAEgoState`、`CARLAObjectList`、`CARLALeadVehicle`、`CARLATrafficLightList`、`CARLARoute`、`CARLAControlCmd`、`CARLALongitudinalCmd`、`CARLALateralCmd`、`CARLATrajectoryCmd`、`CARLAChassisFeedback` 共 11 个组件 JSON。
  - topic 和主要端口字段与协议基本一致。
  - 组件类型为 `carla-sim-input/output`，没有 `function_body`；软件团队确认 GAASD 当前只能提供组件显示和连线关系。
  - 结论：该包可作为后续正式 UI 组件定义参考，但当前不能承担真实数据订阅/发布运行逻辑，P0 测试继续使用 C 包装组件。

---

## 当前状态（2026-04-01 更新）

**工作目标**：将 `huanyuan1/thicv-pilot/planningFigure/` 里的规划算法从 C++ 迁移到纯 C。

**当前验证版路径**：`huanyuan1/thicv-pilot/planningFigure/`
- C 文件放在各自子目录（`GaussConvert/`、`GetMinDistanceOfPoint/` 等）
- C++ 框架文件放在 `src/`
- 构建已通过：`[100%] Built target planning`，`start_all.sh` 可运行

**最近完成的工作**：
- 新增 `ACC/accSimStep.c`，实现单步 ACC 仿真推进逻辑（前车更新→测距→目标速度→PID→自车更新）
- 将 `ACC/accSimStep.c` 接入 `planningFigure/CMakeLists.txt`
- 新建了 4 个 C 模块（见下方"已完成记录"第一条）
- 修复了 `localPlanning_m.cpp:3257` 遗留的坐标交叉赋值 Bug
- 在 AGENTS.md 中新增了"坐标系约定"章节

---

## 下一步任务（按优先级）

### P1 — 立即可做

- [ ] **将坐标系约定注释补充到 `gaussConvert.h` 的 Doxygen**
  - 在 `gaussConvert` 的 `@param` 里说明 `dNorth_X=北坐标/dEast_Y=东坐标`
  - 在 `gaussConvertOutput` 结构体字段注释里写清楚含义

- [ ] **提交当前所有修改到 git**
  - 修改的文件：`GaussConvert/`、`GetMinDistanceOfPoint/`、`AssessTrajectory/`、`GetOptimalTrajectoryIndex/`、`include/localPlanningNew.h`、`include/localPlanning_m.hpp`、`src/localPlanning_m.cpp`、`src/planningFigure_m.cpp`、`CMakeLists.txt`、`AGENTS.md`、`../../start_all.sh`

### P2 — 下一个大模块

- [ ] **坐标转换模块 `geometry_m.cpp`**（AGENTS.md 中标记为 🔴）
  - 参考文件：`src/geometry_m.cpp`
  - 输出位置：建议放 `Geometry/` 目录
  - 注意：涉及坐标变换，严格遵守 AGENTS.md 的坐标系约定

- [ ] **线性插值 `interpolate_m.cpp`**（标记为 🔴）
  - 参考文件：`src/interpolate_m.cpp`

### P3 — 工具库（其他模块依赖）

- [ ] **`DataStructure/DynArray/dynArray.c`**（已有 `dynArray.h`，实现未完成）
  - 接口在 `dynArray.h` 已定义，用 `/codex-impl` 生成实现

---

## 已完成工作记录

### [2026-03-30] 最优轨迹选取模块全面重写

**目标**：移植 `localPlanning_m.cpp` 中的障碍物距离评估链路到 C 语言。

**新建文件**：

| 文件 | 来源 C++ | 说明 |
|------|----------|------|
| `GaussConvert/gaussConvert.c/.h` | `localPlanning_m.cpp:62~95` + `115~139` | 经纬度转高斯坐标、全局→局部坐标变换 |
| `GetMinDistanceOfPoint/getMinDistanceOfPoint.c/.h` | `localPlanning_m.cpp:3204~3327` + `3341+` | 计算路径点到障碍物最近距离，含 5 个 static 辅助函数 |
| `AssessTrajectory/assessTrajectory.c/.h` | `localPlanning_m.cpp:3154~3189` | 遍历轨迹各点求最小障碍物距离 |
| `GetOptimalTrajectoryIndex/getOptimalTrajectoryIndex.c` | 原有 C++ 风格文件，完全重写 | 红绿灯处理 + 最优轨迹选取 |

**修改文件**：

| 文件 | 改了什么 |
|------|----------|
| `include/localPlanningNew.h` | 末尾新增 9 个 typedef struct（高斯/坐标变换/findMin/getMinDistanceOfPoint 相关）|
| `include/localPlanning_m.hpp` | 12 个 C++ struct 加 `Cpp` 后缀，避免与 C typedef 名称冲突 |
| `src/localPlanning_m.cpp` | 同步更新 12 个 struct 使用处为 `Cpp` 后缀 |
| `src/planningFigure_m.cpp` | 同步更新 4 个 struct 使用处为 `Cpp` 后缀 |
| `CMakeLists.txt` | 新增 4 个 C 文件到编译目标；include 目录补全；源文件路径改为 `./src/` 前缀 |
| `../../start_all.sh` | 新增：启动前备份 planning 二进制，防止 make 失败删除唯一可运行版本 |

**Bug 修复**：

| 位置 | 问题 | 修复方式 |
|------|------|----------|
| `getMinDistanceOfPoint.c` `appendSingleRoadsideObjectDistances` | 从 `localPlanning_m.cpp:3257` 照抄了坐标交叉赋值（East/North 写反）| 改为正确写法：`gaussNorthTemp = outputGC.dNorth_X; gaussEastTemp = outputGC.dEast_Y` |

**说明**：此 Bug 在原 C++ `localPlanning_m.cpp` 中存在，`planningFigure_m.cpp:1047` 是正确的参考实现。不影响当前运行的原因：后续只算欧氏距离，x/y 对调不改变 `sqrt(dx²+dy²)` 的值。

**注释**：4 个新 C 文件均包含：
- 函数级 Doxygen 注释
- 内联步骤注释（含对应 C++ 文件行号）
- 所有局部变量行内说明

---

**本次工作遗留的编译调试记录**（供参考，非待办）：

- CMakeCache 指向 Docker 路径 `/tmp/Modularization`，本地编译需删除 cache 重新 cmake
- libzmq.so 符号链接缺失，需手动 `ln -s libzmq.so.5 libzmq.so`
- `.cpp` 源文件曾丢失（git stash），从 `stash@{0}` 恢复后更新 CMakeLists.txt 路径前缀
- make 失败会删除 planning 二进制（POSIX 行为），已在 start_all.sh 加备份逻辑

---

## 坐标系约定速查（详见 AGENTS.md）

```
gaussConvert 输出：dNorth_X = 北坐标(y)，dEast_Y = 东坐标(x)
IMU 字段：       gaussx   = 北坐标(y)，gaussy   = 东坐标(x)

正确赋值：
  gaussNorthTemp = outputGC.dNorth_X;
  gaussEastTemp  = outputGC.dEast_Y;
  dX = gaussEastTemp;   /* x = East */
  dY = gaussNorthTemp;  /* y = North */
```
