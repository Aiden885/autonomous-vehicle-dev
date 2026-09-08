# Latest ACC/LKS Generated Code Integration Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在本地调试平台中增加使用 `newaccpro4` 与 `lks3` 最新 GAASD 生成代码的 Pangu-CARLA 测试场景。

**Architecture:** 保留现有 `newaccpro3/lks2` 场景作为回退基线。只补齐最新生成模块附带的 ZMQ 边界节点，使其在 5701/5702 与 Pangu 的 `acc_input/acc_output`、`lks_input/lks_output` 通道之间转发；ACC/LKS 算法计算文件保持不变。构建与安装落在独立缓存目录，新场景由现有调试平台自动扫描。

**Tech Stack:** Bash、C++17、CMake、Pangu/ICVOS、ZeroMQ、CARLA 0.9.15、Flask 场景面板。

**Spec:** 本会话已确认的隔离接入方案；用户允许对生成源码做最小边界修补，不要求完整严谨验证。

## Global Constraints

- 不覆盖 `scenarios/newaccpro3_pangu_carla_20260701` 和 `scenarios/lks2_pangu_carla_20260701`。
- 不修改 `run_*`、`composite_block_*`、真值表等 ACC/LKS 算法文件。
- 保持现有 ZMQ 协议主题与端口：输入 `5701`，控制输出 `5702`。
- 构建缓存分别使用 `~/.cache/gaasd-pangu/newaccpro4_codegen_build` 与 `~/.cache/gaasd-pangu/lks3_codegen_build`。
- UI 复用现有场景自动发现、ACC 按键、LKS 驾驶接管和参数面板。

---

### Task 1: Boundary Completeness Check

**Files:**
- Create: `tools/pangu_generated_closed_loop/check_generated_boundaries.py`

**Interfaces:**
- Consumes: `project/newaccpro4/icvos/src/modules` 与 `project/lks3/icvos/src/modules`。
- Produces: 检查边界节点是否声明 Pangu publisher/subscriber、是否实际调用 `PUB_MSG` 与 `PublishControl`。

- [ ] 编写边界完整性检查并在修改前运行，确认当前生成骨架失败。
- [ ] ACC 与 LKS 边界修补后再次运行，确认检查通过。

### Task 2: Complete Generated ZMQ Boundary Nodes

**Files:**
- Modify: `project/newaccpro4/icvos/src/modules/module_empty/module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43/src/AccZmqInputModule_*.{h,cc}`
- Modify: `project/newaccpro4/icvos/src/modules/module_empty/module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43/src/AccZmqOutputModule_*.{h,cc}`
- Modify: `project/newaccpro4/icvos/src/modules/module_empty/module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43/conf/AccZmq*.pt`
- Modify: `project/lks3/icvos/src/modules/module_empty/module_empty_a807407f_21e6_442d_a6e4_13a788ec666d/src/LksZmqInputModule_*.{h,cc}`
- Modify: `project/lks3/icvos/src/modules/module_empty/module_empty_a807407f_21e6_442d_a6e4_13a788ec666d/src/LksZmqOutputModule_*.{h,cc}`
- Modify: `project/lks3/icvos/src/modules/module_empty/module_empty_a807407f_21e6_442d_a6e4_13a788ec666d/conf/LksZmq*.pt`

**Interfaces:**
- Consumes: CARLA Bridge JSON topics through `AccZmqBridge`/`LksZmqBridge`。
- Produces: Pangu 输入消息及 CARLA `control_cmd` 输出。

- [ ] 输入节点轮询 5701 后发布完整 `AccInput`/`LksInput`。
- [ ] 输出节点订阅 `AccOutput`/`LksOutput` 后向 5702 发布控制命令。
- [ ] LKS 保留制动退出和驾驶员转向接管的既有边界行为。

### Task 3: Reproducible Latest-Project Builds

**Files:**
- Create: `tools/pangu_generated_closed_loop/build_generated_project.sh`

**Interfaces:**
- Consumes: 项目名、模块 UUID、Pangu 框架和 Docker 镜像。
- Produces: 独立 `install/` 运行目录。

- [ ] 将框架、项目配置和最新生成模块复制到缓存构建根目录。
- [ ] 在 Pangu Docker 中配置、编译、安装，并校验主模块和两个边界动态库。

### Task 4: Debug Panel Scenarios

**Files:**
- Create: `scenarios/newaccpro4_codegen_carla_20260817/{scenario.yaml,run.sh,stop.sh,bridge_config.json}`
- Create: `scenarios/lks3_codegen_carla_20260817/{scenario.yaml,run.sh,stop.sh,bridge_config.json}`

**Interfaces:**
- Consumes: Task 3 的安装目录与现有 CARLA 场景脚本。
- Produces: 调试平台自动发现的 ACC/LKS 场景。

- [ ] ACC 场景启动 CARLA、前车、最新 Pangu 三进程、记录器和 E 启控指令。
- [ ] LKS 场景启动 Town04 单车、最新 Pangu 三进程、记录器和参数透传。
- [ ] stop 脚本清理各自容器、记录器、Bridge 与 CARLA。

### Task 5: Focused Verification

**Files:**
- No production files.

**Interfaces:**
- Consumes: Task 1-4 的实现。
- Produces: 编译及简化闭环结果。

- [ ] 运行边界静态检查和 Shell 语法检查。
- [ ] 构建 ACC、LKS 最新 Pangu 安装目录。
- [ ] 启动 mock 或 CARLA 场景，确认算法主进程及两个边界进程存活、5701/5702 链路有输入输出。
- [ ] 记录未覆盖的完整道路效果验证，不将简化测试表述为最终算法验收。
