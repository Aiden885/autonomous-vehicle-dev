# GAASD 2.7.0.5 ACC 决策与控制适配分析

日期：2026-06-03

本文基于本机新版 GAASD 2.7.0.5 安装目录、`gaasd_server` 工具链、当前 CARLA Bridge、已有 GAASD 工程和场景快照进行源码级检查，目的是明确新版 GAASD 对我们 ACC 决策与控制闭环方案的影响，并给出下一步可在画布上继续搭建和联调的方案。

## 1. 结论

当前新版 GAASD 不是一次完整切换到新 C++ 节点/函数模型，而是存在两套并行链路：

| 链路 | 工具 | 当前状态 | 对 ACC 的影响 |
|---|---|---|---|
| 新版节点/通用代码生成 | `new_gaasd` | GUI 源码里已接入，但本机 profile 未发现可用 `new_gaasd`；开发团队也说 `gaas_codegen` 路径有误，等后续 C++ 版本 | 暂时不能作为 ACC 闭环主路径 |
| 函数画布/示波器仿真 | `gaas_codegen` + `run_simulation.sh` | 仍是旧链路，能生成 `FuncStep()`，但仿真脚本仍要求旧字典 `struct.json` | 这是当前唯一可落地跑 CARLA 闭环的路径，但必须规避新版字典和真值表 bug |

因此，新版 GAASD 下我们的 ACC 方案要调整为：

1. 近期继续使用旧 C 函数模型跑闭环，不直接切到 C++ `FuncModule` 组件。
2. CARLA 输入输出继续使用单输出旧 C 组件：`CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、`CARLAACCLeadDistance`、`CARLAACCLongitudinalCmd`。
3. ACC 控制先继续用基础模块搭建公式，不依赖多输出组件。
4. ACC 决策层不要直接依赖当前真值表生成结果；真值表生成器存在实测 bug，会把大部分规则生成成无条件赋值，最终 `y` 基本被覆盖为 8。
5. 新版工程如果使用新版 THICV 字典，需要先解决 `run_simulation.sh` 仍硬检查 `struct.json` 的问题，否则示波器仿真会在编译前退出。
6. 等 GAASD 团队提供真正可用的 C++ `new_gaasd`、新版 codescan 和新版仿真脚本后，再迁移到新版节点/函数层结构。

## 2. 新版 GAASD 安装与工具链现状

### 2.1 安装位置

当前本机存在新版旁路安装：

| 内容 | 路径 |
|---|---|
| 新版 GAASD App | `/home/aiden/gaasd_versions/gaasd-2.7.0.5/app` |
| 新版隔离 profile | `/home/aiden/gaasd_versions/gaasd-2.7.0.5/home` |
| 新版启动脚本 | `/home/aiden/gaasd_versions/gaasd-2.7.0.5/run-gaasd-2.7.0.5.sh` |

但实际检查发现 `/usr/bin/gaasd -> /opt/gaasd/gaasd` 的二进制与旁路新版二进制哈希一致，`build.env.json` 都显示：

```json
{
  "projectName": "gaasd_1",
  "version": "build-2.7.0.5",
  "originVersion": "2.7.0.5"
}
```

需要注意的是，虽然二进制一致，`resources/app/dist` 内部 JS 资源和运行 profile 不一定一致。后续测试必须固定启动入口，否则 GUI 可能读取不同的 `HOME/gaasd_server`。

### 2.2 profile 内工具链

新版 profile 已安装：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/codeTools/
  gaas_codegen
  codescan
  dist/run_simulation.sh
```

但未发现可用 `new_gaasd`。源码中多处新版功能要求 `new_gaasd`：

```text
resources/app/dist/Action/WsServer.js
  普通代码生成: $HOME/gaasd_server/codeTools/new_gaasd gen_code ...

resources/app/dist/common/environments/linuxEnv.js
  codeScan: new_gaasd import ...
  fullScan: new_gaasd scan ...
  submodule_gen_code: ./new_gaasd submodule_gen_code ...
```

这说明新版 GUI 已经按 `new_gaasd` 设计，但当前本机拿到的工具包不完整，或者开发团队当前发布的 `gaasd_code_tools_20260529.tar` 仍不是最终 C++ 版。

### 2.3 新版 profile 组件库

新版 profile 的 THICV 组件库只包含少量新风格组件：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/components/THICV/component/
  PidController_b42c0e67.json
  TopCanvas_237dcf0f.json
  VehicleModel_4c08435f.json
```

这些组件已经带有新版配置概念，例如 `PidController` 包含：

| 类型 | 示例 |
|---|---|
| param | `pidParam.gains.kp/ki/kd`、`antiWindup`、`ts` |
| state | `ePrev`、`iSum` |
| 输入端口 | `r`、`y` |
| 输出端口 | `u` |

这说明新版 GAASD 正在往“组件实例参数 + 组件状态 + 复合变量”方向改，但这条路线还没有与 CARLA 函数模型闭环完全打通。

## 3. 新版源码关键变化

### 3.1 新版代码生成链路

在 `resources/app/dist/Action/WsServer.js` 中，普通代码生成调用：

```text
$HOME/gaasd_server/codeTools/new_gaasd gen_code <genCodeConfig>
```

这属于新版节点/应用代码生成链路。

但函数模型生成仍调用旧工具：

```text
$HOME/gaasd_server/codeTools/gaas_codegen \
  --input-dir <project>/icvos/blocks/functions \
  --output-dir <project>/icvos/src/oscilloscopeFunctions \
  --main-canvas <graphName> \
  --scan-path <components>/<vendor>/component \
  --is-oscilloscope true
```

这说明当前画布 ACC 仿真仍是旧函数模型，核心运行入口还是 `FuncStep()`。

### 3.2 函数画布生成前处理

`Generate.functionModelGeneratePreparation()` 会：

1. 删除工程下的 `icvos/blocks/functions`。
2. 重新创建 `icvos/blocks/functions`。
3. 把当前选择的组件库字典复制到工程：

```text
<components>/<vendor>/dictionaryData
  -> <project>/icvos/blocks/functions/dictionaryData
```

这一步非常关键。只要你选择新版 THICV 组件库，工程就会得到新版拆分字典，而不是旧的 `struct.json/baseType.json` 字典。

### 3.3 新版字典格式

新版 profile 的 THICV 字典是拆分格式：

```text
GlobalContext.json
GlobalContextTypes.json
GlobalVariable.json
PidController.json
PidControllerTypes.json
TopCanvas.json
VehicleModel.json
baseTypes.json
enum.json
includes.json
typedef.json
using.json
```

旧版 THICV 字典是：

```text
baseType.json
enum.json
includes.json
macros.json
struct.json
typedef.json
```

新版 GUI 的部分源码已经兼容新旧字段，例如读取类型时会尝试：

```text
GlobalContextTypes.json 或 struct.json
baseTypes.json 或 baseType.json
```

但仿真运行脚本没有同步更新。

### 3.4 示波器仿真运行链路

示波器启动走 `functionModelStart`，实际调用：

```text
bash run_simulation.sh \
  -t <imagePath> \
  -i <imageName>:env \
  -c module_sim_container_<projectId> \
  <projectPath> <step> <duration> <port> <scopeIds...>
```

默认配置：

| 配置项 | 值 |
|---|---|
| WebSocket | `ws://127.0.0.1:9002` |
| 仿真端口 | `9002` |
| 默认步长 | `0.2s` |
| 默认时长 | `10s` |
| 脚本 | `$HOME/gaasd_server/codeTools/dist/run_simulation.sh` |

源码中明确要求 `scopeIds` 非空：

```text
scopeIds.length === 0 -> 报 “缺少示波器ID”
```

所以 ACC 联调工程必须至少连接一个示波器信号，否则页面层不会启动真实仿真。

## 4. 新版 GAASD 当前主要问题

### 4.1 代码扫描工具仍不能扫描新版 C++ 组件

本机实测用新版 profile 的 `codescan` 扫描我们生成的 C++ `FuncModule<Traits>` 组件：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/codeTools/codescan \
  generated/gaasd_p0_acc_min_components_src \
  -o /tmp/gaasd_codescan_test_acc_min \
  --custom
```

输出结果：

```text
找到 0 个JSON文件
已输出 0 个结构体到 ...
result: success
```

结论：

```text
当前 codescan 仍是旧 C 扫描器或旧逻辑，不能把 C++ FuncModule 组件扫描成 GAASD 可用组件。
```

因此，`generated/gaasd_p0_acc_min_components_src`、`generated/gaasd_p1_components_src`、`generated/gaasd_lks_components_src` 暂时不能作为当前画布闭环的导入来源。

### 4.2 `run_simulation.sh` 与新版字典不兼容

新版 profile 和全局 `run_simulation.sh` 是同一份旧脚本，仍硬检查：

```bash
CODE_DICT_DIR="${PROJECT_DIR}/icvos/blocks/functions/dictionaryData"
if [[ ! -f "${CODE_DICT_DIR}/struct.json" ]]; then
    echo "错误: 未找到结构体字典: ${CODE_DICT_DIR}/struct.json" >&2
    exit 1
fi
```

而 `project/newaccpro2` 当前字典是新版拆分格式，没有 `struct.json`：

```text
project/newaccpro2/icvos/blocks/functions/dictionaryData/
  GlobalContextTypes.json
  baseTypes.json
  ...
```

这会导致示波器仿真在编译前直接失败。这个问题属于新版 GAASD 工具链内部不一致，不是 CARLA Bridge 或 ACC 控制公式的问题。

### 4.3 当前 `newaccpro2` 生成代码是正确的，但运行脚本会卡在字典

`project/newaccpro2` 已生成示波器入口：

```text
project/newaccpro2/icvos/src/oscilloscopeFunctions/c_process_empty_.../FuncStep.c
```

核心逻辑：

```c
leadV = CARLAACCLeadSpeed();
distance = CARLAACCLeadDistance();
egoV = CARLAACCEgoSpeed();

targetSpeed =
    Kdist * (distance - desiredDistance)
  + Kspeed * (leadV - egoV)
  + leadV;

CARLAACCLongitudinalCmd(targetSpeed, enable);
scope_push_send(... targetSpeed, egoV, leadV, distance);
```

当前参数：

```text
desiredDistance = 15
Kdist = 0.9
Kspeed = 0.8
enable = 1
```

这说明画布到 `FuncStep()` 的生成链路是通的。当前主要问题是仿真启动脚本和新版字典不匹配。

### 4.4 旧 ACC 决策真值表生成结果错误

`project/accpro2` 当前没有生成 `FuncStep.c`，并且真值表生成结果存在明显 bug：

```c
y=1;
y=2;
y=3;
y=4;
y=7;
y=5;
y=5;
y=6;

if(u==0&&(v==6||v==7)) {
  y=8;
}
if(!(u==0&&(v==6||v==7))) {
  y=8;
}
```

这段代码会导致大部分情况下 `y` 最终都变成 8。它不是我们决策表逻辑的问题，而是 `gaas_codegen` 对真值表的代码生成不正确。

因此，在新版 GAASD 的 ACC 决策方案中，不能把当前真值表组件作为可靠执行单元。

### 4.5 组件库路径存在混用风险

`project/newaccpro2/log/2026-06-03.log` 中实际使用的是：

```text
/home/aiden/gaasd_server/codeTools/gaas_codegen
/home/aiden/gaasd_server/components/THICV/component
```

不是新版隔离 profile 下的：

```text
/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/...
```

这说明当前某次生成很可能不是通过隔离新版启动脚本完成的，或者 GAASD 内部仍解析到了全局 HOME。后续复测必须统一：

```text
到底使用 /home/aiden/gaasd_server
还是 /home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server
```

否则同一工程会出现组件库、字典、代码生成工具不一致的问题。

## 5. 当前 Bridge 和 adapter 状态

### 5.1 Bridge 已具备的消息

当前 `tools/carla_bridge/carla_bridge.py` 发布链路已经包括：

| topic | 作用 |
|---|---|
| `sim_clock` | 仿真时钟 |
| `ego_state` | 自车状态 |
| `object_list` | 障碍物列表 |
| `lead_vehicle` | 前车摘要 |
| `lane_tracking` | 车道中心偏差和航向误差 |
| `chassis_feedback` | 底盘反馈 |
| `bridge_status` | Bridge 状态 |

其中 `lane_tracking` 已经由 Bridge 计算并发布，不再是待实现项。

### 5.2 adapter C ABI 已具备的接口

当前 `tools/carla_bridge/adapter/include/carla_gaasd_adapter.h` 已提供：

| C 函数 | 用途 |
|---|---|
| `carla_adapter_read_ego_state` | 读自车速度/位置/航向/加速度 |
| `carla_adapter_read_lead_vehicle` | 读前车速度/距离/相对速度/TTC |
| `carla_adapter_read_chassis_feedback` | 读底盘反馈 |
| `carla_adapter_read_lane_tracking` | 读横向偏差/航向误差 |
| `carla_adapter_read_object_list` | 读障碍物列表 |
| `carla_adapter_publish_longitudinal_cmd` | 发布纵向目标速度 |
| `carla_adapter_publish_lateral_cmd` | 发布横向转角 |
| `carla_adapter_publish_control_cmd` | 发布速度/加速度/转角综合控制 |
| `carla_adapter_get_status` | 读 adapter 状态 |

所以 Bridge/adapter 的运行能力已经足够支持 ACC、LKS 和后续基础决策测试。当前主要不确定性在 GAASD 生成和仿真工具链。

## 6. ACC 决策与控制方案调整

### 6.1 原方案的问题

原 ACC 完整方案大致是：

```text
CARLA 输入
  -> egoV / leadV / distance
  -> 决策层真值表 y
  -> 根据 y 选择控制参数
  -> 目标速度计算
  -> CARLA 纵向控制命令
```

这个方向仍然正确，但在新版 GAASD 下要调整实现方式：

| 原设想 | 新版下的问题 | 调整 |
|---|---|---|
| 使用 C++ `FuncModule` 组件 | 当前 scanner 输出 0 组件 | 暂时不用 |
| 使用多输出 CARLA 状态组件 | GAASD 生成器对多输出组件历史上不稳定 | 继续拆成单输出组件 |
| 使用真值表组件实现决策 | 当前生成 C 代码错误 | 改为自定义单输出决策函数，或先只观察不参与控制 |
| 直接用新版 THICV 字典跑示波器 | `run_simulation.sh` 仍要求 `struct.json` | 使用旧字典组件库，或补兼容字典 |
| 直接迁移到节点方式 | 本机缺 `new_gaasd`，团队也说等 C++ 版本 | 后续再切换 |

### 6.2 新版下推荐的 ACC 分层

建议把 ACC 拆成四层，逐层验证：

```text
层 1：CARLA 输入层
  CARLAACCEgoSpeed
  CARLAACCLeadSpeed
  CARLAACCLeadDistance

层 2：决策层
  ACCCommandDecision 或基础逻辑块
  输入：u, v
  输出：decisionY

层 3：控制参数层
  根据 decisionY 得到 desiredDistance / maxSpeed / enable
  近期可以先固定参数，decisionY 只接示波器观察

层 4：纵向控制层
  distanceError = distance - desiredDistance
  relativeSpeed = leadV - egoV
  targetSpeed = leadV + Kdist * distanceError + Kspeed * relativeSpeed
  targetSpeed 限幅
  CARLAACCLongitudinalCmd(targetSpeed, enable)
```

### 6.3 第一版可运行画布

第一版不要把决策输出直接接入控制，先验证新版 GAASD 工程能稳定跑完整闭环：

```text
CARLAACCEgoSpeed.egoV
CARLAACCLeadSpeed.leadV
CARLAACCLeadDistance.distance

distanceError = distance - 15
speedError = leadV - egoV
distTerm = Kdist * distanceError
speedTerm = Kspeed * speedError
targetRaw = leadV + distTerm + speedTerm
targetSpeed = limit(targetRaw, 0, 5)
enable = 1

CARLAACCLongitudinalCmd(targetSpeed, enable)
```

推荐参数：

| 参数 | 值 | 说明 |
|---|---|---|
| `desiredDistance` | `15.0 m` | 当前直道跟车目标距离 |
| `Kdist` | `0.35 ~ 0.9` | 现有 `newaccpro2` 用 0.9，收敛快但可能更激进；稳定演示可先用 0.35 |
| `Kspeed` | `0.8` | 相对速度反馈 |
| `maxSpeed` | `5.0 m/s` | 约 18 km/h |
| `enable` | `1` | 第一轮固定使能 |
| 示波器周期 | `0.1s` | 已验证适合当前 Bridge 超时配置 |
| 仿真时长 | `30s ~ 60s` | 先 30 秒排故，再 60 秒稳定性 |

示波器至少接：

```text
targetSpeed
egoV
leadV
distance
```

### 6.4 第二版加入决策观察

第二版加入决策层，但只观察 `decisionY`，不直接控制车辆：

```text
常量 u
常量 v
  -> ACCCommandDecision(u, v)
  -> decisionY
  -> 示波器
```

先用常量切换测试：

| u | v | 期望 decisionY |
|---|---|---|
| 0 | 1 | 1 |
| 0 | 2 | 2 |
| 0 | 3 | 3 |
| 0 | 4 | 4 |
| 0 | 5 | 7 |
| 1 | 1 | 5 |
| 2 | 1 | 5 |
| 1 | 2 | 6 |
| 0 | 6 | 8 |
| 0 | 7 | 8 |
| 其他 | 8 |

由于真值表生成器不可靠，建议不要用当前 GAASD 真值表组件执行上表，而是生成一个旧 C 原子函数组件：

```c
int ACCCommandDecision(short u, short v)
```

这样它仍然能作为画布上的一个“决策组件”存在，但不会受真值表生成 bug 影响。

### 6.5 第三版让决策影响控制

第三版再让 `decisionY` 影响控制参数。建议先只影响 `desiredDistance` 和 `enable`，不要一次把所有模式都接上：

| decisionY | 含义 | 控制参数建议 |
|---|---|---|
| 1 | 降速 | `maxSpeed = 3.0 m/s` |
| 2 | 增速 | `maxSpeed = 5.0 m/s` |
| 3 | 降距 | `desiredDistance = 12.0 m` |
| 4 | 增距 | `desiredDistance = 18.0 m` |
| 5 | 无继承启控 | `enable = 1`，`desiredDistance = 15.0 m` |
| 6 | 继承启控 | `enable = 1`，沿用当前参数 |
| 7 | 扭矩仲裁 | `enable = 1`，可先等同正常 ACC |
| 8 | 待命 | `enable = 0` |

如果画布基础模块难以表达参数选择，优先做一个单输出自定义函数：

```c
double ACCDesiredDistance(int decisionY)
double ACCMaxSpeed(int decisionY)
int ACCEnable(int decisionY)
```

这样每个组件仍是单输出，避免多输出生成风险。

### 6.6 第四版再考虑完整节点化

等 GAASD 团队提供稳定新版 C++ 工具链后，再切换到节点框架：

```text
Bridge
  -> GAASD ACC 节点订阅 ego_state / lead_vehicle / lane_tracking
  -> ACC 节点内部展示函数层
  -> 发布 control_cmd
  -> Bridge
  -> CARLA
```

这条路线更接近软件团队说的“节点级和函数层打通”，但当前不适合马上作为联调主路径。

## 7. 新版画布建议

### 7.1 近期推荐新建工程

不要直接覆盖 `accpro2`。建议新建一个专门适配新版的工程：

```text
project/newacc_full_v2705
```

目的：

1. 不破坏已经跑通过的 `accpro2_basic` 和场景快照。
2. 避免旧工程 schema 被新版 GAASD 保存后不可逆改写。
3. 方便对比新版字典和旧字典差异。

### 7.2 画布模块布局

建议画布分成 5 个区域，从左到右：

```text
[CARLA 输入]
  egoV
  leadV
  distance

[状态/指令输入]
  u
  v
  enableFallback

[决策层]
  ACCCommandDecision
  decisionY

[控制计算]
  desiredDistance
  distanceError
  relativeSpeed
  targetRaw
  targetSpeed

[CARLA 输出与观测]
  CARLAACCLongitudinalCmd
  Oscilloscope(targetSpeed, egoV, leadV, distance, decisionY)
```

### 7.3 第一轮连线

先搭建不带决策的控制闭环：

| 来源 | 目标 | 说明 |
|---|---|---|
| `CARLAACCLeadDistance.distance` | `subtract.input1` | 当前前车距离 |
| 常量 `15` | `subtract.input2` | 目标距离 |
| `subtract.output` | `multiply.input1` | 距离误差 |
| 常量 `Kdist` | `multiply.input2` | 距离反馈增益 |
| `CARLAACCLeadSpeed.leadV` | `subtract_speed.input1` | 前车速度 |
| `CARLAACCEgoSpeed.egoV` | `subtract_speed.input2` | 自车速度 |
| `subtract_speed.output` | `multiply_speed.input1` | 相对速度 |
| 常量 `Kspeed` | `multiply_speed.input2` | 速度反馈增益 |
| `distTerm`、`speedTerm`、`leadV` | 加法链 | 目标速度原始值 |
| `targetSpeed` | `CARLAACCLongitudinalCmd.speed` | 控制输出 |
| 常量 `1` | `CARLAACCLongitudinalCmd.enable` | 固定使能 |
| `targetSpeed/egoV/leadV/distance` | 示波器 | 运行观测 |

如果可用基础库里有 `limit`，把 `targetRaw` 接入 `limit`：

```text
targetSpeed = limit(targetRaw, 0, 5)
```

如果新版 `limit` 暂时不稳定，第一轮可以不接限幅，因为 Bridge 内部会对速度和油门/刹车做安全限制；但正式演示前建议补限幅。

### 7.4 第二轮连线

加入决策观察：

| 来源 | 目标 | 说明 |
|---|---|---|
| 常量 `u` | `ACCCommandDecision.u` | ACC 状态 |
| 常量 `v` | `ACCCommandDecision.v` | 驾驶员/系统指令 |
| `ACCCommandDecision.decisionY` | 示波器 | 观察决策结果 |

此阶段不要把 `decisionY` 接进控制参数，避免把控制闭环和决策生成问题混在一起。

### 7.5 第三轮连线

再让 `decisionY` 控制 `desiredDistance/maxSpeed/enable`：

```text
decisionY -> ACCDesiredDistance -> desiredDistance
decisionY -> ACCMaxSpeed -> maxSpeed
decisionY -> ACCEnable -> enable
targetRaw + maxSpeed -> limit -> targetSpeed
enable -> CARLAACCLongitudinalCmd.enable
```

## 8. 需要立即处理的工具链问题

### 8.1 选择一个固定运行环境

后续不要混用两个路径：

| 方案 | 优点 | 风险 |
|---|---|---|
| 使用全局 `/home/aiden/gaasd_server` | 与当前 `newaccpro2` 生成日志一致，旧 C 组件多 | 不是隔离新版 profile |
| 使用新版 profile `/home/aiden/gaasd_versions/.../home/gaasd_server` | 更符合旁路安装 | 组件库只有少量新组件，缺旧 C CARLA 组件，且缺 `new_gaasd` |

近期推荐：

```text
继续使用全局 /home/aiden/gaasd_server 跑 ACC 闭环。
新版 profile 只用于观察新功能，不用于当前 CARLA 闭环主测试。
```

原因是当前闭环依赖旧 C 组件库和 `gaas_codegen`，全局环境反而更完整。

### 8.2 处理 `struct.json` 兼容

如果新建工程必须使用新版字典，需要二选一：

| 方案 | 说明 |
|---|---|
| 等 GAASD 团队修复 `run_simulation.sh` | 正式方案 |
| 临时补兼容字典 | 在工程 `dictionaryData` 中生成空/转换版 `struct.json` 和 `baseType.json` |

对当前 ACC 标量工程来说，临时空 `struct.json` 很可能足够，因为 `newaccpro2.h` 的 `STRUCTS_START/END` 为空，没有实际结构体依赖。但这是兼容补丁，不应当作为正式方案提交给软件团队。

### 8.3 避开当前真值表生成器

在 GAASD 修复真值表生成前，ACC 决策不要靠真值表执行。推荐：

1. 画布上保留“决策层”概念。
2. 用自定义单输出 C 函数 `ACCCommandDecision` 表达规则。
3. 把 `decisionY` 接示波器观察。
4. 等真值表修复后，再切回官方真值表组件。

### 8.4 继续使用单输出 CARLA 组件

不要在第一轮新版适配里使用：

```text
CARLAEgoState 多输出组件
CARLALeadVehicle 多输出组件
CARLAObjectList 多输出数组组件
CARLAControlCmd 综合控制组件
```

原因：

1. 旧经验表明多输出组件会触发 GAASD 生成器参数不匹配问题。
2. 当前 ACC 只需要 3 个输入和 1 个输出。
3. 单输出组件已经被 `newaccpro2` 生成验证。

## 9. 与软件团队需要对齐的问题

建议向 GAASD 团队明确以下问题：

| 问题 | 影响 |
|---|---|
| 当前 `gaasd_code_tools_20260529.tar` 是否应包含 `new_gaasd` | 没有它无法验证新版节点/函数层链路 |
| 当前 `codescan` 是否应支持 C++ `FuncModule<Traits>` | 现在扫描 C++ 组件输出 0 个组件 |
| `run_simulation.sh` 是否会更新以支持 `GlobalContextTypes.json/baseTypes.json` | 现在新版字典工程示波器会失败 |
| 真值表生成器是否已修复条件包装丢失问题 | 当前 `accpro2` 生成的决策 C 代码不可用 |
| 新版 profile 和全局 `/home/aiden/gaasd_server` 的推荐使用方式 | 避免生成代码时读错组件库 |
| 示波器启动是否必须依赖 scopeId | 当前源码要求必须有 scopeId，否则不会启动 |

## 10. 后续执行顺序

建议按以下顺序推进，不要跳步：

### 第一步：固化当前最小控制闭环

目标：

```text
在新版 GAASD 下确认 newaccpro2 这种基础模块 ACC 能重新跑通 CARLA。
```

检查项：

1. 使用全局旧 C 组件库。
2. 工程有 `FuncStep.c`。
3. `objectCode/total/DLL/libcarla_gaasd_adapter.so` 存在。
4. 字典有 `struct.json`，或已经补兼容文件。
5. 示波器接 `targetSpeed/egoV/leadV/distance`。
6. 联调环境通过 UI 或脚本启动，CARLA 视角和前车位置正确。

### 第二步：新建完整 ACC 工程

目标：

```text
新建 newacc_full_v2705，按“输入-决策-控制-输出”布局搭建画布。
```

第一版只让决策输出接示波器，不影响车辆。

### 第三步：替换真值表为自定义决策函数

目标：

```text
解决当前真值表生成器不可靠问题。
```

组件建议：

```text
ACCCommandDecision(u, v) -> decisionY
ACCDesiredDistance(decisionY) -> desiredDistance
ACCMaxSpeed(decisionY) -> maxSpeed
ACCEnable(decisionY) -> enable
```

这些都设计成单输出 C 函数组件。

### 第四步：决策影响控制参数

目标：

```text
验证不同 u/v 指令会改变目标车距、速度上限或控制使能。
```

先测试三个典型场景：

| 场景 | 输入 | 期望 |
|---|---|---|
| 正常跟车 | `u=0, v=5` | `decisionY=7`，保持正常 ACC |
| 增距 | `u=0, v=4` | 目标距离变大，车速下降 |
| 取消/刹车 | `u=0, v=6` | `enable=0`，Bridge 安全制动 |

### 第五步：再考虑 C++ 新组件与节点化

目标：

```text
等 GAASD 团队修复工具链后，把当前可跑逻辑迁移为节点级 ACC。
```

这一步不是当前 CARLA 闭环的前置条件。

## 11. 当前推荐决策

近期不要把工作重点放在“适配新版 C++ 组件扫描”上，因为本机工具链还不支持；也不要继续依赖真值表组件跑 ACC 决策，因为生成结果已实测错误。

最稳路线是：

```text
旧 C 单输出 CARLA 组件
  + GAASD 基础算术模块
  + 单输出自定义决策函数
  + adapter.so
  + 当前 Bridge
  + CARLA 本地场景
```

这条路线和已经跑通的 ACC 基础模块闭环最接近，改动最少，也最容易定位新版 GAASD 的 bug。

