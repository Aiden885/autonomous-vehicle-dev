# ACC/LKS 状态总结与三周工作计划

## 1. 本阶段方向

本阶段作出以下调整：

1. CARLA/Pangu 边界可以复用最新源码组件已经确认的输入输出端口，避免重新设计接口。
2. 验证“手动画布 -> GAASD 生成代码 -> 编译运行 -> CARLA 闭环”。

## 2. 当前实际状态

### 2.1 已经完成并可以复用的部分


| 项目           | 当前状态                                              | 结论                         |
| ------------ | ------------------------------------------------- | -------------------------- |
| ACC 手动画布     | `newaccpro3` 中已有 89 个工程组件，包含决策和目标速度基础模块网络         | 核心功能链可以复用，但层级和命名需要重新整理     |
| LKS 手动画布     | `lks2` 中已有 118 个工程组件，包含决策、预瞄、三点误差、横向加速度限幅和转向控制    | 核心功能链可以复用，但需重构层级、命名并替换常量输入 |
| ACC 接口       | 输入和输出字段已经确认                                       | 可以冻结，不再扩展非必要字段             |
| LKS 接口       | 输入和输出字段已经确认                                       | 可以冻结，不再扩展非必要字段             |
| CARLA Bridge | `5701` 状态发布、`5702` 控制接收、驾驶输入转发均已实现                | 可以直接复用                     |
| Pangu 边界节点   | ACC/LKS 的 `ZmqBridgeModule`、Protobuf 通道和模块封装已经跑通过 | 可以作为手动画布代码的运行外壳            |
| ACC 场景       | Town01、waypoint PID 前车和路线车距已经具备                   | 可以继续做纵向闭环                  |
| LKS 场景       | Town04 连续缓弯、车道多项式、驾驶接管和数据记录已经具备                   | 可以继续做横向闭环                  |
| 调试平台         | 能启动场景、发送按键、修改 LKS 参数、显示日志和服务状态                    | 可以继续作为人工检查入口               |




### 2.2 目前仍未完成的部分

1. **画布结构还不够清晰。** 现有工程是按功能逐步叠加形成的，部分本应属于不同物理环节的模块仍处于同一层。需要按“车上输入 / 环境感知 / 决策 / 控制 / 输出”重新整理，并将每个复合模块限定为一类明确的物理职责。
2. **内部模块命名还不够精确。** 现有内部模块仍有操作类名称和编号式名称，无法直接表达物理含义。顶层保持 `MainFlow`、`Decision` 和 `Control` 等简洁名称，下层复合模块和基础模块实例按对应物理量或功能精确命名。
3. `newaccpro3` 顶层目前仍直接使用 `CARLAACCEgoSpeed`、`CARLAACCLeadSpeed`、
  `CARLAACCLeadDistance`、`CARLAACCDriverCommand` 和 `CARLAACCLongitudinalCmd`，
   尚未整理为最新的车上信息输入、感知输入、ACC 主流程和输出边界。
4. `lks2` 顶层的 `egoV`、`c0~c3`、`curvature`、`brakePressed`、
  `driverSteerNorm` 仍有常量占位，尚未全部接入最新车上信息和感知输入边界。
5. 两个手动画布虽然都曾生成代码，但此前都出现过代码生成器缺陷，需要在当前 GAASD 版本下重新建立未使用历史补丁的生成结果。
6. ACC 的真值表、局部状态和参数需重新检查生成结果；LKS 的全局参数、`atan/fmin/fmax` 和复合组件端口也需在当前版本下重新检查。



### 2.3 当前成熟度判断


| 层级             | ACC           | LKS           |
| -------------- | ------------- | ------------- |
| 算法公式和功能定义      | 已确定           | 已确定           |
| 物理层级和模块职责      | 待重构           | 待重构           |
| 模块、端口和状态命名     | 待统一           | 待统一           |
| 最新边界接口接入       | 待整理           | 待接入           |
| GAASD 当前版本生成代码 | 待重新验证         | 待重新验证         |
| 离线/mock 测试     | 历史版本已通过       | 历史版本已通过       |
| 独立 CARLA 闭环    | 已跑通，需按手工新基线复测 | 已跑通，需按手工新基线复测 |




## 3. 手动画布目标结构



### 3.1 ACC 顶层

```text
VehicleInputChannel                PerceptionInputChannel
egoV / commandType                 leadV / distance
             └───────────────┬───────────────┘
                             ↓
                          MainFlow
                 ┌────────────┴────────────┐
                 ↓                         ↓
              Decision                 Control
                 └────────────┬────────────┘
                              ↓
                    targetSpeed / enable
```

ACC 按物理职责拟整理为以下层级：


| 父模块                  | 子模块                     | 物理职责                           |
| -------------------- | ----------------------- | ------------------------------ |
| `Decision`           | `ControlStateUpdate`    | 统一管理系统状态、决策结果和跨周期记忆。           |
| `ControlStateUpdate` | `SystemStateClassifier` | 根据车速、上一周期在控状态和历史状态判定 ACC 系统状态。 |
| `ControlStateUpdate` | `DecisionTable`         | 根据系统状态和驾驶指令输出决策编号。             |
| `ControlStateUpdate` | `ControlMemoryUpdate`   | 计算下一周期的在控状态。                   |
| `ControlStateUpdate` | `HistoryUpdate`         | 维护是否存在可恢复的历史巡航状态。              |
| `Decision`           | `TimeGapUpdate`         | 按决策指令调整目标时距。                   |
| `Decision`           | `MaxSpeedUpdate`        | 按决策指令调整巡航速度上限。                 |
| `Control`            | `DesiredDistance`       | 由自车速度和目标时距计算期望跟车距离。            |
| `Control`            | `DistanceError`         | 计算实际前车距离与期望距离之差。               |
| `Control`            | `RelativeSpeed`         | 计算前车与自车的相对速度。                  |
| `Control`            | `RawTargetSpeed`        | 合成前车速度、车距误差和相对速度项。             |
| `Control`            | `TargetSpeedLimiter`    | 将目标速度限制在可用范围内。                 |
| `Control`            | `EnableGate`            | 根据 ACC 在控状态决定最终输出。             |




### 3.2 LKS 顶层

```text
VehicleInputChannel                PerceptionInputChannel
egoV / brakePressed /              c0 / c1 / c2 / c3 / curvature
driverSteerNorm
             └───────────────┬───────────────┘
                             ↓
                          MainFlow
                 ┌────────────┴────────────┐
                 ↓                         ↓
               Decision                    Control
                 └────────────┬────────────┘
                              ↓
                   steerRad / controlEnabled
```

LKS 按物理职责拟整理为以下层级：


| 父模块                   | 子模块                     | 物理职责                     |
| --------------------- | ----------------------- | ------------------------ |
| `Decision`            | `SpeedEnableCheck`      | 判断车速是否达到 LKS 工作条件。       |
| `Decision`            | `BrakeExitCheck`        | 判断制动输入是否要求 LKS 退出。       |
| `Decision`            | `DriverOverrideCheck`   | 判断驾驶员主动转向是否构成接管。         |
| `Decision`            | `ControlEnableDecision` | 汇总三类条件并输出控制使能。           |
| `Control`             | `PreviewDistance`       | 根据自车速度和道路曲率计算远预瞄距离。      |
| `Control`             | `LaneErrorEvaluation`   | 统一管理三个预瞄位置的车道横向误差计算。     |
| `LaneErrorEvaluation` | `NearPreviewError`      | 计算近预瞄点误差。                |
| `LaneErrorEvaluation` | `MiddlePreviewError`    | 计算中预瞄点误差。                |
| `LaneErrorEvaluation` | `FarPreviewError`       | 计算远预瞄点误差。                |
| `Control`             | `ErrorWeightedSum`      | 按权重融合近、中、远三点误差。          |
| `Control`             | `SteerCommand`          | 将加权误差转换为最终转角命令。          |
| `SteerCommand`        | `RawSteerCalculation`   | 计算未限制的原始转向命令。            |
| `SteerCommand`        | `LateralAccelLimit`     | 根据车速、轴距和横向加速度上限计算动态转角边界。 |
| `SteerCommand`        | `SteerEnableGate`       | 执行转角限制和控制使能门控。           |




### 3.3 命名整理原则

1. ACC 和 LKS 位于两个独立工程中，自定义复合模块不再使用 `Acc` 或 `Lks` 前缀，顶层统一使用 `VehicleInputChannel`、`PerceptionInputChannel`、`MainFlow`、`Decision` 和 `Control`。
2. 名称表达物理功能或输出物理量，不使用 `Module1`、`Eval_P1`、`multiply_2` 等编号式实例名。
3. GAASD 官方基础组件的类型名保持原样，但其画布实例名按物理作用命名，例如 `DesiredDistanceProduct`、`DistanceError`、`SpeedSquared` 和 `SteerLimitRad`。
4. 外部协议字段、边界组件端口和手动画布内部端口使用同一名称，不再进行二次改名。
5. 端口统一使用简短的小驼峰物理量名称，不添加模块前缀、通信方式后缀或单位后缀；单位只写在接口元数据中。
6. 同一物理量贯穿不同层级时不重复改名；画布产生的中间量使用 `desiredDistance`、`distanceError`、`relativeSpeed`、`previewDistance`、`weightedError` 等直观名称。
7. 重命名前先建立“原名称 -> 新名称 -> 物理含义 -> 上下游端口”对照表，每次只修改一个子系统并检查原有连线。



## 4. 统一接口和端口命名

表中“统一名称”同时用于 CARLA/Bridge 协议字段、Pangu 边界组件端口和手动画布内部端口。信号进入 `MainFlow`、`Decision` 或 `Control` 后仍沿用该名称；单位由接口元数据表达，不写入变量名。

### 4.1 ACC


| 统一名称          | 类型       | 单位  | 方向                 | 物理含义            |
| ------------- | -------- | --- | ------------------ | --------------- |
| `egoV`        | `double` | m/s | CARLA/Pangu -> ACC | 自车纵向速度          |
| `leadV`       | `double` | m/s | CARLA/Pangu -> ACC | 前车纵向速度          |
| `distance`    | `double` | m   | CARLA/Pangu -> ACC | 自车至前车的跟车距离      |
| `commandType` | `int`    | 1   | 驾驶输入 -> ACC        | ACC 驾驶指令类型      |
| `targetSpeed` | `double` | m/s | ACC -> CARLA/Pangu | ACC 目标速度        |
| `enable`      | `int`    | 1   | ACC -> CARLA/Pangu | ACC 控制使能，取值 0/1 |


`commandType` 保持单周期事件语义：非零指令只保持一个计算周期，下一周期恢复为 `0`。

### 4.2 LKS


| 统一名称              | 类型       | 单位   | 方向                 | 物理含义            |
| ----------------- | -------- | ---- | ------------------ | --------------- |
| `egoV`            | `double` | m/s  | CARLA/Pangu -> LKS | 自车纵向速度          |
| `c0`              | `double` | m    | CARLA/Pangu -> LKS | 车道中心线多项式常数项     |
| `c1`              | `double` | 1    | CARLA/Pangu -> LKS | 车道中心线多项式一阶项     |
| `c2`              | `double` | 1/m  | CARLA/Pangu -> LKS | 车道中心线多项式二阶项     |
| `c3`              | `double` | 1/m² | CARLA/Pangu -> LKS | 车道中心线多项式三阶项     |
| `curvature`       | `double` | 1/m  | CARLA/Pangu -> LKS | 道路曲率            |
| `brakePressed`    | `int`    | 1    | 驾驶输入 -> LKS        | 制动踏板状态，取值 0/1   |
| `driverSteerNorm` | `double` | 1    | 驾驶输入 -> LKS        | 驾驶员转向输入归一化值     |
| `steerRad`        | `double` | rad  | LKS -> CARLA/Pangu | LKS 转角命令        |
| `controlEnabled`  | `int`    | 1    | LKS -> CARLA/Pangu | LKS 控制使能，取值 0/1 |


`driverSteerNorm` 必须表示驾驶员真实输入，不能使用 LKS 执行后的车辆实际转角。

## 5. 三周实施计划



### 第 1 周：基线恢复与 ACC 手动画布重构



#### 本周重点

先保护现有成果，完成公共接口和命名基线整理，并在此基础上重构 ACC 手动画布、检查固定输入计算和代码生成结果。

#### 主要工作

1. 复制 `newaccpro3` 和 `lks2` 为新工作副本，保留原工程并记录工程文件、数据库和组件数量。
2. 逐层检查 ACC 复合组件，列出现有端口、参数、状态和主要连线，并标记需要保留、拆分、合并和移动的模块。
3. 建立 ACC 原名称、新名称、物理含义和上下游端口对照表，统一协议字段、Pangu 边界端口和画布端口名称，检查类型、单位、方向及上下游连线。
4. 整理顶层 `VehicleInputChannel`、`PerceptionInputChannel`、`MainFlow`、`Decision`、`Control` 和输出端口。
5. 在 `Decision` 中区分系统状态分类、决策表、在控记忆、历史状态、时距更新和限速更新。
6. 在 `Control` 中区分期望距离、距离误差、相对速度、原始目标速度、目标速度限制和使能门控。
7. 按物理含义整理复合模块和基础模块实例名，检查重构前后的连线、参数默认值和跨周期状态，并用固定输入对照期望距离、距离误差、相对速度和目标速度计算。
8. 增加必要观测信号，尝试从手动画布生成代码和编译，记录画布语义、原始代码和报错位置。



#### 预期阶段结果

- ACC/LKS 手动画布工作副本。
- ACC 现有结构清单、目标层级图和物理职责对照表。
- 层级和命名更清晰的 ACC 手动画布。
- ACC 顶层边界连线表、内部模块命名表及参数和状态清单。
- ACC 固定输入计算对照记录。
- ACC 首轮代码生成和编译问题记录。



### 第 2 周：LKS 手动画布重构与生成检查



#### 本周重点

完成 LKS 结构和命名基线梳理，并在保留已确定的三点预瞄和转向限制公式的基础上，将 LKS 整理为输入、决策、控制和输出职责清晰的手动画布。

#### 主要工作

1. 逐层检查 LKS 复合组件，列出现有端口、参数、状态和主要连线，并标记需要保留、拆分、合并和移动的模块。
2. 建立 LKS 原名称、新名称、物理含义和上下游端口对照表，统一协议字段、Pangu 边界端口和画布端口名称，检查类型、单位、方向及上下游连线。
3. 整理顶层 `VehicleInputChannel`、`PerceptionInputChannel`、`MainFlow`、`Decision`、`Control` 和输出端口。
4. 将 `egoV/c0~c3/curvature/brakePressed/driverSteerNorm` 常量占位替换为真实边界端口。
5. 在 `Decision` 中将低速判断、制动退出、驾驶员转向接管和使能汇总分成职责独立的子模块。
6. 在 `Control` 中划分预瞄距离、三点车道误差、误差加权、原始转向、横向加速度约束和使能门控。
7. 将 `Eval_P1/P2/P3` 等编号式名称替换为可直接表达近、中、远预瞄点的名称，并检查参数名、单位、默认值和上下游连线。
8. 用固定直道和弯道多项式检查预瞄距离、三点误差、加权误差和转角，并尝试生成代码和编译。



#### 预期阶段结果

- LKS 现有结构清单、目标层级图和物理职责对照表。
- 层级和命名更清晰的 LKS 手动画布。
- LKS 顶层边界连线表、参数表和内部模块命名表。
- LKS 固定输入计算对照记录。
- LKS 首轮代码生成和编译问题记录。



### 第 3 周：生成代码修正、Pangu/CARLA 联调与数据复测





#### 本周重点

将前两周形成的 ACC/LKS 手动画布接入实际运行链路，集中处理生成代码、Pangu 周期运行、CARLA 场景效果和数据记录中暴露的问题。

#### 主要工作

1. 处理 ACC 首轮生成和编译问题，对已定位的生成器缺陷做最小修正。
2. 在 mock 链路中检查 ACC 的 `systemState/decision/enable/targetSpeed` 周期变化，再接入 Town01 跟车场景检查驾驶指令和纵向效果。
3. 处理 LKS 首轮生成和编译问题，在 mock 链路中检查预瞄距离、加权误差、动态转角边界和使能状态。
4. 接入 Town04 连续弯道场景，检查转向方向、车道偏差、转角限制以及驾驶接管和制动退出。
5. 对 ACC/LKS 问题修正后重复关键工况，比较画布固定输入、生成代码 mock 输出和 CARLA 数据。
6. 更新调试平台入口，保存工程快照、参数版本和问题清单，并梳理下一阶段唯一控制出口的接口需求。



#### 预期阶段结果

- ACC/LKS 手动画布的生成代码、编译结果和问题对照记录。
- ACC/LKS mock 周期测试记录。
- ACC 跟车和 LKS 弯道场景数据及分析摘要。
- 重构后工程快照、参数版本和 GAASD 生成问题清单。
- ACC+LKS 后续组合时的唯一 `control_cmd` 出口需求。

