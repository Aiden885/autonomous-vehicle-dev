# GAASD ACC 完整决策控制画布方案

## 1. 路线修正

ACC 画布主线应当尽可能使用 GAASD 基础模块搭建，不能把内部决策逻辑整体封装成自定义 C 组件。自定义组件只保留在 CARLA 通信边界，用来完成 GAASD 与 Bridge/adapter 的数据交换。

新的分层原则如下：

| 层级 | 实现方式 | 说明 |
| --- | --- | --- |
| CARLA 输入边界 | 自定义组件 | 从 Bridge 读取 `egoV`、`leadV`、`distance` |
| ACC 决策层 | GAASD 基础模块 | 用比较、逻辑、真值表或一热逻辑网络表达 S0-S3、R1-R8 |
| ACC 控制层 | GAASD 基础模块 | 用加减乘除、限幅、状态变量搭建时距控制 |
| CARLA 输出边界 | 自定义组件 | 把 `targetSpeed` 和 `enable` 发送给 Bridge |

上一版生成的 `gaasd_acc_full_decision_components` 组件包只作为备用验证件，不作为当前主线方案。它可以用于排查某个决策函数的正确性，但不建议导入正式 ACC 画布，否则会弱化 GAASD 基础模块建模的目标。

## 2. 已确认可用的基础模块

根据 `project/accpro2/data/temp.db` 和新版 GAASD 安装目录 `preload/defaultComponent/operatorComponent` 反查，当前可用基础模块包括：

| 模块 | 用途 |
| --- | --- |
| `constant` | 常量输入，例如目标时距、增益、驾驶指令 |
| `read-local-state` / `write-local-state` | 新版局部状态读写，保存 `timeGap`、`maxSpeed`、`hasHistory` 等跨周期状态 |
| `static-variable` | 旧版跨周期状态保存方式；新版中作为状态组件不可用时的兜底 |
| `add` / `subtract` / `multiply` / `divide` | 基础算术 |
| `less-than` / `less-equal` / `greater-than` / `greater-equal` | 比较判断 |
| `equal` / `not-equal` | 状态和指令判等 |
| `logic-and` / `logic-or` / `logic-not` | 组合条件 |
| `fmin` / `fmax` | 限幅，可实现 `min/max/limit` |
| `truth-table` | 决策表表达，作为 ACC 决策层主线模块 |
| `oscilloscope` | 观测 `egoV`、`leadV`、`distance`、`targetSpeed`、`y` |

当前 `accpro2` 已实际使用过 `constant`、`subtract`、`multiply`、`add`、`less-than`、`logic-not`、`logic-and`、`static-variable`、`truth-table`、`oscilloscope`，说明这些模块至少能在画布中拖拽、连线和保存。新版 GAASD 示例组件 `PidController` 已使用 `read-local-state/write-local-state` 读写 `state_.iSum`、`state_.ePrev`，因此完整 ACC 方案应优先使用新版状态组件表达跨周期状态。

## 3. CARLA 边界组件

继续复用已跑通的最小 CARLA 组件包：

`tools/carla_bridge/gaasd_p0_acc_min_components/`

主线只需要以下 4 个自定义组件：

| 组件 | 方向 | 作用 |
| --- | --- | --- |
| `CARLAACCEgoSpeed` | CARLA -> GAASD | 输出自车速度 `egoV`，单位 m/s |
| `CARLAACCLeadSpeed` | CARLA -> GAASD | 输出前车速度 `leadV`，单位 m/s |
| `CARLAACCLeadDistance` | CARLA -> GAASD | 输出前车距离 `distance`，单位 m |
| `CARLAACCLongitudinalCmd` | GAASD -> CARLA | 输入 `speed`、`enable`，其中 `speed` 端口连接 ACC 目标速度 |

不建议在正式画布中继续使用 `CARLAACCComputeTargetSpeed`。目标速度计算应改用基础模块搭建。

## 4. 控制层画布

本阶段假设前车一直存在，不做无前车定速巡航，只做时距跟车。

基础公式：

```text
desiredDistance = max(minDistance, egoV * timeGap)
distanceError   = distance - desiredDistance
relativeSpeed   = leadV - egoV
rawTargetSpeed  = leadV + kDist * distanceError + kSpeed * relativeSpeed
targetSpeed     = min(maxSpeed, max(0, rawTargetSpeed))
```

推荐初始参数：

| 参数 | 建议值 | 说明 |
| --- | --- | --- |
| `timeGap` | `1.8` | 目标时距，单位 s |
| `minDistance` | `5.0` | 最小跟车距离，单位 m |
| `kDist` | `0.25` | 距离误差增益 |
| `kSpeed` | `0.4` | 相对速度增益 |
| `maxSpeed` | `5.0` | 测试场景速度上限，单位 m/s |

冷启动注意：

如果后续接入完整 `enable` 门控和 ACC 状态机，自车从 `egoV=0` 静止起步时可能被低速状态 S3 锁住。旧版 `accpro2` 能从 0 起步，是因为 `enable` 固定为 1；完整决策方案一旦接入 `enable`，必须显式处理冷启动。

建议分两种模式：

| 模式 | 设置 | 适用场景 |
| --- | --- | --- |
| 文档一致模式 | `vMin=0.5 m/s`，S3 不允许 ACC 接管 | 验证 Word 文档状态机，场景需要让 ego 初始速度高于 `vMin` |
| CARLA 测试模式 | `vMin=0.0 m/s`，避免静止时进入 S3 | 当前 CARLA 从静止起步的闭环测试 |

在 CARLA 测试模式下，还要避免 R5 在零速时把 `maxSpeed` 覆盖成 0，具体见 6.3 节。

推荐基础模块连线：

```text
egoV -> multiply(egoV, timeGap) -> fmax(minDistance, egoV*timeGap) -> desiredDistance

distance -> subtract(distance, desiredDistance) -> distanceError
leadV, egoV -> subtract(leadV, egoV) -> relativeSpeed

distanceError -> multiply(distanceError, kDist) -> distTerm
relativeSpeed -> multiply(relativeSpeed, kSpeed) -> speedTerm

leadV + distTerm + speedTerm -> rawTargetSpeed
rawTargetSpeed -> fmax(0, rawTargetSpeed) -> nonNegativeTarget
nonNegativeTarget -> fmin(maxSpeed, nonNegativeTarget) -> targetSpeed

targetSpeed -> CARLAACCLongitudinalCmd.speed
enable -> CARLAACCLongitudinalCmd.enable
```

这是当前最优先保留和验证的基础模块链路。

## 5. 决策层画布

### 5.1 驾驶指令

先用 `constant` 或后续输入组件提供 `commandType`。当前主线把 `commandType` 定义为“驾驶指令事件脉冲”，不是持续电平状态。

| 值 | 指令 |
| --- | --- |
| `0` | 无指令 |
| `1` | 降速，待命无历史时表示当前速度启控 |
| `2` | 增速，待命有历史时表示继承历史启控 |
| `3` | 降距 |
| `4` | 增距 |
| `5` | 油门 |
| `6` | 刹车 |
| `7` | 取消 |

指令触发方式：

```text
无指令周期：commandType = 0
有指令周期：commandType = 1..7，只保持 1 个 GAASD 仿真周期
下一周期：commandType 回到 0
```

这样真值表可以直接使用 `commandType`，不需要在画布里再搭 `lastCommandType`、`cmdChanged`、`cmdPulse` 这一组边沿检测模块。`commandType=0` 时真值表输出 `y=0`，后级不更新 `timeGap/maxSpeed` 等参数；`commandType` 非零的单个周期才触发一次 R1-R8。

注意：如果后续输入源不能保证“非零指令只持续一个仿真周期”，例如长期把常量 `commandType` 设成 `1/2/3/4`，仍会重复触发调参动作。那种情况下再补边沿检测模块。当前画布先按单周期脉冲输入设计，减少核心链路复杂度。

### 5.2 状态计算

不建议直接维护 `systemState` 一个状态变量。更稳的方式是每个周期用基础模块重新合成状态，避免低速保护被状态记忆绕过。

基础状态量：

| 状态量 | 来源 |
| --- | --- |
| `controlEnabled` | 上一周期 `enable`，用局部状态 `controlEnabled` 保存 |
| `hasHistory` | 是否有历史目标，用局部状态 `hasHistory` 保存 |
| `egoV` | `CARLAACCEgoSpeed` |
| `vMin` | 常量，例如 `0.5 m/s` |

基础判断：

```text
isLowSpeed = egoV < vMin
isActive   = controlEnabled
isStandby  = not(controlEnabled)
hasHist    = hasHistory
noHist     = not(hasHistory)

S3 = isLowSpeed
S0 = not(isLowSpeed) and isActive
S1 = not(isLowSpeed) and isStandby and hasHist
S2 = not(isLowSpeed) and isStandby and noHist
```

如需生成数值型 `systemState` 给真值表或示波器：

```text
systemState = 0*S0 + 1*S1 + 2*S2 + 3*S3
```

由于 `S0/S1/S2/S3` 是互斥条件，以上可用 `multiply` 和 `add` 搭出。

### 5.3 决策输出 R1-R8

当前以真值表为主线实现方式。

方式 A：真值表组件，主线方案。

这是最符合 Word 文档决策表表达的方式，应作为正式画布中的决策模块。当前已发现 GAASD 真值表生成代码存在条件包装错误，但该问题属于 GAASD 工具链 bug，不改变画布设计方向。画布应先按真值表搭好，等团队修复后直接生成运行。

真值表输出端口按旧版 GAASD 的习惯直接命名为 `y`，不再额外引入 `decisionEvent` 或 `activeDecisionY`。`y` 的推荐取值定义如下：

| 输出值 | 含义 |
| --- | --- |
| `0` | 无新动作，保持当前控制参数 |
| `1` | R1 降速 |
| `2` | R2 增速 |
| `3` | R3 降距 |
| `4` | R4 增距 |
| `5` | R5 无继控制 |
| `6` | R6 继承控制 |
| `7` | R7 扭矩仲裁 |
| `8` | R8 待命 |

旧版 `accpro2` 的真值表组件实际也是这种结构：输入端口为 `u/v`，输出端口为 `y`，动作直接写 `y=1` 到 `y=8`。因此画布层不需要把真值表输出再包装成另一个“决策事件”变量。

需要注意的是：完整 ACC 决策用于闭环控制时，建议补充 `y=0` 表示“无新动作/保持参数”。原因是 `S0 + commandType=0` 表示“继续保持控制”，不是“重复执行上一动作”。如果无新指令时输出上一动作编号，例如上一动作是 R3，那么后级 `y==3` 会在每个周期继续触发降距，导致 `timeGap` 持续减小。

推荐真值表输入先保持 2 个：

```text
systemState, commandType -> y
```

原因是旧版 `accpro2` 已验证过 2 输入真值表。当前主线不需要 3 输入真值表。

方式 B：基础逻辑块一热编码，验证和兜底方案。

该方式用于在真值表 bug 修复前核对决策逻辑，或在需要排查问题时临时替代真值表。它不作为最终主线。用 `equal` 判断状态和指令，用 `logic-and/or/not` 组合条件，再用 `multiply/add` 得到 `y`。

核心条件：

```text
isCmd0 = commandType == 0
isCmd1 = commandType == 1
isCmd2 = commandType == 2
isCmd3 = commandType == 3
isCmd4 = commandType == 4
isCmd5 = commandType == 5
isCmd6 = commandType == 6
isCmd7 = commandType == 7
```

决策条件：

```text
keepLast = S0 and isCmd0
R1 = S0 and isCmd1
R2 = S0 and isCmd2
R3 = S0 and isCmd3
R4 = S0 and isCmd4
R7 = S0 and isCmd5
R5 = (S1 and isCmd1) or (S2 and isCmd1)
R6 = S1 and isCmd2
R8 = not(keepLast or R1 or R2 or R3 or R4 or R5 or R6 or R7)
```

数值输出：

```text
y =
    keepLast * 0
  + R1 * 1
  + R2 * 2
  + R3 * 3
  + R4 * 4
  + R5 * 5
  + R6 * 6
  + R7 * 7
  + R8 * 8
```

后续参数更新直接使用 `y` 生成布尔条件，例如 `R3 = (y == 3)`、`R4 = (y == 4)`。这不是额外的“事件解码层”，只是 GAASD 基础模块把数值型真值表输出拆给不同参数更新分支使用。

## 6. 决策结果对控制参数的影响

### 6.1 控制使能

用基础逻辑块计算：

```text
enable =
    (S0 and not(isCmd6 or isCmd7))
 or (S1 and (isCmd1 or isCmd2))
 or (S2 and isCmd1)
```

该 `enable` 一路连接到：

```text
CARLAACCLongitudinalCmd.enable
WS_ControlEnabled.controlEnabled
```

### 6.2 目标时距更新

文档中 R3/R4 分别表示降距/增距。这里直接用基础比较块生成 `R3 = (y == 3)`、`R4 = (y == 4)`。

用基础模块表达：

```text
timeGapRaw = timeGap + timeGapStep * R4 - timeGapStep * R3
timeGapNext = min(maxTimeGap, max(minTimeGap, timeGapRaw))
```

推荐参数：

| 参数 | 值 |
| --- | --- |
| `timeGapStep` | `0.2 s` |
| `minTimeGap` | `1.0 s` |
| `maxTimeGap` | `5.0 s` |

`timeGapNext` 回写到 `WS_TimeGap.timeGap`，下一周期由 `RS_TimeGap.timeGap` 读出并连接到控制层 `desiredDistance` 计算。

### 6.3 速度上限更新

R1/R2 分别表示降速/增速，R5 表示当前速度启控。这里直接用基础比较块生成 `R1 = (y == 1)`、`R2 = (y == 2)`、`R5 = (y == 5)`。

```text
speedCandidate = maxSpeed - speedStep * R1 + speedStep * R2
speedClamped = min(maxSpeedCap, max(minSpeed, speedCandidate))
maxSpeedNext = R5 * egoV + (1 - R5) * speedClamped
```

冷启动修正：

如果 ego 从静止开始，R5 按“当前速度启控”会把 `maxSpeedNext` 设为 `0`，导致目标速度被上限锁死。CARLA 测试模式下建议增加一个 R5 捕获条件：

```text
r5CaptureAllowed = R5 and (egoV > r5CaptureMinSpeed)
r5KeepAllowed = R5 and not(r5CaptureAllowed)

speedCandidate = maxSpeed - speedStep * R1 + speedStep * R2
speedClamped = min(maxSpeedCap, max(minSpeed, speedCandidate))
maxSpeedNext =
    r5CaptureAllowed * egoV
  + r5KeepAllowed * maxSpeed
  + not(R5) * speedClamped
```

推荐 `r5CaptureMinSpeed=0.5 m/s`。这样静止起步时 R5 不会把 `maxSpeed` 清零；车辆已有速度后，再按 R5 仍可按文档语义捕获当前速度。

推荐参数：

| 参数 | 值 |
| --- | --- |
| `speedStep` | `1.3889 m/s`，对应 5 km/h |
| `minSpeed` | `0.0 m/s` |
| `maxSpeedCap` | `5.0 m/s`，测试场景先调低 |
| `r5CaptureMinSpeed` | `0.5 m/s` | CARLA 测试模式下避免零速捕获 |

`maxSpeedNext` 回写到 `WS_MaxSpeed.maxSpeed`，下一周期由 `RS_MaxSpeed.maxSpeed` 读出并连接到控制层 `targetSpeed` 限幅。

### 6.4 历史标志和上一决策

历史标志：

```text
hasHistoryNext = hasHistory or enable
```

`hasHistory` 用局部状态保存。决策编号 `y` 不需要回写保存；`y=0` 时各参数保持已有 `timeGap/maxSpeed/controlEnabled` 状态即可。

### 6.5 R7 扭矩仲裁

当前 Bridge 输入是目标速度，不是驾驶员油门扭矩，因此 R7 暂时只做标志输出。这里直接用 `R7 = (y == 7)`：

```text
arbitrationFlag = R7
```

后续如果需要真实扭矩仲裁，需要新增边界输入，例如驾驶员踏板开度、驾驶员目标速度或驱动扭矩。当前 CARLA ACC 跟车验证不需要先实现这部分。

## 7. 推荐搭建顺序

### 第一阶段：保留已跑通的基础控制闭环

目标：确认 CARLA 边界组件和基础运算链路正常。

```text
CARLA 输入 -> 基础运算目标速度 -> CARLAACCLongitudinalCmd
```

这就是 `accpro2/newaccpro2` 的主链路，优先保持不破坏。

### 第二阶段：加入决策观察，不参与控制

目标：把 `S0/S1/S2/S3`、`y`、`enable` 算出来并接示波器，但暂时不控制 `targetSpeed`。

观测信号建议：

```text
egoV, leadV, distance, targetSpeed, systemState, y, enable, timeGap
```

### 第三阶段：逐个接入状态反馈

目标：逐个接入 `enable`、`timeGapNext`、`maxSpeedNext`、`hasHistoryNext`，每接一个反馈量都生成一次代码，检查 GAASD 是否报组合环或拓扑排序错误。

接入顺序：

1. 先接 `enable -> CARLAACCLongitudinalCmd.enable`。
2. 再接 `enable -> WS_ControlEnabled.controlEnabled`。
3. 再接 `Fmin_TimeGapMax.return -> WS_TimeGap.timeGap`，并用 `RS_TimeGap.timeGap -> desiredDistance`。
4. 再接 `Add_MaxSpeedNext.result -> WS_MaxSpeed.maxSpeed`，并用 `RS_MaxSpeed.maxSpeed -> targetSpeed` 限幅。
5. 最后接 `hasHistoryNext`。

新版局部状态读写仍需分阶段验证，不能一次性把多个状态读写全部铺开。若生成代码报错，先断开本阶段新增的 `write-local-state` 写入线，只保留读取和观测。

### 第四阶段：真值表生成修复后进入运行验证

真值表代码生成 bug 修复后，直接使用真值表组件作为主线决策输出。基础逻辑块一热编码只保留为对照验证，不再参与正式控制链路。

```text
systemState, commandType -> y
```

修复前可以先完成真值表配置、画布连线和示波器观测；是否接入控制闭环取决于当时生成代码是否已验证正确。

## 8. 完整画布连线表

本节按新版 GAASD 默认基础模块端口编写。已核对的默认端口如下：

| 模块 | 输入端口 | 输出端口 |
| --- | --- | --- |
| `constant` | 无 | `out` |
| `add` / `subtract` / `multiply` / `divide` | `a`、`b` | `result` |
| `fmax` / `fmin` | `x`、`y` | `return` |
| `less-than` / `greater-than` / `equal` / `not-equal` | `a`、`b` | `result` |
| `logic-and` / `logic-or` | `a`、`b` | `result` |
| `logic-not` | `operand` | `result` |
| `read-local-state` | 无 | 配置字段名，例如 `timeGap` |
| `write-local-state` | 配置字段名，例如 `timeGap` | 同名输出，可不接 |
| `truth-table` | `u`、`v` | `y` |
| `CARLAACCLongitudinalCmd` | `speed`、`enable` | 无 |

旧工程中部分基础模块端口可能显示为 `input_1/input_2/input_3`，含义等价于 `a/b` 或多输入加法。实际连线时以画布显示端口为准。

新版状态组件使用前，需要在当前 ACC 复合画布或模块的 `State` 中先配置以下局部状态字段：

| 状态字段 | 类型 | 初始值 | 用途 |
| --- | --- | --- | --- |
| `controlEnabled` | `int` 或 `bool` | `0` | 保存上一周期 ACC 是否在控 |
| `hasHistory` | `int` 或 `bool` | `0` | 保存是否已有历史启控目标 |
| `timeGap` | `double` | `1.8` | 保存当前目标时距 |
| `maxSpeed` | `double` | `5.0` | 保存当前速度上限 |

对应画布组件配置为：`RS_TimeGap` 读取 `state_.timeGap`，`WS_TimeGap` 写入 `state_.timeGap`；其他状态同理。若新版状态组件暂时因 GAASD bug 不能生成代码，可临时把 `RS_/WS_` 换回旧版 `static-variable`，但正式新版方案优先使用状态组件。

### 8.1 建议先放置的组件实例

| 实例名 | 组件类型 | 建议配置 |
| --- | --- | --- |
| `In_EgoV` | `CARLAACCEgoSpeed` | 输出自车速度 |
| `In_LeadV` | `CARLAACCLeadSpeed` | 输出前车速度 |
| `In_Distance` | `CARLAACCLeadDistance` | 输出前车距离 |
| `Cmd_Longitudinal` | `CARLAACCLongitudinalCmd` | 发布目标速度和使能 |
| `TT_ACCDecision` | `truth-table` | 输入 `systemState/commandType`，输出 `y` |
| `RS_ControlEnabled` | `read-local-state` | 读取局部状态 `controlEnabled`，初始值 `0` |
| `WS_ControlEnabled` | `write-local-state` | 写入局部状态 `controlEnabled` |
| `RS_HasHistory` | `read-local-state` | 读取局部状态 `hasHistory`，初始值 `0` |
| `WS_HasHistory` | `write-local-state` | 写入局部状态 `hasHistory` |
| `RS_TimeGap` | `read-local-state` | 读取局部状态 `timeGap`，初始值 `1.8` |
| `WS_TimeGap` | `write-local-state` | 写入局部状态 `timeGap` |
| `RS_MaxSpeed` | `read-local-state` | 读取局部状态 `maxSpeed`，初始值 `5.0` |
| `WS_MaxSpeed` | `write-local-state` | 写入局部状态 `maxSpeed` |
| `Scope_ACC` | `oscilloscope` | 建议观察 8 个量，端口数量按 GUI 支持情况扩展 |

建议常量实例：

| 实例名 | 值 |
| --- | --- |
| `C_0` | `0` |
| `C_1` | `1` |
| `C_2` | `2` |
| `C_3` | `3` |
| `C_4` | `4` |
| `C_5` | `5` |
| `C_6` | `6` |
| `C_7` | `7` |
| `C_8` | `8` |
| `C_CommandType` | 正常为 `0`；调试事件时只允许单周期设为 `1..7` |
| `C_VMin` | CARLA 测试模式 `0.0`；文档一致模式 `0.5` |
| `C_MinDistance` | `5.0` |
| `C_KDist` | `0.25` |
| `C_KSpeed` | `0.4` |
| `C_TimeGapStep` | `0.2` |
| `C_MinTimeGap` | `1.0` |
| `C_MaxTimeGap` | `5.0` |
| `C_SpeedStep` | `1.3889` |
| `C_MinSpeed` | `0.0` |
| `C_MaxSpeedCap` | `5.0` |
| `C_R5CaptureMinSpeed` | `0.5` |

### 8.2 实例名对应的模块类型

画布里没有这些实例时，需要先从组件库拖入对应模块，再按建议实例名重命名。连线表里的写法例如 `Mul_DesiredDistance.a`，含义是：`Mul_DesiredDistance` 这个乘法模块的输入端口 `a`。

命名前缀约定：

| 前缀 | 模块类型 | 示例 |
| --- | --- | --- |
| `C_` | `constant` 常量 | `C_KDist` |
| `RS_` | `read-local-state` 读取局部状态 | `RS_TimeGap` |
| `WS_` | `write-local-state` 写入局部状态 | `WS_TimeGap` |
| `SV_` | `static-variable` 静态变量兜底 | 仅旧版或状态组件不可用时使用 |
| `In_` | CARLA 输入组件 | `In_EgoV` |
| `Cmd_` | CARLA 输出组件 | `Cmd_Longitudinal` |
| `Mul_` | `multiply` 乘法 | `Mul_DistTerm` |
| `Sub_` | `subtract` 减法 | `Sub_DistanceError` |
| `Add_` | `add` 加法 | `Add_RawTarget` |
| `Fmax_` | `fmax` 浮点最大值 | `Fmax_DesiredDistance` |
| `Fmin_` | `fmin` 浮点最小值 | `Fmin_TargetSpeed` |
| `Less_` | `less-than` 小于判断 | `Less_LowSpeed` |
| `Gt_` | `greater-than` 大于判断 | `Gt_R5CaptureSpeed` |
| `Eq_` | `equal` 等于判断 | `Eq_R1` |
| `Ne_` | `not-equal` 不等于判断 | 当前主线不使用 |
| `And_` | `logic-and` 逻辑与 | `And_EnableS0` |
| `Or_` | `logic-or` 逻辑或 | `Or_Enable` |
| `Not_` | `logic-not` 逻辑非 | `Not_LowSpeed` |
| `TT_` | `truth-table` 真值表 | `TT_ACCDecision` |
| `Scope_` | `oscilloscope` 示波器 | `Scope_ACC` |

基础跟车控制需要新建的基础模块：

| 实例名 | 模块类型 | 输入端口 | 输出端口 | 作用 |
| --- | --- | --- | --- | --- |
| `Mul_DesiredDistance` | `multiply` | `a`、`b` | `result` | 计算 `egoV * timeGap` |
| `Fmax_DesiredDistance` | `fmax` | `x`、`y` | `return` | 计算 `max(minDistance, egoV*timeGap)` |
| `Sub_DistanceError` | `subtract` | `a`、`b` | `result` | 计算 `distance - desiredDistance` |
| `Sub_RelativeSpeed` | `subtract` | `a`、`b` | `result` | 计算 `leadV - egoV` |
| `Mul_DistTerm` | `multiply` | `a`、`b` | `result` | 计算距离控制项 |
| `Mul_SpeedTerm` | `multiply` | `a`、`b` | `result` | 计算相对速度控制项 |
| `Add_TargetPart1` | `add` | `a`、`b` | `result` | 计算 `leadV + distTerm` |
| `Add_RawTarget` | `add` | `a`、`b` | `result` | 计算未限幅目标速度 |
| `Fmax_NonNegativeTarget` | `fmax` | `x`、`y` | `return` | 限制目标速度不小于 0 |
| `Fmin_TargetSpeed` | `fmin` | `x`、`y` | `return` | 限制目标速度不超过 `maxSpeed` |

状态、真值表和使能链路需要新建的基础模块：

| 实例名 | 模块类型 | 输入端口 | 输出端口 | 作用 |
| --- | --- | --- | --- | --- |
| `RS_ControlEnabled` / `WS_ControlEnabled` | `read-local-state` / `write-local-state` | 写：`controlEnabled` | 读：`controlEnabled` | 保存上一周期控制使能 |
| `RS_HasHistory` / `WS_HasHistory` | `read-local-state` / `write-local-state` | 写：`hasHistory` | 读：`hasHistory` | 保存是否有历史目标 |
| `Less_LowSpeed` | `less-than` | `a`、`b` | `result` | 判断 `egoV < vMin` |
| `Not_LowSpeed` | `logic-not` | `operand` | `result` | 得到非低速状态 |
| `Not_ControlEnabled` | `logic-not` | `operand` | `result` | 得到待命状态 |
| `Not_HasHistory` | `logic-not` | `operand` | `result` | 得到无历史状态 |
| `And_S0` | `logic-and` | `a`、`b` | `result` | 计算 S0 在控状态 |
| `And_S1_Pre` | `logic-and` | `a`、`b` | `result` | 计算 S1 中间条件 |
| `And_S1` | `logic-and` | `a`、`b` | `result` | 计算 S1 待命有历史 |
| `And_S2_Pre` | `logic-and` | `a`、`b` | `result` | 计算 S2 中间条件 |
| `And_S2` | `logic-and` | `a`、`b` | `result` | 计算 S2 待命无历史 |
| `Mul_State1` | `multiply` | `a`、`b` | `result` | 生成 S1 数值项 |
| `Mul_State2` | `multiply` | `a`、`b` | `result` | 生成 S2 数值项 |
| `Mul_State3` | `multiply` | `a`、`b` | `result` | 生成 S3 数值项 |
| `Add_State12` | `add` | `a`、`b` | `result` | 合并 S1/S2 数值项 |
| `Add_SystemState` | `add` | `a`、`b` | `result` | 输出 `systemState` |
| `Eq_R1` 到 `Eq_R8` | `equal` | `a`、`b` | `result` | 判断 `y == 1..8` |
| `Eq_Cmd1`、`Eq_Cmd2`、`Eq_Cmd6`、`Eq_Cmd7` | `equal` | `a`、`b` | `result` | 判断驾驶指令 |
| `Or_Cmd67` | `logic-or` | `a`、`b` | `result` | 判断刹车或取消 |
| `Not_Cmd67` | `logic-not` | `operand` | `result` | 判断不是刹车/取消 |
| `Or_Cmd12` | `logic-or` | `a`、`b` | `result` | 判断启控类指令 |
| `And_EnableS0` | `logic-and` | `a`、`b` | `result` | S0 使能分支 |
| `And_EnableS1` | `logic-and` | `a`、`b` | `result` | S1 使能分支 |
| `And_EnableS2` | `logic-and` | `a`、`b` | `result` | S2 使能分支 |
| `Or_Enable01` | `logic-or` | `a`、`b` | `result` | 合并 S0/S1 使能 |
| `Or_Enable` | `logic-or` | `a`、`b` | `result` | 输出总使能 |

参数更新需要新建的基础模块：

| 实例名 | 模块类型 | 输入端口 | 输出端口 | 作用 |
| --- | --- | --- | --- | --- |
| `RS_TimeGap` / `WS_TimeGap` | `read-local-state` / `write-local-state` | 写：`timeGap` | 读：`timeGap` | 保存目标时距 |
| `RS_MaxSpeed` / `WS_MaxSpeed` | `read-local-state` / `write-local-state` | 写：`maxSpeed` | 读：`maxSpeed` | 保存目标速度上限 |
| `Mul_TimeGapUp`、`Mul_TimeGapDown` | `multiply` | `a`、`b` | `result` | 生成增距/降距项 |
| `Add_TimeGapUp` | `add` | `a`、`b` | `result` | 当前时距加增距项 |
| `Sub_TimeGapRaw` | `subtract` | `a`、`b` | `result` | 减去降距项 |
| `Fmax_TimeGapMin` | `fmax` | `x`、`y` | `return` | 时距下限 |
| `Fmin_TimeGapMax` | `fmin` | `x`、`y` | `return` | 时距上限 |
| `Mul_SpeedDown`、`Mul_SpeedUp` | `multiply` | `a`、`b` | `result` | 生成降速/增速项 |
| `Sub_MaxSpeedDown` | `subtract` | `a`、`b` | `result` | 当前速度上限减降速项 |
| `Add_MaxSpeedCandidate` | `add` | `a`、`b` | `result` | 加增速项 |
| `Fmax_MaxSpeedMin` | `fmax` | `x`、`y` | `return` | 速度下限 |
| `Fmin_MaxSpeedCap` | `fmin` | `x`、`y` | `return` | 速度上限 |
| `Gt_R5CaptureSpeed` | `greater-than` | `a`、`b` | `result` | 判断 R5 是否允许捕获当前速度 |
| `And_R5CaptureAllowed`、`And_R5KeepAllowed` | `logic-and` | `a`、`b` | `result` | R5 捕获/保持条件 |
| `Not_R5CaptureAllowed`、`Not_R5` | `logic-not` | `operand` | `result` | R5 条件取非 |
| `Mul_R5CaptureTerm`、`Mul_R5KeepTerm`、`Mul_MaxSpeedClampTerm` | `multiply` | `a`、`b` | `result` | 生成速度上限三路候选项 |
| `Add_MaxSpeedNextA`、`Add_MaxSpeedNext` | `add` | `a`、`b` | `result` | 合成最终 `maxSpeedNext` |
| `Or_HasHistoryNext` | `logic-or` | `a`、`b` | `result` | 更新历史标志 |

### 8.3 第一阶段：基础跟车控制闭环

这一阶段不接真值表和完整状态机，`enable` 可以先用常量 `1`，用于保留已跑通的 ACC 跟车链路。

| 连线 | 源端口 | 目标端口 | 含义 |
| --- | --- | --- | --- |
| 1 | `In_EgoV.egoV` | `Mul_DesiredDistance.a` | 自车速度参与期望距离 |
| 2 | `RS_TimeGap.timeGap` | `Mul_DesiredDistance.b` | 当前目标时距 |
| 3 | `C_MinDistance.out` | `Fmax_DesiredDistance.x` | 最小距离 |
| 4 | `Mul_DesiredDistance.result` | `Fmax_DesiredDistance.y` | `egoV * timeGap` |
| 5 | `In_Distance.distance` | `Sub_DistanceError.a` | 实际距离 |
| 6 | `Fmax_DesiredDistance.return` | `Sub_DistanceError.b` | 期望距离 |
| 7 | `In_LeadV.leadV` | `Sub_RelativeSpeed.a` | 前车速度 |
| 8 | `In_EgoV.egoV` | `Sub_RelativeSpeed.b` | 自车速度 |
| 9 | `Sub_DistanceError.result` | `Mul_DistTerm.a` | 距离误差 |
| 10 | `C_KDist.out` | `Mul_DistTerm.b` | 距离误差增益 |
| 11 | `Sub_RelativeSpeed.result` | `Mul_SpeedTerm.a` | 相对速度 |
| 12 | `C_KSpeed.out` | `Mul_SpeedTerm.b` | 相对速度增益 |
| 13 | `In_LeadV.leadV` | `Add_TargetPart1.a` | 前车速度前馈 |
| 14 | `Mul_DistTerm.result` | `Add_TargetPart1.b` | 距离项 |
| 15 | `Add_TargetPart1.result` | `Add_RawTarget.a` | 中间目标速度 |
| 16 | `Mul_SpeedTerm.result` | `Add_RawTarget.b` | 速度项 |
| 17 | `C_0.out` | `Fmax_NonNegativeTarget.x` | 目标速度下限 |
| 18 | `Add_RawTarget.result` | `Fmax_NonNegativeTarget.y` | 未限幅目标速度 |
| 19 | `RS_MaxSpeed.maxSpeed` | `Fmin_TargetSpeed.x` | 当前速度上限 |
| 20 | `Fmax_NonNegativeTarget.return` | `Fmin_TargetSpeed.y` | 非负目标速度 |
| 21 | `Fmin_TargetSpeed.return` | `Cmd_Longitudinal.speed` | 发送目标速度 |
| 22 | `C_1.out` | `Cmd_Longitudinal.enable` | 第一阶段固定使能 |

第一阶段跑通后，`Fmin_TargetSpeed.return` 就是完整 ACC 画布中的 `targetSpeed`。

### 8.4 第二阶段：驾驶指令脉冲输入

当前方案不在画布内搭边沿检测模块，`C_CommandType.out` 直接作为 `commandType` 使用。

| 场景 | `C_CommandType` 要求 |
| --- | --- |
| 正常跟车、无新指令 | 保持 `0` |
| 单次降速/增速/降距/增距测试 | 只在 1 个 GAASD 仿真周期内设为 `1/2/3/4`，下一周期回到 `0` |
| 长时间运行稳定性测试 | 不允许长期保持 `1/2/3/4`，否则会重复调参 |

如果当前 GAASD 画布只能用常量，不能在运行时产生单周期脉冲，则这一阶段先保持 `C_CommandType=0`，只验证基础跟车和状态机。后续可由 Bridge、UI 或输入组件提供单周期 `commandType`。

### 8.5 第三阶段：状态 S0-S3 计算

| 连线 | 源端口 | 目标端口 | 含义 |
| --- | --- | --- | --- |
| 1 | `In_EgoV.egoV` | `Less_LowSpeed.a` | 自车速度 |
| 2 | `C_VMin.out` | `Less_LowSpeed.b` | 低速阈值 |
| 3 | `Less_LowSpeed.result` | `Not_LowSpeed.operand` | 取非得到非低速 |
| 4 | `RS_ControlEnabled.controlEnabled` | `Not_ControlEnabled.operand` | 取非得到待命 |
| 5 | `RS_HasHistory.hasHistory` | `Not_HasHistory.operand` | 取非得到无历史 |
| 6 | `Not_LowSpeed.result` | `And_S0.a` | 非低速 |
| 7 | `RS_ControlEnabled.controlEnabled` | `And_S0.b` | 上周期在控 |
| 8 | `Not_LowSpeed.result` | `And_S1_Pre.a` | 非低速 |
| 9 | `Not_ControlEnabled.result` | `And_S1_Pre.b` | 待命 |
| 10 | `And_S1_Pre.result` | `And_S1.b` | 待命且非低速 |
| 11 | `RS_HasHistory.hasHistory` | `And_S1.a` | 有历史 |
| 12 | `Not_LowSpeed.result` | `And_S2_Pre.a` | 非低速 |
| 13 | `Not_ControlEnabled.result` | `And_S2_Pre.b` | 待命 |
| 14 | `And_S2_Pre.result` | `And_S2.a` | 待命且非低速 |
| 15 | `Not_HasHistory.result` | `And_S2.b` | 无历史 |

状态定义：

| 状态 | 信号 |
| --- | --- |
| `S0` | `And_S0.result` |
| `S1` | `And_S1.result` |
| `S2` | `And_S2.result` |
| `S3` | `Less_LowSpeed.result` |

生成数值型 `systemState` 的连线：

| 连线 | 源端口 | 目标端口 | 含义 |
| --- | --- | --- | --- |
| 1 | `And_S1.result` | `Mul_State1.a` | S1 |
| 2 | `C_1.out` | `Mul_State1.b` | S1 编码 1 |
| 3 | `And_S2.result` | `Mul_State2.a` | S2 |
| 4 | `C_2.out` | `Mul_State2.b` | S2 编码 2 |
| 5 | `Less_LowSpeed.result` | `Mul_State3.a` | S3 |
| 6 | `C_3.out` | `Mul_State3.b` | S3 编码 3 |
| 7 | `Mul_State1.result` | `Add_State12.a` | S1 数值项 |
| 8 | `Mul_State2.result` | `Add_State12.b` | S2 数值项 |
| 9 | `Add_State12.result` | `Add_SystemState.a` | 中间状态 |
| 10 | `Mul_State3.result` | `Add_SystemState.b` | S3 数值项 |

`Add_SystemState.result` 即 `systemState`，后面接真值表 `u` 端口。

### 8.6 第四阶段：真值表和 R1-R8 条件

真值表输入输出：

| 连线 | 源端口 | 目标端口 | 含义 |
| --- | --- | --- | --- |
| 1 | `Add_SystemState.result` | `TT_ACCDecision.u` | ACC 状态 |
| 2 | `C_CommandType.out` | `TT_ACCDecision.v` | 驾驶指令脉冲 |

真值表推荐配置：

| 条件 | 动作 |
| --- | --- |
| `u==0 && v==0` | `y=0` |
| `u==0 && v==1` | `y=1` |
| `u==0 && v==2` | `y=2` |
| `u==0 && v==3` | `y=3` |
| `u==0 && v==4` | `y=4` |
| `u==0 && v==5` | `y=7` |
| `u==1 && v==1` | `y=5` |
| `u==2 && v==1` | `y=5` |
| `u==1 && v==2` | `y=6` |
| `u==0 && (v==6 || v==7)` | `y=8` |
| 默认 | `y=8` |

从 `y` 生成 R 条件：

| 连线 | 源端口 | 目标端口 | 含义 |
| --- | --- | --- | --- |
| 1 | `TT_ACCDecision.y` | `Eq_R1.a` | 决策输出 |
| 2 | `C_1.out` | `Eq_R1.b` | R1 |
| 3 | `TT_ACCDecision.y` | `Eq_R2.a` | 决策输出 |
| 4 | `C_2.out` | `Eq_R2.b` | R2 |
| 5 | `TT_ACCDecision.y` | `Eq_R3.a` | 决策输出 |
| 6 | `C_3.out` | `Eq_R3.b` | R3 |
| 7 | `TT_ACCDecision.y` | `Eq_R4.a` | 决策输出 |
| 8 | `C_4.out` | `Eq_R4.b` | R4 |
| 9 | `TT_ACCDecision.y` | `Eq_R5.a` | 决策输出 |
| 10 | `C_5.out` | `Eq_R5.b` | R5 |
| 11 | `TT_ACCDecision.y` | `Eq_R6.a` | 决策输出 |
| 12 | `C_6.out` | `Eq_R6.b` | R6 |
| 13 | `TT_ACCDecision.y` | `Eq_R7.a` | 决策输出 |
| 14 | `C_7.out` | `Eq_R7.b` | R7 |
| 15 | `TT_ACCDecision.y` | `Eq_R8.a` | 决策输出 |
| 16 | `C_8.out` | `Eq_R8.b` | R8 |

### 8.7 第五阶段：enable 使能链路

先生成 `isCmd1/isCmd2/isCmd6/isCmd7`。这些信号用 `C_CommandType.out` 和常量 `C_1/C_2/C_6/C_7` 通过 `equal` 连接，方法与 8.6 的 `Eq_R1` 相同。

| 连线 | 源端口 | 目标端口 | 含义 |
| --- | --- | --- | --- |
| 1 | `Eq_Cmd6.result` | `Or_Cmd67.a` | 刹车 |
| 2 | `Eq_Cmd7.result` | `Or_Cmd67.b` | 取消 |
| 3 | `Or_Cmd67.result` | `Not_Cmd67.operand` | 非刹车/取消 |
| 4 | `And_S0.result` | `And_EnableS0.a` | 在控 |
| 5 | `Not_Cmd67.result` | `And_EnableS0.b` | 非退出类指令 |
| 6 | `Eq_Cmd1.result` | `Or_Cmd12.a` | 降速/启控 |
| 7 | `Eq_Cmd2.result` | `Or_Cmd12.b` | 增速/继承启控 |
| 8 | `And_S1.result` | `And_EnableS1.a` | 待命有历史 |
| 9 | `Or_Cmd12.result` | `And_EnableS1.b` | 允许启控指令 |
| 10 | `And_S2.result` | `And_EnableS2.a` | 待命无历史 |
| 11 | `Eq_Cmd1.result` | `And_EnableS2.b` | 当前速度启控 |
| 12 | `And_EnableS0.result` | `Or_Enable01.a` | S0 使能分支 |
| 13 | `And_EnableS1.result` | `Or_Enable01.b` | S1 使能分支 |
| 14 | `Or_Enable01.result` | `Or_Enable.a` | 中间使能 |
| 15 | `And_EnableS2.result` | `Or_Enable.b` | S2 使能分支 |
| 16 | `Or_Enable.result` | `Cmd_Longitudinal.enable` | 输出到 CARLA |
| 17 | `Or_Enable.result` | `WS_ControlEnabled.controlEnabled` | 回写在控状态 |

接入这一阶段后，第一阶段中的 `C_1.out -> Cmd_Longitudinal.enable` 要删除，改用 `Or_Enable.result`。

### 8.8 第六阶段：timeGap 更新链路

| 连线 | 源端口 | 目标端口 | 含义 |
| --- | --- | --- | --- |
| 1 | `C_TimeGapStep.out` | `Mul_TimeGapUp.a` | 时距步长 |
| 2 | `Eq_R4.result` | `Mul_TimeGapUp.b` | 增距 |
| 3 | `C_TimeGapStep.out` | `Mul_TimeGapDown.a` | 时距步长 |
| 4 | `Eq_R3.result` | `Mul_TimeGapDown.b` | 降距 |
| 5 | `RS_TimeGap.timeGap` | `Add_TimeGapUp.a` | 当前时距 |
| 6 | `Mul_TimeGapUp.result` | `Add_TimeGapUp.b` | 增距项 |
| 7 | `Add_TimeGapUp.result` | `Sub_TimeGapRaw.a` | 增距后时距 |
| 8 | `Mul_TimeGapDown.result` | `Sub_TimeGapRaw.b` | 降距项 |
| 9 | `C_MinTimeGap.out` | `Fmax_TimeGapMin.x` | 时距下限 |
| 10 | `Sub_TimeGapRaw.result` | `Fmax_TimeGapMin.y` | 未限幅时距 |
| 11 | `C_MaxTimeGap.out` | `Fmin_TimeGapMax.x` | 时距上限 |
| 12 | `Fmax_TimeGapMin.return` | `Fmin_TimeGapMax.y` | 下限处理后的时距 |
| 13 | `Fmin_TimeGapMax.return` | `WS_TimeGap.timeGap` | 回写目标时距 |

`RS_TimeGap.timeGap` 同时继续连接到 8.3 的 `Mul_DesiredDistance.b`，用状态中保持的时距参与控制。

### 8.9 第七阶段：maxSpeed 更新链路

速度上限候选值：

| 连线 | 源端口 | 目标端口 | 含义 |
| --- | --- | --- | --- |
| 1 | `C_SpeedStep.out` | `Mul_SpeedDown.a` | 速度步长 |
| 2 | `Eq_R1.result` | `Mul_SpeedDown.b` | 降速 |
| 3 | `C_SpeedStep.out` | `Mul_SpeedUp.a` | 速度步长 |
| 4 | `Eq_R2.result` | `Mul_SpeedUp.b` | 增速 |
| 5 | `RS_MaxSpeed.maxSpeed` | `Sub_MaxSpeedDown.a` | 当前速度上限 |
| 6 | `Mul_SpeedDown.result` | `Sub_MaxSpeedDown.b` | 降速项 |
| 7 | `Sub_MaxSpeedDown.result` | `Add_MaxSpeedCandidate.a` | 降速后上限 |
| 8 | `Mul_SpeedUp.result` | `Add_MaxSpeedCandidate.b` | 增速项 |
| 9 | `C_MinSpeed.out` | `Fmax_MaxSpeedMin.x` | 速度下限 |
| 10 | `Add_MaxSpeedCandidate.result` | `Fmax_MaxSpeedMin.y` | 候选速度 |
| 11 | `C_MaxSpeedCap.out` | `Fmin_MaxSpeedCap.x` | 速度上限 |
| 12 | `Fmax_MaxSpeedMin.return` | `Fmin_MaxSpeedCap.y` | 下限处理后速度 |

R5 零速保护和回写：

| 连线 | 源端口 | 目标端口 | 含义 |
| --- | --- | --- | --- |
| 1 | `In_EgoV.egoV` | `Gt_R5CaptureSpeed.a` | 当前速度 |
| 2 | `C_R5CaptureMinSpeed.out` | `Gt_R5CaptureSpeed.b` | 捕获阈值 |
| 3 | `Eq_R5.result` | `And_R5CaptureAllowed.a` | R5 |
| 4 | `Gt_R5CaptureSpeed.result` | `And_R5CaptureAllowed.b` | 速度足够 |
| 5 | `And_R5CaptureAllowed.result` | `Not_R5CaptureAllowed.operand` | 取非 |
| 6 | `Eq_R5.result` | `And_R5KeepAllowed.a` | R5 |
| 7 | `Not_R5CaptureAllowed.result` | `And_R5KeepAllowed.b` | 零速时保持原上限 |
| 8 | `Eq_R5.result` | `Not_R5.operand` | 非 R5 |
| 9 | `And_R5CaptureAllowed.result` | `Mul_R5CaptureTerm.a` | 允许捕获 |
| 10 | `In_EgoV.egoV` | `Mul_R5CaptureTerm.b` | 捕获当前速度 |
| 11 | `And_R5KeepAllowed.result` | `Mul_R5KeepTerm.a` | 零速 R5 保持 |
| 12 | `RS_MaxSpeed.maxSpeed` | `Mul_R5KeepTerm.b` | 原速度上限 |
| 13 | `Not_R5.result` | `Mul_MaxSpeedClampTerm.a` | 非 R5 |
| 14 | `Fmin_MaxSpeedCap.return` | `Mul_MaxSpeedClampTerm.b` | 正常限幅速度 |
| 15 | `Mul_R5CaptureTerm.result` | `Add_MaxSpeedNextA.a` | 捕获项 |
| 16 | `Mul_R5KeepTerm.result` | `Add_MaxSpeedNextA.b` | 保持项 |
| 17 | `Add_MaxSpeedNextA.result` | `Add_MaxSpeedNext.b` | 中间值 |
| 18 | `Mul_MaxSpeedClampTerm.result` | `Add_MaxSpeedNext.a` | 正常更新项 |
| 19 | `Add_MaxSpeedNext.result` | `WS_MaxSpeed.maxSpeed` | 回写速度上限 |

`RS_MaxSpeed.maxSpeed` 同时继续连接到 8.3 的 `Fmin_TargetSpeed.x`。

### 8.10 第八阶段：历史状态和示波器

历史状态：

| 连线 | 源端口 | 目标端口 | 含义 |
| --- | --- | --- | --- |
| 1 | `RS_HasHistory.hasHistory` | `Or_HasHistoryNext.a` | 旧历史标志 |
| 2 | `Or_Enable.result` | `Or_HasHistoryNext.b` | 本周期启控 |
| 3 | `Or_HasHistoryNext.result` | `WS_HasHistory.hasHistory` | 回写历史标志 |

示波器建议先接这些信号。如果示波器端口不够，优先接前 4 个。

| 源端口 | 示波器端口 | 含义 |
| --- | --- | --- |
| `Fmin_TargetSpeed.return` | `Scope_ACC.input_1` | 目标速度 |
| `In_EgoV.egoV` | `Scope_ACC.input_2` | 自车速度 |
| `In_LeadV.leadV` | `Scope_ACC.input_3` | 前车速度 |
| `In_Distance.distance` | `Scope_ACC.input_4` | 前车距离 |
| `TT_ACCDecision.y` | `Scope_ACC.input_5` | 决策输出 |
| `Or_Enable.result` | `Scope_ACC.input_6` | 控制使能 |
| `RS_TimeGap.timeGap` | `Scope_ACC.input_7` | 目标时距 |
| `RS_MaxSpeed.maxSpeed` | `Scope_ACC.input_8` | 速度上限 |

### 8.11 分阶段生成代码检查点

| 阶段 | 生成代码前应达到的状态 | 重点检查 |
| --- | --- | --- |
| 1 | 只接 8.3 基础跟车链路，`enable=1` | `FuncStep.c` 能持续调用 CARLA 输入、目标速度计算、纵向命令 |
| 2 | 加 8.4、8.5、8.6，只接示波器，不接输出控制 | `y/systemState/commandType` 是否按预期变化 |
| 3 | 加 8.7，用 `enable` 替换固定常量 1 | 静止起步是否被 S3 锁住；CARLA 测试模式 `C_VMin=0` |
| 4 | 加 8.8、8.9 | `timeGap/maxSpeed` 是否只在对应指令触发时变化 |
| 5 | 加 8.10 | `hasHistory` 是否按启控后保持 |

如果某一步生成代码报组合环或状态写入错误，先断开本阶段新增的 `write-local-state` 输入写入线，只保留读状态和观测输出，确认问题后再逐条恢复。

## 9. 当前不再推荐的内容

以下内容降级为备用，不作为主线：

| 内容 | 原因 |
| --- | --- |
| `tools/carla_bridge/gaasd_acc_full_decision_components/` | 把 ACC 内部决策封装成自定义组件，偏离基础模块建模目标 |
| `CARLAACCComputeTargetSpeed` | 目标速度公式可用基础块搭建，不应继续封装 |
| 完整 SPPVT 扭矩控制 | 当前 CARLA 闭环只需要目标速度，SPPVT 可以后续单独验证 |
| 无前车定速巡航 | 当前测试假设前车持续存在，先验证时距控制 |

## 10. 下一步

1. 在新版 GAASD 画布中继续基于 `newaccpro2` 或副本搭建，不要导入完整决策自定义包作为主线。
2. 保留 4 个 CARLA 边界组件，删除或不使用 `CARLAACCComputeTargetSpeed`。
3. 用真值表搭建 `y` 主线输出，用基础模块接入 `commandType` 脉冲输入，并补齐 `desiredDistance`、`targetSpeed`、`enable`、`timeGapNext`、`maxSpeedNext`。
4. 等新版 GAASD 修复示波器、仿真脚本和真值表生成后，先跑第一阶段基础控制，再接入真值表决策输出。

## 11. 后续提醒：驾驶指令脉冲源（暂不实现）

当前主线按“无指令时 `commandType=0`、有指令时只保持一个仿真周期非零”的脉冲约定设计，画布内不搭边沿检测。该约定要成立，前提是**指令源能产生真正的单周期脉冲**。

现状限制（待后续解决，先不实现）：

- 现在唯一可用的指令源是 `constant` 块，它是固定电平，无法做到“只非零一帧”。用常量保持 `commandType=1`，启控后会被真值表持续判为降速，`maxSpeed` 每周期递减。
- 适配器当前没有“读驾驶指令”接口（只有 read ego/lead/lane/object/chassis 与 publish cmd），Bridge 也没有驾驶指令下行通道。
- 即便后续加了 Bridge 指令通道，Bridge 异步发布与 GAASD 周期不严格对齐，一次按键可能跨 1~2 个仿真周期，无法保证“恰好一帧”。

后续方案（择期实现）：

- 新增驾驶指令边界组件，例如 `CARLAACCDriverCommand`，与 4 个 CARLA I/O 组件并列。
- 适配器新增 `carla_adapter_read_driver_command()`；Bridge 侧对驾驶指令做边沿检测，把一次按下转换为“一帧非零”后再发给 GAASD。
- 把脉冲生成/边沿检测放在边界（Bridge/适配器），画布保持简单，且单次按键稳定只触发一次，不依赖时序。

在该边界组件实现前，当前阶段先按以下方式推进，不阻塞主线：

- 第一、二阶段（基础控制 + 决策观察）不依赖指令脉冲，可正常搭建与验证。
- 第三阶段需要单次指令事件时，临时用常量单周期赋值观察单步效果，或临时在画布加一组边沿检测兜底；正式脉冲源等后续边界组件落地后替换。
