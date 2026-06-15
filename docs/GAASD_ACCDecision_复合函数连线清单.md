# ACCDecision 组件(composite-block)内部精确连线清单

> 配套主方案 `GAASD_ACC_完整决策控制画布方案.md`。
> 本文件是“决策层”子系统的内部逐块逐线清单。
> 容器用 **`组件`(componentType=composite-block)**,不是 composite-function——
> 它带自有 cppClass、可配局部参数/局部状态,官方 PidController demo 就是用它 + read-local-state 实现的。
> 主画布只保留 CARLA I/O + 控制律(时距公式);本组件封装全部决策状态机。

## 基于官方 PidController 范式的 4 点改进

解析官方 PidController(composite-block)内部后,本清单按其范式改进:

1. **子系统端口 = 组件内部的 `input`/`output` 块**。进入组件内部后,先放 `input` 块(egoV、commandType)、`output` 块(enable、timeGap、maxSpeed),它们就是对外端口,内部连线连到这些块。
2. **可调增益用「局部参数 Param + `read-local-param`」,不用 constant 块**。kDist/kSpeed/timeGapStep/speedStep/各限幅值等需要调参的量,声明为局部参数,用 `read-local-param` 读出——这样在属性面板就能调,不必改图重新生成。PidController 的 kp/ki/kd/ts 就是这么做的。
3. **结构性常量(判等用的 0-8)仍用 `constant` 块**。它们不是调参对象,是真值表/解码的固定比较值。
4. **关键中间量可用 `variable` 块命名**(可选)。PidController 用 variable 命名 e/de/uRaw 等。本图大,只对 systemState 和三个输出等关键量用 variable,其余直接块对块连,控制块数。

## 0. 复合函数接口

**输入端口(2):**
| 端口 | 含义 |
| --- | --- |
| `egoV` | 自车速度,来自主画布 CARLAACCEgoSpeed |
| `commandType` | 驾驶指令(0=无,1-7),来自主画布常量(单周期脉冲约定) |

**局部状态 State(4,在复合函数 State 配置):**
| 字段 | 类型 | 初始值 |
| --- | --- | --- |
| `controlEnabled` | int | 0 |
| `hasHistory` | int | 0 |
| `timeGap` | double | 1.8 |
| `maxSpeed` | double | 5.0 |

**输出端口(3):**
| 端口 | 含义 |
| --- | --- |
| `enable` | 控制使能 → 主画布 CARLAACCLongitudinalCmd.enable |
| `timeGap` | 当前目标时距 → 主画布控制律 desiredDistance |
| `maxSpeed` | 当前速度上限 → 主画布控制律 targetSpeed 限幅 |

**端口约定(新版默认):**
add/subtract/multiply `a,b→result`(多输入版 `input_1/2/3→result`);
fmax/fmin `x,y→return`;less-than/greater-than/equal `a,b→result`;
logic-and/or `a,b→result`(3输入 `input_1/2/3`);logic-not `operand→result`;
read-local-state `→out`;write-local-state `配置字段←输入`;truth-table `u,v→y`;constant `→out`。

## 1. 参数与常量

### 1a. 局部参数 Param(在组件 Param 配置,用 read-local-param 读出,可调)

| Param 字段 | 类型 | 默认值 | 读出块(范式 `RP_`) |
| --- | --- | --- | --- |
| `vMin` | double | 0.0(CARLA测试)/0.5(文档) | `RP_vMin` |
| `timeGapStep` | double | 0.2 | `RP_timeGapStep` |
| `minTimeGap` | double | 1.0 | `RP_minTimeGap` |
| `maxTimeGap` | double | 5.0 | `RP_maxTimeGap` |
| `speedStep` | double | 1.0 | `RP_speedStep` |
| `minSpeed` | double | 0.0 | `RP_minSpeed` |

> 段G 已简化:不再用 `maxSpeedCap`(固定上限)和 `r5CaptureMinSpeed`(零速保护),两者已从参数表删除。`maxSpeed` 状态默认 50,即可调限速。

> 下文段表里凡写 `C_vMin`、`C_timeGapStep` 等可调量,一律用对应的 `RP_*`(read-local-param)实现。

### 1b. 结构性常量 constant(固定比较值,不可调)

| 实例 | 值 | 用途 |
| --- | --- | --- |
| `C_2` / `C_3` | 2 / 3 | systemState 编码权重 |
| `C_1`..`C_8` | 1..8 | 解码 R1-R8、判等 commandType |

## 2. 分段连线(也是建议的分阶段搭建顺序)

### 段A：读局部状态
| 实例 | 类型 | 配置/连线 |
| --- | --- | --- |
| `RS_ControlEnabled` | read-local-state | 字段 controlEnabled |
| `RS_HasHistory` | read-local-state | 字段 hasHistory |
| `RS_TimeGap` | read-local-state | 字段 timeGap |
| `RS_MaxSpeed` | read-local-state | 字段 maxSpeed |

### 段B：状态计算 S0-S3 + systemState
**当前为 CARLA 测试模式 vMin=0,低速分支 S3 是死代码,直接省去**(egoV≥0 时 egoV<0 恒假)。
故 `S0=controlEnabled`、`S3=0`,状态计算简化为:

| 实例 | 类型 | 输入连线 | 产出 |
| --- | --- | --- | --- |
| `NOT_Standby` | logic-not | operand←`RS_ControlEnabled.out` | 非在控(待命) |
| `NOT_NoHist` | logic-not | operand←`RS_HasHistory.out` | 无历史 |
| `AND_S1` | logic-and(2) | a←`NOT_Standby.result`,b←`RS_HasHistory.out` | S1 有史待命 |
| `AND_S2` | logic-and(2) | a←`NOT_Standby.result`,b←`NOT_NoHist.result` | S2 无史待命 |
| `MUL_S2w` | multiply | a←`AND_S2.result`,b←`C_2` | 2*S2 |
| `ADD_SS` | add | a←`AND_S1.result`,b←`MUL_S2w.result` | **systemState**(=S1*1+S2*2;S0=0) |

> S0 不需要单独块,`RS_ControlEnabled.out` 就是 S0。
> 文档一致模式(vMin=0.5、需 S3 低速保护)再加回 `LT_LowSpeed`+`RP_vMin`+取反+S3 编码。

### 段C：真值表决策
| 实例 | 类型 | 输入 | 产出 |
| --- | --- | --- | --- |
| `TT_Decision` | truth-table | u←`ADD_SS2.result`(systemState),v←`commandType` | `y`(decisionEvent) |

真值表配置(u=systemState, v=commandType → y),y=0 表示保持/无动作:
| u\v | 1降速 | 2增速 | 3降距 | 4增距 | 5油门 | 6刹车 | 7取消 | 0/其他 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0(S0在控) | 1 | 2 | 3 | 4 | 7 | 8 | 8 | 0 |
| 1(S1有史) | 5 | 6 | 0 | 0 | 0 | 0 | 0 | 0 |
| 2(S2无史) | 5 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 3(S3低速) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

### 段D：解码 R1-R8
| 实例 | 类型 | 输入 | 产出 |
| --- | --- | --- | --- |
| `EQ_R1` | equal | a←`TT_Decision.y`,b←`C_1` | R1 |
| `EQ_R2` | equal | a←`TT_Decision.y`,b←`C_2` | R2 |
| `EQ_R3` | equal | a←`TT_Decision.y`,b←`C_3` | R3 |
| `EQ_R4` | equal | a←`TT_Decision.y`,b←`C_4` | R4 |
| `EQ_R5` | equal | a←`TT_Decision.y`,b←`C_5` | R5 |
| `EQ_R6` | equal | a←`TT_Decision.y`,b←`C_6` | R6 |
| `EQ_R7` | equal | a←`TT_Decision.y`,b←`C_7` | R7(扭矩仲裁标志,暂仅观测) |
| `EQ_R8` | equal | a←`TT_Decision.y`,b←`C_8` | R8(待命,暂不用) |

### 段E：enable + 控制状态转换(状态机写法)

**enable 直接 = 当前在控状态 controlEnabled**(无需计算块):
- 输出端口 `enable` ← `RS_ControlEnabled.out`

**启控/退控只改下一周期 controlEnabled**:
`controlEnabledNext = (controlEnabled 或 启控R5R6) 且 非(刹车cmd6 或 取消cmd7)`

| 实例 | 类型 | 输入 | 产出 |
| --- | --- | --- | --- |
| `EQ_Cmd6` | equal | a←`commandType`,b←`C_6` | 刹车 |
| `EQ_Cmd7` | equal | a←`commandType`,b←`C_7` | 取消 |
| `OR_TurnOn` | logic-or | a←`EQ_R5.result`,b←`EQ_R6.result` | 启控 |
| `OR_BrakeCancel` | logic-or | a←`EQ_Cmd6.result`,b←`EQ_Cmd7.result` | 退控 |
| `NOT_NoBrakeCancel` | logic-not | operand←`OR_BrakeCancel.result` | 非退控 |
| `OR_Engaged` | logic-or | a←`RS_ControlEnabled.out`,b←`OR_TurnOn.result` | 已控或要启控 |
| `AND_CtrlNext` | logic-and(2) | a←`OR_Engaged.result`,b←`NOT_NoBrakeCancel.result` | **controlEnabledNext** |

> 代价:启控/退控有 1 周期(0.05s)延迟,可忽略。S0 已等于 controlEnabled,不再单独算。

### 段F：时距更新 timeGapNext
timeGapRaw = timeGap + step*R4 - step*R3;clamp[minTimeGap,maxTimeGap]
| 实例 | 类型 | 输入 | 产出 |
| --- | --- | --- | --- |
| `MUL_GapUp` | multiply | a←`EQ_R4.result`,b←`C_timeGapStep` | step*R4 |
| `MUL_GapDn` | multiply | a←`EQ_R3.result`,b←`C_timeGapStep` | step*R3 |
| `ADD_GapRaw1` | add | a←`RS_TimeGap.out`,b←`MUL_GapUp.result` | timeGap+up |
| `SUB_GapRaw` | subtract | a←`ADD_GapRaw1.result`,b←`MUL_GapDn.result` | timeGapRaw |
| `FMAX_Gap` | fmax | x←`SUB_GapRaw.result`,y←`C_minTimeGap` | 下限保护 |
| `FMIN_Gap` | fmin | x←`FMAX_Gap.return`,y←`C_maxTimeGap` | **timeGapNext** |
| `WS_TimeGap` | write-local-state | 字段 timeGap ←`FMIN_Gap.return` | 回写 |
| 输出端口 `timeGap` | | ←`FMIN_Gap.return` | 给控制律 |

### 段G：速度上限更新 maxSpeedNext(简化版)

`maxSpeed` 本身就是可调的限速设定值,默认 50;增速直接加、减速直接减,只保下限 0。
**不要 R5 零速捕获、不要固定上限**(否则默认顶在上限、增速无效)。

`maxSpeedNext = max(minSpeed=0, maxSpeed − step·R1 + step·R2)`

| 实例 | 类型 | 输入 | 产出 |
| --- | --- | --- | --- |
| `RS_MaxSpeed` | read-local-state | 字段 maxSpeed(默认50) | 当前限速 |
| `MUL_SpdDn` | multiply | a←`EQ_R1.result`,b←`C_speedStep` | step*R1 |
| `MUL_SpdUp` | multiply | a←`EQ_R2.result`,b←`C_speedStep` | step*R2 |
| `SUB_SpdCand1` | subtract | a←`RS_MaxSpeed.out`,b←`MUL_SpdDn.result` | maxSpeed-down |
| `ADD_SpdCand` | add | a←`SUB_SpdCand1.result`,b←`MUL_SpdUp.result` | speedCandidate |
| `FMAX_Spd` | fmax | x←`ADD_SpdCand.result`,y←`C_minSpeed`(0) | **maxSpeedNext**(只保下限) |
| `WS_MaxSpeed` | write-local-state | 字段 maxSpeed ←`FMAX_Spd.return` | 回写 |
| 输出端口 `maxSpeed` | | ←`FMAX_Spd.return` | 给控制律 |

> 已删除:`FMIN_Spd`、`C_maxSpeedCap`(固定上限)、`GT_EgoCap`、`AND_R5Cap`、`NOT_R5Cap`、`AND_R5Keep`、`NOT_R5`、`MUL_TermCap/Keep/Norm`、`ADD_SpdA`、`ADD_SpdNext`、`C_r5CaptureMinSpeed`。段G 也不再需要 egoV。
> 可选:想要绝对安全天花板,再加一个 `fmin`,但夹的值要远高于默认(如 150),不能等于默认值 50。

### 段H：状态回写(controlEnabled / hasHistory)
| 实例 | 类型 | 输入 | 产出 |
| --- | --- | --- | --- |
| `WS_ControlEnabled` | write-local-state | 字段 controlEnabled ←`AND_CtrlNext.result` | 回写 |
| `OR_HasHist` | logic-or | a←`RS_HasHistory.out`,b←`AND_CtrlNext.result` | hasHistoryNext |
| `WS_HasHistory` | write-local-state | 字段 hasHistory ←`OR_HasHist.result` | 回写 |

> hasHistory 用 `AND_CtrlNext`(新值),启控当帧即标有史。

## 3. 主画布侧改动(控制律升级为时距公式)

复合函数输出 `enable/timeGap/maxSpeed` 接回主画布;控制律改为:
```
desiredDistance = fmax(C_minDistance, egoV * timeGap)
distanceError   = distance - desiredDistance
relativeSpeed   = leadV - egoV
rawTargetSpeed  = leadV + kDist*distanceError + kSpeed*relativeSpeed
targetSpeed     = fmin(maxSpeed, fmax(0, rawTargetSpeed))
```
关键改动:原来固定 `desiredDistance=15` 的常量,改成 `multiply(egoV, timeGap) → fmax(minDistance, ...)`;
原来限幅用的常量 maxSpeed,改成接复合函数输出的 `maxSpeed`。

## 4. 块数与搭建顺序

组件内部约 75-85 块。**务必分段搭、每段生成一次代码验证**。

**段0(先做):** 进入组件内部,放端口块和参数读出块
- `input` 块:`egoV`、`commandType`(对外输入端口)
- `output` 块:`enable`、`timeGap`、`maxSpeed`(对外输出端口)
- `read-local-param` 块:按 1a 表读出所有可调参数
- `read-local-state` 块:按段A读出 4 个状态

**之后顺序:** B 状态计算 → C 真值表 → D 解码 → E enable → F 时距 → G 限速 → H 回写。

**最小验证版优先(强烈建议):** 先只搭到 enable 输出
= 段0 + A + B + C + D + E + 段H的 controlEnabled 回写。
生成一次代码,验证三件事:① composite-block 的 codegen 能用;② read/write-local-state、read-local-param 能生成;③ 真值表在组件内能生成。
**三项过了再加 F(时距)、G(限速)**,否则别堆 75 块。
