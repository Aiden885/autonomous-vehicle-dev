# GAASD LKS 决策与完整控制画布方案

## 0. 审核结论与已确认决策（2026-06-15）

本方案经对照 `docs/LKS算法设计原理0206.docx`（V0.6）审核，技术结论：
控制部分完整实现、决策合理简化、车道线默认有效，三项要求均正确落实，
作为正式方案采用。审核确认的关键决策：

1. **多项式接口采用**：Bridge 把 CARLA 地图中心线拟合成三次多项式
   `c0/c1/c2/c3` + 曲率，作标量传入；GAASD 用多项式在 3 个预瞄点求横向偏差。
   这是避开 GAASD 不支持 vector 端口的正解。
2. **推进方式**：先搭画布 + 离线验证控制逻辑，不等 codegen。离线阶段
   `c0/c1/c2/c3/curvature/egoV` 用常量输入，暂不依赖 Bridge/adapter 改造。
3. **测试 vMin = 1.0 m/s**（非文档 10 m/s），否则 CARLA 低速场景激活不了。
4. **第一版 `lksSwitchOn = 1` 常量自动激活**，跑通后再接键盘 E/C。
5. **决策用逻辑块（and/or/not）而非真值表**，规避真值表 codegen 缺陷。

> ⚠️ **头号前提（Codex 原方案未提）**：本方案用了
> `LKSDecision/LKSControl/LanePolynomialEval` 复合组件 + 局部状态 + 新基础组件，
> 正是当前新版 GAASD codegen 生成不出可编译代码的那些构造（已在 ACC 决策画布确认：
> run/composite_block 签名不一致、truth-table 未定义、`C++_None`、复合输入未赋值）。
> **因此本方案画布能搭、设计正确，但生成可运行代码需等代码生成团队修复 codegen。**
> 当前阶段目标 = 搭好画布 + 逐块核对逻辑正确性。

## 1. 设计依据

本方案依据以下资料制定：

- `docs/LKS算法设计原理0206.docx`
- 当前 CARLA Bridge 的 `gaasd.carla.lane_tracking.v1`
- 当前 LKS 最小组件和测试场景
- GAASD 新版局部参数、局部状态和基础运算组件能力

原理文档的最终修订记录为 V0.6。V0.6 已明确：

1. 控制器采用三预瞄点加权的位置式比例控制。
2. 不再单独使用航向误差。
3. 启控时保留驾驶员当前方向盘转角 `theta0`。
4. 预瞄距离随车速和道路曲直状态变化。
5. 输出转角受最大横向加速度 `3 m/s^2` 限制。

因此，当前最小 LKS 公式：

```text
steerRad = 0.072 * lateralOffset + 0.48 * headingError
```

只能作为早期链路验证，不能作为本次正式控制方案。

## 2. 本次实现范围

### 2.1 完整实现

- 系统关闭、待命、在控三种状态的判定。
- 低速、制动、主动换道等退出条件。
- 三预瞄点位置计算。
- 动态预瞄距离计算。
- 直道/弯道预瞄距离切换。
- 启控方向盘基准角捕获和保持。
- 三预瞄点加权比例控制。
- 基于最大横向加速度的动态转角限幅。
- CARLA 横向控制输出。

### 2.2 简化项

- 不实现真实摄像头车道线检测。
- CARLA 地图中心线作为理想车道线真值。
- 车道线检测有效状态固定为 `1`。
- 决策层不使用真值表，直接使用逻辑与、逻辑或和逻辑非实现。
- 第一轮控制验证不模拟真实驾驶员转向和转向灯，可用常量代替。

### 2.3 不属于 LKS 算法的测试辅助

LKS 本身只负责横向控制。为了让 CARLA 自车在无人踩油门时持续行驶，
测试场景需要额外提供固定目标速度。该固定速度只用于仿真，不属于 LKS
控制算法。

## 3. 总体架构

```text
CARLA 地图和车辆状态
        |
        v
Bridge 理想车道感知
  - 自车速度
  - 当前转向值
  - 车道中心线多项式
  - 道路曲率
        |
        v
GAASD 边界输入组件
        |
        +----------------------+
        |                      |
        v                      v
LKSDecision                LKSControl
简化状态判断               三预瞄点完整控制
        |                      |
        +----------+-----------+
                   v
          LKS 横向控制命令
                   |
                   v
             Bridge -> CARLA
```

建议在 GAASD 中建立两个复合组件：

- `LKSDecision`：只负责系统状态和使能判断。
- `LKSControl`：只负责预瞄点和转角控制计算。

两个复合组件内部尽量使用 GAASD 基础模块，不把控制公式封装成单个自定义
C/C++ 函数。

## 4. 坐标和符号约定

原文档车身坐标系采用：

- `x` 向前。
- `y` 向左。
- 归一化方向盘转角正值表示左转。

当前 CARLA Bridge 采用：

- 横向误差正值表示车道中心位于自车右侧。
- CARLA 转向正值表示向右转。

为了避免在输出端再次反号，本方案统一使用 CARLA 约定：

- `x` 向前。
- `y` 向右。
- 预瞄横向误差为正表示目标车道中心在车辆右侧。
- 转向指令为正表示向右转。
- `Kp` 保持正数。

Bridge 生成车道模型时必须完成上述坐标转换。GAASD 画布内部不得再次取反。

## 5. 车道线接口设计

### 5.1 不使用 vector 端口

原文档控制器输入为车道中心线点集 `C(k)`。当前 GAASD 组件规范不支持直接
使用 `std::vector` 作为端口，因此不能把可变长度车道点集传入画布。

推荐由 Bridge 将 CARLA 地图中心线拟合成三次多项式：

```text
y(x) = c0 + c1*x + c2*x^2 + c3*x^3
```

其中：

- `x`：相对车辆前保险杠参考点的前向距离，单位 m。
- `y`：车道中心相对参考点的右向偏差，单位 m。
- `c0`：当前横向偏差。
- `c1`：车道中心线一阶项。
- `c2`：车道中心线二阶项。
- `c3`：车道中心线三阶项。

### 5.2 Bridge 新增或扩展字段

建议扩展 `gaasd.carla.lane_tracking.v1`：

```json
{
  "valid": true,
  "lateral_offset_m": 0.2,
  "heading_error_rad": 0.01,
  "lane_model": {
    "c0_m": 0.2,
    "c1": 0.01,
    "c2_1pm": 0.0005,
    "c3_1pm2": 0.0,
    "curvature_1pm": 0.001,
    "range_m": 30.0
  }
}
```

`heading_error_rad` 为兼容旧接口保留，正式 V0.6 控制不使用该字段。

### 5.3 GAASD 边界输入

第一版建议使用单输出组件，避免多输出组件再次触发生成器问题：

| 组件实例 | 输出 |
|---|---|
| `CARLAEgoSpeed` | `egoV` |
| `CARLALKSLaneC0` | `c0` |
| `CARLALKSLaneC1` | `c1` |
| `CARLALKSLaneC2` | `c2` |
| `CARLALKSLaneC3` | `c3` |
| `CARLALKSRoadCurvature` | `curvature` |
| `CARLALKSCurrentSteer` | `currentSteerNorm` |

车道有效标志本阶段使用常量 `1`，不需要单独输入组件。

## 5.4 GAASD 基础算子真实端口对照（实测自 2.7.0.5 组件库）

> 来源：`gaasd-2.7.0.5/app/resources/app/preload/defaultComponent/operatorComponent`
> 与 `initialComponent`。**本表是权威端口名，下文各连线表中与本表冲突的端口名
> 以本表为准。**

端口命名有两套约定：

| 类别 | 组件（componentType） | 输入端口 | 输出端口 |
|---|---|---|---|
| 二元运算/比较/逻辑 | `add` `subtract` `multiply` `divide` `greater-equal` `greater-than` `less-equal` `less-than` `equal` `not-equal` `logic-and` `logic-or` | `a`, `b` | `result` |
| 一元逻辑/取负 | `logic-not` `negate` | `operand` | `result` |
| 数学函数（atomic-function） | `fabs` `sqrt` `atan` `sin` `cos` `exp` `log` … | `x` | `return` |
| 双参数学函数 | `fmin` `fmax` | `x`, `y` | `return` |
| 幂 | `pow` | `base`, `exponent` | `return` |
| 数据来源 | `constant` `read-local-param` `read-local-state` `read-global-param` | 无 | `out` |
| 状态记忆 | `static-variable` | `value` | `value` |
| 写状态 | `write-local-state` | `in` | 无 |
| 子系统端口 | `input` / `output` / `oscilloscope` | `output→in`，`oscilloscope→input_1,2…` | `input→out` |

**据此修正本方案的既有错误：**

1. **`atan` 存在**（`x`→`return`）。§7.6、§9.2 中“基础库无反正切、需新增 `AtanVal`”
   作废——横向加速度限幅可直接用 `atan` 搭，无需新组件、无需等团队。
2. **绝对值用 `fabs`（double）**，端口 `x`→`return`。`abs` 仅 int。§6.4/§7.1 中
   `ABS_*` 的 `sourceValue`/`absoluteValue` 端口名作废，改用 `fabs` 的 `x`/`return`。
3. **`logic-not` 端口为 `operand`**（非 `input`）。§6.4 各 `NOT_*` 改 `operand <- ...`。
4. **比较结果为 `double`（0.0/1.0），非 `bool`**，可直接接入乘法/加法；§7.1.1 中
   “bool→double 转换”无需。
5. **无 clamp 算子**：限幅用 `fmax(lo, fmin(val, hi))` 组合。§7.5/§7.6 的 clamp 据此搭；
   负下限用 `negate` 或 `multiply(-1)` 得到 `-thetaLimit`。
6. `x2 = ld/2` 用 `divide`（`a<-ld`, `b<-2`）或 `multiply`（`b<-0.5`）。

## 6. 简化决策方案

### 6.1 决策输入

| 输入 | 含义 | 第一阶段来源 |
|---|---|---|
| `lksSwitchOn` | LKS 总开关 | 常量或键盘指令 |
| `egoV` | 自车速度，m/s | CARLA 输入组件 |
| `brakePressed` | 驾驶员制动 | 第一阶段常量 0 |
| `turnSignalOn` | 左右转向灯是否开启 | 第一阶段常量 0 |
| `driverSteerNorm` | 驾驶员方向盘输入 | 第一阶段常量 0 |
| `laneValid` | 车道线是否有效 | 固定常量 1 |

`driverSteerNorm` 必须表示驾驶员输入，不能使用 LKS 输出后的车辆实际转向值，
否则 LKS 自己产生的转角会被误判为驾驶员主动换道。

### 6.2 主动换道判断

```text
driverSteerHigh = abs(driverSteerNorm) >= driverSteerThreshold
activeLaneChange = driverSteerHigh
```

> 2026-06-17 简化：转向灯（turnSignalOn）暂不考虑（CARLA 不易仿真转向灯），
> 主动换道**只按驾驶员转角**判断，故 `activeLaneChange = driverSteerHigh`。
> 同时**不设总开关 lksSwitchOn**（LKS 常驻启用），决策也不再输出三态 systemState。
> 最终：`controlEnabled = NOT (brakePressed OR driverSteerHigh OR NOT speedOk)`，
> 即 `controlEnabled = speedOk AND (NOT brakePressed) AND (NOT driverSteerHigh)`。
> 决策输入仅 3 个：egoV、brakePressed、driverSteerNorm。

原文档建议：

```text
driverSteerThreshold = 0.1
```

### 6.3 激活和退出条件

由于车道有效状态固定为 `1`，决策可简化为：

```text
speedOk = egoV >= vMin

exitCondition =
    brakePressed
    OR activeLaneChange
    OR NOT speedOk

controlEnabled =
    lksSwitchOn
    AND NOT exitCondition
```

状态编码：

```text
systemState = 0    LKS 关闭
systemState = 1    LKS 开启但待命
systemState = 2    LKS 在控
```

可用以下算式避免额外真值表：

```text
systemState = lksSwitchOn * (1 + controlEnabled)
```

### 6.4 决策模块连线

| 实例名 | 类型 | 连线 |
|---|---|---|
| `C_VMin` | 局部参数读取 | 输出最低适控车速 |
| `GE_SpeedOk` | 大于等于 | `a <- egoV`；`b <- C_VMin.out` |
| `ABS_DriverSteer` | 绝对值 | `sourceValue <- driverSteerNorm` |
| `GE_SteerHigh` | 大于等于 | `a <- ABS_DriverSteer.absoluteValue`；`b <- DriverSteerThreshold` |
| `OR_LaneChange` | 逻辑或 | `a <- turnSignalOn`；`b <- GE_SteerHigh.result` |
| `NOT_SpeedOk` | 逻辑非 | `input <- GE_SpeedOk.result` |
| `OR_Exit1` | 逻辑或 | `a <- brakePressed`；`b <- OR_LaneChange.result` |
| `OR_Exit2` | 逻辑或 | `a <- OR_Exit1.result`；`b <- NOT_SpeedOk.result` |
| `NOT_Exit` | 逻辑非 | `input <- OR_Exit2.result` |
| `AND_Enable` | 逻辑与 | `a <- lksSwitchOn`；`b <- NOT_Exit.result` |
| `ADD_StateBase` | 加法 | `a <- 1`；`b <- AND_Enable.result` |
| `MUL_SystemState` | 乘法 | `a <- lksSwitchOn`；`b <- ADD_StateBase.result` |

输出：

```text
controlEnabled <- AND_Enable.result
systemState <- MUL_SystemState.result
```

## 7. 完整控制方案

## 7.1 动态预瞄距离

原文档公式：

```text
ld = alphaL * (l0 + rt * egoV)
```

其中：

```text
alphaL = 1          直道
alphaL = rAlpha     弯道
```

原文档表 18 中 `rAlpha` 是数学分数 `2/3`，不是数值 `23`。

道路曲率判断：

```text
isCurve = abs(curvature) >= curvatureThreshold
alphaL = 1 - isCurve * (1 - rAlpha)
```

连线：

| 实例名 | 类型 | 连线 |
|---|---|---|
| `ABS_Curvature` | 绝对值 | `sourceValue <- curvature` |
| `GE_IsCurve` | 大于等于 | `a <- ABS_Curvature.absoluteValue`；`b <- curvatureThreshold` |
| `SUB_AlphaDiff` | 减法 | `a <- 1`；`b <- rAlpha` |
| `MUL_AlphaReduce` | 乘法 | `a <- GE_IsCurve.result`；`b <- SUB_AlphaDiff.result` |
| `SUB_Alpha` | 减法 | `a <- 1`；`b <- MUL_AlphaReduce.result` |
| `MUL_RtV` | 乘法 | `a <- rt`；`b <- egoV` |
| `ADD_BaseLookahead` | 加法 | `a <- l0`；`b <- MUL_RtV.result` |
| `MUL_Lookahead` | 乘法 | `a <- SUB_Alpha.result`；`b <- ADD_BaseLookahead.result` |

输出：

```text
previewDistance <- MUL_Lookahead.result
```

### 7.1.1 `LKSPreviewDistance` 子系统详细画线步骤

第一阶段先把动态预瞄距离单独做成一个子系统，命名为：

```text
LKSPreviewDistance
```

该子系统只计算预瞄距离，不接车道多项式、不接转角控制。

#### 输入端口

| 端口名 | 类型 | 说明 |
|---|---|---|
| `egoV` | `double` | 自车速度，m/s |
| `curvature` | `double` | 道路曲率，1/m |
| `l0` | `double` | 基础预瞄距离 |
| `rt` | `double` | 预瞄时间系数 |
| `rAlpha` | `double` | 弯道缩放系数 |
| `curvatureThreshold` | `double` | 曲率阈值 |

第一阶段顶层建议这样接：

| 子系统输入 | 顶层来源 |
|---|---|
| `egoV` | 常量 `C_EgoV`，初值 `5.0` |
| `curvature` | 常量 `C_Curvature`，初值 `0.0` |
| `l0` | 全局参数 `lks_l0`，初值 `5.0` |
| `rt` | 全局参数 `lks_rt`，初值 `0.5` |
| `rAlpha` | 全局参数 `lks_rAlpha`，初值 `0.6666667` |
| `curvatureThreshold` | 全局参数 `lks_curvatureThreshold`，初值 `0.001` |

#### 输出端口

| 端口名 | 类型 | 说明 | 连接来源 |
|---|---|---|---|
| `previewDistance` | `double` | 最终预瞄距离 | `MUL_Lookahead.result` |
| `isCurve` | `double/bool/int` | 是否弯道 | `GE_IsCurve.result` |
| `alphaL` | `double` | 当前预瞄缩放系数 | `SUB_Alpha.result` |

如果 GAASD 的比较模块输出为 `bool` 且无法直接接入乘法模块，则需要在
`GE_IsCurve.result` 与 `MUL_AlphaReduce.a` 之间增加 `bool -> double`
转换模块。若画布允许直接连接，则不需要转换。

#### 子系统内部模块

| 实例名 | 基础模块 | 参数 |
|---|---|---|
| `ABS_Curvature` | 绝对值 | - |
| `GE_IsCurve` | 大于等于 | - |
| `C_One` | 常量 | `1.0` |
| `SUB_AlphaDiff` | 减法 | - |
| `MUL_AlphaReduce` | 乘法 | - |
| `SUB_Alpha` | 减法 | - |
| `MUL_RtV` | 乘法 | - |
| `ADD_BaseLookahead` | 加法 | - |
| `MUL_Lookahead` | 乘法 | - |

#### 曲率判断连线

| 模块 | 端口连接 |
|---|---|
| `ABS_Curvature` | `sourceValue <- curvature` |
| `GE_IsCurve` | `a <- ABS_Curvature.absoluteValue` |
| `GE_IsCurve` | `b <- curvatureThreshold` |

得到：

```text
isCurve = abs(curvature) >= curvatureThreshold
```

#### 预瞄缩放系数连线

| 模块 | 端口连接 |
|---|---|
| `SUB_AlphaDiff` | `a <- C_One.out` |
| `SUB_AlphaDiff` | `b <- rAlpha` |
| `MUL_AlphaReduce` | `a <- GE_IsCurve.result` |
| `MUL_AlphaReduce` | `b <- SUB_AlphaDiff.result` |
| `SUB_Alpha` | `a <- C_One.out` |
| `SUB_Alpha` | `b <- MUL_AlphaReduce.result` |

得到：

```text
alphaL = 1 - isCurve * (1 - rAlpha)
```

#### 基础预瞄距离连线

| 模块 | 端口连接 |
|---|---|
| `MUL_RtV` | `a <- rt` |
| `MUL_RtV` | `b <- egoV` |
| `ADD_BaseLookahead` | `a <- l0` |
| `ADD_BaseLookahead` | `b <- MUL_RtV.result` |

得到：

```text
baseLookahead = l0 + rt * egoV
```

#### 最终预瞄距离连线

| 模块 | 端口连接 |
|---|---|
| `MUL_Lookahead` | `a <- SUB_Alpha.result` |
| `MUL_Lookahead` | `b <- ADD_BaseLookahead.result` |

得到：

```text
previewDistance = alphaL * baseLookahead
```

#### 输出端口连线

| 输出端口 | 接到 |
|---|---|
| `previewDistance` | `MUL_Lookahead.result` |
| `isCurve` | `GE_IsCurve.result` |
| `alphaL` | `SUB_Alpha.result` |

#### 第一轮验算

直道测试输入：

```text
egoV = 5.0
curvature = 0.0
l0 = 5.0
rt = 0.5
rAlpha = 0.6666667
curvatureThreshold = 0.001
```

预期输出：

```text
isCurve = 0
alphaL = 1.0
previewDistance = 7.5
```

弯道测试时将 `curvature` 改成：

```text
curvature = 0.002
```

预期输出：

```text
isCurve = 1
alphaL = 0.6666667
previewDistance = 5.0
```

## 7.2 三个预瞄点

本方案使用：

```text
x1 = nearPreviewDistance
x2 = previewDistance / 2
x3 = previewDistance
```

`x1` 用一个很小的正距离近似原文“首个 x 坐标大于 0 的点”。
注意：`x1` 是常量输入，不从 `previewDistance` 计算；`x3` 直接接
`previewDistance`；只有 `x2` 需要在顶层增加一个乘 `0.5` 的基础块。

顶层预瞄点连线：

| 实例名 | 类型 | 连线 |
|---|---|---|
| `C_Half` | 常量 | 值设为 `0.5` |
| `MUL_X2Half` | 乘法 | `a <- LKSPreviewDistance.previewDistance`；`b <- C_Half.out` |

三个预瞄点来源：

| 预瞄点 | 来源 |
|---|---|
| `x1` | `nearPreviewDistance` |
| `x2` | `MUL_X2Half.result` |
| `x3` | `LKSPreviewDistance.previewDistance` |

每个预瞄误差由车道多项式计算：

```text
e(x) = ((c3*x + c2)*x + c1)*x + c0
```

分别计算：

```text
e1 = e(x1)
e2 = e(x2)
e3 = e(x3)
```

建议将上述基础块组合封装成 `LanePolynomialEval` 复合组件，放置三个实例：

| 实例 | 输入 x | 输出 |
|---|---|---|
| `Eval_P1` | `nearPreviewDistance` | `e1` |
| `Eval_P2` | `MUL_X2Half.result` | `e2` |
| `Eval_P3` | `LKSPreviewDistance.previewDistance` | `e3` |

`LanePolynomialEval` 内部连线：

| 实例名 | 类型 | 连线 |
|---|---|---|
| `MUL_C3X` | 乘法 | `a <- c3`；`b <- x` |
| `ADD_C2` | 加法 | `a <- MUL_C3X.result`；`b <- c2` |
| `MUL_X2` | 乘法 | `a <- ADD_C2.result`；`b <- x` |
| `ADD_C1` | 加法 | `a <- MUL_X2.result`；`b <- c1` |
| `MUL_X3` | 乘法 | `a <- ADD_C1.result`；`b <- x` |
| `ADD_C0` | 加法 | `a <- MUL_X3.result`；`b <- c0` |

输出：

```text
e <- ADD_C0.result
```

## 7.3 三预瞄点加权误差

原文档参数：

```text
w1 = 0.2
w2 = 0.3
w3 = 0.5
```

计算：

```text
weightedError = w1*e1 + w2*e2 + w3*e3
```

连线：

| 实例名 | 类型 | 连线 |
|---|---|---|
| `MUL_W1E1` | 乘法 | `a <- w1`；`b <- e1` |
| `MUL_W2E2` | 乘法 | `a <- w2`；`b <- e2` |
| `MUL_W3E3` | 乘法 | `a <- w3`；`b <- e3` |
| `ADD_Weight12` | 加法 | `a <- MUL_W1E1.result`；`b <- MUL_W2E2.result` |
| `ADD_WeightedError` | 加法 | `a <- ADD_Weight12.result`；`b <- MUL_W3E3.result` |

## 7.4 启控基准角 theta0

原文档要求：

```text
thetaRaw = theta0 + Kp * weightedError
```

`theta0` 是 LKS 从待命进入在控时的当前归一化转向值。它必须只在启控上升沿
更新，不能每帧更新。

局部状态：

| 状态 | 默认值 |
|---|---:|
| `prevControlEnabled` | 0 |
| `theta0Norm` | 0.0 |

上升沿：

```text
enableRising = controlEnabled AND NOT prevControlEnabled
```

状态更新：

```text
theta0Next =
    enableRising * currentSteerNorm
    + (1 - enableRising) * theta0Norm

prevControlEnabledNext = controlEnabled
```

控制计算本周期使用 `theta0Next`，确保刚启控时立即采用当前转向值。

## 7.5 位置式比例控制

原 Simulink 模型中解析到的比例增益为：

```text
Kp = 0.08 1/m
```

此前方案文档曾写 `Kp = 0.025 1/m`，该数值来源仍需复核；在没有进一步
依据前，第一轮画布搭建优先采用原 Simulink 模型中的 `0.08`。工程上应将
`Kp` 作为可调参数 `lks_Kp`，后续根据 CARLA 闭环效果再标定。

计算：

```text
thetaCorrection = Kp * weightedError
thetaRaw = theta0Next + thetaCorrection
```

`thetaRaw` 是归一化转向指令。第一阶段先不搭任何限幅，直接用于后续
`steerRad` 换算；固定限幅和横向加速度动态限幅等控制主链确认后再增加。

## 7.6 横向加速度动态限幅

原文档：

```text
ayMax = 3 m/s^2
phiAyLimit = atan(ayMax * wheelBase / egoV^2)
thetaLimit = phiAyLimit / frontWheelMaxRad
```

工程实现需要避免低速除零：

```text
speedSquare = egoV * egoV
safeSpeedSquare = max(speedSquare, limitSpeedFloor^2)
radiusTerm = ayMax * wheelBase / safeSpeedSquare
phiAyLimit = atan(radiusTerm)
phiLimit = min(phiAyLimit, frontWheelMaxRad)
thetaLimit = phiLimit / frontWheelMaxRad
```

最终限幅：

```text
thetaLimited = clamp(thetaRaw, -thetaLimit, thetaLimit)
```

基础组件库已提供 `atan` 算子（atomic-function，端口 `x`→`return`，
实测自 2.7.0.5，见 §5.4）。本段直接用它实现，无需新增组件。
另有 `atan2` 可用。`min`/`max` 用 `fmin`/`fmax`（`x`,`y`→`return`）。

## 7.7 输出到 CARLA

当前 Bridge 接口接收 `steerRad`，因此需要把归一化转向换算为协议转角：

第一阶段无任何限幅时：

```text
steerRad = thetaRaw * steerCommandScaleRad
```

后续增加限幅后：

```text
steerRad = thetaLimited * steerCommandScaleRad
```

其中 `steerCommandScaleRad` 必须与 Bridge 的 `max_abs_steer_rad` 一致，
否则 Bridge 再归一化时比例会错误。

第一轮无人驾驶 CARLA 测试：

```text
lksSteerRad = controlEnabled * steerRad
targetSpeed = 固定测试速度
vehicleCommandEnable = 1
```

这里 `vehicleCommandEnable` 固定为 `1` 是为了让测试纵向速度继续工作。
`controlEnabled=0` 时只把 LKS 转角清零，不能直接把联合控制命令的总使能置零，
因为当前 Bridge 对总使能为零的处理是全制动。

正式节点集成时应提供独立的横向使能，不应使用“总使能清零”等价表达 LKS
待命。

## 8. 参数表

### 8.1 原文档确定参数

| 参数 | 含义 | 数值 |
|---|---|---:|
| `Kp` | 多目标比例系数 | 0.08 1/m |
| `w1` | 近预瞄点权重 | 0.2 |
| `w2` | 中预瞄点权重 | 0.3 |
| `w3` | 远预瞄点权重 | 0.5 |
| `l0` | 基础最小预瞄距离 | 5 m |
| `rt` | 预瞄时间系数 | 0.5 s |
| `rAlpha` | 弯道缩放系数 | 2/3 |
| `ayMax` | 最大横向加速度 | 3 m/s^2 |
| `driverSteerThreshold` | 驾驶员转向覆盖阈值 | 0.1 |
| `vMin` | 文档最低适控速度 | 10 m/s |

### 8.2 工程参数

| 参数 | 推荐来源 | 说明 |
|---|---|---|
| `wheelBase` | CARLA 车辆物理参数 | 不应长期硬编码 |
| `frontWheelMaxRad` | CARLA 前轮 `max_steer_angle` | 用于横向加速度限幅 |
| `steerCommandScaleRad` | Bridge 配置 | 当前默认 0.6 rad |
| `curvatureThreshold` | 联调标定 | 文档未给出 |
| `nearPreviewDistance` | 0.5 m 初值 | 近似首个 `x>0` 点 |
| `limitSpeedFloor` | 1.0 m/s 初值 | 防止除零 |
| `testTargetSpeed` | CARLA 测试配置 | 不属于 LKS 算法 |

### 8.3 参数承载方式（全局参数 vs 常量）

**算法整定参数 → 用全局参数**（`read-global-param` 读取，集中调参）：

| 名称 | 全局参数名 | 初值 |
|---|---|---|
| l0 | `lks_l0` | 5.0 |
| rt | `lks_rt` | 0.5 |
| rAlpha | `lks_rAlpha` | 0.6666667 |
| curvatureThreshold | `lks_curvatureThreshold` | 0.001 |
| nearPreviewDistance | `lks_nearPreviewDistance` | 0.5 |
| w1 | `lks_w1` | 0.2 |
| w2 | `lks_w2` | 0.3 |
| w3 | `lks_w3` | 0.5 |
| Kp | `lks_Kp` | 0.08 |
| steerScale | `lks_steerScale` | 0.6 |

**第一阶段模拟输入信号 → 暂用普通常量**（`constant`，后续被边界组件替换）：

| 名称 | 初值 | 后续替换 |
|---|---|---|
| egoV | 5.0 | CARLAEgoSpeed |
| curvature | 0.0 | CARLALKSRoadCurvature |
| c0 | 0.8 | CARLALKSLaneC0 |
| c1 | 0.0 | CARLALKSLaneC1 |
| c2 | 0.0 | CARLALKSLaneC2 |
| c3 | 0.0 | CARLALKSLaneC3 |

> 此分类取代 §7.1.1 中“局部参数”的写法。各子系统内部直接用 `read-global-param`
> 读全局参数，不必把整定参数作为输入端口层层传递。

## 9. 与现有 LKS 组件的关系

### 9.1 可以复用

- `CARLAEgoSpeed`
- `CARLALKSControlCmd` 的联合控制发布能力
- `gaasd.carla.lane_tracking.v1` topic
- `carla_adapter_read_lane_tracking()`
- `lkspro1_basic_carla_20260526` 场景启动和初始偏差设置

### 9.2 需要扩展或替换

- `CARLALKSLateralOffset`：可作为调试信号，不能独立支持完整三预瞄控制。
- `CARLALKSHeadingError`：V0.6 正式控制不再使用。
- `CARLALKSValid`：本阶段由常量 `1` 替代。
- Bridge：需要增加车道多项式和道路曲率计算。
- adapter：需要缓存并向 GAASD 提供多项式系数和曲率。
- GAASD：基础库已含 `atan`/`atan2`/`fmin`/`fmax`，无需新增数学组件（见 §5.4）。

## 10. 推荐实施顺序

### 阶段一：控制数学链离线验证

1. 不连接 CARLA。
2. 使用常量模拟 `c0/c1/c2/c3/curvature/egoV`。
3. 搭建动态预瞄距离、三个多项式预瞄点、加权误差和比例控制。
4. 检查直线和曲线输入下的计算结果。

### 阶段二：启控状态和动态限幅

1. 增加 `prevControlEnabled` 和 `theta0Norm` 局部状态。
2. 验证启控上升沿只捕获一次 `theta0`。
3. 增加横向加速度限幅和 `atan` 组件。
4. 检查车速升高时允许转角自动减小。

### 阶段三：Bridge 理想车道感知接入

1. Bridge 生成车道多项式和道路曲率。
2. adapter 扩展缓存和读取接口。
3. GAASD 新增单输出边界组件。
4. 验证画布计算的 `e1/e2/e3` 与 CARLA 几何关系一致。

### 阶段四：CARLA 单车闭环

1. 复用 LKS 单车场景。
2. 初始横向偏差设置为 `0.8 m`。
3. 初始航向误差可先设为 `0 deg`，再增加至 `5 deg`。
4. 先跑直道，再选择弯道路段验证 `rAlpha=2/3`。

### 阶段五：简化决策和驾驶员退出

1. 接入 LKS 开关。
2. 接入制动退出。
3. 接入转向灯和驾驶员转向覆盖。
4. 验证关闭、待命、在控状态切换。

## 11. 示波器观测信号

至少观察：

```text
systemState
controlEnabled
egoV
curvature
previewDistance
e1
e2
e3
weightedError
theta0Norm
thetaRaw
thetaLimit
thetaLimited
steerRad
```

## 12. 验收标准

### 12.1 决策

- 开关关闭时状态为关闭。
- 开关开启但车速不足时状态为待命。
- 满足激活条件时进入在控。
- 制动、转向灯或驾驶员大转角任一出现时退出在控。
- 条件恢复后可重新进入在控，并重新捕获 `theta0`。

### 12.2 控制

- 直道初始偏差下，三个预瞄误差和横向偏差逐步收敛。
- 弯道时预瞄距离按 `2/3` 缩短。
- 车速升高时 `thetaLimit` 减小。
- 估算横向加速度不超过 `3 m/s^2`。
- 无持续单向偏置、明显发散或高频左右振荡。
- 控制退出时 LKS 转向贡献归零。

## 13. 待确认事项

1. 第一轮 CARLA 测试是否采用 `vMin=1.0 m/s` 的测试配置。原文档
   `vMin=10 m/s`，若严格使用，当前低速场景无法激活。
2. 第一轮 LKS 开关是否使用键盘 `E` 激活、`C` 取消，还是先用常量
   `lksSwitchOn=1` 自动激活。
3. 是否接受“Bridge 输出三次车道多项式，GAASD 计算三个预瞄点”的接口方案。
   该方案能够避开 `vector` 端口，同时保留完整控制计算在画布中。
4. 第一轮 CARLA 固定测试速度建议在确认后设置；该数值只影响测试，不属于
   LKS 算法参数。
