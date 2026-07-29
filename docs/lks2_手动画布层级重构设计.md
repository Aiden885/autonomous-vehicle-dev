# lks2 手动画布层级重构设计

## 1. 重构目标

本次只调整 `project/lks2` 的画布层级、模块名称和排布，不修改现有算法公式、参数来源、主流程输入输出和示波器配置。

顶层目标结构：

```text
MainFlow
├── Decision
│   ├── SpeedEnableCheck
│   ├── DriverOverrideCheck
│   └── ControlEnableDecision
└── Control
    ├── PreviewDistance
    ├── LaneErrorEvaluation
    │   ├── NearPreviewError
    │   ├── MiddlePreviewError
    │   └── FarPreviewError
    ├── ErrorWeightedSum
    └── SteerCommand
        ├── RawSteerCalculation
        ├── LateralAccelLimit
        └── SteerEnableGate
```

## 2. Decision 分层

### SpeedEnableCheck

根据自车速度和最低工作速度判断当前是否低于 LKS 工作速度范围：

```text
speedAvailable   = egoV >= lks_vMin
speedUnavailable = NOT(speedAvailable)
```

### DriverOverrideCheck

根据驾驶员归一化方向盘输入判断是否发生主动转向接管：

```text
driverOverride = abs(driverSteerNorm) >= lks_driverSteerThreshold
```

### ControlEnableDecision

汇总制动、低速和主动转向三个退出条件：

```text
controlEnabled = NOT(brakePressed OR speedUnavailable OR driverOverride)
```

这与重构前的逻辑完全一致，只把原来混排在同一层的基础组件放入三个具有明确物理含义的子模块。

## 3. Control 分层

### PreviewDistance

保留原 `LKSPreviewDistance` 公式，根据车速和道路曲率计算远预瞄距离。

### LaneErrorEvaluation

统一管理三次车道多项式的三个预瞄点评估：

```text
x1 = lks_nearPreviewDistance
x2 = previewDistance / 2
x3 = previewDistance
e(x) = ((c3 * x + c2) * x + c1) * x + c0
```

三个实例分别重命名为 `NearPreviewError`、`MiddlePreviewError` 和 `FarPreviewError`。

### ErrorWeightedSum

保留原 `LKSErrorFusion` 公式并改为物理含义明确的端口名：

```text
weightedError = w1 * nearError + w2 * middleError + w3 * farError
```

### SteerCommand

转角命令进一步拆为三层：

```text
RawSteerCalculation: rawSteerNorm = Kp * weightedError

LateralAccelLimit:
speedSquareSafe = max(egoV * egoV, 0.25)
normalizedSteerLimit = atan(ayMax * wheelBase / speedSquareSafe)
                       / frontWheelMaxRad
limitedSteerNorm = clamp(rawSteerNorm,
                         -normalizedSteerLimit,
                         +normalizedSteerLimit)

SteerEnableGate:
steerRad = limitedSteerNorm * steerScale * controlEnabled
```

## 4. 顶层接口保持不变

主流程输入仍为：

```text
egoV, c0, c1, c2, c3, curvature, brakePressed, driverSteerNorm
```

主流程输出仍为：

```text
steerRad, controlEnabled, valid, previewDistance, weightedError
```

模块层正式 `lks_input / lks_output` 通道、`frame_id` 透传和示波器观察信号均保持不变。

## 5. 数据库修改原则

- 保留原计算组件 UUID、参数读取路径和计算连线。
- 只为新增子模块创建必要的边界端口和层级连线。
- 禁止跨子系统直接连线，所有跨层信号必须经过输入/输出边界。
- `cbdes.db` 修改成功后同步到 `temp.db`。
- 执行前保存完整工程快照；事务内任一校验失败即整体回滚。
