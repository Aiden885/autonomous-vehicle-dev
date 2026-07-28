# ACC/LKS 扫描画布层级示意

本文档用于预览 `modules_refined_20260708.tar.gz` 扫描后大概会形成的画布层级。  
源码包路径：`/home/aiden/文档/temp/modules_refined_20260708.tar.gz`

## 1. ACC 顶层：AccMainFlow

`AccMainFlow` 一级展开后应看到四块：两个输入通道、一个决策层、一个控制层。

```mermaid
flowchart LR
  A["AccMainFlow<br/>ACC主流程"]

  V["AccVehicleInputChannel<br/>车上信息输入通道<br/>egoSpeed / commandType"]
  P["AccPerceptionInputChannel<br/>车外感知输入通道<br/>leadSpeed / leadDistance"]
  D["AccDecision<br/>ACC决策层"]
  C["AccSpeedControl<br/>ACC控制层"]

  O1["targetSpeed<br/>目标速度"]
  O2["enable / valid<br/>使能 / 有效"]
  O3["timeGap / maxSpeed<br/>时距 / 限速"]

  A --> V
  A --> P
  V --> D
  D --> C
  V --> C
  P --> C
  C --> O1
  D --> O2
  D --> O3
```

## 2. ACC 决策层：AccDecision

`AccDecision` 负责把车速、驾驶指令和内部状态转换成 ACC 决策结果。

```mermaid
flowchart LR
  I1["egoSpeed"]
  I2["commandType"]

  U["AccControlStateUpdate<br/>在控状态更新<br/>状态/决策/enable"]
  G["AccTimeGapUpdate<br/>时距更新"]
  V["AccMaxSpeedUpdate<br/>限速更新"]

  I1 --> U
  I2 --> U
  U --> G
  U --> V

  U --> O1["enable"]
  G --> O2["timeGap"]
  V --> O3["maxSpeed"]
  U --> O4["decision"]
  U --> O5["systemState"]
```

### 2.1 AccControlStateUpdate 展开

```mermaid
flowchart LR
  I1["egoSpeed"]
  I2["commandType"]
  I3["previousControlEnabled"]
  I4["previousHasHistory"]

  S["AccSystemStateClassifier<br/>系统状态判断"]
  T["AccDecisionTable<br/>决策表"]
  M["AccControlMemoryUpdate<br/>在控状态记忆更新"]
  H["AccHistoryUpdate<br/>历史状态更新"]

  I1 --> S
  I3 --> S
  I4 --> S
  S --> T
  I2 --> T
  T --> M
  I1 --> M
  I2 --> M
  M --> H
  I4 --> H

  M --> O1["enable / nextControlEnabled"]
  H --> O2["nextHasHistory"]
  T --> O3["decision"]
  S --> O4["systemState"]
```

## 3. ACC 控制层：AccSpeedControl

`AccSpeedControl` 负责把跟车误差转换成目标速度。

```mermaid
flowchart LR
  I1["egoSpeed"]
  I2["leadSpeed"]
  I3["leadDistance"]
  I4["enable"]
  I5["timeGap"]
  I6["maxSpeed"]

  D["AccDesiredDistance<br/>目标车距<br/>egoSpeed*timeGap"]
  E["AccDistanceError<br/>距离误差<br/>leadDistance-desiredDistance"]
  R["AccRelativeSpeed<br/>相对速度<br/>leadSpeed-egoSpeed"]
  U["AccRawTargetSpeed<br/>未限幅目标速度"]
  L["AccTargetSpeedLimit<br/>目标速度限幅"]
  G["AccEnableGate<br/>使能门控"]

  I1 --> D
  I5 --> D
  I3 --> E
  D --> E
  I1 --> R
  I2 --> R
  I2 --> U
  E --> U
  R --> U
  U --> L
  I6 --> L
  L --> G
  I4 --> G

  G --> O1["targetSpeed"]
  G --> O2["valid"]
  D --> O3["desiredDistance"]
  E --> O4["distanceError"]
  R --> O5["relativeSpeed"]
```

ACC 控制公式：

```text
desiredDistance = max(minDistance, egoSpeed * timeGap)
distanceError   = leadDistance - desiredDistance
relativeSpeed   = leadSpeed - egoSpeed
rawTargetSpeed  = leadSpeed + distanceGain * distanceError + speedGain * relativeSpeed
targetSpeed     = clamp(rawTargetSpeed, 0, maxSpeed)
finalOutput     = enable ? targetSpeed : 0
```

## 4. LKS 顶层：LksMainFlow

`LksMainFlow` 一级展开后应看到四块：两个输入通道、一个决策层、一个控制层。

```mermaid
flowchart LR
  A["LksMainFlow<br/>LKS主流程"]

  V["LksVehicleInputChannel<br/>车上信息输入通道<br/>egoSpeed / brakePressed / driverSteerNorm"]
  P["LksPerceptionInputChannel<br/>车外感知输入通道<br/>c0 c1 c2 c3 / curvature"]
  D["LksDecision<br/>LKS决策层<br/>是否允许接管方向盘"]
  C["LksControl<br/>LKS控制层<br/>预瞄/误差/转角"]

  A --> V
  A --> P
  V --> D
  D --> C
  V --> C
  P --> C

  C --> O1["lksSteerRad<br/>方向盘转角"]
  D --> O2["controlEnabled<br/>LKS使能"]
  C --> O3["previewDistance<br/>远预瞄距离"]
  C --> O4["weightedError<br/>加权横向误差"]
```

## 5. LKS 决策层：LksDecision

`LksDecision` 当前只负责控制使能判断，内部由 `LksEnableLogic` 展开。

```mermaid
flowchart LR
  D["LksDecision<br/>LKS决策层"]
  E["LksEnableLogic<br/>控制使能判断"]

  D --> E
  E --> O["controlEnabled"]
```

### 5.1 LksEnableLogic 展开

```mermaid
flowchart LR
  I1["egoSpeed"]
  I2["brakePressed"]
  I3["driverSteerNorm"]

  L["LksLowSpeedCheck<br/>车辆速度过低判断"]
  B["LksBrakePressedCheck<br/>驾驶员踩刹车判断"]
  D["LksDriverSteerOverrideCheck<br/>驾驶员主动打方向判断"]
  E["LksEnableDecision<br/>控制使能汇总"]

  I1 --> L
  I2 --> B
  I3 --> D
  L --> E
  B --> E
  D --> E
  E --> O["controlEnabled"]
```

LKS 使能逻辑：

```text
speedReady     = egoSpeed >= vMin
brakeActive    = brakePressed != 0
driverOverride = abs(driverSteerNorm) >= driverSteerThreshold
controlEnabled = speedReady && !brakeActive && !driverOverride
```

## 6. LKS 控制层：LksControl

`LksControl` 包含远预瞄距离计算、三点预瞄误差模型和方向盘转角命令计算。

```mermaid
flowchart LR
  I1["egoSpeed"]
  I2["curvature"]
  I3["c0 c1 c2 c3"]
  I4["controlEnabled"]

  P["LksPreviewDistance<br/>远预瞄距离计算"]
  M["LksLaneErrorModel<br/>三点预瞄误差模型"]
  S["LksSteerCommand<br/>方向盘转角命令计算"]

  I1 --> P
  I2 --> P
  P --> M
  I3 --> M
  M --> S
  I1 --> S
  I4 --> S

  S --> O1["lksSteerRad"]
  P --> O2["previewDistance"]
  M --> O3["weightedError"]
```

## 7. LKS 预瞄距离：LksPreviewDistance

```mermaid
flowchart LR
  I1["egoSpeed"]
  I2["curvature"]

  B["LksBasePreviewDistance<br/>基础预瞄距离"]
  C["LksCurveScaleSelect<br/>弯道缩放选择"]
  A["LksPreviewScaleApply<br/>预瞄距离缩放"]

  I1 --> B
  I2 --> C
  B --> A
  C --> A
  A --> O["previewDistance"]
```

公式：

```text
basePreviewDistance = l0 + rt * egoSpeed
curvatureScale      = abs(curvature) >= curvatureThreshold ? rAlpha : 1.0
previewDistance     = basePreviewDistance * curvatureScale
```

## 8. LKS 三点预瞄误差：LksLaneErrorModel

```mermaid
flowchart LR
  I1["previewDistance"]
  I2["c0 c1 c2 c3"]

  X["LksPreviewPointSelector<br/>近/中/远预瞄点"]
  N["LksNearPointLaneError<br/>近点误差"]
  M["LksMiddlePointLaneError<br/>中点误差"]
  F["LksFarPointLaneError<br/>远点误差"]
  W["LksWeightedErrorFusion<br/>误差加权融合"]

  I1 --> X
  X --> N
  X --> M
  X --> F
  I2 --> N
  I2 --> M
  I2 --> F
  N --> W
  M --> W
  F --> W
  W --> O["weightedError"]
```

公式：

```text
nearX   = nearPreviewDistance
middleX = previewDistance / 2
farX    = previewDistance

laneError(x) = c0 + c1*x + c2*x^2 + c3*x^3

weightedError = w1*nearError + w2*middleError + w3*farError
```

## 9. LKS 转角命令：LksSteerCommand

```mermaid
flowchart LR
  I1["weightedError"]
  I2["egoSpeed"]
  I3["controlEnabled"]

  R["LksRawSteerFromError<br/>原始转角"]
  V["LksSpeedSquareProtection<br/>车速平方保护"]
  A["LksLateralAccelerationLimit<br/>横向加速度限幅"]
  C["LksSteerClamp<br/>转角裁剪"]
  S["LksSteerScaleApply<br/>转角比例缩放"]
  G["LksEnableSteerGate<br/>使能门控"]

  I1 --> R
  I2 --> V
  V --> A
  R --> C
  A --> C
  C --> S
  S --> G
  I3 --> G
  G --> O["lksSteerRad"]
```

公式：

```text
rawSteerNorm = kp * weightedError
speedSquareSafe = max(egoSpeed^2, speedSquareFloor)
normalizedSteerLimit = atan(ayMax * wheelBase / speedSquareSafe) / frontWheelMaxRad
limitedSteerNorm = clamp(rawSteerNorm, -normalizedSteerLimit, normalizedSteerLimit)
scaledSteerRad = limitedSteerNorm * steerScale
lksSteerRad = controlEnabled ? scaledSteerRad : 0
```
