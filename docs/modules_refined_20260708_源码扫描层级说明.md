# ACC/LKS 扫描源码层级说明

## 1. 源码包

- 新源码目录：`generated/modules_refined_20260708`
- 新扫描包：`/home/aiden/文档/temp/modules_refined_20260708.tar.gz`
- 原始参考包：`/home/aiden/文档/temp/modules.tar.gz` 未覆盖

## 2. ACC 层级

ACC 顶层入口已从 `AccTargetSpeed` 调整为 `AccMainFlow`。

`AccMainFlow` 的一级子模块为：

1. `AccVehicleInputChannel`
   - 车上信息输入通道，对应 CAN/车身总线
   - 汇总 `egoSpeed`、`commandType`
   - 作用是把自车速度和驾驶员 ACC 物理按键指令作为一组车上信号传入后续算法

2. `AccPerceptionInputChannel`
   - 车外感知输入通道，对应以太网/感知链路
   - 汇总 `leadSpeed`、`leadDistance`
   - 作用是把前车速度和前车距离作为一组环境感知信号传入后续算法

3. `AccDecision`
   - ACC 决策层
   - 负责状态判断、按键指令解释、启停控制、时距更新、限速更新和历史状态维护
   - 内部子模块：
     - `AccControlStateUpdate`：统一更新 ACC 系统状态、决策码、当前 enable 和下一周期在控记忆
       - `AccSystemStateClassifier`：根据车速、上一周期在控记忆、历史状态判断 ACC 状态
       - `AccDecisionTable`：根据系统状态和驾驶指令输出决策码
       - `AccControlMemoryUpdate`：根据启控/取消决策更新下一周期在控状态记忆
       - `AccHistoryUpdate`：维护是否存在可恢复的历史巡航状态
     - `AccTimeGapUpdate`：根据决策调整跟车时距
     - `AccMaxSpeedUpdate`：根据决策调整巡航限速

4. `AccSpeedControl`
   - ACC 控制层
   - 负责根据自车速度、前车速度、前车距离、时距和限速计算目标速度
   - 内部子模块：
     - `AccDesiredDistance`：由自车速度和时距计算目标车距
     - `AccDistanceError`：计算实际车距与目标车距误差
     - `AccRelativeSpeed`：计算前车速度与自车速度差
     - `AccRawTargetSpeed`：计算未限幅目标速度
     - `AccTargetSpeedLimit`：对目标速度做上下限约束
     - `AccEnableGate`：根据 ACC 使能决定是否输出有效目标速度

## 3. LKS 层级

LKS 顶层入口为 `LksMainFlow`。

`LksMainFlow` 的一级子模块为：

1. `LksVehicleInputChannel`
   - 车上信息输入通道，对应 CAN/车身总线
   - 汇总 `egoSpeed`、`brakePressed`、`driverSteerNorm`
   - 作用是把车速、制动状态和驾驶员主动转向输入作为一组车上信号传入后续算法

2. `LksPerceptionInputChannel`
   - 车外感知输入通道，对应以太网/感知链路
   - 汇总 `c0`、`c1`、`c2`、`c3`、`curvature`
   - 作用是把车道多项式和道路曲率作为一组环境感知信号传入后续算法

3. `LksDecision`
   - LKS 决策层
   - 负责判断当前是否允许 LKS 接管方向盘控制
   - 内部子模块：
     - `LksEnableLogic`：控制使能判断
       - `LksLowSpeedCheck`：判断车速是否低于 LKS 工作阈值
       - `LksBrakePressedCheck`：判断驾驶员是否踩刹车
       - `LksDriverSteerOverrideCheck`：判断驾驶员是否主动打方向
       - `LksEnableDecision`：汇总上述条件并输出 LKS 使能

4. `LksControl`
   - LKS 控制层
   - 负责预瞄距离计算、三点误差建模和方向盘转角命令计算
   - 内部子模块：
     - `LksPreviewDistance`：远预瞄距离计算
       - `LksBasePreviewDistance`：根据车速计算基础远预瞄距离
       - `LksCurveScaleSelect`：根据曲率选择弯道缩放系数
       - `LksPreviewScaleApply`：应用缩放得到最终远预瞄距离
     - `LksLaneErrorModel`：三点预瞄误差模型
       - `LksPreviewPointSelector`：生成近/中/远三个预瞄点
       - `LksNearPointLaneError`：计算近预瞄点横向误差
       - `LksMiddlePointLaneError`：计算中预瞄点横向误差
       - `LksFarPointLaneError`：计算远预瞄点横向误差
       - `LksWeightedErrorFusion`：三点误差加权融合
     - `LksSteerCommand`：方向盘转角命令计算
       - `LksRawSteerFromError`：根据加权误差计算原始转向命令
       - `LksSpeedSquareProtection`：车速平方保护，避免低速除零
       - `LksLateralAccelerationLimit`：根据横向加速度约束计算转角限幅
       - `LksSteerClamp`：转角归一化限幅
       - `LksSteerScaleApply`：归一化转角转换为弧度
       - `LksEnableSteerGate`：根据 LKS 使能决定是否输出转角

## 4. 命名调整

- ACC 顶层组件名：`AccMainFlow`
- ACC 输入通道：`AccVehicleInputChannel`、`AccPerceptionInputChannel`
- LKS 顶层组件名：`LksMainFlow`
- LKS 输入通道：`LksVehicleInputChannel`、`LksPerceptionInputChannel`
- LKS 算法接口统一使用 `c0`、`c1`、`c2`、`c3`、`curvature`
- ZMQ 兼容层仍可读取 CARLA Bridge JSON 中的 `c0_m`、`c2_per_m`、`c3_per_m2`、`curvature_per_m`
- 进入 Pangu/画布后的字段不再使用 `c0_m -> c0` 这种不一致连接

## 5. 格式约束

- 每个模块均保留 `XxxTraits`
- 每个 `Traits` 均包含且仅包含 `Input`、`Output`、`Param`、`State`、`Sub`
- `Input/Output/Param/State` 字段均使用 C 风格基础类型
- `Sub` 只保存子模块对象，不保存指针、引用或容器
- 组件级 `run()` 保留 `MBD_AUTO_GEN_BEGIN/END`
