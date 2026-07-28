#pragma once

#include <cmath>

#include "FuncModule.hpp"

namespace control {

struct LksVehicleInputChannelTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed from chassis/CAN bus, m/s.
    Real brakePressed = 0.0; ///< Brake pedal state from chassis/CAN bus.
    Real driverSteerNorm = 0.0; ///< Driver steering takeover input from chassis/CAN bus.
  };
  struct Output {
    Real egoSpeed = 0.0; ///< Forwarded vehicle speed, m/s.
    Real brakePressed = 0.0; ///< Forwarded brake pedal state.
    Real driverSteerNorm = 0.0; ///< Forwarded driver steering input.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class LksVehicleInputChannel : public FuncModule<LksVehicleInputChannelTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksPerceptionInputChannelTraits {
  struct Input {
    Real c0 = 0.0; ///< Lane polynomial constant coefficient, m.
    Real c1 = 0.0; ///< Lane polynomial first-order coefficient.
    Real c2 = 0.0; ///< Lane polynomial second-order coefficient, 1/m.
    Real c3 = 0.0; ///< Lane polynomial third-order coefficient, 1/m^2.
    Real curvature = 0.0; ///< Road curvature from lane perception, 1/m.
  };
  struct Output {
    Real c0 = 0.0; ///< Forwarded lane polynomial constant coefficient, m.
    Real c1 = 0.0; ///< Forwarded lane polynomial first-order coefficient.
    Real c2 = 0.0; ///< Forwarded lane polynomial second-order coefficient, 1/m.
    Real c3 = 0.0; ///< Forwarded lane polynomial third-order coefficient, 1/m^2.
    Real curvature = 0.0; ///< Forwarded road curvature, 1/m.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class LksPerceptionInputChannel
    : public FuncModule<LksPerceptionInputChannelTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksLowSpeedCheckTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
  };
  struct Output {
    Real speedReady = 0.0; ///< One when speed is high enough for LKS.
  };
  struct Param {
    Real vMin = 1.0; ///< Minimum LKS operating speed, m/s.
  };
  struct State {};
  struct Sub {};
};

class LksLowSpeedCheck : public FuncModule<LksLowSpeedCheckTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksBrakePressedCheckTraits {
  struct Input {
    Real brakePressed = 0.0; ///< Brake pedal state.
  };
  struct Output {
    Real brakeActive = 0.0; ///< One when brake is pressed.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class LksBrakePressedCheck : public FuncModule<LksBrakePressedCheckTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksDriverSteerOverrideCheckTraits {
  struct Input {
    Real driverSteerNorm = 0.0; ///< Driver steering input, normalized.
  };
  struct Output {
    Real driverOverride = 0.0; ///< One when driver steering takeover is active.
  };
  struct Param {
    Real driverSteerThreshold = 0.1; ///< Driver steering takeover threshold.
  };
  struct State {};
  struct Sub {};
};

class LksDriverSteerOverrideCheck
    : public FuncModule<LksDriverSteerOverrideCheckTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksEnableDecisionTraits {
  struct Input {
    Real speedReady = 0.0; ///< Speed condition flag.
    Real brakeActive = 0.0; ///< Brake condition flag.
    Real driverOverride = 0.0; ///< Driver takeover condition flag.
  };
  struct Output {
    Real controlEnabled = 0.0; ///< One when LKS is allowed to control steering.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class LksEnableDecision : public FuncModule<LksEnableDecisionTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksEnableLogicTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
    Real brakePressed = 0.0; ///< Brake pedal state.
    Real driverSteerNorm = 0.0; ///< Driver steering input, normalized.
  };
  struct Output {
    Real controlEnabled = 0.0; ///< LKS control enable flag.
  };
  struct Param {
    Real vMin = 1.0; ///< Minimum LKS operating speed, m/s.
    Real driverSteerThreshold = 0.1; ///< Driver steering takeover threshold.
  };
  struct State {};
  struct Sub {
    LksLowSpeedCheck lowSpeedCheck; ///< 判断自车速度是否达到 LKS 可工作的最低速度阈值。
    LksBrakePressedCheck brakePressedCheck; ///< 判断驾驶员是否踩下制动踏板，制动时应退出 LKS 控制。
    LksDriverSteerOverrideCheck driverSteerOverrideCheck; ///< 判断驾驶员是否主动打方向，主动转向时应由驾驶员接管。
    LksEnableDecision enableDecision; ///< 汇总低速、制动和主动转向条件，输出最终 LKS 控制使能。
  };
};

class LksEnableLogic : public FuncModule<LksEnableLogicTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksDecisionTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed from vehicle input channel, m/s.
    Real brakePressed = 0.0; ///< Brake pedal state from vehicle input channel.
    Real driverSteerNorm = 0.0; ///< Driver steering input from vehicle input channel.
  };
  struct Output {
    Real controlEnabled = 0.0; ///< LKS decision output, one means steering control is allowed.
  };
  struct Param {
    Real vMin = 1.0; ///< Minimum LKS operating speed, m/s.
    Real driverSteerThreshold = 0.1; ///< Driver steering takeover threshold.
  };
  struct State {};
  struct Sub {
    LksEnableLogic enableLogic; ///< LKS 决策层内部的控制使能判断，汇总低速、制动和驾驶员主动转向接管条件。
  };
};

class LksDecision : public FuncModule<LksDecisionTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksBasePreviewDistanceTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
  };
  struct Output {
    Real basePreviewDistance = 5.0; ///< Speed-dependent base preview distance, m.
  };
  struct Param {
    Real l0 = 5.0; ///< Base preview distance offset, m.
    Real rt = 0.5; ///< Speed preview coefficient, s.
  };
  struct State {};
  struct Sub {};
};

class LksBasePreviewDistance : public FuncModule<LksBasePreviewDistanceTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksCurveScaleSelectTraits {
  struct Input {
    Real curvature = 0.0; ///< Road curvature, 1/m.
  };
  struct Output {
    Real curvatureScale = 1.0; ///< Preview distance scale for curve handling.
  };
  struct Param {
    Real rAlpha = 0.6666667; ///< Curve preview distance scale.
    Real curvatureThreshold = 0.001; ///< Curvature threshold for curve detection.
  };
  struct State {};
  struct Sub {};
};

class LksCurveScaleSelect : public FuncModule<LksCurveScaleSelectTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksPreviewScaleApplyTraits {
  struct Input {
    Real basePreviewDistance = 5.0; ///< Base preview distance, m.
    Real curvatureScale = 1.0; ///< Curve scale coefficient.
  };
  struct Output {
    Real previewDistance = 5.0; ///< Final far preview distance, m.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class LksPreviewScaleApply : public FuncModule<LksPreviewScaleApplyTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksPreviewDistanceTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
    Real curvature = 0.0; ///< Road curvature, 1/m.
  };
  struct Output {
    Real previewDistance = 5.0; ///< Far preview distance, m.
  };
  struct Param {
    Real l0 = 5.0; ///< Base preview distance offset, m.
    Real rt = 0.5; ///< Speed preview coefficient, s.
    Real rAlpha = 0.6666667; ///< Curve preview distance scale.
    Real curvatureThreshold = 0.001; ///< Curvature threshold for curve detection.
  };
  struct State {};
  struct Sub {
    LksBasePreviewDistance basePreviewDistance; ///< 根据自车速度计算基础远预瞄距离，速度越高预瞄越远。
    LksCurveScaleSelect curveScaleSelect; ///< 根据道路曲率判断是否进入弯道，并选择弯道预瞄缩放系数。
    LksPreviewScaleApply previewScaleApply; ///< 将基础预瞄距离和弯道缩放系数相乘，得到最终远预瞄距离。
  };
};

class LksPreviewDistance : public FuncModule<LksPreviewDistanceTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksPreviewPointSelectorTraits {
  struct Input {
    Real previewDistance = 5.0; ///< Far preview distance, m.
  };
  struct Output {
    Real nearX = 0.5; ///< Near preview point longitudinal coordinate, m.
    Real middleX = 2.5; ///< Middle preview point longitudinal coordinate, m.
    Real farX = 5.0; ///< Far preview point longitudinal coordinate, m.
  };
  struct Param {
    Real nearPreviewDistance = 0.5; ///< Near preview point distance, m.
  };
  struct State {};
  struct Sub {};
};

class LksPreviewPointSelector : public FuncModule<LksPreviewPointSelectorTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksNearPointLaneErrorTraits {
  struct Input {
    Real c0 = 0.0; ///< Lane polynomial constant coefficient, m.
    Real c1 = 0.0; ///< Lane polynomial first-order coefficient.
    Real c2 = 0.0; ///< Lane polynomial second-order coefficient, 1/m.
    Real c3 = 0.0; ///< Lane polynomial third-order coefficient, 1/m^2.
    Real x = 0.5; ///< Near preview point longitudinal coordinate, m.
  };
  struct Output {
    Real error = 0.0; ///< Lane lateral error at near preview point, m.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class LksNearPointLaneError : public FuncModule<LksNearPointLaneErrorTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksMiddlePointLaneErrorTraits {
  struct Input {
    Real c0 = 0.0; ///< Lane polynomial constant coefficient, m.
    Real c1 = 0.0; ///< Lane polynomial first-order coefficient.
    Real c2 = 0.0; ///< Lane polynomial second-order coefficient, 1/m.
    Real c3 = 0.0; ///< Lane polynomial third-order coefficient, 1/m^2.
    Real x = 2.5; ///< Middle preview point longitudinal coordinate, m.
  };
  struct Output {
    Real error = 0.0; ///< Lane lateral error at middle preview point, m.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class LksMiddlePointLaneError : public FuncModule<LksMiddlePointLaneErrorTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksFarPointLaneErrorTraits {
  struct Input {
    Real c0 = 0.0; ///< Lane polynomial constant coefficient, m.
    Real c1 = 0.0; ///< Lane polynomial first-order coefficient.
    Real c2 = 0.0; ///< Lane polynomial second-order coefficient, 1/m.
    Real c3 = 0.0; ///< Lane polynomial third-order coefficient, 1/m^2.
    Real x = 5.0; ///< Far preview point longitudinal coordinate, m.
  };
  struct Output {
    Real error = 0.0; ///< Lane lateral error at far preview point, m.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class LksFarPointLaneError : public FuncModule<LksFarPointLaneErrorTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksWeightedErrorFusionTraits {
  struct Input {
    Real nearError = 0.0; ///< Near preview lateral error, m.
    Real middleError = 0.0; ///< Middle preview lateral error, m.
    Real farError = 0.0; ///< Far preview lateral error, m.
  };
  struct Output {
    Real weightedError = 0.0; ///< Weighted lane lateral error, m.
  };
  struct Param {
    Real w1 = 0.2; ///< Near preview error weight.
    Real w2 = 0.3; ///< Middle preview error weight.
    Real w3 = 0.5; ///< Far preview error weight.
  };
  struct State {};
  struct Sub {};
};

class LksWeightedErrorFusion : public FuncModule<LksWeightedErrorFusionTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksLaneErrorModelTraits {
  struct Input {
    Real c0 = 0.0; ///< Lane polynomial constant coefficient, m.
    Real c1 = 0.0; ///< Lane polynomial first-order coefficient.
    Real c2 = 0.0; ///< Lane polynomial second-order coefficient, 1/m.
    Real c3 = 0.0; ///< Lane polynomial third-order coefficient, 1/m^2.
    Real previewDistance = 5.0; ///< Far preview distance, m.
  };
  struct Output {
    Real weightedError = 0.0; ///< Weighted lane lateral error, m.
    Real nearError = 0.0; ///< Near preview lateral error, m.
    Real middleError = 0.0; ///< Middle preview lateral error, m.
    Real farError = 0.0; ///< Far preview lateral error, m.
  };
  struct Param {
    Real nearPreviewDistance = 0.5; ///< Near preview point distance, m.
    Real w1 = 0.2; ///< Near preview error weight.
    Real w2 = 0.3; ///< Middle preview error weight.
    Real w3 = 0.5; ///< Far preview error weight.
  };
  struct State {};
  struct Sub {
    LksPreviewPointSelector previewPointSelector; ///< 根据远预瞄距离生成近点、中点、远点三个纵向预瞄位置。
    LksNearPointLaneError nearPointLaneError; ///< 在近预瞄点处计算车道多项式横向误差，反映近处贴线情况。
    LksMiddlePointLaneError middlePointLaneError; ///< 在中预瞄点处计算车道多项式横向误差，反映中距离趋势。
    LksFarPointLaneError farPointLaneError; ///< 在远预瞄点处计算车道多项式横向误差，反映前方道路走向。
    LksWeightedErrorFusion weightedErrorFusion; ///< 将近/中/远三点误差按权重融合成一个控制用横向误差。
  };
};

class LksLaneErrorModel : public FuncModule<LksLaneErrorModelTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksRawSteerFromErrorTraits {
  struct Input {
    Real weightedError = 0.0; ///< Weighted lane lateral error, m.
  };
  struct Output {
    Real rawSteerNorm = 0.0; ///< Raw normalized steering command.
  };
  struct Param {
    Real kp = 0.09; ///< Lateral error proportional gain.
  };
  struct State {};
  struct Sub {};
};

class LksRawSteerFromError : public FuncModule<LksRawSteerFromErrorTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksSpeedSquareProtectionTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
  };
  struct Output {
    Real speedSquareSafe = 0.25; ///< Speed squared with lower bound, m^2/s^2.
  };
  struct Param {
    Real speedSquareFloor = 0.25; ///< Lower bound of speed squared, m^2/s^2.
  };
  struct State {};
  struct Sub {};
};

class LksSpeedSquareProtection
    : public FuncModule<LksSpeedSquareProtectionTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksLateralAccelerationLimitTraits {
  struct Input {
    Real speedSquareSafe = 0.25; ///< Speed squared with lower bound, m^2/s^2.
  };
  struct Output {
    Real normalizedSteerLimit = 1.0; ///< Steering limit normalized by front wheel max angle.
  };
  struct Param {
    Real ayMax = 3.0; ///< Maximum lateral acceleration, m/s^2.
    Real wheelBase = 2.9; ///< Vehicle wheelbase, m.
    Real frontWheelMaxRad = 0.5236; ///< Maximum front wheel steering angle, rad.
  };
  struct State {};
  struct Sub {};
};

class LksLateralAccelerationLimit
    : public FuncModule<LksLateralAccelerationLimitTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksSteerClampTraits {
  struct Input {
    Real rawSteerNorm = 0.0; ///< Raw normalized steering command.
    Real normalizedSteerLimit = 1.0; ///< Symmetric normalized steering limit.
  };
  struct Output {
    Real limitedSteerNorm = 0.0; ///< Steering command after symmetric clamp.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class LksSteerClamp : public FuncModule<LksSteerClampTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksSteerScaleApplyTraits {
  struct Input {
    Real limitedSteerNorm = 0.0; ///< Steering command after clamp.
  };
  struct Output {
    Real scaledSteerRad = 0.0; ///< Scaled steering command, rad.
  };
  struct Param {
    Real steerScale = 0.6; ///< Steering scale from normalized command to radians.
  };
  struct State {};
  struct Sub {};
};

class LksSteerScaleApply : public FuncModule<LksSteerScaleApplyTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksEnableSteerGateTraits {
  struct Input {
    Real scaledSteerRad = 0.0; ///< Steering command before enable gate, rad.
    Real controlEnabled = 0.0; ///< LKS control enable flag.
  };
  struct Output {
    Real steerRad = 0.0; ///< Final steering command, rad.
  };
  struct Param {};
  struct State {};
  struct Sub {};
};

class LksEnableSteerGate : public FuncModule<LksEnableSteerGateTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksSteerCommandTraits {
  struct Input {
    Real weightedError = 0.0; ///< Weighted lane lateral error, m.
    Real egoSpeed = 0.0; ///< Vehicle speed, m/s.
    Real controlEnabled = 0.0; ///< LKS control enable flag.
  };
  struct Output {
    Real steerRad = 0.0; ///< Final steering command, rad.
  };
  struct Param {
    Real kp = 0.09; ///< Lateral error proportional gain.
    Real steerScale = 0.6; ///< Steering scale from normalized command to radians.
    Real ayMax = 3.0; ///< Maximum lateral acceleration, m/s^2.
    Real wheelBase = 2.9; ///< Vehicle wheelbase, m.
    Real frontWheelMaxRad = 0.5236; ///< Maximum front wheel steering angle, rad.
  };
  struct State {};
  struct Sub {
    LksRawSteerFromError rawSteerFromError; ///< 根据加权横向误差和比例增益计算原始归一化转向命令。
    LksSpeedSquareProtection speedSquareProtection; ///< 对车速平方做下限保护，避免低速下横向加速度限幅除零。
    LksLateralAccelerationLimit lateralAccelerationLimit; ///< 根据车速、轴距和最大横向加速度计算动态转角上限。
    LksSteerClamp steerClamp; ///< 使用动态转角上限对原始转向命令做对称限幅。
    LksSteerScaleApply steerScaleApply; ///< 将归一化转向命令换算为弧度单位方向盘/前轮转角命令。
    LksEnableSteerGate enableSteerGate; ///< 根据 LKS 使能状态决定是否真正输出转角命令。
  };
};

class LksSteerCommand : public FuncModule<LksSteerCommandTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksControlTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed from vehicle input channel, m/s.
    Real curvature = 0.0; ///< Road curvature from perception input channel, 1/m.
    Real c0 = 0.0; ///< Lane polynomial constant coefficient, m.
    Real c1 = 0.0; ///< Lane polynomial first-order coefficient.
    Real c2 = 0.0; ///< Lane polynomial second-order coefficient, 1/m.
    Real c3 = 0.0; ///< Lane polynomial third-order coefficient, 1/m^2.
    Real controlEnabled = 0.0; ///< LKS enable decision from decision layer.
  };
  struct Output {
    Real lksSteerRad = 0.0; ///< Final LKS steering command, rad.
    Real previewDistance = 5.0; ///< Far preview distance, m.
    Real weightedError = 0.0; ///< Weighted lane lateral error, m.
  };
  struct Param {
    Real l0 = 5.0; ///< Base preview distance offset, m.
    Real rt = 0.5; ///< Speed preview coefficient, s.
    Real rAlpha = 0.6666667; ///< Curve preview distance scale.
    Real curvatureThreshold = 0.001; ///< Curvature threshold for curve detection.
    Real nearPreviewDistance = 0.5; ///< Near preview point distance, m.
    Real w1 = 0.2; ///< Near preview error weight.
    Real w2 = 0.3; ///< Middle preview error weight.
    Real w3 = 0.5; ///< Far preview error weight.
    Real kp = 0.09; ///< Lateral error proportional gain.
    Real steerScale = 0.6; ///< Steering scale from normalized command to radians.
    Real ayMax = 3.0; ///< Maximum lateral acceleration, m/s^2.
    Real wheelBase = 2.9; ///< Vehicle wheelbase, m.
    Real frontWheelMaxRad = 0.5236; ///< Maximum front wheel steering angle, rad.
  };
  struct State {};
  struct Sub {
    LksPreviewDistance previewDistance; ///< LKS 控制层的远预瞄距离计算，根据车速和曲率确定控制预瞄尺度。
    LksLaneErrorModel laneErrorModel; ///< LKS 控制层的三点预瞄误差模型，计算近/中/远预瞄点横向误差并加权融合。
    LksSteerCommand steerCommand; ///< LKS 控制层的方向盘转角命令计算，将加权误差转换为最终转角输出。
  };
};

class LksControl : public FuncModule<LksControlTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

struct LksMainFlowTraits {
  struct Input {
    Real egoSpeed = 0.0; ///< Vehicle speed from chassis/CAN bus, m/s.
    Real curvature = 0.0; ///< Road curvature from environment perception, 1/m.
    Real c0 = 0.0; ///< Lane polynomial constant coefficient, m.
    Real c1 = 0.0; ///< Lane polynomial first-order coefficient.
    Real c2 = 0.0; ///< Lane polynomial second-order coefficient, 1/m.
    Real c3 = 0.0; ///< Lane polynomial third-order coefficient, 1/m^2.
    Real brakePressed = 0.0; ///< Brake pedal state from chassis/CAN bus.
    Real driverSteerNorm = 0.0; ///< Driver steering takeover input from chassis/CAN bus.
  };
  struct Output {
    Real lksSteerRad = 0.0; ///< Final LKS steering command, rad.
    Real controlEnabled = 0.0; ///< LKS control enable flag.
    Real previewDistance = 5.0; ///< Far preview distance, m.
    Real weightedError = 0.0; ///< Weighted lane lateral error, m.
  };
  struct Param {
    Real l0 = 5.0; ///< Base preview distance offset, m.
    Real rt = 0.5; ///< Speed preview coefficient, s.
    Real rAlpha = 0.6666667; ///< Curve preview distance scale.
    Real curvatureThreshold = 0.001; ///< Curvature threshold for curve detection.
    Real nearPreviewDistance = 0.5; ///< Near preview point distance, m.
    Real w1 = 0.2; ///< Near preview error weight.
    Real w2 = 0.3; ///< Middle preview error weight.
    Real w3 = 0.5; ///< Far preview error weight.
    Real kp = 0.09; ///< Lateral error proportional gain.
    Real steerScale = 0.6; ///< Steering scale from normalized command to radians.
    Real vMin = 1.0; ///< Minimum LKS operating speed, m/s.
    Real driverSteerThreshold = 0.1; ///< Driver steering takeover threshold.
    Real ayMax = 3.0; ///< Maximum lateral acceleration, m/s^2.
    Real wheelBase = 2.9; ///< Vehicle wheelbase, m.
    Real frontWheelMaxRad = 0.5236; ///< Maximum front wheel steering angle, rad.
  };
  struct State {};
  struct Sub {
    LksVehicleInputChannel vehicleInputChannel; ///< 车上信息输入通道，汇总 CAN/车身总线中的车速、制动状态和驾驶员方向盘接管输入。
    LksPerceptionInputChannel perceptionInputChannel; ///< 车外感知输入通道，汇总以太网/感知链路中的车道多项式系数和道路曲率。
    LksDecision decision; ///< LKS 决策层，负责判断当前是否允许 LKS 接管方向盘控制。
    LksControl control; ///< LKS 控制层，负责预瞄距离、三点误差加权和方向盘转角命令计算。
  };
};

class LksMainFlow : public FuncModule<LksMainFlowTraits> {
public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

} // namespace control
