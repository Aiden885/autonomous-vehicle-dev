#include "LKSAlgorithm.hpp"

namespace control {

/**
 * @brief Forward vehicle-bus LKS input signals.
 * @cn_name LKS车上信息输入
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Input
 * @tag_level2 VehicleBus
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksVehicleInputChannel::run(const Input& input, Output& output) {
  output.egoSpeed = input.egoSpeed;
  output.brakePressed = input.brakePressed;
  output.driverSteerNorm = input.driverSteerNorm;
}

/**
 * @brief Forward environment-perception LKS input signals.
 * @cn_name LKS车外感知输入
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Input
 * @tag_level2 EnvironmentPerception
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksPerceptionInputChannel::run(const Input& input, Output& output) {
  output.c0 = input.c0;
  output.c1 = input.c1;
  output.c2 = input.c2;
  output.c3 = input.c3;
  output.curvature = input.curvature;
}

/**
 * @brief Check whether vehicle speed is high enough for LKS.
 * @cn_name LKS低速判断
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Enable
 * @tag_level2 LowSpeedCheck
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksLowSpeedCheck::run(const Input& input, Output& output) {
  output.speedReady = 0.0;
  if (input.egoSpeed >= param_.vMin) {
    output.speedReady = 1.0;
  }
}

/**
 * @brief Check whether brake pedal is pressed.
 * @cn_name LKS制动判断
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Enable
 * @tag_level2 BrakeCheck
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksBrakePressedCheck::run(const Input& input, Output& output) {
  output.brakeActive = 0.0;
  if (input.brakePressed != 0.0) {
    output.brakeActive = 1.0;
  }
}

/**
 * @brief Check whether driver steering takeover is active.
 * @cn_name LKS驾驶员转向接管判断
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Enable
 * @tag_level2 DriverOverrideCheck
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksDriverSteerOverrideCheck::run(const Input& input, Output& output) {
  const Real absDriverSteer0 = std::fabs(input.driverSteerNorm);
  output.driverOverride = 0.0;
  if (absDriverSteer0 >= param_.driverSteerThreshold) {
    output.driverOverride = 1.0;
  }
}

/**
 * @brief Combine physical conditions into LKS enable flag.
 * @cn_name LKS使能决策
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Enable
 * @tag_level2 EnableDecision
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksEnableDecision::run(const Input& input, Output& output) {
  output.controlEnabled = 0.0;
  if (input.speedReady != 0.0 && input.brakeActive == 0.0 &&
      input.driverOverride == 0.0) {
    output.controlEnabled = 1.0;
  }
}

/**
 * @brief Run LKS enable logic from speed, brake and driver takeover.
 * @cn_name LKS控制使能判断
 * @type component
 * @tag_level0 LKS
 * @tag_level1 Enable
 * @tag_level2 EnableLogic
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksEnableLogic::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [LksEnableLogic] ===
  LksLowSpeedCheckTraits::Param lowSpeedParam;
  lowSpeedParam.vMin = param_.vMin;
  sub_.lowSpeedCheck.setParam(lowSpeedParam);

  LksLowSpeedCheckTraits::Input lowSpeedInput;
  lowSpeedInput.egoSpeed = input.egoSpeed;
  LksLowSpeedCheckTraits::Output lowSpeedOutput;
  sub_.lowSpeedCheck.run(lowSpeedInput, lowSpeedOutput);

  LksBrakePressedCheckTraits::Input brakeInput;
  brakeInput.brakePressed = input.brakePressed;
  LksBrakePressedCheckTraits::Output brakeOutput;
  sub_.brakePressedCheck.run(brakeInput, brakeOutput);

  LksDriverSteerOverrideCheckTraits::Param overrideParam;
  overrideParam.driverSteerThreshold = param_.driverSteerThreshold;
  sub_.driverSteerOverrideCheck.setParam(overrideParam);

  LksDriverSteerOverrideCheckTraits::Input overrideInput;
  overrideInput.driverSteerNorm = input.driverSteerNorm;
  LksDriverSteerOverrideCheckTraits::Output overrideOutput;
  sub_.driverSteerOverrideCheck.run(overrideInput, overrideOutput);

  LksEnableDecisionTraits::Input decisionInput;
  decisionInput.speedReady = lowSpeedOutput.speedReady;
  decisionInput.brakeActive = brakeOutput.brakeActive;
  decisionInput.driverOverride = overrideOutput.driverOverride;
  LksEnableDecisionTraits::Output decisionOutput;
  sub_.enableDecision.run(decisionInput, decisionOutput);

  output.controlEnabled = decisionOutput.controlEnabled;
  // === MBD_AUTO_GEN_END [LksEnableLogic] ===
}

/**
 * @brief Run LKS decision layer for steering takeover permission.
 * @cn_name LKS决策层
 * @type component
 * @tag_level0 LKS
 * @tag_level1 Decision
 * @tag_level2 ControlEnableDecision
 * @version 2.1
 * @date 2026-07-09
 * @author ZYK
 */
void LksDecision::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [LksDecision] ===
  LksEnableLogicTraits::Param enableParam;
  enableParam.vMin = param_.vMin;
  enableParam.driverSteerThreshold = param_.driverSteerThreshold;
  sub_.enableLogic.setParam(enableParam);

  LksEnableLogicTraits::Input enableInput;
  enableInput.egoSpeed = input.egoSpeed;
  enableInput.brakePressed = input.brakePressed;
  enableInput.driverSteerNorm = input.driverSteerNorm;
  LksEnableLogicTraits::Output enableOutput;
  sub_.enableLogic.run(enableInput, enableOutput);

  output.controlEnabled = enableOutput.controlEnabled;
  // === MBD_AUTO_GEN_END [LksDecision] ===
}

/**
 * @brief Compute speed-dependent base preview distance.
 * @cn_name LKS基础预瞄距离
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Preview
 * @tag_level2 BasePreviewDistance
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksBasePreviewDistance::run(const Input& input, Output& output) {
  output.basePreviewDistance = param_.l0 + param_.rt * input.egoSpeed;
}

/**
 * @brief Select preview scale from road curvature.
 * @cn_name LKS弯道预瞄缩放
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Preview
 * @tag_level2 CurveScale
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksCurveScaleSelect::run(const Input& input, Output& output) {
  const Real absCurvature0 = std::fabs(input.curvature);
  output.curvatureScale = 1.0;
  if (absCurvature0 >= param_.curvatureThreshold) {
    output.curvatureScale = param_.rAlpha;
  }
}

/**
 * @brief Apply curvature preview scale.
 * @cn_name LKS预瞄距离缩放
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Preview
 * @tag_level2 PreviewScaleApply
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksPreviewScaleApply::run(const Input& input, Output& output) {
  output.previewDistance = input.basePreviewDistance * input.curvatureScale;
}

/**
 * @brief Run far preview distance calculation.
 * @cn_name LKS远预瞄距离计算
 * @type component
 * @tag_level0 LKS
 * @tag_level1 Preview
 * @tag_level2 PreviewDistance
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksPreviewDistance::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [LksPreviewDistance] ===
  LksBasePreviewDistanceTraits::Param baseParam;
  baseParam.l0 = param_.l0;
  baseParam.rt = param_.rt;
  sub_.basePreviewDistance.setParam(baseParam);

  LksBasePreviewDistanceTraits::Input baseInput;
  baseInput.egoSpeed = input.egoSpeed;
  LksBasePreviewDistanceTraits::Output baseOutput;
  sub_.basePreviewDistance.run(baseInput, baseOutput);

  LksCurveScaleSelectTraits::Param scaleParam;
  scaleParam.rAlpha = param_.rAlpha;
  scaleParam.curvatureThreshold = param_.curvatureThreshold;
  sub_.curveScaleSelect.setParam(scaleParam);

  LksCurveScaleSelectTraits::Input scaleInput;
  scaleInput.curvature = input.curvature;
  LksCurveScaleSelectTraits::Output scaleOutput;
  sub_.curveScaleSelect.run(scaleInput, scaleOutput);

  LksPreviewScaleApplyTraits::Input applyInput;
  applyInput.basePreviewDistance = baseOutput.basePreviewDistance;
  applyInput.curvatureScale = scaleOutput.curvatureScale;
  LksPreviewScaleApplyTraits::Output applyOutput;
  sub_.previewScaleApply.run(applyInput, applyOutput);

  output.previewDistance = applyOutput.previewDistance;
  // === MBD_AUTO_GEN_END [LksPreviewDistance] ===
}

/**
 * @brief Select near, middle and far preview points.
 * @cn_name LKS预瞄点选择
 * @type element
 * @tag_level0 LKS
 * @tag_level1 LaneError
 * @tag_level2 PreviewPointSelector
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksPreviewPointSelector::run(const Input& input, Output& output) {
  output.nearX = param_.nearPreviewDistance;
  output.middleX = input.previewDistance / 2.0;
  output.farX = input.previewDistance;
}

/**
 * @brief Evaluate lane polynomial at near preview point.
 * @cn_name LKS近点车道误差
 * @type element
 * @tag_level0 LKS
 * @tag_level1 LaneError
 * @tag_level2 NearPointError
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksNearPointLaneError::run(const Input& input, Output& output) {
  const Real c3x0 = input.c3 * input.x;
  const Real c3xPlusC20 = c3x0 + input.c2;
  const Real secondTerm0 = c3xPlusC20 * input.x;
  const Real firstTerm0 = input.c1 + secondTerm0;
  output.error = input.x * firstTerm0 + input.c0;
}

/**
 * @brief Evaluate lane polynomial at middle preview point.
 * @cn_name LKS中点车道误差
 * @type element
 * @tag_level0 LKS
 * @tag_level1 LaneError
 * @tag_level2 MiddlePointError
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksMiddlePointLaneError::run(const Input& input, Output& output) {
  const Real c3x0 = input.c3 * input.x;
  const Real c3xPlusC20 = c3x0 + input.c2;
  const Real secondTerm0 = c3xPlusC20 * input.x;
  const Real firstTerm0 = input.c1 + secondTerm0;
  output.error = input.x * firstTerm0 + input.c0;
}

/**
 * @brief Evaluate lane polynomial at far preview point.
 * @cn_name LKS远点车道误差
 * @type element
 * @tag_level0 LKS
 * @tag_level1 LaneError
 * @tag_level2 FarPointError
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksFarPointLaneError::run(const Input& input, Output& output) {
  const Real c3x0 = input.c3 * input.x;
  const Real c3xPlusC20 = c3x0 + input.c2;
  const Real secondTerm0 = c3xPlusC20 * input.x;
  const Real firstTerm0 = input.c1 + secondTerm0;
  output.error = input.x * firstTerm0 + input.c0;
}

/**
 * @brief Fuse three preview-point lateral errors by weights.
 * @cn_name LKS误差加权融合
 * @type element
 * @tag_level0 LKS
 * @tag_level1 LaneError
 * @tag_level2 WeightedErrorFusion
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksWeightedErrorFusion::run(const Input& input, Output& output) {
  const Real nearTerm0 = param_.w1 * input.nearError;
  const Real middleTerm0 = param_.w2 * input.middleError;
  const Real farTerm0 = param_.w3 * input.farError;
  output.weightedError = nearTerm0 + middleTerm0 + farTerm0;
}

/**
 * @brief Run three-point lane polynomial error model.
 * @cn_name LKS三点车道误差模型
 * @type component
 * @tag_level0 LKS
 * @tag_level1 LaneError
 * @tag_level2 ThreePointErrorModel
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksLaneErrorModel::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [LksLaneErrorModel] ===
  LksPreviewPointSelectorTraits::Param pointParam;
  pointParam.nearPreviewDistance = param_.nearPreviewDistance;
  sub_.previewPointSelector.setParam(pointParam);

  LksPreviewPointSelectorTraits::Input pointInput;
  pointInput.previewDistance = input.previewDistance;
  LksPreviewPointSelectorTraits::Output pointOutput;
  sub_.previewPointSelector.run(pointInput, pointOutput);

  LksNearPointLaneErrorTraits::Input nearInput;
  nearInput.c0 = input.c0;
  nearInput.c1 = input.c1;
  nearInput.c2 = input.c2;
  nearInput.c3 = input.c3;
  nearInput.x = pointOutput.nearX;
  LksNearPointLaneErrorTraits::Output nearOutput;
  sub_.nearPointLaneError.run(nearInput, nearOutput);

  LksMiddlePointLaneErrorTraits::Input middleInput;
  middleInput.c0 = input.c0;
  middleInput.c1 = input.c1;
  middleInput.c2 = input.c2;
  middleInput.c3 = input.c3;
  middleInput.x = pointOutput.middleX;
  LksMiddlePointLaneErrorTraits::Output middleOutput;
  sub_.middlePointLaneError.run(middleInput, middleOutput);

  LksFarPointLaneErrorTraits::Input farInput;
  farInput.c0 = input.c0;
  farInput.c1 = input.c1;
  farInput.c2 = input.c2;
  farInput.c3 = input.c3;
  farInput.x = pointOutput.farX;
  LksFarPointLaneErrorTraits::Output farOutput;
  sub_.farPointLaneError.run(farInput, farOutput);

  LksWeightedErrorFusionTraits::Param fusionParam;
  fusionParam.w1 = param_.w1;
  fusionParam.w2 = param_.w2;
  fusionParam.w3 = param_.w3;
  sub_.weightedErrorFusion.setParam(fusionParam);

  LksWeightedErrorFusionTraits::Input fusionInput;
  fusionInput.nearError = nearOutput.error;
  fusionInput.middleError = middleOutput.error;
  fusionInput.farError = farOutput.error;
  LksWeightedErrorFusionTraits::Output fusionOutput;
  sub_.weightedErrorFusion.run(fusionInput, fusionOutput);

  output.weightedError = fusionOutput.weightedError;
  output.nearError = nearOutput.error;
  output.middleError = middleOutput.error;
  output.farError = farOutput.error;
  // === MBD_AUTO_GEN_END [LksLaneErrorModel] ===
}

/**
 * @brief Convert weighted lane error to raw steering command.
 * @cn_name LKS原始转向计算
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Steer
 * @tag_level2 RawSteer
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksRawSteerFromError::run(const Input& input, Output& output) {
  output.rawSteerNorm = param_.kp * input.weightedError;
}

/**
 * @brief Protect speed-squared denominator used by lateral acceleration limit.
 * @cn_name LKS车速平方保护
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Steer
 * @tag_level2 SpeedSquareProtection
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksSpeedSquareProtection::run(const Input& input, Output& output) {
  const Real speedSquare0 = input.egoSpeed * input.egoSpeed;
  output.speedSquareSafe = speedSquare0;
  if (output.speedSquareSafe < param_.speedSquareFloor) {
    output.speedSquareSafe = param_.speedSquareFloor;
  }
}

/**
 * @brief Compute steering limit from lateral acceleration constraint.
 * @cn_name LKS横向加速度限幅
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Steer
 * @tag_level2 LateralAccelerationLimit
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksLateralAccelerationLimit::run(const Input& input, Output& output) {
  const Real lateralAccelNumerator0 = param_.ayMax * param_.wheelBase;
  const Real lateralAccelRatio0 = lateralAccelNumerator0 / input.speedSquareSafe;
  const Real lateralAccelLimitRad0 = std::atan(lateralAccelRatio0);
  output.normalizedSteerLimit = lateralAccelLimitRad0 / param_.frontWheelMaxRad;
}

/**
 * @brief Clamp raw steering command by speed-dependent limit.
 * @cn_name LKS转向限幅裁剪
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Steer
 * @tag_level2 SteerClamp
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksSteerClamp::run(const Input& input, Output& output) {
  const Real negativeLimit0 = -input.normalizedSteerLimit;
  output.limitedSteerNorm = input.rawSteerNorm;
  if (output.limitedSteerNorm > input.normalizedSteerLimit) {
    output.limitedSteerNorm = input.normalizedSteerLimit;
  }
  if (output.limitedSteerNorm < negativeLimit0) {
    output.limitedSteerNorm = negativeLimit0;
  }
}

/**
 * @brief Convert normalized steering command to radians.
 * @cn_name LKS转向比例缩放
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Steer
 * @tag_level2 SteerScale
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksSteerScaleApply::run(const Input& input, Output& output) {
  output.scaledSteerRad = input.limitedSteerNorm * param_.steerScale;
}

/**
 * @brief Gate steering command by LKS enable flag.
 * @cn_name LKS转向使能门控
 * @type element
 * @tag_level0 LKS
 * @tag_level1 Steer
 * @tag_level2 EnableSteerGate
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksEnableSteerGate::run(const Input& input, Output& output) {
  output.steerRad = 0.0;
  if (input.controlEnabled != 0.0) {
    output.steerRad = input.scaledSteerRad;
  }
}

/**
 * @brief Run LKS steering command calculation.
 * @cn_name LKS方向盘转角命令计算
 * @type component
 * @tag_level0 LKS
 * @tag_level1 Steer
 * @tag_level2 SteerCommand
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksSteerCommand::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [LksSteerCommand] ===
  LksRawSteerFromErrorTraits::Param rawParam;
  rawParam.kp = param_.kp;
  sub_.rawSteerFromError.setParam(rawParam);

  LksRawSteerFromErrorTraits::Input rawInput;
  rawInput.weightedError = input.weightedError;
  LksRawSteerFromErrorTraits::Output rawOutput;
  sub_.rawSteerFromError.run(rawInput, rawOutput);

  LksSpeedSquareProtectionTraits::Input speedInput;
  speedInput.egoSpeed = input.egoSpeed;
  LksSpeedSquareProtectionTraits::Output speedOutput;
  sub_.speedSquareProtection.run(speedInput, speedOutput);

  LksLateralAccelerationLimitTraits::Param limitParam;
  limitParam.ayMax = param_.ayMax;
  limitParam.wheelBase = param_.wheelBase;
  limitParam.frontWheelMaxRad = param_.frontWheelMaxRad;
  sub_.lateralAccelerationLimit.setParam(limitParam);

  LksLateralAccelerationLimitTraits::Input limitInput;
  limitInput.speedSquareSafe = speedOutput.speedSquareSafe;
  LksLateralAccelerationLimitTraits::Output limitOutput;
  sub_.lateralAccelerationLimit.run(limitInput, limitOutput);

  LksSteerClampTraits::Input clampInput;
  clampInput.rawSteerNorm = rawOutput.rawSteerNorm;
  clampInput.normalizedSteerLimit = limitOutput.normalizedSteerLimit;
  LksSteerClampTraits::Output clampOutput;
  sub_.steerClamp.run(clampInput, clampOutput);

  LksSteerScaleApplyTraits::Param scaleParam;
  scaleParam.steerScale = param_.steerScale;
  sub_.steerScaleApply.setParam(scaleParam);

  LksSteerScaleApplyTraits::Input scaleInput;
  scaleInput.limitedSteerNorm = clampOutput.limitedSteerNorm;
  LksSteerScaleApplyTraits::Output scaleOutput;
  sub_.steerScaleApply.run(scaleInput, scaleOutput);

  LksEnableSteerGateTraits::Input gateInput;
  gateInput.scaledSteerRad = scaleOutput.scaledSteerRad;
  gateInput.controlEnabled = input.controlEnabled;
  LksEnableSteerGateTraits::Output gateOutput;
  sub_.enableSteerGate.run(gateInput, gateOutput);

  output.steerRad = gateOutput.steerRad;
  // === MBD_AUTO_GEN_END [LksSteerCommand] ===
}

/**
 * @brief Run LKS control layer from lane model to steering command.
 * @cn_name LKS控制层
 * @type component
 * @tag_level0 LKS
 * @tag_level1 Control
 * @tag_level2 PreviewErrorSteerControl
 * @version 2.1
 * @date 2026-07-09
 * @author ZYK
 */
void LksControl::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [LksControl] ===
  LksPreviewDistanceTraits::Param previewParam;
  previewParam.l0 = param_.l0;
  previewParam.rt = param_.rt;
  previewParam.rAlpha = param_.rAlpha;
  previewParam.curvatureThreshold = param_.curvatureThreshold;
  sub_.previewDistance.setParam(previewParam);

  LksPreviewDistanceTraits::Input previewInput;
  previewInput.egoSpeed = input.egoSpeed;
  previewInput.curvature = input.curvature;
  LksPreviewDistanceTraits::Output previewOutput;
  sub_.previewDistance.run(previewInput, previewOutput);

  LksLaneErrorModelTraits::Param laneErrorParam;
  laneErrorParam.nearPreviewDistance = param_.nearPreviewDistance;
  laneErrorParam.w1 = param_.w1;
  laneErrorParam.w2 = param_.w2;
  laneErrorParam.w3 = param_.w3;
  sub_.laneErrorModel.setParam(laneErrorParam);

  LksLaneErrorModelTraits::Input laneErrorInput;
  laneErrorInput.c0 = input.c0;
  laneErrorInput.c1 = input.c1;
  laneErrorInput.c2 = input.c2;
  laneErrorInput.c3 = input.c3;
  laneErrorInput.previewDistance = previewOutput.previewDistance;
  LksLaneErrorModelTraits::Output laneErrorOutput;
  sub_.laneErrorModel.run(laneErrorInput, laneErrorOutput);

  LksSteerCommandTraits::Param steerParam;
  steerParam.kp = param_.kp;
  steerParam.steerScale = param_.steerScale;
  steerParam.ayMax = param_.ayMax;
  steerParam.wheelBase = param_.wheelBase;
  steerParam.frontWheelMaxRad = param_.frontWheelMaxRad;
  sub_.steerCommand.setParam(steerParam);

  LksSteerCommandTraits::Input steerInput;
  steerInput.weightedError = laneErrorOutput.weightedError;
  steerInput.egoSpeed = input.egoSpeed;
  steerInput.controlEnabled = input.controlEnabled;
  LksSteerCommandTraits::Output steerOutput;
  sub_.steerCommand.run(steerInput, steerOutput);

  output.lksSteerRad = steerOutput.steerRad;
  output.previewDistance = previewOutput.previewDistance;
  output.weightedError = laneErrorOutput.weightedError;
  // === MBD_AUTO_GEN_END [LksControl] ===
}

/**
 * @brief Run LKS lane keeping pipeline.
 * @cn_name LKS车道保持主流程
 * @type component
 * @tag_level0 LKS
 * @tag_level1 ClosedLoop
 * @tag_level2 MainFlow
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void LksMainFlow::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [LksMainFlow] ===
  LksVehicleInputChannelTraits::Input vehicleInput;
  vehicleInput.egoSpeed = input.egoSpeed;
  vehicleInput.brakePressed = input.brakePressed;
  vehicleInput.driverSteerNorm = input.driverSteerNorm;
  LksVehicleInputChannelTraits::Output vehicleOutput;
  sub_.vehicleInputChannel.run(vehicleInput, vehicleOutput);

  LksPerceptionInputChannelTraits::Input environmentInput;
  environmentInput.c0 = input.c0;
  environmentInput.c1 = input.c1;
  environmentInput.c2 = input.c2;
  environmentInput.c3 = input.c3;
  environmentInput.curvature = input.curvature;
  LksPerceptionInputChannelTraits::Output environmentOutput;
  sub_.perceptionInputChannel.run(environmentInput, environmentOutput);

  LksDecisionTraits::Param decisionParam;
  decisionParam.vMin = param_.vMin;
  decisionParam.driverSteerThreshold = param_.driverSteerThreshold;
  sub_.decision.setParam(decisionParam);

  LksDecisionTraits::Input decisionInput;
  decisionInput.egoSpeed = vehicleOutput.egoSpeed;
  decisionInput.brakePressed = vehicleOutput.brakePressed;
  decisionInput.driverSteerNorm = vehicleOutput.driverSteerNorm;
  LksDecisionTraits::Output decisionOutput;
  sub_.decision.run(decisionInput, decisionOutput);

  LksControlTraits::Param controlParam;
  controlParam.l0 = param_.l0;
  controlParam.rt = param_.rt;
  controlParam.rAlpha = param_.rAlpha;
  controlParam.curvatureThreshold = param_.curvatureThreshold;
  controlParam.nearPreviewDistance = param_.nearPreviewDistance;
  controlParam.w1 = param_.w1;
  controlParam.w2 = param_.w2;
  controlParam.w3 = param_.w3;
  controlParam.kp = param_.kp;
  controlParam.steerScale = param_.steerScale;
  controlParam.ayMax = param_.ayMax;
  controlParam.wheelBase = param_.wheelBase;
  controlParam.frontWheelMaxRad = param_.frontWheelMaxRad;
  sub_.control.setParam(controlParam);

  LksControlTraits::Input controlInput;
  controlInput.egoSpeed = vehicleOutput.egoSpeed;
  controlInput.curvature = environmentOutput.curvature;
  controlInput.c0 = environmentOutput.c0;
  controlInput.c1 = environmentOutput.c1;
  controlInput.c2 = environmentOutput.c2;
  controlInput.c3 = environmentOutput.c3;
  controlInput.controlEnabled = decisionOutput.controlEnabled;
  LksControlTraits::Output controlOutput;
  sub_.control.run(controlInput, controlOutput);

  output.lksSteerRad = controlOutput.lksSteerRad;
  output.controlEnabled = decisionOutput.controlEnabled;
  output.previewDistance = controlOutput.previewDistance;
  output.weightedError = controlOutput.weightedError;
  // === MBD_AUTO_GEN_END [LksMainFlow] ===
}

} // namespace control
