#include "AccMainFlow.hpp"

namespace control {

/**
 * @brief Forward vehicle-bus ACC input signals.
 * @cn_name ACC车上信息输入
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Input
 * @tag_level2 VehicleBus
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccVehicleInputChannel::run(const Input& input, Output& output) {
  /** @brief Keep chassis speed and driver button command as vehicle-bus signals. */
  output.egoSpeed = input.egoSpeed;
  output.commandType = input.commandType;
}

/**
 * @brief Forward environment-perception ACC input signals.
 * @cn_name ACC车外感知输入
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Input
 * @tag_level2 EnvironmentPerception
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccPerceptionInputChannel::run(const Input& input, Output& output) {
  /** @brief Keep lead vehicle state as environment-perception signals. */
  output.leadSpeed = input.leadSpeed;
  output.leadDistance = input.leadDistance;
}

/**
 * @brief Classify ACC operating state from speed and latch memory.
 * @cn_name ACC系统状态判断
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Decision
 * @tag_level2 SystemState
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccSystemStateClassifier::run(const Input& input, Output& output) {
  /** @brief Default state means standby without historical cruise settings. */
  output.systemState = 2;
  if (input.egoSpeed < param_.vMin) {
    output.systemState = 3;
  } else if (input.controlEnabled != 0) {
    output.systemState = 0;
  } else if (input.hasHistory != 0) {
    output.systemState = 1;
  }
}

/**
 * @brief Map ACC system state and driver command to decision code.
 * @cn_name ACC决策表
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Decision
 * @tag_level2 DecisionTable
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccDecisionTable::run(const Input& input, Output& output) {
  /** @brief Decision 8 is the cancel or no-action fallback. */
  output.decision = 8;
  if (input.systemState == 0) {
    if (input.commandType == 1) {
      output.decision = 1;
    } else if (input.commandType == 2) {
      output.decision = 2;
    } else if (input.commandType == 3) {
      output.decision = 3;
    } else if (input.commandType == 4) {
      output.decision = 4;
    } else if (input.commandType == 5) {
      output.decision = 7;
    } else if (input.commandType == 6 || input.commandType == 7) {
      output.decision = 8;
    }
  } else if (input.systemState == 1) {
    if (input.commandType == 1) {
      output.decision = 5;
    } else if (input.commandType == 2) {
      output.decision = 6;
    } else if (input.commandType == 6 || input.commandType == 7) {
      output.decision = 8;
    }
  } else if (input.systemState == 2) {
    if (input.commandType == 1) {
      output.decision = 5;
    } else if (input.commandType == 6 || input.commandType == 7) {
      output.decision = 8;
    }
  } else if (input.systemState == 3) {
    if (input.commandType == 6 || input.commandType == 7) {
      output.decision = 8;
    }
  }
}

/**
 * @brief Update ACC control memory and expose current-cycle enable.
 * @cn_name ACC在控状态记忆更新
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Decision
 * @tag_level2 ControlMemoryUpdate
 * @version 2.1
 * @date 2026-07-09
 * @author ZYK
 */
void AccControlMemoryUpdate::run(const Input& input, Output& output) {
  const int turnOn = (input.decision == 5 || input.decision == 6) ? 1 : 0;
  const int cancel = (input.commandType == 6 || input.commandType == 7) ? 1 : 0;

  /** @brief Current-cycle output uses previous control memory, matching control latency. */
  output.enableOutput = (input.egoSpeed >= param_.vMin &&
                         input.previousControlEnabled != 0)
                            ? 1
                            : 0;
  output.nextControlEnabled =
      ((input.previousControlEnabled != 0 || turnOn != 0) && cancel == 0) ? 1 : 0;
}

/**
 * @brief Update ACC following time gap according to driver decision.
 * @cn_name ACC时距更新
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Decision
 * @tag_level2 TimeGapUpdate
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccTimeGapUpdate::run(const Input& input, Output& output) {
  Real nextTimeGap0 = input.previousTimeGap;
  if (input.decision == 3) {
    nextTimeGap0 = nextTimeGap0 - param_.timeGapStep;
  } else if (input.decision == 4) {
    nextTimeGap0 = nextTimeGap0 + param_.timeGapStep;
  }
  if (nextTimeGap0 < param_.minTimeGap) {
    nextTimeGap0 = param_.minTimeGap;
  }
  if (nextTimeGap0 > param_.maxTimeGap) {
    nextTimeGap0 = param_.maxTimeGap;
  }
  output.nextTimeGap = nextTimeGap0;
}

/**
 * @brief Update ACC maximum cruise speed according to driver decision.
 * @cn_name ACC限速更新
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Decision
 * @tag_level2 MaxSpeedUpdate
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccMaxSpeedUpdate::run(const Input& input, Output& output) {
  Real nextMaxSpeed0 = input.previousMaxSpeed;
  if (input.decision == 1) {
    nextMaxSpeed0 = nextMaxSpeed0 - param_.speedStep;
  } else if (input.decision == 2) {
    nextMaxSpeed0 = nextMaxSpeed0 + param_.speedStep;
  }
  if (nextMaxSpeed0 < param_.minSpeed) {
    nextMaxSpeed0 = param_.minSpeed;
  }
  if (nextMaxSpeed0 > param_.maxSpeedCap) {
    nextMaxSpeed0 = param_.maxSpeedCap;
  }
  output.nextMaxSpeed = nextMaxSpeed0;
}

/**
 * @brief Update ACC historical cruise-setting flag.
 * @cn_name ACC历史状态更新
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Decision
 * @tag_level2 HistoryUpdate
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccHistoryUpdate::run(const Input& input, Output& output) {
  output.nextHasHistory =
      (input.previousHasHistory != 0 || input.nextControlEnabled != 0) ? 1 : 0;
}

/**
 * @brief Update ACC system state, decision and control-state memory.
 * @cn_name ACC在控状态更新
 * @type component
 * @tag_level0 ACC
 * @tag_level1 Decision
 * @tag_level2 ControlStateUpdate
 * @version 2.1
 * @date 2026-07-09
 * @author ZYK
 */
void AccControlStateUpdate::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [AccControlStateUpdate] ===
  AccSystemStateClassifierTraits::Param systemStateParam;
  systemStateParam.vMin = param_.vMin;
  sub_.systemStateClassifier.setParam(systemStateParam);

  AccSystemStateClassifierTraits::Input systemStateInput;
  systemStateInput.egoSpeed = input.egoSpeed;
  systemStateInput.controlEnabled = input.previousControlEnabled;
  systemStateInput.hasHistory = input.previousHasHistory;
  AccSystemStateClassifierTraits::Output systemStateOutput;
  sub_.systemStateClassifier.run(systemStateInput, systemStateOutput);

  AccDecisionTableTraits::Input decisionInput;
  decisionInput.systemState = systemStateOutput.systemState;
  decisionInput.commandType = input.commandType;
  AccDecisionTableTraits::Output decisionOutput;
  sub_.decisionTable.run(decisionInput, decisionOutput);

  AccControlMemoryUpdateTraits::Param memoryParam;
  memoryParam.vMin = param_.vMin;
  sub_.controlMemoryUpdate.setParam(memoryParam);

  AccControlMemoryUpdateTraits::Input memoryInput;
  memoryInput.egoSpeed = input.egoSpeed;
  memoryInput.previousControlEnabled = input.previousControlEnabled;
  memoryInput.decision = decisionOutput.decision;
  memoryInput.commandType = input.commandType;
  AccControlMemoryUpdateTraits::Output memoryOutput;
  sub_.controlMemoryUpdate.run(memoryInput, memoryOutput);

  AccHistoryUpdateTraits::Input historyInput;
  historyInput.previousHasHistory = input.previousHasHistory;
  historyInput.nextControlEnabled = memoryOutput.nextControlEnabled;
  AccHistoryUpdateTraits::Output historyOutput;
  sub_.historyUpdate.run(historyInput, historyOutput);

  output.enable = memoryOutput.enableOutput;
  output.nextControlEnabled = memoryOutput.nextControlEnabled;
  output.nextHasHistory = historyOutput.nextHasHistory;
  output.decision = decisionOutput.decision;
  output.systemState = systemStateOutput.systemState;
  // === MBD_AUTO_GEN_END [AccControlStateUpdate] ===
}

/**
 * @brief Run ACC decision and parameter update logic.
 * @cn_name ACC决策组件
 * @type component
 * @tag_level0 ACC
 * @tag_level1 Decision
 * @tag_level2 DecisionAndSettings
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccDecision::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [AccDecision] ===
  /** @brief Initialize stored time gap and cruise speed once. */
  if (state_.initialized == 0) {
    state_.timeGap = param_.initialTimeGap;
    state_.maxSpeed = param_.initialMaxSpeed;
    state_.initialized = 1;
  }

  AccControlStateUpdateTraits::Param controlStateParam;
  controlStateParam.vMin = param_.vMin;
  sub_.controlStateUpdate.setParam(controlStateParam);

  AccControlStateUpdateTraits::Input controlStateInput;
  controlStateInput.egoSpeed = input.egoSpeed;
  controlStateInput.commandType = input.commandType;
  controlStateInput.previousControlEnabled = state_.controlEnabled;
  controlStateInput.previousHasHistory = state_.hasHistory;
  AccControlStateUpdateTraits::Output controlStateOutput;
  sub_.controlStateUpdate.run(controlStateInput, controlStateOutput);

  AccTimeGapUpdateTraits::Param timeGapParam;
  timeGapParam.timeGapStep = param_.timeGapStep;
  timeGapParam.minTimeGap = param_.minTimeGap;
  timeGapParam.maxTimeGap = param_.maxTimeGap;
  sub_.timeGapUpdate.setParam(timeGapParam);

  AccTimeGapUpdateTraits::Input timeGapInput;
  timeGapInput.previousTimeGap = state_.timeGap;
  timeGapInput.decision = controlStateOutput.decision;
  AccTimeGapUpdateTraits::Output timeGapOutput;
  sub_.timeGapUpdate.run(timeGapInput, timeGapOutput);

  AccMaxSpeedUpdateTraits::Param maxSpeedParam;
  maxSpeedParam.speedStep = param_.speedStep;
  maxSpeedParam.minSpeed = param_.minSpeed;
  maxSpeedParam.maxSpeedCap = param_.maxSpeedCap;
  sub_.maxSpeedUpdate.setParam(maxSpeedParam);

  AccMaxSpeedUpdateTraits::Input maxSpeedInput;
  maxSpeedInput.previousMaxSpeed = state_.maxSpeed;
  maxSpeedInput.decision = controlStateOutput.decision;
  AccMaxSpeedUpdateTraits::Output maxSpeedOutput;
  sub_.maxSpeedUpdate.run(maxSpeedInput, maxSpeedOutput);

  state_.controlEnabled = controlStateOutput.nextControlEnabled;
  state_.hasHistory = controlStateOutput.nextHasHistory;
  state_.timeGap = timeGapOutput.nextTimeGap;
  state_.maxSpeed = maxSpeedOutput.nextMaxSpeed;
  if (controlStateOutput.decision >= 1 && controlStateOutput.decision <= 7) {
    state_.lastDecision = controlStateOutput.decision;
  }

  output.enable = controlStateOutput.enable;
  output.timeGap = timeGapOutput.nextTimeGap;
  output.maxSpeed = maxSpeedOutput.nextMaxSpeed;
  output.decision = controlStateOutput.decision;
  output.systemState = controlStateOutput.systemState;
  // === MBD_AUTO_GEN_END [AccDecision] ===
}

/**
 * @brief Convert time gap to desired following distance.
 * @cn_name ACC目标距离
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Control
 * @tag_level2 DesiredDistance
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccDesiredDistance::run(const Input& input, Output& output) {
  const Real timeGapDistance0 = input.egoSpeed * input.timeGap;
  output.desiredDistance = param_.minDistance;
  if (timeGapDistance0 > param_.minDistance) {
    output.desiredDistance = timeGapDistance0;
  }
}

/**
 * @brief Compute gap error between actual and desired distance.
 * @cn_name ACC距离误差
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Control
 * @tag_level2 DistanceError
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccDistanceError::run(const Input& input, Output& output) {
  output.distanceError = input.leadDistance - input.desiredDistance;
}

/**
 * @brief Compute lead-minus-ego relative speed.
 * @cn_name ACC相对速度
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Control
 * @tag_level2 RelativeSpeed
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccRelativeSpeed::run(const Input& input, Output& output) {
  output.relativeSpeed = input.leadSpeed - input.egoSpeed;
}

/**
 * @brief Compute unbounded ACC target speed.
 * @cn_name ACC未限幅目标车速
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Control
 * @tag_level2 RawTargetSpeed
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccRawTargetSpeed::run(const Input& input, Output& output) {
  const Real distanceTerm0 = param_.distanceGain * input.distanceError;
  const Real speedTerm0 = param_.speedGain * input.relativeSpeed;
  output.rawTargetSpeed = input.leadSpeed + distanceTerm0 + speedTerm0;
}

/**
 * @brief Limit target speed by zero and current cruise speed.
 * @cn_name ACC目标车速限幅
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Control
 * @tag_level2 TargetSpeedLimit
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccTargetSpeedLimit::run(const Input& input, Output& output) {
  output.limitedTargetSpeed = input.rawTargetSpeed;
  if (output.limitedTargetSpeed < 0.0) {
    output.limitedTargetSpeed = 0.0;
  }
  if (output.limitedTargetSpeed > input.maxSpeed) {
    output.limitedTargetSpeed = input.maxSpeed;
  }
}

/**
 * @brief Gate ACC speed command by enable state.
 * @cn_name ACC控制使能门控
 * @type element
 * @tag_level0 ACC
 * @tag_level1 Control
 * @tag_level2 EnableGate
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccEnableGate::run(const Input& input, Output& output) {
  output.targetSpeed = 0.0;
  output.valid = 0;
  if (input.enable != 0) {
    output.targetSpeed = input.limitedTargetSpeed;
    output.valid = 1;
  }
}

/**
 * @brief Run ACC longitudinal speed control from lead vehicle tracking error.
 * @cn_name ACC控制组件
 * @type component
 * @tag_level0 ACC
 * @tag_level1 Control
 * @tag_level2 LongitudinalSpeedControl
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccSpeedControl::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [AccSpeedControl] ===
  AccDesiredDistanceTraits::Param desiredParam;
  desiredParam.minDistance = param_.minDistance;
  sub_.desiredDistance.setParam(desiredParam);

  AccDesiredDistanceTraits::Input desiredInput;
  desiredInput.egoSpeed = input.egoSpeed;
  desiredInput.timeGap = input.timeGap;
  AccDesiredDistanceTraits::Output desiredOutput;
  sub_.desiredDistance.run(desiredInput, desiredOutput);

  AccDistanceErrorTraits::Input distanceErrorInput;
  distanceErrorInput.leadDistance = input.leadDistance;
  distanceErrorInput.desiredDistance = desiredOutput.desiredDistance;
  AccDistanceErrorTraits::Output distanceErrorOutput;
  sub_.distanceError.run(distanceErrorInput, distanceErrorOutput);

  AccRelativeSpeedTraits::Input relativeSpeedInput;
  relativeSpeedInput.egoSpeed = input.egoSpeed;
  relativeSpeedInput.leadSpeed = input.leadSpeed;
  AccRelativeSpeedTraits::Output relativeSpeedOutput;
  sub_.relativeSpeed.run(relativeSpeedInput, relativeSpeedOutput);

  AccRawTargetSpeedTraits::Param rawTargetParam;
  rawTargetParam.distanceGain = param_.distanceGain;
  rawTargetParam.speedGain = param_.speedGain;
  sub_.rawTargetSpeed.setParam(rawTargetParam);

  AccRawTargetSpeedTraits::Input rawTargetInput;
  rawTargetInput.leadSpeed = input.leadSpeed;
  rawTargetInput.distanceError = distanceErrorOutput.distanceError;
  rawTargetInput.relativeSpeed = relativeSpeedOutput.relativeSpeed;
  AccRawTargetSpeedTraits::Output rawTargetOutput;
  sub_.rawTargetSpeed.run(rawTargetInput, rawTargetOutput);

  AccTargetSpeedLimitTraits::Input limitInput;
  limitInput.rawTargetSpeed = rawTargetOutput.rawTargetSpeed;
  limitInput.maxSpeed = input.maxSpeed;
  AccTargetSpeedLimitTraits::Output limitOutput;
  sub_.targetSpeedLimit.run(limitInput, limitOutput);

  AccEnableGateTraits::Input gateInput;
  gateInput.limitedTargetSpeed = limitOutput.limitedTargetSpeed;
  gateInput.enable = input.enable;
  AccEnableGateTraits::Output gateOutput;
  sub_.enableGate.run(gateInput, gateOutput);

  output.targetSpeed = gateOutput.targetSpeed;
  output.valid = gateOutput.valid;
  output.desiredDistance = desiredOutput.desiredDistance;
  output.distanceError = distanceErrorOutput.distanceError;
  output.relativeSpeed = relativeSpeedOutput.relativeSpeed;
  // === MBD_AUTO_GEN_END [AccSpeedControl] ===
}

/**
 * @brief Run ACC main flow with decision and control subcomponents.
 * @cn_name ACC主流程
 * @type component
 * @tag_level0 ACC
 * @tag_level1 ClosedLoop
 * @tag_level2 MainFlow
 * @version 2.1
 * @date 2026-07-08
 * @author ZYK
 */
void AccMainFlow::run(const Input& input, Output& output) {
  // === MBD_AUTO_GEN_BEGIN [AccMainFlow] ===
  AccVehicleInputChannelTraits::Input vehicleInput;
  vehicleInput.egoSpeed = input.egoSpeed;
  vehicleInput.commandType = input.commandType;
  AccVehicleInputChannelTraits::Output vehicleOutput;
  sub_.vehicleInputChannel.run(vehicleInput, vehicleOutput);

  AccPerceptionInputChannelTraits::Input perceptionInput;
  perceptionInput.leadSpeed = input.leadSpeed;
  perceptionInput.leadDistance = input.leadDistance;
  AccPerceptionInputChannelTraits::Output perceptionOutput;
  sub_.perceptionInputChannel.run(perceptionInput, perceptionOutput);

  AccDecisionTraits::Param decisionParam;
  decisionParam.initialTimeGap = param_.initialTimeGap;
  decisionParam.initialMaxSpeed = param_.initialMaxSpeed;
  decisionParam.vMin = param_.vMin;
  decisionParam.timeGapStep = param_.timeGapStep;
  decisionParam.minTimeGap = param_.minTimeGap;
  decisionParam.maxTimeGap = param_.maxTimeGap;
  decisionParam.speedStep = param_.speedStep;
  decisionParam.minSpeed = param_.minSpeed;
  decisionParam.maxSpeedCap = param_.maxSpeedCap;
  sub_.decision.setParam(decisionParam);

  AccDecisionTraits::Input decisionInput;
  decisionInput.egoSpeed = vehicleOutput.egoSpeed;
  decisionInput.commandType = vehicleOutput.commandType;
  AccDecisionTraits::Output decisionOutput;
  sub_.decision.run(decisionInput, decisionOutput);

  AccSpeedControlTraits::Param controlParam;
  controlParam.minDistance = param_.minDistance;
  controlParam.distanceGain = param_.distanceGain;
  controlParam.speedGain = param_.speedGain;
  sub_.speedControl.setParam(controlParam);

  AccSpeedControlTraits::Input controlInput;
  controlInput.egoSpeed = vehicleOutput.egoSpeed;
  controlInput.leadSpeed = perceptionOutput.leadSpeed;
  controlInput.leadDistance = perceptionOutput.leadDistance;
  controlInput.enable = decisionOutput.enable;
  controlInput.timeGap = decisionOutput.timeGap;
  controlInput.maxSpeed = decisionOutput.maxSpeed;
  AccSpeedControlTraits::Output controlOutput;
  sub_.speedControl.run(controlInput, controlOutput);

  output.targetSpeed = controlOutput.targetSpeed;
  output.enable = decisionOutput.enable != 0;
  output.valid = controlOutput.valid != 0;
  output.timeGap = decisionOutput.timeGap;
  output.maxSpeed = decisionOutput.maxSpeed;
  output.decision = decisionOutput.decision;
  output.systemState = decisionOutput.systemState;
  // === MBD_AUTO_GEN_END [AccMainFlow] ===
}

} // namespace control
