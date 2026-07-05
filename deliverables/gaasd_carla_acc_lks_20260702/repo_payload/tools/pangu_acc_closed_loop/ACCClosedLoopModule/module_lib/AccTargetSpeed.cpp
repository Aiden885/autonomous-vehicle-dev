#include "AccTargetSpeed.hpp"

namespace control {
namespace {

Real ClampValue(Real value, Real low, Real high) {
  Real result = value;
  if (result < low) {
    result = low;
  }
  if (result > high) {
    result = high;
  }
  return result;
}

int ComputeSystemState(Real egoSpeed, bool controlEnabled, bool hasHistory,
                       Real vMin) {
  int state = 2;
  if (egoSpeed < vMin) {
    state = 3;
  } else if (controlEnabled) {
    state = 0;
  } else if (hasHistory) {
    state = 1;
  }
  return state;
}

int ComputeDecision(int systemState, int commandType) {
  int decision = 8;
  if (systemState == 0) {
    if (commandType == 1) {
      decision = 1;
    } else if (commandType == 2) {
      decision = 2;
    } else if (commandType == 3) {
      decision = 3;
    } else if (commandType == 4) {
      decision = 4;
    } else if (commandType == 5) {
      decision = 7;
    } else if (commandType == 6 || commandType == 7) {
      decision = 8;
    }
  } else if (systemState == 1) {
    if (commandType == 1) {
      decision = 5;
    } else if (commandType == 2) {
      decision = 6;
    } else if (commandType == 6 || commandType == 7) {
      decision = 8;
    }
  } else if (systemState == 2) {
    if (commandType == 1) {
      decision = 5;
    } else if (commandType == 6 || commandType == 7) {
      decision = 8;
    }
  } else if (systemState == 3) {
    if (commandType == 6 || commandType == 7) {
      decision = 8;
    }
  }
  return decision;
}

bool ComputeNextEnable(int decision, int commandType, bool controlEnabled) {
  const bool turnOn = (decision == 5 || decision == 6);
  const bool cancel = (commandType == 6 || commandType == 7);
  return (controlEnabled || turnOn) && !cancel;
}

}  // namespace

/**
 * @brief 执行ACC目标速度和使能决策计算。
 * @cn_name ACC目标速度计算
 * @type atomic
 * @tag_level0 巡航控制
 * @tag_level1 ACC控制
 * @tag_level2 目标速度计算
 * @version 2.0
 * @date 2026-06-10
 * @author 组件改写引擎
 */
void AccTargetSpeed::run(const Input& input, Output& output) {
  if (!state_.initialized) {
    state_.timeGap = ClampValue(param_.initialTimeGap, param_.minTimeGap,
                                param_.maxTimeGap);
    state_.maxSpeed = ClampValue(param_.initialMaxSpeed, param_.minSpeed,
                                 param_.maxSpeedCap);
    state_.initialized = true;
  }

  const Real egoSpeed0 = input.egoSpeed;
  const Real leadSpeed0 = input.leadSpeed;
  const Real leadDistance0 = input.leadDistance > 0.0 ? input.leadDistance : 0.0;
  const int commandType0 = input.commandType;
  const int systemState0 =
      ComputeSystemState(egoSpeed0, state_.controlEnabled, state_.hasHistory,
                         param_.vMin);
  const int decision0 = ComputeDecision(systemState0, commandType0);
  const bool enableOutput0 = egoSpeed0 >= param_.vMin && state_.controlEnabled;
  const bool enableNext0 =
      ComputeNextEnable(decision0, commandType0, state_.controlEnabled);

  Real timeGapNext0 = state_.timeGap;
  if (decision0 == 3) {
    timeGapNext0 = timeGapNext0 - param_.timeGapStep;
  } else if (decision0 == 4) {
    timeGapNext0 = timeGapNext0 + param_.timeGapStep;
  }
  timeGapNext0 =
      ClampValue(timeGapNext0, param_.minTimeGap, param_.maxTimeGap);

  Real maxSpeedNext0 = state_.maxSpeed;
  if (decision0 == 1) {
    maxSpeedNext0 = maxSpeedNext0 - param_.speedStep;
  } else if (decision0 == 2) {
    maxSpeedNext0 = maxSpeedNext0 + param_.speedStep;
  }
  maxSpeedNext0 =
      ClampValue(maxSpeedNext0, param_.minSpeed, param_.maxSpeedCap);

  const Real desiredDistance0 =
      param_.minDistance > egoSpeed0 * timeGapNext0
          ? param_.minDistance
          : egoSpeed0 * timeGapNext0;
  const Real distanceError0 = leadDistance0 - desiredDistance0;
  const Real relativeSpeed0 = leadSpeed0 - egoSpeed0;
  const Real rawTargetSpeed0 =
      leadSpeed0 + param_.distanceGain * distanceError0 +
      param_.speedGain * relativeSpeed0;
  const Real targetSpeedEnabled0 =
      ClampValue(rawTargetSpeed0, 0.0, maxSpeedNext0);
  const Real targetSpeed0 = enableOutput0 ? targetSpeedEnabled0 : 0.0;
  const bool hasHistoryNext0 = state_.hasHistory || enableNext0;

  state_.controlEnabled = enableNext0;
  state_.hasHistory = hasHistoryNext0;
  state_.timeGap = timeGapNext0;
  state_.maxSpeed = maxSpeedNext0;
  if (decision0 >= 1 && decision0 <= 7) {
    state_.lastDecision = decision0;
  }

  output.targetSpeed = targetSpeed0;
  output.enable = enableOutput0;
  output.valid = enableOutput0;
  output.timeGap = timeGapNext0;
  output.maxSpeed = maxSpeedNext0;
  output.decision = decision0;
  output.systemState = systemState0;
}

} // namespace control
