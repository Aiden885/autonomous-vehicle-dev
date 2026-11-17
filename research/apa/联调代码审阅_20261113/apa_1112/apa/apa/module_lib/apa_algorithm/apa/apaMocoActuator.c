#include "apaMocoActuator.h"
#include "apaEnums.h"

#include <math.h>

static Real apaMocoActuatorClamp(Real value, Real minimum, Real maximum) {
  return fminf(maximum, fmaxf(minimum, value));
}

void apaMocoActuate(const ApaMocoActuatorInput_t *input,
                    ApaMocoActuatorState_t *state,
                    ApaMocoActuatorOutput_t *output) {
  const Real driveGain = APA_MOCO_DRIVE_GAIN_MPS2;
  const Real creepThrottle = APA_MOCO_CREEP_THROTTLE;
  const Real accelerationFilterTime = 0.12f;
  const Real accelerationPreviewTime = 0.12f;
  const Real throttleFallRate = 0.8f;
  const Real standstillSpeed = 0.02f;
  const Real holdDeceleration = 1.2f;
  const Boolean inputValid =
      isfinite(input->VehicleSpeed) && isfinite(input->VehicleAcceleration) &&
      isfinite(input->GravityAcceleration) && fabsf(input->GravityAcceleration) <= 9.80665f &&
      isfinite(input->VehicleMotionSpeed) && input->VehicleMotionSpeed >= 0 &&
      isfinite(input->TargetSpeed) && isfinite(input->TargetAcceleration) &&
      isfinite(input->SpeedLimit) && input->SpeedLimit > standstillSpeed &&
      isfinite(input->CycleTime) && input->CycleTime > 0.0f && input->CycleTime <= 0.1f &&
      input->FeedbackGear >= APA_GEAR_POS_N && input->FeedbackGear <= APA_GEAR_POS_P &&
      input->TargetGear >= APA_GEAR_POS_N && input->TargetGear <= APA_GEAR_POS_P;
  const Boolean stopped =
      inputValid && fmaxf(fabsf(input->VehicleSpeed), input->VehicleMotionSpeed) <= standstillSpeed;
  *output = (ApaMocoActuatorOutput_t){0};
  output->Gear = input->FeedbackGear >= APA_GEAR_POS_N && input->FeedbackGear <= APA_GEAR_POS_P
                     ? input->FeedbackGear
                     : APA_GEAR_POS_N;
  if (stopped && input->GearChangeAllowed) {
    output->Gear = input->TargetGear;
  }
  if (inputValid && input->Enable &&
      fmaxf(fabsf(input->VehicleSpeed), input->VehicleMotionSpeed) >
          input->SpeedLimit + APA_MOCO_OVERSPEED_TOLERANCE_MPS) {
    state->Overspeed = true;
  }
  output->Overspeed = state->Overspeed;
  const Integer direction = output->Gear == APA_GEAR_POS_R ? -1 : 1;
  if (state->Direction != direction) {
    state->Bias = 0;
    state->Throttle = 0;
    state->Brake = 0;
    state->Direction = direction;
    state->LaunchHold = stopped || direction * input->VehicleSpeed < 0;
    state->FilteredAcceleration = 0;
  }
  output->HandBrake = stopped && output->Gear == APA_GEAR_POS_P;
  if (!inputValid || state->Overspeed || !input->Enable || input->StopReq ||
      (output->Gear != APA_GEAR_POS_D && output->Gear != APA_GEAR_POS_R) ||
      output->Gear != input->TargetGear || direction * input->TargetSpeed < 0 ||
      (fabsf(input->TargetSpeed) <= standstillSpeed && stopped)) {
    state->Bias = 0;
    state->Throttle = 0;
    state->Brake = 1;
    state->FilteredAcceleration = 0;
    output->Brake = 1;
    state->LaunchHold = true;
    return;
  }
  const Real travelSpeed = direction * input->VehicleSpeed;
  const Real actualAcceleration = direction * input->VehicleAcceleration;
  const Real targetSpeed = fabsf(input->TargetSpeed);
  Real requestedAcceleration = direction * input->TargetAcceleration;

  if (stopped) {
    state->FilteredAcceleration = 0;
  }
  state->FilteredAcceleration += input->CycleTime / (accelerationFilterTime + input->CycleTime) *
                                 (actualAcceleration - state->FilteredAcceleration);
  const Boolean overspeedRelease = travelSpeed > targetSpeed + 0.03f;
  const Boolean overspeedBraking = travelSpeed > targetSpeed + 0.10f;
  const Boolean launching = travelSpeed < APA_MOCO_LAUNCH_SPEED_MPS;
  const Boolean predictiveUnload =
      travelSpeed >= 0 &&
      travelSpeed + fmaxf(0, state->FilteredAcceleration) * accelerationPreviewTime >
          targetSpeed + 0.015f;
  if (predictiveUnload) {
    requestedAcceleration = fminf(requestedAcceleration, 0);
  }
  if (overspeedBraking) {
    requestedAcceleration =
        fminf(requestedAcceleration, -fminf(holdDeceleration, 3.0f * (travelSpeed - targetSpeed)));
  }
  const Boolean braking = overspeedBraking || (targetSpeed <= standstillSpeed && !stopped);
  const Boolean rollback = travelSpeed < -0.001f;
  if (rollback)
    state->LaunchHold = true;
  Boolean driveReady = false;
  Real throttle = 0;
  Real brake = 0;
  if (braking) {
    state->LaunchHold = false;
    state->Bias = 0;
    brake = apaMocoActuatorClamp(
        fmaxf(-requestedAcceleration, targetSpeed <= standstillSpeed ? holdDeceleration : 0) /
            APA_MOCO_BRAKE_GAIN_MPS2,
        0,
        0.8f);
  } else if (overspeedRelease) {
    state->Bias =
        state->Bias > 0.0f
            ? fmaxf(0.0f, state->Bias - 3.0f * (travelSpeed - targetSpeed) * input->CycleTime)
            : 0.0f;
  } else {
    Real candidateBias = state->Bias;
    if (launching) {
      requestedAcceleration = fminf(requestedAcceleration, APA_MOCO_LAUNCH_ACCEL_MPS2);
    }
    const Real accelerationError = requestedAcceleration - state->FilteredAcceleration;

    if (!predictiveUnload) {
      candidateBias = apaMocoActuatorClamp(
          candidateBias + 1.5f * accelerationError * input->CycleTime, -1.0f, 1.0f);
    }

    const Real driveDemand = (requestedAcceleration + 0.20f * accelerationError + candidateBias -
                              direction * input->GravityAcceleration) /
                                 driveGain +
                             creepThrottle;
    const Real desiredThrottle = apaMocoActuatorClamp(driveDemand, 0.0f, 1.0f);
    if (driveDemand < 0) {
      state->Bias = candidateBias;
      brake = apaMocoActuatorClamp(-driveDemand * driveGain / APA_MOCO_BRAKE_GAIN_MPS2, 0, 0.8f);
      driveReady = true;
    } else {
      throttle =
          apaMocoActuatorClamp(desiredThrottle,
                               fmaxf(0, state->Throttle - throttleFallRate * input->CycleTime),
                               state->Throttle + input->CycleTime);
      throttle = fminf(throttle, 1.0f);
      driveReady = fabsf(throttle - desiredThrottle) <= 0.005f;

      if ((driveDemand - desiredThrottle) * (candidateBias - state->Bias) <= 0) {
        state->Bias = candidateBias;
      }
    }
  }

  if (state->LaunchHold && !braking) {
    if (!driveReady || rollback) {
      brake = fmaxf(brake, fmaxf(state->Brake, holdDeceleration / APA_MOCO_BRAKE_GAIN_MPS2));
    } else {
      const Real serviceBrake = brake;
      brake = fmaxf(serviceBrake, state->Brake - APA_MOCO_BRAKE_RISE_PER_SEC * input->CycleTime);
      if (brake <= serviceBrake)
        state->LaunchHold = false;
    }
  }
  brake = fminf(brake, state->Brake + APA_MOCO_BRAKE_RISE_PER_SEC * input->CycleTime);
  output->LaunchHold = state->LaunchHold && !braking && throttle > 0 && brake > 0;
  if (brake > 0 && !output->LaunchHold) {
    throttle = 0;
    state->FilteredAcceleration = 0;
  }
  state->Throttle = throttle;
  output->Throttle = throttle;
  state->Brake = brake;
  output->Brake = brake;
}
