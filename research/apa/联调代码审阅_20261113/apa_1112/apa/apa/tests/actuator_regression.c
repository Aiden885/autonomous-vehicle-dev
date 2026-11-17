#include "apaEnums.h"
#include "apaMocoActuator.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static ApaMocoActuatorInput_t input_for(int direction) {
  ApaMocoActuatorInput_t in = {0};
  in.Enable = true;
  in.CycleTime = 0.02f;
  in.SpeedLimit = 1.1f;
  in.TargetGear = in.FeedbackGear =
      direction < 0 ? APA_GEAR_POS_R : APA_GEAR_POS_D;
  in.TargetSpeed = direction * 0.15f;
  return in;
}

static void valid_pedals(const ApaMocoActuatorOutput_t *out) {
  assert(out->Throttle >= 0 && out->Throttle <= 1);
  assert(out->Brake >= 0 && out->Brake <= 1);
  assert(!(out->Throttle > 0 && out->HandBrake));
  assert(!(out->Throttle > 0 && out->Brake > 0) || out->LaunchHold);
}

static void stalled_launch_builds_traction(int direction) {
  ApaMocoActuatorInput_t in = input_for(direction);
  ApaMocoActuatorState_t state = {0};
  ApaMocoActuatorOutput_t out;
  in.TargetAcceleration = direction * 0.2f;
  float previous = 0;
  for (int i = 0; i < 50; ++i) {
    apaMocoActuate(&in, &state, &out);
    valid_pedals(&out);
    assert(out.Throttle <= previous + in.CycleTime + 1.e-6f);
    previous = out.Throttle;
  }
  assert(out.Throttle > 0.35f && state.Bias > 0);
}

static void grade_launch_moves_without_rollback(int direction) {
  ApaMocoActuatorInput_t in = input_for(direction);
  ApaMocoActuatorState_t state = {0};
  ApaMocoActuatorOutput_t out;
  in.GravityAcceleration = -direction * 0.78f;
  in.StopReq = true;
  apaMocoActuate(&in, &state, &out);
  in.StopReq = false;
  float distance = 0;
  for (int i = 0; i < 250; ++i) {
    in.TargetAcceleration =
        direction * fminf(0.35f, 2 * (0.15f - direction * in.VehicleSpeed));
    apaMocoActuate(&in, &state, &out);
    valid_pedals(&out);
    const float force = direction * out.Throttle * APA_MOCO_DRIVE_GAIN_MPS2 +
                        in.GravityAcceleration;
    const float resistance = 0.28f + out.Brake * APA_MOCO_BRAKE_GAIN_MPS2;
    const float before = in.VehicleSpeed;
    const float free_speed = before + force * in.CycleTime;
    const float next = copysignf(
        fmaxf(0, fabsf(free_speed) - resistance * in.CycleTime), free_speed);
    in.VehicleSpeed = next;
    in.VehicleMotionSpeed = fabsf(next);
    in.VehicleAcceleration = (next - before) / in.CycleTime;
    distance += direction * next * in.CycleTime;
    assert(direction * next >= -0.005f && fabsf(next) < 0.4f);
  }
  assert(distance > 0.05f);
}

static void microroll_is_arrested(int direction) {
  ApaMocoActuatorInput_t in = input_for(direction);
  ApaMocoActuatorState_t state = {0};
  ApaMocoActuatorOutput_t out;
  in.VehicleSpeed = -direction * 0.005f;
  in.VehicleMotionSpeed = 0.005f;
  in.TargetAcceleration = direction * 0.2f;
  apaMocoActuate(&in, &state, &out);
  valid_pedals(&out);
  assert(out.Brake > 0);
  in.StopReq = true;
  apaMocoActuate(&in, &state, &out);
  assert(out.Brake == 1 && out.Throttle == 0 && !out.LaunchHold);
}

static void speed_and_gear_limits_override_launch(int direction) {
  ApaMocoActuatorInput_t in = input_for(direction);
  ApaMocoActuatorState_t state = {0};
  ApaMocoActuatorOutput_t out;
  in.TargetAcceleration = direction * 0.35f;
  in.VehicleSpeed = direction * (in.SpeedLimit + 0.3f);
  in.VehicleMotionSpeed = fabsf(in.VehicleSpeed);
  apaMocoActuate(&in, &state, &out);
  assert(out.Overspeed && out.Brake == 1 && out.Throttle == 0);
  in.VehicleSpeed = in.VehicleMotionSpeed = 0;
  apaMocoActuate(&in, &state, &out);
  assert(out.Overspeed && out.Brake == 1 && out.Throttle == 0);
  state = (ApaMocoActuatorState_t){0};
  in.FeedbackGear = APA_GEAR_POS_N;
  apaMocoActuate(&in, &state, &out);
  assert(out.Gear == APA_GEAR_POS_N && out.Brake == 1 && out.Throttle == 0);
}

int main(void) {
  for (int direction = -1; direction <= 1; direction += 2) {
    stalled_launch_builds_traction(direction);
    grade_launch_moves_without_rollback(direction);
    microroll_is_arrested(direction);
    speed_and_gear_limits_override_launch(direction);
  }
  puts("APA actuator regression passed: grade launch, stationary load, "
       "rollback and overrides (D/R)");
  return 0;
}
