#include "apaHybridAStar.h"
#include "apaParkingSearchFallback.h"
#include <assert.h>
#include <string.h>

/* Deterministic search substitute: exercise the production adapter, native
 * safety gate and execution callback without depending on graph tie order. */
static const ApaParkingPathPlanningInput_t *request;
static Boolean perturbRoot, expectLocal;
static Integer executionChecks;

static Boolean check_execution(Integer count, const Real *x, const Real *y,
                                const Real *yaw, const Real *odo, void *data) {
  const ApaParkingPathPlanningInput_t *input = data;
  assert(count > 1);
  assert(hypotf(x[0] - input->VehicleX, y[0] - input->VehicleY) < 1.e-4f);
  assert(fabsf(atan2f(sinf(yaw[0] - input->VehicleYaw),
                     cosf(yaw[0] - input->VehicleYaw))) < 1.e-4f);
  assert(hypotf(x[count - 1] - input->TarX, y[count - 1] - input->TarY) <=
         input->GoalPositionTolerance);
  assert(fabsf(atan2f(sinf(yaw[count - 1] - input->TarTheta),
                     cosf(yaw[count - 1] - input->TarTheta))) < 1.e-4f);
  const Real direction = (input->TarX - input->VehicleX) * cosf(input->VehicleYaw);
  for (Integer i = 0; i + 1 < count; ++i)
    assert(odo[i] * direction > 0);
  ++executionChecks;
  return true;
}

static int supply_straight(ApaHybridAStar_t *planner,
                           const VehicleCollisionCheckInput_t *environment,
                           Boolean local) {
  const Boolean parkout = request->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKOUT;
  assert(local == expectLocal);
  assert(environment->ParkingOutFlag == parkout);
  assert(planner->StartPose[0] == (parkout ? request->VehicleX : request->TarX));
  assert(planner->GoalPose[0] == (parkout ? request->TarX : request->VehicleX));
  assert(parkout ? planner->StartPositionTolerance == 0
                 : planner->StartPositionTolerance == request->GoalPositionTolerance);
  assert(!parkout || !planner->BuildParkingSeed);
  ApaHybridAStarMotionPath_t motion = {0};
  motion.Path.PathN = 2;
  motion.Path.ValidFlag = true;
  motion.Path.PathSource = APA_PATH_SOURCE_SEARCH;
  motion.Path.X[0] = planner->StartPose[0] + (perturbRoot ? 0.05f : 0);
  motion.Path.Y[0] = planner->StartPose[1];
  motion.Path.Tht[0] = planner->StartPose[2];
  motion.Path.X[1] = planner->GoalPose[0];
  motion.Path.Y[1] = planner->GoalPose[1];
  motion.Path.Tht[1] = planner->GoalPose[2];
  motion.Path.OdoStep[0] = (motion.Path.X[1] - motion.Path.X[0]) * cosf(motion.Path.Tht[0]);
  ApaPath_t quality = {0};
  const Boolean accepted = planner->CandidateCheck(&motion, &quality, planner->CandidateUserData);
  if (parkout && perturbRoot) {
    assert(!accepted);
    planner->Fault = APA_HYBRID_FAULT_HYBRID_OPEN_EMPTY;
    return APA_HYBRID_STATE_FAILED;
  }
  assert(accepted);
  assert(fabsf(quality.X[0] - request->VehicleX) < 1.e-4f);
  assert(fabsf(quality.OdoStep[0] -
               (request->TarX - request->VehicleX) * cosf(request->VehicleYaw)) <
         request->GoalPositionTolerance);
  planner->EvaluateCandidates(true, planner->CandidateUserData);
  return APA_HYBRID_STATE_GOAL_REACHED;
}

int apaParkingSearchRun(ApaHybridAStar_t *planner,
                        const VehicleCollisionCheckInput_t *environment,
                        const ApaPlanningSceneQuery_t *scene) {
  (void)scene;
  return supply_straight(planner, environment, false);
}

int apaParkingSearchLocal(ApaHybridAStar_t *planner,
                          const VehicleCollisionCheckInput_t *environment,
                          const ApaPlanningSceneQuery_t *scene) {
  (void)scene;
  return supply_straight(planner, environment, true);
}

static void run_case(Integer action, Integer kind, Boolean near, Boolean offset) {
  ApaParkingPathPlanningInput_t input = {0};
  input.ParkingAction = action;
  input.TargetSlotKind = kind;
  input.Rmin = 3;
  input.GoalPositionTolerance = 0.2f;
  input.GoalYawTolerance = 0.03f;
  input.SltEntr = 1;
  input.SltPt1X = input.SltPt2X = 1;
  input.SltPt3X = input.SltPt4X = 9;
  input.SltPt1Y = input.SltPt4Y = -2;
  input.SltPt2Y = input.SltPt3Y = 2;
  const Real inside = near ? 2 : 5;
  input.VehicleX = action == APA_SCENARIO_PARKING_ACTION_PARKOUT ? inside : 0;
  input.TarX = action == APA_SCENARIO_PARKING_ACTION_PARKOUT ? 0 : inside;
  input.VehicleYaw = input.TarTheta = kind == APA_SCENARIO_SLOT_KIND_VH ? 0 : 3.14159265f;
  ApaParkingPathPlanningParam_t param = {0};
  param.VehLlF = 3.6f;
  param.VehLr = 0.8f;
  param.VehW = 1.8f;
  param.NeedHybridAlgFusion = 1;
  param.SceneQuery.CheckExecution = check_execution;
  param.SceneQuery.ExecutionData = &input;
  ApaParkingPathPlanningOutput_t output = {0};
  request = &input;
  perturbRoot = offset;
  expectLocal = near;
  executionChecks = 0;
  const Boolean success = apaParkingSelectCandidate(&input, &param, &output);
  const Boolean expected = !(action == APA_SCENARIO_PARKING_ACTION_PARKOUT && offset);
  assert(success == expected);
  assert(executionChecks == (expected ? 1 : 0));
  if (success) {
    assert(output.SelectedSlotKind == (action == APA_SCENARIO_PARKING_ACTION_PARKOUT
                                           ? APA_SCENARIO_SLOT_KIND_PARKOUT : kind));
    assert(action == APA_SCENARIO_PARKING_ACTION_PARKOUT
               ? output.StepCount == 0 && output.St2Idx == 0
               : output.StepCount > 0);
  }
}

int main(void) {
  for (Integer action = 1; action <= 2; ++action)
    for (Integer kind = 1; kind <= 4; ++kind)
      for (int near = 0; near < 2; ++near)
        for (int offset = 0; offset < 2; ++offset)
          run_case(action, kind, near, offset);
  return 0;
}
