#include "apaParkingPathPostProcess.h"
#include "apaParkingPathPlanningInternal.h"
#include "apaParkingSearchFallback.h"
#include "apaPathAlgo.h"
#include "apaPlannerConfig.h"
#include <stdio.h>
#include <string.h>
#include "apaParkingObservation.h"

typedef struct {
  const ApaParkingPathPlanningInput_t *Input;
  ApaParkingPathPlanningParam_t Param;
  VehicleCollisionCheckInput_t Envelope;
  ApaParkingPathPlanningOutput_t *Output;
  Real Cost;
  ApaParkingQuality_t Quality;
  Integer Budget, Checked, Accepted, Attempts;
  const char *RejectStage;
  uint64_t CandidateId, SourceId;
  Boolean CollisionPoseValid;
  Real CollisionX, CollisionY, CollisionYaw;
} ApaParkingPostProcess_t;

static Boolean apaParkingJumpCollision(Integer index, Real x, Real y, Real yaw, void *data) {
  ApaParkingPostProcess_t *context = data;
  VehicleCollisionCheckInput_t envelope = context->Envelope;
  VehicleCollisionCheckOutput_t checked = {0};
  (void)index;
  envelope.VehPosX = x;
  envelope.VehPosY = y;
  envelope.VehPosTheta = yaw;
  apaPlanningSceneCollisionCheck(&envelope, &context->Param.SceneQuery, &checked);
  if (checked.IsCollision && !context->CollisionPoseValid) {
    context->CollisionPoseValid = true;
    context->CollisionX = x;
    context->CollisionY = y;
    context->CollisionYaw = yaw;
  }
  return checked.IsCollision;
}

static Boolean apaParkingJumpAccept(const ApaPath_t *path, void *data) {
  ApaParkingPostProcess_t *context = data;
  const ApaParkingPathPlanningInput_t *input = context->Input;
  context->RejectStage = "execution_budget";
  if (context->Param.SceneQuery.CheckExecution && context->Budget <= 0)
    return false;
  ApaParkingPathPlanningOutput_t candidate = {0};
  candidate.SelectedSlotKind = input->TargetSlotKind;
  candidate.PlanFlg = 1;
  candidate.PlannerCase = context->Output->PlannerCase;
  candidate.PtCount = path->PathN;
  candidate.PathLen = apaPathCalcAbsLength(path);

  candidate.SegNum = apaPathCalcSegNum(path);
  candidate.StpN = candidate.SegNum - 1;
  candidate.StepCount = 1;
  for (Integer i = 0; i < path->PathN; ++i) {
    candidate.PthPtX[i] = path->X[i];
    candidate.PthPtY[i] = path->Y[i];
    candidate.PthPtTht[i] = path->Tht[i];
    candidate.PthPtOdo[i] = path->OdoStep[i];
    if (i + 1 < path->PathN) {
      const Real yaw = fabsf(apaCoordWrapToPi(path->Tht[i + 1] - path->Tht[i]));
      context->RejectStage = "curvature_yaw";
      if (yaw > fabsf(path->OdoStep[i]) / input->Rmin + 1.e-5f)
        return false;
    }
    if (i > 0 && !candidate.St2Idx && path->OdoStep[i - 1] * path->OdoStep[i] < 0)
      candidate.St2Idx = i + 1;
  }
  if (!candidate.St2Idx)
    candidate.St2Idx = path->PathN;
  if (input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_VH)
    candidate.St2Idx = 0;
  if (input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_VT ||
      input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_DTH_ED_VT) {
    for (Integer i = 0; i + 1 < path->PathN; ++i)
      if (path->OdoStep[i] > 0) {
        apaCoordGround2Veh(input->VehicleX,
                           input->VehicleY,
                           input->VehicleYaw,
                           path->X[i],
                           path->Y[i],
                           path->Tht[i],
                           &candidate.Stp1PtRX,
                           &candidate.Stp1PtRY,
                           &candidate.Stp1PtRTheta);
        break;
      }
  }
  context->CandidateId = apaParkingPathId(&candidate);
  apaParkingLogRuns(&candidate, "jump_attempt");
  context->RejectStage = "structure";
  if (!apaParkingPathPlanningCandidateIsStructurallyValid(input, &candidate))
    return false;
  context->RejectStage = "collision_full_path";
  if (!apaParkingPathPlanningOutputPathIsSafe(input, &context->Param, &candidate, true))
    return false;
  const Real cost = apaParkingManeuverCost(&candidate, input, &context->Param);
  const ApaParkingQuality_t quality = apaParkingManeuverQuality(&candidate, cost);
  context->RejectStage = "quality";
  fprintf(stderr,
          "[APA jump cost] attempt=%ld id=%016llx candidate=%.9g incumbent=%.9g\n",
          (long)context->Attempts,
          (unsigned long long)context->CandidateId,
          (double)cost,
          (double)context->Cost);
  if (apaParkingQualityCompare(quality, context->Quality) >= 0)
    return false;
  if (context->Param.SceneQuery.CheckExecution) {
    --context->Budget;
    ++context->Checked;
    context->RejectStage = "execution";
    fprintf(stderr,
            "[APA jump execution] attempt=%ld id=%016llx event=begin\n",
            (long)context->Attempts,
            (unsigned long long)context->CandidateId);
    if (!context->Param.SceneQuery.CheckExecution(candidate.PtCount,
                                                  candidate.PthPtX,
                                                  candidate.PthPtY,
                                                  candidate.PthPtTht,
                                                  candidate.PthPtOdo,
                                                  context->Param.SceneQuery.ExecutionData)) {
      fprintf(stderr,
              "[APA jump execution] attempt=%ld id=%016llx event=end result=rejected\n",
              (long)context->Attempts,
              (unsigned long long)context->CandidateId);
      return false;
    }
  }
  fprintf(stderr,
          "[APA jump execution] attempt=%ld id=%016llx event=end result=%s\n",
          (long)context->Attempts,
          (unsigned long long)context->CandidateId,
          context->Param.SceneQuery.CheckExecution ? "passed" : "not_checked");
  *context->Output = candidate;
  context->Cost = cost;
  context->Quality = quality;
  ++context->Accepted;
  return true;
}

static void apaParkingJumpObserve(const ApaPath_t *source,
                                  Integer round,
                                  Integer start,
                                  Integer end,
                                  const char *stage,
                                  void *data) {
  ApaParkingPostProcess_t *context = data;
  if (!strcmp(stage, "begin")) {
    ++context->Attempts;
    context->RejectStage = "unknown";
    context->CandidateId = 0;
    context->SourceId = apaParkingPathId(context->Output);
    context->CollisionPoseValid = false;
    context->CollisionX = 0;
    context->CollisionY = 0;
    context->CollisionYaw = 0;
    return;
  }
  if (!strcmp(stage, "acceptance"))
    stage = context->RejectStage;
  fprintf(
      stderr,
      "[APA jump attempt] attempt=%ld round=%ld source=%016llx candidate=%016llx start=%ld end=%ld "
      "start_pose=%.6f,%.6f,%.6f end_pose=%.6f,%.6f,%.6f result=%s collision_pose_valid=%d "
      "collision_pose=%.6f,%.6f,%.6f\n",
      (long)context->Attempts,
      (long)round,
      (unsigned long long)context->SourceId,
      (unsigned long long)context->CandidateId,
      (long)start,
      (long)end,
      (double)source->X[start],
      (double)source->Y[start],
      (double)source->Tht[start],
      (double)source->X[end],
      (double)source->Y[end],
      (double)source->Tht[end],
      stage,
      context->CollisionPoseValid,
      (double)context->CollisionX,
      (double)context->CollisionY,
      (double)context->CollisionYaw);
}

Boolean apaParkingPathPostProcess(const ApaParkingPathPlanningInput_t *input,
                                  const ApaParkingPathPlanningParam_t *param,
                                  Integer executionBudget,
                                  ApaParkingPathPlanningOutput_t *output) {
  if (input->ParkingAction != APA_SCENARIO_PARKING_ACTION_PARKIN ||
      (param->SltState != APA_SLOT_STATUS_TARGET_SLOT &&
       param->SltState != APA_SLOT_STATUS_TARGET_SLOT_REPLAN) ||
      !output->PlanFlg || output->StpN < 1 ||
      !apaParkingPathPlanningCandidateIsStructurallyValid(input, output))
    return false;
  ApaPlannerConfig_t config;
  apaPlannerConfigInitDefault(&config);
  if (!config.Conf.EnableJumpPoints)
    return false;
  ApaPath_t path;
  apaPathInit(&path);
  path.ValidFlag = true;
  path.PathN = output->PtCount;
  for (Integer i = 0; i < path.PathN; ++i) {
    path.X[i] = output->PthPtX[i];
    path.Y[i] = output->PthPtY[i];
    path.Tht[i] = output->PthPtTht[i];
    path.OdoStep[i] = output->PthPtOdo[i];
  }
  ApaParkingPostProcess_t context = {.Input = input,
                                     .Param = *param,
                                     .Output = output,
                                     .Cost = apaParkingManeuverCost(output, input, param),
                                     .Budget = executionBudget};
  context.Quality = apaParkingManeuverQuality(output, context.Cost);

  context.Param.Diagnostics = 0;
  apaParkingPathPlanningBuildPathVccInput(input, param, &context.Envelope);
  const Integer changes = output->StpN;
  const Real length = output->PathLen;
  const Integer slotClass = input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_HT
                                ? APA_SLOT_CLASS_PARALLEL
                                : APA_SLOT_CLASS_VERTICAL;
  const Boolean changed = apaPathJumpPointsObserved(&path,
                                                    slotClass,
                                                    APA_EXP_DIREC_NONE,
                                                    apaParkingJumpCollision,
                                                    &context,
                                                    true,
                                                    (ApaPathNearSlotProbeFunc)0,
                                                    config.Conf.PathLenMin,
                                                    config.Conf.PathLenMax,
                                                    input->Rmin,
                                                    apaParkingJumpAccept,
                                                    &context,
                                                    apaParkingJumpObserve,
                                                    &context);
  fprintf(stderr,
          "[APA jump points] slot=%ld attempts=%ld accepted=%ld execution_checks=%ld "
          "changes=%ld->%ld length=%.3f->%.3f\n",
          (long)input->TargetSlotKind,
          (long)context.Attempts,
          (long)context.Accepted,
          (long)context.Checked,
          (long)changes,
          (long)output->StpN,
          (double)length,
          (double)output->PathLen);
  return changed;
}
