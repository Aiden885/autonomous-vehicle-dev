#include "apaParkingSearchFallback.h"
#include "apaGoalTolerance.h"
#include <string.h>
#include <limits.h>
#include "apaParkingSearch.h"
#include "apaParkingPathPlanningInternal.h"
#include "apaParkingPathSafetyGate.h"
#include "apaPlannerHybridParity.h"
#include "apaParkingPathPostProcess.h"
#include "apaParkingObservation.h"
#include "apaParallelParkinManeuver.h"

typedef struct {
  double Length, Steering, Cusp, SteeringChange, Clearance, GoalOffset;
} ApaCostParts;
static Real apaParkingManeuverCostObserved(const ApaParkingPathPlanningOutput_t *,
                                           const ApaParkingPathPlanningInput_t *,
                                           const ApaParkingPathPlanningParam_t *,
                                           ApaCostParts *);

#define APA_FINAL_CANDIDATES 3

#define APA_EXECUTION_CANDIDATES 16
#define APA_EXECUTION_QUALITY_CANDIDATES 8
typedef struct {
  ApaParkingPathPlanningOutput_t Path;
  Real Cost;
  ApaParkingQuality_t Quality;
  Boolean Executable;
} ApaParkingCandidate_t;

typedef struct {
  ApaParkingCandidate_t Pending[APA_FINAL_CANDIDATES];
  ApaParkingCandidate_t Evaluated[APA_EXECUTION_CANDIDATES];
  Integer Count, EvaluatedCount, Best;
  Integer CacheHits;
} ApaParkingCandidates_t;

static Integer apaParkingExecutionLimit(const ApaParkingCandidates_t *pool) {
  if (pool->Best < 0)
    return APA_EXECUTION_CANDIDATES;
  return pool->EvaluatedCount > APA_EXECUTION_QUALITY_CANDIDATES ? pool->EvaluatedCount
                                                                 : APA_EXECUTION_QUALITY_CANDIDATES;
}

static Boolean apaParkingSamePath(const ApaParkingPathPlanningOutput_t *a,
                                  const ApaParkingPathPlanningOutput_t *b) {
  const size_t bytes = a->PtCount * sizeof(Real);
  return a->PtCount == b->PtCount && memcmp(a->PthPtX, b->PthPtX, bytes) == 0 &&
         memcmp(a->PthPtY, b->PthPtY, bytes) == 0 && memcmp(a->PthPtTht, b->PthPtTht, bytes) == 0 &&
         memcmp(a->PthPtOdo, b->PthPtOdo, bytes) == 0;
}

static double apaParkingBestCost(const ApaParkingCandidates_t *pool) {
  return pool->Best >= 0 ? pool->Evaluated[pool->Best].Cost : INFINITY;
}

static ApaParkingQuality_t apaParkingBestQuality(const ApaParkingCandidates_t *pool) {
  return pool->Best >= 0 ? pool->Evaluated[pool->Best].Quality
                         : (ApaParkingQuality_t){INT_MAX, INT_MAX, INFINITY};
}

ApaParkingQuality_t apaParkingManeuverQuality(const ApaParkingPathPlanningOutput_t *path,
                                              double cost) {
  ApaParkingRunQuality_t runs = {0};
  for (Integer i = 0; i + 1 < path->PtCount; ++i)
    apaParkingRunQualityAdvance(&runs, path->PthPtOdo[i]);
  return apaParkingRunQualityFinish(&runs, cost);
}

static void apaParkingOffer(ApaParkingCandidates_t *pool,
                            const ApaParkingPathPlanningOutput_t *path,
                            const ApaParkingPathPlanningInput_t *input,
                            const ApaParkingPathPlanningParam_t *param) {
  const unsigned long long id = (unsigned long long)apaParkingPathId(path);
  if (pool->EvaluatedCount >= apaParkingExecutionLimit(pool)) {
    fprintf(stderr, "[APA candidate offer] id=%016llx result=execution_budget\n", id);
    return;
  }
  for (Integer i = 0; i < pool->EvaluatedCount; ++i) {
    if (apaParkingSamePath(&pool->Evaluated[i].Path, path)) {
      fprintf(stderr, "[APA candidate offer] id=%016llx result=duplicate_evaluated\n", id);
      ++pool->CacheHits;
      return;
    }
  }
  for (Integer i = 0; i < pool->Count; ++i)
    if (apaParkingSamePath(&pool->Pending[i].Path, path)) {
      fprintf(stderr, "[APA candidate offer] id=%016llx result=duplicate_pending\n", id);
      return;
    }
  ApaCostParts parts = {0};
  const Real cost = apaParkingManeuverCostObserved(path, input, param, &parts);
  const ApaParkingQuality_t quality = apaParkingManeuverQuality(path, cost);
  apaParkingLogRuns(path, "offer");
  fprintf(stderr,
          "[APA candidate cost] id=%016llx total=%.9g length=%.9g steering=%.9g cusp=%.9g "
          "steering_change=%.9g clearance=%.9g goal_offset=%.9g incumbent=%.9g\n",
          id,
          (double)cost,
          parts.Length,
          parts.Steering,
          parts.Cusp,
          parts.SteeringChange,
          parts.Clearance,
          parts.GoalOffset,
          apaParkingBestCost(pool));
  fprintf(stderr,
          "[APA candidate rank] id=%016llx changes=%d short=%d cost=%.9g score=%.9g\n",
          id,
          quality.Changes,
          quality.ShortRuns,
          quality.Cost,
          quality.Cost + APA_PARKING_SHORT_RUN_COST * quality.ShortRuns);
  if (apaParkingQualityCompare(quality, apaParkingBestQuality(pool)) >= 0) {
    fprintf(stderr, "[APA candidate offer] id=%016llx result=quality\n", id);
    return;
  }

  const Integer capacity = APA_FINAL_CANDIDATES - (pool->Best >= 0);
  Integer place = 0;
  while (place < pool->Count &&
         apaParkingQualityCompare(pool->Pending[place].Quality, quality) <= 0)
    ++place;
  const Integer last = pool->Count < capacity ? pool->Count : capacity - 1;
  if (place > last) {
    fprintf(stderr, "[APA candidate offer] id=%016llx result=pool_capacity\n", id);
    return;
  }
  if (pool->Count >= capacity)
    fprintf(stderr,
            "[APA candidate offer] id=%016llx result=evicted by=%016llx\n",
            (unsigned long long)apaParkingPathId(&pool->Pending[last].Path),
            id);
  fprintf(stderr, "[APA candidate offer] id=%016llx result=pending rank=%ld\n", id, (long)place);
  if (pool->Count < capacity)
    ++pool->Count;
  for (Integer i = last; i > place; --i)
    pool->Pending[i] = pool->Pending[i - 1];
  pool->Pending[place] = (ApaParkingCandidate_t){.Path = *path, .Cost = cost, .Quality = quality};
}

static ApaHybridAStarCandidateFeedback_t
apaParkingEvaluate(ApaParkingCandidates_t *pool,
                   const ApaParkingPathPlanningInput_t *input,
                   const ApaParkingPathPlanningParam_t *param,
                   Boolean flush) {

  const Boolean ready = flush || (pool->Best < 0 && pool->Count >= APA_FINAL_CANDIDATES);
  for (Integer i = 0;
       ready && i < pool->Count && pool->EvaluatedCount < apaParkingExecutionLimit(pool);
       ++i) {
    const ApaParkingCandidate_t *candidate = &pool->Pending[i];
    if (apaParkingQualityCompare(candidate->Quality, apaParkingBestQuality(pool)) >= 0)
      break;
    const Integer index = pool->EvaluatedCount++;
    ApaParkingCandidate_t *checked = &pool->Evaluated[index];
    *checked = *candidate;
    const ApaParkingPathPlanningOutput_t *path = &checked->Path;
    fprintf(stderr,
            "[APA candidate execution] id=%016llx event=begin check=%ld enabled=%d\n",
            (unsigned long long)apaParkingPathId(path),
            (long)pool->EvaluatedCount,
            param->SceneQuery.CheckExecution != 0);
    checked->Executable = !param->SceneQuery.CheckExecution ||
                          param->SceneQuery.CheckExecution(path->PtCount,
                                                           path->PthPtX,
                                                           path->PthPtY,
                                                           path->PthPtTht,
                                                           path->PthPtOdo,
                                                           param->SceneQuery.ExecutionData);
    fprintf(
        stderr,
        "[APA candidate quality] slot=%ld check=%ld cost=%.3f length=%.3f executable=%d case=%ld\n",
        (long)input->TargetSlotKind,
        (long)pool->EvaluatedCount,
        (double)checked->Cost,
        (double)path->PathLen,
        checked->Executable,
        (long)path->PlannerCase);
    fprintf(stderr,
            "[APA candidate execution] id=%016llx event=end result=%s\n",
            (unsigned long long)apaParkingPathId(path),
            !param->SceneQuery.CheckExecution ? "not_checked"
                                              : checked->Executable ? "passed" : "rejected");
    if (checked->Executable) {
      pool->Best = index;
      break;
    }
  }
  if (ready) {
    for (Integer i = 0; i < pool->Count; ++i) {
      Boolean evaluated = false;
      for (Integer j = 0; j < pool->EvaluatedCount; ++j)
        if (apaParkingSamePath(&pool->Pending[i].Path, &pool->Evaluated[j].Path))
          evaluated = true;
      if (!evaluated)
        fprintf(stderr,
                "[APA candidate execution] id=%016llx event=skip result=%s\n",
                (unsigned long long)apaParkingPathId(&pool->Pending[i].Path),
                pool->EvaluatedCount >= apaParkingExecutionLimit(pool) ? "execution_budget"
                                                                       : "incumbent_quality");
    }
    pool->Count = 0;
  }
  const ApaParkingQuality_t best = apaParkingBestQuality(pool);
  return (ApaHybridAStarCandidateFeedback_t){.BestCost = best.Cost,
                                             .BudgetExhausted = pool->EvaluatedCount >=
                                                                apaParkingExecutionLimit(pool),
                                             .EvaluatedCandidates = pool->EvaluatedCount,
                                             .BestDirectionChanges = best.Changes,
                                             .BestShortRuns = best.ShortRuns};
}

typedef struct {
  ApaParkingHybridTrial_t Request;
  const ApaHybridAStar_t *Planner;
  ApaParkingCandidates_t *Candidates;
  const VehicleCollisionCheckInput_t *Environment;
  Boolean ReverseSearch;
} ApaParkingSearchTrial_t;

static Boolean
apaParkingSearchSeed(Integer index, ApaHybridAStarMotionPath_t *outward, void *userData) {
  ApaParkingSearchTrial_t *trial = userData;
  const ApaParkingPathPlanningInput_t *input = trial->Request.Input;
  if (input->TargetSlotKind != APA_SCENARIO_SLOT_KIND_HT)
    return false;
  const VehicleCollisionCheckInput_t *e = trial->Environment;
  ApaSlot_t slot;
  const Real points[8] = {input->SltPt1X,
                          input->SltPt1Y,
                          input->SltPt2X,
                          input->SltPt2Y,
                          input->SltPt3X,
                          input->SltPt3Y,
                          input->SltPt4X,
                          input->SltPt4Y};
  apaPlannerHybridInitSlot(&slot,
                           input->TarX,
                           input->TarY,
                           input->TarTheta,
                           input->SltEntr,
                           points,
                           APA_PARKING_TYPE_RIGHT,
                           0,
                           APA_PARKING_DIREC_REAR);
  ApaPlannerHybridCollisionContext_t collision;
  apaPlannerHybridCollisionContextInit(&collision, trial->Planner->PlannerGlobal, &slot);
  collision.SceneTemplate = *e;

  for (Integer i = 0; i < e->BoundaryLineNum; ++i)
    if (!apaPlannerHybridCollisionAddBoundaryLine(&collision,
                                                  e->BoundaryLineX1[i],
                                                  e->BoundaryLineY1[i],
                                                  e->BoundaryLineX2[i],
                                                  e->BoundaryLineY2[i]))
      return false;
  for (Integer i = 0; i < e->VirtualBoundaryLineNum; ++i)
    if (!apaPlannerHybridCollisionAddVirtualBoundaryLine(&collision,
                                                         e->VirtualBoundaryLineX1[i],
                                                         e->VirtualBoundaryLineY1[i],
                                                         e->VirtualBoundaryLineX2[i],
                                                         e->VirtualBoundaryLineY2[i]))
      return false;
  for (Integer i = 0; i < e->BoundarySepPtsNum; ++i)
    if (!apaPlannerHybridCollisionAddBoundaryPoint(
            &collision, e->BoundarySepPtsX[i], e->BoundarySepPtsY[i]))
      return false;
  for (Integer i = 0; i < e->SlotTrackStopperNum; ++i)
    if (!apaPlannerHybridCollisionAddSlotTrackStopper(&collision,
                                                      e->SlotTrackStopperX1[i],
                                                      e->SlotTrackStopperY1[i],
                                                      e->SlotTrackStopperX2[i],
                                                      e->SlotTrackStopperY2[i]))
      return false;
  for (Integer i = 0; i < e->SeparateStopperNum; ++i)
    if (!apaPlannerHybridCollisionAddStopper(
            &collision, e->SeparateStopperX[i], e->SeparateStopperY[i]))
      return false;

  apaPlannerHybridBindScene(&collision, &trial->Request.Param->SceneQuery);
  const Real vehicle[3] = {input->VehicleX, input->VehicleY, input->VehicleYaw};
  ApaParallelParkinManeuverDiag_t diag = {0};
  const Boolean generated = apaParallelParkinManeuverRegionCandidate(
      &slot,
      vehicle,
      apaGoalPositionTolerance(input->GoalPositionTolerance),
      index,
      &collision,
      outward,
      &diag);
  fprintf(stderr,
          "[APA_SEARCH_TAIL] index=%ld generated=%d reason=%s loops=%ld\n",
          (long)index,
          generated,
          diag.Reason ? diag.Reason : "invalid",
          (long)diag.Loops);
  if (generated)
    apaPlannerHybridReverseMotion(outward);
  return generated;
}

static Boolean apaParkingSearchAccept(const ApaHybridAStarMotionPath_t *source,
                                      ApaPath_t *quality,
                                      void *userData) {
  ApaParkingSearchTrial_t *trial = userData;
  ApaHybridAStarMotionPath_t motion;
  ApaPlannerHybridSegments_t segments;
  ApaParkingPathSafetyGateInput_t gate = {0};
  ApaParkingPathSafetyGateOutput_t checked = {0};
  const ApaParkingPathPlanningParam_t *param = trial->Request.Param;
  const ApaParkingPathPlanningInput_t *input = trial->Request.Input;
  if (source->Path.PathN < 2 || source->Path.PathN > APA_PATH_MAX_POINTS)
    return false;
  const Real anchor[3] = {source->Path.X[0], source->Path.Y[0], source->Path.Tht[0]};
  const Real *root = trial->Planner->StartPose;
  /* Goal-region anchors belong only to park-in. A park-out root is the actual
   * vehicle pose and must never inherit destination tolerance. */
  const Real positionTolerance = trial->ReverseSearch
                                     ? apaGoalPositionTolerance(input->GoalPositionTolerance)
                                     : 1.e-4f;
  const Real yawTolerance = trial->ReverseSearch
                                ? apaGoalYawTolerance(input->GoalYawTolerance)
                                : 1.e-5f;
  if (!isfinite(anchor[0]) || !isfinite(anchor[1]) || !isfinite(anchor[2]) ||
      hypotf(anchor[0] - root[0], anchor[1] - root[1]) > positionTolerance ||
      fabsf(apaCoordWrapToPi(anchor[2] - root[2])) > yawTolerance)
    return false;
  if (!apaPlannerHybridCompleteMotion(
          source, trial->ReverseSearch ? anchor : root, trial->Planner->GoalPose, &motion)) {
    apaPlanningRecordFailure(param->Diagnostics,
                             APA_FAULT_PATH_NOT_MATCH_TARPOS,
                             "parking_search_endpoint_alignment",
                             3);
    return false;
  }

  if (trial->ReverseSearch)
    apaPlannerHybridReverseMotion(&motion);
  if (!apaPlannerHybridBuildSegmentsFromMotion(
          &motion, APA_PATH_SEG_MAX, &segments, param->Diagnostics))
    return false;
  segments.PlannerCase = source->Path.PathSource == APA_PATH_SOURCE_GEO
                             ? APA_PARKING_CONNECTION_PLANNER_CASE
                             : APA_PARKING_SEARCH_PLANNER_CASE;
  if (!apaParkingPathPlanningBuildHybridCandidate(
          &segments, &trial->Request, &gate.CandidateOutput)) {
    const Integer error = gate.CandidateOutput.ErrorType;
    apaPlanningRecordFailure(param->Diagnostics,
                             error == APA_FAULT_PATH_N_EXCEED_TO_ADD ||
                                     error == APA_FAULT_SEG_NUM_OVERMANY
                                 ? error
                                 : APA_FAULT_PATH_STATUS_ERR,
                             "parking_search_discretization",
                             4);
    return false;
  }
  gate.PlannerInput = *trial->Request.Input;
  gate.PlannerParam = *param;
  apaParkingPathSafetyGate(&gate, &checked);
  if (!checked.ValidatedOutput.PlanFlg) {
    fprintf(stderr,
            "[APA candidate gate] id=%016llx result=safety_rejected error=%ld\n",
            (unsigned long long)apaParkingPathId(&gate.CandidateOutput),
            (long)checked.ValidatedOutput.ErrorType);
    return false;
  }
  apaParkingOffer(trial->Candidates, &checked.ValidatedOutput, trial->Request.Input, param);
  *quality = motion.Path;
  quality->CollisionFlag = APA_COLLISION_RISK_NOT_EXIST;
  return true;
}

static ApaHybridAStarCandidateFeedback_t apaParkingSearchEvaluate(Boolean flush, void *userData) {
  ApaParkingSearchTrial_t *trial = userData;
  return apaParkingEvaluate(trial->Candidates, trial->Request.Input, trial->Request.Param, flush);
}

static void apaParkingSearch(const ApaParkingPathPlanningInput_t *input,
                             const ApaParkingPathPlanningParam_t *param,
                             ApaParkingCandidates_t *pool,
                             Boolean directFirst) {
  ApaPlannerGlobalState_t global = APA_PLANNER_GLOBAL_STATE_INIT;
  ApaHybridAStar_t planner = {0};
  VehicleCollisionCheckInput_t environment;
  ApaParkingSearchTrial_t trial = {0};
  apaParkingPathPlanningBuildPathVccInput(input, param, &environment);

  {
    environment.VehPosX = input->VehicleX;
    environment.VehPosY = input->VehicleY;
    environment.VehPosTheta = input->VehicleYaw;
    VehicleCollisionCheckOutput_t checked = {0};
    apaPlanningSceneCollisionCheck(&environment, &param->SceneQuery, &checked);
    if (checked.IsCollision) {
      apaPlanningRecordFailure(param->Diagnostics,
                               APA_FAULT_COLLISION_RISK_AT_NEW_OBS,
                               "parking_search_start_collision",
                               5);
      return;
    }
  }

  apaPlannerHybridInitPlannerGlobal(&global, param->VehW, input->Rmin, 1);
  global.Planner.Veh.LF = environment.VehLF;
  global.Planner.Veh.LB = environment.VehLB;
  global.Planner.Veh.LL = environment.VehLL;
  global.Planner.Veh.WB = environment.VehWB;
  global.Planner.Veh.Lf = environment.VehLF - environment.VehWB;
  apaHybridAStarInit(&planner);
  apaHybridAStarSetPlannerGlobal(&planner, &global);

  /* Both actions search out of the slot. Only park-in reverses the result
   * back into vehicle-to-destination order before common acceptance. */
  trial.ReverseSearch = input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKIN;
  planner.StartPose[0] = trial.ReverseSearch ? input->TarX : input->VehicleX;
  planner.StartPose[1] = trial.ReverseSearch ? input->TarY : input->VehicleY;
  planner.StartPose[2] = trial.ReverseSearch ? input->TarTheta : input->VehicleYaw;
  planner.StartPositionTolerance = trial.ReverseSearch
                                       ? apaGoalPositionTolerance(input->GoalPositionTolerance)
                                       : 0.0f;
  planner.GoalPose[0] = trial.ReverseSearch ? input->VehicleX : input->TarX;
  planner.GoalPose[1] = trial.ReverseSearch ? input->VehicleY : input->TarY;
  planner.GoalPose[2] = trial.ReverseSearch ? input->VehicleYaw : input->TarTheta;
  planner.Diagnostics = param->Diagnostics;
  trial.Request.Input = input;
  trial.Request.Param = param;
  trial.Planner = &planner;
  trial.Environment = &environment;
  if (input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKIN &&
      input->TargetSlotKind == APA_SCENARIO_SLOT_KIND_HT)
    planner.BuildParkingSeed = apaParkingSearchSeed;
  planner.CandidateCheck = apaParkingSearchAccept;
  trial.Candidates = pool;
  planner.EvaluateCandidates = apaParkingSearchEvaluate;
  planner.CandidateUserData = &trial;
  const Integer state = directFirst
                            ? apaParkingSearchLocal(&planner, &environment, &param->SceneQuery)
                            : apaParkingSearchRun(&planner, &environment, &param->SceneQuery);
  fprintf(stderr,
          "[APA_PARKIN_SEARCH] slot=%ld state=%ld expanded=%ld "
          "accepted=%ld rejected=%ld evaluated=%ld executable=%d cache_hits=%ld "
          "action=%ld reverse_search=%d\n",
          (long)input->TargetSlotKind,
          (long)state,
          (long)planner.ExpandIterations,
          (long)planner.AcceptedCandidates,
          (long)planner.RejectedGoalCandidates,
          (long)pool->EvaluatedCount,
          pool->Best >= 0,
          (long)pool->CacheHits,
          (long)input->ParkingAction,
          trial.ReverseSearch);
  if (state != APA_HYBRID_STATE_GOAL_REACHED && pool->Best < 0)
    apaPlanningRecordFailure(
        param->Diagnostics, apaPlanningSearchError(planner.Fault), "parking_search", 1);
}

static Boolean apaParkingChoose(const ApaParkingCandidates_t *pool,
                                const ApaParkingPathPlanningInput_t *input,
                                const ApaParkingPathPlanningParam_t *param,
                                ApaParkingPathPlanningOutput_t *output) {
  if (pool->Best >= 0) {
    *output = pool->Evaluated[pool->Best].Path;
    apaParkingLogRuns(output, "selected_before_jump");

    apaParkingPathPostProcess(
        input, param, apaParkingExecutionLimit(pool) - pool->EvaluatedCount, output);
    apaParkingPathPlanningFinalizeL1Output(output, APA_PATH_SEG_MAX);
    apaParkingLogRuns(output, "selected_final");
    return output->PlanFlg != 0;
  }
  apaParkingPathPlanningZeroOutput(output);
  output->SelectedSlotKind = input->TargetSlotKind;
  if (pool->EvaluatedCount)
    apaPlanningRecordFailure(
        param->Diagnostics, APA_FAULT_PATH_STATUS_ERR, "candidate_execution", 6);
  output->ErrorType =
      param->Diagnostics && param->Diagnostics->Stage
          ? param->Diagnostics->ErrorType
          : pool->EvaluatedCount ? APA_FAULT_PATH_STATUS_ERR : APA_FAULT_UNKNOWN_ERR;
  return false;
}

static Boolean apaParkingIsNearGoal(const ApaParkingPathPlanningInput_t *input) {

  const Real yaw = fabsf(apaCoordWrapToPi(input->TarTheta - input->VehicleYaw));
  return hypotf(input->TarX - input->VehicleX, input->TarY - input->VehicleY) + input->Rmin * yaw <=
         APA_VEHICLE_WHEELBASE_M;
}

Boolean apaParkingSelectCandidate(const ApaParkingPathPlanningInput_t *input,
                                  const ApaParkingPathPlanningParam_t *param,
                                  ApaParkingPathPlanningOutput_t *output) {
  ApaParkingCandidates_t pool = {.Best = -1};
  if (output->PlanFlg) {

    apaParkingOffer(&pool, output, input, param);
    apaParkingEvaluate(&pool, input, param, true);
  }
  if (param->NeedHybridAlgFusion)
    apaParkingSearch(input, param, &pool, apaParkingIsNearGoal(input));
  return apaParkingChoose(&pool, input, param, output);
}

static Real apaParkingManeuverCostObserved(const ApaParkingPathPlanningOutput_t *path,
                                           const ApaParkingPathPlanningInput_t *input,
                                           const ApaParkingPathPlanningParam_t *param,
                                           ApaCostParts *parts) {
  const Integer end = path->PtCount - 1;
  const Real goalOffset = apaParkingGoalOffsetCost(
      hypotf(path->PthPtX[end] - input->TarX, path->PthPtY[end] - input->TarY),
      apaCoordWrapToPi(path->PthPtTht[end] - input->TarTheta),
      input->Rmin);
  Real cost = goalOffset;
  if (parts)
    parts->GoalOffset = goalOffset;
  Integer previous = 0;
  Real previousCurvature = 0;
  VehicleCollisionCheckInput_t envelope;
  apaParkingPathPlanningBuildPathVccInput(input, param, &envelope);
  for (Integer i = 0; i + 1 < path->PtCount; ++i) {
    const Real distance = path->PthPtOdo[i];
    if (fabsf(distance) < 1.e-6f)
      continue;
    const Integer direction = distance > 0.0f ? 1 : -1;
    const Real yaw = path->PthPtTht[i];
    const Real dyaw = apaCoordWrapToPi(path->PthPtTht[i + 1] - yaw);
    const Real curvature = dyaw / distance;
    cost +=
        apaParkingMotionCost(distance, curvature, previousCurvature, previous, 1.0 / input->Rmin);
    if (parts) {
      const double maximumCurvature = 1.0 / input->Rmin;
      parts->Length += fabs(distance);
      parts->Steering += 0.05 * fabs((double)distance * curvature) / maximumCurvature;
      parts->Cusp += previous && previous * distance < 0 ? 1.5 : 0.0;
      parts->SteeringChange += apaParkingSteeringChangeCost(
          distance, curvature, previousCurvature, previous, maximumCurvature);
    }
    previous = direction;
    previousCurvature = curvature;
    const Real chord =
        hypotf(path->PthPtX[i + 1] - path->PthPtX[i], path->PthPtY[i + 1] - path->PthPtY[i]);
    const Integer samples = (Integer)ceilf(fabsf(distance) / 0.16f);
    for (Integer sample = 0; sample < samples; ++sample) {
      const Real t = ((Real)sample + 0.5f) / (Real)samples;
      const Real span = fabsf(dyaw) > 1.e-5f
                            ? direction * chord * sinf(t * dyaw / 2.0f) / sinf(dyaw / 2.0f)
                            : distance * t;
      envelope.VehPosX = path->PthPtX[i] + span * cosf(yaw + t * dyaw / 2.0f);
      envelope.VehPosY = path->PthPtY[i] + span * sinf(yaw + t * dyaw / 2.0f);
      envelope.VehPosTheta = yaw + t * dyaw;
      const Real penalty = apaPlanningSceneClearancePenalty(&envelope, &param->SceneQuery);
      cost += fabsf(distance) / (Real)samples * penalty;
      if (parts)
        parts->Clearance += fabsf(distance) / (Real)samples * penalty;
    }
  }
  return cost;
}

Real apaParkingManeuverCost(const ApaParkingPathPlanningOutput_t *path,
                            const ApaParkingPathPlanningInput_t *input,
                            const ApaParkingPathPlanningParam_t *param) {
  return apaParkingManeuverCostObserved(path, input, param, 0);
}

ApaParkingManeuverAssessment_t apaParkingAssessManeuver(const ApaParkingPathPlanningOutput_t *path,
                                                        const ApaParkingPathPlanningInput_t *input,
                                                        const ApaParkingPathPlanningParam_t *param,
                                                        Real reserve) {
  ApaParkingManeuverAssessment_t result = {{0, 0, INFINITY}, false};
  if (path->PtCount < 2 || path->PtCount > PATH_PT_MAX || !isfinite(input->Rmin) ||
      input->Rmin <= 0.0f || !isfinite(reserve) || reserve < 0.0f)
    return result;
  result.Quality = apaParkingManeuverQuality(path, apaParkingManeuverCost(path, input, param));
  VehicleCollisionCheckInput_t envelope;
  apaParkingPathPlanningBuildPathVccInput(input, param, &envelope);
  Real margins[3];
  vehicleCollisionCheckResolveBodyMargins(&envelope, margins);
  const Real corner =
      hypotf(param->VehLlF + margins[0] + reserve, 0.5f * param->VehW + margins[2] + reserve);
  for (Integer i = 0; i + 1 < path->PtCount; ++i) {
    const Real distance = path->PthPtOdo[i];
    const Real yaw = path->PthPtTht[i];
    const Real dyaw = apaCoordWrapToPi(path->PthPtTht[i + 1] - yaw);
    const Real chord =
        hypotf(path->PthPtX[i + 1] - path->PthPtX[i], path->PthPtY[i + 1] - path->PthPtY[i]);
    const Integer samples =
        (Integer)fmaxf(1.0f, ceilf((fabsf(distance) + corner * fabsf(dyaw)) / 0.02f));
    const Real direction = distance < 0.0f ? -1.0f : 1.0f;
    for (Integer sample = 0; sample <= samples; ++sample) {
      const Real t = (Real)sample / (Real)samples;
      const Real span = fabsf(dyaw) > 1.e-5f
                            ? direction * chord * sinf(t * dyaw / 2.0f) / sinf(dyaw / 2.0f)
                            : distance * t;
      envelope.VehPosX = path->PthPtX[i] + span * cosf(yaw + t * dyaw / 2.0f);
      envelope.VehPosY = path->PthPtY[i] + span * sinf(yaw + t * dyaw / 2.0f);
      envelope.VehPosTheta = yaw + t * dyaw;
      if (!apaPlanningSceneClearanceBandIsClear(&envelope, &param->SceneQuery, margins, reserve))
        return result;
    }
  }
  result.HasClearanceReserve = true;
  return result;
}
