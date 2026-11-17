/**
 * @file apaHybridAStar.h
 * @brief Hybrid A* search, candidate validation and bounded quality selection.
 *
 * Source:
 *   - Local_Planner/path_planning_alg: HybridAStar / CalcNextNode
 *   - ClassHybridAStar.m
 *   - HashNodesList.m
 *   - PriorityQueueNodesList.m
 *   - getHeuristicCost.m
 *   - getTotalCost.m
 *   - get_planner_global.m
 *   - RSPath.m
 *   - VehicleCollisionCheck.m
 *   - ClassSlot.m
 *
 * Date: 2026-04-14
 */
#ifndef APA_HYBRID_ASTAR_H
#define APA_HYBRID_ASTAR_H

#include "apaCoordUtils.h"
#include "apaMathUtils.h"
#include "apaPathTypes.h"
#include "apaPlannerGlobal.h"
#include "apaHybridAStarCollision.h"
#include "apaSearchHeuristic.h"
#include "apaSearchTypes.h"
#include "apaHybridPlanningTypes.h"
#include "apaSlotTypes.h"
#include <stdio.h>

#define APA_HYBRID_ASTAR_ZERO_STEER_EPS 1e-4f
/* Local_Planner/HybridAStar: cal_counter < uint16(1500). Independent of
 * the allocated node budget, which counts Open + Close rather than pops. */
#define APA_HYBRID_ASTAR_MAX_ITERATIONS 1500

typedef enum {
  APA_HYBRID_STATE_IDLE = 0,
  APA_HYBRID_STATE_READY = 1,
  APA_HYBRID_STATE_RUNNING = 2,
  APA_HYBRID_STATE_GOAL_REACHED = 3,
  APA_HYBRID_STATE_FAILED = 4
} ApaHybridAStarState_e;

typedef struct {
  Real OriginX;
  Real OriginY;
  Real XResolution;
  Real YResolution;
  Real YawOrigin;
  Real YawResolution;
  Integer XCount;
  Integer YCount;
  Integer YawCount;
} ApaHybridAStarGridSpec_t;

typedef Boolean (*ApaHybridAStarCollisionCheck_f)(const ApaHybridNode_t *fromNode,
                                                  const ApaHybridNode_t *toNode,
                                                  void *userData);

/* Interval controls retain steering-signed curvature, independent of travel direction. */
typedef struct {
  ApaPath_t Path;
  Real Curvature[APA_PATH_MAX_POINTS];
} ApaHybridAStarMotionPath_t;

typedef Boolean (*ApaHybridAStarCandidateCheck_f)(const ApaHybridAStarMotionPath_t *motion,
                                                  ApaPath_t *qualityPath,
                                                  void *userData);

/* CandidateCheck/Selected callbacks are synchronous. */
typedef struct {
  double BestCost; /* INFINITY until a complete candidate passes execution. */
  Boolean BudgetExhausted;
  Integer EvaluatedCandidates;
  Integer BestDirectionChanges, BestShortRuns;
} ApaHybridAStarCandidateFeedback_t;

typedef struct ApaHybridAStar {
  ApaPlannerGlobalState_t *PlannerGlobal;
  ApaHybridAStarCollisionCheck_f CollisionCheck;
  void *CollisionUserData;
  ApaHybridAStarCandidateCheck_f CandidateCheck;
  void (*CandidateSelected)(void *userData);
  ApaHybridAStarCandidateFeedback_t (*EvaluateCandidates)(Boolean flush, void *userData);
  Boolean (*BuildParkingSeed)(Integer index, ApaHybridAStarMotionPath_t *outward, void *userData);
  void *CandidateUserData;
  ApaPlanningDiagnostics_t *Diagnostics;
  ApaPathQualityCompareContext_t QualityContext;
  ApaPath_t BestQualityPath;
  Integer ExpectedSegmentCount;
  Integer AcceptedCandidates;
  Integer TruncatedPrimitives;
  Integer MotionCandidates;
  Integer OpenDominated;
  Integer ClosedDominated;
  Integer ReopenedStates;
  Boolean HasAcceptedCandidate;
  Integer RejectedGoalCandidates;
  ApaSlot_t ActiveSlot;
  ApaHybridAStarCollisionEnv_t FirstSegmentCollisionEnv;
  Boolean FirstSegmentBoundsValid;
  ApaNodePool_t NodePool;
  ApaHashMap_t OpenNodes;
  ApaHashMap_t CloseNodes;
  ApaPriorityQueue_t OpenQueue;
  ApaPlannerHeuristicContext_t HeuristicContext;
  ApaSlotHeuristicView_t SlotView;
  ApaHybridAStarGridSpec_t GridSpec;
  ApaHybridNode_t StartNode;
  ApaHybridNode_t GoalNode;
  ApaHybridNode_t CurrentNode;
  ApaHybridNode_t GoalResultNode;
  ApaRsPath_t AnalyticRsPath;
  ApaPath_t AnalyticPath;
  Real StartPose[3];
  /* Optional reverse-search start region; zero requires an exact start. No IPC. */
  Real StartPositionTolerance;
  Real GoalPose[3];
  Integer ParkingType;
  Integer CollisionDemand;
  Boolean ReversePlanFlag;
  Boolean ReverseRefreshPending;
  Boolean ActiveSlotValid;
  Boolean UseDefaultCollisionBridge;
  Boolean AnalyticPathValid;
  Boolean Seeded;
  Boolean GoalReached;
  Boolean GoalPoseValid;
  Boolean ParkingOutFlag;
  Boolean VerticalLastStep;
  Integer ExpandIterations;
  Integer MaxExpandIterations;
  Integer16 CurrentNodeIndex;
  ApaHybridAStarFault_e Fault;
  ApaHybridAStarState_e RunState;
} ApaHybridAStar_t;

static inline Boolean apaHybridAStarParkingSearch(const ApaHybridAStar_t *planner) {
  return planner->PlannerGlobal && planner->PlannerGlobal->Planner.Conf.ParkingSearch;
}

/* Gear/steer changes continuation cost; segment zero has independent bounds. */
static inline UInteger16 apaHybridAStarMotionClass(const ApaHybridAStar_t *planner,
                                                   const ApaHybridNode_t *node) {
  if (!apaHybridAStarParkingSearch(planner) || node->ParentNodeIndex < 0)
    return 0;
  const ApaPlannerVehConfig_t *veh = &planner->PlannerGlobal->Planner.Veh;
  const Integer turn = (Integer)roundf(2.0f * veh->MinCircle * tanf(node->Delta) / veh->WB) + 2;
  return (UInteger16)(1 + (node->D > 0.0f ? 5 : 0) + turn + (node->SegCnt == 0 ? 0 : 10));
}

static inline Boolean apaHybridAStarBuildMotionPathFromGoal(const ApaHybridAStar_t *planner,
                                                            const ApaHybridNode_t *goalNode,
                                                            const ApaRsPath_t *rsPath,
                                                            const ApaPath_t *analyticPath,
                                                            ApaHybridAStarMotionPath_t *motion);

static inline Boolean apaHybridAStarCheckGoalCandidate(ApaHybridAStar_t *planner,
                                                       const ApaHybridNode_t *goalNode,
                                                       const ApaRsPath_t *rsPath,
                                                       const ApaPath_t *analyticPath) {
  ApaHybridAStarMotionPath_t motion;
  ApaPath_t qualityPath;
  if (!apaHybridAStarBuildMotionPathFromGoal(planner, goalNode, rsPath, analyticPath, &motion)) {
    apaPlanningRecordFailure(
        planner->Diagnostics, APA_FAULT_HYBRID_UNKNOWN_ERR, "hybrid_reconstruction", 1);
    if (++planner->RejectedGoalCandidates <= 8)
      fprintf(stderr,
              "[APA_HYBRID_CANDIDATE] accepted=0 reason=reconstruction_or_capacity "
              "analytic_points=%ld continue_search=1\n",
              analyticPath ? (long)analyticPath->PathN : 0L);
    return false;
  }
  qualityPath = motion.Path;
  if (planner->CandidateCheck &&
      !planner->CandidateCheck(&motion, &qualityPath, planner->CandidateUserData)) {
    ++planner->RejectedGoalCandidates;
    return false;
  }
  ++planner->AcceptedCandidates;
  const Boolean better =
      !planner->HasAcceptedCandidate ||
      apaPathQualityCompare(&planner->BestQualityPath, &qualityPath, &planner->QualityContext) ==
          APA_PATH_COMP_NEW_PATH_BETTER;
  if (better) {
    planner->BestQualityPath = qualityPath;
    planner->GoalResultNode = *goalNode;
    planner->AnalyticPathValid = rsPath && analyticPath;
    if (planner->AnalyticPathValid) {
      planner->AnalyticRsPath = *rsPath;
      planner->AnalyticPath = *analyticPath;
    }
    if (planner->CandidateSelected)
      planner->CandidateSelected(planner->CandidateUserData);
  }
  planner->HasAcceptedCandidate = true;
  return true;
}

static inline Boolean apaHybridAStarShouldRejectByCollision(const ApaHybridAStar_t *planner,
                                                            const ApaHybridNode_t *fromNode,
                                                            const ApaHybridNode_t *toNode);

static inline void apaHybridAStarSyncDefaultCollisionEnv(ApaHybridAStar_t *planner);

static inline Boolean apaHybridAStarTryResolveParentNode(const ApaHybridAStar_t *planner,
                                                         const ApaHybridNode_t *node,
                                                         ApaHybridNode_t *parentNode);

static inline ApaHybridAStarState_e apaHybridAStarRunToTerminal(ApaHybridAStar_t *planner);

static inline Boolean
apaHybridAStarShouldRejectPrimitiveByCollision(const ApaHybridAStar_t *planner,
                                               const ApaHybridNode_t *fromNode,
                                               const ApaHybridNode_t *toNode);

static inline Boolean
apaHybridAStarShouldRejectConnectorByCollision(const ApaHybridAStar_t *planner,
                                               const ApaHybridNode_t *fromNode,
                                               const ApaHybridNode_t *toNode);

typedef struct {
  void (*Visit)(const ApaRsPath_t *path, void *context);
  void *Context;
} ApaRsVisitor_t;

static inline Real apaHybridAStarRsDistance(const ApaHybridAStar_t *planner,
                                            const ApaHybridNode_t *node);

static inline void apaHybridAStarRefreshOpenSetAfterReversePhase(ApaHybridAStar_t *planner,
                                                                 ApaHybridNode_t *pendingNode);

static inline void apaHybridAStarGridInit(ApaHybridAStarGridSpec_t *grid) {
  grid->OriginX = 0.0f;
  grid->OriginY = 0.0f;
  grid->XResolution = 1.0f;
  grid->YResolution = 1.0f;
  grid->YawOrigin = -APA_PI;
  grid->YawResolution = APA_PI;
  grid->XCount = 0;
  grid->YCount = 0;
  grid->YawCount = 0;
}

static inline void apaHybridAStarGridInitFromPlanner(ApaHybridAStarGridSpec_t *grid,
                                                     const ApaPlannerMapConfig_t *mapConfig) {
  Real halfMapLength;

  halfMapLength = 0.5f * mapConfig->MapEdgeLen;
  grid->OriginX = -halfMapLength;
  grid->OriginY = -halfMapLength;
  grid->XResolution = mapConfig->XyGridResolution;
  grid->YResolution = mapConfig->XyGridResolution;
  grid->YawOrigin = mapConfig->MinYaw;
  grid->YawResolution = mapConfig->YawGridResolution;
  grid->XCount = mapConfig->MapEdgePixNum;
  grid->YCount = mapConfig->MapEdgePixNum;
  grid->YawCount = mapConfig->MapYawPixNum;
}

static inline void apaHybridAStarInit(ApaHybridAStar_t *planner) {
  Integer i;

  planner->PlannerGlobal = (ApaPlannerGlobalState_t *)0;
  planner->CollisionCheck = (ApaHybridAStarCollisionCheck_f)0;
  planner->CollisionUserData = (void *)0;
  planner->CandidateCheck = (ApaHybridAStarCandidateCheck_f)0;
  planner->CandidateSelected = 0;
  planner->EvaluateCandidates = 0;
  planner->BuildParkingSeed = 0;
  planner->CandidateUserData = (void *)0;
  planner->Diagnostics = 0;
  apaPathQualityCompareContextInit(&planner->QualityContext);
  apaPathInit(&planner->BestQualityPath);
  planner->ExpectedSegmentCount = 1;
  planner->HasAcceptedCandidate = false;
  planner->AcceptedCandidates = 0;
  planner->TruncatedPrimitives = 0;
  planner->MotionCandidates = planner->OpenDominated = planner->ClosedDominated =
      planner->ReopenedStates = 0;
  planner->RejectedGoalCandidates = 0;
  apaSlotInit(&planner->ActiveSlot);
  apaHybridAStarCollisionEnvInit(&planner->FirstSegmentCollisionEnv);
  planner->FirstSegmentBoundsValid = false;
  apaNodePoolInit(&planner->NodePool);
  apaHashMapInit(&planner->OpenNodes);
  apaHashMapInit(&planner->CloseNodes);
  apaPqInit(&planner->OpenQueue);
  apaPlannerHeuristicContextInitDefault(&planner->HeuristicContext);
  apaSlotHeuristicViewInit(&planner->SlotView);
  apaHybridAStarGridInit(&planner->GridSpec);
  apaNodeInit(&planner->StartNode);
  apaNodeInit(&planner->GoalNode);
  apaNodeInit(&planner->CurrentNode);
  apaNodeInit(&planner->GoalResultNode);
  apaRsPathInit(&planner->AnalyticRsPath);
  apaPathInit(&planner->AnalyticPath);

  for (i = 0; i < 3; i = i + 1) {
    planner->StartPose[i] = 0.0f;
    planner->GoalPose[i] = 0.0f;
  }
  planner->StartPositionTolerance = 0.0f;

  planner->ParkingType = APA_PARKING_TYPE_UNKNOWN;
  planner->CollisionDemand = APA_COLLISION_DEMAND_SIMPLIFY;
  planner->ReversePlanFlag = false;
  planner->ReverseRefreshPending = false;
  planner->ActiveSlotValid = false;
  planner->UseDefaultCollisionBridge = false;
  planner->AnalyticPathValid = false;
  planner->Seeded = false;
  planner->GoalReached = false;
  planner->GoalPoseValid = false;
  planner->ParkingOutFlag = false;
  planner->VerticalLastStep = false;
  planner->ExpandIterations = 0;
  planner->MaxExpandIterations = APA_HYBRID_ASTAR_MAX_ITERATIONS;
  planner->CurrentNodeIndex = -1;
  planner->Fault = APA_HYBRID_FAULT_NO_ERR;
  planner->RunState = APA_HYBRID_STATE_IDLE;
}

static inline void apaHybridAStarResetSearch(ApaHybridAStar_t *planner) {
  planner->RejectedGoalCandidates = 0;
  planner->HasAcceptedCandidate = false;
  planner->AcceptedCandidates = 0;
  planner->TruncatedPrimitives = 0;
  planner->MotionCandidates = planner->OpenDominated = planner->ClosedDominated =
      planner->ReopenedStates = 0;
  apaPathInit(&planner->BestQualityPath);
  apaNodePoolInit(&planner->NodePool);
  apaHashMapInit(&planner->OpenNodes);
  apaHashMapInit(&planner->CloseNodes);
  apaPqInit(&planner->OpenQueue);
  apaNodeInit(&planner->CurrentNode);
  apaNodeInit(&planner->GoalResultNode);
  apaRsPathInit(&planner->AnalyticRsPath);
  apaPathInit(&planner->AnalyticPath);
  planner->AnalyticPathValid = false;
  planner->Seeded = false;
  planner->GoalReached = false;
  planner->ReverseRefreshPending = false;
  planner->ExpandIterations = 0;
  planner->CurrentNodeIndex = -1;
  planner->Fault = APA_HYBRID_FAULT_NO_ERR;
  planner->RunState = APA_HYBRID_STATE_IDLE;
}

static inline void apaHybridAStarSetCollisionCheck(ApaHybridAStar_t *planner,
                                                   ApaHybridAStarCollisionCheck_f collisionCheck,
                                                   void *userData) {
  planner->CollisionCheck = collisionCheck;
  planner->CollisionUserData = userData;
  planner->UseDefaultCollisionBridge = false;
}

static inline void
apaHybridAStarSetDefaultCollisionEnv(ApaHybridAStar_t *planner,
                                     ApaHybridAStarCollisionEnv_t *collisionEnv) {
  planner->UseDefaultCollisionBridge = true;
  apaHybridAStarSetCollisionCheck(planner, apaHybridAStarCollisionCheckDefault, collisionEnv);
  planner->UseDefaultCollisionBridge = true;
  apaHybridAStarSyncDefaultCollisionEnv(planner);
}

static inline void apaHybridAStarSetCollisionDemand(ApaHybridAStar_t *planner,
                                                    Integer collisionDemand) {
  planner->CollisionDemand = collisionDemand;
  apaHybridAStarSyncDefaultCollisionEnv(planner);
}

static inline void apaHybridAStarSetParkingOutFlag(ApaHybridAStar_t *planner,
                                                   Boolean parkingOutFlag) {
  planner->ParkingOutFlag = parkingOutFlag;
  apaHybridAStarSyncDefaultCollisionEnv(planner);
}

static inline void apaHybridAStarSetVerticalLastStep(ApaHybridAStar_t *planner,
                                                     Boolean verticalLastStep) {
  planner->VerticalLastStep = verticalLastStep;
  apaHybridAStarSyncDefaultCollisionEnv(planner);
}

static inline void apaHybridAStarSyncDefaultCollisionEnv(ApaHybridAStar_t *planner) {
  ApaHybridAStarCollisionEnv_t *collisionEnv;

  if (!planner->UseDefaultCollisionBridge) {
    return;
  }
  if (planner->CollisionUserData == (void *)0) {
    return;
  }

  collisionEnv = (ApaHybridAStarCollisionEnv_t *)planner->CollisionUserData;
  apaHybridAStarCollisionEnvSetPlannerGlobal(collisionEnv, planner->PlannerGlobal);
  apaHybridAStarCollisionEnvSetCollisionDemand(collisionEnv, planner->CollisionDemand);
  apaHybridAStarCollisionEnvSetParkingOutFlag(collisionEnv, planner->ParkingOutFlag);
  apaHybridAStarCollisionEnvSetVerticalLastStep(collisionEnv, planner->VerticalLastStep);
  if (planner->ActiveSlotValid) {
    apaHybridAStarCollisionEnvSetSlot(collisionEnv, &planner->ActiveSlot);
  } else {
    apaHybridAStarCollisionEnvClearSlot(collisionEnv);
  }
  if (planner->GoalPoseValid) {
    apaHybridAStarCollisionEnvSetTargetPose(collisionEnv, planner->GoalPose);
  } else {
    apaHybridAStarCollisionEnvClearTargetPose(collisionEnv);
  }
}

static inline void apaHybridAStarSyncHeuristicContextFromPlanner(ApaHybridAStar_t *planner) {
  const ApaPlannerConfig_t *plannerConfig;

  if (planner->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return;
  }

  plannerConfig = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  planner->HeuristicContext.MinCircle = plannerConfig->Veh.MinCircle;
  planner->HeuristicContext.HCost = plannerConfig->Map.HCost;
  planner->HeuristicContext.UseGCost = (Boolean)(plannerConfig->Conf.UseGCost != 0);
  planner->HeuristicContext.ReverseFlag = (UInteger8)plannerConfig->Slot.ReverseFlag;
  planner->HeuristicContext.ReverseTheta = plannerConfig->Slot.ReverseTheta;
}

static inline void apaHybridAStarSyncHeuristicContextToPlanner(ApaHybridAStar_t *planner) {
  if (planner->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return;
  }

  apaPlannerGlobalSetReverseFlag(planner->PlannerGlobal,
                                 (Integer)planner->HeuristicContext.ReverseFlag);
  apaPlannerGlobalSetReverseTheta(planner->PlannerGlobal, planner->HeuristicContext.ReverseTheta);
}

static inline void apaHybridAStarSetPlannerGlobal(ApaHybridAStar_t *planner,
                                                  ApaPlannerGlobalState_t *plannerGlobal) {
  const ApaPlannerMapConfig_t *mapConfig;

  planner->PlannerGlobal = plannerGlobal;
  if (plannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return;
  }

  mapConfig = apaPlannerGlobalGetMap(plannerGlobal);
  apaHybridAStarGridInitFromPlanner(&planner->GridSpec, mapConfig);
  apaHybridAStarSyncHeuristicContextFromPlanner(planner);
  apaHybridAStarSyncDefaultCollisionEnv(planner);
  planner->RunState = APA_HYBRID_STATE_READY;
}

static inline void apaHybridAStarLoadSlot(ApaHybridAStar_t *planner, const ApaSlot_t *slot) {
  Integer slotClass;

  apaSlotCopy(&planner->ActiveSlot, slot);
  planner->ActiveSlotValid = true;
  planner->SlotView.TarPos[0] = slot->TarPos[0];
  planner->SlotView.TarPos[1] = slot->TarPos[1];
  planner->SlotView.TarPos[2] = slot->TarPos[2];

  slotClass = slot->SlotClass;
  if (slotClass == APA_SLOT_CLASS_UNKNOWN) {
    slotClass = apaSlotCalcSlotClass(slot->SlotPts, slot->EnterEdge);
  }

  planner->SlotView.SlotClass = slotClass;
  planner->SlotView.SlotSubStatus = slot->SlotSubStatus;
  planner->ParkingType = slot->ParkingType;
  apaHybridAStarSyncDefaultCollisionEnv(planner);
}

static inline Integer apaHybridAStarRoundToInteger(Real value) {
  if (value >= 0.0f) {
    return (Integer)floorf(value + 0.5f);
  }
  return (Integer)ceilf(value - 0.5f);
}

static inline Boolean apaHybridAStarPoseToGrid(const ApaHybridAStarGridSpec_t *grid,
                                               const Real pose[3],
                                               UInteger16 *xidx,
                                               UInteger16 *yidx,
                                               UInteger16 *yawidx) {
  Real yawWrapped;
  Real yawOffset;
  Integer xIndex;
  Integer yIndex;
  Integer yawIndex;

  if (grid->XCount <= 0 || grid->YCount <= 0 || grid->YawCount <= 0) {
    return false;
  }
  if (apaMathLess(grid->XResolution, APA_MATH_EPSILON) ||
      apaMathLess(grid->YResolution, APA_MATH_EPSILON) ||
      apaMathLess(grid->YawResolution, APA_MATH_EPSILON)) {
    return false;
  }

  xIndex = apaHybridAStarRoundToInteger((pose[0] - grid->OriginX) / grid->XResolution);
  yIndex = apaHybridAStarRoundToInteger((pose[1] - grid->OriginY) / grid->YResolution);

  yawWrapped = apaCoordWrapToPi(pose[2]);
  yawOffset = yawWrapped - grid->YawOrigin;
  if (yawOffset < 0.0f) {
    yawOffset = yawOffset + APA_TWO_PI;
  }

  yawIndex = (Integer)ceilf(yawOffset / grid->YawResolution) - 1;
  if (yawIndex < 0)
    yawIndex = grid->YawCount - 1;
  if (yawIndex >= grid->YawCount) {
    yawIndex = grid->YawCount - 1;
  }

  if (xIndex < 0 || xIndex >= grid->XCount) {
    return false;
  }
  if (yIndex < 0 || yIndex >= grid->YCount) {
    return false;
  }
  if (yawIndex < 0 || yawIndex >= grid->YawCount) {
    return false;
  }

  *xidx = (UInteger16)xIndex;
  *yidx = (UInteger16)yIndex;
  *yawidx = (UInteger16)yawIndex;
  return true;
}

static inline Boolean apaHybridAStarNodeInitFromPose(const ApaHybridAStarGridSpec_t *grid,
                                                     const Real pose[3],
                                                     Real travelDistance,
                                                     Real steerAngle,
                                                     Integer parentNodeIndex,
                                                     UInteger8 segCnt,
                                                     Real trajCost,
                                                     Real heuristicCost,
                                                     Real totalCost,
                                                     ApaHybridNode_t *node) {
  UInteger16 xidx;
  UInteger16 yidx;
  UInteger16 yawidx;
  Boolean validIndex;

  validIndex = apaHybridAStarPoseToGrid(grid, pose, &xidx, &yidx, &yawidx);
  if (!validIndex) {
    apaNodeInit(node);
    return false;
  }

  node->Xidx = xidx;
  node->Yidx = yidx;
  node->Yawidx = yawidx;
  node->MotionClass = 0U;
  node->D = travelDistance;
  node->Delta = steerAngle;
  node->X = pose[0];
  node->Y = pose[1];
  node->Theta = apaCoordWrapToPi(pose[2]);
  node->ParentNodeIndex = parentNodeIndex;
  node->SegCnt = segCnt;
  node->TrajCost = trajCost;
  node->HeuristicCost = heuristicCost;
  node->TotalCost = totalCost;
  return true;
}

static inline Boolean apaHybridAStarLoadStartGoalPoses(ApaHybridAStar_t *planner,
                                                       const Real startPose[3],
                                                       const Real goalPose[3]) {
  Boolean startOk;
  Boolean goalOk;

  planner->StartPose[0] = startPose[0];
  planner->StartPose[1] = startPose[1];
  planner->StartPose[2] = apaCoordWrapToPi(startPose[2]);
  planner->GoalPose[0] = goalPose[0];
  planner->GoalPose[1] = goalPose[1];
  planner->GoalPose[2] = apaCoordWrapToPi(goalPose[2]);
  planner->GoalPoseValid = true;
  apaHybridAStarSyncDefaultCollisionEnv(planner);

  startOk = apaHybridAStarNodeInitFromPose(
      &planner->GridSpec,
      planner->StartPose,

      planner->PlannerGlobal ? apaPlannerGlobalGetMap(planner->PlannerGlobal)->MotionResolution
                             : 0.0f,
      0.0f,
      -1,
      0U,
      0.0f,
      0.0f,
      0.0f,
      &planner->StartNode);
  goalOk = apaHybridAStarNodeInitFromPose(&planner->GridSpec,
                                          planner->GoalPose,
                                          0.0f,
                                          0.0f,
                                          -1,
                                          0U,
                                          0.0f,
                                          0.0f,
                                          0.0f,
                                          &planner->GoalNode);

  if (!startOk) {
    planner->Fault = APA_HYBRID_FAULT_ILLEGAL_START_GRID;
    planner->RunState = APA_HYBRID_STATE_FAILED;
    return false;
  }
  if (!goalOk) {
    planner->Fault = APA_HYBRID_FAULT_ILLEGAL_GOAL_GRID;
    planner->RunState = APA_HYBRID_STATE_FAILED;
    return false;
  }

  planner->Fault = APA_HYBRID_FAULT_NO_ERR;
  if (planner->RunState != APA_HYBRID_STATE_FAILED) {
    planner->RunState = APA_HYBRID_STATE_READY;
  }
  return true;
}

static inline void apaHybridAStarConfigure(ApaHybridAStar_t *planner,
                                           ApaPlannerGlobalState_t *plannerGlobal,
                                           const Real startPose[3],
                                           const Real goalPose[3],
                                           const ApaSlot_t *slot,
                                           Boolean reversePlanFlag) {
  apaHybridAStarInit(planner);
  apaHybridAStarSetPlannerGlobal(planner, plannerGlobal);
  if (apaHybridAStarParkingSearch(planner))
    planner->MaxExpandIterations = plannerGlobal->Planner.Conf.PlanningCalPowerMaxPerSlot;
  apaHybridAStarLoadSlot(planner, slot);
  planner->ReversePlanFlag = reversePlanFlag;

  planner->GridSpec.OriginX = 0.5f * (startPose[0] + goalPose[0]) -
                              0.5f * (planner->GridSpec.XCount - 1) * planner->GridSpec.XResolution;
  planner->GridSpec.OriginY = 0.5f * (startPose[1] + goalPose[1]) -
                              0.5f * (planner->GridSpec.YCount - 1) * planner->GridSpec.YResolution;
  (void)apaHybridAStarLoadStartGoalPoses(planner, startPose, goalPose);
}

static inline Real apaHybridAStarCalcHeuristic(ApaHybridAStar_t *planner,
                                               const ApaHybridNode_t *node) {
  ApaPlannerHeuristicContext_t context;
  Real heuristicCost;
  UInteger8 reverseFlagBefore;

  if (apaHybridAStarParkingSearch(planner)) {
    const Real distance = hypotf(node->X - planner->GoalNode.X, node->Y - planner->GoalNode.Y);
    const Real heading = planner->PlannerGlobal->Planner.Veh.MinCircle *
                         fabsf(apaCoordWrapToPi(node->Theta - planner->GoalNode.Theta));
    return fmaxf(fmaxf(distance, heading), apaHybridAStarRsDistance(planner, node));
  }

  reverseFlagBefore = planner->HeuristicContext.ReverseFlag;
  context = planner->HeuristicContext;
  heuristicCost = apaSearchGetHeuristicCost(&planner->SlotView,
                                            node,
                                            &planner->GoalNode,
                                            planner->ReversePlanFlag,
                                            planner->ParkingType,
                                            &context);
  planner->HeuristicContext = context;
  if (context.ReverseFlag != reverseFlagBefore) {
    planner->ReverseRefreshPending = true;
  }
  apaHybridAStarSyncHeuristicContextToPlanner(planner);
  return heuristicCost;
}

static inline Boolean
apaHybridAStarShouldRefreshOpenSetForReversePhase(const ApaHybridAStar_t *planner) {
  Integer slotClass;
  Integer slotSubStatus;

  if (!planner->ReverseRefreshPending) {
    return false;
  }

  slotClass = planner->SlotView.SlotClass;
  slotSubStatus = planner->SlotView.SlotSubStatus;
  if (slotClass == APA_SLOT_CLASS_PARALLEL) {
    return false;
  }
  if (slotSubStatus != APA_SLOT_SUB_END_FIRST && slotSubStatus != APA_SLOT_SUB_END_SECOND) {
    return false;
  }
  return true;
}

static inline Boolean apaHybridAStarShouldUsePriorityQueue(const ApaHybridAStar_t *planner) {
  const ApaPlannerConfig_t *plannerConfig;

  if (planner->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return true;
  }

  plannerConfig = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  return (Boolean)(plannerConfig->Conf.UsePriorityQueue != 0);
}

static inline void apaHybridAStarRecomputeNodeCost(const ApaHybridAStar_t *planner,
                                                   ApaHybridNode_t *node) {
  ApaPlannerHeuristicContext_t context;
  Real heuristicCost;

  context = planner->HeuristicContext;
  heuristicCost = apaSearchGetHeuristicCost(&planner->SlotView,
                                            node,
                                            &planner->GoalNode,
                                            planner->ReversePlanFlag,
                                            planner->ParkingType,
                                            &context);
  node->HeuristicCost = heuristicCost;
  node->TotalCost = apaTotalCost(node->TrajCost,
                                 heuristicCost,
                                 planner->HeuristicContext.HCost,
                                 planner->HeuristicContext.UseGCost);
}

static inline void apaHybridAStarRefreshOpenNodeCosts(ApaHybridAStar_t *planner) {
  Integer nodeIndex;

  for (nodeIndex = 0; nodeIndex < APA_HASH_CAPACITY; nodeIndex = nodeIndex + 1) {
    if (apaNodePoolGetState(&planner->NodePool, nodeIndex) == (UInteger8)APA_NODE_STATE_OPEN) {
      apaHybridAStarRecomputeNodeCost(planner, &planner->NodePool.Nodes[nodeIndex]);
    }
  }
}

static inline void apaHybridAStarRefreshOpenQueueCosts(ApaHybridAStar_t *planner) {
  Integer queueIndex;

  for (queueIndex = 0; queueIndex < planner->OpenQueue.Count; queueIndex = queueIndex + 1) {
    Integer openIndex;
    Boolean openIndexValid;
    Boolean openNode;

    openIndex = planner->OpenQueue.NodeIndexByHeap[queueIndex];
    openIndexValid = (openIndex >= 0);
    if (openIndexValid) {
      openIndexValid = (openIndex < APA_HASH_CAPACITY);
    }
    openNode = false;
    if (openIndexValid) {
      openNode =
          (apaNodePoolGetState(&planner->NodePool, openIndex) == (UInteger8)APA_NODE_STATE_OPEN);
    }
    if (openNode) {
      planner->OpenQueue.TotalCostByHeap[queueIndex] = planner->NodePool.Nodes[openIndex].TotalCost;
    }
  }
  apaPqRebuild(&planner->OpenQueue);
}

static inline Boolean apaHybridAStarRefreshHashNodesList(ApaHybridAStar_t *planner,
                                                         ApaHybridNode_t *pendingNode) {
  Boolean refreshFlag;

  refreshFlag = false;

  if (!planner->ReverseRefreshPending) {
    return false;
  }
  if (apaHybridAStarShouldUsePriorityQueue(planner)) {
    return false;
  }

  if (apaHybridAStarShouldRefreshOpenSetForReversePhase(planner)) {
    apaHybridAStarRefreshOpenNodeCosts(planner);
    if (pendingNode != (ApaHybridNode_t *)0) {
      apaHybridAStarRecomputeNodeCost(planner, pendingNode);
    }
    refreshFlag = true;
  }
  planner->ReverseRefreshPending = false;

  return refreshFlag;
}

static inline Boolean apaHybridAStarRefreshPriorityQueueNodesList(ApaHybridAStar_t *planner,
                                                                  ApaHybridNode_t *pendingNode) {
  Boolean refreshFlag;

  refreshFlag = false;

  if (!planner->ReverseRefreshPending) {
    return false;
  }
  if (!apaHybridAStarShouldUsePriorityQueue(planner)) {
    return false;
  }

  if (planner->ReverseRefreshPending) {
    if (apaHybridAStarShouldRefreshOpenSetForReversePhase(planner)) {
      apaHybridAStarRefreshOpenNodeCosts(planner);
      apaHybridAStarRefreshOpenQueueCosts(planner);
      if (pendingNode != (ApaHybridNode_t *)0) {
        apaHybridAStarRecomputeNodeCost(planner, pendingNode);
      }
      refreshFlag = true;
    }
    planner->ReverseRefreshPending = false;
  }

  return refreshFlag;
}

static inline void apaHybridAStarRefreshOpenSetAfterReversePhase(ApaHybridAStar_t *planner,
                                                                 ApaHybridNode_t *pendingNode) {
  if (apaHybridAStarShouldUsePriorityQueue(planner)) {
    (void)apaHybridAStarRefreshPriorityQueueNodesList(planner, pendingNode);
  } else if (apaHybridAStarRefreshHashNodesList(planner, pendingNode)) {
    apaHybridAStarRefreshOpenQueueCosts(planner);
  } else {
  }
}

static inline Integer apaHybridAStarGetSteerSampleCount(const ApaHybridAStar_t *planner) {
  const ApaPlannerConfig_t *plannerConfig;
  Integer sampleCount;

  if (planner->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return 1;
  }

  plannerConfig = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);

  sampleCount = 2 * (Integer)floorf(plannerConfig->Map.NSteer + 0.5f) + 1;
  if (sampleCount < 1) {
    sampleCount = 1;
  }
  return sampleCount;
}

static inline Real apaHybridAStarCalcSteerAngle(const ApaHybridAStar_t *planner,
                                                Integer steerIndex,
                                                Integer steerCount) {
  const ApaPlannerConfig_t *plannerConfig;
  Real maxSteer;
  Real ratio;
  Real steerAngle;

  if (planner->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return 0.0f;
  }

  plannerConfig = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  maxSteer = plannerConfig->Veh.MaxSteer;
  if (steerCount <= 1) {
    return 0.0f;
  }

  ratio = ((Real)steerIndex / (Real)(steerCount - 1)) * 2.0f;
  steerAngle = (ratio - 1.0f) * maxSteer;
  return steerAngle;
}

static inline void apaHybridAStarAdvancePose(const ApaPlannerVehConfig_t *vehConfig,
                                             const Real currentPose[3],
                                             Real travelDistance,
                                             Real steerAngle,
                                             Real nextPose[3]) {
  Real absSteer;

  absSteer = apaMathAbs(steerAngle);
  if (absSteer < APA_HYBRID_ASTAR_ZERO_STEER_EPS) {
    apaCoordAdvanceStraightPose(currentPose, travelDistance, nextPose);
  } else {
    Real turnRadius;
    Real yawStep;
    Real centerHeading;
    Real centerX;
    Real centerY;

    turnRadius = vehConfig->WB / tanf(steerAngle);
    yawStep = travelDistance / turnRadius;
    centerHeading = currentPose[2] - APA_PI * 0.5f;
    centerX = currentPose[0] - (turnRadius * cosf(centerHeading));
    centerY = currentPose[1] - (turnRadius * sinf(centerHeading));
    nextPose[2] = apaCoordWrapToPi(currentPose[2] + yawStep);
    nextPose[0] = centerX + (turnRadius * cosf(centerHeading + yawStep));
    nextPose[1] = centerY + (turnRadius * sinf(centerHeading + yawStep));
  }
}

/* Local_Planner/choose_step_length uses front axle or axle midpoint for
 * front-in parking, and rear axle for the other parking directions. */
static inline Boolean apaHybridAStarStepPoseInSlot(const ApaHybridAStar_t *planner,
                                                   const Real pose[3]) {
  const Real *slotPts = planner->ActiveSlot.SlotPts;
  Real frontX;
  Real frontY;

  if (planner->ActiveSlot.ParkingDirec != APA_PARKING_DIREC_FRONT) {
    return apaMathIsPtInRect(pose[0], pose[1], slotPts);
  }
  apaVehicleFrontAxlePt(
      pose[0], pose[1], pose[2], planner->PlannerGlobal->Planner.Veh.WB, &frontX, &frontY);
  return apaMathIsPtInRect(frontX, frontY, slotPts) ||
         apaMathIsPtInRect(0.5f * (pose[0] + frontX), 0.5f * (pose[1] + frontY), slotPts);
}

/* Archived Local_Planner/choose_step_length frame, rounding and direction semantics. */
static inline void apaHybridAStarChooseStepLength(ApaHybridAStar_t *planner, Integer direction) {
  const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  Real pose[3] = {planner->CurrentNode.X, planner->CurrentNode.Y, planner->CurrentNode.Theta};
  Real targetLocal[3];
  Real endPose[3];
  Real dx = pose[0] - planner->GoalNode.X;
  Real dy = pose[1] - planner->GoalNode.Y;
  Real yawAdjustLength =
      config->Veh.MinCircle * fabsf(apaCoordWrapToPi(pose[2] - planner->GoalNode.Theta));
  Real gridResolution = config->Map.XyGridResolution;
  Real stepLength;
  Real stepLengthTimes;
  Boolean frontIn = planner->ActiveSlot.ParkingDirec == APA_PARKING_DIREC_FRONT;
  Boolean startInSlot;
  Boolean endInSlot;

  apaCoordGroundToVehPose(planner->GoalPose, pose, targetLocal);
  stepLength = 0.5f * (sqrtf(dx * dx + dy * dy) + fabsf(targetLocal[1]));
  stepLength = 0.5f * (stepLength + yawAdjustLength);
  stepLength = fmaxf(stepLength, 1.5f * gridResolution);
  stepLength = fminf(stepLength, frontIn ? 8.0f : 4.0f);
  stepLengthTimes = roundf(10.0f * stepLength / gridResolution) / 10.0f;

  apaCoordAdvanceStraightPose(pose, (Real)direction * stepLengthTimes * gridResolution, endPose);
  startInSlot = apaHybridAStarStepPoseInSlot(planner, pose);
  endInSlot = apaHybridAStarStepPoseInSlot(planner, endPose);
  if (startInSlot && !endInSlot) {
    Real limitedTimes = fminf(stepLengthTimes, 1.0f / gridResolution);
    apaCoordAdvanceStraightPose(pose, (Real)direction * limitedTimes * gridResolution, endPose);
    endInSlot = apaHybridAStarStepPoseInSlot(planner, endPose);
    if ((frontIn && endInSlot) || (!frontIn && !endInSlot)) {
      limitedTimes = fminf(stepLengthTimes, 0.5f / gridResolution);
    }
    stepLengthTimes = limitedTimes;
  } else if (!startInSlot && endInSlot) {
    stepLengthTimes = fmaxf(stepLengthTimes / 2.0f, 0.5f / gridResolution);
  }
  apaPlannerGlobalSetStepLengthTimes(planner->PlannerGlobal, stepLengthTimes);
}

static inline UInteger8 apaHybridAStarNextSegmentCount(const ApaHybridNode_t *parent,
                                                       Real travelDistance) {
  return (UInteger8)(parent->SegCnt + (apaMathLess(parent->D * travelDistance, 0.0f) ? 1U : 0U));
}

/* First-segment bounds use search direction, including reverse search. */
static inline Boolean apaHybridAStarAllowFirstSegmentOptim(const ApaHybridAStar_t *planner) {
  const Integer surrounding = planner->ActiveSlot.SurroundingType;
  return planner->ActiveSlotValid && planner->ReversePlanFlag &&
         planner->SlotView.SlotClass == APA_SLOT_CLASS_VERTICAL &&
         planner->SlotView.SlotSubStatus == APA_SLOT_SUB_END_NONE &&
         (surrounding == APA_SURROUND_OBS_EXIST_BOTH ||
          (surrounding == APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY &&
           planner->ParkingType == APA_PARKING_TYPE_LEFT) ||
          (surrounding == APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY &&
           planner->ParkingType == APA_PARKING_TYPE_RIGHT));
}

static inline void apaHybridAStarConfigureFirstSegmentBounds(ApaHybridAStar_t *planner) {
  ApaHybridAStarCollisionEnv_t *env = &planner->FirstSegmentCollisionEnv;
  const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  Real left[4], right[4];
  Real sideX, sideY, sideLength, latX, latY, latLength;

  planner->FirstSegmentBoundsValid = false;
  apaHybridAStarCollisionEnvInit(env);
  if (!config->Conf.EnableFirstSegVirtualBound || !apaHybridAStarAllowFirstSegmentOptim(planner))
    return;

  apaSlotCalcSideLine(planner->ActiveSlot.SlotPts,
                      planner->ActiveSlot.EnterEdge,
                      planner->ActiveSlot.TarPos,
                      planner->SlotView.SlotClass,
                      left,
                      right);
  sideX = left[0] - left[2];
  sideY = left[1] - left[3];
  sideLength = sqrtf(sideX * sideX + sideY * sideY);
  latX = left[0] - right[0];
  latY = left[1] - right[1];
  latLength = sqrtf(latX * latX + latY * latY);
  if (!isfinite(sideLength) || sideLength <= APA_MATH_EPSILON || !isfinite(latLength) ||
      latLength <= APA_MATH_EPSILON || !isfinite(config->Conf.MaxFirstSegLatDis) ||
      config->Conf.MaxFirstSegLatDis < 0.0f)
    return;
  latX *= config->Conf.MaxFirstSegLatDis / latLength;
  latY *= config->Conf.MaxFirstSegLatDis / latLength;

  apaHybridAStarCollisionEnvSetPlannerGlobal(env, planner->PlannerGlobal);
  apaHybridAStarCollisionEnvSetSlot(env, &planner->ActiveSlot);
  apaHybridAStarCollisionEnvSetTargetPose(env, planner->ActiveSlot.TarPos);
  apaHybridAStarCollisionEnvSetParkingOutFlag(env, planner->ParkingOutFlag);
  apaHybridAStarCollisionEnvSetVerticalLastStep(env, planner->VerticalLastStep);

  if (apaHybridAStarCollisionEnvAddVirtualLine(env,
                                               left[0] + sideX + latX,
                                               left[1] + sideY + latY,
                                               left[2] + sideX + latX,
                                               left[3] + sideY + latY) &&
      apaHybridAStarCollisionEnvAddVirtualLine(env,
                                               right[0] + sideX - latX,
                                               right[1] + sideY - latY,
                                               right[2] + sideX - latX,
                                               right[3] + sideY - latY)) {
    planner->FirstSegmentBoundsValid = true;
  }
}

/* MATLAB's veh_start_flag suppresses the seed's artificial D switch cost,
 * except when already near the vertical slot with little pose correction. */
static inline Boolean apaHybridAStarChargeSwitchBack(const ApaHybridAStar_t *planner,
                                                     const ApaHybridNode_t *parent) {
  if (parent->ParentNodeIndex >= 0)
    return true;
  const ApaPlannerVehConfig_t *veh = &apaPlannerGlobalGetPlanner(planner->PlannerGlobal)->Veh;
  const Real dx = planner->StartPose[0] - planner->GoalPose[0];
  const Real dy = planner->StartPose[1] - planner->GoalPose[1];
  const Real distance = sqrtf(dx * dx + dy * dy);
  return planner->SlotView.SlotClass == APA_SLOT_CLASS_VERTICAL &&
         apaSlotCalcVehNearSlotFlag(planner->StartPose, planner->ActiveSlot.SlotPts, 0, veh) &&
         (apaMathLess(distance, 0.8f) ||
          (apaMathLess(fabsf(planner->StartPose[2] - planner->GoalPose[2]),
                       10.0f / 180.0f * 3.14f) &&
           apaMathLess(distance, 4.5f)));
}

/* Slot/direction costs use collision-trimmed travel and child segment count. */
static inline Real apaHybridAStarCalcTrajCost(const ApaHybridAStar_t *planner,
                                              const ApaHybridNode_t *parentNode,
                                              Real travelDistance,
                                              Real steerAngle) {
  const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  const ApaPlannerMapConfig_t *map = &config->Map;
  if (config->Conf.ParkingSearch) {
    const Boolean change =
        parentNode->ParentNodeIndex >= 0 && parentNode->D * travelDistance < 0.0f;
    return parentNode->TrajCost +
           fabsf(travelDistance) * (travelDistance < 0.0f ? map->BackCost : 1.0f) +
           (change ? map->SbCost : 0.0f) +
           map->SteerChangeCost * fabsf(steerAngle - parentNode->Delta) +
           map->SteerCost * fabsf(steerAngle) * fabsf(travelDistance);
  }
  const Real pose[3] = {parentNode->X, parentNode->Y, parentNode->Theta};
  const Boolean inSlot = apaHybridAStarStepPoseInSlot(planner, pose);
  const Integer slotClass = planner->SlotView.SlotClass;
  const Integer subStatus = planner->SlotView.SlotSubStatus;
  const Boolean switchBack = apaMathLess(parentNode->D * travelDistance, 0.0f) &&
                             apaHybridAStarChargeSwitchBack(planner, parentNode);
  Real distanceWeight = travelDistance < 0.0f ? map->BackCost : 1.0f;

  if (inSlot && slotClass == APA_SLOT_CLASS_VERTICAL) {
    if ((planner->ActiveSlot.ParkingDirec == APA_PARKING_DIREC_FRONT && travelDistance > 0.0f) ||
        (planner->ActiveSlot.ParkingDirec == APA_PARKING_DIREC_REAR && travelDistance < 0.0f))
      distanceWeight = map->BackCostInSlot;
  }
  Real cost = parentNode->TrajCost + distanceWeight * fabsf(travelDistance);
  if (switchBack)
    cost += map->SbCost;

  if (slotClass == APA_SLOT_CLASS_VERTICAL &&
      (subStatus == APA_SLOT_SUB_END_FIRST || subStatus == APA_SLOT_SUB_END_SECOND)) {
    cost +=
        (config->Veh.MaxSteer - fabsf(steerAngle)) / config->Veh.MaxSteer * config->Veh.MinCircle;
    if (switchBack)
      cost += config->Slot.ReverseFlag == 0 ? -map->SbCost : map->SbCost;
  } else if (slotClass == APA_SLOT_CLASS_PARALLEL &&
             apaMathIsPtInRect(parentNode->X, parentNode->Y, planner->ActiveSlot.SlotPts)) {
    cost += (config->Veh.MaxSteer - fabsf(steerAngle)) / config->Veh.MaxSteer *
            config->Veh.MinCircle * 1.5f;
  } else {
    const Boolean firstSegmentCost =
        apaHybridAStarNextSegmentCount(parentNode, travelDistance) == 0U &&
        apaHybridAStarAllowFirstSegmentOptim(planner) && map->EnableFirstSegDeltaCost;
    cost += (firstSegmentCost ? map->SteerCost1stSeg : map->SteerCost) * fabsf(steerAngle);
    cost += map->SteerChangeCost * fabsf(steerAngle - parentNode->Delta);
  }
  return cost;
}

static inline Real apaHybridAStarWrapPositive(Real angle) {
  Real wrapped;

  wrapped = fmodf(angle, APA_TWO_PI);
  if (wrapped < 0.0f) {
    wrapped = wrapped + APA_TWO_PI;
  }
  return wrapped;
}

static inline void apaHybridAStarRsPathInitStraight(ApaRsPath_t *path, Real length) {
  apaRsPathInit(path);
  path->Type[0] = APA_RS_SEG_STRAIGHT;
  path->Lengths[0] = length;
  path->TotalLength = apaMathAbs(length);
}

static inline Integer apaHybridAStarSwapRsSegType(Integer segType) {
  if (segType == APA_RS_SEG_LEFT) {
    return APA_RS_SEG_RIGHT;
  }
  if (segType == APA_RS_SEG_RIGHT) {
    return APA_RS_SEG_LEFT;
  }
  return segType;
}

static inline void apaHybridAStarMirrorRsPathForReverse(ApaRsPath_t *reversePath) {
  Integer i;

  for (i = 0; i < APA_RS_MAX_SEGMENTS; i = i + 1) {
    reversePath->Type[i] = apaHybridAStarSwapRsSegType(reversePath->Type[i]);
    reversePath->Lengths[i] = -reversePath->Lengths[i];
  }
}

static inline Boolean apaHybridAStarDubinsLoadScaledPath(
    Integer patternIndex, Real radius, Real t, Real u, Real v, ApaRsPath_t *path) {
  return apaRsPathInitWithLengths(
      path, patternIndex, t * radius, u * radius, v * radius, 0.0f, 0.0f);
}

static inline Boolean apaHybridAStarRsLoadScaledPath(
    Integer patternIndex, Real radius, Real t, Real u, Real v, ApaRsPath_t *path) {
  return apaRsPathInitWithLengths(
      path, patternIndex, t * radius, u * radius, v * radius, 0.0f, 0.0f);
}

static inline Boolean apaHybridAStarRsLoadScaledPath4(
    Integer patternIndex, Real radius, Real t, Real u, Real v, Real w, ApaRsPath_t *path) {
  return apaRsPathInitWithLengths(
      path, patternIndex, t * radius, u * radius, v * radius, w * radius, 0.0f);
}

static inline void apaHybridAStarCalcDubinsFrame(
    Real goalX, Real goalY, Real goalYaw, Real *alpha, Real *beta, Real *distance) {
  Real theta;

  *distance = sqrtf((goalX * goalX) + (goalY * goalY));
  theta = atan2f(goalY, goalX);
  *alpha = apaHybridAStarWrapPositive(-theta);
  *beta = apaHybridAStarWrapPositive(goalYaw - theta);
}

static inline Boolean
apaHybridAStarTryDubinsLsl(Real goalX, Real goalY, Real goalYaw, Real radius, ApaRsPath_t *path) {
  Real alpha;
  Real beta;
  Real distance;
  Real pSquared;
  Real tmp;
  Real t;
  Real u;
  Real v;

  apaHybridAStarCalcDubinsFrame(goalX, goalY, goalYaw, &alpha, &beta, &distance);
  pSquared = 2.0f + (distance * distance) - (2.0f * cosf(alpha - beta)) +
             (2.0f * distance * (sinf(alpha) - sinf(beta)));
  if (pSquared < 0.0f) {
    if (pSquared < -APA_MATH_EPSILON) {
      return false;
    }
    pSquared = 0.0f;
  }

  tmp = atan2f(cosf(beta) - cosf(alpha), distance + sinf(alpha) - sinf(beta));
  t = apaHybridAStarWrapPositive(-alpha + tmp);
  u = sqrtf(pSquared);
  v = apaHybridAStarWrapPositive(beta - tmp);
  return apaHybridAStarDubinsLoadScaledPath(APA_RS_PATTERN_LSL, radius, t, u, v, path);
}

static inline Boolean
apaHybridAStarTryDubinsRsr(Real goalX, Real goalY, Real goalYaw, Real radius, ApaRsPath_t *path) {
  Real alpha;
  Real beta;
  Real distance;
  Real pSquared;
  Real tmp;
  Real t;
  Real u;
  Real v;

  apaHybridAStarCalcDubinsFrame(goalX, goalY, goalYaw, &alpha, &beta, &distance);
  pSquared = 2.0f + (distance * distance) - (2.0f * cosf(alpha - beta)) +
             (2.0f * distance * (-sinf(alpha) + sinf(beta)));
  if (pSquared < 0.0f) {
    if (pSquared < -APA_MATH_EPSILON) {
      return false;
    }
    pSquared = 0.0f;
  }

  tmp = atan2f(cosf(alpha) - cosf(beta), distance - sinf(alpha) + sinf(beta));
  t = apaHybridAStarWrapPositive(alpha - tmp);
  u = sqrtf(pSquared);
  v = apaHybridAStarWrapPositive(-beta + tmp);
  return apaHybridAStarDubinsLoadScaledPath(APA_RS_PATTERN_RSR, radius, t, u, v, path);
}

static inline Boolean
apaHybridAStarTryDubinsLsr(Real goalX, Real goalY, Real goalYaw, Real radius, ApaRsPath_t *path) {
  Real alpha;
  Real beta;
  Real distance;
  Real pSquared;
  Real p;
  Real tmp;
  Real t;
  Real v;

  apaHybridAStarCalcDubinsFrame(goalX, goalY, goalYaw, &alpha, &beta, &distance);
  pSquared = -2.0f + (distance * distance) + (2.0f * cosf(alpha - beta)) +
             (2.0f * distance * (sinf(alpha) + sinf(beta)));
  if (pSquared < 0.0f) {
    if (pSquared < -APA_MATH_EPSILON) {
      return false;
    }
    pSquared = 0.0f;
  }

  p = sqrtf(pSquared);
  tmp = atan2f(-cosf(alpha) - cosf(beta), distance + sinf(alpha) + sinf(beta)) - atan2f(-2.0f, p);
  t = apaHybridAStarWrapPositive(-alpha + tmp);
  v = apaHybridAStarWrapPositive(-beta + tmp);
  return apaHybridAStarDubinsLoadScaledPath(APA_RS_PATTERN_LSR, radius, t, p, v, path);
}

static inline Boolean
apaHybridAStarTryDubinsRsl(Real goalX, Real goalY, Real goalYaw, Real radius, ApaRsPath_t *path) {
  Real alpha;
  Real beta;
  Real distance;
  Real pSquared;
  Real p;
  Real tmp;
  Real t;
  Real v;

  apaHybridAStarCalcDubinsFrame(goalX, goalY, goalYaw, &alpha, &beta, &distance);
  pSquared = -2.0f + (distance * distance) + (2.0f * cosf(alpha - beta)) -
             (2.0f * distance * (sinf(alpha) + sinf(beta)));
  if (pSquared < 0.0f) {
    if (pSquared < -APA_MATH_EPSILON) {
      return false;
    }
    pSquared = 0.0f;
  }

  p = sqrtf(pSquared);
  tmp = atan2f(cosf(alpha) + cosf(beta), distance - sinf(alpha) - sinf(beta)) - atan2f(2.0f, p);
  t = apaHybridAStarWrapPositive(alpha - tmp);
  v = apaHybridAStarWrapPositive(beta - tmp);
  return apaHybridAStarDubinsLoadScaledPath(APA_RS_PATTERN_RSL, radius, t, p, v, path);
}

static inline Boolean
apaHybridAStarTryRsLrlCore(Real goalX, Real goalY, Real goalYaw, Real *t, Real *u, Real *v) {
  Real shiftedX;
  Real shiftedY;
  Real polarRadius;
  Real polarTheta;
  Real asinArg;

  shiftedX = goalX - sinf(goalYaw);
  shiftedY = goalY - 1.0f + cosf(goalYaw);
  polarRadius = sqrtf((shiftedX * shiftedX) + (shiftedY * shiftedY));
  if (polarRadius > 4.0f) {
    if (polarRadius > (4.0f + APA_MATH_EPSILON)) {
      return false;
    }
    polarRadius = 4.0f;
  }

  polarTheta = atan2f(shiftedY, shiftedX);
  asinArg = 0.25f * polarRadius;
  if (asinArg > 1.0f) {
    asinArg = 1.0f;
  }

  *u = -2.0f * asinf(asinArg);
  *t = apaCoordWrapToPi(polarTheta + (0.5f * (*u)) + APA_PI);
  *v = apaCoordWrapToPi(goalYaw - (*t) + (*u));

  if (*t < 0.0f && *t > (-APA_MATH_EPSILON)) {
    *t = 0.0f;
  }
  if (*u > 0.0f && *u < APA_MATH_EPSILON) {
    *u = 0.0f;
  }

  if (*t < 0.0f) {
    return false;
  }
  if (*u > 0.0f) {
    return false;
  }
  return true;
}

static inline void apaHybridAStarTryRecordRsCandidate(
    const ApaRsVisitor_t *visitor, Integer patternIndex, Real radius, Real t, Real u, Real v) {
  ApaRsPath_t candidateRsPath;

  if (apaHybridAStarRsLoadScaledPath(patternIndex, radius, t, u, v, &candidateRsPath)) {
    visitor->Visit(&candidateRsPath, visitor->Context);
  }
}

static inline void apaHybridAStarTryRecordRsCandidate4(const ApaRsVisitor_t *visitor,
                                                       Integer patternIndex,
                                                       Real radius,
                                                       Real t,
                                                       Real u,
                                                       Real v,
                                                       Real w) {
  ApaRsPath_t candidateRsPath;

  if (apaHybridAStarRsLoadScaledPath4(patternIndex, radius, t, u, v, w, &candidateRsPath)) {
    visitor->Visit(&candidateRsPath, visitor->Context);
  }
}

static inline void apaHybridAStarTryRecordRsCandidate5(const ApaRsVisitor_t *visitor,
                                                       Integer patternIndex,
                                                       Real radius,
                                                       Real t,
                                                       Real u,
                                                       Real v,
                                                       Real w,
                                                       Real x) {
  ApaRsPath_t candidateRsPath;

  if (apaRsPathInitWithLengths(&candidateRsPath,
                               patternIndex,
                               t * radius,
                               u * radius,
                               v * radius,
                               w * radius,
                               x * radius)) {
    visitor->Visit(&candidateRsPath, visitor->Context);
  }
}

static inline void apaHybridAStarTryRsCccFamily(
    const ApaRsVisitor_t *visitor, Real goalX, Real goalY, Real goalYaw, Real radius) {
  Real t;
  Real u;
  Real v;
  Real backX;
  Real backY;

  if (apaHybridAStarTryRsLrlCore(goalX, goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate(visitor, APA_RS_PATTERN_LRL, radius, t, u, v);
  }
  if (apaHybridAStarTryRsLrlCore(-goalX, goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate(visitor, APA_RS_PATTERN_LRL, radius, -t, -u, -v);
  }
  if (apaHybridAStarTryRsLrlCore(goalX, -goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate(visitor, APA_RS_PATTERN_RLR, radius, t, u, v);
  }
  if (apaHybridAStarTryRsLrlCore(-goalX, -goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate(visitor, APA_RS_PATTERN_RLR, radius, -t, -u, -v);
  }

  backX = (goalX * cosf(goalYaw)) + (goalY * sinf(goalYaw));
  backY = (goalX * sinf(goalYaw)) - (goalY * cosf(goalYaw));

  if (apaHybridAStarTryRsLrlCore(backX, backY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate(visitor, APA_RS_PATTERN_LRL, radius, v, u, t);
  }
  if (apaHybridAStarTryRsLrlCore(-backX, backY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate(visitor, APA_RS_PATTERN_LRL, radius, -v, -u, -t);
  }
  if (apaHybridAStarTryRsLrlCore(backX, -backY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate(visitor, APA_RS_PATTERN_RLR, radius, v, u, t);
  }
  if (apaHybridAStarTryRsLrlCore(-backX, -backY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate(visitor, APA_RS_PATTERN_RLR, radius, -v, -u, -t);
  }
}

static inline void
apaHybridAStarCalcTauOmega(Real u, Real v, Real xi, Real eta, Real phi, Real *tau, Real *omega) {
  Real delta;
  Real coeffA;
  Real coeffB;
  Real t1;
  Real t2;

  delta = apaCoordWrapToPi(u - v);
  coeffA = sinf(u) - sinf(delta);
  coeffB = cosf(u) - cosf(delta) - 1.0f;
  t1 = atan2f((eta * coeffA) - (xi * coeffB), (xi * coeffA) + (eta * coeffB));
  t2 = (2.0f * (cosf(delta) - cosf(v) - cosf(u))) + 3.0f;
  if (t2 < 0.0f) {
    *tau = apaCoordWrapToPi(t1 + APA_PI);
  } else {
    *tau = apaCoordWrapToPi(t1);
  }
  *omega = apaCoordWrapToPi((*tau) - u + v - phi);
}

static inline Boolean
apaHybridAStarTryRsLrlrNegCore(Real goalX, Real goalY, Real goalYaw, Real *t, Real *u, Real *v) {
  Real xi;
  Real eta;
  Real rho;

  xi = goalX + sinf(goalYaw);
  eta = goalY - 1.0f - cosf(goalYaw);
  rho = 0.25f * (2.0f + sqrtf((xi * xi) + (eta * eta)));

  if (rho > 1.0f) {
    if (rho > (1.0f + APA_MATH_EPSILON)) {
      return false;
    }
    rho = 1.0f;
  }

  *u = acosf(rho);
  apaHybridAStarCalcTauOmega(*u, -(*u), xi, eta, goalYaw, t, v);

  if (*t < 0.0f && *t > (-APA_MATH_EPSILON)) {
    *t = 0.0f;
  }
  if (*v > 0.0f && *v < APA_MATH_EPSILON) {
    *v = 0.0f;
  }

  if (*t < 0.0f) {
    return false;
  }
  if (*v > 0.0f) {
    return false;
  }
  return true;
}

static inline Boolean
apaHybridAStarTryRsLrlrPosCore(Real goalX, Real goalY, Real goalYaw, Real *t, Real *u, Real *v) {
  Real xi;
  Real eta;
  Real rho;

  xi = goalX + sinf(goalYaw);
  eta = goalY - 1.0f - cosf(goalYaw);
  rho = (20.0f - (xi * xi) - (eta * eta)) / 16.0f;

  if (rho < 0.0f || rho > 1.0f) {
    if (rho < 0.0f && rho > (-APA_MATH_EPSILON)) {
      rho = 0.0f;
    } else if (rho > 1.0f && rho < (1.0f + APA_MATH_EPSILON)) {
      rho = 1.0f;
    } else {
      return false;
    }
  }

  *u = -acosf(rho);
  if (*u < (-0.5f * APA_PI)) {
    if (*u < ((-0.5f * APA_PI) - APA_MATH_EPSILON)) {
      return false;
    }
    *u = -0.5f * APA_PI;
  }

  apaHybridAStarCalcTauOmega(*u, *u, xi, eta, goalYaw, t, v);
  if (*t < 0.0f && *t > (-APA_MATH_EPSILON)) {
    *t = 0.0f;
  }
  if (*v < 0.0f && *v > (-APA_MATH_EPSILON)) {
    *v = 0.0f;
  }

  if (*t < 0.0f) {
    return false;
  }
  if (*v < 0.0f) {
    return false;
  }
  return true;
}

static inline void apaHybridAStarTryRsCcccFamily(
    const ApaRsVisitor_t *visitor, Real goalX, Real goalY, Real goalYaw, Real radius) {
  Real t;
  Real u;
  Real v;

  if (apaHybridAStarTryRsLrlrNegCore(goalX, goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LRLR, radius, t, u, -u, v);
  }
  if (apaHybridAStarTryRsLrlrNegCore(-goalX, goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LRLR, radius, -t, -u, u, -v);
  }
  if (apaHybridAStarTryRsLrlrNegCore(goalX, -goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RLRL, radius, t, u, -u, v);
  }
  if (apaHybridAStarTryRsLrlrNegCore(-goalX, -goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RLRL, radius, -t, -u, u, -v);
  }

  if (apaHybridAStarTryRsLrlrPosCore(goalX, goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LRLR, radius, t, u, u, v);
  }
  if (apaHybridAStarTryRsLrlrPosCore(-goalX, goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LRLR, radius, -t, -u, -u, -v);
  }
  if (apaHybridAStarTryRsLrlrPosCore(goalX, -goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RLRL, radius, t, u, u, v);
  }
  if (apaHybridAStarTryRsLrlrPosCore(-goalX, -goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RLRL, radius, -t, -u, -u, -v);
  }
}

static inline Boolean
apaHybridAStarTryRsLrsrCore(Real goalX, Real goalY, Real goalYaw, Real *t, Real *u, Real *v) {
  Real xi;
  Real eta;
  Real rho;
  Real theta;

  xi = goalX + sinf(goalYaw);
  eta = goalY - 1.0f - cosf(goalYaw);
  rho = sqrtf((xi * xi) + (eta * eta));
  theta = atan2f(xi, -eta);

  if (rho < 2.0f) {
    if (rho < (2.0f - APA_MATH_EPSILON)) {
      return false;
    }
    rho = 2.0f;
  }

  *t = theta;
  *u = 2.0f - rho;
  *v = apaCoordWrapToPi((*t) + (0.5f * APA_PI) - goalYaw);

  if (*t < 0.0f && *t > (-APA_MATH_EPSILON)) {
    *t = 0.0f;
  }
  if (*u > 0.0f && *u < APA_MATH_EPSILON) {
    *u = 0.0f;
  }
  if (*v > 0.0f && *v < APA_MATH_EPSILON) {
    *v = 0.0f;
  }

  if (*t < 0.0f) {
    return false;
  }
  if (*u > 0.0f) {
    return false;
  }
  if (*v > 0.0f) {
    return false;
  }
  return true;
}

static inline Boolean
apaHybridAStarTryRsLrslCore(Real goalX, Real goalY, Real goalYaw, Real *t, Real *u, Real *v) {
  Real xi;
  Real eta;
  Real rho;
  Real theta;
  Real radiusTmp;

  xi = goalX - sinf(goalYaw);
  eta = goalY - 1.0f + cosf(goalYaw);
  rho = sqrtf((xi * xi) + (eta * eta));
  theta = atan2f(eta, xi);

  if (rho < 2.0f) {
    if (rho < (2.0f - APA_MATH_EPSILON)) {
      return false;
    }
    rho = 2.0f;
  }

  radiusTmp = sqrtf((rho * rho) - 4.0f);
  *u = 2.0f - radiusTmp;
  *t = apaCoordWrapToPi(theta + atan2f(radiusTmp, -2.0f));
  *v = apaCoordWrapToPi(goalYaw - (0.5f * APA_PI) - (*t));

  if (*t < 0.0f && *t > (-APA_MATH_EPSILON)) {
    *t = 0.0f;
  }
  if (*u > 0.0f && *u < APA_MATH_EPSILON) {
    *u = 0.0f;
  }
  if (*v > 0.0f && *v < APA_MATH_EPSILON) {
    *v = 0.0f;
  }

  if (*t < 0.0f) {
    return false;
  }
  if (*u > 0.0f) {
    return false;
  }
  if (*v > 0.0f) {
    return false;
  }
  return true;
}

static inline void apaHybridAStarTryRsCcscFamily(
    const ApaRsVisitor_t *visitor, Real goalX, Real goalY, Real goalYaw, Real radius) {
  Real t;
  Real u;
  Real v;
  Real backX;
  Real backY;
  Real halfPi;

  halfPi = 0.5f * APA_PI;

  if (apaHybridAStarTryRsLrslCore(goalX, goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LRSL, radius, t, -halfPi, u, v);
  }
  if (apaHybridAStarTryRsLrslCore(-goalX, goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LRSL, radius, -t, halfPi, -u, -v);
  }
  if (apaHybridAStarTryRsLrslCore(goalX, -goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RLSR, radius, t, -halfPi, u, v);
  }
  if (apaHybridAStarTryRsLrslCore(-goalX, -goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RLSR, radius, -t, halfPi, -u, -v);
  }

  if (apaHybridAStarTryRsLrsrCore(goalX, goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LRSR, radius, t, -halfPi, u, v);
  }
  if (apaHybridAStarTryRsLrsrCore(-goalX, goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LRSR, radius, -t, halfPi, -u, -v);
  }
  if (apaHybridAStarTryRsLrsrCore(goalX, -goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RLSL, radius, t, -halfPi, u, v);
  }
  if (apaHybridAStarTryRsLrsrCore(-goalX, -goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RLSL, radius, -t, halfPi, -u, -v);
  }

  backX = (goalX * cosf(goalYaw)) + (goalY * sinf(goalYaw));
  backY = (goalX * sinf(goalYaw)) - (goalY * cosf(goalYaw));

  if (apaHybridAStarTryRsLrslCore(backX, backY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LSRL, radius, v, u, -halfPi, t);
  }
  if (apaHybridAStarTryRsLrslCore(-backX, backY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LSRL, radius, -v, -u, halfPi, -t);
  }
  if (apaHybridAStarTryRsLrslCore(backX, -backY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RSLR, radius, v, u, -halfPi, t);
  }
  if (apaHybridAStarTryRsLrslCore(-backX, -backY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RSLR, radius, -v, -u, halfPi, -t);
  }

  if (apaHybridAStarTryRsLrsrCore(backX, backY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RSRL, radius, v, u, -halfPi, t);
  }
  if (apaHybridAStarTryRsLrsrCore(-backX, backY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_RSRL, radius, -v, -u, halfPi, -t);
  }
  if (apaHybridAStarTryRsLrsrCore(backX, -backY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LSLR, radius, v, u, -halfPi, t);
  }
  if (apaHybridAStarTryRsLrsrCore(-backX, -backY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate4(visitor, APA_RS_PATTERN_LSLR, radius, -v, -u, halfPi, -t);
  }
}

static inline Boolean
apaHybridAStarTryRsLrslrCore(Real goalX, Real goalY, Real goalYaw, Real *t, Real *u, Real *v) {
  Real xi;
  Real eta;
  Real rho;
  Real theta;
  Real inner;

  xi = goalX + sinf(goalYaw);
  eta = goalY - 1.0f - cosf(goalYaw);
  rho = sqrtf((xi * xi) + (eta * eta));
  theta = atan2f(eta, xi);

  if (rho < 2.0f) {
    if (rho < (2.0f - APA_MATH_EPSILON)) {
      return false;
    }
    rho = 2.0f;
  }

  inner = (rho * rho) - 4.0f;
  if (inner < 0.0f) {
    if (inner < (-APA_MATH_EPSILON)) {
      return false;
    }
    inner = 0.0f;
  }

  *u = 4.0f - sqrtf(inner);
  if (*u > 0.0f) {
    if (*u > APA_MATH_EPSILON) {
      return false;
    }
    *u = 0.0f;
  }

  (void)theta;
  *t = apaCoordWrapToPi(
      atan2f(((4.0f - (*u)) * xi) - (2.0f * eta), (-2.0f * xi) + (((*u) - 4.0f) * eta)));
  *v = apaCoordWrapToPi((*t) - goalYaw);

  if (*t < 0.0f && *t > (-APA_MATH_EPSILON)) {
    *t = 0.0f;
  }
  if (*v < 0.0f && *v > (-APA_MATH_EPSILON)) {
    *v = 0.0f;
  }

  if (*t < 0.0f) {
    return false;
  }
  if (*v < 0.0f) {
    return false;
  }
  return true;
}

static inline void apaHybridAStarTryRsCcsccFamily(
    const ApaRsVisitor_t *visitor, Real goalX, Real goalY, Real goalYaw, Real radius) {
  Real t;
  Real u;
  Real v;
  Real halfPi;

  halfPi = 0.5f * APA_PI;

  if (apaHybridAStarTryRsLrslrCore(goalX, goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate5(
        visitor, APA_RS_PATTERN_LRSLR, radius, t, -halfPi, u, -halfPi, v);
  }
  if (apaHybridAStarTryRsLrslrCore(-goalX, goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate5(
        visitor, APA_RS_PATTERN_LRSLR, radius, -t, halfPi, -u, halfPi, -v);
  }
  if (apaHybridAStarTryRsLrslrCore(goalX, -goalY, -goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate5(
        visitor, APA_RS_PATTERN_RLSRL, radius, t, -halfPi, u, -halfPi, v);
  }
  if (apaHybridAStarTryRsLrslrCore(-goalX, -goalY, goalYaw, &t, &u, &v)) {
    apaHybridAStarTryRecordRsCandidate5(
        visitor, APA_RS_PATTERN_RLSRL, radius, -t, halfPi, -u, halfPi, -v);
  }
}

static inline void apaHybridAStarAdvanceRsPose(const Real currentPose[3],
                                               Integer segType,
                                               Real radius,
                                               Real travelDistance,
                                               Real nextPose[3]) {
  Real curvature;

  curvature = 0.0f;
  if (segType == APA_RS_SEG_LEFT) {
    curvature = 1.0f / radius;
  }
  if (segType == APA_RS_SEG_RIGHT) {
    curvature = -1.0f / radius;
  }

  if (apaMathAbs(curvature) < APA_MATH_EPSILON) {
    apaCoordAdvanceStraightPose(currentPose, travelDistance, nextPose);
  } else {
    Real nextYaw;
    Real invCurvature;

    nextYaw = apaCoordWrapToPi(currentPose[2] + (travelDistance * curvature));
    invCurvature = 1.0f / curvature;
    nextPose[0] = currentPose[0] + ((sinf(nextYaw) - sinf(currentPose[2])) * invCurvature);
    nextPose[1] = currentPose[1] - ((cosf(nextYaw) - cosf(currentPose[2])) * invCurvature);
    nextPose[2] = nextYaw;
  }
}

static inline void
apaHybridAStarNodeSetPose(ApaHybridNode_t *node, const Real pose[3], Real travelDistance) {
  apaNodeInit(node);
  node->X = pose[0];
  node->Y = pose[1];
  node->Theta = pose[2];
  node->D = travelDistance;
}

static inline Boolean
apaHybridAStarSampleRsPath(ApaHybridAStar_t *planner, const ApaRsPath_t *rsPath, ApaPath_t *path) {
  const ApaPlannerConfig_t *plannerConfig;
  const ApaPlannerVehConfig_t *vehConfig;
  Real currentPose[3];
  Real nextPose[3];
  Real pathStep;
  Real totalLength;
  Integer segIndex;
  Integer pointIndex;
  ApaHybridNode_t fromNode;
  ApaHybridNode_t toNode;
  Real posTol;
  Real yawTol;
  Real goalErr[3];
  Boolean goalMatched;
  Real sampledDistance;
  const Boolean compactControls = apaHybridAStarParkingSearch(planner);

  if (planner->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return false;
  }

  plannerConfig = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  vehConfig = &plannerConfig->Veh;
  pathStep = plannerConfig->Map.CollisionDetectStepLength;
  if (pathStep <= APA_MATH_EPSILON) {
    pathStep = plannerConfig->Map.MotionResolution;
  }
  if (pathStep <= APA_MATH_EPSILON) {
    pathStep = plannerConfig->Map.XyGridResolution;
  }
  if (pathStep <= APA_MATH_EPSILON) {
    return false;
  }

  totalLength = rsPath->TotalLength;
  if (totalLength <= APA_MATH_EPSILON) {
    return false;
  }
  if (plannerConfig->Conf.PathLenMax > 0.0f && totalLength > plannerConfig->Conf.PathLenMax) {
    return false;
  }

  apaPathInit(path);
  path->ValidFlag = true;
  path->ErrorType = APA_FAULT_NO_ERR;
  path->PathSource = APA_PATH_SOURCE_SEARCH;
  path->PathType = APA_PATH_TYPE_ALL;
  path->CollisionFlag = APA_COLLISION_RISK_NOT_EXIST;
  currentPose[0] = planner->CurrentNode.X;
  currentPose[1] = planner->CurrentNode.Y;
  currentPose[2] = planner->CurrentNode.Theta;
  path->X[0] = currentPose[0];
  path->Y[0] = currentPose[1];
  path->Tht[0] = currentPose[2];
  path->OdoStep[0] = 0.0f;
  path->PathN = 1;
  sampledDistance = 0.0f;

  segIndex = 0;
  while (segIndex < APA_RS_MAX_SEGMENTS) {
    Integer segType;
    Real segLength;
    Real remaining;
    Real directionSign;
    Boolean segmentUsed;

    segType = rsPath->Type[segIndex];
    segLength = rsPath->Lengths[segIndex];
    segmentUsed = (segType != APA_RS_SEG_NONE);
    if (segmentUsed) {
      segmentUsed = !apaMathEqual(segLength, 0.0f);
    }

    if (segmentUsed) {
      const Real segmentStart[3] = {currentPose[0], currentPose[1], currentPose[2]};
      Real segmentTravel = 0.0f;
      remaining = apaMathAbs(segLength);
      directionSign = 1.0f;
      if (segLength < 0.0f) {
        directionSign = -1.0f;
      }

      while (remaining > APA_MATH_EPSILON) {
        Real stepAbs;
        Real stepSigned;

        stepAbs = pathStep;
        if (stepAbs > remaining) {
          stepAbs = remaining;
        }
        stepSigned = stepAbs * directionSign;
        segmentTravel += stepAbs;
        apaHybridAStarAdvanceRsPose(compactControls ? segmentStart : currentPose,
                                    segType,
                                    vehConfig->MinCircle,
                                    compactControls ? segmentTravel * directionSign : stepSigned,
                                    nextPose);
        apaHybridAStarNodeSetPose(&fromNode, currentPose, stepSigned);
        apaHybridAStarNodeSetPose(&toNode, nextPose, stepSigned);
        if (segType == APA_RS_SEG_LEFT || segType == APA_RS_SEG_RIGHT) {
          toNode.Delta = atanf(vehConfig->WB / vehConfig->MinCircle);
          if (segType == APA_RS_SEG_RIGHT)
            toNode.Delta = -toNode.Delta;
        }
        if (apaHybridAStarShouldRejectConnectorByCollision(planner, &fromNode, &toNode)) {
          return false;
        }

        if (!compactControls || remaining <= stepAbs + APA_MATH_EPSILON) {
          if (path->PathN >= APA_PATH_MAX_POINTS)
            return false;
          pointIndex = path->PathN;
          path->OdoStep[pointIndex - 1] = compactControls ? segLength : stepSigned;
          path->X[pointIndex] = nextPose[0];
          path->Y[pointIndex] = nextPose[1];
          path->Tht[pointIndex] = nextPose[2];
          path->OdoStep[pointIndex] = 0.0f;
          path->PathN = pointIndex + 1;
        }

        currentPose[0] = nextPose[0];
        currentPose[1] = nextPose[1];
        currentPose[2] = nextPose[2];
        remaining = remaining - stepAbs;
        sampledDistance = sampledDistance + stepAbs;
      }
    }

    segIndex = segIndex + 1;
  }

  if (path->PathN < 2) {
    return false;
  }

  posTol = plannerConfig->Map.XyGridResolution;
  if (posTol < pathStep) {
    posTol = pathStep;
  }
  yawTol = plannerConfig->Map.YawGridResolution;
  if (yawTol < APA_MATH_EPSILON) {
    yawTol = APA_MATH_EPSILON;
  }
  apaCoordGroundToVehPose(planner->GoalPose, currentPose, goalErr);
  goalMatched = (fabsf(goalErr[0]) <= posTol) && (fabsf(goalErr[1]) <= posTol) &&
                (fabsf(goalErr[2]) <= yawTol);
  if (!goalMatched) {
    return false;
  }

  path->SegNum = apaPathCalcSegNum(path);
  path->SegNumFlag = true;
  return true;
}

/* RS heuristic and connections share families; candidates require full admission. */
static inline void
apaHybridAStarEnumerateRs(const Real goal[3], Real radius, const ApaRsVisitor_t *visitor) {
  typedef Boolean (*DubinsBuilder)(Real, Real, Real, Real, ApaRsPath_t *);
  const DubinsBuilder builders[4] = {apaHybridAStarTryDubinsLsl,
                                     apaHybridAStarTryDubinsRsr,
                                     apaHybridAStarTryDubinsLsr,
                                     apaHybridAStarTryDubinsRsl};
  ApaRsPath_t path;
  if (fabsf(goal[1]) <= 1.0e-4f && fabsf(goal[2]) <= 1.0e-4f) {
    apaHybridAStarRsPathInitStraight(&path, goal[0]);
    visitor->Visit(&path, visitor->Context);
  }
  for (Integer reverse = 0; reverse < 2; ++reverse) {
    const Real sign = reverse ? -1.0f : 1.0f;
    for (Integer i = 0; i < 4; ++i) {
      if (builders[i](sign * goal[0] / radius, sign * goal[1] / radius, goal[2], radius, &path)) {
        if (reverse)
          apaHybridAStarMirrorRsPathForReverse(&path);
        visitor->Visit(&path, visitor->Context);
      }
    }
    if (!reverse) {

      apaHybridAStarTryRsCccFamily(visitor, goal[0] / radius, goal[1] / radius, goal[2], radius);
      apaHybridAStarTryRsCcccFamily(visitor, goal[0] / radius, goal[1] / radius, goal[2], radius);
      apaHybridAStarTryRsCcscFamily(visitor, goal[0] / radius, goal[1] / radius, goal[2], radius);
      apaHybridAStarTryRsCcsccFamily(visitor, goal[0] / radius, goal[1] / radius, goal[2], radius);
    }
  }
}

static inline void apaHybridAStarRecordRsDistance(const ApaRsPath_t *path, void *context) {
  Real *distance = (Real *)context;
  if (path->TotalLength < *distance)
    *distance = path->TotalLength;
}

static inline Real apaHybridAStarRsDistance(const ApaHybridAStar_t *planner,
                                            const ApaHybridNode_t *node) {
  const Real pose[3] = {node->X, node->Y, node->Theta};
  Real goal[3], distance = 1.0e20f;
  apaCoordGroundToVehPose(pose, planner->GoalPose, goal);
  ApaRsVisitor_t visitor = {apaHybridAStarRecordRsDistance, &distance};
  apaHybridAStarEnumerateRs(goal, planner->PlannerGlobal->Planner.Veh.MinCircle, &visitor);
  return distance < 1.0e20f ? distance : hypotf(goal[0], goal[1]);
}

static inline void apaHybridAStarCheckRsCandidate(const ApaRsPath_t *rs, void *context) {
  ApaHybridAStar_t *planner = (ApaHybridAStar_t *)context;
  if (apaHybridAStarParkingSearch(planner) && planner->HasAcceptedCandidate &&
      !planner->PlannerGlobal->Planner.Conf.UseMoreCalUnits)
    return;
  ApaPath_t path;
  if (apaHybridAStarSampleRsPath(planner, rs, &path))
    (void)apaHybridAStarCheckGoalCandidate(planner, &planner->CurrentNode, rs, &path);
}

static inline Boolean apaHybridAStarTryAnalyticConnect(ApaHybridAStar_t *planner) {
  if (!planner->PlannerGlobal || planner->PlannerGlobal->Planner.Veh.MinCircle <= APA_MATH_EPSILON)
    return false;
  const Real pose[3] = {planner->CurrentNode.X, planner->CurrentNode.Y, planner->CurrentNode.Theta};
  Real goal[3];
  apaCoordGroundToVehPose(pose, planner->GoalPose, goal);
  const Integer previous = planner->AcceptedCandidates;
  ApaRsVisitor_t visitor = {apaHybridAStarCheckRsCandidate, planner};
  apaHybridAStarEnumerateRs(goal, planner->PlannerGlobal->Planner.Veh.MinCircle, &visitor);
  return planner->AcceptedCandidates > previous;
}

static inline Boolean apaHybridAStarNodeWithinGoalWindow(const ApaHybridAStar_t *planner,
                                                         const ApaHybridNode_t *node) {
  const ApaPlannerConfConfig_t *confConfig;
  Real nodePose[3];
  Real localErr[3];
  Boolean longInRange;
  Boolean latInRange;
  Boolean yawInRange;

  if (planner->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return false;
  }

  confConfig = apaPlannerGlobalGetConf(planner->PlannerGlobal);
  nodePose[0] = node->X;
  nodePose[1] = node->Y;
  nodePose[2] = node->Theta;
  apaCoordGroundToVehPose(planner->GoalPose, nodePose, localErr);

  longInRange = (localErr[0] >= confConfig->LongAllowableErrMin) &&
                (localErr[0] <= confConfig->LongAllowableErrMax);
  latInRange = (localErr[1] >= confConfig->LatAllowableErrMin) &&
               (localErr[1] <= confConfig->LatAllowableErrMax);
  yawInRange = (localErr[2] >= confConfig->YawAllowableErrMin) &&
               (localErr[2] <= confConfig->YawAllowableErrMax);
  return longInRange && latInRange && yawInRange;
}

static inline Boolean apaHybridAStarTryDirectGoalLink(ApaHybridAStar_t *planner) {
  const ApaPlannerConfig_t *plannerConfig;
  Real currentPose[3];
  Real goalLocal[3];
  Real heuristicCost;
  Real trajCost;
  Real totalCost;
  Real travelDistance;
  ApaHybridNode_t goalChild;

  if (planner->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return false;
  }

  plannerConfig = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  currentPose[0] = planner->CurrentNode.X;
  currentPose[1] = planner->CurrentNode.Y;
  currentPose[2] = planner->CurrentNode.Theta;
  apaCoordGroundToVehPose(currentPose, planner->GoalPose, goalLocal);
  if (fabsf(goalLocal[1]) > 1.0e-4f) {
    return false;
  }
  if (fabsf(goalLocal[2]) > 1.0e-4f) {
    return false;
  }
  if (fabsf(goalLocal[0]) > plannerConfig->Map.MotionStepLength) {
    return false;
  }

  travelDistance = goalLocal[0];

  trajCost = apaHybridAStarCalcTrajCost(planner, &planner->CurrentNode, travelDistance, 0.0f);
  if (!apaHybridAStarNodeInitFromPose(
          &planner->GridSpec,
          planner->GoalPose,
          travelDistance,
          0.0f,
          planner->CurrentNodeIndex,
          apaHybridAStarNextSegmentCount(&planner->CurrentNode, travelDistance),
          trajCost,
          0.0f,
          0.0f,
          &goalChild)) {
    return false;
  }
  if (apaHybridAStarShouldRejectConnectorByCollision(planner, &planner->CurrentNode, &goalChild)) {
    return false;
  }
  if (!apaHybridAStarCheckGoalCandidate(
          planner, &goalChild, (const ApaRsPath_t *)0, (const ApaPath_t *)0))
    return false;

  heuristicCost = apaHybridAStarCalcHeuristic(planner, &goalChild);
  totalCost = apaTotalCost(
      trajCost, heuristicCost, planner->HeuristicContext.HCost, planner->HeuristicContext.UseGCost);
  goalChild.HeuristicCost = heuristicCost;
  goalChild.TotalCost = totalCost;

  return true;
}

static inline Boolean apaHybridAStarTryPushOpenNode(ApaHybridAStar_t *planner,
                                                    const ApaHybridNode_t *node) {
  ApaHybridNode_t candidateNode;
  Integer closeIndex;
  Integer openIndex;
  Integer addReason;

  candidateNode = *node;
  candidateNode.MotionClass = apaHybridAStarMotionClass(planner, &candidateNode);
  apaHybridAStarRefreshOpenSetAfterReversePhase(planner, &candidateNode);

  if (apaHashMapContainsState(
          &planner->CloseNodes, &planner->NodePool, &candidateNode, &closeIndex)) {
    if (!apaHybridAStarParkingSearch(planner) ||
        candidateNode.TrajCost >= planner->NodePool.Nodes[closeIndex].TrajCost - 1.0e-5f) {
      ++planner->ClosedDominated;
      return false;
    }

    apaHashMapRemove(&planner->CloseNodes, &planner->NodePool, closeIndex);
    ++planner->ReopenedStates;
  }

  if (apaHashMapContainsState(
          &planner->OpenNodes, &planner->NodePool, &candidateNode, &openIndex)) {
    ApaHybridNode_t oldNode;
    Boolean replaceOk;
    Boolean pushOk;

    oldNode = planner->NodePool.Nodes[openIndex];
    if ((apaHybridAStarParkingSearch(planner) ? candidateNode.TrajCost >= oldNode.TrajCost - 1.0e-5f
                                              : candidateNode.TotalCost >= oldNode.TotalCost)) {
      ++planner->OpenDominated;
      return false;
    }

    planner->NodePool.Nodes[openIndex] = candidateNode;
    apaNodePoolSetState(&planner->NodePool, openIndex, (UInteger8)APA_NODE_STATE_OPEN);
    replaceOk = apaPqReplaceNode(&planner->OpenQueue, openIndex, candidateNode.TotalCost);
    if (!replaceOk) {
      pushOk = apaPqPush(&planner->OpenQueue, openIndex, candidateNode.TotalCost);
      if (!pushOk) {
        planner->Fault = APA_HYBRID_FAULT_HYBRID_OPEN_RUN_OUT;
        planner->RunState = APA_HYBRID_STATE_FAILED;
        return false;
      }
    }

    return true;
  }

  if (planner->PlannerGlobal &&
      planner->NodePool.ValidNodeCnt >=
          apaPlannerGlobalGetConf(planner->PlannerGlobal)->PlanningCalPowerMaxPerSlot) {
    planner->Fault = APA_HYBRID_FAULT_EXCEED_CAL_POWER;
    planner->RunState = APA_HYBRID_STATE_FAILED;
    return false;
  }
  addReason = apaHashMapAdd(&planner->OpenNodes,
                            &planner->NodePool,
                            &candidateNode,
                            (UInteger8)APA_NODE_STATE_OPEN,
                            &openIndex);
  if (addReason != 0) {
    planner->Fault = APA_HYBRID_FAULT_HYBRID_OPEN_RUN_OUT;
    planner->RunState = APA_HYBRID_STATE_FAILED;
    return false;
  }
  if (!apaPqPush(&planner->OpenQueue, openIndex, candidateNode.TotalCost)) {
    apaHashMapRemove(&planner->OpenNodes, &planner->NodePool, openIndex);
    apaNodePoolRemove(&planner->NodePool, openIndex);
    planner->Fault = APA_HYBRID_FAULT_HYBRID_OPEN_RUN_OUT;
    planner->RunState = APA_HYBRID_STATE_FAILED;
    return false;
  }

  return true;
}

static inline Boolean apaHybridAStarSeedStart(ApaHybridAStar_t *planner) {
  Boolean pushOk;

  apaHybridAStarResetSearch(planner);
  apaHybridAStarConfigureFirstSegmentBounds(planner);
  if (planner->SlotView.SlotClass == APA_SLOT_CLASS_VERTICAL &&
      (planner->SlotView.SlotSubStatus == APA_SLOT_SUB_END_FIRST ||
       planner->SlotView.SlotSubStatus == APA_SLOT_SUB_END_SECOND)) {

    planner->HeuristicContext.ReverseTheta =
        apaHybridAStarWrapPositive(planner->StartPose[2] + APA_PI);
    planner->HeuristicContext.ReverseFlag = 0U;
    apaHybridAStarSyncHeuristicContextToPlanner(planner);
  }
  planner->StartNode.HeuristicCost = apaHybridAStarCalcHeuristic(planner, &planner->StartNode);
  planner->StartNode.TotalCost = apaTotalCost(planner->StartNode.TrajCost,
                                              planner->StartNode.HeuristicCost,
                                              planner->HeuristicContext.HCost,
                                              planner->HeuristicContext.UseGCost);

  pushOk = apaHybridAStarTryPushOpenNode(planner, &planner->StartNode);
  if (!pushOk) {
    return false;
  }

  planner->Seeded = true;
  planner->RunState = APA_HYBRID_STATE_RUNNING;
  return true;
}

static inline Boolean apaHybridAStarPopCurrent(ApaHybridAStar_t *planner) {
  Boolean popOk;

  if (apaPqEmpty(&planner->OpenQueue)) {
    planner->Fault = APA_HYBRID_FAULT_HYBRID_OPEN_EMPTY;
    planner->RunState = APA_HYBRID_STATE_FAILED;
    return false;
  }

  popOk = true;
  while (popOk) {
    Integer queueNodeIndex;
    Integer addReason;
    Boolean queueNodeIndexValid;
    Boolean queueNodeOpen;

    popOk = apaPqPop(&planner->OpenQueue, &queueNodeIndex, (Real *)0);
    if (popOk) {
      queueNodeIndexValid = (queueNodeIndex >= 0);
      if (queueNodeIndexValid) {
        queueNodeIndexValid = (queueNodeIndex < APA_HASH_CAPACITY);
      }
      queueNodeOpen = false;
      if (queueNodeIndexValid) {
        queueNodeOpen = (apaNodePoolGetState(&planner->NodePool, queueNodeIndex) ==
                         (UInteger8)APA_NODE_STATE_OPEN);
      }
      if (queueNodeOpen) {
        planner->CurrentNode = planner->NodePool.Nodes[queueNodeIndex];
        apaHashMapRemove(&planner->OpenNodes, &planner->NodePool, queueNodeIndex);
        addReason = apaHashMapAddIndex(&planner->CloseNodes, &planner->NodePool, queueNodeIndex);
        if (addReason != 0) {
          planner->Fault = APA_HYBRID_FAULT_HYBRID_CLOSE_RUN_OUT;
          planner->RunState = APA_HYBRID_STATE_FAILED;
          return false;
        }
        apaNodePoolSetState(&planner->NodePool, queueNodeIndex, (UInteger8)APA_NODE_STATE_CLOSED);
        planner->CurrentNodeIndex = (Integer16)queueNodeIndex;

        return true;
      }
    }
  }

  planner->Fault = APA_HYBRID_FAULT_HYBRID_OPEN_EMPTY;
  planner->RunState = APA_HYBRID_STATE_FAILED;
  return false;
}

static inline Boolean apaHybridAStarShouldRejectByCollision(const ApaHybridAStar_t *planner,
                                                            const ApaHybridNode_t *fromNode,
                                                            const ApaHybridNode_t *toNode) {
  if (planner->CollisionCheck == (ApaHybridAStarCollisionCheck_f)0) {
    return false;
  }

  return planner->CollisionCheck(fromNode, toNode, planner->CollisionUserData);
}

static inline Boolean apaHybridAStarTryResolveParentNode(const ApaHybridAStar_t *planner,
                                                         const ApaHybridNode_t *node,
                                                         ApaHybridNode_t *parentNode) {
  Integer nodeIndex;

  nodeIndex = node->ParentNodeIndex;
  if (nodeIndex >= 0 && nodeIndex < APA_HASH_CAPACITY) {
    if (apaNodePoolGetState(&planner->NodePool, nodeIndex) == (UInteger8)APA_NODE_STATE_CLOSED) {
      *parentNode = planner->NodePool.Nodes[nodeIndex];
      return true;
    }
  }

  return false;
}

/* Native collision results are authoritative, without distance-window exemptions. */
static inline Boolean
apaHybridAStarShouldRejectPrimitiveByCollision(const ApaHybridAStar_t *planner,
                                               const ApaHybridNode_t *fromNode,
                                               const ApaHybridNode_t *toNode) {
  ApaVehicleCollisionCheckResult_t result;
  Real travel;
  if (!planner->UseDefaultCollisionBridge || planner->CollisionUserData == (void *)0) {
    return apaHybridAStarShouldRejectByCollision(planner, fromNode, toNode);
  }
  return apaHybridAStarCollisionCheckMotionDetailed(
      fromNode, toNode, planner->CollisionUserData, planner->CollisionDemand, &result, &travel);
}

static inline Boolean
apaHybridAStarShouldRejectConnectorByCollision(const ApaHybridAStar_t *planner,
                                               const ApaHybridNode_t *fromNode,
                                               const ApaHybridNode_t *toNode) {
  return apaHybridAStarShouldRejectPrimitiveByCollision(planner, fromNode, toNode);
}

/* Collision-trimmed prefixes retain the checked primitive, not just its endpoint. */
static inline Boolean apaHybridAStarPrimitivePrefixIsClear(const ApaHybridAStar_t *planner,
                                                           const Real start[3],
                                                           Real distance,
                                                           Real steer,
                                                           Real *firstBlockedDistance) {
  const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  const Real step = config->Map.CollisionDetectStepLength;
  if (!isfinite(step) || step <= APA_MATH_EPSILON)
    return false;
  const Integer count = (Integer)ceilf(fabsf(distance) / step);
  const Boolean checkFirstSegment =
      planner->FirstSegmentBoundsValid &&
      apaHybridAStarNextSegmentCount(&planner->CurrentNode, distance) == 0U;
  ApaHybridNode_t from = planner->CurrentNode, to = from;
  for (Integer i = 1; i <= count; ++i) {
    const Real travel = distance * (Real)i / (Real)count;
    Real pose[3];
    apaHybridAStarAdvancePose(&config->Veh, start, travel, steer, pose);
    to.X = pose[0];
    to.Y = pose[1];
    to.Theta = pose[2];
    to.D = distance / (Real)count;
    to.Delta = steer;
    ApaVehicleCollisionCheckResult_t firstSegmentResult;
    if (apaHybridAStarShouldRejectPrimitiveByCollision(planner, &from, &to) ||
        (checkFirstSegment &&
         apaVehicleCollisionCheckPose(
             &planner->FirstSegmentCollisionEnv, to.X, to.Y, to.Theta, &firstSegmentResult))) {
      *firstBlockedDistance = travel;
      return false;
    }
    from = to;
  }
  return count > 0;
}

static inline Boolean apaHybridAStarPushPrimitiveEnd(ApaHybridAStar_t *planner,
                                                     const Real start[3],
                                                     Real travel,
                                                     Real steer) {
  const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  Real pose[3];
  apaHybridAStarAdvancePose(&config->Veh, start, travel, steer, pose);
  const UInteger8 segments = apaHybridAStarNextSegmentCount(&planner->CurrentNode, travel);
  const Real cost = apaHybridAStarCalcTrajCost(planner, &planner->CurrentNode, travel, steer);
  ApaHybridNode_t child;
  if (!apaHybridAStarNodeInitFromPose(&planner->GridSpec,
                                      pose,
                                      travel,
                                      steer,
                                      planner->CurrentNodeIndex,
                                      segments,
                                      cost,
                                      0.0f,
                                      0.0f,
                                      &child))
    return false;
  child.MotionClass = apaHybridAStarMotionClass(planner, &child);
  if (apaNodeMatchesIndex(&child,
                          planner->CurrentNode.Xidx,
                          planner->CurrentNode.Yidx,
                          planner->CurrentNode.Yawidx) &&
      child.MotionClass == planner->CurrentNode.MotionClass)
    return false;
  child.HeuristicCost = apaHybridAStarCalcHeuristic(planner, &child);
  child.TotalCost = apaTotalCost(cost,
                                 child.HeuristicCost,
                                 planner->HeuristicContext.HCost,
                                 planner->HeuristicContext.UseGCost);
  return apaHybridAStarTryPushOpenNode(planner, &child);
}

static inline Boolean apaHybridAStarExpandOnePrimitive(ApaHybridAStar_t *planner,
                                                       Integer direction,
                                                       Integer steerIndex,
                                                       Integer steerCount) {
  if (!planner->PlannerGlobal) {
    planner->Fault = APA_HYBRID_FAULT_UNKNOWN_ERR;
    planner->RunState = APA_HYBRID_STATE_FAILED;
    return false;
  }
  const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  const Real steer = apaHybridAStarCalcSteerAngle(planner, steerIndex, steerCount);
  const Real start[3] = {
      planner->CurrentNode.X, planner->CurrentNode.Y, planner->CurrentNode.Theta};
  Real travel = config->Map.MotionStepLength * (Real)direction;
  Real blocked = 0.0f;
  if (!apaHybridAStarPrimitivePrefixIsClear(planner, start, travel, steer, &blocked)) {

    const Real rewind = fminf(config->Map.MotionResolution, config->Map.XyGridResolution);
    const Real minimum = 1.5f * config->Map.XyGridResolution;
    const Real detection = config->Map.CollisionDetectStepLength;
    Boolean found = false;
    if (!isfinite(rewind) || rewind <= APA_MATH_EPSILON || !isfinite(blocked))
      return false;
    const double sampleCount = floor((double)(detection + rewind + APA_MATH_EPSILON) / rewind);
    if (!isfinite(sampleCount) || sampleCount < 1.0 || sampleCount > INT32_MAX)
      return false;
    for (Integer sample = 0; sample < (Integer)sampleCount; ++sample) {
      const Real back = (Real)(sample + 1) * rewind;
      const Real prefix = fabsf(blocked) - back;
      if (prefix < minimum || prefix <= APA_MATH_EPSILON)
        break;
      Real ignored;
      const Real candidate = (Real)direction * prefix;
      if (apaHybridAStarPrimitivePrefixIsClear(planner, start, candidate, steer, &ignored)) {
        travel = candidate;
        found = true;
        ++planner->TruncatedPrimitives;
        break;
      }
    }
    if (!found)
      return false;
  }
  Boolean accepted = apaHybridAStarPushPrimitiveEnd(planner, start, travel, steer);
  return accepted;
}

/* Finite primitive lengths derive from MATLAB segment thresholds. Shorter
 * alternatives require separate tracking validation. */
static inline Boolean apaHybridAStarExpandParking(ApaHybridAStar_t *planner) {
  const ApaPlannerConfig_t *config = &planner->PlannerGlobal->Planner;
  const Real step = config->Conf.PathLenMin;
  const Real start[3] = {
      planner->CurrentNode.X, planner->CurrentNode.Y, planner->CurrentNode.Theta};
  Boolean accepted = false;
  for (Integer direction = -1; direction <= 1; direction += 2) {
    for (Integer turn = -2; turn <= 2; ++turn) {
      const Real steer = atanf(config->Veh.WB * (Real)turn / (2.0f * config->Veh.MinCircle));
      for (Integer scale = 1; scale <= 8; scale *= 2) {
        const Real travel = (Real)direction * step * (Real)scale;
        Real blocked;
        ++planner->MotionCandidates;
        if (!apaHybridAStarPrimitivePrefixIsClear(planner, start, travel, steer, &blocked))
          break;
        accepted = apaHybridAStarPushPrimitiveEnd(planner, start, travel, steer) || accepted;
        if (planner->RunState == APA_HYBRID_STATE_FAILED)
          return accepted;
      }
    }
  }
  return accepted;
}

static inline Boolean apaHybridAStarExpandCurrent(ApaHybridAStar_t *planner) {
  Integer directionOrder[2];
  Integer steerCount;
  Integer directionIndex;
  Integer steerIndex;
  Boolean anyAccepted;

  if (apaHybridAStarParkingSearch(planner))
    return apaHybridAStarExpandParking(planner);

  directionOrder[0] = 1;
  directionOrder[1] = -1;
  if (planner->ReversePlanFlag) {
    directionOrder[0] = -1;
    directionOrder[1] = 1;
  }

  steerCount = apaHybridAStarGetSteerSampleCount(planner);
  anyAccepted = false;

  for (directionIndex = 0; directionIndex < 2; directionIndex = directionIndex + 1) {
    if (planner->PlannerGlobal != (ApaPlannerGlobalState_t *)0) {
      apaHybridAStarChooseStepLength(planner, directionOrder[directionIndex]);
    }
    for (steerIndex = 0; steerIndex < steerCount; steerIndex = steerIndex + 1) {
      Boolean accepted;

      accepted = apaHybridAStarExpandOnePrimitive(
          planner, directionOrder[directionIndex], steerIndex, steerCount);
      if (planner->RunState == APA_HYBRID_STATE_FAILED) {
        return false;
      }
      if (accepted) {
        anyAccepted = true;
      }
    }
  }

  return anyAccepted;
}

static inline void apaHybridAStarConfigureQuality(ApaHybridAStar_t *planner,
                                                  Boolean hasStopper,
                                                  Integer corridorFlag) {
  const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
  apaPathQualityCompareContextInit(&planner->QualityContext);
  apaPathQualityCompareContextBindSlot(
      &planner->QualityContext, &planner->ActiveSlot, &config->Veh);
  planner->QualityContext.ParkingSysMainState = APA_SYS_STATE_SEARCHING;
  planner->QualityContext.ExpDirecFlag = config->Conf.ExpectStartDirection;
  planner->QualityContext.ExpectDirectionOdoThresh = config->Conf.ExpectDirectionOdoThresh;
  planner->QualityContext.EnterPosLen = config->Conf.EnterPosLen;
  planner->ExpectedSegmentCount = apaSlotCalcExpectedSegNum(planner->SlotView.SlotClass,
                                                            planner->ParkingType,
                                                            planner->ActiveSlot.SurroundingType,
                                                            planner->SlotView.SlotSubStatus,
                                                            planner->ActiveSlot.SlotStatus,
                                                            hasStopper,
                                                            corridorFlag,
                                                            0,
                                                            0);
}

/* Quality threshold uses the available search budget; per-slot admission is separate. */
static inline Integer apaHybridAStarQualityMinimumNodes(const ApaHybridAStar_t *planner) {
  const ApaPlannerConfConfig_t *conf = apaPlannerGlobalGetConf(planner->PlannerGlobal);
  const Integer available = conf->PlanningCalPowerMaxPerSlot;
  if (available <= 0)
    return 0;
  const Integer seventyPercent = (Integer)roundf((Real)available * 0.7f);
  Integer minimum = seventyPercent;
  if (planner->ActiveSlot.SlotSubStatus != APA_SLOT_SUB_END_NONE) {
    minimum = (Integer)roundf((Real)available * 0.5f);
  } else if (planner->ActiveSlot.SlotStatus == APA_SLOT_STATUS_FAIL_PLAN &&
             apaMathLess(planner->QualityContext.VehVel, 0.2f)) {
    minimum = (Integer)roundf((Real)available * 0.5f);
    if (minimum < 200)
      minimum = 200;
    if (minimum > seventyPercent)
      minimum = seventyPercent;
  }
  return minimum < 500 ? minimum : 500;
}

static inline Boolean apaHybridAStarQualityTargetReached(const ApaHybridAStar_t *planner) {
  if (!planner->HasAcceptedCandidate)
    return false;
  const ApaPlannerConfConfig_t *conf = apaPlannerGlobalGetConf(planner->PlannerGlobal);
  if (!conf->UseMoreCalUnits)
    return true;
  const Integer segments = apaPathCalcSegNumForMotion(&planner->BestQualityPath,
                                                      planner->QualityContext.VehVel,
                                                      planner->QualityContext.GearPos,
                                                      planner->QualityContext.ParkingSysMainState);
  return segments <= planner->ExpectedSegmentCount &&
         planner->NodePool.ValidNodeCnt > apaHybridAStarQualityMinimumNodes(planner);
}

/* Search exhaustion preserves any fully admitted incumbent. */
static inline ApaHybridAStarState_e apaHybridAStarFinish(ApaHybridAStar_t *planner) {
  planner->GoalReached = planner->HasAcceptedCandidate;
  planner->RunState =
      planner->GoalReached ? APA_HYBRID_STATE_GOAL_REACHED : APA_HYBRID_STATE_FAILED;
  if (planner->Diagnostics) {
    planner->Diagnostics->SearchFault = planner->Fault;
    planner->Diagnostics->AcceptedCandidates += planner->AcceptedCandidates;
    planner->Diagnostics->RejectedCandidates += planner->RejectedGoalCandidates;
    planner->Diagnostics->TruncatedPrimitives += planner->TruncatedPrimitives;
  }
  if (!planner->GoalReached)
    apaPlanningRecordFailure(
        planner->Diagnostics, apaPlanningSearchError(planner->Fault), "hybrid_search", 1);
  return planner->RunState;
}

static inline ApaHybridAStarState_e apaHybridAStarStep(ApaHybridAStar_t *planner) {
  if (planner->RunState == APA_HYBRID_STATE_GOAL_REACHED ||
      planner->RunState == APA_HYBRID_STATE_FAILED)
    return planner->RunState;
  if (!planner->Seeded && !apaHybridAStarSeedStart(planner))
    return apaHybridAStarFinish(planner);
  const ApaPlannerConfConfig_t *conf = apaPlannerGlobalGetConf(planner->PlannerGlobal);
  if (planner->ExpandIterations >= planner->MaxExpandIterations ||
      planner->NodePool.ValidNodeCnt >= conf->PlanningCalPowerMaxPerSlot) {
    planner->Fault = APA_HYBRID_FAULT_EXCEED_CAL_POWER;
    return apaHybridAStarFinish(planner);
  }
  if (!apaHybridAStarPopCurrent(planner))
    return apaHybridAStarFinish(planner);
  if (apaHybridAStarNodeWithinGoalWindow(planner, &planner->CurrentNode))
    (void)apaHybridAStarCheckGoalCandidate(planner, &planner->CurrentNode, 0, 0);
  if (apaHybridAStarQualityTargetReached(planner))
    return apaHybridAStarFinish(planner);
  (void)apaHybridAStarTryDirectGoalLink(planner);
  if (apaHybridAStarQualityTargetReached(planner))
    return apaHybridAStarFinish(planner);
  (void)apaHybridAStarTryAnalyticConnect(planner);
  if (apaHybridAStarQualityTargetReached(planner))
    return apaHybridAStarFinish(planner);
  ++planner->ExpandIterations;
  (void)apaHybridAStarExpandCurrent(planner);
  if (planner->RunState == APA_HYBRID_STATE_FAILED)
    return apaHybridAStarFinish(planner);
  if (apaPqEmpty(&planner->OpenQueue)) {
    planner->Fault = APA_HYBRID_FAULT_HYBRID_OPEN_EMPTY;
    return apaHybridAStarFinish(planner);
  }
  planner->RunState = APA_HYBRID_STATE_RUNNING;
  return planner->RunState;
}

static inline ApaHybridAStarState_e apaHybridAStarRunToTerminal(ApaHybridAStar_t *planner) {
  ApaHybridAStarState_e runState;
  Boolean running;

  if (planner->PlannerGlobal && planner->QualityContext.SlotClass == APA_SLOT_CLASS_UNKNOWN)
    apaHybridAStarConfigureQuality(planner, false, 1);
  runState = planner->RunState;
  running = true;
  while (running) {
    Boolean terminalState;

    terminalState =
        (runState == APA_HYBRID_STATE_GOAL_REACHED) || (runState == APA_HYBRID_STATE_FAILED);
    if (terminalState) {
      running = false;
    } else {
      runState = apaHybridAStarStep(planner);
    }
  }

  return runState;
}

static inline Boolean apaHybridAStarAppendPath(ApaPath_t *dst, const ApaPath_t *src) {
  Integer i;
  Integer dstIndex;

  if (src->PathN <= 1) {
    return true;
  }
  if (dst->PathN <= 0) {
    apaPathCopy(dst, src);
    return true;
  }
  if ((dst->PathN + src->PathN - 1) > APA_PATH_MAX_POINTS) {
    return false;
  }

  dst->OdoStep[dst->PathN - 1] = src->OdoStep[0];
  for (i = 1; i < src->PathN; i = i + 1) {
    dstIndex = dst->PathN - 1 + i;
    dst->X[dstIndex] = src->X[i];
    dst->Y[dstIndex] = src->Y[i];
    dst->Tht[dstIndex] = src->Tht[i];
    dst->OdoStep[dstIndex] = src->OdoStep[i];
    dst->Radius[dstIndex] = src->Radius[i];
  }
  dst->PathN = dst->PathN + src->PathN - 1;
  return true;
}

static inline Boolean apaHybridAStarBuildMotionPathFromGoal(const ApaHybridAStar_t *planner,
                                                            const ApaHybridNode_t *goalNode,
                                                            const ApaRsPath_t *rsPath,
                                                            const ApaPath_t *analyticPath,
                                                            ApaHybridAStarMotionPath_t *motion) {
  ApaPath_t *path = &motion->Path;
  ApaHybridNode_t traceNodes[APA_PATH_MAX_POINTS];
  ApaHybridNode_t walkNode;
  Integer traceCount;
  Boolean tracing;
  Boolean reachedStart = false;
  Integer i;

  apaPathInit(path);
  for (i = 0; i < APA_PATH_MAX_POINTS; ++i)
    motion->Curvature[i] = 0.0f;

  walkNode = *goalNode;
  traceCount = 0;
  tracing = true;

  while (tracing && traceCount < APA_PATH_MAX_POINTS) {

    if (apaHybridAStarParkingSearch(planner) && traceCount > 0 && walkNode.ParentNodeIndex >= 0 &&
        traceNodes[traceCount - 1].D * walkNode.D > 0.0f &&
        fabsf(traceNodes[traceCount - 1].Delta - walkNode.Delta) < 1.0e-6f) {
      traceNodes[traceCount - 1].D += walkNode.D;
    } else {
      traceNodes[traceCount] = walkNode;
      traceCount = traceCount + 1;
    }

    if (walkNode.ParentNodeIndex < 0 &&
        hypotf(walkNode.X - planner->StartNode.X, walkNode.Y - planner->StartNode.Y) < 1.0e-4f &&
        fabsf(apaCoordWrapToPi(walkNode.Theta - planner->StartNode.Theta)) < 1.0e-4f) {
      tracing = false;
      reachedStart = true;
    } else {
      tracing = apaHybridAStarTryResolveParentNode(planner, &walkNode, &walkNode);
    }
  }

  if (traceCount <= 0) {
    return false;
  }
  if (!reachedStart) {
    apaPlanningRecordFailure(planner->Diagnostics,
                             tracing ? APA_FAULT_PATH_N_EXCEED_TO_ADD
                                     : APA_FAULT_CLOSE_PARENT_INVALID,
                             tracing ? "hybrid_trace_capacity" : "hybrid_parent_invalid",
                             4);
    return false;
  }

  path->ValidFlag = true;
  path->ErrorType = APA_FAULT_NO_ERR;
  path->PathSource = APA_PATH_SOURCE_SEARCH;
  path->PathType = APA_PATH_TYPE_ALL;
  path->CollisionFlag = APA_COLLISION_RISK_NOT_EXIST;
  path->PathN = traceCount;

  for (i = 0; i < traceCount; i = i + 1) {
    Integer srcIndex;

    srcIndex = traceCount - 1 - i;
    path->X[i] = traceNodes[srcIndex].X;
    path->Y[i] = traceNodes[srcIndex].Y;
    path->Tht[i] = traceNodes[srcIndex].Theta;
    if (i < traceCount - 1) {
      const ApaHybridNode_t *child = &traceNodes[srcIndex - 1];
      const Real wheelbase = planner->PlannerGlobal
                                 ? apaPlannerGlobalGetPlanner(planner->PlannerGlobal)->Veh.WB
                                 : APA_VEHICLE_WHEELBASE_M;
      path->OdoStep[i] = child->D;
      if (fabsf(child->Delta) >= APA_HYBRID_ASTAR_ZERO_STEER_EPS) {
        if (wheelbase <= APA_MATH_EPSILON)
          return false;
        motion->Curvature[i] = tanf(child->Delta) / wheelbase;
      }
    } else {
      path->OdoStep[i] = 0.0f;
    }
  }
  if (analyticPath && rsPath) {
    Integer interval = path->PathN - 1;
    const ApaPlannerConfig_t *config = apaPlannerGlobalGetPlanner(planner->PlannerGlobal);
    Real step = config->Map.CollisionDetectStepLength;
    if (step <= APA_MATH_EPSILON)
      step = config->Map.MotionResolution;
    if (step <= APA_MATH_EPSILON)
      step = config->Map.XyGridResolution;
    if (step <= APA_MATH_EPSILON || config->Veh.MinCircle <= APA_MATH_EPSILON)
      return false;
    if (!apaHybridAStarAppendPath(path, analyticPath)) {
      apaPlanningRecordFailure(
          planner->Diagnostics, APA_FAULT_PATH_N_EXCEED_TO_ADD, "hybrid_rs_point_capacity", 4);
      return false;
    }
    for (i = 0; i < APA_RS_MAX_SEGMENTS; ++i) {
      Real remaining = fabsf(rsPath->Lengths[i]);
      const Integer type = rsPath->Type[i];
      if (type == APA_RS_SEG_NONE || apaMathEqual(remaining, 0.0f))
        continue;
      const Real curvature =
          type == APA_RS_SEG_LEFT
              ? 1.0f / config->Veh.MinCircle
              : (type == APA_RS_SEG_RIGHT ? -1.0f / config->Veh.MinCircle : 0.0f);
      if (apaHybridAStarParkingSearch(planner)) {
        if (interval >= path->PathN - 1)
          return false;
        motion->Curvature[interval++] = curvature;
        continue;
      }
      while (remaining > APA_MATH_EPSILON) {
        const Real distance = remaining < step ? remaining : step;
        if (interval >= path->PathN - 1)
          return false;
        motion->Curvature[interval++] = curvature;
        remaining -= distance;
      }
    }
    if (interval != path->PathN - 1)
      return false;
  }

  path->SegNum = apaPathCalcSegNum(path);
  path->SegNumFlag = true;
  return true;
}

static inline Boolean apaHybridAStarBuildMotionPath(const ApaHybridAStar_t *planner,
                                                    ApaHybridAStarMotionPath_t *motion) {
  if (!planner->GoalReached)
    return false;
  return apaHybridAStarBuildMotionPathFromGoal(
      planner,
      &planner->GoalResultNode,
      planner->AnalyticPathValid ? &planner->AnalyticRsPath : (const ApaRsPath_t *)0,
      planner->AnalyticPathValid ? &planner->AnalyticPath : (const ApaPath_t *)0,
      motion);
}

static inline Boolean apaHybridAStarBuildPath(const ApaHybridAStar_t *planner, ApaPath_t *path) {
  ApaHybridAStarMotionPath_t motion;
  if (!apaHybridAStarBuildMotionPath(planner, &motion)) {
    apaPathInit(path);
    return false;
  }
  *path = motion.Path;
  return true;
}

#endif
