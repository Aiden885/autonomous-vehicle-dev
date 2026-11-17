/**
 * @file apaParallelParkoutPlanTypes.h
 * @brief APA 平行泊出规划特有的上下文及配置控制块结构
 *
 * @date 2026-04-13
 */
#ifndef APA_PARALLEL_PARKOUT_PLAN_TYPES_H
#define APA_PARALLEL_PARKOUT_PLAN_TYPES_H

#include "baseTypes.h"
#include "apaSlotTypes.h"
#include "apaPathTypes.h"
#include "apaPlannerConfig.h"
#include "apaCoordUtils.h"
#include "apaMacros.h"
#include "vehicleCollisionCheckTypes.h"

#define APA_PARKOUT_COLLISION_STAGE_NONE 0
#define APA_PARKOUT_COLLISION_STAGE_INITIAL_STRAIGHT 1
#define APA_PARKOUT_COLLISION_STAGE_MANEUVER_FORWARD 2
#define APA_PARKOUT_COLLISION_STAGE_MANEUVER_BACKWARD 3
#define APA_PARKOUT_COLLISION_STAGE_FINAL_ARC 4

#define APA_PARKOUT_BRANCH_DIR_UNKNOWN 0
#define APA_PARKOUT_BRANCH_DIR_LEFT 1
#define APA_PARKOUT_BRANCH_DIR_RIGHT 2
#define APA_PARKOUT_BRANCH_DIR_FRONT 3

#define APA_PARKOUT_FAIL_RS_NONE 0
#define APA_PARKOUT_FAIL_RS_SIDE_SENSOR_CLOSE 1
#define APA_PARKOUT_FAIL_RS_FRONT_SPACE_INSUFFICIENT 2
#define APA_PARKOUT_FAIL_RS_FORWARD_DANGER_BLOCKED 3
#define APA_PARKOUT_FAIL_RS_REVERSE_GEOMETRY_INFEASIBLE 4

#define APA_PARKOUT_BACK_STRATEGY_NONE 0
#define APA_PARKOUT_BACK_STRATEGY_BACK_ENABLE 1
#define APA_PARKOUT_BACK_STRATEGY_MEB 2

typedef struct {
  Real TurnRadius;
  Real SampleStep;
  Real CollinearThresh;
  Real OdoThresh;
  Real BackoutDistanceLimit;
  Boolean DisableFtPlannerCandidates;
} ApaParallelParkoutConfig_t;

typedef struct {
  ApaParallelParkoutConfig_t Config;
  ApaSlot_t Slot;
  Real StartPos[3];
  Real GoalPos[3];
  Integer SlotClass;
  Boolean SlotOnTheRight;
  Real BackSlotPtNear[2];
  Integer DirOut;
  Real FrontUS[4];
  Real LeftUS[2];
  Integer RplnFlag;
  Integer RequestedDir;
  Real LeftSideSpace;
  Real RightSideSpace;
  Integer BackEnable;
  Integer MebDir;
} ApaParallelParkoutContext_t;

typedef struct {
  Integer FailReason;
  Integer SafeSpaceFlag;
  Integer DangerPtType;
  Integer CheckCount;
  Real DangerPtX;
  Real DangerPtY;
  Real SelectedPoseX;
  Real SelectedPoseY;
  Real SelectedPoseTheta;
  Real ForwardSafeLen;
  Integer CollisionType;
  Integer CollisionStage;
  Real CollisionPoseX;
  Real CollisionPoseY;
  Real CollisionPoseTheta;
  Integer BranchDir;
  Integer PkOutFailRs;
  Integer BackStrategy;
  Real BackoutDistance;
  Integer FrspLineNum;
  Real FrspLineX1[VCC_MAX_BOUNDARY_LINES];
  Real FrspLineY1[VCC_MAX_BOUNDARY_LINES];
  Real FrspLineX2[VCC_MAX_BOUNDARY_LINES];
  Real FrspLineY2[VCC_MAX_BOUNDARY_LINES];
  Integer FrspSepPtNum;
  Real FrspSepPtX[VCC_MAX_SEPARATE_PTS];
  Real FrspSepPtY[VCC_MAX_SEPARATE_PTS];
  Integer FrspVirtualLineNum;
  Real FrspVirtualLineX1[VCC_MAX_VIRTUAL_LINES];
  Real FrspVirtualLineY1[VCC_MAX_VIRTUAL_LINES];
  Real FrspVirtualLineX2[VCC_MAX_VIRTUAL_LINES];
  Real FrspVirtualLineY2[VCC_MAX_VIRTUAL_LINES];
  Integer FrspSeparateStopperNum;
  Real FrspSeparateStopperX[VCC_MAX_SEPARATE_PTS];
  Real FrspSeparateStopperY[VCC_MAX_SEPARATE_PTS];
  Integer FrspSlotTrackStopperNum;
  Real FrspSlotTrackStopperX1[VCC_MAX_TRACK_STOPPER];
  Real FrspSlotTrackStopperY1[VCC_MAX_TRACK_STOPPER];
  Real FrspSlotTrackStopperX2[VCC_MAX_TRACK_STOPPER];
  Real FrspSlotTrackStopperY2[VCC_MAX_TRACK_STOPPER];
} ApaParallelParkoutDiag_t;

static inline void apaParallelParkoutDiagInit(ApaParallelParkoutDiag_t *diag) {
  Integer lineIdx0;
  Integer sepIdx0;
  Integer virtualIdx0;
  Integer stopperIdx0;
  Integer slotTrackIdx0;

  diag->FailReason = APA_FAULT_UNKNOWN_ERR;
  diag->SafeSpaceFlag = 0;
  diag->DangerPtType = 0;
  diag->CheckCount = 0;
  diag->DangerPtX = 0.0f;
  diag->DangerPtY = 0.0f;
  diag->SelectedPoseX = 0.0f;
  diag->SelectedPoseY = 0.0f;
  diag->SelectedPoseTheta = 0.0f;
  diag->ForwardSafeLen = 0.0f;
  diag->CollisionType = VCC_COLLISION_NONE;
  diag->CollisionStage = APA_PARKOUT_COLLISION_STAGE_NONE;
  diag->CollisionPoseX = 0.0f;
  diag->CollisionPoseY = 0.0f;
  diag->CollisionPoseTheta = 0.0f;
  diag->BranchDir = APA_PARKOUT_BRANCH_DIR_UNKNOWN;
  diag->PkOutFailRs = APA_PARKOUT_FAIL_RS_NONE;
  diag->BackStrategy = APA_PARKOUT_BACK_STRATEGY_NONE;
  diag->BackoutDistance = 0.0f;
  diag->FrspLineNum = 0;
  lineIdx0 = 0;
  while (lineIdx0 < VCC_MAX_BOUNDARY_LINES) {
    diag->FrspLineX1[lineIdx0] = 0.0f;
    diag->FrspLineY1[lineIdx0] = 0.0f;
    diag->FrspLineX2[lineIdx0] = 0.0f;
    diag->FrspLineY2[lineIdx0] = 0.0f;
    lineIdx0 = lineIdx0 + 1;
  }
  diag->FrspSepPtNum = 0;
  sepIdx0 = 0;
  while (sepIdx0 < VCC_MAX_SEPARATE_PTS) {
    diag->FrspSepPtX[sepIdx0] = 0.0f;
    diag->FrspSepPtY[sepIdx0] = 0.0f;
    sepIdx0 = sepIdx0 + 1;
  }
  diag->FrspVirtualLineNum = 0;
  virtualIdx0 = 0;
  while (virtualIdx0 < VCC_MAX_VIRTUAL_LINES) {
    diag->FrspVirtualLineX1[virtualIdx0] = 0.0f;
    diag->FrspVirtualLineY1[virtualIdx0] = 0.0f;
    diag->FrspVirtualLineX2[virtualIdx0] = 0.0f;
    diag->FrspVirtualLineY2[virtualIdx0] = 0.0f;
    virtualIdx0 = virtualIdx0 + 1;
  }
  diag->FrspSeparateStopperNum = 0;
  stopperIdx0 = 0;
  while (stopperIdx0 < VCC_MAX_SEPARATE_PTS) {
    diag->FrspSeparateStopperX[stopperIdx0] = 0.0f;
    diag->FrspSeparateStopperY[stopperIdx0] = 0.0f;
    stopperIdx0 = stopperIdx0 + 1;
  }
  diag->FrspSlotTrackStopperNum = 0;
  slotTrackIdx0 = 0;
  while (slotTrackIdx0 < VCC_MAX_TRACK_STOPPER) {
    diag->FrspSlotTrackStopperX1[slotTrackIdx0] = 0.0f;
    diag->FrspSlotTrackStopperY1[slotTrackIdx0] = 0.0f;
    diag->FrspSlotTrackStopperX2[slotTrackIdx0] = 0.0f;
    diag->FrspSlotTrackStopperY2[slotTrackIdx0] = 0.0f;
    slotTrackIdx0 = slotTrackIdx0 + 1;
  }
}

Boolean apaParallelParkoutPlanWithDiag(const ApaParallelParkoutContext_t *ctx,
                                       const VehicleCollisionCheckInput_t *obsInput,
                                       const ApaPlannerVehConfig_t *vehConfig,
                                       ApaParallelParkoutDiag_t *diag,
                                       ApaComplexPath_t *outPath);

/* forward declaration for gate linkage */
Boolean apaParallelParkoutPlan(const ApaParallelParkoutContext_t *ctx,
                               const VehicleCollisionCheckInput_t *obsInput,
                               const ApaPlannerVehConfig_t *vehConfig,
                               ApaComplexPath_t *outPath);

#endif
