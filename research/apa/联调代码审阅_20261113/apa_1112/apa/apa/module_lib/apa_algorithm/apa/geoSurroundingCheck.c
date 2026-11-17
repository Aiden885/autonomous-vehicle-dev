#include "apaPlannerGlobal.h"
#include "apaSlotTypes.h"
#include "apaVehicleCollisionCheck.h"
#include "geoSurroundingCheckTypes.h"
#include <math.h>

static Real geoSurroundingCheckSafeDivide(Real num, Real den) {
  Real result0;

  if (fabsf(den) < 1e-6f) {
    if (num > 0.0f) {
      result0 = 1e6f;
    } else if (num < 0.0f) {
      result0 = -1e6f;
    } else {
      result0 = 0.0f;
    }
  } else {
    result0 = num / den;
  }
  return result0;
}

static void geoSurroundingCheckInitPlannerGlobal(ApaPlannerGlobalState_t *plannerGlobal) {
  apaPlannerGlobalInit(plannerGlobal);
}

static void geoSurroundingCheckInitSlot(const GeoSurroundingCheckInput_t *input,
                                        Integer slotClass,
                                        ApaSlot_t *slot) {
  apaSlotInit(slot);
  slot->EnterEdge = input->SltEntr;
  slot->SlotStatus = APA_SLOT_STATUS_TARGET_SLOT;
  slot->SlotClass = slotClass;

  slot->TarPos[0] = input->TarPtX;
  slot->TarPos[1] = input->TarPtY;
  slot->TarPos[2] = input->TarPtTheta;

  slot->SlotPts[0] = input->SltPt1X;
  slot->SlotPts[1] = input->SltPt1Y;
  slot->SlotPts[2] = input->SltPt2X;
  slot->SlotPts[3] = input->SltPt2Y;
  slot->SlotPts[4] = input->SltPt3X;
  slot->SlotPts[5] = input->SltPt3Y;
  slot->SlotPts[6] = input->SltPt4X;
  slot->SlotPts[7] = input->SltPt4Y;
}

static void geoSurroundingCheckLoadObs(const GeoSurroundingCheckInput_t *input,
                                       ApaVehicleCollisionObs_t *obs) {
  Integer idx;

  idx = 0;
  while ((idx < input->BoundaryLineNum) && (idx < SURD_LINE_MAX)) {
    (void)apaVehicleCollisionObsAddBoundaryLine(obs,
                                                input->BoundaryP1X[idx],
                                                input->BoundaryP1Y[idx],
                                                input->BoundaryP2X[idx],
                                                input->BoundaryP2Y[idx]);
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->BoundarySepPtNum) && (idx < SURD_PT_MAX)) {
    (void)apaVehicleCollisionObsAddBoundarySeparatePoint(
        obs, input->BoundarySepPtX[idx], input->BoundarySepPtY[idx]);
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->StopperPtNum) && (idx < SURD_PT_MAX)) {
    (void)apaVehicleCollisionObsAddSeparateStopper(
        obs, input->StopperPtX[idx], input->StopperPtY[idx]);
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->SlotTrackStopperNum) && (idx < VCC_MAX_TRACK_STOPPER)) {
    (void)apaVehicleCollisionObsAddSlotTrackStopper(obs,
                                                    input->SlotTrackStopperX1[idx],
                                                    input->SlotTrackStopperY1[idx],
                                                    input->SlotTrackStopperX2[idx],
                                                    input->SlotTrackStopperY2[idx]);
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->VirtualBoundaryLineNum) && (idx < VCC_MAX_VIRTUAL_LINES)) {
    (void)apaVehicleCollisionObsAddVirtualBoundaryLine(obs,
                                                       input->VirtualBoundaryLineX1[idx],
                                                       input->VirtualBoundaryLineY1[idx],
                                                       input->VirtualBoundaryLineX2[idx],
                                                       input->VirtualBoundaryLineY2[idx]);
    idx = idx + 1;
  }
}

static Integer geoSurroundingCheckVehicleCollisionCheck(
    Real ptX, Real ptY, Real ptTheta, const GeoSurroundingCheckInput_t *input, Integer slotClass) {
  ApaPlannerGlobalState_t plannerGlobal = APA_PLANNER_GLOBAL_STATE_INIT;
  ApaVehicleCollisionCheckContext_t collisionContext;
  ApaVehicleCollisionCheckResult_t collisionResult;
  ApaSlot_t slot;
  Boolean isCollision;
  Integer collisionFlag;

  geoSurroundingCheckInitPlannerGlobal(&plannerGlobal);
  apaVehicleCollisionCheckContextInit(&collisionContext);
  apaVehicleCollisionCheckContextSetPlannerGlobal(&collisionContext, &plannerGlobal);
  apaVehicleCollisionCheckContextSetCheckType(&collisionContext, APA_COLLISION_CHECK_PLANNING);
  apaVehicleCollisionCheckContextSetCollisionDemand(&collisionContext,
                                                    APA_COLLISION_DEMAND_SIMPLIFY);

  geoSurroundingCheckInitSlot(input, slotClass, &slot);
  apaVehicleCollisionCheckContextSetSlot(&collisionContext, &slot);
  apaVehicleCollisionCheckContextSetTargetPose(&collisionContext, slot.TarPos);
  geoSurroundingCheckLoadObs(input, &collisionContext.Obs);

  apaVehicleCollisionCheckResultInit(&collisionResult);
  isCollision =
      apaVehicleCollisionCheckPose(&collisionContext, ptX, ptY, ptTheta, &collisionResult);

  if (isCollision) {
    collisionFlag = 1;
  } else {
    collisionFlag = 0;
  }
  return collisionFlag;
}

/**
 * @brief 环绕检测模块
 * @en_name geoSurroundingCheck
 * @cn_name 环绕检测
 * @type module
 * @param[IN] GeoSurroundingCheckInput_t Input 环绕检测输入
 * @param[OUT] GeoSurroundingCheckOutput_t Output 环绕检测输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 安全检查
 * @source matlab_rewrite geo_SurroundingCheck.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void geoSurroundingCheck(const GeoSurroundingCheckInput_t *input,
                         GeoSurroundingCheckOutput_t *output) {
  Real slotPts[8];
  Real tarPos[3];
  Real sideLineOne[4];
  Real sideLineTwo[4];
  Integer slotClass;
  Integer surdType;

  *output = (GeoSurroundingCheckOutput_t){0};

  slotPts[0] = input->SltPt1X;
  slotPts[1] = input->SltPt1Y;
  slotPts[2] = input->SltPt2X;
  slotPts[3] = input->SltPt2Y;
  slotPts[4] = input->SltPt3X;
  slotPts[5] = input->SltPt3Y;
  slotPts[6] = input->SltPt4X;
  slotPts[7] = input->SltPt4Y;

  tarPos[0] = input->TarPtX;
  tarPos[1] = input->TarPtY;
  tarPos[2] = input->TarPtTheta;

  slotClass = apaSlotCalcSlotClass(slotPts, input->SltEntr);
  apaSlotCalcSideLine(slotPts, input->SltEntr, tarPos, slotClass, sideLineOne, sideLineTwo);

  if (slotClass == APA_SLOT_CLASS_PARALLEL) {

    Real vecLonX, vecLonY, normLon;
    vecLonX = sideLineOne[0] - sideLineTwo[0];
    vecLonY = sideLineOne[1] - sideLineTwo[1];
    normLon = sqrtf(vecLonX * vecLonX + vecLonY * vecLonY);
    vecLonX = geoSurroundingCheckSafeDivide(vecLonX, normLon) * 1.5f;
    vecLonY = geoSurroundingCheckSafeDivide(vecLonY, normLon) * 1.5f;

    Real vecLatX, vecLatY, normLat;
    vecLatX = sideLineOne[0] - sideLineOne[2];
    vecLatY = sideLineOne[1] - sideLineOne[3];
    normLat = sqrtf(vecLatX * vecLatX + vecLatY * vecLatY);
    vecLatX = geoSurroundingCheckSafeDivide(vecLatX, normLat) * 0.5f;
    vecLatY = geoSurroundingCheckSafeDivide(vecLatY, normLat) * 0.5f;

    Real posFrX, posFrY;
    Real posReX, posReY;
    posFrX = input->TarPtX + vecLonX + vecLatX;
    posFrY = input->TarPtY + vecLonY + vecLatY;
    posReX = input->TarPtX - vecLonX + vecLatX;
    posReY = input->TarPtY - vecLonY + vecLatY;

    Integer colFr, colRe;
    colFr = geoSurroundingCheckVehicleCollisionCheck(
        posFrX, posFrY, input->TarPtTheta, input, slotClass);
    colRe = geoSurroundingCheckVehicleCollisionCheck(
        posReX, posReY, input->TarPtTheta, input, slotClass);

    surdType = (1 - colFr) + 2 * (1 - colRe);
  } else {
    Real vecFrX, vecFrY, normFr;
    vecFrX = sideLineOne[0] - sideLineOne[2];
    vecFrY = sideLineOne[1] - sideLineOne[3];
    normFr = sqrtf(vecFrX * vecFrX + vecFrY * vecFrY);
    vecFrX = geoSurroundingCheckSafeDivide(vecFrX, normFr) * 1.5f;
    vecFrY = geoSurroundingCheckSafeDivide(vecFrY, normFr) * 1.5f;

    Real vecLtX, vecLtY, normLt;
    vecLtX = sideLineOne[2] - sideLineTwo[2];
    vecLtY = sideLineOne[3] - sideLineTwo[3];
    normLt = sqrtf(vecLtX * vecLtX + vecLtY * vecLtY);
    vecLtX = geoSurroundingCheckSafeDivide(vecLtX, normLt) * 1.0f;
    vecLtY = geoSurroundingCheckSafeDivide(vecLtY, normLt) * 1.0f;

    Real vecRtX, vecRtY;
    vecRtX = -vecLtX;
    vecRtY = -vecLtY;

    Real posLtX, posLtY;
    Real posRtX, posRtY;
    posLtX = input->TarPtX + vecFrX + vecLtX;
    posLtY = input->TarPtY + vecFrY + vecLtY;
    posRtX = input->TarPtX + vecFrX + vecRtX;
    posRtY = input->TarPtY + vecFrY + vecRtY;

    Integer colLt, colRt;
    colLt = geoSurroundingCheckVehicleCollisionCheck(
        posLtX, posLtY, input->TarPtTheta, input, slotClass);
    colRt = geoSurroundingCheckVehicleCollisionCheck(
        posRtX, posRtY, input->TarPtTheta, input, slotClass);

    surdType = (1 - colLt) + 2 * (1 - colRt);
  }

  output->SurroundingType = surdType;
}
