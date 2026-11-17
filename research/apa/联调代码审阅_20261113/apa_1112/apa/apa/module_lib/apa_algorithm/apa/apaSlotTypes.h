/**
 * @file apaSlotTypes.h
 * @brief APA 车位数据结构 — 对应 ClassSlot.m / ClassMultiSlot.m / ClassSlotListArr.m
 *
 * 将 MATLAB OOP handle 类改写为 C99 struct + 工具函数。
 * ClassSlot 拆分为：数据结构 (ApaSlot_t) + 静态方法 (apaSlot_* 函数)
 *
 * @date 2026-04-13
 */
#ifndef APA_SLOT_TYPES_H
#define APA_SLOT_TYPES_H

#include "apaEnums.h"
#include "apaCoordUtils.h"
#include "apaGeoUtils.h"
#include "apaMathUtils.h"
#include "apaPlannerConfig.h"
#include "apaVehicleUtils.h"
#include "baseTypes.h"
#include "rectCollisionCheckForBoundaryTypes.h"
#include <string.h>

/** @brief 最大车位数量 — 对应 ClassMultiSlot 的 20 个车位 */
#define APA_MAX_SLOTS 20

/**
 * @brief 单车位数据 — 对应 ClassSlot properties
 *
 * slot_pts 存储格式：[P1x,P1y, P2x,P2y, P3x,P3y, P4x,P4y]
 * tar_pos 存储格式：[x, y, yaw]
 */
typedef struct {
  /* 基础属性（来自 ClassSlotListArr 输入） */
  Integer SlotId;        /**< 车位编号 */
  Real SlotPts[8];       /**< 车位四角点 [P1x,P1y,...,P4x,P4y] */
  Integer EnterEdge;     /**< 入口边编号 */
  Real TarPos[3];        /**< 目标位姿 [x, y, yaw] */
  Integer SlotStatus;    /**< 车位状态 ApaSlotStatus_e */
  Integer SlotSubStatus; /**< 车位子状态 ApaSlotSubStatus_e */

  /* 派生属性（惰性计算，初始为 UNKNOWN/INVALID） */
  Integer SlotClass;       /**< 车位分类 ApaSlotClass_e */
  Integer ParkingType;     /**< 泊车类型（左/右） ApaParkingType_e */
  Integer ParkingDirec;    /**< 泊车方向（车头/车尾） ApaParkingDirec_e */
  Integer SurroundingType; /**< 环绕类型 ApaSurroundingType_e */
} ApaSlot_t;

/**
 * @brief 车位列表数组 — 对应 ClassSlotListArr
 *
 * 扁平化存储，供批量初始化 ApaSlot_t 使用。
 */
typedef struct {
  Integer SlotIdList[APA_MAX_SLOTS];
  Real SlotPtsList[APA_MAX_SLOTS][8];
  Integer EnterEdgeList[APA_MAX_SLOTS];
  Real TarPosList[APA_MAX_SLOTS][3];
  Integer SlotStatusList[APA_MAX_SLOTS];
  Integer SlotSubStatusList[APA_MAX_SLOTS];
} ApaSlotListArr_t;

/**
 * @brief 多车位管理器 — 对应 ClassMultiSlot
 */
typedef struct {
  ApaSlot_t Slots[APA_MAX_SLOTS];
  Integer ActiveCount; /**< 实际有效车位数 */
} ApaMultiSlot_t;

/** @brief 初始化单车位为默认值 */
static inline void apaSlotInit(ApaSlot_t *slot) {
  Integer i;

  slot->SlotId = 0;
  slot->EnterEdge = 0;
  slot->SlotStatus = APA_SLOT_STATUS_INVALID;
  slot->SlotSubStatus = APA_SLOT_SUB_END_NONE;
  slot->SlotClass = APA_SLOT_CLASS_UNKNOWN;
  slot->ParkingType = APA_PARKING_TYPE_UNKNOWN;
  slot->ParkingDirec = APA_PARKING_DIREC_INVALID;
  slot->SurroundingType = APA_SURROUND_UNKNOWN;

  for (i = 0; i < 8; i = i + 1) {
    slot->SlotPts[i] = 0.0f;
  }
  for (i = 0; i < 3; i = i + 1) {
    slot->TarPos[i] = 0.0f;
  }
}

/** @brief 从 SlotListArr 初始化第 idx 个车位 */
static inline void apaSlotInitFromArr(ApaSlot_t *slot, const ApaSlotListArr_t *arr, Integer idx) {
  Integer i;

  slot->SlotId = arr->SlotIdList[idx];
  slot->EnterEdge = arr->EnterEdgeList[idx];
  slot->SlotStatus = arr->SlotStatusList[idx];
  slot->SlotSubStatus = arr->SlotSubStatusList[idx];

  for (i = 0; i < 8; i = i + 1) {
    slot->SlotPts[i] = arr->SlotPtsList[idx][i];
  }
  for (i = 0; i < 3; i = i + 1) {
    slot->TarPos[i] = arr->TarPosList[idx][i];
  }

  slot->SlotClass = APA_SLOT_CLASS_UNKNOWN;
  slot->ParkingType = APA_PARKING_TYPE_UNKNOWN;
  slot->ParkingDirec = APA_PARKING_DIREC_INVALID;
  slot->SurroundingType = APA_SURROUND_UNKNOWN;
}

/** @brief 初始化多车位管理器 */
static inline void apaMultiSlotInit(ApaMultiSlot_t *ms, const ApaSlotListArr_t *arr) {
  Integer i;

  ms->ActiveCount = APA_MAX_SLOTS;
  for (i = 0; i < APA_MAX_SLOTS; i = i + 1) {
    apaSlotInitFromArr(&ms->Slots[i], arr, i);
  }
}

/** @brief 拷贝车位数据（值拷贝） — 对应 ClassSlot.assign() */
static inline void apaSlotCopy(ApaSlot_t *dst, const ApaSlot_t *src) {
  Integer i;

  dst->SlotId = src->SlotId;
  dst->EnterEdge = src->EnterEdge;
  dst->SlotStatus = src->SlotStatus;
  dst->SlotSubStatus = src->SlotSubStatus;
  dst->SlotClass = src->SlotClass;
  dst->ParkingType = src->ParkingType;
  dst->ParkingDirec = src->ParkingDirec;
  dst->SurroundingType = src->SurroundingType;

  for (i = 0; i < 8; i = i + 1) {
    dst->SlotPts[i] = src->SlotPts[i];
  }
  for (i = 0; i < 3; i = i + 1) {
    dst->TarPos[i] = src->TarPos[i];
  }
}

/** @brief 入口边编号归一到 MATLAB ClassSlot 的 1..4 语义 */
static inline Integer apaSlotNormalizeEnterEdge(Integer enterEdge) {
  Integer normalized;

  normalized = enterEdge;
  if (normalized <= 0) {
    normalized = 1;
  }
  while (normalized > 4) {
    normalized = normalized - 4;
  }
  return normalized;
}

/** @brief 坐标下标按 ClassSlot.slotPtsPtrOverflowProcess 规则回卷 */
static inline Integer apaSlotWrapCoordIndex(Integer coordPtrOneBased) {
  Integer wrapped;

  wrapped = coordPtrOneBased;
  while (wrapped > 8) {
    wrapped = wrapped - 8;
  }
  while (wrapped <= 0) {
    wrapped = wrapped + 8;
  }
  return wrapped - 1;
}

/** @brief Store four scalar slot corners in the canonical flat layout. */
static inline void apaSlotSetPoints(Real slotPts[8],
                                    Real pt1X,
                                    Real pt1Y,
                                    Real pt2X,
                                    Real pt2Y,
                                    Real pt3X,
                                    Real pt3Y,
                                    Real pt4X,
                                    Real pt4Y) {
  slotPts[0] = pt1X;
  slotPts[1] = pt1Y;
  slotPts[2] = pt2X;
  slotPts[3] = pt2Y;
  slotPts[4] = pt3X;
  slotPts[5] = pt3Y;
  slotPts[6] = pt4X;
  slotPts[7] = pt4Y;
}

/** @brief 按入口边读取 pt1..pt4 */
static inline void apaSlotGetEnterEdgePoint(
    const Real slotPts[8], Integer enterEdge, Integer pointOffset, Real *outX, Real *outY) {
  Integer edge;
  Integer xPtr;
  Integer yPtr;

  edge = apaSlotNormalizeEnterEdge(enterEdge);
  xPtr = apaSlotWrapCoordIndex((2 * edge) - 1 + (2 * pointOffset));
  yPtr = apaSlotWrapCoordIndex((2 * edge) + (2 * pointOffset));
  *outX = slotPts[xPtr];
  *outY = slotPts[yPtr];
}

/* Collision-check adapters use P1-P2 as the entrance, P3-P4 as bottom. */
static inline void apaSlotOrderFromEntrance(
    Integer entry, Real *x1, Real *y1, Real *x2, Real *y2, Real *x3, Real *y3, Real *x4, Real *y4) {
  const Real raw[8] = {*x1, *y1, *x2, *y2, *x3, *y3, *x4, *y4};
  apaSlotGetEnterEdgePoint(raw, entry, 0, x1, y1);
  apaSlotGetEnterEdgePoint(raw, entry, 1, x2, y2);
  apaSlotGetEnterEdgePoint(raw, entry, 2, x3, y3);
  apaSlotGetEnterEdgePoint(raw, entry, 3, x4, y4);
}

/** @brief Read a 1-based slot edge; callers provide an index in [1, 4]. */
static inline void apaSlotGetEdge(const Real slotPts[8],
                                  Integer edgeIndex,
                                  Real *edgeX1,
                                  Real *edgeY1,
                                  Real *edgeX2,
                                  Real *edgeY2) {
  apaSlotGetEnterEdgePoint(slotPts, edgeIndex, 0, edgeX1, edgeY1);
  apaSlotGetEnterEdgePoint(slotPts, edgeIndex, 1, edgeX2, edgeY2);
}

/**
 * @brief 计算车位分类 — 对应 ClassSlot.calcSlotClass()
 *
 * 通过入口边长度与侧边长度的比值判定垂直/平行车位。
 */
static inline Integer apaSlotCalcSlotClass(const Real slotPts[8], Integer enterEdge) {
  Real p1x;
  Real p1y;
  Real p2x;
  Real p2y;
  Real p3x;
  Real p3y;
  Real enterLen2;
  Real sideLen2;

  apaSlotGetEnterEdgePoint(slotPts, enterEdge, 0, &p1x, &p1y);
  apaSlotGetEnterEdgePoint(slotPts, enterEdge, 1, &p2x, &p2y);
  apaSlotGetEnterEdgePoint(slotPts, enterEdge, 2, &p3x, &p3y);

  enterLen2 = (p1x - p2x) * (p1x - p2x) + (p1y - p2y) * (p1y - p2y);
  sideLen2 = (p2x - p3x) * (p2x - p3x) + (p2y - p3y) * (p2y - p3y);

  if (enterLen2 > sideLen2) {
    return APA_SLOT_CLASS_PARALLEL;
  }
  return APA_SLOT_CLASS_VERTICAL;
}

/**
 * @brief 计算侧边线 — 对应 ClassSlot.calcSideLine()
 *
 * 输出左/右侧边线，每条线 4 值 [入口端X,Y, 底端X,Y]
 */
static inline void apaSlotCalcSideLine(const Real slotPts[8],
                                       Integer enterEdge,
                                       const Real tarPos[3],
                                       Integer slotClass,
                                       Real lnLeft[4],
                                       Real lnRight[4]) {
  Real p1x;
  Real p1y;
  Real p2x;
  Real p2y;
  Real p3x;
  Real p3y;
  Real p4x;
  Real p4y;
  Integer resolvedSlotClass;

  apaSlotGetEnterEdgePoint(slotPts, enterEdge, 0, &p1x, &p1y);
  apaSlotGetEnterEdgePoint(slotPts, enterEdge, 1, &p2x, &p2y);
  apaSlotGetEnterEdgePoint(slotPts, enterEdge, 2, &p3x, &p3y);
  apaSlotGetEnterEdgePoint(slotPts, enterEdge, 3, &p4x, &p4y);

  resolvedSlotClass = slotClass;
  if (resolvedSlotClass == APA_SLOT_CLASS_UNKNOWN) {
    resolvedSlotClass = apaSlotCalcSlotClass(slotPts, enterEdge);
  }

  if (resolvedSlotClass == APA_SLOT_CLASS_VERTICAL) {
    lnLeft[0] = p2x;
    lnLeft[1] = p2y;
    lnLeft[2] = p3x;
    lnLeft[3] = p3y;

    lnRight[0] = p1x;
    lnRight[1] = p1y;
    lnRight[2] = p4x;
    lnRight[3] = p4y;
  } else {
    Real pt1Pose[3];
    Real pt2Pose[3];
    Real pt1Local[3];
    Real pt2Local[3];
    Real alpha1;
    Real alpha2;
    Real frontInsideX;
    Real frontInsideY;
    Real frontOutsideX;
    Real frontOutsideY;
    Real rearInsideX;
    Real rearInsideY;
    Real rearOutsideX;
    Real rearOutsideY;

    pt1Pose[0] = p1x;
    pt1Pose[1] = p1y;
    pt1Pose[2] = 0.0f;
    pt2Pose[0] = p2x;
    pt2Pose[1] = p2y;
    pt2Pose[2] = 0.0f;
    apaCoordGroundToVehPose(tarPos, pt1Pose, pt1Local);
    apaCoordGroundToVehPose(tarPos, pt2Pose, pt2Local);
    alpha1 = apaCoordWrapToPi(atan2f(pt1Local[1], pt1Local[0]));
    alpha2 = apaCoordWrapToPi(atan2f(pt2Local[1], pt2Local[0]));

    if (apaMathLess(apaMathAbs(alpha1), APA_PI * 0.5f) &&
        apaMathGreater(apaMathAbs(alpha2), APA_PI * 0.5f)) {
      frontInsideX = p1x;
      frontInsideY = p1y;
      rearInsideX = p2x;
      rearInsideY = p2y;
      rearOutsideX = p3x;
      rearOutsideY = p3y;
      frontOutsideX = p4x;
      frontOutsideY = p4y;
    } else {
      rearInsideX = p1x;
      rearInsideY = p1y;
      frontInsideX = p2x;
      frontInsideY = p2y;
      frontOutsideX = p3x;
      frontOutsideY = p3y;
      rearOutsideX = p4x;
      rearOutsideY = p4y;
    }

    lnLeft[0] = frontInsideX;
    lnLeft[1] = frontInsideY;
    lnLeft[2] = frontOutsideX;
    lnLeft[3] = frontOutsideY;

    lnRight[0] = rearInsideX;
    lnRight[1] = rearInsideY;
    lnRight[2] = rearOutsideX;
    lnRight[3] = rearOutsideY;
  }
}

static inline void apaSlotCalcSlotBounds(const Real slotPts[8], Real bounds[4]) {
  Integer idx;
  Real xMin;
  Real yMin;
  Real xMax;
  Real yMax;

  xMin = slotPts[0];
  yMin = slotPts[1];
  xMax = slotPts[0];
  yMax = slotPts[1];

  for (idx = 1; idx < 4; idx = idx + 1) {
    Real ptX;
    Real ptY;

    ptX = slotPts[2 * idx];
    ptY = slotPts[(2 * idx) + 1];
    if (apaMathLess(ptX, xMin)) {
      xMin = ptX;
    }
    if (apaMathGreater(ptX, xMax)) {
      xMax = ptX;
    }
    if (apaMathLess(ptY, yMin)) {
      yMin = ptY;
    }
    if (apaMathGreater(ptY, yMax)) {
      yMax = ptY;
    }
  }

  bounds[0] = xMin;
  bounds[1] = yMin;
  bounds[2] = xMax;
  bounds[3] = yMax;
}

static inline Boolean apaSlotPointInSlotWithBounds(const Real slotPts[8],
                                                   const Real slotBounds[4],
                                                   Real pointX,
                                                   Real pointY) {
  if (apaMathLess(pointX, slotBounds[0])) {
    return false;
  }
  if (apaMathGreater(pointX, slotBounds[2])) {
    return false;
  }
  if (apaMathLess(pointY, slotBounds[1])) {
    return false;
  }
  if (apaMathGreater(pointY, slotBounds[3])) {
    return false;
  }
  return apaMathIsPtInRect(pointX, pointY, slotPts);
}

/* @source matlab_rewrite ClassSlot.m#calcParkingDirec */
static inline Integer apaSlotCalcParkingDirecFromSideLines(const Real lnLeft[4],
                                                           const Real lnRight[4],
                                                           const Real tarPos[3]) {
  Real enterDx;
  Real enterDy;
  Real sideDx;
  Real sideDy;
  Real enterEdgeLen;
  Real sideEdgeLen;
  Real centerX;
  Real centerY;
  Real centerLocalX;
  Real centerLocalY;
  Real centerLocalYaw;

  enterDx = lnLeft[0] - lnRight[0];
  enterDy = lnLeft[1] - lnRight[1];
  sideDx = lnLeft[0] - lnLeft[2];
  sideDy = lnLeft[1] - lnLeft[3];
  enterEdgeLen = apaMathNorm2D(enterDx, enterDy);
  sideEdgeLen = apaMathNorm2D(sideDx, sideDy);
  if (apaMathGreater(enterEdgeLen, sideEdgeLen)) {
    return APA_PARKING_DIREC_REAR;
  }

  centerX = 0.5f * (lnLeft[0] + lnRight[0]);
  centerY = 0.5f * (lnLeft[1] + lnRight[1]);
  apaCoordGround2Veh(tarPos[0],
                     tarPos[1],
                     tarPos[2],
                     centerX,
                     centerY,
                     0.0f,
                     &centerLocalX,
                     &centerLocalY,
                     &centerLocalYaw);
  (void)centerLocalY;
  (void)centerLocalYaw;

  if (apaMathGreater(centerLocalX, 0.0f)) {
    return APA_PARKING_DIREC_REAR;
  }
  return APA_PARKING_DIREC_FRONT;
}

/* @source matlab_rewrite ClassSlot.m#calcParkingDirec */
static inline Integer
apaSlotCalcParkingDirec(const Real slotPts[8], Integer enterEdge, const Real tarPos[3]) {
  Real lnLeft[4];
  Real lnRight[4];

  apaSlotCalcSideLine(slotPts, enterEdge, tarPos, APA_SLOT_CLASS_VERTICAL, lnLeft, lnRight);
  return apaSlotCalcParkingDirecFromSideLines(lnLeft, lnRight, tarPos);
}

/* @source matlab_rewrite ClassSlot.m#getParkingPos */
static inline void apaSlotCalcParkingPos(const Real slotPts[8],
                                         Integer enterEdge,
                                         const Real tarPos[3],
                                         Integer slotClass,
                                         Real enterPosLen,
                                         Real transitionPosLen,
                                         Real enterPos[3],
                                         Real transitionPos[3]) {
  Real lnOne[4];
  Real lnTwo[4];
  Real posVecX;
  Real posVecY;
  Real enterBiasLen;
  Real transitionBiasLen;
  Integer idx;
  Integer resolvedSlotClass;

  resolvedSlotClass = slotClass;
  if (resolvedSlotClass == APA_SLOT_CLASS_UNKNOWN) {
    resolvedSlotClass = apaSlotCalcSlotClass(slotPts, enterEdge);
  }
  apaSlotCalcSideLine(slotPts, enterEdge, tarPos, resolvedSlotClass, lnOne, lnTwo);
  (void)lnTwo;

  for (idx = 0; idx < 3; idx = idx + 1) {
    enterPos[idx] = tarPos[idx];
    transitionPos[idx] = tarPos[idx];
  }

  apaMathNormalize2D(lnOne[0] - lnOne[2], lnOne[1] - lnOne[3], &posVecX, &posVecY);
  if (resolvedSlotClass == APA_SLOT_CLASS_PARALLEL) {
    enterBiasLen = 0.0f;
    transitionBiasLen = 1.0f;
  } else {
    enterBiasLen = enterPosLen;
    transitionBiasLen = transitionPosLen;
  }

  enterPos[0] = tarPos[0] + (posVecX * enterBiasLen);
  enterPos[1] = tarPos[1] + (posVecY * enterBiasLen);
  transitionPos[0] = tarPos[0] + (posVecX * transitionBiasLen);
  transitionPos[1] = tarPos[1] + (posVecY * transitionBiasLen);
}

/* @source matlab_rewrite ClassSlot.m#calcVehInSlotFlag */
static inline Boolean apaSlotCalcVehInSlotFlag(const Real vehPos[3],
                                               const Real slotPts[8],
                                               const Real slotBounds[4],
                                               const ApaPlannerVehConfig_t *vehConfig) {
  Real localBounds[4];
  const Real *resolvedBounds;
  Real frontX;
  Real frontY;
  Boolean rearInSlot;

  resolvedBounds = slotBounds;
  if (resolvedBounds == 0) {
    apaSlotCalcSlotBounds(slotPts, localBounds);
    resolvedBounds = localBounds;
  }

  rearInSlot = apaSlotPointInSlotWithBounds(slotPts, resolvedBounds, vehPos[0], vehPos[1]);
  if (rearInSlot) {
    return true;
  }

  apaVehicleFrontAxlePt(vehPos[0], vehPos[1], vehPos[2], vehConfig->WB, &frontX, &frontY);
  return apaSlotPointInSlotWithBounds(slotPts, resolvedBounds, frontX, frontY);
}

/* @source matlab_rewrite ClassSlot.m#calcVehNearSlotFlag */
static inline Boolean apaSlotCalcVehNearSlotFlag(const Real vehPos[3],
                                                 const Real slotPts[8],
                                                 const Real slotBounds[4],
                                                 const ApaPlannerVehConfig_t *vehConfig) {
  Real localBounds[4];
  const Real *resolvedBounds;
  Real rectEdgesX1[4];
  Real rectEdgesY1[4];
  Real rectEdgesX2[4];
  Real rectEdgesY2[4];
  Integer edgeIdx;
  Boolean inSlot;

  resolvedBounds = slotBounds;
  if (resolvedBounds == 0) {
    apaSlotCalcSlotBounds(slotPts, localBounds);
    resolvedBounds = localBounds;
  }

  inSlot = apaSlotCalcVehInSlotFlag(vehPos, slotPts, resolvedBounds, vehConfig);
  if (inSlot) {
    return true;
  }

  apaVehicleRect(vehPos[0],
                 vehPos[1],
                 vehPos[2],
                 vehConfig->LF,
                 vehConfig->LB,
                 0.5f * vehConfig->W,
                 rectEdgesX1,
                 rectEdgesY1,
                 rectEdgesX2,
                 rectEdgesY2);

  edgeIdx = 0;
  while (edgeIdx < 4) {
    if (apaGeoLineIntersectRect(rectEdgesX1[edgeIdx],
                                rectEdgesY1[edgeIdx],
                                rectEdgesX2[edgeIdx],
                                rectEdgesY2[edgeIdx],
                                slotPts)) {
      return true;
    }
    edgeIdx = edgeIdx + 1;
  }
  return false;
}

static inline void
apaSlotTranslateRect(const Real rectIn[8], Real deltaX, Real deltaY, Real rectOut[8]) {
  Integer idx;

  idx = 0;
  while (idx < 8) {
    rectOut[idx] = rectIn[idx] + deltaX;
    rectOut[idx + 1] = rectIn[idx + 1] + deltaY;
    idx = idx + 2;
  }
}

static inline void apaSlotFillRectCollisionInput(const RectCollisionCheckForBoundaryInput_t *obs,
                                                 const Real rectPts[8],
                                                 RectCollisionCheckForBoundaryInput_t *rectInput) {
  Integer itemCount;
  Integer idx;

  memset(rectInput, 0, sizeof(*rectInput));
  rectInput->RectX1 = rectPts[0];
  rectInput->RectY1 = rectPts[1];
  rectInput->RectX2 = rectPts[2];
  rectInput->RectY2 = rectPts[3];
  rectInput->RectX3 = rectPts[4];
  rectInput->RectY3 = rectPts[5];
  rectInput->RectX4 = rectPts[6];
  rectInput->RectY4 = rectPts[7];

  itemCount = obs->BoundaryLineNum;
  if (itemCount > MAX_BOUNDARY_LINES) {
    itemCount = MAX_BOUNDARY_LINES;
  }
  rectInput->BoundaryLineNum = itemCount;
  for (idx = 0; idx < itemCount; idx = idx + 1) {
    rectInput->BoundaryLineX1[idx] = obs->BoundaryLineX1[idx];
    rectInput->BoundaryLineY1[idx] = obs->BoundaryLineY1[idx];
    rectInput->BoundaryLineX2[idx] = obs->BoundaryLineX2[idx];
    rectInput->BoundaryLineY2[idx] = obs->BoundaryLineY2[idx];
  }

  itemCount = obs->SeparatePtsNum;
  if (itemCount > MAX_SEPARATE_PTS) {
    itemCount = MAX_SEPARATE_PTS;
  }
  rectInput->SeparatePtsNum = itemCount;
  for (idx = 0; idx < itemCount; idx = idx + 1) {
    rectInput->SeparatePtsX[idx] = obs->SeparatePtsX[idx];
    rectInput->SeparatePtsY[idx] = obs->SeparatePtsY[idx];
  }
}

static inline Boolean
apaSlotRectCollidesWithBoundary(const Real rectPts[8],
                                const RectCollisionCheckForBoundaryInput_t *obs) {
  RectCollisionCheckForBoundaryInput_t rectInput;
  RectCollisionCheckForBoundaryOutput_t rectOutput;

  apaSlotFillRectCollisionInput(obs, rectPts, &rectInput);
  rectOutput = (RectCollisionCheckForBoundaryOutput_t){0};
  rectCollisionCheckForBoundary(&rectInput, &rectOutput);
  return rectOutput.IsCollision;
}

/**
 * @brief 计算车位对应通道宽度 — 对应 ClassSlot.calcCorridorWidth()
 */
static inline void apaSlotCalcCorridorWidth(const Real slotPts[8],
                                            Integer enterEdge,
                                            const Real tarPos[3],
                                            Integer slotClass,
                                            const RectCollisionCheckForBoundaryInput_t *obs,
                                            Integer *corridorFlag,
                                            Real *corridorWidth) {
  Real lnOne[4];
  Real lnTwo[4];
  Real vecLonX;
  Real vecLonY;
  Real vecLatX;
  Real vecLatY;
  Real sideEdgeLen;
  Real rectPts[8];
  Real rectPtsTemp[8];
  Real cropLen;
  Real lMax;
  Real lBias;
  Real lStep;
  Real width;
  Integer resolvedSlotClass;
  Boolean loopDone;
  Boolean collided;

  resolvedSlotClass = slotClass;
  if (resolvedSlotClass == APA_SLOT_CLASS_UNKNOWN) {
    resolvedSlotClass = apaSlotCalcSlotClass(slotPts, enterEdge);
  }
  apaSlotCalcSideLine(slotPts, enterEdge, tarPos, resolvedSlotClass, lnOne, lnTwo);

  apaMathNormalize2D(lnOne[0] - lnOne[2], lnOne[1] - lnOne[3], &vecLonX, &vecLonY);
  vecLatX = lnOne[0] - lnTwo[0];
  vecLatY = lnOne[1] - lnTwo[1];
  sideEdgeLen = apaMathNorm2D(lnOne[0] - lnOne[2], lnOne[1] - lnOne[3]);

  rectPts[0] = lnOne[0];
  rectPts[1] = lnOne[1];
  rectPts[2] = lnOne[2];
  rectPts[3] = lnOne[3];
  rectPts[4] = lnTwo[2];
  rectPts[5] = lnTwo[3];
  rectPts[6] = lnTwo[0];
  rectPts[7] = lnTwo[1];

  cropLen = sideEdgeLen - 0.2f;
  rectPts[2] = rectPts[2] + (vecLonX * cropLen);
  rectPts[3] = rectPts[3] + (vecLonY * cropLen);
  rectPts[4] = rectPts[4] + (vecLonX * cropLen);
  rectPts[5] = rectPts[5] + (vecLonY * cropLen);
  rectPts[0] = rectPts[0] + vecLatX;
  rectPts[1] = rectPts[1] + vecLatY;
  rectPts[2] = rectPts[2] + vecLatX;
  rectPts[3] = rectPts[3] + vecLatY;

  if (resolvedSlotClass == APA_SLOT_CLASS_VERTICAL) {
    rectPts[4] = rectPts[4] - vecLatX;
    rectPts[5] = rectPts[5] - vecLatY;
    rectPts[6] = rectPts[6] - vecLatX;
    rectPts[7] = rectPts[7] - vecLatY;
    lMax = 8.0f;
    lBias = 4.0f;
  } else {
    lMax = 6.0f;
    lBias = 3.0f;
  }

  width = lBias;
  lStep = 0.2f;
  loopDone = false;
  while (apaMathLess(lBias, lMax) && (!loopDone)) {
    apaSlotTranslateRect(rectPts, vecLonX * lBias, vecLonY * lBias, rectPtsTemp);
    collided = apaSlotRectCollidesWithBoundary(rectPtsTemp, obs);
    if (collided) {
      loopDone = true;
    } else {
      width = lBias;
      lBias = lBias + lStep;
    }
  }

  *corridorWidth = width;
  if (resolvedSlotClass == APA_SLOT_CLASS_VERTICAL) {
    if (apaMathLess(width, 5.0f)) {
      *corridorFlag = 0;
    } else if (apaMathGreater(width, 7.0f)) {
      *corridorFlag = 2;
    } else {
      *corridorFlag = 1;
    }
  } else {
    if (apaMathLess(width, 4.0f)) {
      *corridorFlag = 0;
    } else if (apaMathGreater(width, 5.0f)) {
      *corridorFlag = 2;
    } else {
      *corridorFlag = 1;
    }
  }
}

/**
 * @brief 自定义车位是否可用 — 对应 ClassSlot.isHandleSlotAvailable()
 */
static inline Boolean
apaSlotIsHandleSlotAvailable(const Real slotPts[8],
                             Integer enterEdge,
                             const Real tarPos[3],
                             Integer slotClass,
                             const Real vehPos[3],
                             Real vehLength,
                             Real vehRearOffset,
                             const RectCollisionCheckForBoundaryInput_t *obs) {
  Real lnOne[4];
  Real lnTwo[4];
  Real enterLine[4];
  Real enterLineCenterX;
  Real enterLineCenterY;
  Real geoCenterDelta;
  Real geoCenterX;
  Real geoCenterY;
  Real geoCenterTheta;
  Real moveX;
  Real moveY;
  Real connectArea[8];
  Integer resolvedSlotClass;
  Boolean collision;

  resolvedSlotClass = slotClass;
  if (resolvedSlotClass == APA_SLOT_CLASS_UNKNOWN) {
    resolvedSlotClass = apaSlotCalcSlotClass(slotPts, enterEdge);
  }
  apaSlotCalcSideLine(slotPts, enterEdge, tarPos, resolvedSlotClass, lnOne, lnTwo);

  enterLine[0] = (lnOne[0] + lnOne[2]) * 0.5f;
  enterLine[1] = (lnOne[1] + lnOne[3]) * 0.5f;
  enterLine[2] = (lnTwo[0] + lnTwo[2]) * 0.5f;
  enterLine[3] = (lnTwo[1] + lnTwo[3]) * 0.5f;
  enterLineCenterX = (enterLine[0] + enterLine[2]) * 0.5f;
  enterLineCenterY = (enterLine[1] + enterLine[3]) * 0.5f;

  geoCenterDelta = (vehLength * 0.5f) - vehRearOffset;
  apaCoordVeh2Ground(vehPos[0],
                     vehPos[1],
                     vehPos[2],
                     geoCenterDelta,
                     0.0f,
                     0.0f,
                     &geoCenterX,
                     &geoCenterY,
                     &geoCenterTheta);
  moveX = geoCenterX - enterLineCenterX;
  moveY = geoCenterY - enterLineCenterY;

  connectArea[0] = enterLine[0];
  connectArea[1] = enterLine[1];
  connectArea[2] = enterLine[0] + moveX;
  connectArea[3] = enterLine[1] + moveY;
  connectArea[4] = enterLine[2] + moveX;
  connectArea[5] = enterLine[3] + moveY;
  connectArea[6] = enterLine[2];
  connectArea[7] = enterLine[3];

  collision = apaSlotRectCollidesWithBoundary(connectArea, obs);
  return !collision;
}

/**
 * @brief 非端头垂直车位一段入库判定 — 对应 ClassSlot.calcExpSegNum()
 */
static inline Boolean apaSlotExpSegVerticalOneStep(Integer parkingType, Integer surroundingType) {
  Boolean oneStep;

  oneStep = false;
  if (surroundingType == APA_SURROUND_OBS_EXIST_NONE) {
    oneStep = true;
  } else if (parkingType == APA_PARKING_TYPE_LEFT &&
             surroundingType == APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY) {
    oneStep = true;
  } else if (parkingType == APA_PARKING_TYPE_RIGHT &&
             surroundingType == APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY) {
    oneStep = true;
  }
  return oneStep;
}

/**
 * @brief 计算车位期望路径段数 — 对应 ClassSlot.calcExpSegNum()
 *
 * parking/surrounding/corridor/stopper 由调用方按当前 st_obs 和车辆位姿预先求值。
 */
static inline Integer apaSlotCalcExpectedSegNum(Integer slotClass,
                                                Integer parkingType,
                                                Integer surroundingType,
                                                Integer slotSubStatus,
                                                Integer slotStatus,
                                                Boolean existStopperFlag,
                                                Integer corridorFlag,
                                                Integer stepNow,
                                                Integer stepNum) {
  Integer segIni;
  Integer segRest;
  Integer segExp;
  Boolean oneStep;
  Boolean parallelOpen;

  segIni = 7;
  if (slotSubStatus == APA_SLOT_SUB_END_NONE && slotClass == APA_SLOT_CLASS_VERTICAL) {
    oneStep = apaSlotExpSegVerticalOneStep(parkingType, surroundingType);
    if (oneStep) {
      segIni = 1;
    } else {
      segIni = 3;
    }
    if (corridorFlag == 0) {
      segIni = segIni + 2;
    }
    if (parkingType == APA_PARKING_TYPE_MEANINGLESS && segIni > 2) {
      segIni = segIni - 2;
    }
  } else if (slotSubStatus == APA_SLOT_SUB_END_NONE && slotClass == APA_SLOT_CLASS_PARALLEL) {
    parallelOpen = false;
    if (surroundingType == APA_SURROUND_OBS_EXIST_NONE ||
        surroundingType == APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY) {
      parallelOpen = true;
    }
    if (parallelOpen) {
      segIni = 1;
    } else if (surroundingType == APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY && !existStopperFlag) {
      segIni = 2;
    } else {
      segIni = 5;
    }
  } else if (slotSubStatus != APA_SLOT_SUB_END_NONE && slotClass == APA_SLOT_CLASS_VERTICAL) {
    if (slotSubStatus == APA_SLOT_SUB_END_FIRST) {
      if (surroundingType == APA_SURROUND_OBS_EXIST_BOTH) {
        segIni = 9;
      } else {
        segIni = 7;
      }
    } else if (slotSubStatus == APA_SLOT_SUB_END_SECOND) {
      if (surroundingType == APA_SURROUND_OBS_EXIST_NONE) {
        segIni = 5;
      } else {
        segIni = 7;
      }
    } else if (slotSubStatus == APA_SLOT_SUB_END_THIRD) {
      oneStep = apaSlotExpSegVerticalOneStep(parkingType, surroundingType);
      if (oneStep) {
        segIni = 3;
      } else {
        segIni = 5;
      }
    } else {
      segIni = 3;
    }
  } else if (slotSubStatus != APA_SLOT_SUB_END_NONE && slotClass == APA_SLOT_CLASS_PARALLEL) {
    parallelOpen = false;
    if (surroundingType == APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY ||
        surroundingType == APA_SURROUND_OBS_EXIST_NONE) {
      parallelOpen = true;
    }
    if (parallelOpen && !existStopperFlag) {
      segIni = 5;
    } else {
      segIni = 7;
    }
  }

  segRest = stepNum - stepNow;
  if (segRest < 1) {
    segRest = 1;
  }

  if (slotStatus == APA_SLOT_STATUS_TARGET_SLOT_REPLAN) {
    segExp = segIni;
    if (segRest < segExp) {
      segExp = segRest;
    }
  } else if (slotStatus == APA_SLOT_STATUS_FAIL_PLAN) {
    segExp = segIni + 2;
  } else {
    segExp = segIni;
  }
  return segExp;
}

/**
 * @brief 计算最大允许段数 — 对应 ClassSlot.calcMaxAllowedSegNum()
 */
static inline Integer apaSlotCalcMaxAllowedSegNum(Integer expectedSegNum,
                                                  Integer slotSubStatus,
                                                  Integer slotStatus,
                                                  Integer replanReqType,
                                                  Integer stepNow,
                                                  Integer stepNum) {
  Integer nRest;
  Integer nMax;

  nRest = stepNum - stepNow;
  if (slotStatus == APA_SLOT_STATUS_TARGET_SLOT_REPLAN &&
      replanReqType == APA_REPLAN_REDUCE_PATH_SEG) {
    nMax = nRest;
  } else if (slotSubStatus == APA_SLOT_SUB_END_FIRST || slotSubStatus == APA_SLOT_SUB_END_SECOND) {
    nMax = expectedSegNum;
    if (nMax < 15) {
      nMax = 15;
    }
  } else if (slotSubStatus == APA_SLOT_SUB_END_THIRD) {
    nMax = expectedSegNum;
    if (nMax < 12) {
      nMax = 12;
    }
  } else {
    nMax = expectedSegNum;
    if (nMax < 9) {
      nMax = 9;
    }
  }
  return nMax;
}

/**
 * @brief 惰性获取车位分类 — 对应 ClassSlot.getSlotClass()
 */
static inline Integer apaSlotGetSlotClass(ApaSlot_t *slot) {
  if (slot->SlotClass != APA_SLOT_CLASS_UNKNOWN) {
    return slot->SlotClass;
  }
  slot->SlotClass = apaSlotCalcSlotClass(slot->SlotPts, slot->EnterEdge);
  return slot->SlotClass;
}

/** @brief 惰性获取泊车方向 — 对应 ClassSlot.getParkingDirec() */
static inline Integer apaSlotGetParkingDirec(ApaSlot_t *slot) {
  if (slot->ParkingDirec != APA_PARKING_DIREC_INVALID) {
    return slot->ParkingDirec;
  }
  slot->ParkingDirec = apaSlotCalcParkingDirec(slot->SlotPts, slot->EnterEdge, slot->TarPos);
  return slot->ParkingDirec;
}

/** @brief 计算入库入口/过渡位姿 — 对应 ClassSlot.getParkingPos() */
static inline void apaSlotGetParkingPos(ApaSlot_t *slot,
                                        Real enterPosLen,
                                        Real transitionPosLen,
                                        Real enterPos[3],
                                        Real transitionPos[3]) {
  Integer slotClass;

  slotClass = apaSlotGetSlotClass(slot);
  apaSlotCalcParkingPos(slot->SlotPts,
                        slot->EnterEdge,
                        slot->TarPos,
                        slotClass,
                        enterPosLen,
                        transitionPosLen,
                        enterPos,
                        transitionPos);
}

#endif
