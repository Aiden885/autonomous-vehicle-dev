/**
 * @file apaPathTypes.h
 * @brief APA 路径数据结构 — 对应 ClassPath.m / ClassComplexPath.m / ClassMultiComplexPath.m
 *
 * 将 MATLAB handle 类改写为 C99 struct 层次：
 *   ClassPath          → ApaPath_t           + apaPath_*  函数族
 *   ClassComplexPath   → ApaComplexPath_t    + apaCPath_* 函数族
 *   ClassMultiComplexPath → ApaMultiCPath_t  + apaMCPath_* 函数族
 *
 * @date 2026-04-13
 */
#ifndef APA_PATH_TYPES_H
#define APA_PATH_TYPES_H

#include "baseTypes.h"
#include "apaVehicleDefaults.h"
#include "apaEnums.h"
#include "apaMathUtils.h"
#include "apaCoordUtils.h"
#include "apaGeoUtils.h"
#include "apaPlannerUtils.h"
#include "apaSlotTypes.h"

/** @brief 单条路径最大点数 — 对应 MATLAB zeros(1, 100) */
#define APA_PATH_MAX_POINTS 100

/** @brief ComplexPath 中 open/close 列表容量 */
#define APA_CPATH_OPEN_CAPACITY 1000
#define APA_CPATH_CLOSE_CAPACITY 1000
#define APA_CPATH_NODE_FIELD_COUNT 5

/** @brief MultiComplexPath 中最大路径数 */
#define APA_MCPATH_MAX_PATHS 20

/** @brief 强制方向首段最小里程 — 对应 conf.expect_direction_odo_thresh 默认值 */
#define APA_PATH_EXPECT_DIRECTION_ODO_THRESH 0.15f

/**
 * @brief 单条路径 — 对应 ClassPath properties (14个字段)
 *
 * x_/y_/tht_/odo_step_/radius_ 以定长数组存储，path_n_ 指示有效长度。
 */
typedef struct {
  /* 公开属性 */
  Boolean ValidFlag;                 /**< 路径是否有效 */
  Integer ErrorType;                 /**< 故障码 ApaPathPlanFault_e */
  Real X[APA_PATH_MAX_POINTS];       /**< x坐标序列 */
  Real Y[APA_PATH_MAX_POINTS];       /**< y坐标序列 */
  Real Tht[APA_PATH_MAX_POINTS];     /**< 航向序列 [rad] */
  Real OdoStep[APA_PATH_MAX_POINTS]; /**< 里程步进序列 */
  Integer PathN;                     /**< 有效点数 */
  Integer PathType;                  /**< 路径类型 ApaPathType_e */
  Integer PathSource;                /**< 路径来源 ApaPathSource_e */
  /* 私有/缓存属性 */
  Integer CollisionFlag;            /**< 碰撞标志 ApaCollisionRisk_e */
  Boolean RadiusFlag;               /**< 曲率半径是否已计算 */
  Real Radius[APA_PATH_MAX_POINTS]; /**< 曲率半径序列 */
  Boolean SegNumFlag;               /**< 段数是否已计算 */
  Integer SegNum;                   /**< 路径段数 */
} ApaPath_t;

/** @brief 初始化路径为默认值 — 对应 ClassPath() 无参构造 */
static inline void apaPathInit(ApaPath_t *p) {
  Integer i;

  p->ValidFlag = false;
  p->ErrorType = APA_FAULT_UNKNOWN_ERR;
  p->PathN = 0;
  p->PathType = APA_PATH_TYPE_UNKNOWN;
  p->PathSource = APA_PATH_SOURCE_INVALID;
  p->CollisionFlag = APA_COLLISION_RISK_UNKNOWN;
  p->RadiusFlag = false;
  p->SegNumFlag = false;
  p->SegNum = 0;

  for (i = 0; i < APA_PATH_MAX_POINTS; i = i + 1) {
    p->X[i] = 0.0f;
    p->Y[i] = 0.0f;
    p->Tht[i] = 0.0f;
    p->OdoStep[i] = 0.0f;
    p->Radius[i] = 0.0f;
  }
}

/**
 * @brief Sum absolute odometry between two path point indices.
 *
 * Requires 0 <= startIndex <= endIndex <= path->PathN.
 */
static inline Real
apaPathTravelToPoint(const ApaPath_t *path, Integer startIndex, Integer endIndex) {
  Real travel;
  Integer idx;

  travel = 0.0f;
  idx = startIndex;
  while (idx < endIndex) {
    travel = travel + fabsf(path->OdoStep[idx]);
    idx = idx + 1;
  }
  return travel;
}

/**
 * @brief 从数组构造路径 — 对应 Path_St_Gen.m 的 struct 字段拷贝语义
 */
static inline void apaPathInitFromArrays(ApaPath_t *p,
                                         const Real *pathPtsX,
                                         const Real *pathPtsY,
                                         const Real *pathPtsTh,
                                         const Real *pathPtsOdoStep,
                                         const Real *pathPtsRadius,
                                         Integer pathPtsN,
                                         Integer collisionRiskType) {
  Integer idx;
  Integer count;

  apaPathInit(p);

  count = pathPtsN;
  if (count < 0) {
    count = 0;
  }
  if (count > APA_PATH_MAX_POINTS) {
    count = APA_PATH_MAX_POINTS;
  }

  p->PathN = count;
  p->ValidFlag = (count > 0);
  p->ErrorType = APA_FAULT_NO_ERR;
  p->CollisionFlag = collisionRiskType;
  p->RadiusFlag = (pathPtsRadius != (const Real *)0);

  for (idx = 0; idx < count; idx = idx + 1) {
    p->X[idx] = pathPtsX[idx];
    p->Y[idx] = pathPtsY[idx];
    p->Tht[idx] = pathPtsTh[idx];
    p->OdoStep[idx] = pathPtsOdoStep[idx];
    if (pathPtsRadius != (const Real *)0) {
      p->Radius[idx] = pathPtsRadius[idx];
    }
  }
}

/* @source matlab_rewrite ClassPath.m#constructor(9 args) */
static inline void apaPathInitFromPublicParams(ApaPath_t *p,
                                               Boolean validFlag,
                                               Integer errorType,
                                               const Real *pathX,
                                               const Real *pathY,
                                               const Real *pathTht,
                                               const Real *pathOdoStep,
                                               Integer pathN,
                                               Integer pathType,
                                               Integer pathSource) {
  Integer idx;
  Integer count;

  apaPathInit(p);
  count = pathN;
  if (count < 0) {
    count = 0;
  }
  if (count > APA_PATH_MAX_POINTS) {
    count = APA_PATH_MAX_POINTS;
  }

  p->ValidFlag = validFlag;
  p->ErrorType = errorType;
  p->PathN = count;
  p->PathType = pathType;
  p->PathSource = pathSource;
  for (idx = 0; idx < count; idx = idx + 1) {
    p->X[idx] = pathX[idx];
    p->Y[idx] = pathY[idx];
    p->Tht[idx] = pathTht[idx];
    p->OdoStep[idx] = pathOdoStep[idx];
  }
}

/* @source matlab_rewrite ClassPath.m#constructor(14 args) */
static inline void apaPathInitFromFullParams(ApaPath_t *p,
                                             Boolean validFlag,
                                             Integer errorType,
                                             const Real *pathX,
                                             const Real *pathY,
                                             const Real *pathTht,
                                             const Real *pathOdoStep,
                                             Integer pathN,
                                             Integer pathType,
                                             Integer pathSource,
                                             Integer collisionFlag,
                                             Boolean radiusFlag,
                                             const Real *pathRadius,
                                             Boolean segNumFlag,
                                             Integer segNum) {
  Integer idx;

  apaPathInitFromPublicParams(
      p, validFlag, errorType, pathX, pathY, pathTht, pathOdoStep, pathN, pathType, pathSource);
  p->CollisionFlag = collisionFlag;
  p->RadiusFlag = radiusFlag;
  p->SegNumFlag = segNumFlag;
  p->SegNum = segNum;
  if (pathRadius != (const Real *)0) {
    for (idx = 0; idx < p->PathN; idx = idx + 1) {
      p->Radius[idx] = pathRadius[idx];
    }
  }
}

/* @source matlab_rewrite ClassPath.m#getPathParm */
static inline void apaPathExportParams(const ApaPath_t *p, ApaPath_t *out) {
  Integer idx;

  if (p != (const ApaPath_t *)0 && out != (ApaPath_t *)0) {
    out->ValidFlag = p->ValidFlag;
    out->ErrorType = p->ErrorType;
    out->PathN = p->PathN;
    out->PathType = p->PathType;
    out->PathSource = p->PathSource;
    out->CollisionFlag = p->CollisionFlag;
    out->RadiusFlag = p->RadiusFlag;
    out->SegNumFlag = p->SegNumFlag;
    out->SegNum = p->SegNum;

    for (idx = 0; idx < APA_PATH_MAX_POINTS; idx = idx + 1) {
      out->X[idx] = p->X[idx];
      out->Y[idx] = p->Y[idx];
      out->Tht[idx] = p->Tht[idx];
      out->OdoStep[idx] = p->OdoStep[idx];
      out->Radius[idx] = p->Radius[idx];
    }
  }
}

/** @brief 拷贝路径 — 对应 ClassPath.assign() */
static inline void apaPathCopy(ApaPath_t *dst, const ApaPath_t *src) {
  Integer i;
  Integer n;

  dst->ValidFlag = src->ValidFlag;
  dst->ErrorType = src->ErrorType;
  dst->PathN = src->PathN;
  dst->PathType = src->PathType;
  dst->PathSource = src->PathSource;
  dst->CollisionFlag = src->CollisionFlag;
  dst->RadiusFlag = src->RadiusFlag;
  dst->SegNumFlag = src->SegNumFlag;
  dst->SegNum = src->SegNum;

  n = src->PathN;
  if (n > APA_PATH_MAX_POINTS) {
    n = APA_PATH_MAX_POINTS;
  }
  for (i = 0; i < n; i = i + 1) {
    dst->X[i] = src->X[i];
    dst->Y[i] = src->Y[i];
    dst->Tht[i] = src->Tht[i];
    dst->OdoStep[i] = src->OdoStep[i];
    dst->Radius[i] = src->Radius[i];
  }

  if (n > 0) {
    dst->OdoStep[n - 1] = 0.0f;
  }
}

/** @brief 清空路径 — 对应 ClassPath.clear() */
static inline void apaPathClear(ApaPath_t *p) {
  p->ValidFlag = false;
  p->ErrorType = APA_FAULT_UNKNOWN_ERR;
  p->PathN = 0;
  p->CollisionFlag = APA_COLLISION_RISK_UNKNOWN;
  p->RadiusFlag = false;
  p->SegNumFlag = false;
}

/** @brief 重置缓存标志 — 对应 ClassPath.resetFlags() */
static inline void apaPathResetFlags(ApaPath_t *p) {
  p->CollisionFlag = APA_COLLISION_RISK_UNKNOWN;
  p->RadiusFlag = false;
  p->SegNumFlag = false;
}

/** @brief 路径翻转 — 对应 ClassPath.reverse() */
static inline void apaPathReverse(ApaPath_t *p) {
  Integer i;
  Integer n;
  Integer dstIdx;
  ApaPath_t pathOri;

  n = p->PathN;
  if (n > 1) {
    apaPathCopy(&pathOri, p);
    for (i = 0; i < n; i = i + 1) {
      dstIdx = n - 1 - i;
      p->X[dstIdx] = pathOri.X[i];
      p->Y[dstIdx] = pathOri.Y[i];
      p->Tht[dstIdx] = pathOri.Tht[i];
      if (i > 0) {
        p->OdoStep[dstIdx] = -pathOri.OdoStep[i - 1];
      }
    }
    p->OdoStep[n - 1] = 0.0f;
    p->RadiusFlag = false;
  }
}

/**
 * @brief 路径拼接 — 对应 ClassPath.addPath()
 *
 * 将 other 的路径点附加到 self 尾部。
 */
static inline void apaPathAppend(ApaPath_t *self, const ApaPath_t *other) {
  Integer srcIdx;
  Integer selfN;
  Integer otherN;
  Integer total;
  Boolean appendAllowed;
  Real endStartDist;

  selfN = self->PathN;
  otherN = other->PathN;

  total = selfN + otherN - 1;
  appendAllowed = true;
  if (selfN <= 0 || otherN <= 0) {
    appendAllowed = false;
  }
  if (appendAllowed && total > APA_PATH_MAX_POINTS) {
    appendAllowed = false;
  }
  if (appendAllowed) {
    endStartDist =
        apaMathNorm2D(other->X[0] - self->X[selfN - 1], other->Y[0] - self->Y[selfN - 1]);
    if (!apaMathEqual(endStartDist, 0.0f)) {
      appendAllowed = false;
    }
  }

  if (appendAllowed) {
    for (srcIdx = 1; srcIdx < otherN; srcIdx = srcIdx + 1) {
      Integer dstPointIdx;
      Integer dstOdoIdx;

      dstPointIdx = self->PathN;
      dstOdoIdx = dstPointIdx - 1;
      self->X[dstPointIdx] = other->X[srcIdx];
      self->Y[dstPointIdx] = other->Y[srcIdx];
      self->Tht[dstPointIdx] = other->Tht[srcIdx];
      self->OdoStep[dstOdoIdx] = other->OdoStep[srcIdx - 1];
      self->PathN = self->PathN + 1;
    }
    if (self->PathN > 0) {
      self->OdoStep[self->PathN - 1] = 0.0f;
    }
    apaPathResetFlags(self);
  }
}

/**
 * @brief 路径交换 — 对应 ClassPath.swap()
 */
static inline void apaPathSwap(ApaPath_t *a, ApaPath_t *b) {
  ApaPath_t tmp;

  tmp = *a;
  *a = *b;
  *b = tmp;
}

/**
 * @brief 计算段数 — 对应 ClassPath.getSegNum() 的核心逻辑
 *
 * 统计里程符号翻转次数。
 */
static inline Integer apaPathCalcSegNum(const ApaPath_t *p) {
  Integer segNum;
  Integer i;

  if (p->PathN < 2) {
    return 0;
  }
  if (apaMathEqual(p->OdoStep[0], 0.0f)) {
    return 30;
  }

  segNum = 1;
  for (i = 0; i < p->PathN - 2; i = i + 1) {
    Real product;

    product = p->OdoStep[i] * p->OdoStep[i + 1];
    if (apaMathLess(product, 0.0f)) {
      segNum = segNum + 1;
    }
  }
  return segNum;
}

/**
 * @brief 计算换向点和分段里程 — 对应 ClassPath.calcSegNumAndRevPtr()
 *
 * C 端使用 0 基下标；revPtsPtrList[0] 是路径起点，最后一个元素是
 * PathN - 1。segOdoList[i] 对应第 i 段的带符号累计 odo。
 */
static inline void apaPathCalcSegNumAndRevPtr(const ApaPath_t *p,
                                              Integer *segNum,
                                              Integer revPtsPtrList[APA_PATH_MAX_POINTS],
                                              Real segOdoList[APA_PATH_MAX_POINTS],
                                              Integer *revPtsNum) {
  Integer idx;
  Integer revCount;
  Integer segCount;

  segCount = 1;
  revCount = 1;
  *segNum = segCount;
  *revPtsNum = revCount;

  for (idx = 0; idx < APA_PATH_MAX_POINTS; idx = idx + 1) {
    revPtsPtrList[idx] = 0;
    segOdoList[idx] = 0.0f;
  }

  if (p->PathN <= 0) {
    return;
  }
  if (apaMathEqual(p->OdoStep[0], 0.0f)) {
    return;
  }

  revPtsPtrList[0] = 0;
  segOdoList[0] = p->OdoStep[0];

  for (idx = 0; idx < p->PathN - 2; idx = idx + 1) {
    Real product;

    product = p->OdoStep[idx] * p->OdoStep[idx + 1];
    if (apaMathLess(product, 0.0f)) {
      segCount = segCount + 1;
      revCount = revCount + 1;
      if (revCount >= APA_PATH_MAX_POINTS) {
        *segNum = segCount;
        *revPtsNum = revCount;
        return;
      }
      revPtsPtrList[revCount - 1] = idx + 1;
    }
    segOdoList[revCount - 1] = segOdoList[revCount - 1] + p->OdoStep[idx + 1];
  }

  revCount = revCount + 1;
  if (revCount >= APA_PATH_MAX_POINTS) {
    *segNum = segCount;
    *revPtsNum = revCount;
    return;
  }
  revPtsPtrList[revCount - 1] = p->PathN - 1;
  *segNum = segCount;
  *revPtsNum = revCount;
}

/**
 * @brief 结合车辆运动状态修正路径段数 — 对应 ClassPath.getSegNumAndRevPtrForMotion()
 */
static inline Integer apaPathCalcSegNumForMotion(const ApaPath_t *p,
                                                 Real vehVel,
                                                 Integer gearPos,
                                                 Integer parkingSysMainState) {
  Integer segNum;
  Integer revPtsNum;
  Integer revPtsPtrList[APA_PATH_MAX_POINTS];
  Real segOdoList[APA_PATH_MAX_POINTS];
  Integer segNumForMotion;
  Boolean pathForwardFlag;
  Boolean vehStoppedFlag;

  segNum = 0;
  revPtsNum = 0;
  apaPathCalcSegNumAndRevPtr(p, &segNum, revPtsPtrList, segOdoList, &revPtsNum);
  (void)revPtsPtrList;
  (void)segOdoList;
  (void)revPtsNum;

  segNumForMotion = segNum;
  if (p->PathType != APA_PATH_TYPE_TERMINATION) {
    pathForwardFlag = apaMathGreater(p->OdoStep[0], 0.0f);
    vehStoppedFlag = apaMathLess(vehVel, 0.2f);
    if (parkingSysMainState == APA_SYS_STATE_SEARCHING) {
      if (pathForwardFlag && vehStoppedFlag) {
        segNumForMotion = segNumForMotion - 1;
      }
    } else if (parkingSysMainState == APA_SYS_STATE_PARKING) {
      if (gearPos == APA_GEAR_POS_D) {
        if (pathForwardFlag) {
          segNumForMotion = segNumForMotion - 1;
        }
      } else if (gearPos == APA_GEAR_POS_R) {
        if (!pathForwardFlag) {
          segNumForMotion = segNumForMotion - 1;
        }
      }
    }
    if (segNumForMotion < 0) {
      segNumForMotion = 0;
    }
  }
  return segNumForMotion;
}

/**
 * @brief 校验路径段数上限 — 对应 ClassPath.pathSegNumValidCheck()
 *
 * maxAllowedSegNum 由调用方按当前 slot_obj.calcMaxAllowedSegNum() 语义预先求值。
 */
static inline Boolean apaPathSegNumValidCheck(const ApaPath_t *p,
                                              Integer maxAllowedSegNum,
                                              Real vehVel,
                                              Integer gearPos,
                                              Integer parkingSysMainState) {
  Integer segNum;
  Boolean valid;

  segNum = apaPathCalcSegNumForMotion(p, vehVel, gearPos, parkingSysMainState);
  valid = true;
  if (segNum > maxAllowedSegNum) {
    valid = false;
  }
  return valid;
}

static inline void apaPathClearSegmentRows(Real rows[APA_PATH_MAX_POINTS][4]) {
  Integer row;
  Integer col;

  for (row = 0; row < APA_PATH_MAX_POINTS; row = row + 1) {
    for (col = 0; col < 4; col = col + 1) {
      rows[row][col] = 0.0f;
    }
  }
}

static inline Boolean apaPathCollinearCompressRows(const Real inputRows[APA_PATH_MAX_POINTS][4],
                                                   Integer inputCount,
                                                   Real collinearThreshold,
                                                   Real outputRows[APA_PATH_MAX_POINTS][4],
                                                   Integer *outputCount) {
  Boolean valid[APA_PATH_MAX_POINTS];
  Integer ptrLeft;
  Integer ptrRight;
  Integer idx;
  Integer col;
  Integer writeIndex;

  if (inputCount < 0 || inputCount > APA_PATH_MAX_POINTS) {
    *outputCount = 0;
    return false;
  }

  for (idx = 0; idx < APA_PATH_MAX_POINTS; idx = idx + 1) {
    valid[idx] = false;
    for (col = 0; col < 4; col = col + 1) {
      outputRows[idx][col] = 0.0f;
    }
  }
  for (idx = 0; idx < inputCount; idx = idx + 1) {
    valid[idx] = true;
  }

  ptrLeft = 0;
  ptrRight = 1;
  while (ptrRight < inputCount) {
    Real slopeK;
    Real interceptB;
    Integer ptr;
    Boolean scanOpen;

    apaGeoLinearFuncSolve(inputRows[ptrLeft][0],
                          inputRows[ptrLeft][1],
                          inputRows[ptrRight][0],
                          inputRows[ptrRight][1],
                          &slopeK,
                          &interceptB);
    ptr = ptrRight + 1;
    scanOpen = (ptr < inputCount);
    while (scanOpen) {
      Real distance;

      distance = apaGeoPointLineDistanceFromSlopeIntercept(
          inputRows[ptr][0], inputRows[ptr][1], slopeK, interceptB);
      if (apaMathLess(distance, collinearThreshold)) {
        valid[ptr - 1] = false;
        ptr = ptr + 1;
        scanOpen = (ptr < inputCount);
      } else {
        scanOpen = false;
      }
    }
    ptrLeft = ptr - 1;
    ptrRight = ptr;
    if (ptrLeft < 0 || ptrRight < 0) {
      *outputCount = 0;
      return false;
    }
  }

  writeIndex = 0;
  for (idx = 0; idx < inputCount; idx = idx + 1) {
    if (valid[idx]) {
      for (col = 0; col < 4; col = col + 1) {
        outputRows[writeIndex][col] = inputRows[idx][col];
      }
      writeIndex = writeIndex + 1;
    }
  }
  *outputCount = writeIndex;
  return true;
}

/**
 * @brief 压缩直线段路径点 — 对应 ClassPath.simplifyStraightPath()
 *
 * 保留每个换向段的起终点，并按 MATLAB 的三段窗口规则在累计 odo
 * 超过 min(abs(seg_odo)/3, odoThresh) 后执行共线压缩。
 */
static inline void apaPathSimplifyStraightPath(ApaPath_t *p, Real collinearThresh, Real odoThresh) {
  Integer revPtsPtrList[APA_PATH_MAX_POINTS];
  Real segOdoList[APA_PATH_MAX_POINTS];
  Integer segNum;
  Integer revPtsNum;
  Real pathX[APA_PATH_MAX_POINTS];
  Real pathY[APA_PATH_MAX_POINTS];
  Real pathTht[APA_PATH_MAX_POINTS];
  Real pathOdoStep[APA_PATH_MAX_POINTS];
  Real segmentRows[APA_PATH_MAX_POINTS][4];
  Real simplifiedRows[APA_PATH_MAX_POINTS][4];
  Integer pathN;
  Integer idx;
  Integer segIdx;
  const Integer nMidPt = 3;

  if (!p->ValidFlag) {
    return;
  }
  if (p->PathN <= 0) {
    return;
  }

  apaPathCalcSegNumAndRevPtr(p, &segNum, revPtsPtrList, segOdoList, &revPtsNum);
  (void)segNum;
  if (revPtsNum <= 1) {
    return;
  }

  for (idx = 0; idx < APA_PATH_MAX_POINTS; idx = idx + 1) {
    pathX[idx] = 0.0f;
    pathY[idx] = 0.0f;
    pathTht[idx] = 0.0f;
    pathOdoStep[idx] = 0.0f;
  }

  pathN = 1;
  pathX[0] = p->X[0];
  pathY[0] = p->Y[0];
  pathTht[0] = p->Tht[0];
  pathOdoStep[0] = p->OdoStep[0];

  for (segIdx = 0; segIdx < revPtsNum - 1; segIdx = segIdx + 1) {
    Integer ptrStart;
    Integer ptrEnd;
    Integer j;
    Integer segmentCount;
    Real segmentOdo;
    Real odoSegThresh;

    ptrStart = revPtsPtrList[segIdx];
    ptrEnd = revPtsPtrList[segIdx + 1];
    if (ptrEnd - ptrStart < nMidPt + 1) {
      for (j = ptrStart + 1; j <= ptrEnd; j = j + 1) {
        if (pathN < APA_PATH_MAX_POINTS) {
          pathX[pathN] = p->X[j];
          pathY[pathN] = p->Y[j];
          pathTht[pathN] = p->Tht[j];
          pathOdoStep[pathN] = p->OdoStep[j];
          pathN = pathN + 1;
        }
      }
    } else {
      odoSegThresh = segOdoList[segIdx];
      if (odoSegThresh < 0.0f) {
        odoSegThresh = -odoSegThresh;
      }
      odoSegThresh = odoSegThresh / (Real)nMidPt;
      if (odoThresh < odoSegThresh) {
        odoSegThresh = odoThresh;
      }

      apaPathClearSegmentRows(segmentRows);
      segmentCount = 0;
      segmentOdo = 0.0f;
      for (j = ptrStart; j <= ptrEnd; j = j + 1) {
        Integer calType;
        Real absOdo;

        if (segmentCount < APA_PATH_MAX_POINTS) {
          segmentRows[segmentCount][0] = p->X[j];
          segmentRows[segmentCount][1] = p->Y[j];
          segmentRows[segmentCount][2] = p->Tht[j];
          segmentRows[segmentCount][3] = p->OdoStep[j];
          segmentCount = segmentCount + 1;
        }

        absOdo = p->OdoStep[j];
        if (absOdo < 0.0f) {
          absOdo = -absOdo;
        }
        segmentOdo = segmentOdo + absOdo;

        calType = 0;
        if (j == ptrEnd) {
          calType = 1;
        } else {
          Real nextAbsOdo;

          nextAbsOdo = p->OdoStep[j + 1];
          if (nextAbsOdo < 0.0f) {
            nextAbsOdo = -nextAbsOdo;
          }
          if (apaMathGreater(segmentOdo + nextAbsOdo, odoSegThresh)) {
            calType = 2;
          }
        }

        if (calType != 0) {
          {
            Boolean compressOk;
            Integer simplifiedCount;
            Integer k;

            compressOk = apaPathCollinearCompressRows((const Real(*)[4])segmentRows,
                                                      segmentCount,
                                                      collinearThresh,
                                                      simplifiedRows,
                                                      &simplifiedCount);
            if (!compressOk) {
              simplifiedCount = segmentCount;
              for (idx = 0; idx < segmentCount; idx = idx + 1) {
                Integer col;

                for (col = 0; col < 4; col = col + 1) {
                  simplifiedRows[idx][col] = segmentRows[idx][col];
                }
              }
            }

            for (k = 1; k < simplifiedCount; k = k + 1) {
              if (pathN < APA_PATH_MAX_POINTS) {
                pathX[pathN] = simplifiedRows[k][0];
                pathY[pathN] = simplifiedRows[k][1];
                pathTht[pathN] = simplifiedRows[k][2];
                pathOdoStep[pathN] = simplifiedRows[k][3];
                pathN = pathN + 1;
              }
            }
          }

          if (calType == 1) {
            apaPathClearSegmentRows(segmentRows);
            segmentOdo = 0.0f;
            segmentCount = 0;
          } else {
            Real lastRow[4];
            Integer col;

            for (col = 0; col < 4; col = col + 1) {
              lastRow[col] = segmentRows[segmentCount - 1][col];
            }
            apaPathClearSegmentRows(segmentRows);
            for (col = 0; col < 4; col = col + 1) {
              segmentRows[0][col] = lastRow[col];
            }
            segmentOdo = lastRow[3];
            if (segmentOdo < 0.0f) {
              segmentOdo = -segmentOdo;
            }
            segmentCount = 1;
          }
        }
      }
    }
  }

  for (idx = 0; idx < pathN - 1; idx = idx + 1) {
    Real dx;
    Real dy;
    Real distance;
    Real signValue;

    dx = pathX[idx] - pathX[idx + 1];
    dy = pathY[idx] - pathY[idx + 1];
    distance = apaMathNorm2D(dx, dy);
    signValue = 0.0f;
    if (apaMathGreater(pathOdoStep[idx], 0.0f)) {
      signValue = 1.0f;
    } else if (apaMathLess(pathOdoStep[idx], 0.0f)) {
      signValue = -1.0f;
    }
    pathOdoStep[idx] = signValue * distance;
  }

  if (p->PathN == pathN) {
    return;
  }

  for (idx = 0; idx < pathN; idx = idx + 1) {
    p->X[idx] = pathX[idx];
    p->Y[idx] = pathY[idx];
    p->Tht[idx] = pathTht[idx];
    p->OdoStep[idx] = pathOdoStep[idx];
  }
  for (idx = pathN; idx < p->PathN; idx = idx + 1) {
    p->X[idx] = 0.0f;
    p->Y[idx] = 0.0f;
    p->Tht[idx] = 0.0f;
    p->OdoStep[idx] = 0.0f;
  }
  p->PathN = pathN;
  apaPathResetFlags(p);
}

/**
 * @brief 在路径头部添加位姿点 — 对应 ClassPath.completePosStart()
 */
static inline void apaPathPrependPos(ApaPath_t *p, Real addX, Real addY, Real addTht) {
  Integer i;
  Integer n;
  Integer addPtFlag;
  Real dist;
  Real pathStartX;
  Real pathStartY;

  n = p->PathN;
  if (n >= APA_PATH_MAX_POINTS) {
    return;
  }
  if (n <= 0) {
    return;
  }

  pathStartX = p->X[0];
  pathStartY = p->Y[0];

  for (i = n; i > 0; i = i - 1) {
    p->X[i] = p->X[i - 1];
    p->Y[i] = p->Y[i - 1];
    p->Tht[i] = p->Tht[i - 1];
    p->OdoStep[i] = p->OdoStep[i - 1];
  }

  p->X[0] = addX;
  p->Y[0] = addY;
  p->Tht[0] = addTht;

  dist = apaMathNorm2D(pathStartX - addX, pathStartY - addY);
  addPtFlag = apaPlannerGetDirFromVehPos(addX, addY, addTht, pathStartX, pathStartY);
  if (addPtFlag != 1) {
    dist = -dist;
  }
  p->OdoStep[0] = dist;

  p->PathN = n + 1;
  apaPathResetFlags(p);
}

/**
 * @brief 在路径尾部添加位姿点 — 对应 ClassPath.completePosEnd()
 */
static inline void apaPathAppendPos(ApaPath_t *p, Real addX, Real addY, Real addTht) {
  Integer n;
  Real dist;

  n = p->PathN;
  if (n >= APA_PATH_MAX_POINTS) {
    return;
  }
  if (n == 0) {
    return;
  }

  dist = apaMathNorm2D(addX - p->X[n - 1], addY - p->Y[n - 1]);
  p->OdoStep[n - 1] = dist;

  if (n >= 2) {
    if (apaMathLess(p->OdoStep[n - 2], 0.0f)) {
      p->OdoStep[n - 1] = -p->OdoStep[n - 1];
    }
  }

  p->X[n] = addX;
  p->Y[n] = addY;
  p->Tht[n] = addTht;
  p->OdoStep[n] = 0.0f;

  p->PathN = n + 1;
  apaPathResetFlags(p);
}

/**
 * @brief 获取路径一致性状态 — 对应 ClassPath.getConsistentStatus()
 *
 * valid/path_n/error_type 三元组合必须与 MATLAB ClassPath 语义一致。
 */
static inline Boolean apaPathGetConsistentStatus(const ApaPath_t *p) {
  Boolean statusValid;

  statusValid = false;
  if (p->ValidFlag && p->PathN != 0 && p->ErrorType == APA_FAULT_NO_ERR) {
    statusValid = true;
  } else if (!p->ValidFlag && p->PathN == 0 && p->ErrorType != APA_FAULT_NO_ERR) {
    statusValid = true;
  }
  return statusValid;
}

/**
 * @brief 获取路径长度校验 — 对应 ClassPath.getPathLengthValid()
 */
static inline Boolean apaPathGetLengthValid(const ApaPath_t *p, Real pathLenMin, Real pathLenMax) {
  Real totalLen;
  Real segLen;
  Integer i;
  Boolean lengthValid;

  lengthValid = false;
  if (!p->ValidFlag) {
    return lengthValid;
  }

  totalLen = 0.0f;
  for (i = 0; i < p->PathN - 1; i = i + 1) {
    Real absOdo;

    absOdo = p->OdoStep[i];
    if (absOdo < 0.0f) {
      absOdo = -absOdo;
    }
    totalLen = totalLen + absOdo;
  }

  if (apaMathLess(totalLen, pathLenMin)) {
    return false;
  }
  if (apaMathGreater(totalLen, pathLenMax)) {
    return false;
  }

  segLen = 0.0f;
  if (p->PathN > 1) {
    for (i = 0; i < p->PathN - 1; i = i + 1) {
      Real absOdo;
      Real product;
      Boolean segmentEnd;

      absOdo = p->OdoStep[i];
      if (absOdo < 0.0f) {
        absOdo = -absOdo;
      }
      segLen = segLen + absOdo;

      product = p->OdoStep[i] * p->OdoStep[i + 1];
      segmentEnd = false;
      if (apaMathLess(product, 0.0f)) {
        segmentEnd = true;
      }
      if (i == p->PathN - 2) {
        segmentEnd = true;
      }
      if (segmentEnd) {
        if (apaMathLess(segLen, pathLenMin)) {
          return false;
        }
        if (apaMathGreater(segLen, pathLenMax)) {
          return false;
        }
        segLen = 0.0f;
      }
    }
  }

  lengthValid = true;
  return lengthValid;
}

/**
 * @brief 坐标重置 — 对应 ClassPath.coordinateReset()
 *
 * 将路径所有点进行坐标系变换（当导航坐标系重置时）。
 */
static inline void apaPathCoordinateReset(ApaPath_t *p, Real resetX, Real resetY, Real resetYaw) {
  Integer i;

  for (i = 0; i < p->PathN; i = i + 1) {
    Real gx;
    Real gy;
    Real gTht;

    apaCoordGround2Veh(resetX, resetY, resetYaw, p->X[i], p->Y[i], p->Tht[i], &gx, &gy, &gTht);
    p->X[i] = gx;
    p->Y[i] = gy;
    p->Tht[i] = gTht;
  }
}

/**
 * @brief 获取强制反向校验结果 — 对应 ClassPath.getForceReverseValid()
 */
static inline Boolean
apaPathGetForceReverseValidWithThreshold(const ApaPath_t *p, Integer expDirecFlag, Real odoThresh) {
  Boolean pathDirecValid;
  Boolean forceDirection;
  Integer idx;
  Real odoSeg;

  pathDirecValid = false;
  forceDirection = false;
  if (expDirecFlag == APA_EXP_DIREC_FORCE_FORWARD || expDirecFlag == APA_EXP_DIREC_FORCE_BACKWARD) {
    forceDirection = true;
  }

  if (!forceDirection) {
    pathDirecValid = true;
  } else if (p->PathN > 0) {
    Boolean firstDirectionOk;

    firstDirectionOk = true;
    if (expDirecFlag == APA_EXP_DIREC_FORCE_FORWARD && apaMathLess(p->OdoStep[0], 0.0f)) {
      firstDirectionOk = false;
    } else if (expDirecFlag == APA_EXP_DIREC_FORCE_BACKWARD &&
               apaMathGreater(p->OdoStep[0], 0.0f)) {
      firstDirectionOk = false;
    }

    if (firstDirectionOk) {
      Boolean sameSegment;

      odoSeg = p->OdoStep[0];
      if (odoSeg < 0.0f) {
        odoSeg = -odoSeg;
      }
      idx = 0;
      sameSegment = true;
      while (idx < p->PathN - 2 && sameSegment) {
        Real product;
        Real absOdo;

        product = p->OdoStep[idx] * p->OdoStep[idx + 1];
        if (apaMathLess(product, 0.0f)) {
          sameSegment = false;
        } else {
          absOdo = p->OdoStep[idx + 1];
          if (absOdo < 0.0f) {
            absOdo = -absOdo;
          }
          odoSeg = odoSeg + absOdo;
          idx = idx + 1;
        }
      }
      if (!apaMathLess(odoSeg, odoThresh)) {
        pathDirecValid = true;
      }
    }
  }
  return pathDirecValid;
}

static inline Boolean apaPathGetForceReverseValid(const ApaPath_t *p, Integer expDirecFlag) {
  return apaPathGetForceReverseValidWithThreshold(
      p, expDirecFlag, APA_PATH_EXPECT_DIRECTION_ODO_THRESH);
}

/**
 * @brief 路径空间分布范数 — 对应 ClassPath.getPathSpatialPattern()
 */
static inline Real apaPathGetSpatialPattern(const ApaPath_t *p,
                                            const Integer revPtsPtrList[APA_PATH_MAX_POINTS],
                                            Integer revPtsNum) {
  Real normInf;
  Integer idx;

  normInf = -1000000.0f;
  for (idx = 0; idx < revPtsNum - 1; idx = idx + 1) {
    Integer pathIdx;

    pathIdx = revPtsPtrList[idx];
    if (pathIdx >= 0 && pathIdx < p->PathN && p->PathN > 0) {
      Real normOne;
      Real absX;
      Real absY;

      absX = p->X[pathIdx] - p->X[p->PathN - 1];
      if (absX < 0.0f) {
        absX = -absX;
      }
      absY = p->Y[pathIdx] - p->Y[p->PathN - 1];
      if (absY < 0.0f) {
        absY = -absY;
      }
      normOne = absX + absY;
      if (apaMathGreater(normOne, normInf)) {
        normInf = normOne;
      }
    }
  }
  return normInf;
}

/**
 * @brief 计算路径末端共线直线段长度 — 对应 ClassPath.calPathStraightTailDist()
 */
static inline Real apaPathCalcStraightTailDist(const ApaPath_t *p) {
  Real tailDist;
  Integer idx;

  tailDist = 0.0f;
  if (p->PathN > 1) {
    Real slopeK;
    Real interceptB;
    Boolean keepScanning;

    idx = p->PathN - 2;
    tailDist = p->OdoStep[idx];
    if (tailDist < 0.0f) {
      tailDist = -tailDist;
    }
    apaGeoLinearFuncSolve(p->X[idx], p->Y[idx], p->X[idx + 1], p->Y[idx + 1], &slopeK, &interceptB);
    idx = idx - 1;
    keepScanning = true;
    while (idx > 0 && keepScanning) {
      Real latDis;

      latDis = apaGeoPointLineDistanceFromSlopeIntercept(p->X[idx], p->Y[idx], slopeK, interceptB);
      if (apaMathGreater(latDis, 0.05f)) {
        keepScanning = false;
      } else {
        Real absOdo;
        Real product;

        absOdo = p->OdoStep[idx];
        if (absOdo < 0.0f) {
          absOdo = -absOdo;
        }
        tailDist = tailDist + absOdo;
        product = p->OdoStep[idx] * p->OdoStep[idx + 1];
        if (apaMathLess(product, 0.0f)) {
          keepScanning = false;
        } else {
          idx = idx - 1;
        }
      }
    }
  }
  return tailDist;
}

/**
 * @brief 累计路径航向变化 — 对应 ClassPath.evaluatePathContinuty()
 */
static inline Real apaPathEvaluateContinuity(const ApaPath_t *p) {
  Real cost;
  Integer idx;

  cost = 0.0f;
  for (idx = 0; idx < p->PathN - 1; idx = idx + 1) {
    Real deltaTht;

    deltaTht = p->Tht[idx + 1] - p->Tht[idx];

    cost = cost + fabsf(apaCoordWrapToPi(deltaTht));
  }
  return cost;
}

/**
 * @brief 计算路径曲率半径 — 对应 ClassPath.calcPathRadius()
 */
static inline void apaPathCalcRadius(ApaPath_t *p) {
  Integer idx;

  for (idx = 0; idx < APA_PATH_MAX_POINTS; idx = idx + 1) {
    p->Radius[idx] = 0.0f;
  }

  if (p->PathN >= 3) {
    for (idx = 1; idx < p->PathN - 1; idx = idx + 1) {
      Real product;

      product = p->OdoStep[idx - 1] * p->OdoStep[idx];
      if (apaMathLess(product, 0.0f)) {
        p->Radius[idx] = 1000000.0f;
      } else {
        Real kappa;
        Real divisor;

        kappa = apaMathCircleCurvature(
            p->X[idx], p->Y[idx], p->X[idx - 1], p->Y[idx - 1], p->X[idx + 1], p->Y[idx + 1], true);
        divisor = apaMathDivideZeroProcess(kappa);
        p->Radius[idx] = 1.0f / divisor;
      }
    }
    p->Radius[0] = p->Radius[1];
    p->Radius[p->PathN - 1] = p->Radius[p->PathN - 2];
  }
}

/**
 * @brief 获取路径曲率半径并维护缓存标志 — 对应 ClassPath.getRadius()
 */
static inline void apaPathGetRadius(ApaPath_t *p) {
  if (!p->RadiusFlag) {
    apaPathCalcRadius(p);
    p->RadiusFlag = true;
  }
}

typedef Boolean (*ApaPathCollisionProbeFunc)(
    Integer pointIndex, Real posX, Real posY, Real posTheta, void *context);

typedef Boolean (*ApaPathNearSlotProbeFunc)(
    Integer pointIndex, Real posX, Real posY, Real posTheta, void *context);

static inline Boolean apaPathNearSlotSkipped(Integer pointIndex,
                                             Real posX,
                                             Real posY,
                                             Real posTheta,
                                             Boolean checkSlotArea,
                                             ApaPathNearSlotProbeFunc nearSlotProbe,
                                             void *context) {
  Boolean skipped;

  skipped = false;
  if (!checkSlotArea && nearSlotProbe != (ApaPathNearSlotProbeFunc)0) {
    skipped = nearSlotProbe(pointIndex, posX, posY, posTheta, context);
  }
  return skipped;
}

/**
 * @brief 路径碰撞扫描 — 对应 ClassPath.collsionCheck()
 *
 * probe 由调用方绑定到 VehicleCollisionCheck；本 helper 固化 ClassPath 的逐点调度。
 */
static inline Boolean apaPathCollisionCheckWithProbe(ApaPath_t *p,
                                                     ApaPathCollisionProbeFunc probe,
                                                     void *probeContext,
                                                     Boolean checkSlotArea,
                                                     ApaPathNearSlotProbeFunc nearSlotProbe,
                                                     void *nearSlotContext) {
  Boolean detected[APA_PATH_MAX_POINTS];
  Boolean existCollision;
  Integer idx;
  Real odoCheck;

  existCollision = true;
  if (p->PathN <= 1) {
    return existCollision;
  }
  if (probe == (ApaPathCollisionProbeFunc)0) {
    return existCollision;
  }

  for (idx = 0; idx < APA_PATH_MAX_POINTS; idx = idx + 1) {
    detected[idx] = false;
  }

  for (idx = 1; idx < p->PathN; idx = idx + 1) {
    Real product;
    Boolean shouldProbe;
    Boolean skipped;

    product = p->OdoStep[idx] * p->OdoStep[idx - 1];
    shouldProbe = false;
    if (apaMathLess(product, 0.0f)) {
      shouldProbe = true;
    }
    if (idx == p->PathN - 1) {
      shouldProbe = true;
    }
    if (shouldProbe) {
      skipped = apaPathNearSlotSkipped(
          idx, p->X[idx], p->Y[idx], p->Tht[idx], checkSlotArea, nearSlotProbe, nearSlotContext);
      if (!skipped) {
        detected[idx] = true;
        existCollision = probe(idx, p->X[idx], p->Y[idx], p->Tht[idx], probeContext);
        if (existCollision) {
          return existCollision;
        }
      }
    }
  }

  apaPathGetRadius(p);
  odoCheck = 0.0f;
  existCollision = false;
  for (idx = 1; idx < p->PathN - 1; idx = idx + 1) {
    Boolean skipped;
    Real absOdo;
    Real absRadius;
    Real odoThresh;

    if (detected[idx]) {
      odoCheck = 0.0f;
    } else {
      absOdo = p->OdoStep[idx];
      if (absOdo < 0.0f) {
        absOdo = -absOdo;
      }
      odoCheck = odoCheck + absOdo;

      absRadius = p->Radius[idx];
      if (absRadius < 0.0f) {
        absRadius = -absRadius;
      }
      if (apaMathGreater(absRadius, 30.0f)) {
        odoThresh = 0.6f;
      } else if (apaMathLess(absRadius, 20.0f)) {
        odoThresh = 0.2f;
      } else {
        odoThresh = 0.4f;
      }

      skipped = apaPathNearSlotSkipped(
          idx, p->X[idx], p->Y[idx], p->Tht[idx], checkSlotArea, nearSlotProbe, nearSlotContext);
      if (!skipped && apaMathGreater(odoCheck, odoThresh)) {
        odoCheck = 0.0f;
        existCollision = probe(idx, p->X[idx], p->Y[idx], p->Tht[idx], probeContext);
        if (existCollision) {
          return existCollision;
        }
      }
    }
  }
  return existCollision;
}

/**
 * @brief 获取并缓存路径碰撞风险 — 对应 ClassPath.getCollisionFlag()
 */
static inline Integer apaPathGetCollisionFlagWithProbe(ApaPath_t *p,
                                                       ApaPathCollisionProbeFunc probe,
                                                       void *probeContext,
                                                       Boolean checkSlotArea,
                                                       ApaPathNearSlotProbeFunc nearSlotProbe) {
  Boolean existCollision;
  Integer collisionFlag;

  collisionFlag = p->CollisionFlag;
  if (collisionFlag == APA_COLLISION_RISK_UNKNOWN) {
    existCollision = apaPathCollisionCheckWithProbe(
        p, probe, probeContext, checkSlotArea, nearSlotProbe, probeContext);
    if (existCollision) {
      collisionFlag = APA_COLLISION_RISK_EXIST;
    } else {
      collisionFlag = APA_COLLISION_RISK_NOT_EXIST;
    }
    p->CollisionFlag = collisionFlag;
  }
  return collisionFlag;
}

#ifndef APA_PATH_QUALITY_ENTER_POS_LEN
#define APA_PATH_QUALITY_ENTER_POS_LEN 2.5f
#endif

typedef struct {
  Integer ParkingSysMainState;
  Integer GearPos;
  Real VehVel;
  Integer ExpDirecFlag;
  Integer SlotClass;
  Integer SurroundingType;
  Integer ParkingDirec;
  Integer ParkingType;
  Integer SlotSubStatus;
  Integer EnterEdge;
  Real SlotPts[8];
  Real TarPos[3];
  Real VehWheelBase;
  Real ExpectDirectionOdoThresh;
  Real EnterPosLen;
} ApaPathQualityCompareContext_t;

static inline void apaPathQualityCompareContextInit(ApaPathQualityCompareContext_t *context) {
  Integer idx;

  context->ParkingSysMainState = APA_SYS_STATE_INVALID;
  context->GearPos = APA_GEAR_POS_N;
  context->VehVel = 0.0f;
  context->ExpDirecFlag = APA_EXP_DIREC_NONE;
  context->SlotClass = APA_SLOT_CLASS_UNKNOWN;
  context->SurroundingType = APA_SURROUND_UNKNOWN;
  context->ParkingDirec = APA_PARKING_DIREC_INVALID;
  context->ParkingType = APA_PARKING_TYPE_UNKNOWN;
  context->SlotSubStatus = APA_SLOT_SUB_END_NONE;
  context->EnterEdge = 1;
  context->VehWheelBase = APA_VEHICLE_WHEELBASE_M;
  context->ExpectDirectionOdoThresh = APA_PATH_EXPECT_DIRECTION_ODO_THRESH;
  context->EnterPosLen = APA_PATH_QUALITY_ENTER_POS_LEN;
  for (idx = 0; idx < 8; idx = idx + 1) {
    context->SlotPts[idx] = 0.0f;
  }
  for (idx = 0; idx < 3; idx = idx + 1) {
    context->TarPos[idx] = 0.0f;
  }
}

/**
 * @brief 将 ClassSlot 运行时状态绑定到 qualityCompare 上下文。
 *
 * 对应 qualityCompare 内部读取 slot_obj.getSlotClass/getParkingDirec 以及
 * slot_obj 已解析的 getSurroundingType/getParkingType 结果。
 */
static inline void apaPathQualityCompareContextBindSlot(ApaPathQualityCompareContext_t *context,
                                                        ApaSlot_t *slot,
                                                        const ApaPlannerVehConfig_t *vehConfig) {
  Integer idx;

  if (context == (ApaPathQualityCompareContext_t *)0 || slot == (ApaSlot_t *)0) {
    return;
  }

  context->SlotClass = apaSlotGetSlotClass(slot);
  context->SurroundingType = slot->SurroundingType;
  context->ParkingDirec = apaSlotGetParkingDirec(slot);
  context->ParkingType = slot->ParkingType;
  context->SlotSubStatus = slot->SlotSubStatus;
  context->EnterEdge = slot->EnterEdge;
  for (idx = 0; idx < 8; idx = idx + 1) {
    context->SlotPts[idx] = slot->SlotPts[idx];
  }
  for (idx = 0; idx < 3; idx = idx + 1) {
    context->TarPos[idx] = slot->TarPos[idx];
  }
  if (vehConfig != (const ApaPlannerVehConfig_t *)0) {
    context->VehWheelBase = vehConfig->WB;
  }
}

static inline Integer
apaPathQualityFirstDirectionCompare(const Real originSegOdoList[APA_PATH_MAX_POINTS],
                                    const Real candidateSegOdoList[APA_PATH_MAX_POINTS],
                                    const ApaPath_t *originPath,
                                    const ApaPath_t *candidatePath,
                                    Integer expDirecFlag,
                                    Real odoThresh) {
  Integer result;
  Boolean originForwardValid;
  Boolean candidateForwardValid;
  Boolean originBackwardValid;
  Boolean candidateBackwardValid;

  result = APA_PATH_COMP_UNKNOWN;
  originForwardValid = apaMathGreater(originSegOdoList[0], odoThresh);
  candidateForwardValid = apaMathGreater(candidateSegOdoList[0], odoThresh);
  originBackwardValid = apaMathLess(originSegOdoList[0], -odoThresh);
  candidateBackwardValid = apaMathLess(candidateSegOdoList[0], -odoThresh);

  if (expDirecFlag == APA_EXP_DIREC_FORCE_FORWARD ||
      expDirecFlag == APA_EXP_DIREC_SUGGEST_FORWARD) {
    if (originForwardValid && !candidateForwardValid) {
      result = APA_PATH_COMP_ORIGIN_PATH_BETTER;
    } else if (!originForwardValid && candidateForwardValid) {
      result = APA_PATH_COMP_NEW_PATH_BETTER;
    }
  } else if (expDirecFlag == APA_EXP_DIREC_FORCE_BACKWARD ||
             expDirecFlag == APA_EXP_DIREC_SUGGEST_BACKWARD) {
    if ((!originBackwardValid || apaMathGreater(originPath->OdoStep[0], 0.0f)) &&
        candidateBackwardValid) {
      result = APA_PATH_COMP_NEW_PATH_BETTER;
    } else if (originBackwardValid &&
               (!candidateBackwardValid || apaMathGreater(candidatePath->OdoStep[0], 0.0f))) {
      result = APA_PATH_COMP_ORIGIN_PATH_BETTER;
    }
  }
  return result;
}

static inline Boolean
apaPathQualityOneStepParkingIn(const ApaPathQualityCompareContext_t *context) {
  Boolean oneStep;

  oneStep = false;
  if (context->SlotClass == APA_SLOT_CLASS_VERTICAL) {
    if ((context->ParkingType == APA_PARKING_TYPE_LEFT ||
         context->ParkingType == APA_PARKING_TYPE_RIGHT) &&
        context->SurroundingType == APA_SURROUND_OBS_EXIST_NONE) {
      oneStep = true;
    } else if (context->ParkingType == APA_PARKING_TYPE_LEFT &&
               context->SurroundingType == APA_SURROUND_OBS_EXIST_RIGHT_REAR_ONLY) {
      oneStep = true;
    } else if (context->ParkingType == APA_PARKING_TYPE_RIGHT &&
               context->SurroundingType == APA_SURROUND_OBS_EXIST_LEFT_FRONT_ONLY) {
      oneStep = true;
    }
  } else if (context->SlotClass == APA_SLOT_CLASS_PARALLEL) {
    if ((context->ParkingType == APA_PARKING_TYPE_LEFT ||
         context->ParkingType == APA_PARKING_TYPE_RIGHT) &&
        context->SurroundingType != APA_SURROUND_OBS_EXIST_BOTH) {
      oneStep = true;
    }
  }
  return oneStep;
}

static inline Real apaPathQualitySegOdoTotal(const Real segOdoList[APA_PATH_MAX_POINTS],
                                             Integer segNum) {
  Real total;
  Integer idx;

  total = 0.0f;
  for (idx = 0; idx < segNum; idx = idx + 1) {
    total = total + apaMathAbs(segOdoList[idx]);
  }
  return total;
}

/* Shared ClassPath.qualityCompare predicate: a direction run, not a point
 * spacing or a constant-curvature primitive. */
static inline Boolean apaPathDirectionRunIsShort(Real length) {
  return apaMathLess(apaMathAbs(length), 0.4f);
}

static inline Integer
apaPathQualityShortSegmentCompare(const Real originSegOdoList[APA_PATH_MAX_POINTS],
                                  const Real candidateSegOdoList[APA_PATH_MAX_POINTS],
                                  Integer originRevPtsNum,
                                  Integer candidateRevPtsNum) {
  Integer originCount;
  Integer candidateCount;
  Integer idx;
  Integer result;

  originCount = 0;
  candidateCount = 0;
  result = APA_PATH_COMP_UNKNOWN;
  for (idx = 0; idx < originRevPtsNum - 1; idx = idx + 1) {
    if (apaPathDirectionRunIsShort(originSegOdoList[idx])) {
      originCount = originCount + 1;
    }
  }
  for (idx = 0; idx < candidateRevPtsNum - 1; idx = idx + 1) {
    if (apaPathDirectionRunIsShort(candidateSegOdoList[idx])) {
      candidateCount = candidateCount + 1;
    }
  }
  if (originCount < candidateCount) {
    result = APA_PATH_COMP_ORIGIN_PATH_BETTER;
  } else if (originCount > candidateCount) {
    result = APA_PATH_COMP_NEW_PATH_BETTER;
  }
  return result;
}

static inline Integer apaPathQualitySlotCoordIndex(Integer coordPtrOneBased) {
  Integer coordPtr;

  coordPtr = coordPtrOneBased;
  while (coordPtr > 8) {
    coordPtr = coordPtr - 8;
  }
  while (coordPtr <= 0) {
    coordPtr = coordPtr + 8;
  }
  return coordPtr - 1;
}

static inline Real
apaPathQualityInvasionDepthTotal(const ApaPath_t *path,
                                 const Integer revPtsPtrList[APA_PATH_MAX_POINTS],
                                 Integer revPtsNum,
                                 const ApaPathQualityCompareContext_t *context,
                                 Real slope,
                                 Real intercept) {
  Integer revIndex;
  Real depthTotal;

  depthTotal = 0.0f;
  revIndex = 1;
  while (revIndex < revPtsNum - 1) {
    Integer pathIndex;
    Real revX;
    Real revY;

    pathIndex = revPtsPtrList[revIndex];
    revX = path->X[pathIndex];
    revY = path->Y[pathIndex];
    if (context->ParkingDirec == APA_PARKING_DIREC_FRONT) {
      Real frontX;
      Real frontY;

      apaCoordFrontAxlePoint(
          revX, revY, path->Tht[pathIndex], context->VehWheelBase, &frontX, &frontY);
      revX = frontX;
      revY = frontY;
    }
    if (apaMathIsPtInRect(revX, revY, context->SlotPts)) {
      Real pointDepth;

      pointDepth = apaGeoPointLineDistanceFromSlopeIntercept(revX, revY, slope, intercept);
      depthTotal = depthTotal + pointDepth;
    }
    revIndex = revIndex + 1;
  }
  return depthTotal;
}

static inline Integer
apaPathQualityInvasionDepthCompare(const ApaPath_t *originPath,
                                   const ApaPath_t *candidatePath,
                                   const Integer originRevPtsPtrList[APA_PATH_MAX_POINTS],
                                   const Integer candidateRevPtsPtrList[APA_PATH_MAX_POINTS],
                                   Integer originRevPtsNum,
                                   Integer candidateRevPtsNum,
                                   const ApaPathQualityCompareContext_t *context) {
  Integer result;

  result = APA_PATH_COMP_UNKNOWN;
  if (context->SlotClass == APA_SLOT_CLASS_VERTICAL &&
      context->SlotSubStatus == APA_SLOT_SUB_END_NONE) {
    Integer pt1XPtr;
    Integer pt1YPtr;
    Integer pt2XPtr;
    Integer pt2YPtr;
    Real pt1X;
    Real pt1Y;
    Real pt2X;
    Real pt2Y;
    Real slope;
    Real intercept;
    Real depthThresh;
    Real originDepthTotal;
    Real candidateDepthTotal;

    pt1XPtr = apaPathQualitySlotCoordIndex((2 * context->EnterEdge) - 1);
    pt1YPtr = apaPathQualitySlotCoordIndex(2 * context->EnterEdge);
    pt2XPtr = apaPathQualitySlotCoordIndex((2 * context->EnterEdge) + 1);
    pt2YPtr = apaPathQualitySlotCoordIndex((2 * context->EnterEdge) + 2);
    pt1X = context->SlotPts[pt1XPtr];
    pt1Y = context->SlotPts[pt1YPtr];
    pt2X = context->SlotPts[pt2XPtr];
    pt2Y = context->SlotPts[pt2YPtr];
    apaGeoLinearFuncSolve(pt1X, pt1Y, pt2X, pt2Y, &slope, &intercept);

    depthThresh = 2.0f;
    if (context->SurroundingType == APA_SURROUND_OBS_EXIST_NONE) {
      depthThresh = 3.0f;
    }

    originDepthTotal = apaPathQualityInvasionDepthTotal(
        originPath, originRevPtsPtrList, originRevPtsNum, context, slope, intercept);
    candidateDepthTotal = apaPathQualityInvasionDepthTotal(
        candidatePath, candidateRevPtsPtrList, candidateRevPtsNum, context, slope, intercept);
    if (apaMathGreater(originDepthTotal, depthThresh) ||
        apaMathGreater(candidateDepthTotal, depthThresh)) {
      if (apaMathGreater(originDepthTotal, candidateDepthTotal)) {
        result = APA_PATH_COMP_NEW_PATH_BETTER;
      } else {
        result = APA_PATH_COMP_ORIGIN_PATH_BETTER;
      }
    }
  }
  return result;
}

static inline Integer
apaPathQualityGoalErrorCompare(const ApaPath_t *originPath,
                               const ApaPath_t *candidatePath,
                               const ApaPathQualityCompareContext_t *context) {
  Real originErrX;
  Real originErrY;
  Real originErrTheta;
  Real candidateErrX;
  Real candidateErrY;
  Real candidateErrTheta;
  Real longThresh;
  Real latThresh;
  Real yawThresh;
  Boolean originValid;
  Boolean candidateValid;
  Integer result;

  result = APA_PATH_COMP_UNKNOWN;
  if (originPath->PathN <= 0 || candidatePath->PathN <= 0) {
    return result;
  }

  apaCoordGround2Veh(context->TarPos[0],
                     context->TarPos[1],
                     context->TarPos[2],
                     originPath->X[originPath->PathN - 1],
                     originPath->Y[originPath->PathN - 1],
                     originPath->Tht[originPath->PathN - 1],
                     &originErrX,
                     &originErrY,
                     &originErrTheta);
  apaCoordGround2Veh(context->TarPos[0],
                     context->TarPos[1],
                     context->TarPos[2],
                     candidatePath->X[candidatePath->PathN - 1],
                     candidatePath->Y[candidatePath->PathN - 1],
                     candidatePath->Tht[candidatePath->PathN - 1],
                     &candidateErrX,
                     &candidateErrY,
                     &candidateErrTheta);
  originErrX = apaMathAbs(originErrX);
  originErrY = apaMathAbs(originErrY);
  originErrTheta = apaMathAbs(originErrTheta);
  candidateErrX = apaMathAbs(candidateErrX);
  candidateErrY = apaMathAbs(candidateErrY);
  candidateErrTheta = apaMathAbs(candidateErrTheta);

  if (context->SlotClass == APA_SLOT_CLASS_VERTICAL) {
    longThresh = 0.2f;
  } else {
    longThresh = 0.1f;
  }
  latThresh = 0.05f;
  yawThresh = APA_PI / 180.0f;

  originValid = true;
  candidateValid = true;
  if (apaMathGreater(originErrX, longThresh) || apaMathGreater(originErrY, latThresh) ||
      apaMathGreater(originErrTheta, yawThresh)) {
    originValid = false;
  }
  if (apaMathGreater(candidateErrX, longThresh) || apaMathGreater(candidateErrY, latThresh) ||
      apaMathGreater(candidateErrTheta, yawThresh)) {
    candidateValid = false;
  }

  if (originValid && !candidateValid) {
    result = APA_PATH_COMP_ORIGIN_PATH_BETTER;
  } else if (!originValid && candidateValid) {
    result = APA_PATH_COMP_NEW_PATH_BETTER;
  } else if (!originValid && !candidateValid) {
    if (apaMathGreater(candidateErrX + longThresh, originErrX) &&
        apaMathGreater(candidateErrY + latThresh, originErrY) &&
        apaMathGreater(candidateErrTheta + yawThresh, originErrTheta)) {
      result = APA_PATH_COMP_ORIGIN_PATH_BETTER;
    } else if (apaMathGreater(originErrX + longThresh, candidateErrX) &&
               apaMathGreater(originErrY + latThresh, candidateErrY) &&
               apaMathGreater(originErrTheta + yawThresh, candidateErrTheta)) {
      result = APA_PATH_COMP_NEW_PATH_BETTER;
    }
  }
  return result;
}

static inline Integer apaPathQualityCompare(const ApaPath_t *originPath,
                                            const ApaPath_t *candidatePath,
                                            const ApaPathQualityCompareContext_t *context) {
  Integer originSegNum;
  Integer candidateSegNum;
  Integer originRawSegNum;
  Integer candidateRawSegNum;
  Integer originRevPtsNum;
  Integer candidateRevPtsNum;
  Integer originRevPtsPtrList[APA_PATH_MAX_POINTS];
  Integer candidateRevPtsPtrList[APA_PATH_MAX_POINTS];
  Real originSegOdoList[APA_PATH_MAX_POINTS];
  Real candidateSegOdoList[APA_PATH_MAX_POINTS];
  Boolean originValid;
  Boolean candidateValid;
  Boolean originCollision;
  Boolean candidateCollision;
  Integer pathSegThresh;
  Integer result;
  Real originDeltaTheta;
  Real candidateDeltaTheta;
  Real originPathOdo;
  Real candidatePathOdo;

  result = APA_PATH_COMP_UNKNOWN;
  if (originPath->PathType == APA_PATH_TYPE_UNKNOWN ||
      candidatePath->PathType == APA_PATH_TYPE_UNKNOWN ||
      originPath->PathType != candidatePath->PathType) {
    return result;
  }

  originValid = originPath->ValidFlag && (originPath->PathN != 0);
  candidateValid = candidatePath->ValidFlag && (candidatePath->PathN != 0);
  if (!originValid && !candidateValid) {
    return result;
  }
  if (originValid && !candidateValid) {
    return APA_PATH_COMP_ORIGIN_PATH_BETTER;
  }
  if (!originValid && candidateValid) {
    return APA_PATH_COMP_NEW_PATH_BETTER;
  }

  originRawSegNum = 0;
  candidateRawSegNum = 0;
  originRevPtsNum = 0;
  candidateRevPtsNum = 0;
  apaPathCalcSegNumAndRevPtr(
      originPath, &originRawSegNum, originRevPtsPtrList, originSegOdoList, &originRevPtsNum);
  apaPathCalcSegNumAndRevPtr(candidatePath,
                             &candidateRawSegNum,
                             candidateRevPtsPtrList,
                             candidateSegOdoList,
                             &candidateRevPtsNum);
  originSegNum = apaPathCalcSegNumForMotion(
      originPath, context->VehVel, context->GearPos, context->ParkingSysMainState);
  candidateSegNum = apaPathCalcSegNumForMotion(
      candidatePath, context->VehVel, context->GearPos, context->ParkingSysMainState);

  if (originPath->CollisionFlag == APA_COLLISION_RISK_EXIST) {
    originCollision = true;
  } else if (originPath->CollisionFlag == APA_COLLISION_RISK_NOT_EXIST) {
    originCollision = false;
  } else {
    return result;
  }
  if (candidatePath->CollisionFlag == APA_COLLISION_RISK_EXIST) {
    candidateCollision = true;
  } else if (candidatePath->CollisionFlag == APA_COLLISION_RISK_NOT_EXIST) {
    candidateCollision = false;
  } else {
    return result;
  }
  if (!originCollision && candidateCollision) {
    return APA_PATH_COMP_ORIGIN_PATH_BETTER;
  }
  if (originCollision && !candidateCollision) {
    return APA_PATH_COMP_NEW_PATH_BETTER;
  }
  if (originCollision && candidateCollision) {
    return result;
  }

  result = apaPathQualityFirstDirectionCompare(originSegOdoList,
                                               candidateSegOdoList,
                                               originPath,
                                               candidatePath,
                                               context->ExpDirecFlag,
                                               context->ExpectDirectionOdoThresh);
  if (context->ExpDirecFlag == APA_EXP_DIREC_FORCE_FORWARD ||
      context->ExpDirecFlag == APA_EXP_DIREC_FORCE_BACKWARD) {
    if (result != APA_PATH_COMP_UNKNOWN) {
      return result;
    }
  }

  pathSegThresh = 2;
  if (apaPathQualityOneStepParkingIn(context)) {
    pathSegThresh = 0;
  }
  if (originSegNum + pathSegThresh < candidateSegNum) {
    return APA_PATH_COMP_ORIGIN_PATH_BETTER;
  }
  if (originSegNum > candidateSegNum + pathSegThresh) {
    return APA_PATH_COMP_NEW_PATH_BETTER;
  }

  if (originPath->PathType == APA_PATH_TYPE_ALL && originSegNum == 1 && candidateSegNum == 1 &&
      apaPlannerVehIsStopped(context->VehVel) && context->ParkingDirec != APA_PARKING_DIREC_FRONT &&
      originPath->PathN >= 2 && candidatePath->PathN >= 2) {
    const Boolean originEndsReverse = apaMathLess(originPath->OdoStep[originPath->PathN - 2], 0.0f);
    const Boolean candidateEndsReverse =
        apaMathLess(candidatePath->OdoStep[candidatePath->PathN - 2], 0.0f);
    if (originEndsReverse && candidateEndsReverse) {
      if (apaMathGreater(originPath->OdoStep[0], 0.0f) &&
          apaMathLess(candidatePath->OdoStep[0], 0.0f)) {
        return APA_PATH_COMP_NEW_PATH_BETTER;
      }
      if (apaMathGreater(candidatePath->OdoStep[0], 0.0f) &&
          apaMathLess(originPath->OdoStep[0], 0.0f)) {
        return APA_PATH_COMP_ORIGIN_PATH_BETTER;
      }
    }
  }

  result = apaPathQualityInvasionDepthCompare(originPath,
                                              candidatePath,
                                              originRevPtsPtrList,
                                              candidateRevPtsPtrList,
                                              originRevPtsNum,
                                              candidateRevPtsNum,
                                              context);
  if (result != APA_PATH_COMP_UNKNOWN) {
    return result;
  }

  result = apaPathQualityShortSegmentCompare(
      originSegOdoList, candidateSegOdoList, originRevPtsNum, candidateRevPtsNum);
  if (result != APA_PATH_COMP_UNKNOWN) {
    return result;
  }

  originDeltaTheta = apaPathEvaluateContinuity(originPath);
  candidateDeltaTheta = apaPathEvaluateContinuity(candidatePath);
  originPathOdo = apaPathQualitySegOdoTotal(originSegOdoList, originRawSegNum);
  candidatePathOdo = apaPathQualitySegOdoTotal(candidateSegOdoList, candidateRawSegNum);
  if (apaMathLess(originDeltaTheta + (APA_PI / 6.0f), candidateDeltaTheta)) {
    return APA_PATH_COMP_ORIGIN_PATH_BETTER;
  }
  if (apaMathLess(candidateDeltaTheta + (APA_PI / 6.0f), originDeltaTheta)) {
    return APA_PATH_COMP_NEW_PATH_BETTER;
  }
  if (apaMathLess(originDeltaTheta + (APA_PI / 12.0f), candidateDeltaTheta) &&
      apaMathLess(originPathOdo + 5.0f, candidatePathOdo)) {
    return APA_PATH_COMP_ORIGIN_PATH_BETTER;
  }
  if (apaMathLess(candidateDeltaTheta + (APA_PI / 12.0f), originDeltaTheta) &&
      apaMathLess(candidatePathOdo + 5.0f, originPathOdo)) {
    return APA_PATH_COMP_NEW_PATH_BETTER;
  }

  if (originPath->PathType == APA_PATH_TYPE_ALL && context->SlotClass == APA_SLOT_CLASS_VERTICAL) {
    Real originTailDist;
    Real candidateTailDist;

    originTailDist = apaPathCalcStraightTailDist(originPath);
    candidateTailDist = apaPathCalcStraightTailDist(candidatePath);
    if (apaMathLess(originTailDist, context->EnterPosLen) ||
        apaMathLess(candidateTailDist, context->EnterPosLen)) {
      if (apaMathGreater(originTailDist, candidateTailDist + 0.5f)) {
        return APA_PATH_COMP_ORIGIN_PATH_BETTER;
      }
      if (apaMathGreater(candidateTailDist, originTailDist + 0.5f)) {
        return APA_PATH_COMP_NEW_PATH_BETTER;
      }
    }
  }

  if (originPath->PathType == APA_PATH_TYPE_ALL) {
    Real originSpatialPattern;
    Real candidateSpatialPattern;

    originSpatialPattern =
        apaPathGetSpatialPattern(originPath, originRevPtsPtrList, originRevPtsNum);
    candidateSpatialPattern =
        apaPathGetSpatialPattern(candidatePath, candidateRevPtsPtrList, candidateRevPtsNum);
    if (apaMathLess(originSpatialPattern + 5.0f, candidateSpatialPattern)) {
      return APA_PATH_COMP_ORIGIN_PATH_BETTER;
    }
    if (apaMathLess(candidateSpatialPattern + 5.0f, originSpatialPattern)) {
      return APA_PATH_COMP_NEW_PATH_BETTER;
    }
  }

  result = apaPathQualityFirstDirectionCompare(originSegOdoList,
                                               candidateSegOdoList,
                                               originPath,
                                               candidatePath,
                                               context->ExpDirecFlag,
                                               context->ExpectDirectionOdoThresh);
  if (result != APA_PATH_COMP_UNKNOWN) {
    return result;
  }

  if (originSegNum < candidateSegNum) {
    return APA_PATH_COMP_ORIGIN_PATH_BETTER;
  }
  if (originSegNum > candidateSegNum) {
    return APA_PATH_COMP_NEW_PATH_BETTER;
  }

  if (apaMathLess(apaMathAbs(originPathOdo - candidatePathOdo), 3.0f) &&
      apaMathGreater(apaMathAbs(originDeltaTheta - candidateDeltaTheta), APA_PI / 18.0f)) {
    if (apaMathLess(originDeltaTheta, candidateDeltaTheta)) {
      return APA_PATH_COMP_ORIGIN_PATH_BETTER;
    }
    if (apaMathLess(candidateDeltaTheta, originDeltaTheta)) {
      return APA_PATH_COMP_NEW_PATH_BETTER;
    }
  }

  if (originPath->PathType == APA_PATH_TYPE_ALL) {
    result = apaPathQualityGoalErrorCompare(originPath, candidatePath, context);
    if (result != APA_PATH_COMP_UNKNOWN) {
      return result;
    }
  }

  if (!apaMathGreater(originPathOdo, candidatePathOdo)) {
    return APA_PATH_COMP_ORIGIN_PATH_BETTER;
  }
  return APA_PATH_COMP_NEW_PATH_BETTER;
}

/**
 * @brief 路径质量比较前刷新碰撞缓存 — 对应 qualityCompare 调用 getCollisionFlag()
 */
static inline Integer
apaPathQualityCompareWithCollisionProbe(ApaPath_t *originPath,
                                        ApaPath_t *candidatePath,
                                        const ApaPathQualityCompareContext_t *context,
                                        ApaPathCollisionProbeFunc originProbe,
                                        void *originProbeContext,
                                        ApaPathCollisionProbeFunc candidateProbe,
                                        void *candidateProbeContext,
                                        Boolean checkSlotArea,
                                        ApaPathNearSlotProbeFunc nearSlotProbe) {
  (void)apaPathGetCollisionFlagWithProbe(
      originPath, originProbe, originProbeContext, checkSlotArea, nearSlotProbe);
  (void)apaPathGetCollisionFlagWithProbe(
      candidatePath, candidateProbe, candidateProbeContext, checkSlotArea, nearSlotProbe);
  return apaPathQualityCompare(originPath, candidatePath, context);
}

/**
 * @brief 复合路径 — 对应 ClassComplexPath
 *
 * 包含一条路径 + 关联的车位信息 + 搜索节点计数。
 */
typedef struct {
  ApaPath_t Path;       /**< 核心路径 */
  Integer SlotId;       /**< 关联车位ID */
  Integer SlotStatus;   /**< 关联车位状态 */
  Integer CollisionCnt; /**< 碰撞计数 */
  Real OpenList[APA_CPATH_OPEN_CAPACITY][APA_CPATH_NODE_FIELD_COUNT];
  Boolean OpenValidList[APA_CPATH_OPEN_CAPACITY];
  Real CloseList[APA_CPATH_CLOSE_CAPACITY][APA_CPATH_NODE_FIELD_COUNT];
  Boolean CloseValidList[APA_CPATH_CLOSE_CAPACITY];
  /* 搜索节点统计（D4 阶段使用） */
  Integer NodesCnt;
  Integer OpenCnt;
  Integer CloseCnt;
  Boolean NodesCntCalcFlag;
} ApaComplexPath_t;

/** @brief 初始化复合路径 */
static inline void apaCPathInit(ApaComplexPath_t *cp) {
  Integer nodeIdx;
  Integer fieldIdx;

  apaPathInit(&cp->Path);
  cp->SlotId = 0;
  cp->SlotStatus = APA_SLOT_STATUS_INVALID;
  cp->CollisionCnt = 0;
  cp->NodesCnt = 0;
  cp->OpenCnt = 0;
  cp->CloseCnt = 0;
  cp->NodesCntCalcFlag = false;

  for (nodeIdx = 0; nodeIdx < APA_CPATH_OPEN_CAPACITY; nodeIdx = nodeIdx + 1) {
    cp->OpenValidList[nodeIdx] = false;
    for (fieldIdx = 0; fieldIdx < APA_CPATH_NODE_FIELD_COUNT; fieldIdx = fieldIdx + 1) {
      cp->OpenList[nodeIdx][fieldIdx] = 0.0f;
    }
  }
  for (nodeIdx = 0; nodeIdx < APA_CPATH_CLOSE_CAPACITY; nodeIdx = nodeIdx + 1) {
    cp->CloseValidList[nodeIdx] = false;
    for (fieldIdx = 0; fieldIdx < APA_CPATH_NODE_FIELD_COUNT; fieldIdx = fieldIdx + 1) {
      cp->CloseList[nodeIdx][fieldIdx] = 0.0f;
    }
  }
}

/* @source matlab_rewrite ClassComplexPath.m#nodesClear */
static inline void apaCPathNodesClear(ApaComplexPath_t *cp) {
  Integer nodeIdx;

  cp->NodesCntCalcFlag = false;
  cp->NodesCnt = 0;
  cp->OpenCnt = 0;
  cp->CloseCnt = 0;
  for (nodeIdx = 0; nodeIdx < APA_CPATH_OPEN_CAPACITY; nodeIdx = nodeIdx + 1) {
    cp->OpenValidList[nodeIdx] = false;
  }
  for (nodeIdx = 0; nodeIdx < APA_CPATH_CLOSE_CAPACITY; nodeIdx = nodeIdx + 1) {
    cp->CloseValidList[nodeIdx] = false;
  }
}

/* @source matlab_rewrite ClassComplexPath.m#nodesAssign */
static inline void apaCPathNodesAssign(ApaComplexPath_t *dst, const ApaComplexPath_t *src) {
  Integer nodeIdx;
  Integer fieldIdx;

  for (nodeIdx = 0; nodeIdx < APA_CPATH_OPEN_CAPACITY; nodeIdx = nodeIdx + 1) {
    dst->OpenValidList[nodeIdx] = src->OpenValidList[nodeIdx];
    for (fieldIdx = 0; fieldIdx < APA_CPATH_NODE_FIELD_COUNT; fieldIdx = fieldIdx + 1) {
      dst->OpenList[nodeIdx][fieldIdx] = src->OpenList[nodeIdx][fieldIdx];
    }
  }
  for (nodeIdx = 0; nodeIdx < APA_CPATH_CLOSE_CAPACITY; nodeIdx = nodeIdx + 1) {
    dst->CloseValidList[nodeIdx] = src->CloseValidList[nodeIdx];
    for (fieldIdx = 0; fieldIdx < APA_CPATH_NODE_FIELD_COUNT; fieldIdx = fieldIdx + 1) {
      dst->CloseList[nodeIdx][fieldIdx] = src->CloseList[nodeIdx][fieldIdx];
    }
  }
  dst->NodesCnt = src->NodesCnt;
  dst->OpenCnt = src->OpenCnt;
  dst->CloseCnt = src->CloseCnt;
  dst->NodesCntCalcFlag = src->NodesCntCalcFlag;
}

static inline void apaCPathCalcNodesCnt(const ApaComplexPath_t *cp,
                                        Integer *nodesCnt,
                                        Integer *openCnt,
                                        Integer *closeCnt) {
  Integer nodeIdx;
  Integer openTotal;
  Integer closeTotal;

  openTotal = 0;
  closeTotal = 0;
  for (nodeIdx = 0; nodeIdx < APA_CPATH_OPEN_CAPACITY; nodeIdx = nodeIdx + 1) {
    if (cp->OpenValidList[nodeIdx]) {
      openTotal = openTotal + 1;
    }
  }
  for (nodeIdx = 0; nodeIdx < APA_CPATH_CLOSE_CAPACITY; nodeIdx = nodeIdx + 1) {
    if (cp->CloseValidList[nodeIdx]) {
      closeTotal = closeTotal + 1;
    }
  }

  *openCnt = openTotal;
  *closeCnt = closeTotal;
  *nodesCnt = openTotal + closeTotal;
}

static inline void
apaCPathSetNodesCnt(ApaComplexPath_t *cp, Integer nodesCnt, Integer openCnt, Integer closeCnt) {
  cp->NodesCnt = nodesCnt;
  cp->OpenCnt = openCnt;
  cp->CloseCnt = closeCnt;
  cp->NodesCntCalcFlag = true;
}

/* @source matlab_rewrite ClassComplexPath.m#getNodesCnt */
static inline void
apaCPathGetNodesCnt(ApaComplexPath_t *cp, Integer *nodesCnt, Integer *openCnt, Integer *closeCnt) {
  Integer nodesTotal;
  Integer openTotal;
  Integer closeTotal;

  if (cp->NodesCntCalcFlag) {
    nodesTotal = cp->NodesCnt;
    openTotal = cp->OpenCnt;
    closeTotal = cp->CloseCnt;
  } else {
    apaCPathCalcNodesCnt(cp, &nodesTotal, &openTotal, &closeTotal);
    apaCPathSetNodesCnt(cp, nodesTotal, openTotal, closeTotal);
  }

  if (nodesCnt != (Integer *)0) {
    *nodesCnt = nodesTotal;
  }
  if (openCnt != (Integer *)0) {
    *openCnt = openTotal;
  }
  if (closeCnt != (Integer *)0) {
    *closeCnt = closeTotal;
  }
}

/** @brief 拷贝复合路径 */
static inline void apaCPathCopy(ApaComplexPath_t *dst, const ApaComplexPath_t *src) {
  apaPathCopy(&dst->Path, &src->Path);
  dst->SlotId = src->SlotId;
  dst->SlotStatus = src->SlotStatus;
  dst->CollisionCnt = src->CollisionCnt;
  apaCPathNodesAssign(dst, src);
}

/** @brief 清空复合路径 */
static inline void apaCPathClear(ApaComplexPath_t *cp) {
  apaPathClear(&cp->Path);
  cp->CollisionCnt = 0;
  apaCPathNodesClear(cp);
}

/** @brief 重置路径缓存标志 */
static inline void apaCPathResetFlags(ApaComplexPath_t *cp) {
  apaPathResetFlags(&cp->Path);
}

static inline Boolean apaCPathSlotCanFailInSearchingMem(Integer slotStatus) {
  Boolean slotAvailable;

  slotAvailable = false;
  if (slotStatus == APA_SLOT_STATUS_TARGET_SLOT || slotStatus == APA_SLOT_STATUS_FINISH_PLAN ||
      slotStatus == APA_SLOT_STATUS_TARGET_FINISH_REPLAN) {
    slotAvailable = true;
  }
  return slotAvailable;
}

static inline void apaCPathMaybeFailSlotInSearchingMem(ApaComplexPath_t *cp,
                                                       Integer parkingSysMainState,
                                                       Integer pathListType) {
  Boolean slotAvailable;

  slotAvailable = apaCPathSlotCanFailInSearchingMem(cp->SlotStatus);
  if (parkingSysMainState == APA_SYS_STATE_SEARCHING && pathListType == APA_PATH_LIST_MEM &&
      slotAvailable) {
    cp->SlotStatus = APA_SLOT_STATUS_FAIL_PLAN;
  }
}

static inline void apaCPathClearWithFault(ApaComplexPath_t *cp, Integer fault) {
  apaCPathClear(cp);
  cp->Path.ErrorType = fault;
}

static inline Boolean apaCPathPathReady(const ApaComplexPath_t *cp) {
  return cp->Path.ValidFlag && cp->Path.PathN != 0;
}

static inline void apaCPathCoordinateResetNode(Real node[APA_CPATH_NODE_FIELD_COUNT],
                                               const Real posResetMem[3]) {
  Real localX;
  Real localY;
  Real localTht;

  apaCoordGround2Veh(posResetMem[0],
                     posResetMem[1],
                     posResetMem[2],
                     node[1],
                     node[2],
                     node[3],
                     &localX,
                     &localY,
                     &localTht);
  node[1] = localX;
  node[2] = localY;
  node[3] = localTht;
}

/* @source matlab_rewrite ClassComplexPath.m#coordinateReset(open/close nodes) */
static inline void apaCPathCoordinateResetNodes(ApaComplexPath_t *cp, const Real posResetMem[3]) {
  Integer nodeIdx;

  for (nodeIdx = 0; nodeIdx < APA_CPATH_OPEN_CAPACITY; nodeIdx = nodeIdx + 1) {
    if (cp->OpenValidList[nodeIdx]) {
      apaCPathCoordinateResetNode(cp->OpenList[nodeIdx], posResetMem);
    }
  }
  for (nodeIdx = 0; nodeIdx < APA_CPATH_CLOSE_CAPACITY; nodeIdx = nodeIdx + 1) {
    if (cp->CloseValidList[nodeIdx]) {
      apaCPathCoordinateResetNode(cp->CloseList[nodeIdx], posResetMem);
    }
  }
}

/* @source matlab_rewrite ClassComplexPath.m#coordinateReset */
static inline void apaCPathCoordinateReset(ApaComplexPath_t *cp,
                                           Integer pathPosResetIdx,
                                           Integer naviPosResetIdx,
                                           const Real posResetMem[3],
                                           Integer parkingSysMainState,
                                           Integer pathListType) {
  Integer resetDiff;

  if (!cp->Path.ValidFlag) {
    return;
  }

  resetDiff = pathPosResetIdx - naviPosResetIdx;
  if (resetDiff < 0) {
    resetDiff = -resetDiff;
  }
  if (resetDiff == 0) {
    return;
  }
  if (resetDiff != 1 && resetDiff != 15) {
    apaCPathClearWithFault(cp, APA_FAULT_NAVI_RESET_ABNORMAL);
    apaCPathMaybeFailSlotInSearchingMem(cp, parkingSysMainState, pathListType);
    return;
  }

  apaPathCoordinateReset(&cp->Path, posResetMem[0], posResetMem[1], posResetMem[2]);
  apaCPathCoordinateResetNodes(cp, posResetMem);
}

/* @source matlab_rewrite ClassComplexPath.m#parkingDirecSwitchCheck */
static inline void apaCPathParkingDirecSwitchCheck(ApaComplexPath_t *cp,
                                                   Integer parkingDirec,
                                                   Integer slotClass,
                                                   const Real targetPose[3],
                                                   Integer parkingSysMainState,
                                                   Integer pathListType) {
  Real endPose[3];
  Real errX;
  Real errY;
  Real errYaw;
  Real distErr;
  Boolean clearFlag;

  if (!apaCPathPathReady(cp)) {
    return;
  }

  endPose[0] = cp->Path.X[cp->Path.PathN - 1];
  endPose[1] = cp->Path.Y[cp->Path.PathN - 1];
  endPose[2] = cp->Path.Tht[cp->Path.PathN - 1];
  errX = targetPose[0] - endPose[0];
  errY = targetPose[1] - endPose[1];
  errYaw = targetPose[2] - endPose[2];
  if (errYaw < 0.0f) {
    errYaw = -errYaw;
  }
  distErr = apaMathNorm2D(errX, errY);

  clearFlag = false;
  if (parkingDirec == APA_PARKING_DIREC_OUT && apaMathLess(distErr, 0.3f)) {
    clearFlag = true;
  } else if (apaMathGreater(distErr, 1.0f) && apaMathGreater(errYaw, APA_PI * 0.5f) &&
             slotClass == APA_SLOT_CLASS_VERTICAL) {
    clearFlag = true;
  }
  if (!clearFlag) {
    return;
  }

  if (parkingSysMainState != APA_SYS_STATE_PARKING) {
    apaCPathClearWithFault(cp, APA_FAULT_APA_DIRECTION_CHANGED);
  }
  apaCPathMaybeFailSlotInSearchingMem(cp, parkingSysMainState, pathListType);
}

/* @source matlab_rewrite ClassComplexPath.m#addVehPosToPath */
static inline void apaCPathAddVehPosToPath(ApaComplexPath_t *cp,
                                           const Real vehPos[3],
                                           Integer parkingSysMainState,
                                           Integer pathListType,
                                           Real addPtPathThresh) {
  Real distGap;

  if (!apaCPathPathReady(cp)) {
    return;
  }

  distGap = apaMathNorm2D(cp->Path.X[0] - vehPos[0], cp->Path.Y[0] - vehPos[1]);
  if (!apaMathGreater(distGap, addPtPathThresh)) {
    return;
  }

  if (cp->Path.PathN >= APA_PATH_MAX_POINTS - 1) {
    apaCPathClearWithFault(cp, APA_FAULT_EXCEED_MAX_LEN_IN_PLANNER_CORE);
    apaCPathMaybeFailSlotInSearchingMem(cp, parkingSysMainState, pathListType);
    return;
  }

  apaPathPrependPos(&cp->Path, vehPos[0], vehPos[1], vehPos[2]);
}

/* @source matlab_rewrite ClassComplexPath.m#pathEndPtCheck */
static inline void apaCPathPathEndPtCheck(ApaComplexPath_t *cp,
                                          const Real slotPts[8],
                                          const Real slotBounds[4],
                                          const Real targetPose[3],
                                          Integer parkingDirec,
                                          const ApaPlannerVehConfig_t *vehConfig,
                                          Integer parkingSysMainState,
                                          Integer pathListType) {
  Real localBounds[4];
  const Real *resolvedBounds;
  Real endPose[3];
  Real frontX;
  Real frontY;
  Real errDist;
  Real wheelBase;
  Boolean endInSlot;
  Boolean frontInSlot;
  Boolean endZero;
  Boolean invalidEnd;

  if (!apaCPathPathReady(cp)) {
    return;
  }
  if (parkingDirec == APA_PARKING_DIREC_OUT) {
    return;
  }

  resolvedBounds = slotBounds;
  if (resolvedBounds == (const Real *)0) {
    apaSlotCalcSlotBounds(slotPts, localBounds);
    resolvedBounds = localBounds;
  }

  endPose[0] = cp->Path.X[cp->Path.PathN - 1];
  endPose[1] = cp->Path.Y[cp->Path.PathN - 1];
  endPose[2] = cp->Path.Tht[cp->Path.PathN - 1];
  errDist = apaMathNorm2D(endPose[0] - targetPose[0], endPose[1] - targetPose[1]);

  invalidEnd = false;
  if (apaMathGreater(errDist, 1.5f)) {
    invalidEnd = true;
  } else {
    wheelBase = 0.0f;
    if (vehConfig != (const ApaPlannerVehConfig_t *)0) {
      wheelBase = vehConfig->WB;
    }
    apaCoordFrontAxlePoint(endPose[0], endPose[1], endPose[2], wheelBase, &frontX, &frontY);
    endInSlot = apaSlotPointInSlotWithBounds(slotPts, resolvedBounds, endPose[0], endPose[1]);
    frontInSlot = apaSlotPointInSlotWithBounds(slotPts, resolvedBounds, frontX, frontY);
    endZero = apaMathEqual(endPose[0], 0.0f) && apaMathEqual(endPose[1], 0.0f) &&
              apaMathEqual(endPose[2], 0.0f);
    if ((!endInSlot && !frontInSlot) || endZero) {
      invalidEnd = true;
    }
  }

  if (!invalidEnd) {
    return;
  }
  if (parkingSysMainState != APA_SYS_STATE_PARKING) {
    apaCPathClearWithFault(cp, APA_FAULT_PATH_NOT_MATCH_TARPOS);
  }
  if (!cp->Path.ValidFlag) {
    apaCPathMaybeFailSlotInSearchingMem(cp, parkingSysMainState, pathListType);
  }
}

/* @source matlab_rewrite ClassComplexPath.m#pathLengthValidCheck */
static inline void apaCPathPathLengthValidCheck(ApaComplexPath_t *cp,
                                                Real pathLenMin,
                                                Real pathLenMax,
                                                Boolean parkingOutFlag,
                                                Integer parkingSysMainState,
                                                Integer pathListType) {
  Real resolvedMin;
  Boolean lengthValid;

  if (!apaCPathPathReady(cp)) {
    return;
  }

  resolvedMin = pathLenMin;
  if (parkingOutFlag) {
    resolvedMin = 0.1f;
  }
  lengthValid = apaPathGetLengthValid(&cp->Path, resolvedMin, pathLenMax);
  if (lengthValid) {
    return;
  }

  apaCPathClearWithFault(cp, APA_FAULT_PATH_LEN_INVALID);
  apaCPathMaybeFailSlotInSearchingMem(cp, parkingSysMainState, pathListType);
}

/* @source matlab_rewrite ClassComplexPath.m#slotAndPathStatusCheck */
static inline void apaCPathSlotAndPathStatusCheck(ApaComplexPath_t *cp,
                                                  Integer slotId,
                                                  Integer parkingSysMainState,
                                                  Integer pathListType) {
  Boolean slotAvailable;
  Boolean slotWaitOrFail;

  if (slotId == 0) {
    return;
  }

  slotAvailable = false;
  if (cp->SlotStatus == APA_SLOT_STATUS_TARGET_SLOT ||
      cp->SlotStatus == APA_SLOT_STATUS_FINISH_PLAN ||
      cp->SlotStatus == APA_SLOT_STATUS_TARGET_SLOT_REPLAN ||
      cp->SlotStatus == APA_SLOT_STATUS_TARGET_FINISH_REPLAN ||
      cp->SlotStatus == APA_SLOT_STATUS_TARGET_FAIL_REPLAN) {
    slotAvailable = true;
  }
  slotWaitOrFail =
      cp->SlotStatus == APA_SLOT_STATUS_WAIT_PLAN || cp->SlotStatus == APA_SLOT_STATUS_FAIL_PLAN;

  if (!cp->Path.ValidFlag) {
    return;
  }
  if (!slotAvailable && !slotWaitOrFail) {
    apaCPathClearWithFault(cp, APA_FAULT_SPT_CLEAR_SLOT_STATUS);
  } else if (slotWaitOrFail && parkingSysMainState == APA_SYS_STATE_SEARCHING) {
    apaCPathClearWithFault(cp, APA_FAULT_TAR_POS_ERR_EXCEEDING_LIMIT);
    if (pathListType == APA_PATH_LIST_MEM) {
      cp->SlotStatus = APA_SLOT_STATUS_FAIL_PLAN;
    }
  }
}

/* @source matlab_rewrite ClassComplexPath.m#newObsCollisionCheck */
static inline void apaCPathNewObsCollisionCheckWithRisk(ApaComplexPath_t *cp,
                                                        Integer slotId,
                                                        Integer pathCollisionFlag,
                                                        Integer parkingSysMainState,
                                                        Integer pathListType) {
  if (!cp->Path.ValidFlag || slotId == 0) {
    cp->CollisionCnt = 0;
    return;
  }

  if (pathCollisionFlag == APA_COLLISION_RISK_NOT_EXIST) {
    if (cp->CollisionCnt > 0) {
      cp->CollisionCnt = cp->CollisionCnt - 1;
    }
    return;
  }

  if (cp->CollisionCnt <= 10) {
    cp->CollisionCnt = cp->CollisionCnt + 1;
    return;
  }

  apaCPathClearWithFault(cp, APA_FAULT_COLLISION_RISK_AT_NEW_OBS);
  apaCPathMaybeFailSlotInSearchingMem(cp, parkingSysMainState, pathListType);
}

/**
 * @brief 多复合路径容器 — 对应 ClassMultiComplexPath
 *
 * 管理最多 APA_MCPATH_MAX_PATHS 条复合路径（每个车位一条）。
 */
typedef struct {
  ApaComplexPath_t Paths[APA_MCPATH_MAX_PATHS]; /**< 路径数组 */
  Integer PosResetIdx;                          /**< 坐标重置索引 */
  Integer PathListType;                         /**< 路径列表类型 ApaPathListType_e */
  Integer ActiveCount;                          /**< 有效路径数 */
} ApaMultiCPath_t;

/** @brief 初始化多复合路径 */
static inline void apaMCPathInit(ApaMultiCPath_t *mc) {
  Integer i;

  mc->PosResetIdx = 0;
  mc->PathListType = APA_PATH_LIST_INVALID;
  mc->ActiveCount = APA_MCPATH_MAX_PATHS;

  for (i = 0; i < APA_MCPATH_MAX_PATHS; i = i + 1) {
    apaCPathInit(&mc->Paths[i]);
  }
}

/** @brief 拷贝多复合路径 */
static inline void apaMCPathCopy(ApaMultiCPath_t *dst, const ApaMultiCPath_t *src) {
  Integer i;

  dst->PosResetIdx = src->PosResetIdx;
  dst->PathListType = src->PathListType;
  dst->ActiveCount = src->ActiveCount;

  for (i = 0; i < APA_MCPATH_MAX_PATHS; i = i + 1) {
    apaCPathCopy(&dst->Paths[i], &src->Paths[i]);
  }
}

/** @brief 对所有有效路径执行坐标重置 — 对应 ClassMultiComplexPath.coordinateReset() */
static inline void
apaMCPathCoordinateReset(ApaMultiCPath_t *mc, Real resetX, Real resetY, Real resetYaw) {
  Integer i;

  for (i = 0; i < mc->ActiveCount; i = i + 1) {
    if (mc->Paths[i].Path.ValidFlag) {
      apaPathCoordinateReset(&mc->Paths[i].Path, resetX, resetY, resetYaw);
    }
  }
}

/** @brief 重置所有路径的缓存标志 — 对应 ClassMultiComplexPath.resetPathFlags() */
static inline void apaMCPathResetAllFlags(ApaMultiCPath_t *mc) {
  Integer i;

  for (i = 0; i < mc->ActiveCount; i = i + 1) {
    apaCPathResetFlags(&mc->Paths[i]);
  }
}

#endif
