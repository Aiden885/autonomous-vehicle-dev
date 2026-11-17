/**
 * @file apaPathAlgo.h
 * @brief APA 路径算法工具 — 离散化 + 曲率校验
 *
 * 对应 ClassPath.m 中的复杂 static 方法：
 *   - curvatureCheck (L307-L426)  → apaPathCurvatureCheck()
 *   - cubicPolyfitCalc (L1806)    → apaPathCubicPolyfit()
 *   - getCubicPath (L1519-L1594)  → apaPathDiscretizeCubic()
 *
 * 以 static inline 头文件方式提供，组合到 AIDE 模块中使用。
 *
 * @date 2026-04-13
 */
#ifndef APA_PATH_ALGO_H
#define APA_PATH_ALGO_H

#include "baseTypes.h"
#include "apaMacros.h"
#include "apaMathUtils.h"
#include "apaCoordUtils.h"
#include "apaPathTypes.h"
#include "apaVehicleCollisionCheck.h"
#include <math.h>

#ifndef APA_PATH_CUBIC_POINT_COUNT
#define APA_PATH_CUBIC_POINT_COUNT 50
#endif

#ifndef APA_PATH_CUBIC_MAX_PATH_LEN
#define APA_PATH_CUBIC_MAX_PATH_LEN 15.0f
#endif

static inline Boolean apaPathSolve4x4(const Real A[4][4], const Real b[4], Real x[4]) {
  Real aug[4][5];
  Integer i;
  Integer j;
  Integer k;

  for (i = 0; i < 4; i = i + 1) {
    for (j = 0; j < 4; j = j + 1) {
      aug[i][j] = A[i][j];
    }
    aug[i][4] = b[i];
  }

  for (k = 0; k < 4; k = k + 1) {
    Integer maxRow;
    Real maxVal;

    maxRow = k;
    maxVal = aug[k][k];
    if (maxVal < 0.0f) {
      maxVal = -maxVal;
    }

    for (i = k + 1; i < 4; i = i + 1) {
      Real absVal;

      absVal = aug[i][k];
      if (absVal < 0.0f) {
        absVal = -absVal;
      }
      if (absVal > maxVal) {
        maxVal = absVal;
        maxRow = i;
      }
    }
    if (maxVal < 1e-10f) {
      return false;
    }

    if (maxRow != k) {
      for (j = 0; j < 5; j = j + 1) {
        Real tmp;

        tmp = aug[k][j];
        aug[k][j] = aug[maxRow][j];
        aug[maxRow][j] = tmp;
      }
    }

    for (i = k + 1; i < 4; i = i + 1) {
      Real factor;

      factor = aug[i][k] / aug[k][k];
      for (j = k; j < 5; j = j + 1) {
        aug[i][j] = aug[i][j] - factor * aug[k][j];
      }
    }
  }

  for (i = 3; i >= 0; i = i - 1) {
    x[i] = aug[i][4];
    for (j = i + 1; j < 4; j = j + 1) {
      x[i] = x[i] - aug[i][j] * x[j];
    }
    x[i] = x[i] / aug[i][i];
  }
  return true;
}

static inline Boolean
apaPathCubicPolyfit(const Real pos1[3], const Real pos2[3], Integer *polyfitType, Real coe[4]) {
  Real x1;
  Real y1;
  Real a1;
  Real x2;
  Real y2;
  Real a2;
  Real absDx;
  Real absDy;
  Real A[4][4];
  Real b[4];

  x1 = pos1[0];
  y1 = pos1[1];
  a1 = pos1[2];
  x2 = pos2[0];
  y2 = pos2[1];
  a2 = pos2[2];

  absDx = x1 - x2;
  if (absDx < 0.0f) {
    absDx = -absDx;
  }
  absDy = y1 - y2;
  if (absDy < 0.0f) {
    absDy = -absDy;
  }

  if (absDx >= absDy) {
    if (absDx < 1e-4f) {
      *polyfitType = APA_POLYFIT_INVALID;
      return false;
    }
    *polyfitType = APA_POLYFIT_DOMAIN_X;
    A[0][0] = 1.0f;
    A[0][1] = x1;
    A[0][2] = x1 * x1;
    A[0][3] = x1 * x1 * x1;
    A[1][0] = 1.0f;
    A[1][1] = x2;
    A[1][2] = x2 * x2;
    A[1][3] = x2 * x2 * x2;
    A[2][0] = 0.0f;
    A[2][1] = 1.0f;
    A[2][2] = 2.0f * x1;
    A[2][3] = 3.0f * x1 * x1;
    A[3][0] = 0.0f;
    A[3][1] = 1.0f;
    A[3][2] = 2.0f * x2;
    A[3][3] = 3.0f * x2 * x2;
    b[0] = y1;
    b[1] = y2;
    b[2] = tanf(a1);
    b[3] = tanf(a2);
  } else {
    if (absDy < 1e-4f) {
      *polyfitType = APA_POLYFIT_INVALID;
      return false;
    }
    *polyfitType = APA_POLYFIT_DOMAIN_Y;
    A[0][0] = 1.0f;
    A[0][1] = y1;
    A[0][2] = y1 * y1;
    A[0][3] = y1 * y1 * y1;
    A[1][0] = 1.0f;
    A[1][1] = y2;
    A[1][2] = y2 * y2;
    A[1][3] = y2 * y2 * y2;
    A[2][0] = 0.0f;
    A[2][1] = 1.0f;
    A[2][2] = 2.0f * y1;
    A[2][3] = 3.0f * y1 * y1;
    A[3][0] = 0.0f;
    A[3][1] = 1.0f;
    A[3][2] = 2.0f * y2;
    A[3][3] = 3.0f * y2 * y2;
    a1 = APA_HALF_PI - a1;
    a2 = APA_HALF_PI - a2;
    b[0] = x1;
    b[1] = x2;
    b[2] = tanf(a1);
    b[3] = tanf(a2);
  }

  return apaPathSolve4x4((const Real(*)[4])A, b, coe);
}

/**
 * @brief 路径曲率校验 — 对应 ClassPath.curvatureCheck
 *
 * 使用滑动窗口法，在每个路径段内对积分曲率/积分里程取平均半径。
 *
 * @param x,y 路径坐标
 * @param odoStep 里程步进
 * @param pathN 路径点数
 * @param radiusThresh 最小允许半径
 * @param radius [out] 每点的曲率半径（填充）
 * @return true = 曲率合格
 */
static inline Boolean apaPathCurvatureCheck(const Real *x,
                                            const Real *y,
                                            const Real *odoStep,
                                            Integer pathN,
                                            Real radiusThresh,
                                            Real *radius) {
  Real odoThresh;
  Boolean existRisk;
  Real pathKappa[APA_PATH_MAX_POINTS];
  Real pathKappaInt[APA_PATH_MAX_POINTS];
  Real segOdoList[APA_PATH_MAX_POINTS];
  Integer segCnt;
  Integer segIdx;
  Real odoThreshSeg;
  Integer ptrLeft;
  Integer ptrRight;
  Real odoInt;
  Real kappaInt;
  Integer i;

  if (pathN < 3) {
    return true;
  }

  odoThresh = 0.8f;

  radius[0] = APA_MATH_INF;
  radius[pathN - 1] = APA_MATH_INF;
  for (i = 1; i < pathN - 1; i = i + 1) {
    Real prevDx;
    Real prevDy;
    Real nextDx;
    Real nextDy;
    Real prevLen;
    Real nextLen;
    Real lenProduct;
    Real crossAbs;
    Real kappa;

    prevDx = x[i] - x[i - 1];
    prevDy = y[i] - y[i - 1];
    nextDx = x[i + 1] - x[i];
    nextDy = y[i + 1] - y[i];
    prevLen = sqrtf(prevDx * prevDx + prevDy * prevDy);
    nextLen = sqrtf(nextDx * nextDx + nextDy * nextDy);
    lenProduct = prevLen * nextLen;
    crossAbs = prevDx * nextDy - prevDy * nextDx;
    if (crossAbs < 0.0f) {
      crossAbs = -crossAbs;
    }

    if ((lenProduct < APA_MATH_EPSILON) || (crossAbs <= (lenProduct * 1.0e-4f))) {
      radius[i] = APA_MATH_INF;
    } else {
      kappa = apaMathCircleCurvature(x[i], y[i], x[i - 1], y[i - 1], x[i + 1], y[i + 1], false);
      if (apaMathEqual(kappa, 0.0f)) {
        radius[i] = APA_MATH_INF;
      } else {
        radius[i] = 1.0f / apaMathDivideZeroProcess(kappa);
      }
    }
    if (radius[i] < 0.0f) {
      radius[i] = -radius[i];
    }
  }

  existRisk = false;
  for (i = 1; i < pathN - 1; i = i + 1) {
    if (apaMathLess(radius[i], radiusThresh)) {
      if (apaMathLess(radius[i], radiusThresh - 2.0f)) {
        return false;
      }
      existRisk = true;
    }
  }
  if (!existRisk) {
    return true;
  }

  for (i = 0; i < pathN; i = i + 1) {
    pathKappa[i] = 0.0f;
    pathKappaInt[i] = 0.0f;
  }
  for (i = 1; i < pathN - 1; i = i + 1) {
    pathKappa[i] = 1.0f / apaMathDivideZeroProcess(radius[i]);
  }
  for (i = 1; i < pathN - 1; i = i + 1) {
    Real absOdo;

    absOdo = odoStep[i - 1];
    if (absOdo < 0.0f) {
      absOdo = -absOdo;
    }
    pathKappaInt[i] = pathKappa[i] * absOdo;
    if (i == pathN - 2) {
      absOdo = odoStep[i];
      if (absOdo < 0.0f) {
        absOdo = -absOdo;
      }
      pathKappaInt[i] = pathKappaInt[i] + pathKappa[i] * absOdo;
    }
  }

  segCnt = 1;
  segOdoList[0] = 0.0f;
  {
    Real absOdo;

    absOdo = odoStep[0];
    if (absOdo < 0.0f) {
      absOdo = -absOdo;
    }
    segOdoList[0] = absOdo;
  }
  for (i = 1; i < pathN - 1; i = i + 1) {
    Real product;
    Real absOdo;

    product = odoStep[i - 1] * odoStep[i];
    if (apaMathLess(product, 0.0f)) {
      segCnt = segCnt + 1;
      segOdoList[segCnt - 1] = 0.0f;
    }
    absOdo = odoStep[i];
    if (absOdo < 0.0f) {
      absOdo = -absOdo;
    }
    segOdoList[segCnt - 1] = segOdoList[segCnt - 1] + absOdo;
  }

  segIdx = 0;
  if (!apaMathGreater(segOdoList[0], odoThresh)) {
    odoThreshSeg = segOdoList[0] * 0.8f;
  } else {
    odoThreshSeg = odoThresh;
  }

  ptrLeft = 1;
  ptrRight = 1;
  {
    Real absOdo;

    absOdo = odoStep[ptrRight - 1];
    if (absOdo < 0.0f) {
      absOdo = -absOdo;
    }
    odoInt = absOdo;
  }
  kappaInt = pathKappaInt[ptrRight];

  while (ptrRight < pathN - 1) {
    Real product;

    product = odoStep[ptrRight - 1] * odoStep[ptrRight];

    if (apaMathLess(product, 0.0f)) {

      ptrLeft = ptrRight + 1;
      ptrRight = ptrLeft;
      {
        Real absOdo;

        absOdo = odoStep[ptrRight - 1];
        if (absOdo < 0.0f) {
          absOdo = -absOdo;
        }
        odoInt = absOdo;
      }
      kappaInt = pathKappaInt[ptrRight];
      segIdx = segIdx + 1;
      if (segIdx < segCnt) {
        if (!apaMathGreater(segOdoList[segIdx], odoThresh)) {
          odoThreshSeg = segOdoList[segIdx] * 0.8f;
        } else {
          odoThreshSeg = odoThresh;
        }
        if (odoThreshSeg < 0.05f) {
          odoThreshSeg = 0.05f;
        }
      }
    } else if (apaMathLess(odoInt, odoThreshSeg) || (ptrRight - ptrLeft <= 1)) {

      ptrRight = ptrRight + 1;
      if (ptrRight < pathN) {
        Real absOdo;

        absOdo = odoStep[ptrRight - 1];
        if (absOdo < 0.0f) {
          absOdo = -absOdo;
        }
        odoInt = odoInt + absOdo;
        kappaInt = kappaInt + pathKappaInt[ptrRight];
      }
    } else {
      Real kappaAvg;
      Real radiusAvg;

      kappaAvg = kappaInt / apaMathDivideZeroProcess(odoInt);
      radiusAvg = 1.0f / apaMathDivideZeroProcess(kappaAvg);
      if (apaMathLess(radiusAvg, radiusThresh)) {
        return false;
      }
      {
        Real absOdo;

        absOdo = odoStep[ptrLeft - 1];
        if (absOdo < 0.0f) {
          absOdo = -absOdo;
        }
        odoInt = odoInt - absOdo;
      }
      kappaInt = kappaInt - pathKappaInt[ptrLeft];
      ptrLeft = ptrLeft + 1;
    }
  }
  return true;
}

static inline Integer apaPathCubicResolveDirSign(const Real startPose[3], Real obsX, Real obsY) {
  Real deltaX;
  Real deltaY;
  Real deltaNorm;
  Real headingDot;

  deltaX = obsX - startPose[0];
  deltaY = obsY - startPose[1];
  deltaNorm = sqrtf(deltaX * deltaX + deltaY * deltaY);
  if (deltaNorm < 1.0e-6f) {
    return 1;
  }

  headingDot =
      (deltaX / deltaNorm) * cosf(startPose[2]) + (deltaY / deltaNorm) * sinf(startPose[2]);
  if (headingDot > 1.0e-4f) {
    return 1;
  }
  if (headingDot < -1.0e-4f) {
    return -1;
  }
  return 0;
}

static inline Real apaPathCalcAbsLength(const ApaPath_t *path) {
  Integer pointIndex;
  Real pathLen;

  pathLen = 0.0f;
  pointIndex = 0;
  while ((pointIndex + 1 < path->PathN) && (pointIndex < APA_PATH_MAX_POINTS)) {
    pathLen = pathLen + fabsf(path->OdoStep[pointIndex]);
    pointIndex = pointIndex + 1;
  }
  return pathLen;
}

/* Sample the fitted polynomial with its analytic tangent. A floating-point
 * coordinate walk plus endpoint repair can leave nearly coincident points
 * with different headings, creating a fictitious curvature spike. */
static inline Boolean apaPathDiscretizeDoubleCubicObserved(const Real startPose[3],
                                                           const Real endPose[3],
                                                           ApaPath_t *path,
                                                           const char **stage) {
  if (stage)
    *stage = "curve_fit";
  const Real yawErr = fabsf(apaCoordWrapToPi(endPose[2] - startPose[2]));
  Integer domain = APA_POLYFIT_INVALID;
  Real coeff[4];
  if (yawErr > APA_HALF_PI - 0.01f || !apaPathCubicPolyfit(startPose, endPose, &domain, coeff))
    return false;
  const Integer direction = apaPathCubicResolveDirSign(startPose, endPose[0], endPose[1]);
  if (!direction)
    return false;
  const Integer count = APA_PATH_CUBIC_POINT_COUNT;
  if (count < 2 || count + 1 > APA_PATH_MAX_POINTS) {
    if (stage)
      *stage = "capacity_curve";
    return false;
  }
  const Integer axis = domain == APA_POLYFIT_DOMAIN_X ? 0 : 1;
  apaPathInit(path);
  path->PathSource = APA_PATH_SOURCE_SEARCH;
  path->PathType = APA_PATH_TYPE_ALL;
  path->PathN = count + 1;
  for (Integer i = 0; i <= count; ++i) {
    const Real t = (Real)i / (Real)count;
    const Real u = startPose[axis] + t * (endPose[axis] - startPose[axis]);
    const Real value = coeff[0] + u * (coeff[1] + u * (coeff[2] + u * coeff[3]));
    const Real slope = coeff[1] + u * (2.0f * coeff[2] + 3.0f * u * coeff[3]);
    Real heading = axis == 0 ? atan2f(slope, 1.0f) : atan2f(1.0f, slope);
    if (cosf(heading - startPose[2]) < 0.0f)
      heading += APA_PI;
    path->X[i] = axis == 0 ? u : value;
    path->Y[i] = axis == 0 ? value : u;
    path->Tht[i] = apaCoordWrapToPi(heading);
  }
  path->X[0] = startPose[0];
  path->Y[0] = startPose[1];
  path->Tht[0] = startPose[2];
  path->X[count] = endPose[0];
  path->Y[count] = endPose[1];
  path->Tht[count] = endPose[2];
  if (stage)
    *stage = "curve_sampling";
  const Real yawStepLimit = fmaxf(3.0f * yawErr / count, 30.0f * APA_PI / 180.0f);
  for (Integer i = 0; i < count; ++i) {
    const Real length = hypotf(path->X[i + 1] - path->X[i], path->Y[i + 1] - path->Y[i]);
    if (length <= 1.e-6f || fabsf(apaCoordWrapToPi(path->Tht[i + 1] - path->Tht[i])) > yawStepLimit)
      return false;
    path->OdoStep[i] = direction * length;
  }
  path->ValidFlag = true;
  path->ErrorType = APA_FAULT_NO_ERR;
  return true;
}

static inline Boolean
apaPathDiscretizeDoubleCubic(const Real startPose[3], const Real endPose[3], ApaPath_t *path) {
  return apaPathDiscretizeDoubleCubicObserved(startPose, endPose, path, 0);
}

static inline Boolean apaPathDiscretizeCubicObserved(const Real startPose[3],
                                                     const Real endPose[3],
                                                     Real minCircle,
                                                     ApaPath_t *path,
                                                     const char **stage) {
  Real startPoseUse[3];
  Real endPoseUse[3];
  Real endPoseLocal[3];
  ApaPath_t localPath;
  Integer pointIndex;
  Real pathLenLimit;
  Real rminUse;

  startPoseUse[0] = startPose[0];
  startPoseUse[1] = startPose[1];
  startPoseUse[2] = apaCoordWrapToPi(startPose[2]);
  endPoseUse[0] = endPose[0];
  endPoseUse[1] = endPose[1];
  endPoseUse[2] = apaCoordWrapToPi(endPose[2]);
  rminUse = minCircle;
  if (rminUse < APA_MATH_EPSILON) {
    rminUse = 5.0f;
  }

  apaCoordGround2Veh(startPoseUse[0],
                     startPoseUse[1],
                     startPoseUse[2],
                     endPoseUse[0],
                     endPoseUse[1],
                     endPoseUse[2],
                     &endPoseLocal[0],
                     &endPoseLocal[1],
                     &endPoseLocal[2]);

  if (!apaPathDiscretizeDoubleCubicObserved(
          (const Real[3]){0.0f, 0.0f, 0.0f}, endPoseLocal, &localPath, stage)) {
    return false;
  }

  pathLenLimit =
      2.0f * sqrtf((startPoseUse[0] - endPoseUse[0]) * (startPoseUse[0] - endPoseUse[0]) +
                   (startPoseUse[1] - endPoseUse[1]) * (startPoseUse[1] - endPoseUse[1]));
  pathLenLimit =
      pathLenLimit + 2.0f * rminUse * fabsf(apaCoordWrapToPi(startPoseUse[2] - endPoseUse[2]));
  if (pathLenLimit < APA_PATH_CUBIC_MAX_PATH_LEN) {
    pathLenLimit = APA_PATH_CUBIC_MAX_PATH_LEN;
  }
  if (stage)
    *stage = "curve_length";
  if (apaPathCalcAbsLength(&localPath) > pathLenLimit) {
    return false;
  }

  *path = localPath;
  pointIndex = 0;
  while ((pointIndex < path->PathN) && (pointIndex < APA_PATH_MAX_POINTS)) {
    Real globalX;
    Real globalY;
    Real globalTheta;

    apaCoordVeh2Ground(startPoseUse[0],
                       startPoseUse[1],
                       startPoseUse[2],
                       path->X[pointIndex],
                       path->Y[pointIndex],
                       path->Tht[pointIndex],
                       &globalX,
                       &globalY,
                       &globalTheta);
    path->X[pointIndex] = globalX;
    path->Y[pointIndex] = globalY;
    path->Tht[pointIndex] = globalTheta;
    pointIndex = pointIndex + 1;
  }

  path->X[0] = startPoseUse[0];
  path->Y[0] = startPoseUse[1];
  path->Tht[0] = startPoseUse[2];
  path->X[path->PathN - 1] = endPoseUse[0];
  path->Y[path->PathN - 1] = endPoseUse[1];
  path->Tht[path->PathN - 1] = endPoseUse[2];
  if (stage)
    *stage = "curvature_initial";
  if (!apaPathCurvatureCheck(path->X, path->Y, path->OdoStep, path->PathN, rminUse, path->Radius)) {
    return false;
  }
  path->RadiusFlag = true;
  return true;
}

static inline Boolean apaPathDiscretizeCubic(const Real startPose[3],
                                             const Real endPose[3],
                                             Real minCircle,
                                             ApaPath_t *path) {
  return apaPathDiscretizeCubicObserved(startPose, endPose, minCircle, path, 0);
}

static inline Boolean
apaPathJumpAppendTail(ApaPath_t *processedPath, const ApaPath_t *sourcePath, Integer ptrEnd) {
  Boolean appendOk;
  Integer srcIndex;

  appendOk = true;
  srcIndex = ptrEnd + 1;
  while (srcIndex < sourcePath->PathN) {
    Integer dstIndex;

    dstIndex = processedPath->PathN;
    if (dstIndex >= APA_PATH_MAX_POINTS) {
      appendOk = false;
      srcIndex = sourcePath->PathN;
    } else {
      processedPath->X[dstIndex] = sourcePath->X[srcIndex];
      processedPath->Y[dstIndex] = sourcePath->Y[srcIndex];
      processedPath->Tht[dstIndex] = sourcePath->Tht[srcIndex];
      if (dstIndex > 0) {
        processedPath->OdoStep[dstIndex - 1] = sourcePath->OdoStep[srcIndex - 1];
      }
      processedPath->PathN = processedPath->PathN + 1;
      srcIndex = srcIndex + 1;
    }
  }
  if (appendOk && processedPath->PathN > 0) {
    processedPath->OdoStep[processedPath->PathN - 1] = 0.0f;
  }
  return appendOk;
}

static inline Boolean apaPathJumpBuildCandidateObserved(const ApaPath_t *sourcePath,
                                                        Integer ptrStart,
                                                        Integer ptrEnd,
                                                        Integer expDirecFlag,
                                                        ApaPathCollisionProbeFunc collisionProbe,
                                                        void *probeContext,
                                                        Boolean checkSlotArea,
                                                        ApaPathNearSlotProbeFunc nearSlotProbe,
                                                        Real pathLenMin,
                                                        Real pathLenMax,
                                                        Real minCircle,
                                                        ApaPath_t *candidatePath,
                                                        const char **stage) {
  Boolean candidateValid;
  Boolean cubicValid;
  Boolean curvatureValid;
  Boolean appendOk;
  Integer collisionFlag;
  Real startPose[3];
  Real endPose[3];
  Real pathLen;
  ApaPath_t cubicPath;

  candidateValid = false;
  startPose[0] = sourcePath->X[ptrStart];
  startPose[1] = sourcePath->Y[ptrStart];
  startPose[2] = sourcePath->Tht[ptrStart];
  endPose[0] = sourcePath->X[ptrEnd];
  endPose[1] = sourcePath->Y[ptrEnd];
  endPose[2] = sourcePath->Tht[ptrEnd];

  cubicValid = apaPathDiscretizeCubicObserved(startPose, endPose, minCircle, &cubicPath, stage);
  if (cubicValid) {
    apaPathSimplifyStraightPath(&cubicPath, 0.02f, 0.5f);
    if (stage)
      *stage = "curvature_simplified";
    curvatureValid = apaPathCurvatureCheck(
        cubicPath.X, cubicPath.Y, cubicPath.OdoStep, cubicPath.PathN, minCircle, cubicPath.Radius);
    if (curvatureValid) {
      cubicPath.RadiusFlag = true;
      if (stage)
        *stage = "collision";
      collisionFlag = apaPathGetCollisionFlagWithProbe(
          &cubicPath, collisionProbe, probeContext, checkSlotArea, nearSlotProbe);
      if (collisionFlag != APA_COLLISION_RISK_EXIST) {
        if (stage)
          *stage = "capacity";
        const Integer total = sourcePath->PathN - (ptrEnd - ptrStart) + cubicPath.PathN - 1;
        if (total > APA_PATH_MAX_POINTS)
          return false;
        apaPathCopy(candidatePath, sourcePath);
        candidatePath->PathN = ptrStart + 1;
        apaPathAppend(candidatePath, &cubicPath);
        if (candidatePath->PathN != ptrStart + cubicPath.PathN)
          return false;
        appendOk = apaPathJumpAppendTail(candidatePath, sourcePath, ptrEnd);
        if (appendOk) {
          if (stage)
            *stage = "length";
          pathLen = apaPathCalcAbsLength(candidatePath);
          if (!apaMathLess(pathLen, pathLenMin) && !apaMathGreater(pathLen, pathLenMax)) {
            if (stage)
              *stage = "direction";
            candidateValid = apaPathGetForceReverseValid(candidatePath, expDirecFlag);
          }
        }
      }
    }
  }
  return candidateValid;
}

static inline Boolean apaPathJumpBuildCandidate(const ApaPath_t *sourcePath,
                                                Integer ptrStart,
                                                Integer ptrEnd,
                                                Integer expDirecFlag,
                                                ApaPathCollisionProbeFunc collisionProbe,
                                                void *probeContext,
                                                Boolean checkSlotArea,
                                                ApaPathNearSlotProbeFunc nearSlotProbe,
                                                Real pathLenMin,
                                                Real pathLenMax,
                                                Real minCircle,
                                                ApaPath_t *candidatePath) {
  return apaPathJumpBuildCandidateObserved(sourcePath,
                                           ptrStart,
                                           ptrEnd,
                                           expDirecFlag,
                                           collisionProbe,
                                           probeContext,
                                           checkSlotArea,
                                           nearSlotProbe,
                                           pathLenMin,
                                           pathLenMax,
                                           minCircle,
                                           candidatePath,
                                           0);
}

/**
 * @brief 尝试用三次曲线路径跳过中间换向点 — 对应 ClassPath.jumpPoints()
 */
typedef Boolean (*ApaPathJumpAcceptFunc)(const ApaPath_t *candidate, void *context);

/* begin/result bracket each attempted splice; no observer can affect admission. */
typedef void (*ApaPathJumpObserveFunc)(const ApaPath_t *source,
                                       Integer round,
                                       Integer start,
                                       Integer end,
                                       const char *stage,
                                       void *context);

static inline Boolean apaPathJumpPointsObserved(ApaPath_t *path,
                                                Integer slotClass,
                                                Integer expDirecFlag,
                                                ApaPathCollisionProbeFunc collisionProbe,
                                                void *probeContext,
                                                Boolean checkSlotArea,
                                                ApaPathNearSlotProbeFunc nearSlotProbe,
                                                Real pathLenMin,
                                                Real pathLenMax,
                                                Real minCircle,
                                                ApaPathJumpAcceptFunc accept,
                                                void *acceptContext,
                                                ApaPathJumpObserveFunc observe,
                                                void *observeContext) {
  Boolean changed;
  Boolean loopActive;
  Integer loopCount;
  Integer pathJumpStartPtr;

  changed = false;
  loopActive = true;
  loopCount = 0;
  pathJumpStartPtr = 0;
  while (loopActive) {
    Integer segNum;
    Integer revPtsNum;
    Integer revPtsPtrList[APA_PATH_MAX_POINTS];
    Real segOdoList[APA_PATH_MAX_POINTS];
    Boolean polyfitFlag;
    Integer revIndex;

    if (loopCount >= 5 || pathJumpStartPtr >= path->PathN) {
      loopActive = false;
    } else {
      loopCount = loopCount + 1;
      segNum = 0;
      revPtsNum = 0;
      apaPathCalcSegNumAndRevPtr(path, &segNum, revPtsPtrList, segOdoList, &revPtsNum);
      (void)segNum;
      (void)segOdoList;
      if (revPtsNum <= 2) {
        loopActive = false;
      } else {
        polyfitFlag = false;
        revIndex = 0;
        while (revIndex < revPtsNum - 2 && !polyfitFlag) {
          Integer ptrStart;
          Integer ptrEndIndex;

          ptrStart = revPtsPtrList[revIndex];
          pathJumpStartPtr = ptrStart;
          ptrEndIndex = revIndex + 2;
          while (ptrEndIndex < revPtsNum && !polyfitFlag) {
            Integer ptrEnd;
            Boolean endpointUsable;
            ApaPath_t candidatePath;

            ptrEnd = revPtsPtrList[ptrEndIndex];
            endpointUsable = true;
            if (slotClass == APA_SLOT_CLASS_VERTICAL && ptrEndIndex == revPtsNum - 1) {
              ptrEnd = ptrEnd - 1;
              if (ptrEnd <= revPtsPtrList[ptrEndIndex - 1]) {
                endpointUsable = false;
              }
            }

            if (observe)
              observe(path, loopCount, ptrStart, ptrEnd, "begin", observeContext);
            const char *stage = "endpoint";
            if (endpointUsable) {
              polyfitFlag = apaPathJumpBuildCandidateObserved(path,
                                                              ptrStart,
                                                              ptrEnd,
                                                              expDirecFlag,
                                                              collisionProbe,
                                                              probeContext,
                                                              checkSlotArea,
                                                              nearSlotProbe,
                                                              pathLenMin,
                                                              pathLenMax,
                                                              minCircle,
                                                              &candidatePath,
                                                              &stage);
              if (polyfitFlag && accept) {
                stage = "acceptance";
                polyfitFlag = accept(&candidatePath, acceptContext);
              }
              if (observe)
                observe(path,
                        loopCount,
                        ptrStart,
                        ptrEnd,
                        polyfitFlag ? "accepted" : stage,
                        observeContext);
              if (polyfitFlag) {
                apaPathCopy(path, &candidatePath);
                apaPathResetFlags(path);
                changed = true;
              }
            }
            if (!endpointUsable && observe)
              observe(path, loopCount, ptrStart, ptrEnd, stage, observeContext);
            ptrEndIndex = ptrEndIndex + 1;
          }
          revIndex = revIndex + 1;
        }
        if (!polyfitFlag) {
          loopActive = false;
        }
      }
    }
  }
  return changed;
}

static inline Boolean apaPathJumpPointsWithAcceptance(ApaPath_t *path,
                                                      Integer slotClass,
                                                      Integer expDirecFlag,
                                                      ApaPathCollisionProbeFunc collisionProbe,
                                                      void *probeContext,
                                                      Boolean checkSlotArea,
                                                      ApaPathNearSlotProbeFunc nearSlotProbe,
                                                      Real pathLenMin,
                                                      Real pathLenMax,
                                                      Real minCircle,
                                                      ApaPathJumpAcceptFunc accept,
                                                      void *acceptContext) {
  return apaPathJumpPointsObserved(path,
                                   slotClass,
                                   expDirecFlag,
                                   collisionProbe,
                                   probeContext,
                                   checkSlotArea,
                                   nearSlotProbe,
                                   pathLenMin,
                                   pathLenMax,
                                   minCircle,
                                   accept,
                                   acceptContext,
                                   0,
                                   0);
}

static inline Boolean apaPathJumpPointsWithProbe(ApaPath_t *path,
                                                 Integer slotClass,
                                                 Integer expDirecFlag,
                                                 ApaPathCollisionProbeFunc collisionProbe,
                                                 void *probeContext,
                                                 Boolean checkSlotArea,
                                                 ApaPathNearSlotProbeFunc nearSlotProbe,
                                                 Real pathLenMin,
                                                 Real pathLenMax,
                                                 Real minCircle) {
  return apaPathJumpPointsWithAcceptance(path,
                                         slotClass,
                                         expDirecFlag,
                                         collisionProbe,
                                         probeContext,
                                         checkSlotArea,
                                         nearSlotProbe,
                                         pathLenMin,
                                         pathLenMax,
                                         minCircle,
                                         (ApaPathJumpAcceptFunc)0,
                                         (void *)0);
}

/**
 * @brief 复合路径跳点 wrapper — 对应 ClassComplexPath.jumpPoints()
 */
static inline Boolean apaCPathJumpPointsWithProbe(ApaComplexPath_t *complexPath,
                                                  Integer slotClass,
                                                  Integer parkingDirec,
                                                  Integer expDirecFlag,
                                                  ApaPathCollisionProbeFunc collisionProbe,
                                                  void *probeContext,
                                                  Boolean checkSlotArea,
                                                  ApaPathNearSlotProbeFunc nearSlotProbe,
                                                  Real pathLenMin,
                                                  Real pathLenMax,
                                                  Real minCircle) {
  Boolean changed;
  Boolean pathReady;
  Boolean targetSlot;
  Boolean parkingOut;

  changed = false;
  pathReady = complexPath->Path.ValidFlag && (complexPath->Path.PathN != 0);
  targetSlot = (complexPath->SlotStatus == APA_SLOT_STATUS_TARGET_SLOT) ||
               (complexPath->SlotStatus == APA_SLOT_STATUS_TARGET_SLOT_REPLAN);
  parkingOut = parkingDirec == APA_PARKING_DIREC_OUT;
  if (pathReady && targetSlot && !parkingOut) {
    changed = apaPathJumpPointsWithProbe(&complexPath->Path,
                                         slotClass,
                                         expDirecFlag,
                                         collisionProbe,
                                         probeContext,
                                         checkSlotArea,
                                         nearSlotProbe,
                                         pathLenMin,
                                         pathLenMax,
                                         minCircle);
  }
  return changed;
}

typedef struct {
  const ApaVehicleCollisionCheckContext_t *CollisionContext;
  Integer LastPointIndex;
  Boolean LastCollision;
  ApaVehicleCollisionCheckResult_t LastResult;
} ApaPathVehicleCollisionProbeContext_t;

static inline void
apaPathVehicleCollisionProbeContextInit(ApaPathVehicleCollisionProbeContext_t *context,
                                        const ApaVehicleCollisionCheckContext_t *collisionContext) {
  if (context == (ApaPathVehicleCollisionProbeContext_t *)0) {
    return;
  }
  context->CollisionContext = collisionContext;
  context->LastPointIndex = -1;
  context->LastCollision = false;
  apaVehicleCollisionCheckResultInit(&context->LastResult);
}

static inline Boolean apaPathVehicleCollisionPoseProbe(
    Integer pointIndex, Real posX, Real posY, Real posTheta, void *context) {
  ApaPathVehicleCollisionProbeContext_t *probeContext;
  ApaVehicleCollisionCheckResult_t result;
  Boolean collisionDetected;

  probeContext = (ApaPathVehicleCollisionProbeContext_t *)context;
  if (probeContext == (ApaPathVehicleCollisionProbeContext_t *)0 ||
      probeContext->CollisionContext == (const ApaVehicleCollisionCheckContext_t *)0 ||
      probeContext->CollisionContext->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return true;
  }

  apaVehicleCollisionCheckResultInit(&result);
  collisionDetected =
      apaVehicleCollisionCheckPose(probeContext->CollisionContext, posX, posY, posTheta, &result);
  probeContext->LastPointIndex = pointIndex;
  probeContext->LastCollision = collisionDetected;
  probeContext->LastResult = result;
  return collisionDetected;
}

static inline Boolean apaPathVehicleCollisionNearSlotProbe(
    Integer pointIndex, Real posX, Real posY, Real posTheta, void *context) {
  ApaPathVehicleCollisionProbeContext_t *probeContext;
  const ApaPlannerConfig_t *plannerConfig;
  const ApaPlannerVehConfig_t *vehConfig;

  (void)pointIndex;
  probeContext = (ApaPathVehicleCollisionProbeContext_t *)context;
  if (probeContext == (ApaPathVehicleCollisionProbeContext_t *)0 ||
      probeContext->CollisionContext == (const ApaVehicleCollisionCheckContext_t *)0 ||
      probeContext->CollisionContext->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    return false;
  }

  plannerConfig = apaPlannerGlobalGetPlanner(probeContext->CollisionContext->PlannerGlobal);
  vehConfig = &plannerConfig->Veh;
  return apaVehicleCollisionCalcVehNearSlotFlag(
      vehConfig, probeContext->CollisionContext, posX, posY, posTheta);
}

/** @brief 在同一碰撞环境下比较路径质量。 */
static inline Integer apaPathQualityCompareWithVehicleCollisionCheck(
    ApaPath_t *originPath,
    ApaPath_t *candidatePath,
    const ApaPathQualityCompareContext_t *context,
    const ApaVehicleCollisionCheckContext_t *collisionContext,
    Boolean checkSlotArea) {
  ApaPathVehicleCollisionProbeContext_t originProbeContext;
  ApaPathVehicleCollisionProbeContext_t candidateProbeContext;

  apaPathVehicleCollisionProbeContextInit(&originProbeContext, collisionContext);
  apaPathVehicleCollisionProbeContextInit(&candidateProbeContext, collisionContext);
  return apaPathQualityCompareWithCollisionProbe(originPath,
                                                 candidatePath,
                                                 context,
                                                 apaPathVehicleCollisionPoseProbe,
                                                 &originProbeContext,
                                                 apaPathVehicleCollisionPoseProbe,
                                                 &candidateProbeContext,
                                                 checkSlotArea,
                                                 apaPathVehicleCollisionNearSlotProbe);
}

static inline void
apaCPathConfigureNewObsCollisionContext(ApaVehicleCollisionCheckContext_t *collisionWork,
                                        ApaSlot_t *slot,
                                        Boolean detectSearchingPathStopperForParSlot,
                                        Boolean parkingOutFlag) {
  Integer slotClass;
  Boolean clearStopper;

  apaVehicleCollisionCheckContextSetCheckType(collisionWork, APA_COLLISION_CHECK_GENERAL);
  apaVehicleCollisionCheckContextSetCollisionDemand(collisionWork, APA_COLLISION_DEMAND_SIMPLIFY);
  apaVehicleCollisionCheckContextSetParkingOutFlag(collisionWork, parkingOutFlag);
  apaVehicleCollisionCheckContextSetSlot(collisionWork, slot);
  apaVehicleCollisionCheckContextSetTargetPose(collisionWork, slot->TarPos);

  slotClass = apaSlotGetSlotClass(slot);
  clearStopper = false;
  if (slotClass == APA_SLOT_CLASS_VERTICAL) {
    clearStopper = true;
  } else if (!detectSearchingPathStopperForParSlot) {
    clearStopper = true;
  }
  if (clearStopper) {
    collisionWork->Obs.SlotTrackStopperNum = 0;
    collisionWork->Obs.SeparateStopperNum = 0;
  }
}

/* @source matlab_rewrite ClassComplexPath.m#newObsCollisionCheck */
static inline void apaCPathNewObsCollisionCheckWithVehicleCollisionCheck(
    ApaComplexPath_t *complexPath,
    ApaSlot_t *slot,
    const ApaVehicleCollisionCheckContext_t *collisionContext,
    Boolean detectSearchingPathStopperForParSlot,
    Boolean parkingOutFlag,
    Integer parkingSysMainState,
    Integer pathListType) {
  ApaVehicleCollisionCheckContext_t collisionWork;
  ApaPathVehicleCollisionProbeContext_t probeContext;
  ApaPlannerGlobalState_t *plannerGlobal;
  Integer savedCollisionModel;
  Integer pathCollisionFlag;

  if (complexPath == (ApaComplexPath_t *)0) {
    return;
  }
  if (slot == (ApaSlot_t *)0 || !complexPath->Path.ValidFlag || slot->SlotId == 0) {
    complexPath->CollisionCnt = 0;
    return;
  }
  if (collisionContext == (const ApaVehicleCollisionCheckContext_t *)0 ||
      collisionContext->PlannerGlobal == (ApaPlannerGlobalState_t *)0) {
    apaCPathNewObsCollisionCheckWithRisk(
        complexPath, slot->SlotId, APA_COLLISION_RISK_EXIST, parkingSysMainState, pathListType);
    return;
  }

  collisionWork = *collisionContext;
  apaCPathConfigureNewObsCollisionContext(
      &collisionWork, slot, detectSearchingPathStopperForParSlot, parkingOutFlag);

  plannerGlobal = collisionWork.PlannerGlobal;
  apaPlannerGlobalEnsureInit(plannerGlobal);
  savedCollisionModel = plannerGlobal->Planner.Conf.CollisionModel;
  apaPlannerGlobalSetCollisionModel(plannerGlobal, APA_COLLISION_MODEL_EIGHT_EDGES);

  apaPathVehicleCollisionProbeContextInit(&probeContext, &collisionWork);
  pathCollisionFlag = apaPathGetCollisionFlagWithProbe(&complexPath->Path,
                                                       apaPathVehicleCollisionPoseProbe,
                                                       &probeContext,
                                                       true,
                                                       apaPathVehicleCollisionNearSlotProbe);

  apaPlannerGlobalSetCollisionModel(plannerGlobal, savedCollisionModel);
  apaCPathNewObsCollisionCheckWithRisk(
      complexPath, slot->SlotId, pathCollisionFlag, parkingSysMainState, pathListType);
}

#endif
