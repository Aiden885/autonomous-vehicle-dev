/**
 * @file apaGeoUtils.h
 * @brief APA 共享基础几何工具函数
 *
 * 提取自 demoHelperCollisionCheck.c 中重复实现的内联子函数。
 * 对应 MATLAB 基础模块库中的独立模块：
 * Arithmetic primitives such as cross product, norm, clamp, and protected
 * denominator handling are provided by apaMathUtils.h.
 *   - calcuRectangleCorners.m   → apaGeoBuildRectangle
 *   - is_Pt_In_Rect.m           → apaGeoPointInQuad
 *   - is_Line_Intersect.m       → apaGeoSegmentsIntersect
 *   - cal_dis_pt_2_line.m       → apaGeoPointSegmentDistance
 *
 * @date 2026-03-15
 */
#ifndef APA_GEO_UTILS_H
#define APA_GEO_UTILS_H

#include "apaMathUtils.h"
#include "apaVehicleDefaults.h"
#include "baseTypes.h"
#include <math.h>

#ifndef APA_GEO_EPSILON
#define APA_GEO_EPSILON 1e-6f
#endif

/**
 * @brief 由中心、航向、长宽构建矩形四角点 — 对应 calcuRectangleCorners.m
 *
 * 角点顺序：右前、右后、左后、左前（顺时针）
 */
static inline void apaGeoBuildRectangle(
    Real centerX, Real centerY, Real yaw, Real length, Real width, Real rectX[4], Real rectY[4]) {
  Real halfLength;
  Real halfWidth;
  Real cosYaw;
  Real sinYaw;
  Real localX[4];
  Real localY[4];
  Integer idx;

  halfLength = 0.5f * length;
  halfWidth = 0.5f * width;
  cosYaw = cosf(yaw);
  sinYaw = sinf(yaw);

  localX[0] = halfLength;
  localY[0] = halfWidth;
  localX[1] = halfLength;
  localY[1] = -halfWidth;
  localX[2] = -halfLength;
  localY[2] = -halfWidth;
  localX[3] = -halfLength;
  localY[3] = halfWidth;

  idx = 0;
  while (idx < 4) {
    rectX[idx] = centerX + localX[idx] * cosYaw - localY[idx] * sinYaw;
    rectY[idx] = centerY + localX[idx] * sinYaw + localY[idx] * cosYaw;
    idx = idx + 1;
  }
}

/**
 * @brief 点到线段最短距离 — 对应 cal_dis_pt_2_line.m / distance_point_to_segment.m
 *
 * 同时输出线段上的最近点坐标。
 */
static inline Real apaGeoPointSegmentDistance(Real pointX,
                                              Real pointY,
                                              Real segX1,
                                              Real segY1,
                                              Real segX2,
                                              Real segY2,
                                              Real *closestX,
                                              Real *closestY) {
  Real segDx;
  Real segDy;
  Real segLenSq;
  Real diffX;
  Real diffY;

  segDx = segX2 - segX1;
  segDy = segY2 - segY1;
  segLenSq = segDx * segDx + segDy * segDy;

  if (segLenSq < APA_GEO_EPSILON) {
    *closestX = segX1;
    *closestY = segY1;
  } else {
    Real relX;
    Real relY;
    Real projRatio;

    relX = pointX - segX1;
    relY = pointY - segY1;
    projRatio = apaMathClamp((relX * segDx + relY * segDy) / segLenSq, 0.0f, 1.0f);
    *closestX = segX1 + segDx * projRatio;
    *closestY = segY1 + segDy * projRatio;
  }

  diffX = pointX - *closestX;
  diffY = pointY - *closestY;
  return sqrtf(diffX * diffX + diffY * diffY);
}

/**
 * @brief 两点式直线求解 — 对应 linear_func_slove.m
 *
 * MATLAB 源逻辑使用 P_start(:,1)-P_end(:,1) 作为分母，并通过
 * divideZeroProcessFloat32 处理竖直线；这里保留同样的符号约定。
 */
static inline void apaGeoLinearFuncSolve(
    Real startX, Real startY, Real endX, Real endY, Real *slopeK, Real *interceptB) {
  Real deltaX;
  Real deltaY;

  deltaX = apaMathDivideZeroProcess(startX - endX);
  deltaY = startY - endY;
  *slopeK = deltaY / deltaX;
  *interceptB = startY - ((*slopeK) * startX);
}

/** @brief 点到斜截式直线距离 — 对应 cal_dis_pt_2_line.m */
static inline Real
apaGeoPointLineDistanceFromSlopeIntercept(Real pointX, Real pointY, Real slopeK, Real interceptB) {
  Real numerator;
  Real denominator;

  numerator = apaMathAbs((slopeK * pointX) - pointY + interceptB);
  denominator = apaMathDivideZeroProcess(sqrtf((slopeK * slopeK) + 1.0f));
  return numerator / denominator;
}

#ifndef APA_GEO_COLLINEAR_MAX_POINTS
#define APA_GEO_COLLINEAR_MAX_POINTS 1024
#endif

/**
 * @brief 路径点共线压缩 — 对应 collinear_detection.m
 *
 * 保留 MATLAB 的滑动窗口规则：当新点到当前 A-B 直线距离小于阈值时，
 * 删除 B 点，继续用后续点扩展该直线段。
 */
static inline Boolean apaGeoCollinearDetection(const Real inputPts[][2],
                                               Integer inputCount,
                                               Real collinearThreshold,
                                               Real outputPts[][2],
                                               Integer *outputCount) {
  Boolean valid[APA_GEO_COLLINEAR_MAX_POINTS];
  Integer ptrLeft;
  Integer ptrRight;
  Integer idx;
  Integer writeIndex;

  if (inputCount < 0 || inputCount > APA_GEO_COLLINEAR_MAX_POINTS) {
    *outputCount = 0;
    return false;
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

    apaGeoLinearFuncSolve(inputPts[ptrLeft][0],
                          inputPts[ptrLeft][1],
                          inputPts[ptrRight][0],
                          inputPts[ptrRight][1],
                          &slopeK,
                          &interceptB);
    ptr = ptrRight + 1;
    scanOpen = (ptr < inputCount);
    while (scanOpen) {
      Real distance;

      distance = apaGeoPointLineDistanceFromSlopeIntercept(
          inputPts[ptr][0], inputPts[ptr][1], slopeK, interceptB);
      if (distance < collinearThreshold) {
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
      outputPts[writeIndex][0] = inputPts[idx][0];
      outputPts[writeIndex][1] = inputPts[idx][1];
      writeIndex = writeIndex + 1;
    }
  }
  while (writeIndex < inputCount) {
    outputPts[writeIndex][0] = 0.0f;
    outputPts[writeIndex][1] = 0.0f;
    writeIndex = writeIndex + 1;
  }

  *outputCount = 0;
  for (idx = 0; idx < inputCount; idx = idx + 1) {
    if (valid[idx]) {
      *outputCount = *outputCount + 1;
    }
  }
  return true;
}

/** @brief 判断点是否在线段的 AABB 范围内 */
static inline Boolean
apaGeoOnSegment(Real pointX, Real pointY, Real segX1, Real segY1, Real segX2, Real segY2) {
  Real minX;
  Real maxX;
  Real minY;
  Real maxY;

  if (segX1 > segX2) {
    minX = segX2;
    maxX = segX1;
  } else {
    minX = segX1;
    maxX = segX2;
  }
  if (segY1 > segY2) {
    minY = segY2;
    maxY = segY1;
  } else {
    minY = segY1;
    maxY = segY2;
  }

  return (pointX >= (minX - APA_GEO_EPSILON)) && (pointX <= (maxX + APA_GEO_EPSILON)) &&
         (pointY >= (minY - APA_GEO_EPSILON)) && (pointY <= (maxY + APA_GEO_EPSILON));
}

/**
 * @brief 线段-线段相交判定 — 对应 is_Line_Intersect.m
 *
 * 返回是否相交，相交时通过 hitX/hitY 输出交点。
 * 含共线重叠检测。
 */
static inline Boolean apaGeoSegmentsIntersect(Real ax1,
                                              Real ay1,
                                              Real ax2,
                                              Real ay2,
                                              Real bx1,
                                              Real by1,
                                              Real bx2,
                                              Real by2,
                                              Real *hitX,
                                              Real *hitY) {
  Real rX;
  Real rY;
  Real sX;
  Real sY;
  Real denom;
  Real qpx;
  Real qpy;
  Boolean hasIntersection;

  rX = ax2 - ax1;
  rY = ay2 - ay1;
  sX = bx2 - bx1;
  sY = by2 - by1;
  denom = apaMathCross2D(rX, rY, sX, sY);
  qpx = bx1 - ax1;
  qpy = by1 - ay1;
  hasIntersection = false;

  if (apaMathAbs(denom) < APA_GEO_EPSILON) {

    Real crossQpR;

    crossQpR = apaMathCross2D(qpx, qpy, rX, rY);
    if (apaMathAbs(crossQpR) < APA_GEO_EPSILON) {
      if (apaGeoOnSegment(ax1, ay1, bx1, by1, bx2, by2)) {
        *hitX = ax1;
        *hitY = ay1;
        hasIntersection = true;
      } else if (apaGeoOnSegment(ax2, ay2, bx1, by1, bx2, by2)) {
        *hitX = ax2;
        *hitY = ay2;
        hasIntersection = true;
      } else if (apaGeoOnSegment(bx1, by1, ax1, ay1, ax2, ay2)) {
        *hitX = bx1;
        *hitY = by1;
        hasIntersection = true;
      } else if (apaGeoOnSegment(bx2, by2, ax1, ay1, ax2, ay2)) {
        *hitX = bx2;
        *hitY = by2;
        hasIntersection = true;
      }
    }
  } else {
    Real tValue;
    Real uValue;

    tValue = apaMathCross2D(qpx, qpy, sX, sY) / denom;
    uValue = apaMathCross2D(qpx, qpy, rX, rY) / denom;
    if ((tValue >= 0.0f) && (tValue <= 1.0f) && (uValue >= 0.0f) && (uValue <= 1.0f)) {
      *hitX = ax1 + tValue * rX;
      *hitY = ay1 + tValue * rY;
      hasIntersection = true;
    }
  }

  return hasIntersection;
}

/**
 * @brief 判定点是否在凸四边形内 — 对应 is_Pt_In_Rect.m
 *
 * 使用叉积符号一致性判定（仅适用于凸多边形）。
 */
static inline Boolean
apaGeoPointInQuad(Real pointX, Real pointY, const Real quadX[4], const Real quadY[4]) {
  Integer idx;
  Integer signValue;
  Boolean insideValue;

  signValue = 0;
  insideValue = true;
  idx = 0;
  while (idx < 4) {
    Integer nextIdx;
    Real edgeX;
    Real edgeY;
    Real relX;
    Real relY;
    Real crossValue;
    Integer currentSign;

    nextIdx = idx + 1;
    if (nextIdx >= 4) {
      nextIdx = 0;
    }
    edgeX = quadX[nextIdx] - quadX[idx];
    edgeY = quadY[nextIdx] - quadY[idx];
    relX = pointX - quadX[idx];
    relY = pointY - quadY[idx];
    crossValue = apaMathCross2D(edgeX, edgeY, relX, relY);

    if (crossValue > APA_GEO_EPSILON) {
      currentSign = 1;
    } else if (crossValue < -APA_GEO_EPSILON) {
      currentSign = -1;
    } else {
      currentSign = 0;
    }

    if (currentSign != 0) {
      if (signValue == 0) {
        signValue = currentSign;
      } else if (signValue != currentSign) {
        insideValue = false;
      }
    }
    idx = idx + 1;
  }

  return insideValue;
}

/**
 * @brief 由后轴位姿、前后悬、车宽及统一外扩余量构建八边形倒角车身顶点（顺时针）
 */
static inline void apaGeoBuildVehicleOctagon(Real poseX,
                                             Real poseY,
                                             Real yaw,
                                             Real vehFront,
                                             Real vehRear,
                                             Real vehWidth,
                                             Real margin,
                                             Real polyX[8],
                                             Real polyY[8]) {
  Real front;
  Real rear;
  Real halfWidth;
  Real fLon;
  Real fLat;
  Real rLon;
  Real rLat;
  Real cosYaw;
  Real sinYaw;
  Real localX[8];
  Real localY[8];
  Integer idx;

  front = vehFront + margin;
  rear = vehRear + margin;
  halfWidth = 0.5f * vehWidth + margin;
  fLon = fminf(fmaxf(0.0f, APA_VEHICLE_FRONT_CHAMFER_LON_M), fmaxf(0.0f, front));
  fLat = fminf(fmaxf(0.0f, APA_VEHICLE_FRONT_CHAMFER_LAT_M), fmaxf(0.0f, halfWidth - 1.0e-3f));
  rLon = fminf(fmaxf(0.0f, APA_VEHICLE_REAR_CHAMFER_LON_M), fmaxf(0.0f, rear));
  rLat = fminf(fmaxf(0.0f, APA_VEHICLE_REAR_CHAMFER_LAT_M), fmaxf(0.0f, halfWidth - 1.0e-3f));
  cosYaw = cosf(yaw);
  sinYaw = sinf(yaw);

  localX[0] = front - fLon;
  localY[0] = halfWidth;
  localX[1] = front;
  localY[1] = halfWidth - fLat;
  localX[2] = front;
  localY[2] = -halfWidth + fLat;
  localX[3] = front - fLon;
  localY[3] = -halfWidth;
  localX[4] = -rear + rLon;
  localY[4] = -halfWidth;
  localX[5] = -rear;
  localY[5] = -halfWidth + rLat;
  localX[6] = -rear;
  localY[6] = halfWidth - rLat;
  localX[7] = -rear + rLon;
  localY[7] = halfWidth;

  idx = 0;
  while (idx < 8) {
    polyX[idx] = poseX + localX[idx] * cosYaw - localY[idx] * sinYaw;
    polyY[idx] = poseY + localX[idx] * sinYaw + localY[idx] * cosYaw;
    idx = idx + 1;
  }
}

/**
 * @brief 判定点是否在凸八边形内（含边界）
 */
static inline Boolean
apaGeoPointInOctagon(Real pointX, Real pointY, const Real polyX[8], const Real polyY[8]) {
  Integer idx;
  Integer signValue;
  Boolean insideValue;

  signValue = 0;
  insideValue = true;
  idx = 0;
  while (idx < 8) {
    Integer nextIdx;
    Real edgeX;
    Real edgeY;
    Real relX;
    Real relY;
    Real crossValue;
    Integer currentSign;

    nextIdx = idx + 1;
    if (nextIdx >= 8) {
      nextIdx = 0;
    }
    edgeX = polyX[nextIdx] - polyX[idx];
    edgeY = polyY[nextIdx] - polyY[idx];
    relX = pointX - polyX[idx];
    relY = pointY - polyY[idx];
    crossValue = apaMathCross2D(edgeX, edgeY, relX, relY);

    if (crossValue > APA_GEO_EPSILON) {
      currentSign = 1;
    } else if (crossValue < -APA_GEO_EPSILON) {
      currentSign = -1;
    } else {
      currentSign = 0;
    }

    if (currentSign != 0) {
      if (signValue == 0) {
        signValue = currentSign;
      } else if (signValue != currentSign) {
        insideValue = false;
      }
    }
    idx = idx + 1;
  }

  return insideValue;
}

/**
 * @brief 判定线段是否与凸八边形相交或包含于其内
 */
static inline Boolean apaGeoSegmentTouchesOctagon(
    Real segX1, Real segY1, Real segX2, Real segY2, const Real polyX[8], const Real polyY[8]) {
  Boolean touches;
  Integer edgeIdx;
  Boolean loop0;

  touches = apaGeoPointInOctagon(segX1, segY1, polyX, polyY) ||
            apaGeoPointInOctagon(segX2, segY2, polyX, polyY);
  edgeIdx = 0;
  loop0 = !touches;
  while (loop0) {
    Integer nextIdx;
    Real hitX;
    Real hitY;

    nextIdx = edgeIdx + 1;
    if (nextIdx >= 8) {
      nextIdx = 0;
    }
    hitX = 0.0f;
    hitY = 0.0f;
    touches = apaGeoSegmentsIntersect(segX1,
                                      segY1,
                                      segX2,
                                      segY2,
                                      polyX[edgeIdx],
                                      polyY[edgeIdx],
                                      polyX[nextIdx],
                                      polyY[nextIdx],
                                      &hitX,
                                      &hitY);
    edgeIdx = edgeIdx + 1;
    loop0 = (edgeIdx < 8) && (!touches);
  }
  return touches;
}

/**
 * @brief 判定点是否在一般多边形内（4顶点版） — 对应 Is_Pt_In_Poly.m
 *
 * 使用叉积符号一致性判定 + AABB 快速排除 + 共线边缘判定。
 * 输入为 polyPts[8] = {x0,y0, x1,y1, x2,y2, x3,y3} 的平坦数组。
 */
static inline Boolean apaGeoPointInPoly4(Real pointX, Real pointY, const Real polyPts[8]) {
  Real minX;
  Real maxX;
  Real minY;
  Real maxY;
  Integer idx;
  Integer signValue;
  Boolean isOnEdge;

  minX = polyPts[0];
  maxX = polyPts[0];
  minY = polyPts[1];
  maxY = polyPts[1];
  idx = 1;
  while (idx < 4) {
    Real px;
    Real py;

    px = polyPts[idx * 2];
    py = polyPts[idx * 2 + 1];
    if (px < minX) {
      minX = px;
    }
    if (px > maxX) {
      maxX = px;
    }
    if (py < minY) {
      minY = py;
    }
    if (py > maxY) {
      maxY = py;
    }
    idx = idx + 1;
  }

  if ((pointX > maxX) || (pointX < minX) || (pointY > maxY) || (pointY < minY)) {
    return false;
  }

  signValue = 0;
  isOnEdge = false;
  idx = 0;
  while (idx < 4) {
    Integer nextIdx;
    Real ax;
    Real ay;
    Real bx;
    Real by;
    Real crossVal;

    nextIdx = idx + 1;
    if (nextIdx >= 4) {
      nextIdx = 0;
    }
    ax = polyPts[nextIdx * 2] - polyPts[idx * 2];
    ay = polyPts[nextIdx * 2 + 1] - polyPts[idx * 2 + 1];
    bx = pointX - polyPts[idx * 2];
    by = pointY - polyPts[idx * 2 + 1];
    crossVal = ax * by - ay * bx;

    if ((crossVal > -APA_GEO_EPSILON) && (crossVal < APA_GEO_EPSILON)) {

      Real segMinX;
      Real segMaxX;
      Real segMinY;
      Real segMaxY;

      segMinX = polyPts[idx * 2];
      segMaxX = polyPts[nextIdx * 2];
      if (segMinX > segMaxX) {
        Real tmp;

        tmp = segMinX;
        segMinX = segMaxX;
        segMaxX = tmp;
      }
      segMinY = polyPts[idx * 2 + 1];
      segMaxY = polyPts[nextIdx * 2 + 1];
      if (segMinY > segMaxY) {
        Real tmp;

        tmp = segMinY;
        segMinY = segMaxY;
        segMaxY = tmp;
      }
      if ((pointX >= segMinX) && (pointX <= segMaxX) && (pointY >= segMinY) &&
          (pointY <= segMaxY)) {
        isOnEdge = true;
      }
    } else {
      Integer curSign;

      if (crossVal > 0.0f) {
        curSign = 1;
      } else {
        curSign = -1;
      }
      if (signValue == 0) {
        signValue = curSign;
      } else if (signValue != curSign) {
        if (!isOnEdge) {
          return false;
        }
      }
    }
    idx = idx + 1;
  }

  return true;
}

/**
 * @brief 线段与矩形相交判定 — 对应 isLineIntersectWithRect.m（简化版）
 *
 * 检测线段是否与矩形的任意边相交。不依赖 SAT，纯几何判定。
 * rectPts[8] = {FL_x, FL_y, FR_x, FR_y, RR_x, RR_y, RL_x, RL_y}
 */
static inline Boolean
apaGeoLineIntersectRect(Real lineX1, Real lineY1, Real lineX2, Real lineY2, const Real rectPts[8]) {
  Integer idx;
  Real hitX;
  Real hitY;

  hitX = 0.0f;
  hitY = 0.0f;
  idx = 0;
  while (idx < 4) {
    Integer nextIdx;

    nextIdx = idx + 1;
    if (nextIdx >= 4) {
      nextIdx = 0;
    }
    if (apaGeoSegmentsIntersect(lineX1,
                                lineY1,
                                lineX2,
                                lineY2,
                                rectPts[idx * 2],
                                rectPts[idx * 2 + 1],
                                rectPts[nextIdx * 2],
                                rectPts[nextIdx * 2 + 1],
                                &hitX,
                                &hitY)) {
      return true;
    }
    idx = idx + 1;
  }

  return false;
}

#endif
