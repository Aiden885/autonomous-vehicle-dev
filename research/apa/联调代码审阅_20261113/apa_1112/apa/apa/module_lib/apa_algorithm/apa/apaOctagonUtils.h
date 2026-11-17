/**
 * @file apaOctagonUtils.h
 * @brief APA octagon geometry helpers
 *
 * This header carries the first D3.6 helper batch and rewrites:
 *   - get_octagon_corners.m
 *   - isPtInOctagon.m
 *
 * The helpers are intentionally header-only so D3.4/D3.5 code can reuse them
 * without introducing extra graph-facing blocks yet.
 *
 * @date 2026-04-13
 */
#ifndef APA_OCTAGON_UTILS_H
#define APA_OCTAGON_UTILS_H

#include "baseTypes.h"
#include "apaMathUtils.h"

#define APA_OCTAGON_RECT_COUNT 9
#define APA_OCTAGON_EDGE_COUNT 8
#define APA_OCTAGON_POINT_COUNT 8

static inline void apaOctagonSetRect(
    Real dst[8], Real flX, Real flY, Real frX, Real frY, Real rrX, Real rrY, Real rlX, Real rlY) {
  dst[0] = flX;
  dst[1] = flY;
  dst[2] = frX;
  dst[3] = frY;
  dst[4] = rrX;
  dst[5] = rrY;
  dst[6] = rlX;
  dst[7] = rlY;
}

static inline void apaOctagonSetEdge(Real dst[4], Real x1, Real y1, Real x2, Real y2) {
  dst[0] = x1;
  dst[1] = y1;
  dst[2] = x2;
  dst[3] = y2;
}

static inline void
apaOctagonPointMove(Real ax, Real ay, Real bx, Real by, Real distanceToA, Real *outX, Real *outY) {
  Real vecX;
  Real vecY;
  Real distance;
  Real scale;

  vecX = bx - ax;
  vecY = by - ay;
  distance = apaMathNorm2D(vecX, vecY);
  scale = distanceToA / apaMathDivideZeroProcess(distance);
  *outX = ax + vecX * scale;
  *outY = ay + vecY * scale;
}

static inline void apaOctagonCalcBounds(const Real octagonPts[16], Real bounds[4]) {
  Integer i;
  Real xMin;
  Real yMin;
  Real xMax;
  Real yMax;

  xMin = octagonPts[0];
  yMin = octagonPts[1];
  xMax = octagonPts[0];
  yMax = octagonPts[1];

  for (i = 1; i < APA_OCTAGON_POINT_COUNT; i = i + 1) {
    Real px;
    Real py;

    px = octagonPts[2 * i];
    py = octagonPts[2 * i + 1];
    if (apaMathLess(px, xMin)) {
      xMin = px;
    }
    if (apaMathGreater(px, xMax)) {
      xMax = px;
    }
    if (apaMathLess(py, yMin)) {
      yMin = py;
    }
    if (apaMathGreater(py, yMax)) {
      yMax = py;
    }
  }

  bounds[0] = xMin;
  bounds[1] = yMin;
  bounds[2] = xMax;
  bounds[3] = yMax;
}

static inline void apaOctagonBuild(const Real rect[8],
                                   Real frontTriLat,
                                   Real frontTriLon,
                                   Real backTriLat,
                                   Real backTriLon,
                                   Real rects[APA_OCTAGON_RECT_COUNT][8],
                                   Real boundaryLines[APA_OCTAGON_EDGE_COUNT][4],
                                   Real octagonPts[16],
                                   Real bounds[4]) {
  const Real rec2Height = 1.5f;
  Integer rectIdx;
  Integer edgeIdx;

  Real cornerFlX;
  Real cornerFlY;
  Real cornerFrX;
  Real cornerFrY;
  Real cornerRrX;
  Real cornerRrY;
  Real cornerRlX;
  Real cornerRlY;

  Real rec1FlX;
  Real rec1FlY;
  Real rec1FrX;
  Real rec1FrY;
  Real rec1RrX;
  Real rec1RrY;
  Real rec1RlX;
  Real rec1RlY;

  Real rec2FlX;
  Real rec2FlY;
  Real rec2FrX;
  Real rec2FrY;
  Real rec2RrX;
  Real rec2RrY;
  Real rec2RlX;
  Real rec2RlY;

  Real rec3FlX;
  Real rec3FlY;
  Real rec3FrX;
  Real rec3FrY;
  Real rec3RrX;
  Real rec3RrY;
  Real rec3RlX;
  Real rec3RlY;

  Real rec4FlX;
  Real rec4FlY;
  Real rec4FrX;
  Real rec4FrY;
  Real rec4RrX;
  Real rec4RrY;
  Real rec4RlX;
  Real rec4RlY;

  Real rec5FlX;
  Real rec5FlY;
  Real rec5FrX;
  Real rec5FrY;
  Real rec5RrX;
  Real rec5RrY;
  Real rec5RlX;
  Real rec5RlY;

  Real rec6FlX;
  Real rec6FlY;
  Real rec6FrX;
  Real rec6FrY;
  Real rec6RrX;
  Real rec6RrY;
  Real rec6RlX;
  Real rec6RlY;

  Real rec7FlX;
  Real rec7FlY;
  Real rec7FrX;
  Real rec7FrY;
  Real rec7RrX;
  Real rec7RrY;
  Real rec7RlX;
  Real rec7RlY;

  Real rec8FlX;
  Real rec8FlY;
  Real rec8FrX;
  Real rec8FrY;
  Real rec8RrX;
  Real rec8RrY;
  Real rec8RlX;
  Real rec8RlY;

  Real rec9FlX;
  Real rec9FlY;
  Real rec9FrX;
  Real rec9FrY;
  Real rec9RrX;
  Real rec9RrY;
  Real rec9RlX;
  Real rec9RlY;

  for (rectIdx = 0; rectIdx < APA_OCTAGON_RECT_COUNT; rectIdx = rectIdx + 1) {
    Integer pointIdx;

    for (pointIdx = 0; pointIdx < 8; pointIdx = pointIdx + 1) {
      rects[rectIdx][pointIdx] = 0.0f;
    }
  }
  for (edgeIdx = 0; edgeIdx < APA_OCTAGON_EDGE_COUNT; edgeIdx = edgeIdx + 1) {
    Integer valueIdx;

    for (valueIdx = 0; valueIdx < 4; valueIdx = valueIdx + 1) {
      boundaryLines[edgeIdx][valueIdx] = 0.0f;
    }
  }

  cornerFlX = rect[0];
  cornerFlY = rect[1];
  cornerFrX = rect[2];
  cornerFrY = rect[3];
  cornerRrX = rect[4];
  cornerRrY = rect[5];
  cornerRlX = rect[6];
  cornerRlY = rect[7];

  apaOctagonPointMove(cornerFlX, cornerFlY, cornerRlX, cornerRlY, frontTriLon, &rec1FlX, &rec1FlY);
  apaOctagonPointMove(cornerFlX, cornerFlY, cornerFrX, cornerFrY, frontTriLat, &rec1FrX, &rec1FrY);
  apaOctagonPointMove(cornerFlX, cornerFlY, cornerRlX, cornerRlY, rec2Height, &rec1RlX, &rec1RlY);
  rec1RrX = rec1FrX + (rec1RlX - cornerFlX);
  rec1RrY = rec1FrY + (rec1RlY - cornerFlY);
  apaOctagonSetRect(
      rects[0], rec1FlX, rec1FlY, rec1FrX, rec1FrY, rec1RrX, rec1RrY, rec1RlX, rec1RlY);

  apaOctagonPointMove(cornerFrX, cornerFrY, cornerFlX, cornerFlY, frontTriLat, &rec3FlX, &rec3FlY);
  apaOctagonPointMove(cornerFrX, cornerFrY, cornerRrX, cornerRrY, frontTriLon, &rec3FrX, &rec3FrY);
  apaOctagonPointMove(cornerFrX, cornerFrY, cornerRrX, cornerRrY, rec2Height, &rec3RrX, &rec3RrY);
  rec3RlX = rec3FlX + (rec3RrX - cornerFrX);
  rec3RlY = rec3FlY + (rec3RrY - cornerFrY);
  apaOctagonSetRect(
      rects[2], rec3FlX, rec3FlY, rec3FrX, rec3FrY, rec3RrX, rec3RrY, rec3RlX, rec3RlY);

  rec2FlX = rec1FrX;
  rec2FlY = rec1FrY;
  rec2RlX = rec1RrX;
  rec2RlY = rec1RrY;
  rec2FrX = rec3FlX;
  rec2FrY = rec3FlY;
  rec2RrX = rec3RlX;
  rec2RrY = rec3RlY;
  apaOctagonSetRect(
      rects[1], rec2FlX, rec2FlY, rec2FrX, rec2FrY, rec2RrX, rec2RrY, rec2RlX, rec2RlY);

  apaOctagonPointMove(cornerRlX, cornerRlY, cornerFlX, cornerFlY, rec2Height, &rec7FlX, &rec7FlY);
  apaOctagonPointMove(cornerRlX, cornerRlY, cornerFlX, cornerFlY, backTriLon, &rec7RlX, &rec7RlY);
  apaOctagonPointMove(cornerRlX, cornerRlY, cornerRrX, cornerRrY, backTriLat, &rec7RrX, &rec7RrY);
  rec7FrX = rec7RrX + (rec7FlX - cornerRlX);
  rec7FrY = rec7RrY + (rec7FlY - cornerRlY);
  apaOctagonSetRect(
      rects[6], rec7FlX, rec7FlY, rec7FrX, rec7FrY, rec7RrX, rec7RrY, rec7RlX, rec7RlY);

  apaOctagonPointMove(cornerRrX, cornerRrY, cornerFrX, cornerFrY, rec2Height, &rec9FrX, &rec9FrY);
  apaOctagonPointMove(cornerRrX, cornerRrY, cornerFrX, cornerFrY, backTriLon, &rec9RrX, &rec9RrY);
  apaOctagonPointMove(cornerRrX, cornerRrY, cornerRlX, cornerRlY, backTriLat, &rec9RlX, &rec9RlY);
  rec9FlX = rec9RlX + (rec9FrX - cornerRrX);
  rec9FlY = rec9RlY + (rec9FrY - cornerRrY);
  apaOctagonSetRect(
      rects[8], rec9FlX, rec9FlY, rec9FrX, rec9FrY, rec9RrX, rec9RrY, rec9RlX, rec9RlY);

  rec8FlX = rec7FrX;
  rec8FlY = rec7FrY;
  rec8RlX = rec7RrX;
  rec8RlY = rec7RrY;
  rec8RrX = rec9RlX;
  rec8RrY = rec9RlY;
  rec8FrX = rec9FlX;
  rec8FrY = rec9FlY;
  apaOctagonSetRect(
      rects[7], rec8FlX, rec8FlY, rec8FrX, rec8FrY, rec8RrX, rec8RrY, rec8RlX, rec8RlY);

  rec4FlX = rec1RlX;
  rec4FlY = rec1RlY;
  rec4FrX = rec1RrX;
  rec4FrY = rec1RrY;
  rec4RlX = rec7FlX;
  rec4RlY = rec7FlY;
  rec4RrX = rec7FrX;
  rec4RrY = rec7FrY;
  apaOctagonSetRect(
      rects[3], rec4FlX, rec4FlY, rec4FrX, rec4FrY, rec4RrX, rec4RrY, rec4RlX, rec4RlY);

  rec5FlX = rec2RlX;
  rec5FlY = rec2RlY;
  rec5FrX = rec2RrX;
  rec5FrY = rec2RrY;
  rec5RlX = rec8FlX;
  rec5RlY = rec8FlY;
  rec5RrX = rec8FrX;
  rec5RrY = rec8FrY;
  apaOctagonSetRect(
      rects[4], rec5FlX, rec5FlY, rec5FrX, rec5FrY, rec5RrX, rec5RrY, rec5RlX, rec5RlY);

  rec6FlX = rec3RlX;
  rec6FlY = rec3RlY;
  rec6FrX = rec3RrX;
  rec6FrY = rec3RrY;
  rec6RlX = rec9FlX;
  rec6RlY = rec9FlY;
  rec6RrX = rec9FrX;
  rec6RrY = rec9FrY;
  apaOctagonSetRect(
      rects[5], rec6FlX, rec6FlY, rec6FrX, rec6FrY, rec6RrX, rec6RrY, rec6RlX, rec6RlY);

  apaOctagonSetEdge(boundaryLines[0], rec1FlX, rec1FlY, rec2FlX, rec2FlY);
  apaOctagonSetEdge(boundaryLines[1], rec2FlX, rec2FlY, rec3FlX, rec3FlY);
  apaOctagonSetEdge(boundaryLines[2], rec3FlX, rec3FlY, rec3FrX, rec3FrY);
  apaOctagonSetEdge(boundaryLines[3], rec3FrX, rec3FrY, rec9RrX, rec9RrY);
  apaOctagonSetEdge(boundaryLines[4], rec9RrX, rec9RrY, rec9RlX, rec9RlY);
  apaOctagonSetEdge(boundaryLines[5], rec9RlX, rec9RlY, rec8RlX, rec8RlY);
  apaOctagonSetEdge(boundaryLines[6], rec8RlX, rec8RlY, rec7RlX, rec7RlY);
  apaOctagonSetEdge(boundaryLines[7], rec7RlX, rec7RlY, rec1FlX, rec1FlY);

  octagonPts[0] = rec1FlX;
  octagonPts[1] = rec1FlY;
  octagonPts[2] = rec2FlX;
  octagonPts[3] = rec2FlY;
  octagonPts[4] = rec3FlX;
  octagonPts[5] = rec3FlY;
  octagonPts[6] = rec3FrX;
  octagonPts[7] = rec3FrY;
  octagonPts[8] = rec9RrX;
  octagonPts[9] = rec9RrY;
  octagonPts[10] = rec9RlX;
  octagonPts[11] = rec9RlY;
  octagonPts[12] = rec8RlX;
  octagonPts[13] = rec8RlY;
  octagonPts[14] = rec7RlX;
  octagonPts[15] = rec7RlY;

  apaOctagonCalcBounds(octagonPts, bounds);
}

static inline Boolean
apaPtInOctagonWithBounds(const Real octagonPts[16], Real ptX, Real ptY, const Real bounds[4]) {
  Integer i;
  Integer sign;

  if (apaMathGreater(ptX, bounds[2])) {
    return false;
  }
  if (apaMathGreater(ptY, bounds[3])) {
    return false;
  }
  if (apaMathLess(ptX, bounds[0])) {
    return false;
  }
  if (apaMathLess(ptY, bounds[1])) {
    return false;
  }

  sign = 0;
  for (i = 0; i < APA_OCTAGON_POINT_COUNT; i = i + 1) {
    Integer nextIdx;
    Real ax;
    Real ay;
    Real bx;
    Real by;
    Real vecAbX;
    Real vecAbY;
    Real vecApX;
    Real vecApY;
    Real crossZ;

    nextIdx = i + 1;
    if (nextIdx >= APA_OCTAGON_POINT_COUNT) {
      nextIdx = 0;
    }

    ax = octagonPts[2 * i];
    ay = octagonPts[2 * i + 1];
    bx = octagonPts[2 * nextIdx];
    by = octagonPts[2 * nextIdx + 1];
    vecAbX = bx - ax;
    vecAbY = by - ay;
    vecApX = ptX - ax;
    vecApY = ptY - ay;
    crossZ = vecAbX * vecApY - vecAbY * vecApX;

    if (apaMathGreater(crossZ, 1.0e-6f)) {
      if (sign < 0) {
        return false;
      }
      sign = 1;
    } else if (apaMathLess(crossZ, -1.0e-6f)) {
      if (sign > 0) {
        return false;
      }
      sign = -1;
    }
  }

  return true;
}

static inline Boolean apaPtInOctagon(const Real octagonPts[16], Real ptX, Real ptY) {
  Real bounds[4];

  apaOctagonCalcBounds(octagonPts, bounds);
  return apaPtInOctagonWithBounds(octagonPts, ptX, ptY, bounds);
}

#endif
