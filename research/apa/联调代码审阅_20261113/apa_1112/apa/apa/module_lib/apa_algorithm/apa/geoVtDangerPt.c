#include "geoVtDangerPtTypes.h"
#include <math.h>

static void geoVtDangerPtCheckRealLines(const GeoVtDangerPtInput_t *input,
                                        Real refX,
                                        Real refY,
                                        Real tgtX,
                                        Real tgtY,
                                        Real cosT,
                                        Real sinT,
                                        Real laneA,
                                        Real laneB,
                                        Real laneC,
                                        Real sideA,
                                        Real sideB,
                                        Real sideC,
                                        Real zoneLen,
                                        Real lenAhead,
                                        Integer *checkCount,
                                        Real *bestRealDist2,
                                        Real *dangerRealX,
                                        Real *dangerRealY,
                                        Integer *ptExReal) {
  const Real minLineNorm2 = 0.1f;
  Integer ii = 0;
  while (ii < input->BoundaryLineNum) {
    if (ii < DGR_LINE_MAX) {
      Real x1 = input->BoundaryP1X[ii];
      Real y1 = input->BoundaryP1Y[ii];
      Real x2 = input->BoundaryP2X[ii];
      Real y2 = input->BoundaryP2Y[ii];
      Real dx = x2 - x1;
      Real dy = y2 - y1;
      Real segLen2 = dx * dx + dy * dy;

      if (segLen2 > minLineNorm2) {
        Real tNum = (refX - x1) * dx + (refY - y1) * dy;
        Real tVal = tNum / segLen2;
        Real projX = x1 + tVal * dx;
        Real projY = y1 + tVal * dy;
        Boolean useProj = (tVal >= 0.0f) & (tVal <= 1.0f);
        *checkCount = *checkCount + 1;

        if (useProj) {
          Real dX = projX - tgtX;
          Real dY = projY - tgtY;
          Real relX = cosT * dX + sinT * dY;
          Real pLane = laneA * projX + laneB * projY + laneC;
          Real pSide = sideA * projX + sideB * projY + sideC;

          if ((relX > 0.0f) & (relX < (zoneLen + lenAhead)) & ((pLane * pSide) < 0.0f)) {
            Real dist2 = (refX - projX) * (refX - projX) + (refY - projY) * (refY - projY);
            if (dist2 < *bestRealDist2) {
              *bestRealDist2 = dist2;
              *dangerRealX = projX;
              *dangerRealY = projY;
              *ptExReal = 1;
            }
          }
        } else {
          Integer kk = 0;
          while (kk < 2) {
            Real cX;
            Real cY;
            if (kk == 0) {
              cX = x1;
              cY = y1;
            } else {
              cX = x2;
              cY = y2;
            }
            Real dX = cX - tgtX;
            Real dY = cY - tgtY;
            Real relX = cosT * dX + sinT * dY;
            Real pLane = laneA * cX + laneB * cY + laneC;
            Real pSide = sideA * cX + sideB * cY + sideC;
            if ((relX > 0.0f) & (relX < (zoneLen + lenAhead)) & ((pLane * pSide) < 0.0f)) {
              Real dist2 = (refX - cX) * (refX - cX) + (refY - cY) * (refY - cY);
              if (dist2 < *bestRealDist2) {
                *bestRealDist2 = dist2;
                *dangerRealX = cX;
                *dangerRealY = cY;
                *ptExReal = 1;
              }
            }
            kk = kk + 1;
          }
        }
      }
    }
    ii = ii + 1;
  }
}

static void geoVtDangerPtCheckVirtualLines(const GeoVtDangerPtInput_t *input,
                                           Real refX,
                                           Real refY,
                                           Real tgtX,
                                           Real tgtY,
                                           Real cosT,
                                           Real sinT,
                                           Real laneA,
                                           Real laneB,
                                           Real laneC,
                                           Real sideA,
                                           Real sideB,
                                           Real sideC,
                                           Real zoneLen,
                                           Real lenAhead,
                                           Integer *checkCount,
                                           Real *bestVirtualDist2,
                                           Real *dangerVirtualX,
                                           Real *dangerVirtualY,
                                           Integer *ptExVirtual) {
  const Real minLineNorm2 = 0.1f;
  Integer ii = 0;
  while (ii < input->VirtualBoundaryLineNum) {
    if (ii < DGR_LINE_MAX) {
      Real x1 = input->VirtualBoundaryP1X[ii];
      Real y1 = input->VirtualBoundaryP1Y[ii];
      Real x2 = input->VirtualBoundaryP2X[ii];
      Real y2 = input->VirtualBoundaryP2Y[ii];
      Real dx = x2 - x1;
      Real dy = y2 - y1;
      Real segLen2 = dx * dx + dy * dy;

      if (segLen2 > minLineNorm2) {
        Real tNum = (refX - x1) * dx + (refY - y1) * dy;
        Real tVal = tNum / segLen2;
        Real projX = x1 + tVal * dx;
        Real projY = y1 + tVal * dy;
        Boolean useProj = (tVal >= 0.0f) & (tVal <= 1.0f);
        *checkCount = *checkCount + 1;

        if (useProj) {
          Real dX = projX - tgtX;
          Real dY = projY - tgtY;
          Real relX = cosT * dX + sinT * dY;
          Real pLane = laneA * projX + laneB * projY + laneC;
          Real pSide = sideA * projX + sideB * projY + sideC;

          if ((relX > 0.0f) & (relX < (zoneLen + lenAhead)) & ((pLane * pSide) < 0.0f)) {
            Real dist2 = (refX - projX) * (refX - projX) + (refY - projY) * (refY - projY);
            if (dist2 < *bestVirtualDist2) {
              *bestVirtualDist2 = dist2;
              *dangerVirtualX = projX;
              *dangerVirtualY = projY;
              *ptExVirtual = 2;
            }
          }
        } else {
          Integer kk = 0;
          while (kk < 2) {
            Real cX;
            Real cY;
            if (kk == 0) {
              cX = x1;
              cY = y1;
            } else {
              cX = x2;
              cY = y2;
            }
            Real dX = cX - tgtX;
            Real dY = cY - tgtY;
            Real relX = cosT * dX + sinT * dY;
            Real pLane = laneA * cX + laneB * cY + laneC;
            Real pSide = sideA * cX + sideB * cY + sideC;
            if ((relX > 0.0f) & (relX < (zoneLen + lenAhead)) & ((pLane * pSide) < 0.0f)) {
              Real dist2 = (refX - cX) * (refX - cX) + (refY - cY) * (refY - cY);
              if (dist2 < *bestVirtualDist2) {
                *bestVirtualDist2 = dist2;
                *dangerVirtualX = cX;
                *dangerVirtualY = cY;
                *ptExVirtual = 2;
              }
            }
            kk = kk + 1;
          }
        }
      }
    }
    ii = ii + 1;
  }
}

static void geoVtDangerPtCheckSepPts(const GeoVtDangerPtInput_t *input,
                                     Real refX,
                                     Real refY,
                                     Real tgtX,
                                     Real tgtY,
                                     Real cosT,
                                     Real sinT,
                                     Real laneA,
                                     Real laneB,
                                     Real laneC,
                                     Real sideA,
                                     Real sideB,
                                     Real sideC,
                                     Real zoneLen,
                                     Real lenAhead,
                                     Integer *checkCount,
                                     Real *bestRealDist2,
                                     Real *dangerRealX,
                                     Real *dangerRealY,
                                     Integer *ptExReal) {
  Integer ii = 0;
  while (ii < input->BoundarySepPtNum) {
    if (ii < DGR_PT_MAX) {
      Real pX = input->BoundarySepPtX[ii];
      Real pY = input->BoundarySepPtY[ii];
      if ((fabsf(pX) > 1e-3f) | (fabsf(pY) > 1e-3f)) {
        *checkCount = *checkCount + 1;
        Real dX = pX - tgtX;
        Real dY = pY - tgtY;
        Real relX = cosT * dX + sinT * dY;
        Real pLane = laneA * pX + laneB * pY + laneC;
        Real pSide = sideA * pX + sideB * pY + sideC;
        if ((relX > 0.0f) & (relX < (zoneLen + lenAhead)) & ((pLane * pSide) < 0.0f)) {
          Real dist2 = (refX - pX) * (refX - pX) + (refY - pY) * (refY - pY);
          if (dist2 < *bestRealDist2) {
            *bestRealDist2 = dist2;
            *dangerRealX = pX;
            *dangerRealY = pY;
            *ptExReal = 1;
          }
        }
      }
    }
    ii = ii + 1;
  }
}

static void geoVtDangerPtCheckStopperPts(const GeoVtDangerPtInput_t *input,
                                         Real refX,
                                         Real refY,
                                         Real tgtX,
                                         Real tgtY,
                                         Real cosT,
                                         Real sinT,
                                         Real laneA,
                                         Real laneB,
                                         Real laneC,
                                         Real sideA,
                                         Real sideB,
                                         Real sideC,
                                         Real zoneLen,
                                         Real lenAhead,
                                         Integer *checkCount,
                                         Real *bestRealDist2,
                                         Real *dangerRealX,
                                         Real *dangerRealY,
                                         Integer *ptExReal) {
  Integer ii = 0;
  while (ii < input->StopperPtNum) {
    if (ii < DGR_PT_MAX) {
      Real pX = input->StopperPtX[ii];
      Real pY = input->StopperPtY[ii];
      if ((fabsf(pX) > 1e-3f) | (fabsf(pY) > 1e-3f)) {
        *checkCount = *checkCount + 1;
        Real dX = pX - tgtX;
        Real dY = pY - tgtY;
        Real relX = cosT * dX + sinT * dY;
        Real pLane = laneA * pX + laneB * pY + laneC;
        Real pSide = sideA * pX + sideB * pY + sideC;
        if ((relX > 0.0f) & (relX < (zoneLen + lenAhead)) & ((pLane * pSide) < 0.0f)) {
          Real dist2 = (refX - pX) * (refX - pX) + (refY - pY) * (refY - pY);
          if (dist2 < *bestRealDist2) {
            *bestRealDist2 = dist2;
            *dangerRealX = pX;
            *dangerRealY = pY;
            *ptExReal = 1;
          }
        }
      }
    }
    ii = ii + 1;
  }
}

/**
 * @brief 计算VT场景危险点，综合真实边界线/虚拟边界线/离散点并输出最危险候选点
 * @en_name geoVtDangerPt
 * @cn_name VT危险点检测
 * @type module
 * @param[IN] Real SltEntrPt1X 入口边点1X
 * @param[IN] Real SltEntrPt1Y 入口边点1Y
 * @param[IN] Real SltEntrPt2X 入口边点2X
 * @param[IN] Real SltEntrPt2Y 入口边点2Y
 * @param[IN] Real TgtX 目标位姿X
 * @param[IN] Real TgtY 目标位姿Y
 * @param[IN] Real TgtTheta 目标位姿航向
 * @param[IN] Real Radius 转弯半径
 * @param[IN] Integer PrkType 泊车方向1左2右
 * @param[IN] Integer BoundaryLineNum 真实边界线数量
 * @param[IN] Real[64] BoundaryP1X 真实边界点1X
 * @param[IN] Real[64] BoundaryP1Y 真实边界点1Y
 * @param[IN] Real[64] BoundaryP2X 真实边界点2X
 * @param[IN] Real[64] BoundaryP2Y 真实边界点2Y
 * @param[IN] Integer VirtualBoundaryLineNum 虚拟边界线数量
 * @param[IN] Real[64] VirtualBoundaryP1X 虚拟边界点1X
 * @param[IN] Real[64] VirtualBoundaryP1Y 虚拟边界点1Y
 * @param[IN] Real[64] VirtualBoundaryP2X 虚拟边界点2X
 * @param[IN] Real[64] VirtualBoundaryP2Y 虚拟边界点2Y
 * @param[IN] Integer BoundarySepPtNum 真实离散点数量
 * @param[IN] Real[128] BoundarySepPtX 真实离散点X
 * @param[IN] Real[128] BoundarySepPtY 真实离散点Y
 * @param[IN] Integer StopperPtNum 挡停点数量
 * @param[IN] Real[128] StopperPtX 挡停点X
 * @param[IN] Real[128] StopperPtY 挡停点Y
 * @param[OUT] Integer PtEx 最终危险点标志0无1真实2虚拟
 * @param[OUT] Real DangerPtX 真实危险点X（PtEx可选择虚拟通道）
 * @param[OUT] Real DangerPtY 真实危险点Y（PtEx可选择虚拟通道）
 * @param[OUT] Real DgrPtRX 真实危险点目标坐标系X
 * @param[OUT] Real DgrPtRY 真实危险点目标坐标系Y
 * @param[OUT] Integer PtExReal 真实边界危险点标志
 * @param[OUT] Real DangerPtRealX 真实边界危险点X
 * @param[OUT] Real DangerPtRealY 真实边界危险点Y
 * @param[OUT] Integer PtExVirtual 虚拟边界危险点标志
 * @param[OUT] Real DangerPtVirtualX 虚拟边界危险点X
 * @param[OUT] Real DangerPtVirtualY 虚拟边界危险点Y
 * @param[OUT] Integer CheckCount 有效检查次数
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite geo_VT_dangerPt.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoVtDangerPt(const GeoVtDangerPtInput_t *input, GeoVtDangerPtOutput_t *output) {
  const Real geoEps = 1e-6f;
  const Real lenAhead = 3.5f;
  const Real latMax = 3.0f;

  Real tgtX;
  Real tgtY;
  Real tgtTheta;
  Real cosT;
  Real sinT;
  Real entryP1X;
  Real entryP1Y;
  Real entryP2X;
  Real entryP2Y;
  Real refX;
  Real refY;
  Real zoneLen;
  Real laneA;
  Real laneB;
  Real laneC;
  Real sideA;
  Real sideB;
  Real sideC;
  Real checkLineOffset0;
  Real checkLineOffset1;
  Real sidePointX;
  Real sidePointY;

  Integer checkCount;
  Integer ptExReal;
  Integer ptExVirtual;
  Real dangerRealX;
  Real dangerRealY;
  Real dangerVirtualX;
  Real dangerVirtualY;
  Real bestRealDist2;
  Real bestVirtualDist2;

  tgtX = input->TgtX;
  tgtY = input->TgtY;
  tgtTheta = input->TgtTheta;
  cosT = cosf(tgtTheta);
  sinT = sinf(tgtTheta);
  entryP1X = input->SltEntrPt1X;
  entryP1Y = input->SltEntrPt1Y;
  entryP2X = input->SltEntrPt2X;
  entryP2Y = input->SltEntrPt2Y;

  Real entryVecX;
  Real entryVecY;
  Real headVecX;
  Real headVecY;
  Real det;
  Real interX;
  Real interY;

  entryVecX = entryP2X - entryP1X;
  entryVecY = entryP2Y - entryP1Y;
  headVecX = cosT;
  headVecY = sinT;
  det = entryVecX * headVecY - entryVecY * headVecX;

  if (fabsf(det) > geoEps) {
    Real dX;
    Real dY;
    Real numT;
    Real ratio;

    dX = tgtX - entryP1X;
    dY = tgtY - entryP1Y;
    numT = dX * headVecY - dY * headVecX;
    ratio = numT / det;
    interX = entryP1X + ratio * entryVecX;
    interY = entryP1Y + ratio * entryVecY;
  } else {
    interX = (entryP1X + entryP2X) * 0.5f;
    interY = (entryP1Y + entryP2Y) * 0.5f;
  }

  refX = interX + lenAhead * cosT;
  refY = interY + lenAhead * sinT;
  zoneLen = sqrtf((tgtX - interX) * (tgtX - interX) + (tgtY - interY) * (tgtY - interY));

  laneA = sinT;
  laneB = -cosT;
  laneC = tgtY * cosT - tgtX * sinT;

  checkLineOffset0 = input->Radius;
  if (checkLineOffset0 > latMax) {
    checkLineOffset1 = latMax;
  } else {
    checkLineOffset1 = checkLineOffset0;
  }

  if (input->PrkType == 1) {
    sidePointX = refX + checkLineOffset1 * cosf(tgtTheta + 1.57079632f);
    sidePointY = refY + checkLineOffset1 * sinf(tgtTheta + 1.57079632f);
  } else {
    sidePointX = refX + checkLineOffset1 * cosf(tgtTheta - 1.57079632f);
    sidePointY = refY + checkLineOffset1 * sinf(tgtTheta - 1.57079632f);
  }
  sideA = laneA;
  sideB = laneB;
  sideC = sidePointY * cosT - sidePointX * sinT;

  checkCount = 0;
  ptExReal = 0;
  ptExVirtual = 0;
  dangerRealX = 0.0f;
  dangerRealY = 0.0f;
  dangerVirtualX = 0.0f;
  dangerVirtualY = 0.0f;
  bestRealDist2 = 1.0e8f;
  bestVirtualDist2 = 1.0e8f;

  geoVtDangerPtCheckRealLines(input,
                              refX,
                              refY,
                              tgtX,
                              tgtY,
                              cosT,
                              sinT,
                              laneA,
                              laneB,
                              laneC,
                              sideA,
                              sideB,
                              sideC,
                              zoneLen,
                              lenAhead,
                              &checkCount,
                              &bestRealDist2,
                              &dangerRealX,
                              &dangerRealY,
                              &ptExReal);
  geoVtDangerPtCheckVirtualLines(input,
                                 refX,
                                 refY,
                                 tgtX,
                                 tgtY,
                                 cosT,
                                 sinT,
                                 laneA,
                                 laneB,
                                 laneC,
                                 sideA,
                                 sideB,
                                 sideC,
                                 zoneLen,
                                 lenAhead,
                                 &checkCount,
                                 &bestVirtualDist2,
                                 &dangerVirtualX,
                                 &dangerVirtualY,
                                 &ptExVirtual);
  geoVtDangerPtCheckSepPts(input,
                           refX,
                           refY,
                           tgtX,
                           tgtY,
                           cosT,
                           sinT,
                           laneA,
                           laneB,
                           laneC,
                           sideA,
                           sideB,
                           sideC,
                           zoneLen,
                           lenAhead,
                           &checkCount,
                           &bestRealDist2,
                           &dangerRealX,
                           &dangerRealY,
                           &ptExReal);
  geoVtDangerPtCheckStopperPts(input,
                               refX,
                               refY,
                               tgtX,
                               tgtY,
                               cosT,
                               sinT,
                               laneA,
                               laneB,
                               laneC,
                               sideA,
                               sideB,
                               sideC,
                               zoneLen,
                               lenAhead,
                               &checkCount,
                               &bestRealDist2,
                               &dangerRealX,
                               &dangerRealY,
                               &ptExReal);

  Integer ptEx;
  ptEx = 0;

  if (ptExReal > 0) {
    ptEx = ptExReal;
  }
  if (ptExVirtual > 0) {
    if (ptExReal == 0) {
      ptEx = ptExVirtual;
    } else {
      Real bestRealDist;
      Real bestVirtualDist;
      bestRealDist = sqrtf(bestRealDist2);
      bestVirtualDist = sqrtf(bestVirtualDist2);
      if (bestVirtualDist < (bestRealDist - 0.2f)) {
        ptEx = ptExVirtual;
      }
    }
  }

  Real dgrRX;
  Real dgrRY;
  if (ptExReal > 0) {
    Real dX;
    Real dY;
    dX = dangerRealX - tgtX;
    dY = dangerRealY - tgtY;
    dgrRX = cosT * dX + sinT * dY;
    dgrRY = cosT * dY - sinT * dX;
  } else {
    dgrRX = 0.0f;
    dgrRY = 0.0f;
  }

  output->PtEx = ptEx;
  output->DangerPtX = dangerRealX;
  output->DangerPtY = dangerRealY;
  output->DgrPtRX = dgrRX;
  output->DgrPtRY = dgrRY;
  output->PtExReal = ptExReal;
  output->DangerPtRealX = dangerRealX;
  output->DangerPtRealY = dangerRealY;
  output->PtExVirtual = ptExVirtual;
  output->DangerPtVirtualX = dangerVirtualX;
  output->DangerPtVirtualY = dangerVirtualY;
  output->CheckCount = checkCount;
}
