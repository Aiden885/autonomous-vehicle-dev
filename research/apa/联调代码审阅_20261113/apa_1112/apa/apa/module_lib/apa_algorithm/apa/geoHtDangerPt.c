#include "geoHtDangerPtTypes.h"
#include <math.h>

/**
 * @brief 计算HT场景危险点，综合真实边界线/虚拟边界线/离散点并输出最危险候选点
 * @en_name geoHtDangerPt
 * @cn_name HT危险点检测
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
 * @param[OUT] Real DangerPtX 最终危险点X
 * @param[OUT] Real DangerPtY 最终危险点Y
 * @param[OUT] Real DgrPtRX 最终危险点目标坐标系X
 * @param[OUT] Real DgrPtRY 最终危险点目标坐标系Y
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
 * @source matlab_rewrite geo_HT_dangerPt.m
 * @version 1.1
 * @date 2026-03-16
 * @author aide
 */
void geoHtDangerPt(const GeoHtDangerPtInput_t *input, GeoHtDangerPtOutput_t *output) {
  const Real geoEps = 1e-6f;
  Real tgtX;
  Real tgtY;
  Real tgtTheta;
  Real pt1X;
  Real pt1Y;
  Real pt1Theta;
  Real vehRelX;
  Real vehRelY;
  Real vehRelTheta;
  Real entryRel1X;
  Real entryRel1Y;
  Real entryRel1Theta;
  Real entryRel2X;
  Real entryRel2Y;
  Real entryRel2Theta;
  Real entryOffsetY;
  Real vehW;
  Real cptX;
  Real cptY;
  Real tsPtX;
  Real tsPtY;
  Real sinT;
  Real cosT;
  Real lnT0;
  Real lnT1;
  Real lnT2;
  Real lnC0;
  Real lnC1;
  Real lnC2;
  Integer pt1Trn;
  Integer checkCount;
  Integer ptExReal;
  Integer ptExVirtual;
  Real dangerRealX;
  Real dangerRealY;
  Real dangerVirtualX;
  Real dangerVirtualY;
  Real bestRealDist2;
  Real bestVirtualDist2;
  Boolean bestRealFromLine;

  Integer ptEx;
  Real dangerX;
  Real dangerY;
  Real dgrRX;
  Real dgrRY;
  Integer idx;

  tgtX = input->TgtX;
  tgtY = input->TgtY;
  tgtTheta = input->TgtTheta;
  pt1X = input->Pt1X;
  pt1Y = input->Pt1Y;
  pt1Theta = input->Pt1Theta;
  vehW = input->VehW;
  if (vehW <= geoEps) {
    vehW = 1.9f;
  }

  cosT = cosf(tgtTheta);
  sinT = sinf(tgtTheta);

  entryRel1X = cosT * (input->SltEntrPt1X - tgtX) + sinT * (input->SltEntrPt1Y - tgtY);
  entryRel1Y = (-sinT * (input->SltEntrPt1X - tgtX)) + (cosT * (input->SltEntrPt1Y - tgtY));
  entryRel1Theta = 0.0f;
  entryRel2X = cosT * (input->SltEntrPt2X - tgtX) + sinT * (input->SltEntrPt2Y - tgtY);
  entryRel2Y = (-sinT * (input->SltEntrPt2X - tgtX)) + (cosT * (input->SltEntrPt2Y - tgtY));
  entryRel2Theta = 0.0f;
  (void)entryRel1X;
  (void)entryRel1Theta;
  (void)entryRel2X;
  (void)entryRel2Theta;

  vehRelX = cosT * (input->VehX - tgtX) + sinT * (input->VehY - tgtY);
  vehRelY = (-sinT * (input->VehX - tgtX)) + (cosT * (input->VehY - tgtY));
  vehRelTheta = input->VehTheta - tgtTheta;
  (void)vehRelX;
  (void)vehRelTheta;

  if ((entryRel1Y > 1.0e-3f) && (entryRel2Y > 1.0e-3f)) {
    pt1Trn = 1;
  } else if ((entryRel1Y < -1.0e-3f) && (entryRel2Y < -1.0e-3f)) {
    pt1Trn = -1;
  } else if (vehRelY > 0.0f) {
    pt1Trn = 1;
  } else {
    pt1Trn = -1;
  }

  entryOffsetY = (entryRel1Y + entryRel2Y) * 0.5f;
  cptX = pt1X - (entryOffsetY * sinf(pt1Theta));
  cptY = pt1Y + (entryOffsetY * cosf(pt1Theta));
  if (pt1Trn == 1) {
    tsPtX = pt1X + ((vehW * 0.5f) * sinf(pt1Theta));
    tsPtY = pt1Y - ((vehW * 0.5f) * cosf(pt1Theta));
  } else {
    tsPtX = pt1X - ((vehW * 0.5f) * sinf(pt1Theta));
    tsPtY = pt1Y + ((vehW * 0.5f) * cosf(pt1Theta));
  }

  lnT0 = sinT;
  lnT1 = -cosT;
  lnT2 = (tsPtY * cosT) - (tsPtX * sinT);
  lnC0 = lnT0;
  lnC1 = lnT1;
  lnC2 = (cptY * cosT) - (cptX * sinT);

  checkCount = 0;
  ptExReal = 0;
  ptExVirtual = 0;
  dangerRealX = 0.0f;
  dangerRealY = 0.0f;
  dangerVirtualX = 0.0f;
  dangerVirtualY = 0.0f;
  bestRealDist2 = 10000.0f;
  bestVirtualDist2 = 10000.0f;
  bestRealFromLine = true;

  idx = 0;
  while ((idx < input->BoundaryLineNum) && (idx < DGR_LINE_MAX)) {
    Real x1;
    Real y1;
    Real x2;
    Real y2;
    Real lnB0;
    Real lnB1;
    Real lnB2;
    Real denom;
    Real ptHX;
    Real ptHY;
    Boolean onSegment;

    x1 = input->BoundaryP1X[idx];
    y1 = input->BoundaryP1Y[idx];
    x2 = input->BoundaryP2X[idx];
    y2 = input->BoundaryP2Y[idx];
    lnB0 = y2 - y1;
    lnB1 = x1 - x2;
    lnB2 = (x2 * y1) - (x1 * y2);
    denom = (lnB0 * lnB0) + (lnB1 * lnB1);
    if (denom > 0.1f) {
      Real relX;
      Real relY;
      Real relTheta;
      Real sideC;
      Real sideT;
      Real dist2;

      ptHX = ((lnB1 * lnB1 * cptX) - (lnB0 * lnB1 * cptY) - (lnB0 * lnB2)) / denom;
      ptHY = ((lnB0 * lnB0 * cptY) - (lnB0 * lnB1 * cptX) - (lnB1 * lnB2)) / denom;
      checkCount = checkCount + 1;
      onSegment = (((x1 - ptHX) * (x2 - ptHX)) <= 0.0f) && (((y1 - ptHY) * (y2 - ptHY)) <= 0.0f);
      if (onSegment) {
        relX = cosf(pt1Theta) * (ptHX - pt1X) + sinf(pt1Theta) * (ptHY - pt1Y);
        relY = (-sinf(pt1Theta) * (ptHX - pt1X)) + (cosf(pt1Theta) * (ptHY - pt1Y));
        relTheta = 0.0f;
        (void)relY;
        (void)relTheta;
        sideC = (lnC0 * ptHX) + (lnC1 * ptHY) + lnC2;
        sideT = (lnT0 * ptHX) + (lnT1 * ptHY) + lnT2;
        if ((relX > 0.0f) && ((sideC * sideT) < 0.0f)) {
          dist2 = ((cptX - ptHX) * (cptX - ptHX)) + ((cptY - ptHY) * (cptY - ptHY));
          if (dist2 < bestRealDist2) {
            bestRealDist2 = dist2;
            dangerRealX = ptHX;
            dangerRealY = ptHY;
            ptExReal = 1;
          }
        }
      } else {
        Integer epIdx;

        epIdx = 0;
        while (epIdx < 2) {
          Real pX;
          Real pY;

          if (epIdx == 0) {
            pX = x1;
            pY = y1;
          } else {
            pX = x2;
            pY = y2;
          }
          relX = cosf(pt1Theta) * (pX - pt1X) + sinf(pt1Theta) * (pY - pt1Y);
          sideC = (lnC0 * pX) + (lnC1 * pY) + lnC2;
          sideT = (lnT0 * pX) + (lnT1 * pY) + lnT2;
          if ((relX > 0.0f) && ((sideC * sideT) < 0.0f)) {
            dist2 = ((cptX - pX) * (cptX - pX)) + ((cptY - pY) * (cptY - pY));
            if (dist2 < bestRealDist2) {
              bestRealDist2 = dist2;
              dangerRealX = pX;
              dangerRealY = pY;
              ptExReal = 1;
            }
          }
          epIdx = epIdx + 1;
        }
      }
    }
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->VirtualBoundaryLineNum) && (idx < DGR_LINE_MAX)) {
    Real x1;
    Real y1;
    Real x2;
    Real y2;
    Real lnB0;
    Real lnB1;
    Real lnB2;
    Real denom;
    Real ptHX;
    Real ptHY;
    Boolean onSegment;

    x1 = input->VirtualBoundaryP1X[idx];
    y1 = input->VirtualBoundaryP1Y[idx];
    x2 = input->VirtualBoundaryP2X[idx];
    y2 = input->VirtualBoundaryP2Y[idx];
    lnB0 = y2 - y1;
    lnB1 = x1 - x2;
    lnB2 = (x2 * y1) - (x1 * y2);
    denom = (lnB0 * lnB0) + (lnB1 * lnB1);
    if (denom > 0.1f) {
      Real relX;
      Real sideC;
      Real sideT;
      Real dist2;

      ptHX = ((lnB1 * lnB1 * cptX) - (lnB0 * lnB1 * cptY) - (lnB0 * lnB2)) / denom;
      ptHY = ((lnB0 * lnB0 * cptY) - (lnB0 * lnB1 * cptX) - (lnB1 * lnB2)) / denom;
      checkCount = checkCount + 1;
      onSegment = (((x1 - ptHX) * (x2 - ptHX)) <= 0.0f) && (((y1 - ptHY) * (y2 - ptHY)) <= 0.0f);
      if (onSegment) {
        relX = cosf(pt1Theta) * (ptHX - pt1X) + sinf(pt1Theta) * (ptHY - pt1Y);
        sideC = (lnC0 * ptHX) + (lnC1 * ptHY) + lnC2;
        sideT = (lnT0 * ptHX) + (lnT1 * ptHY) + lnT2;
        if ((relX > 0.0f) && ((sideC * sideT) < 0.0f)) {
          dist2 = ((cptX - ptHX) * (cptX - ptHX)) + ((cptY - ptHY) * (cptY - ptHY));
          if (dist2 < bestVirtualDist2) {
            bestVirtualDist2 = dist2;
            dangerVirtualX = ptHX;
            dangerVirtualY = ptHY;
            ptExVirtual = 2;
          }
        }
      } else {
        Integer epIdx;

        epIdx = 0;
        while (epIdx < 2) {
          Real pX;
          Real pY;

          if (epIdx == 0) {
            pX = x1;
            pY = y1;
          } else {
            pX = x2;
            pY = y2;
          }
          relX = cosf(pt1Theta) * (pX - pt1X) + sinf(pt1Theta) * (pY - pt1Y);
          sideC = (lnC0 * pX) + (lnC1 * pY) + lnC2;
          sideT = (lnT0 * pX) + (lnT1 * pY) + lnT2;
          if ((relX > 0.0f) && ((sideC * sideT) < 0.0f)) {
            dist2 = ((cptX - pX) * (cptX - pX)) + ((cptY - pY) * (cptY - pY));
            if (dist2 < bestVirtualDist2) {
              bestVirtualDist2 = dist2;
              dangerVirtualX = pX;
              dangerVirtualY = pY;
              ptExVirtual = 2;
            }
          }
          epIdx = epIdx + 1;
        }
      }
    }
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->BoundarySepPtNum) && (idx < DGR_PT_MAX)) {
    Real pX;
    Real pY;
    Real relX;
    Real sideC;
    Real sideT;
    Real dist2;

    pX = input->BoundarySepPtX[idx];
    pY = input->BoundarySepPtY[idx];
    if ((fabsf(pX) > 1.0e-3f) || (fabsf(pY) > 1.0e-3f)) {
      checkCount = checkCount + 1;
      relX = cosT * (pX - tgtX) + sinT * (pY - tgtY);
      sideC = (lnC0 * pX) + (lnC1 * pY) + lnC2;
      sideT = (lnT0 * pX) + (lnT1 * pY) + lnT2;
      if ((relX > 0.0f) && ((sideC * sideT) < 0.0f)) {
        dist2 = ((cptX - pX) * (cptX - pX)) + ((cptY - pY) * (cptY - pY));
        if (dist2 < bestRealDist2) {
          bestRealDist2 = dist2;
          dangerRealX = pX;
          dangerRealY = pY;
          ptExReal = 1;
          bestRealFromLine = false;
        }
      }
    }
    idx = idx + 1;
  }

  idx = 0;
  while ((idx < input->StopperPtNum) && (idx < DGR_PT_MAX)) {
    Real pX;
    Real pY;
    Real relX;
    Real sideC;
    Real sideT;
    Real dist2;

    pX = input->StopperPtX[idx];
    pY = input->StopperPtY[idx];
    if ((fabsf(pX) > 1.0e-3f) || (fabsf(pY) > 1.0e-3f)) {
      checkCount = checkCount + 1;
      relX = cosT * (pX - tgtX) + sinT * (pY - tgtY);
      sideC = (lnC0 * pX) + (lnC1 * pY) + lnC2;
      sideT = (lnT0 * pX) + (lnT1 * pY) + lnT2;
      if ((relX > 0.0f) && ((sideC * sideT) < 0.0f)) {
        dist2 = ((cptX - pX) * (cptX - pX)) + ((cptY - pY) * (cptY - pY));
        if (dist2 < bestRealDist2) {
          bestRealDist2 = dist2;
          dangerRealX = pX;
          dangerRealY = pY;
          ptExReal = 1;
          bestRealFromLine = false;
        }
      }
    }
    idx = idx + 1;
  }

  ptEx = 0;
  dangerX = 0.0f;
  dangerY = 0.0f;

  if (ptExReal > 0) {
    ptEx = ptExReal;
    dangerX = dangerRealX;
    dangerY = dangerRealY;
  }
  if (ptExVirtual > 0) {
    if (ptExReal == 0) {
      ptEx = ptExVirtual;
      dangerX = dangerVirtualX;
      dangerY = dangerVirtualY;
    } else if ((bestVirtualDist2 < bestRealDist2) ||
               ((!bestRealFromLine) && (bestVirtualDist2 == bestRealDist2))) {
      ptEx = ptExVirtual;
      dangerX = dangerVirtualX;
      dangerY = dangerVirtualY;
    }
  }

  dgrRX = 0.0f;
  dgrRY = 0.0f;
  if (ptEx > 0) {
    Real dX;
    Real dY;
    dX = dangerX - tgtX;
    dY = dangerY - tgtY;
    dgrRX = cosT * dX + sinT * dY;
    dgrRY = cosT * dY - sinT * dX;

    {
      Real llfLow;
      Real llfHigh;
      Boolean condInOverhang;

      llfLow = input->LLf + 0.5f;
      llfHigh = input->LLf + 1.7f;
      condInOverhang = (dgrRX > llfLow) && (dgrRX < llfHigh);
      if (condInOverhang) {
        Real adjustment;
        Real altAdj;

        altAdj = (dgrRX - input->LLf) / 5.0f;
        if (altAdj < 0.2f) {
          adjustment = altAdj;
        } else {
          adjustment = 0.2f;
        }
        dgrRX = dgrRX - adjustment;

        dangerX = tgtX + dgrRX * cosT - dgrRY * sinT;
        dangerY = tgtY + dgrRX * sinT + dgrRY * cosT;
      }
    }
  }

  output->PtEx = ptEx;
  output->DangerPtX = dangerX;
  output->DangerPtY = dangerY;
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
