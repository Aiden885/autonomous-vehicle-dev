#include "geoStPtSpcDthEdVtTypes.h"
#include "geoAxisTransTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include <math.h>

#ifndef APA_MATH_EPSILON
#define APA_MATH_EPSILON 1e-6f
#endif

static inline void geoStPtSpcDthEdVtCheckSpace(VehicleCollisionCheckInput_t *vccIn,
                                               const ApaPlanningSceneQuery_t *scene,
                                               Real startX,
                                               Real startY,
                                               Real startTheta,
                                               Real stepDX,
                                               Real stepDY,
                                               Real checkLimit,
                                               Real initStep,
                                               Real minPrcs,
                                               Real *outSpc,
                                               Real *outCrshX,
                                               Real *outCrshY,
                                               Real *outCrshTheta,
                                               Integer *outIntrvRS,
                                               Integer *chkCnt) {
  Real sfLen = 0.0f;
  Real lenTmp = 0.0f;
  Boolean isCollision = false;
  Boolean loopCond = true;
  Real chckPosX;
  Real chckPosY;
  Real chckPosTheta;
  Real avlPosX = startX;
  Real avlPosY = startY;
  Real avlPosTheta = startTheta;
  Real crshPosX = startX;
  Real crshPosY = startY;
  Real crshPosTheta = startTheta;
  VehicleCollisionCheckOutput_t vccOut;
  while (loopCond) {
    if (lenTmp > checkLimit) {
      loopCond = false;
    }
    if (loopCond) {
      chckPosX = startX + lenTmp * stepDX;
      chckPosY = startY + lenTmp * stepDY;
      chckPosTheta = startTheta;
      vccIn->VehPosX = chckPosX;
      vccIn->VehPosY = chckPosY;
      vccIn->VehPosTheta = chckPosTheta;
      apaPlanningSceneCollisionCheck(vccIn, scene, &vccOut);
      *chkCnt = *chkCnt + 1;

      if (vccOut.IsCollision) {
        isCollision = true;
        *outIntrvRS = (Integer)vccOut.CollisionType;
        crshPosX = chckPosX;
        crshPosY = chckPosY;
        crshPosTheta = chckPosTheta;
        loopCond = false;
      }
    }
    if (loopCond) {
      sfLen = lenTmp;
      avlPosX = chckPosX;
      avlPosY = chckPosY;
      avlPosTheta = chckPosTheta;
      lenTmp = lenTmp + initStep;
    }
  }

  if (!isCollision) {
    *outSpc = sfLen;
  } else {
    if (lenTmp < APA_MATH_EPSILON) {
      *outSpc = 0.0f;
      *outCrshX = crshPosX;
      *outCrshY = crshPosY;
      *outCrshTheta = crshPosTheta;
    } else {
      Real lenPrcs = initStep * 0.5f;
      Real stPosX = avlPosX;
      Real stPosY = avlPosY;
      Real stPosTheta = avlPosTheta;
      Real edPosX = crshPosX;
      Real edPosY = crshPosY;
      Real edPosTheta = crshPosTheta;
      lenTmp = 0.0f;

      loopCond = true;
      while (loopCond) {
        if (lenPrcs <= minPrcs) {
          loopCond = false;
        }
        if (loopCond) {
          chckPosX = (stPosX + edPosX) * 0.5f;
          chckPosY = (stPosY + edPosY) * 0.5f;
          chckPosTheta = (stPosTheta + edPosTheta) * 0.5f;

          vccIn->VehPosX = chckPosX;
          vccIn->VehPosY = chckPosY;
          vccIn->VehPosTheta = chckPosTheta;
          apaPlanningSceneCollisionCheck(vccIn, scene, &vccOut);
          *chkCnt = *chkCnt + 1;

          if (vccOut.IsCollision) {
            edPosX = chckPosX;
            edPosY = chckPosY;
            edPosTheta = chckPosTheta;
          } else {
            stPosX = chckPosX;
            stPosY = chckPosY;
            stPosTheta = chckPosTheta;
            lenTmp = lenTmp + lenPrcs;
          }
          lenPrcs = lenPrcs * 0.5f;
        }
      }
      *outSpc = sfLen + lenTmp;
      *outCrshX = edPosX;
      *outCrshY = edPosY;
      *outCrshTheta = edPosTheta;
    }
  }
}

/**
 * @file geoStPtSpcDthEdVt.c
 * @brief 斜边垂直泊车预留空间分析
 * @en_name geoStPtSpcDthEdVt
 * @cn_name 斜边垂直泊车预留空间分析
 * @type module
 * @param[IN] GeoStPtSpcDthEdVtInput_t Input 预留空间分析输入
 * @param[OUT] GeoStPtSpcDthEdVtOutput_t Output 预留空间分析输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 几何规划
 * @source matlab_rewrite geo_StPtSpc_DthEdVt.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void geoStPtSpcDthEdVt(const GeoStPtSpcDthEdVtInput_t *input, GeoStPtSpcDthEdVtOutput_t *output) {
  geoStPtSpcDthEdVtWithScene(input, (const ApaPlanningSceneQuery_t *)0, output);
}

void geoStPtSpcDthEdVtWithScene(const GeoStPtSpcDthEdVtInput_t *input,
                                const ApaPlanningSceneQuery_t *scene,
                                GeoStPtSpcDthEdVtOutput_t *output) {
  GeoAxisTransInput_t transIn;
  GeoAxisTransOutput_t transOut;
  Real ptrY;
  Real ptrTheta;
  Real latChckMax;
  Real sinVal;
  Integer chkCnt;
  Real crshPosX;
  Real crshPosY;
  Real crshPosTheta;
  Real ftSpc;
  Real ltSpc;
  Real rtSpc;
  Integer intrvRS;
  VehicleCollisionCheckInput_t vccIn;
  Integer bi;
  Real cPosX;
  Real cPosY;
  Real cPosTheta;

  transIn.PtX = input->StPtX;
  transIn.PtY = input->StPtY;
  transIn.PtTheta = input->StPtTheta;
  transIn.Ori1X = 0.0f;
  transIn.Ori1Y = 0.0f;
  transIn.Ori1Theta = 0.0f;
  transIn.Ori2X = input->TarPtX;
  transIn.Ori2Y = input->TarPtY;
  transIn.Ori2Theta = input->TarPtTheta;

  geoAxisTrans(&transIn, &transOut);

  ptrY = transOut.ResultY;
  ptrTheta = transOut.ResultTheta;

  if (fabsf(ptrTheta) <= APA_MATH_EPSILON) {
    latChckMax = 2.0f;
  } else if (ptrTheta > 0.0f) {
    sinVal = sinf(ptrTheta);
    latChckMax = fmaxf(2.0f, 5.2f * tanf(ptrTheta * 0.5f) - ptrY / sinVal + 1.5f);
  } else {
    sinVal = sinf(ptrTheta);
    latChckMax = fmaxf(2.0f, -5.2f * tanf(ptrTheta * 0.5f) - ptrY / sinVal + 1.5f);
  }

  chkCnt = 0;
  crshPosX = 0.0f;
  crshPosY = 0.0f;
  crshPosTheta = 0.0f;
  ftSpc = 0.0f;
  ltSpc = 0.0f;
  rtSpc = 0.0f;
  intrvRS = 0;

  if (input->UseVccTemplate) {
    vccIn = input->VccTemplate;
  } else {
    vccIn = (VehicleCollisionCheckInput_t){0};
    vccIn.SlotPtsValid = true;
    vccIn.SlotPts[0] = input->SlotPointsX[0];
    vccIn.SlotPts[1] = input->SlotPointsY[0];
    vccIn.SlotPts[2] = input->SlotPointsX[1];
    vccIn.SlotPts[3] = input->SlotPointsY[1];
    vccIn.SlotPts[4] = input->SlotPointsX[2];
    vccIn.SlotPts[5] = input->SlotPointsY[2];
    vccIn.SlotPts[6] = input->SlotPointsX[3];
    vccIn.SlotPts[7] = input->SlotPointsY[3];

    vccIn.SeparateStopperNum = input->SeparateStopperNum;
    for (bi = 0; bi < input->SeparateStopperNum; bi = bi + 1) {
      vccIn.SeparateStopperX[bi] = input->SeparateStopperX[bi];
      vccIn.SeparateStopperY[bi] = input->SeparateStopperY[bi];
    }
  }
  if (!vccIn.SlotPtsValid) {
    vccIn.SlotPtsValid = true;
    vccIn.SlotPts[0] = input->SlotPointsX[0];
    vccIn.SlotPts[1] = input->SlotPointsY[0];
    vccIn.SlotPts[2] = input->SlotPointsX[1];
    vccIn.SlotPts[3] = input->SlotPointsY[1];
    vccIn.SlotPts[4] = input->SlotPointsX[2];
    vccIn.SlotPts[5] = input->SlotPointsY[2];
    vccIn.SlotPts[6] = input->SlotPointsX[3];
    vccIn.SlotPts[7] = input->SlotPointsY[3];
  }

  vccIn.CheckType = VCC_CHECK_TYPE_PLANNING;
  vccIn.CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  vccIn.CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  vccIn.UseAutoMargin = true;
  vccIn.VehLF = input->VehLF;
  vccIn.VehLB = input->VehLB;
  vccIn.VehW = input->VehW;
  if (vccIn.VehWB < APA_MATH_EPSILON) {
    vccIn.VehWB = input->VehLF;
  }
  if (vccIn.VehLL < APA_MATH_EPSILON) {
    vccIn.VehLL = input->VehLF + input->VehLB;
  }

  geoStPtSpcDthEdVtCheckSpace(&vccIn,
                              scene,
                              input->StPtX,
                              input->StPtY,
                              input->StPtTheta,
                              cosf(input->StPtTheta),
                              sinf(input->StPtTheta),
                              latChckMax,
                              0.5f,
                              0.1f,
                              &ftSpc,
                              &crshPosX,
                              &crshPosY,
                              &crshPosTheta,
                              &intrvRS,
                              &chkCnt);

  cPosX = input->StPtX + ftSpc * cosf(input->StPtTheta);
  cPosY = input->StPtY + ftSpc * sinf(input->StPtTheta);
  cPosTheta = input->StPtTheta;

  geoStPtSpcDthEdVtCheckSpace(&vccIn,
                              scene,
                              cPosX,
                              cPosY,
                              cPosTheta,
                              -sinf(cPosTheta),
                              cosf(cPosTheta),
                              3.5f,
                              1.0f,
                              0.1f,
                              &ltSpc,
                              &crshPosX,
                              &crshPosY,
                              &crshPosTheta,
                              &intrvRS,
                              &chkCnt);

  geoStPtSpcDthEdVtCheckSpace(&vccIn,
                              scene,
                              cPosX,
                              cPosY,
                              cPosTheta,
                              sinf(cPosTheta),
                              -cosf(cPosTheta),
                              3.5f,
                              1.0f,
                              0.05f,
                              &rtSpc,
                              &crshPosX,
                              &crshPosY,
                              &crshPosTheta,
                              &intrvRS,
                              &chkCnt);

  output->FtSpc = ftSpc;
  output->LtSpc = ltSpc;
  output->RtSpc = rtSpc;
  output->ChkCnt = chkCnt;
  output->IntrvRS = intrvRS;
  output->CrshPosX = crshPosX;
  output->CrshPosY = crshPosY;
  output->CrshPosTheta = crshPosTheta;
  output->ChckPosCX = cPosX;
  output->ChckPosCY = cPosY;
  output->ChckPosCTheta = cPosTheta;
}
