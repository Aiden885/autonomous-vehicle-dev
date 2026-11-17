#include "geoTrnDirectDisctetTypes.h"
#include <math.h>

/**
 * @brief 将圆弧段按固定步长离散为路径点序列，对齐 MATLAB
 *        geo_TrnDirectDisctet.m：贴近清理 (清理阈值随车位类型 0.2 / 0.06)、
 *        odo 写回、按车位类型与末段标志选择终点、超角度 break。
 * @en_name geoTrnDirectDisctet
 * @cn_name 弧线离散化
 * @type module
 * @param[IN] Real Crv 曲率
 * @param[IN] Real CcX 圆心X
 * @param[IN] Real CcY 圆心Y
 * @param[IN] Real ArcLen 弧长
 * @param[IN] Integer GoDir 行驶方向1前进负1后退
 * @param[IN] Real StepLen 离散步长
 * @param[IN] Real ArcAngle 圆弧角度
 * @param[IN] Real StPtX 弧线起点X
 * @param[IN] Real StPtY 弧线起点Y
 * @param[IN] Real StPtTheta 弧线起点航向
 * @param[IN] Real EdPtX 弧线终点X
 * @param[IN] Real EdPtY 弧线终点Y
 * @param[IN] Real EdPtTheta 弧线终点航向
 * @param[IN] Real LstPtX 上一离散点X
 * @param[IN] Real LstPtY 上一离散点Y
 * @param[IN] Real LstPtTheta 上一离散点航向
 * @param[IN] Real VehPosX 车辆位姿X
 * @param[IN] Real VehPosY 车辆位姿Y
 * @param[IN] Real VehPosTheta 车辆位姿航向
 * @param[IN] Integer SltTyp 车位类型0垂直1平行
 * @param[IN] Integer LstStp 末段标志1最后一段
 * @param[IN] Real TarPtX 目标点X
 * @param[IN] Real TarPtY 目标点Y
 * @param[IN] Real TarPtTheta 目标点航向
 * @param[IN] Integer InPtCount 进入时已有点数
 * @param[IN] Real[100] InPtX 进入时点X数组
 * @param[IN] Real[100] InPtY 进入时点Y数组
 * @param[IN] Real[100] InPtTht 进入时点航向数组
 * @param[IN] Real[100] InPtOdo 进入时点里程数组
 * @param[OUT] Integer PtCount 有效路径点数量
 * @param[OUT] Real[100] PthPtX 离散后点X数组
 * @param[OUT] Real[100] PthPtY 离散后点Y数组
 * @param[OUT] Real[100] PthPtTht 离散后点航向数组
 * @param[OUT] Real[100] PthPtOdo 离散后点里程数组
 * @param[OUT] Real LstPtX 最后离散点X
 * @param[OUT] Real LstPtY 最后离散点Y
 * @param[OUT] Real LstPtTheta 最后离散点航向
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 路径离散化
 * @source matlab_rewrite geo_TrnDirectDisctet.m
 * @version 2.0
 * @date 2026-05-01
 * @author aide
 */
void geoTrnDirectDisctet(const GeoTrnDirectDisctetInput_t *input,
                         GeoTrnDirectDisctetOutput_t *output) {
  Real crv;
  Real ccX;
  Real ccY;
  Real arcLen;
  Integer goDir;
  Real stepLen;
  Real arcAngle;
  Real stPtTheta;
  Real edPtX;
  Real edPtY;
  Real edPtTheta;
  Real lstX;
  Real lstY;
  Real lstTheta;
  Real vehX;
  Real vehY;
  Real vehTheta;
  Integer sltTyp;
  Integer lstStp;
  Real tarX;
  Real tarY;
  Real tarTheta;
  Integer ptN;
  Integer idx;

  crv = input->Crv;
  ccX = input->CcX;
  ccY = input->CcY;
  arcLen = input->ArcLen;
  goDir = input->GoDir;
  stepLen = input->StepLen;
  arcAngle = input->ArcAngle;
  stPtTheta = input->StPtTheta;
  edPtX = input->EdPtX;
  edPtY = input->EdPtY;
  edPtTheta = input->EdPtTheta;
  lstX = input->LstPtX;
  lstY = input->LstPtY;
  lstTheta = input->LstPtTheta;
  vehX = input->VehPosX;
  vehY = input->VehPosY;
  vehTheta = input->VehPosTheta;
  sltTyp = input->SltTyp;
  lstStp = input->LstStp;
  tarX = input->TarPtX;
  tarY = input->TarPtY;
  tarTheta = input->TarPtTheta;

  stPtTheta = lstTheta + remainderf(stPtTheta - lstTheta, 2.0f * APA_PI);
  vehTheta = lstTheta + remainderf(vehTheta - lstTheta, 2.0f * APA_PI);

  ptN = input->InPtCount;
  if (ptN < 0) {
    ptN = 0;
  }
  if (ptN > PATH_PT_MAX) {
    ptN = PATH_PT_MAX;
  }
  idx = 0;
  while (idx < PATH_PT_MAX) {
    output->PthPtX[idx] = input->InPtX[idx];
    output->PthPtY[idx] = input->InPtY[idx];
    output->PthPtTht[idx] = input->InPtTht[idx];
    output->PthPtOdo[idx] = input->InPtOdo[idx];
    idx = idx + 1;
  }

  Real goDirF;
  Real thtStep;
  goDirF = (Real)goDir;
  thtStep = stepLen * crv * goDirF;

  Real absArcAngle;
  absArcAngle = fabsf(arcAngle);

  Real absCrv;
  absCrv = fabsf(crv);

  Real remaining;
  Boolean condPositiveStep;
  remaining = arcLen;
  condPositiveStep = (stepLen > 0.0f);

  if (absCrv < GEO_EPSILON) {

  } else {
    Real kSkpThd;
    if (sltTyp == 0) {
      kSkpThd = 0.2f;
    } else {
      kSkpThd = 0.06f;
    }

    Real curTheta;
    Boolean nearSegmentStart;
    Boolean condLoop;
    Real nextAngleDiff;
    Real nextAbsAngleDiff;

    nearSegmentStart = (hypotf(lstX - input->StPtX, lstY - input->StPtY) <= 0.015f) &&
                       (fabsf(lstTheta - stPtTheta) <= 0.01f);
    curTheta = nearSegmentStart ? stPtTheta : lstTheta;

    condLoop = condPositiveStep && (remaining > stepLen);
    if (condLoop) {
      nextAngleDiff = (curTheta + thtStep) - stPtTheta;
      nextAbsAngleDiff = fabsf(nextAngleDiff);
      if (nextAbsAngleDiff > absArcAngle) {
        condLoop = false;
      }
    }

    while (condLoop) {
      Real newTheta;

      newTheta = curTheta + thtStep;

      Real sinNew;
      Real cosNew;
      Real invCrv;
      Real ptSaveX;
      Real ptSaveY;
      Real ptSaveTheta;

      sinNew = sinf(newTheta);
      cosNew = cosf(newTheta);
      invCrv = 1.0f / crv;
      ptSaveX = ccX + invCrv * sinNew;
      ptSaveY = ccY - invCrv * cosNew;
      ptSaveTheta = newTheta;

      Real diffX;
      Real diffY;
      Real diffTheta;
      diffX = ptSaveX - vehX;
      diffY = ptSaveY - vehY;
      diffTheta = ptSaveTheta - vehTheta;
      if (diffX < 0.0f) {
        diffX = -diffX;
      }
      if (diffY < 0.0f) {
        diffY = -diffY;
      }
      if (diffTheta < 0.0f) {
        diffTheta = -diffTheta;
      }

      Boolean condClose;
      Boolean condReset;
      condClose = (diffX < kSkpThd) && (diffY < kSkpThd) && (diffTheta < (1.5f * APA_PI / 180.0f));

      condReset = false;
      if (condClose) {
        Real cosV;
        Real sinV;
        Real dvx;
        Real dvy;
        Real ptrX;
        Real ptrY;
        Real absPtrX;
        Real absPtrY;
        Boolean condBranchA;
        Boolean condBranchB;

        cosV = cosf(vehTheta);
        sinV = sinf(vehTheta);
        dvx = ptSaveX - vehX;
        dvy = ptSaveY - vehY;
        ptrX = cosV * dvx + sinV * dvy;
        ptrY = -sinV * dvx + cosV * dvy;
        absPtrX = ptrX;
        if (absPtrX < 0.0f) {
          absPtrX = -absPtrX;
        }
        absPtrY = ptrY;
        if (absPtrY < 0.0f) {
          absPtrY = -absPtrY;
        }

        condBranchA = (absPtrY < 0.05f);
        condBranchB = (remaining > 1.5f) && (absPtrY < 0.15f) && (absPtrY < (0.04f * remaining));
        if ((absPtrX < 0.15f) && (remaining > 0.8f) && (condBranchA || condBranchB)) {
          condReset = true;
        }
      }

      if (condReset) {
        ptN = 1;
      } else {
        Integer newN;
        newN = ptN + 1;
        if (newN > PATH_PT_MAX) {
          newN = PATH_PT_MAX;
        }
        ptN = newN;
        output->PthPtX[ptN - 1] = ptSaveX;
        output->PthPtY[ptN - 1] = ptSaveY;
        output->PthPtTht[ptN - 1] = ptSaveTheta;
        if (ptN > 1) {
          output->PthPtOdo[ptN - 2] = goDirF * stepLen;
        }
      }

      curTheta = ptSaveTheta;
      lstX = ptSaveX;
      lstY = ptSaveY;
      lstTheta = ptSaveTheta;
      remaining = remaining - stepLen;
      condLoop = condPositiveStep && (remaining > stepLen);
      if (condLoop) {
        nextAngleDiff = (curTheta + thtStep) - stPtTheta;
        nextAbsAngleDiff = fabsf(nextAngleDiff);
        if (nextAbsAngleDiff > absArcAngle) {
          condLoop = false;
        }
      }
    }
  }

  if (sltTyp == 0) {
    if (remaining >= 0.0099f) {
      Integer newN;
      newN = ptN + 1;
      if (newN > PATH_PT_MAX) {
        newN = PATH_PT_MAX;
      }
      ptN = newN;
      output->PthPtX[ptN - 1] = edPtX;
      output->PthPtY[ptN - 1] = edPtY;
      output->PthPtTht[ptN - 1] = edPtTheta;
      lstX = edPtX;
      lstY = edPtY;
      lstTheta = edPtTheta;
      if (ptN > 1) {
        output->PthPtOdo[ptN - 2] = goDirF * remaining;
      }
    }
  } else {
    if (remaining >= 0.0099f) {
      Integer newN;
      Integer odoIdx;
      newN = ptN + 1;
      if (newN > PATH_PT_MAX) {
        newN = PATH_PT_MAX;
      }
      ptN = newN;
      if (lstStp == 1) {
        output->PthPtX[ptN - 1] = tarX;
        output->PthPtY[ptN - 1] = tarY;
        output->PthPtTht[ptN - 1] = tarTheta;
      } else {
        output->PthPtX[ptN - 1] = edPtX;
        output->PthPtY[ptN - 1] = edPtY;
        output->PthPtTht[ptN - 1] = edPtTheta;
      }
      odoIdx = ptN - 2;
      if (odoIdx < 0) {
        odoIdx = 0;
      }
      output->PthPtOdo[odoIdx] = goDirF * remaining;
    }
    if (ptN > 0) {
      lstX = output->PthPtX[ptN - 1];
      lstY = output->PthPtY[ptN - 1];
      lstTheta = output->PthPtTht[ptN - 1];
    }
  }

  output->PtCount = ptN;
  output->LstPtX = lstX;
  output->LstPtY = lstY;
  output->LstPtTheta = lstTheta;
}
