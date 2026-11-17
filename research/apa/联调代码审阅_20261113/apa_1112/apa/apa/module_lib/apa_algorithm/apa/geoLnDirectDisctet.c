#include "geoLnDirectDisctetTypes.h"
#include <math.h>

/**
 * @brief 将直线段按固定步长离散为路径点序列，对齐 MATLAB
 *        geo_LnDirectDisctet.m：贴近清理（点接近车辆位姿时整段重置）、
 *        odo 写回、按车位类型 (垂直/平行) 与末段标志选择终点。
 * @en_name geoLnDirectDisctet
 * @cn_name 直线离散化
 * @type module
 * @param[IN] Real LineLen 直线段长度
 * @param[IN] Integer GoDir 行驶方向1前进负1后退
 * @param[IN] Real StepLen 离散步长
 * @param[IN] Real EdPtX 线段终点X
 * @param[IN] Real EdPtY 线段终点Y
 * @param[IN] Real EdPtTheta 线段终点航向
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
 * @source matlab_rewrite geo_LnDirectDisctet.m
 * @version 2.0
 * @date 2026-05-01
 * @author aide
 */
void geoLnDirectDisctet(const GeoLnDirectDisctetInput_t *input,
                        GeoLnDirectDisctetOutput_t *output) {
  Real lineLen;
  Integer goDir;
  Real stepLen;
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

  lineLen = input->LineLen;
  goDir = input->GoDir;
  stepLen = input->StepLen;
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
  goDirF = (Real)goDir;

  Real remaining;
  Boolean condPositiveStep;
  remaining = lineLen;
  condPositiveStep = (stepLen > 0.0f);

  Boolean condLoop;
  condLoop = condPositiveStep && (remaining > stepLen);

  while (condLoop) {
    Real cosTheta;
    Real sinTheta;
    Real ptSaveX;
    Real ptSaveY;
    Real ptSaveTheta;
    Boolean condClose;
    Boolean condReset;

    cosTheta = cosf(lstTheta);
    sinTheta = sinf(lstTheta);
    ptSaveX = lstX + goDirF * stepLen * cosTheta;
    ptSaveY = lstY + goDirF * stepLen * sinTheta;
    ptSaveTheta = lstTheta;

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

    condClose = ((double)diffX < 0.2) && ((double)diffY < 0.2) &&
                ((double)diffTheta < 0.026179938779914941);

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

      condBranchA = ((double)absPtrY < 0.05);
      condBranchB =
          (remaining > 1.5f) && ((double)absPtrY < 0.15) && (absPtrY < (0.04f * remaining));
      if (((double)absPtrX < 0.15) && ((double)remaining > 0.8) && (condBranchA || condBranchB)) {
        condReset = true;
      }
    }

    if (condReset) {

      ptN = 1;
    } else {
      Integer newN;
      Integer odoIdx;
      newN = ptN + 1;
      if (newN > PATH_PT_MAX) {
        newN = PATH_PT_MAX;
      }
      ptN = newN;

      output->PthPtX[ptN - 1] = ptSaveX;
      output->PthPtY[ptN - 1] = ptSaveY;
      output->PthPtTht[ptN - 1] = ptSaveTheta;
      odoIdx = ptN - 1 - 1;
      if (odoIdx < 0) {
        odoIdx = 0;
      }
      output->PthPtOdo[odoIdx] = goDirF * stepLen;
    }

    lstX = ptSaveX;
    lstY = ptSaveY;
    lstTheta = ptSaveTheta;
    remaining = remaining - stepLen;
    condLoop = condPositiveStep && (remaining > stepLen);
  }

  if (sltTyp == 0) {

    if ((double)remaining >= 0.0099) {
      Integer newN;
      Integer odoIdx;
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
        odoIdx = ptN - 2;
        output->PthPtOdo[odoIdx] = goDirF * remaining;
      }
    }
  } else {

    if ((double)remaining >= 0.0099) {
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
      odoIdx = ptN - 1 - 1;
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
