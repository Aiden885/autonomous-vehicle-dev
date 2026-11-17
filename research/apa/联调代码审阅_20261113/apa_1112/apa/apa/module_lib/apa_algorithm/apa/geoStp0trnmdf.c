#include "geoCircleIntrvCheckTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include "geoStPtSpcVtTypes.h"
#include "geoStp0trnmdfTypes.h"
#include <math.h>

static Boolean geoStp0TryPlan(Real stpStX,
                              Real stpStY,
                              Real stpStTheta,
                              Real vehX,
                              Real vehY,
                              Real vehTheta,
                              Real rd1,
                              Real rd2,
                              Real tarX,
                              Real tarY,
                              Real tarTheta,
                              Real sltPt1X,
                              Real sltPt1Y,
                              Real sltPt2X,
                              Real sltPt2Y,
                              Real sltPt3X,
                              Real sltPt3Y,
                              Real sltPt4X,
                              Real sltPt4Y,
                              Real vehW,
                              Real vehFrontOffset,
                              Integer *plnChkCnt,
                              GeoRev2CircleLinkOutput_t *revOut,
                              Real *st3RelX) {
  GeoRev2CircleLinkInput_t revIn;
  Boolean result0;

  result0 = false;
  revIn.Pt1X = stpStX;
  revIn.Pt1Y = stpStY;
  revIn.Pt1Theta = stpStTheta;
  revIn.Pt2X = vehX;
  revIn.Pt2Y = vehY;
  revIn.Pt2Theta = vehTheta;
  revIn.Dir = 1;
  revIn.R1 = rd1;
  revIn.R2 = rd2;
  geoRev2CircleLink(&revIn, revOut);

  if (revOut->GeoOk == 1) {
    GeoCircleIntrvCheckInput_t chkIn;
    GeoCircleIntrvCheckOutput_t chkOut;

    chkIn = (GeoCircleIntrvCheckInput_t){0};
    chkIn.CptX = revOut->Ct1X;
    chkIn.CptY = revOut->Ct1Y;
    chkIn.Radius = rd1;
    chkIn.Pt1X = stpStX;
    chkIn.Pt1Y = stpStY;
    chkIn.Pt1Theta = stpStTheta;
    chkIn.Pt2X = revOut->St2X;
    chkIn.Pt2Y = revOut->St2Y;
    chkIn.Pt2Theta = revOut->St2Theta;
    chkIn.GoDir = 1;
    chkIn.TrnDir = revOut->TrnD1;
    chkIn.CheckStep = 0.2f;
    chkIn.GoalX = tarX;
    chkIn.GoalY = tarY;
    chkIn.GoalYaw = tarTheta;
    chkIn.SlotP1X = sltPt1X;
    chkIn.SlotP1Y = sltPt1Y;
    chkIn.SlotP2X = sltPt2X;
    chkIn.SlotP2Y = sltPt2Y;
    chkIn.SlotP3X = sltPt3X;
    chkIn.SlotP3Y = sltPt3Y;
    chkIn.SlotP4X = sltPt4X;
    chkIn.SlotP4Y = sltPt4Y;
    chkIn.SlotClass = VCC_SLOT_CLASS_VERTICAL;
    chkIn.VehWidth = vehW;
    chkIn.VehFrontOffset = vehFrontOffset;
    geoCircleIntrvCheck(&chkIn, &chkOut);
    *plnChkCnt = *plnChkCnt + chkOut.CheckCount;

    if (chkOut.IntrvFlg <= 0) {
      Real cosVeh;
      Real sinVeh;
      Real dxSt3;
      Real dySt3;

      cosVeh = cosf(vehTheta);
      sinVeh = sinf(vehTheta);
      dxSt3 = revOut->St3X - vehX;
      dySt3 = revOut->St3Y - vehY;
      *st3RelX = cosVeh * dxSt3 + sinVeh * dySt3;

      if (*st3RelX >= 0.0f) {
        chkIn.CptX = revOut->Ct2X;
        chkIn.CptY = revOut->Ct2Y;
        chkIn.Radius = rd2;
        chkIn.Pt1X = revOut->St2X;
        chkIn.Pt1Y = revOut->St2Y;
        chkIn.Pt1Theta = revOut->St2Theta;
        chkIn.Pt2X = revOut->St3X;
        chkIn.Pt2Y = revOut->St3Y;
        chkIn.Pt2Theta = revOut->St3Theta;
        chkIn.GoDir = -1;
        chkIn.TrnDir = revOut->TrnD2;
        chkIn.CheckStep = 0.2f;
        chkIn.GoalX = tarX;
        chkIn.GoalY = tarY;
        chkIn.GoalYaw = tarTheta;
        chkIn.SlotP1X = sltPt1X;
        chkIn.SlotP1Y = sltPt1Y;
        chkIn.SlotP2X = sltPt2X;
        chkIn.SlotP2Y = sltPt2Y;
        chkIn.SlotP3X = sltPt3X;
        chkIn.SlotP3Y = sltPt3Y;
        chkIn.SlotP4X = sltPt4X;
        chkIn.SlotP4Y = sltPt4Y;
        chkIn.VehWidth = vehW;
        chkIn.VehFrontOffset = vehFrontOffset;
        geoCircleIntrvCheck(&chkIn, &chkOut);
        *plnChkCnt = *plnChkCnt + chkOut.CheckCount;

        if (chkOut.IntrvFlg <= 0) {
          result0 = true;
        }
      }
    }
  }

  return result0;
}

static void geoStp0trnmdfApplyMdf(const GeoStp0trnmdfInput_t *input,
                                  GeoStp0trnmdfOutput_t *output,
                                  const GeoStPtSpcVtOutput_t *spcOut,
                                  Real kApaRmin,
                                  Real kRFac,
                                  Real ctRelY,
                                  Boolean condLeft,
                                  Integer *plnChkCnt,
                                  Integer *mdfFlg) {
  Real wdHalf;
  Real llfTerm;
  Real spaceTerm;
  Real rd2;
  Real rSpcLmt;
  wdHalf = input->VehW * 0.5f;
  llfTerm = input->VehLlF + input->VehLfMin * 0.5f;
  if (condLeft) {
    spaceTerm = spcOut->RightSpace + wdHalf + ctRelY;
  } else {
    spaceTerm = spcOut->LeftSpace + wdHalf - ctRelY;
  }
  rd2 = 1.1f * kApaRmin;
  if (spaceTerm > llfTerm) {
    Real d2;
    d2 = spaceTerm * spaceTerm - llfTerm * llfTerm;
    if (d2 < 0.0f) {
      d2 = 0.0f;
    }
    rSpcLmt = sqrtf(d2) - wdHalf;
    if (rSpcLmt < rd2) {
      rSpcLmt = rd2;
    }
  } else {
    rSpcLmt = rd2;
  }

  Real stpStX;
  Real stpStY;
  Real stpStTheta;
  Real rd1First;
  Real rd1Use;
  Real st3RelX;
  Boolean ok;
  GeoRev2CircleLinkOutput_t revOut;
  stpStX = input->PathStx[2];
  stpStY = input->PathSty[2];
  stpStTheta = input->PathStTh[2];
  rd1First = kRFac * kApaRmin;

  ok = geoStp0TryPlan(stpStX,
                      stpStY,
                      stpStTheta,
                      input->VehX,
                      input->VehY,
                      input->VehTheta,
                      rd1First,
                      rd2,
                      input->TarX,
                      input->TarY,
                      input->TarTheta,
                      input->SltPt1X,
                      input->SltPt1Y,
                      input->SltPt2X,
                      input->SltPt2Y,
                      input->SltPt3X,
                      input->SltPt3Y,
                      input->SltPt4X,
                      input->SltPt4Y,
                      input->VehW,
                      llfTerm,
                      plnChkCnt,
                      &revOut,
                      &st3RelX);

  rd1Use = rd1First;
  if (ok == false) {
    if (rd1Use > rSpcLmt) {
      rd1Use = rSpcLmt;
    }
    ok = geoStp0TryPlan(stpStX,
                        stpStY,
                        stpStTheta,
                        input->VehX,
                        input->VehY,
                        input->VehTheta,
                        rd1Use,
                        rd2,
                        input->TarX,
                        input->TarY,
                        input->TarTheta,
                        input->SltPt1X,
                        input->SltPt1Y,
                        input->SltPt2X,
                        input->SltPt2Y,
                        input->SltPt3X,
                        input->SltPt3Y,
                        input->SltPt4X,
                        input->SltPt4Y,
                        input->VehW,
                        llfTerm,
                        plnChkCnt,
                        &revOut,
                        &st3RelX);
  }

  if (ok) {
    Integer segNew;
    Integer idx;
    *mdfFlg = 1;
    segNew = input->PathSegNum + 1;
    if (segNew > APA_PATH_SEG_MAX) {
      segNew = APA_PATH_SEG_MAX;
    }
    output->PathSegNum = segNew;

    idx = APA_PATH_SEG_MAX - 1;
    while (idx > 0) {
      output->PathGoDir[idx] = output->PathGoDir[idx - 1];
      output->PathTrnDir[idx] = output->PathTrnDir[idx - 1];
      output->PathCrv[idx] = output->PathCrv[idx - 1];
      output->PathTht[idx] = output->PathTht[idx - 1];
      output->PathLen[idx] = output->PathLen[idx - 1];
      output->PathStx[idx] = output->PathStx[idx - 1];
      output->PathSty[idx] = output->PathSty[idx - 1];
      output->PathStTh[idx] = output->PathStTh[idx - 1];
      idx = idx - 1;
    }

    idx = APA_PATH_CC_MAX - 1;
    while (idx > 1) {
      output->PathCC[idx] = output->PathCC[idx - 2];
      idx = idx - 1;
    }

    output->PathGoDir[0] = 1;
    output->PathGoDir[1] = 1;
    output->PathGoDir[2] = -1;

    output->PathTrnDir[0] = 0;
    output->PathTrnDir[1] = revOut.TrnD2;
    output->PathTrnDir[2] = revOut.TrnD1;

    output->PathCrv[0] = 0.0f;
    output->PathCrv[1] = (Real)revOut.TrnD2 / rd2;
    output->PathCrv[2] = (Real)revOut.TrnD1 / rd1Use;

    output->PathTht[0] = 0.0f;
    output->PathTht[1] = revOut.Tht2;
    output->PathTht[2] = revOut.Tht1;

    output->PathLen[0] = fabsf(st3RelX);
    output->PathLen[1] = revOut.Len2;
    output->PathLen[2] = revOut.Len1;

    output->PathStx[0] = input->VehX;
    output->PathStx[1] = revOut.St3X;
    output->PathStx[2] = revOut.St2X;

    output->PathSty[0] = input->VehY;
    output->PathSty[1] = revOut.St3Y;
    output->PathSty[2] = revOut.St2Y;

    output->PathStTh[0] = input->VehTheta;
    output->PathStTh[1] = revOut.St3Theta;
    output->PathStTh[2] = revOut.St2Theta;

    output->PathCC[0] = 0.0f;
    output->PathCC[1] = 0.0f;
    output->PathCC[2] = revOut.Ct2X;
    output->PathCC[3] = revOut.Ct2Y;
    output->PathCC[4] = revOut.Ct1X;
    output->PathCC[5] = revOut.Ct1Y;
  } else {
    *mdfFlg = 0;
  }
}

static Real geoStp0trnmdfCalcKFac(Real len1, Real len2) {
  Real kFacTmp;
  if (len1 > 2.2f) {
    kFacTmp = 1.0f;
  } else {
    Real kFacTmp1;
    Real kFacTmp2;
    if (len1 > 0.5f) {
      kFacTmp1 = len1 * 0.7f - 0.6f;
    } else {
      kFacTmp1 = 0.1f;
    }
    if (len2 < 2.5f) {
      kFacTmp2 = 1.0f;
    } else {
      if (len2 < 4.0f) {
        kFacTmp2 = 2.5f / (len2 * len2);
      } else {
        kFacTmp2 = 0.1f;
      }
    }
    kFacTmp = kFacTmp1 * kFacTmp2;
  }
  return kFacTmp;
}

/**
 * @brief 对已有VT路径执行stp0转向修正，必要时将头部两段重构为三段
 * @en_name geoStp0trnmdf
 * @cn_name stp0转向修正
 * @type module
 * @param[IN] Real VehX 车辆X
 * @param[IN] Real VehY 车辆Y
 * @param[IN] Real VehTheta 车辆航向
 * @param[IN] Real TarX 目标X
 * @param[IN] Real TarY 目标Y
 * @param[IN] Real TarTheta 目标航向
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Integer SltEntr 车位入口边编号
 * @param[IN] Integer SltState 车位状态
 * @param[IN] Real RminDefault 默认最小半径
 * @param[IN] Real RminSuccSlot 成功车位最小半径
 * @param[IN] Real VehW 车宽
 * @param[IN] Real VehLlF 前悬到参考点长度
 * @param[IN] Real VehLfMin 最小前向安全长度
 * @param[IN] Integer PathSegNum 当前路径段数
 * @param[IN] Integer[30] PathGoDir 行驶方向数组
 * @param[IN] Integer[30] PathTrnDir 转向数组
 * @param[IN] Real[30] PathCrv 曲率数组
 * @param[IN] Real[30] PathTht 转角数组
 * @param[IN] Real[30] PathLen 长度数组
 * @param[IN] Real[30] PathStx 起点X数组
 * @param[IN] Real[30] PathSty 起点Y数组
 * @param[IN] Real[30] PathStTh 起点航向数组
 * @param[IN] Real[60] PathCC 圆心数组
 * @param[OUT] Integer MdfFlg 修正成功标志1成功0失败
 * @param[OUT] Integer PlnChkCnt 规划检查次数
 * @param[OUT] Integer PathSegNum 修正后路径段数
 * @param[OUT] Integer[30] PathGoDir 行驶方向数组
 * @param[OUT] Integer[30] PathTrnDir 转向数组
 * @param[OUT] Real[30] PathCrv 曲率数组
 * @param[OUT] Real[30] PathTht 转角数组
 * @param[OUT] Real[30] PathLen 长度数组
 * @param[OUT] Real[30] PathStx 起点X数组
 * @param[OUT] Real[30] PathSty 起点Y数组
 * @param[OUT] Real[30] PathStTh 起点航向数组
 * @param[OUT] Real[60] PathCC 圆心数组
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 路径规划
 * @source matlab_rewrite geo_stp0trnmdf.m
 * @version 1.0
 * @date 2026-03-05
 * @author aide
 */
void geoStp0trnmdf(const GeoStp0trnmdfInput_t *input, GeoStp0trnmdfOutput_t *output) {
  Integer i;
  Integer plnChkCnt;

  Integer mdfFlg;
  Real kApaRmin;

  plnChkCnt = 0;
  mdfFlg = 0;

  (void)input->TarTheta;

  if (input->SltState == 1) {
    kApaRmin = input->RminSuccSlot;
  } else {
    if (input->SltState == 3) {
      kApaRmin = input->RminSuccSlot;
    } else {
      kApaRmin = input->RminDefault;
    }
  }

  output->PathSegNum = input->PathSegNum;
  i = 0;
  while (i < APA_PATH_SEG_MAX) {
    output->PathGoDir[i] = input->PathGoDir[i];
    output->PathTrnDir[i] = input->PathTrnDir[i];
    output->PathCrv[i] = input->PathCrv[i];
    output->PathTht[i] = input->PathTht[i];
    output->PathLen[i] = input->PathLen[i];
    output->PathStx[i] = input->PathStx[i];
    output->PathSty[i] = input->PathSty[i];
    output->PathStTh[i] = input->PathStTh[i];
    i = i + 1;
  }
  i = 0;
  while (i < APA_PATH_CC_MAX) {
    output->PathCC[i] = input->PathCC[i];
    i = i + 1;
  }

  if (input->PathSegNum >= 3) {
    GeoStPtSpcVtInput_t spcIn;
    GeoStPtSpcVtOutput_t spcOut;
    spcIn = (GeoStPtSpcVtInput_t){0};
    spcOut = (GeoStPtSpcVtOutput_t){0};
    spcIn.VehPosX = input->PathStx[1];
    spcIn.VehPosY = input->PathSty[1];
    spcIn.VehPosTheta = input->PathStTh[1];
    spcIn.TarPtX = input->TarX;
    spcIn.TarPtY = input->TarY;
    spcIn.TarPtTheta = input->TarTheta;
    spcIn.SltPt1X = input->SltPt1X;
    spcIn.SltPt1Y = input->SltPt1Y;
    spcIn.SltPt2X = input->SltPt2X;
    spcIn.SltPt2Y = input->SltPt2Y;
    spcIn.SltPt3X = input->SltPt3X;
    spcIn.SltPt3Y = input->SltPt3Y;
    spcIn.SltPt4X = input->SltPt4X;
    spcIn.SltPt4Y = input->SltPt4Y;
    spcIn.SltEntr = input->SltEntr;
    spcIn.SltType = 2;
    spcIn.CheckLen = 0.0f;
    spcIn.FallbackSpace = 4.1f;
    geoStPtSpcVt(&spcIn, &spcOut);
    plnChkCnt = plnChkCnt + spcOut.CheckCount;

    Real len1;
    Real len2;
    Real kFacTmp;
    len1 = input->PathLen[0];
    len2 = input->PathLen[1];
    kFacTmp = geoStp0trnmdfCalcKFac(len1, len2);

    Real kRFac;
    kRFac = 1.0f + kFacTmp;
    if (kRFac < 1.06f) {
      kRFac = 1.06f;
    }
    if (kRFac > 1.5f) {
      kRFac = 1.5f;
    }

    Real cosVeh;
    Real sinVeh;
    Real dxTar;
    Real dyTar;
    Real tarRelY;
    Real dxCt;
    Real dyCt;
    Real ctRelY;
    cosVeh = cosf(input->VehTheta);
    sinVeh = sinf(input->VehTheta);
    dxTar = input->TarX - input->VehX;
    dyTar = input->TarY - input->VehY;
    tarRelY = cosVeh * dyTar - sinVeh * dxTar;
    dxCt = input->PathCC[2] - input->VehX;
    dyCt = input->PathCC[3] - input->VehY;
    ctRelY = cosVeh * dyCt - sinVeh * dxCt;

    Boolean condLeft;
    Boolean condRight;
    Boolean condDoMdf;
    condLeft = (tarRelY > 0.0f) && (input->PathTrnDir[0] == 0) && (input->PathTrnDir[1] == 1);
    condRight = (tarRelY < 0.0f) && (input->PathTrnDir[0] == 0) && (input->PathTrnDir[1] == -1);
    if (condLeft) {
      condDoMdf = true;
    } else {
      condDoMdf = condRight;
    }

    if (condDoMdf) {
      geoStp0trnmdfApplyMdf(
          input, output, &spcOut, kApaRmin, kRFac, ctRelY, condLeft, &plnChkCnt, &mdfFlg);
    }
  }

  output->MdfFlg = mdfFlg;
  output->PlnChkCnt = plnChkCnt;
}
