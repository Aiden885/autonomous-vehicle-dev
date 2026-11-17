#include "geoCircleIntrvCheckTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include "geoStp1trnmdfTypes.h"
#include <math.h>

static Real geoStp1trnmdfCalcKFac(Real len1In, Real len2In) {
  Real kFacTmp;
  if (len1In > 2.5f) {
    kFacTmp = 1.0f;
  } else {
    Real kFacTmp1;
    Real kFacTmp2;
    if (len1In > 0.5f) {
      kFacTmp1 = len1In * 0.6f - 0.5f;
    } else {
      kFacTmp1 = 0.1f;
    }
    if (len2In < 2.5f) {
      kFacTmp2 = 1.0f;
    } else {
      if (len2In < 4.0f) {
        kFacTmp2 = 2.5f / (len2In * len2In);
      } else {
        kFacTmp2 = 0.1f;
      }
    }
    kFacTmp = kFacTmp1 * kFacTmp2;
  }
  return kFacTmp;
}

static void geoStp1trnmdfApplyMdf(const GeoStp1trnmdfInput_t *input,
                                  GeoStp1trnmdfOutput_t *output,
                                  Real kApaRmin,
                                  Real kRFac,
                                  Real vehX,
                                  Real vehY,
                                  Real vehTheta,
                                  Real cosVeh,
                                  Real sinVeh,
                                  Integer *plnChkCnt,
                                  Integer *mdfFlg) {
  Real rd1;
  Real rd2;
  Real stpStX;
  Real stpStY;
  Real stpStTh;
  Real stpEdX;
  Real stpEdY;
  Real stpEdTh;

  rd1 = kRFac * kApaRmin;
  rd2 = 1.1f * kApaRmin;
  stpStX = input->PathStx[2];
  stpStY = input->PathSty[2];
  stpStTh = input->PathStTh[2];
  stpEdX = vehX;
  stpEdY = vehY;
  stpEdTh = vehTheta;

  GeoRev2CircleLinkInput_t revIn;
  GeoRev2CircleLinkOutput_t revOut;
  revIn.Pt1X = stpStX;
  revIn.Pt1Y = stpStY;
  revIn.Pt1Theta = stpStTh;
  revIn.Pt2X = stpEdX;
  revIn.Pt2Y = stpEdY;
  revIn.Pt2Theta = stpEdTh;
  revIn.Dir = -1;
  revIn.R1 = rd1;
  revIn.R2 = rd2;
  geoRev2CircleLink(&revIn, &revOut);

  if (revOut.GeoOk == 1) {
    GeoCircleIntrvCheckInput_t chkIn;
    GeoCircleIntrvCheckOutput_t chkOut;
    chkIn = (GeoCircleIntrvCheckInput_t){0};
    chkIn.CptX = revOut.Ct1X;
    chkIn.CptY = revOut.Ct1Y;
    chkIn.Radius = rd1;
    chkIn.Pt1X = stpStX;
    chkIn.Pt1Y = stpStY;
    chkIn.Pt1Theta = stpStTh;
    chkIn.Pt2X = revOut.St2X;
    chkIn.Pt2Y = revOut.St2Y;
    chkIn.Pt2Theta = revOut.St2Theta;
    chkIn.GoDir = -1;
    chkIn.TrnDir = revOut.TrnD1;
    chkIn.CheckStep = 0.2f;
    chkIn.GoalX = input->TarX;
    chkIn.GoalY = input->TarY;
    chkIn.GoalYaw = input->TarTheta;
    chkIn.SlotP1X = input->SltPt1X;
    chkIn.SlotP1Y = input->SltPt1Y;
    chkIn.SlotP2X = input->SltPt2X;
    chkIn.SlotP2Y = input->SltPt2Y;
    chkIn.SlotP3X = input->SltPt3X;
    chkIn.SlotP3Y = input->SltPt3Y;
    chkIn.SlotP4X = input->SltPt4X;
    chkIn.SlotP4Y = input->SltPt4Y;
    chkIn.SlotClass = VCC_SLOT_CLASS_VERTICAL;
    chkIn.VehWidth = input->VehW;
    chkIn.VehFrontOffset = input->VehLlF + input->VehLfMin * 0.5f;
    geoCircleIntrvCheck(&chkIn, &chkOut);
    *plnChkCnt = *plnChkCnt + chkOut.CheckCount;

    Real dxSt3;
    Real dySt3;
    Real st3RelX;
    dxSt3 = revOut.St3X - vehX;
    dySt3 = revOut.St3Y - vehY;
    st3RelX = cosVeh * dxSt3 + sinVeh * dySt3;

    Boolean condFail1;
    Boolean condSt3;
    condFail1 = (chkOut.IntrvFlg > 0);
    condSt3 = (st3RelX > 0.0f);

    if (condFail1) {
      *mdfFlg = 0;
    } else {
      if (condSt3) {
        *mdfFlg = 0;
      } else {
        chkIn.CptX = revOut.Ct2X;
        chkIn.CptY = revOut.Ct2Y;
        chkIn.Radius = rd2;
        chkIn.Pt1X = revOut.St2X;
        chkIn.Pt1Y = revOut.St2Y;
        chkIn.Pt1Theta = revOut.St2Theta;
        chkIn.Pt2X = revOut.St3X;
        chkIn.Pt2Y = revOut.St3Y;
        chkIn.Pt2Theta = revOut.St3Theta;
        chkIn.GoDir = 1;
        chkIn.TrnDir = revOut.TrnD2;
        chkIn.CheckStep = 0.2f;
        chkIn.GoalX = input->TarX;
        chkIn.GoalY = input->TarY;
        chkIn.GoalYaw = input->TarTheta;
        chkIn.SlotP1X = input->SltPt1X;
        chkIn.SlotP1Y = input->SltPt1Y;
        chkIn.SlotP2X = input->SltPt2X;
        chkIn.SlotP2Y = input->SltPt2Y;
        chkIn.SlotP3X = input->SltPt3X;
        chkIn.SlotP3Y = input->SltPt3Y;
        chkIn.SlotP4X = input->SltPt4X;
        chkIn.SlotP4Y = input->SltPt4Y;
        chkIn.VehWidth = input->VehW;
        chkIn.VehFrontOffset = input->VehLlF + input->VehLfMin * 0.5f;
        geoCircleIntrvCheck(&chkIn, &chkOut);
        *plnChkCnt = *plnChkCnt + chkOut.CheckCount;

        if (chkOut.IntrvFlg > 0) {
          *mdfFlg = 0;
        } else {
          *mdfFlg = 1;

          Integer segNew;
          Integer idx;
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

          output->PathGoDir[0] = -1;
          output->PathGoDir[1] = -1;
          output->PathGoDir[2] = 1;

          output->PathTrnDir[0] = 0;
          output->PathTrnDir[1] = revOut.TrnD2;
          output->PathTrnDir[2] = revOut.TrnD1;

          output->PathCrv[0] = 0.0f;
          output->PathCrv[1] = (Real)revOut.TrnD2 / rd2;
          output->PathCrv[2] = (Real)revOut.TrnD1 / rd1;

          output->PathTht[0] = 0.0f;
          output->PathTht[1] = revOut.Tht2;
          output->PathTht[2] = revOut.Tht1;

          output->PathLen[0] = fabsf(st3RelX);
          output->PathLen[1] = revOut.Len2;
          output->PathLen[2] = revOut.Len1;

          output->PathStx[0] = vehX;
          output->PathStx[1] = revOut.St3X;
          output->PathStx[2] = revOut.St2X;

          output->PathSty[0] = vehY;
          output->PathSty[1] = revOut.St3Y;
          output->PathSty[2] = revOut.St2Y;

          output->PathStTh[0] = vehTheta;
          output->PathStTh[1] = revOut.St3Theta;
          output->PathStTh[2] = revOut.St2Theta;

          output->PathCC[0] = 0.0f;
          output->PathCC[1] = 0.0f;
          output->PathCC[2] = revOut.Ct2X;
          output->PathCC[3] = revOut.Ct2Y;
          output->PathCC[4] = revOut.Ct1X;
          output->PathCC[5] = revOut.Ct1Y;
        }
      }
    }
  } else {
    *mdfFlg = 0;
  }
}

/**
 * @brief 对已有VT路径执行stp1转向修正，必要时在路径头部插入三段倒车-前进过渡
 * @en_name geoStp1trnmdf
 * @cn_name stp1转向修正
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
 * @source matlab_rewrite geo_stp1trnmdf.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoStp1trnmdf(const GeoStp1trnmdfInput_t *input, GeoStp1trnmdfOutput_t *output) {
  Integer i;
  Integer plnChkCnt;

  Integer mdfFlg;
  Real vehX;
  Real vehY;
  Real vehTheta;
  Real tarX;
  Real tarY;
  Real tarTheta;
  Real kApaRmin;

  vehX = input->VehX;
  vehY = input->VehY;
  vehTheta = input->VehTheta;
  tarX = input->TarX;
  tarY = input->TarY;
  tarTheta = input->TarTheta;
  (void)tarTheta;
  plnChkCnt = 0;
  mdfFlg = 0;

  Boolean condS1;
  Boolean condS3;
  condS1 = (input->SltState == 1);
  condS3 = (input->SltState == 3);
  if (condS1) {
    kApaRmin = input->RminSuccSlot;
  } else {
    if (condS3) {
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

  Real len1In;
  Real len2In;
  Real kFacTmp;
  len1In = input->PathLen[0];
  len2In = input->PathLen[1];
  kFacTmp = geoStp1trnmdfCalcKFac(len1In, len2In);

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
  cosVeh = cosf(vehTheta);
  sinVeh = sinf(vehTheta);
  dxTar = tarX - vehX;
  dyTar = tarY - vehY;
  tarRelY = cosVeh * dyTar - sinVeh * dxTar;

  Boolean condSegEnough;
  Boolean condLeft;
  Boolean condRight;
  Boolean condTrn1Zero;
  Boolean condTrn2Neg;
  Boolean condTrn2Pos;
  Boolean condCaseA;
  Boolean condCaseB;
  Boolean condDoMdf;

  condSegEnough = (input->PathSegNum >= 3);
  condLeft = (tarRelY > 0.0f);
  condRight = (tarRelY < 0.0f);
  condTrn1Zero = (input->PathTrnDir[0] == 0);
  condTrn2Neg = (input->PathTrnDir[1] == -1);
  condTrn2Pos = (input->PathTrnDir[1] == 1);

  if (condLeft) {
    if (condTrn1Zero) {
      condCaseA = condTrn2Neg;
    } else {
      condCaseA = false;
    }
  } else {
    condCaseA = false;
  }
  if (condRight) {
    if (condTrn1Zero) {
      condCaseB = condTrn2Pos;
    } else {
      condCaseB = false;
    }
  } else {
    condCaseB = false;
  }
  if (condCaseA) {
    condDoMdf = true;
  } else {
    condDoMdf = condCaseB;
  }
  if (condSegEnough == false) {
    condDoMdf = false;
  }

  if (condDoMdf) {
    geoStp1trnmdfApplyMdf(
        input, output, kApaRmin, kRFac, vehX, vehY, vehTheta, cosVeh, sinVeh, &plnChkCnt, &mdfFlg);
  } else {
    mdfFlg = 0;
  }

  output->MdfFlg = mdfFlg;
  output->PlnChkCnt = plnChkCnt;
}
