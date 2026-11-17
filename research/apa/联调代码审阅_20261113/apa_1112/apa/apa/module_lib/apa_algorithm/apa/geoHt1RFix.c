#include "apaCoordUtils.h"
#include "geoHt1RFixTypes.h"
#include "geo2CircleLinkHTTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include "geoDivideTypes.h"
#include <math.h>

static void geoHt1RFixFillCircleInput(const GeoHt1RFixInput_t *input,
                                      Real centerX,
                                      Real centerY,
                                      Real radius,
                                      Real startX,
                                      Real startY,
                                      Real startTheta,
                                      Real endX,
                                      Real endY,
                                      Real endTheta,
                                      Integer goDir,
                                      Integer turnDir,
                                      GeoCircleIntrvCheckInput_t *circleInput) {
  Integer idx;
  Integer count;

  *circleInput = (GeoCircleIntrvCheckInput_t){0};
  circleInput->UsePlanningCheck = true;
  circleInput->SlotClass = VCC_SLOT_CLASS_PARALLEL;
  circleInput->DisableSlotBoundaryFallback = true;
  circleInput->CptX = centerX;
  circleInput->CptY = centerY;
  circleInput->Radius = radius;
  circleInput->Pt1X = startX;
  circleInput->Pt1Y = startY;
  circleInput->Pt1Theta = startTheta;
  circleInput->Pt2X = endX;
  circleInput->Pt2Y = endY;
  circleInput->Pt2Theta = endTheta;
  circleInput->GoDir = goDir;
  circleInput->TrnDir = turnDir;
  circleInput->CheckStep = 0.2f;
  circleInput->GoalX = input->PendX;
  circleInput->GoalY = input->PendY;
  circleInput->GoalYaw = input->PendTheta;
  circleInput->SlotP1X = input->SlotP1X;
  circleInput->SlotP1Y = input->SlotP1Y;
  circleInput->SlotP2X = input->SlotP2X;
  circleInput->SlotP2Y = input->SlotP2Y;
  circleInput->SlotP3X = input->SlotP3X;
  circleInput->SlotP3Y = input->SlotP3Y;
  circleInput->SlotP4X = input->SlotP4X;
  circleInput->SlotP4Y = input->SlotP4Y;
  circleInput->VehWidth = input->VehWidth;
  circleInput->VehFrontOffset = input->VehFrontOffset;
  circleInput->VehRearOffset = input->VehRearOffset;

  count = input->BoundaryLineNum;
  if (count > GEO_BOUNDARY_LINE_MAX) {
    count = GEO_BOUNDARY_LINE_MAX;
  }
  circleInput->BoundaryLineNum = count;
  idx = 0;
  while (idx < count) {
    circleInput->BoundaryLineX1[idx] = input->BoundaryLineX1[idx];
    circleInput->BoundaryLineY1[idx] = input->BoundaryLineY1[idx];
    circleInput->BoundaryLineX2[idx] = input->BoundaryLineX2[idx];
    circleInput->BoundaryLineY2[idx] = input->BoundaryLineY2[idx];
    idx = idx + 1;
  }

  count = input->VirtualBoundaryLineNum;
  if (count > VCC_MAX_VIRTUAL_LINES) {
    count = VCC_MAX_VIRTUAL_LINES;
  }
  circleInput->VirtualBoundaryLineNum = count;
  idx = 0;
  while (idx < count) {
    circleInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryLineX1[idx];
    circleInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryLineY1[idx];
    circleInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryLineX2[idx];
    circleInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryLineY2[idx];
    idx = idx + 1;
  }

  count = input->BoundarySepPtNum;
  if (count > VCC_MAX_SEPARATE_PTS) {
    count = VCC_MAX_SEPARATE_PTS;
  }
  circleInput->BoundarySepPtNum = count;
  idx = 0;
  while (idx < count) {
    circleInput->BoundarySepPtX[idx] = input->BoundarySepPtX[idx];
    circleInput->BoundarySepPtY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }

  count = input->StopperPtNum;
  if (count > VCC_MAX_SEPARATE_PTS) {
    count = VCC_MAX_SEPARATE_PTS;
  }
  circleInput->StopperPtNum = count;
  idx = 0;
  while (idx < count) {
    circleInput->StopperPtX[idx] = input->StopperPtX[idx];
    circleInput->StopperPtY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
}

static void geoHt1RFixTryScale(const GeoHt1RFixInput_t *input,
                               GeoHt1RFixOutput_t *output,
                               Real r,
                               Real sgn,
                               Real dtheta,
                               Real scale,
                               Real *scaleHi,
                               Real *scaleLo) {
  Real theta;
  Real thetaToCenter;
  Real ptX;
  Real ptY;
  Real ptTheta;
  Real revCenterX;
  Real revCenterY;
  Real revCurve;
  Integer revTurnDir;
  Real revLength;
  Real revTheta0;
  Real revTheta1;
  Real revT1ToC;
  Real revAnchorX;
  Real revAnchorY;
  Real revAnchorTheta;

  theta = apaCoordWrapToPi(dtheta * scale + input->P0Theta);
  thetaToCenter = theta - sgn * APA_PI / 2.0f;
  ptX = input->CenterX + r * cosf(thetaToCenter);
  ptY = input->CenterY + r * sinf(thetaToCenter);
  ptTheta = theta;

  revCurve = -input->Curve;
  revTurnDir = -input->TurnDir;
  revLength = 1.0f;

  {
    Real revSgn;
    Real revR;
    Real absRevCurve;

    if (revTurnDir > 0) {
      revSgn = 1.0f;
    } else {
      revSgn = -1.0f;
    }
    if (revCurve < 0.0f) {
      absRevCurve = -revCurve;
    } else {
      absRevCurve = revCurve;
    }
    revR = geoDivideScalar(1.0f, absRevCurve);

    revTheta0 = ptTheta;
    revTheta1 = apaCoordWrapToPi(revTheta0 - revSgn * revLength / revR);
    revT1ToC = apaCoordWrapToPi(revTheta1 - revSgn * APA_PI / 2.0f);

    revCenterX = ptX + revSgn * revR * (-sinf(revTheta0));
    revCenterY = ptY + revSgn * revR * cosf(revTheta0);

    {
      GeoCircleIntrvCheckInput_t reverseCheckInput;
      GeoCircleIntrvCheckOutput_t reverseCheckOutput;
      Real candidateX;
      Real candidateY;

      candidateX = revCenterX + revR * cosf(revT1ToC);
      candidateY = revCenterY + revR * sinf(revT1ToC);
      geoHt1RFixFillCircleInput(input,
                                revCenterX,
                                revCenterY,
                                revR,
                                ptX,
                                ptY,
                                ptTheta,
                                candidateX,
                                candidateY,
                                revTheta1,
                                -1,
                                revTurnDir,
                                &reverseCheckInput);
      reverseCheckOutput = (GeoCircleIntrvCheckOutput_t){0};
      geoCircleIntrvCheckWithScene(&reverseCheckInput, &input->SceneQuery, &reverseCheckOutput);
      if (reverseCheckOutput.IntrvFlg == 0) {
        revAnchorX = candidateX;
        revAnchorY = candidateY;
        revAnchorTheta = revTheta1;
        revLength = 1.0f;
      } else {
        revAnchorX = reverseCheckOutput.AvlPosX;
        revAnchorY = reverseCheckOutput.AvlPosY;
        revAnchorTheta = reverseCheckOutput.AvlPosTheta;
        revLength = reverseCheckOutput.SafeLen;
      }
    }
  }

  {
    Geo2CircleLinkHTInput_t linkIn;
    Geo2CircleLinkHTOutput_t linkOut;

    linkIn.Pt1X = revAnchorX;
    linkIn.Pt1Y = revAnchorY;
    linkIn.Pt1Theta = revAnchorTheta;
    linkIn.Pt2X = input->PendX;
    linkIn.Pt2Y = input->PendY;
    linkIn.Pt2Theta = input->PendTheta;
    linkIn.Dir = input->TurnDir;
    linkIn.R1 = input->RdLs1;
    linkIn.R2 = input->RdLs2;

    geo2CircleLinkHT(&linkIn, &linkOut);

    {
      Real st3rX;
      Real st3rY;
      Real normSt3r;
      Integer goDir1;
      Real dth2;
      Real dth3;
      Real dth4;
      Real dth5;
      Real len5;

      {
        Real st3Dx;
        Real st3Dy;
        Real pendCos;
        Real pendSin;

        st3Dx = linkOut.St3X - input->PendX;
        st3Dy = linkOut.St3Y - input->PendY;
        pendCos = cosf(input->PendTheta);
        pendSin = sinf(input->PendTheta);
        st3rX = (pendCos * st3Dx) + (pendSin * st3Dy);
        st3rY = (pendCos * st3Dy) - (pendSin * st3Dx);
      }
      normSt3r = sqrtf((st3rX * st3rX) + (st3rY * st3rY));
      if (st3rX > 0.0f) {
        goDir1 = 1;
      } else if (st3rX < 0.0f) {
        goDir1 = -1;
      } else {
        goDir1 = 0;
      }

      dth2 = apaCoordWrapToPi(revAnchorTheta - linkOut.St2Theta);
      dth3 = apaCoordWrapToPi(linkOut.St2Theta - linkOut.St3Theta);
      dth4 = apaCoordWrapToPi(ptTheta - revAnchorTheta);
      dth5 = apaCoordWrapToPi(input->P0Theta - ptTheta);

      {
        Real absDth5;
        if (dth5 < 0.0f) {
          absDth5 = -dth5;
        } else {
          absDth5 = dth5;
        }
        len5 = absDth5 * r;
      }

      {
        GeoCircleIntrvCheckInput_t arcCheckInput1;
        GeoCircleIntrvCheckInput_t arcCheckInput2;
        GeoCircleIntrvCheckOutput_t arcCheckOutput1;
        GeoCircleIntrvCheckOutput_t arcCheckOutput2;

        geoHt1RFixFillCircleInput(input,
                                  linkOut.Ct1X,
                                  linkOut.Ct1Y,
                                  input->RdLs1,
                                  revAnchorX,
                                  revAnchorY,
                                  revAnchorTheta,
                                  linkOut.St2X,
                                  linkOut.St2Y,
                                  linkOut.St2Theta,
                                  1,
                                  linkOut.TrnD1,
                                  &arcCheckInput1);
        arcCheckOutput1 = (GeoCircleIntrvCheckOutput_t){0};
        geoCircleIntrvCheckWithScene(&arcCheckInput1, &input->SceneQuery, &arcCheckOutput1);
        geoHt1RFixFillCircleInput(input,
                                  linkOut.Ct2X,
                                  linkOut.Ct2Y,
                                  input->RdLs2,
                                  linkOut.St2X,
                                  linkOut.St2Y,
                                  linkOut.St2Theta,
                                  linkOut.St3X,
                                  linkOut.St3Y,
                                  linkOut.St3Theta,
                                  1,
                                  linkOut.TrnD2,
                                  &arcCheckInput2);
        arcCheckOutput2 = (GeoCircleIntrvCheckOutput_t){0};
        geoCircleIntrvCheckWithScene(&arcCheckInput2, &input->SceneQuery, &arcCheckOutput2);

        if ((arcCheckOutput1.IntrvFlg == 0) && (arcCheckOutput2.IntrvFlg == 0)) {
          output->AnchorX[0] = input->PendX;
          output->AnchorY[0] = input->PendY;
          output->AnchorTheta[0] = input->PendTheta;
          output->AnchorX[1] = linkOut.St3X;
          output->AnchorY[1] = linkOut.St3Y;
          output->AnchorTheta[1] = linkOut.St3Theta;
          output->AnchorX[2] = linkOut.St2X;
          output->AnchorY[2] = linkOut.St2Y;
          output->AnchorTheta[2] = linkOut.St2Theta;
          output->AnchorX[3] = revAnchorX;
          output->AnchorY[3] = revAnchorY;
          output->AnchorTheta[3] = revAnchorTheta;
          output->AnchorX[4] = ptX;
          output->AnchorY[4] = ptY;
          output->AnchorTheta[4] = ptTheta;

          output->CtrX[0] = 0.0f;
          output->CtrY[0] = 0.0f;
          output->CtrX[1] = linkOut.Ct2X;
          output->CtrY[1] = linkOut.Ct2Y;
          output->CtrX[2] = linkOut.Ct1X;
          output->CtrY[2] = linkOut.Ct1Y;
          output->CtrX[3] = revCenterX;
          output->CtrY[3] = revCenterY;
          output->CtrX[4] = input->CenterX;
          output->CtrY[4] = input->CenterY;

          output->Curves[0] = 0.0f;
          output->Curves[1] = geoDivideScalar((Real)linkOut.TrnD2, input->RdLs2);
          output->Curves[2] = geoDivideScalar((Real)linkOut.TrnD1, input->RdLs1);
          output->Curves[3] = revCurve;
          output->Curves[4] = input->Curve;

          output->GoDirs[0] = goDir1;
          output->GoDirs[1] = -1;
          output->GoDirs[2] = -1;
          output->GoDirs[3] = 1;
          output->GoDirs[4] = -1;

          output->TurnDirs[0] = 0;
          output->TurnDirs[1] = linkOut.TrnD2;
          output->TurnDirs[2] = linkOut.TrnD1;
          output->TurnDirs[3] = revTurnDir;
          output->TurnDirs[4] = input->TurnDir;

          output->DThetas[0] = 0.0f;
          output->DThetas[1] = dth3;
          output->DThetas[2] = dth2;
          output->DThetas[3] = dth4;
          output->DThetas[4] = dth5;

          output->Lengths[0] = normSt3r;
          output->Lengths[1] = linkOut.Len2;
          output->Lengths[2] = linkOut.Len1;
          output->Lengths[3] = revLength;
          output->Lengths[4] = len5;

          output->Success = 1;
          output->SegNum = 5;
          *scaleHi = scale;
        } else {
          *scaleLo = scale;
        }
      }
    }
  }
}

/**
 * @brief HT单弯修正 — 二分搜索最优弧段缩放，生成5段路径
 *
 * 对应 MATLAB geo_ht_1R_fix.m。
 * 算法：在弧线上做二分搜索，对每个候选点生成"倒车弧→双圆连接→直线"的5段路径，
 * 检测碰撞并找到最优可行解。
 *
 * @en_name geoHt1RFix
 * @cn_name HT单弯修正
 * @type module
 * @param[IN] Real P0X 当前位姿X
 * @param[IN] Real P0Y 当前位姿Y
 * @param[IN] Real P0Theta 当前位姿航向
 * @param[IN] Real P1Theta 目标位姿航向
 * @param[IN] Real CenterX 圆弧中心X
 * @param[IN] Real CenterY 圆弧中心Y
 * @param[IN] Real Curve 弧线曲率
 * @param[IN] Integer TurnDir 转弯方向
 * @param[IN] Real PendX 目标终点X
 * @param[IN] Real PendY 目标终点Y
 * @param[IN] Real PendTheta 目标终点航向
 * @param[IN] Real RdLs1 双圆连接半径1
 * @param[IN] Real RdLs2 双圆连接半径2
 * @param[OUT] Integer Success 是否成功
 * @param[OUT] Integer SegNum 路径段数
 * @param[OUT] Real[5] AnchorX 锚点X数组
 * @param[OUT] Real[5] AnchorY 锚点Y数组
 * @param[OUT] Real[5] AnchorTheta 锚点航向数组
 * @param[OUT] Real[5] CtrX 圆心X数组
 * @param[OUT] Real[5] CtrY 圆心Y数组
 * @param[OUT] Real[5] Curves 曲率数组
 * @param[OUT] Integer[5] GoDirs 行驶方向数组
 * @param[OUT] Integer[5] TurnDirs 转弯方向数组
 * @param[OUT] Real[5] DThetas 弧段角度数组
 * @param[OUT] Real[5] Lengths 段长度数组
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 几何路径规划
 * @source matlab_rewrite geo_ht_1R_fix.m
 * @version 1.1
 * @date 2026-03-16
 * @author aide
 */
void geoHt1RFix(const GeoHt1RFixInput_t *input, GeoHt1RFixOutput_t *output) {

  Real r;
  Real sgn;
  Real dtheta;
  Real ds;
  Real minDist;
  Real epsDist;
  Real scaleLo;
  Real scaleHi;
  Real scale;
  Integer ii;
  Integer maxIter;
  Boolean condArcLongEnough;

  output->Success = 0;
  output->SegNum = 0;
  ii = 0;
  while (ii < HT_1R_SEG_MAX) {
    output->AnchorX[ii] = 0.0f;
    output->AnchorY[ii] = 0.0f;
    output->AnchorTheta[ii] = 0.0f;
    output->CtrX[ii] = 0.0f;
    output->CtrY[ii] = 0.0f;
    output->Curves[ii] = 0.0f;
    output->GoDirs[ii] = 0;
    output->TurnDirs[ii] = 0;
    output->DThetas[ii] = 0.0f;
    output->Lengths[ii] = 0.0f;
    ii = ii + 1;
  }

  {
    Real absCurve;

    if (input->Curve < 0.0f) {
      absCurve = -input->Curve;
    } else {
      absCurve = input->Curve;
    }
    r = geoDivideScalar(1.0f, absCurve);
  }
  if (input->TurnDir > 0) {
    sgn = 1.0f;
  } else {
    sgn = -1.0f;
  }

  dtheta = apaCoordWrapToPi(input->P1Theta - input->P0Theta);
  {
    Real absDth;

    if (dtheta < 0.0f) {
      absDth = -dtheta;
    } else {
      absDth = dtheta;
    }
    ds = absDth * r;
  }

  minDist = 0.4f;
  epsDist = 0.05f;

  condArcLongEnough = (ds >= minDist);
  if (condArcLongEnough) {

    Boolean condLoop;

    scaleHi = 1.0f;
    scaleLo = minDist / ds;
    scale = (scaleHi + scaleLo) / 2.0f;
    maxIter = 20;

    ii = 0;
    condLoop = (ii < maxIter) && ((ds * (scale - scaleLo)) > epsDist);
    while (condLoop) {
      geoHt1RFixTryScale(input, output, r, sgn, dtheta, scale, &scaleHi, &scaleLo);

      scale = (scaleHi + scaleLo) / 2.0f;
      ii = ii + 1;
      condLoop = (ii < maxIter) && ((ds * (scale - scaleLo)) > epsDist);
    }
  }
}
