#include "apaCoordUtils.h"
#include "apaMathUtils.h"
#include "geoVt1RFixTypes.h"
#include "geoRev2CircleLinkTypes.h"
#include <math.h>

static void geoVt1RFixCalcCandidate(const GeoVt1RFixInput_t *input,
                                    Real scale,
                                    Real dtheta,
                                    Real r,
                                    Integer sgn,
                                    Real *ptX,
                                    Real *ptY,
                                    Real *ptTheta) {
  Real theta;
  Real thetaToCenter;
  Real sgnF;
  sgnF = (Real)sgn;
  theta = apaCoordWrapToPi(dtheta * scale + input->P1Theta);
  thetaToCenter = theta - sgnF * (APA_PI * 0.5f);
  *ptX = input->CenterX + r * cosf(thetaToCenter);
  *ptY = input->CenterY + r * sinf(thetaToCenter);
  *ptTheta = theta;
}

static Boolean
geoVt1RFixQuickCheck(const VehicleCollisionCheckInput_t *tmpl, Real ptX, Real ptY, Real ptTheta) {
  VehicleCollisionCheckInput_t scratch;
  VehicleCollisionCheckOutput_t result;
  Boolean isCollision;
  scratch = *tmpl;
  scratch.CheckType = VCC_CHECK_TYPE_PLANNING;
  scratch.CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  scratch.CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  scratch.VehPosX = ptX;
  scratch.VehPosY = ptY;
  scratch.VehPosTheta = ptTheta;
  result.IsCollision = false;
  vehicleCollisionCheck(&scratch, &result);
  isCollision = result.IsCollision;
  return isCollision;
}

static Boolean geoVt1RFixArcCheck(const GeoCircleIntrvCheckInput_t *tmpl,
                                  Real cptX,
                                  Real cptY,
                                  Real radius,
                                  Real pt1X,
                                  Real pt1Y,
                                  Real pt1Theta,
                                  Real pt2X,
                                  Real pt2Y,
                                  Real pt2Theta,
                                  Integer goDir,
                                  Integer trnDir) {
  GeoCircleIntrvCheckInput_t scratch;
  GeoCircleIntrvCheckOutput_t result;
  Boolean isInterference;
  scratch = *tmpl;
  scratch.CptX = cptX;
  scratch.CptY = cptY;
  scratch.Radius = radius;
  scratch.Pt1X = pt1X;
  scratch.Pt1Y = pt1Y;
  scratch.Pt1Theta = pt1Theta;
  scratch.Pt2X = pt2X;
  scratch.Pt2Y = pt2Y;
  scratch.Pt2Theta = pt2Theta;
  scratch.GoDir = goDir;
  scratch.TrnDir = trnDir;
  result.IntrvFlg = 0;
  geoCircleIntrvCheck(&scratch, &result);
  isInterference = (result.IntrvFlg != 0);
  return isInterference;
}

/**
 * @brief VT单半径修正；UseFullSearch 启用时输出4段锚点，否则仅返回半径和弧长。
 * @en_name geoVt1RFix
 * @cn_name VT单半径修正
 * @type module
 * @param[IN] Real P0X 车端起点X
 * @param[IN] Real P0Y 车端起点Y
 * @param[IN] Real P0Theta 车端起点航向
 * @param[IN] Real P1X 弧线起点X
 * @param[IN] Real P1Y 弧线起点Y
 * @param[IN] Real P1Theta 弧线起点航向
 * @param[IN] Real CenterX 圆心X
 * @param[IN] Real CenterY 圆心Y
 * @param[IN] Integer TurnDir 转弯方向1左转负1右转
 * @param[IN] Real Curve 曲率
 * @param[IN] Real PendX 终点X
 * @param[IN] Real PendY 终点Y
 * @param[IN] Real PendTheta 终点航向
 * @param[IN] Real Rd1 双圆段1半径
 * @param[IN] Real Rd2 双圆段2半径
 * @param[IN] Boolean UseFullSearch 是否启用候选scale搜索
 * @param[IN] VehicleCollisionCheckInput_t VccTemplate 快速碰撞检查模板
 * @param[IN] GeoCircleIntrvCheckInput_t IntrvTemplate 圆弧干涉检查模板
 * @param[OUT] Integer Success 成功标志1成功0失败
 * @param[OUT] Real FixedRadius 修正后半径
 * @param[OUT] Real DeltaTheta p0/p1夹角
 * @param[OUT] Real ArcLen p0->p1弧长
 * @param[OUT] Real[4] AnchorX 4段锚点X
 * @param[OUT] Real[4] AnchorY 4段锚点Y
 * @param[OUT] Real[4] AnchorTheta 4段锚点航向
 * @param[OUT] Real[4] SegCenterX 4段圆心X
 * @param[OUT] Real[4] SegCenterY 4段圆心Y
 * @param[OUT] Real[4] SegCurve 4段曲率
 * @param[OUT] Integer[4] SegGoDir 4段行驶方向
 * @param[OUT] Integer[4] SegTurnDir 4段转弯方向
 * @param[OUT] Real[4] SegDTheta 4段角度变化
 * @param[OUT] Real[4] SegLength 4段长度
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 路径规划
 * @source matlab_rewrite geo_vt_1R_fix.m
 * @version 2.1
 * @date 2026-05-01
 * @author aide
 */
void geoVt1RFix(const GeoVt1RFixInput_t *input, GeoVt1RFixOutput_t *output) {
  Real curve;
  Real absCurve;
  Real r;
  Integer sgn;
  Real dtheta;
  Real ds;
  Integer i;
  Real minDist;
  Real epsilonDist;
  Boolean arcLongEnough;
  Boolean doBasicSuccess;
  Boolean doFullSearch;

  Real scale1;
  Real scale2;
  Real scale;
  Real scaleCandidates[20];
  Integer numCandidates;
  Integer iter;
  Integer goDir;
  Boolean condLoop;
  Boolean foundSolution;

  minDist = 0.4f;
  epsilonDist = 0.05f;

  curve = input->Curve;
  absCurve = apaMathAbs(curve);
  if (absCurve < GEO_EPSILON) {
    r = 1e6f;
  } else {
    r = 1.0f / absCurve;
  }

  if (input->TurnDir > 0) {
    sgn = 1;
  } else {
    sgn = -1;
  }

  dtheta = apaCoordWrapToPi(input->P0Theta - input->P1Theta);
  ds = apaMathAbs(dtheta) * r;

  output->Success = 0;
  output->FixedRadius = r;
  output->DeltaTheta = dtheta;
  output->ArcLen = ds;
  for (i = 0; i < 4; i = i + 1) {
    output->AnchorX[i] = 0.0f;
    output->AnchorY[i] = 0.0f;
    output->AnchorTheta[i] = 0.0f;
    output->SegCenterX[i] = 0.0f;
    output->SegCenterY[i] = 0.0f;
    output->SegCurve[i] = 0.0f;
    output->SegGoDir[i] = 0;
    output->SegTurnDir[i] = 0;
    output->SegDTheta[i] = 0.0f;
    output->SegLength[i] = 0.0f;
  }

  arcLongEnough = (ds >= minDist);
  doBasicSuccess = arcLongEnough && (!input->UseFullSearch);
  doFullSearch = arcLongEnough && input->UseFullSearch;

  if (doBasicSuccess) {
    output->Success = 1;
  }

  if (doFullSearch) {

    goDir = -1;
    scale1 = 1.0f;
    scale2 = minDist / ds;
    scale = (scale1 + scale2) * 0.5f;
    numCandidates = 0;
    for (i = 0; i < 20; i = i + 1) {
      scaleCandidates[i] = 0.0f;
    }

    iter = 0;
    condLoop = (ds * (scale - scale2) > epsilonDist) && (iter < 64);
    while (condLoop) {
      Real ptX;
      Real ptY;
      Real ptTheta;
      GeoRev2CircleLinkInput_t linkIn;
      GeoRev2CircleLinkOutput_t linkOut;
      Boolean collision;

      geoVt1RFixCalcCandidate(input, scale, dtheta, r, sgn, &ptX, &ptY, &ptTheta);

      linkIn = (GeoRev2CircleLinkInput_t){0};
      linkIn.Pt1X = ptX;
      linkIn.Pt1Y = ptY;
      linkIn.Pt1Theta = ptTheta;
      linkIn.Pt2X = input->PendX;
      linkIn.Pt2Y = input->PendY;
      linkIn.Pt2Theta = input->PendTheta;
      linkIn.Dir = goDir;
      linkIn.R1 = input->Rd1;
      linkIn.R2 = input->Rd2;
      linkOut = (GeoRev2CircleLinkOutput_t){0};
      geoRev2CircleLink(&linkIn, &linkOut);

      collision =
          geoVt1RFixQuickCheck(&input->VccTemplate, linkOut.St2X, linkOut.St2Y, linkOut.St2Theta);
      if (collision) {
        scale2 = scale;
        scale = (scale1 + scale2) * 0.5f;
      } else {
        if (numCandidates < 20) {
          scaleCandidates[numCandidates] = scale;
          numCandidates = numCandidates + 1;
        }
        scale1 = scale;
        scale = (scale1 + scale2) * 0.5f;
      }
      iter = iter + 1;
      condLoop = (ds * (scale - scale2) > epsilonDist) && (iter < 64);
    }

    foundSolution = false;
    i = numCandidates - 1;
    while ((i >= 0) && !foundSolution) {
      Real candScale;
      Real ptX;
      Real ptY;
      Real ptTheta;
      GeoRev2CircleLinkInput_t linkIn;
      GeoRev2CircleLinkOutput_t linkOut;
      Boolean intrv1;
      Boolean intrv2;

      candScale = scaleCandidates[numCandidates - 1];
      geoVt1RFixCalcCandidate(input, candScale, dtheta, r, sgn, &ptX, &ptY, &ptTheta);

      linkIn = (GeoRev2CircleLinkInput_t){0};
      linkIn.Pt1X = ptX;
      linkIn.Pt1Y = ptY;
      linkIn.Pt1Theta = ptTheta;
      linkIn.Pt2X = input->PendX;
      linkIn.Pt2Y = input->PendY;
      linkIn.Pt2Theta = input->PendTheta;
      linkIn.Dir = goDir;
      linkIn.R1 = input->Rd1;
      linkIn.R2 = input->Rd2;
      linkOut = (GeoRev2CircleLinkOutput_t){0};
      geoRev2CircleLink(&linkIn, &linkOut);

      intrv1 = geoVt1RFixArcCheck(&input->IntrvTemplate,
                                  linkOut.Ct1X,
                                  linkOut.Ct1Y,
                                  input->Rd1,
                                  ptX,
                                  ptY,
                                  ptTheta,
                                  linkOut.St2X,
                                  linkOut.St2Y,
                                  linkOut.St2Theta,
                                  goDir,
                                  linkOut.TrnD1);
      intrv2 = geoVt1RFixArcCheck(&input->IntrvTemplate,
                                  linkOut.Ct2X,
                                  linkOut.Ct2Y,
                                  input->Rd2,
                                  linkOut.St2X,
                                  linkOut.St2Y,
                                  linkOut.St2Theta,
                                  linkOut.St3X,
                                  linkOut.St3Y,
                                  linkOut.St3Theta,
                                  -goDir,
                                  linkOut.TrnD2);

      if (!intrv1 && !intrv2) {

        Real dTheta1;
        Real dTheta2;
        Real dTheta3;
        Real dTheta4;
        Real length1;
        Real length2;
        Real length3;
        Real length4;
        Real diffX;
        Real diffY;
        Real curve2;
        Real curve3;
        Integer goDir1;

        dTheta1 = 0.0f;
        dTheta2 = apaCoordWrapToPi(linkOut.St2Theta - linkOut.St3Theta);
        dTheta3 = apaCoordWrapToPi(ptTheta - linkOut.St2Theta);
        dTheta4 = apaCoordWrapToPi(input->P1Theta - ptTheta);

        diffX = linkOut.St3X - input->PendX;
        diffY = linkOut.St3Y - input->PendY;
        length1 = sqrtf(diffX * diffX + diffY * diffY);
        length2 = linkOut.Len2;
        length3 = linkOut.Len1;
        length4 = apaMathAbs(dTheta4) * r;

        if (input->Rd2 > GEO_EPSILON) {
          curve2 = (Real)linkOut.TrnD2 / input->Rd2;
        } else {
          curve2 = 0.0f;
        }
        if (input->Rd1 > GEO_EPSILON) {
          curve3 = (Real)linkOut.TrnD1 / input->Rd1;
        } else {
          curve3 = 0.0f;
        }
        goDir1 = apaMathSign(linkOut.St3rX);

        output->AnchorX[0] = input->PendX;
        output->AnchorY[0] = input->PendY;
        output->AnchorTheta[0] = input->PendTheta;
        output->AnchorX[1] = linkOut.St3X;
        output->AnchorY[1] = linkOut.St3Y;
        output->AnchorTheta[1] = linkOut.St3Theta;
        output->AnchorX[2] = linkOut.St2X;
        output->AnchorY[2] = linkOut.St2Y;
        output->AnchorTheta[2] = linkOut.St2Theta;
        output->AnchorX[3] = ptX;
        output->AnchorY[3] = ptY;
        output->AnchorTheta[3] = ptTheta;
        output->SegCenterX[0] = 0.0f;
        output->SegCenterY[0] = 0.0f;
        output->SegCenterX[1] = linkOut.Ct2X;
        output->SegCenterY[1] = linkOut.Ct2Y;
        output->SegCenterX[2] = linkOut.Ct1X;
        output->SegCenterY[2] = linkOut.Ct1Y;
        output->SegCenterX[3] = input->CenterX;
        output->SegCenterY[3] = input->CenterY;
        output->SegCurve[0] = 0.0f;
        output->SegCurve[1] = curve2;
        output->SegCurve[2] = curve3;
        output->SegCurve[3] = curve;
        output->SegGoDir[0] = goDir1;
        output->SegGoDir[1] = goDir;
        output->SegGoDir[2] = -goDir;
        output->SegGoDir[3] = goDir;
        output->SegTurnDir[0] = 0;
        output->SegTurnDir[1] = linkOut.TrnD2;
        output->SegTurnDir[2] = linkOut.TrnD1;
        output->SegTurnDir[3] = input->TurnDir;
        output->SegDTheta[0] = dTheta1;
        output->SegDTheta[1] = dTheta2;
        output->SegDTheta[2] = dTheta3;
        output->SegDTheta[3] = dTheta4;
        output->SegLength[0] = length1;
        output->SegLength[1] = length2;
        output->SegLength[2] = length3;
        output->SegLength[3] = length4;
        output->Success = 1;
        foundSolution = true;
      }
      i = i - 1;
    }
  }
}
