#include "apaCoordUtils.h"
#include "geoCircle2lineRTypes.h"
#include "geoDivideTypes.h"
#include <math.h>

static void geoCircle2lineRCalcPtsAndArc(const GeoCircle2lineRInput_t *input,
                                         GeoCircle2lineROutput_t *output,
                                         Boolean condRight) {
  Real R;
  R = input->Radius;

  Real trnF;
  trnF = (Real)output->TrnDir;

  Real angSt;
  Real angEd;
  Real negTrnF;
  negTrnF = -trnF;
  Real halfPiTrn;
  halfPiTrn = negTrnF * APA_HALF_PI;

  angSt = input->VstTheta + halfPiTrn;
  Real cosAngSt;
  Real sinAngSt;
  cosAngSt = cosf(angSt);
  sinAngSt = sinf(angSt);
  Real rCosSt;
  Real rSinSt;
  rCosSt = R * cosAngSt;
  rSinSt = R * sinAngSt;
  output->StPtX = output->CptX + rCosSt;
  output->StPtY = output->CptY + rSinSt;
  output->StPtTheta = input->VstTheta;

  angEd = input->VtgtTheta + halfPiTrn;
  Real cosAngEd;
  Real sinAngEd;
  cosAngEd = cosf(angEd);
  sinAngEd = sinf(angEd);
  Real rCosEd;
  Real rSinEd;
  rCosEd = R * cosAngEd;
  rSinEd = R * sinAngEd;
  output->EdPtX = output->CptX + rCosEd;
  output->EdPtY = output->CptY + rSinEd;
  output->EdPtTheta = input->VtgtTheta;

  Real edDx;
  Real edDy;
  Real cosTgt;
  Real sinTgt;
  Real edPtrThetaRaw;
  edDx = output->EdPtX - input->VtgtX;
  edDy = output->EdPtY - input->VtgtY;
  cosTgt = cosf(input->VtgtTheta);
  sinTgt = sinf(input->VtgtTheta);
  output->EdPtrX = cosTgt * edDx + sinTgt * edDy;
  output->EdPtrY = cosTgt * edDy - sinTgt * edDx;
  edPtrThetaRaw = output->EdPtTheta - input->VtgtTheta;
  output->EdPtrTheta = apaCoordWrapToPi(edPtrThetaRaw);

  Real arcAngle;

  if (condRight) {
    Boolean condDirFwd;
    condDirFwd = (input->Dir > 0);
    if (condDirFwd) {
      Boolean condWrap;
      condWrap = (input->VtgtTheta > input->VstTheta);
      if (condWrap) {
        Real vstP2pi;
        vstP2pi = input->VstTheta + APA_TWO_PI;
        arcAngle = vstP2pi - input->VtgtTheta;
      } else {
        arcAngle = input->VstTheta - input->VtgtTheta;
      }
    } else {
      Boolean condWrap2;
      condWrap2 = (input->VtgtTheta < input->VstTheta);
      if (condWrap2) {
        Real vtgtP2pi;
        vtgtP2pi = input->VtgtTheta + APA_TWO_PI;
        arcAngle = vtgtP2pi - input->VstTheta;
      } else {
        arcAngle = input->VtgtTheta - input->VstTheta;
      }
    }
  } else {
    Boolean condDirFwd;
    condDirFwd = (input->Dir > 0);
    if (condDirFwd) {
      Boolean condWrap;
      condWrap = (input->VtgtTheta < input->VstTheta);
      if (condWrap) {
        Real vtgtP2pi;
        vtgtP2pi = input->VtgtTheta + APA_TWO_PI;
        arcAngle = vtgtP2pi - input->VstTheta;
      } else {
        arcAngle = input->VtgtTheta - input->VstTheta;
      }
    } else {
      Boolean condWrap2;
      condWrap2 = (input->VtgtTheta > input->VstTheta);
      if (condWrap2) {
        Real vstP2pi;
        vstP2pi = input->VstTheta + APA_TWO_PI;
        arcAngle = vstP2pi - input->VtgtTheta;
      } else {
        arcAngle = input->VstTheta - input->VtgtTheta;
      }
    }
  }

  Real absAngle;
  absAngle = fabsf(arcAngle);

  output->ArcAngle = arcAngle;
  output->ArcLen = absAngle * R;
}

/**
 * @brief 指定半径计算与起点和目标点方向直线同时相切的圆弧参数，含圆心起止点弧长等
 * @en_name geoCircle2lineR
 * @cn_name 指定半径双切线圆弧
 * @type module
 * @param[IN] Real VstX 起点X
 * @param[IN] Real VstY 起点Y
 * @param[IN] Real VstTheta 起点航向角
 * @param[IN] Real VtgtX 目标点X
 * @param[IN] Real VtgtY 目标点Y
 * @param[IN] Real VtgtTheta 目标点航向角
 * @param[IN] Integer Ptyp 停车位类型编号
 * @param[IN] Integer Dir 行驶方向1前进2后退
 * @param[IN] Real Radius 指定半径
 * @param[OUT] Real CptX 圆心X
 * @param[OUT] Real CptY 圆心Y
 * @param[OUT] Real StPtX 圆弧起点X
 * @param[OUT] Real StPtY 圆弧起点Y
 * @param[OUT] Real StPtTheta 圆弧起点航向角
 * @param[OUT] Real EdPtX 圆弧终点X
 * @param[OUT] Real EdPtY 圆弧终点Y
 * @param[OUT] Real EdPtTheta 圆弧终点航向角
 * @param[OUT] Real EdPtrX 终点在目标位姿坐标系X
 * @param[OUT] Real EdPtrY 终点在目标位姿坐标系Y
 * @param[OUT] Real EdPtrTheta 终点在目标位姿坐标系航向
 * @param[OUT] Integer TrnDir 转弯方向1左转负1右转
 * @param[OUT] Integer GoDir 行进方向1同向负1反向
 * @param[OUT] Real ArcLen 圆弧长度
 * @param[OUT] Real ArcAngle 圆弧对应圆心角
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 几何原语
 * @source matlab_rewrite geo_circle_2line_R.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoCircle2lineR(const GeoCircle2lineRInput_t *input, GeoCircle2lineROutput_t *output) {

  Real vstX;
  Real vstY;
  Real vstTheta;
  Real vtgtX;
  Real vtgtY;
  Real vtgtTheta;
  Integer ptyp;
  Real R;

  vstX = input->VstX;
  vstY = input->VstY;
  vstTheta = input->VstTheta;
  vtgtX = input->VtgtX;
  vtgtY = input->VtgtY;
  vtgtTheta = input->VtgtTheta;
  ptyp = input->Ptyp;
  R = input->Radius;

  Real cosVtgt;
  Real sinVtgt;
  cosVtgt = cosf(vtgtTheta);
  sinVtgt = sinf(vtgtTheta);

  Real diffAx;
  Real diffAy;
  diffAx = vstX - vtgtX;
  diffAy = vstY - vtgtY;

  Real ptRy;
  Real ptRtheta;
  Real ptRthetaRaw;
  Real tA1;
  Real tA2;
  tA1 = cosVtgt * diffAy;
  tA2 = sinVtgt * diffAx;
  ptRy = tA1 - tA2;
  ptRthetaRaw = vstTheta - vtgtTheta;
  ptRtheta = apaCoordWrapToPi(ptRthetaRaw);

  Real prod;
  Integer goDir;
  prod = ptRy * ptRtheta;
  Boolean condNeg;
  condNeg = (prod < 0.0f);
  if (condNeg) {
    goDir = 1;
  } else {
    goDir = -1;
  }

  Real sinTgt;
  Real cosTgt;
  sinTgt = sinf(vtgtTheta);
  cosTgt = cosf(vtgtTheta);

  Real aa1;
  Real bb1;
  Real cc1;
  Real p1;
  Real p2;
  aa1 = sinTgt;
  bb1 = -cosTgt;
  p1 = vtgtX + cosTgt;
  p2 = vtgtY + sinTgt;
  Real p1v;
  Real p2v;
  p1v = p1 * vtgtY;
  p2v = vtgtX * p2;
  cc1 = p1v - p2v;

  Real sinSt;
  Real cosSt;
  sinSt = sinf(vstTheta);
  cosSt = cosf(vstTheta);

  Real aa2;
  Real bb2;
  Real cc2;
  Real q1;
  Real q2;
  aa2 = sinSt;
  bb2 = -cosSt;
  q1 = vstX + cosSt;
  q2 = vstY + sinSt;
  Real q1v;
  Real q2v;
  q1v = q1 * vstY;
  q2v = vstX * q2;
  cc2 = q1v - q2v;

  Integer trnDir;
  Boolean condRight;
  condRight = (ptyp >= 31);

  Real cptX;
  Real cptY;
  GeoDivideInput_t divideXInput;
  GeoDivideInput_t divideYInput;
  GeoDivideOutput_t divideXOutput;
  GeoDivideOutput_t divideYOutput;

  if (condRight) {
    trnDir = -1;
    cc1 = cc1 - R;
    cc2 = cc2 - R;
  } else {
    trnDir = 1;
    cc1 = cc1 + R;
    cc2 = cc2 + R;
  }

  Real dd;
  Real dA;
  Real dB;
  dA = aa1 * bb2;
  dB = aa2 * bb1;
  dd = dA - dB;

  Real numCx1;
  Real numCx2;
  Real numCx;
  Real numCy1;
  Real numCy2;
  Real numCy;
  numCx1 = bb1 * cc2;
  numCx2 = bb2 * cc1;
  numCx = numCx1 - numCx2;
  numCy1 = aa2 * cc1;
  numCy2 = aa1 * cc2;
  numCy = numCy1 - numCy2;

  divideXInput.Numerator = numCx;
  divideXInput.Denominator = dd;
  divideYInput.Numerator = numCy;
  divideYInput.Denominator = dd;
  divideXOutput = (GeoDivideOutput_t){0};
  divideYOutput = (GeoDivideOutput_t){0};
  geoDivide(&divideXInput, &divideXOutput);
  geoDivide(&divideYInput, &divideYOutput);
  cptX = divideXOutput.Result;
  cptY = divideYOutput.Result;

  output->CptX = cptX;
  output->CptY = cptY;
  output->TrnDir = trnDir;
  output->GoDir = goDir;

  geoCircle2lineRCalcPtsAndArc(input, output, condRight);
}
