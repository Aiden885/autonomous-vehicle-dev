#include "geoCircle1pt1lineTypes.h"
#include <math.h>

static void geoCircle1pt1lineTurnRight(const GeoCircle1pt1lineInput_t *input,
                                       GeoCircle1pt1lineOutput_t *output,
                                       Real lineA,
                                       Real lineB,
                                       Real lineC) {
  output->TrnDir = -1;

  Real sinDir;
  Real cosDir;
  Real angDir;
  angDir = input->VstTheta - APA_HALF_PI;
  sinDir = sinf(angDir);
  cosDir = cosf(angDir);

  Real termAx;
  Real termBy;
  Real num;
  termAx = lineA * input->VstX;
  termBy = lineB * input->VstY;
  num = termAx + termBy;
  num = num + lineC;

  Real dotA;
  Real dotB;
  Real dot;
  Real d0;
  dotA = lineA * cosDir;
  dotB = lineB * sinDir;
  dot = dotA + dotB;
  d0 = 1.0f - dot;

  Real absD0;
  Boolean condTiny;
  absD0 = fabsf(d0);
  condTiny = (absD0 < GEO_EPSILON);
  if (condTiny) {
    output->Radius = GEO_CIRCLE_1PT1LINE_DEGEN_RADIUS;
  } else {
    output->Radius = num / d0;
  }

  Real rCos;
  Real rSin;
  rCos = (output->Radius) * cosDir;
  rSin = (output->Radius) * sinDir;
  output->CptX = input->VstX + rCos;
  output->CptY = input->VstY + rSin;

  Real angLn;
  Real cosLn;
  Real sinLn;
  Real rCosLn;
  Real rSinLn;
  angLn = input->VtgtTheta + APA_HALF_PI;
  cosLn = cosf(angLn);
  sinLn = sinf(angLn);
  rCosLn = (output->Radius) * cosLn;
  rSinLn = (output->Radius) * sinLn;
  output->LnPtX = output->CptX + rCosLn;
  output->LnPtY = output->CptY + rSinLn;
  output->LnPtTheta = input->VtgtTheta;

  Boolean condDirFwd;
  condDirFwd = (input->Dir > 0);
  if (condDirFwd) {
    Boolean condWrap;
    condWrap = (input->VtgtTheta > input->VstTheta);
    if (condWrap) {
      Real vstPlus2pi;
      vstPlus2pi = input->VstTheta + APA_TWO_PI;
      output->ArcAngle = vstPlus2pi - input->VtgtTheta;
    } else {
      output->ArcAngle = input->VstTheta - input->VtgtTheta;
    }
  } else {
    Boolean condWrap2;
    condWrap2 = (input->VtgtTheta < input->VstTheta);
    if (condWrap2) {
      Real vtgtPlus2pi;
      vtgtPlus2pi = input->VtgtTheta + APA_TWO_PI;
      output->ArcAngle = vtgtPlus2pi - input->VstTheta;
    } else {
      output->ArcAngle = input->VtgtTheta - input->VstTheta;
    }
  }

  Real absAngleR;
  absAngleR = fabsf(output->ArcAngle);
  output->ArcLen = absAngleR * fabsf(output->Radius);
}

static void geoCircle1pt1lineTurnLeft(const GeoCircle1pt1lineInput_t *input,
                                      GeoCircle1pt1lineOutput_t *output,
                                      Real lineA,
                                      Real lineB,
                                      Real lineC) {
  output->TrnDir = 1;

  Real sinDir;
  Real cosDir;
  Real angDir;
  angDir = input->VstTheta + APA_HALF_PI;
  sinDir = sinf(angDir);
  cosDir = cosf(angDir);

  Real termAx;
  Real termBy;
  Real num;
  termAx = lineA * input->VstX;
  termBy = lineB * input->VstY;
  num = termAx + termBy;
  num = num + lineC;

  Real dotA;
  Real dotB;
  Real dot;
  Real d0;
  dotA = lineA * cosDir;
  dotB = lineB * sinDir;
  dot = dotA + dotB;
  d0 = -1.0f - dot;

  Real absD0;
  Boolean condTiny;
  absD0 = fabsf(d0);
  condTiny = (absD0 < GEO_EPSILON);
  if (condTiny) {
    output->Radius = GEO_CIRCLE_1PT1LINE_DEGEN_RADIUS;
  } else {
    output->Radius = num / d0;
  }

  Real rCos;
  Real rSin;
  rCos = (output->Radius) * cosDir;
  rSin = (output->Radius) * sinDir;
  output->CptX = input->VstX + rCos;
  output->CptY = input->VstY + rSin;

  Real angLn;
  Real cosLn;
  Real sinLn;
  Real rCosLn;
  Real rSinLn;
  angLn = input->VtgtTheta - APA_HALF_PI;
  cosLn = cosf(angLn);
  sinLn = sinf(angLn);
  rCosLn = (output->Radius) * cosLn;
  rSinLn = (output->Radius) * sinLn;
  output->LnPtX = output->CptX + rCosLn;
  output->LnPtY = output->CptY + rSinLn;
  output->LnPtTheta = input->VtgtTheta;

  Boolean condDirFwd;
  condDirFwd = (input->Dir > 0);
  if (condDirFwd) {
    Boolean condWrap;
    condWrap = (input->VtgtTheta < input->VstTheta);
    if (condWrap) {
      Real vtgtPlus2pi;
      vtgtPlus2pi = input->VtgtTheta + APA_TWO_PI;
      output->ArcAngle = vtgtPlus2pi - input->VstTheta;
    } else {
      output->ArcAngle = input->VtgtTheta - input->VstTheta;
    }
  } else {
    Boolean condWrap2;
    condWrap2 = (input->VtgtTheta > input->VstTheta);
    if (condWrap2) {
      Real vstPlus2pi;
      vstPlus2pi = input->VstTheta + APA_TWO_PI;
      output->ArcAngle = vstPlus2pi - input->VtgtTheta;
    } else {
      output->ArcAngle = input->VstTheta - input->VtgtTheta;
    }
  }

  Real absAngleR;
  absAngleR = fabsf(output->ArcAngle);
  output->ArcLen = absAngleR * fabsf(output->Radius);
}

static void geoCircle1pt1lineFillLnPtR(const GeoCircle1pt1lineInput_t *input,
                                       GeoCircle1pt1lineOutput_t *output) {
  Real diffX;
  Real diffY;
  Real cosT;
  Real sinT;
  Real thetaDelta;

  diffX = output->LnPtX - input->VtgtX;
  diffY = output->LnPtY - input->VtgtY;
  cosT = cosf(input->VtgtTheta);
  sinT = sinf(input->VtgtTheta);

  output->LnPtRX = (cosT * diffX) + (sinT * diffY);
  output->LnPtRY = (cosT * diffY) - (sinT * diffX);

  thetaDelta = output->LnPtTheta - input->VtgtTheta;
  while (thetaDelta > APA_PI) {
    thetaDelta = thetaDelta - APA_TWO_PI;
  }
  while (thetaDelta < -APA_PI) {
    thetaDelta = thetaDelta + APA_TWO_PI;
  }
  output->LnPtRTheta = thetaDelta;
}

/**
 * @brief 计算过起点且与目标点方向直线相切的圆弧参数，根据起点在目标坐标系中的位置判断左转或右转
 * @en_name geoCircle1pt1line
 * @cn_name 过一点切直线圆弧
 * @type module
 * @param[IN] Real VstX 起点X坐标
 * @param[IN] Real VstY 起点Y坐标
 * @param[IN] Real VstTheta 起点航向角
 * @param[IN] Real VtgtX 目标点X坐标
 * @param[IN] Real VtgtY 目标点Y坐标
 * @param[IN] Real VtgtTheta 目标点航向角
 * @param[IN] Integer Dir 行驶方向1前进2后退
 * @param[OUT] Real CptX 圆心X坐标
 * @param[OUT] Real CptY 圆心Y坐标
 * @param[OUT] Real Radius 圆弧半径
 * @param[OUT] Real LnPtX 圆弧与直线切点X
 * @param[OUT] Real LnPtY 圆弧与直线切点Y
 * @param[OUT] Real LnPtTheta 切点航向角
 * @param[OUT] Real LnPtRX 目标坐标系切点X
 * @param[OUT] Real LnPtRY 目标坐标系切点Y
 * @param[OUT] Real LnPtRTheta 目标坐标系切点航向角
 * @param[OUT] Integer TrnDir 圆弧方向1左转负1右转
 * @param[OUT] Real ArcLen 圆弧长度
 * @param[OUT] Real ArcAngle 圆弧对应圆心角
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 几何原语
 * @source matlab_rewrite geo_circle_1pt1line.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */

void geoCircle1pt1line(const GeoCircle1pt1lineInput_t *input, GeoCircle1pt1lineOutput_t *output) {

  Real vstX;
  Real vstY;
  Real vtgtX;
  Real vtgtY;
  Real vtgtTheta;
  vstX = input->VstX;
  vstY = input->VstY;
  vtgtX = input->VtgtX;
  vtgtY = input->VtgtY;
  vtgtTheta = input->VtgtTheta;

  Real cosVtgt;
  Real sinVtgt;
  cosVtgt = cosf(vtgtTheta);
  sinVtgt = sinf(vtgtTheta);

  Real diffX0;
  Real diffY0;
  diffX0 = vstX - vtgtX;
  diffY0 = vstY - vtgtY;

  Real ptRy;
  Real tmpAx;
  Real tmpAy;
  tmpAx = cosVtgt * diffY0;
  tmpAy = sinVtgt * diffX0;
  ptRy = tmpAx - tmpAy;

  Real sinT;
  Real cosT;
  sinT = sinf(vtgtTheta);
  cosT = cosf(vtgtTheta);

  Real lineA;
  Real lineB;
  Real lineC;
  Real vtgtXpC;
  Real vtgtYpS;
  Real prod1;
  Real prod2;
  lineA = sinT;
  lineB = -cosT;
  vtgtXpC = vtgtX + cosT;
  vtgtYpS = vtgtY + sinT;
  prod1 = vtgtXpC * vtgtY;
  prod2 = vtgtX * vtgtYpS;
  lineC = prod1 - prod2;

  Boolean condRight;
  condRight = (ptRy < 0.0f);

  if (condRight) {
    geoCircle1pt1lineTurnRight(input, output, lineA, lineB, lineC);
  } else {
    geoCircle1pt1lineTurnLeft(input, output, lineA, lineB, lineC);
  }

  geoCircle1pt1lineFillLnPtR(input, output);
}
