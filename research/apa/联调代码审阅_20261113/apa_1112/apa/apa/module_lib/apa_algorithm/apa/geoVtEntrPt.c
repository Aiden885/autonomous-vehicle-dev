#include "geoAxisTransTypes.h"
#include "geoLineCrossPtTypes.h"
#include "geoVtEntrPtTypes.h"
#include <math.h>
/**
 * internal structs
 */
static void geoVtEntrPtGetEntrPoints(
    const GeoVtEntrPtInput_t *input, Real *entP1X0, Real *entP1Y0, Real *entP2X0, Real *entP2Y0) {
  if (input->SltEntr == 2) {
    *entP1X0 = input->SltPt2X;
    *entP1Y0 = input->SltPt2Y;
    *entP2X0 = input->SltPt3X;
    *entP2Y0 = input->SltPt3Y;
  } else if (input->SltEntr == 3) {
    *entP1X0 = input->SltPt3X;
    *entP1Y0 = input->SltPt3Y;
    *entP2X0 = input->SltPt4X;
    *entP2Y0 = input->SltPt4Y;
  } else if (input->SltEntr == 4) {
    *entP1X0 = input->SltPt4X;
    *entP1Y0 = input->SltPt4Y;
    *entP2X0 = input->SltPt1X;
    *entP2Y0 = input->SltPt1Y;
  } else {
    *entP1X0 = input->SltPt1X;
    *entP1Y0 = input->SltPt1Y;
    *entP2X0 = input->SltPt2X;
    *entP2Y0 = input->SltPt2Y;
  }
}

static void geoVtEntrPtCalcWds(Real rel1Y, Real rel2Y, Real *ltWd1, Real *rtWd1) {
  Real ltWd0;
  Real rtWd0;
  if (rel1Y > 0.0f) {
    ltWd0 = rel1Y;
    rtWd0 = -rel2Y;
  } else {
    rtWd0 = -rel1Y;
    ltWd0 = rel2Y;
  }
  if (ltWd0 < 0.0f) {
    *ltWd1 = 0.0f;
  } else {
    *ltWd1 = ltWd0;
  }
  if (rtWd0 < 0.0f) {
    *rtWd1 = 0.0f;
  } else {
    *rtWd1 = rtWd0;
  }
}

/**
 * @brief 根据车位入口边、目标位姿及周边开口类型计算VT泊入入口点
 * @en_name geoVtEntrPt
 * @cn_name VT入口点计算
 * @type module
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Integer SltEntr 入口边编号1到4
 * @param[IN] Real VehX 车辆X（接口保留）
 * @param[IN] Real VehY 车辆Y（接口保留）
 * @param[IN] Real VehTheta 车辆航向（接口保留）
 * @param[IN] Real TgtX 目标位X
 * @param[IN] Real TgtY 目标位Y
 * @param[IN] Real TgtTheta 目标位航向
 * @param[IN] Integer SurdType 周边开口类型0无1左2右3双侧
 * @param[IN] Integer PrkType 泊车方向1左2右
 * @param[OUT] Real EntrPtX 规划入口点X
 * @param[OUT] Real EntrPtY 规划入口点Y
 * @param[OUT] Real LtWd 入口边左侧宽度
 * @param[OUT] Real RtWd 入口边右侧宽度
 * @param[OUT] Real SltDepth 车位深度估计
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 路径规划
 * @source matlab_rewrite geo_vt_entrPt.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void geoVtEntrPt(const GeoVtEntrPtInput_t *input, GeoVtEntrPtOutput_t *output) {
  const Real geoEps = 1e-6f;

  Real p1x = input->SltPt1X;
  Real p1y = input->SltPt1Y;
  Real p2x = input->SltPt2X;
  Real p2y = input->SltPt2Y;
  Real tgtX = input->TgtX;
  Real tgtY = input->TgtY;
  Real tgtTheta = input->TgtTheta;
  Real cosT = cosf(tgtTheta);
  Real sinT = sinf(tgtTheta);
  Real entP1X0;
  Real entP1Y0;
  Real entP2X0;
  Real entP2Y0;
  Real entP1X1;
  Real entP1Y1;
  Real entP2X1;
  Real entP2Y1;

  geoVtEntrPtGetEntrPoints(input, &entP1X0, &entP1Y0, &entP2X0, &entP2Y0);

  Real edgeDx;
  Real edgeDy;
  Real edgeLen2;
  Boolean condEdgeValid0;
  edgeDx = entP2X0 - entP1X0;
  edgeDy = entP2Y0 - entP1Y0;
  edgeLen2 = edgeDx * edgeDx + edgeDy * edgeDy;
  condEdgeValid0 = (edgeLen2 >= geoEps);
  if (condEdgeValid0) {
    entP1X1 = entP1X0;
    entP1Y1 = entP1Y0;
    entP2X1 = entP2X0;
    entP2Y1 = entP2Y0;
  } else {
    entP1X1 = p1x;
    entP1Y1 = p1y;
    entP2X1 = p2x;
    entP2Y1 = p2y;
  }

  GeoAxisTransInput_t axisInput;
  GeoAxisTransOutput_t axisOutput;

  axisInput.PtX = entP1X1;
  axisInput.PtY = entP1Y1;
  axisInput.PtTheta = 0.0f;
  axisInput.Ori1X = 0.0f;
  axisInput.Ori1Y = 0.0f;
  axisInput.Ori1Theta = 0.0f;
  axisInput.Ori2X = tgtX;
  axisInput.Ori2Y = tgtY;
  axisInput.Ori2Theta = tgtTheta;
  geoAxisTrans(&axisInput, &axisOutput);

  Real rel1X = axisOutput.ResultX;
  Real rel1Y = axisOutput.ResultY;

  axisInput.PtX = entP2X1;
  axisInput.PtY = entP2Y1;
  geoAxisTrans(&axisInput, &axisOutput);
  Real rel2X = axisOutput.ResultX;
  Real rel2Y = axisOutput.ResultY;

  Real ltWd1;
  Real rtWd1;

  geoVtEntrPtCalcWds(rel1Y, rel2Y, &ltWd1, &rtWd1);

  Real aa;
  Real bb;
  Real cc;
  Real lineDen;
  Real lineNum;
  Real sltDepth0;
  Real sltDepth1;
  Real entOfst;
  aa = entP2Y1 - entP1Y1;
  bb = entP1X1 - entP2X1;
  cc = entP2X1 * entP1Y1 - entP1X1 * entP2Y1;
  lineDen = sqrtf(aa * aa + bb * bb);
  lineNum = fabsf(aa * tgtX + bb * tgtY + cc);
  if (lineDen > geoEps) {
    sltDepth0 = lineNum / lineDen;
  } else {
    sltDepth0 = 0.0f;
  }
  sltDepth1 = sltDepth0;
  entOfst = sltDepth1 - 1.0f;

  Real ln1x0;
  Real ln1y0;
  Real ln1x1;
  Real ln1y1;

  Boolean condPrkLeft = (input->PrkType == APA_PARK_TYPE_LEFT);
  Boolean condPrkRight = (input->PrkType == APA_PARK_TYPE_RIGHT);
  Boolean condLeftOpen =
      (input->SurdType == APA_SURD_LEFT_OPEN) | (input->SurdType == APA_SURD_BOTH_OPEN);
  Boolean condRightOpen =
      (input->SurdType == APA_SURD_RIGHT_OPEN) | (input->SurdType == APA_SURD_BOTH_OPEN);
  Boolean condShiftLeft0 = condPrkLeft & condLeftOpen;
  Boolean condShiftRight0 = condPrkRight & condRightOpen;

  if (condShiftLeft0 || condShiftRight0) {
    Real shiftX0 = entOfst * cosT;
    Real shiftY0 = entOfst * sinT;
    ln1x0 = entP1X1 - shiftX0;
    ln1y0 = entP1Y1 - shiftY0;
    ln1x1 = entP2X1 - shiftX0;
    ln1y1 = entP2Y1 - shiftY0;
  } else {
    ln1x0 = entP1X1;
    ln1y0 = entP1Y1;
    ln1x1 = entP2X1;
    ln1y1 = entP2Y1;
  }

  GeoLineCrossPtInput_t lineInput;
  GeoLineCrossPtOutput_t lineOutput;
  lineInput.Ln1X0 = ln1x0;
  lineInput.Ln1Y0 = ln1y0;
  lineInput.Ln1X1 = ln1x1;
  lineInput.Ln1Y1 = ln1y1;
  lineInput.Ln2X0 = tgtX;
  lineInput.Ln2Y0 = tgtY;
  lineInput.Ln2X1 = tgtX + cosT;
  lineInput.Ln2Y1 = tgtY + sinT;
  geoLineCrossPt(&lineInput, &lineOutput);

  output->EntrPtX = lineOutput.CrossX;
  output->EntrPtY = lineOutput.CrossY;
  output->LtWd = ltWd1;
  output->RtWd = rtWd1;
  output->SltDepth = sltDepth1;
  (void)rel1X;
  (void)rel2X;
  (void)input->VehX;
  (void)input->VehY;
  (void)input->VehTheta;
}
