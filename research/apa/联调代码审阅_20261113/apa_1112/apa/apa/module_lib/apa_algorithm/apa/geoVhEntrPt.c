#include "geoVhEntrPtTypes.h"
#include "geoLineCrossPtTypes.h"
#include <math.h>

/**
 * @brief VH入口点计算 — 入口边偏移后与目标航向线的交点
 * @en_name geoVhEntrPt
 * @cn_name VH入口点计算
 * @type module
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Integer SltEntr 车位入口边编号
 * @param[IN] Real TgtX 目标X
 * @param[IN] Real TgtY 目标Y
 * @param[IN] Real TgtTheta 目标航向
 * @param[IN] Integer PrkType 泊车方向1左2右
 * @param[IN] Integer SurdType 周边类型
 * @param[OUT] Real EntrX 入口点X
 * @param[OUT] Real EntrY 入口点Y
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 VH入口点
 * @source matlab_rewrite geo_vh_entrPt.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void geoVhEntrPt(const GeoVhEntrPtInput_t *input,
                 const GeoVhEntrPtParam_t *param,
                 GeoVhEntrPtOutput_t *output) {
  Real sltEntLn1X;
  Real sltEntLn1Y;
  Real sltEntLn2X;
  Real sltEntLn2Y;
  Real entOfst;
  Real cosT;
  Real sinT;
  GeoLineCrossPtInput_t lcIn;
  GeoLineCrossPtOutput_t lcOut;

  *output = (GeoVhEntrPtOutput_t){0};

  if (input->SltEntr == 1) {
    sltEntLn1X = input->SltPt1X;
    sltEntLn1Y = input->SltPt1Y;
    sltEntLn2X = input->SltPt2X;
    sltEntLn2Y = input->SltPt2Y;
  } else if (input->SltEntr == 2) {
    sltEntLn1X = input->SltPt2X;
    sltEntLn1Y = input->SltPt2Y;
    sltEntLn2X = input->SltPt3X;
    sltEntLn2Y = input->SltPt3Y;
  } else if (input->SltEntr == 3) {
    sltEntLn1X = input->SltPt3X;
    sltEntLn1Y = input->SltPt3Y;
    sltEntLn2X = input->SltPt4X;
    sltEntLn2Y = input->SltPt4Y;
  } else {
    sltEntLn1X = input->SltPt4X;
    sltEntLn1Y = input->SltPt4Y;
    sltEntLn2X = input->SltPt1X;
    sltEntLn2Y = input->SltPt1Y;
  }

  if (input->PrkType == 1) {
    if (input->SurdType == 3 || input->SurdType == 1) {
      entOfst = -param->WheelBase;
    } else {
      entOfst = -param->LLf;
    }
  } else if (input->PrkType == 2) {
    if (input->SurdType == 3 || input->SurdType == 2) {
      entOfst = -param->WheelBase;
    } else {
      entOfst = -param->LLf;
    }
  } else {
    entOfst = -param->LLf;
  }

  cosT = cosf(input->TgtTheta);
  sinT = sinf(input->TgtTheta);

  lcIn.Ln1X0 = sltEntLn1X + entOfst * cosT;
  lcIn.Ln1Y0 = sltEntLn1Y + entOfst * sinT;
  lcIn.Ln1X1 = sltEntLn2X + entOfst * cosT;
  lcIn.Ln1Y1 = sltEntLn2Y + entOfst * sinT;

  lcIn.Ln2X0 = input->TgtX;
  lcIn.Ln2Y0 = input->TgtY;
  lcIn.Ln2X1 = input->TgtX + cosT;
  lcIn.Ln2Y1 = input->TgtY + sinT;

  geoLineCrossPt(&lcIn, &lcOut);

  output->EntrX = lcOut.CrossX;
  output->EntrY = lcOut.CrossY;
}
