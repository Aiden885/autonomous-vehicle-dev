#include "geoInboxJudgeTypes.h"

/**
 * @file geoInboxJudge.c
 * @brief 几何点入框判断
 * @en_name geoInboxJudge
 * @cn_name 几何点入框判断
 * @type module
 * @param[IN] GeoInboxJudgeInput_t Input 点入框判断输入
 * @param[OUT] GeoInboxJudgeOutput_t Output 点入框判断输出
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 几何计算
 * @source matlab_rewrite geo_InboxJudge.m
 * @version 1.0
 * @date 2026-04-14
 * @author aide
 */
void geoInboxJudge(const GeoInboxJudgeInput_t *input, GeoInboxJudgeOutput_t *output) {
  Real lnPar12_1;
  Real lnPar12_2;
  Real lnPar12_3;
  Real lnPar43_1;
  Real lnPar43_2;
  Real lnPar43_3;
  Real lnPar14_1;
  Real lnPar14_2;
  Real lnPar14_3;
  Real lnPar23_1;
  Real lnPar23_2;
  Real lnPar23_3;

  Real x0, y0, x1, y1;
  Real val1, val2, val3, val4;

  x0 = input->BoxPtsX[0];
  y0 = input->BoxPtsY[0];
  x1 = input->BoxPtsX[1];
  y1 = input->BoxPtsY[1];
  lnPar12_1 = y1 - y0;
  lnPar12_2 = x0 - x1;
  lnPar12_3 = x1 * y0 - x0 * y1;

  x0 = input->BoxPtsX[3];
  y0 = input->BoxPtsY[3];
  x1 = input->BoxPtsX[2];
  y1 = input->BoxPtsY[2];
  lnPar43_1 = y1 - y0;
  lnPar43_2 = x0 - x1;
  lnPar43_3 = x1 * y0 - x0 * y1;

  x0 = input->BoxPtsX[0];
  y0 = input->BoxPtsY[0];
  x1 = input->BoxPtsX[3];
  y1 = input->BoxPtsY[3];
  lnPar14_1 = y1 - y0;
  lnPar14_2 = x0 - x1;
  lnPar14_3 = x1 * y0 - x0 * y1;

  x0 = input->BoxPtsX[1];
  y0 = input->BoxPtsY[1];
  x1 = input->BoxPtsX[2];
  y1 = input->BoxPtsY[2];
  lnPar23_1 = y1 - y0;
  lnPar23_2 = x0 - x1;
  lnPar23_3 = x1 * y0 - x0 * y1;

  val1 = input->PtX * lnPar12_1 + input->PtY * lnPar12_2 + lnPar12_3;
  val2 = input->PtX * lnPar43_1 + input->PtY * lnPar43_2 + lnPar43_3;
  val3 = input->PtX * lnPar14_1 + input->PtY * lnPar14_2 + lnPar14_3;
  val4 = input->PtX * lnPar23_1 + input->PtY * lnPar23_2 + lnPar23_3;

  if (((val1 * val2) < 0.0f) && ((val3 * val4) < 0.0f)) {
    output->Inbox = true;
  } else {
    output->Inbox = false;
  }
}
