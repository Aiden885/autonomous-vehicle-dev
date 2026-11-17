#include "calcVehCornerPointTypes.h"
#include <math.h>

/**
 * @brief 根据车辆位姿和几何参数计算车辆包络矩形的4个角点世界坐标。
 *        支持前/后/侧安全裕量扩展。坐标系：后轴中心为原点，X正前方，Y正左方。
 *        旋转变换使用右手坐标系 DCM（航向角为正时逆时针）。
 * @en_name calcVehCornerPoint
 * @cn_name 车辆角点计算
 * @type module
 * @param[IN] Real PosX 车辆后轴中心X
 * @param[IN] Real PosY 车辆后轴中心Y
 * @param[IN] Real PosTheta 车辆航向角
 * @param[IN] Real FrontOverhang 前悬加轴距LF
 * @param[IN] Real RearOverhang 后悬LB
 * @param[IN] Real VehWidth 车宽
 * @param[IN] Real FrontMargin 前部安全裕量
 * @param[IN] Real RearMargin 后部安全裕量
 * @param[IN] Real SideMargin 侧向安全裕量
 * @param[OUT] Real FlX 前左X
 * @param[OUT] Real FlY 前左Y
 * @param[OUT] Real FrX 前右X
 * @param[OUT] Real FrY 前右Y
 * @param[OUT] Real RrX 后右X
 * @param[OUT] Real RrY 后右Y
 * @param[OUT] Real RlX 后左X
 * @param[OUT] Real RlY 后左Y
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite get_veh_corner_point.m + get_ego_corners.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void calcVehCornerPoint(const CalcVehCornerPointInput_t *input,
                        CalcVehCornerPointOutput_t *output) {
  Real posX;
  Real posY;
  Real theta;
  Real lf;
  Real lb;
  Real halfW;
  Real cosT;
  Real sinT;

  posX = input->PosX;
  posY = input->PosY;
  theta = input->PosTheta;
  lf = input->FrontOverhang + input->FrontMargin;
  lb = input->RearOverhang + input->RearMargin;
  halfW = input->VehWidth * 0.5f + input->SideMargin;

  cosT = cosf(theta);
  sinT = sinf(theta);

  Real flLocalX;
  Real flLocalY;
  Real frLocalX;
  Real frLocalY;
  Real rrLocalX;
  Real rrLocalY;
  Real rlLocalX;
  Real rlLocalY;

  flLocalX = lf;
  flLocalY = halfW;
  frLocalX = lf;
  frLocalY = -halfW;
  rrLocalX = -lb;
  rrLocalY = -halfW;
  rlLocalX = -lb;
  rlLocalY = halfW;

  Real flWx;
  Real flWy;
  Real frWx;
  Real frWy;
  Real rrWx;
  Real rrWy;
  Real rlWx;
  Real rlWy;

  flWx = cosT * flLocalX - sinT * flLocalY + posX;
  flWy = sinT * flLocalX + cosT * flLocalY + posY;
  frWx = cosT * frLocalX - sinT * frLocalY + posX;
  frWy = sinT * frLocalX + cosT * frLocalY + posY;
  rrWx = cosT * rrLocalX - sinT * rrLocalY + posX;
  rrWy = sinT * rrLocalX + cosT * rrLocalY + posY;
  rlWx = cosT * rlLocalX - sinT * rlLocalY + posX;
  rlWy = sinT * rlLocalX + cosT * rlLocalY + posY;

  output->FlX = flWx;
  output->FlY = flWy;
  output->FrX = frWx;
  output->FrY = frWy;
  output->RrX = rrWx;
  output->RrY = rrWy;
  output->RlX = rlWx;
  output->RlY = rlWy;
}
