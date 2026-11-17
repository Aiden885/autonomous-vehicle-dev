#include "calcRectangleCornersTypes.h"
#include <math.h>

/**
 * @brief 根据中心位姿和长宽计算矩形4角点的世界坐标。
 *        使用2D旋转矩阵将局部角点变换到世界坐标系。
 * @en_name calcRectangleCorners
 * @cn_name 通用矩形角点计算
 * @type module
 * @param[IN] Real CenterX 矩形中心X
 * @param[IN] Real CenterY 矩形中心Y
 * @param[IN] Real Theta 矩形朝向
 * @param[IN] Real Length 矩形长度
 * @param[IN] Real Width 矩形宽度
 * @param[OUT] Real LuX 左上角X
 * @param[OUT] Real LuY 左上角Y
 * @param[OUT] Real RuX 右上角X
 * @param[OUT] Real RuY 右上角Y
 * @param[OUT] Real RlX 右下角X
 * @param[OUT] Real RlY 右下角Y
 * @param[OUT] Real LlX 左下角X
 * @param[OUT] Real LlY 左下角Y
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite calcuRectangleCorners.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void calcRectangleCorners(const CalcRectangleCornersInput_t *input,
                          CalcRectangleCornersOutput_t *output) {
  Real cx;
  Real cy;
  Real theta;
  Real halfL;
  Real halfW;
  Real cosT;
  Real sinT;

  cx = input->CenterX;
  cy = input->CenterY;
  theta = input->Theta;
  halfL = input->Length * 0.5f;
  halfW = input->Width * 0.5f;
  cosT = cosf(theta);
  sinT = sinf(theta);

  Real luLx;
  Real luLy;
  Real ruLx;
  Real ruLy;
  Real rlLx;
  Real rlLy;
  Real llLx;
  Real llLy;

  luLx = halfL;
  luLy = halfW;
  ruLx = halfL;
  ruLy = -halfW;
  rlLx = -halfL;
  rlLy = -halfW;
  llLx = -halfL;
  llLy = halfW;

  output->LuX = cosT * luLx - sinT * luLy + cx;
  output->LuY = sinT * luLx + cosT * luLy + cy;
  output->RuX = cosT * ruLx - sinT * ruLy + cx;
  output->RuY = sinT * ruLx + cosT * ruLy + cy;
  output->RlX = cosT * rlLx - sinT * rlLy + cx;
  output->RlY = sinT * rlLx + cosT * rlLy + cy;
  output->LlX = cosT * llLx - sinT * llLy + cx;
  output->LlY = sinT * llLx + cosT * llLy + cy;
}
