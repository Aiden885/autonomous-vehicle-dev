#include "isLineIntersectWithRectTypes.h"
#include "isLineIntersectTypes.h"
#include "collisionCheckSatTypes.h"

/**
 * @brief 检测线段是否与矩形任一内边相交
 *
 * 对应 MATLAB isLineIntersectWithRect.m：
 * - 若启用SAT，直接用 collisionCheckSat 检测
 * - 否则依次检测线段与矩形四条边的相交情况
 *
 * @en_name isLineIntersectWithRect
 * @cn_name 线段与矩形相交检测
 * @type module
 * @param[IN] IsLineIntersectWithRectInput_t Input 线段与矩形输入
 * @param[OUT] IsLineIntersectWithRectOutput_t Output 线段与矩形输出
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite isLineIntersectWithRect.m
 * @version 1.0
 * @date 2026-04-28
 * @author aide
 */
void isLineIntersectWithRect(const IsLineIntersectWithRectInput_t *input,
                             IsLineIntersectWithRectOutput_t *output) {
  Boolean isCollision0;

  if (input->UseSAT) {
    CollisionCheckSatInput_t satIn;
    CollisionCheckSatOutput_t satOut;

    satIn.RectX1 = input->RectX1;
    satIn.RectY1 = input->RectY1;
    satIn.RectX2 = input->RectX2;
    satIn.RectY2 = input->RectY2;
    satIn.RectX3 = input->RectX3;
    satIn.RectY3 = input->RectY3;
    satIn.RectX4 = input->RectX4;
    satIn.RectY4 = input->RectY4;
    satIn.LineX1 = input->LineX1;
    satIn.LineY1 = input->LineY1;
    satIn.LineX2 = input->LineX2;
    satIn.LineY2 = input->LineY2;

    collisionCheckSat(&satIn, &satOut);
    isCollision0 = satOut.IsColliding;
  } else {

    IsLineIntersectInput_t liIn;
    IsLineIntersectOutput_t liOut;

    liIn.Cx = input->LineX1;
    liIn.Cy = input->LineY1;
    liIn.Dx = input->LineX2;
    liIn.Dy = input->LineY2;

    liIn.Ax = input->RectX1;
    liIn.Ay = input->RectY1;
    liIn.Bx = input->RectX2;
    liIn.By = input->RectY2;
    isLineIntersect(&liIn, &liOut);
    isCollision0 = liOut.IsIntersect;

    if (isCollision0 == false) {
      liIn.Ax = input->RectX2;
      liIn.Ay = input->RectY2;
      liIn.Bx = input->RectX3;
      liIn.By = input->RectY3;
      isLineIntersect(&liIn, &liOut);
      isCollision0 = liOut.IsIntersect;
    }

    if (isCollision0 == false) {
      liIn.Ax = input->RectX3;
      liIn.Ay = input->RectY3;
      liIn.Bx = input->RectX4;
      liIn.By = input->RectY4;
      isLineIntersect(&liIn, &liOut);
      isCollision0 = liOut.IsIntersect;
    }

    if (isCollision0 == false) {
      liIn.Ax = input->RectX4;
      liIn.Ay = input->RectY4;
      liIn.Bx = input->RectX1;
      liIn.By = input->RectY1;
      isLineIntersect(&liIn, &liOut);
      isCollision0 = liOut.IsIntersect;
    }
  }

  output->IsCollision = isCollision0;
}
