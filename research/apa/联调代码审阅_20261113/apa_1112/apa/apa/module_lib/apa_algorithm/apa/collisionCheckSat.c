#include "collisionCheckSatTypes.h"

static Boolean collisionCheckSatProjectAndTest(Real axisX,
                                               Real axisY,
                                               Real px1,
                                               Real py1,
                                               Real px2,
                                               Real py2,
                                               Real px3,
                                               Real py3,
                                               Real px4,
                                               Real py4,
                                               Real lx1,
                                               Real ly1,
                                               Real lx2,
                                               Real ly2) {

  Real proj1;
  Real proj2;
  Real proj3;
  Real proj4;
  Real polyMin;
  Real polyMax;
  Real lineProj1;
  Real lineProj2;
  Real lineMin;
  Real lineMax;
  Boolean condSep;

  proj1 = px1 * axisX + py1 * axisY;
  proj2 = px2 * axisX + py2 * axisY;
  proj3 = px3 * axisX + py3 * axisY;
  proj4 = px4 * axisX + py4 * axisY;

  polyMin = proj1;
  polyMax = proj1;
  if (proj2 < polyMin) {
    polyMin = proj2;
  }
  if (proj2 > polyMax) {
    polyMax = proj2;
  }
  if (proj3 < polyMin) {
    polyMin = proj3;
  }
  if (proj3 > polyMax) {
    polyMax = proj3;
  }
  if (proj4 < polyMin) {
    polyMin = proj4;
  }
  if (proj4 > polyMax) {
    polyMax = proj4;
  }

  lineProj1 = lx1 * axisX + ly1 * axisY;
  lineProj2 = lx2 * axisX + ly2 * axisY;

  if (lineProj1 < lineProj2) {
    lineMin = lineProj1;
    lineMax = lineProj2;
  } else {
    lineMin = lineProj2;
    lineMax = lineProj1;
  }

  Boolean condNoOverlap1;
  Boolean condNoOverlap2;
  condNoOverlap1 = (polyMax < lineMin);
  condNoOverlap2 = (lineMax < polyMin);
  condSep = condNoOverlap1 || condNoOverlap2;

  return condSep;
}

/**
 * @brief 基于分离轴定理(SAT)检测矩形与线段是否碰撞。
 *        在矩形的4条边法线和线段法线共5个轴上投影，
 *        若存在投影不重叠的轴则不碰撞，否则碰撞。
 * @en_name collisionCheckSat
 * @cn_name SAT矩形线段碰撞检测
 * @type module
 * @param[IN] Real RectX1 矩形角点1X
 * @param[IN] Real RectY1 矩形角点1Y
 * @param[IN] Real RectX2 矩形角点2X
 * @param[IN] Real RectY2 矩形角点2Y
 * @param[IN] Real RectX3 矩形角点3X
 * @param[IN] Real RectY3 矩形角点3Y
 * @param[IN] Real RectX4 矩形角点4X
 * @param[IN] Real RectY4 矩形角点4Y
 * @param[IN] Real LineX1 线段起点X
 * @param[IN] Real LineY1 线段起点Y
 * @param[IN] Real LineX2 线段终点X
 * @param[IN] Real LineY2 线段终点Y
 * @param[OUT] Boolean IsColliding 是否碰撞
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite collision_Check_SAT.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void collisionCheckSat(const CollisionCheckSatInput_t *input, CollisionCheckSatOutput_t *output) {

  Real px1;
  Real py1;
  Real px2;
  Real py2;
  Real px3;
  Real py3;
  Real px4;
  Real py4;
  Real lx1;
  Real ly1;
  Real lx2;
  Real ly2;

  px1 = input->RectX1;
  py1 = input->RectY1;
  px2 = input->RectX2;
  py2 = input->RectY2;
  px3 = input->RectX3;
  py3 = input->RectY3;
  px4 = input->RectX4;
  py4 = input->RectY4;
  lx1 = input->LineX1;
  ly1 = input->LineY1;
  lx2 = input->LineX2;
  ly2 = input->LineY2;

  Real e1x;
  Real e1y;
  Real e2x;
  Real e2y;
  Real e3x;
  Real e3y;
  Real e4x;
  Real e4y;

  e1x = px2 - px1;
  e1y = py2 - py1;
  e2x = px3 - px2;
  e2y = py3 - py2;
  e3x = px4 - px3;
  e3y = py4 - py3;
  e4x = px1 - px4;
  e4y = py1 - py4;

  Real n1x;
  Real n1y;
  Real n2x;
  Real n2y;
  Real n3x;
  Real n3y;
  Real n4x;
  Real n4y;
  Real nlx;
  Real nly;

  n1x = -e1y;
  n1y = e1x;
  n2x = -e2y;
  n2y = e2x;
  n3x = -e3y;
  n3y = e3x;
  n4x = -e4y;
  n4y = e4x;

  Real lvx;
  Real lvy;
  lvx = lx2 - lx1;
  lvy = ly2 - ly1;
  nlx = -lvy;
  nly = lvx;

  Boolean separated;
  separated = false;

  Boolean sep1;
  sep1 = collisionCheckSatProjectAndTest(
      n1x, n1y, px1, py1, px2, py2, px3, py3, px4, py4, lx1, ly1, lx2, ly2);
  if (sep1) {
    separated = true;
  }

  if (!separated) {
    Boolean sep2;
    sep2 = collisionCheckSatProjectAndTest(
        n2x, n2y, px1, py1, px2, py2, px3, py3, px4, py4, lx1, ly1, lx2, ly2);
    if (sep2) {
      separated = true;
    }
  }

  if (!separated) {
    Boolean sep3;
    sep3 = collisionCheckSatProjectAndTest(
        n3x, n3y, px1, py1, px2, py2, px3, py3, px4, py4, lx1, ly1, lx2, ly2);
    if (sep3) {
      separated = true;
    }
  }

  if (!separated) {
    Boolean sep4;
    sep4 = collisionCheckSatProjectAndTest(
        n4x, n4y, px1, py1, px2, py2, px3, py3, px4, py4, lx1, ly1, lx2, ly2);
    if (sep4) {
      separated = true;
    }
  }

  if (!separated) {
    Boolean sep5;
    sep5 = collisionCheckSatProjectAndTest(
        nlx, nly, px1, py1, px2, py2, px3, py3, px4, py4, lx1, ly1, lx2, ly2);
    if (sep5) {
      separated = true;
    }
  }

  if (separated) {
    output->IsColliding = false;
  } else {
    output->IsColliding = true;
  }
}
