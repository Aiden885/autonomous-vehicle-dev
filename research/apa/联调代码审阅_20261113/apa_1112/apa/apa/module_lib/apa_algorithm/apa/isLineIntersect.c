#include "apaMathUtils.h"
#include "isLineIntersectTypes.h"

/**
 * @brief 基于AABB预筛选+叉积跨立判定的快速线段相交检测。
 *        先用包围盒快速排除明显不相交情况，再用叉积判定两线段是否跨立。
 * @en_name isLineIntersect
 * @cn_name 线段相交检测
 * @type module
 * @param[IN] Real Ax 线段A起点X
 * @param[IN] Real Ay 线段A起点Y
 * @param[IN] Real Bx 线段A终点X
 * @param[IN] Real By 线段A终点Y
 * @param[IN] Real Cx 线段B起点X
 * @param[IN] Real Cy 线段B起点Y
 * @param[IN] Real Dx 线段B终点X
 * @param[IN] Real Dy 线段B终点Y
 * @param[OUT] Boolean IsIntersect 是否相交
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite is_Line_Intersect.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void isLineIntersect(const IsLineIntersectInput_t *input, IsLineIntersectOutput_t *output) {
  Real ax;
  Real ay;
  Real bx;
  Real by;
  Real cx;
  Real cy;
  Real dx;
  Real dy;
  Boolean result0;

  ax = input->Ax;
  ay = input->Ay;
  bx = input->Bx;
  by = input->By;
  cx = input->Cx;
  cy = input->Cy;
  dx = input->Dx;
  dy = input->Dy;

  result0 = false;

  Real maxABx;
  Real minABx;
  Real maxABy;
  Real minABy;
  Real maxCDx;
  Real minCDx;
  Real maxCDy;
  Real minCDy;

  if (ax > bx) {
    maxABx = ax;
    minABx = bx;
  } else {
    maxABx = bx;
    minABx = ax;
  }
  if (ay > by) {
    maxABy = ay;
    minABy = by;
  } else {
    maxABy = by;
    minABy = ay;
  }
  if (cx > dx) {
    maxCDx = cx;
    minCDx = dx;
  } else {
    maxCDx = dx;
    minCDx = cx;
  }
  if (cy > dy) {
    maxCDy = cy;
    minCDy = dy;
  } else {
    maxCDy = dy;
    minCDy = cy;
  }

  Boolean condSepX1;
  Boolean condSepX2;
  Boolean condSepY1;
  Boolean condSepY2;
  Boolean condAnyAabbSep;

  condSepX1 = apaMathLess(maxABx, minCDx);
  condSepX2 = apaMathLess(maxCDx, minABx);
  condSepY1 = apaMathLess(maxABy, minCDy);
  condSepY2 = apaMathLess(maxCDy, minABy);
  condAnyAabbSep = condSepX1 || condSepX2 || condSepY1 || condSepY2;

  if (!condAnyAabbSep) {

    Real crossAD;
    Real crossBD;
    Real crossProduct1;
    crossAD = apaMathCross2D(ax - dx, ay - dy, cx - dx, cy - dy);
    crossBD = apaMathCross2D(bx - dx, by - dy, cx - dx, cy - dy);
    crossProduct1 = crossAD * crossBD;

    Boolean condNotStraddle1;
    condNotStraddle1 = apaMathGreater(crossProduct1, 0.0f);

    if (!condNotStraddle1) {

      Real crossDB;
      Real crossCB;
      Real crossProduct2;

      crossDB = apaMathCross2D(dx - bx, dy - by, ax - bx, ay - by);
      crossCB = apaMathCross2D(cx - bx, cy - by, ax - bx, ay - by);

      Real negDB;
      Real negCB;
      negDB = -crossDB;
      negCB = -crossCB;
      crossProduct2 = negDB * negCB;

      Boolean condNotStraddle2;
      condNotStraddle2 = apaMathGreater(crossProduct2, 0.0f);

      if (!condNotStraddle2) {
        result0 = true;
      }
    }
  }

  output->IsIntersect = result0;
}
