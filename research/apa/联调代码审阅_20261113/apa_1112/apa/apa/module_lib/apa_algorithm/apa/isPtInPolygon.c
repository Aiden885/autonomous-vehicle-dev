#include "apaMathUtils.h"
#include "isPtInPolygonTypes.h"

static Integer isPtInPolygonTestEdge(
    Real ptX, Real ptY, Real vx, Real vy, Real vxPrev, Real vyPrev, Integer crossCount) {

  Boolean cond1a;
  Boolean cond1b;
  Boolean cond2a;
  Boolean cond2b;
  Boolean condUp;
  Boolean condDown;
  Boolean condCrossCandidate;

  cond1a = !apaMathGreater(vy, ptY);
  cond1b = apaMathLess(ptY, vyPrev);
  cond2a = !apaMathGreater(vyPrev, ptY);
  cond2b = apaMathLess(ptY, vy);
  condUp = cond1a && cond1b;
  condDown = cond2a && cond2b;
  condCrossCandidate = condUp || condDown;

  if (condCrossCandidate) {

    Real dyEdge;
    dyEdge = vyPrev - vy;
    Real diffVxPrev;
    Real diffPtY;
    Real numerator0;
    Real safeDyEdge0;
    Real dxEdge;
    Real crossX;
    Boolean condPtLeft;

    diffVxPrev = vxPrev - vx;
    diffPtY = ptY - vy;
    numerator0 = diffVxPrev * diffPtY;
    safeDyEdge0 = apaMathDivideZeroProcess(dyEdge);
    dxEdge = numerator0 / safeDyEdge0;
    crossX = vx + dxEdge;

    condPtLeft = apaMathLess(ptX, crossX);
    if (condPtLeft) {
      crossCount = crossCount + 1;
    }
  }

  return crossCount;
}

/**
 * @brief 使用射线法（ray-casting）判定点是否在闭合多边形内部。
 *        从待检测点向X正方向射出射线，统计与多边形边的交叉次数，
 *        奇数次为内部，偶数次为外部。首先检测点是否恰好在顶点上。
 * @en_name isPtInPolygon
 * @cn_name 点在多边形内检测
 * @type module
 * @param[IN] Real PtX 待检测点X
 * @param[IN] Real PtY 待检测点Y
 * @param[IN] Real PolyX 多边形顶点X数组
 * @param[IN] Real PolyY 多边形顶点Y数组
 * @param[IN] Integer PolyN 多边形顶点数
 * @param[OUT] Boolean IsInside 点是否在多边形内
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite is_Pt_In_Ploy.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void isPtInPolygon(const IsPtInPolygonInput_t *input, IsPtInPolygonOutput_t *output) {
  Real ptX;
  Real ptY;
  Integer polyN;
  Boolean result0;
  Integer k;

  ptX = input->PtX;
  ptY = input->PtY;
  polyN = input->PolyN;
  result0 = false;

  Boolean condValid;
  Boolean condTooFew;
  Boolean condTooMany;
  condTooFew = (polyN < 3);
  condTooMany = (polyN > MAX_POLYGON_VERTICES);
  condValid = !condTooFew && !condTooMany;

  if (condValid) {

    Boolean onVertex;
    onVertex = false;
    for (k = 0; k < polyN; k = k + 1) {
      Real vx;
      Real vy;
      Boolean condXeq;
      Boolean condYeq;

      vx = input->PolyX[k];
      vy = input->PolyY[k];
      condXeq = apaMathEqual(ptX, vx);
      condYeq = apaMathEqual(ptY, vy);
      if (condXeq && condYeq) {
        onVertex = true;
      }
    }

    if (!onVertex) {

      Integer crossCount;
      crossCount = 0;

      for (k = 1; k < polyN; k = k + 1) {
        Real vx;
        Real vy;
        Integer prevIdx;
        Real vxPrev;
        Real vyPrev;

        vx = input->PolyX[k];
        vy = input->PolyY[k];
        prevIdx = k - 1;
        vxPrev = input->PolyX[prevIdx];
        vyPrev = input->PolyY[prevIdx];

        crossCount = isPtInPolygonTestEdge(ptX, ptY, vx, vy, vxPrev, vyPrev, crossCount);
      }

      Integer parity;
      parity = crossCount % 2;
      Boolean condOdd;
      condOdd = (parity != 0);
      if (condOdd) {
        result0 = true;
      }
    }
  }

  output->IsInside = result0;
}
