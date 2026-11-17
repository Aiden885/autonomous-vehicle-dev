#include "rectCollisionCheckForBoundaryTypes.h"

static Boolean rectCollisionCheckForBoundaryTestLine(Real rx1,
                                                     Real ry1,
                                                     Real rx2,
                                                     Real ry2,
                                                     Real rx3,
                                                     Real ry3,
                                                     Real rx4,
                                                     Real ry4,
                                                     Real lx1,
                                                     Real ly1,
                                                     Real lx2,
                                                     Real ly2) {
  CollisionCheckSatInput_t satIn;
  CollisionCheckSatOutput_t satOut;

  satIn.RectX1 = rx1;
  satIn.RectY1 = ry1;
  satIn.RectX2 = rx2;
  satIn.RectY2 = ry2;
  satIn.RectX3 = rx3;
  satIn.RectY3 = ry3;
  satIn.RectX4 = rx4;
  satIn.RectY4 = ry4;
  satIn.LineX1 = lx1;
  satIn.LineY1 = ly1;
  satIn.LineX2 = lx2;
  satIn.LineY2 = ly2;
  collisionCheckSat(&satIn, &satOut);

  return satOut.IsColliding;
}

static Boolean rectCollisionCheckForBoundaryPtInRect(Real px,
                                                     Real py,
                                                     Real rx1,
                                                     Real ry1,
                                                     Real rx2,
                                                     Real ry2,
                                                     Real rx3,
                                                     Real ry3,
                                                     Real rx4,
                                                     Real ry4) {

  Real c1;
  Real c2;
  Real c3;
  Real c4;

  c1 = (rx2 - rx1) * (py - ry1) - (ry2 - ry1) * (px - rx1);
  c2 = (rx3 - rx2) * (py - ry2) - (ry3 - ry2) * (px - rx2);
  c3 = (rx4 - rx3) * (py - ry3) - (ry4 - ry3) * (px - rx3);
  c4 = (rx1 - rx4) * (py - ry4) - (ry1 - ry4) * (px - rx4);

  Boolean allNonNeg;
  Boolean allNonPos;
  Boolean result0;
  allNonNeg = (c1 >= 0.0f) && (c2 >= 0.0f) && (c3 >= 0.0f) && (c4 >= 0.0f);
  allNonPos = (c1 <= 0.0f) && (c2 <= 0.0f) && (c3 <= 0.0f) && (c4 <= 0.0f);

  if (allNonNeg || allNonPos) {
    result0 = true;
  } else {
    result0 = false;
  }
  return result0;
}

/**
 * @brief 检测矩形是否与边界线段集合或离散障碍物点集合发生碰撞。
 *        先遍历所有边界线段检测线段与矩形是否碰撞(SAT)，
 *        再遍历离散点检测是否在矩形内部(叉积法)。
 * @en_name rectCollisionCheckForBoundary
 * @cn_name 矩形边界碰撞检测
 * @type module
 * @param[IN] Real RectX1 矩形角点1X
 * @param[IN] Real RectY1 矩形角点1Y
 * @param[IN] Real RectX2 矩形角点2X
 * @param[IN] Real RectY2 矩形角点2Y
 * @param[IN] Real RectX3 矩形角点3X
 * @param[IN] Real RectY3 矩形角点3Y
 * @param[IN] Real RectX4 矩形角点4X
 * @param[IN] Real RectY4 矩形角点4Y
 * @param[IN] Integer BoundaryLineNum 边界线段数
 * @param[IN] Real BoundaryLineX1 边界线段起点X数组
 * @param[IN] Real BoundaryLineY1 边界线段起点Y数组
 * @param[IN] Real BoundaryLineX2 边界线段终点X数组
 * @param[IN] Real BoundaryLineY2 边界线段终点Y数组
 * @param[IN] Integer SeparatePtsNum 离散障碍物点数
 * @param[IN] Real SeparatePtsX 离散点X数组
 * @param[IN] Real SeparatePtsY 离散点Y数组
 * @param[OUT] Boolean IsCollision 是否碰撞
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite Rect_Collision_Check_For_Boundary.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void rectCollisionCheckForBoundary(const RectCollisionCheckForBoundaryInput_t *input,
                                   RectCollisionCheckForBoundaryOutput_t *output) {
  Real rx1;
  Real ry1;
  Real rx2;
  Real ry2;
  Real rx3;
  Real ry3;
  Real rx4;
  Real ry4;
  Integer lineNum;
  Integer ptsNum;
  Boolean collisionFound;
  Integer i;

  rx1 = input->RectX1;
  ry1 = input->RectY1;
  rx2 = input->RectX2;
  ry2 = input->RectY2;
  rx3 = input->RectX3;
  ry3 = input->RectY3;
  rx4 = input->RectX4;
  ry4 = input->RectY4;
  lineNum = input->BoundaryLineNum;
  ptsNum = input->SeparatePtsNum;

  collisionFound = false;

  if (lineNum > MAX_BOUNDARY_LINES) {
    lineNum = MAX_BOUNDARY_LINES;
  }
  if (ptsNum > MAX_SEPARATE_PTS) {
    ptsNum = MAX_SEPARATE_PTS;
  }

  for (i = 0; i < lineNum; i = i + 1) {
    if (!collisionFound) {
      Real blx1;
      Real bly1;
      Real blx2;
      Real bly2;
      Boolean hit;

      blx1 = input->BoundaryLineX1[i];
      bly1 = input->BoundaryLineY1[i];
      blx2 = input->BoundaryLineX2[i];
      bly2 = input->BoundaryLineY2[i];
      hit = rectCollisionCheckForBoundaryTestLine(
          rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4, blx1, bly1, blx2, bly2);
      if (hit) {
        collisionFound = true;
      }
    }
  }

  for (i = 0; i < ptsNum; i = i + 1) {
    if (!collisionFound) {
      Real spx;
      Real spy;
      Boolean inside;

      spx = input->SeparatePtsX[i];
      spy = input->SeparatePtsY[i];
      inside =
          rectCollisionCheckForBoundaryPtInRect(spx, spy, rx1, ry1, rx2, ry2, rx3, ry3, rx4, ry4);
      if (inside) {
        collisionFound = true;
      }
    }
  }

  output->IsCollision = collisionFound;
}
