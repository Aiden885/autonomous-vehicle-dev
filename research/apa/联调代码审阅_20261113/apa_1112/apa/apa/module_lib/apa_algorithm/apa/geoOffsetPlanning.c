#include "apaPlannerGlobal.h"
#include "apaMathUtils.h"
#include "geoOffsetPlanningTypes.h"
#include "geoDivideTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include <math.h>

static void geoOffsetPlanningInitVccInput(const GeoOffsetPlanningInput_t *input,
                                          Real ptX,
                                          Real ptY,
                                          Real ptTheta,
                                          VehicleCollisionCheckInput_t *vccInput) {
  ApaPlannerGlobalState_t plannerGlobal = APA_PLANNER_GLOBAL_STATE_INIT;
  const ApaPlannerConfig_t *plannerConfig;
  Integer idx;

  *vccInput = (VehicleCollisionCheckInput_t){0};

  apaPlannerGlobalInit(&plannerGlobal);
  plannerConfig = apaPlannerGlobalGetPlanner(&plannerGlobal);

  vccInput->CheckType = VCC_CHECK_TYPE_PLANNING;
  vccInput->SlotClass = VCC_SLOT_CLASS_PARALLEL;
  vccInput->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  vccInput->CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  vehicleCollisionCheckLoadDefaultMargins(vccInput);
  vccInput->SlotPtsValid = true;
  vccInput->SlotPts[0] = input->SltPt1X;
  vccInput->SlotPts[1] = input->SltPt1Y;
  vccInput->SlotPts[2] = input->SltPt2X;
  vccInput->SlotPts[3] = input->SltPt2Y;
  vccInput->SlotPts[4] = input->SltPt3X;
  vccInput->SlotPts[5] = input->SltPt3Y;
  vccInput->SlotPts[6] = input->SltPt4X;
  vccInput->SlotPts[7] = input->SltPt4Y;
  vccInput->VehPosX = ptX;
  vccInput->VehPosY = ptY;
  vccInput->VehPosTheta = ptTheta;
  vccInput->VehLF = plannerConfig->Veh.LF;
  vccInput->VehLB = plannerConfig->Veh.LB;
  vccInput->VehW = plannerConfig->Veh.W;
  vccInput->VehWB = plannerConfig->Veh.WB;

  idx = 0;
  while ((idx < input->BoundaryLineNum) && (idx < OFS_LINE_MAX) && (idx < VCC_MAX_BOUNDARY_LINES)) {
    vccInput->BoundaryLineX1[idx] = input->BoundaryP1X[idx];
    vccInput->BoundaryLineY1[idx] = input->BoundaryP1Y[idx];
    vccInput->BoundaryLineX2[idx] = input->BoundaryP2X[idx];
    vccInput->BoundaryLineY2[idx] = input->BoundaryP2Y[idx];
    idx = idx + 1;
  }
  vccInput->BoundaryLineNum = idx;

  idx = 0;
  while ((idx < input->VirtualBoundaryLineNum) && (idx < OFS_LINE_MAX) &&
         (idx < VCC_MAX_VIRTUAL_LINES)) {
    vccInput->VirtualBoundaryLineX1[idx] = input->VirtualBoundaryP1X[idx];
    vccInput->VirtualBoundaryLineY1[idx] = input->VirtualBoundaryP1Y[idx];
    vccInput->VirtualBoundaryLineX2[idx] = input->VirtualBoundaryP2X[idx];
    vccInput->VirtualBoundaryLineY2[idx] = input->VirtualBoundaryP2Y[idx];
    idx = idx + 1;
  }
  vccInput->VirtualBoundaryLineNum = idx;

  idx = 0;
  while ((idx < input->BoundarySepPtNum) && (idx < OFS_PT_MAX) && (idx < VCC_MAX_SEPARATE_PTS)) {
    vccInput->BoundarySepPtsX[idx] = input->BoundarySepPtX[idx];
    vccInput->BoundarySepPtsY[idx] = input->BoundarySepPtY[idx];
    idx = idx + 1;
  }
  vccInput->BoundarySepPtsNum = idx;

  idx = 0;
  while ((idx < input->StopperPtNum) && (idx < OFS_PT_MAX) && (idx < VCC_MAX_SEPARATE_PTS)) {
    vccInput->SeparateStopperX[idx] = input->StopperPtX[idx];
    vccInput->SeparateStopperY[idx] = input->StopperPtY[idx];
    idx = idx + 1;
  }
  vccInput->SeparateStopperNum = idx;
}

static Integer geoOffsetPlanningVehicleCollisionCheck(Real ptX,
                                                      Real ptY,
                                                      Real ptTheta,
                                                      const GeoOffsetPlanningInput_t *input) {
  VehicleCollisionCheckInput_t vccInput;
  VehicleCollisionCheckOutput_t vccOutput;
  Integer collisionFlag;

  geoOffsetPlanningInitVccInput(input, ptX, ptY, ptTheta, &vccInput);
  vehicleCollisionCheck(&vccInput, &vccOutput);

  if (vccOutput.IsCollision) {
    collisionFlag = 1;
  } else {
    collisionFlag = 0;
  }
  return collisionFlag;
}

static Real
geoOffsetPlanningSignedLateral(Real originX, Real originY, Real heading, Real pointX, Real pointY) {
  return apaMathCross2D(cosf(heading), sinf(heading), pointX - originX, pointY - originY);
}

static void geoOffsetPlanningChooseCorrectCandidate(
    Real x0, Real y0, Real theta0, Real xa, Real ya, Real xb, Real yb, Real *resX, Real *resY) {
  Real cpA = fabsf(geoOffsetPlanningSignedLateral(x0, y0, theta0, xa, ya));
  Real cpB = fabsf(geoOffsetPlanningSignedLateral(x0, y0, theta0, xb, yb));
  if (cpA < cpB) {
    *resX = xa;
    *resY = ya;
  } else {
    *resX = xb;
    *resY = yb;
  }
}

static void geoOffsetPlanningCalcAngleChange(Real xSt,
                                             Real ySt,
                                             Real xEd,
                                             Real yEd,
                                             Real xc,
                                             Real yc,
                                             Real r,
                                             Real initTheta,
                                             Real *dTheta,
                                             Real *newTheta) {
  Real vStX = xSt - xc;
  Real vStY = ySt - yc;
  Real vEdX = xEd - xc;
  Real vEdY = yEd - yc;

  Real dotP = vStX * vEdX + vStY * vEdY;
  Real crossZ = vStX * vEdY - vStY * vEdX;

  Real val = dotP / (r * r);
  if (val > 1.0f)
    val = 1.0f;
  if (val < -1.0f)
    val = -1.0f;

  Real angMag = acosf(val);
  Real dir;
  if ((crossZ > 0.0f)) {
    dir = 1.0f;
  } else {
    if (crossZ < 0.0f) {
      dir = -1.0f;
    } else {
      dir = 0.0f;
    }
  }

  *dTheta = angMag * dir;
  *newTheta = initTheta + *dTheta;
}

static void geoOffsetPlanningOffsetWithTwoCircles(Real x0,
                                                  Real y0,
                                                  Real theta0,
                                                  Real bndX1,
                                                  Real bndY1,
                                                  Real bndX2,
                                                  Real bndY2,
                                                  Real r,
                                                  Integer turn,
                                                  Real *xm,
                                                  Real *ym,
                                                  Real *thtm,
                                                  Real *deltaThtm,
                                                  Real *xe,
                                                  Real *ye,
                                                  Real *thte,
                                                  Real *deltaThte,
                                                  Real *xc1Out,
                                                  Real *yc1Out,
                                                  Real *xc2Out,
                                                  Real *yc2Out) {
  Real xc1 = 0, yc1 = 0;
  Real xc2 = 0, yc2 = 0;
  Real xeTmp = 0, yeTmp = 0;

  if (turn == 1) {
    xc1 = x0 - r * sinf(theta0);
    yc1 = y0 + r * cosf(theta0);
  } else if (turn == -1) {
    xc1 = x0 + r * sinf(theta0);
    yc1 = y0 - r * cosf(theta0);
  }

  if (fabsf(bndX1 - bndX2) < 1e-3f) {
    Real dTmp = (bndX1 - xc1) * (bndX1 - xc1) + yc1 * yc1 - 4.0f * r * r;
    Real delta = 4.0f * (yc1 * yc1 - dTmp);
    if (delta > 0.0f) {
      xc2 = bndX1;
      Real sqD = sqrtf(delta);
      Real yc2_1 = (2.0f * yc1 + sqD) / 2.0f;
      Real yc2_2 = (2.0f * yc1 - sqD) / 2.0f;
      geoOffsetPlanningChooseCorrectCandidate(x0, y0, theta0, xc2, yc2_1, xc2, yc2_2, &xc2, &yc2);

      Real ye_1 = yc2 + r;
      Real ye_2 = yc2 - r;
      geoOffsetPlanningChooseCorrectCandidate(x0, y0, theta0, xc2, ye_1, xc2, ye_2, &xeTmp, &yeTmp);
    }
  } else {
    Real k = (bndY1 - bndY2) / (bndX1 - bndX2);
    Real b = bndY1 - k * bndX1;

    Real A = k * k + 1.0f;
    Real B = 2.0f * (k * (b - yc1) - xc1);
    Real C = (b - yc1) * (b - yc1) + xc1 * xc1 - 4.0f * r * r;
    Real delta = B * B - 4.0f * A * C;

    if (delta > 0.0f) {
      Real sqD = sqrtf(delta);
      Real xc2_1 = (-B + sqD) / (2.0f * A);
      Real yc2_1 = k * xc2_1 + b;
      Real xc2_2 = (-B - sqD) / (2.0f * A);
      Real yc2_2 = k * xc2_2 + b;
      geoOffsetPlanningChooseCorrectCandidate(
          x0, y0, theta0, xc2_1, yc2_1, xc2_2, yc2_2, &xc2, &yc2);

      Real ptOfs = r / sqrtf(1.0f + k * k);
      Real xe_1 = xc2 + ptOfs;
      Real ye_1 = k * xe_1 + b;
      Real xe_2 = xc2 - ptOfs;
      Real ye_2 = k * xe_2 + b;
      geoOffsetPlanningChooseCorrectCandidate(
          x0, y0, theta0, xe_1, ye_1, xe_2, ye_2, &xeTmp, &yeTmp);
    }
  }

  *xc1Out = xc1;
  *yc1Out = yc1;
  *xc2Out = xc2;
  *yc2Out = yc2;
  *xm = (xc1 + xc2) / 2.0f;
  *ym = (yc1 + yc2) / 2.0f;
  *xe = xeTmp;
  *ye = yeTmp;

  geoOffsetPlanningCalcAngleChange(x0, y0, *xm, *ym, xc1, yc1, r, theta0, deltaThtm, thtm);
  geoOffsetPlanningCalcAngleChange(*xm, *ym, *xe, *ye, xc2, yc2, r, *thtm, deltaThte, thte);
}

static void
geoOffsetPlanningGetBoundaries(const GeoOffsetPlanningInput_t *input, Real *frBnd, Real *reBnd) {
  Real M[2];
  Integer d;

  M[0] = cosf(input->SlotPoseTheta);
  M[1] = sinf(input->SlotPoseTheta);

  for (d = 0; d < 4; d = d + 1) {
    frBnd[d] = 0.0f;
    reBnd[d] = 0.0f;
  }

  for (d = 0; d < 2; d = d + 1) {
    Real dir;
    if (d == 0) {
      dir = 1.0f;
    } else {
      dir = -1.0f;
    }
    Real ckDs = 0.0f;
    Boolean found = 0;

    while ((ckDs < 5.0f) && (!found)) {
      Real cPos[3];
      cPos[0] = input->SlotPoseX + dir * ckDs * M[0];
      cPos[1] = input->SlotPoseY + dir * ckDs * M[1];
      cPos[2] = input->SlotPoseTheta;

      Integer isCol = geoOffsetPlanningVehicleCollisionCheck(cPos[0], cPos[1], cPos[2], input);
      if (isCol) {
        Real tgPos[2];
        tgPos[0] = input->SlotPoseX + dir * (ckDs - 0.05f) * M[0];
        tgPos[1] = input->SlotPoseY + dir * (ckDs - 0.05f) * M[1];

        if (dir > 0.0f) {
          frBnd[0] = tgPos[0];
          frBnd[1] = tgPos[1];
          frBnd[2] = tgPos[0] - M[1];
          frBnd[3] = tgPos[1] + M[0];
        } else {
          reBnd[0] = tgPos[0];
          reBnd[1] = tgPos[1];
          reBnd[2] = tgPos[0] - M[1];
          reBnd[3] = tgPos[1] + M[0];
        }
        found = 1;
      } else {
        ckDs = ckDs + 0.05f;
      }
    }
  }

  if (input->ExistWheelStop) {
    reBnd[0] = input->VehPosX;
    reBnd[1] = input->VehPosY;
    reBnd[2] = input->VehPosX - sinf(input->VehPosTheta);
    reBnd[3] = input->VehPosY + cosf(input->VehPosTheta);
  }
}

/**
 * @brief 偏移量修正规划模块
 * @en_name geoOffsetPlanning
 * @cn_name 偏移量修正规划
 * @type module
 * @param[IN] Real VehPosX 车辆位置X
 * @param[IN] Real VehPosY 车辆位置Y
 * @param[IN] Real VehPosTheta 车辆位置航向角
 * @param[IN] Integer OffsetDirection 偏移方向
 * @param[IN] Real AtLeastOffsetDistance 至少偏移距离
 * @param[IN] Real SlotPoseX 车位位置X
 * @param[IN] Real SlotPoseY 车位位置Y
 * @param[IN] Real SlotPoseTheta 车位位置航向角
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Real Radius 规划的转弯半径
 * @param[IN] Integer ExistWheelStop 是否存在轮挡
 * @param[IN] Integer BoundaryLineNum 真实边界线数量
 * @param[IN] Real[16] BoundaryP1X 真实边界点1X
 * @param[IN] Real[16] BoundaryP1Y 真实边界点1Y
 * @param[IN] Real[16] BoundaryP2X 真实边界点2X
 * @param[IN] Real[16] BoundaryP2Y 真实边界点2Y
 * @param[IN] Integer VirtualBoundaryLineNum 虚拟边界线数量
 * @param[IN] Real[16] VirtualBoundaryP1X 虚拟边界点1X
 * @param[IN] Real[16] VirtualBoundaryP1Y 虚拟边界点1Y
 * @param[IN] Real[16] VirtualBoundaryP2X 虚拟边界点2X
 * @param[IN] Real[16] VirtualBoundaryP2Y 虚拟边界点2Y
 * @param[IN] Integer BoundarySepPtNum 真实离散点数量
 * @param[IN] Real[32] BoundarySepPtX 真实离散点X
 * @param[IN] Real[32] BoundarySepPtY 真实离散点Y
 * @param[IN] Integer StopperPtNum 挡停点数量
 * @param[IN] Real[32] StopperPtX 挡停点X
 * @param[IN] Real[32] StopperPtY 挡停点Y
 * @param[OUT] Integer AnchorsNum 规划出锚点数
 * @param[OUT] Real[20] AnchorX X坐标
 * @param[OUT] Real[20] AnchorY Y坐标
 * @param[OUT] Real[20] AnchorTheta 航向角
 * @param[OUT] Integer[20] AnchorTurnDir 转弯方向
 * @param[OUT] Integer[20] AnchorGoDir 行驶方向
 * @param[OUT] Real[20] AnchorDTheta 变化角度
 * @param[OUT] Real[20] AnchorCurv 曲率
 * @param[OUT] Real[20] AnchorCcX 圆心X
 * @param[OUT] Real[20] AnchorCcY 圆心Y
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 偏移修正
 * @source matlab_rewrite geo_offset_planning.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */

void geoOffsetPlanning(const GeoOffsetPlanningInput_t *input, GeoOffsetPlanningOutput_t *output) {
  Real frBnd[4] = {0};
  Real reBnd[4] = {0};

  *output = (GeoOffsetPlanningOutput_t){0};

  geoOffsetPlanningGetBoundaries(input, frBnd, reBnd);

  Real rWork = input->Radius;
  if (rWork < 0.001f)
    rWork = 0.001f;
  Real curve = 1.0f / rWork;

  Real xSt = input->VehPosX;
  Real ySt = input->VehPosY;
  Real tSt = input->VehPosTheta;

  Real d2Fr = apaMathCross2D(frBnd[2] - frBnd[0],
                             frBnd[3] - frBnd[1],
                             input->VehPosX - frBnd[0],
                             input->VehPosY - frBnd[1]);
  Real d2Re = apaMathCross2D(reBnd[2] - reBnd[0],
                             reBnd[3] - reBnd[1],
                             input->VehPosX - reBnd[0],
                             input->VehPosY - reBnd[1]);
  Integer drvDir;
  if ((fabsf(d2Fr) > fabsf(d2Re))) {
    drvDir = 1;
  } else {
    drvDir = -1;
  }

  Integer i = 0;
  Boolean stopFlag = false;
  while ((i < OFS_ANCHOR_MAX) && (!stopFlag)) {
    Real *bnd;
    if (drvDir == 1) {
      bnd = frBnd;
    } else {
      bnd = reBnd;
    }

    Real xM = 0, yM = 0, tM = 0, dtM = 0;
    Real xE = 0, yE = 0, tE = 0, dtE = 0;
    Real xc1 = 0, yc1 = 0, xc2 = 0, yc2 = 0;

    geoOffsetPlanningOffsetWithTwoCircles(xSt,
                                          ySt,
                                          tSt,
                                          bnd[0],
                                          bnd[1],
                                          bnd[2],
                                          bnd[3],
                                          input->Radius,
                                          input->OffsetDirection,
                                          &xM,
                                          &yM,
                                          &tM,
                                          &dtM,
                                          &xE,
                                          &yE,
                                          &tE,
                                          &dtE,
                                          &xc1,
                                          &yc1,
                                          &xc2,
                                          &yc2);

    output->AnchorX[i] = xSt;
    output->AnchorY[i] = ySt;
    output->AnchorTheta[i] = tSt;
    output->AnchorTurnDir[i] = input->OffsetDirection;
    output->AnchorGoDir[i] = drvDir;
    output->AnchorDTheta[i] = dtM;
    output->AnchorCurv[i] = (Real)input->OffsetDirection * curve;
    output->AnchorCcX[i] = xc1;
    output->AnchorCcY[i] = yc1;

    output->AnchorX[i + 1] = xM;
    output->AnchorY[i + 1] = yM;
    output->AnchorTheta[i + 1] = tM;
    output->AnchorTurnDir[i + 1] = -input->OffsetDirection;
    output->AnchorGoDir[i + 1] = drvDir;
    output->AnchorDTheta[i + 1] = dtE;
    output->AnchorCurv[i + 1] = (Real)(-input->OffsetDirection) * curve;
    output->AnchorCcX[i + 1] = xc2;
    output->AnchorCcY[i + 1] = yc2;

    xSt = xE;
    ySt = yE;
    tSt = tE;

    if (drvDir == -1) {
      Real dist = geoOffsetPlanningSignedLateral(xE, yE, tE, input->VehPosX, input->VehPosY);
      if (fabsf(dist) > input->AtLeastOffsetDistance) {
        if ((i + 2) < OFS_ANCHOR_MAX) {
          output->AnchorX[i + 2] = xE;
          output->AnchorY[i + 2] = yE;
          output->AnchorTheta[i + 2] = tE;
          output->AnchorTurnDir[i + 2] = 0;
          output->AnchorGoDir[i + 2] = 0;
          output->AnchorDTheta[i + 2] = 0.0f;
          output->AnchorCurv[i + 2] = 0.0f;
          output->AnchorCcX[i + 2] = 0.0f;
          output->AnchorCcY[i + 2] = 0.0f;
          output->AnchorsNum = i + 3;
        } else {
          output->AnchorsNum = OFS_ANCHOR_MAX;
        }
        stopFlag = true;
      }
    }
    if (!stopFlag) {
      drvDir = -drvDir;
      i = i + 2;
    }
  }
}
