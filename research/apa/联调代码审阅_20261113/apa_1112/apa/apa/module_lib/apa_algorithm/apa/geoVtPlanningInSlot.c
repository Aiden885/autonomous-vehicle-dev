#include "geoVtPlanningInSlotTypes.h"
#include "apaVehicleDefaults.h"
#include "apaMod2piTypes.h"
#include "geoCircleIntrvCheckTypes.h"
#include <math.h>

#ifndef GEO_VT_PLANNING_DEFAULT_WHEELBASE
#define GEO_VT_PLANNING_DEFAULT_WHEELBASE APA_VEHICLE_WHEELBASE_M
#endif

static inline void geoVtPlanningInSlotInitCollisionInput(VehicleCollisionCheckInput_t *vccIn,
                                                         const GeoVtPlanningInSlotInput_t *input) {
  Integer stopperNum;
  Integer stopperIndex;
  if (input->UseVccTemplate) {
    *vccIn = input->VccTemplate;
  } else {
    *vccIn = (VehicleCollisionCheckInput_t){0};
    vccIn->SlotPtsValid = true;
    vccIn->SlotPts[0] = input->SlotPointsX[0];
    vccIn->SlotPts[1] = input->SlotPointsY[0];
    vccIn->SlotPts[2] = input->SlotPointsX[1];
    vccIn->SlotPts[3] = input->SlotPointsY[1];
    vccIn->SlotPts[4] = input->SlotPointsX[2];
    vccIn->SlotPts[5] = input->SlotPointsY[2];
    vccIn->SlotPts[6] = input->SlotPointsX[3];
    vccIn->SlotPts[7] = input->SlotPointsY[3];
    stopperNum = input->SeparateStopperNum;
    if (stopperNum < 0) {
      stopperNum = 0;
    }
    if (stopperNum > VCC_MAX_SEPARATE_PTS) {
      stopperNum = VCC_MAX_SEPARATE_PTS;
    }
    vccIn->SeparateStopperNum = stopperNum;
    for (stopperIndex = 0; stopperIndex < stopperNum; stopperIndex = stopperIndex + 1) {
      vccIn->SeparateStopperX[stopperIndex] = input->SeparateStopperX[stopperIndex];
      vccIn->SeparateStopperY[stopperIndex] = input->SeparateStopperY[stopperIndex];
    }
  }
  if (!vccIn->SlotPtsValid) {
    vccIn->SlotPtsValid = true;
    vccIn->SlotPts[0] = input->SlotPointsX[0];
    vccIn->SlotPts[1] = input->SlotPointsY[0];
    vccIn->SlotPts[2] = input->SlotPointsX[1];
    vccIn->SlotPts[3] = input->SlotPointsY[1];
    vccIn->SlotPts[4] = input->SlotPointsX[2];
    vccIn->SlotPts[5] = input->SlotPointsY[2];
    vccIn->SlotPts[6] = input->SlotPointsX[3];
    vccIn->SlotPts[7] = input->SlotPointsY[3];
  }
  vccIn->CheckType = VCC_CHECK_TYPE_PLANNING;
  vccIn->SlotClass = VCC_SLOT_CLASS_VERTICAL;
  vccIn->CollisionModel = VCC_COLLISION_MODEL_EIGHT_EDGES;
  vccIn->CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  vccIn->VehLF = input->VehLF;
  vccIn->VehLB = input->VehLB;
  vccIn->VehW = input->VehW;
  if (vccIn->VehWB <= 0.0f) {
    vccIn->VehWB = GEO_VT_PLANNING_DEFAULT_WHEELBASE;
  }
  if (vccIn->VehLL <= 0.0f) {
    vccIn->VehLL = input->VehLF + input->VehLB;
  }
  if (!input->UseVccTemplate) {
    vehicleCollisionCheckLoadDefaultMargins(vccIn);
  }
}

static inline Boolean geoVtPlanningInSlotCheckWaypointCollision(VehicleCollisionCheckInput_t *vccIn,
                                                                Real wpX,
                                                                Real wpY,
                                                                Real wpTheta) {
  VehicleCollisionCheckOutput_t vccOut;
  vccIn->VehPosX = wpX;
  vccIn->VehPosY = wpY;
  vccIn->VehPosTheta = wpTheta;
  vehicleCollisionCheck(vccIn, &vccOut);
  return vccOut.IsCollision;
}

static inline Boolean
geoVtPlanningInSlotCheckArcCollision(const GeoVtPlanningInSlotOutput_t *output,
                                     const VehicleCollisionCheckInput_t *vccIn,
                                     const GeoVtPlanningInSlotInput_t *input,
                                     Real r1,
                                     Real r2,
                                     Real r3) {
  Boolean arcColl = false;
  Integer ai;
  Integer bi;
  Integer pi;
  for (ai = 0; ai < 2; ai = ai + 1) {
    GeoCircleIntrvCheckInput_t arcIn = {0};
    GeoCircleIntrvCheckOutput_t arcOut;
    arcIn.CptX = output->CentersX[ai];
    arcIn.CptY = output->CentersY[ai];
    if (ai == 0) {
      arcIn.Radius = r1;
    } else if (ai == 1) {
      arcIn.Radius = r2;
    } else {
      arcIn.Radius = r3;
    }
    arcIn.Pt1X = output->AnchorsX[ai];
    arcIn.Pt1Y = output->AnchorsY[ai];
    arcIn.Pt1Theta = output->AnchorsTheta[ai];
    arcIn.Pt2X = output->AnchorsX[ai + 1];
    arcIn.Pt2Y = output->AnchorsY[ai + 1];
    arcIn.Pt2Theta = output->AnchorsTheta[ai + 1];
    arcIn.GoDir = output->GoDirs[ai];
    arcIn.TrnDir = output->TurnDirs[ai];
    arcIn.CheckStep = 0.2f;
    arcIn.VehWidth = input->VehW;
    arcIn.VehFrontOffset = input->VehLF;
    arcIn.VehRearOffset = input->VehLB;
    arcIn.UsePlanningCheck = true;

    arcIn.DisableSlotBoundaryFallback = true;
    arcIn.UseCollisionContract = false;
    arcIn.ParkingOutFlag = vccIn->ParkingOutFlag;
    arcIn.SlotP1X = vccIn->SlotPts[0];
    arcIn.SlotP1Y = vccIn->SlotPts[1];
    arcIn.SlotP2X = vccIn->SlotPts[2];
    arcIn.SlotP2Y = vccIn->SlotPts[3];
    arcIn.SlotP3X = vccIn->SlotPts[4];
    arcIn.SlotP3Y = vccIn->SlotPts[5];
    arcIn.SlotP4X = vccIn->SlotPts[6];
    arcIn.SlotP4Y = vccIn->SlotPts[7];
    arcIn.SlotClass = vccIn->SlotClass;
    arcIn.BoundaryLineNum = vccIn->BoundaryLineNum;
    if (arcIn.BoundaryLineNum > GEO_BOUNDARY_LINE_MAX) {
      arcIn.BoundaryLineNum = GEO_BOUNDARY_LINE_MAX;
    }
    for (bi = 0; bi < arcIn.BoundaryLineNum; bi = bi + 1) {
      arcIn.BoundaryLineX1[bi] = vccIn->BoundaryLineX1[bi];
      arcIn.BoundaryLineY1[bi] = vccIn->BoundaryLineY1[bi];
      arcIn.BoundaryLineX2[bi] = vccIn->BoundaryLineX2[bi];
      arcIn.BoundaryLineY2[bi] = vccIn->BoundaryLineY2[bi];
    }
    arcIn.VirtualBoundaryLineNum = vccIn->VirtualBoundaryLineNum;
    if (arcIn.VirtualBoundaryLineNum > VCC_MAX_VIRTUAL_LINES) {
      arcIn.VirtualBoundaryLineNum = VCC_MAX_VIRTUAL_LINES;
    }
    for (bi = 0; bi < arcIn.VirtualBoundaryLineNum; bi = bi + 1) {
      arcIn.VirtualBoundaryLineX1[bi] = vccIn->VirtualBoundaryLineX1[bi];
      arcIn.VirtualBoundaryLineY1[bi] = vccIn->VirtualBoundaryLineY1[bi];
      arcIn.VirtualBoundaryLineX2[bi] = vccIn->VirtualBoundaryLineX2[bi];
      arcIn.VirtualBoundaryLineY2[bi] = vccIn->VirtualBoundaryLineY2[bi];
    }
    arcIn.BoundarySepPtNum = vccIn->BoundarySepPtsNum;
    if (arcIn.BoundarySepPtNum > VCC_MAX_SEPARATE_PTS) {
      arcIn.BoundarySepPtNum = VCC_MAX_SEPARATE_PTS;
    }
    for (pi = 0; pi < arcIn.BoundarySepPtNum; pi = pi + 1) {
      arcIn.BoundarySepPtX[pi] = vccIn->BoundarySepPtsX[pi];
      arcIn.BoundarySepPtY[pi] = vccIn->BoundarySepPtsY[pi];
    }
    arcIn.StopperPtNum = vccIn->SeparateStopperNum;
    if (arcIn.StopperPtNum > VCC_MAX_SEPARATE_PTS) {
      arcIn.StopperPtNum = VCC_MAX_SEPARATE_PTS;
    }
    for (pi = 0; pi < arcIn.StopperPtNum; pi = pi + 1) {
      arcIn.StopperPtX[pi] = vccIn->SeparateStopperX[pi];
      arcIn.StopperPtY[pi] = vccIn->SeparateStopperY[pi];
    }
    arcIn.SlotTrackStopperNum = vccIn->SlotTrackStopperNum;
    if (arcIn.SlotTrackStopperNum > VCC_MAX_TRACK_STOPPER) {
      arcIn.SlotTrackStopperNum = VCC_MAX_TRACK_STOPPER;
    }
    for (bi = 0; bi < arcIn.SlotTrackStopperNum; bi = bi + 1) {
      arcIn.SlotTrackStopperX1[bi] = vccIn->SlotTrackStopperX1[bi];
      arcIn.SlotTrackStopperY1[bi] = vccIn->SlotTrackStopperY1[bi];
      arcIn.SlotTrackStopperX2[bi] = vccIn->SlotTrackStopperX2[bi];
      arcIn.SlotTrackStopperY2[bi] = vccIn->SlotTrackStopperY2[bi];
    }
    geoCircleIntrvCheck(&arcIn, &arcOut);
    if (arcOut.IntrvFlg != 0) {
      arcColl = true;
    }
  }
  return arcColl;
}

static inline void geoVtPlanningInSlotCalcDirsAndLengths(GeoVtPlanningInSlotOutput_t *output,
                                                         Integer sgn,
                                                         Real startX,
                                                         Real startY,
                                                         Real startTheta,
                                                         Real wp2X,
                                                         Real wp2Y,
                                                         Real wp2Theta,
                                                         Real wp3X,
                                                         Real wp3Y,
                                                         Real wp3Theta,
                                                         Real originX,
                                                         Real originY,
                                                         Real relativeP3Theta,
                                                         Real r1,
                                                         Real r2,
                                                         Real r3) {
  ApaMod2piInput_t modIn;
  ApaMod2piOutput_t modOut;
  Real dp0X = wp2X - startX;
  Real dp0Y = wp2Y - startY;
  Real dp1X = wp3X - wp2X;
  Real dp1Y = wp3Y - wp2Y;
  Real dp2X = originX - wp3X;
  Real dp2Y = originY - wp3Y;

  Real dirDot;

  dirDot = cosf(startTheta) * dp0X + sinf(startTheta) * dp0Y;
  if (dirDot > 0.0f) {
    output->GoDirs[0] = 1;
  } else if (dirDot < 0.0f) {
    output->GoDirs[0] = -1;
  } else {
    output->GoDirs[0] = 0;
  }
  dirDot = cosf(wp2Theta) * dp1X + sinf(wp2Theta) * dp1Y;
  if (dirDot > 0.0f) {
    output->GoDirs[1] = 1;
  } else if (dirDot < 0.0f) {
    output->GoDirs[1] = -1;
  } else {
    output->GoDirs[1] = 0;
  }
  dirDot = cosf(wp3Theta) * dp2X + sinf(wp3Theta) * dp2Y;
  if (dirDot > 0.0f) {
    output->GoDirs[2] = 1;
  } else if (dirDot < 0.0f) {
    output->GoDirs[2] = -1;
  } else {
    output->GoDirs[2] = 0;
  }

  output->TurnDirs[0] = sgn;
  output->TurnDirs[1] = -sgn;
  output->TurnDirs[2] = -sgn;

  modIn.Angle = wp2Theta - startTheta;
  apaMod2pi(&modIn, &modOut);
  output->DThetas[0] = modOut.Result;
  modIn.Angle = wp3Theta - wp2Theta;
  apaMod2pi(&modIn, &modOut);
  output->DThetas[1] = modOut.Result;
  modIn.Angle = APA_HALF_PI - relativeP3Theta;
  apaMod2pi(&modIn, &modOut);
  output->DThetas[2] = modOut.Result;

  output->Lengths[0] = fabsf(output->DThetas[0]) * r1;
  output->Lengths[1] = fabsf(output->DThetas[1]) * r2;
  output->Lengths[2] = fabsf(output->DThetas[2]) * r3;
}

/**
 * @file geoVtPlanningInSlot.c
 * @brief 槽内 VT 规划
 * @en_name geoVtPlanningInSlot
 * @cn_name 槽内 VT 规划
 * @type module
 * @param[IN] GeoVtPlanningInSlotInput_t Input 槽内VT规划输入
 * @param[OUT] GeoVtPlanningInSlotOutput_t Output 槽内VT规划输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 几何规划
 * @source matlab_rewrite geo_vt_planning_in_slot.m
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
void geoVtPlanningInSlot(const GeoVtPlanningInSlotInput_t *input,
                         GeoVtPlanningInSlotOutput_t *output) {
  Real startX;
  Real startY;
  Real startTheta;
  Real goalX;
  Real goalY;
  Real goalTheta;
  Real r1;
  Real r2;
  Integer ci;

  startX = input->StartX;
  startY = input->StartY;
  startTheta = input->StartTheta;
  goalX = input->GoalX;
  goalY = input->GoalY;
  goalTheta = input->GoalTheta;
  r1 = input->R1;
  r2 = input->R2;

  output->Success = false;
  for (ci = 0; ci < 3; ci = ci + 1) {
    output->AnchorsX[ci] = 0.0f;
    output->AnchorsY[ci] = 0.0f;
    output->AnchorsTheta[ci] = 0.0f;
    output->CentersX[ci] = 0.0f;
    output->CentersY[ci] = 0.0f;
    output->Curves[ci] = 0.0f;
    output->GoDirs[ci] = 0;
    output->TurnDirs[ci] = 0;
    output->DThetas[ci] = 0.0f;
    output->Lengths[ci] = 0.0f;
  }

  Real originX;
  Real originY;
  Real originTheta;

  originX = goalX;
  originY = goalY;
  originTheta = goalTheta - APA_HALF_PI;

  Real dx;
  Real dy;
  Real cosOri;
  Real sinOri;
  Real p1X;
  Real p1Y;
  Real p1Theta;

  dx = startX - originX;
  dy = startY - originY;
  cosOri = cosf(originTheta);
  sinOri = sinf(originTheta);
  p1X = cosOri * dx + sinOri * dy;
  p1Y = -sinOri * dx + cosOri * dy;

  ApaMod2piInput_t modIn;
  ApaMod2piOutput_t modOut;
  modIn.Angle = startTheta - originTheta;
  apaMod2pi(&modIn, &modOut);
  p1Theta = modOut.Result;

  Real cosP1;
  Real sinP1;
  Integer sgn;

  cosP1 = cosf(p1Theta);
  sinP1 = sinf(p1Theta);
  if (cosP1 > 0.0f) {
    sgn = 1;
  } else {
    sgn = -1;
  }

  Real t1X;
  Real t1Y;
  Real c1X;
  Real c1Y;

  t1X = -sinP1;
  t1Y = cosP1;
  c1X = p1X + ((Real)sgn) * r1 * t1X;
  c1Y = p1Y + ((Real)sgn) * r1 * t1Y;

  if (fabsf(c1X) > r1) {
    sgn = -sgn;
    c1X = p1X + ((Real)sgn) * r1 * t1X;
    c1Y = p1Y + ((Real)sgn) * r1 * t1Y;
  }

  Real maxTheta;
  Real minTheta;
  Real acosArg;

  acosArg = (r2 + fabsf(c1X)) / (r1 + r2);
  if (acosArg > 1.0f) {
    acosArg = 1.0f;
  }
  if (acosArg < -1.0f) {
    acosArg = -1.0f;
  }
  maxTheta = acosf(acosArg);
  minTheta = atan2f(-c1Y, ((Real)sgn) * r2 - c1X);

  if (sgn == -1) {
    maxTheta = APA_PI - maxTheta;
    if (minTheta < 0.0f) {
      minTheta = minTheta + APA_TWO_PI;
    }
  }

  Real startThetaSearch;
  Real endThetaSearch;
  Integer candidatesNum;
  Integer canIdx;

  startThetaSearch = maxTheta - ((Real)sgn) * 1e-3f;
  endThetaSearch = minTheta;
  candidatesNum = 10;

  for (canIdx = 0; canIdx < candidatesNum; canIdx = canIdx + 1) {
    Real ratio;
    Real curTheta;
    Real uvC1C2X;
    Real uvC1C2Y;
    Real c2X;
    Real c2Y;
    Real r2xX;
    Real c2R2xX;
    Real c2R2xY;
    Real mX;
    Real mY;
    Real k1;
    Real k;
    Real r3;
    Real c3X;
    Real uvC3C2X;
    Real uvC3C2Y;
    Real normC3C2;

    ratio = (Real)canIdx / (Real)(candidatesNum - 1);
    curTheta = startThetaSearch + ratio * (endThetaSearch - startThetaSearch);

    uvC1C2X = cosf(curTheta);
    uvC1C2Y = sinf(curTheta);
    c2X = c1X + (r1 + r2) * uvC1C2X;
    c2Y = c1Y + (r1 + r2) * uvC1C2Y;

    r2xX = ((Real)sgn) * r2;
    c2R2xX = c2X - r2xX;
    c2R2xY = c2Y;
    mX = (c2X + r2xX) * 0.5f;
    mY = c2Y * 0.5f;

    k1 = c2R2xY / c2R2xX;
    k = -1.0f / k1;
    r3 = fabsf(-mY / k + mX);
    c3X = ((Real)sgn) * r3;

    uvC3C2X = c2X - c3X;
    uvC3C2Y = c2Y;
    normC3C2 = sqrtf(uvC3C2X * uvC3C2X + uvC3C2Y * uvC3C2Y);
    uvC3C2X = uvC3C2X / normC3C2;
    uvC3C2Y = uvC3C2Y / normC3C2;

    Real p2X;
    Real p2Y;
    Real p2Theta;
    Real p3X;
    Real p3Y;
    Real p3Theta;

    p2X = (c1X + c2X) * 0.5f;
    p2Y = (c1Y + c2Y) * 0.5f;
    p2Theta = atan2f(uvC1C2Y, uvC1C2X) + ((Real)sgn) * APA_HALF_PI;

    p3X = c3X + r3 * uvC3C2X;
    p3Y = r3 * uvC3C2Y;
    p3Theta = atan2f(uvC3C2Y, uvC3C2X) - ((Real)sgn) * APA_HALF_PI;

    Real wp3X;
    Real wp3Y;
    Real wp3Theta;
    Real wp2X;
    Real wp2Y;
    Real wp2Theta;

    wp3X = cosOri * p3X - sinOri * p3Y + originX;
    wp3Y = sinOri * p3X + cosOri * p3Y + originY;
    modIn.Angle = p3Theta + originTheta;
    apaMod2pi(&modIn, &modOut);
    wp3Theta = modOut.Result;

    wp2X = cosOri * p2X - sinOri * p2Y + originX;
    wp2Y = sinOri * p2X + cosOri * p2Y + originY;
    modIn.Angle = p2Theta + originTheta;
    apaMod2pi(&modIn, &modOut);
    wp2Theta = modOut.Result;

    Boolean collFlg;
    collFlg = false;

    VehicleCollisionCheckInput_t vccIn;
    geoVtPlanningInSlotInitCollisionInput(&vccIn, input);

    if (geoVtPlanningInSlotCheckWaypointCollision(&vccIn, wp3X, wp3Y, wp3Theta)) {
      collFlg = true;
    }

    if (!collFlg) {
      if (geoVtPlanningInSlotCheckWaypointCollision(&vccIn, wp2X, wp2Y, wp2Theta)) {
        collFlg = true;
      }
    }

    if (!collFlg) {
      output->AnchorsX[0] = startX;
      output->AnchorsY[0] = startY;
      output->AnchorsTheta[0] = startTheta;
      output->AnchorsX[1] = wp2X;
      output->AnchorsY[1] = wp2Y;
      output->AnchorsTheta[1] = wp2Theta;
      output->AnchorsX[2] = wp3X;
      output->AnchorsY[2] = wp3Y;
      output->AnchorsTheta[2] = wp3Theta;

      output->CentersX[0] = cosOri * c1X - sinOri * c1Y + originX;
      output->CentersY[0] = sinOri * c1X + cosOri * c1Y + originY;
      output->CentersX[1] = cosOri * c2X - sinOri * c2Y + originX;
      output->CentersY[1] = sinOri * c2X + cosOri * c2Y + originY;
      output->CentersX[2] = cosOri * c3X + originX;
      output->CentersY[2] = sinOri * c3X + originY;

      output->Curves[0] = ((Real)sgn) / r1;
      output->Curves[1] = -((Real)sgn) / r2;
      output->Curves[2] = -((Real)sgn) / r3;

      geoVtPlanningInSlotCalcDirsAndLengths(output,
                                            sgn,
                                            startX,
                                            startY,
                                            startTheta,
                                            wp2X,
                                            wp2Y,
                                            wp2Theta,
                                            wp3X,
                                            wp3Y,
                                            wp3Theta,
                                            originX,
                                            originY,
                                            p3Theta,
                                            r1,
                                            r2,
                                            r3);

      if (!geoVtPlanningInSlotCheckArcCollision(output, &vccIn, input, r1, r2, r3)) {
        output->Success = true;
      }
    }

    if (output->Success) {
      canIdx = candidatesNum;
    }
  }
}
