#include "apaCoordUtils.h"
#include "apaParkinClearance.h"
#include "apaMathUtils.h"
#include "apaPlannerConfig.h"
#include "apaSlotTypes.h"
#include "vehicleCollisionCheckCore.h"
#include <math.h>

#define VCC_POLYGON_EDGE_COUNT 8
#define VCC_DEFAULT_FRONT_TRI_LAT APA_VEHICLE_FRONT_CHAMFER_LAT_M
#define VCC_DEFAULT_FRONT_TRI_LON APA_VEHICLE_FRONT_CHAMFER_LON_M
#define VCC_DEFAULT_BACK_TRI_LAT APA_VEHICLE_REAR_CHAMFER_LAT_M
#define VCC_DEFAULT_BACK_TRI_LON APA_VEHICLE_REAR_CHAMFER_LON_M
#define VCC_GEOM_EPSILON 1.0e-5f
#define VCC_PI_VALUE 3.14159265358979323846f
#define VCC_DEG_TO_RAD_FACTOR 0.017453292519943295f
#define VCC_RAD_TO_DEG_FACTOR 57.29577951308232f
#define VCC_LOW_ANGLE_DEG 28.0f
#define VCC_UP_ANGLE_DEG 32.0f
#define VCC_ANGLE_GAP_DEG 4.0f

void vehicleCollisionCheckLoadMargins(VehicleCollisionCheckInput_t *input,
                                      const ApaPlannerVehConfig_t *vehicle) {
  input->UseAutoMargin = true;
  input->FrontSafeMargin = vehicle->FrontSafeMargin;
  input->RearSafeMargin = vehicle->RearSafeMargin;
  input->SideSafeMargin = vehicle->SideSafeMargin;
  input->FrontSafeMarginNearSlotVt = vehicle->FrontSafeMarginNearSlotVt;
  input->RearSafeMarginNearSlotVt = vehicle->RearSafeMarginNearSlotVt;
  input->SideSafeMarginNearSlotVt = vehicle->SideSafeMarginNearSlotVt;
  input->FrontSafeMarginAwaySlotVt = vehicle->FrontSafeMarginAwaySlotVt;
  input->RearSafeMarginAwaySlotVt = vehicle->RearSafeMarginAwaySlotVt;
  input->SideSafeMarginAwaySlotVt = vehicle->SideSafeMarginAwaySlotVt;
  input->FrontSafeMarginNearSlotHt = vehicle->FrontSafeMarginNearSlotHt;
  input->RearSafeMarginNearSlotHt = vehicle->RearSafeMarginNearSlotHt;
  input->SideSafeMarginNearSlotHt = vehicle->SideSafeMarginNearSlotHt;
  input->FrontSafeMarginAwaySlotHt = vehicle->FrontSafeMarginAwaySlotHt;
  input->RearSafeMarginAwaySlotHt = vehicle->RearSafeMarginAwaySlotHt;
  input->SideSafeMarginAwaySlotHt = vehicle->SideSafeMarginAwaySlotHt;
  if (vehicleCollisionCheckUsesCrossSlotMargins(input->SlotClass)) {
    input->WhlBseLongMargin = vehicle->WhlBseLongMarginVt;
  } else {
    input->WhlBseLongMargin = vehicle->WhlBseLongMarginHt;
  }
  input->WhlBseLatMargin = vehicle->WhlBseLatMargin;
  input->FrontTriLat = vehicle->FrontTriLat;
  input->FrontTriLon = vehicle->FrontTriLon;
  input->BackTriLat = vehicle->BackTriLat;
  input->BackTriLon = vehicle->BackTriLon;
}

void vehicleCollisionCheckLoadDefaultMargins(VehicleCollisionCheckInput_t *input) {
  ApaPlannerConfig_t config;
  apaPlannerConfigInitDefault(&config);
  vehicleCollisionCheckLoadMargins(input, &config.Veh);
}

static void vccResetOutput(VehicleCollisionCheckDetailedOutput_t *output) {
  Integer i;

  output->Summary.IsCollision = false;
  output->Summary.CollisionType = VCC_COLLISION_NONE;
  output->Summary.CollisionIndex = 0;
  output->Summary.CollisionDepth = 99.0f;
  output->Summary.CrossLine = 0U;
  output->CollisionPoint[0] = 99.0f;
  output->CollisionPoint[1] = 99.0f;
  output->DeepPointArea = 0U;
  output->Summary.SafeMargin1stBound[0] = 0.0f;
  output->Summary.SafeMargin1stBound[1] = 0.0f;
  output->Summary.SafeMargin1stBound[2] = 0.0f;
  for (i = 0; i < VCC_COLLISION_AREA_COUNT; i = i + 1) {
    output->Summary.CollisionArea[i] = 0U;
  }
}

static Real vccDegToRad(Real deg) {
  return deg * VCC_DEG_TO_RAD_FACTOR;
}

static Real vccRadToDeg(Real rad) {
  return rad * VCC_RAD_TO_DEG_FACTOR;
}

static void vccGroundToVehLocal(Real refX,
                                Real refY,
                                Real refTheta,
                                Real targetX,
                                Real targetY,
                                Real targetTheta,
                                Real *localX,
                                Real *localY,
                                Real *localTheta) {
  Real dx;
  Real dy;
  Real cosT;
  Real sinT;

  dx = targetX - refX;
  dy = targetY - refY;
  cosT = cosf(refTheta);
  sinT = sinf(refTheta);
  *localX = (cosT * dx) + (sinT * dy);
  *localY = (-sinT * dx) + (cosT * dy);
  *localTheta = apaCoordWrapToHalfOpenPi(targetTheta - refTheta);
}

static Real vccAngleToSlot(const Real slotPts[8], Real vehTheta) {
  Real edgeLength[4];
  Real edgeAngle[4];
  Real maxLength;
  Integer maxIndex;
  Integer i;
  Real angleDiff;

  for (i = 0; i < 4; i = i + 1) {
    Integer nextIndex;
    Real x0;
    Real y0;
    Real x1;
    Real y1;
    Real dx;
    Real dy;

    nextIndex = i + 1;
    if (nextIndex >= 4) {
      nextIndex = 0;
    }
    x0 = slotPts[i * 2];
    y0 = slotPts[i * 2 + 1];
    x1 = slotPts[nextIndex * 2];
    y1 = slotPts[nextIndex * 2 + 1];
    dx = x1 - x0;
    dy = y1 - y0;
    edgeLength[i] = sqrtf((dx * dx) + (dy * dy));
    edgeAngle[i] = atan2f(dy, dx);
  }

  maxIndex = 0;
  maxLength = edgeLength[0];
  for (i = 1; i < 4; i = i + 1) {
    if (edgeLength[i] > maxLength) {
      maxLength = edgeLength[i];
      maxIndex = i;
    }
  }

  angleDiff = apaCoordWrapToHalfOpenPi(vehTheta - edgeAngle[maxIndex]);
  if (angleDiff > (0.5f * VCC_PI_VALUE)) {
    angleDiff = angleDiff - VCC_PI_VALUE;
  } else if (angleDiff < (-0.5f * VCC_PI_VALUE)) {
    angleDiff = angleDiff + VCC_PI_VALUE;
  }
  return angleDiff;
}

typedef struct {
  Boolean Near;
  Boolean In;
} VccSlotFlags_t;

static VccSlotFlags_t vccResolveSlotFlagsFromPts(const VehicleCollisionCheckInput_t *input) {
  VccSlotFlags_t flags = {input->VehNearSlotFlag, input->VehInSlotFlag};
  ApaPlannerVehConfig_t vehConfig;
  Real vehPos[3];
  Real slotBounds[4];
  Boolean canResolve;

  canResolve = input->SlotPtsValid;
  if (input->VehW <= VCC_GEOM_EPSILON) {
    canResolve = false;
  }
  if (input->VehLF <= VCC_GEOM_EPSILON) {
    canResolve = false;
  }
  if (input->VehWB <= VCC_GEOM_EPSILON) {
    canResolve = false;
  }

  if (canResolve) {
    vehConfig = (ApaPlannerVehConfig_t){0};
    vehConfig.W = input->VehW;
    vehConfig.LF = input->VehLF;
    vehConfig.LB = input->VehLB;
    vehConfig.WB = input->VehWB;
    if (input->VehLL > VCC_GEOM_EPSILON) {
      vehConfig.LL = input->VehLL;
    } else {
      vehConfig.LL = input->VehLF + input->VehLB;
    }

    vehPos[0] = input->VehPosX;
    vehPos[1] = input->VehPosY;
    vehPos[2] = input->VehPosTheta;
    apaSlotCalcSlotBounds(input->SlotPts, slotBounds);
    flags.In = apaSlotCalcVehInSlotFlag(vehPos, input->SlotPts, slotBounds, &vehConfig);
    flags.Near = apaSlotCalcVehNearSlotFlag(vehPos, input->SlotPts, slotBounds, &vehConfig);
  }
  return flags;
}

static void vccResolveFirstBoundaryMargins(const VehicleCollisionCheckInput_t *input,
                                           const VccSlotFlags_t *flags,
                                           Real *frontSafeMargin,
                                           Real *rearSafeMargin,
                                           Real *sideSafeMargin,
                                           Real *frontTriLat,
                                           Real *frontTriLon,
                                           Real *backTriLat,
                                           Real *backTriLon) {
  Real fsMargin;
  Real rsMargin;
  Real ssMargin;
  Real deltaToTp;
  Real lowAngleRad;
  Real upAngleRad;
  Boolean isVertical;
  Boolean isGeneralOrStatic;
  Boolean isDynamic;
  Real ftLat;
  Real ftLon;
  Real btLat;
  Real btLon;

  if (input->CheckType == VCC_CHECK_TYPE_PLANNING) {
    *frontSafeMargin = *rearSafeMargin = *sideSafeMargin = APA_PARKIN_PHYSICAL_MARGIN_M;
    *frontTriLat = APA_VEHICLE_FRONT_CHAMFER_LAT_M;
    *frontTriLon = APA_VEHICLE_FRONT_CHAMFER_LON_M;
    *backTriLat = APA_VEHICLE_REAR_CHAMFER_LAT_M;
    *backTriLon = APA_VEHICLE_REAR_CHAMFER_LON_M;
    return;
  }

  isVertical = (vehicleCollisionCheckUsesCrossSlotMargins(input->SlotClass));
  isGeneralOrStatic =
      (input->CheckType == VCC_CHECK_TYPE_GENERAL) || (input->CheckType == VCC_CHECK_TYPE_STATIC);
  isDynamic = (input->CheckType == VCC_CHECK_TYPE_DYNAMIC);
  lowAngleRad = vccDegToRad(VCC_LOW_ANGLE_DEG);
  upAngleRad = vccDegToRad(VCC_UP_ANGLE_DEG);

  if (input->TarPosValid) {
    deltaToTp = apaMathAbs(apaCoordWrapToHalfOpenPi(input->TarPosTheta - input->VehPosTheta));
  } else if (input->SlotPtsValid) {
    deltaToTp = apaMathAbs(vccAngleToSlot(input->SlotPts, input->VehPosTheta));
  } else {
    deltaToTp = 0.0f;
  }

  if (isVertical) {
    if (flags->Near) {
      if (deltaToTp < lowAngleRad) {
        fsMargin = input->FrontSafeMarginNearSlotVt;
        rsMargin = input->RearSafeMarginNearSlotVt;
        ssMargin = input->SideSafeMarginNearSlotVt;
      } else if (deltaToTp > upAngleRad) {
        fsMargin = input->FrontSafeMarginAwaySlotVt;
        rsMargin = input->RearSafeMarginAwaySlotVt;
        ssMargin = input->SideSafeMarginAwaySlotVt;
      } else {
        Real deltaToLow;
        Real angleGapRatio;
        deltaToLow = vccRadToDeg(deltaToTp) - VCC_LOW_ANGLE_DEG;
        angleGapRatio = deltaToLow / VCC_ANGLE_GAP_DEG;
        fsMargin = apaMathLinearInterpolate(
            input->FrontSafeMarginNearSlotVt, input->FrontSafeMarginAwaySlotVt, angleGapRatio);
        rsMargin = apaMathLinearInterpolate(
            input->RearSafeMarginNearSlotVt, input->RearSafeMarginAwaySlotVt, angleGapRatio);
        ssMargin = apaMathLinearInterpolate(
            input->SideSafeMarginNearSlotVt, input->SideSafeMarginAwaySlotVt, angleGapRatio);
      }
    } else {
      fsMargin = input->FrontSafeMarginAwaySlotVt;
      rsMargin = input->RearSafeMarginAwaySlotVt;
      ssMargin = input->SideSafeMarginAwaySlotVt;
    }
  } else {
    if (flags->Near) {
      fsMargin = input->FrontSafeMarginNearSlotHt;
      rsMargin = input->RearSafeMarginNearSlotHt;
      ssMargin = input->SideSafeMarginNearSlotHt;
    } else {
      fsMargin = input->FrontSafeMarginAwaySlotHt;
      rsMargin = input->RearSafeMarginAwaySlotHt;
      ssMargin = input->SideSafeMarginAwaySlotHt;
    }
  }

  if (input->ParkingOutFlag) {
    fsMargin = 0.45f;
    rsMargin = 0.30f;
    ssMargin = 0.17f;
  }

  if (isGeneralOrStatic) {
    if (isVertical) {
      if (flags->Near) {
        if (deltaToTp < lowAngleRad) {
          fsMargin = 0.30f;
          rsMargin = 0.20f;
          ssMargin = 0.08f;
        } else if (deltaToTp > upAngleRad) {
          fsMargin = 0.30f;
          rsMargin = 0.30f;
          ssMargin = 0.25f;
        } else {
          Real ratio;
          ratio = (vccRadToDeg(deltaToTp) - VCC_LOW_ANGLE_DEG) / VCC_ANGLE_GAP_DEG;
          fsMargin = 0.30f;
          rsMargin = apaMathLinearInterpolate(0.20f, 0.30f, ratio);
          ssMargin = apaMathLinearInterpolate(0.08f, 0.25f, ratio);
        }

        if (input->VerticalLastStep) {
          rsMargin = -0.1f;
          ssMargin = -0.3f;
        }
      } else {
        fsMargin = 0.30f;
        rsMargin = 0.30f;
        ssMargin = 0.25f;

        if (input->TarPosValid) {
          Real lonError;
          Real latError;
          Real headError;
          Real localX;
          Real localY;
          Real localTheta;
          vccGroundToVehLocal(input->TarPosX,
                              input->TarPosY,
                              input->TarPosTheta,
                              input->VehPosX,
                              input->VehPosY,
                              input->VehPosTheta,
                              &localX,
                              &localY,
                              &localTheta);
          lonError = apaMathAbs(localX);
          latError = apaMathAbs(localY);
          headError = apaMathAbs(localTheta);
          if ((lonError < 5.5f) && (latError < 0.3f)) {
            if (headError > vccDegToRad(30.0f)) {
              ssMargin = 0.25f;
            } else if (headError > vccDegToRad(20.0f)) {
              ssMargin = 0.20f;
            } else if (headError > vccDegToRad(10.0f)) {
              ssMargin = 0.15f;
            } else {
              ssMargin = 0.10f;
            }
          }
        }
      }
    } else {
      if (flags->Near) {
        fsMargin = 0.20f;
        rsMargin = 0.15f;
        ssMargin = 0.07f;

        if (input->TarPosValid && flags->In) {
          Real headError;
          headError = apaMathAbs(apaCoordWrapToHalfOpenPi(input->TarPosTheta - input->VehPosTheta));
          if (headError < vccDegToRad(12.0f)) {
            fsMargin = 0.28f;
          }
        }
      } else {
        fsMargin = 0.30f;
        rsMargin = 0.20f;
        ssMargin = 0.20f;
      }
    }
    if (input->ParkingOutFlag) {
      fsMargin = 0.25f;
      rsMargin = 0.15f;
      ssMargin = 0.10f;
    }
  } else if (isDynamic) {

    Real offset;
    Real ratio;
    offset = -0.15f;
    ratio = 1.5f;
    fsMargin = apaMathMax(fsMargin - offset, fsMargin * ratio);
    rsMargin = apaMathMax(rsMargin - offset, rsMargin * ratio);
    ssMargin = apaMathMax(ssMargin - offset, ssMargin * ratio);
  }

  if (ssMargin < 0.0f) {
    Real expandLen;
    Real expandWid;
    Real refLen;
    Real refWid;
    refLen = input->VehLL;
    refWid = input->VehW;
    if (refLen < VCC_GEOM_EPSILON) {
      refLen = input->VehLF + input->VehLB;
    }
    if (refWid < VCC_GEOM_EPSILON) {
      refWid = 1.0f;
    }
    expandLen = refLen + fsMargin + rsMargin;
    expandWid = refWid + (2.0f * ssMargin);
    if (input->FrontTriLat > VCC_GEOM_EPSILON) {
      ftLat = (input->FrontTriLat / refWid) * expandWid;
    } else {
      ftLat = VCC_DEFAULT_FRONT_TRI_LAT;
    }
    if (input->FrontTriLon > VCC_GEOM_EPSILON) {
      ftLon = (input->FrontTriLon / refLen) * expandLen;
    } else {
      ftLon = VCC_DEFAULT_FRONT_TRI_LON;
    }
    if (input->BackTriLat > VCC_GEOM_EPSILON) {
      btLat = (input->BackTriLat / refWid) * expandWid;
    } else {
      btLat = VCC_DEFAULT_BACK_TRI_LAT;
    }
    if (input->BackTriLon > VCC_GEOM_EPSILON) {
      btLon = (input->BackTriLon / refLen) * expandLen;
    } else {
      btLon = VCC_DEFAULT_BACK_TRI_LON;
    }
  } else {
    if (input->FrontTriLat > VCC_GEOM_EPSILON) {
      ftLat = input->FrontTriLat;
    } else {
      ftLat = VCC_DEFAULT_FRONT_TRI_LAT;
    }
    if (input->FrontTriLon > VCC_GEOM_EPSILON) {
      ftLon = input->FrontTriLon;
    } else {
      ftLon = VCC_DEFAULT_FRONT_TRI_LON;
    }
    if (input->BackTriLat > VCC_GEOM_EPSILON) {
      btLat = input->BackTriLat;
    } else {
      btLat = VCC_DEFAULT_BACK_TRI_LAT;
    }
    if (input->BackTriLon > VCC_GEOM_EPSILON) {
      btLon = input->BackTriLon;
    } else {
      btLon = VCC_DEFAULT_BACK_TRI_LON;
    }
  }

  *frontSafeMargin = fsMargin;
  *rearSafeMargin = rsMargin;
  *sideSafeMargin = ssMargin;
  *frontTriLat = ftLat;
  *frontTriLon = ftLon;
  *backTriLat = btLat;
  *backTriLon = btLon;
}

static void vccResolveWheelBaseMargins(const VehicleCollisionCheckInput_t *input,
                                       const VccSlotFlags_t *flags,
                                       Real *whlBseLongMargin,
                                       Real *whlBseLatMargin) {
  Real longMargin;
  Real latMargin;
  Boolean isVertical;

  if (input->CheckType == VCC_CHECK_TYPE_PLANNING) {
    *whlBseLongMargin = *whlBseLatMargin = APA_PARKIN_PHYSICAL_MARGIN_M;
    return;
  }

  isVertical = (vehicleCollisionCheckUsesCrossSlotMargins(input->SlotClass));

  longMargin = input->WhlBseLongMargin;
  latMargin = input->WhlBseLatMargin;

  if (!isVertical) {
    longMargin = longMargin - 0.08f;
  }
  if (!flags->Near) {
    longMargin = longMargin + 0.2f;
    latMargin = latMargin + 0.2f;
  }
  if (flags->In && isVertical) {
    latMargin = latMargin - 0.15f;
    if (latMargin < 0.0f) {
      latMargin = 0.0f;
    }
  }

  *whlBseLongMargin = longMargin;
  *whlBseLatMargin = latMargin;
}

static void
vccClosestPtOnSeg(Real px, Real py, Real ax, Real ay, Real bx, Real by, Real *cpx, Real *cpy) {
  Real dx;
  Real dy;
  Real t;
  Real lenSq;
  Real pdx;
  Real pdy;

  dx = bx - ax;
  dy = by - ay;
  lenSq = (dx * dx) + (dy * dy);
  if (lenSq < VCC_GEOM_EPSILON) {
    *cpx = ax;
    *cpy = ay;
  } else {
    pdx = px - ax;
    pdy = py - ay;
    t = ((pdx * dx) + (pdy * dy)) / lenSq;
    if (t < 0.0f) {
      t = 0.0f;
    }
    if (t > 1.0f) {
      t = 1.0f;
    }
    *cpx = ax + (t * dx);
    *cpy = ay + (t * dy);
  }
}

static Boolean vccValueBetween(Real value, Real boundA, Real boundB) {
  Boolean between;
  Real minValue;
  Real maxValue;

  if (boundA < boundB) {
    minValue = boundA;
    maxValue = boundB;
  } else {
    minValue = boundB;
    maxValue = boundA;
  }
  between = (value >= (minValue - VCC_GEOM_EPSILON)) && (value <= (maxValue + VCC_GEOM_EPSILON));
  return between;
}

static Boolean vccPointOnSegment(Real ax, Real ay, Real bx, Real by, Real px, Real py) {
  Boolean onLine;
  Boolean inX;
  Boolean inY;
  Boolean onSegment;
  Real cross;

  cross = apaMathCross2D(bx - ax, by - ay, px - ax, py - ay);
  onLine = (fabsf(cross) <= VCC_GEOM_EPSILON);
  inX = vccValueBetween(px, ax, bx);
  inY = vccValueBetween(py, ay, by);
  onSegment = onLine && inX && inY;
  return onSegment;
}

static Boolean
vccSegmentsIntersect(Real ax, Real ay, Real bx, Real by, Real cx, Real cy, Real dx, Real dy) {
  Real c1;
  Real c2;
  Real c3;
  Real c4;
  Boolean straddleAB;
  Boolean straddleCD;
  Boolean endpointHit;
  Boolean intersects;

  c1 = apaMathCross2D(bx - ax, by - ay, cx - ax, cy - ay);
  c2 = apaMathCross2D(bx - ax, by - ay, dx - ax, dy - ay);
  c3 = apaMathCross2D(dx - cx, dy - cy, ax - cx, ay - cy);
  c4 = apaMathCross2D(dx - cx, dy - cy, bx - cx, by - cy);
  straddleAB = (((c1 > VCC_GEOM_EPSILON) && (c2 < -VCC_GEOM_EPSILON)) ||
                ((c1 < -VCC_GEOM_EPSILON) && (c2 > VCC_GEOM_EPSILON)));
  straddleCD = (((c3 > VCC_GEOM_EPSILON) && (c4 < -VCC_GEOM_EPSILON)) ||
                ((c3 < -VCC_GEOM_EPSILON) && (c4 > VCC_GEOM_EPSILON)));
  endpointHit =
      vccPointOnSegment(ax, ay, bx, by, cx, cy) || vccPointOnSegment(ax, ay, bx, by, dx, dy) ||
      vccPointOnSegment(cx, cy, dx, dy, ax, ay) || vccPointOnSegment(cx, cy, dx, dy, bx, by);
  intersects = (straddleAB && straddleCD) || endpointHit;
  return intersects;
}

static Boolean vccPtInRect(Real px,
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
  Boolean allNonNeg;
  Boolean allNonPos;
  Boolean result0;

  c1 = (rx2 - rx1) * (py - ry1) - (ry2 - ry1) * (px - rx1);
  c2 = (rx3 - rx2) * (py - ry2) - (ry3 - ry2) * (px - rx2);
  c3 = (rx4 - rx3) * (py - ry3) - (ry4 - ry3) * (px - rx3);
  c4 = (rx1 - rx4) * (py - ry4) - (ry1 - ry4) * (px - rx4);

  allNonNeg = (c1 >= 0.0f) && (c2 >= 0.0f) && (c3 >= 0.0f) && (c4 >= 0.0f);
  allNonPos = (c1 <= 0.0f) && (c2 <= 0.0f) && (c3 <= 0.0f) && (c4 <= 0.0f);

  if (allNonNeg || allNonPos) {
    result0 = true;
  } else {
    result0 = false;
  }
  return result0;
}

static void vccBuildRect(Real posX,
                         Real posY,
                         Real cosT,
                         Real sinT,
                         Real lf,
                         Real lb,
                         Real halfW,
                         Real *ox1,
                         Real *oy1,
                         Real *ox2,
                         Real *oy2,
                         Real *ox3,
                         Real *oy3,
                         Real *ox4,
                         Real *oy4) {

  *ox1 = cosT * lf - sinT * halfW + posX;
  *oy1 = sinT * lf + cosT * halfW + posY;
  *ox2 = cosT * lf - sinT * (-halfW) + posX;
  *oy2 = sinT * lf + cosT * (-halfW) + posY;
  *ox3 = cosT * (-lb) - sinT * (-halfW) + posX;
  *oy3 = sinT * (-lb) + cosT * (-halfW) + posY;
  *ox4 = cosT * (-lb) - sinT * halfW + posX;
  *oy4 = sinT * (-lb) + cosT * halfW + posY;
}

static void vccBuildRectArray(
    Real posX, Real posY, Real cosT, Real sinT, Real lf, Real lb, Real halfW, Real rect[8]) {
  vccBuildRect(posX,
               posY,
               cosT,
               sinT,
               lf,
               lb,
               halfW,
               &rect[0],
               &rect[1],
               &rect[2],
               &rect[3],
               &rect[4],
               &rect[5],
               &rect[6],
               &rect[7]);
}

Boolean vehicleCollisionCheckRectContainsPoint(const Real rect[8], Real px, Real py) {
  return vccPtInRect(
      px, py, rect[0], rect[1], rect[2], rect[3], rect[4], rect[5], rect[6], rect[7]);
}

static void vccRectCenter(const Real rect[8], Real center[2]) {
  center[0] = 0.5f * (rect[0] + rect[4]);
  center[1] = 0.5f * (rect[1] + rect[5]);
}

static void
vccPointByDistance(const Real pointA[2], const Real pointB[2], Real distanceToA, Real pointOut[2]) {
  Real vecX;
  Real vecY;
  Real distanceAB;
  Real ratio;

  vecX = pointB[0] - pointA[0];
  vecY = pointB[1] - pointA[1];
  distanceAB = sqrtf((vecX * vecX) + (vecY * vecY));
  if (distanceAB < VCC_GEOM_EPSILON) {
    pointOut[0] = pointA[0];
    pointOut[1] = pointA[1];
  } else {
    ratio = distanceToA / distanceAB;
    pointOut[0] = pointA[0] + (vecX * ratio);
    pointOut[1] = pointA[1] + (vecY * ratio);
  }
}

static void vccBuildBoundaryLines4(const Real rect[8], Real boundaryLine[8][4]) {
  boundaryLine[0][0] = rect[0];
  boundaryLine[0][1] = rect[1];
  boundaryLine[0][2] = rect[2];
  boundaryLine[0][3] = rect[3];
  boundaryLine[1][0] = rect[2];
  boundaryLine[1][1] = rect[3];
  boundaryLine[1][2] = rect[4];
  boundaryLine[1][3] = rect[5];
  boundaryLine[2][0] = rect[4];
  boundaryLine[2][1] = rect[5];
  boundaryLine[2][2] = rect[6];
  boundaryLine[2][3] = rect[7];
  boundaryLine[3][0] = rect[6];
  boundaryLine[3][1] = rect[7];
  boundaryLine[3][2] = rect[0];
  boundaryLine[3][3] = rect[1];
}

static void
vccStoreRect(Real rect[8], const Real fl[2], const Real fr[2], const Real rr[2], const Real rl[2]) {
  rect[0] = fl[0];
  rect[1] = fl[1];
  rect[2] = fr[0];
  rect[3] = fr[1];
  rect[4] = rr[0];
  rect[5] = rr[1];
  rect[6] = rl[0];
  rect[7] = rl[1];
}

static void vccBuildNineRects(const Real rect[8],
                              Real tempF,
                              Real tempL,
                              Real tempM,
                              Real rects[VCC_COLLISION_AREA_COUNT][8],
                              Real boundaryLine[8][4]) {
  Real cornerFl[2];
  Real cornerFr[2];
  Real cornerRr[2];
  Real cornerRl[2];
  Real rec1Fl[2];
  Real rec1Fr[2];
  Real rec1Rl[2];
  Real rec1Rr[2];
  Real rec2Fl[2];
  Real rec2Fr[2];
  Real rec2Rl[2];
  Real rec2Rr[2];
  Real rec3Fl[2];
  Real rec3Fr[2];
  Real rec3Rl[2];
  Real rec3Rr[2];
  Real rec4Fl[2];
  Real rec4Fr[2];
  Real rec4Rl[2];
  Real rec4Rr[2];
  Real rec5Fl[2];
  Real rec5Fr[2];
  Real rec5Rl[2];
  Real rec5Rr[2];
  Real rec6Fl[2];
  Real rec6Fr[2];
  Real rec6Rl[2];
  Real rec6Rr[2];
  Real rec7Fl[2];
  Real rec7Fr[2];
  Real rec7Rl[2];
  Real rec7Rr[2];
  Real rec8Fl[2];
  Real rec8Fr[2];
  Real rec8Rl[2];
  Real rec8Rr[2];
  Real rec9Fl[2];
  Real rec9Fr[2];
  Real rec9Rl[2];
  Real rec9Rr[2];

  cornerFl[0] = rect[0];
  cornerFl[1] = rect[1];
  cornerFr[0] = rect[2];
  cornerFr[1] = rect[3];
  cornerRr[0] = rect[4];
  cornerRr[1] = rect[5];
  cornerRl[0] = rect[6];
  cornerRl[1] = rect[7];

  rec1Fl[0] = cornerFl[0];
  rec1Fl[1] = cornerFl[1];
  vccPointByDistance(rec1Fl, cornerFr, tempF, rec1Fr);
  vccPointByDistance(rec1Fl, cornerRl, tempL, rec1Rl);
  rec1Rr[0] = rec1Fr[0] + (rec1Rl[0] - rec1Fl[0]);
  rec1Rr[1] = rec1Fr[1] + (rec1Rl[1] - rec1Fl[1]);
  vccStoreRect(rects[0], rec1Fl, rec1Fr, rec1Rr, rec1Rl);

  rec2Fl[0] = rec1Fr[0];
  rec2Fl[1] = rec1Fr[1];
  vccPointByDistance(cornerFr, cornerFl, tempF, rec2Fr);
  rec2Rl[0] = rec1Rr[0];
  rec2Rl[1] = rec1Rr[1];
  rec2Rr[0] = rec2Fr[0] + (rec2Rl[0] - rec2Fl[0]);
  rec2Rr[1] = rec2Fr[1] + (rec2Rl[1] - rec2Fl[1]);
  vccStoreRect(rects[1], rec2Fl, rec2Fr, rec2Rr, rec2Rl);

  rec3Fl[0] = rec2Fr[0];
  rec3Fl[1] = rec2Fr[1];
  rec3Fr[0] = cornerFr[0];
  rec3Fr[1] = cornerFr[1];
  rec3Rl[0] = rec2Rr[0];
  rec3Rl[1] = rec2Rr[1];
  rec3Rr[0] = rec3Fr[0] + (rec3Rl[0] - rec3Fl[0]);
  rec3Rr[1] = rec3Fr[1] + (rec3Rl[1] - rec3Fl[1]);
  vccStoreRect(rects[2], rec3Fl, rec3Fr, rec3Rr, rec3Rl);

  rec4Fl[0] = rec1Rl[0];
  rec4Fl[1] = rec1Rl[1];
  rec4Fr[0] = rec1Rr[0];
  rec4Fr[1] = rec1Rr[1];
  vccPointByDistance(rec4Fl, cornerRl, tempM, rec4Rl);
  rec4Rr[0] = rec4Fr[0] + (rec4Rl[0] - rec4Fl[0]);
  rec4Rr[1] = rec4Fr[1] + (rec4Rl[1] - rec4Fl[1]);
  vccStoreRect(rects[3], rec4Fl, rec4Fr, rec4Rr, rec4Rl);

  rec5Fl[0] = rec2Rl[0];
  rec5Fl[1] = rec2Rl[1];
  rec5Fr[0] = rec2Rr[0];
  rec5Fr[1] = rec2Rr[1];
  rec5Rl[0] = rec4Rr[0];
  rec5Rl[1] = rec4Rr[1];
  rec5Rr[0] = rec5Fr[0] + (rec5Rl[0] - rec5Fl[0]);
  rec5Rr[1] = rec5Fr[1] + (rec5Rl[1] - rec5Fl[1]);
  vccStoreRect(rects[4], rec5Fl, rec5Fr, rec5Rr, rec5Rl);

  rec6Fl[0] = rec3Rl[0];
  rec6Fl[1] = rec3Rl[1];
  rec6Fr[0] = rec3Rr[0];
  rec6Fr[1] = rec3Rr[1];
  rec6Rl[0] = rec5Rr[0];
  rec6Rl[1] = rec5Rr[1];
  rec6Rr[0] = rec6Fr[0] + (rec6Rl[0] - rec6Fl[0]);
  rec6Rr[1] = rec6Fr[1] + (rec6Rl[1] - rec6Fl[1]);
  vccStoreRect(rects[5], rec6Fl, rec6Fr, rec6Rr, rec6Rl);

  rec7Fl[0] = rec4Rl[0];
  rec7Fl[1] = rec4Rl[1];
  rec7Fr[0] = rec4Rr[0];
  rec7Fr[1] = rec4Rr[1];
  rec7Rl[0] = cornerRl[0];
  rec7Rl[1] = cornerRl[1];
  rec7Rr[0] = rec7Fr[0] + (rec7Rl[0] - rec7Fl[0]);
  rec7Rr[1] = rec7Fr[1] + (rec7Rl[1] - rec7Fl[1]);
  vccStoreRect(rects[6], rec7Fl, rec7Fr, rec7Rr, rec7Rl);

  rec8Fl[0] = rec5Rl[0];
  rec8Fl[1] = rec5Rl[1];
  rec8Fr[0] = rec5Rr[0];
  rec8Fr[1] = rec5Rr[1];
  rec8Rl[0] = rec7Rr[0];
  rec8Rl[1] = rec7Rr[1];
  rec8Rr[0] = rec8Fr[0] + (rec8Rl[0] - rec8Fl[0]);
  rec8Rr[1] = rec8Fr[1] + (rec8Rl[1] - rec8Fl[1]);
  vccStoreRect(rects[7], rec8Fl, rec8Fr, rec8Rr, rec8Rl);

  rec9Fl[0] = rec6Rl[0];
  rec9Fl[1] = rec6Rl[1];
  rec9Fr[0] = rec6Rr[0];
  rec9Fr[1] = rec6Rr[1];
  rec9Rl[0] = rec8Rr[0];
  rec9Rl[1] = rec8Rr[1];
  rec9Rr[0] = cornerRr[0];
  rec9Rr[1] = cornerRr[1];
  vccStoreRect(rects[8], rec9Fl, rec9Fr, rec9Rr, rec9Rl);

  vccBuildBoundaryLines4(rect, boundaryLine);
}

static void vccBuildOctagonAreas(const Real rect[8],
                                 Real frontTriLat,
                                 Real frontTriLon,
                                 Real backTriLat,
                                 Real backTriLon,
                                 Real rects[VCC_COLLISION_AREA_COUNT][8],
                                 Real boundaryLine[8][4],
                                 Real octagonPts[16]) {
  Real cornerFl[2];
  Real cornerFr[2];
  Real cornerRr[2];
  Real cornerRl[2];
  Real rec1Fl[2];
  Real rec1Fr[2];
  Real rec1Rl[2];
  Real rec1Rr[2];
  Real rec2Fl[2];
  Real rec2Fr[2];
  Real rec2Rl[2];
  Real rec2Rr[2];
  Real rec3Fl[2];
  Real rec3Fr[2];
  Real rec3Rl[2];
  Real rec3Rr[2];
  Real rec4Fl[2];
  Real rec4Fr[2];
  Real rec4Rl[2];
  Real rec4Rr[2];
  Real rec5Fl[2];
  Real rec5Fr[2];
  Real rec5Rl[2];
  Real rec5Rr[2];
  Real rec6Fl[2];
  Real rec6Fr[2];
  Real rec6Rl[2];
  Real rec6Rr[2];
  Real rec7Fl[2];
  Real rec7Fr[2];
  Real rec7Rl[2];
  Real rec7Rr[2];
  Real rec8Fl[2];
  Real rec8Fr[2];
  Real rec8Rl[2];
  Real rec8Rr[2];
  Real rec9Fl[2];
  Real rec9Fr[2];
  Real rec9Rl[2];
  Real rec9Rr[2];
  Real rec2H;
  Real delta2[2];

  rec2H = 1.5f;
  cornerFl[0] = rect[0];
  cornerFl[1] = rect[1];
  cornerFr[0] = rect[2];
  cornerFr[1] = rect[3];
  cornerRr[0] = rect[4];
  cornerRr[1] = rect[5];
  cornerRl[0] = rect[6];
  cornerRl[1] = rect[7];

  vccPointByDistance(cornerFl, cornerRl, frontTriLon, rec1Fl);
  vccPointByDistance(cornerFl, cornerFr, frontTriLat, rec1Fr);
  vccPointByDistance(cornerFl, cornerRl, rec2H, rec1Rl);
  rec1Rr[0] = rec1Fr[0] + (rec1Rl[0] - cornerFl[0]);
  rec1Rr[1] = rec1Fr[1] + (rec1Rl[1] - cornerFl[1]);
  vccStoreRect(rects[0], rec1Fl, rec1Fr, rec1Rr, rec1Rl);

  vccPointByDistance(cornerFr, cornerFl, frontTriLat, rec3Fl);
  vccPointByDistance(cornerFr, cornerRr, frontTriLon, rec3Fr);
  vccPointByDistance(cornerFr, cornerRr, rec2H, rec3Rr);
  rec3Rl[0] = rec3Fl[0] + (rec3Rr[0] - cornerFr[0]);
  rec3Rl[1] = rec3Fl[1] + (rec3Rr[1] - cornerFr[1]);
  vccStoreRect(rects[2], rec3Fl, rec3Fr, rec3Rr, rec3Rl);

  rec2Fl[0] = rec1Fr[0];
  rec2Fl[1] = rec1Fr[1];
  rec2Fr[0] = rec3Fl[0];
  rec2Fr[1] = rec3Fl[1];
  rec2Rl[0] = rec1Rr[0];
  rec2Rl[1] = rec1Rr[1];
  rec2Rr[0] = rec3Rl[0];
  rec2Rr[1] = rec3Rl[1];
  vccStoreRect(rects[1], rec2Fl, rec2Fr, rec2Rr, rec2Rl);

  vccPointByDistance(cornerRl, cornerFl, rec2H, rec7Fl);
  vccPointByDistance(cornerRl, cornerFl, backTriLon, rec7Rl);
  vccPointByDistance(cornerRl, cornerRr, backTriLat, rec7Rr);
  delta2[0] = rec7Fl[0] - cornerRl[0];
  delta2[1] = rec7Fl[1] - cornerRl[1];
  rec7Fr[0] = rec7Rr[0] + delta2[0];
  rec7Fr[1] = rec7Rr[1] + delta2[1];
  vccStoreRect(rects[6], rec7Fl, rec7Fr, rec7Rr, rec7Rl);

  vccPointByDistance(cornerRr, cornerFr, rec2H, rec9Fr);
  vccPointByDistance(cornerRr, cornerFr, backTriLon, rec9Rr);
  vccPointByDistance(cornerRr, cornerRl, backTriLat, rec9Rl);
  rec9Fl[0] = rec9Rl[0] + (rec9Fr[0] - cornerRr[0]);
  rec9Fl[1] = rec9Rl[1] + (rec9Fr[1] - cornerRr[1]);
  vccStoreRect(rects[8], rec9Fl, rec9Fr, rec9Rr, rec9Rl);

  rec8Fl[0] = rec7Fr[0];
  rec8Fl[1] = rec7Fr[1];
  rec8Fr[0] = rec9Fl[0];
  rec8Fr[1] = rec9Fl[1];
  rec8Rl[0] = rec7Rr[0];
  rec8Rl[1] = rec7Rr[1];
  rec8Rr[0] = rec9Rl[0];
  rec8Rr[1] = rec9Rl[1];
  vccStoreRect(rects[7], rec8Fl, rec8Fr, rec8Rr, rec8Rl);

  rec4Fl[0] = rec1Rl[0];
  rec4Fl[1] = rec1Rl[1];
  rec4Fr[0] = rec1Rr[0];
  rec4Fr[1] = rec1Rr[1];
  rec4Rl[0] = rec7Fl[0];
  rec4Rl[1] = rec7Fl[1];
  rec4Rr[0] = rec7Fr[0];
  rec4Rr[1] = rec7Fr[1];
  vccStoreRect(rects[3], rec4Fl, rec4Fr, rec4Rr, rec4Rl);

  rec5Fl[0] = rec2Rl[0];
  rec5Fl[1] = rec2Rl[1];
  rec5Fr[0] = rec2Rr[0];
  rec5Fr[1] = rec2Rr[1];
  rec5Rl[0] = rec8Fl[0];
  rec5Rl[1] = rec8Fl[1];
  rec5Rr[0] = rec8Fr[0];
  rec5Rr[1] = rec8Fr[1];
  vccStoreRect(rects[4], rec5Fl, rec5Fr, rec5Rr, rec5Rl);

  rec6Fl[0] = rec3Rl[0];
  rec6Fl[1] = rec3Rl[1];
  rec6Fr[0] = rec3Rr[0];
  rec6Fr[1] = rec3Rr[1];
  rec6Rl[0] = rec9Fl[0];
  rec6Rl[1] = rec9Fl[1];
  rec6Rr[0] = rec9Fr[0];
  rec6Rr[1] = rec9Fr[1];
  vccStoreRect(rects[5], rec6Fl, rec6Fr, rec6Rr, rec6Rl);

  boundaryLine[0][0] = rec1Fl[0];
  boundaryLine[0][1] = rec1Fl[1];
  boundaryLine[0][2] = rec2Fl[0];
  boundaryLine[0][3] = rec2Fl[1];
  boundaryLine[1][0] = rec2Fl[0];
  boundaryLine[1][1] = rec2Fl[1];
  boundaryLine[1][2] = rec3Fl[0];
  boundaryLine[1][3] = rec3Fl[1];
  boundaryLine[2][0] = rec3Fl[0];
  boundaryLine[2][1] = rec3Fl[1];
  boundaryLine[2][2] = rec3Fr[0];
  boundaryLine[2][3] = rec3Fr[1];
  boundaryLine[3][0] = rec3Fr[0];
  boundaryLine[3][1] = rec3Fr[1];
  boundaryLine[3][2] = rec9Rr[0];
  boundaryLine[3][3] = rec9Rr[1];
  boundaryLine[4][0] = rec9Rr[0];
  boundaryLine[4][1] = rec9Rr[1];
  boundaryLine[4][2] = rec9Rl[0];
  boundaryLine[4][3] = rec9Rl[1];
  boundaryLine[5][0] = rec9Rl[0];
  boundaryLine[5][1] = rec9Rl[1];
  boundaryLine[5][2] = rec8Rl[0];
  boundaryLine[5][3] = rec8Rl[1];
  boundaryLine[6][0] = rec8Rl[0];
  boundaryLine[6][1] = rec8Rl[1];
  boundaryLine[6][2] = rec7Rl[0];
  boundaryLine[6][3] = rec7Rl[1];
  boundaryLine[7][0] = rec7Rl[0];
  boundaryLine[7][1] = rec7Rl[1];
  boundaryLine[7][2] = rec1Fl[0];
  boundaryLine[7][3] = rec1Fl[1];

  octagonPts[0] = rec1Fl[0];
  octagonPts[1] = rec1Fl[1];
  octagonPts[2] = rec2Fl[0];
  octagonPts[3] = rec2Fl[1];
  octagonPts[4] = rec3Fl[0];
  octagonPts[5] = rec3Fl[1];
  octagonPts[6] = rec3Fr[0];
  octagonPts[7] = rec3Fr[1];
  octagonPts[8] = rec9Rr[0];
  octagonPts[9] = rec9Rr[1];
  octagonPts[10] = rec9Rl[0];
  octagonPts[11] = rec9Rl[1];
  octagonPts[12] = rec8Rl[0];
  octagonPts[13] = rec8Rl[1];
  octagonPts[14] = rec7Rl[0];
  octagonPts[15] = rec7Rl[1];
}

static void vccResolveDepthAgainstBoundary(const Real collisionPoint[2],
                                           const Real boundaryLine[8][4],
                                           Integer boundaryLineNum,
                                           Real *collisionDepth,
                                           UInteger8 *crossLine) {
  Integer i;

  *collisionDepth = 99.0f;
  *crossLine = 0U;
  for (i = 0; i < boundaryLineNum; i = i + 1) {
    Real closestX;
    Real closestY;
    Real dx;
    Real dy;
    Real distance;

    vccClosestPtOnSeg(collisionPoint[0],
                      collisionPoint[1],
                      boundaryLine[i][0],
                      boundaryLine[i][1],
                      boundaryLine[i][2],
                      boundaryLine[i][3],
                      &closestX,
                      &closestY);
    dx = collisionPoint[0] - closestX;
    dy = collisionPoint[1] - closestY;
    distance = sqrtf((dx * dx) + (dy * dy));
    if (distance <= *collisionDepth) {
      *collisionDepth = distance;
      *crossLine = (UInteger8)(i + 1);
    }
  }
}

Boolean vehicleCollisionCheckLineHitsRect(Real x1, Real y1, Real x2, Real y2, const Real rect[8]) {
  Boolean hit;
  Integer i;

  hit = vehicleCollisionCheckRectContainsPoint(rect, x1, y1) ||
        vehicleCollisionCheckRectContainsPoint(rect, x2, y2);
  for (i = 0; i < 4; i = i + 1) {
    if (!hit) {
      Integer nextIndex;

      nextIndex = i + 1;
      if (nextIndex >= 4) {
        nextIndex = 0;
      }
      hit = vccSegmentsIntersect(x1,
                                 y1,
                                 x2,
                                 y2,
                                 rect[i * 2],
                                 rect[i * 2 + 1],
                                 rect[nextIndex * 2],
                                 rect[nextIndex * 2 + 1]);
    }
  }
  return hit;
}

static Boolean
vccPolygonContainsPointFlat(const Real *polygon, Integer pointCount, Real px, Real py) {
  Boolean hasPositive;
  Boolean hasNegative;
  Integer i;

  hasPositive = false;
  hasNegative = false;
  for (i = 0; i < pointCount; i = i + 1) {
    Integer nextIndex;
    Real cross;

    nextIndex = i + 1;
    if (nextIndex >= pointCount) {
      nextIndex = 0;
    }
    cross = apaMathCross2D(polygon[nextIndex * 2] - polygon[i * 2],
                           polygon[nextIndex * 2 + 1] - polygon[i * 2 + 1],
                           px - polygon[i * 2],
                           py - polygon[i * 2 + 1]);
    if (cross > VCC_GEOM_EPSILON) {
      hasPositive = true;
    }
    if (cross < -VCC_GEOM_EPSILON) {
      hasNegative = true;
    }
  }
  return !(hasPositive && hasNegative);
}

static Boolean vccLineHitsPolygonFlat(
    Real x1, Real y1, Real x2, Real y2, const Real *polygon, Integer pointCount) {
  Boolean hit;
  Integer i;

  hit = vccPolygonContainsPointFlat(polygon, pointCount, x1, y1) ||
        vccPolygonContainsPointFlat(polygon, pointCount, x2, y2);
  for (i = 0; i < pointCount; i = i + 1) {
    if (!hit) {
      Integer nextIndex;

      nextIndex = i + 1;
      if (nextIndex >= pointCount) {
        nextIndex = 0;
      }
      hit = vccSegmentsIntersect(x1,
                                 y1,
                                 x2,
                                 y2,
                                 polygon[i * 2],
                                 polygon[i * 2 + 1],
                                 polygon[nextIndex * 2],
                                 polygon[nextIndex * 2 + 1]);
    }
  }
  return hit;
}

static Boolean vccShouldCheckBoundaryStart(const VehicleCollisionCheckInput_t *input,
                                           Integer lineIndex) {
  Boolean shouldCheck;

  shouldCheck = false;
  if (lineIndex == 0) {
    shouldCheck = true;
  } else if ((fabsf(input->BoundaryLineX1[lineIndex] - input->BoundaryLineX2[lineIndex - 1]) >
              VCC_GEOM_EPSILON) ||
             (fabsf(input->BoundaryLineY1[lineIndex] - input->BoundaryLineY2[lineIndex - 1]) >
              VCC_GEOM_EPSILON)) {
    shouldCheck = true;
  }
  return shouldCheck;
}

static void vccWriteAllCollision(VehicleCollisionCheckDetailedOutput_t *output,
                                 Integer collisionType,
                                 Integer collisionIndex,
                                 Integer areaIndex,
                                 const Real collisionPoint[2],
                                 const Real boundaryLine[8][4],
                                 Integer boundaryLineNum) {
  output->Summary.IsCollision = true;
  output->Summary.CollisionType = collisionType;
  output->Summary.CollisionIndex = collisionIndex;
  if ((areaIndex >= 0) && (areaIndex < VCC_COLLISION_AREA_COUNT)) {
    output->Summary.CollisionArea[areaIndex] = 1U;
  }
  output->CollisionPoint[0] = collisionPoint[0];
  output->CollisionPoint[1] = collisionPoint[1];
  vccResolveDepthAgainstBoundary(collisionPoint,
                                 boundaryLine,
                                 boundaryLineNum,
                                 &output->Summary.CollisionDepth,
                                 &output->Summary.CrossLine);
}

static void vccWriteSimplifyCollision(VehicleCollisionCheckDetailedOutput_t *output,
                                      Integer collisionType,
                                      const Real *collisionPoint) {
  output->Summary.IsCollision = true;
  output->Summary.CollisionType = collisionType;
  if (collisionPoint) {
    output->CollisionPoint[0] = collisionPoint[0];
    output->CollisionPoint[1] = collisionPoint[1];
  }
}

static Boolean vccCheckType2All(const VehicleCollisionCheckInput_t *input,
                                const Real rects[VCC_COLLISION_AREA_COUNT][8],
                                const Real boundaryLine[8][4],
                                VehicleCollisionCheckDetailedOutput_t *output) {
  Boolean found;
  Boolean checkSeparateStopper;
  Integer trackNum;
  Integer sepNum;
  Integer j;

  found = false;
  checkSeparateStopper = (input->SlotClass == VCC_SLOT_CLASS_PARALLEL);
  trackNum = input->SlotTrackStopperNum;
  sepNum = input->SeparateStopperNum;
  if (trackNum > VCC_MAX_TRACK_STOPPER) {
    trackNum = VCC_MAX_TRACK_STOPPER;
  }
  if (sepNum > VCC_MAX_SEPARATE_PTS) {
    sepNum = VCC_MAX_SEPARATE_PTS;
  }

  for (j = 0; j < VCC_COLLISION_AREA_COUNT; j = j + 1) {
    Integer i;

    for (i = 0; i < trackNum; i = i + 1) {
      if (!found && vehicleCollisionCheckRectContainsPoint(
                        rects[j], input->SlotTrackStopperX1[i], input->SlotTrackStopperY1[i])) {
        Real collisionPoint[2];

        collisionPoint[0] = input->SlotTrackStopperX1[i];
        collisionPoint[1] = input->SlotTrackStopperY1[i];
        vccWriteAllCollision(
            output, VCC_COLLISION_STOPPER, i + 1, j, collisionPoint, boundaryLine, 4);
        found = true;
      }
      if (!found && vehicleCollisionCheckRectContainsPoint(
                        rects[j], input->SlotTrackStopperX2[i], input->SlotTrackStopperY2[i])) {
        Real collisionPoint[2];

        collisionPoint[0] = input->SlotTrackStopperX2[i];
        collisionPoint[1] = input->SlotTrackStopperY2[i];
        vccWriteAllCollision(
            output, VCC_COLLISION_STOPPER, i + 1, j, collisionPoint, boundaryLine, 4);
        found = true;
      }
    }
  }

  if ((!found) && checkSeparateStopper) {
    for (j = 0; j < VCC_COLLISION_AREA_COUNT; j = j + 1) {
      Integer i;

      for (i = 0; i < sepNum; i = i + 1) {
        if (!found && vehicleCollisionCheckRectContainsPoint(
                          rects[j], input->SeparateStopperX[i], input->SeparateStopperY[i])) {
          Real collisionPoint[2];

          collisionPoint[0] = input->SeparateStopperX[i];
          collisionPoint[1] = input->SeparateStopperY[i];
          vccWriteAllCollision(
              output, VCC_COLLISION_STOPPER, i + 1, j, collisionPoint, boundaryLine, 4);
          found = true;
        }
      }
    }
  }
  return found;
}

static Boolean vccCheckType2Simplify(const VehicleCollisionCheckInput_t *input,
                                     const Real wbRect[8],
                                     VehicleCollisionCheckDetailedOutput_t *output) {
  Boolean found;
  Integer trackNum;
  Integer sepNum;
  Integer i;

  found = false;
  trackNum = input->SlotTrackStopperNum;
  sepNum = input->SeparateStopperNum;
  if (trackNum > VCC_MAX_TRACK_STOPPER) {
    trackNum = VCC_MAX_TRACK_STOPPER;
  }
  if (sepNum > VCC_MAX_SEPARATE_PTS) {
    sepNum = VCC_MAX_SEPARATE_PTS;
  }

  for (i = 0; i < trackNum; i = i + 1) {
    if (!found && vehicleCollisionCheckRectContainsPoint(
                      wbRect, input->SlotTrackStopperX1[i], input->SlotTrackStopperY1[i])) {
      vccWriteSimplifyCollision(
          output,
          VCC_COLLISION_STOPPER,
          (const Real[2]){input->SlotTrackStopperX1[i], input->SlotTrackStopperY1[i]});
      found = true;
    }
    if (!found && vehicleCollisionCheckRectContainsPoint(
                      wbRect, input->SlotTrackStopperX2[i], input->SlotTrackStopperY2[i])) {
      vccWriteSimplifyCollision(
          output,
          VCC_COLLISION_STOPPER,
          (const Real[2]){input->SlotTrackStopperX2[i], input->SlotTrackStopperY2[i]});
      found = true;
    }
  }

  for (i = 0; i < sepNum; i = i + 1) {
    if (!found && vehicleCollisionCheckRectContainsPoint(
                      wbRect, input->SeparateStopperX[i], input->SeparateStopperY[i])) {
      vccWriteSimplifyCollision(
          output,
          VCC_COLLISION_STOPPER,
          (const Real[2]){input->SeparateStopperX[i], input->SeparateStopperY[i]});
      found = true;
    }
  }
  return found;
}

static void vccResolveVirtualMargins(const VehicleCollisionCheckInput_t *input,
                                     Real fsMargin,
                                     Real rsMargin,
                                     Real ssMargin,
                                     Real *virtFs,
                                     Real *virtRs,
                                     Real *virtSs) {
  if (input->UseAutoMargin) {
    *virtFs = *virtRs = *virtSs = APA_PARKIN_MARGIN_M;
    return;
  }
  const Real base = APA_PARKIN_PHYSICAL_MARGIN_M;
  *virtFs = fsMargin < base - VCC_GEOM_EPSILON ? fminf(fsMargin, APA_PARKIN_MARGIN_M)
                                               : APA_PARKIN_MARGIN_M + fmaxf(0.0f, fsMargin - base);
  *virtRs = rsMargin < base - VCC_GEOM_EPSILON ? fminf(rsMargin, APA_PARKIN_MARGIN_M)
                                               : APA_PARKIN_MARGIN_M + fmaxf(0.0f, rsMargin - base);
  *virtSs = ssMargin < base - VCC_GEOM_EPSILON ? fminf(ssMargin, APA_PARKIN_MARGIN_M)
                                               : APA_PARKIN_MARGIN_M + fmaxf(0.0f, ssMargin - base);
}

void vehicleCollisionCheckResolveVirtualMargins(const VehicleCollisionCheckInput_t *input,
                                                Real margins[3]) {
  Real body[3];
  vehicleCollisionCheckResolveBodyMargins(input, body);
  vccResolveVirtualMargins(input, body[0], body[1], body[2], &margins[0], &margins[1], &margins[2]);
}

static Boolean vccBottomClearEx(const VehicleCollisionCheckInput_t *input,
                                Real x1,
                                Real y1,
                                Real x2,
                                Real y2,
                                Boolean isVirtual) {
  ApaParkinBottom_t bottom;
  if (!apaParkinBottom(input, &bottom) ||
      apaParkinBottomDepth(&bottom, x1, y1) > APA_PARKIN_CLEARANCE_EPS_M ||
      apaParkinBottomDepth(&bottom, x2, y2) > APA_PARKIN_CLEARANCE_EPS_M)
    return false;
  const double t1 = apaParkinBottomAlong(&bottom, x1, y1);
  const double t2 = apaParkinBottomAlong(&bottom, x2, y2);
  if (!apaParkinBottomSpan(&bottom, fmin(t1, t2), fmax(t1, t2)))
    return false;
  Real rect[8], margins[3];
  Real octRects[VCC_COLLISION_AREA_COUNT][8];
  Real octBoundary[8][4];
  Real octPts[16];
  vccBuildRectArray(input->VehPosX,
                    input->VehPosY,
                    cosf(input->VehPosTheta),
                    sinf(input->VehPosTheta),
                    input->VehLF,
                    input->VehLB,
                    input->VehW / 2.0f,
                    rect);
  vccBuildOctagonAreas(rect,
                       APA_VEHICLE_FRONT_CHAMFER_LAT_M,
                       APA_VEHICLE_FRONT_CHAMFER_LON_M,
                       APA_VEHICLE_REAR_CHAMFER_LAT_M,
                       APA_VEHICLE_REAR_CHAMFER_LON_M,
                       octRects,
                       octBoundary,
                       octPts);
  (void)octRects;
  (void)octBoundary;
  if (vccLineHitsPolygonFlat(x1, y1, x2, y2, octPts, VCC_POLYGON_EDGE_COUNT))
    return false;
  double gap = 1.e30;
  for (Integer i = 0; i < VCC_POLYGON_EDGE_COUNT; ++i) {
    const Integer next = (i + 1) % VCC_POLYGON_EDGE_COUNT;
    Real cx, cy;
    vccClosestPtOnSeg(octPts[2 * i], octPts[2 * i + 1], x1, y1, x2, y2, &cx, &cy);
    gap = fmin(gap, hypot((double)cx - octPts[2 * i], (double)cy - octPts[2 * i + 1]));
    vccClosestPtOnSeg(
        x1, y1, octPts[2 * i], octPts[2 * i + 1], octPts[2 * next], octPts[2 * next + 1], &cx, &cy);
    gap = fmin(gap, hypot((double)cx - x1, (double)cy - y1));
    vccClosestPtOnSeg(
        x2, y2, octPts[2 * i], octPts[2 * i + 1], octPts[2 * next], octPts[2 * next + 1], &cx, &cy);
    gap = fmin(gap, hypot((double)cx - x2, (double)cy - y2));
  }
  vehicleCollisionCheckResolveBodyMargins(input, margins);
  if (isVirtual) {
    Real vm[3];
    vccResolveVirtualMargins(input, margins[0], margins[1], margins[2], &vm[0], &vm[1], &vm[2]);
    return gap > APA_PARKIN_CLEARANCE_EPS_M &&
           gap + APA_PARKIN_CLEARANCE_EPS_M >= apaParkinVirtualBottomMargin(vm[0], vm[1], vm[2]);
  }
  return gap > APA_PARKIN_CLEARANCE_EPS_M &&
         gap + APA_PARKIN_CLEARANCE_EPS_M >=
             apaParkinBottomMargin(input, margins[0], margins[1], margins[2]);
}

static Boolean
vccBottomClear(const VehicleCollisionCheckInput_t *input, Real x1, Real y1, Real x2, Real y2) {
  return vccBottomClearEx(input, x1, y1, x2, y2, false);
}

static Boolean vccCheckType1All(const VehicleCollisionCheckInput_t *input,
                                const Real rects[VCC_COLLISION_AREA_COUNT][8],
                                const Real boundaryLine[8][4],
                                Integer boundaryLineNum,
                                VehicleCollisionCheckDetailedOutput_t *output) {
  Boolean found;
  Integer lineNum;
  Integer sepNum;
  Integer j;

  found = false;
  lineNum = input->BoundaryLineNum;
  sepNum = input->BoundarySepPtsNum;
  if (lineNum > VCC_MAX_BOUNDARY_LINES) {
    lineNum = VCC_MAX_BOUNDARY_LINES;
  }
  if (sepNum > VCC_MAX_SEPARATE_PTS) {
    sepNum = VCC_MAX_SEPARATE_PTS;
  }

  for (j = 0; j < VCC_COLLISION_AREA_COUNT; j = j + 1) {
    Integer i;
    Boolean areaCollision;
    Real bestDistance;
    Real bestPoint[2];
    Real currentCenter[2];
    Integer bestLineIndex;

    areaCollision = false;
    bestDistance = 99.0f;
    bestPoint[0] = 99.0f;
    bestPoint[1] = 99.0f;
    bestLineIndex = -1;
    vccRectCenter(rects[j], currentCenter);

    for (i = 0; i < lineNum; i = i + 1) {
      if (vccBottomClear(input,
                         input->BoundaryLineX1[i],
                         input->BoundaryLineY1[i],
                         input->BoundaryLineX2[i],
                         input->BoundaryLineY2[i]))
        continue;
      if ((!found) && vccShouldCheckBoundaryStart(input, i) &&
          vehicleCollisionCheckRectContainsPoint(
              rects[j], input->BoundaryLineX1[i], input->BoundaryLineY1[i])) {
        Real collisionPoint[2];

        collisionPoint[0] = input->BoundaryLineX1[i];
        collisionPoint[1] = input->BoundaryLineY1[i];
        vccWriteAllCollision(output,
                             VCC_COLLISION_BOUNDARY,
                             i + 1,
                             j,
                             collisionPoint,
                             boundaryLine,
                             boundaryLineNum);
        found = true;
      }

      if ((!found) && vehicleCollisionCheckLineHitsRect(input->BoundaryLineX1[i],
                                                        input->BoundaryLineY1[i],
                                                        input->BoundaryLineX2[i],
                                                        input->BoundaryLineY2[i],
                                                        rects[j])) {
        Real closestX;
        Real closestY;
        Real dx;
        Real dy;
        Real distance;

        vccClosestPtOnSeg(currentCenter[0],
                          currentCenter[1],
                          input->BoundaryLineX1[i],
                          input->BoundaryLineY1[i],
                          input->BoundaryLineX2[i],
                          input->BoundaryLineY2[i],
                          &closestX,
                          &closestY);
        dx = currentCenter[0] - closestX;
        dy = currentCenter[1] - closestY;
        distance = sqrtf((dx * dx) + (dy * dy));
        areaCollision = true;
        if (distance < bestDistance) {
          bestDistance = distance;
          bestPoint[0] = closestX;
          bestPoint[1] = closestY;
          bestLineIndex = i;
        }
      }
    }

    if ((!found) && areaCollision) {
      vccWriteAllCollision(output,
                           VCC_COLLISION_BOUNDARY,
                           bestLineIndex + 1,
                           j,
                           bestPoint,
                           boundaryLine,
                           boundaryLineNum);
      found = true;
    }
  }

  for (j = 0; j < VCC_COLLISION_AREA_COUNT; j = j + 1) {
    Integer i;

    for (i = 0; i < sepNum; i = i + 1) {
      if (vccBottomClear(input,
                         input->BoundarySepPtsX[i],
                         input->BoundarySepPtsY[i],
                         input->BoundarySepPtsX[i],
                         input->BoundarySepPtsY[i]))
        continue;
      if ((!found) && vehicleCollisionCheckRectContainsPoint(
                          rects[j], input->BoundarySepPtsX[i], input->BoundarySepPtsY[i])) {
        Real collisionPoint[2];

        collisionPoint[0] = input->BoundarySepPtsX[i];
        collisionPoint[1] = input->BoundarySepPtsY[i];
        vccWriteAllCollision(output,
                             VCC_COLLISION_BOUNDARY,
                             lineNum + i + 1,
                             j,
                             collisionPoint,
                             boundaryLine,
                             boundaryLineNum);
        found = true;
      }
    }
  }
  return found;
}

static Boolean vccCheckType1SimplifyFour(const VehicleCollisionCheckInput_t *input,
                                         const Real rect[8],
                                         VehicleCollisionCheckDetailedOutput_t *output) {
  Boolean found;
  Integer lineNum;
  Integer sepNum;
  Integer i;

  found = false;
  lineNum = input->BoundaryLineNum;
  sepNum = input->BoundarySepPtsNum;
  if (lineNum > VCC_MAX_BOUNDARY_LINES) {
    lineNum = VCC_MAX_BOUNDARY_LINES;
  }
  if (sepNum > VCC_MAX_SEPARATE_PTS) {
    sepNum = VCC_MAX_SEPARATE_PTS;
  }

  for (i = 0; i < lineNum; i = i + 1) {
    if (vccBottomClear(input,
                       input->BoundaryLineX1[i],
                       input->BoundaryLineY1[i],
                       input->BoundaryLineX2[i],
                       input->BoundaryLineY2[i]))
      continue;
    if ((!found) && vccShouldCheckBoundaryStart(input, i) &&
        vehicleCollisionCheckRectContainsPoint(
            rect, input->BoundaryLineX1[i], input->BoundaryLineY1[i])) {
      vccWriteSimplifyCollision(
          output,
          VCC_COLLISION_BOUNDARY,
          (const Real[2]){input->BoundaryLineX1[i], input->BoundaryLineY1[i]});
      found = true;
    }
    if ((!found) && vehicleCollisionCheckLineHitsRect(input->BoundaryLineX1[i],
                                                      input->BoundaryLineY1[i],
                                                      input->BoundaryLineX2[i],
                                                      input->BoundaryLineY2[i],
                                                      rect)) {
      vccWriteSimplifyCollision(output, VCC_COLLISION_BOUNDARY, 0);
      found = true;
    }
  }

  for (i = 0; i < sepNum; i = i + 1) {
    if (vccBottomClear(input,
                       input->BoundarySepPtsX[i],
                       input->BoundarySepPtsY[i],
                       input->BoundarySepPtsX[i],
                       input->BoundarySepPtsY[i]))
      continue;
    if ((!found) && vehicleCollisionCheckRectContainsPoint(
                        rect, input->BoundarySepPtsX[i], input->BoundarySepPtsY[i])) {
      vccWriteSimplifyCollision(
          output,
          VCC_COLLISION_BOUNDARY,
          (const Real[2]){input->BoundarySepPtsX[i], input->BoundarySepPtsY[i]});
      found = true;
    }
  }
  return found;
}

static Boolean vccCheckType1SimplifyOctagon(const VehicleCollisionCheckInput_t *input,
                                            const Real octagonPts[16],
                                            VehicleCollisionCheckDetailedOutput_t *output) {
  Boolean found;
  Integer lineNum;
  Integer sepNum;
  Integer i;

  found = false;
  lineNum = input->BoundaryLineNum;
  sepNum = input->BoundarySepPtsNum;
  if (lineNum > VCC_MAX_BOUNDARY_LINES) {
    lineNum = VCC_MAX_BOUNDARY_LINES;
  }
  if (sepNum > VCC_MAX_SEPARATE_PTS) {
    sepNum = VCC_MAX_SEPARATE_PTS;
  }

  for (i = 0; i < lineNum; i = i + 1) {
    if (vccBottomClear(input,
                       input->BoundaryLineX1[i],
                       input->BoundaryLineY1[i],
                       input->BoundaryLineX2[i],
                       input->BoundaryLineY2[i])) {
      continue;
    }
    if ((!found) && vccShouldCheckBoundaryStart(input, i) &&
        vccPolygonContainsPointFlat(octagonPts,
                                    VCC_POLYGON_EDGE_COUNT,
                                    input->BoundaryLineX1[i],
                                    input->BoundaryLineY1[i])) {
      vccWriteSimplifyCollision(
          output,
          VCC_COLLISION_BOUNDARY,
          (const Real[2]){input->BoundaryLineX1[i], input->BoundaryLineY1[i]});
      found = true;
    }
    if ((!found) && vccLineHitsPolygonFlat(input->BoundaryLineX1[i],
                                           input->BoundaryLineY1[i],
                                           input->BoundaryLineX2[i],
                                           input->BoundaryLineY2[i],
                                           octagonPts,
                                           VCC_POLYGON_EDGE_COUNT)) {
      vccWriteSimplifyCollision(output, VCC_COLLISION_BOUNDARY, 0);
      found = true;
    }
  }

  for (i = 0; i < sepNum; i = i + 1) {
    if (vccBottomClear(input,
                       input->BoundarySepPtsX[i],
                       input->BoundarySepPtsY[i],
                       input->BoundarySepPtsX[i],
                       input->BoundarySepPtsY[i])) {
      continue;
    }
    if ((!found) && vccPolygonContainsPointFlat(octagonPts,
                                                VCC_POLYGON_EDGE_COUNT,
                                                input->BoundarySepPtsX[i],
                                                input->BoundarySepPtsY[i])) {
      vccWriteSimplifyCollision(
          output,
          VCC_COLLISION_BOUNDARY,
          (const Real[2]){input->BoundarySepPtsX[i], input->BoundarySepPtsY[i]});
      found = true;
    }
  }
  return found;
}

static Boolean vccCheckType3All(const VehicleCollisionCheckInput_t *input,
                                const Real rects[VCC_COLLISION_AREA_COUNT][8],
                                const Real fullCenter[2],
                                const Real boundaryLine[8][4],
                                Integer boundaryLineNum,
                                VehicleCollisionCheckDetailedOutput_t *output) {
  Boolean found;
  Integer vLineNum;
  Integer i;

  found = false;
  vLineNum = input->VirtualBoundaryLineNum;
  if (vLineNum > VCC_MAX_VIRTUAL_LINES) {
    vLineNum = VCC_MAX_VIRTUAL_LINES;
  }

  for (i = 0; i < vLineNum; i = i + 1) {
    if (vccBottomClearEx(input,
                         input->VirtualBoundaryLineX1[i],
                         input->VirtualBoundaryLineY1[i],
                         input->VirtualBoundaryLineX2[i],
                         input->VirtualBoundaryLineY2[i],
                         true))
      continue;
    if (!found) {
      Integer j;
      Boolean areaCollision;
      Real bestDistance;
      Real bestPoint[2];
      Integer bestArea;

      areaCollision = false;
      bestDistance = 99.0f;
      bestPoint[0] = 99.0f;
      bestPoint[1] = 99.0f;
      bestArea = -1;
      for (j = 0; j < VCC_COLLISION_AREA_COUNT; j = j + 1) {
        if (vehicleCollisionCheckLineHitsRect(input->VirtualBoundaryLineX1[i],
                                              input->VirtualBoundaryLineY1[i],
                                              input->VirtualBoundaryLineX2[i],
                                              input->VirtualBoundaryLineY2[i],
                                              rects[j])) {
          Real closestX;
          Real closestY;
          Real dx;
          Real dy;
          Real distance;

          vccClosestPtOnSeg(fullCenter[0],
                            fullCenter[1],
                            input->VirtualBoundaryLineX1[i],
                            input->VirtualBoundaryLineY1[i],
                            input->VirtualBoundaryLineX2[i],
                            input->VirtualBoundaryLineY2[i],
                            &closestX,
                            &closestY);
          dx = fullCenter[0] - closestX;
          dy = fullCenter[1] - closestY;
          distance = sqrtf((dx * dx) + (dy * dy));
          output->Summary.CollisionArea[j] = 1U;
          areaCollision = true;
          if (distance < bestDistance) {
            bestDistance = distance;
            bestPoint[0] = closestX;
            bestPoint[1] = closestY;
            bestArea = j;
          }
        }
      }
      if (areaCollision && (bestArea >= 0)) {
        vccWriteAllCollision(output,
                             VCC_COLLISION_VIRTUAL,
                             i + 1,
                             bestArea,
                             bestPoint,
                             boundaryLine,
                             boundaryLineNum);
        for (j = 0; j < VCC_COLLISION_AREA_COUNT; ++j) {
          if (vehicleCollisionCheckRectContainsPoint(rects[j], bestPoint[0], bestPoint[1]))
            bestArea = j;
        }
        output->DeepPointArea = (UInteger8)(bestArea + 1);
        found = true;
      }
    }
  }
  return found;
}

static Boolean vccCheckType3SimplifyFour(const VehicleCollisionCheckInput_t *input,
                                         const Real rect[8],
                                         VehicleCollisionCheckDetailedOutput_t *output) {
  Boolean found;
  Integer vLineNum;
  Integer i;

  found = false;
  vLineNum = input->VirtualBoundaryLineNum;
  if (vLineNum > VCC_MAX_VIRTUAL_LINES) {
    vLineNum = VCC_MAX_VIRTUAL_LINES;
  }
  for (i = 0; i < vLineNum; i = i + 1) {
    if (vccBottomClearEx(input,
                         input->VirtualBoundaryLineX1[i],
                         input->VirtualBoundaryLineY1[i],
                         input->VirtualBoundaryLineX2[i],
                         input->VirtualBoundaryLineY2[i],
                         true))
      continue;
    if ((!found) && vehicleCollisionCheckLineHitsRect(input->VirtualBoundaryLineX1[i],
                                                      input->VirtualBoundaryLineY1[i],
                                                      input->VirtualBoundaryLineX2[i],
                                                      input->VirtualBoundaryLineY2[i],
                                                      rect)) {
      vccWriteSimplifyCollision(output, VCC_COLLISION_VIRTUAL, 0);
      found = true;
    }
  }
  return found;
}

static Boolean vccCheckType3SimplifyOctagon(const VehicleCollisionCheckInput_t *input,
                                            const Real octagonPts[16],
                                            VehicleCollisionCheckDetailedOutput_t *output) {
  Boolean found;
  Integer vLineNum;
  Integer i;

  found = false;
  vLineNum = input->VirtualBoundaryLineNum;
  if (vLineNum > VCC_MAX_VIRTUAL_LINES) {
    vLineNum = VCC_MAX_VIRTUAL_LINES;
  }
  for (i = 0; i < vLineNum; i = i + 1) {
    if (vccBottomClearEx(input,
                         input->VirtualBoundaryLineX1[i],
                         input->VirtualBoundaryLineY1[i],
                         input->VirtualBoundaryLineX2[i],
                         input->VirtualBoundaryLineY2[i],
                         true)) {
      continue;
    }
    if ((!found) && vccLineHitsPolygonFlat(input->VirtualBoundaryLineX1[i],
                                           input->VirtualBoundaryLineY1[i],
                                           input->VirtualBoundaryLineX2[i],
                                           input->VirtualBoundaryLineY2[i],
                                           octagonPts,
                                           VCC_POLYGON_EDGE_COUNT)) {
      vccWriteSimplifyCollision(output, VCC_COLLISION_VIRTUAL, 0);
      found = true;
    }
  }
  return found;
}

void vehicleCollisionCheckResolveBodyMargins(const VehicleCollisionCheckInput_t *input,
                                             Real margins[3]) {
  if (input->UseAutoMargin) {
    const VccSlotFlags_t flags = vccResolveSlotFlagsFromPts(input);
    Real frontTriLat, frontTriLon, backTriLat, backTriLon;
    vccResolveFirstBoundaryMargins(input,
                                   &flags,
                                   &margins[0],
                                   &margins[1],
                                   &margins[2],
                                   &frontTriLat,
                                   &frontTriLon,
                                   &backTriLat,
                                   &backTriLon);
  } else {
    margins[0] = input->FrontSafeMargin;
    margins[1] = input->RearSafeMargin;
    margins[2] = input->SideSafeMargin;
  }
}

/** @brief 车辆碰撞检查及几何诊断。 */
void vehicleCollisionCheckDetailed(const VehicleCollisionCheckInput_t *input,
                                   VehicleCollisionCheckDetailedOutput_t *output) {
  Real posX;
  Real posY;
  Real theta;
  Real cosT;
  Real sinT;
  Integer collisionModel;
  Integer collisionDemand;
  Real fsMargin;
  Real rsMargin;
  Real ssMargin;
  Real wbLongMargin;
  Real wbLatMargin;
  Real frontTriLat;
  Real frontTriLon;
  Real backTriLat;
  Real backTriLon;
  Real fullLf;
  Real fullLb;
  Real fullHalfW;
  Real wbRect[8];
  Real fullRect[8];
  Real thirdRect[8];
  Real thirdCenter[2];
  Real rects[VCC_COLLISION_AREA_COUNT][8];
  Real boundaryLine[8][4];
  Real stopperBoundaryLine[8][4];
  Real octagonPts[16];
  Boolean condIsPlanning;

  posX = input->VehPosX;
  posY = input->VehPosY;
  theta = input->VehPosTheta;
  cosT = cosf(theta);
  sinT = sinf(theta);
  collisionModel = input->CheckType == VCC_CHECK_TYPE_PLANNING ? VCC_COLLISION_MODEL_EIGHT_EDGES
                                                               : input->CollisionModel;
  if (collisionModel != VCC_COLLISION_MODEL_EIGHT_EDGES) {
    collisionModel = VCC_COLLISION_MODEL_FOUR_EDGES;
  }
  collisionDemand = input->CollisionDemand;
  if (collisionDemand != VCC_COLLISION_DEMAND_ALL) {
    collisionDemand = VCC_COLLISION_DEMAND_SIMPLIFY;
  }

  vccResetOutput(output);

  if (input->UseAutoMargin) {
    const VccSlotFlags_t flags = vccResolveSlotFlagsFromPts(input);
    vccResolveFirstBoundaryMargins(input,
                                   &flags,
                                   &fsMargin,
                                   &rsMargin,
                                   &ssMargin,
                                   &frontTriLat,
                                   &frontTriLon,
                                   &backTriLat,
                                   &backTriLon);
    vccResolveWheelBaseMargins(input, &flags, &wbLongMargin, &wbLatMargin);
  } else {
    fsMargin = input->FrontSafeMargin;
    rsMargin = input->RearSafeMargin;
    ssMargin = input->SideSafeMargin;
    wbLongMargin = input->WhlBseLongMargin;
    wbLatMargin = input->WhlBseLatMargin;
    if (input->FrontTriLat > VCC_GEOM_EPSILON) {
      frontTriLat = input->FrontTriLat;
    } else {
      frontTriLat = VCC_DEFAULT_FRONT_TRI_LAT;
    }
    if (input->FrontTriLon > VCC_GEOM_EPSILON) {
      frontTriLon = input->FrontTriLon;
    } else {
      frontTriLon = VCC_DEFAULT_FRONT_TRI_LON;
    }
    if (input->BackTriLat > VCC_GEOM_EPSILON) {
      backTriLat = input->BackTriLat;
    } else {
      backTriLat = VCC_DEFAULT_BACK_TRI_LAT;
    }
    if (input->BackTriLon > VCC_GEOM_EPSILON) {
      backTriLon = input->BackTriLon;
    } else {
      backTriLon = VCC_DEFAULT_BACK_TRI_LON;
    }
  }

  fullLf = input->VehLF + fsMargin;
  fullLb = input->VehLB + rsMargin;
  fullHalfW = input->VehW * 0.5f + ssMargin;
  condIsPlanning = (input->CheckType == VCC_CHECK_TYPE_PLANNING);

  vccBuildRectArray(posX,
                    posY,
                    cosT,
                    sinT,
                    input->VehWB + wbLongMargin,
                    wbLongMargin,
                    (input->VehW * 0.5f) + wbLatMargin,
                    wbRect);
  vccBuildRectArray(posX, posY, cosT, sinT, fullLf, fullLb, fullHalfW, fullRect);
  if (condIsPlanning) {
    Real virtFs;
    Real virtRs;
    Real virtSs;

    vccResolveVirtualMargins(input, fsMargin, rsMargin, ssMargin, &virtFs, &virtRs, &virtSs);
    vccBuildRectArray(posX,
                      posY,
                      cosT,
                      sinT,
                      input->VehLF + virtFs,
                      input->VehLB + virtRs,
                      (input->VehW * 0.5f) + virtSs,
                      thirdRect);
    vccRectCenter(thirdRect, thirdCenter);
  }

  if (collisionDemand == VCC_COLLISION_DEMAND_ALL) {
    Real stopperRecH;
    Real thirdRecH;
    Real boundaryRecH;

    stopperRecH = input->VehWB + (2.0f * wbLongMargin);
    vccBuildNineRects(wbRect, 0.6f, 1.5f, stopperRecH - 3.0f, rects, stopperBoundaryLine);
    (void)vccCheckType2All(
        input, (const Real(*)[8])rects, (const Real(*)[4])stopperBoundaryLine, output);

    if (condIsPlanning && (!output->Summary.IsCollision)) {
      if (collisionModel == VCC_COLLISION_MODEL_EIGHT_EDGES) {
        vccBuildOctagonAreas(thirdRect,
                             frontTriLat,
                             frontTriLon,
                             backTriLat,
                             backTriLon,
                             rects,
                             boundaryLine,
                             octagonPts);
        (void)octagonPts;
        (void)vccCheckType3All(input,
                               (const Real(*)[8])rects,
                               thirdCenter,
                               (const Real(*)[4])boundaryLine,
                               VCC_POLYGON_EDGE_COUNT,
                               output);
      } else {
        thirdRecH = (thirdRect[0] - thirdRect[6]) * cosT + (thirdRect[1] - thirdRect[7]) * sinT;
        vccBuildNineRects(thirdRect, 0.6f, 1.5f, thirdRecH - 3.0f, rects, boundaryLine);
        (void)vccCheckType3All(
            input, (const Real(*)[8])rects, thirdCenter, (const Real(*)[4])boundaryLine, 4, output);
      }
    }

    if (!output->Summary.IsCollision) {
      output->Summary.SafeMargin1stBound[0] = fsMargin;
      output->Summary.SafeMargin1stBound[1] = rsMargin;
      output->Summary.SafeMargin1stBound[2] = ssMargin;
      boundaryRecH = input->VehLF + input->VehLB + fsMargin + rsMargin;
      if (collisionModel == VCC_COLLISION_MODEL_EIGHT_EDGES) {
        vccBuildOctagonAreas(fullRect,
                             frontTriLat,
                             frontTriLon,
                             backTriLat,
                             backTriLon,
                             rects,
                             boundaryLine,
                             octagonPts);
        (void)octagonPts;
        (void)vccCheckType1All(input,
                               (const Real(*)[8])rects,
                               (const Real(*)[4])boundaryLine,
                               VCC_POLYGON_EDGE_COUNT,
                               output);
      } else {
        vccBuildNineRects(fullRect, 0.6f, 1.5f, boundaryRecH - 3.0f, rects, boundaryLine);
        (void)vccCheckType1All(
            input, (const Real(*)[8])rects, (const Real(*)[4])boundaryLine, 4, output);
      }
    }
  } else {
    (void)vccCheckType2Simplify(input, wbRect, output);

    if (condIsPlanning && (!output->Summary.IsCollision)) {
      if (collisionModel == VCC_COLLISION_MODEL_EIGHT_EDGES) {
        vccBuildOctagonAreas(thirdRect,
                             frontTriLat,
                             frontTriLon,
                             backTriLat,
                             backTriLon,
                             rects,
                             boundaryLine,
                             octagonPts);
        (void)rects;
        (void)boundaryLine;
        (void)vccCheckType3SimplifyOctagon(input, octagonPts, output);
      } else {
        (void)vccCheckType3SimplifyFour(input, thirdRect, output);
      }
    }

    if (!output->Summary.IsCollision) {
      output->Summary.SafeMargin1stBound[0] = fsMargin;
      output->Summary.SafeMargin1stBound[1] = rsMargin;
      output->Summary.SafeMargin1stBound[2] = ssMargin;
      if (collisionModel == VCC_COLLISION_MODEL_EIGHT_EDGES) {
        vccBuildOctagonAreas(fullRect,
                             frontTriLat,
                             frontTriLon,
                             backTriLat,
                             backTriLon,
                             rects,
                             boundaryLine,
                             octagonPts);
        (void)rects;
        (void)boundaryLine;
        (void)vccCheckType1SimplifyOctagon(input, octagonPts, output);
      } else {
        (void)vccCheckType1SimplifyFour(input, fullRect, output);
      }
    }
  }
}

/**
 * @brief 检查实体、轮挡及虚拟线碰撞，输出裕量、碰撞分区和深度。
 * @en_name vehicleCollisionCheck
 * @cn_name 车辆碰撞检测
 * @type module
 * @param[IN] Integer CheckType 碰撞检测类型
 * @param[IN] Real VehPosX 车辆后轴X
 * @param[IN] Real VehPosY 车辆后轴Y
 * @param[IN] Real VehPosTheta 车辆航向角
 * @param[IN] Real VehLF 前悬加轴距
 * @param[IN] Real VehLB 后悬
 * @param[IN] Real VehLL 车长
 * @param[IN] Real VehW 车宽
 * @param[IN] Real VehWB 轴距
 * @param[IN] Boolean UseAutoMargin 自动按MATLAB语义计算裕量
 * @param[IN] Boolean VehNearSlotFlag 车辆贴近车位标志
 * @param[IN] Boolean VehInSlotFlag 车辆在车位内标志
 * @param[IN] Real FrontSafeMargin 前部安全裕量
 * @param[IN] Real RearSafeMargin 后部安全裕量
 * @param[IN] Real SideSafeMargin 侧向安全裕量
 * @param[IN] Real WhlBseLongMargin 轴基纵向裕量
 * @param[IN] Real WhlBseLatMargin 轴基横向裕量
 * @param[IN] Integer BoundaryLineNum 第一边界线段数
 * @param[IN] Real BoundaryLineX1 边界线段起点X
 * @param[IN] Real BoundaryLineY1 边界线段起点Y
 * @param[IN] Real BoundaryLineX2 边界线段终点X
 * @param[IN] Real BoundaryLineY2 边界线段终点Y
 * @param[IN] Integer BoundarySepPtsNum 边界离散点数
 * @param[IN] Real BoundarySepPtsX 边界离散点X
 * @param[IN] Real BoundarySepPtsY 边界离散点Y
 * @param[IN] Integer SlotTrackStopperNum stopper数
 * @param[IN] Real SlotTrackStopperX1 stopper端点1X
 * @param[IN] Real SlotTrackStopperY1 stopper端点1Y
 * @param[IN] Real SlotTrackStopperX2 stopper端点2X
 * @param[IN] Real SlotTrackStopperY2 stopper端点2Y
 * @param[IN] Integer SeparateStopperNum 离散stopper数
 * @param[IN] Real SeparateStopperX 离散stopperX
 * @param[IN] Real SeparateStopperY 离散stopperY
 * @param[IN] Integer VirtualBoundaryLineNum 虚拟边界线段数
 * @param[IN] Real VirtualBoundaryLineX1 虚拟边界起点X
 * @param[IN] Real VirtualBoundaryLineY1 虚拟边界起点Y
 * @param[IN] Real VirtualBoundaryLineX2 虚拟边界终点X
 * @param[IN] Real VirtualBoundaryLineY2 虚拟边界终点Y
 * @param[OUT] Boolean IsCollision 是否碰撞
 * @param[OUT] Integer CollisionType 碰撞类型
 * @param[OUT] Integer CollisionIndex 碰撞元素索引
 * @param[OUT] Real CollisionDepth 碰撞深度（距最近车辆边线距离）
 * @param[OUT] UInteger8 CrossLine 最近车辆边线序号（1-8八边形/1-4四边形）
 * @param[OUT] UInteger8[9] CollisionArea 9区碰撞标志
 * @param[OUT] Real[3] SafeMargin1stBound 实际使用的第一边界裕量[front, rear, side]
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite VehicleCollisionCheck.m
 * @version 1.2
 * @date 2026-05-02
 * @author aide
 */
void vehicleCollisionCheck(const VehicleCollisionCheckInput_t *input,
                           VehicleCollisionCheckOutput_t *output) {
  VehicleCollisionCheckDetailedOutput_t checked;
  vehicleCollisionCheckDetailed(input, &checked);
  *output = checked.Summary;
}
