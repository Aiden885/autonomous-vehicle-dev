#include "calcVehCornerPointTypes.h"
#include "apaMathUtils.h"
#include "obsSlightAdjustForSpecificPosTypes.h"
#include <math.h>

#ifndef OBS_SLIGHT_MAX_PROCESS_COUNT
#define OBS_SLIGHT_MAX_PROCESS_COUNT 20
#endif

#ifndef OBS_SLIGHT_DEPTH_THRESH
#define OBS_SLIGHT_DEPTH_THRESH 0.1f
#endif

#ifndef OBS_SLIGHT_SAFE_OFFSET
#define OBS_SLIGHT_SAFE_OFFSET 0.075f
#endif

#ifndef OBS_SLIGHT_SHORTEN_OFFSET
#define OBS_SLIGHT_SHORTEN_OFFSET 0.09f
#endif

#ifndef OBS_SLIGHT_EPS
#define OBS_SLIGHT_EPS 1.0e-6f
#endif

typedef struct {
  Real X;
  Real Y;
} ObsSlightPoint_t;

typedef struct {
  Real X1;
  Real Y1;
  Real X2;
  Real Y2;
} ObsSlightLine_t;

typedef struct {
  ObsSlightPoint_t Fl;
  ObsSlightPoint_t Fr;
  ObsSlightPoint_t Rr;
  ObsSlightPoint_t Rl;
} ObsSlightRect_t;

typedef struct {
  ObsSlightPoint_t Front;
  ObsSlightPoint_t Rear;
  ObsSlightPoint_t Left;
  ObsSlightPoint_t Right;
  ObsSlightPoint_t FrontLeft;
  ObsSlightPoint_t FrontRight;
  ObsSlightPoint_t RearLeft;
  ObsSlightPoint_t RearRight;
} ObsSlightBiasBase_t;

typedef struct {
  Boolean HasIntersection;
  Boolean LineParallel;
  ObsSlightPoint_t Point;
} ObsSlightIntersection_t;

static ObsSlightPoint_t obsSlightMakePoint(Real x, Real y) {
  ObsSlightPoint_t result0;

  result0.X = x;
  result0.Y = y;

  return result0;
}

static ObsSlightPoint_t obsSlightAddPoint(const ObsSlightPoint_t *left,
                                          const ObsSlightPoint_t *right) {
  ObsSlightPoint_t result0;

  result0.X = left->X + right->X;
  result0.Y = left->Y + right->Y;

  return result0;
}

static ObsSlightPoint_t obsSlightSubPoint(const ObsSlightPoint_t *left,
                                          const ObsSlightPoint_t *right) {
  ObsSlightPoint_t result0;

  result0.X = left->X - right->X;
  result0.Y = left->Y - right->Y;

  return result0;
}

static ObsSlightPoint_t obsSlightScalePoint(const ObsSlightPoint_t *point, Real scale) {
  ObsSlightPoint_t result0;

  result0.X = point->X * scale;
  result0.Y = point->Y * scale;

  return result0;
}

static void obsSlightCalcRect(const ObsSlightAdjustForSpecificPosInput_t *input,
                              Real frontMargin,
                              Real rearMargin,
                              Real sideMargin,
                              ObsSlightRect_t *rect) {
  CalcVehCornerPointInput_t cornerInput;
  CalcVehCornerPointOutput_t cornerOutput;

  cornerInput.PosX = input->PosX;
  cornerInput.PosY = input->PosY;
  cornerInput.PosTheta = input->PosTheta;
  cornerInput.FrontOverhang = input->StObs.VehLF;
  cornerInput.RearOverhang = input->StObs.VehLB;
  cornerInput.VehWidth = input->StObs.VehW;
  cornerInput.FrontMargin = frontMargin;
  cornerInput.RearMargin = rearMargin;
  cornerInput.SideMargin = sideMargin;
  calcVehCornerPoint(&cornerInput, &cornerOutput);

  rect->Fl = obsSlightMakePoint(cornerOutput.FlX, cornerOutput.FlY);
  rect->Fr = obsSlightMakePoint(cornerOutput.FrX, cornerOutput.FrY);
  rect->Rr = obsSlightMakePoint(cornerOutput.RrX, cornerOutput.RrY);
  rect->Rl = obsSlightMakePoint(cornerOutput.RlX, cornerOutput.RlY);
}

static void obsSlightCalcBiasBase(const ObsSlightAdjustForSpecificPosInput_t *input,
                                  ObsSlightBiasBase_t *biasBase) {
  ObsSlightRect_t rect0;
  ObsSlightPoint_t frontRaw0;
  ObsSlightPoint_t leftRaw0;
  ObsSlightPoint_t frontLeftRaw0;
  ObsSlightPoint_t frontRightRaw0;

  obsSlightCalcRect(input, 0.0f, 0.0f, 0.0f, &rect0);

  frontRaw0 = obsSlightSubPoint(&rect0.Fl, &rect0.Rl);
  apaMathNormalize2D(frontRaw0.X, frontRaw0.Y, &biasBase->Front.X, &biasBase->Front.Y);
  biasBase->Rear = obsSlightScalePoint(&biasBase->Front, -1.0f);

  leftRaw0 = obsSlightSubPoint(&rect0.Fl, &rect0.Fr);
  apaMathNormalize2D(leftRaw0.X, leftRaw0.Y, &biasBase->Left.X, &biasBase->Left.Y);
  biasBase->Right = obsSlightScalePoint(&biasBase->Left, -1.0f);

  frontLeftRaw0 = obsSlightAddPoint(&biasBase->Front, &biasBase->Left);
  apaMathNormalize2D(
      frontLeftRaw0.X, frontLeftRaw0.Y, &biasBase->FrontLeft.X, &biasBase->FrontLeft.Y);
  biasBase->RearRight = obsSlightScalePoint(&biasBase->FrontLeft, -1.0f);

  frontRightRaw0 = obsSlightSubPoint(&biasBase->Front, &biasBase->Left);
  apaMathNormalize2D(
      frontRightRaw0.X, frontRightRaw0.Y, &biasBase->FrontRight.X, &biasBase->FrontRight.Y);
  biasBase->RearLeft = obsSlightScalePoint(&biasBase->FrontRight, -1.0f);
}

static void obsSlightResetOutput(const ObsSlightAdjustForSpecificPosInput_t *input,
                                 ObsSlightAdjustForSpecificPosOutput_t *output) {
  Integer areaIndex0;

  output->ObsValid = false;
  output->StObs = input->StObs;
  output->LineProcessCount = 0;
  output->PointProcessCount = 0;

  areaIndex0 = 0;
  while (areaIndex0 < OBS_SLIGHT_VEH_AREA_COUNT) {
    output->VehCollisionArea[areaIndex0] = false;
    areaIndex0 = areaIndex0 + 1;
  }
}

static void obsSlightApplyCollisionContext(const ObsSlightAdjustForSpecificPosInput_t *input,
                                           VehicleCollisionCheckInput_t *collisionInput) {
  Integer slotIndex0;

  collisionInput->CheckType = input->CheckType;
  collisionInput->CollisionModel = VCC_COLLISION_MODEL_FOUR_EDGES;
  collisionInput->CollisionDemand = VCC_COLLISION_DEMAND_ALL;
  collisionInput->ParkingOutFlag = input->ParkingOutFlag;
  collisionInput->SlotClass = input->SlotClass;
  collisionInput->VehPosX = input->PosX;
  collisionInput->VehPosY = input->PosY;
  collisionInput->VehPosTheta = input->PosTheta;

  if (input->SlotPtsValid) {
    collisionInput->SlotPtsValid = true;
  } else {
    collisionInput->SlotPtsValid = input->StObs.SlotPtsValid;
  }
  slotIndex0 = 0;
  while (slotIndex0 < 8) {
    if (input->SlotPtsValid) {
      collisionInput->SlotPts[slotIndex0] = input->SlotPts[slotIndex0];
    } else {
      collisionInput->SlotPts[slotIndex0] = input->StObs.SlotPts[slotIndex0];
    }
    slotIndex0 = slotIndex0 + 1;
  }
}

static void obsSlightBuildLineCollisionObs(const ObsSlightAdjustForSpecificPosInput_t *input,
                                           const VehicleCollisionCheckInput_t *tempObs,
                                           VehicleCollisionCheckInput_t *collisionObs) {
  *collisionObs = *tempObs;
  obsSlightApplyCollisionContext(input, collisionObs);
  collisionObs->BoundarySepPtsNum = 0;
  collisionObs->SeparateStopperNum = 0;
  collisionObs->SlotTrackStopperNum = 0;
  collisionObs->VirtualBoundaryLineNum = 0;
}

static void obsSlightBuildPointCollisionObs(const ObsSlightAdjustForSpecificPosInput_t *input,
                                            const VehicleCollisionCheckInput_t *tempObs,
                                            VehicleCollisionCheckInput_t *collisionObs) {
  *collisionObs = input->StObs;
  obsSlightApplyCollisionContext(input, collisionObs);
  collisionObs->BoundaryLineNum = 0;
  collisionObs->SeparateStopperNum = 0;
  collisionObs->SlotTrackStopperNum = 0;
  collisionObs->VirtualBoundaryLineNum = 0;
  collisionObs->BoundarySepPtsNum = tempObs->BoundarySepPtsNum;

  {
    Integer pointIndex0;

    pointIndex0 = 0;
    while (pointIndex0 < VCC_MAX_SEPARATE_PTS) {
      collisionObs->BoundarySepPtsX[pointIndex0] = tempObs->BoundarySepPtsX[pointIndex0];
      collisionObs->BoundarySepPtsY[pointIndex0] = tempObs->BoundarySepPtsY[pointIndex0];
      pointIndex0 = pointIndex0 + 1;
    }
  }
}

static void obsSlightCopyBoundaryLines(const VehicleCollisionCheckInput_t *sourceObs,
                                       VehicleCollisionCheckInput_t *targetObs) {
  Integer lineIndex0;

  targetObs->BoundaryLineNum = sourceObs->BoundaryLineNum;
  lineIndex0 = 0;
  while (lineIndex0 < VCC_MAX_BOUNDARY_LINES) {
    targetObs->BoundaryLineX1[lineIndex0] = sourceObs->BoundaryLineX1[lineIndex0];
    targetObs->BoundaryLineY1[lineIndex0] = sourceObs->BoundaryLineY1[lineIndex0];
    targetObs->BoundaryLineX2[lineIndex0] = sourceObs->BoundaryLineX2[lineIndex0];
    targetObs->BoundaryLineY2[lineIndex0] = sourceObs->BoundaryLineY2[lineIndex0];
    lineIndex0 = lineIndex0 + 1;
  }
}

static void obsSlightCopyBoundaryPoints(const VehicleCollisionCheckInput_t *sourceObs,
                                        VehicleCollisionCheckInput_t *targetObs) {
  Integer pointIndex0;

  targetObs->BoundarySepPtsNum = sourceObs->BoundarySepPtsNum;
  pointIndex0 = 0;
  while (pointIndex0 < VCC_MAX_SEPARATE_PTS) {
    targetObs->BoundarySepPtsX[pointIndex0] = sourceObs->BoundarySepPtsX[pointIndex0];
    targetObs->BoundarySepPtsY[pointIndex0] = sourceObs->BoundarySepPtsY[pointIndex0];
    pointIndex0 = pointIndex0 + 1;
  }
}

static void obsSlightMarkCollisionArea(const VehicleCollisionCheckOutput_t *collisionOutput,
                                       ObsSlightAdjustForSpecificPosOutput_t *output) {
  Boolean frontHit0;
  Boolean rearHit0;

  frontHit0 = (collisionOutput->CollisionArea[0] != 0U) ||
              (collisionOutput->CollisionArea[1] != 0U) ||
              (collisionOutput->CollisionArea[2] != 0U);
  rearHit0 = (collisionOutput->CollisionArea[6] != 0U) ||
             (collisionOutput->CollisionArea[7] != 0U) || (collisionOutput->CollisionArea[8] != 0U);

  if (frontHit0) {
    output->VehCollisionArea[0] = true;
  }
  if (rearHit0) {
    output->VehCollisionArea[1] = true;
  }
  if (collisionOutput->CollisionArea[3] != 0U) {
    output->VehCollisionArea[2] = true;
  }
  if (collisionOutput->CollisionArea[5] != 0U) {
    output->VehCollisionArea[3] = true;
  }
}

static Boolean obsSlightRejectCollision(const ObsSlightAdjustForSpecificPosInput_t *input,
                                        const VehicleCollisionCheckOutput_t *collisionOutput,
                                        ObsSlightAdjustForSpecificPosOutput_t *output) {
  Boolean depthReject0;
  Boolean centerReject0;
  Boolean reject0;

  depthReject0 = false;
  if (input->ConsiderCollisionDepthFlag) {
    if (collisionOutput->CollisionDepth > OBS_SLIGHT_DEPTH_THRESH) {
      depthReject0 = true;
    }
  }

  centerReject0 = false;
  if ((!depthReject0) && (collisionOutput->CollisionArea[4] != 0U)) {
    output->VehCollisionArea[4] = true;
    centerReject0 = true;
  }

  reject0 = depthReject0 || centerReject0;

  return reject0;
}

static Boolean obsSlightSelectBias(const VehicleCollisionCheckOutput_t *collisionOutput,
                                   const ObsSlightBiasBase_t *biasBase,
                                   ObsSlightPoint_t *biasPoint) {
  Boolean biasValid0;
  Real scale0;
  ObsSlightPoint_t base0;
  Boolean area1Hit0;
  Boolean area3Hit0;
  Boolean area7Hit0;
  Boolean area9Hit0;

  biasValid0 = true;
  scale0 = collisionOutput->CollisionDepth + OBS_SLIGHT_SAFE_OFFSET;
  base0 = biasBase->Front;
  area1Hit0 = collisionOutput->CollisionArea[0] != 0U;
  area3Hit0 = collisionOutput->CollisionArea[2] != 0U;
  area7Hit0 = collisionOutput->CollisionArea[6] != 0U;
  area9Hit0 = collisionOutput->CollisionArea[8] != 0U;

  switch (collisionOutput->CrossLine) {
  case 1U:
    if (area1Hit0 && !area3Hit0) {
      scale0 = (collisionOutput->CollisionDepth * 1.4f) + OBS_SLIGHT_SAFE_OFFSET;
      base0 = biasBase->FrontLeft;
    } else {
      if (!area1Hit0 && area3Hit0) {
        scale0 = (collisionOutput->CollisionDepth * 1.4f) + OBS_SLIGHT_SAFE_OFFSET;
        base0 = biasBase->FrontRight;
      } else {
        base0 = biasBase->Front;
      }
    }
    break;
  case 2U:
    base0 = biasBase->Right;
    break;
  case 3U:
    if (area7Hit0 && !area9Hit0) {
      scale0 = (collisionOutput->CollisionDepth * 1.4f) + OBS_SLIGHT_SAFE_OFFSET;
      base0 = biasBase->RearLeft;
    } else {
      if (!area7Hit0 && area9Hit0) {
        scale0 = (collisionOutput->CollisionDepth * 1.4f) + OBS_SLIGHT_SAFE_OFFSET;
        base0 = biasBase->RearRight;
      } else {
        base0 = biasBase->Rear;
      }
    }
    break;
  case 4U:
    base0 = biasBase->Left;
    break;
  default:
    biasValid0 = false;
    break;
  }

  *biasPoint = obsSlightScalePoint(&base0, scale0);

  return biasValid0;
}

static Boolean obsSlightAppendBoundaryLine(VehicleCollisionCheckInput_t *tempObs,
                                           const ObsSlightLine_t *line) {
  Boolean success0;
  Integer writeIndex0;

  success0 = false;
  if ((tempObs->BoundaryLineNum >= 0) && (tempObs->BoundaryLineNum < VCC_MAX_BOUNDARY_LINES)) {
    writeIndex0 = tempObs->BoundaryLineNum;
    tempObs->BoundaryLineX1[writeIndex0] = line->X1;
    tempObs->BoundaryLineY1[writeIndex0] = line->Y1;
    tempObs->BoundaryLineX2[writeIndex0] = line->X2;
    tempObs->BoundaryLineY2[writeIndex0] = line->Y2;
    tempObs->BoundaryLineNum = tempObs->BoundaryLineNum + 1;
    success0 = true;
  }

  return success0;
}

static void obsSlightAppendOriginalLineEndsIfRoom(VehicleCollisionCheckInput_t *tempObs,
                                                  const ObsSlightLine_t *line) {
  Integer writeIndex0;
  Integer writeIndex1;
  Integer newCount0;

  newCount0 = tempObs->BoundarySepPtsNum + 2;
  if ((tempObs->BoundarySepPtsNum >= 0) && (newCount0 <= VCC_MAX_SEPARATE_PTS)) {
    writeIndex0 = tempObs->BoundarySepPtsNum;
    writeIndex1 = writeIndex0 + 1;
    tempObs->BoundarySepPtsX[writeIndex0] = line->X1;
    tempObs->BoundarySepPtsY[writeIndex0] = line->Y1;
    tempObs->BoundarySepPtsX[writeIndex1] = line->X2;
    tempObs->BoundarySepPtsY[writeIndex1] = line->Y2;
    tempObs->BoundarySepPtsNum = newCount0;
  }
}

static ObsSlightLine_t obsSlightRectLineAt(const ObsSlightRect_t *rect, Integer lineIndex) {
  ObsSlightLine_t result0;

  if (lineIndex == 0) {
    result0.X1 = rect->Fl.X;
    result0.Y1 = rect->Fl.Y;
    result0.X2 = rect->Fr.X;
    result0.Y2 = rect->Fr.Y;
  } else {
    if (lineIndex == 1) {
      result0.X1 = rect->Fr.X;
      result0.Y1 = rect->Fr.Y;
      result0.X2 = rect->Rr.X;
      result0.Y2 = rect->Rr.Y;
    } else {
      if (lineIndex == 2) {
        result0.X1 = rect->Rr.X;
        result0.Y1 = rect->Rr.Y;
        result0.X2 = rect->Rl.X;
        result0.Y2 = rect->Rl.Y;
      } else {
        result0.X1 = rect->Rl.X;
        result0.Y1 = rect->Rl.Y;
        result0.X2 = rect->Fl.X;
        result0.Y2 = rect->Fl.Y;
      }
    }
  }

  return result0;
}

static Boolean obsSlightPointInRect(const ObsSlightRect_t *rect, const ObsSlightPoint_t *point) {
  Real c1;
  Real c2;
  Real c3;
  Real c4;
  Boolean allNonNeg0;
  Boolean allNonPos0;
  Boolean result0;

  c1 = ((rect->Fr.X - rect->Fl.X) * (point->Y - rect->Fl.Y)) -
       ((rect->Fr.Y - rect->Fl.Y) * (point->X - rect->Fl.X));
  c2 = ((rect->Rr.X - rect->Fr.X) * (point->Y - rect->Fr.Y)) -
       ((rect->Rr.Y - rect->Fr.Y) * (point->X - rect->Fr.X));
  c3 = ((rect->Rl.X - rect->Rr.X) * (point->Y - rect->Rr.Y)) -
       ((rect->Rl.Y - rect->Rr.Y) * (point->X - rect->Rr.X));
  c4 = ((rect->Fl.X - rect->Rl.X) * (point->Y - rect->Rl.Y)) -
       ((rect->Fl.Y - rect->Rl.Y) * (point->X - rect->Rl.X));

  allNonNeg0 = (c1 >= 0.0f) && (c2 >= 0.0f) && (c3 >= 0.0f) && (c4 >= 0.0f);
  allNonPos0 = (c1 <= 0.0f) && (c2 <= 0.0f) && (c3 <= 0.0f) && (c4 <= 0.0f);
  result0 = allNonNeg0 || allNonPos0;

  return result0;
}

static ObsSlightIntersection_t obsSlightLineSegmentIntersection(const ObsSlightLine_t *line1,
                                                                const ObsSlightLine_t *line2) {
  ObsSlightIntersection_t result0;
  Real d1x0;
  Real d1y0;
  Real d2x0;
  Real d2y0;
  Real denominator0;

  result0.HasIntersection = false;
  result0.LineParallel = false;
  result0.Point = obsSlightMakePoint(0.0f, 0.0f);

  d1x0 = line1->X2 - line1->X1;
  d1y0 = line1->Y2 - line1->Y1;
  d2x0 = line2->X2 - line2->X1;
  d2y0 = line2->Y2 - line2->Y1;
  denominator0 = (d1x0 * d2y0) - (d1y0 * d2x0);

  if (apaMathAbs(denominator0) < 1.0e-10f) {
    result0.LineParallel = true;
  } else {
    Real p31x0;
    Real p31y0;
    Real t0;
    Real u0;
    Boolean tInRange0;
    Boolean uInRange0;

    p31x0 = line2->X1 - line1->X1;
    p31y0 = line2->Y1 - line1->Y1;
    t0 = ((p31x0 * d2y0) - (p31y0 * d2x0)) / denominator0;
    u0 = ((p31x0 * d1y0) - (p31y0 * d1x0)) / denominator0;
    tInRange0 = (t0 >= 0.0f) && (t0 <= 1.0f);
    uInRange0 = (u0 >= 0.0f) && (u0 <= 1.0f);
    if (tInRange0 && uInRange0) {
      result0.Point.X = line1->X1 + (t0 * d1x0);
      result0.Point.Y = line1->Y1 + (t0 * d1y0);
      result0.HasIntersection = true;
    }
  }

  return result0;
}

static ObsSlightLine_t obsSlightMakeLineFromPoints(const ObsSlightPoint_t *startPoint,
                                                   const ObsSlightPoint_t *endPoint) {
  ObsSlightLine_t result0;

  result0.X1 = startPoint->X;
  result0.Y1 = startPoint->Y;
  result0.X2 = endPoint->X;
  result0.Y2 = endPoint->Y;

  return result0;
}

static Boolean obsSlightAppendShortenedSegment(VehicleCollisionCheckInput_t *tempObs,
                                               const ObsSlightPoint_t *lineEnd,
                                               const ObsSlightPoint_t *intersectionPoint) {
  Boolean success0;
  Real distance0;

  success0 = true;
  distance0 = apaMathDistance2D(lineEnd->X, lineEnd->Y, intersectionPoint->X, intersectionPoint->Y);
  if (distance0 > OBS_SLIGHT_SHORTEN_OFFSET) {
    ObsSlightPoint_t vec0;
    ObsSlightPoint_t vecBase0;
    ObsSlightPoint_t shift0;
    ObsSlightPoint_t newEnd0;
    ObsSlightLine_t newLine0;

    vec0 = obsSlightSubPoint(lineEnd, intersectionPoint);
    apaMathNormalize2D(vec0.X, vec0.Y, &vecBase0.X, &vecBase0.Y);
    shift0 = obsSlightScalePoint(&vecBase0, OBS_SLIGHT_SHORTEN_OFFSET);
    newEnd0 = obsSlightAddPoint(intersectionPoint, &shift0);
    newLine0 = obsSlightMakeLineFromPoints(lineEnd, &newEnd0);
    success0 = obsSlightAppendBoundaryLine(tempObs, &newLine0);
  }

  return success0;
}

static Boolean obsSlightAppendParallelSegment(VehicleCollisionCheckInput_t *tempObs,
                                              const ObsSlightLine_t *obsLine,
                                              const ObsSlightRect_t *rect,
                                              const ObsSlightLine_t *parallelLine) {
  Boolean success0;
  ObsSlightPoint_t rectCenter0;
  ObsSlightPoint_t parallelCenter0;
  ObsSlightPoint_t vec0;
  ObsSlightPoint_t vecBase0;
  ObsSlightPoint_t shift0;
  ObsSlightLine_t newLine0;

  rectCenter0.X = (rect->Fl.X + rect->Rr.X) * 0.5f;
  rectCenter0.Y = (rect->Fl.Y + rect->Rr.Y) * 0.5f;
  parallelCenter0.X = (parallelLine->X1 + parallelLine->X2) * 0.5f;
  parallelCenter0.Y = (parallelLine->Y1 + parallelLine->Y2) * 0.5f;
  vec0 = obsSlightSubPoint(&parallelCenter0, &rectCenter0);
  apaMathNormalize2D(vec0.X, vec0.Y, &vecBase0.X, &vecBase0.Y);
  shift0 = obsSlightScalePoint(&vecBase0, OBS_SLIGHT_SHORTEN_OFFSET);

  newLine0.X1 = obsLine->X1 + shift0.X;
  newLine0.Y1 = obsLine->Y1 + shift0.Y;
  newLine0.X2 = obsLine->X2 + shift0.X;
  newLine0.Y2 = obsLine->Y2 + shift0.Y;
  success0 = obsSlightAppendBoundaryLine(tempObs, &newLine0);

  return success0;
}

static Boolean obsSlightAppendOutsideSegments(VehicleCollisionCheckInput_t *tempObs,
                                              const ObsSlightLine_t *obsLine,
                                              const ObsSlightPoint_t *intersectionPoint1,
                                              const ObsSlightPoint_t *intersectionPoint2) {
  Boolean success0;
  ObsSlightPoint_t obsEnd1;
  ObsSlightPoint_t obsEnd2;
  Real end1ToInt1Dist0;
  Real end1ToInt2Dist0;
  Boolean pairWithEnd1First0;

  obsEnd1 = obsSlightMakePoint(obsLine->X1, obsLine->Y1);
  obsEnd2 = obsSlightMakePoint(obsLine->X2, obsLine->Y2);
  end1ToInt1Dist0 =
      apaMathDistance2D(obsEnd1.X, obsEnd1.Y, intersectionPoint1->X, intersectionPoint1->Y);
  end1ToInt2Dist0 =
      apaMathDistance2D(obsEnd1.X, obsEnd1.Y, intersectionPoint2->X, intersectionPoint2->Y);
  pairWithEnd1First0 = end1ToInt1Dist0 < end1ToInt2Dist0;

  if (pairWithEnd1First0) {
    success0 = obsSlightAppendShortenedSegment(tempObs, &obsEnd1, intersectionPoint1);
    if (success0) {
      success0 = obsSlightAppendShortenedSegment(tempObs, &obsEnd2, intersectionPoint2);
    }
  } else {
    success0 = obsSlightAppendShortenedSegment(tempObs, &obsEnd2, intersectionPoint1);
    if (success0) {
      success0 = obsSlightAppendShortenedSegment(tempObs, &obsEnd1, intersectionPoint2);
    }
  }

  return success0;
}

static Boolean obsSlightSplitLineByRect(VehicleCollisionCheckInput_t *tempObs,
                                        const ObsSlightLine_t *obsLine,
                                        const ObsSlightRect_t *rect,
                                        Boolean end1Inside,
                                        Boolean end2Inside) {
  Boolean success0;

  if (end1Inside && end2Inside) {
    success0 = true;
  } else {
    if (!end1Inside && !end2Inside) {
      Integer rectLineIndex0;
      Integer intersectionCount0;
      Boolean searchActive0;
      Boolean lineParallel0;
      ObsSlightPoint_t intersectionPoint1;
      ObsSlightPoint_t intersectionPoint2;
      ObsSlightLine_t parallelLine0;

      rectLineIndex0 = 0;
      intersectionCount0 = 0;
      searchActive0 = true;
      lineParallel0 = false;
      intersectionPoint1 = obsSlightMakePoint(0.0f, 0.0f);
      intersectionPoint2 = obsSlightMakePoint(0.0f, 0.0f);
      parallelLine0 = obsSlightRectLineAt(rect, 0);
      while ((rectLineIndex0 < 4) && searchActive0) {
        ObsSlightLine_t rectLine0;
        ObsSlightIntersection_t intersection0;

        rectLine0 = obsSlightRectLineAt(rect, rectLineIndex0);
        intersection0 = obsSlightLineSegmentIntersection(obsLine, &rectLine0);
        lineParallel0 = intersection0.LineParallel;
        if (intersection0.HasIntersection) {
          intersectionCount0 = intersectionCount0 + 1;
          if (intersectionCount0 == 1) {
            intersectionPoint1 = intersection0.Point;
          }
          if (intersectionCount0 == 2) {
            intersectionPoint2 = intersection0.Point;
            searchActive0 = false;
          }
        }
        if (intersection0.LineParallel) {
          parallelLine0 = rectLine0;
        }
        rectLineIndex0 = rectLineIndex0 + 1;
      }

      if (lineParallel0) {
        success0 = obsSlightAppendParallelSegment(tempObs, obsLine, rect, &parallelLine0);
      } else {
        success0 = obsSlightAppendOutsideSegments(
            tempObs, obsLine, &intersectionPoint1, &intersectionPoint2);
      }
    } else {
      Integer rectLineIndex1;
      Boolean searchActive1;
      ObsSlightPoint_t intersectionPoint3;
      ObsSlightPoint_t outsideEnd0;

      rectLineIndex1 = 0;
      searchActive1 = true;
      intersectionPoint3 = obsSlightMakePoint(0.0f, 0.0f);
      while ((rectLineIndex1 < 4) && searchActive1) {
        ObsSlightLine_t rectLine1;
        ObsSlightIntersection_t intersection1;

        rectLine1 = obsSlightRectLineAt(rect, rectLineIndex1);
        intersection1 = obsSlightLineSegmentIntersection(obsLine, &rectLine1);
        if (intersection1.HasIntersection) {
          intersectionPoint3 = intersection1.Point;
          searchActive1 = false;
        }
        rectLineIndex1 = rectLineIndex1 + 1;
      }

      if (end1Inside) {
        outsideEnd0 = obsSlightMakePoint(obsLine->X2, obsLine->Y2);
      } else {
        outsideEnd0 = obsSlightMakePoint(obsLine->X1, obsLine->Y1);
      }
      success0 = obsSlightAppendShortenedSegment(tempObs, &outsideEnd0, &intersectionPoint3);
    }
  }

  return success0;
}

static Boolean obsSlightMoveBoundaryLine(const ObsSlightAdjustForSpecificPosInput_t *input,
                                         VehicleCollisionCheckInput_t *tempObs,
                                         const VehicleCollisionCheckOutput_t *collisionOutput,
                                         const ObsSlightPoint_t *biasPoint) {
  Boolean success0;
  Integer lineIndex0;

  success0 = false;
  lineIndex0 = collisionOutput->CollisionIndex - 1;
  if ((lineIndex0 >= 0) && (lineIndex0 < tempObs->BoundaryLineNum) &&
      (lineIndex0 < VCC_MAX_BOUNDARY_LINES)) {
    ObsSlightLine_t obsLine0;
    ObsSlightRect_t rect0;
    ObsSlightPoint_t obsEnd1;
    ObsSlightPoint_t obsEnd2;
    Boolean end1Inside0;
    Boolean end2Inside0;

    obsLine0.X1 = tempObs->BoundaryLineX1[lineIndex0];
    obsLine0.Y1 = tempObs->BoundaryLineY1[lineIndex0];
    obsLine0.X2 = tempObs->BoundaryLineX2[lineIndex0];
    obsLine0.Y2 = tempObs->BoundaryLineY2[lineIndex0];

    obsSlightAppendOriginalLineEndsIfRoom(tempObs, &obsLine0);

    tempObs->BoundaryLineX1[lineIndex0] = obsLine0.X1 + biasPoint->X;
    tempObs->BoundaryLineY1[lineIndex0] = obsLine0.Y1 + biasPoint->Y;
    tempObs->BoundaryLineX2[lineIndex0] = obsLine0.X2 + biasPoint->X;
    tempObs->BoundaryLineY2[lineIndex0] = obsLine0.Y2 + biasPoint->Y;

    obsSlightCalcRect(input,
                      collisionOutput->SafeMargin1stBound[0],
                      collisionOutput->SafeMargin1stBound[1],
                      collisionOutput->SafeMargin1stBound[2],
                      &rect0);
    obsEnd1 = obsSlightMakePoint(obsLine0.X1, obsLine0.Y1);
    obsEnd2 = obsSlightMakePoint(obsLine0.X2, obsLine0.Y2);
    end1Inside0 = obsSlightPointInRect(&rect0, &obsEnd1);
    end2Inside0 = obsSlightPointInRect(&rect0, &obsEnd2);
    success0 = obsSlightSplitLineByRect(tempObs, &obsLine0, &rect0, end1Inside0, end2Inside0);
  }

  return success0;
}

static Boolean obsSlightRunLineLoop(const ObsSlightAdjustForSpecificPosInput_t *input,
                                    const ObsSlightBiasBase_t *biasBase,
                                    VehicleCollisionCheckInput_t *tempObs,
                                    VehicleCollisionCheckInput_t *collisionObs,
                                    ObsSlightAdjustForSpecificPosOutput_t *output) {
  Boolean valid0;
  Boolean loopActive0;
  Integer processCount0;

  valid0 = true;
  loopActive0 = true;
  processCount0 = 0;

  while ((processCount0 < OBS_SLIGHT_MAX_PROCESS_COUNT) && loopActive0 && valid0) {
    VehicleCollisionCheckOutput_t collisionOutput0;

    vehicleCollisionCheck(collisionObs, &collisionOutput0);
    processCount0 = processCount0 + 1;
    output->LineProcessCount = processCount0;

    if (!collisionOutput0.IsCollision) {
      loopActive0 = false;
    } else {
      Boolean reject0;

      reject0 = obsSlightRejectCollision(input, &collisionOutput0, output);
      if (reject0) {
        valid0 = false;
      } else {
        ObsSlightPoint_t biasPoint0;
        Boolean biasValid0;

        obsSlightMarkCollisionArea(&collisionOutput0, output);
        biasValid0 = obsSlightSelectBias(&collisionOutput0, biasBase, &biasPoint0);
        if (!biasValid0) {
          valid0 = false;
        } else {
          valid0 = obsSlightMoveBoundaryLine(input, tempObs, &collisionOutput0, &biasPoint0);
          if (valid0) {
            obsSlightCopyBoundaryLines(tempObs, collisionObs);
          }
        }
      }
    }
  }

  return valid0;
}

static Boolean obsSlightMoveBoundaryPoint(VehicleCollisionCheckInput_t *collisionObs,
                                          const VehicleCollisionCheckOutput_t *collisionOutput,
                                          const ObsSlightPoint_t *biasPoint) {
  Boolean success0;
  Integer pointIndex0;

  success0 = false;
  pointIndex0 = collisionOutput->CollisionIndex - 1;
  if ((pointIndex0 >= 0) && (pointIndex0 < collisionObs->BoundarySepPtsNum) &&
      (pointIndex0 < VCC_MAX_SEPARATE_PTS)) {
    collisionObs->BoundarySepPtsX[pointIndex0] =
        collisionObs->BoundarySepPtsX[pointIndex0] + biasPoint->X;
    collisionObs->BoundarySepPtsY[pointIndex0] =
        collisionObs->BoundarySepPtsY[pointIndex0] + biasPoint->Y;
    success0 = true;
  }

  return success0;
}

static Boolean obsSlightRunPointLoop(const ObsSlightAdjustForSpecificPosInput_t *input,
                                     const ObsSlightBiasBase_t *biasBase,
                                     VehicleCollisionCheckInput_t *collisionObs,
                                     ObsSlightAdjustForSpecificPosOutput_t *output) {
  Boolean valid0;
  Boolean loopActive0;
  Integer processCount0;

  valid0 = true;
  loopActive0 = true;
  processCount0 = 0;

  while ((processCount0 < OBS_SLIGHT_MAX_PROCESS_COUNT) && loopActive0 && valid0) {
    VehicleCollisionCheckOutput_t collisionOutput0;

    vehicleCollisionCheck(collisionObs, &collisionOutput0);
    processCount0 = processCount0 + 1;
    output->PointProcessCount = processCount0;

    if (!collisionOutput0.IsCollision) {
      loopActive0 = false;
    } else {
      Boolean reject0;

      reject0 = obsSlightRejectCollision(input, &collisionOutput0, output);
      if (reject0) {
        valid0 = false;
      } else {
        ObsSlightPoint_t biasPoint0;
        Boolean biasValid0;

        obsSlightMarkCollisionArea(&collisionOutput0, output);
        biasValid0 = obsSlightSelectBias(&collisionOutput0, biasBase, &biasPoint0);
        if (!biasValid0) {
          valid0 = false;
        } else {
          valid0 = obsSlightMoveBoundaryPoint(collisionObs, &collisionOutput0, &biasPoint0);
        }
      }
    }
  }

  return valid0;
}

/**
 * @brief Slightly move obstacle boundary lines and separate points away from a
 *        specific vehicle pose until the configured collision check clears.
 * @en_name obsSlightAdjustForSpecificPos
 * @cn_name 特定位姿障碍物微调
 * @type module
 * @param[IN] Integer CheckType collision check type
 * @param[IN] Real PosX vehicle/reference pose X
 * @param[IN] Real PosY vehicle/reference pose Y
 * @param[IN] Real PosTheta vehicle/reference pose heading
 * @param[IN] Boolean SlotPtsValid explicit slot points valid
 * @param[IN] Real SlotPts slot corner points
 * @param[IN] Integer SlotClass slot class
 * @param[IN] Boolean ConsiderCollisionDepthFlag reject deep collisions
 * @param[IN] Boolean ParkingOutFlag parking-out collision mode
 * @param[IN] VehicleCollisionCheckInput_t StObs source obstacle context
 * @param[OUT] Boolean ObsValid adjustment success flag
 * @param[OUT] VehicleCollisionCheckInput_t StObs adjusted obstacle context
 * @param[OUT] Boolean VehCollisionArea front/rear/left/right/center flags
 * @param[OUT] Integer LineProcessCount line loop count
 * @param[OUT] Integer PointProcessCount point loop count
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 障碍物调整
 * @source matlab_rewrite obs_slight_adjust_for_specific_pos.m
 * @version 1.0
 * @date 2026-05-06
 * @author aide
 */
void obsSlightAdjustForSpecificPos(const ObsSlightAdjustForSpecificPosInput_t *input,
                                   ObsSlightAdjustForSpecificPosOutput_t *output) {
  ObsSlightBiasBase_t biasBase0;
  VehicleCollisionCheckInput_t tempObs0;
  VehicleCollisionCheckInput_t collisionObs0;
  Boolean valid0;

  obsSlightResetOutput(input, output);
  tempObs0 = input->StObs;
  obsSlightCalcBiasBase(input, &biasBase0);

  obsSlightBuildLineCollisionObs(input, &tempObs0, &collisionObs0);
  valid0 = obsSlightRunLineLoop(input, &biasBase0, &tempObs0, &collisionObs0, output);

  if (valid0) {
    obsSlightBuildPointCollisionObs(input, &tempObs0, &collisionObs0);
    valid0 = obsSlightRunPointLoop(input, &biasBase0, &collisionObs0, output);
  }

  if (valid0) {
    obsSlightCopyBoundaryPoints(&collisionObs0, &tempObs0);
    output->StObs = tempObs0;
    output->ObsValid = true;
  }
}
