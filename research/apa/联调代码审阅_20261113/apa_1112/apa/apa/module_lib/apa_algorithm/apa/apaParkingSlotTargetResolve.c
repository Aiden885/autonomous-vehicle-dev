#include "apaParkingSlotTargetResolve.h"
#include "apaPlanningTypes.h"
#include "apaCoordUtils.h"
#include <math.h>

static Boolean apaParkingSlotTargetCoordinatesAreFinite(const Real points[8]) {
  Integer index;
  Boolean valid0;
  index = 0;
  valid0 = true;
  while (index < 8) {
    Boolean finite0;
    finite0 = isfinite(points[index]);
    if (!finite0) {
      valid0 = false;
    }
    index = index + 1;
  }
  return valid0;
}

static Boolean apaParkingSlotTargetQuadIsConvex(const Real points[8]) {
  Integer index;
  Real firstCross;
  Boolean valid0;
  index = 0;
  firstCross = 0.0f;
  valid0 = true;
  while (index < 4) {
    Integer next0;
    Integer after0;
    Real dx0;
    Real dy0;
    Real dx1;
    Real dy1;
    Real cross0;
    Boolean crossValid0;
    next0 = (index + 1) % 4;
    after0 = (index + 2) % 4;
    dx0 = points[2 * next0] - points[2 * index];
    dy0 = points[2 * next0 + 1] - points[2 * index + 1];
    dx1 = points[2 * after0] - points[2 * next0];
    dy1 = points[2 * after0 + 1] - points[2 * next0 + 1];
    cross0 = dx0 * dy1 - dy0 * dx1;
    crossValid0 = isfinite(cross0) && (fabsf(cross0) > 1.0e-4f);
    if (index == 0) {
      firstCross = cross0;
    } else {
      crossValid0 = crossValid0 && ((cross0 > 0.0f) == (firstCross > 0.0f));
    }
    if (!crossValid0) {
      valid0 = false;
    }
    index = index + 1;
  }
  return valid0;
}

static Boolean apaParkingSlotTargetBuildGeometry(const ApaParkingSlotTargetResolveInput_t *input,
                                                 ApaParkingSlotTargetResolveOutput_t *output) {
  Integer entry0;
  Integer entryNext0;
  Integer back0;
  Integer backNext0;
  Real entryWidth0;
  Real backWidth0;
  Real axisX0;
  Real axisY0;
  Boolean valid0;
  entry0 = input->EnterEdge - 1;
  entryNext0 = (entry0 + 1) % 4;
  back0 = (entry0 + 2) % 4;
  backNext0 = (entry0 + 3) % 4;
  output->EntryX = 0.5f * input->SlotPts[2 * entry0] + 0.5f * input->SlotPts[2 * entryNext0];
  output->EntryY =
      0.5f * input->SlotPts[2 * entry0 + 1] + 0.5f * input->SlotPts[2 * entryNext0 + 1];
  output->BackX = 0.5f * input->SlotPts[2 * back0] + 0.5f * input->SlotPts[2 * backNext0];
  output->BackY = 0.5f * input->SlotPts[2 * back0 + 1] + 0.5f * input->SlotPts[2 * backNext0 + 1];
  output->CenterX = 0.5f * output->EntryX + 0.5f * output->BackX;
  output->CenterY = 0.5f * output->EntryY + 0.5f * output->BackY;
  axisX0 = output->EntryX - output->BackX;
  axisY0 = output->EntryY - output->BackY;
  output->Depth = hypotf(axisX0, axisY0);
  entryWidth0 = hypotf(input->SlotPts[2 * entry0] - input->SlotPts[2 * entryNext0],
                       input->SlotPts[2 * entry0 + 1] - input->SlotPts[2 * entryNext0 + 1]);
  backWidth0 = hypotf(input->SlotPts[2 * back0] - input->SlotPts[2 * backNext0],
                      input->SlotPts[2 * back0 + 1] - input->SlotPts[2 * backNext0 + 1]);
  output->Width = fminf(entryWidth0, backWidth0);
  output->HeadingToEntry = atan2f(axisY0, axisX0);
  valid0 = (output->Depth > 1.0e-3f) && (output->Depth <= 1000.0f) && (output->Width > 1.0e-3f) &&
           (output->Width <= 1000.0f);
  return valid0;
}

static void apaParkingSlotTargetDescribeRequest(const ApaParkingSlotTargetResolveInput_t *input,
                                                ApaParkingSlotTargetResolveOutput_t *output) {
  if (input->ParkingAction != APA_SCENARIO_PARKING_ACTION_PARKOUT) {
    Real heading = apaCoordWrapToPi(input->RequestedTargetTheta);
    Real cosine = cosf(heading - output->HeadingToEntry);
    Real sine = fabsf(sinf(heading - output->HeadingToEntry));
    Real entryExtent =
        fmaxf(cosine * input->VehFront, -cosine * input->VehRear) + 0.5f * input->VehWidth * sine;
    Real backExtent =
        fmaxf(-cosine * input->VehFront, cosine * input->VehRear) + 0.5f * input->VehWidth * sine;
    Real depth = (output->EntryX - input->RequestedTargetX) * cosf(output->HeadingToEntry) +
                 (output->EntryY - input->RequestedTargetY) * sinf(output->HeadingToEntry);
    output->BackClearance = output->Depth - depth - backExtent;
    output->EntryOverhang = fmaxf(entryExtent - depth, 0.0f);
  }
}

void apaParkingSlotTargetResolve(const ApaParkingSlotTargetResolveInput_t *input,
                                 ApaParkingSlotTargetResolveOutput_t *output) {
  Boolean parkout = input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKOUT;
  Boolean supported = (input->SlotKind == APA_SCENARIO_SLOT_KIND_VT) ||
                      (input->SlotKind == APA_SCENARIO_SLOT_KIND_HT) ||
                      (input->SlotKind == APA_SCENARIO_SLOT_KIND_DTH_ED_VT) ||
                      (input->SlotKind == APA_SCENARIO_SLOT_KIND_VH) ||
                      (parkout && input->SlotKind == APA_SCENARIO_SLOT_KIND_PARKOUT);
  *output = (ApaParkingSlotTargetResolveOutput_t){0};
  output->Status = APA_SLOT_TARGET_INVALID_REQUEST;
  if (!supported || ((input->ParkingAction != 0) &&
                     (input->ParkingAction != APA_SCENARIO_PARKING_ACTION_PARKIN) && !parkout)) {
    output->Status = APA_SLOT_TARGET_UNSUPPORTED;
  } else if ((input->EnterEdge < 1) || (input->EnterEdge > 4)) {
    output->Status = APA_SLOT_TARGET_INVALID_ENTRY;
  } else if (!isfinite(input->RequestedTargetX) || !isfinite(input->RequestedTargetY) ||
             !isfinite(input->RequestedTargetTheta)) {
    output->Status = APA_SLOT_TARGET_INVALID_REQUEST;
  } else if (!isfinite(input->VehFront) || !isfinite(input->VehRear) ||
             !isfinite(input->VehWidth) || (input->VehFront <= 0.0f) || (input->VehRear < 0.0f) ||
             (input->VehWidth <= 0.0f)) {
    output->Status = APA_SLOT_TARGET_INVALID_MODEL_POLICY;
  } else if (!apaParkingSlotTargetCoordinatesAreFinite(input->SlotPts) ||
             !apaParkingSlotTargetQuadIsConvex(input->SlotPts) ||
             !apaParkingSlotTargetBuildGeometry(input, output)) {
    output->Status = APA_SLOT_TARGET_INVALID_GEOMETRY;
  } else {
    apaParkingSlotTargetDescribeRequest(input, output);
    output->Status = APA_SLOT_TARGET_OK;
    output->Valid = true;
  }
}
