#ifndef APA_PLANNING_SLOT_GUIDANCE_H
#define APA_PLANNING_SLOT_GUIDANCE_H
#include "vehicleCollisionCheckTypes.h"
#include <math.h>

/* Slot geometry and optional search guidance; SlotPts are entrance-first. */
static inline Boolean apaPlanningSlotContainsPoint(const Real p[8], double x, double y) {
  Boolean positive = false, negative = false;
  double area = 0;
  for (Integer i = 0; i < 4; ++i) {
    const Integer j = (i + 1) % 4;
    const double ax = (double)p[2 * i] - x, ay = (double)p[2 * i + 1] - y;
    const double bx = (double)p[2 * j] - x, by = (double)p[2 * j + 1] - y;
    const double cross = ax * by - ay * bx;
    if (!isfinite(cross))
      return false;
    positive = positive || cross > 1.e-6;
    negative = negative || cross < -1.e-6;
    area += cross;
  }
  return fabs(area) > 1.e-6 && !(positive && negative);
}

/* One reference for soft-cost entry in both front-in and reverse-in parking. */
static inline void apaPlanningSlotBodyCenter(const VehicleCollisionCheckInput_t *input,
                                              double center[2]) {
  const double offset = 0.5 * ((double)input->VehLF - input->VehLB);
  center[0] = input->VehPosX + offset * cos(input->VehPosTheta);
  center[1] = input->VehPosY + offset * sin(input->VehPosTheta);
}

static inline Boolean
apaPlanningSlotBodyCenterIsInside(const VehicleCollisionCheckInput_t *input) {
  double center[2];
  apaPlanningSlotBodyCenter(input, center);
  return input->SlotPtsValid &&
         apaPlanningSlotContainsPoint(input->SlotPts, center[0], center[1]);
}

typedef struct {
  double X, Y, InwardX, InwardY;
} ApaPlanningSlotEntrance_t;

/* Pure geometry, independent of parking action and soft-cost activation. */
static inline Boolean apaPlanningSlotEntrance(const VehicleCollisionCheckInput_t *in,
                                               ApaPlanningSlotEntrance_t *entrance) {
  if (!in->SlotPtsValid)
    return false;
  const Real *p = in->SlotPts;
  for (Integer i = 0; i < 8; ++i)
    if (!isfinite(p[i]))
      return false;
  const double dx = (double)p[2] - p[0], dy = (double)p[3] - p[1], length = hypot(dx, dy);
  if (length <= 1.e-6)
    return false;
  double nx = -dy / length, ny = dx / length;
  const double depth = nx * (p[4] + (double)p[6] - p[0] - p[2]) / 2 +
                       ny * (p[5] + (double)p[7] - p[1] - p[3]) / 2;
  if (fabs(depth) <= 1.e-6)
    return false;
  if (depth < 0) {
    nx = -nx;
    ny = -ny;
  }
  *entrance = (ApaPlanningSlotEntrance_t){(p[0] + (double)p[2]) / 2,
                                          (p[1] + (double)p[3]) / 2, nx, ny};
  return true;
}

/* Signed straight travel to clear the entrance with the whole body. This is
 * only a seed hint: every prefix still needs the common hard swept check.
 * Parallel slots retain their curve search; no outward straight is assumed. */
static inline double
apaPlanningSlotStraightExitDistance(const VehicleCollisionCheckInput_t *in) {
  if (in->CheckType != VCC_CHECK_TYPE_PLANNING ||
      !vehicleCollisionCheckUsesCrossSlotMargins(in->SlotClass) ||
      !isfinite(in->VehLF) || !isfinite(in->VehLB) || !isfinite(in->VehW) ||
      in->VehLF <= 0 || in->VehLB < 0 || in->VehW <= 0 ||
      !apaPlanningSlotBodyCenterIsInside(in))
    return 0;
  ApaPlanningSlotEntrance_t entrance;
  if (!apaPlanningSlotEntrance(in, &entrance))
    return 0;
  const double c = cos(in->VehPosTheta), s = sin(in->VehPosTheta);
  const double projection = entrance.InwardX * c + entrance.InwardY * s;
  if (fabs(projection) < 0.25)
    return 0;
  const double depth = entrance.InwardX * (in->VehPosX - entrance.X) +
                       entrance.InwardY * (in->VehPosY - entrance.Y);
  const double extent = fmax(in->VehLF * projection, -in->VehLB * projection) +
                        in->VehW / 2 * fabs(-entrance.InwardX * s + entrance.InwardY * c);
  const double distance = (depth + extent) / fabs(projection);
  return isfinite(distance) && distance > 0 ? copysign(distance, -projection) : 0;
}

/* Edge matching requires finite overlap; 1 cm classification tolerance
 * does not change hard geometry or admission margins. */
static inline Integer
apaPlanningSlotLineEdge(const Real p[8], double x1, double y1, double x2, double y2) {
  for (Integer i = 0; i < 4; ++i) {
    const Integer j = (i + 1) % 4;
    const double dx = (double)p[2 * j] - p[2 * i], dy = (double)p[2 * j + 1] - p[2 * i + 1];
    const double length = hypot(dx, dy);
    if (length <= 1.e-6)
      continue;
    const double ax = x1 - p[2 * i], ay = y1 - p[2 * i + 1];
    const double bx = x2 - p[2 * i], by = y2 - p[2 * i + 1];
    if (fabs(dx * ay - dy * ax) > 0.01 * length || fabs(dx * by - dy * bx) > 0.01 * length)
      continue;
    const double a = (ax * dx + ay * dy) / length, b = (bx * dx + by * dy) / length;
    if (fmin(length, fmax(a, b)) - fmax(0, fmin(a, b)) > 1.e-4)
      return i;
  }
  return -1;
}

static inline void apaPlanningSlotSoftVirtualLines(const VehicleCollisionCheckInput_t *input,
                                                   VehicleCollisionCheckInput_t *soft) {
  const Boolean inside = apaPlanningSlotBodyCenterIsInside(input);
  soft->VirtualBoundaryLineNum = 0;
  for (Integer i = 0; i < input->VirtualBoundaryLineNum && i < VCC_MAX_VIRTUAL_LINES; ++i) {
    double x1 = input->VirtualBoundaryLineX1[i], y1 = input->VirtualBoundaryLineY1[i];
    double x2 = input->VirtualBoundaryLineX2[i], y2 = input->VirtualBoundaryLineY2[i];
    if (!isfinite(x1) || !isfinite(y1) || !isfinite(x2) || !isfinite(y2))
      continue;
    const Integer edge = apaPlanningSlotLineEdge(input->SlotPts, x1, y1, x2, y2);
    if (edge == 0 || (inside && edge < 0))
      continue;
    if (inside && edge > 0) {

      const Integer next = (edge + 1) % 4;
      const double dx = (double)input->SlotPts[2 * next] - input->SlotPts[2 * edge];
      const double dy = (double)input->SlotPts[2 * next + 1] - input->SlotPts[2 * edge + 1];
      const double n = dx * dx + dy * dy;
      if (!isfinite(n) || n <= 1.e-12)
        continue;
      const double a =
          ((x1 - input->SlotPts[2 * edge]) * dx + (y1 - input->SlotPts[2 * edge + 1]) * dy) / n;
      const double b =
          ((x2 - input->SlotPts[2 * edge]) * dx + (y2 - input->SlotPts[2 * edge + 1]) * dy) / n;
      const double span = b - a;
      const double lower = fmax(0, fmin(a, b)), upper = fmin(1, fmax(a, b));
      if (!isfinite(a) || !isfinite(b) || !isfinite(span) || fabs(span) <= 1.e-12 ||
          !(lower < upper))
        continue;
      const double lo = fmax(0, fmin(1, (lower - a) / span));
      const double hi = fmax(0, fmin(1, (upper - a) / span));
      const double ax = x1, ay = y1;
      x1 = ax + lo * (x2 - ax);
      y1 = ay + lo * (y2 - ay);
      x2 = ax + hi * (x2 - ax);
      y2 = ay + hi * (y2 - ay);
    }
    const Integer at = soft->VirtualBoundaryLineNum++;
    soft->VirtualBoundaryLineX1[at] = x1;
    soft->VirtualBoundaryLineY1[at] = y1;
    soft->VirtualBoundaryLineX2[at] = x2;
    soft->VirtualBoundaryLineY2[at] = y2;
  }
}

/* Guidance fades in over half a vehicle length after the body centre enters the slot. */
static inline double apaPlanningSlotDepthWeight(const VehicleCollisionCheckInput_t *in) {
  if (in->CheckType != VCC_CHECK_TYPE_PLANNING || in->ParkingOutFlag ||
      !vehicleCollisionCheckUsesCrossSlotMargins(in->SlotClass) || !in->SlotPtsValid ||
      !in->TarPosValid || !isfinite(in->TarPosX) || !isfinite(in->TarPosY) ||
      !isfinite(in->TarPosTheta))
    return 0;
  double center[2];
  apaPlanningSlotBodyCenter(in, center);
  const double x = center[0], y = center[1];
  if (!apaPlanningSlotContainsPoint(in->SlotPts, x, y))
    return 0;
  ApaPlanningSlotEntrance_t entrance;
  if (!apaPlanningSlotEntrance(in, &entrance))
    return 0;
  const double depth = entrance.InwardX * (x - entrance.X) + entrance.InwardY * (y - entrance.Y);
  const double t = fmax(0, fmin(1, depth / fmax(0.5, 0.5 * ((double)in->VehLF + in->VehLB))));
  return t * t * (3 - 2 * t);
}

static inline Real apaPlanningSlotAlignmentPenalty(const VehicleCollisionCheckInput_t *in) {
  const double weight = apaPlanningSlotDepthWeight(in);
  if (weight <= 0)
    return 0;
  double center[2];
  apaPlanningSlotBodyCenter(in, center);
  const double dx = center[0] - in->TarPosX;
  const double dy = center[1] - in->TarPosY;
  const double lateral = -sin(in->TarPosTheta) * dx + cos(in->TarPosTheta) * dy;

  const double angle =
      atan2(sin(in->VehPosTheta - in->TarPosTheta), cos(in->VehPosTheta - in->TarPosTheta));
  const double heading = ((double)in->VehLF + in->VehLB) * sin(angle / 2);
  return (Real)(weight * fmin(2.0, (lateral * lateral + heading * heading) / (0.25 * 0.25)));
}
#endif
