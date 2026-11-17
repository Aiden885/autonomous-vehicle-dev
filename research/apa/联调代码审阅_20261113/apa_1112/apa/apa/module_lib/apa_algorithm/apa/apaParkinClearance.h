#ifndef APA_PARKIN_CLEARANCE_H
#define APA_PARKIN_CLEARANCE_H

#include "apaVehicleDefaults.h"
#include "vehicleCollisionCheckTypes.h"
#include <math.h>

/* Numerical comparison tolerance for float world coordinates (0.1 mm). */
#define APA_PARKIN_CLEARANCE_EPS_M 1.e-4

/* Reduced bottom clearance applies only behind the opposite entrance edge
 * within its span. Intruding obstacles and other edges require the full envelope. */
typedef struct {
  double x, y, tx, ty, nx, ny, length;
} ApaParkinBottom_t;

static inline Boolean apaParkinBottom(const VehicleCollisionCheckInput_t *in,
                                      ApaParkinBottom_t *out) {
  if (in->CheckType != VCC_CHECK_TYPE_PLANNING || !in->SlotPtsValid)
    return false;
  const Real *p = in->SlotPts;
  out->x = p[4];
  out->y = p[5];
  out->length = hypot((double)p[6] - p[4], (double)p[7] - p[5]);
  if (out->length <= 1.e-6)
    return false;
  out->tx = (p[6] - p[4]) / out->length;
  out->ty = (p[7] - p[5]) / out->length;
  out->nx = -out->ty;
  out->ny = out->tx;
  if (out->nx * ((p[0] + p[2]) / 2.0 - out->x) + out->ny * ((p[1] + p[3]) / 2.0 - out->y) < 0.0) {
    out->nx = -out->nx;
    out->ny = -out->ny;
  }
  return true;
}

static inline double apaParkinBottomDepth(const ApaParkinBottom_t *b, double x, double y) {
  return (x - b->x) * b->nx + (y - b->y) * b->ny;
}
static inline double apaParkinBottomAlong(const ApaParkinBottom_t *b, double x, double y) {
  return (x - b->x) * b->tx + (y - b->y) * b->ty;
}
static inline Boolean apaParkinBottomSpan(const ApaParkinBottom_t *b, double lo, double hi) {
  return hi >= -APA_PARKIN_CLEARANCE_EPS_M && lo <= b->length + APA_PARKIN_CLEARANCE_EPS_M;
}

/* Explicit body-only checks and soft-cost bands retain their supplied margins. */
static inline double apaParkinBottomMargin(const VehicleCollisionCheckInput_t *input,
                                           double front,
                                           double rear,
                                           double side) {
  return fmax(0.0,
              fmin(front, fmin(rear, side)) -
                  (input->SlotClass == VCC_SLOT_CLASS_PARALLEL
                       ? 0.0
                       : APA_PARKIN_PHYSICAL_MARGIN_M - APA_PARKIN_BACK_MARGIN_M));
}
static inline double apaParkinVirtualBottomMargin(double front, double rear, double side) {
  return fmax(0.0,
              fmin(front, fmin(rear, side)) - (APA_PARKIN_MARGIN_M - APA_PARKIN_BACK_MARGIN_M));
}
#endif
