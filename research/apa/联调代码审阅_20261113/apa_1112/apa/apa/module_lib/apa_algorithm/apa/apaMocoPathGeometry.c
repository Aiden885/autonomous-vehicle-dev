#include "apaMocoPathGeometry.h"
#include "apaCoordUtils.h"
#include <math.h>

void apaMocoProjectPath(const Real *x,
                        const Real *y,
                        const Real *yaw,
                        Integer count,
                        Real vehicleX,
                        Real vehicleY,
                        Real vehicleYaw,
                        Integer lastIndex,
                        Real lastProgress,
                        Integer searchBack,
                        Integer searchAhead,
                        Real maxAdvance,
                        Real maxDistance,
                        Real maxHeading,
                        ApaMocoPathProjection_t *p) {
  Integer i, first, last;
  Real prefix = 0.0f, best = 1.0e30f;
  *p = (ApaMocoPathProjection_t){0};
  if (count < 2 || count > 100 || !isfinite(vehicleX) || !isfinite(vehicleY) ||
      !isfinite(vehicleYaw) || !isfinite(lastProgress))
    return;
  first = lastIndex - searchBack;
  if (first < 0)
    first = 0;
  last = lastIndex + searchAhead;
  if (last > count - 2)
    last = count - 2;
  for (i = 0; i < count - 1; ++i) {
    Real dx = x[i + 1] - x[i], dy = y[i + 1] - y[i];
    Real length = hypotf(dx, dy);
    if (!isfinite(length) || !isfinite(yaw[i]) || !isfinite(yaw[i + 1]))
      return;
    if (i >= first && i <= last && length > 1.0e-5f) {
      Real t = ((vehicleX - x[i]) * dx + (vehicleY - y[i]) * dy) / (length * length);
      Real clamped = fmaxf(0.0f, fminf(t, 1.0f));
      Real progress = prefix + clamped * length;
      if (i == count - 2 && t > 1.0f)
        progress = prefix + t * length;
      Real px = x[i] + clamped * dx, py = y[i] + clamped * dy;
      Real pyaw = apaCoordWrapToHalfOpenPi(yaw[i] +
                                           clamped * apaCoordWrapToHalfOpenPi(yaw[i + 1] - yaw[i]));
      Real error = apaCoordWrapToHalfOpenPi(pyaw - vehicleYaw);
      Real distance = hypotf(px - vehicleX, py - vehicleY);
      if (progress >= lastProgress - 0.15f && progress <= lastProgress + maxAdvance &&
          distance <= maxDistance && fabsf(error) <= maxHeading && distance < best) {
        best = distance;
        p->Index = i;
        p->Fraction = clamped;
        p->X = px;
        p->Y = py;
        p->Yaw = pyaw;
        p->Progress = progress;
        p->CrossTrackError =
            -sinf(vehicleYaw) * (px - vehicleX) + cosf(vehicleYaw) * (py - vehicleY);
        p->HeadingError = error;
        p->Valid = true;
      }
    }
    prefix += length;
  }
  p->Length = prefix;
  p->Remaining = prefix - p->Progress;
}

void apaMocoPathAtDistance(const Real *x,
                           const Real *y,
                           const Real *yaw,
                           Integer count,
                           Real distance,
                           Integer *index,
                           Real *targetX,
                           Real *targetY,
                           Real *targetYaw) {
  Integer i;
  Real remain = fmaxf(distance, 0.0f);
  *index = count - 1;
  *targetX = x[count - 1];
  *targetY = y[count - 1];
  *targetYaw = yaw[count - 1];
  for (i = 0; i < count - 1; ++i) {
    Real length = hypotf(x[i + 1] - x[i], y[i + 1] - y[i]);
    if (length > 1.0e-5f && remain <= length) {
      Real t = remain / length;
      *index = i;
      *targetX = x[i] + t * (x[i + 1] - x[i]);
      *targetY = y[i] + t * (y[i + 1] - y[i]);
      *targetYaw =
          apaCoordWrapToHalfOpenPi(yaw[i] + t * apaCoordWrapToHalfOpenPi(yaw[i + 1] - yaw[i]));
      return;
    }
    remain -= length;
  }
}
