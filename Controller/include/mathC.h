#ifndef MATHC_H
#define MATHC_H
#include <math.h>
#include "trajPoint.h"
#ifdef __cplusplus
extern "C" {
#endif

void smoothCurvatureSG9(const Traj* traj, double* out);
double limitSymmetrical(double value, double limit);
double limit(double v, double min, double max);

#ifdef __cplusplus
}
#endif
#endif // MATHC_H

