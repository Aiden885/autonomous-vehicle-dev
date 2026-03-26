#ifndef ACCELERATION_CONTROLLER_H
#define ACCELERATION_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

// #include <stddef.h>
#include "state.h"
#include "trajPoint.h"
#include "getNearestIndex.h"

double calcAcceleration(double vKp, double vKi, double vKd,
                        double aKp, double aKi, double aKd,
                        double integral_max, double derivative_max,
                        double dT,
                        const State* s, const Traj* traj);

#ifdef __cplusplus
}
#endif

#endif /* ACCELERATION_CONTROLLER_H */
