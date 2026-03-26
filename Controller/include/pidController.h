#ifndef FUNC_PID_H
#define FUNC_PID_H

#include "mathC.h"

typedef struct
{
    double integral;
    double lastError;
} PIDState;


#ifdef __cplusplus
extern "C" {
#endif

double pid(
    double kp,double ki,double kd,double dt,
    double integral_max,double derivative_max,
    double error,
    PIDState* state
);

#ifdef __cplusplus
}
#endif

#endif /* FUNC_PID_H */
