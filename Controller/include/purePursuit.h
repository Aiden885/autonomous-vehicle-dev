#ifndef GET_PURE_PURSUIT_STEER_H
#define GET_PURE_PURSUIT_STEER_H

#include "state.h"
#include "trajPoint.h"

#ifdef __cplusplus
extern "C" {
#endif

double purePursuit(const State* state, const TrajPoint* target, double wheelBase);

#ifdef __cplusplus
}
#endif

#endif /* GET_PURE_PURSUIT_STEER_H */
