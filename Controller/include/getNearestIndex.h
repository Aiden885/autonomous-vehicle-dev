#ifndef GET_NEAREST_INDEX_H
#define GET_NEAREST_INDEX_H

#include "state.h"
#include "trajPoint.h"

#ifdef __cplusplus
extern "C" {
#endif

int getNearestIndex(const State* state, const Traj* traj);

#ifdef __cplusplus
}
#endif

#endif /* GET_NEAREST_INDEX_H */
