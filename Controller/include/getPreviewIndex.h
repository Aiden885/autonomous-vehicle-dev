#ifndef GET_PREVIEW_INDEX_H
#define GET_PREVIEW_INDEX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "state.h"
#include "trajPoint.h"

int getPreviewIndex(const Traj* traj, double previewDist);

#ifdef __cplusplus
}
#endif

#endif // GET_PREVIEW_INDEX_H
