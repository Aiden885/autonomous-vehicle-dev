#ifndef APA_PARKING_SEARCH_H
#define APA_PARKING_SEARCH_H

#include "apaPlanningScene.h"
#include "apaPathTypes.h"
#include <math.h>

/* Independent parking search; the MATLAB candidate adapters and acceptance
 * callbacks remain the boundary. No C++ resources cross this C interface. */
struct ApaHybridAStar;
#define APA_PARKING_SWITCH_COST 1.5
#define APA_PARKING_SHORT_RUN_COST 1.5
#define APA_PARKING_GOAL_OFFSET_WEIGHT 2.0
/* Transitive order: actual direction changes, geometric/clearance cost plus
 * 1.5 per short direction run, then short-run count and original cost as ties.
 * Keep an open run until it changes direction or the full path ends. */
typedef struct {
  int Changes, ShortRuns, Direction;
  double OpenLength;
} ApaParkingRunQuality_t;
typedef struct {
  int Changes, ShortRuns;
  double Cost;
} ApaParkingQuality_t;
static inline void apaParkingRunQualityAdvance(ApaParkingRunQuality_t *q, double distance) {
  if (fabs(distance) < 1.e-6)
    return;
  const int direction = distance > 0.0 ? 1 : -1;
  if (q->Direction && direction != q->Direction) {
    ++q->Changes;
    q->ShortRuns += apaPathDirectionRunIsShort((Real)q->OpenLength);
    q->OpenLength = 0.0;
  }
  q->Direction = direction;
  q->OpenLength = fmin(0.4, q->OpenLength + fabs(distance));
}
static inline ApaParkingQuality_t apaParkingRunQualityFinish(const ApaParkingRunQuality_t *q,
                                                             double cost) {
  const ApaParkingQuality_t result = {
      q->Changes,
      q->ShortRuns + (q->Direction && apaPathDirectionRunIsShort((Real)q->OpenLength)),
      cost};
  return result;
}
static inline int apaParkingQualityCompare(ApaParkingQuality_t a, ApaParkingQuality_t b) {
  if (a.Changes != b.Changes)
    return a.Changes < b.Changes ? -1 : 1;
  const double scoreA = a.Cost + APA_PARKING_SHORT_RUN_COST * a.ShortRuns;
  const double scoreB = b.Cost + APA_PARKING_SHORT_RUN_COST * b.ShortRuns;
  if (scoreA != scoreB)
    return scoreA < scoreB ? -1 : 1;
  if (a.ShortRuns != b.ShortRuns)
    return a.ShortRuns < b.ShortRuns ? -1 : 1;
  return a.Cost < b.Cost ? -1 : a.Cost > b.Cost ? 1 : 0;
}
static inline double
apaParkingGoalOffsetCost(double positionError, double yawError, double turnRadius) {
  return APA_PARKING_GOAL_OFFSET_WEIGHT * (positionError + turnRadius * fabs(yawError));
}
/* One geometric quality measure for templates, RS and graph motions. */
static inline double apaParkingSteeringChangeCost(double distance,
                                                  double curvature,
                                                  double previousCurvature,
                                                  int previousDirection,
                                                  double maximumCurvature) {
  if (!previousDirection)
    return 0.0;
  double cost = 0.10 * fabs(curvature - previousCurvature);
  if (previousDirection * distance > 0.0 && curvature * previousCurvature < 0.0)
    cost += 0.20 * fmin(fabs(curvature), fabs(previousCurvature));
  return cost / maximumCurvature;
}

static inline double apaParkingMotionCost(double distance,
                                          double curvature,
                                          double previousCurvature,
                                          int previousDirection,
                                          double maximumCurvature) {
  return fabs(distance) + 0.05 * fabs(distance * curvature) / maximumCurvature +
         (previousDirection && previousDirection * distance < 0 ? APA_PARKING_SWITCH_COST : 0) +
         apaParkingSteeringChangeCost(
             distance, curvature, previousCurvature, previousDirection, maximumCurvature);
}
#ifdef __cplusplus
extern "C" {
#endif
int apaParkingSearchRun(struct ApaHybridAStar *planner,
                        const VehicleCollisionCheckInput_t *environment,
                        const ApaPlanningSceneQuery_t *scene);
/* Evaluate direct RS candidates first, then continue the same bounded graph
 * for a solution or improvement. Execution failures never restart the graph. */
int apaParkingSearchLocal(struct ApaHybridAStar *planner,
                          const VehicleCollisionCheckInput_t *environment,
                          const ApaPlanningSceneQuery_t *scene);
#ifdef __cplusplus
}
#endif
#endif
