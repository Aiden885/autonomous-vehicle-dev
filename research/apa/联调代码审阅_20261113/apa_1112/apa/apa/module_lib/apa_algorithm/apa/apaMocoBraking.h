#ifndef APA_MOCO_BRAKING_H
#define APA_MOCO_BRAKING_H
#include "baseTypes.h"
#include "apaMocoActuator.h"
#include <math.h>

typedef struct {
  Real AccelerationUpperLimit, AccelerationLowerLimit, StopDeceleration;
  Real ReactionTime, StopBuffer;
} ApaMocoBrakingParam_t;

/* Shared stopping budget: distance includes StopBuffer; time ends at zero speed.
 * Invalid input returns the 1000 m failure distance. */
typedef struct {
  Real Speed, Acceleration, Jerk, Deceleration;
  Real ReactionTime, ReactionSpeed, ReactionDistance;
  Real RampTime, RampSpeed, RampDistance;
  Real StopTime, Distance;
  Boolean Valid;
} ApaMocoBrakingProfile_t;

static inline ApaMocoBrakingProfile_t
apaMocoBuildBrakingProfile(Real velocity,
                           Real feedbackAcceleration,
                           Real lastAcceleration,
                           const ApaMocoBrakingParam_t *param) {
  ApaMocoBrakingProfile_t profile = {0};
  Real speed0;
  Real direction0;
  const Real jerk0 = APA_MOCO_BRAKE_BUILD_JERK_MPS3;
  Real deceleration0;
  Real feedbackAcceleration0;
  Real commandAcceleration0;
  Real acceleration0;
  Real rampTime0;
  Real rampTravel0;
  Real rampSpeed0;
  Real stopTravel0;
  Real reactionSpeed0;
  Real reactionTravel0;
  Real distance0;
  Boolean valid0;
  direction0 = 1.0f;
  deceleration0 = -param->AccelerationLowerLimit;
  if (velocity < 0.0f) {
    direction0 = -1.0f;
    deceleration0 = param->AccelerationUpperLimit;
  }
  deceleration0 = fminf(deceleration0, param->StopDeceleration);

  valid0 = isfinite(velocity) && isfinite(feedbackAcceleration) && isfinite(lastAcceleration) &&
           isfinite(deceleration0) && isfinite(param->ReactionTime) &&
           isfinite(param->StopBuffer) && (deceleration0 > 0.0f) && (param->ReactionTime >= 0.0f) &&
           (param->StopBuffer >= 0.0f);
  distance0 = 1000.0f;
  if (valid0) {
    speed0 = fabsf(velocity);
    feedbackAcceleration0 = direction0 * feedbackAcceleration;
    commandAcceleration0 = direction0 * lastAcceleration;
    acceleration0 = fmaxf(feedbackAcceleration0, commandAcceleration0);
    acceleration0 = fmaxf(acceleration0, 0.0f);
    reactionTravel0 = (speed0 * param->ReactionTime) +
                      (0.5f * acceleration0 * param->ReactionTime * param->ReactionTime);
    reactionSpeed0 = speed0 + (acceleration0 * param->ReactionTime);

    rampTime0 = fminf(deceleration0 / jerk0, sqrtf(2.0f * reactionSpeed0 / jerk0));
    rampTravel0 = reactionSpeed0 * rampTime0 - jerk0 * rampTime0 * rampTime0 * rampTime0 / 6.0f;
    rampSpeed0 = fmaxf(0.0f, reactionSpeed0 - 0.5f * jerk0 * rampTime0 * rampTime0);
    stopTravel0 = (rampSpeed0 * rampSpeed0) / (2.0f * deceleration0);
    distance0 = rampTravel0 + stopTravel0 + reactionTravel0 + param->StopBuffer;
    profile.Speed = speed0;
    profile.Acceleration = acceleration0;
    profile.Jerk = jerk0;
    profile.Deceleration = deceleration0;
    profile.ReactionTime = param->ReactionTime;
    profile.ReactionSpeed = reactionSpeed0;
    profile.ReactionDistance = reactionTravel0;
    profile.RampTime = rampTime0;
    profile.RampSpeed = rampSpeed0;
    profile.RampDistance = rampTravel0;
    profile.StopTime = param->ReactionTime + rampTime0 + rampSpeed0 / deceleration0;
    profile.Valid = isfinite(distance0) && isfinite(profile.StopTime);
  }
  profile.Distance = distance0;
  return profile;
}

/* Three-phase travel excludes the final spatial buffer. */
static inline double apaMocoBrakingDistanceAtTime(const ApaMocoBrakingProfile_t *p, double time) {
  const double t = fmax(0.0, fmin(time, p->StopTime));
  if (t <= p->ReactionTime)
    return p->Speed * t + 0.5 * p->Acceleration * t * t;
  const double ramp = fmin(t - p->ReactionTime, p->RampTime);
  double distance =
      p->ReactionDistance + p->ReactionSpeed * ramp - p->Jerk * ramp * ramp * ramp / 6.0;
  const double brake = fmax(0.0, t - p->ReactionTime - p->RampTime);
  return distance + p->RampSpeed * brake - 0.5 * p->Deceleration * brake * brake;
}
static inline double apaMocoBrakingTimeAtDistance(const ApaMocoBrakingProfile_t *p,
                                                  double distance) {
  double lo = 0.0, hi = p->StopTime;
  for (int i = 0; i < 32; ++i) {
    const double mid = (lo + hi) / 2.0;
    if (apaMocoBrakingDistanceAtTime(p, mid) < distance)
      lo = mid;
    else
      hi = mid;
  }
  return (lo + hi) / 2.0;
}
#endif
