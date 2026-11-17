#ifndef APA_MOCO_PARKING_CONTROL_H
#define APA_MOCO_PARKING_CONTROL_H
#include "apaMocoParkingControlTypes.h"
#include "apaMocoBraking.h"

static inline ApaMocoBrakingProfile_t
apaMocoParkingBrakingProfile(Real velocity,
                             Real acceleration,
                             Real previousAcceleration,
                             const ApaMocoParkingControlParam_t *param) {
  const ApaMocoBrakingParam_t limits = {
      .AccelerationUpperLimit = param->Longi.AccelerationUpperLimit,
      .AccelerationLowerLimit = param->Longi.AccelerationLowerLimit,
      .StopDeceleration = param->Longi.StopDeceleration,
      .ReactionTime = param->ReactionTime,
      .StopBuffer = param->StopBuffer};
  return apaMocoBuildBrakingProfile(velocity, acceleration, previousAcceleration, &limits);
}
static inline Real apaMocoParkingSpeedResponseTime(const ApaMocoParkingControlParam_t *param) {
  return param->Longi.KpSpeed > 0.0f
             ? (1.0f + param->Longi.AccelerationFeedbackGain) / param->Longi.KpSpeed
             : INFINITY;
}
void apaMocoParkingControlDefaultParam(ApaMocoParkingControlParam_t *param);
Real apaMocoParkingSpeedForDistance(const ApaMocoParkingControlInput_t *input,
                                    const ApaMocoParkingControlParam_t *param,
                                    Real lastAcceleration,
                                    Real distance,
                                    Integer direction);
void apaMocoParkingControl(const ApaMocoParkingControlInput_t *input,
                           const ApaMocoParkingControlParam_t *param,
                           ApaMocoParkingControlState_t *state,
                           ApaMocoParkingControlOutput_t *output);
#endif
