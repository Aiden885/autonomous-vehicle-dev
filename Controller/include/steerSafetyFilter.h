#ifndef STEER_SAFETY_FILTER_H
#define STEER_SAFETY_FILTER_H


void steerSafetyFilter(
    double& steer,
    double& last_safe_rad,
    double  maxSteerRad

);

#endif  // STEER_SAFETY_FILTER_H
