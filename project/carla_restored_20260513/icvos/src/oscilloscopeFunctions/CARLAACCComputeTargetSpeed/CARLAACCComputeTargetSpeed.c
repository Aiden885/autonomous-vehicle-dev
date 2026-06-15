
#include "carla.h"
double CARLAACCComputeTargetSpeed(double egoV, double leadV, double distance)
{
    double desiredDist = 15.0;
    double maxSpeed = 18.0 / 3.6;
    double kDist = 0.35;
    double kSpeed = 0.8;
    double distanceSafe = 0.0;
    double distDiff = 0.0;
    double speedDiff = 0.0;
    double targetSpeed = 0.0;

    if (distance < 0.0) {
        distanceSafe = 0.0;
    } else {
        distanceSafe = distance;
    }

    distDiff = distanceSafe - desiredDist;
    speedDiff = leadV - egoV;
    targetSpeed = (kDist * distDiff) + (kSpeed * speedDiff) + leadV;

    if (targetSpeed < 0.0) {
        targetSpeed = 0.0;
    }
    if (targetSpeed > maxSpeed) {
        targetSpeed = maxSpeed;
    }

    return targetSpeed;
}
