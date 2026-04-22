
#include "accpro1.h"
double computeSpeedError(double targetSpeed, double egoV)
{
    double error = 0.0;
    error = targetSpeed - egoV;
    return error;
}