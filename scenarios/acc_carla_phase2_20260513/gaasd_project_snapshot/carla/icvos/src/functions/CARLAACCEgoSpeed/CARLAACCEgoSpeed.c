
#include "carla.h"
double CARLAACCEgoSpeed(void)
{
    extern int carla_adapter_read_ego_state(
        double *egoV,
        double *egoX,
        double *egoY,
        double *egoYawRad,
        double *egoAcc,
        int *valid);
    double egoV = 0.0;
    double egoX = 0.0;
    double egoY = 0.0;
    double egoYawRad = 0.0;
    double egoAcc = 0.0;
    int valid = 0;
    int rc = 0;
    double result = 0.0;

    rc = carla_adapter_read_ego_state(&egoV, &egoX, &egoY, &egoYawRad, &egoAcc, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = egoV;
    }

    return result;
}