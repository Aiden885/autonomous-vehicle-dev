
#include "newaccpro2.h"
double CARLAACCLeadSpeed(void)
{
    extern int carla_adapter_read_lead_vehicle(
        double *leadV,
        double *distance,
        double *relativeSpeed,
        double *ttc,
        int *valid);
    double leadV = 0.0;
    double distance = 1000000.0;
    double relativeSpeed = 0.0;
    double ttc = 1000000.0;
    int valid = 0;
    int rc = 0;
    double result = 0.0;

    rc = carla_adapter_read_lead_vehicle(&leadV, &distance, &relativeSpeed, &ttc, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = leadV;
    }

    return result;
}