
#include "carla.h"
void CARLAACCLongitudinalCmd(double speed, int enable)
{
    extern int carla_adapter_publish_longitudinal_cmd(double targetSpeed, int enable);
    int rc = 0;

    rc = carla_adapter_publish_longitudinal_cmd(speed, enable);
    (void)rc;
}