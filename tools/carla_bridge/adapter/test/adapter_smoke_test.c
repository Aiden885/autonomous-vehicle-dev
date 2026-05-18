#include "carla_gaasd_adapter.h"

#include <stdio.h>

int main(void)
{
    double egoV = 0.0;
    double egoX = 0.0;
    double egoY = 0.0;
    double egoYaw = 0.0;
    double egoAcc = 0.0;
    int valid = 0;
    int rc = 0;

    rc = carla_adapter_init();
    if (rc != 0) {
        printf("init_rc=%d\n", rc);
        return 1;
    }

    rc = carla_adapter_read_ego_state(&egoV, &egoX, &egoY, &egoYaw, &egoAcc, &valid);
    printf("read_ego_rc=%d valid=%d egoV=%.3f egoX=%.3f egoY=%.3f yaw=%.3f acc=%.3f\n",
           rc, valid, egoV, egoX, egoY, egoYaw, egoAcc);

    rc = carla_adapter_publish_longitudinal_cmd(0.0, 0);
    printf("publish_rc=%d\n", rc);

    carla_adapter_shutdown();
    return rc == 0 ? 0 : 2;
}
