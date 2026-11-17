#include "carla_gaasd_adapter.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static double monotonic_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

int main(int argc, char **argv)
{
    const double durationSec = argc > 1 ? atof(argv[1]) : 5.0;
    const double targetSpeed = argc > 2 ? atof(argv[2]) : 1.5;
    const double deadline = monotonic_sec() + durationSec;
    double nextCommand = monotonic_sec();
    int egoValidCount = 0;
    int leadValidCount = 0;
    int chassisValidCount = 0;
    int commandCount = 0;
    int rc = 0;

    rc = carla_adapter_init();
    if (rc != 0) {
        fprintf(stderr, "adapter init failed: %d\n", rc);
        return 1;
    }

    while (monotonic_sec() < deadline) {
        double egoV = 0.0;
        double egoX = 0.0;
        double egoY = 0.0;
        double egoYaw = 0.0;
        double egoAcc = 0.0;
        double leadV = 0.0;
        double distance = 0.0;
        double relativeSpeed = 0.0;
        double ttc = 0.0;
        double chassisSpeed = 0.0;
        double steer = 0.0;
        int egoValid = 0;
        int leadValid = 0;
        int chassisValid = 0;
        int mode = 0;

        rc = carla_adapter_poll(50);
        if (rc < 0) {
            fprintf(stderr, "adapter poll failed: %d\n", rc);
            return 2;
        }

        rc = carla_adapter_read_ego_state(&egoV, &egoX, &egoY, &egoYaw, &egoAcc, &egoValid);
        if (rc == 0 && egoValid != 0) {
            egoValidCount = egoValidCount + 1;
        }

        rc = carla_adapter_read_lead_vehicle(&leadV, &distance, &relativeSpeed, &ttc, &leadValid);
        if (rc == 0 && leadValid != 0) {
            leadValidCount = leadValidCount + 1;
        }

        rc = carla_adapter_read_chassis_feedback(&chassisSpeed, &steer, &mode, &chassisValid);
        if (rc == 0 && chassisValid != 0) {
            chassisValidCount = chassisValidCount + 1;
        }

        if (monotonic_sec() >= nextCommand) {
            rc = carla_adapter_publish_longitudinal_cmd(targetSpeed, egoValid);
            if (rc != 0) {
                fprintf(stderr, "publish failed: %d\n", rc);
                return 3;
            }
            commandCount = commandCount + 1;
            nextCommand = monotonic_sec() + 0.05;
        }
    }

    printf("live_bridge_result ego_valid=%d lead_valid=%d chassis_valid=%d commands=%d\n",
           egoValidCount, leadValidCount, chassisValidCount, commandCount);
    carla_adapter_shutdown();

    if (egoValidCount <= 0 || leadValidCount <= 0 || chassisValidCount <= 0 || commandCount <= 0) {
        return 4;
    }
    return 0;
}
