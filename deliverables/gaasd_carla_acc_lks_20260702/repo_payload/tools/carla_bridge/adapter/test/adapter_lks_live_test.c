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
    const double durationSec = argc > 1 ? atof(argv[1]) : 2.0;
    const double deadline = monotonic_sec() + durationSec;
    double nextCommand = monotonic_sec();
    double lateralOffset = 0.0;
    double headingError = 0.0;
    int egoValidCount = 0;
    int laneValidCount = 0;
    int chassisValidCount = 0;
    int commandCount = 0;
    int rc = carla_adapter_init();

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
        double chassisSpeed = 0.0;
        double steer = 0.0;
        int laneId = 0;
        int roadId = 0;
        int egoValid = 0;
        int laneValid = 0;
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

        rc = carla_adapter_read_lane_tracking(
            &lateralOffset,
            &headingError,
            &laneId,
            &roadId,
            &laneValid);
        if (rc == 0 && laneValid != 0) {
            laneValidCount = laneValidCount + 1;
        }

        rc = carla_adapter_read_chassis_feedback(&chassisSpeed, &steer, &mode, &chassisValid);
        if (rc == 0 && chassisValid != 0) {
            chassisValidCount = chassisValidCount + 1;
        }

        if (monotonic_sec() >= nextCommand) {
            rc = carla_adapter_publish_control_cmd(0.0, 0.0, 0.0, egoValid && laneValid);
            if (rc != 0) {
                fprintf(stderr, "publish failed: %d\n", rc);
                return 3;
            }
            commandCount = commandCount + 1;
            nextCommand = monotonic_sec() + 0.05;
        }
    }

    printf(
        "lks_live_result ego_valid=%d lane_valid=%d chassis_valid=%d commands=%d "
        "lateral_offset=%.6f heading_error=%.6f\n",
        egoValidCount,
        laneValidCount,
        chassisValidCount,
        commandCount,
        lateralOffset,
        headingError);
    carla_adapter_shutdown();

    if (egoValidCount <= 0 || laneValidCount <= 0 || chassisValidCount <= 0 || commandCount <= 0) {
        return 4;
    }
    return 0;
}
