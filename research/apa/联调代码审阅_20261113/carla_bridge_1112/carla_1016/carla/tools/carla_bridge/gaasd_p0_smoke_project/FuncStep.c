#include <stdio.h>

extern int carla_adapter_init(void);
extern int carla_adapter_poll(int timeout_ms);
extern int carla_adapter_read_ego_state(
    double *egoV,
    double *egoX,
    double *egoY,
    double *egoYawRad,
    double *egoAcc,
    int *valid);
extern int carla_adapter_read_lead_vehicle(
    double *leadV,
    double *distance,
    double *relativeSpeed,
    double *ttc,
    int *valid);
extern int carla_adapter_publish_longitudinal_cmd(
    double targetSpeed,
    int enable);

void FuncStep(void)
{
    static int initialized = 0;
    static int step = 0;
    double egoV = 0.0;
    double egoX = 0.0;
    double egoY = 0.0;
    double egoYaw = 0.0;
    double egoAcc = 0.0;
    double leadV = 0.0;
    double distance = 0.0;
    double relativeSpeed = 0.0;
    double ttc = 0.0;
    int egoValid = 0;
    int leadValid = 0;
    int rc = 0;
    int enable = 0;
    double targetSpeed = 0.0;

    if (initialized == 0) {
        rc = carla_adapter_init();
        printf("[gaasd_p0_smoke] init rc=%d\n", rc);
        initialized = 1;
    }

    (void)carla_adapter_poll(0);

    rc = carla_adapter_read_ego_state(
        &egoV,
        &egoX,
        &egoY,
        &egoYaw,
        &egoAcc,
        &egoValid);
    if (rc != 0) {
        printf("[gaasd_p0_smoke] read ego rc=%d\n", rc);
    }

    rc = carla_adapter_read_lead_vehicle(
        &leadV,
        &distance,
        &relativeSpeed,
        &ttc,
        &leadValid);
    if (rc != 0) {
        printf("[gaasd_p0_smoke] read lead rc=%d\n", rc);
    }

    if (egoValid != 0) {
        enable = 1;
        targetSpeed = 1.0;
    }

    rc = carla_adapter_publish_longitudinal_cmd(targetSpeed, enable);
    if (rc != 0) {
        printf("[gaasd_p0_smoke] publish rc=%d\n", rc);
    }

    printf("[gaasd_p0_smoke] step=%d egoValid=%d egoV=%.3f leadValid=%d leadV=%.3f distance=%.3f target=%.3f enable=%d\n",
           step,
           egoValid,
           egoV,
           leadValid,
           leadV,
           distance,
           targetSpeed,
           enable);
    step = step + 1;
}
