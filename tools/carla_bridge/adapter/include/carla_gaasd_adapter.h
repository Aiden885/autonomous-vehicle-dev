#ifndef CARLA_GAASD_ADAPTER_H
#define CARLA_GAASD_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

int carla_adapter_init(void);
void carla_adapter_shutdown(void);
int carla_adapter_poll(int timeout_ms);

int carla_adapter_read_ego_state(
    double *egoV,
    double *egoX,
    double *egoY,
    double *egoYawRad,
    double *egoAcc,
    int *valid);

int carla_adapter_read_lead_vehicle(
    double *leadV,
    double *distance,
    double *relativeSpeed,
    double *ttc,
    int *valid);

int carla_adapter_read_chassis_feedback(
    double *speed,
    double *steerRad,
    int *mode,
    int *valid);

int carla_adapter_publish_longitudinal_cmd(
    double targetSpeed,
    int enable);

int carla_adapter_get_status(
    int *initialized,
    int *egoFresh,
    int *leadFresh,
    int *chassisFresh,
    unsigned long long *commandCount);

#ifdef __cplusplus
}
#endif

#endif
