/**
 * @brief 读取CARLA自车状态
 * @en_name CARLAEgoState
 * @cn_name CARLA自车状态
 * @type module
 * @param[OUT] double* egoV 自车纵向速度，单位m/s
 * @param[OUT] double* egoX 自车位置X，单位m
 * @param[OUT] double* egoY 自车位置Y，单位m
 * @param[OUT] double* egoYawRad 自车航向角，单位rad
 * @param[OUT] double* egoAcc 自车纵向加速度，单位m/s^2
 * @param[OUT] int* valid 数据有效标志，1表示有效
 * @retval None 无返回值，输出通过参数返回
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 仿真输入
 * @version 1.0
 * @date 2026-04-29
 * @author liuruyu
 */
void CARLAEgoState(
    double *egoV,
    double *egoX,
    double *egoY,
    double *egoYawRad,
    double *egoAcc,
    int *valid)
{
    extern int carla_adapter_read_ego_state(
        double *egoV,
        double *egoX,
        double *egoY,
        double *egoYawRad,
        double *egoAcc,
        int *valid);
    int rc = 0;

    rc = carla_adapter_read_ego_state(egoV, egoX, egoY, egoYawRad, egoAcc, valid);
    if (rc != 0) {
        if (egoV != 0) {
            *egoV = 0.0;
        }
        if (egoX != 0) {
            *egoX = 0.0;
        }
        if (egoY != 0) {
            *egoY = 0.0;
        }
        if (egoYawRad != 0) {
            *egoYawRad = 0.0;
        }
        if (egoAcc != 0) {
            *egoAcc = 0.0;
        }
        if (valid != 0) {
            *valid = 0;
        }
    }
}

/**
 * @brief 读取CARLA前车状态
 * @en_name CARLALeadVehicle
 * @cn_name CARLA前车状态
 * @type module
 * @param[OUT] double* leadV 前车纵向速度，单位m/s
 * @param[OUT] double* distance 自车到前车净距离，单位m
 * @param[OUT] double* relativeSpeed 前车相对自车纵向速度，单位m/s
 * @param[OUT] double* ttc 碰撞时间估计，单位s
 * @param[OUT] int* valid 数据有效标志，1表示有效
 * @retval None 无返回值，输出通过参数返回
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 仿真输入
 * @version 1.0
 * @date 2026-04-29
 * @author liuruyu
 */
void CARLALeadVehicle(
    double *leadV,
    double *distance,
    double *relativeSpeed,
    double *ttc,
    int *valid)
{
    extern int carla_adapter_read_lead_vehicle(
        double *leadV,
        double *distance,
        double *relativeSpeed,
        double *ttc,
        int *valid);
    int rc = 0;

    rc = carla_adapter_read_lead_vehicle(leadV, distance, relativeSpeed, ttc, valid);
    if (rc != 0) {
        if (leadV != 0) {
            *leadV = 0.0;
        }
        if (distance != 0) {
            *distance = 1000000.0;
        }
        if (relativeSpeed != 0) {
            *relativeSpeed = 0.0;
        }
        if (ttc != 0) {
            *ttc = 1000000.0;
        }
        if (valid != 0) {
            *valid = 0;
        }
    }
}

/**
 * @brief 发布CARLA纵向速度控制命令
 * @en_name CARLALongitudinalCmd
 * @cn_name CARLA纵向控制命令
 * @type module
 * @param[IN] double speed 目标速度，单位m/s
 * @param[IN] int enable 控制使能，1表示使能
 * @param[OUT] int* status 发布状态，0表示成功
 * @retval None 无返回值，输出通过参数返回
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 仿真输出
 * @version 1.0
 * @date 2026-04-29
 * @author liuruyu
 */
void CARLALongitudinalCmd(
    double speed,
    int enable,
    int *status)
{
    extern int carla_adapter_publish_longitudinal_cmd(double targetSpeed, int enable);
    int rc = 0;

    rc = carla_adapter_publish_longitudinal_cmd(speed, enable);
    if (status != 0) {
        *status = rc;
    }
}

/**
 * @brief 读取CARLA底盘控制反馈
 * @en_name CARLAChassisFeedback
 * @cn_name CARLA底盘反馈
 * @type module
 * @param[OUT] double* speed 当前车辆速度，单位m/s
 * @param[OUT] double* steerRad 当前转向角，单位rad
 * @param[OUT] int* mode 控制模式
 * @param[OUT] int* valid 数据有效标志，1表示有效
 * @retval None 无返回值，输出通过参数返回
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 仿真输入
 * @version 1.0
 * @date 2026-04-29
 * @author liuruyu
 */
void CARLAChassisFeedback(
    double *speed,
    double *steerRad,
    int *mode,
    int *valid)
{
    extern int carla_adapter_read_chassis_feedback(
        double *speed,
        double *steerRad,
        int *mode,
        int *valid);
    int rc = 0;

    rc = carla_adapter_read_chassis_feedback(speed, steerRad, mode, valid);
    if (rc != 0) {
        if (speed != 0) {
            *speed = 0.0;
        }
        if (steerRad != 0) {
            *steerRad = 0.0;
        }
        if (mode != 0) {
            *mode = 0;
        }
        if (valid != 0) {
            *valid = 0;
        }
    }
}
