/**
 * @brief 读取CARLA障碍物列表
 * @en_name CARLAObjectList
 * @cn_name CARLA障碍物列表
 * @type module
 * @param[IN] int maxObjects 最大输出目标数量
 * @param[OUT] int* objectCount 实际输出目标数量
 * @param[OUT] int* objectId 目标ID数组
 * @param[OUT] int* objectType 目标类型数组，1表示车辆，2表示行人
 * @param[OUT] double* objectX 目标ENU坐标X数组，单位m
 * @param[OUT] double* objectY 目标ENU坐标Y数组，单位m
 * @param[OUT] double* objectYawRad 目标航向角数组，单位rad
 * @param[OUT] double* objectV 目标速度数组，单位m/s
 * @param[OUT] double* objectVx 目标X向速度数组，单位m/s
 * @param[OUT] double* objectVy 目标Y向速度数组，单位m/s
 * @param[OUT] double* objectLength 目标长度数组，单位m
 * @param[OUT] double* objectWidth 目标宽度数组，单位m
 * @param[OUT] double* objectHeight 目标高度数组，单位m
 * @param[OUT] int* valid 数据有效标志，1表示有效
 * @retval void 返回值为空
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 仿真输入
 * @version 1.0
 * @date 2026-05-19
 * @author liuruyu
 */
void CARLAObjectList(
    int maxObjects,
    int *objectCount,
    int *objectId,
    int *objectType,
    double *objectX,
    double *objectY,
    double *objectYawRad,
    double *objectV,
    double *objectVx,
    double *objectVy,
    double *objectLength,
    double *objectWidth,
    double *objectHeight,
    int *valid)
{
    extern int carla_adapter_read_object_list(
        int maxObjects,
        int *objectCount,
        int *objectId,
        int *objectType,
        double *objectX,
        double *objectY,
        double *objectYawRad,
        double *objectV,
        double *objectVx,
        double *objectVy,
        double *objectLength,
        double *objectWidth,
        double *objectHeight,
        int *valid);
    int rc = 0;

    rc = carla_adapter_read_object_list(
        maxObjects,
        objectCount,
        objectId,
        objectType,
        objectX,
        objectY,
        objectYawRad,
        objectV,
        objectVx,
        objectVy,
        objectLength,
        objectWidth,
        objectHeight,
        valid);
    if (rc != 0) {
        if (objectCount != NULL) {
            *objectCount = 0;
        }
        if (valid != NULL) {
            *valid = 0;
        }
    }
}

/**
 * @brief 发布CARLA横向转向控制命令
 * @en_name CARLALateralCmd
 * @cn_name CARLA横向控制命令
 * @type module
 * @param[IN] double steerRad 目标转向角，单位rad
 * @param[IN] int enable 控制使能，1表示使能
 * @param[OUT] int* status 发布状态，0表示成功
 * @retval void 返回值为空
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 仿真输出
 * @version 1.0
 * @date 2026-05-19
 * @author liuruyu
 */
void CARLALateralCmd(
    double steerRad,
    int enable,
    int *status)
{
    extern int carla_adapter_publish_lateral_cmd(double steerRad, int enable);
    int rc = 0;

    rc = carla_adapter_publish_lateral_cmd(steerRad, enable);
    if (status != NULL) {
        *status = rc;
    }
}

/**
 * @brief 发布CARLA横纵向联合控制命令
 * @en_name CARLAControlCmd
 * @cn_name CARLA联合控制命令
 * @type module
 * @param[IN] double targetSpeed 目标速度，单位m/s
 * @param[IN] double targetAccel 目标加速度，单位m/s^2
 * @param[IN] double steerRad 目标转向角，单位rad
 * @param[IN] int enable 控制使能，1表示使能
 * @param[OUT] int* status 发布状态，0表示成功
 * @retval void 返回值为空
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 仿真输出
 * @version 1.0
 * @date 2026-05-19
 * @author liuruyu
 */
void CARLAControlCmd(
    double targetSpeed,
    double targetAccel,
    double steerRad,
    int enable,
    int *status)
{
    extern int carla_adapter_publish_control_cmd(
        double targetSpeed,
        double targetAccel,
        double steerRad,
        int enable);
    int rc = 0;

    rc = carla_adapter_publish_control_cmd(targetSpeed, targetAccel, steerRad, enable);
    if (status != NULL) {
        *status = rc;
    }
}
