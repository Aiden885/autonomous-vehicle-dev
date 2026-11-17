/**
 * @brief 读取CARLA自车相对车道中心的横向偏差
 * @en_name CARLALKSLateralOffset
 * @cn_name CARLA横向偏差
 * @type module
 * @retval double lateralOffset 横向偏差，正值表示车道中心位于自车右侧，单位m
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 LKS最小闭环
 * @version 1.0
 * @date 2026-05-26
 * @author liuruyu
 */
double CARLALKSLateralOffset(void)
{
    extern int carla_adapter_read_lane_tracking(
        double *lateralOffset,
        double *headingError,
        int *laneId,
        int *roadId,
        int *valid);
    double lateralOffset = 0.0;
    double headingError = 0.0;
    int laneId = 0;
    int roadId = 0;
    int valid = 0;
    int rc = 0;
    double result = 0.0;

    rc = carla_adapter_read_lane_tracking(&lateralOffset, &headingError, &laneId, &roadId, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = lateralOffset;
    }

    return result;
}

/**
 * @brief 读取CARLA自车相对车道切线方向的航向误差
 * @en_name CARLALKSHeadingError
 * @cn_name CARLA航向误差
 * @type module
 * @retval double headingError 航向误差，正值表示需向右修正，单位rad
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 LKS最小闭环
 * @version 1.0
 * @date 2026-05-26
 * @author liuruyu
 */
double CARLALKSHeadingError(void)
{
    extern int carla_adapter_read_lane_tracking(
        double *lateralOffset,
        double *headingError,
        int *laneId,
        int *roadId,
        int *valid);
    double lateralOffset = 0.0;
    double headingError = 0.0;
    int laneId = 0;
    int roadId = 0;
    int valid = 0;
    int rc = 0;
    double result = 0.0;

    rc = carla_adapter_read_lane_tracking(&lateralOffset, &headingError, &laneId, &roadId, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = headingError;
    }

    return result;
}

/**
 * @brief 读取CARLA车道跟踪输入数据是否有效
 * @en_name CARLALKSValid
 * @cn_name CARLA车道数据有效
 * @type module
 * @retval int valid 数据有效标志，1表示有效
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 LKS最小闭环
 * @version 1.0
 * @date 2026-05-26
 * @author liuruyu
 */
int CARLALKSValid(void)
{
    extern int carla_adapter_read_lane_tracking(
        double *lateralOffset,
        double *headingError,
        int *laneId,
        int *roadId,
        int *valid);
    double lateralOffset = 0.0;
    double headingError = 0.0;
    int laneId = 0;
    int roadId = 0;
    int valid = 0;
    int rc = 0;
    int result = 0;

    rc = carla_adapter_read_lane_tracking(&lateralOffset, &headingError, &laneId, &roadId, &valid);
    if (rc == 0) {
        result = valid;
    }

    return result;
}

/**
 * @brief 发布CARLA车道保持测试所需的横纵向联合控制命令
 * @en_name CARLALKSControlCmd
 * @cn_name CARLALKS控制命令
 * @type module
 * @param[IN] double targetSpeed 目标速度，单位m/s
 * @param[IN] double steerRad 目标转向角，单位rad
 * @param[IN] int enable 控制使能，1表示使能
 * @retval void 返回值为空
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 LKS最小闭环
 * @version 1.0
 * @date 2026-05-26
 * @author liuruyu
 */
void CARLALKSControlCmd(double targetSpeed, double steerRad, int enable)
{
    extern int carla_adapter_publish_control_cmd(
        double targetSpeed,
        double targetAccel,
        double steerRad,
        int enable);
    int rc = 0;

    rc = carla_adapter_publish_control_cmd(targetSpeed, 0.0, steerRad, enable);
    (void)rc;
}
