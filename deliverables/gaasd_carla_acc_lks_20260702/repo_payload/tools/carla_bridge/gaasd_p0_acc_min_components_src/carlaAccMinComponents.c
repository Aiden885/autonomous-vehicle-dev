/**
 * @brief 读取CARLA自车纵向速度
 * @en_name CARLAACCEgoSpeed
 * @cn_name CARLA自车速度
 * @type module
 * @retval double egoV 自车纵向速度，单位m/s
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 1.0
 * @date 2026-05-07
 * @author liuruyu
 */
double CARLAACCEgoSpeed(void)
{
    extern int carla_adapter_read_ego_state(
        double *egoV,
        double *egoX,
        double *egoY,
        double *egoYawRad,
        double *egoAcc,
        int *valid);
    double egoV = 0.0;
    double egoX = 0.0;
    double egoY = 0.0;
    double egoYawRad = 0.0;
    double egoAcc = 0.0;
    int valid = 0;
    int rc = 0;
    double result = 0.0;

    rc = carla_adapter_read_ego_state(&egoV, &egoX, &egoY, &egoYawRad, &egoAcc, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = egoV;
    }

    return result;
}

/**
 * @brief 读取CARLA前车纵向速度
 * @en_name CARLAACCLeadSpeed
 * @cn_name CARLA前车速度
 * @type module
 * @retval double leadV 前车纵向速度，单位m/s
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 1.0
 * @date 2026-05-07
 * @author liuruyu
 */
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

/**
 * @brief 读取CARLA前车净距离
 * @en_name CARLAACCLeadDistance
 * @cn_name CARLA前车距离
 * @type module
 * @retval double distance 自车到前车净距离，单位m
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 1.0
 * @date 2026-05-07
 * @author liuruyu
 */
double CARLAACCLeadDistance(void)
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
    double result = 1000000.0;

    rc = carla_adapter_read_lead_vehicle(&leadV, &distance, &relativeSpeed, &ttc, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = distance;
    }

    return result;
}

/**
 * @brief 读取ACC驾驶指令类型
 * @en_name CARLAACCDriverCommand
 * @cn_name CARLA驾驶指令
 * @type module
 * @retval int commandType 驾驶指令类型，0表示无指令，1到7表示ACC按键或踏板事件
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 1.0
 * @date 2026-06-08
 * @author ZYK
 */
int CARLAACCDriverCommand(void)
{
    extern int carla_adapter_read_driver_command(int *commandType, int *valid);
    int commandType = 0;
    int valid = 0;
    int rc = 0;
    int result = 0;

    rc = carla_adapter_read_driver_command(&commandType, &valid);
    if ((rc == 0) && (valid != 0)) {
        result = commandType;
    }

    return result;
}

/**
 * @brief 计算ACC目标跟车速度
 * @en_name CARLAACCComputeTargetSpeed
 * @cn_name CARLA计算目标速度
 * @type module
 * @param[IN] double egoV 自车当前速度，单位m/s
 * @param[IN] double leadV 前车当前速度，单位m/s
 * @param[IN] double distance 当前车间距，单位m
 * @retval double targetSpeed 限幅后的ACC目标速度，单位m/s
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 1.0
 * @date 2026-05-07
 * @author liuruyu
 */
double CARLAACCComputeTargetSpeed(double egoV, double leadV, double distance)
{
    double desiredDist = 15.0;
    double maxSpeed = 18.0 / 3.6;
    double kDist = 0.35;
    double kSpeed = 0.8;
    double distanceSafe = 0.0;
    double distDiff = 0.0;
    double speedDiff = 0.0;
    double targetSpeed = 0.0;

    if (distance < 0.0) {
        distanceSafe = 0.0;
    } else {
        distanceSafe = distance;
    }

    distDiff = distanceSafe - desiredDist;
    speedDiff = leadV - egoV;
    targetSpeed = (kDist * distDiff) + (kSpeed * speedDiff) + leadV;

    if (targetSpeed < 0.0) {
        targetSpeed = 0.0;
    }
    if (targetSpeed > maxSpeed) {
        targetSpeed = maxSpeed;
    }

    return targetSpeed;
}

/**
 * @brief 发布CARLA纵向速度控制命令
 * @en_name CARLAACCLongitudinalCmd
 * @cn_name CARLA纵向控制命令
 * @type module
 * @param[IN] double speed 目标速度，单位m/s
 * @param[IN] int enable 控制使能，1表示使能
 * @retval void 返回值为空
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 1.0
 * @date 2026-05-07
 * @author liuruyu
 */
void CARLAACCLongitudinalCmd(double speed, int enable)
{
    extern int carla_adapter_publish_longitudinal_cmd(double targetSpeed, int enable);
    int rc = 0;

    rc = carla_adapter_publish_longitudinal_cmd(speed, enable);
    (void)rc;
}
