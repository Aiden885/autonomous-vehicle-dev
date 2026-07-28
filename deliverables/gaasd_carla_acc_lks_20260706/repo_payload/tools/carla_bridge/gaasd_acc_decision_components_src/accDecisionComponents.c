/**
 * @brief 计算自车与前车的当前时距
 * @en_name ACCTimeHeadway
 * @cn_name ACC当前时距
 * @type module
 * @param[IN] double egoV 自车纵向速度，单位m/s
 * @param[IN] double distance 自车与前车间距，单位m
 * @retval double tau 当前时距，单位s；egoV过低时返回99.0防止除零
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策
 * @version 1.0
 * @date 2026-05-27
 * @author liuruyu
 */
double ACCTimeHeadway(double egoV, double distance)
{
    double tau = 99.0;
    double dist = distance;

    if (dist < 0.0) {
        dist = 0.0;
    }

    if (egoV >= 0.5) {
        tau = dist / egoV;
    }

    return tau;
}

/**
 * @brief ACC时距控制决策：根据当前时距与目标时距计算期望速度
 * @en_name ACCDecision
 * @cn_name ACC时距决策
 * @type module
 * @param[IN] double egoV 自车纵向速度，单位m/s
 * @param[IN] double leadV 前车纵向速度，单位m/s
 * @param[IN] double distance 自车与前车间距，单位m
 * @param[IN] double tau 当前时距，单位s（由ACCTimeHeadway提供）
 * @param[IN] double tauSet 目标时距设定值，单位s
 * @param[IN] double vMax 最大允许速度上限，单位m/s
 * @retval double targetSpeed 期望速度，单位m/s
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策
 * @version 1.0
 * @date 2026-05-27
 * @author liuruyu
 */
double ACCDecision(double egoV, double leadV, double distance,
                   double tau, double tauSet, double vMax)
{
    /* 距离误差增益，与 P0 一致，可通过仿真整定 */
    static const double K_DIST = 0.5;

    double dSet = 0.0;
    double distError = 0.0;
    double targetSpeed = 0.0;
    double dist = distance;

    if (dist < 0.0) {
        dist = 0.0;
    }

    /* 动态期望距离：τ_set × v_ego（时距恒定控制核心，随车速自适应调整） */
    dSet = tauSet * egoV;

    /* 目标速度 = 前车速度 + 距离误差反馈 */
    distError = dist - dSet;
    targetSpeed = leadV + K_DIST * distError;

    /* 限幅 [0, vMax] */
    if (targetSpeed < 0.0) {
        targetSpeed = 0.0;
    }
    if (targetSpeed > vMax) {
        targetSpeed = vMax;
    }

    return targetSpeed;
}
