/**
 * @brief 判断ACC车速子态
 * @en_name ACCSpeedSubState
 * @cn_name ACC车速子态
 * @type module
 * @param[IN] double egoV 自车纵向速度，单位m/s
 * @param[IN] double vMinKmh 最低适控车速，单位km/h
 * @retval int speedSubState 车速子态，0=适速Sv0，1=低速Sv1
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
int ACCSpeedSubState(double egoV, double vMinKmh)
{
    double vMin = 0.0;
    int speedSubState = 0;

    vMin = vMinKmh / 3.6;
    if (egoV < vMin) {
        speedSubState = 1;
    } else {
        speedSubState = 0;
    }

    return speedSubState;
}

/**
 * @brief 根据控制使能和历史标志计算ACC系统子态
 * @en_name ACCSystemSubState
 * @cn_name ACC系统子态
 * @type module
 * @param[IN] int controlEnabled 控制使能，0=待命，1=在控
 * @param[IN] int hasHistory 是否存在历史控制目标，0=无史，1=有史
 * @retval int systemSubState 系统子态，0=无史待命Ss0，1=有史待命Ss1，2=无史在控Ss2，3=有史在控Ss3
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
int ACCSystemSubState(int controlEnabled, int hasHistory)
{
    int systemSubState = 0;

    if (controlEnabled != 0) {
        if (hasHistory != 0) {
            systemSubState = 3;
        } else {
            systemSubState = 2;
        }
    } else {
        if (hasHistory != 0) {
            systemSubState = 1;
        } else {
            systemSubState = 0;
        }
    }

    return systemSubState;
}

/**
 * @brief 根据车速子态和系统子态判定ACC系统状态
 * @en_name ACCSystemState
 * @cn_name ACC系统状态
 * @type module
 * @param[IN] int speedSubState 车速子态，0=适速Sv0，1=低速Sv1
 * @param[IN] int systemSubState 系统子态，0=Ss0，1=Ss1，2=Ss2，3=Ss3
 * @retval int systemState 系统状态，0=在控S0，1=适速有史待命S1，2=适速无史待命S2，3=低速S3
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
int ACCSystemState(int speedSubState, int systemSubState)
{
    int systemState = 2;

    if (speedSubState != 0) {
        systemState = 3;
    } else {
        if (systemSubState == 0) {
            systemState = 2;
        } else if (systemSubState == 1) {
            systemState = 1;
        } else {
            systemState = 0;
        }
    }

    return systemState;
}

/**
 * @brief ACC状态机下一状态查表
 * @en_name ACCNextState
 * @cn_name ACC下一状态
 * @type module
 * @param[IN] int systemState 当前系统状态，0=S0，1=S1，2=S2，3=S3
 * @param[IN] int commandType 驾驶指令，0=无，1=降速/当速启控，2=增速/继承启控，3=降距，4=增距，5=油门，6=刹车，7=取消
 * @retval int nextState 下一系统状态，0=S0，1=S1，2=S2，3=S3
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
int ACCNextState(int systemState, int commandType)
{
    int nextState = 3;

    if (systemState == 0) {
        if ((commandType == 6) || (commandType == 7)) {
            nextState = 1;
        } else {
            nextState = 0;
        }
    } else if (systemState == 1) {
        if ((commandType == 1) || (commandType == 2)) {
            nextState = 0;
        } else {
            nextState = 1;
        }
    } else if (systemState == 2) {
        if (commandType == 1) {
            nextState = 0;
        } else {
            nextState = 2;
        }
    } else {
        nextState = 3;
    }

    return nextState;
}

/**
 * @brief ACC决策查表
 * @en_name ACCDecisionOutput
 * @cn_name ACC决策输出
 * @type module
 * @param[IN] int systemState 当前系统状态，0=S0，1=S1，2=S2，3=S3
 * @param[IN] int commandType 驾驶指令，0=无，1=降速/当速启控，2=增速/继承启控，3=降距，4=增距，5=油门，6=刹车，7=取消
 * @param[IN] int lastActiveDecision 上一次有效控制决策，范围1~6
 * @retval int decisionY 决策输出，1=R1降速，2=R2增速，3=R3降距，4=R4增距，5=R5无继控制，6=R6继承控制，7=R7扭矩仲裁，8=R8待命
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
int ACCDecisionOutput(int systemState, int commandType, int lastActiveDecision)
{
    int decisionY = 8;

    if (systemState == 0) {
        if (commandType == 0) {
            if ((lastActiveDecision >= 1) && (lastActiveDecision <= 6)) {
                decisionY = lastActiveDecision;
            } else {
                decisionY = 8;
            }
        } else if (commandType == 1) {
            decisionY = 1;
        } else if (commandType == 2) {
            decisionY = 2;
        } else if (commandType == 3) {
            decisionY = 3;
        } else if (commandType == 4) {
            decisionY = 4;
        } else if (commandType == 5) {
            decisionY = 7;
        } else {
            decisionY = 8;
        }
    } else if (systemState == 1) {
        if (commandType == 1) {
            decisionY = 5;
        } else if (commandType == 2) {
            decisionY = 6;
        } else {
            decisionY = 8;
        }
    } else if (systemState == 2) {
        if (commandType == 1) {
            decisionY = 5;
        } else {
            decisionY = 8;
        }
    } else {
        decisionY = 8;
    }

    return decisionY;
}

/**
 * @brief ACC控制使能查表
 * @en_name ACCControlEnable
 * @cn_name ACC控制使能
 * @type module
 * @param[IN] int systemState 当前系统状态，0=S0，1=S1，2=S2，3=S3
 * @param[IN] int commandType 驾驶指令，0=无，1=降速/当速启控，2=增速/继承启控，3=降距，4=增距，5=油门，6=刹车，7=取消
 * @retval int controlEnable 控制使能，0=不控制，1=ACC控制
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
int ACCControlEnable(int systemState, int commandType)
{
    int controlEnable = 0;

    if (systemState == 0) {
        if ((commandType == 6) || (commandType == 7)) {
            controlEnable = 0;
        } else {
            controlEnable = 1;
        }
    } else if (systemState == 1) {
        if ((commandType == 1) || (commandType == 2)) {
            controlEnable = 1;
        } else {
            controlEnable = 0;
        }
    } else if (systemState == 2) {
        if (commandType == 1) {
            controlEnable = 1;
        } else {
            controlEnable = 0;
        }
    } else {
        controlEnable = 0;
    }

    return controlEnable;
}

/**
 * @brief 更新ACC历史标志
 * @en_name ACCHasHistoryNext
 * @cn_name ACC历史更新
 * @type module
 * @param[IN] int systemState 当前系统状态，0=S0，1=S1，2=S2，3=S3
 * @param[IN] int commandType 驾驶指令，0=无，1=降速/当速启控，2=增速/继承启控，3=降距，4=增距，5=油门，6=刹车，7=取消
 * @param[IN] int hasHistory 当前历史标志，0=无历史，1=有历史
 * @retval int nextHasHistory 下一历史标志，0=无历史，1=有历史
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
int ACCHasHistoryNext(int systemState, int commandType, int hasHistory)
{
    int nextHasHistory = 0;

    if (hasHistory != 0) {
        nextHasHistory = 1;
    } else {
        nextHasHistory = 0;
    }

    if (systemState == 0) {
        nextHasHistory = 1;
    } else if ((systemState == 1) && ((commandType == 1) || (commandType == 2))) {
        nextHasHistory = 1;
    } else if ((systemState == 2) && (commandType == 1)) {
        nextHasHistory = 1;
    }

    return nextHasHistory;
}

/**
 * @brief 更新ACC上一次有效决策
 * @en_name ACCLastDecisionNext
 * @cn_name ACC历史决策
 * @type module
 * @param[IN] int decisionY 当前决策输出，范围1~8
 * @param[IN] int lastActiveDecision 当前上一次有效决策，范围1~6
 * @retval int nextLastActiveDecision 下一上一次有效决策，范围1~6
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
int ACCLastDecisionNext(int decisionY, int lastActiveDecision)
{
    int nextLastActiveDecision = 5;

    if ((lastActiveDecision >= 1) && (lastActiveDecision <= 6)) {
        nextLastActiveDecision = lastActiveDecision;
    }

    if ((decisionY >= 1) && (decisionY <= 6)) {
        nextLastActiveDecision = decisionY;
    }

    return nextLastActiveDecision;
}

/**
 * @brief 根据决策更新目标时距
 * @en_name ACCTimeGapNext
 * @cn_name ACC目标时距
 * @type module
 * @param[IN] int decisionY 当前决策输出，1~8
 * @param[IN] double currentTimeGap 当前目标时距，单位s
 * @param[IN] double timeGapStep 时距调整步长，单位s
 * @param[IN] double minTimeGap 最小目标时距，单位s
 * @param[IN] double maxTimeGap 最大目标时距，单位s
 * @retval double nextTimeGap 下一目标时距，单位s
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
double ACCTimeGapNext(int decisionY, double currentTimeGap, double timeGapStep,
                      double minTimeGap, double maxTimeGap)
{
    double nextTimeGap = currentTimeGap;

    if (decisionY == 3) {
        nextTimeGap = currentTimeGap - timeGapStep;
    } else if (decisionY == 4) {
        nextTimeGap = currentTimeGap + timeGapStep;
    }

    if (nextTimeGap < minTimeGap) {
        nextTimeGap = minTimeGap;
    }
    if (nextTimeGap > maxTimeGap) {
        nextTimeGap = maxTimeGap;
    }

    return nextTimeGap;
}

/**
 * @brief 根据决策更新最高期望速度
 * @en_name ACCMaxSpeedNext
 * @cn_name ACC目标速度上限
 * @type module
 * @param[IN] int decisionY 当前决策输出，1~8
 * @param[IN] double currentMaxSpeed 当前最高期望速度，单位m/s
 * @param[IN] double egoV 自车纵向速度，单位m/s
 * @param[IN] double speedStep 速度调整步长，单位m/s
 * @param[IN] double minSpeed 最小速度上限，单位m/s
 * @param[IN] double maxSpeedCap 最大速度上限，单位m/s
 * @retval double nextMaxSpeed 下一最高期望速度，单位m/s
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
double ACCMaxSpeedNext(int decisionY, double currentMaxSpeed, double egoV,
                       double speedStep, double minSpeed, double maxSpeedCap)
{
    double nextMaxSpeed = currentMaxSpeed;

    if (decisionY == 1) {
        nextMaxSpeed = currentMaxSpeed - speedStep;
    } else if (decisionY == 2) {
        nextMaxSpeed = currentMaxSpeed + speedStep;
    } else if (decisionY == 5) {
        nextMaxSpeed = egoV;
    }

    if (nextMaxSpeed < minSpeed) {
        nextMaxSpeed = minSpeed;
    }
    if (nextMaxSpeed > maxSpeedCap) {
        nextMaxSpeed = maxSpeedCap;
    }

    return nextMaxSpeed;
}

/**
 * @brief 判断ACC扭矩仲裁标志
 * @en_name ACCTorqueArbitrationFlag
 * @cn_name ACC扭矩仲裁
 * @type module
 * @param[IN] int decisionY 当前决策输出，1~8
 * @retval int arbitrationFlag 扭矩仲裁标志，0=未触发，1=触发R7
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
int ACCTorqueArbitrationFlag(int decisionY)
{
    int arbitrationFlag = 0;

    if (decisionY == 7) {
        arbitrationFlag = 1;
    }

    return arbitrationFlag;
}

/**
 * @brief 按时距计算期望跟车距离
 * @en_name ACCDesiredDistance
 * @cn_name ACC期望距离
 * @type module
 * @param[IN] double egoV 自车纵向速度，单位m/s
 * @param[IN] double timeGap 目标时距，单位s
 * @param[IN] double minDistance 最小期望距离，单位m
 * @retval double desiredDistance 期望跟车距离，单位m
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
double ACCDesiredDistance(double egoV, double timeGap, double minDistance)
{
    double desiredDistance = 0.0;

    desiredDistance = egoV * timeGap;
    if (desiredDistance < minDistance) {
        desiredDistance = minDistance;
    }

    return desiredDistance;
}

/**
 * @brief 计算当前实际时距
 * @en_name ACCActualTimeGap
 * @cn_name ACC实际时距
 * @type module
 * @param[IN] double egoV 自车纵向速度，单位m/s
 * @param[IN] double distance 当前车间距，单位m
 * @retval double actualTimeGap 实际时距，单位s；低速时返回9999
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
double ACCActualTimeGap(double egoV, double distance)
{
    double actualTimeGap = 9999.0;
    double distanceSafe = distance;

    if (distanceSafe < 0.0) {
        distanceSafe = 0.0;
    }

    if (egoV > 0.1) {
        actualTimeGap = distanceSafe / egoV;
    }

    return actualTimeGap;
}

/**
 * @brief 计算简化时距控制目标速度
 * @en_name ACCTimeGapTargetSpeed
 * @cn_name ACC时距目标速度
 * @type module
 * @param[IN] double egoV 自车纵向速度，单位m/s
 * @param[IN] double leadV 前车纵向速度，单位m/s
 * @param[IN] double distance 当前车间距，单位m
 * @param[IN] double desiredDistance 期望跟车距离，单位m
 * @param[IN] double kDist 距离误差增益
 * @param[IN] double kSpeed 相对速度误差增益
 * @param[IN] double maxSpeed 最高目标速度，单位m/s
 * @retval double targetSpeed 目标速度，单位m/s
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
double ACCTimeGapTargetSpeed(double egoV, double leadV, double distance,
                             double desiredDistance, double kDist,
                             double kSpeed, double maxSpeed)
{
    double distanceSafe = distance;
    double distanceError = 0.0;
    double relativeSpeed = 0.0;
    double targetSpeed = 0.0;

    if (distanceSafe < 0.0) {
        distanceSafe = 0.0;
    }

    distanceError = distanceSafe - desiredDistance;
    relativeSpeed = leadV - egoV;
    targetSpeed = leadV + (kDist * distanceError) + (kSpeed * relativeSpeed);

    if (targetSpeed < 0.0) {
        targetSpeed = 0.0;
    }
    if (targetSpeed > maxSpeed) {
        targetSpeed = maxSpeed;
    }

    return targetSpeed;
}

/**
 * @brief 简化扭矩仲裁目标速度
 * @en_name ACCTorqueArbitratedTargetSpeed
 * @cn_name ACC仲裁目标速度
 * @type module
 * @param[IN] int decisionY 当前决策输出，1~8
 * @param[IN] double accTargetSpeed ACC计算目标速度，单位m/s
 * @param[IN] double driverTargetSpeed 驾驶员期望目标速度，单位m/s
 * @retval double targetSpeed 仲裁后的目标速度，单位m/s
 * @granularity atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC决策控制
 * @version 1.0
 * @date 2026-06-03
 * @author liuruyu
 */
double ACCTorqueArbitratedTargetSpeed(int decisionY, double accTargetSpeed,
                                      double driverTargetSpeed)
{
    double targetSpeed = accTargetSpeed;

    if (decisionY == 7) {
        if (driverTargetSpeed > accTargetSpeed) {
            targetSpeed = driverTargetSpeed;
        }
    }

    return targetSpeed;
}
