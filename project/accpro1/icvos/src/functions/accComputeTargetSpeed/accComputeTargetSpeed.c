
#include "accpro1.h"
#ifndef ACC_DESIRED_DIST
#define ACC_DESIRED_DIST (15.0)
#endif
#ifndef ACC_MAX_SPEED
#define ACC_MAX_SPEED (12.0 / 3.6)
#endif

double accComputeTargetSpeed(
    double egoV,
    double leadV,
    double distance
)
{
    double kDist        = 0.5;   /* 距离误差增益（与 cruiseController 保持一致，硬编码）*/
    double kSpeed       = 0.5;   /* 速度差增益（与 cruiseController 保持一致，硬编码）*/
    double distDiff     = 0.0;   /* 当前车间距与期望车间距之差（m）*/
    double speedDiff    = 0.0;   /* 前车速度与自车速度之差（m/s）*/
    double targetSpeed0 = 0.0;   /* ACC控制律输出的原始目标速度（m/s，未限幅）*/
    double targetSpeed1 = 0.0;   /* 限幅后的最终目标速度（m/s）*/
    bool tooSlow        = false; /* 原始目标速度 <= 0，下钳位为 0 */
    bool tooFast        = false; /* 原始目标速度 > ACC_MAX_SPEED，上钳位 */
    bool inputValid     = false; /* egoV / leadV / distance 均为有限值 */
    double distanceSafe = 0.0;   /* 负距离钳位后的车间距（m），保证 >= 0 */

    /* 步骤0：输入合法性检查，非有限值（NaN / Inf）直接返回 0（安全停车）*/
    inputValid = (isfinite(egoV) && isfinite(leadV) && isfinite(distance));
    if (!inputValid)
    {
        targetSpeed1 = 0.0;
    }
    else
    {

    /* ---- 来源：generateSpeedList.c cruiseController（2026-01-19）----
     * 将原始 Protobuf 入参（Pc__Imu*, Infopack__ObjectsVec*）替换为普通 double，
     * 控制律完全相同，适用于仿真/无传感器场景 */

    /* 步骤1：距离下界钳位（distance 应为正值，负值表示前车在身后，视为 0）*/
    bool distanceNegative = (distance < 0.0); /* distance 为负，钳位为 0 */
    if (distanceNegative)
    {
        distanceSafe = 0.0;
    }
    else
    {
        distanceSafe = distance;
    }

    /* 步骤2：计算距离误差和速度差 */
    distDiff  = distanceSafe - ACC_DESIRED_DIST;
    speedDiff = leadV - egoV;

    /* 步骤3：ACC控制律合成原始目标速度
     *   距离偏大（distDiff > 0）→ targetSpeed 升高，加速追赶
     *   距离偏小（distDiff < 0）→ targetSpeed 降低，减速保距
     *   速度差前馈保证稳态时与前车同速 */
    targetSpeed0 = kDist * distDiff + kSpeed * speedDiff + leadV;

    /* 步骤4：限幅到 [0, ACC_MAX_SPEED]
     *   下界 0：防止倒车指令
     *   上界 ACC_MAX_SPEED：不超过期望巡航速度 */
    tooSlow = (targetSpeed0 <= 0.0);
    tooFast = (targetSpeed0 > ACC_MAX_SPEED);

    if (tooSlow)
    {
        targetSpeed1 = 0.0;
    }
    else
    {
        if (tooFast)
        {
            targetSpeed1 = ACC_MAX_SPEED;
        }
        else
        {
            targetSpeed1 = targetSpeed0;
        }
    }

    } /* end else (inputValid) */

    return targetSpeed1;
}