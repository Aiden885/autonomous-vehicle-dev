#include "calcAcceleration.h"
#include <stdio.h>
#include "pidController.h"
#include "mathC.h"

/**
 * @brief 计算纵向加速度控制指令
 * @en_name calcAcceleration
 * @cn_name 纵向加速度控制计算
 * @type module
 * @param type=double,enName=vKp,cnName=速度比例增益,range=[0,100],default=0.0,desc="比例增益，放大误差"
 * @param type=double,enName=vKi,cnName=速度积分增益,range=[0,10],default=0.0,desc="积分增益，消除静态误差"
 * @param type=double,enName=vKd,cnName=速度微分增益,range=[0,10],default=0.0,desc="微分增益，预测误差变化趋势"
 * @param type=double,enName=aKp,cnName=加速度比例增益,range=[0,100],default=0.0,desc="比例增益，放大误差"
 * @param type=double,enName=aKi,cnName=加速度积分增益,range=[0,10],default=0.0,desc="积分增益，消除静态误差"
 * @param type=double,enName=aKd,cnName=加速度微分增益,range=[0,10],default=0.0,desc="微分增益，预测误差变化趋势"
 * @param type=double,enName=integral_max,cnName=积分增益最大值,range=[0,10],default=0.0,desc="积分增益，消除静态误差"
 * @param type=double,enName=derivative_max,cnName=微分增益最大值,range=[0,10],default=0.0,desc="微分增益，预测误差变化趋势"
 * @param type=double,enName=dT,cnName=控制周期,range=[0,10],default=0.0,desc="离散PID控制周期"
 * @param[IN] State* s 当前车辆状态指针
 * @param[IN] Traj* traj 参考轨迹指针
 * @retval double acc1 返回纵向加速度控制输出（m/s^2）
 * @granularity composite
 * @tag_level0 功能模块库
 * @tag_level1 车辆控制
 * @tag_level2 纵向控制
 * @formula 
 * @version 1.0
 * @date 2026-01-28
 * @author lupeng
 */
double calcAcceleration(double vKp, double vKi, double vKd,
                        double aKp, double aKi, double aKd,
                        double integral_max, double derivative_max,
                        double dT,
                        const State* s, const Traj* traj)
{

    // if (s == NULL || traj == NULL || traj->points == NULL || traj->size == 0) {
    //     return 0.0;
    // }
    const double previewDist0 = 3.0;

    /* ---------- 最近点 ---------- */
    int nearestID0 = getNearestIndex(s, traj);
    int nearestID1 = (nearestID0 < 0) ? 0 : nearestID0;

    size_t nearestIndex0 = (size_t)nearestID1;

    /* ---------- 预瞄点查找 ---------- */
    size_t id0 = nearestIndex0;

    for (size_t i = nearestIndex0; i < traj->size; i++) {
        double ds0 = traj->points[i].s - traj->points[nearestIndex0].s;
        if (ds0 >= previewDist0) {
            id0 = i;
            break;
        }
    }

    /* ---------- fallback ---------- */
    size_t id1 = (id0 == nearestIndex0)
        ? (traj->size - 1)
        : id0;

    /* ---------- 参考值 ---------- */
    double v_ref0 = traj->points[id1].v;
    double a_ref0 = traj->points[id1].acc;

    /* ---------- 误差 ---------- */
    double v_error0 = v_ref0 - s->v;
    double a_error0 = a_ref0 - s->accX;

    /* ---------- PID ---------- */
    PIDState vPidState0 = {0};
    PIDState aPidState0 = {0};

    double v_pidFeedback0 =
        pid(vKp, vKi, vKd, dT,
            integral_max, derivative_max,
            v_error0,
            &vPidState0);

    double a_pidFeedback0 =
        pid(aKp, aKi, aKd, dT,
            integral_max, derivative_max,
            a_error0,
            &aPidState0);

    /* ---------- 加速度合成 ---------- */
    double acc0 = v_pidFeedback0 + a_pidFeedback0;

    /* ---------- 限幅 ---------- */
    const double maxAcc0 = 4.0;
    const double minAcc0 = -3.5;

    double acc1 = limit(acc0, minAcc0, maxAcc0);

    /* ---------- 返回 ---------- */
    return acc1;

}
