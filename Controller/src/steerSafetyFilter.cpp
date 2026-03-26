#include "steerSafetyFilter.h"
#include <math.h>
#include <algorithm>


#define dT  (0.05)      //class=param,type=double,name=控制周期(dT),group=PID,range=[0,100],desc="控制周期"


/**
 * @brief 转向角安全滤波与限幅处理（死区 + 变率限制 + EMA 平滑）
 * @en_name steerSafetyFilter
 * @cn_name 转向角安全滤波器
 * @type module
 *
 * @param[IN] double& steer 转向角指令（输入为原始控制输出，输出为安全滤波后的转向角，单位 rad）
 * @param[IN] double& last_safe_rad 上一帧安全转向角（函数内部更新，用于死区、变率与平滑计算）
 * @param[IN] double maxSteerRad 转向角最大物理限制（对称 ±maxSteerRad）
 *
 * @retval void 无返回值（通过引用参数输出安全转向角）
 * @granularity composite
 * @tag_level1 controller
 * @tag_level2 LatController
 * @formula
 *
 * @version 1.0
 * @date 2026-01-19
 * @author lupeng
 */
void steerSafetyFilter(
    double& steer,
    double& last_safe_rad,
    double  maxSteerRad
)
{
    const double deadband_rad   = 0.0028;   // 死区（rad）
    const double maxRateRadPerS = 1.10;     // 最大变化率（rad/s）
    const double alpha          = 0.1745;   // EMA 平滑系数
    // =========================================================

    const double max_abs_rad = std::max(1e-9, std::fabs(maxSteerRad));

    auto isFinite = [](double v) -> bool {
        return std::isfinite(v) != 0;
    };

    auto clampAbs = [](double v, double a) -> double {
        const double aa = std::max(0.0, a);
        return std::max(-aa, std::min(aa, v));
    };

    /* ---------- 非法输入保护 ---------- */
    if (!isFinite(steer)) {
        steer = clampAbs(last_safe_rad, max_abs_rad);
    }

    /* ---------- 幅值限制 + 死区 ---------- */
    double target = clampAbs(steer, max_abs_rad);
    const double diff = target - last_safe_rad;

    if (std::fabs(diff) < deadband_rad) {
        target = last_safe_rad;
    }

    /* ---------- dt 非法，直接透传 ---------- */
    if (!(dT > 0.0) || !isFinite(dT)) {
        steer = clampAbs(target, max_abs_rad);
        last_safe_rad = steer;
        return;
    }

    /* ---------- 变率限制 ---------- */
    const double max_step = maxRateRadPerS * dT;
    double limited = target;
    const double d = target - last_safe_rad;

    if (d >  max_step)      limited = last_safe_rad + max_step;
    else if (d < -max_step) limited = last_safe_rad - max_step;

    limited = clampAbs(limited, max_abs_rad);

    /* ---------- EMA 平滑 ---------- */
    double out = limited;
    if (alpha > 0.0) {
        out = alpha * limited + (1.0 - alpha) * last_safe_rad;
        out = clampAbs(out, max_abs_rad);
    }

    steer = out;
    last_safe_rad = steer;
}
