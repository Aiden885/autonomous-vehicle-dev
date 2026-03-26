#include "mathC.h"

/**
 * @brief 轨迹曲率平滑（9 点 Savitzky–Golay 二次滤波）
 * @en_name smoothCurvatureSG9
 * @cn_name 轨迹曲率平滑
 * @type module
 * @param[IN] Traj* traj 轨迹点数组结构体
 * @param[OUT] double* out 轨迹点曲率数组指针
 * @retval void 返回值空
 * @granularity atomic
 * @tag_level0 基础模块库
 * @tag_level1 规划与决策
 * @tag_level2 轨迹规划
 * @formula
 * @version 1.0
 * @date 2026-01-15
 * @author lupeng
 */
void smoothCurvatureSG9(const Traj* traj, double* out)
{
    int n = traj->size;

    const double c[9] = {
        -21.0/231.0, 14.0/231.0, 39.0/231.0, 54.0/231.0, 59.0/231.0,
         54.0/231.0, 39.0/231.0, 14.0/231.0,-21.0/231.0
    };

    for(int i=0;i<n;i++)
    {
        double v=0;

        for(int j=-4;j<=4;j++)
        {
            int idx=i+j;

            if(idx<0) idx=-idx;
            if(idx>=n) idx=2*n-2-idx;

            double k=traj->points[idx].k;

            if(!isfinite(k))
                k=0;

            v+=c[j+4]*k;
        }

        out[i]=v;
    }
}


/**
 * @brief 对称限幅函数，防止积分微分过饱和
 * @en_name limitSymmetrical
 * @cn_name 对称限幅函数
 * @type module
 * @param[IN] double value 输入值
 * @param[IN] double limit 正限幅值
 * @retval double value 限幅后的值，范围[-limit,limit]
 * @granularity atomic
 * @tag_level0 基础模块库
 * @tag_level1 数学运算
 * @tag_level2 数学函数
 * @formula /
 * @version 1.0
 * @date 2026-01-10
 * @author lupeng
 */
double limitSymmetrical(double value, double limit)
{
    // if (value > limit) value = limit;
    // if (value < -limit) value = -limit;
    // return value;

    double value0 = value;
    /* 第一次判断 */
    double value1;
    if (value0 > limit)
        value1 = limit;
    else
        value1 = value0;
    /* 第二次判断 */
    double value2;
    if (value1 < -limit)
        value2 = -limit;
    else
        value2 = value1;
    return value2;
}


/**
 * @brief 对输入值进行限幅处理
 * @en_name clamp
 * @cn_name 数值限幅函数
 * @type module
 * @param[IN] double v 输入数值
 * @param[IN] double min 最小限幅值
 * @param[IN] double max 最大限幅值
 * @retval double v 返回限幅后的数值，若 v < min 返回 min，若 v > max 返回 max，否则返回 v
 * @granularity atomic
 * @tag_level0 基础模块库
 * @tag_level1 数学运算
 * @tag_level2 数学函数
 * @formula 
 * v_out = min(max(v, min), max)
 * @version 1.0
 * @date 2026-01-28
 * @author lupeng
 */
double limit(double v, double min, double max)
{
    double v0 = v;

    /* 下限裁剪 */
    double v1;
    if (v0 < min)
        v1 = min;
    else
        v1 = v0;

    /* 上限裁剪 */
    double v2;
    if (v1 > max)
        v2 = max;
    else
        v2 = v1;

    return v2;
}
