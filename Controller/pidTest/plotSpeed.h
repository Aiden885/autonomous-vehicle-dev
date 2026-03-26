#ifndef SPEED_PLOT_H
#define SPEED_PLOT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化可视化窗口
 */
void speedPlotInit();

/**
 * @brief 更新速度曲线
 * @param time 当前时间
 * @param v 实际速度
 * @param v_ref 参考速度
 */
void speedPlotUpdate(
    double time,
    double v,
    double v_ref
);

/**
 * @brief 关闭可视化
 */
void speedPlotClose();

#ifdef __cplusplus
}
#endif

#endif