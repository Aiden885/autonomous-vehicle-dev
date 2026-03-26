#ifndef PREVIEW_DISTANCE_PLANNER_HPP
#define PREVIEW_DISTANCE_PLANNER_HPP

#include <vector>

/**
 * @brief 根据曲率序列生成平滑且受限的预瞄距离
 *
 * 该函数内部包含：
 *  - 曲率插值映射
 *  - 弯道进入/退出判定（窗口 + 滞回）
 *  - 一阶低通滤波
 *  - 上升/下降变率限制
 *
 * @param kappaList        曲率序列
 * @param previewIdx       当前预瞄索引
 * @param dT               控制周期(s)
 *
 * @param k_min             曲率下限
 * @param k_max             曲率上限
 * @param minPreviewDist    最小预瞄距离
 * @param maxPreviewDist    最大预瞄距离
 *
 * @param winN              弯道检测窗口长度
 * @param k_enter           进入弯道阈值
 * @param k_exit            退出弯道阈值
 *
 * @param tau_L             低通时间常数
 * @param dL_upMax          最大增加速率 (m/s)
 * @param dL_downMax        最大减小速率 (m/s)
 * @param initPreviewDist   初始预瞄距离
 *
 * @return double           最终预瞄距离
 */
double calcPreviewDistByCurvature(
    const std::vector<double>& kappaList,
    int    previewIdx,
    double dT,

    double k_min,
    double k_max,
    double minPreviewDist,
    double maxPreviewDist,

    int    winN,
    double k_enter,
    double k_exit,

    double tau_L,
    double dL_upMax,
    double dL_downMax,
    double initPreviewDist
);

#endif // PREVIEW_DISTANCE_PLANNER_HPP
