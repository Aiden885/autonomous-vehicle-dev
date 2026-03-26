#ifndef DEBUG_VISUALIZER_H
#define DEBUG_VISUALIZER_H

#include <vector>
#include "controlData.hpp"   // 你的 Control::State, Traj, TrajPoint
#include <opencv2/opencv.hpp>

namespace Control {

/**
 * @brief 调试可视化车辆轨迹、转向和曲率
 * 
 * @param debugFlag 是否启用绘图
 * @param s 车辆当前状态
 * @param traj 参考轨迹
 * @param kappaList 轨迹曲率平滑后的值
 * @param previewIdx 预瞄点索引
 * @param previewDist 预瞄点距离
 * @param k 当前曲率值
 * @param NearestIndex 最近点索引
 * @param NearestIdxLatDis 最近点横向距离
 * @param NearestIdxDis2IMU 最近点到IMU距离
 * @param pureSteerDeg 纯前馈转角
 * @param feedbackSteerDeg 反馈转角
 * @param finalSteerDeg 最终转角
 * @param in_curve 是否在曲线上
 * @param dT 控制周期
 */
void debugVisualizer(
    bool debugFlag,
    const State& s,
    const Traj& traj,
    const std::vector<double>& kappaList,
    int previewIdx,
    double previewDist,
    double k,
    int NearestIndex,
    double NearestIdxLatDis,
    double NearestIdxDis2IMU,
    double pureSteerDeg,
    double feedbackSteerDeg,
    double finalSteerDeg,
    double dT
);

} // namespace Control

#endif
