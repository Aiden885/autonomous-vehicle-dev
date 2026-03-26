/**
 * @brief ST图优化相关函数
 * @file STgraphOpt.cpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

#include "STgraphOpt.hpp"
#include <iostream>
#include <cmath>


namespace planning{

/**
 * @brief ST图优化
 * @param[IN] param 优化参数：权重和容忍误差
 * @param[IN] input 带有速度的轨迹
 * @param[OUT] output 速度优化后的轨迹
 
 * @cn_name: ST图优化
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void STOptimization(const STgraphOptParam &param, const STgraphOptInput &input, STgraphOptOutput &output){
  PlanningTrajectory trajectory = input.trajectory;
  if (trajectory.planningPoints.size() <= 2){
    return;
  }

  PlanningTrajectory trajectoryIn = trajectory;
  PlanningTrajectory smoothTrajectoryVelocityOut = trajectoryIn;

  double change = param.tolerance;
  double vtemp;
  int nIterations = 0;

  int size = trajectoryIn.planningPoints.size();

  while (change >= param.tolerance){
    change = 0.0;
    for (int i = 1; i < size - 1; i++){
      vtemp = smoothTrajectoryVelocityOut.planningPoints[i].v; 

      smoothTrajectoryVelocityOut.planningPoints[i].v += param.weight_data
          * (trajectoryIn.planningPoints[i].v - smoothTrajectoryVelocityOut.planningPoints[i].v);

      smoothTrajectoryVelocityOut.planningPoints[i].v += param.weight_smooth
          * (smoothTrajectoryVelocityOut.planningPoints[i - 1].v + smoothTrajectoryVelocityOut.planningPoints[i + 1].v
              - (2.0 * smoothTrajectoryVelocityOut.planningPoints[i].v));

      change += fabs(vtemp - smoothTrajectoryVelocityOut.planningPoints[i].v);

    }
    nIterations++;
  }

  output.trajectory = smoothTrajectoryVelocityOut;
  // std::cout << nIterations << std::endl;
}


}