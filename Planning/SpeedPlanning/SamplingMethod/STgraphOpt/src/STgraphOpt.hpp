#pragma once
/**
 * @brief ST图优化相关声明
 * @file STgraphOpt.hpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

#ifndef STgraphOpt_hpp
#define STgraphOpt_hpp

#include <vector>
#include <stdint.h>
#include <cmath>
#include <tuple>

/**
 * \namespace planning
 * \brief planning namespace
 */

namespace planning{

struct PlanningPoint {
  // local path
  double x;
  double y;
  double angle;
  // global path
  double gaussX;
  double gaussY;
  double gaussAngle;
  // frenet path
  double s;
  double l;
  double frenetAngle;
  // speed
  double v;
  // curvature
  double curvature;
  // accumS
  double accumS;
  // 路点的roadID 和laneID
  int32_t roadID;
  int32_t laneID;
  int32_t pointID;
  // 为了便于应用，增加路点和lane的索引值
  int32_t roadIDIndex;
  int32_t laneIDIndex;
};


struct PlanningTrajectory {
  std::vector<PlanningPoint> planningPoints;
};

struct STgraphOptParam {
  double weight_data = 0.8;
  double weight_smooth = 0.5;
  double tolerance = 0.1;
};

struct STgraphOptInput {
  PlanningTrajectory trajectory;
};

struct STgraphOptOutput {
  PlanningTrajectory trajectory;
};

void STOptimization(const STgraphOptParam &param, const STgraphOptInput &input, STgraphOptOutput &output);

}

#endif /* STgraphOpt_hpp */
