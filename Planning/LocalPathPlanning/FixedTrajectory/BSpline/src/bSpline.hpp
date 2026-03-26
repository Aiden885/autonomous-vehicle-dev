#pragma once
/**
 * @brief B样条曲线生成相关声明
 * @file bSpline.hpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

#ifndef bSpline_hpp
#define bSpline_hpp

#include <vector>
#include <stdint.h>
#include <cmath>

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

struct BSplineParam {
  double cpDistance = 3;
  double curvePointNum = 50;
};

struct BSplineInput {
  PlanningPoint startPoint;
  PlanningPoint endPoint;
};

struct BSplineOutput {
  PlanningTrajectory globalTrajectory;
};

struct CpBSplineParam {
};

struct CpBSplineInput {
  std::vector<PlanningPoint> cp;
  double t;
};


struct CpBSplineOutput {
  PlanningPoint Point;
};

void generateBsplinePath(const BSplineParam &param, const BSplineInput &input, BSplineOutput &output);

void pointOnCubicBspline(const CpBSplineParam &param, const CpBSplineInput &input, CpBSplineOutput &output);

}
#endif /* BSpline_hpp */
