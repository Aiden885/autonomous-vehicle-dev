#pragma once
/**
 * @brief 轨迹速度规划初始化相关声明 
 * @file initTrajectorySpeed.hpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

#ifndef initTrajectorySpeed_hpp
#define initTrajectorySpeed_hpp

#include <vector>
#include <stdint.h>
#include <cmath>
#include "prediction.pb.h"

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

struct TrajSpeedInitParam {
};

struct TrajSpeedInitInput {
  PlanningTrajectory trajectory;
  prediction::ObjectList prediction;
};

struct TrajSpeedInitOutput {
  PlanningTrajectory globalTrajectory;
};

struct GetMinObjDisParam {
  double predictFrequency = 0.1;
};

struct GetMinObjDisInput {
  PlanningPoint point;
  prediction::ObjectList prediction;
  double t;
};

struct GetMinObjDisOutput {
  double distance;
};

struct GetDistanceParam {
};

struct GetDistanceInput {
  double x1;
  double y1; 
  double x2;
  double y2;
};

struct GetDistanceOutput {
  double distance;
};

struct SpeedModelParam {
  double maxspeed = 10;
  double minspeed = 0;
  double d1 = 4.5;
  double d2 = 0.5;
};

struct SpeedModelInput {
  double distance;
};

struct SpeedModelOutput {
  double speed;
};

struct FindMinParam {
};

struct FindMinInput {
  std::vector<double> array;
};

struct FindMinOutput {
  double flag;
};

void initSpeedForTrajectory(const TrajSpeedInitParam &param, const TrajSpeedInitInput &input, TrajSpeedInitOutput &output);

void getMinDistanceOfPoint(const GetMinObjDisParam &param, const GetMinObjDisInput &input, GetMinObjDisOutput &output);

void getDistance(const GetDistanceParam &param, const GetDistanceInput &input, GetDistanceOutput &output);

void speedModel(const SpeedModelParam &param, const SpeedModelInput &input, SpeedModelOutput &output);

void findMin(const FindMinParam &param, const FindMinInput &input, FindMinOutput &output);

}

#endif /* initTrajectorySpeed_hpp */
