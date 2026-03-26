/**
 * @brief 轨迹速度规划初始化相关函数
 * @file initTrajectorySpeed.cpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@tsinghua.edu.com)
 * @date 2023-11-25
 */

#include "initTrajectorySpeed.hpp"
#include <iostream>
#include <cmath>

namespace planning{

/**
 * @brief 根据障碍物位置，轨迹速度规划初始化
 * @param[IN] param 无
 * @param[IN] input 轨迹与障碍物
 * @param[OUT] output 带有速度的轨迹
 
 * @cn_name: 轨迹速度规划初始化
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */

void initSpeedForTrajectory(const TrajSpeedInitParam &param, const TrajSpeedInitInput &input, TrajSpeedInitOutput &output){
  
  PlanningTrajectory trajectory = input.trajectory;
  prediction::ObjectList prediction = input.prediction;

  int i = 0;
  double t = 0;
  double distance;
  bool stopFlag = false;
  for (i = 0; i < trajectory.planningPoints.size() - 1; i++)
  {
    GetMinObjDisParam param1;
    GetMinObjDisInput input1{
      trajectory.planningPoints[i], 
      prediction, 
      t
    };
    GetMinObjDisOutput output1{0};
    getMinDistanceOfPoint(param1, input1, output1);
    distance = output1.distance;


    SpeedModelParam param2{};
    SpeedModelInput input2{distance};
    SpeedModelOutput output2{0};
    speedModel(param2, input2, output2);
    trajectory.planningPoints[i].v = output2.speed;

    if (trajectory.planningPoints[i].v > 0){
      GetDistanceParam param3;
      GetDistanceInput input3{
        trajectory.planningPoints[i].x,
        trajectory.planningPoints[i].y,
        trajectory.planningPoints[i+1].x,
        trajectory.planningPoints[i+1].y
      };
      GetDistanceOutput output3{0};
      getDistance(param3, input3, output3);

      t += output3.distance / trajectory.planningPoints[i].v;
    }
    else{
      stopFlag = true;
      break;
    }
  }
  if (stopFlag)
  {
    for (; i < trajectory.planningPoints.size(); i++)
    {
      trajectory.planningPoints[i].v = 0;
    }
  }
  else
  {
    trajectory.planningPoints[i].v = trajectory.planningPoints[i-1].v;
  }

  output.globalTrajectory = trajectory;
}

/**
 * @brief 在预测范围内，动态计算轨迹点到最近障碍物的距离
 * @param[IN] param 预测频率
 * @param[IN] input 轨迹点，障碍物，未来时间
 * @param[OUT] output 最近距离
 
 * @cn_name: 轨迹点到障碍物的最近动态距离计算
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getMinDistanceOfPoint(const GetMinObjDisParam &param, const GetMinObjDisInput &input, GetMinObjDisOutput &output){
  PlanningPoint point = input.point;
  prediction::ObjectList prediction = input.prediction;
  double t = input.t;

  int index;
  double tRemainder;
  PlanningPoint predictTempPoint;
  std::vector<double> distanceFromObject;
  index = (int)t / param.predictFrequency;
  tRemainder = t - index * param.predictFrequency;

  if (index >= 19){
    output.distance = 100;
    return;
  }


  for (auto object : prediction.object()){
    predictTempPoint.x = object.predictpoint(index).x() * (1 - tRemainder) + object.predictpoint(index + 1).x() * tRemainder;
    predictTempPoint.y = object.predictpoint(index).y() * (1 - tRemainder) + object.predictpoint(index + 1).y() * tRemainder;
    
    GetDistanceParam param4;
    GetDistanceInput input4{
      point.x,
      point.y,
      predictTempPoint.x + object.w() / 2,
      predictTempPoint.y + object.l() / 2
    };
    GetDistanceOutput output4{0};
    getDistance(param4, input4, output4);

    GetDistanceParam param5;
    GetDistanceInput input5{
      point.x,
      point.y,
      predictTempPoint.x - object.w() / 2,
      predictTempPoint.y + object.l() / 2
    };
    GetDistanceOutput output5{0};
    getDistance(param5, input5, output5);

    GetDistanceParam param6;
    GetDistanceInput input6{
      point.x,
      point.y,
      predictTempPoint.x + object.w() / 2,
      predictTempPoint.y - object.l() / 2
    };
    GetDistanceOutput output6{0};
    getDistance(param6, input6, output6);    

    GetDistanceParam param7;
    GetDistanceInput input7{
      point.x,
      point.y,
      predictTempPoint.x - object.w() / 2,
      predictTempPoint.y -object.l() / 2
    };
    GetDistanceOutput output7{0};
    getDistance(param7, input7, output7);

    distanceFromObject.push_back(output4.distance);
    distanceFromObject.push_back(output5.distance);
    distanceFromObject.push_back(output6.distance);
    distanceFromObject.push_back(output7.distance);
  }

  if(distanceFromObject.size() > 0){
    FindMinParam param1;
    FindMinInput input1 = {distanceFromObject};
    FindMinOutput output1 = {0};
    findMin(param1, input1, output1);
    output.distance = output1.flag;

    return;
  }
  else{
    output.distance = 100;
    return;
  } 
}

/**
 * @brief 计算距离
 * @param[IN] param 无
 * @param[IN] input 两个点的横纵坐标
 * @param[OUT] output 距离
 
 * @cn_name: 计算距离
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getDistance(const GetDistanceParam& param, const GetDistanceInput& input, GetDistanceOutput& output){
  double x1 = input.x1;
  double x2 = input.x2;
  double y1 = input.y1;
  double y2 = input.y2;

  output.distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

/**
 * @brief 根据最大最小速度，与距离障碍物距离，生成速度的模型
 * @param[IN] param 最大速度，最小速度，模型参数
 * @param[IN] input 距离障碍物距离
 * @param[OUT] output 生成的轨迹点速度
 
 * @cn_name: 速度生成模型
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void speedModel(const SpeedModelParam &param, const SpeedModelInput &input, SpeedModelOutput &output){
  double distance = input.distance;
  double maxspeed = param.maxspeed;
  double minspeed = param.minspeed;
  double d1 = param.d1;
  double d2 = param.d2;

  if (distance < 0)
  {
    output.speed = minspeed;
    return;
  }

  if (distance < d2)
  {
    output.speed = minspeed;
    return;
  }
  else if (distance > d1)
  {
    output.speed = maxspeed;
    return;
  }
  else
  {
    output.speed = minspeed + (maxspeed - minspeed) * (distance - d2) / (d1 - d2);
    return;
  }
} 

/**
 * @brief double数组最小值
 * @param[IN] param 无
 * @param[IN] input double数组
 * @param[OUT] output 最小值
 
 * @cn_name: double数组最小值
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void findMin(const FindMinParam &param, const FindMinInput &input, FindMinOutput &output){
  std::vector<double> array = input.array;
  if (array.size() == 0){
    output.flag = -1;
    return;
  }
  double flag = array[0];
  for (int i = 1; i < array.size(); i++)
  {
    if (array[i] < flag)
    {
      flag = array[i];
    }
  }
  output.flag = flag;
  return;
}



}