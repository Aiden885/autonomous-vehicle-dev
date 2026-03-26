/**
 * @brief B样条曲线生成相关函数
 * @file bSpline.cpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

#include "bSpline.hpp"
#include <iostream>
#include <cmath>

namespace planning{
/**
 * @brief B样条曲线生成
 * @param[IN] param01 曲线参数
 * @param[IN] input01 起始点与终止点
 * @param[OUT] output01 生成路径
 
 * @cn_name: 生成B样条曲线
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */

void generateBsplinePath(const BSplineParam &param, const BSplineInput &input, BSplineOutput &output){
  PlanningPoint startPoint = input.startPoint;
  PlanningPoint endPoint = input.endPoint;
  PlanningTrajectory path;

  double dx = endPoint.gaussX - startPoint.gaussX;
  double dy = endPoint.gaussY - startPoint.gaussY;

  PlanningPoint controlPoint[4] = {0};
  controlPoint[0].x = 0.0;
  controlPoint[0].y = 0.0;
  controlPoint[0].angle = static_cast<int32_t>(360 + startPoint.gaussAngle) % 360;
  controlPoint[3].y = dy; 
  controlPoint[3].x = dx; 
  controlPoint[3].angle = static_cast<int32_t>(360 + endPoint.gaussAngle) % 360;
  double controlPointDistance = param.cpDistance;
  controlPoint[1].x = controlPointDistance * std::sin(startPoint.gaussAngle / 180 * M_PI);
  controlPoint[1].y = controlPointDistance * std::cos(startPoint.gaussAngle / 180 * M_PI);
  controlPoint[1].angle = controlPoint[1].angle;
  controlPoint[2].x = controlPoint[3].x - controlPointDistance * std::sin(controlPoint[3].gaussAngle / 180 * M_PI);
  controlPoint[2].y = controlPoint[3].y - controlPointDistance * std::cos(controlPoint[3].gaussAngle / 180 * M_PI);
  controlPoint[2].angle = controlPoint[3].angle;

  std::vector<PlanningPoint> controlPointList;
  controlPointList.clear();
  for (uint32_t i = 0; i < 4; i++){
    controlPointList.push_back(controlPoint[i]);
  }

  double dt = 1.0 / (param.curvePointNum - 1);
  for (uint32_t i = 0; i < param.curvePointNum; i++){
    PlanningPoint resultpoint;
    
    CpBSplineParam param1;
    CpBSplineInput input1 = {controlPointList, i*dt};
    CpBSplineOutput output1 = {startPoint};

    pointOnCubicBspline(param1, input1, output1);//在这个函数里确定贝塞尔曲线每个点的位置

    resultpoint = output1.Point;

    path.planningPoints.push_back(resultpoint);
    path.planningPoints[i].gaussX = resultpoint.x + startPoint.gaussX;
    path.planningPoints[i].gaussY = resultpoint.y + startPoint.gaussY;

    if (i == 0){
 		resultpoint.angle = startPoint.gaussAngle;
    }
    else {
    	resultpoint.angle = static_cast<int32_t>(
    						360 + atan2(resultpoint.y - path.planningPoints[i-1].y, resultpoint.x - path.planningPoints[i-1].x)) % 360;
    }
    path.planningPoints[i].gaussAngle = static_cast<int32_t>(360 + resultpoint.angle) % 360;

    std::cout << "i: " << i << std::endl;
    std::cout << "x: " << path.planningPoints[i].gaussX << std::endl;
    std::cout << "y: " << path.planningPoints[i].gaussY << std::endl;
    std::cout << "a: " << path.planningPoints[i].gaussAngle << std::endl;
    std::cout << "\n" << std::endl;
  }

  output.globalTrajectory = path;
 }

/**
 * @brief B样条曲线轨迹点计算
 * @param[IN] param 无
 * @param[IN] input 控制点与轨迹点参数
 * @param[OUT] output 生成轨迹点
 
 * @cn_name: 计算B样条曲线上的点
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void pointOnCubicBspline(const CpBSplineParam &param, const CpBSplineInput &input, CpBSplineOutput &output){
	std::vector<PlanningPoint> cp = input.cp;
	double t = input.t;
    PlanningPoint *cpTemp = new PlanningPoint[6];
    for(int i=0; i<4; i++)
        cpTemp[i+1] = cp[i];
    //  将折线延长线上两点加入作为首点和尾点
    cpTemp[0].x = 2*cpTemp[1].x - cpTemp[2].x;                  
    cpTemp[0].y = 2*cpTemp[1].y - cpTemp[2].y;
    cpTemp[5].x = 2*cpTemp[4].x - cpTemp[3].x;
    cpTemp[5].y = 2*cpTemp[4].y - cpTemp[3].y;
    PlanningPoint nodeCp1, nodeCp2, nodeCp3, nodeCp4, nodeCp;
    double F03, F13, F23, F33;
    int i = (int)(t*3);
    double tRemainder = t*3 - (double)i; 
    
    nodeCp1 = cpTemp[i]; 
    nodeCp2 = cpTemp[i+1]; 
    nodeCp3 = cpTemp[i+2]; 
    nodeCp4 = cpTemp[i+3];

    F03 = 1.0/6*(-tRemainder*tRemainder*tRemainder + 3*tRemainder*tRemainder - 3*tRemainder + 1);
    F13 = 1.0/6*(3*tRemainder*tRemainder*tRemainder - 6*tRemainder*tRemainder + 4);
    F23 = 1.0/6*(-3*tRemainder*tRemainder*tRemainder + 3*tRemainder*tRemainder + 3*tRemainder + 1);
    F33 = 1.0/6*tRemainder*tRemainder*tRemainder;

    nodeCp.x=F03*nodeCp1.x+F13*nodeCp2.x+F23*nodeCp3.x+F33*nodeCp4.x;
    nodeCp.y=F03*nodeCp1.y+F13*nodeCp2.y+F23*nodeCp3.y+F33*nodeCp4.y;
    delete []cpTemp;

    output.Point = nodeCp;
}


}