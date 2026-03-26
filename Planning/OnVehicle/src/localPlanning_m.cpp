#include "localPlanning_m.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "interpolate_m.h"
#include <algorithm>

namespace planning{

/**
 * @brief 局部规划函数
 * @file localPlanning_m.cpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@mails.tsinghua.edu.cn)
 * @date 2023-11-25
 */

#define PLANNING_VELOCITY_MULTIPLIER 2.6
#define DEFAULT_PLANNING_DISTANCE 5
#define MAX_PLANNING_DISTANCE 6
#define MIN_PLANNING_DISTANCE 5
#define DISTANCE_RANGE 3

#define DEFAULT_SECOND_PLANNING_DISTANCE_LONGER 5 // distance from control planning to lidar planning

// 20220826
#define POINT_DIS_THRESHOLD_ON_CURRENT_LANE 3 // 3    //认为车在当前路点的距离阈值（m）
#define POINT_DIS_THRESHOLD_ON_OTHER_LANE 2
#define POINT_ANGLE_THRESHOLD 120 // 认为车在当前路点的朝向角阈值（degree）

#define END_SPEED_NUM 1        // 末速度采样数量 原始数值是1 20230212
#define DESIRED_SPEED 12 / 3.6 // m/s
#define SPEED_RANGE 3 / 3.6    // m/s
#define CURVATURE_SPEED 10 / 3.6
#define CURVATURE_THRESHOLS 0.1 // 曲率门限值

#define VEHICLE_DIAGONAL 2.7951
#define VEHICLE_DIAGONAL_DEGREE 26.5651
#define PREDICT_FREQUENCY 0.1

#define STOP_JUDGE_DISTANCE 6 // 20230214  减小 8

#define MAX_FRENET_S 8                      // 规划预瞄距离
#define ACC_RELA_DIST 15.0                  // ACC跟车保持距离，目前这个数值不合适，会被认为是障碍物
#define ACC_BEGIN_DIST (ACC_RELA_DIST + 30) // ACC进入跟车的距离 +5
#define ACC_FOLLOW_OBJECT_ID 2              // ACC 跟车前车ID
#define doNothing()

#define SAFE_DISTANCE 1.0

// add by shyp 20230209 formular from Fusion program
/**
 * @brief 经纬度转全局坐标
 * @param[IN] param 无
 * @param[IN] input 坐标经纬度，初始化全局坐标xy值
 * @param[OUT] output 全局坐标值
 
 * @cn_name: 经纬度转全局坐标

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void gaussConvert(const gaussConvertParam &param, const gaussConvertInput &input, gaussConvertOutput &output)
{
    double longitude1 = input.longitude1;
    double latitude1 = input.latitude1;
    double dNorth_X = input.dNorth_X;
    double dEast_Y = input.dEast_Y;

    double a = 6378137.0;

    double e2 = 0.0066943799013;

    double latitude2Rad = (M_PI / 180.0) * latitude1;

    int beltNo = int((longitude1 + 1.5) / 3.0);
    int L = beltNo * 3;
    double l0 = longitude1 - L;
    double tsin = sin(latitude2Rad);
    double tcos = cos(latitude2Rad);
    double t = tan(latitude2Rad);
    double m = (M_PI / 180.0) * l0 * tcos;
    double et2 = e2 * pow(tcos, 2);
    double et3 = e2 * pow(tsin, 2);
    double X = 111132.9558 * latitude1 - 16038.6496 * sin(2 * latitude2Rad) + 16.8607 * sin(4 * latitude2Rad) - 0.0220 * sin(6 * latitude2Rad);
    double N = a / sqrt(1 - et3);

    dNorth_X = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);
    dEast_Y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
    //   std::cout << BOLDRED << "x1: " << x1 << std::endl;
    //   std::cout << BOLDRED << "y1: " << y1 << std::endl;
    output.dNorth_X = dNorth_X;
    output.dEast_Y = dEast_Y;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// by syp
// 2D的坐标变换公式，已知全局坐标，车辆坐标系远点在全局坐标系中的坐标，求改点在车辆坐标系中的位置
// 坐标系都为笛卡尔坐标系，xy为右上
// double dX0, double dY0 新坐标原点在旧坐标系中的位置
// double dPhi0为原有X轴在旧坐标系中的角度
/**
 * @brief 全局坐标转局部坐标
 * @param[IN] param 无
 * @param[IN] input 全局坐标经纬度、角度，初始化局部坐标xy值、角度
 * @param[OUT] output 局部坐标值、角度
 
 * @cn_name: 全局坐标转自局部坐标

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void coordTran2D(const coordTran2DParam &param, const coordTran2DInput &input, coordTran2DOutput &output)
{
    double dX = input.dX;
    double dY = input.dY;
    double dPhi = input.dPhi;
    double dX0 = input.dX0;
    double dY0 = input.dY0;
    double dPhi0 = input.dPhi0;

    double xTemp, yTemp;

    // 坐标平移
    xTemp = dX - dX0; // 终点转换后的坐标X
    yTemp = dY - dY0; // 终点转换后的坐标Y

    // 坐标旋转
    dPhi0 = -dPhi0;
    dX = xTemp * cos(dPhi0) - yTemp * sin(dPhi0);
    dY = xTemp * sin(dPhi0) + yTemp * cos(dPhi0);
    dPhi = dPhi + dPhi0; // 终点转换后的角度

    output.dX = dX;
    output.dY = dY;
    output.dPhi = dPhi;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 2022
/**
 * @brief 判断点是否在区域内
 * @param[IN] param 无
 * @param[IN] input 目标区域中心XY坐标，判断点XY坐标
 * @param[OUT] output 判断结果
 
 * @cn_name: 判断点是否在区域内

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void inArea(const inAreaParam &param, const inAreaInput &input, inAreaOutput &output)
{
    // std::cout << "inArea" << std::setprecision(10) << longitudeX << " " << latitudeY << " " << targetX << " " << targetY << std::endl;
    double longitudeX = input.longitudeX;
    double latitudeY = input.latitudeY;
    double targetX = input.targetX;
    double targetY = input.targetY;
    bool ret = false;
    if (std::abs(longitudeX - targetX) < STOP_JUDGE_DISTANCE && std::abs(latitudeY - targetY) < STOP_JUDGE_DISTANCE - 2)
    {

        ret = true;
    }
    else
    {
        ret = false;
    }

    output.flag = ret;
    return;
}

/**
 * @brief 计算两点之间的距离
 * @param[IN] param 无
 * @param[IN] input 两点的XY坐标值
 * @param[OUT] output 距离
 
 * @cn_name: 计算两点之间的距离

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void pointDistance(const pointDistanceParam &param, const pointDistanceInput &input, pointDistanceOutput &output)
{
    double x1 = input.x1;
    double x2 = input.x2;
    double y1 = input.y1;
    double y2 = input.y2;

    double distance;
    double dX;
    double dY;
    dX = std::abs(x1 - x2);
    dY = std::abs(y1 - y2);
    distance = sqrt(dX * dX + dY * dY);
    output.distance = distance;
    return;
}

// revise courseangle of roadpoint from 0 for north to 0 for east:
/**
 * @brief 逆时针旋转坐标系90度，向东0度变成向北0度
 * @param[IN] param 无
 * @param[IN] input 原始角度
 * @param[OUT] output 转化角度
 
 * @cn_name: 坐标系逆时针旋转90度

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void courseAngleRevise(const courseAngleReviseParam &param, const courseAngleReviseInput &input, courseAngleReviseOutput &output)
{
    double ang = input.ang;
    if (ang > 90)
    {
        ang = ang - 90;
    }
    else
    {
        ang = ang + 270;
    }
    output.ang = ang;
    return;
}

/**
 * @brief 从文件中加载停止点
 * @param[IN] param 文件地址
 * @param[IN] input 停止点、停止点对应路点、X偏移、Y偏移
 * @param[OUT] output 更新后的停止点、更新后的停止点对应路点
 
 * @cn_name: 加载停止点

 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void loadStopPoints(const loadStopPointsParam &param, const loadStopPointsInput &input, loadStopPointsOutput &output)
{
    std::string fileName = param.fileName;
    std::vector<GaussRoadPoint> rawStopPoints = input.rawStopPoints;
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId = input.stopPointRoadLanePointId;
    double offsetX = input.offsetX;
    double offsetY = input.offsetY;

    rawStopPoints.clear();
    std::string line;
    std::ifstream fs;
    fs.open(fileName, std::ios::in);
    if (fs.fail())
    {
        std::cout << RED << "!!!!!!FATAL!!!!!!LOAD STOP POINT FAIL!!!!!!" << RESET << std::endl;
    }
    else
        ;
    GaussRoadPoint roadPoint;
    rawStopPoints.push_back(roadPoint); // push an empty one so that index starting from 1;
    while (getline(fs, line))
    {
        if (line.length() > 0)
        {
            std::stringstream ss(line);
            ss >> roadPoint.GaussX >> roadPoint.GaussY >> roadPoint.yaw >> std::get<0>(stopPointRoadLanePointId) >> std::get<1>(stopPointRoadLanePointId) >> std::get<2>(stopPointRoadLanePointId);
            roadPoint.GaussX += offsetX;
            roadPoint.GaussY += offsetY;
            rawStopPoints.push_back(roadPoint);
        }
        else
            ;
    }
    fs.close();
    output.rawStopPoints = rawStopPoints;
    output.stopPointRoadLanePointId = stopPointRoadLanePointId;
}

// 新增从yaml文件读取停止位置
/**
 * @brief 从文件中加载停止点
 * @param[IN] param 地图、文件地址
 * @param[IN] input 停止点、停止点对应路点、X偏移、Y偏移
 * @param[OUT] output 更新的停止点、更新的停止点对应路点
 
 * @cn_name: 从Yaml文件中加载停止点
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void loadStopPointsFromYaml(const loadStopPointsFromYamlParam &param, const loadStopPointsFromYamlInput &input, loadStopPointsFromYamlOutput &output)
{
    RoadMap map_ = param.map_;
    std::string fileName = param.fileName;
    std::vector<GaussRoadPoint> rawStopPoints = input.rawStopPoints;
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId = input.stopPointRoadLanePointId;
    double offsetX = input.offsetX;
    double offsetY = input.offsetY;

    try
    {
        YAML::LoadFile(fileName); // this is rvalue
    }
    catch (YAML::BadFile &bfe)
    {
        std::cout << BOLDRED << "yaml file is not exist!" << std::endl;
        std::cout << "    " << bfe.what() << RESET << std::endl;
        exit(0);
    }

    YAML::Node yaml_node = YAML::LoadFile(fileName);

    try
    {
        yaml_node["roadIdToStop"].as<int>();                 // this is rvalue
        yaml_node["laneIdToStop"].as<int>();                 // this is rvalue
        yaml_node["approximateLocationToStop"].as<double>(); // this is rvalue
    }
    catch (YAML::TypedBadConversion<int> &tbce_int)
    {
        std::cout << BOLDRED << "roadIdToStop or laneIdToStop is not exist!" << std::endl;
        std::cout << "    " << tbce_int.what() << RESET << std::endl;
        exit(0);
    }
    catch (YAML::TypedBadConversion<double> &tbce_double)
    {
        std::cout << BOLDRED << "approximateLocationToStop is not exist!" << std::endl;
        std::cout << "    " << tbce_double.what() << RESET << std::endl;
        exit(0);
    }

    int road_id = yaml_node["roadIdToStop"].as<int>();
    int lane_id = yaml_node["laneIdToStop"].as<int>();
    auto approximate_location = yaml_node["approximateLocationToStop"].as<double>();

    if (approximate_location > 100.0)
        approximate_location = 100.0;
    if (approximate_location < 0.0)
        approximate_location = 0.0;

    bool find_flag = false;
    for (const auto &road_iter : map_.roads)
    {
        if (road_iter.id == road_id)
        {
            for (const auto &lane_iter : road_iter.lanes)
            {
                if (lane_iter.id == lane_id)
                {
                    auto target_index = static_cast<unsigned int>(static_cast<unsigned int>(lane_iter.gaussRoadPoints.size()) * approximate_location);
                    std::cout << "this is the \'real\' index: " << target_index << std::endl;

                    if (target_index > 0)
                        target_index -= 1;                                                      // make 'zeroth' equals to first.
                    std::get<2>(stopPointRoadLanePointId) = static_cast<int>(target_index) + 1; // start from one instead of zero
                    std::get<0>(stopPointRoadLanePointId) = road_id;
                    std::get<1>(stopPointRoadLanePointId) = lane_id;

                    for (int i = 0; i < (int)lane_iter.gaussRoadPoints.size(); i++)
                    {
                        if (i == target_index)
                        {
                            std::cout << "stopPoint's gaussX: " << std::fixed << std::setprecision(3) << lane_iter.gaussRoadPoints.at(i).GaussX
                                      << " stopPoint's gaussY: " << lane_iter.gaussRoadPoints.at(i).GaussY << " stopPoint's yaw: " << lane_iter.gaussRoadPoints.at(i).yaw << std::endl;
                            std::cout.unsetf(std::ios::fixed);
                            rawStopPoints.clear();
                            GaussRoadPoint roadPoint{};
                            rawStopPoints.push_back(roadPoint);

                            roadPoint.GaussX = lane_iter.gaussRoadPoints.at(i).GaussX + offsetX;
                            roadPoint.GaussY = lane_iter.gaussRoadPoints.at(i).GaussY + offsetY;
                            roadPoint.yaw = lane_iter.gaussRoadPoints.at(i).yaw;
                            rawStopPoints.emplace_back(roadPoint);
                            find_flag = true;
                            break;
                        }
                    }
                    break;
                }
            }
            break;
        }
    }

    if (!find_flag)
    {
        std::cout << BOLDRED << "sorry, the value of roadIdToStop or laneIdToStop is illegal!" << RESET << std::endl;
        exit(0);
    }
    output.rawStopPoints = rawStopPoints;
    output.stopPointRoadLanePointId = stopPointRoadLanePointId;
}

/**
 * @brief 从全局规划中，获取后续车道和道路Id
 * @param[IN] param 储存全局规划车道和道路的向量
 * @param[IN] input 自车当前车道和道路Id
 * @param[OUT] output 后续车道和道路Id
 
 * @cn_name: 获取后续车道和道路Id
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getNextRoadLaneId(const getNextRoadLaneIdParam &param, const getNextRoadLaneIdInput &input, getNextRoadLaneIdOutput &output)
{
    std::vector<std::tuple<int32_t, int32_t>> routingList = param.routingList;
    std::tuple<int32_t, int32_t> currentId = input.currentId;

    std::cout << "getNextRoadLaneId";
    for (int i = 0; i < (int)routingList.size(); i++)
    {
        std::cout << " " << std::get<0>(routingList[i]) << "," << std::get<1>(routingList[i]) << ";";
    }
    std::cout << std::endl;

    int32_t nextIndex = 1;

    static uint32_t currentIdIndex = 1; // （貌似应当是从0开始？不知为啥要从1开始，注意是static，后续会更新为0）

    std::cout << "static uint32_t currentIdIndex  "
              << "," << currentIdIndex << std::endl;
    uint32_t pathMaxIdIndex = static_cast<uint32_t>(routingList.size()) - 1; // 这条全局路径上有多少<Road,Lane>

    if (currentIdIndex < pathMaxIdIndex)
    {
        std::cout << "currentId " << std::get<0>(currentId) << "," << std::get<1>(currentId) << std::endl;
        std::cout << "routingList[currentIdIndex] "
                  << "," << currentIdIndex << "," << std::get<0>(routingList[currentIdIndex]) << "," << std::get<1>(routingList[currentIdIndex]) << std::endl;
        if (currentId == routingList[currentIdIndex]) // 如果是刚刚的Road
        {
            nextIndex = currentIdIndex + 1;
            std::cout << "if (currentId == routingList[currentIdIndex]) " << std::endl;
        }
        else if (currentId == routingList[currentIdIndex + 1]) // 如果换了下一个<Road,Lane>
        {
            std::cout << " else if (currentId == routingList[currentIdIndex + 1]) " << std::endl;
            currentIdIndex = currentIdIndex + 1;  // 当前Road更新为新的
            if (currentIdIndex >= pathMaxIdIndex) // 如果超出最大的Road数了（即走完了所有路）
            {
                nextIndex = MINIMAL_ID;
            }
            else
            {
                nextIndex = currentIdIndex + 1;
            }
        }
        else // 如果既不在刚刚的Road，又不在下一个Road（即出问题了）
        {
            std::cout << " else  " << std::endl;
            for (int i = 0; i <= pathMaxIdIndex; i++)
            {
                if (currentId == routingList[i]) // 如果<RoadId,LaneId>相同，那么就认为是这个<Road,Lane>
                {
                    currentIdIndex = i;
                    if (currentIdIndex == pathMaxIdIndex)
                    {
                        nextIndex = MINIMAL_ID;
                    }
                    else
                    {
                        nextIndex = currentIdIndex + 1;
                        std::cout << " nextIndex = currentIdIndex + 1/////////////////////////// " << std::endl;
                    }
                    break;
                }
                else
                    ;
            }
        }
    }
    else // 如果当前已经是最后一个Road
    {
        if (currentId == routingList[currentIdIndex]) // 如果真实道路即为最后一个
        {
            doNothing();
        }
        else // 如果真实道路不在最后一个（出问题了）
        {
            for (int i = 0; i <= pathMaxIdIndex; i++)
            {
                if (currentId == routingList[i])
                {
                    currentIdIndex = i;
                    break;
                }
            }
        }
        nextIndex = MINIMAL_ID;
    }

    for (int i = nextIndex; i < routingList.size(); i++)
    {
        std::cout << " routingList[nextIndex]" << std::get<0>(routingList[i]) << "," << std::get<1>(routingList[i]) << std::endl;
    }
    output.nextRoadLaneId = routingList[nextIndex];
    return;
}

/**
 * @brief 根据速度、曲率获取规划距离
 * @param[IN] param 无
 * @param[IN] input 速度、曲率
 * @param[OUT] output 规划距离
 
 * @cn_name: 计算规划距离
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getPlanningDistance(const getPlanningDistanceParam &param, const getPlanningDistanceInput &input, getPlanningDistanceOutput &output)
{
    double velocity = input.velocity;
    double curvature = input.curvature;
    double distance = DEFAULT_PLANNING_DISTANCE;
    static double lastPlanningDistance = DEFAULT_PLANNING_DISTANCE;

    double distanceCurvature = 0;
    curvature = std::fabs(curvature);
    double planningInner = 2;      // old 2.5
    double planningCurvature0 = 4; // old 15
    double planningCurvature5 = 0;

    if (curvature > 0.5)
    {
        distanceCurvature = (planningInner - planningCurvature5) * (5 - curvature) / 4.5 + planningCurvature5;
    }
    else if (curvature <= 0.5)
    {
        distanceCurvature = -(planningCurvature0 - planningInner) * curvature / 0.5 + planningCurvature0;
    }

    if (distanceCurvature < 0)
    {
        distanceCurvature = -2 > distanceCurvature ? 0 : distanceCurvature;
    }
    else
        ;
    // std::cout << "distanceCurvature" << distanceCurvature << "   curvature" << curvature << RESET << std::endl;

    double distanceVelocity = PLANNING_VELOCITY_MULTIPLIER * velocity;
    if (distanceVelocity > MAX_PLANNING_DISTANCE)
    {
        distanceVelocity = MAX_PLANNING_DISTANCE;
    }
    else if (distanceVelocity < MIN_PLANNING_DISTANCE)
    {
        distanceVelocity = MIN_PLANNING_DISTANCE;
    }
    else
        ;

    distance = distanceCurvature + distanceVelocity;

    limitPlanningDistanceParam paramLPD{};
    limitPlanningDistanceInput inputLPD{distance, lastPlanningDistance};
    limitPlanningDistanceOutput outputLPD{0};

    limitPlanningDistance(paramLPD, inputLPD, outputLPD);
    distance = outputLPD.targetDistance;
    // distance = limitPlanningDistance(distance, lastPlanningDistance);
    lastPlanningDistance = distance;
    output.distance = distance;
    return;
}

/**
 * @brief 根据车辆信息获取规划预瞄点
 * @param[IN] param 当前位置，决策数据，地图
 * @param[IN] input 规划距离，车辆朝向角，后续车道Id
 * @param[OUT] output 规划预瞄点
 
 * @cn_name: 计算规划预瞄点
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getPlanningPoint(const getPlanningPointParam &param, const getPlanningPointInput &input, getPlanningPointOutput &output)
{
    double distance = input.distance;
    GaussRoadPoint currentPoint = input.currentPoint;
    DecisionData decisionData = input.decisionData;
    double yaw = input.yaw;
    std::tuple<int32_t, int32_t> nextId = input.nextId;
    RoadMap map = param.map;

    bool b_findPlanningPoint = false;
    GaussRoadPoint planningPointCandidate;
    // std::cout << BOLDBLUE << "Planning point distance: " << distance << std::endl;
    double tempCandidateDistanceDiff = DISTANCE_RANGE; // 选择预瞄距离最合适的预瞄点

    fixRoadLaneIndexParam paramFRLI{map};
    fixRoadLaneIndexInput inputFRLI{decisionData};
    fixRoadLaneIndexOutput outputFRLI{std::tuple<int32_t, int32_t>(0, 0)};
    fixRoadLaneIndex(paramFRLI, inputFRLI, outputFRLI);

    std::tuple<int32_t, int32_t> fixedRoadLaneIndex = outputFRLI.indexTuple;

    // std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
    int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
    int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);

    for (int32_t index = decisionData.currentIndex; index < map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.size(); index++)
    {
        GaussRoadPoint thisPoint = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[index];

        pointDistanceParam paramPD{};
        pointDistanceInput inputPD{currentPoint.GaussX, thisPoint.GaussX, currentPoint.GaussY, thisPoint.GaussY};
        pointDistanceOutput outputPD{0};
        pointDistance(paramPD, inputPD, outputPD);

        double distanceFound = outputPD.distance;
        if (distanceFound > distance - DISTANCE_RANGE && distanceFound < distance + DISTANCE_RANGE)
        {
            getAngleParam paramGA{};
            getAngleInput inputGA{currentPoint.GaussX, currentPoint.GaussY, thisPoint.GaussX, thisPoint.GaussY};
            getAngleOutput outputGA{0};
            getAngle(paramGA, inputGA, outputGA);

            double planningAngle = outputGA.ang;

            getAngleDiffParam paramGAD{};
            getAngleDiffInput inputGAD{yaw, planningAngle};
            getAngleDiffOutput outputGAD{0};
            getAngleDiff(paramGAD, inputGAD, outputGAD);

            double angleDiff = outputGAD.ret;
            //      170412 Qing
            if (std::abs(angleDiff) < 90)
            {
                if (std::abs(distanceFound - distance) < tempCandidateDistanceDiff)
                {
                    tempCandidateDistanceDiff = std::abs(distanceFound - distance);
                    planningPointCandidate = thisPoint;
                }
                b_findPlanningPoint = true;
            }
            else
                ;
        }
        else
            ;
    }

    //  always check next segment
    if (true)
    {
        // 找对应的index, for next road and lane
        for (int i = 0; i < (int)map.roads.size(); i++)
        {
            if (std::get<0>(nextId) == map.roads[i].id)
            {
                roadIndex = i;
                break;
            }
        }
        for (int i = 0; i < (int)map.roads[roadIndex].lanes.size(); i++)
        {
            if (std::get<1>(nextId) == map.roads[laneIndex].lanes[i].id)
            {
                laneIndex = i;
                break;
            }
        }

        // std::cout << "check next segment: " << std::get<0>(nextId) << ";" << std::get<1>(nextId) << std::endl;
        for (int32_t index = 0; index < map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.size(); index++)
        {
            GaussRoadPoint thisPoint = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[index];
            // std::cout << "check next segment first point: "<< thisPoint.GaussX << ";" <<thisPoint.GaussY << std::endl;
            pointDistanceParam paramPD{};
            pointDistanceInput inputPD{currentPoint.GaussX, thisPoint.GaussX, currentPoint.GaussY, thisPoint.GaussY};
            pointDistanceOutput outputPD{0};
            pointDistance(paramPD, inputPD, outputPD);

            double distanceFound = outputPD.distance;
            // std::cout << "check next segment distance found: "<< distanceFound << std::endl;
            if (distanceFound > distance - DISTANCE_RANGE && distanceFound < distance + DISTANCE_RANGE)
            {
                getAngleParam paramGA{};
                getAngleInput inputGA{currentPoint.GaussX, currentPoint.GaussY, thisPoint.GaussX, thisPoint.GaussY};
                getAngleOutput outputGA{0};
                getAngle(paramGA, inputGA, outputGA);

                double planningAngle = outputGA.ang;

                getAngleDiffParam paramGAD{};
                getAngleDiffInput inputGAD{yaw, planningAngle};
                getAngleDiffOutput outputGAD{0};
                getAngleDiff(paramGAD, inputGAD, outputGAD);

                double angleDiff = outputGAD.ret;

                if (std::abs(angleDiff) < 100)
                {
                    if (std::abs(distanceFound - distance) < tempCandidateDistanceDiff)
                    {
                        tempCandidateDistanceDiff = std::abs(distanceFound - distance);
                        planningPointCandidate = thisPoint;
                    }
                    b_findPlanningPoint = true;
                }
            }
            else
                ;
        }
    }
    else
        ;

    GaussRoadPoint ret;
    if (true == b_findPlanningPoint)
    {
        ret = planningPointCandidate;
        // std::cout << std::setprecision(10) << "Planning Point xy: " << planningPointCandidate.GaussX << ";" << planningPointCandidate.GaussY << RESET << std::endl;
    }
    else
    {
        ret = currentPoint;
        std::cout << "NO PLANNING POINT FOUND!" << RESET << std::endl;
    }
    output.ret = ret;
    return;
}

/**
 * @brief 限制规划距离，防止突变
 * @param[IN] param 无
 * @param[IN] input 计划规划距离，上次规划距离
 * @param[OUT] output 更新后的规划距离
 
 * @cn_name: 限制规划距离
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void limitPlanningDistance(const limitPlanningDistanceParam &param, const limitPlanningDistanceInput &input, limitPlanningDistanceOutput &output)
{
    // std::cout << BOLDYELLOW << "Target Planning Distan: " << targetDistance << std::endl;
    // std::cout << "Last Planning Distance: " << lastTargetDistance << std::endl;
    double targetDistance = input.targetDistance;
    double lastTargetDistance = input.lastTargetDistance;

    if (targetDistance > lastTargetDistance + 2)
    {
        targetDistance = lastTargetDistance + 2;
    }
    else if (targetDistance < lastTargetDistance - 2)
    {
        targetDistance = lastTargetDistance - 2;
    }
    output.targetDistance = targetDistance;
    return;
}

/**
 * @brief 计算坐标系中两点角度
 * @param[IN] param 无
 * @param[IN] input 两点XY坐标
 * @param[OUT] output 两点角度
 
 * @cn_name: 计算两点角度
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getAngle(const getAngleParam &param, const getAngleInput &input, getAngleOutput &output)
{
    double x0 = input.x0;
    double y0 = input.y0;
    double x1 = input.x1;
    double y1 = input.y1;

    double deltaY = y1 - y0;
    double deltaX = x1 - x0;

    output.ang = std::atan2(deltaY, deltaX) / M_PI * 180;
    return;
}

// range is : ang0: Yaw : 0 ~ 360; ang1: Planning Angle zhijiaozuobiao : -180 ~ 180
/**
 * @brief 计算两角度之差，并正则化
 * @param[IN] param 无
 * @param[IN] input 两角度
 * @param[OUT] output 两角度之差
 
 * @cn_name: 计算两角度之差
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getAngleDiff(const getAngleDiffParam &param, const getAngleDiffInput &input, getAngleDiffOutput &output)
{
    double ang0 = input.ang0;
    double ang1 = input.ang1;

    if (ang0 > 180)
    {
        ang0 = ang0 - 360;
    }
    else
        ;

    double ret = ang1 - ang0;
    if (ret > 180)
    {
        ret = ret - 360;
    }
    else if (ret < -180)
    {
        ret = ret + 360;
    }
    else
        ;
    output.ret = ret;
    return;
}

// 在这个函数中完成了规划过程,将结果（规划曲线的x，y，angle，以及速度文件）存在decisiondata的trajectorylist中
void localPlanning(const pc::Imu &imu, double velocity, const RoadMap &map, DecisionData &decisionData, const prediction::ObjectList &predictionMsg,
                   const std::vector<std::tuple<int32_t, int32_t>> &routingList, const std::vector<GaussRoadPoint> stopPoints, const infopack::TrafficLight &trafficLight,
                   std::vector<infopack::ObjectsProto> objectsCmd)
{
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << RED << "1111111111111111111111111111111111111111111111duration proces: " << duration.count() << RESET << std::endl;

    initLocalPlanningParam paramILP{};
    initLocalPlanningInput inputILP{decisionData};
    initLocalPlanningOutput outputILP{decisionData};

    initLocalPlanning(paramILP, inputILP, outputILP);

    decisionData = outputILP.decisionData;

    // initLocalPlanning(decisionData);

    // imu定位确定的当前全局坐标
    GaussRoadPoint locationPoint;
    locationPoint.GaussX = imu.gaussx();
    locationPoint.GaussY = imu.gaussy();
    locationPoint.yaw = imu.yaw();
    std::cout << GREEN << std::setprecision(10) << "locationPoint: " << locationPoint.GaussX << ";" << locationPoint.GaussY << ";" << locationPoint.yaw << RESET << std::endl;

    getNextRoadLaneIdParam paramGNRLI{routingList};
    getNextRoadLaneIdInput inputGNRLI{std::tuple<int32_t, int32_t>(decisionData.currentId, decisionData.currentLaneId)};
    getNextRoadLaneIdOutput outputGNRLI{std::tuple<int32_t, int32_t>(decisionData.currentId, decisionData.currentLaneId)};

    getNextRoadLaneId(paramGNRLI, inputGNRLI, outputGNRLI);

    std::tuple<int32_t, int32_t> nextId = outputGNRLI.nextRoadLaneId;

    // std::tuple<int32_t, int32_t> nextId =
    //     getNextRoadLaneId(std::tuple<int32_t, int32_t>(decisionData.currentId, decisionData.currentLaneId), routingList); // 下个路段序号？？？这里只能处理后续一段路，应该考虑如果需要后续多段路时
    decisionData.nextId = nextId;

    getCurrentPositionParam paramGCP{imu, map};
    getCurrentPositionInput inputGCP{decisionData};
    getCurrentPositionOutput outputGCP{decisionData, true};

    getCurrentPosition(paramGCP, inputGCP, outputGCP);

    decisionData = outputGCP.decisionData;

    restartParam paramR{predictionMsg};
    restartInput inputR{velocity};
    restartOutput outputR{true};

    restart(paramR, inputR, outputR);

    if (!outputGCP.flag) // 获取当前位置最近路点
    {
        // 如果没有获取到路点
        std::cout << RED << "get current position failed :(" << RESET << std::endl;
        stopParam paramS{};
        stopInput inputS{decisionData, velocity};
        stopOutput outputS{decisionData};

        stop(paramS, inputS, outputS);
        decisionData = outputS.decisionData;
        // stop(decisionData, velocity); // 停车
    }
    else if (!outputR.flag)
    {
        std::cout << RED << "restart failed :(" << RESET << std::endl;
        stopParam paramS{};
        stopInput inputS{decisionData, velocity};
        stopOutput outputS{decisionData};

        stop(paramS, inputS, outputS);
        decisionData = outputS.decisionData;
        // stop(decisionData, velocity); // 停车
    }
    else
    {
        // std::cout << RED << "get current position success:(" << RESET << std::endl;
        //  根据地图信息，找对应的index

        // std::cout << "test for map initial 111111: "<< map.roads[2].lanes[0].id<< std::endl;
        //   std::cout << "test for map initial 111111: "<< map.roads[2].lanes[1].id<< std::endl;

        fixRoadLaneIndexParam paramFRLI{map};
        fixRoadLaneIndexInput inputFRLI{decisionData};
        fixRoadLaneIndexOutput outputFRLI{std::tuple<int32_t, int32_t>(0, 0)};

        fixRoadLaneIndex(paramFRLI, inputFRLI, outputFRLI);
        std::tuple<int32_t, int32_t> fixedRoadLaneIndex = outputFRLI.indexTuple;

        // std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
        int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
        int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);
        // std::cout << "decision data current ID*************: " << decisionData.currentId << ";" << decisionData.currentLaneId << std::endl;
        // std::cout << "Road Lane ID in local planning: " << roadIndex << "; " << laneIndex << std::endl;
        GaussRoadPoint currentPoint = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[decisionData.currentIndex];
        // std::cout << GREEN << std::setprecision(10) << "currentPoint: " << currentPoint.GaussX << ";" << currentPoint.GaussY << ";" << currentPoint.yaw << RESET << std::endl;
        // std::cout << YELLOW << "routing list size: " << routingList.size() << RESET << std::endl;

        std::cout << "decision data current ID: " << decisionData.currentId << ";" << decisionData.currentLaneId << std::endl;
        // std::cout << "decision data next ID: " << std::get<0>(nextId) << ";" << std::get<1>(nextId) << std::endl;

        // 先判断全局规划是否要求换道/以及是否能换道
        //  active lane change
        int tempCurrentLaneId, tempCurrentIndex;
        int currentLaneId = decisionData.currentLaneId;
        int currentIndex = decisionData.currentIndex;

        // 全局规划中，要求换道
        if (decisionData.currentId == std::get<0>(decisionData.nextId) && decisionData.currentLaneId != std::get<1>(decisionData.nextId))
        {
            // std::cout << RED << "enter active lane change!" << RESET << std::endl;
            bool isLeft = std::get<1>(decisionData.nextId) > decisionData.currentLaneId;

            changeCurrentPointParam paramCCP{map.roads[roadIndex], map.roads[roadIndex].lanes[laneIndex]};
            changeCurrentPointInput inputCCP{tempCurrentLaneId, tempCurrentIndex, currentPoint, isLeft};
            changeCurrentPointOutput outputCCP{tempCurrentLaneId, tempCurrentIndex, currentPoint, true};
            changeCurrentPoint(paramCCP, inputCCP, outputCCP);

            tempCurrentLaneId = outputCCP.laneId;
            tempCurrentIndex = outputCCP.pointIndex;
            currentPoint = outputCCP.currentPoint;

            if (outputCCP.flag)
            {
                decisionData.currentLaneId = tempCurrentLaneId;
                decisionData.currentIndex = tempCurrentIndex;
                // std::cout << BOLDCYAN << "after changeCurrentPoint: " << decisionData.currentLaneId << "; " << decisionData.currentIndex << std::endl;

                // 生成referenceline，referencelane是在道路上的点集，起点是当前位置最近的道路上的点
                ReferenceLine referenceLine;

                getReferenceLineParam paramGRL{map};
                getReferenceLineInput inputGRL{decisionData, locationPoint};
                getReferenceLineOutput outputGRL{referenceLine};

                getReferenceLine(paramGRL, inputGRL, outputGRL);
                // ReferenceLine referenceLine = getReferenceLine(map, decisionData, locationPoint);
                referenceLine = outputGRL.referenceLine;

                // 计算当前位置在frenet坐标系中坐标
                getFrenetLocationPointParam paramGFLP{};
                getFrenetLocationPointInput inputGFLP{decisionData, referenceLine, locationPoint};
                getFrenetLocationPointOutput outputGFLP{decisionData};

                getFrenetLocationPoint(paramGFLP, inputGFLP, outputGFLP);
                decisionData = outputGFLP.decisionData;
                // getFrenetLocationPoint(decisionData, referenceLine, locationPoint);

                getTrajectoryPlanningResultParam paramGTPR{imu, predictionMsg, map, trafficLight, objectsCmd};
                getTrajectoryPlanningResultInput inputGTPR{velocity, decisionData, referenceLine};
                getTrajectoryPlanningResultOutput outputGTPR{decisionData, 0};

                getTrajectoryPlanningResult(paramGTPR, inputGTPR, outputGTPR);
                decisionData = outputGTPR.decisionData;
                decisionData.optimalTrajectoryIndex = outputGTPR.index;

                // decisionData.optimalTrajectoryIndex = getTrajectoryPlanningResult(velocity, decisionData, imu, predictionMsg, map, referenceLine, trafficLight, objectsCmd); // 20220825 修改函数输入
                std::cout << RED << "optimalTrajectoryIndex in active lane change:" << decisionData.optimalTrajectoryIndex << RESET << std::endl;
                if (decisionData.optimalTrajectoryIndex != -1)
                {
                    PlanningTrajectory optimalGlobalTrajectory;
                    changeLocalToGlobalParam paramCLTG{};
                    changeLocalToGlobalInput inputCLTG{decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint};
                    changeLocalToGlobalOutput outputCLTG{optimalGlobalTrajectory};

                    changeLocalToGlobal(paramCLTG, inputCLTG, outputCLTG);
                    optimalGlobalTrajectory = outputCLTG.globalTrajectory;
                    // PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);

                    decisionData.optimalGlobalTrajectory = optimalGlobalTrajectory;
                    return;
                }
                else
                {
                    decisionData.currentLaneId = currentLaneId;
                    decisionData.currentIndex = currentIndex;
                }
            }
            tempCurrentLaneId = currentLaneId;
            tempCurrentIndex = currentIndex;
        }

        // 全局规划没有要求换道，在当前道路上继续前行

        getCurrentPositionParam paramGCP{imu, map};
        getCurrentPositionInput inputGCP{decisionData};
        getCurrentPositionOutput outputGCP{decisionData, true};

        getCurrentPosition(paramGCP, inputGCP, outputGCP);

        decisionData = outputGCP.decisionData;

        // getCurrentPosition(decisionData, imu, map);
        // 生成参考线
        ReferenceLine referenceLine;
        getReferenceLineParam paramGRL{map};
        getReferenceLineInput inputGRL{decisionData, locationPoint};
        getReferenceLineOutput outputGRL{referenceLine};

        getReferenceLine(paramGRL, inputGRL, outputGRL);
        // ReferenceLine referenceLine = getReferenceLine(map, decisionData, locationPoint);
        referenceLine = outputGRL.referenceLine;

        std::cout << RED << "referenceLine!!!!2222222222222222222222222!: " << referenceLine.referenceLinePoints.back().s << "; " << referenceLine.referenceLinePoints.back().l << "; "
                  << referenceLine.referenceLinePoints.back().s << RESET << std::endl;

        getFrenetLocationPointParam paramGFLP{};
        getFrenetLocationPointInput inputGFLP{decisionData, referenceLine, locationPoint};
        getFrenetLocationPointOutput outputGFLP{decisionData};

        getFrenetLocationPoint(paramGFLP, inputGFLP, outputGFLP);
        decisionData = outputGFLP.decisionData;

        // getFrenetLocationPoint(decisionData, referenceLine, locationPoint);
        // std::cout << RED << "3333333333333333333333333!" << RESET << std::endl;
        // auto start = std::chrono::steady_clock::now();

        getTrajectoryPlanningResultParam paramGTPR{imu, predictionMsg, map, trafficLight, objectsCmd};
        getTrajectoryPlanningResultInput inputGTPR{velocity, decisionData, referenceLine};
        getTrajectoryPlanningResultOutput outputGTPR{decisionData, 0};

        getTrajectoryPlanningResult(paramGTPR, inputGTPR, outputGTPR);
        decisionData = outputGTPR.decisionData;
        decisionData.optimalTrajectoryIndex = outputGTPR.index;

        //  decisionData.optimalTrajectoryIndex = getTrajectoryPlanningResult(velocity, decisionData, imu, predictionMsg, map, referenceLine, trafficLight,
        //                                                                   objectsCmd); // 20220825 修改函数输入
        //                                                                                // std::cout << RED << "optimalTrajectoryIndex for no lane changing:" << decisionData.optimalTrajectoryIndex
        //                                                                                <<
        //                                                                                // RESET << std::endl; std::cout << "total path number" << decisionData.finalPathList.size() << std::endl;
        //                                                                                // std::cout << "total trajectory number" << decisionData.controlTrajectoryList.size() << std::endl;

        end = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // std::cout << RED << "22222222222222222222222222222222duration proces: " << duration.count() << RESET << std::endl;
        //  std::cout << RED << "444444444444444444444444444!   " << duration.count() << RESET << std::endl;
        //   如果直行道没有可行道路，那么考虑左右换道
        //    passive lane change
        if (decisionData.optimalTrajectoryIndex == -1)
        {
            getCurrentPositionParam paramGCP{imu, map};
            getCurrentPositionInput inputGCP{decisionData};
            getCurrentPositionOutput outputGCP{decisionData, true};

            getCurrentPosition(paramGCP, inputGCP, outputGCP);

            decisionData = outputGCP.decisionData;
            // getCurrentPosition(decisionData, imu, map);
            std::cout << RED << "enter passive lane change!" << RESET << std::endl;

            fixRoadLaneIndexParam paramFRLI{map};
            fixRoadLaneIndexInput inputFRLI{decisionData};
            fixRoadLaneIndexOutput outputFRLI{std::tuple<int32_t, int32_t>(0, 0)};

            fixRoadLaneIndex(paramFRLI, inputFRLI, outputFRLI);
            std::tuple<int32_t, int32_t> fixRoadLaneIndex = outputFRLI.indexTuple;

            // std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
            int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
            int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);

            for (int i = 0; i <= 1; i++)
            // for (int i = 0; i <= 0; i++) // only change left lane
            {
                //?????? 最后一个变量 i ^ i 永远是0吧，应该只能右换道

                changeCurrentPointParam paramCCP{map.roads[roadIndex], map.roads[roadIndex].lanes[laneIndex]};
                changeCurrentPointInput inputCCP{tempCurrentLaneId, tempCurrentIndex, currentPoint, (bool)i};
                changeCurrentPointOutput outputCCP{tempCurrentLaneId, tempCurrentIndex, currentPoint, true};

                changeCurrentPoint(paramCCP, inputCCP, outputCCP);
                tempCurrentLaneId = outputCCP.laneId;
                tempCurrentIndex = outputCCP.pointIndex;
                currentPoint = outputCCP.currentPoint;

                if (!outputCCP.flag)
                {
                    continue;
                }
                decisionData.currentLaneId = tempCurrentLaneId;
                decisionData.currentIndex = tempCurrentIndex;

                getReferenceLineParam paramGRL{map};
                getReferenceLineInput inputGRL{decisionData, locationPoint};
                getReferenceLineOutput outputGRL{referenceLine};

                getReferenceLine(paramGRL, inputGRL, outputGRL);
                referenceLine = outputGRL.referenceLine;

                // ReferenceLine referenceLine = getReferenceLine(map, decisionData, locationPoint);

                getFrenetLocationPointParam paramGFLP{};
                getFrenetLocationPointInput inputGFLP{decisionData, referenceLine, locationPoint};
                getFrenetLocationPointOutput outputGFLP{decisionData};

                getFrenetLocationPoint(paramGFLP, inputGFLP, outputGFLP);
                decisionData = outputGFLP.decisionData;

                // getFrenetLocationPoint(decisionData, referenceLine, locationPoint);
                // std::cout << RED << "3333333333333333333333333!" << RESET << std::endl;
                // auto start = std::chrono::steady_clock::now();

                getTrajectoryPlanningResultParam paramGTPR{imu, predictionMsg, map, trafficLight, objectsCmd};
                getTrajectoryPlanningResultInput inputGTPR{velocity, decisionData, referenceLine};
                getTrajectoryPlanningResultOutput outputGTPR{decisionData, 0};

                getTrajectoryPlanningResult(paramGTPR, inputGTPR, outputGTPR);
                decisionData = outputGTPR.decisionData;
                decisionData.optimalTrajectoryIndex = outputGTPR.index;

                // decisionData.optimalTrajectoryIndex = getTrajectoryPlanningResult(velocity, decisionData, imu, predictionMsg, map, referenceLine, trafficLight, objectsCmd); // 20220825 修改函数输入
                // std::cout << RED << "optimal trajectory index after changing planning point: " << decisionData.optimalTrajectoryIndex << RESET << std::endl;
                if (decisionData.optimalTrajectoryIndex != -1)
                {
                    break;
                }
                else
                {
                    decisionData.currentLaneId = currentLaneId;
                    decisionData.currentIndex = currentIndex;
                }
            }
            tempCurrentLaneId = currentLaneId;
            tempCurrentIndex = currentIndex;
        }
    }
    // 如果没找到可行轨迹，则停车
    if (decisionData.optimalTrajectoryIndex == -1)
    {
        std::cout << RED << "NO WAY!" << RESET << std::endl;
        decisionData.optimalTrajectoryIndex = 0;

        stopParam paramS{};
        stopInput inputS{decisionData, velocity};
        stopOutput outputS{decisionData};

        stop(paramS, inputS, outputS);
        decisionData = outputS.decisionData;
        // stop(decisionData, velocity);
    }
    else
    {
        // 计算曲率及弯道限速
        int turnIndex = -1;
        for (int iter = 2; iter < decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() - 2; iter++)
        {
            calculateCurvatureParam paramCC{};
            calculateCurvatureInput inputCC{decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter],
                                            decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2],
                                            decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2]};
            calculateCurvatureOutput outputCC{0};

            calculateCurvature(paramCC, inputCC, outputCC);
            double curvatureTemp = outputCC.curvature;

            // double curvatureTemp = fabs(calculateCurvature(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter],
            //                                                decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2],
            //                                                decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2]));
            // // std::cout <<  RED<<"?????????????????????????????????????????curvatureTemp = ????????????????????" <<        curvatureTemp <<std::endl;
            if (curvatureTemp > CURVATURE_THRESHOLS) // 曲率
            {
                //  std::cout<<"data"<<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter] .gaussX<< ","
                //                           <<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter] .gaussY<< ";"
                //                           << decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2].gaussX<< ","
                //                           << decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2].gaussY<< ";"
                //                          <<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2] .gaussX<< ","
                //                          <<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2] .gaussY<< ","<< std::endl;
                turnIndex = iter;
                break;
            }
        }
        if (turnIndex >= 0)
        {
            for (int iter = 0; iter < decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size(); iter++)
            {
                decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter].v =
                    std::min(CURVATURE_SPEED, decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter].v);
            }
        }
    }

    // 如果到了stopPoint,就停车
    stopPointJudgeParam paramSPJ{imu, stopPoints};
    stopPointJudgeInput inputSPJ{};
    stopPointJudgeOutput outputSPJ{true};

    stopPointJudge(paramSPJ, inputSPJ, outputSPJ);

    if (outputSPJ.flag)
    {
        std::cout << "stopPointJudge(imu, stopPoints))-------------------" << std::endl;
        stopParam paramS{};
        stopInput inputS{decisionData, velocity};
        stopOutput outputS{decisionData};

        stop(paramS, inputS, outputS);
        decisionData = outputS.decisionData;
        // stop(decisionData, velocity);
    }
    PlanningTrajectory optimalGlobalTrajectory;

    changeLocalToGlobalParam paramCLTG{};
    changeLocalToGlobalInput inputCLTG{decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint};
    changeLocalToGlobalOutput outputCLTG{optimalGlobalTrajectory};

    changeLocalToGlobal(paramCLTG, inputCLTG, outputCLTG);
    optimalGlobalTrajectory = outputCLTG.globalTrajectory;
    // PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);
    decisionData.optimalGlobalTrajectory = optimalGlobalTrajectory;

    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() << std::endl;
    //  std::cout << RED << "eeeeeeeeeeeeeeeeeeeeeeee   1duration proces: " << duration.count() << RESET << std::endl;

    return;
}

/**
 * @brief 车辆停车后，判断能否重新启动
 * @param[IN] param 栅格障碍物
 * @param[IN] input 车速
 * @param[OUT] output 判断结果
 
 * @cn_name: 判断车辆能否重新启动
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void restart(const restartParam &param, const restartInput &input, restartOutput &output)
{
    prediction::ObjectList prediction = param.prediction;
    double velocity = input.velocity;
    output.flag = true;
    // 这个函数导致避障后不能启动，暂时不用了
    return;

    bool restartflag = true;
    if (velocity < 0.1)
    {
        for (auto object : prediction.object())
        {
            if (object.predictpoint(0).y() > -1)
            {
                if (fabs(object.predictpoint(0).x()) < 1 && object.predictpoint(0).y() < 15)
                {
                    restartflag = false;
                }
                if (fabs(object.predictpoint(0).x()) < 2 && object.predictpoint(0).y() < 10)
                {
                    restartflag = false;
                }
                if (fabs(object.predictpoint(0).x()) < 4 && object.predictpoint(0).y() < 5)
                {
                    restartflag = false;
                }
            }
        }
    }
    output.flag = restartflag;
    return;
}

// 停车
/**
 * @brief 停车时修改速度曲线
 * @param[IN] param 无
 * @param[IN] input 决策数据，当前车速
 * @param[OUT] output 更新的决策数据
 
 * @cn_name: 处理停车速度
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void stop(const stopParam &param, const stopInput &input, stopOutput &output)
{
    DecisionData decisionData = input.decisionData;
    double velocity = input.velocity;

    if (decisionData.optimalTrajectoryIndex == -1) // 如果没有选到曲线，就认为选0号并且速度赋为0
    {
        decisionData.optimalTrajectoryIndex = 0;
    }
    int stopIndex = (int)(1.25 * velocity * velocity);
    if (stopIndex > CURVE_POINT_NUM / 2)
    {
        stopIndex = CURVE_POINT_NUM / 2;
    }
    if (stopIndex <= 0)
    {
        stopIndex = 1;
    }

    std::vector<double> curvePointSpeed;
    interp_m::interpLinearParam paramIL{};
    interp_m::interpLinearInput inputIL{velocity, 0, stopIndex + 1};
    interp_m::interpLinearOutput outputIL{curvePointSpeed};

    interp_m::linear(paramIL, inputIL, outputIL);
    curvePointSpeed = outputIL.result;

    for (int i = 0; i <= stopIndex - 1; i++)
    {

        decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[i].v = 0;
    }

    for (int i = stopIndex; i < CURVE_POINT_NUM; i++)
    {
        decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[i].v = 0;
    }

    output.decisionData = decisionData;
}

/**
 * @brief 判断车辆是否在停车区域
 * @param[IN] param imu，停止点
 * @param[IN] input 无
 * @param[OUT] output 判断结果
 
 * @cn_name: 判断停车
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void stopPointJudge(const stopPointJudgeParam &param, const stopPointJudgeInput &input, stopPointJudgeOutput &output)
{
    pc::Imu imu = param.imu;
    std::vector<GaussRoadPoint> stopPoints = param.stopPoints;
    bool ret = false;
    for (uint32_t i = 1; i < stopPoints.size(); i++)
    {
        // ting zai hou mian.
        inAreaParam paramIA{};
        inAreaInput inputIA{imu.gaussx(), imu.gaussy(), stopPoints[i].GaussX, stopPoints[i].GaussY};
        inAreaOutput outputIA{true};
        inArea(paramIA, inputIA, outputIA);

        if (outputIA.flag)
        {
            ret = true;
        }
    }
    output.flag = ret;
    return;
}

/**
 * @brief 计算贝塞尔曲线上的点
 * @param[IN] param 无
 * @param[IN] input 贝塞尔曲线控制点，参数
 * @param[OUT] output 判断结果
 
 * @cn_name: 计算贝塞尔曲线上的点
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void pointOnCubicBezier(const pointOnCubicBezierParam &param, const pointOnCubicBezierInput &input, pointOnCubicBezierOutput &output)
{
    /*
     cp在此是四個元素的陣列:
     cp[0]為起始點，或上圖中的P0
     cp[1]為第一個控制點，或上圖中的P1
     cp[2]為第二個控制點，或上圖中的P2
     cp[3]為結束點，或上圖中的P3
     t為參數值，0 <= t <= 1
     */
    std::vector<PlanningPoint> cp = input.cp;
    double t = input.t;

    double as, bs, cs;
    double al, bl, cl;
    double tSquared, tCubed;
    PlanningPoint result;

    /*計算多項式係數*/

    cs = 3.0 * (cp[1].s - cp[0].s);
    bs = 3.0 * (cp[2].s - cp[1].s) - cs;
    as = cp[3].s - cp[0].s - cs - bs;

    cl = 3.0 * (cp[1].l - cp[0].l);
    bl = 3.0 * (cp[2].l - cp[1].l) - cl;
    al = cp[3].l - cp[0].l - cl - bl;

    /*計算位於參數值t的曲線點*/

    tSquared = t * t;
    tCubed = tSquared * t;

    // std::cout << "wwwwwwwwwwww" <<std::endl;

    result.s = (as * tCubed) + (bs * tSquared) + (cs * t) + cp[0].s;
    result.l = (al * tCubed) + (bl * tSquared) + (cl * t) + cp[0].l;
    output.result = result;
    return;
}

// 在这个函数中，根据起点和终点，完成了一条曲线的一段的生成，
/**
 * @brief 在frenet中生成贝塞尔曲线
 * @param[IN] param 无
 * @param[IN] input 起点、终点、初始化轨迹
 * @param[OUT] output 生成轨迹
 
 * @cn_name: 生成贝塞尔曲线
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void generateBezierPathInFrenet(const generateBezierPathInFrenetParam &param, const generateBezierPathInFrenetInput &input, generateBezierPathInFrenetOutput &output)
{
    PlanningPoint startPoint = input.startPoint;
    PlanningPoint endPoint = input.endPoint;
    PlanningTrajectory curve = input.curve;

    double ds = endPoint.s - startPoint.s;
    double dl = endPoint.l - startPoint.l;
    // double distance = sqrt(pow(ds, 2) + pow(dl, 2));
    double distance = sqrt(ds * ds + dl * dl);

    PlanningPoint controlPoint[4] = {0}; // 控制点是从车辆当前坐标开始，相当于frenet坐标系做了一个平移

    controlPoint[0].s = 0.0;
    controlPoint[0].l = 0.0;

    controlPoint[0].frenetAngle = static_cast<int32_t>(360 + startPoint.frenetAngle) % 360;

    controlPoint[3].s = ds;
    controlPoint[3].l = dl;
    // std::cout << "control point 4: "<< ds <<"; " << dl << std::endl;

    controlPoint[3].frenetAngle = static_cast<int32_t>(360 + endPoint.frenetAngle) % 360;
    // std::cout << CYAN << endPoint.frenetAngle << " ; " << startPoint.frenetAngle << " ; " << controlPoint[3].frenetAngle << std::endl;

    double controlPointDistance;
    // 生成从起点到第一段终点

    controlPointDistance = CP_DISTANCE * distance / 10; // 中间2点分别与起点和终点的距离，

    controlPoint[1].s = controlPointDistance * std::cos(startPoint.frenetAngle / 180 * M_PI);
    controlPoint[1].l = controlPointDistance * std::sin(startPoint.frenetAngle / 180 * M_PI);
    // std::cout << "control point 2: " << controlPoint[1].s << "; " << controlPoint[1].l << std::endl;
    controlPoint[1].frenetAngle = controlPoint[0].frenetAngle;

    controlPoint[2].s = controlPoint[3].s - controlPointDistance * std::cos(controlPoint[3].frenetAngle / 180 * M_PI);
    controlPoint[2].l = controlPoint[3].l - controlPointDistance * std::sin(controlPoint[3].frenetAngle / 180 * M_PI);
    // std::cout << "control point 3: " << controlPoint[2].s << "; " << controlPoint[2].l << std::endl;
    controlPoint[2].frenetAngle = controlPoint[3].frenetAngle;

    // std::cout << CYAN << controlPoint[2].x + startPoint.x << " ; " << controlPoint[2].y + startPoint.y << " ; " << controlPoint[3].angle << std::endl;
    // std::cout << CYAN << -controlPointDistance * std::sin(controlPoint[3].angle / 180 * M_PI) << " h " << -controlPointDistance * std::cos(controlPoint[3].angle / 180 * M_PI) << " h " <<
    // controlPoint[3].angle << std::endl;

    std::vector<PlanningPoint> controlPointList;
    controlPointList.clear();
    for (uint32_t i = 0; i < 4; i++)
    {
        controlPointList.push_back(controlPoint[i]);
    }

    // std::cout << "qqqqqqqqqqqqqqq " <<  std::endl;

    double dt = 1.0 / (CURVE_POINT_NUM - 1); // 步长
    for (uint32_t i = 0; i < CURVE_POINT_NUM; i++)
    {
        PlanningPoint bezierPoint, resultPoint;

        pointOnCubicBezierParam paramPCB{};
        pointOnCubicBezierInput inputPCB{controlPointList, i * dt};
        pointOnCubicBezierOutput outputPCB{bezierPoint};
        pointOnCubicBezier(paramPCB, inputPCB, outputPCB);

        bezierPoint = outputPCB.result; // 在这个函数里确定贝塞尔曲线每个点的位置
        // std::cout << "wwwwwwwwwwww" << std::endl;

        resultPoint.s = bezierPoint.s + startPoint.s; // 将曲线恢复到以路点为起点的frenet坐标系
        resultPoint.l = bezierPoint.l + startPoint.l;

        if (i == 0)
        {
            resultPoint.frenetAngle = startPoint.frenetAngle;
        }
        else
        {
            resultPoint.frenetAngle = static_cast<int32_t>(360 + atan2(resultPoint.s - curve.planningPoints[i - 1].s, resultPoint.l - curve.planningPoints[i - 1].l) / M_PI * 180) % 360;
        }
        curve.planningPoints.push_back(resultPoint);
        // std::cout << CYAN << "curve.points[i].angle" << curve.points[i].angle << std::endl;
    }
    output.curve = curve;
}

/**
 * @brief 清除上次生成的轨迹
 * @param[IN] param 无
 * @param[IN] input 储存轨迹的向量
 * @param[OUT] output 判断结果
 
 * @cn_name: 清除轨迹
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void clearPathList(const clearPathListParam &param, const clearPathListInput &input, clearPathListOutput &output)
{
    std::vector<PlanningTrajectory> pathList = input.pathList;
    pathList.clear();
    output.pathList = pathList;
}

// 在这个函数中，主要完成了多条轨迹的生成，并作出了轨迹的选择。
/**
 * @brief 完成了多条轨迹和速度曲线的生成，并转化到笛卡尔坐标系中，作出了轨迹的选择
 * @param[IN] param imu，栅格障碍物，地图，交通灯，路测障碍物
 * @param[IN] input 决策数据
 * @param[OUT] output 更新后的决策数据，是否成功生成轨迹
 
 * @cn_name: 获取轨迹规划结果
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getTrajectoryPlanningResult(const getTrajectoryPlanningResultParam &param, const getTrajectoryPlanningResultInput &input, getTrajectoryPlanningResultOutput &output)
{
    pc::Imu imu = param.imu;
    prediction::ObjectList predictionMsg = param.predictionMsg;
    RoadMap map = param.map;
    infopack::TrafficLight trafficLight = param.trafficLight;
    std::vector<infopack::ObjectsProto> objectsCmd = param.objectsCmd;

    double velocity = input.velocity;
    DecisionData decisionData = input.decisionData;
    ReferenceLine referenceLine = input.referenceLine;

    std::vector<PlanningTrajectory> frenetPathList; // frenet坐标系下规划路径

    generateBezierPathListInFrenetParam paramGBPLIF{};
    generateBezierPathListInFrenetInput inputGBPLIF{referenceLine, decisionData.frenetLocationPoint, frenetPathList};
    generateBezierPathListInFrenetOutput outputGBPLIF{frenetPathList};

    generateBezierPathListInFrenet(paramGBPLIF, inputGBPLIF, outputGBPLIF);
    frenetPathList = outputGBPLIF.pathList;
    // generateBezierPathListInFrenet(referenceLine, decisionData.frenetLocationPoint, frenetPathList);

    PlanningTrajectory tempPath;
    int prevIndex = 0, lastIndex = 0;

    for (int i = 0; i < CURVE_NUM; i++)
    {
        for (int j = 0; j < CURVE_POINT_NUM; j++)
        {
            frenet2CartesianParam paramF2C{};
            frenet2CartesianInput inputF2C{frenetPathList[i].planningPoints[j].s,
                                           frenetPathList[i].planningPoints[j].l,
                                           frenetPathList[i].planningPoints[j].x,
                                           frenetPathList[i].planningPoints[j].y,
                                           referenceLine,
                                           lastIndex,
                                           prevIndex};
            frenet2CartesianOutput outputF2C{frenetPathList[i].planningPoints[j].x, frenetPathList[i].planningPoints[j].y, lastIndex};

            frenet2Cartesian(paramF2C, inputF2C, outputF2C);

            frenetPathList[i].planningPoints[j].x = outputF2C.x;
            frenetPathList[i].planningPoints[j].y = outputF2C.y;
            lastIndex = outputF2C.lastIndex;

            tempPath.planningPoints.push_back(frenetPathList[i].planningPoints[j]);
        } // for (int j = 0; j < CURVE_POINT_NUM; j++)

        prevIndex = 0;
        lastIndex = 0;
        decisionData.finalPathList.push_back(tempPath);
        tempPath.planningPoints.clear();
    } // for (int i = 0; i < CURVE_NUM; i++)
    frenetPathList.clear();

    generateSpeedListParam paramGSL{objectsCmd, imu};
    generateSpeedListInput inputGSL{velocity, decisionData};
    generateSpeedListOutput outputGSL{decisionData};

    generateSpeedList(paramGSL, inputGSL, outputGSL);
    decisionData = outputGSL.decisionData;
    // generateSpeedList(velocity, decisionData, objectsCmd, imu);

    //  生成轨迹
    generateTrajectoryListParam paramGTL{};
    generateTrajectoryListInput inputGTL{decisionData};
    generateTrajectoryListOutput outputGTL{decisionData};

    generateTrajectoryList(paramGTL, inputGTL, outputGTL);
    decisionData = outputGTL.decisionData;
    // generateTrajectoryList(decisionData);

    // std::cout << RED << "777777777777777777777!" << RESET << std::endl;

    // 在规划的路径中选择一条最优路径
    getOptimalTrajectoryIndexParam paramGOTI{predictionMsg, trafficLight, imu, objectsCmd};
    getOptimalTrajectoryIndexInput inputGOTI{decisionData};
    getOptimalTrajectoryIndexOutput outputGOTI{decisionData, 0};

    getOptimalTrajectoryIndex(paramGOTI, inputGOTI, outputGOTI);

    int32_t optimalTrajectoryIndex = outputGOTI.index;
    decisionData = outputGOTI.decisionData;
    // int32_t optimalTrajectoryIndex = getOptimalTrajectoryIndex(decisionData, predictionMsg, trafficLight, objectsCmd, imu);
    // std::cout << "optimalTrajectoryIndex after getOptimalTrajectoryIndex: " << optimalTrajectoryIndex << std::endl;

    output.index = optimalTrajectoryIndex;
    output.decisionData = decisionData;
    return;
}

// 20220826 两点间距离公式
/**
 * @brief 计算两点之间的距离
 * @param[IN] param 无
 * @param[IN] input 两点XY坐标
 * @param[OUT] output 两点之间的距离
 
 * @cn_name: 计算两点之间的距离
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getDistance(const getDistanceParam &param, const getDistanceInput &input, getDistanceOutput &output)
{
    double x1 = input.x1;
    double y1 = input.y1;
    double x2 = input.x2;
    double y2 = input.y2;

    output.dis = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    return;
}

// 计算点到线段的距离，如果这个公式是对的，相比原来的方法就是降维打击
/**
 * @brief 计算点到线段的距离
 * @param[IN] param 无
 * @param[IN] input 线段起点，线段终点，检查点
 * @param[OUT] output 点到线段的距离
 
 * @cn_name: 计算点到线段的距离
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void pointToLineDistance(const pointToLineDistanceParam &param, const pointToLineDistanceInput &input, pointToLineDistanceOutput &output)
{
    GaussRoadPoint startPoint = input.startPoint;
    GaussRoadPoint stopPoint = input.stopPoint;
    GaussRoadPoint checkPoint = input.checkPoint;

    getDistanceParam param01{};
    getDistanceInput input01{startPoint.GaussX, startPoint.GaussY, stopPoint.GaussX, stopPoint.GaussY};
    getDistanceOutput output01{0};
    getDistance(param01, input01, output01);
    double lengthStart2Stop = output01.dis;

    getDistanceParam param02{};
    getDistanceInput input02{startPoint.GaussX, startPoint.GaussY, checkPoint.GaussX, checkPoint.GaussY};
    getDistanceOutput output02{0};
    getDistance(param02, input02, output02);
    double lengthStart2Check = output02.dis;

    getDistanceParam param03{};
    getDistanceInput input03{stopPoint.GaussX, stopPoint.GaussY, checkPoint.GaussX, checkPoint.GaussY};
    getDistanceOutput output03{0};
    getDistance(param03, input03, output03);
    double lengthStop2Check = output03.dis;

    // 垂足在线段外，距离点到线段端点的距离
    if (lengthStart2Check * lengthStart2Check > lengthStop2Check * lengthStop2Check + lengthStart2Stop * lengthStart2Stop)
    {
        output.dis = lengthStop2Check;
        return;
    }

    if (lengthStop2Check * lengthStop2Check > lengthStart2Check * lengthStart2Check + lengthStart2Stop * lengthStart2Stop)
    {
        output.dis = lengthStart2Check;
        return;
    }

    // 垂足在线段内  ，最短距离是三角形ABC以边BC的高，可通过海伦公式先求出面积，再求出高得到答案。
    double l = (lengthStart2Stop + lengthStart2Check + lengthStop2Check) / 2;
    double s = sqrt(l * (l - lengthStart2Stop) * (l - lengthStart2Check) * (l - lengthStop2Check));
    // std::cout<< "new pointToLineDistance++++++ "<< 2*s/lengthStart2Stop <<std:: endl;

    output.dis = 2 * s / lengthStart2Stop;
    return;
}

/**
 * @brief 转化角度到0-180中，便于比较
 * @param[IN] param 无
 * @param[IN] input 原角度
 * @param[OUT] output 转化后的角度
 
 * @cn_name: 正则化角度
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void angleRegulate(const angleRegulateParam &param, const angleRegulateInput &input, angleRegulateOutput &output)
{
    double angleInput = input.angleInput;
    if (angleInput > 180.0)
    {
        angleInput = 360.0 - angleInput;
    }
    output.angleOutput = angleInput;
    return;
}

// 车辆当前位置最近的路点，保存在 decisionData.currentId 、 decisionData.currentLaneId 、  decisionData.currentIndex 中
/**
 * @brief 寻找距离车辆位置最近的路点
 * @param[IN] param imu，地图
 * @param[IN] input 决策数据
 * @param[OUT] output 更新的决策树护具
 
 * @cn_name: 寻找距离车辆位置最近的路点
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getCurrentPosition(const getCurrentPositionParam &param, const getCurrentPositionInput &input, getCurrentPositionOutput &output)
{
    pc::Imu imu = param.imu;
    RoadMap map = param.map;
    DecisionData decisionData = input.decisionData;

    if (map.roads.size() == 0)
    {
        std::cout << " Map size false" << std::endl;
        output.decisionData = decisionData;
        output.flag = false;
        return;
    } // map数据检查

    // 本函数中使用的临时变量
    // std::cout << BOLDBLUE << "location gaussx: " << imu.gaussx() << "; location y: " << imu.gaussy() << "; location yaw: " << imu.yaw() << RESET << std::endl;
    int32_t lastRoadId, lastLaneId, lastIndex; // 分别是上次计算获得的RoadID，上次的LaneID，上次的路点Index，用于加快程序运行速度
    lastRoadId = decisionData.currentId;
    lastLaneId = decisionData.currentLaneId;
    lastIndex = decisionData.currentIndex;
    // std::cout << "20221011 decisiondata currentID in get current position: " << decisionData.currentId << "; " << decisionData.currentLaneId << std::endl;

    // 找对应的roadindex 和laneindex
    int32_t lastRoadIndex = -1;
    int32_t lastLaneIndex = -1;
    for (int i = 0; i < (int)map.roads.size(); i++)
    {
        if (lastRoadId == map.roads[i].id)
        {
            lastRoadIndex = i;
            break;
        }
    }

    if (lastRoadIndex != -1) // 找到road
    {

        for (int i = 0; i < (int)map.roads[lastRoadIndex].lanes.size(); i++)
        {
            if (lastLaneId == map.roads[lastRoadIndex].lanes[i].id)
            {
                lastLaneIndex = i;
                break;
            }
        }
    }

    float minDis = 0x7f7f7f7f;
    int32_t targetRoad = -1;
    int32_t targetLane = -1;
    int32_t targetIndex = -1;

    if (lastRoadIndex != -1 && lastLaneIndex != -1)
    {
        // 开始执行
        // 先找上次的点之后，这条车道上的点
        // std::cout << BOLDBLUE << "20221011 (2): " << map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints.size() << RESET << std::endl;
        for (int i = lastIndex; i < map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints.size(); i++)
        {
            // std::cout << BLUE << "***** i 1 =  " << i << std::endl;
            GaussRoadPoint thisPoint = map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints[i]; // 询问的这个点
                                                                                                         // std::cout << BOLDBLUE << "20221011 (3): " << thisPoint.yaw << std::endl;
                                                                                                         //  考虑航向角
            angleRegulateParam paramAR{};
            angleRegulateInput inputAR{abs(thisPoint.yaw - imu.yaw())};
            angleRegulateOutput outputAR{0};

            angleRegulate(paramAR, inputAR, outputAR);
            if (outputAR.angleOutput > POINT_ANGLE_THRESHOLD)
            {
                // std::cout << BLUE << "***** This point x " << thisPoint.GaussX << " y " << thisPoint.GaussY << std::endl;
                // std::cout << BLUE << "***** This point yaw " << thisPoint.yaw << " IMU yaw " << imu.yaw() << " regulate " << angleRegulate(abs(thisPoint.yaw - imu.yaw())) << RESET << std::endl;
                // std::cout << BLUE << "***** Angle wrong 1 " << abs(thisPoint.yaw - imu.yaw()) << RESET << std::endl;
                break;
            }
            // 考虑距离
            // std::cout << BLUE << "123456789 imu.yaw" << imu.yaw() << RESET << std::endl;

            getDistanceParam paramGD{};
            getDistanceInput inputGD{imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY};
            getDistanceOutput outputGD{0};

            getDistance(paramGD, inputGD, outputGD);
            double dis = outputGD.dis;

            // double dis = getDistance(imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY); // 计算该点与自车距离
            if (dis <= minDis) //????这个minDis很大，意味着第一次遇到的点就一定是满足条件的点吧
            {
                minDis = dis;
                targetIndex = i;
            }
            else
            {
                break; // 当计算的dis第一次大于minDis的时候，就退出循环了，对于一些曲线道路。这个是否合理？？？
            }
        }

        // ？？？这里有个问题，需要增加一个判断，如果最小点是本线段的最后一点，应该与后继线段的第一点做一个比较，避免达到阈值才到下一条线段
        if (targetIndex != -1 && minDis <= POINT_DIS_THRESHOLD_ON_CURRENT_LANE) // 如果找到了，就直接退出
        {
            decisionData.currentId = lastRoadId;
            decisionData.currentLaneId = lastLaneId;
            decisionData.currentIndex = targetIndex;
            // std::cout << " *** hhhhhhhhhhhhhh Flag 1 ***" << std::endl;
            output.decisionData = decisionData;
            output.flag = true;
            return;
        }

        // 再找这条车道上的其他点，即上次点之前的点。？？？为什么要往前找，走过的路再回头似乎不合适
        targetIndex = -1; // 20230227 添加赋值
        for (int i = 0; i < map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints.size(); i++)
        {
            GaussRoadPoint thisPoint = map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints[i]; // 询问的这个点
            // 考虑航向角
            //  std::cout << BLUE << "***** i 2 =  " << i << std::endl;
            angleRegulateParam paramAR{};
            angleRegulateInput inputAR{abs(thisPoint.yaw - imu.yaw())};
            angleRegulateOutput outputAR{0};

            angleRegulate(paramAR, inputAR, outputAR);
            if (outputAR.angleOutput > POINT_ANGLE_THRESHOLD)
            {
                // std::cout << BLUE << "***** This point x " << thisPoint.GaussX << " y " << thisPoint.GaussY << std::endl;
                // std::cout << BLUE << "***** This point yaw " << thisPoint.yaw << " IMU yaw " << imu.yaw() << " regulate " << angleRegulate(abs(thisPoint.yaw - imu.yaw())) << RESET << std::endl;
                // std::cout << BLUE << "***** Angle wrong 2 " << abs(thisPoint.yaw - imu.yaw()) << RESET << std::endl;
                continue;
            }
            // 考虑距离
            getDistanceParam paramGD{};
            getDistanceInput inputGD{imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY};
            getDistanceOutput outputGD{0};

            getDistance(paramGD, inputGD, outputGD);
            double dis = outputGD.dis;
            // double dis = getDistance(imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY); // 计算该点与自车距离
            if (dis <= minDis)
            {
                minDis = dis;
                targetIndex = i;
            }
        }

        if (targetIndex != -1 && minDis <= POINT_DIS_THRESHOLD_ON_CURRENT_LANE) // 如果找到了，也直接退出
        {
            decisionData.currentId = lastRoadId;
            decisionData.currentLaneId = lastLaneId;
            decisionData.currentIndex = targetIndex;
            // std::cout << " *** hhhhhhhhhhhhhh Flag 2 ***" << std::endl;
            output.decisionData = decisionData;
            output.flag = true;
            return;
        }
    }

    // 在当前道路的后继道路上找点
    int nextRoadId = std::get<0>(decisionData.nextId);
    int nexLaneId = std::get<1>(decisionData.nextId);
    // std::cout << BOLDBLUE << "20221011 (3): " << nextRoadId << "; " << nexLaneId << std::endl;

    int32_t nextRoadIndex = -1, nextLaneIndex = -1; // 后继road 和 lane 的 index
    for (int i = 0; i < map.roads.size(); i++)
    {
        if (nextRoadId == map.roads[i].id)
        {
            nextRoadIndex = i;
            break;
        }
    }

    if (nextRoadIndex != -1)
    {
        for (int i = 0; i < map.roads[nextRoadIndex].lanes.size(); i++)
        {
            if (nexLaneId == map.roads[nextRoadIndex].lanes[i].id)
            {
                nextLaneIndex = i;
                break;
            }
        }
    }

    if (nextRoadIndex != -1 && nexLaneId != -1)
    {
        targetIndex = -1; // 20230227 添加赋值
        for (int i = 0; i < map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.size(); i++)
        {
            GaussRoadPoint thisPoint = map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints[i]; // 询问的这个点
            // 考虑航向角
            //  std::cout << BLUE << "***** i 2 =  " << i << std::endl;
            angleRegulateParam paramAR{};
            angleRegulateInput inputAR{abs(thisPoint.yaw - imu.yaw())};
            angleRegulateOutput outputAR{0};

            angleRegulate(paramAR, inputAR, outputAR);
            if (outputAR.angleOutput > POINT_ANGLE_THRESHOLD)
            {
                // std::cout << BLUE << "***** This point x " << thisPoint.GaussX << " y " << thisPoint.GaussY << std::endl;
                // std::cout << BLUE << "***** This point yaw " << thisPoint.yaw << " IMU yaw " << imu.yaw() << " regulate " << angleRegulate(abs(thisPoint.yaw - imu.yaw())) << RESET << std::endl;
                // std::cout << BLUE << "***** Angle wrong 2 " << abs(thisPoint.yaw - imu.yaw()) << RESET << std::endl;
                continue;
            }
            // 考虑距离
            getDistanceParam paramGD{};
            getDistanceInput inputGD{imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY};
            getDistanceOutput outputGD{0};

            getDistance(paramGD, inputGD, outputGD);
            double dis = outputGD.dis;
            // double dis = getDistance(imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY); // 计算该点与自车距离
            if (dis <= minDis)
            {
                minDis = dis;
                targetIndex = i;
            }
        }
        if (targetIndex != -1 && minDis <= POINT_DIS_THRESHOLD_ON_OTHER_LANE) // 如果找到了，也直接退出
        {
            decisionData.currentId = nextRoadId;
            decisionData.currentLaneId = nexLaneId;
            decisionData.currentIndex = targetIndex;

            output.decisionData = decisionData;
            output.flag = true;
            return;
        }
    }

    // 再在所有道路里寻找
    // 先用很小的时间代价给这些路排个序，更可能在的路先访问
    std::priority_queue<std::tuple<float, int32_t>, std::vector<std::tuple<float, int32_t>>, std::greater<std::tuple<float, int32_t>>> heap; // 小根堆
    for (int i = 0; i < map.roads.size(); i++)
    {
        std::tuple<float, int32_t> thisroad;
        std::get<0>(thisroad) = 0x7f7f7f7f; // 距离
        std::get<1>(thisroad) = i;          // index
        for (int j = 1; j <= 5; j++)        // 把一段路分成五段,以近似曲线,这样得到的"可能性"更准确
        {
            int32_t pointsNum = map.roads[i].lanes[0].gaussRoadPoints.size();
            GaussRoadPoint currentPoint;
            currentPoint.GaussX = imu.gaussx();
            currentPoint.GaussY = imu.gaussy(); // 当前位置包装成GaussRoadPoint类
                                                //??????这里跟j 没有关系呢，反复算的是同一个点
                                                // float dis = pointToLineDistance(map.roads[i].lanes[0].gaussRoadPoints[0], map.roads[i].lanes[0].gaussRoadPoints[pointsNum - 1], currentPoint);

            pointToLineDistanceParam paramPTLD{};
            pointToLineDistanceInput inputPTLD{map.roads[i].lanes[0].gaussRoadPoints[int((j - 1) * (pointsNum - 1) / 5)], map.roads[i].lanes[0].gaussRoadPoints[int(j * (pointsNum - 1) / 5)],
                                               currentPoint};
            pointToLineDistanceOutput outputPTLD{0};

            pointToLineDistance(paramPTLD, inputPTLD, outputPTLD);
            float dis = outputPTLD.dis;

            // float dis = pointToLineDistance(map.roads[i].lanes[0].gaussRoadPoints[int((j - 1) * (pointsNum - 1) / 5)], map.roads[i].lanes[0].gaussRoadPoints[int(j * (pointsNum - 1) / 5)],
            // currentPoint);
            std::get<0>(thisroad) = std::min(std::get<0>(thisroad), dis);
        }
        heap.push(thisroad);
    }

    // 选"估计距离"前5小的搜,搜不到就认为没有
    for (int i = 0; i < 5 && heap.empty() == false; i++) // 搜五个Road
    {
        Road thisRoad = map.roads[std::get<1>(heap.top())];
        for (int j = 0; j < thisRoad.lanes.size(); j++) // 搜每个Road里的所有Lane
        {
            Lane thisLane = thisRoad.lanes[j];
            for (int k = 0; k < thisLane.gaussRoadPoints.size(); k++) // 搜每个Lane里的所有点
            {
                // std::cout << BLUE << "***** k 3 =  " << k << " " << thisLane.gaussRoadPoints.size() << std::endl;
                GaussRoadPoint thisPoint = thisLane.gaussRoadPoints[k];
                // 考虑航向角
                angleRegulateParam paramAR{};
                angleRegulateInput inputAR{abs(thisPoint.yaw - imu.yaw())};
                angleRegulateOutput outputAR{0};

                angleRegulate(paramAR, inputAR, outputAR);
                if (outputAR.angleOutput > POINT_ANGLE_THRESHOLD)
                {
                    // std::cout << BLUE << "***** This point x " << thisPoint.GaussX << " y " << thisPoint.GaussY << std::endl;
                    // std::cout << BLUE << "***** This point yaw " << thisPoint.yaw << " IMU yaw " << imu.yaw() << " regulate " << angleRegulate(abs(thisPoint.yaw - imu.yaw())) << RESET << std::endl;
                    // std::cout << BLUE << "***** Angle wrong 3  " << abs(thisPoint.yaw - imu.yaw()) << RESET << std::endl;
                    continue;
                }
                // 考虑距离
                getDistanceParam paramGD{};
                getDistanceInput inputGD{imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY};
                getDistanceOutput outputGD{0};

                getDistance(paramGD, inputGD, outputGD);
                double dis = outputGD.dis;
                // float dis = getDistance(imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY);
                // std::cout << " *** Flag 1 ***" << std::endl;
                if (dis < minDis)
                {
                    minDis = dis;
                    targetRoad = map.roads[std::get<1>(heap.top())].id;
                    targetLane = thisLane.id;
                    targetIndex = k;
                }
                // std::cout << " *** Flag 2 ***" << std::endl;
            }
        }
        // 如果这条Road搜到了,就不往下搜了
        if (targetIndex != -1 && minDis <= POINT_DIS_THRESHOLD_ON_OTHER_LANE) // 如果找到了，也直接退出
        {
            // std::cout << " *** hhhhhhhhhhhhhh minDis ***" << minDis << std::endl;
            decisionData.currentId = targetRoad;
            decisionData.currentLaneId = targetLane;
            decisionData.currentIndex = targetIndex;
            // std::cout << " *** hhhhhhhhhhhhhh current ID ***" << targetRoad << "; " << targetLane << std::endl;
            // std::cout << " *** hhhhhhhhhhhhhh Flag 3 ***" << std::endl;
            output.decisionData = decisionData;
            output.flag = true;
            return;
        }
        // std::cout << " *** Flag 3 ***" << std::endl;
        heap.pop(); // 把堆顶弹出,便于下次使用
    }
    // std::cout << " *** Flag 4 ***" << std::endl;
    // std::cout << " *** hhhhhhhhhhhhhh Flag 4 ***" << std::endl;
    output.decisionData = decisionData;
    output.flag = false;
    return;
}

// 生成速度采样后，多种 轨迹上每个点的速度
/**
 * @brief 根据道路信息等，采样生成速度曲线
 * @param[IN] param 路测障碍物，imu，停止点
 * @param[IN] input 车速，决策数据
 * @param[OUT] output 更新后的决策数据
 
 * @cn_name: 生成速度曲线
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void generateSpeedList(const generateSpeedListParam &param, const generateSpeedListInput &input, generateSpeedListOutput &output)
{
    std::vector<infopack::ObjectsProto> objectsCmd = param.objectsCmd;
    pc::Imu imu = param.imu;
    double velocity = input.velocity;
    DecisionData decisionData = input.decisionData;

    double currentSpeed = velocity;
    if (currentSpeed < 0.1)
        currentSpeed = 1.2;

    double dDesireSpeed = DESIRED_SPEED;
    // 处理ACC跟车
    for (int i = 0; i < decisionData.controlTrajectoryList.size(); i++)
    {
        hasPrecedingVehicleParam paramHPV{imu, objectsCmd};
        hasPrecedingVehicleInput inputHPV{};
        hasPrecedingVehicleOutput outputHPV{true};

        hasPrecedingVehicle(paramHPV, inputHPV, outputHPV);

        if (outputHPV.flag)
        {
            // 计算acc速度
            cruiseControllerParam paramCC{imu, objectsCmd};
            cruiseControllerInput inputCC{};
            cruiseControllerOutput outputCC{0};

            cruiseController(paramCC, inputCC, outputCC);

            dDesireSpeed = std::min(outputCC.velocity, DESIRED_SPEED);
        }
    }
    // std::cout << "dDesireSpeed = " << dDesireSpeed << std::endl;
    //  清空速度列表
    decisionData.speedList.clear(); // 20220825
    decisionData.speedList.reserve(END_SPEED_NUM);

    std::vector<double> endPointSpeedList;
    endPointSpeedList.clear();
    endPointSpeedList.reserve(END_SPEED_NUM);

    ///////20230213 简化速度规划为1条
    double endPointSpeed = dDesireSpeed;
    endPointSpeedList.push_back(dDesireSpeed);

    // 使用定加速法生成平滑曲线
    std::vector<double> curvePointSpeed;
    interp_m::interpConstAccParam paramICA{};
    interp_m::interpConstAccInput inputICA{currentSpeed, dDesireSpeed, CURVE_POINT_NUM, 2.0};
    interp_m::interpConstAccOutput outputICA{curvePointSpeed};

    interp_m::constant_acceleration(paramICA, inputICA, outputICA);

    curvePointSpeed = outputICA.result;
    // std::vector<double> curvePointSpeed = interp_m::constant_acceleration(currentSpeed, dDesireSpeed, CURVE_POINT_NUM, 2.0);
    decisionData.speedList.push_back(curvePointSpeed);
    output.decisionData = decisionData;
}

// 生成轨迹采样结果,将之前生成的轨迹和速度合并在controlTrajectoryList中
/**
 * @brief 生成采样轨迹，供后续选择
 * @param[IN] param 无
 * @param[IN] input 决策数据
 * @param[OUT] output 更新后的决策数据
 
 * @cn_name: 生成采样曲线
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void generateTrajectoryList(const generateTrajectoryListParam &param, const generateTrajectoryListInput &input, generateTrajectoryListOutput &output)
{
    DecisionData decisionData = input.decisionData;

    decisionData.controlTrajectoryList.clear();
    decisionData.controlTrajectoryList.reserve(CURVE_NUM * END_SPEED_NUM);

    for (int i = 0; i < CURVE_NUM; i++)
    {
        for (int j = 0; j < END_SPEED_NUM; j++)
        {
            PlanningTrajectory trajectoryPointList;
            trajectoryPointList.planningPoints.clear();
            for (int k = 0; k < CURVE_POINT_NUM; k++)
            {
                PlanningPoint trajectoryPoint;
                trajectoryPoint.x = decisionData.finalPathList[i].planningPoints[k].x;
                trajectoryPoint.y = decisionData.finalPathList[i].planningPoints[k].y;
                trajectoryPoint.angle = decisionData.finalPathList[i].planningPoints[k].angle;
                trajectoryPoint.v = decisionData.speedList[j][k];
                trajectoryPointList.planningPoints.push_back(trajectoryPoint);
            }
            decisionData.controlTrajectoryList.push_back(trajectoryPointList);
        }
    }
    output.decisionData = decisionData;
}

// 处理红绿灯，以前包含避障的判断，现在都注释掉了,好多地方不对，重写了
/**
 * @brief 根据红绿灯信息更新决策数据
 * @param[IN] param 栅格障碍物，交通灯
 * @param[IN] input 决策数据
 * @param[OUT] output 更新后的决策数据，红绿灯信息
 
 * @cn_name: 处理交通信号灯
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void processTrafficLight(const processTrafficLightParam &param, const processTrafficLightInput &input, processTrafficLightOutput &output)
{
    DecisionData decisionData = input.decisionData;
    prediction::ObjectList prediction = param.prediction;
    infopack::TrafficLight trafficLight = param.trafficLight;

    bool traffic_light_active = trafficLight.active();
    infopack::TrafficLight_State traffic_light_state = trafficLight.state();
    if (traffic_light_active)
    {
        if (traffic_light_state == infopack::TrafficLight::RED_LIGHT || traffic_light_state == infopack::TrafficLight::YELLOW_LIGHT) // 原来算法计算了一下黄灯时，以当前速度可以通过的可行性
        {
            output.decisionData = decisionData;
            output.flag = 0;
            return; // 红灯或黄灯，就没有可行走的轨迹了
        }
        else
        {
            for (int i = 0; i < decisionData.controlTrajectoryList.size(); i++)
            {
                decisionData.feasibleTrajectoryIndexList.push_back(i);
            }
            output.decisionData = decisionData;
            output.flag = 2;
            return; // 绿灯所有轨迹都可选
        }
    }    // if (traffic_light_active)
    else // 红绿灯无效下，所有轨迹都可选
    {
        for (int i = 0; i < decisionData.controlTrajectoryList.size(); i++)
        {
            decisionData.feasibleTrajectoryIndexList.push_back(i);
        }

        // std::cout << RED << "feasibleTrajectoryIndexList number: " << decisionData.feasibleTrajectoryIndexList.size() << RESET << std::endl;
        output.decisionData = decisionData;
        output.flag = 2;
        return;
    }
}

/**
 * @brief 评估轨迹，并获取最优轨迹索引
 * @param[IN] param 栅格障碍物，交通灯，imu，路测障碍物
 * @param[IN] input 决策数据
 * @param[OUT] output 更新后的决策数据，最优轨迹索引
 
 * @cn_name: 获取最优轨迹索引
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getOptimalTrajectoryIndex(const getOptimalTrajectoryIndexParam &param, const getOptimalTrajectoryIndexInput &input, getOptimalTrajectoryIndexOutput &output)
{
    prediction::ObjectList prediction = param.prediction;
    infopack::TrafficLight trafficLight = param.trafficLight;
    pc::Imu imu = param.imu;
    std::vector<infopack::ObjectsProto> objectsCmd = param.objectsCmd;

    DecisionData decisionData = input.decisionData;

    decisionData.feasibleTrajectoryIndexList.clear();
    decisionData.feasibleTrajectoryIndexList.reserve(0);

    // 处理交通信号灯
    processTrafficLightParam paramPTL{prediction, trafficLight};
    processTrafficLightInput inputPTL{decisionData};
    processTrafficLightOutput outputPTL{decisionData, 0};

    processTrafficLight(paramPTL, inputPTL, outputPTL);
    decisionData = outputPTL.decisionData;

    // int status = processTrafficLight(decisionData, prediction, trafficLight);
    if (outputPTL.flag != 2)
    {
        output.decisionData = decisionData;
        output.index = -1;
        return;
    }

    // double maxDistance = assessTrajectory(decisionData.controlTrajectoryList[decisionData.feasibleTrajectoryIndexList[0]], prediction);
    double maxDistance = -1;
    double tempDistance;
    int32_t index = -1;
    for (int i = 0; i < (int)decisionData.feasibleTrajectoryIndexList.size(); i++)
    {

        assessTrajectoryParam paramAT{prediction, imu, objectsCmd};
        assessTrajectoryInput inputAT{decisionData.controlTrajectoryList[decisionData.feasibleTrajectoryIndexList[i]]};
        assessTrajectoryOutput outputAT{0};

        assessTrajectory(paramAT, inputAT, outputAT);
        tempDistance = outputAT.minDistance;

        if (tempDistance > SAFE_DISTANCE) // 认为不会碰撞的安全距离
        {

            index = i;
            break; // 找到第一条可行路线就退出
        }
    }

    if (index == -1)
    {
        if (prediction.object_size() != 0)
        {
            output.decisionData = decisionData;
            output.index = -1;
            return;
        }
        else
        {
            output.decisionData = decisionData;
            output.index = 0;
            return;
        }
    }
    output.decisionData = decisionData;
    output.index = decisionData.feasibleTrajectoryIndexList[index];
    return;
}

// true表示可行，false表示碰撞
/**
 * @brief 轨迹碰撞检测，true表示可行，false表示碰撞
 * @param[IN] param 障碍物
 * @param[IN] input 轨迹
 * @param[OUT] output 是否碰撞
 
 * @cn_name: 检测轨迹碰撞
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void trajectoryCollisionCheck(const trajectoryCollisionCheckParam &param, const trajectoryCollisionCheckInput &input, trajectoryCollisionCheckOutput &output)
{
    prediction::ObjectList prediction = param.prediction;
    PlanningTrajectory trajectory = input.trajectory;

    double t = 0;
    int index = 0;
    double tRemainder;
    PlanningPoint predictTempPoint;

    // std::cout << RED << "prediction " << prediction.object(0).w() << ";" <<  prediction.object(0).l() << std::endl;
    // std::cout << CYAN << "trajsize " << trajectory.trajectoryPoints.size() << ";" << std::endl;
    for (int i = 0; i < (int)trajectory.planningPoints.size() - 1; i++)
    {
        // std::cout << RED << "trajhh " << trajectory.planningPoints[i].y << ";" <<  i << std::endl;
        t = 0;
        for (auto object : prediction.object())
        {
            index = (int)t / PREDICT_FREQUENCY;

            tRemainder = t - index * PREDICT_FREQUENCY;
            predictTempPoint.x = object.predictpoint(index).x() * (1 - tRemainder) + object.predictpoint(index + 1).x() * tRemainder;
            predictTempPoint.y = object.predictpoint(index).y() * (1 - tRemainder) + object.predictpoint(index + 1).y() * tRemainder;
            // std::cout << RED <<  "trajTemp " << predictTempPoint.x << ";" <<  predictTempPoint.y << std::endl;
            pointCollisionCheckParam paramPCC{};
            pointCollisionCheckInput inputPCC{trajectory.planningPoints[i], predictTempPoint, object.w() / 2, object.l() / 2};
            pointCollisionCheckOutput outputPCC{true};
            pointCollisionCheck(paramPCC, inputPCC, outputPCC);

            if (!outputPCC.flag)
            {
                output.trajectory = trajectory;
                output.flag = false;
                return;
            }
        }
        // std::cout<< GREEN << "tra" << (*trajectoryPointIt).y <<RESET<< std::endl;
        getDistanceParam paramGD{};
        getDistanceInput inputGD{trajectory.planningPoints[i].x, trajectory.planningPoints[i].y, trajectory.planningPoints[i + 1].x, trajectory.planningPoints[i + 1].y};
        getDistanceOutput outputGD{0};

        getDistance(paramGD, inputGD, outputGD);
        double len = outputGD.dis;

        // double len = getDistance(trajectory.planningPoints[i].x, trajectory.planningPoints[i].y, trajectory.planningPoints[i + 1].x, trajectory.planningPoints[i + 1].y);
        double vMean = (trajectory.planningPoints[i].v + trajectory.planningPoints[i + 1].v) / 2;
        if (vMean <= 0.1)
        {
            output.trajectory = trajectory;
            output.flag = true;
            return;
        }
        t += len / vMean;
        if (t >= 1.95)
        {
            output.trajectory = trajectory;
            output.flag = true;
            return;
        }
    }
    // delete predictTempPoint;
    output.trajectory = trajectory;
    output.flag = true;
    return;
}

/**
 * @brief 判断是否能通过黄灯
 * @param[IN] param 无
 * @param[IN] input 轨迹，剩余时间，到停止线前的距离
 * @param[OUT] output 是否通过
 
 * @cn_name: 判断黄灯能否通过
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void ableToPassYellowLight(const ableToPassYellowLightParam &param, const ableToPassYellowLightInput &input, ableToPassYellowLightOutput &output)
{
    PlanningTrajectory trajectory = input.trajectory;
    double remaining_time = input.remaining_time;
    double lane_length_before_intersection = input.lane_length_before_intersection;

    double t = 0;
    double passDistance = 0;
    for (int i = 0; i < trajectory.planningPoints.size() - 1; i++)
    {
        t = 0;
        PlanningPoint currentPoint = trajectory.planningPoints[i];
        PlanningPoint nextPoint = trajectory.planningPoints[i + 1];
        getDistanceParam paramGD{};
        getDistanceInput inputGD{currentPoint.x, currentPoint.y, nextPoint.x, nextPoint.y};
        getDistanceOutput outputGD{0};

        getDistance(paramGD, inputGD, outputGD);
        double len = outputGD.dis;

        double vMean = (currentPoint.v + nextPoint.v) / 2;
        t += len / vMean;
        passDistance += len;
        if (t <= remaining_time)
        {
            if (passDistance > lane_length_before_intersection + 2) // rear bumper passes the stop line
            {
                output.trajectory = trajectory;
                output.flag = true;
                return;
            }
        }
        else
        {
            output.trajectory = trajectory;
            output.flag = false;
            return;
        }
    }
    output.trajectory = trajectory;
    output.flag = false;
    return;
}

/**
 * @brief 轨迹点碰撞检测
 * @param[IN] param 无
 * @param[IN] input 点，障碍物中心点，障碍物长宽
 * @param[OUT] output 是否碰撞
 
 * @cn_name: 检测轨迹点碰撞
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void pointCollisionCheck(const pointCollisionCheckParam &param, const pointCollisionCheckInput &input, pointCollisionCheckOutput &output)
{
    PlanningPoint trajectoryPoint = input.trajectoryPoint;
    PlanningPoint predictPoint = input.predictPoint;
    double w = input.w;
    double l = input.l;

    double xNew, yNew;
    yNew = trajectoryPoint.y - predictPoint.y;
    xNew = trajectoryPoint.x - predictPoint.x;

    PlanningPoint vehPoint[5];
    PlanningPoint objPoint[5];

    vehPoint[0].x = xNew + VEHICLE_DIAGONAL * sin((VEHICLE_DIAGONAL_DEGREE + trajectoryPoint.angle) / 180 * M_PI);
    vehPoint[1].x = xNew + VEHICLE_DIAGONAL * sin((180 - VEHICLE_DIAGONAL_DEGREE + trajectoryPoint.angle) / 180 * M_PI);
    vehPoint[2].x = xNew + VEHICLE_DIAGONAL * sin((180 + VEHICLE_DIAGONAL_DEGREE + trajectoryPoint.angle) / 180 * M_PI);
    vehPoint[3].x = xNew + VEHICLE_DIAGONAL * sin((0 - VEHICLE_DIAGONAL_DEGREE + trajectoryPoint.angle) / 180 * M_PI);
    vehPoint[0].y = yNew + VEHICLE_DIAGONAL * cos((VEHICLE_DIAGONAL_DEGREE + trajectoryPoint.angle) / 180 * M_PI);
    vehPoint[1].y = yNew + VEHICLE_DIAGONAL * cos((180 - VEHICLE_DIAGONAL_DEGREE + trajectoryPoint.angle) / 180 * M_PI);
    vehPoint[2].y = yNew + VEHICLE_DIAGONAL * cos((180 + VEHICLE_DIAGONAL_DEGREE + trajectoryPoint.angle) / 180 * M_PI);
    vehPoint[3].y = yNew + VEHICLE_DIAGONAL * cos((0 - VEHICLE_DIAGONAL_DEGREE + trajectoryPoint.angle) / 180 * M_PI);

    objPoint[0].x = w;
    objPoint[1].x = -w;
    objPoint[2].x = -w;
    objPoint[3].x = w;
    objPoint[0].y = l;
    objPoint[1].y = l;
    objPoint[2].y = -l;
    objPoint[3].y = -l;

    vehPoint[4] = vehPoint[0];
    objPoint[4] = objPoint[0];

    // std::cout<< RED << "veh" << vehPoint[0].y << std::endl;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            boundaryCollisionCheckParam paramBCC{};
            boundaryCollisionCheckInput inputBCC{vehPoint[j], objPoint[i], vehPoint[j + 1], objPoint[i + 1]};
            boundaryCollisionCheckOutput outputBCC{false};
            boundaryCollisionCheck(paramBCC, inputBCC, outputBCC);

            if (!outputBCC.flag)
            {
                output.flag = false;
                return;
            }
        }
        innerCollisionCheckParam paramICC{};
        innerCollisionCheckInput inputICC{vehPoint[0], vehPoint[1], vehPoint[2], vehPoint[3], objPoint[i]};
        innerCollisionCheckOutput outputICC{false};
        innerCollisionCheck(paramICC, inputICC, outputICC);

        if (!outputICC.flag)
        {
            output.flag = false;
            return;
        }
    }
    output.flag = true;
    return;
}

/**
 * @brief 边碰撞检测
 * @param[IN] param 无
 * @param[IN] input 俩线段顶点
 * @param[OUT] output 是否碰撞
 
 * @cn_name: 检测边碰撞
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void boundaryCollisionCheck(const boundaryCollisionCheckParam &param, const boundaryCollisionCheckInput &input, boundaryCollisionCheckOutput &output)
{
    PlanningPoint p0 = input.p0;
    PlanningPoint p1 = input.p1;
    PlanningPoint p2 = input.p2;
    PlanningPoint p3 = input.p3;

    double z0, z1, z2, z3;
    z0 = ((p1.x - p0.x) * (p2.y - p1.y) - (p2.x - p1.x) * (p1.y - p0.y));
    z1 = ((p2.x - p1.x) * (p3.y - p2.y) - (p3.x - p2.x) * (p2.y - p1.y));
    z2 = ((p3.x - p2.x) * (p0.y - p3.y) - (p0.x - p3.x) * (p3.y - p2.y));
    z3 = ((p0.x - p3.x) * (p1.y - p0.y) - (p1.x - p0.x) * (p0.y - p3.y));

    if ((z0 * z1 > 0) && (z1 * z2 > 0) && (z2 * z3 > 0))
    {
        output.flag = false;
        return;
    }
    else
    {
        output.flag = true;
        return;
    }
}

/**
 * @brief 内部包含检测
 * @param[IN] param 无
 * @param[IN] input 检测点和障碍物顶点
 * @param[OUT] output 是否包含
 
 * @cn_name: 检测内部包含
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void innerCollisionCheck(const innerCollisionCheckParam &param, const innerCollisionCheckInput &input, innerCollisionCheckOutput &output)
{
    PlanningPoint p0 = input.p0;
    PlanningPoint p1 = input.p1;
    PlanningPoint p2 = input.p2;
    PlanningPoint p3 = input.p3;
    PlanningPoint obj = input.obj;

    struct NormalVector
    {
        double x;
        double y;
    };

    struct TestVector
    {
        double x;
        double y;
    };

    NormalVector normalVector01, normalVector12;
    TestVector testVector0, testVector2;
    double innerProduct01, innerProduct12, innerProduct23, innerProduct30;

    normalVector01.x = p2.x - p1.x;
    normalVector12.x = p3.x - p2.x;
    normalVector01.y = p2.y - p1.y;
    normalVector12.y = p3.y - p2.y;
    testVector0.x = obj.x - p0.x;
    testVector2.x = obj.x - p2.x;
    testVector0.y = obj.y - p0.y;
    testVector2.y = obj.y - p2.y;

    innerProduct01 = normalVector01.x * testVector0.x + normalVector01.y * testVector0.y;
    innerProduct12 = normalVector12.x * testVector2.x + normalVector12.y * testVector2.y;
    innerProduct23 = -normalVector01.x * testVector2.x - normalVector01.y * testVector2.y;
    innerProduct30 = -normalVector12.x * testVector0.x - normalVector12.y * testVector0.y;

    if (innerProduct01 >= 0 && innerProduct12 >= 0 && innerProduct23 >= 0 && innerProduct30 >= 0)
    {
        output.flag = false;
        return;
    }
    output.flag = true;
    return;
}

// 通过当前预瞄点，计算换道后的预瞄点？？？是预瞄点还是当前点都可以用这个函数
// 找另一条车道上，与当前点最近的点
/**
 * @brief 判断是否可以换道
 * @param[IN] param 当前道路，当前车道
 * @param[IN] input 原规划预瞄点车道Id，原规划预瞄点路点Id，原规划预瞄点，向左还是向右变道
 * @param[OUT] output 新预瞄点车道Id，新规划预瞄点路点Id，新规划预瞄点，是否变道
 
 * @cn_name: 判断是否可以换道
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void changeCurrentPoint(const changeCurrentPointParam &param, const changeCurrentPointInput &input, changeCurrentPointOutput &output)
{
    Road road = param.road;
    Lane curLane = param.curLane;

    int laneId = input.laneId;
    int pointIndex = input.pointIndex;
    GaussRoadPoint currentPoint = input.currentPoint;
    bool isLeft = input.isLeft;

    // 计算换道后的laneID
    int changeId; // 换道后的laneID

    if (isLeft)
    {
        if (curLane.leftLaneId.size() == 0)
        {
            output.laneId = laneId;
            output.pointIndex = pointIndex;
            output.currentPoint = currentPoint;
            output.flag = false;
            return;
        }
        else
            changeId = curLane.id + 1; // 从右往左。laneID 增加 1
    }
    else
    {
        if (curLane.rightLaneId.size() == 0)
        {
            output.laneId = laneId;
            output.pointIndex = pointIndex;
            output.currentPoint = currentPoint;
            output.flag = false;
            return;
        }
        else
            changeId = curLane.id - 1;
    }

    Lane changeLane; // 换道后的lane 对象
    for (auto lane : road.lanes)
    {
        if (lane.id == changeId)
        {
            changeLane = lane;
            laneId = changeId;
            break;
        }
    }

    double disCur = 10000; // 距离
    double disTemp;
    GaussRoadPoint *currentPointTemp = new GaussRoadPoint();
    int index = 0;
    int indexTemp = 0; // 点索引
    for (auto roadPoint : changeLane.gaussRoadPoints)
    {
        // disTemp = sqrt((pow(roadPoint.GaussX - currentPoint.GaussX, 2)) + pow(roadPoint.GaussY - currentPoint.GaussY, 2));
        disTemp = sqrt((roadPoint.GaussX - currentPoint.GaussX) * (roadPoint.GaussX - currentPoint.GaussX) + (roadPoint.GaussY - currentPoint.GaussY) * (roadPoint.GaussY - currentPoint.GaussY));
        if (disTemp < disCur)
        {
            indexTemp = index;
            disCur = disTemp;
            currentPointTemp->curvature = roadPoint.curvature;
            currentPointTemp->GaussX = roadPoint.GaussX;
            currentPointTemp->GaussY = roadPoint.GaussY;
            currentPointTemp->yaw = roadPoint.yaw;
        }
        index++;
    }
    pointIndex = indexTemp;
    currentPoint = *currentPointTemp;
    // delete planningPointTemp；
    delete currentPointTemp;
    output.laneId = laneId;
    output.pointIndex = pointIndex;
    output.currentPoint = currentPoint;
    output.flag = true;
    return;
};

/**
 * @brief 局部坐标轨迹转为全局轨迹
 * @param[IN] param 无
 * @param[IN] input 局部坐标轨迹，局部坐标轨迹原地对应的全局点
 * @param[OUT] output 全局轨迹
 
 * @cn_name: 局部坐标轨迹转为全局坐标轨迹
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void changeLocalToGlobal(const changeLocalToGlobalParam &param, const changeLocalToGlobalInput &input, changeLocalToGlobalOutput &output)
{
    PlanningTrajectory trajectory = input.trajectory;
    GaussRoadPoint gaussRoadPoint = input.gaussRoadPoint;

    double dX, dY, dTheta;
    PlanningPoint globalTrajectoryPoint;
    PlanningTrajectory globalTrajectory;
    double len, pointTheta;
    dX = gaussRoadPoint.GaussX - trajectory.planningPoints[0].x;
    dY = gaussRoadPoint.GaussY - trajectory.planningPoints[0].y;
    dTheta = gaussRoadPoint.yaw - trajectory.planningPoints[0].angle;

    static double cumul_s = 0;
    for (int i = 0; i < (int)trajectory.planningPoints.size(); i++)
    {
        // len = sqrt(pow(trajectory.planningPoints[i].x - trajectory.planningPoints[0].x, 2) + pow(trajectory.planningPoints[i].y - trajectory.planningPoints[0].y, 2));
        len = sqrt((trajectory.planningPoints[i].x - trajectory.planningPoints[0].x) * (trajectory.planningPoints[i].x - trajectory.planningPoints[0].x) +
                   (trajectory.planningPoints[i].y - trajectory.planningPoints[0].y) * (trajectory.planningPoints[i].y - trajectory.planningPoints[0].y));
        pointTheta = atan2(trajectory.planningPoints[i].x - trajectory.planningPoints[0].x, trajectory.planningPoints[i].y - trajectory.planningPoints[0].y) / M_PI * 180;
        globalTrajectoryPoint.gaussX = len * cos((dTheta + pointTheta) / 180 * M_PI) + dX;
        globalTrajectoryPoint.gaussY = len * sin((dTheta + pointTheta) / 180 * M_PI) + dY;
        globalTrajectoryPoint.gaussAngle = dTheta + trajectory.planningPoints[i].angle;
        if (globalTrajectoryPoint.gaussAngle >= 360)
        {
            globalTrajectoryPoint.gaussAngle -= 360;
        }
        else if (globalTrajectoryPoint.gaussAngle <= 0)
        {
            globalTrajectoryPoint.gaussAngle += 360;
        }

        globalTrajectoryPoint.v = trajectory.planningPoints[i].v;
        if (i == 0 || i == (int)trajectory.planningPoints.size() - 1)
        {
            globalTrajectoryPoint.curvature = 0;
        }
        else
        {
            calculateCurvatureParam paramCC{};
            calculateCurvatureInput inputCC{trajectory.planningPoints[i], trajectory.planningPoints[i - 1], trajectory.planningPoints[i + 1]};
            calculateCurvatureOutput outputCC{0};

            calculateCurvature(paramCC, inputCC, outputCC);
            globalTrajectoryPoint.curvature = outputCC.curvature;
        }
        globalTrajectoryPoint.s = cumul_s;
        if (i < (int)trajectory.planningPoints.size() - 1)
        {
            getDistanceParam paramGD{};
            getDistanceInput inputGD{trajectory.planningPoints[i + 1].x, trajectory.planningPoints[i + 1].y, trajectory.planningPoints[i].x, trajectory.planningPoints[i].y};
            getDistanceOutput outputGD{0};

            getDistance(paramGD, inputGD, outputGD);
            cumul_s += outputGD.dis;

            // cumul_s += getDistance(trajectory.planningPoints[i + 1].x, trajectory.planningPoints[i + 1].y, trajectory.planningPoints[i].x, trajectory.planningPoints[i].y);
        }
        else
        {
            cumul_s = 0;
        }
        globalTrajectory.planningPoints.push_back(globalTrajectoryPoint);
    }
    output.globalTrajectory = globalTrajectory;
    return;
}

// 局部规划初始化
/**
 * @brief 局部规划决策数据初始化
 * @param[IN] param 无
 * @param[IN] input 决策数据
 * @param[OUT] output 初始化后的决策数据
 
 * @cn_name: 初始化局部规划
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void initLocalPlanning(const initLocalPlanningParam &param, const initLocalPlanningInput &input, initLocalPlanningOutput &output)
{
    DecisionData decisionData = input.decisionData;

    decisionData.controlTrajectoryList.clear();
    for (int i = 0; i < 1; i++)
    {
        PlanningTrajectory tempTrajectory;
        for (int j = 0; j < CURVE_POINT_NUM; j++)
        {
            tempTrajectory.planningPoints.push_back(PlanningPoint());
            tempTrajectory.planningPoints[j].x = 0;
            tempTrajectory.planningPoints[j].y = 0;
            tempTrajectory.planningPoints[j].v = 0;
            tempTrajectory.planningPoints[j].angle = 0;
        }
        decisionData.controlTrajectoryList.push_back(tempTrajectory);
    }
    decisionData.optimalTrajectoryIndex = -1; // 最优轨迹序号

    output.decisionData = decisionData;
}

// 返回decisionDate中roadID 和laneID的索引
/**
 * @brief 返回decisionDate中roadID 和laneID的索引
 * @param[IN] param 地图
 * @param[IN] input 决策数据
 * @param[OUT] output roadID 和laneID的索引
 
 * @cn_name: 获得道路和车道的索引
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void fixRoadLaneIndex(const fixRoadLaneIndexParam &param, const fixRoadLaneIndexInput &input, fixRoadLaneIndexOutput &output)
{
    RoadMap map = param.map;
    DecisionData decisionData = input.decisionData;
    // 找对应的index
    //  std::cout << "ROAD MAP SIZE: " << map.roads.size() << std::endl;
    //  std::cout << "ROAD CURRENT ID AND LANE ID: " << decisionData.currentId << "; " << decisionData.currentLaneId << std::endl;
    int32_t roadIndex = -1;
    int32_t laneIndex = -1;
    // std::cout << "LANE MAP SIZE: " << map.roads[2].lanes.size() << std::endl;
    for (int i = 0; i < (int)map.roads.size(); i++)
    {
        // std::cout << "ROAD MAP ID : " << map.roads[i].id << std::endl;
        if (decisionData.currentId == map.roads[i].id)
        {
            roadIndex = i;
            break;
        }
    }

    // std::cout << "inter  ******* FIXED ROAD CURRENT ID AND LANE ID: " << roadIndex << "; " << laneIndex << std::endl;

    for (int i = 0; i < (int)map.roads[roadIndex].lanes.size(); i++)
    {
        // std::cout << "LANE MAP ID: " << map.roads[roadIndex].lanes[i].id << std::endl;
        if (decisionData.currentLaneId == map.roads[roadIndex].lanes[i].id)
        {
            laneIndex = i;
            break;
        }
    }
    // std::cout << "FIXED ROAD CURRENT ID AND LANE ID: " << roadIndex << "; " << laneIndex << std::endl;
    output.indexTuple = std::tuple<int32_t, int32_t>(roadIndex, laneIndex);
    return;
}

/**
 * @brief 由相邻三点近似计算曲率
 * @param[IN] param 无
 * @param[IN] input 点，前一点，后一点
 * @param[OUT] output 曲率
 
 * @cn_name: 计算曲率
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void calculateCurvature(const calculateCurvatureParam &param, const calculateCurvatureInput &input, calculateCurvatureOutput &output)
{
    PlanningPoint p0 = input.p0;
    PlanningPoint p1 = input.p1;
    PlanningPoint p2 = input.p2;

    double a1, a2, b1, b2, x, y;
    a1 = p1.x - p0.x;
    a2 = p2.x - p0.x;
    b1 = p1.y - p0.y;
    b2 = p2.y - p0.y;
    if (a1 * b2 == a2 * b1)
    {
        output.curvature = 0;
        return;
    }
    else
    {
        y = (a1 * a1 * a2 + b1 * b1 * a2 - a2 * a2 * a1 - b2 * b2 * a1) / (a2 * b1 - a1 * b2);
        x = (a1 * a1 * b2 + b1 * b1 * b2 - a2 * a2 * b1 - b2 * b2 * b1) / (a1 * b2 - a2 * b1);
        if (x > 100 || y > 100)
        {
            output.curvature = 0;
            return;
        }
        // return 2 / sqrt(pow(x, 2) + pow(y, 2));
        output.curvature = 2 / sqrt(x * x + y * y);
        return;
    }
}

// 优选估计，目前主要是与障碍物距离为判断条件，之前还结合速度等信息
/**
 * @brief 返回距离障碍物最近距离，进行轨迹评估
 * @param[IN] param 栅格障碍物，imu，路测障碍物
 * @param[IN] input 轨迹
 * @param[OUT] output 距离障碍物最近距离
 
 * @cn_name: 轨迹评估
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void assessTrajectory(const assessTrajectoryParam &param, const assessTrajectoryInput &input, assessTrajectoryOutput &output)
{
    PlanningTrajectory trajectory = input.trajectory;
    prediction::ObjectList prediction = param.prediction;
    std::vector<infopack::ObjectsProto> objectsCmd = param.objectsCmd;
    pc::Imu imu = param.imu;

    // std::cout<<"assessTrajectory----------------------start"<<std::endl;
    std::vector<double> distanceFromPointToObject;
    double minDisLength = 10;

    for (int i = 0; i < trajectory.planningPoints.size(); i++)
    {
        getMinDistanceOfPointParam paramGMDOP{prediction, imu, objectsCmd};
        getMinDistanceOfPointInput inputGMDOP{trajectory.planningPoints[i]};
        getMinDistanceOfPointOutput outputGMDOP{0};

        getMinDistanceOfPoint(paramGMDOP, inputGMDOP, outputGMDOP);

        distanceFromPointToObject.push_back(outputGMDOP.minDistance);
        // std::cout<<  YELLOW << i << "  prediction size  " << prediction.object_size()<< " X "<< prediction.object(0).predictpoint(0).x()<<
        //  " Y "<< prediction.object(0).predictpoint(0).y() <<RESET << std::endl;
        // std::cout<<  YELLOW << i << "  X  " << trajectory.planningPoints[i].x << " Y "<< trajectory.planningPoints[i].y <<RESET << std::endl;
        // std::cout << YELLOW << i << "    getMinDistanceOfPoint:    " << getMinDistanceOfPoint(trajectory.planningPoints[i], prediction) << RESET << std::endl;
    }
    // std::cout << "hh1 " << trajectory.trajectoryPoints[trajectory.trajectoryPoints.size()-1].v << std::endl;

    findMinParam paramFMin{};
    findMinInput inputFMin{distanceFromPointToObject};
    findMinOutput outputFMin{-1};

    findMin(paramFMin, inputFMin, outputFMin);
    output.minDistance = outputFMin.value;
    return;
    // return trajectory.planningPoints[trajectory.planningPoints.size() - 1].v * 0.3;
}

// 这是在车辆坐标系下进行的距离计算
/**
 * @brief 返回点距离障碍物最近距离
 * @param[IN] param 栅格障碍物，imu，路测障碍物
 * @param[IN] input 检测点
 * @param[OUT] output 距离障碍物最近距离
 
 * @cn_name: 计算点距离障碍物最近距离（一般场景）
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getMinDistanceOfPoint(const getMinDistanceOfPointParam &param, const getMinDistanceOfPointInput &input, getMinDistanceOfPointOutput &output)
{
    PlanningPoint point = input.point;
    prediction::ObjectList prediction = param.prediction;
    std::vector<infopack::ObjectsProto> objectsCmd = param.objectsCmd;
    pc::Imu imu = param.imu;

    int index = 0; //????
    double tRemainder;
    PlanningPoint predictTempPoint;
    std::vector<double> distanceFromObject;
    // index = (int)t / PREDICT_FREQUENCY;
    // tRemainder = t - index * PREDICT_FREQUENCY;
    for (auto object : prediction.object()) // 激光感知障碍物，现在是小方框集合，只计算到该点的距离
    {
        // 20230214   减少碰撞检测物体数量
        // if ((object.predictpoint(0).x() + object.w() / 2 + LASER_OFFSET_FRONT > -2) && (object.predictpoint(0).x() - object.w() / 2 + LASER_OFFSET_FRONT < MAX_FRENET_S(imu.velocity())) &&
        if ((object.predictpoint(0).x() + object.w() / 2 > -2) && (object.predictpoint(0).x() - object.w() / 2 < 15) && (object.predictpoint(0).y() - object.l() / 2) < 2 &&
            (object.predictpoint(0).y() + object.l() / 2) > -2)
        {
            predictTempPoint.x = -object.predictpoint(index).y();
            predictTempPoint.y = object.predictpoint(index).x();

            // std::cout << "predictTempPoint:" << point.x << " " << point.y << " " << predictTempPoint.x << " " << predictTempPoint.y << std::endl;

            getDistanceParam paramGD{};
            getDistanceInput inputGD{point.x, point.y, predictTempPoint.x, predictTempPoint.y};
            getDistanceOutput outputGD{0};

            getDistance(paramGD, inputGD, outputGD);

            distanceFromObject.push_back(outputGD.dis);
        }
        else
        {
            distanceFromObject.push_back(100);
        }
    }
    ////////////////////////////////////////////////////////////////////lu ce
    for (int i = 0; i < (int)objectsCmd.size(); i++)
    {
        const infopack::ObjectsProto objProto = objectsCmd[i];

        double latitudeTemp = objProto.lat();
        double longitudeTemp = objProto.lon();
        double gaussNorthTemp, gaussEastTemp;

        gaussConvertParam paramGC{};
        gaussConvertInput inputGC{longitudeTemp, latitudeTemp, gaussNorthTemp, gaussEastTemp};
        gaussConvertOutput outputGC{gaussNorthTemp, gaussEastTemp};

        gaussConvert(paramGC, inputGC, outputGC);

        gaussEastTemp = outputGC.dNorth_X;
        gaussNorthTemp = outputGC.dEast_Y;
        // gaussConvert(longitudeTemp, latitudeTemp, gaussNorthTemp, gaussEastTemp);

        // 计算4个顶点平面坐标 xy  shi cheng liang you qian
        double dXForShow = gaussEastTemp;
        double dYForShow = gaussNorthTemp;
        double dYawForShow = M_PI / 2 - objProto.yaw() * M_PI / 180.;
        double lengthTemp = objProto.len() / 100.;  // 原单位是cm，转成m
        double widthTemp = objProto.width() / 100.; // 原单位是cm，转成m

        // cout << "***objectid = " << objProto.objectid() << "," << longitudeTemp << "," << latitudeTemp << ","
        //     << dXForShow << "," << dYForShow << "," << objProto.yaw() << endl;

        double pointTemp[4][3]; // 障碍物矩形的4个角点，右上、右下、左下、左上；坐标 x，y,yaw
        pointTemp[0][0] = dXForShow + (lengthTemp / 2) * cos(dYawForShow) - (widthTemp / 2) * sin(dYawForShow);
        pointTemp[0][1] = dYForShow + (lengthTemp / 2) * sin(dYawForShow) + (widthTemp / 2) * cos(dYawForShow);
        pointTemp[1][0] = dXForShow + (lengthTemp / 2) * cos(dYawForShow) - (-widthTemp / 2) * sin(dYawForShow);
        pointTemp[1][1] = dYForShow + (lengthTemp / 2) * sin(dYawForShow) + (-widthTemp / 2) * cos(dYawForShow);
        pointTemp[2][0] = dXForShow + (-lengthTemp / 2) * cos(dYawForShow) - (-widthTemp / 2) * sin(dYawForShow);
        pointTemp[2][1] = dYForShow + (-lengthTemp / 2) * sin(dYawForShow) + (-widthTemp / 2) * cos(dYawForShow);
        pointTemp[3][0] = dXForShow + (-lengthTemp / 2) * cos(dYawForShow) - (widthTemp / 2) * sin(dYawForShow);
        pointTemp[3][1] = dYForShow + (-lengthTemp / 2) * sin(dYawForShow) + (widthTemp / 2) * cos(dYawForShow);
        pointTemp[0][2] = pointTemp[1][2] = pointTemp[2][2] = pointTemp[3][2] = dYawForShow;

        // cout<< "conner = " <<  pointTemp[0][0] <<","<<pointTemp[0][1] <<","<<
        //                                               pointTemp[1][0] <<","<<pointTemp[1][1] <<","<<
        //                                               pointTemp[2][0] <<","<<pointTemp[2][1] <<","<<
        //                                               pointTemp[3][0] <<","<<pointTemp[3][1] <<endl ;
        // 将平面直角坐标转化为车辆局部坐标，右上坐标系,--
        double cvPointTemp[4][2];
        double dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow;
        dXVehicleForShow = imu.gaussy();
        dYVehicleForShow = imu.gaussx();
        dYawVehicleForShow = M_PI / 2 - imu.yaw() * M_PI / 180.;

        for (int j = 0; j < 4; j++)
        {
            coordTran2DParam paramCT{};
            coordTran2DInput inputCT{pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow};
            coordTran2DOutput outputCT{pointTemp[j][0], pointTemp[j][1], pointTemp[j][2]};

            coordTran2D(paramCT, inputCT, outputCT);

            pointTemp[j][0] = outputCT.dX;
            pointTemp[j][1] = outputCT.dY;
            pointTemp[j][2] = outputCT.dPhi;

            // CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow); //  前左
            // std::cout << "distanceFromObject:" << j << " " << point.x << " " << point.y << " " << -pointTemp[j][1] << " " << pointTemp[j][0] << " " << getDistance(point.x, point.y,
            // -pointTemp[j][1], pointTemp[j][0]) << std::endl; // you qian 这里计算的是与障碍物四个角点的距离，这个有点不对，如果障碍物很大，而点在障碍物之内，这个计算就不能处理这种问题
            getDistanceParam paramGD{};
            getDistanceInput inputGD{point.x, point.y, -pointTemp[j][1], pointTemp[j][0]};
            getDistanceOutput outputGD{0};

            getDistance(paramGD, inputGD, outputGD);
            distanceFromObject.push_back(outputGD.dis);

            // distanceFromObject.push_back(getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]));
        }
    }

    findMinParam paramFMin{};
    findMinInput inputFMin{distanceFromObject};
    findMinOutput outputFMin{-1};

    findMin(paramFMin, inputFMin, outputFMin);

    output.minDistance = outputFMin.value;
    return;
}

/**
 * @brief 返回double数组最小值
 * @param[IN] param 无
 * @param[IN] input double数组
 * @param[OUT] output double数组最小值
 
 * @cn_name: 返回double数组最小值
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void findMin(const findMinParam &param, const findMinInput &input, findMinOutput &output)
{
    std::vector<double> array = input.array;
    // 这个地方会有一个问题，就是本身没有障碍物，结果返回一个-1，直接作距离安全判断的时候容易出现误判，需要注意对返回值进行检查
    if (array.size() == 0)
    {
        output.value = -1;
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
    output.value = flag;
    return;
}

/**
 * @brief 返回double数组最大值
 * @param[IN] param 无
 * @param[IN] input double数组
 * @param[OUT] output double数组最大值
 
 * @cn_name: 返回double数组最大值
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void findMax(const findMaxParam &param, const findMaxInput &input, findMaxOutput &output)
{
    std::vector<double> array = input.array;
    if (array.size() == 0)
    {
        output.value = -1;
        return;
    }
    double flag = array[0];
    for (int i = 1; i < array.size(); i++)
    {
        if (array[i] > flag)
        {
            flag = array[i];
        }
    }
    output.value = flag;
    return;
}

// the reference line is in the local coordinate of location point
// 生成referenceline,并将结果返回
/**
 * @brief 获取局部规划参考轨迹
 * @param[IN] param 地图
 * @param[IN] input 决策数据，当前位置
 * @param[OUT] output 参考轨迹
 
 * @cn_name: 获取参考轨迹
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getReferenceLine(const getReferenceLineParam &param, const getReferenceLineInput &input, getReferenceLineOutput &output)
{
    // std::cout << "xxxxxxxxxxxxxxxxxx xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
    //  fix road lane id
    RoadMap map = param.map;
    DecisionData decisionData = input.decisionData;
    GaussRoadPoint locationPoint = input.locationPoint;

    fixRoadLaneIndexParam paramFRLI{map};
    fixRoadLaneIndexInput inputFRLI{decisionData};
    fixRoadLaneIndexOutput outputFRLI{std::tuple<int32_t, int32_t>(0, 0)};
    fixRoadLaneIndex(paramFRLI, inputFRLI, outputFRLI);

    std::tuple<int32_t, int32_t> fixedRoadLaneIndex = outputFRLI.indexTuple;
    // std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
    int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
    int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);
    // init reference line
    ReferenceLine referenceLine;                                                                                    // reference line结果
    GaussRoadPoint currentPoint = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[decisionData.currentIndex]; // 当前位置最近路点坐标

    PlanningPoint frenetCurrentPoint;

    double deltaX = currentPoint.GaussX - locationPoint.GaussX;
    double deltaY = currentPoint.GaussY - locationPoint.GaussY;
    frenetCurrentPoint.x = (deltaY)*cos(locationPoint.yaw / 180.0 * M_PI) - (deltaX)*sin(locationPoint.yaw / 180.0 * M_PI);
    frenetCurrentPoint.y = (deltaX)*cos(locationPoint.yaw / 180.0 * M_PI) + (deltaY)*sin(locationPoint.yaw / 180.0 * M_PI);
    frenetCurrentPoint.angle = currentPoint.yaw - locationPoint.yaw;
    frenetCurrentPoint.s = 0;
    frenetCurrentPoint.l = 0;

    // std::cout << "frenet current point in get reference line: " << frenetCurrentPoint.x << "; " << frenetCurrentPoint.y << "; " << frenetCurrentPoint.angle << std::endl;

    referenceLine.referenceLinePoints.push_back(frenetCurrentPoint);
    double deltaS; // to record the distance S

    // push back reference point into reference line
    // 本lane后继点 也进行坐标转换，压入referenceLine队列。条件是s在MAX_FRENET_S 15米之内
    for (int index = decisionData.currentIndex + 1; index < map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.size(); index++)
    {
        // calculate delta s between each roadpoint and current point
        deltaS = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[index].s - currentPoint.s;
        // std::cout << "xxxxxxdeltaS "<<deltaS << std::endl;
        if (deltaS <= MAX_FRENET_S)
        {
            GaussRoadPoint referencePoint = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[index];
            PlanningPoint frenetReferencePoint;
            frenetReferencePoint.s = deltaS;
            double deltaX = referencePoint.GaussX - locationPoint.GaussX;
            double deltaY = referencePoint.GaussY - locationPoint.GaussY;
            frenetReferencePoint.x = (deltaY)*cos(locationPoint.yaw / 180.0 * M_PI) - (deltaX)*sin(locationPoint.yaw / 180.0 * M_PI);
            frenetReferencePoint.y = (deltaX)*cos(locationPoint.yaw / 180.0 * M_PI) + (deltaY)*sin(locationPoint.yaw / 180.0 * M_PI);
            // std::cout << "xxxxxxxxxxxxxxxxxx: frenetReferencePoint.x "<< frenetReferencePoint.x <<"; "<<frenetReferencePoint.y << std::endl;
            //.std::cout << "xxxxxxxxxxxxxxxxxx: "<< referencePoint.GaussX <<"; "<<referencePoint.GaussY << std::endl;

            frenetReferencePoint.angle = referencePoint.yaw - locationPoint.yaw; // s与车辆前进方向夹角，不是右前坐标系的X方向
            frenetReferencePoint.l = 0;
            referenceLine.referenceLinePoints.push_back(frenetReferencePoint);
        }
        else
        {
            output.referenceLine = referenceLine;
            return;
        }
    }
    // if the remaining distance less than MAX_FRENET_S(imu.velocity()), then check !!!the successor road lane
    // get next road id and lane id
    //??????这里出现的问题就是，只能找一条后继道路，如果后继道路很短，如十字路口的连接道路，reference line就很短
    int nextRoadIndex, nextLaneIndex;
    // nextRoadIndex = map.roads[roadIndex].successorId;

    for (int i = 0; i < map.roads.size(); i++)
    {
        if (std::get<0>(decisionData.nextId) == map.roads[i].id)
        {
            nextRoadIndex = i;
            break;
        }
    }

    // std::cout << "next lane ID: " << std::get<1>(decisionData.nextId) << std::endl;
    for (int i = 0; i < map.roads[nextRoadIndex].lanes.size(); i++)
    {

        if (std::get<1>(decisionData.nextId) == map.roads[nextRoadIndex].lanes[i].id)
        {
            nextLaneIndex = i;
            break;
        }
    }

    double accumS1 = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.back().s - currentPoint.s; // 之前段路的累计S

    getDistanceParam paramGD{};
    getDistanceInput inputGD{map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.back().GaussX, map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.back().GaussY,
                             map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.front().GaussX, map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.front().GaussY};
    getDistanceOutput outputGD{0};

    getDistance(paramGD, inputGD, outputGD);
    double accumS2 = outputGD.dis;

    // std::cout << GREEN << "yyyyyyyyyyyyyyyyyyyyyyaaaaaaaaaaaaaaaaaaaccumuS: " << accumS1 << "; " << accumS2 << std::endl;
    for (int index = 0; index < map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.size(); index++)
    {
        deltaS = map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints[index].s + accumS1 + accumS2; // 累计S
        if (deltaS <= MAX_FRENET_S)
        {
            GaussRoadPoint referencePoint = map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints[index];
            PlanningPoint frenetReferencePoint;
            frenetReferencePoint.s = deltaS;
            double deltaX = referencePoint.GaussX - locationPoint.GaussX;
            double deltaY = referencePoint.GaussY - locationPoint.GaussY;
            frenetReferencePoint.x = (deltaY)*cos(locationPoint.yaw / 180.0 * M_PI) - (deltaX)*sin(locationPoint.yaw / 180.0 * M_PI);
            frenetReferencePoint.y = (deltaX)*cos(locationPoint.yaw / 180.0 * M_PI) + (deltaY)*sin(locationPoint.yaw / 180.0 * M_PI);
            frenetReferencePoint.angle = referencePoint.yaw - locationPoint.yaw;
            frenetReferencePoint.l = 0;
            referenceLine.referenceLinePoints.push_back(frenetReferencePoint);
        }
        else
        {
            output.referenceLine = referenceLine;
            return;
        }
    }

    output.referenceLine = referenceLine;
    return;
}

// 获取车辆当前位置，在frenet坐标系中坐标值
/**
 * @brief 获取车辆当前位置在frenet坐标系中坐标值
 * @param[IN] param 无
 * @param[IN] input 决策数据，参考轨迹，当前位置
 * @param[OUT] output 更新后的决策数据
 
 * @cn_name: 获取车辆在frenet坐标系中坐标值
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void getFrenetLocationPoint(const getFrenetLocationPointParam &param, const getFrenetLocationPointInput &input, getFrenetLocationPointOutput &output)
{
    DecisionData decisionData = input.decisionData;
    ReferenceLine referenceLine = input.referenceLine;
    GaussRoadPoint locationPoint = input.locationPoint;

    PlanningPoint frenetCurrentPoint = referenceLine.referenceLinePoints.front(); // current point on location point's local coordinate
    // 车辆当前位置，在frenet坐标系下的x\y,等同于s\l,因为frenet坐标原点就是当前找到的最近路点
    decisionData.frenetLocationPoint.s = -frenetCurrentPoint.x * sin(frenetCurrentPoint.angle / 180.0 * M_PI) + (-frenetCurrentPoint.y) * cos(frenetCurrentPoint.angle / 180.0 * M_PI);
    decisionData.frenetLocationPoint.l = (-frenetCurrentPoint.y) * sin(frenetCurrentPoint.angle / 180.0 * M_PI) - (-frenetCurrentPoint.x) * cos(frenetCurrentPoint.angle / 180.0 * M_PI);
    decisionData.frenetLocationPoint.frenetAngle = frenetCurrentPoint.angle; // 但是这个角度还是frenet坐标系s与车辆前进方向夹角

    // std::cout << "frenetLocationPoint.l in getFrenetLocationPoint: " << decisionData.frenetLocationPoint.l << std::endl;
    output.decisionData = decisionData;
}

// 将frenet坐标系坐标转为自车坐标系坐标，xy为右前
/**
 * @brief 将frenet坐标系坐标转为自车坐标系坐标，xy为右前
 * @param[IN] param 无
 * @param[IN] input 坐标点sl值，初始化xy值，参考轨迹，上次搜索的索引值，本次需返回的坐标点对应的索引值
 * @param[OUT] output 坐标点xy值，坐标点对应的索引值
 
 * @cn_name: frenet坐标系转局部坐标系
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void frenet2Cartesian(const frenet2CartesianParam &param, const frenet2CartesianInput &input, frenet2CartesianOutput &output)
{
    double s = input.s;
    double l = input.l;
    double x = input.x;
    double y = input.y;
    ReferenceLine referenceLine = input.referenceLine;
    int lastIndex = input.lastIndex;
    int prevIndex = input.prevIndex;

    int indexFront = referenceLine.referenceLinePoints.size() - 1, indexBack = referenceLine.referenceLinePoints.size() - 2;
    double rels = 0.2;

    for (int i = prevIndex; i < referenceLine.referenceLinePoints.size(); i++)
    {
        if (referenceLine.referenceLinePoints[i].s >= s)
        {
            indexFront = i;
            indexBack = i - 1;
            break;
        }
    }

    if (indexBack <= 0)
    {
        indexBack = 0;
        indexFront = 1;
    }

    rels = referenceLine.referenceLinePoints[indexFront].s - referenceLine.referenceLinePoints[indexBack].s;
    // end  20230213 根据苏州地图问题修改

    double t = (s - referenceLine.referenceLinePoints[indexBack].s) / rels;

    // 路点上对应点的坐标
    double x0 = referenceLine.referenceLinePoints[indexBack].x * (1 - t) + referenceLine.referenceLinePoints[indexFront].x * t;
    double y0 = referenceLine.referenceLinePoints[indexBack].y * (1 - t) + referenceLine.referenceLinePoints[indexFront].y * t;

    // l在xy两个方向上的分解
    x = x0 - l * (referenceLine.referenceLinePoints[indexFront].y - referenceLine.referenceLinePoints[indexBack].y) / rels;
    y = y0 + l * (referenceLine.referenceLinePoints[indexFront].x - referenceLine.referenceLinePoints[indexBack].x) / rels;

    lastIndex = indexFront;

    output.x = x;
    output.y = y;
    output.lastIndex = lastIndex;
}

// 计算frenet坐标系下的bezier曲线
/**
 * @brief 在frenet坐标系中生成多条贝塞尔曲线
 * @param[IN] param 无
 * @param[IN] input 参考轨迹，车辆在frenet坐标系下坐标，初始化的轨迹向量数组
 * @param[OUT] output 轨迹向量数组
 
 * @cn_name: 在frenet坐标系中生成多条贝塞尔曲线
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: plannings
 */
void generateBezierPathListInFrenet(const generateBezierPathListInFrenetParam &param, const generateBezierPathListInFrenetInput &input, generateBezierPathListInFrenetOutput &output)
{
    ReferenceLine referenceLine = input.referenceLine;
    PlanningPoint frenetLocationPoint = input.frenetLocationPoint;
    std::vector<PlanningTrajectory> pathList = input.pathList;

    double s = referenceLine.referenceLinePoints.back().s;

    PlanningPoint startPoint, endPoint; // frenet  坐标系下起点和终点的坐标???
    startPoint = frenetLocationPoint;   // 车辆位置在frenet坐标系下坐标

    endPoint.s = s;
    endPoint.frenetAngle = 0;

    PlanningTrajectory curve;

    for (int i = 0; i < CURVE_NUM; i++)
    {
        // 终点位置l，分别向两侧递增
        if (i % 2 == 0)
        {
            endPoint.l = -(i + 1) / 2 * CURVE_DISTANCE;
        }
        else
        {
            endPoint.l = (i + 1) / 2 * CURVE_DISTANCE;
        }
        //  std:: cout << RED << "endPoint.l: " << endPoint.l << RESET <<std::endl;
        // std::cout << RED << "endPoint.s: " << endPoint.s << RESET << std::endl;
        generateBezierPathInFrenetParam paramGBPIF{};
        generateBezierPathInFrenetInput inputGBPIF{startPoint, endPoint, curve};
        generateBezierPathInFrenetOutput outputGBPIF{curve};

        generateBezierPathInFrenet(paramGBPIF, inputGBPIF, outputGBPIF);
        curve = outputGBPIF.curve;

        // generateBezierPathInFrenet(startPoint, endPoint, curve);

        // std::cout << RED << "endPoint.l: " << curve.planningPoints.back().l << RESET <<std::endl;
        // std::cout << RED << "curve.size: " << curve.planningPoints.size() << RESET << std::endl;
        pathList.push_back(curve);
        curve.planningPoints.clear();
    }
    output.pathList = pathList;
    return;
}

// 车辆前方是否有需要跟随的车辆,目前这个逻辑有点简单
/**
 * @brief 判断车辆前方是否有需要跟随的车辆
 * @param[IN] param imu，路测障碍物
 * @param[IN] input 无
 * @param[OUT] output 判断结果
 
 * @cn_name: 判断是否有需要跟随的车辆
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void hasPrecedingVehicle(const hasPrecedingVehicleParam &param, const hasPrecedingVehicleInput &input, hasPrecedingVehicleOutput &output)
{
    pc::Imu imu = param.imu;
    std::vector<infopack::ObjectsProto> objectsCmd = param.objectsCmd;

    for (auto &objProto : objectsCmd)
    // for (int i = 0; i < (int) objectsCmd.size(); i++)
    {
        if (objProto.type() == 0 && objProto.objectid() == ACC_FOLLOW_OBJECT_ID) // 这个待修改，应该是锁定车辆ID
        {
            getDistanceParam paramGD{};
            getDistanceInput inputGD{imu.gaussx(), imu.gaussy(), objProto.x(), objProto.y()};
            getDistanceOutput outputGD{0};

            getDistance(paramGD, inputGD, outputGD);
            double dist = outputGD.dis;
            // double dist = getDistance(imu.gaussx(), imu.gaussy(), objProto.x(), objProto.y()); // 两车距离
            double angle = atan2(objProto.x() - imu.gaussx(), objProto.y() - imu.gaussy()); // 两车连线夹角
            angle = int(angle * 180 / 3.14 + 360) % 360;                                    // 转角度，范围为0-360
            double yaw = int(90 - imu.yaw() + 360) % 360;
            std::cout << "+++++++++++++++++ACC begin distance " << dist << " angle " << angle << " yaw " << yaw << std::endl;
            // if (dist < ACC_BEGIN_DIST && abs(angle - yaw) < 90) // 距离较近，且前后关系正确
            if (dist < ACC_BEGIN_DIST) // 距离较近，且前后关系正确
            {
                output.flag = true;
                return;
            }
        }
    }

    // std::cout << "-------------------------ACC  NONE  " << std::endl;
    output.flag = false;
    return;
}

/**
 * @brief acc场景下，车辆目标速度计算
 * @param[IN] param imu，路测障碍物
 * @param[IN] input 前车索引，距前车距离
 * @param[OUT] output 预期速度
 
 * @cn_name: 计算车辆acc目标速度
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite>： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void cruiseController(const cruiseControllerParam &param, const cruiseControllerInput &input, cruiseControllerOutput &output)
{
    pc::Imu imu = param.imu;
    std::vector<infopack::ObjectsProto> objectsCmd = param.objectsCmd;

    int objectIndex = -1;
    for (int i = 0; i < objectsCmd.size(); i++)
    {
        if (objectsCmd[i].type() == 0 && objectsCmd[i].objectid() == ACC_FOLLOW_OBJECT_ID) // 这个待修改，应该是锁定车辆ID
        {
            objectIndex = i;
            break;
        }
    }

    if (objectIndex == -1)
    {
        output.velocity = -1;
        return;
    }

    getDistanceParam paramGD{};
    getDistanceInput inputGD{imu.gaussx(), imu.gaussy(), objectsCmd[objectIndex].x(), objectsCmd[objectIndex].y()};
    getDistanceOutput outputGD{0};

    getDistance(paramGD, inputGD, outputGD);
    double distance = outputGD.dis;

    // double distance = getDistance(imu.gaussx(), imu.gaussy(), objectsCmd[objectIndex].x(), objectsCmd[objectIndex].y()); // 两车距离
    double distanceDiff = distance - ACC_RELA_DIST;
    double speedDiff = objectsCmd[objectIndex].velocity() - imu.velocity();

    // double kDistance = 0.5, kSpeed = 0.5; // old 1 0.5
    double kDistance = 0.5, kSpeed = 0.5;
    static bool b_hasStopped = false;

    double targetSpeed = kDistance * distanceDiff + kSpeed * speedDiff + objectsCmd[objectIndex].velocity();
    std::cout << "distanceDiff " << distanceDiff << "speedDiff " << speedDiff << " objectsCmd[objectIndex].velocity()" << objectsCmd[objectIndex].velocity() << "targetSpeed" << targetSpeed
              << std::endl;

    if (targetSpeed <= 0)
        targetSpeed = 0;

    output.velocity = targetSpeed;
    return;
}

}