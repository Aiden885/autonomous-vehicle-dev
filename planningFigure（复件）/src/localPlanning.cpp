#include "localPlanning.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "interpolate.hpp"
#include <algorithm>

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

#define STOP_JUDGE_DISTANCE 2 // 20230214  减小 8

#define MAX_FRENET_S 8                      // 规划预瞄距离
#define ACC_RELA_DIST 15.0                  // ACC跟车保持距离，目前这个数值不合适，会被认为是障碍物
#define ACC_BEGIN_DIST (ACC_RELA_DIST + 30) // ACC进入跟车的距离 +5
#define ACC_FOLLOW_OBJECT_ID 2              // ACC 跟车前车ID
#define doNothing()

#define SAFE_DISTANCE 1.8

// add by shyp 20230209 formular from Fusion program
void gaussConvert(double longitude1, double latitude1, double &dNorth_X, double &dEast_Y)
{

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
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// by syp
// 2D的坐标变换公式，已知全局坐标，车辆坐标系远点在全局坐标系中的坐标，求改点在车辆坐标系中的位置
// 坐标系都为笛卡尔坐标系，xy为右上
// double dX0, double dY0 新坐标原点在旧坐标系中的位置
// double dPhi0为原有X轴在旧坐标系中的角度
void CoordTran2DForNew0INOld(double &dX, double &dY, double &dPhi, double dX0, double dY0, double dPhi0)
{

    double xTemp, yTemp;

    // 坐标平移
    xTemp = dX - dX0; // 终点转换后的坐标X
    yTemp = dY - dY0; // 终点转换后的坐标Y

    // 坐标旋转
    dPhi0 = -dPhi0;
    dX = xTemp * cos(dPhi0) - yTemp * sin(dPhi0);
    dY = xTemp * sin(dPhi0) + yTemp * cos(dPhi0);
    dPhi = dPhi + dPhi0; // 终点转换后的角度
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 2022
bool inArea(double longitudeX, double latitudeY, double targetX, double targetY)
{
    bool ret = false;
    if (std::abs(longitudeX - targetX) < STOP_JUDGE_DISTANCE && std::abs(latitudeY - targetY) < STOP_JUDGE_DISTANCE)
    {
        ret = true;
    }
    else
    {
        ret = false;
    }

    return ret;
}

double pointDistance(double x1, double x2, double y1, double y2)
{
    double distance;
    double dX;
    double dY;
    dX = std::abs(x1 - x2);
    dY = std::abs(y1 - y2);
    distance = sqrt(dX * dX + dY * dY);
    return distance;
}

// revise courseangle of roadpoint from 0 for north to 0 for east:
double courseAngleRevise(double ang)
{
    if (ang > 90)
    {
        ang = ang - 90;
    }
    else
    {
        ang = ang + 270;
    }
    return ang;
}

void loadStopPoints(const std::string fileName, std::vector<GaussRoadPoint> &rawStopPoints, std::tuple<int32_t, int32_t, int32_t> &stopPointRoadLanePointId, double offsetX, double offsetY)
{
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
}

// 新增从yaml文件读取停止位置
void loadStopPointsFromYaml(const RoadMap &map_, const std::string &fileName, std::vector<GaussRoadPoint> &rawStopPoints, std::tuple<int32_t, int32_t, int32_t> &stopPointRoadLanePointId,
                            double offsetX, double offsetY)
{

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

                    for (int i = 0; i < lane_iter.gaussRoadPoints.size(); i++)
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
}

std::tuple<int32_t, int32_t> getNextRoadLaneId(std::tuple<int32_t, int32_t> currentId, const std::vector<std::tuple<int32_t, int32_t>> &routingList)
{
    int32_t nextIndex = 1;

    static uint32_t currentIdIndex = 1;                                      // （貌似应当是从0开始？但是下面有处理）
    uint32_t pathMaxIdIndex = static_cast<uint32_t>(routingList.size()) - 1; // 这条全局路径上有多少<Road,Lane>

    if (currentIdIndex < pathMaxIdIndex)
    {
        if (currentId == routingList[currentIdIndex]) // 如果是刚刚的Road
        {
            nextIndex = currentIdIndex + 1;
        }
        else if (currentId == routingList[currentIdIndex + 1]) // 如果换了下一个<Road,Lane>
        {
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

    return routingList[nextIndex];
}

double getPlanningDistance(double velocity, double curvature)
{
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
    distance = limitPlanningDistance(distance, lastPlanningDistance);
    lastPlanningDistance = distance;
    return distance;
}

GaussRoadPoint getPlanningPoint(double distance, const GaussRoadPoint &currentPoint, const DecisionData &decisionData, double yaw, std::tuple<int32_t, int32_t> nextId, const RoadMap &map)
{
    bool b_findPlanningPoint = false;
    GaussRoadPoint planningPointCandidate;
    // std::cout << BOLDBLUE << "Planning point distance: " << distance << std::endl;
    double tempCandidateDistanceDiff = DISTANCE_RANGE; // 选择预瞄距离最合适的预瞄点
    std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
    int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
    int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);

    for (int32_t index = decisionData.currentIndex; index < map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.size(); index++)
    {
        GaussRoadPoint thisPoint = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[index];
        double distanceFound = pointDistance(currentPoint.GaussX, thisPoint.GaussX, currentPoint.GaussY, thisPoint.GaussY);
        if (distanceFound > distance - DISTANCE_RANGE && distanceFound < distance + DISTANCE_RANGE)
        {
            double planningAngle = getAngle(currentPoint.GaussX, currentPoint.GaussY, thisPoint.GaussX, thisPoint.GaussY);
            double angleDiff = getAngleDiff(yaw, planningAngle);
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
        for (int i = 0; i < map.roads.size(); i++)
        {
            if (std::get<0>(nextId) == map.roads[i].id)
            {
                roadIndex = i;
                break;
            }
        }
        for (int i = 0; i < map.roads[roadIndex].lanes.size(); i++)
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
            double distanceFound = pointDistance(currentPoint.GaussX, thisPoint.GaussX, currentPoint.GaussY, thisPoint.GaussY);
            // std::cout << "check next segment distance found: "<< distanceFound << std::endl;
            if (distanceFound > distance - DISTANCE_RANGE && distanceFound < distance + DISTANCE_RANGE)
            {
                double planningAngle = getAngle(currentPoint.GaussX, currentPoint.GaussY, thisPoint.GaussX, thisPoint.GaussY);
                double angleDiff = getAngleDiff(yaw, planningAngle);

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
    return ret;
}

double limitPlanningDistance(double targetDistance, double lastTargetDistance)
{
    // std::cout << BOLDYELLOW << "Target Planning Distan: " << targetDistance << std::endl;
    // std::cout << "Last Planning Distance: " << lastTargetDistance << std::endl;
    if (targetDistance > lastTargetDistance + 2)
    {
        targetDistance = lastTargetDistance + 2;
    }
    else if (targetDistance < lastTargetDistance - 2)
    {
        targetDistance = lastTargetDistance - 2;
    }
    return targetDistance;
}

double getAngle(double x0, double y0, double x1, double y1)
{
    double deltaY = y1 - y0;
    double deltaX = x1 - x0;
    return std::atan2(deltaY, deltaX) / M_PI * 180;
}

// range is : ang0: Yaw : 0 ~ 360; ang1: Planning Angle zhijiaozuobiao : -180 ~ 180
double getAngleDiff(double ang0, double ang1)
{
    if (ang0 > 180)
    {
        ang0 = ang0 - 360;
    }
    else
        ;
    /*
  ang0 = -ang0;
  //  double temp1 = ang0;

  ang1 = ang1 - 90;
  if (ang1 < -180)
  {
    ang1 = ang1 + 360;
  }
  else
    ;*/

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

    return ret;
}

// 在这个函数中完成了规划过程,将结果（规划曲线的x，y，angle，以及速度文件）存在decisiondata的trajectorylist中
void localPlanning(const pc::Imu &imu, double velocity, const RoadMap &map, DecisionData &decisionData, const prediction::ObjectList &predictionMsg,
                   const std::vector<std::tuple<int32_t, int32_t>> &routingList, const std::vector<GaussRoadPoint> stopPoints, const infopack::TrafficLight &trafficLight,
                   std::vector<infopack::ObjectsProto> objectsCmd)
{
    // std::cout << GREEN << std::setprecision(10) << "routing list size at first: " << routingList.size() << RESET << std::endl;
    //  局部规划初始化LocalPlanning
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << RED << "1111111111111111111111111111111111111111111111duration proces: " << duration.count() << RESET << std::endl;
    initLocalPlanning(decisionData);

    // imu定位确定的当前全局坐标
    GaussRoadPoint locationPoint;
    locationPoint.GaussX = imu.gaussx();
    locationPoint.GaussY = imu.gaussy();
    locationPoint.yaw = imu.yaw();
    std::cout << GREEN << std::setprecision(10) << "locationPoint: " << locationPoint.GaussX << ";" << locationPoint.GaussY << ";" << locationPoint.yaw << RESET << std::endl;
    std::tuple<int32_t, int32_t> nextId =
        getNextRoadLaneId(std::tuple<int32_t, int32_t>(decisionData.currentId, decisionData.currentLaneId), routingList); // 下个路段序号？？？这里只能处理后续一段路，应该考虑如果需要后续多段路时
    decisionData.nextId = nextId;

    if (!getCurrentPosition(decisionData, imu, map)) // 获取当前位置最近路点
    {
        // 如果没有获取到路点
        std::cout << RED << "get current position failed :(" << RESET << std::endl;
        stop(decisionData, velocity); // 停车
    }
    else if (!restart(predictionMsg, velocity))
    {
        std::cout << RED << "restart failed :(" << RESET << std::endl;
        stop(decisionData, velocity); // 停车
    }
    else
    {
        // std::cout << RED << "get current position success:(" << RESET << std::endl;
        //  根据地图信息，找对应的index

        // std::cout << "test for map initial 111111: "<< map.roads[2].lanes[0].id<< std::endl;
        //   std::cout << "test for map initial 111111: "<< map.roads[2].lanes[1].id<< std::endl;
        std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
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

            if (true == changeCurrentPoint(map.roads[roadIndex], map.roads[roadIndex].lanes[laneIndex], tempCurrentLaneId, tempCurrentIndex, currentPoint, isLeft))
            {
                decisionData.currentLaneId = tempCurrentLaneId;
                decisionData.currentIndex = tempCurrentIndex;
                // std::cout << BOLDCYAN << "after changeCurrentPoint: " << decisionData.currentLaneId << "; " << decisionData.currentIndex << std::endl;

                // 生成referenceline，referencelane是在道路上的点集，起点是当前位置最近的道路上的点
                ReferenceLine referenceLine = getReferenceLine(map, decisionData, locationPoint);

                // 计算当前位置在frenet坐标系中坐标
                getFrenetLocationPoint(decisionData, referenceLine, locationPoint);

                decisionData.optimalTrajectoryIndex = getTrajectoryPlanningResult(velocity, decisionData, imu, predictionMsg, map, referenceLine, trafficLight, objectsCmd); // 20220825 修改函数输入
                // std::cout << RED << "optimalTrajectoryIndex in active lane change:" << decisionData.optimalTrajectoryIndex << RESET << std::endl;
                if (decisionData.optimalTrajectoryIndex != -1)
                {
                    PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);

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
        getCurrentPosition(decisionData, imu, map);
        // 生成参考线
        ReferenceLine referenceLine = getReferenceLine(map, decisionData, locationPoint);
        std::cout << RED << "referenceLine!!!!2222222222222222222222222!: " << referenceLine.referenceLinePoints.back().s << "; " << referenceLine.referenceLinePoints.back().l << "; "
                  << referenceLine.referenceLinePoints.back().s << RESET << std::endl;

        getFrenetLocationPoint(decisionData, referenceLine, locationPoint);
        // std::cout << RED << "3333333333333333333333333!" << RESET << std::endl;
        // auto start = std::chrono::steady_clock::now();
        decisionData.optimalTrajectoryIndex = getTrajectoryPlanningResult(velocity, decisionData, imu, predictionMsg, map, referenceLine, trafficLight, objectsCmd); // 20220825 修改函数输入
        // std::cout << RED << "optimalTrajectoryIndex for no lane changing:" << decisionData.optimalTrajectoryIndex <<
        // RESET << std::endl; std::cout << "total path number" << decisionData.finalPathList.size() << std::endl;
        // std::cout << "total trajectory number" << decisionData.controlTrajectoryList.size() << std::endl;

        end = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // std::cout << RED << "22222222222222222222222222222222duration proces: " << duration.count() << RESET << std::endl;
        //  std::cout << RED << "444444444444444444444444444!   " << duration.count() << RESET << std::endl;
        //   如果直行道没有可行道路，那么考虑左右换道
        //    passive lane change
        if (decisionData.optimalTrajectoryIndex == -1)
        {
            getCurrentPosition(decisionData, imu, map);
            std::cout << RED << "enter passive lane change!" << RESET << std::endl;
            std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
            int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
            int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);

            for (int i = 0; i <= 1; i++)
            // for (int i = 0; i <= 0; i++) // only change left lane
            {
                //?????? 最后一个变量 i ^ i 永远是0吧，应该只能右换道
                if (false == changeCurrentPoint(map.roads[roadIndex], map.roads[roadIndex].lanes[laneIndex], tempCurrentLaneId, tempCurrentIndex, currentPoint, (bool)i))
                {
                    continue;
                }
                decisionData.currentLaneId = tempCurrentLaneId;
                decisionData.currentIndex = tempCurrentIndex;

                ReferenceLine referenceLine = getReferenceLine(map, decisionData, locationPoint);

                getFrenetLocationPoint(decisionData, referenceLine, locationPoint);
                decisionData.optimalTrajectoryIndex = getTrajectoryPlanningResult(velocity, decisionData, imu, predictionMsg, map, referenceLine, trafficLight, objectsCmd); // 20220825 修改函数输入
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
        stop(decisionData, velocity);
    }
    else
    {
        // 计算曲率及弯道限速
        int turnIndex = -1;
        for (int iter = 2; iter < decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() - 2; iter++)
        {
            double curvatureTemp = fabs(calculateCurvature(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter],
                                                           decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2],
                                                           decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2]));
            // std::cout <<  RED<<"?????????????????????????????????????????curvatureTemp = ????????????????????" <<        curvatureTemp <<std::endl;
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
    if (stopPointJudge(imu, stopPoints))
    {
        std::cout << "stopPointJudge(imu, stopPoints))-------------------" << std::endl;
        stop(decisionData, velocity);
    }
    PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);
    decisionData.optimalGlobalTrajectory = optimalGlobalTrajectory;

    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // std::cout << RED << "eeeeeeeeeeeeeeeeeeeeeeee   1duration proces: " << duration.count() << RESET << std::endl;

    return;
}

bool restart(const prediction::ObjectList &prediction, double velocity)
{

    // 这个函数导致避障后不能启动，暂时不用了
    return true;

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
    return restartflag;
}
// 停车
void stop(DecisionData &decisionData, double velocity)
{
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
    std::vector<double> curvePointSpeed = interp::linear(velocity, 0, stopIndex + 1);
    // std::vector<double> curvePointSpeed = interp::constant_acceleration(velocity, 0, stopIndex + 1);

    // 20230214 修改少设置一个数的问题
    // for (int i = 0; i < stopIndex - 1; i++)
    for (int i = 0; i <= stopIndex - 1; i++)
    {
        // 20230214测试直接速度设置为0
        // decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[i].v = curvePointSpeed[i + 1];
        decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[i].v = 0;
    }

    for (int i = stopIndex; i < CURVE_POINT_NUM; i++)
    {
        decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[i].v = 0;
    }

    // for (int i = 0; i < CURVE_POINT_NUM; i++)
    // {
    //   if (i < stopIndex -1)
    //   {
    //     decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[i].v = velocity / (stopIndex) * (stopIndex - i - 1);
    //   }
    //   else
    //   {
    //     decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[i].v = 0;
    //   }
    // }
}

bool stopPointJudge(const pc::Imu &imu, const std::vector<GaussRoadPoint> &stopPoints)
{
    bool ret = false;
    for (uint32_t i = 1; i < stopPoints.size(); i++)
    {
        // ting zai hou mian.
        if (inArea(imu.gaussx(), imu.gaussy(), stopPoints[i].GaussX, stopPoints[i].GaussY))
        {
            ret = true;
        }
    }
    return ret;
}

PlanningPoint pointOnCubicBezier(std::vector<PlanningPoint> cp, double t)
{
    /*
     cp在此是四個元素的陣列:
     cp[0]為起始點，或上圖中的P0
     cp[1]為第一個控制點，或上圖中的P1
     cp[2]為第二個控制點，或上圖中的P2
     cp[3]為結束點，或上圖中的P3
     t為參數值，0 <= t <= 1
     */

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

    return result;
}

// 在这个函数中，根据起点和终点，完成了一条曲线的一段的生成，
void generateBezierPathInFrenet(const PlanningPoint &startPoint, const PlanningPoint &endPoint, PlanningTrajectory &curve)
{
    double ds = endPoint.s - startPoint.s;
    double dl = endPoint.l - startPoint.l;
    double distance = sqrt(pow(ds, 2) + pow(dl, 2));

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
        bezierPoint = pointOnCubicBezier(controlPointList, i * dt); // 在这个函数里确定贝塞尔曲线每个点的位置
        // std::cout << "wwwwwwwwwwww" <<std::endl;

        resultPoint.s = bezierPoint.s + startPoint.s; // 将曲线恢复到以路点为起点的frenet坐标系
        resultPoint.l = bezierPoint.l + startPoint.l;

        // std::cout << "eeeeeeeeeeeeee" <<std::endl;
        // std::cout << CYAN << " resultPoint.s; " << resultPoint.s << RESET<<std::endl;
        // std::cout << CYAN << " resultPoint.l; " << resultPoint.l <<RESET<< std::endl;

        if (i == 0)
        {
            resultPoint.frenetAngle = startPoint.frenetAngle;
        }
        else
        {
            resultPoint.frenetAngle =
                static_cast<int32_t>(360 + atan2(curve.planningPoints[i].s - curve.planningPoints[i - 1].s, curve.planningPoints[i].l - curve.planningPoints[i - 1].l) / M_PI * 180) % 360;
        }
        curve.planningPoints.push_back(resultPoint);
        // std::cout << CYAN << "curve.points[i].angle" << curve.points[i].angle << std::endl;
    }
}

void clearPathList(std::vector<PlanningTrajectory> &pathList) { pathList.clear(); }

// 在这个函数中，主要完成了多条轨迹的生成，并作出了轨迹的选择。
int32_t getTrajectoryPlanningResult(double velocity, DecisionData &decisionData, const pc::Imu &imu, const prediction::ObjectList &predictionMsg, const RoadMap &map,
                                    const ReferenceLine &referenceLine, const infopack::TrafficLight &trafficLight, std::vector<infopack::ObjectsProto> objectsCmd) // 20220825 修改函数输入
{

    std::vector<PlanningTrajectory> frenetPathList; // frenet坐标系下规划路径

    // std::cout << RED << "decisionData.frenetLocationPoint: " << decisionData.frenetLocationPoint.l << std::endl;
    //
    generateBezierPathListInFrenet(referenceLine, decisionData.frenetLocationPoint, frenetPathList);

    // std::cout << RED << "44444444444444444444444!" << RESET << std::endl;
    //  from sl to xy
    PlanningTrajectory tempPath;
    int prevIndex = 0, lastIndex = 0;
    // std::cout << RED << "rrrrrrrrrrrrrrrrrrr"
    //           << "frenetPathList.size: " << frenetPathList.size() << std::endl;
    // std::cout << RED << "rrrrrrrrrrrrrrrrrrr"
    //           << "frenetPathList.planningpoint.size: " << frenetPathList[0].planningPoints.size() << std::endl;
    for (int i = 0; i < CURVE_NUM; i++)
    {
        for (int j = 0; j < CURVE_POINT_NUM; j++)
        {
            // std::cout << RED <<"tttttttttttttttttt"<< std::endl;
            frenet2Cartesian(frenetPathList[i].planningPoints[j].s, frenetPathList[i].planningPoints[j].l, frenetPathList[i].planningPoints[j].x, frenetPathList[i].planningPoints[j].y, referenceLine,
                             lastIndex, prevIndex);
            // std::cout << RED <<"tttttttttttttttttt2"<< std::endl;

            tempPath.planningPoints.push_back(frenetPathList[i].planningPoints[j]);
            // if (i == 0 && j < 3)
            // {
            //   std::cout << "refexi: " << j << "  " << referenceLine.referenceLinePoints[j].s << std::endl;
            //   std::cout << "refey: " << referenceLine.referenceLinePoints[j].l << std::endl;
            //   std::cout << "refes: " << referenceLine.referenceLinePoints[j].accumS << std::endl;
            //   std::cout << "rrrrrrrrrrrrrrrrrrrrrxxxxxxxxxxxxxiiiiiiiiiiii: " << frenetPathList[i].planningPoints[j].s << std::endl;
            //   std::cout << "rrrrrrrrrrrrrrrrrrrrryyyyyyyyyyyyyiiiiiiiiiiii: " << frenetPathList[i].planningPoints[j].l << std::endl;
            //   std::cout << "rrrrrrrrrrrrrrrrrrrrrxxxxxxxxxxxxx: " << referenceLine.referenceLinePoints[j].x << std::endl;
            //   std::cout << "rrrrrrrrrrrrrrrrrrrrryyyyyyyyyyyyy: " << referenceLine.referenceLinePoints[j].y << std::endl;
            // }
        } // for (int j = 0; j < CURVE_POINT_NUM; j++)

        // std::cout << "i:" << i << std::endl;
        prevIndex = 0;
        lastIndex = 0;
        decisionData.finalPathList.push_back(tempPath);
        tempPath.planningPoints.clear();
    } // for (int i = 0; i < CURVE_NUM; i++)
    frenetPathList.clear();

    // std::cout << RED << "5555555555555555555555!" << RESET << std::endl;

    generateSpeedList(velocity, decisionData, objectsCmd, imu);
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][0] << RESET << std::endl;
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][10] << RESET << std::endl;
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][20] << RESET << std::endl;
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][30] << RESET << std::endl;
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][40] << RESET << std::endl;
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][49] << RESET << std::endl;

    // std::cout << RED << "666666666666666666666666!" << RESET << std::endl;
    //  生成轨迹
    generateTrajectoryList(decisionData);

    // std::cout << RED << "777777777777777777777!" << RESET << std::endl;

    // 在规划的路径中选择一条最优路径
    int32_t optimalTrajectoryIndex = getOptimalTrajectoryIndex(decisionData, predictionMsg, trafficLight, objectsCmd, imu);
    // std::cout << "optimalTrajectoryIndex after getOptimalTrajectoryIndex: " << optimalTrajectoryIndex << std::endl;

    return optimalTrajectoryIndex;
}

// 20220826 两点间距离公式
double getDistance(double x1, double y1, double x2, double y2) { return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)); }

// 计算点到线段的距离
// 更换一种计算方法，目前这种逻辑比较复杂，且double进行相等的比较是不对的
// double pointToLineDistance(const GaussRoadPoint &startPoint, const GaussRoadPoint &stopPoint, const GaussRoadPoint &checkPoint)
// {
//   double distance = 0.0;
//   if (startPoint.GaussX == stopPoint.GaussX) //
//   {
//     // 点在线段外
//     if ((checkPoint.GaussY < startPoint.GaussY && checkPoint.GaussY < stopPoint.GaussY) || (checkPoint.GaussY > startPoint.GaussY && checkPoint.GaussY > stopPoint.GaussY))
//     {
//       distance = std::min(getDistance(checkPoint.GaussX, checkPoint.GaussY, startPoint.GaussX, startPoint.GaussY),
//                           getDistance(checkPoint.GaussX, checkPoint.GaussY, stopPoint.GaussX, stopPoint.GaussY));
//     }
//     else
//     {
//       distance = abs(checkPoint.GaussX - startPoint.GaussX);
//     }
//   }
//   else if (startPoint.GaussY == stopPoint.GaussY) // 起点终点在同一竖列
//   {
//     if ((checkPoint.GaussY > startPoint.GaussY && checkPoint.GaussY > stopPoint.GaussY) || (checkPoint.GaussY > startPoint.GaussY && checkPoint.GaussY > stopPoint.GaussY))
//     {
//       distance = std::min(getDistance(checkPoint.GaussX, checkPoint.GaussY, startPoint.GaussX, startPoint.GaussY),
//                           getDistance(checkPoint.GaussX, checkPoint.GaussY, stopPoint.GaussX, stopPoint.GaussY));
//     }
//     else
//     {
//       distance = abs(checkPoint.GaussY - startPoint.GaussY);
//     }
//   }
//   else
//   {
//     double k = (startPoint.GaussY - stopPoint.GaussY) / (startPoint.GaussX - stopPoint.GaussX); // startPoint到stopPoint的直线参数
//     double b = startPoint.GaussY - k * startPoint.GaussX;

//     double x1 = (checkPoint.GaussX + k * checkPoint.GaussY - k * b) / (k * k + 1); // 垂足位置
//     double y1 = x1 * k + b;
//     if ((x1 < startPoint.GaussX && x1 < stopPoint.GaussX) || (x1 > startPoint.GaussX && x1 > stopPoint.GaussX)) // 垂足在线段外
//     {
//       distance = std::min(getDistance(checkPoint.GaussX, checkPoint.GaussY, startPoint.GaussX, startPoint.GaussY),
//                           getDistance(checkPoint.GaussX, checkPoint.GaussY, stopPoint.GaussX, stopPoint.GaussY));
//     }
//     else
//     {
//       distance = getDistance(checkPoint.GaussX, checkPoint.GaussY, x1, y1);
//     }
//   }
//   return distance;
// }

// 计算点到线段的距离，如果这个公式是对的，相比原来的方法就是降维打击
double pointToLineDistance(const GaussRoadPoint &startPoint, const GaussRoadPoint &stopPoint, const GaussRoadPoint &checkPoint)
{

    double lengthStart2Stop = getDistance(startPoint.GaussX, startPoint.GaussY, stopPoint.GaussX, stopPoint.GaussY);
    double lengthStart2Check = getDistance(startPoint.GaussX, startPoint.GaussY, checkPoint.GaussX, checkPoint.GaussY);
    double lengthStop2Check = getDistance(stopPoint.GaussX, stopPoint.GaussY, checkPoint.GaussX, checkPoint.GaussY);

    // 垂足在线段外，距离点到线段端点的距离
    if (lengthStart2Check * lengthStart2Check > lengthStop2Check * lengthStop2Check + lengthStart2Stop * lengthStart2Stop)
        return lengthStop2Check;

    if (lengthStop2Check * lengthStop2Check > lengthStart2Check * lengthStart2Check + lengthStart2Stop * lengthStart2Stop)
        return lengthStart2Check;

    // 垂足在线段内  ，最短距离是三角形ABC以边BC的高，可通过海伦公式先求出面积，再求出高得到答案。
    double l = (lengthStart2Stop + lengthStart2Check + lengthStop2Check) / 2;
    double s = sqrt(l * (l - lengthStart2Stop) * (l - lengthStart2Check) * (l - lengthStop2Check));
    // std::cout<< "new pointToLineDistance++++++ "<< 2*s/lengthStart2Stop <<std:: endl;
    return 2 * s / lengthStart2Stop;
}

double angleRegulate(double angleInput)
{
    if (angleInput > 180.0)
    {
        angleInput = 360.0 - angleInput;
    }
    return angleInput;
}

// 车辆当前位置最近的路点，保存在 decisionData.currentId 、 decisionData.currentLaneId 、  decisionData.currentIndex 中
bool getCurrentPosition(DecisionData &decisionData, const pc::Imu &imu, const RoadMap &map)
{
    if (map.roads.size() == 0)
    {
        std::cout << " Map size false" << std::endl;
        return false;
    } // map数据检查

    // 本函数中使用的临时变量
    // std::cout << BOLDBLUE << "location gaussx: " << imu.gaussx() << "; location y: " << imu.gaussy() << "; location yaw: " << imu.yaw() << RESET << std::endl;
    int32_t lastRoadId, lastLaneId, lastIndex; // 分别是上次的RoadID，上次的LaneID，上次的路点Index，用于加快程序运行速度
    lastRoadId = decisionData.currentId;
    lastLaneId = decisionData.currentLaneId;
    lastIndex = decisionData.currentIndex;

    //   std::cout << "20221011 decisiondata currentID in get current position: " <<
    //   decisionData.currentId << "; " << decisionData.currentLaneId <<"," <<decisionData.currentIndex << std::endl;

    // 找对应的roadindex 和laneindex
    int32_t lastRoadIndex = -1, lastLaneIndex = -1;
    for (int i = 0; i < map.roads.size(); i++)
    {
        if (lastRoadId == map.roads[i].id)
        {
            lastRoadIndex = i;
            break;
        }
    }

    if (lastRoadIndex != -1)
    {
        for (int i = 0; i < map.roads[lastRoadIndex].lanes.size(); i++)
        {
            if (lastLaneId == map.roads[lastRoadIndex].lanes[i].id)
            {
                lastLaneIndex = i;
                break;
            }
        }
    }

    // std::cout << "20221011 decisiondata currentID in get current position:  lastRoadIndex = " <<
    //  lastRoadIndex << "; " << lastRoadIndex << std::endl;

    float minDis = 0x7f7f7f7f;
    int32_t targetRoad = -1;
    int32_t targetLane = -1;
    int32_t targetIndex = -1;

    if (lastRoadIndex != -1 && lastLaneIndex != -1)
    {
        // 开始执行
        // 先找上次的点附近的这条车道上的点
        // std::cout << BOLDBLUE << "20221011 (2): " << map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints.size() << RESET << std::endl;
        for (int i = lastIndex; i < map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints.size(); i++)
        {
            // std::cout << BLUE << "***** i 1 =  " << i << std::endl;
            GaussRoadPoint thisPoint = map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints[i]; // 询问的这个点
            // std::cout << BOLDBLUE << "20221011 (3): " << thisPoint.yaw << std::endl;
            //  考虑航向角
            if (angleRegulate(abs(thisPoint.yaw - imu.yaw())) > POINT_ANGLE_THRESHOLD)
            {
                // std::cout << BLUE << "***** This point x " << thisPoint.GaussX << " y " << thisPoint.GaussY << std::endl;
                // std::cout << BLUE << "***** This point yaw " << thisPoint.yaw << " IMU yaw " << imu.yaw() << " regulate " << angleRegulate(abs(thisPoint.yaw - imu.yaw())) << RESET << std::endl;
                // std::cout << BLUE << "***** Angle wrong 1 " << abs(thisPoint.yaw - imu.yaw()) << RESET << std::endl;
                break;
            }
            // 考虑距离
            // std::cout << BLUE << "123456789 imu.yaw" << imu.yaw() << RESET << std::endl;
            double dis = getDistance(imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY); // 计算该点与自车距离
            if (dis <= minDis)                                                                        //????这个minDis很大，意味着第一次遇到的点就一定是满足条件的点吧
            {
                minDis = dis;
                targetIndex = i;
            }
            else
            {
                break; // 当计算的dis第一次大于minDis的时候，就退出循环了，对于一些曲线道路。这个是否合理？？？
            }
        }

        if (targetIndex != -1 && minDis <= POINT_DIS_THRESHOLD_ON_CURRENT_LANE) // 如果找到了，就直接退出
        {
            decisionData.currentId = lastRoadId;
            decisionData.currentLaneId = lastLaneId;
            decisionData.currentIndex = targetIndex;
            // std::cout << " *** hhhhhhhhhhhhhh Flag 1 ***" << std::endl;
            return true;
        }

        // 再找这条车道上的其他点
        targetIndex = -1; // 20230227 添加赋值
        for (int i = 0; i < map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints.size(); i++)
        {
            GaussRoadPoint thisPoint = map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints[i]; // 询问的这个点
            // 考虑航向角
            //  std::cout << BLUE << "***** i 2 =  " << i << std::endl;
            if (angleRegulate(abs(thisPoint.yaw - imu.yaw())) > POINT_ANGLE_THRESHOLD)
            {
                // std::cout << BLUE << "***** This point x " << thisPoint.GaussX << " y " << thisPoint.GaussY << std::endl;
                // std::cout << BLUE << "***** This point yaw " << thisPoint.yaw << " IMU yaw " << imu.yaw() << " regulate " << angleRegulate(abs(thisPoint.yaw - imu.yaw())) << RESET << std::endl;
                // std::cout << BLUE << "***** Angle wrong 2 " << abs(thisPoint.yaw - imu.yaw()) << RESET << std::endl;
                continue;
            }
            // 考虑距离
            double dis = getDistance(imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY); // 计算该点与自车距离
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
            return true;
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
            if (angleRegulate(abs(thisPoint.yaw - imu.yaw())) > POINT_ANGLE_THRESHOLD)
            {
                // std::cout << BLUE << "***** This point x " << thisPoint.GaussX << " y " << thisPoint.GaussY << std::endl;
                // std::cout << BLUE << "***** This point yaw " << thisPoint.yaw << " IMU yaw " << imu.yaw() << " regulate " << angleRegulate(abs(thisPoint.yaw - imu.yaw())) << RESET << std::endl;
                // std::cout << BLUE << "***** Angle wrong 2 " << abs(thisPoint.yaw - imu.yaw()) << RESET << std::endl;
                continue;
            }
            // 考虑距离
            double dis = getDistance(imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY); // 计算该点与自车距离
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
            return true;
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
            float dis =
                pointToLineDistance(map.roads[i].lanes[0].gaussRoadPoints[int((j - 1) * (pointsNum - 1) / 5)], map.roads[i].lanes[0].gaussRoadPoints[int(j * (pointsNum - 1) / 5)], currentPoint);
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

                if (angleRegulate(abs(thisPoint.yaw - imu.yaw())) > POINT_ANGLE_THRESHOLD)
                {
                    // std::cout << BLUE << "***** This point x " << thisPoint.GaussX << " y " << thisPoint.GaussY << std::endl;
                    // std::cout << BLUE << "***** This point yaw " << thisPoint.yaw << " IMU yaw " << imu.yaw() << " regulate " << angleRegulate(abs(thisPoint.yaw - imu.yaw())) << RESET << std::endl;
                    // std::cout << BLUE << "***** Angle wrong 3  " << abs(thisPoint.yaw - imu.yaw()) << RESET << std::endl;
                    continue;
                }
                // 考虑距离
                float dis = getDistance(imu.gaussx(), imu.gaussy(), thisPoint.GaussX, thisPoint.GaussY);
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
            return true;
        }
        // std::cout << " *** Flag 3 ***" << std::endl;
        heap.pop(); // 把堆顶弹出,便于下次使用
    }
    // std::cout << " *** Flag 4 ***" << std::endl;
    // std::cout << " *** hhhhhhhhhhhhhh Flag 4 ***" << std::endl;
    return false;
}

// 生成速度采样后，多种 轨迹上每个点的速度
void generateSpeedList(double velocity, DecisionData &decisionData, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu)
{
    double currentSpeed = velocity;
    if (currentSpeed < 0.1)
        currentSpeed = 1.2;

    double dDesireSpeed = DESIRED_SPEED;
    // 处理ACC跟车
    for (int i = 0; i < decisionData.controlTrajectoryList.size(); i++)
    {
        if (hasPrecedingVehicle(imu, objectsCmd))
        {
            // 计算acc速度
            dDesireSpeed = std::min(cruiseController(imu, objectsCmd), DESIRED_SPEED);
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
    std::vector<double> curvePointSpeed = interp::constant_acceleration(currentSpeed, dDesireSpeed, CURVE_POINT_NUM, 2.0);
    decisionData.speedList.push_back(curvePointSpeed);

    // 20230212 简化速度规划，直接只留一条 begin
    // for (int i = 0; i < END_SPEED_NUM; i++)
    // {
    //   double endPointSpeed = 0.0;
    //   if (i % 2 == 0)
    //   {
    //     endPointSpeed = DESIRED_SPEED + (double)i * SPEED_RANGE / (END_SPEED_NUM - 1);
    //   }
    //   else
    //   {
    //     endPointSpeed = DESIRED_SPEED - (double)(i + 1) * SPEED_RANGE / (END_SPEED_NUM - 1);
    //   }

    //   endPointSpeedList.push_back(endPointSpeed);

    //   // std::cout << "222end point speed: " << endPointSpeed << std::endl;
    // }
    // endPointSpeedList.push_back(DESIRED_SPEED);

    // 20230212 简化速度规划，直接只留一条 end

    // for (int i = 0; i < END_SPEED_NUM; i++)
    // {
    //   // std::vector<double> curvePointSpeed = interp::linear(currentSpeed, endPointSpeedList[i], CURVE_POINT_NUM);
    //   std::vector<double> curvePointSpeed = interp::constant_acceleration(currentSpeed, endPointSpeedList[i], CURVE_POINT_NUM, 2.0);

    //   // double endPointSpeedFirst = endPointSpeedList[i];
    //   // double endPointSpeedSecond = endPointSpeedList[i];
    //   // std::vector<double> curvePointSpeed;
    //   // curvePointSpeed.clear();
    //   // curvePointSpeed.reserve(CURVE_POINT_NUM);
    //   // for (int k = 0; k < CURVE_POINT_NUM * 2; k++)
    //   // {
    //   //   if (k < CURVE_POINT_NUM)
    //   //   {
    //   //     curvePointSpeed.push_back(currentSpeed + (endPointSpeedFirst - currentSpeed) * ((double)k / (CURVE_POINT_NUM - 1.0)));
    //   //   }
    //   //   else
    //   //   {
    //   //     curvePointSpeed.push_back(endPointSpeedFirst);
    //   //   }
    //   // }
    //   decisionData.speedList.push_back(curvePointSpeed);
    // }
}

// 生成轨迹采样结果,将之前生成的轨迹和速度合并在controlTrajectoryList中
void generateTrajectoryList(DecisionData &decisionData)
{
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
}

// 处理红绿灯，以前包含避障的判断，现在都注释掉了,好多地方不对，重写了
//  int processTrafficLight(DecisionData &decisionData, const prediction::ObjectList &prediction, const infopack::TrafficLight &trafficLight)
//  {

//   bool traffic_light_active = trafficLight.active();
//   infopack::TrafficLight_State traffic_light_state = trafficLight.state();
//   if (traffic_light_active)
//   {
//     for (int i = 0; i < END_SPEED_NUM; i++)
//     {
//       if (traffic_light_state == infopack::TrafficLight::RED_LIGHT ||
//           (traffic_light_state == infopack::TrafficLight::YELLOW_LIGHT &&
//            !ableToPassYellowLight(decisionData.controlTrajectoryList[i], trafficLight.remaining_time(), trafficLight.lane_length_before_intersection())))
//       {
//         //if (trajectoryCollisionCheck(decisionData.controlTrajectoryList[i], prediction))
//         {
//           decisionData.feasibleTrajectoryIndexList.push_back(i);

//         }
//       }
//       else
//       {
//         for (int i = 0; i < END_SPEED_NUM; i++)
//         {
//           if (trajectoryCollisionCheck(decisionData.controlTrajectoryList[i], prediction))
//           {
//             decisionData.feasibleTrajectoryIndexList.push_back(i);
//           }
//         }
//         return 2;
//       }
//     }

//     // std::cout << RED << "feasibleTrajectoryIndexList number: " << decisionData.feasibleTrajectoryIndexList.size() << RESET << std::endl;

//     if (decisionData.feasibleTrajectoryIndexList.size() == 0)
//     {
//       return 0;
//     }
//     else
//     {
//       // TODO: stop at the stop line
//       return 1;
//     }
//   } // if (traffic_light_active)--------------------
//   else
//   {
//     for (int i = 0; i < decisionData.controlTrajectoryList.size(); i++)
//     {
//       // 20230215 不做碰撞检测
//       // if (trajectoryCollisionCheck(decisionData.controlTrajectoryList[i], prediction))
//       {
//         decisionData.feasibleTrajectoryIndexList.push_back(i);
//       }
//     }

//     // std::cout << RED << "feasibleTrajectoryIndexList number: " << decisionData.feasibleTrajectoryIndexList.size() << RESET << std::endl;

//     return 2;
//   }
// }

// 处理红绿灯，以前包含避障的判断，现在都注释掉了,好多地方不对，重写了
int processTrafficLight(DecisionData &decisionData, const prediction::ObjectList &prediction, const infopack::TrafficLight &trafficLight)
{

    bool traffic_light_active = trafficLight.active();
    infopack::TrafficLight_State traffic_light_state = trafficLight.state();
    if (traffic_light_active)
    {
        if (traffic_light_state == infopack::TrafficLight::RED_LIGHT || traffic_light_state == infopack::TrafficLight::YELLOW_LIGHT) // 原来算法计算了一下黄灯时，以当前速度可以通过的可行性
        {
            return 0; // 红灯或黄灯，就没有可行走的轨迹了
        }
        else
        {
            for (int i = 0; i < decisionData.controlTrajectoryList.size(); i++)
            {
                decisionData.feasibleTrajectoryIndexList.push_back(i);
            }

            return 2; // 绿灯所有轨迹都可选
        }
    }    // if (traffic_light_active)
    else // 红绿灯无效下，所有轨迹都可选
    {
        for (int i = 0; i < decisionData.controlTrajectoryList.size(); i++)
        {
            decisionData.feasibleTrajectoryIndexList.push_back(i);
        }

        // std::cout << RED << "feasibleTrajectoryIndexList number: " << decisionData.feasibleTrajectoryIndexList.size() << RESET << std::endl;

        return 2;
    }
}

// 20230215 备份
//  int32_t getOptimalTrajectoryIndex(DecisionData &decisionData, const prediction::ObjectList &prediction, const infopack::TrafficLight &trafficLight)
//  {
//    decisionData.feasibleTrajectoryIndexList.clear();
//    int status = foo(decisionData, prediction, trafficLight);
//    if (status != 2)
//    {
//      return -1;
//    }

//   // double maxDistance = assessTrajectory(decisionData.controlTrajectoryList[decisionData.feasibleTrajectoryIndexList[0]], prediction);
//   double maxDistance = -1;
//   double tempDistance;
//   int32_t index = 0;
//   for (int i = 0; i < decisionData.feasibleTrajectoryIndexList.size(); i++)
//   {
//     tempDistance = assessTrajectory(decisionData.controlTrajectoryList[decisionData.feasibleTrajectoryIndexList[i]], prediction);
//    // std::cout << YELLOW << i << "    TEMPDISTANCE:    " << tempDistance << RESET << std::endl;
//       //20230215  不要最安全的，尽量靠近
//     if (tempDistance > maxDistance)
//     {
//       index = i;
//       maxDistance = tempDistance;
//     }
//   }

//   return decisionData.feasibleTrajectoryIndexList[index];
// }

int32_t getOptimalTrajectoryIndex(DecisionData &decisionData, const prediction::ObjectList &prediction, const infopack::TrafficLight &trafficLight, std::vector<infopack::ObjectsProto> objectsCmd,
                                  const pc::Imu &imu)
{
    decisionData.feasibleTrajectoryIndexList.clear();

    // 处理交通信号灯
    int status = processTrafficLight(decisionData, prediction, trafficLight);
    if (status != 2)
    {
        return -1;
    }

    // double maxDistance = assessTrajectory(decisionData.controlTrajectoryList[decisionData.feasibleTrajectoryIndexList[0]], prediction);
    double maxDistance = -1;
    double tempDistance;
    int32_t index = -1;
    for (int i = 0; i < decisionData.feasibleTrajectoryIndexList.size(); i++)
    {
        tempDistance = assessTrajectory(decisionData.controlTrajectoryList[decisionData.feasibleTrajectoryIndexList[i]], prediction, objectsCmd, imu);
        // std::cout << YELLOW << i << "    TEMPDISTANCE:    " << tempDistance << RESET << std::endl;
        // 20230215  不要最安全的，尽量靠近
        if (tempDistance > SAFE_DISTANCE) // 认为不会碰撞的安全距离
        {

            index = i;
            break; // 找到第一条可行路线就退出
        }
    }

    // return 0; /// no!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //  std::cout << "index " <<index << "  size " <<  decisionData.feasibleTrajectoryIndexList.size() <<std::endl;
    //  20230218 解决无感知速度为零的问题，待验证
    if (index == -1)
    {
        if (prediction.object_size() != 0)
            return -1;
        else
            return 0;
    }

    return decisionData.feasibleTrajectoryIndexList[index];
}

// true表示可行，false表示碰撞
bool trajectoryCollisionCheck(PlanningTrajectory &trajectory, const prediction::ObjectList &prediction)
{
    double t = 0;
    int index = 0;
    double tRemainder;
    PlanningPoint predictTempPoint;

    // std::cout << RED << "prediction " << prediction.object(0).w() << ";" <<  prediction.object(0).l() << std::endl;
    // std::cout << CYAN << "trajsize " << trajectory.trajectoryPoints.size() << ";" << std::endl;
    for (int i = 0; i < trajectory.planningPoints.size() - 1; i++)
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
            if (!pointCollisionCheck(trajectory.planningPoints[i], predictTempPoint, object.w() / 2, object.l() / 2))
            {
                return false;
            }
        }
        // std::cout<< GREEN << "tra" << (*trajectoryPointIt).y <<RESET<< std::endl;
        double len = getDistance(trajectory.planningPoints[i].x, trajectory.planningPoints[i].y, trajectory.planningPoints[i + 1].x, trajectory.planningPoints[i + 1].y);
        double vMean = (trajectory.planningPoints[i].v + trajectory.planningPoints[i + 1].v) / 2;
        if (vMean <= 0.1)
        {
            return true;
        }
        t += len / vMean;
        if (t >= 1.95)
        {
            return true;
        }
    }
    // delete predictTempPoint;
    return true;
}

bool ableToPassYellowLight(PlanningTrajectory &trajectory, double remaining_time, double lane_length_before_intersection)
{
    double t = 0;
    double passDistance = 0;
    for (int i = 0; i < trajectory.planningPoints.size() - 1; i++)
    {
        t = 0;
        PlanningPoint currentPoint = trajectory.planningPoints[i];
        PlanningPoint nextPoint = trajectory.planningPoints[i + 1];
        double len = getDistance(currentPoint.x, currentPoint.y, nextPoint.x, nextPoint.y);
        double vMean = (currentPoint.v + nextPoint.v) / 2;
        t += len / vMean;
        passDistance += len;
        if (t <= remaining_time)
        {
            if (passDistance > lane_length_before_intersection + 2) // rear bumper passes the stop line
            {
                return true;
            }
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool pointCollisionCheck(const PlanningPoint &trajectoryPoint, PlanningPoint &predictPoint, double w, double l)
{
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
            if (!boundaryCollisionCheck(vehPoint[j], objPoint[i], vehPoint[j + 1], objPoint[i + 1]))
            {
                return false;
            }
        }
        if (!innerCollisionCheck(vehPoint[0], vehPoint[1], vehPoint[2], vehPoint[3], objPoint[i]))
        {
            return false;
        }
    }
    return true;
}

bool boundaryCollisionCheck(const PlanningPoint &p0, const PlanningPoint &p1, const PlanningPoint &p2, const PlanningPoint &p3)
{
    double z0, z1, z2, z3;
    z0 = ((p1.x - p0.x) * (p2.y - p1.y) - (p2.x - p1.x) * (p1.y - p0.y));
    z1 = ((p2.x - p1.x) * (p3.y - p2.y) - (p3.x - p2.x) * (p2.y - p1.y));
    z2 = ((p3.x - p2.x) * (p0.y - p3.y) - (p0.x - p3.x) * (p3.y - p2.y));
    z3 = ((p0.x - p3.x) * (p1.y - p0.y) - (p1.x - p0.x) * (p0.y - p3.y));

    if ((z0 * z1 > 0) && (z1 * z2 > 0) && (z2 * z3 > 0))
        return false;
    else
        return true;
}

bool innerCollisionCheck(const PlanningPoint &p0, const PlanningPoint &p1, const PlanningPoint &p2, const PlanningPoint &p3, const PlanningPoint &obj)
{
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
        return false;
    return true;
}

// 通过当前预瞄点，计算换道后的预瞄点？？？是预瞄点还是当前点都可以用这个函数
// 找另一条车道上，与当前点最近的点
bool changeCurrentPoint(const Road &road, const Lane &curLane, int &laneId, int &pointIndex, GaussRoadPoint &currentPoint, bool isLeft)
{
    int changeId;
    if (isLeft)
    {
        if (curLane.leftLaneId.size() == 0)
            return false;
        else
            changeId = curLane.id + 1; // 从右往左。laneID 增加 1
    }
    else
    {
        if (curLane.rightLaneId.size() == 0)
            return false;
        else
            changeId = curLane.id - 1;
    }

    Lane changeLane;
    for (auto lane : road.lanes)
    {
        if (lane.id == changeId)
        {
            changeLane = lane;
            laneId = changeId;
            break;
        }
    }

    double disCur = 10000;
    double disTemp;
    GaussRoadPoint *currentPointTemp = new GaussRoadPoint(); // 这个指针没有delete，呜呜
    int index = 0;
    int indexTemp = 0;
    for (auto roadPoint : changeLane.gaussRoadPoints)
    {
        disTemp = sqrt(pow(roadPoint.GaussX - currentPoint.GaussX, 2) + pow(roadPoint.GaussY - currentPoint.GaussY, 2));
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
    return true;
};

PlanningTrajectory changeLocalToGlobal(PlanningTrajectory &trajectory, GaussRoadPoint gaussRoadPoint)
{
    double dX, dY, dTheta;
    PlanningPoint globalTrajectoryPoint;
    PlanningTrajectory globalTrajectory;
    double len, pointTheta;
    dX = gaussRoadPoint.GaussX - trajectory.planningPoints[0].x;
    dY = gaussRoadPoint.GaussY - trajectory.planningPoints[0].y;
    dTheta = gaussRoadPoint.yaw - trajectory.planningPoints[0].angle;

    static double cumul_s = 0;
    for (int i = 0; i < trajectory.planningPoints.size(); i++)
    {
        len = sqrt(pow(trajectory.planningPoints[i].x - trajectory.planningPoints[0].x, 2) + pow(trajectory.planningPoints[i].y - trajectory.planningPoints[0].y, 2));
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
        if (i == 0 || i == trajectory.planningPoints.size() - 1)
        {
            globalTrajectoryPoint.curvature = 0;
        }
        else
        {
            globalTrajectoryPoint.curvature = calculateCurvature(trajectory.planningPoints[i], trajectory.planningPoints[i - 1], trajectory.planningPoints[i + 1]);
        }
        globalTrajectoryPoint.s = cumul_s;
        if (i < trajectory.planningPoints.size() - 1)
        {
            cumul_s += getDistance(trajectory.planningPoints[i + 1].x, trajectory.planningPoints[i + 1].y, trajectory.planningPoints[i].x, trajectory.planningPoints[i].y);
        }
        else
        {
            cumul_s = 0;
        }
        globalTrajectory.planningPoints.push_back(globalTrajectoryPoint);
    }
    return globalTrajectory;
}

// 局部规划初始化
void initLocalPlanning(DecisionData &decisionData)
{
    // 轨迹初始化
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
}

// 返回decisionDate中roadID 和laneID的索引
std::tuple<int32_t, int32_t> fixRoadLaneIndex(const DecisionData &decisionData, const RoadMap &map)
{
    // 找对应的index
    //  std::cout << "ROAD MAP SIZE: " << map.roads.size() << std::endl;
    //  std::cout << "ROAD CURRENT ID AND LANE ID: " << decisionData.currentId << "; " << decisionData.currentLaneId << std::endl;
    int32_t roadIndex = -1;
    int32_t laneIndex = -1;
    // std::cout << "LANE MAP SIZE: " <<  map.roads.size() << std::endl;
    for (int i = 0; i < map.roads.size(); i++)
    {
        // std::cout << "ROAD MAP ID : " << map.roads[i].id << std::endl;
        if (decisionData.currentId == map.roads[i].id)
        {
            roadIndex = i;
            break;
        }
    }

    // std::cout << "inter  ******* FIXED ROAD CURRENT ID AND LANE ID: " << roadIndex << "; " << laneIndex << std::endl;

    // 20231013
    if (roadIndex != -1)
    {
        for (int i = 0; i < map.roads[roadIndex].lanes.size(); i++)
        {
            // std::cout << "LANE MAP ID: " << map.roads[roadIndex].lanes[i].id << std::endl;
            if (decisionData.currentLaneId == map.roads[roadIndex].lanes[i].id)
            {
                laneIndex = i;
                break;
            }
        }
    }

    std::cout << " " << roadIndex << "; " << laneIndex << std::endl;
    return std::tuple<int32_t, int32_t>(roadIndex, laneIndex);
}

double calculateCurvature(const PlanningPoint &p0, const PlanningPoint &p1, const PlanningPoint &p2)
{
    double a1, a2, b1, b2, x, y;
    a1 = p1.x - p0.x;
    a2 = p2.x - p0.x;
    b1 = p1.y - p0.y;
    b2 = p2.y - p0.y;
    if (a1 * b2 == a2 * b1)
        return 0;
    else
    {
        y = (a1 * a1 * a2 + b1 * b1 * a2 - a2 * a2 * a1 - b2 * b2 * a1) / (a2 * b1 - a1 * b2);
        x = (a1 * a1 * b2 + b1 * b1 * b2 - a2 * a2 * b1 - b2 * b2 * b1) / (a1 * b2 - a2 * b1);
        if (x > 100 || y > 100)
            return 0;
        return 2 / sqrt(pow(x, 2) + pow(y, 2));
    }
}

// 20230214   backup
//  double assessTrajectory(PlanningTrajectory &trajectory, const prediction::ObjectList &prediction)
//  {
//    std::vector<double> distanceFromPointToObject;
//    double t = 0;
//    for (int i = 0; i < trajectory.planningPoints.size(); i++)
//    {
//      distanceFromPointToObject.push_back(getMinDistanceOfPoint(trajectory.planningPoints[i], prediction, t));
//      double len = getDistance(trajectory.planningPoints[i].x, trajectory.planningPoints[i].y, trajectory.planningPoints[i + 1].x, trajectory.planningPoints[i + 1].y);
//      double vMean = (trajectory.planningPoints[i].v + trajectory.planningPoints[i + 1].v) / 2;
//      if (vMean < 0.1)
//      {
//        vMean = 0.1;
//      }
//      t += len / vMean;
//      if (t >= 1.95)
//      {
//        // std::cout << "hh0 " << trajectory.trajectoryPoints[trajectory.trajectoryPoints.size()-1].v << std::endl;
//        return findMin(distanceFromPointToObject) + trajectory.planningPoints[trajectory.planningPoints.size() - 1].v * 0.06;
//        // return trajectory.planningPoints[trajectory.planningPoints.size() - 1].v * 0.6;
//      }
//    }
//    // std::cout << "hh1 " << trajectory.trajectoryPoints[trajectory.trajectoryPoints.size()-1].v << std::endl;
//    return findMin(distanceFromPointToObject) + trajectory.planningPoints[trajectory.planningPoints.size() - 1].v * 0.03;
//    // return trajectory.planningPoints[trajectory.planningPoints.size() - 1].v * 0.3;
//  }

// 优选估计，目前主要是与障碍物距离为判断条件，之前还结合速度等信息
double assessTrajectory(PlanningTrajectory &trajectory, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu)
{

    // std::cout<<"assessTrajectory----------------------start"<<std::endl;
    std::vector<double> distanceFromPointToObject;
    double minDisLength = 10;

    for (int i = 0; i < trajectory.planningPoints.size(); i++)
    {

        distanceFromPointToObject.push_back(getMinDistanceOfPoint(trajectory.planningPoints[i], prediction, objectsCmd, imu));
        // std::cout<<  YELLOW << i << "  prediction size  " << prediction.object_size()<< " X "<< prediction.object(0).predictpoint(0).x()<<
        //  " Y "<< prediction.object(0).predictpoint(0).y() <<RESET << std::endl;
        // std::cout<<  YELLOW << i << "  X  " << trajectory.planningPoints[i].x << " Y "<< trajectory.planningPoints[i].y <<RESET << std::endl;
        // std::cout << YELLOW << i << "    getMinDistanceOfPoint:    " << getMinDistanceOfPoint(trajectory.planningPoints[i], prediction) << RESET << std::endl;
    }
    // std::cout << "hh1 " << trajectory.trajectoryPoints[trajectory.trajectoryPoints.size()-1].v << std::endl;

    return findMin(distanceFromPointToObject);
    // return trajectory.planningPoints[trajectory.planningPoints.size() - 1].v * 0.3;
}

// double getMinDistanceOfPoint(const PlanningPoint &point, const prediction::ObjectList &prediction, const double &t)
// {
//   int index;
//   double tRemainder;
//   PlanningPoint predictTempPoint;
//   std::vector<double> distanceFromObject;
//   index = (int)t / PREDICT_FREQUENCY;
//   tRemainder = t - index * PREDICT_FREQUENCY;
//   for (auto object : prediction.object())
//   {
//     // 20230214   减少碰撞检测物体数量
//     // if ((object.predictpoint(0).x() + object.w() / 2 > -2) && (object.predictpoint(0).x() - object.w() / 2 < 15) &&
//     //    (object.predictpoint(0).y() - object.l() / 2) < 4 && (object.predictpoint(0).y() + object.l() / 2) > -4)
//     {
//       predictTempPoint.x = object.predictpoint(index).x() * (1 - tRemainder) + object.predictpoint(index + 1).x() * tRemainder;
//       predictTempPoint.y = object.predictpoint(index).y() * (1 - tRemainder) + object.predictpoint(index + 1).y() * tRemainder;
//       distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x + object.w() / 2, predictTempPoint.y + object.l() / 2));
//       distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x + object.w() / 2, predictTempPoint.y - object.l() / 2));
//       distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x - object.w() / 2, predictTempPoint.y + object.l() / 2));
//       distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x - object.w() / 2, predictTempPoint.y - object.l() / 2));
//     }
//     // else
//     // {
//     //   distanceFromObject.push_back(100);
//     // }
//   }
//   return findMin(distanceFromObject);
// }

double getMinDistanceOfPoint(const PlanningPoint &point, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu)
{
    int index = 0; //????
    double tRemainder;
    PlanningPoint predictTempPoint;
    std::vector<double> distanceFromObject;
    // index = (int)t / PREDICT_FREQUENCY;
    // tRemainder = t - index * PREDICT_FREQUENCY;
    for (auto object : prediction.object()) // jiguang ganzhi wuti
    {
        // 20230214   减少碰撞检测物体数量
        if ((object.predictpoint(0).x() + object.w() / 2 > -2) && (object.predictpoint(0).x() - object.w() / 2 < 15) && (object.predictpoint(0).y() - object.l() / 2) < 2 &&
            (object.predictpoint(0).y() + object.l() / 2) > -2)
        {
            predictTempPoint.x = -object.predictpoint(index).y();
            predictTempPoint.y = object.predictpoint(index).x();

            // std::cout << "predictTempPoint:" << point.x << " " << point.y << " " << predictTempPoint.x << " " << predictTempPoint.y << std::endl;
            distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x, predictTempPoint.y));
            //     distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x + object.l() / 2, predictTempPoint.y + object.w() / 2));
            //     distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x + object.l() / 2, predictTempPoint.y - object.w() / 2));
            //     distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x - object.l() / 2, predictTempPoint.y + object.w() / 2));
            //     distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x - object.l() / 2, predictTempPoint.y - object.w() / 2));
            //
            // luce ganzhi cheliang
        }
        else
        {
            distanceFromObject.push_back(100);
        }
    }
    ////////////////////////////////////////////////////////////////////lu ce
    for (int i = 0; i < objectsCmd.size(); i++)
    {
        const infopack::ObjectsProto objProto = objectsCmd[i];

        // 转换xy经纬度为平面坐标
        double latitudeTemp = objProto.lat();
        double longitudeTemp = objProto.lon();
        double gaussNorthTemp, gaussEastTemp;
        gaussConvert(longitudeTemp, latitudeTemp, gaussNorthTemp, gaussEastTemp);

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
            CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow,
                                    dYawVehicleForShow); // qian zuo
                                                         // std::cout << "distanceFromObject:" << j << " " << point.x << " " << point.y << " " << -pointTemp[j][1] << " " << pointTemp[j][0] << " " <<
                                                         // getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]) << std::endl; // you qian
            distanceFromObject.push_back(getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]));
        }
    }

    return findMin(distanceFromObject);
}

double findMin(const std::vector<double> &array)
{
    if (array.size() == 0)
        return -1;
    double flag = array[0];
    for (int i = 1; i < array.size(); i++)
    {
        if (array[i] < flag)
        {
            flag = array[i];
        }
    }
    return flag;
}

double findMax(const std::vector<double> &array)
{
    if (array.size() == 0)
        return -1;
    double flag = array[0];
    for (int i = 1; i < array.size(); i++)
    {
        if (array[i] > flag)
        {
            flag = array[i];
        }
    }
    return flag;
}

// the reference line is in the local coordinate of location point
// 生成referenceline,并将结果返回
ReferenceLine getReferenceLine(const RoadMap &map, const DecisionData &decisionData, GaussRoadPoint locationPoint)
{
    // std::cout << "xxxxxxxxxxxxxxxxxx xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
    //  fix road lane id
    std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
    int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
    int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);
    // init reference line
    ReferenceLine referenceLine;                                                                                    // reference line结果
    GaussRoadPoint currentPoint = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[decisionData.currentIndex]; // 当前位置最近路点坐标

    // std::cout << "current point in get reference line: " << std::setprecision(10)<<  currentPoint.GaussX << "; " << currentPoint.GaussY << "; " << currentPoint.yaw << std::endl;
    // std::cout << "location point in get reference line: " << std::setprecision(10)<< locationPoint.GaussX << "; " << locationPoint.GaussY << "; " << locationPoint.yaw << std::endl;

    // frenetCurrentPoint should be type of planningpoint
    // GaussRoadPoint frenetCurrentPoint = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[decisionData.currentIndex];

    // 当前点转换到车辆坐标系，？？？这个坐标系是右前
    // 并计算frenet坐标系s，压入referenceline队列
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
            //  std::cout << "xxxxxxxxxxxxxxxxxx: "<< currentPoint.GaussX <<"; "<<currentPoint.GaussY << std::endl;

            frenetReferencePoint.angle = referencePoint.yaw - locationPoint.yaw; // s与车辆前进方向夹角，不是右前坐标系的X方向
            frenetReferencePoint.l = 0;
            referenceLine.referenceLinePoints.push_back(frenetReferencePoint);
        }
        else
        {
            return referenceLine;
        }
    }
    // if the remaining distance less than MAX_FRENET_S, then check !!!the successor road lane
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
    // calculate the distance between the first point of next road segment and current point
    // std::cout << GREEN << "yyyyyyyyyyyyyyyyyyyyyy: " << std::endl;
    // std::cout << GREEN << "roadIndex: " << roadIndex << "; " << laneIndex << "; " << nextRoadIndex << "; " << nextLaneIndex << std::endl;
    //
    double accumS1 = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.back().s - currentPoint.s; // 上一段路的累计S
    double accumS2 =
        getDistance(map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.back().GaussX, map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.back().GaussY,
                    map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.front().GaussX, map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.front().GaussY); // 两段路首位之间的距离
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
            return referenceLine;
        }
    }
    return referenceLine;
}

// 获取车辆当前位置，在frenet坐标系中坐标值
void getFrenetLocationPoint(DecisionData &decisionData, const ReferenceLine &referenceLine, GaussRoadPoint locationPoint)
{
    PlanningPoint frenetCurrentPoint = referenceLine.referenceLinePoints.front(); // current point on location point's local coordinate
    // 车辆当前位置，在frenet坐标系下的x\y,等同于s\l,因为frenet坐标原点就是当前找到的最近路点
    decisionData.frenetLocationPoint.s = -frenetCurrentPoint.x * sin(frenetCurrentPoint.angle / 180.0 * M_PI) + (-frenetCurrentPoint.y) * cos(frenetCurrentPoint.angle / 180.0 * M_PI);
    decisionData.frenetLocationPoint.l = (-frenetCurrentPoint.y) * sin(frenetCurrentPoint.angle / 180.0 * M_PI) - (-frenetCurrentPoint.x) * cos(frenetCurrentPoint.angle / 180.0 * M_PI);
    decisionData.frenetLocationPoint.frenetAngle = frenetCurrentPoint.angle; // 但是这个角度还是frenet坐标系s与车辆前进方向夹角

    // std::cout << "frenetLocationPoint.l in getFrenetLocationPoint: " << decisionData.frenetLocationPoint.l << std::endl;
}

// 将frenet坐标系坐标转为自车坐标系坐标，xy为右前
void frenet2Cartesian(const double &s, const double &l, double &x, double &y, const ReferenceLine &referenceLine, int &lastIndex, const int &prevIndex)
{
    // std::cout << RED << "SSSSSSSSSSSSSSSSS: " << s << std::endl;
    // std::cout << RED << "SSSSSSSSSSSSSSSSS front: " << referenceLine.referenceLinePoints.front().s<< "; " << referenceLine.referenceLinePoints.back().s<< std::endl;

    // if (s < referenceLine.referenceLinePoints.front().s || s > referenceLine.referenceLinePoints.back().s){
    //   std::cout << "frenet2Cartesian input error" << std::endl;
    //   return;
    // }

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

    // 20230213 根据苏州地图问题修改，这里看着确实不合理呀，为什么要有这样的判断
    //  if (referenceLine.referenceLinePoints[indexFront].s - referenceLine.referenceLinePoints[indexBack].s < 0.1)
    //  {
    //    rels = referenceLine.referenceLinePoints[indexFront].s - referenceLine.referenceLinePoints[indexBack].s;
    //  }
    //  else
    //  {
    //    rels = 0.2;
    //  }
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
}

// 计算frenet坐标系下的bezier曲线
void generateBezierPathListInFrenet(const ReferenceLine &referenceLine, const PlanningPoint &frenetLocationPoint, std::vector<PlanningTrajectory> &pathList)
{
    double s = referenceLine.referenceLinePoints.back().s;

    PlanningPoint startPoint, endPoint; // frenet  坐标系下起点和终点的坐标???
    // startPoint.s = frenetLocationPoint.s;
    // startPoint.l = frenetLocationPoint.l;
    // std:: cout << RED << "frenetLocationPoint.l: " << startPoint.l << std::endl;
    // std:: cout << RED << "referenceLine.referenceLinePoints.front().yaw: " << referenceLine.referenceLinePoints.front().yaw << std::endl;
    // startPoint.frenetAngle = 90 - referenceLine.referenceLinePoints.front().yaw;
    startPoint = frenetLocationPoint; // 车辆位置在frenet坐标系下坐标
    // std::cout << RED << "startPoint " << startPoint.s << "startPoint frenet angle " << startPoint.frenetAngle << std::endl;
    // std::cout << RED << "startPoint " << startPoint.l << "startPoint frenet angle " << startPoint.frenetAngle << std::endl;

    // std::cout << RED << "angleeeeeeeeeeeeeeeeeeeeeeeeeeeeeee: " << startPoint.frenetAngle << std::endl;
    endPoint.s = s;
    endPoint.frenetAngle = 0;
    // std::cout << RED << "endPoint " << endPoint.s << std::endl;

    // std::cout << RED << "referenceLine.referenceLinePoints.back().yaw: " << referenceLine.referenceLinePoints.back().frenetAngle << std::endl;

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
        generateBezierPathInFrenet(startPoint, endPoint, curve);

        // std::cout << RED << "endPoint.l: " << curve.planningPoints.back().l << RESET <<std::endl;
        // std:: cout << RED << "curve.size: " << curve.planningPoints.size() << RESET <<std::endl;
        pathList.push_back(curve);
        curve.planningPoints.clear();
    }

    // for (int i = 0; i < CURVE_NUM; i++)
    // {
    //   if (i % 2 == 0)
    //   {
    //     firstEndPointList[i].x = firstEndPoint.x + (i + 1) / 2 * CURVE_DISTANCE * sin(firstTheta / 180 * M_PI);
    //     firstEndPointList[i].y = firstEndPoint.y + (i + 1) / 2 * CURVE_DISTANCE * cos(firstTheta / 180 * M_PI);
    //     firstEndPointList[i].angle = firstEndPoint.angle;
    //   }
    //   else
    //   {
    //     firstEndPointList[i].x = firstEndPoint.x - (i + 1) / 2 * CURVE_DISTANCE * sin(firstTheta / 180 * M_PI);
    //     firstEndPointList[i].y = firstEndPoint.y - (i + 1) / 2 * CURVE_DISTANCE * cos(firstTheta / 180 * M_PI);
    //     firstEndPointList[i].angle = firstEndPoint.angle;
    //   }
    //   firstEndPointList.push_back(firstEndPointList[i]);
    // }
}

// bool calculateTrajectoryCost(const std::vector<Trajectory> &trajectoryList, const prediction::ObjectList &prediction, Trajectory &selectTrajectory)
// {
//   int centerIndex = (CURVE_NUM - 1)/2;
//   std::vector<double> costList;
//   std::vector<double> trajectoryDistanceList;
//   std::vector<double> tempDistanceList;
//   std::vector<Trajectory> feasibleTrajectoryList;
//   Point predictPoint;
//   bool collisionFlag = false;

//   for (int i = 0; i < trajectoryList.size(); i++)
//   {
//     for (int j = 0; j < trajectoryList[i].trajectoryPoints.size(); j++)
//     {
//       for (auto object : prediction.object())
//       {
//         predictPoint.x = object.predictpoint(0).x();
//         predictPoint.y = object.predictpoint(0).y();
//         if (!pointCollisionCheck(trajectoryList[i].trajectoryPoints[j], predictPoint, object.w() / 2, object.l() / 2))
//         {
//           collisionFlag = true;
//           break;
//         }
//       }
//       if (!collisionFlag)
//       {
//         tempDistanceList.push_back(getMinDistanceOfPoint(trajectoryList[i].trajectoryPoints[j], prediction));
//       }
//     }
//     if (!collisionFlag)
//     {
//       trajectoryDistanceList.push_back(findMin(tempDistanceList));
//       costList.push_back(fabs(i - centerIndex)/centerIndex + 2*CURVE_DISTANCE/(trajectoryDistanceList.back() + 0.01));
//       feasibleTrajectoryList.push_back(trajectoryList[i]);
//     }
//     collisionFlag = false;
//     tempDistanceList.clear();
//   }

//   if (feasibleTrajectoryList.size() == 0)
//   {
//     return false;
//   }

//   int minIndex = 0;
//   int minCost = costList[0];
//   for (int i = 0; i < trajectoryList.size(); i++)
//   {
//     if (costList[i] < minCost)
//     {
//       minIndex = i;
//       minCost = costList[i];
//     }
//   }

//   selectTrajectory = feasibleTrajectoryList[minIndex];
//   return true;
// }

// double speedModel(const double &distance, const double &maxspeed = 10, const double &minspeed = 0, const double &d1 = 4.5, const double &d2 = 0.5)
// {
//   if (distance < 0)
//   {
//     std::cout << "input error" << std::endl;
//     return minspeed;
//   }

//   if (distance < d2)
//   {
//     return minspeed;
//   }
//   else if (distance > d1)
//   {
//     return maxspeed;
//   }
//   else
//   {
//     return minspeed + (maxspeed - minspeed) * (distance -d2) / (d1 - d2);
//   }
// }

// void initSpeedForTrajectory(Trajectory &trajectory, const prediction::ObjectList &prediction)
// {
//   int i = 0;
//   double t = 0;
//   double distance;
//   bool stopFlag = false;
//   for (i = 0; i < trajectory.trajectoryPoints.size() - 1; i++)
//   {
//     distance = getMinDistanceOfPoint(trajectory.trajectoryPoints[i], prediction, t);
//     trajectory.trajectoryPoints[i].v = speedModel(distance);
//     if (trajectory.trajectoryPoints[i].v > 0)
//     {
//       t += getDistance(trajectory.trajectoryPoints[i].x,trajectory.trajectoryPoints[i].y,trajectory.trajectoryPoints[i+1].x,trajectory.trajectoryPoints[i+1].y) / trajectory.trajectoryPoints[i].v;
//     }
//     else
//     {
//       stopFlag = true;
//       break;
//     }
//   }
//   if (stopFlag)
//   {
//     for (; i < trajectory.trajectoryPoints.size(); i++)
//     {
//       trajectory.trajectoryPoints[i].v = 0;
//     }
//   }
//   else
//   {
//     trajectory.trajectoryPoints[i].v = trajectory.trajectoryPoints[i-1].v;
//   }
// }

// 车辆前方是否有需要跟随的车辆,目前这个逻辑有点简单
bool hasPrecedingVehicle(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> objectsCmd)
{
    return false;

    for (int i = 0; i < objectsCmd.size(); i++)
    {
        if (objectsCmd[i].type() == 0 && objectsCmd[i].objectid() == ACC_FOLLOW_OBJECT_ID) // 这个待修改，应该是锁定车辆ID
        {
            double dist = getDistance(imu.gaussx(), imu.gaussy(), objectsCmd[i].x(), objectsCmd[i].y()); // 两车距离
            double angle = atan2(objectsCmd[i].x() - imu.gaussx(), objectsCmd[i].y() - imu.gaussy());    // 两车连线夹角
            angle = int(angle * 180 / 3.14 + 360) % 360;                                                 // 转角度，范围为0-360
            double yaw = int(90 - imu.yaw() + 360) % 360;
            std::cout << "+++++++++++++++++ACC begin distance " << dist << " angle " << angle << " yaw " << yaw << std::endl;
            // if (dist < ACC_BEGIN_DIST && abs(angle - yaw) < 90) // 距离较近，且前后关系正确
            if (dist < ACC_BEGIN_DIST) // 距离较近，且前后关系正确
            {

                return true;
            }
        }
    }

    // std::cout << "-------------------------ACC  NONE  " << std::endl;
    return false;
}

double cruiseController(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> objectsCmd)
{

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
        return -1;

    double distance = getDistance(imu.gaussx(), imu.gaussy(), objectsCmd[objectIndex].x(), objectsCmd[objectIndex].y()); // 两车距离
    double distanceDiff = distance - ACC_RELA_DIST;
    double speedDiff = objectsCmd[objectIndex].velocity() - imu.velocity();

    // double kDistance = 0.5, kSpeed = 0.5; // old 1 0.5
    double kDistance = 0.5, kSpeed = 0.5;
    static bool b_hasStopped = false;

    double targetSpeed = kDistance * distanceDiff + kSpeed * speedDiff + objectsCmd[objectIndex].velocity();
    std::cout << "distanceDiff " << distanceDiff << "speedDiff " << speedDiff << " objectsCmd[objectIndex].velocity()" << objectsCmd[objectIndex].velocity() << "targetSpeed" << targetSpeed
              << std::endl;

    // 从decision-functon.cpp中拷贝过来，暂时不知道啥用处，注释掉了
    //  if (iovData.vehicleSpeed < 0.1)
    //  {
    //    if (distanceDiff > 0.7)
    //      targetSpeed = 0.7;
    //    else
    //      targetSpeed = 0;
    //  }

    // std::cout << "[ACC]   PV  speed: " << iovData.vehicleSpeed << std::endl;
    // std::cout << "[ACC] inter dist.: " << distance << std::endl;
    // std::cout << GREEN << "[ACC] dist. error: " << RESET << distanceDiff << std::endl;
    // std::cout << GREEN << "[ACC] speed error: " << RESET << speedDiff << std::endl;
    // std::cout << "[ACC] 1st  target: " << targetSpeed << std::endl;
    // if (targetSpeed < 0.5)
    // {
    //   targetSpeed = 0;
    //   b_hasStopped = true;
    // }
    // else if (targetSpeed >= 0.5 && targetSpeed < 1.8)
    // {
    //   if (true == b_hasStopped)
    //   {
    //     targetSpeed = 0.0;
    //   }
    //   else
    //   {
    //     //  	  targetSpeed = 1.0;
    //   }
    // }
    // else
    // {
    //   b_hasStopped = false;
    // }
    // if (targetSpeed > 8)
    // {
    //   targetSpeed = 8;
    //   std::cout << "! Top Cruise Speed Limited" << std::endl;
    // }
    // if (distance < ACC_MINIMAL_DISTANCE)
    // {
    //   targetSpeed = 0;
    //   b_hasStopped = true;
    // }
    // else
    //   ;
    // // std::cout << "[CV] targetSpeed = " << targetSpeed << " m/s\n";
    // return targetSpeed;

    if (targetSpeed <= 0)
        targetSpeed = 0;
    return targetSpeed;
}
