#include "localPlanning.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "interpolate.hpp"
#include "ztgeographycoordinatetransform.h"
#include <algorithm>

#include "mapMsg/TrafficLight.h"

// add by shyp 20230209 formular from Fusion program
void gaussConvert(double longitude1, double latitude1, double &dNorth_X, double &dEast_Y)
{
    ZtGeographyCoordinateTransform ztTranTemp;
    ztTranTemp.BL2XY(latitude1, longitude1, dEast_Y, dNorth_X);

    // double a = 6378137.0;

    // double e2 = 0.0066943799013;

    // double latitude2Rad = (M_PI / 180.0) * latitude1;

    // int beltNo = int((longitude1 + 1.5) / 3.0);
    // int L = beltNo * 3;
    // double l0 = longitude1 - L;
    // double tsin = sin(latitude2Rad);
    // double tcos = cos(latitude2Rad);
    // double t = tan(latitude2Rad);
    // double m = (M_PI / 180.0) * l0 * tcos;
    // double et2 = e2 * pow(tcos, 2);
    // double et3 = e2 * pow(tsin, 2);
    // double X = 111132.9558 * latitude1 - 16038.6496 * sin(2 * latitude2Rad) + 16.8607 * sin(4 * latitude2Rad) - 0.0220 * sin(6 * latitude2Rad);
    // double N = a / sqrt(1 - et3);

    // dNorth_X = X + N * t * (0.5 * pow(m, 2) + (5.0 - pow(t, 2) + 9.0 * et2 + 4 * pow(et2, 2)) * pow(m, 4) / 24.0 + (61.0 - 58.0 * pow(t, 2) + pow(t, 4)) * pow(m, 6) / 720.0);
    // dEast_Y = 500000 + N * (m + (1.0 - pow(t, 2) + et2) * pow(m, 3) / 6.0 + (5.0 - 18.0 * pow(t, 2) + pow(t, 4) + 14.0 * et2 - 58.0 * et2 * pow(t, 2)) * pow(m, 5) / 120.0);
    // //   std::cout << BOLDRED << "x1: " << x1 << std::endl;
    // //   std::cout << BOLDRED << "y1: " << y1 << std::endl;
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
    // std::cout << "inArea" << std::setprecision(10) << longitudeX << " " << latitudeY << " " << targetX << " " << targetY << std::endl;
    bool ret = false;
    if (std::abs(longitudeX - targetX) < STOP_JUDGE_DISTANCE && std::abs(latitudeY - targetY) < STOP_JUDGE_DISTANCE - 2)
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

void loadStopPoints(const std::string fileName, std::vector<GaussRoadPoint> &rawStopPoints, std::tuple<int32_t, int32_t, int32_t> &stopPointRoadLanePointId,
                    double offsetX, double offsetY)
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
            ss >> roadPoint.GaussX >> roadPoint.GaussY >> roadPoint.yaw >> std::get<0>(stopPointRoadLanePointId) >>
                std::get<1>(stopPointRoadLanePointId) >> std::get<2>(stopPointRoadLanePointId);
            roadPoint.GaussX += offsetX;
            roadPoint.GaussY += offsetY;
            rawStopPoints.push_back(roadPoint);
        }
        else
            ;
    }
    fs.close();
}

bool loadMapFilePathName(const std::string yamlFileName, std::string &mapFilePathName) // 从yaml文件加载地图文件全路径文件名
{
    try
    {
        YAML::LoadFile(yamlFileName); // this is rvalue
    }
    catch (YAML::BadFile &bfe)
    {
        std::cout << BOLDRED << "yaml file is not exist!" << std::endl;
        std::cout << "    " << bfe.what() << RESET << std::endl;
        return false;
    }

    YAML::Node yaml_node = YAML::LoadFile(yamlFileName);

    try
    {
        yaml_node["roadMapPathName"].as<std::string>(); // this is rvalue
    }
    catch (YAML::TypedBadConversion<std::string> &tbce_double)
    {
        std::cout << BOLDRED << "roadMapPathName is not exist!" << std::endl;
        std::cout << "    " << tbce_double.what() << RESET << std::endl;
        return false;
    }

    mapFilePathName = yaml_node["roadMapPathName"].as<std::string>();
    std::cout << "mapFilePathName" << mapFilePathName << std::endl;
    if (mapFilePathName.empty())
        return false;

    return true;
}

// 新增从yaml文件读取停止位置
void loadStopPointsFromYaml(const RoadMap &map_, const std::string &fileName, std::vector<GaussRoadPoint> &rawStopPoints,
                            std::tuple<int32_t, int32_t, int32_t> &stopPointRoadLanePointId, double offsetX, double offsetY)
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
                    auto target_index = static_cast<unsigned int>(
                        static_cast<unsigned int>(lane_iter.gaussRoadPoints.size()) * approximate_location);
                    std::cout << "this is the \'real\' index: " << target_index << std::endl;

                    if (target_index > 0)
                        target_index -= 1; // make 'zeroth' equals to first.
                    std::get<2>(stopPointRoadLanePointId) =
                        static_cast<int>(target_index) + 1; // start from one instead of zero
                    std::get<0>(stopPointRoadLanePointId) = road_id;
                    std::get<1>(stopPointRoadLanePointId) = lane_id;

                    for (int i = 0; i < (int)lane_iter.gaussRoadPoints.size(); i++)
                    {
                        if (i == target_index)
                        {
                            std::cout << "stopPoint's gaussX: " << std::fixed << std::setprecision(3)
                                      << lane_iter.gaussRoadPoints.at(i).GaussX
                                      << " stopPoint's gaussY: "
                                      << lane_iter.gaussRoadPoints.at(i).GaussY
                                      << " stopPoint's yaw: "
                                      << lane_iter.gaussRoadPoints.at(i).yaw << std::endl;
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
        std::cout << BOLDRED << "sorry, the value of roadIdToStop or laneIdToStop is illegal!"
                  << RESET << std::endl;
        exit(0);
    }
}

// 看不懂，为了处理什么样的问题，进行了如此复杂的处理
std::tuple<int32_t, int32_t> getNextRoadLaneId(std::tuple<int32_t, int32_t> currentId, const std::vector<std::tuple<int32_t, int32_t>> &routingList)
{
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
    return routingList[nextIndex];
}

std::vector<std::tuple<int32_t, int32_t>> getNextRoadLaneIdList(std::tuple<int32_t, int32_t> currentId, const std::vector<std::tuple<int32_t, int32_t>> &routingList)
{
    // 从全局规划中，找未走过的道路
    // 有可能包含已经走完的道路，从头开始找第一次与当前roidID相一致的路，把之前的删了
    // 不包含当road lane
    std::vector<std::tuple<int32_t, int32_t>> resultList;
    bool bFindfinishRoadIDIndexTemp = false; // 找到同名road lane标志
    for (int i = 0; i < (int)routingList.size(); i++)
    {
        resultList.push_back(routingList[i]);
        if (!bFindfinishRoadIDIndexTemp)
        {
            if (routingList[i] == currentId)
            {
                bFindfinishRoadIDIndexTemp = true;
                resultList.clear();    // 删掉之前的road
                resultList.reserve(0); //
            }
        }
    }

    return resultList;
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

GaussRoadPoint getPlanningPoint(double distance, const GaussRoadPoint &currentPoint, const DecisionData &decisionData,
                                double yaw, std::tuple<int32_t, int32_t> nextId, const RoadMap &map)
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

// 备份，改动太多了，没法改了，新写函数见下
//  // 在这个函数中完成了规划过程,将结果（规划曲线的x，y，angle，以及速度文件）存在decisiondata的trajectorylist中
//  // void localPlanning(const pc::Imu &imu, double velocity, const RoadMap &map, DecisionData &decisionData,
//  //                    const prediction::ObjectList &predictionMsg, const std::vector<std::tuple<int32_t, int32_t>> &routingList,
//  //                    const std::vector<GaussRoadPoint> stopPoints, const infopack::TrafficLight &trafficLight, std::vector<infopack::ObjectsProto> objectsCmd)
//  void localPlanning(const pc::Imu &imu, double velocity, const RoadMap &map, DecisionData &decisionData,
//                     const prediction::ObjectList &predictionMsg,std::vector<RoutingList>  & routingListVector   ,
//                     const std::vector<GaussRoadPoint> stopPoints, const infopack::TrafficLight &trafficLight, const std::vector<infopack::ObjectsProto> & objectsCmd)

// {

//     // std::cout << GREEN << std::setprecision(10) << "routing list size at first: " << routingList.size() << RESET << std::endl;
//     //  局部规划初始化LocalPlanning
//     auto start = std::chrono::steady_clock::now();
//     auto end = std::chrono::steady_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//     //std::cout << RED << "1111111111111111111111111111111111111111111111duration proces: " << duration.count() << RESET << std::endl;
//     //std::cout << decisionData.currentId << std::endl;
//     //std::cout << decisionData.controlTrajectoryList.size() << std::endl;
//     initLocalPlanning(decisionData);
//     //std::cout << RED << "2222222222222222222222222222222222222222222222222222 " << RESET << std::endl;

//     // imu定位确定的当前全局坐标
//     GaussRoadPoint locationPoint;
//     locationPoint.GaussX = imu.gaussx();
//     locationPoint.GaussY = imu.gaussy();
//     locationPoint.yaw = imu.yaw();
//     // std::cout << GREEN << std::setprecision(10) << "locationPoint: " << locationPoint.GaussX << ";" << locationPoint.GaussY << ";" << locationPoint.yaw << RESET << std::endl;
//     //不再使用nextID
//     //std::tuple<int32_t, int32_t> nextId = getNextRoadLaneId(std::tuple<int32_t, int32_t>(decisionData.currentId, decisionData.currentLaneId), routingList); // 下个路段序号？？？这里只能处理后续一段路，应该考虑如果需要后续多段路时
//     //decisionData.nextId = nextId;

//     //再当前道路和其他备选道路中选择一条最有的道路。
//    // std::vector<std::tuple<int32_t, int32_t>> bestRoutingList = SelectBestRoutingList(routingListVector  );

//     //获取后续路段的road lane 序列
//     decisionData.nextIdList.clear();
//     decisionData.nextIdList.reserve(0);
//     //decisionData.nextIdList =  getNextRoadLaneIdList(std::tuple<int32_t, int32_t>(decisionData.currentId, decisionData.currentLaneId), routingList);
//      decisionData.nextIdList =  getNextRoadLaneIdList(std::tuple<int32_t, int32_t>(decisionData.currentId, decisionData.currentLaneId), routingListVector[0].routingList);

//     if (!getCurrentPosition(decisionData, imu, map)) // 获取当前位置最近路点
//     {
//         // 如果没有获取到路点
//         std::cout << RED << "get current position failed :(" << RESET << std::endl;
//         stop(decisionData, velocity); // 停车
//     }
//     else if (!restart(predictionMsg, velocity))
//     {
//         std::cout << RED << "restart failed :(" << RESET << std::endl;
//         stop(decisionData, velocity); // 停车
//     }
//     else // 获取当前位置最近路点成功
//     {
//         // std::cout << RED << "get current position success:(" << RESET << std::endl;
//         //  根据地图信息，找对应的index

//         // std::cout << "test for map initial 111111: "<< map.roads[2].lanes[0].id<< std::endl;
//         //   std::cout << "test for map initial 111111: "<< map.roads[2].lanes[1].id<< std::endl;
//         std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
//         int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
//         int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);
//         // std::cout << "decision data current ID*************: " << decisionData.currentId << ";" << decisionData.currentLaneId << std::endl;
//         // std::cout << "Road Lane ID in local planning: " << roadIndex << "; " << laneIndex << std::endl;
//         GaussRoadPoint currentPoint = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[decisionData.currentIndex];//车道上的当前位置最近点
//         // std::cout << GREEN << std::setprecision(10) << "currentPoint: " << currentPoint.GaussX << ";" << currentPoint.GaussY << ";" << currentPoint.yaw << RESET << std::endl;
//         // std::cout << YELLOW << "routing list size: " << routingList.size() << RESET << std::endl;

//         // std::cout << "decision data current ID: " << decisionData.currentId << ";" << decisionData.currentLaneId << std::endl;
//         // std::cout << "decision data next ID: " << std::get<0>(nextId) << ";" << std::get<1>(nextId) << std::endl;

//         // 先判断全局规划是否要求换道/以及是否能换道
//         //  active lane change
//         int tempCurrentLaneId, tempCurrentIndex;
//         int currentLaneId = decisionData.currentLaneId;
//         int currentIndex = decisionData.currentIndex;

//         // 全局规划中，要求换道
//         if (decisionData.nextIdList.size() > 0 &&
//         decisionData.currentId == std::get<0>(decisionData.nextIdList[0]) && decisionData.currentLaneId != std::get<1>(decisionData.nextIdList[0]))
//         {
//             std::cout << RED << "enter active lane change!" << RESET << std::endl;
//             bool isLeft = std::get<1>(decisionData.nextIdList[0]) > decisionData.currentLaneId;//往左lane编号增加

//             if (true == changeCurrentPoint(map.roads[roadIndex], map.roads[roadIndex].lanes[laneIndex], tempCurrentLaneId, tempCurrentIndex, currentPoint, isLeft))
//             {
//                  if(GetOptimalGlobalTrajectory(imu,  map, decisionData, predictionMsg, trafficLight, objectsCmd, tempCurrentLaneId, tempCurrentIndex))
//                  {
//                     PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);
//                     decisionData.optimalGlobalTrajectory = optimalGlobalTrajectory;
//                     return;
//                  }

//             }

//         } // if (decisionData.currentId == std::get<0>(decisionData.nextId) && decisionData.currentLaneId != std::get<1>(decisionData.nextId))

//         tempCurrentLaneId = currentLaneId;
//         tempCurrentIndex =currentIndex;

//         // 全局规划没有要求换道，在当前道路上继续前行
//         //???是否会出现主动换道过不去，一直沿着当前路往前走，直到做到本lane的终点，卡死在本lane的终点出，再也拐不过去了
//         std::cout << RED << "enter no  lane change, go along current road!" << RESET << std::endl;
//         getCurrentPosition(decisionData, imu, map);
//         // 生成参考线
//     //     std::vector<PlanningPoint>().swap(decisionData.referenceLine.referenceLinePoints);
//     //    decisionData.referenceLine.referenceLinePoints.reserve(0);
//     //    decisionData.referenceLine = getReferenceLine(map, decisionData, locationPoint);

//     //     // std::cout<<  std::setprecision(10)<<" referenceLine.size =" <<  decisionData.referenceLine.referenceLinePoints.size()
//     //     // <<" referenceLine.s=" <<  decisionData.referenceLine.referenceLinePoints.back().s
//     //     // <<   " referenceLine.l= " << decisionData.referenceLine.referenceLinePoints.back().l
//     //     // <<" gaussX = "  <<  decisionData.referenceLine.referenceLinePoints.front().gaussX//.back().gaussX
//     //     // << " gaussY= "<< decisionData.referenceLine.referenceLinePoints.back().gaussY
//     //     // << " roadID= " << decisionData.referenceLine.referenceLinePoints.back().roadID
//     //     // <<" laneID ="  << decisionData.referenceLine.referenceLinePoints.back().laneID
//     //     // << " pointID =" << decisionData.referenceLine.referenceLinePoints.back().pointID <<std::endl;

//     //     getFrenetLocationPoint(decisionData,  decisionData.referenceLine, locationPoint);

//     //     decisionData.optimalTrajectoryIndex = getTrajectoryPlanningResult(velocity, decisionData, imu, predictionMsg, map,   trafficLight, objectsCmd); // 20220825 修改函数输入
//     //     // std::cout << RED << "optimalTrajectoryIndex for no lane changing:" << decisionData.optimalTrajectoryIndex << RESET << std::endl;
//     //     // std::cout << "total path number" << decisionData.finalPathList.size() << std::endl;
//     //     // std::cout << "total trajectory number" << decisionData.controlTrajectoryList.size() << std::endl;

//    GetOptimalGlobalTrajectory(imu,  map, decisionData, predictionMsg, trafficLight, objectsCmd, tempCurrentLaneId, tempCurrentIndex);
//    // std::cout << RED << "GetOptimalGlobalTrajectory---------------------- " << decisionData.optimalTrajectoryIndex<<RESET << std::endl;

//         //   如果直行道没有可行道路，那么考虑左右换道
//         //    passive lane change
//         if (decisionData.optimalTrajectoryIndex == -1)
//         {
//             getCurrentPosition(decisionData, imu, map);
//             std::cout << RED << "enter passive lane change!" << RESET << std::endl;
//             std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
//             int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
//             int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);

//             //之前的无条件的换道
//             for (int i = 0; i <= 1; i++)
//             // for (int i = 0; i <= 0; i++) // only change left lane
//             {
//                 std::cout << "之前的无条件的换道" << i<<std::endl;
//                 //?????? 最后一个变量 i ^ i 永远是0吧，应该只能右换道
//                 // if (false == changeCurrentPoint(map.roads[roadIndex], map.roads[roadIndex].lanes[laneIndex], tempCurrentLaneId, tempCurrentIndex, currentPoint, (bool)i))
//                 // {
//                 //     continue;
//                 // }
//                 // decisionData.currentLaneId = tempCurrentLaneId;
//                 // decisionData.currentIndex = tempCurrentIndex;

//                 // std::vector<PlanningPoint>().swap(decisionData.referenceLine.referenceLinePoints);
//                 // decisionData.referenceLine.referenceLinePoints.reserve(0);
//                 // decisionData.referenceLine = getReferenceLine(map, decisionData, locationPoint);

//                 // getFrenetLocationPoint(decisionData, decisionData.referenceLine, locationPoint);
//                 // decisionData.optimalTrajectoryIndex = getTrajectoryPlanningResult(velocity, decisionData, imu, predictionMsg, map,  trafficLight, objectsCmd); // 20220825 修改函数输入
//                 // // std::cout << RED << "optimal trajectory index after changing planning point: " << decisionData.optimalTrajectoryIndex << RESET << std::endl;
//                 // if (decisionData.optimalTrajectoryIndex != -1)
//                 // {
//                 //     break;
//                 // }
//                 // else
//                 // {
//                 //     decisionData.currentLaneId = currentLaneId;
//                 //     decisionData.currentIndex = currentIndex;
//                 // }

//                   if( changeCurrentPoint(map.roads[roadIndex], map.roads[roadIndex].lanes[laneIndex], tempCurrentLaneId, tempCurrentIndex, currentPoint, (bool)i))
//                  {
//                      if(GetOptimalGlobalTrajectory(imu,  map, decisionData, predictionMsg, trafficLight, objectsCmd, tempCurrentLaneId, tempCurrentIndex))
//                      {
//                         PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);
//                         decisionData.optimalGlobalTrajectory = optimalGlobalTrajectory;
//                         break;
//                      }

//                  }

//                  std::cout << "之前的无条件的换道------------------------" << i<<std::endl;
//             tempCurrentLaneId = currentLaneId;
//             tempCurrentIndex = currentIndex;

//             }//for (int i = 0; i <= 1; i++)左、右换道的一个循环
//             // tempCurrentLaneId = currentLaneId;
//             // tempCurrentIndex = currentIndex;

//         }////    passive lane change
//     }//else // 获取当前位置最近路点成功

//     //std::cout <<"decisionData.optimalTrajectoryIndex" << decisionData.optimalTrajectoryIndex<<std::endl;
//     // 如果没找到可行轨迹，则停车
//     if (decisionData.optimalTrajectoryIndex == -1)
//     {
//         std::cout << RED << "NO WAY!" << RESET << std::endl;
//         decisionData.optimalTrajectoryIndex = 0;
//         stop(decisionData, velocity);
//     }
//     else
//     {
//         //std::cout <<"decisionData.optimalTrajectoryIndex--------------" << decisionData.optimalTrajectoryIndex<<std::endl;
//         // 计算曲率及弯道限速
//         int turnIndex = -1;
//         for (int iter = 2; iter < (int) decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() - 2; iter++)
//         {
//             // std::cout <<  RED<<" (int) decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() - 2" <<        (int) decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() - 2 <<std::endl;
//             double curvatureTemp = fabs(calculateCurvature(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter],
//                                                            decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2],
//                                                            decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2]));
//             // std::cout <<  RED<<"?????????????????????????????????????????curvatureTemp = ????????????????????" <<        curvatureTemp <<std::endl;
//             if (curvatureTemp > CURVATURE_THRESHOLS) // 曲率
//             {
//                 //  std::cout<<"data"<<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter] .gaussX<< ","
//                 //                           <<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter] .gaussY<< ";"
//                 //                           << decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2].gaussX<< ","
//                 //                           << decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2].gaussY<< ";"
//                 //                          <<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2] .gaussX<< ","
//                 //                          <<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2] .gaussY<< ","<< std::endl;
//                 turnIndex = iter;
//                 break;
//             }
//         }
//         if (turnIndex >= 0)
//         {
//             for (int iter = 0; iter < decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size(); iter++)
//             {
//                 decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter].v = std::min(CURVATURE_SPEED, decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter].v);
//             }
//         }
//     }

//     // 如果到了stopPoint,就停车
//     if (stopPointJudge(imu, stopPoints))
//     {
//         //std::cout << "stopPointJudge(imu, stopPoints))-------------------" << std::endl;
//         stop(decisionData, velocity);
//     }

//     //std::cout << RED << "changeLocalToGlobal---------------------- " << RESET << std::endl;
//     PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);
//     decisionData.optimalGlobalTrajectory = optimalGlobalTrajectory;

//     end = std::chrono::steady_clock::now();
//     duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//     //std::cout << RED << "eeeeeeeeeeeeeeeeeeeeeeee   1duration proces: " << duration.count() << RESET << std::endl;

//     return;
// }

// 在这个函数中完成了规划过程,将结果（规划曲线的x，y，angle，以及速度文件）存在decisiondata的trajectorylist中
// void localPlanning(const pc::Imu &imu, double velocity, const RoadMap &map, DecisionData &decisionData,
//                    const prediction::ObjectList &predictionMsg, const std::vector<std::tuple<int32_t, int32_t>> &routingList,
//                    const std::vector<GaussRoadPoint> stopPoints, const infopack::TrafficLight &trafficLight, std::vector<infopack::ObjectsProto> objectsCmd)
void localPlanning(const pc::Imu &imu, double velocity, RoadMap &map, DecisionData &decisionData,
                   const prediction::ObjectList &predictionMsg, std::vector<RoutingList> &routingListVector,
                   const std::vector<GaussRoadPoint> stopPoints, infopack::TrafficLight &trafficLight,
                   const std::vector<infopack::ObjectsProto> &objectsCmd, std::map<std::string, infopack::IntersectionState> &spatTrafficLightMap, infopack::TrafficLightFromPerc &trafficLightFromPerc)

{

    //  局部规划初始化LocalPlanning
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    initLocalPlanning(decisionData);

    // imu定位确定的当前全局坐标
    GaussRoadPoint locationPoint;
    locationPoint.GaussX = imu.gaussx();
    locationPoint.GaussY = imu.gaussy();
    locationPoint.yaw = imu.yaw();

    // 再当前道路和其他备选道路中选择一条最有的道路。
    // std::vector<std::tuple<int32_t, int32_t>> bestRoutingList = SelectBestRoutingList(routingListVector  );

    // 获取后续路段的road lane 序列
    decisionData.nextIdList.clear();
    decisionData.nextIdList.reserve(0);
    decisionData.nextIdList = getNextRoadLaneIdList(std::tuple<int32_t, int32_t>(decisionData.currentId, decisionData.currentLaneId), routingListVector[0].roadlanelist);

    if (!getCurrentPosition(decisionData, imu, map)) // 获取当前位置最近路点
    {
        // 如果没有获取到路点
        std::cout << RED << "localPlanning get current position failed :(" << RESET << std::endl;
        stop(decisionData, velocity); // 停车
    }
    else if (!restart(predictionMsg, velocity))
    {
        std::cout << RED << "restart failed :(" << RESET << std::endl;
        stop(decisionData, velocity); // 停车
    }
    else // 获取当前位置最近路点成功
    {
        //  根据地图信息，找对应的index
        std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
        int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
        int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);

        GaussRoadPoint currentPoint = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints[decisionData.currentIndex]; // 车道上的当前位置最近点

        // 处理红绿灯信息，在这里把红绿灯的信息准备好 20230713
        // 提取当前路和下一段路的road、lane，
        // 注意，如果是换道的话就用当前lane的红绿灯,如果是最后一段路，也用当前lane的红绿灯
        TrafficLight trafficLightTemp;
        bool bTrafficLightTemp;
        if (decisionData.nextIdList.size() == 0) // 后续没有路了
        {
            bTrafficLightTemp = map.trafficeLightMap_.GetTrafficLightByRoadLane(decisionData.currentId, decisionData.currentLaneId, decisionData.currentIndex,
                                                                                decisionData.currentId, decisionData.currentLaneId, trafficLightTemp); // 根据roadlane信息查找对象

            std::cout << "check road lane for intersection" << decisionData.currentId << "," << decisionData.currentLaneId << "," << decisionData.currentIndex
                      << "," << decisionData.currentId << "," << decisionData.currentLaneId << std::endl;
        }
        else
        {
            bTrafficLightTemp = map.trafficeLightMap_.GetTrafficLightByRoadLane(decisionData.currentId, decisionData.currentLaneId, decisionData.currentIndex,
                                                                                std::get<0>(decisionData.nextIdList[0]), std::get<1>(decisionData.nextIdList[0]), trafficLightTemp);

            // std::cout << "check road lane intersection" << decisionData.currentId << "," << decisionData.currentLaneId << "," << decisionData.currentIndex
            //           << "," << std::get<0>(decisionData.nextIdList[0]) << "," << std::get<1>(decisionData.nextIdList[0]) << std::endl;
        }

        // if (!bTrafficLightTemp) // 没找到有用的路口信息
        // {
        //     std::cout << "no find trafficLightTemp" << std::endl;
        // }

        // 默认设置前方为绿灯
        trafficLight.Clear();
        trafficLight.set_state(infopack::TrafficLight_State_GREEN_LIGHT);

        // 先处理感知获得的红绿灯信息
        bool hasTrafficLightFromPerc = false;
        if (trafficLightFromPerc.active()) // 设置红绿灯状态
        {
            hasTrafficLightFromPerc = true;
            switch (trafficLightFromPerc.state()) // 0 red 、 1 yellow 、 2 green
            {
            case 0:
            case 1:
                trafficLight.set_state(infopack::TrafficLight_State_RED_LIGHT);
                break;
            default:
                trafficLight.set_state(infopack::TrafficLight_State_GREEN_LIGHT);
            }

            trafficLight.set_stoproadid(trafficLightTemp.currentRoadID_);
            trafficLight.set_stoplaneid(trafficLightTemp.currentLaneID_);
            trafficLight.set_stoppointid(trafficLightTemp.stopPoint_);
            // trafficLight.set_remaining_time(intersectionState.phases(i).likelyendtime());
            //  trafficLight.set_active(intersectionState.phases(i).a);
            trafficLight.set_directionoftravel((int)trafficLightTemp.directionofTravel_);
        }

        // std::cout << "hasTrafficLightFromPerc = " <<hasTrafficLightFromPerc<<  " bTrafficLightTemp =" << bTrafficLightTemp << std::endl;

        // 只有在前方找到了路口并且对应相位一致的情况下，根据相位设置红绿灯
        if (bTrafficLightTemp && (!hasTrafficLightFromPerc)) // 如果前方有灯,且没有感知红绿灯信息
        {
            //  std::cout << "find trafficLightTemp" << trafficLightTemp.intersetionID_ << "," << trafficLightTemp.phaseID_<< ","
            //  << trafficLightTemp.currentRoadID_ <<"," <<trafficLightTemp.currentLaneID_<<","<<trafficLightTemp.stopPoint_<< std::endl;

            // 查找当前接收到的红绿灯相位信息中是否有前方路口的红绿灯信息
            std::map<std::string, infopack::IntersectionState>::iterator iter;
            iter = spatTrafficLightMap.find(trafficLightTemp.intersetionID_);

            // if (iter == spatTrafficLightMap.end()) // 当前接收到的红绿灯信息包含前方路口
            // {
            //     std::cout << "iter == spatTrafficLightMap.end()  not find ---- " << trafficLightTemp.intersetionID_ << std::endl;
            // }
            // else
            // {
            //      std::cout << "iter == spatTrafficLightMap.end()  find  +++" << trafficLightTemp.intersetionID_ << std::endl;
            // }

            if (iter != spatTrafficLightMap.end()) // 当前接收到的红绿灯信息包含前方路口
            {
                infopack::IntersectionState intersectionState = iter->second; // 找到路口了

                for (int i = 0; i < (int)intersectionState.phases_size(); i++) // 匹配相位
                {
                    // std::cout << "check intersectionState" << intersectionState.intersectionid() << "," << intersectionState.phases(i).phaseid()
                    // <<" for trafficLightTemp.phaseID_" <<trafficLightTemp.phaseID_ << std::endl;
                    if (intersectionState.phases(i).phaseid() == trafficLightTemp.phaseID_) // 相位也匹配上了
                    {
                        // 设置红绿灯状态及相关信息
                        trafficLight.Clear();
                        // std::cout << YELLOW << "lightstate@@@@@@@" << intersectionState.phases(i).lightstate() << RESET << std::endl;
                        switch (intersectionState.phases(i).lightstate())
                        {
                        case infopack::Phase_LightState_LightState_unavailable:
                        case infopack::Phase_LightState_LightState_dark:
                            trafficLight.set_state(infopack::TrafficLight_State_GREEN_LIGHT);
                            // std::cout <<"1111111111111111111111111111111"<<std::endl;
                            break;

                        case infopack::Phase_LightState_LightState_flashing_red:
                        case infopack::Phase_LightState_LightState_red:
                        case infopack::Phase_LightState_LightState_flashing_green:
                            trafficLight.set_state(infopack::TrafficLight_State_RED_LIGHT);
                            // std::cout <<"22222222222222222222222222222222"<<std::endl;
                            break;
                        case infopack::Phase_LightState_LightState_permissive_green:
                            // case infopack:: Phase_LightState_LightState_protected_green:
                            trafficLight.set_state(infopack::TrafficLight_State_GREEN_LIGHT);
                            // std::cout <<"3333333333333333333333333333333"<<std::endl;
                            break;

                        case infopack::Phase_LightState_LightState_yellow:
                        case infopack::Phase_LightState_LightState_flashing_yellow:
                        case infopack::Phase_LightState_LightState_protected_green: // 高铁新城这个是黄灯？？直行加右转的路口
                            trafficLight.set_state(infopack::TrafficLight_State_RED_LIGHT);
                            // std::cout <<"44444444444444444444444"<<std::endl;
                            break;
                        }

                        trafficLight.set_stoproadid(trafficLightTemp.currentRoadID_);
                        trafficLight.set_stoplaneid(trafficLightTemp.currentLaneID_);
                        trafficLight.set_stoppointid(trafficLightTemp.stopPoint_);
                        trafficLight.set_remaining_time(intersectionState.phases(i).likelyendtime());
                        // trafficLight.set_active(intersectionState.phases(i).a);
                        trafficLight.set_directionoftravel((int)trafficLightTemp.directionofTravel_);

                        // std::cout << "trafficLight result=" << trafficLight.stoproadid() << "," << trafficLight.stoplaneid() << "," << trafficLight.stoppointid()
                        //           << ",light state " << trafficLight.state() << ", light state V2X" << intersectionState.phases(i).lightstate() << std::endl;
                    }
                }
            }
        }

        // 先判断全局规划是否要求换道/以及是否能换道
        //  active lane change
        int tempCurrentLaneId, tempCurrentIndex;
        int currentLaneId = decisionData.currentLaneId;
        int currentIndex = decisionData.currentIndex;

        // 全局规划中，要求换道
        // 注意这里只是比较了下一条road、lane，但是如果是多次换道，如换两条道 0-》1-》2，在延续参考线的时候，可能会把2 也加进去
        // 但是如果直接选用2道那条道。对安全的检测等问题都得作额外处理
        if (decisionData.nextIdList.size() > 0 &&
            decisionData.currentId == std::get<0>(decisionData.nextIdList[0]) && decisionData.currentLaneId != std::get<1>(decisionData.nextIdList[0]))
        {
            std::cout << RED << "enter active lane change!" << RESET << std::endl;
            bool isLeft = std::get<1>(decisionData.nextIdList[0]) > decisionData.currentLaneId; // 往左lane编号增加,确认过的眼神
            // std::cout << RED << "1111111111111111111111111111111111111111111111" << RESET << std::endl;

            // 在这里增加换道的安全检查
            bool bChangelaneSafe = true; // 待换道路安全属性
            // 检查车辆前方是否有其他障碍物
            ////////////////////////////////////////////////////测试换道
            decisionData.checkLineVector.clear();
            decisionData.checkLineVector.reserve(0);
            double dRefLineLength = (double)(MAX_FRENET_S(imu.velocity()));
            // dRefLineLength = dRefLineLength * (SEGMENTED_FRENET_NUMBER + 1) / SEGMENTED_FRENET_NUMBER; // 比防碰撞的规划曲线多一段长度

            dRefLineLength = dRefLineLength * 1.2; // 比防碰撞的规划曲线多一段长度
            int i = 0;                             // 只检查当前路

            ReferenceLine checkLine;
            checkLine.referenceLinePoints.clear();

            if (GetCheckLine(map, routingListVector[i], decisionData.currentId, decisionData.currentLaneId, decisionData.currentIndex, dRefLineLength, checkLine))
            {
                decisionData.checkLineVector.push_back(checkLine); // 这个一定能找到,wuwu buyidingya
            }
            else
            {
                bChangelaneSafe = false;
            }

            std::cout << "decisionData.checkLineVector.size = " << decisionData.checkLineVector.size() << std::endl;

            // 20230923
            if (decisionData.checkLineVector.size() == 0)
            {
                bChangelaneSafe = false;
            }
            else
            {
                decisionData.checkLineVector[i].dFrontObsDis = dRefLineLength; // 探测的最远距离
                decisionData.checkLineVector[i].bOK = true;

                for (int j = 0; j < (int)decisionData.checkLineVector[i].referenceLinePoints.size(); j = j + 5) // 每隔5点计算一次，大约是1米的距离
                {

                    // 将平面直角坐标转化为车辆局部坐标，
                    PlanningPoint pointTemp; // 参考线的检查点，要转成车辆坐标系，右前, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu
                    pointTemp.x = decisionData.checkLineVector[i].referenceLinePoints[j].gaussY;
                    pointTemp.y = decisionData.checkLineVector[i].referenceLinePoints[j].gaussX;
                    pointTemp.angle = decisionData.checkLineVector[i].referenceLinePoints[j].gaussAngle;
                    double dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow;
                    dXVehicleForShow = imu.gaussy();
                    dYVehicleForShow = imu.gaussx();
                    dYawVehicleForShow = M_PI / 2 - imu.yaw() * M_PI / 180.;
                    CoordTran2DForNew0INOld(pointTemp.x, pointTemp.y, pointTemp.angle, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow); //  前左,
                    // 使用的时候要用右前
                    double dTemp;
                    dTemp = pointTemp.x;
                    pointTemp.x = -pointTemp.y;
                    pointTemp.y = dTemp;
                    std::cout << "check line pointTemp.x =" << pointTemp.x << "pointTemp.y = " << pointTemp.y << std::endl;

                    // double dDistance = getMinDistanceOfPoint(pointTemp, predictionMsg, objectsCmd, imu);
                    double dDistance = getMinDistanceOfPointCheckSafety(pointTemp, predictionMsg, objectsCmd, imu, CHANGE_LANE_SAFE_WIDTH);
                    std::cout << "change line front dDistance " << dDistance << std::endl;
                    if (dDistance < CHANGE_LANE_SAFE_WIDTH) // 存在距离过近的障碍物,距离小于2米，说明这条路不通
                    {
                        decisionData.checkLineVector[i].dFrontObsDis = pointTemp.y;
                        decisionData.checkLineVector[i].bOK = false;
                        break;
                    }

                } // for(int j =0; j< (int) decisionData.checkLineVector[i].referenceLinePoints.size();j=j+5)//每隔5点计算一次，大约是1米的距离

                std::cout << "decisionData.checkLineVector[i]. =  " << i
                          << " dFrontObsDis " << decisionData.checkLineVector[i].dFrontObsDis << " " << decisionData.checkLineVector[i].bOK << std::endl;

                if (decisionData.checkLineVector[i].bOK == false)
                {
                    bChangelaneSafe = false;
                    std::cout << "change line front dDistance is dange" << i << std::endl;
                }
            }

            // 检查车辆后方是否有其他障碍物
            if (bChangelaneSafe)
            {
                double dChangeDis; // 换道后中心线的位置
                if (isLeft)
                {
                    dChangeDis = -4; // 假设车道宽度为4米
                }
                else
                {
                    dChangeDis = 4;
                }

                // 按照向后10米。每隔1米一个点构造一条点集，用于判断是否会发生碰撞
                for (int j = 1; j <= CHANGE_LANE_BACK_SAFE_LENGTH; j++) // 这里是右前坐标系, 0点在前向已经检测过了
                {
                    PlanningPoint pointTemp;
                    pointTemp.x = dChangeDis;
                    pointTemp.y = j * (-1.0);

                    // double dDistance = getMinDistanceOfPoint(pointTemp, predictionMsg, objectsCmd, imu);
                    double dDistance = getMinDistanceOfPointCheckSafety(pointTemp, predictionMsg, objectsCmd, imu, CHANGE_LANE_SAFE_WIDTH);

                    // getMinDistanceSquareOfPoint
                    // std::cout << "back dDistance " << dDistance <<std::endl;
                    if (dDistance < CHANGE_LANE_SAFE_WIDTH) // 存在距离过近的障碍物,距离小于2米，说明这条路不通
                    {
                        // decisionData.checkLineVector[i].dBackObsDis =  pointTemp.y;
                        // decisionData.checkLineVector[i].bOK = false;
                        bChangelaneSafe = false;
                        break;
                    }
                }
            }

            //

            if (bChangelaneSafe && changeCurrentPoint(map.roads[roadIndex], map.roads[roadIndex].lanes[laneIndex], tempCurrentLaneId, tempCurrentIndex, currentPoint, isLeft))
            {
                // std::cout << RED << "22222222222222222222222222222222222222" << RESET << std::endl;
                if (GetOptimalGlobalTrajectory(imu, map, decisionData, predictionMsg, trafficLight, objectsCmd, tempCurrentLaneId, tempCurrentIndex, stopPoints))
                {
                    //  std::cout << RED << "3333333333333333333333333" << RESET << std::endl;
                    PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);
                    decisionData.optimalGlobalTrajectory = optimalGlobalTrajectory;
                    return;
                }
            }

        } // if (decisionData.currentId == std::get<0>(decisionData.nextId) && decisionData.currentLaneId != std::get<1>(decisionData.nextId))

        tempCurrentLaneId = currentLaneId;
        tempCurrentIndex = currentIndex;

        // end = std::chrono::steady_clock::now();
        // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // std::cout << RED << "主动换道aaaaaaaaaaaaaaaaaaaaaaa   1duration proces: " << duration.count() << RESET << std::endl;

        // 全局规划没有要求换道，在当前道路上继续前行
        //???是否会出现主动换道过不去，一直沿着当前路往前走，直到做到本lane的终点，卡死在本lane的终点出，再也拐不过去了
        // std::cout << RED << "enter no  lane change, go along current road!" << RESET << std::endl;
        getCurrentPosition(decisionData, imu, map);
        GetOptimalGlobalTrajectory(imu, map, decisionData, predictionMsg, trafficLight, objectsCmd, tempCurrentLaneId, tempCurrentIndex, stopPoints);
        // std::cout << "decisionData.optimalTrajectoryIndex" << decisionData.optimalTrajectoryIndex << std::endl;

        end = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // std::cout << RED << "直行aaaaaaaaaaaaaaaaaaaaaaa   1duration proces: " << duration.count() << RESET << std::endl;
        // 如果直行道没有可行道路，那么考虑左右换道
        // 这里的问题是对于静态的物体，每次都进入这种条件，还算稳定地走换道的曲线，但是对于动态目标，可能会出现在不同道路之间来回跳的问题
        // 先这样简单处理
        //  passive lane change
        if (decisionData.optimalTrajectoryIndex == -1)
        {
            getCurrentPosition(decisionData, imu, map);
            std::cout << RED << "enter passive lane change!" << RESET << std::endl;
            std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
            int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
            int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);

            ////////////////////////////////////////////////////测试换道
            decisionData.checkLineVector.clear();
            decisionData.checkLineVector.reserve(0);
            double dRefLineLength = (double)MAX_FRENET_S(imu.velocity());
            dRefLineLength = dRefLineLength * (SEGMENTED_FRENET_NUMBER + 1) / SEGMENTED_FRENET_NUMBER; // 比防碰撞的规划曲线多一段长度
            for (int i = 1; i < routingListVector.size(); i++)
            {

                ReferenceLine checkLine;
                checkLine.referenceLinePoints.clear();

                if (GetCheckLine(map, routingListVector[i], decisionData.currentId, decisionData.currentLaneId, decisionData.currentIndex, dRefLineLength, checkLine))
                {
                    decisionData.checkLineVector.push_back(checkLine);
                }
            }
            std::cout << "decisionData.checkLineVector.size = " << decisionData.checkLineVector.size() << std::endl;
            // 20230923

            // 进行车辆前进方向的碰撞检测的计算
            for (int i = 0; i < decisionData.checkLineVector.size(); i++)
            {
                decisionData.checkLineVector[i].dFrontObsDis = dRefLineLength; // 探测的最远距离
                decisionData.checkLineVector[i].bOK = true;
                for (int j = 0; j < (int)decisionData.checkLineVector[i].referenceLinePoints.size(); j = j + 5) // 每隔5点计算一次，大约是1米的距离
                {

                    // 将平面直角坐标转化为车辆局部坐标，
                    PlanningPoint pointTemp; // 参考线的检查点，要转成车辆坐标系，右前, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu
                    pointTemp.x = decisionData.checkLineVector[i].referenceLinePoints[j].gaussY;
                    pointTemp.y = decisionData.checkLineVector[i].referenceLinePoints[j].gaussX;
                    pointTemp.angle = decisionData.checkLineVector[i].referenceLinePoints[j].gaussAngle;
                    double dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow;
                    dXVehicleForShow = imu.gaussy();
                    dYVehicleForShow = imu.gaussx();
                    dYawVehicleForShow = M_PI / 2 - imu.yaw() * M_PI / 180.;
                    CoordTran2DForNew0INOld(pointTemp.x, pointTemp.y, pointTemp.angle, dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow); //  前左,
                    // 使用的时候要用右前
                    double dTemp;
                    dTemp = pointTemp.x;
                    pointTemp.x = -pointTemp.y;
                    pointTemp.y = dTemp;

                    double dDistance = getMinDistanceOfPoint(pointTemp, predictionMsg, objectsCmd, imu);
                    // std::cout << "dDistance " << dDistance <<std::endl;
                    if (dDistance < 2) // 存在距离过近的障碍物,距离小于2米，说明这条路不通
                    {
                        decisionData.checkLineVector[i].dFrontObsDis = pointTemp.y;
                        decisionData.checkLineVector[i].bOK = false;
                        break;
                    }

                } // for(int j =0; j< (int) decisionData.checkLineVector[i].referenceLinePoints.size();j=j+5)//每隔5点计算一次，大约是1米的距离

                std::cout << "decisionData.checkLineVector[i]. =  " << i << " dFrontObsDis " << decisionData.checkLineVector[i].dFrontObsDis << " " << decisionData.checkLineVector[i].bOK << std::endl;
            } // for(int i=1; i < decisionData.checkLineVector.size(); i++) ，进行车辆前进方向的碰撞检测的计算

            // 进行车辆后方，并道方向是否有车辆的防碰撞检测，
            // 由于无法之后车辆后方的轨迹，不能简单地找到车辆后方所有的轨迹，这样反向查找太多了，
            // 只是简单按照车辆正后方10米的距离吧，如果在并道车道内有障碍物，就认为是不能换道了，
            // 但是这种可能会导致在后方有个车缓慢跟车流出空隙，等本车变道，就但是本车认为这样不安全，始终换不过去
            // 应该同时考虑车辆的速度信息才合理
            for (int i = 0; (int)i < decisionData.checkLineVector.size(); i++)
            {
                if (!decisionData.checkLineVector[i].bOK) // 不通的路，就不用在浪费时间进行检测了
                    continue;

                double dChangeDis; // 换道后中心线的位置
                if (routingListVector[i + 1].leftRightLane == RoutingList::LEFT)
                {
                    dChangeDis = routingListVector[i + 1].nChangeLane * 4 * (-1); // 假设车道宽度为4米
                }
                else if (routingListVector[i + 1].leftRightLane == RoutingList::RIGHT)
                {
                    dChangeDis = routingListVector[i + 1].nChangeLane * 4;
                }
                else
                {
                    continue;
                }
                // 按照向后10米。每隔1米一个点构造一条点集，用于判断是否会发生碰撞
                for (int j = 1; j <= CHANGE_LANE_BACK_SAFE_LENGTH; j++) // 这里是右前坐标系, 0点在前向已经检测过了
                {
                    PlanningPoint pointTemp;
                    pointTemp.x = dChangeDis;
                    pointTemp.y = j * (-1.0);

                    double dDistance = getMinDistanceOfPoint(pointTemp, predictionMsg, objectsCmd, imu);
                    // getMinDistanceSquareOfPoint
                    // std::cout << "dDistance " << dDistance <<std::endl;
                    if (dDistance < 2) // 存在距离过近的障碍物,距离小于2米，说明这条路不通
                    {
                        decisionData.checkLineVector[i].dBackObsDis = pointTemp.y;
                        decisionData.checkLineVector[i].bOK = false;
                        break;
                    }
                }
            }

            for (int i = 0; (int)i < decisionData.checkLineVector.size(); i++)
            {
                if (!decisionData.checkLineVector[i].bOK) // 不通的路
                    continue;

                // 更新全局规划路径
                decisionData.nextIdList.clear();
                decisionData.nextIdList.reserve(0);
                if ((int)routingListVector.size() < i + 1)
                    continue;
                decisionData.nextIdList = routingListVector[i + 1].roadlanelist;
                // 去掉本road的所有点
                int currRoadIDTemp;
                if (decisionData.nextIdList.size() > 0)
                    currRoadIDTemp = std::get<0>(decisionData.nextIdList[0]);
                for (int j = 0; j < decisionData.nextIdList.size(); j++)
                {
                    if (std::get<0>(decisionData.nextIdList[0]) == currRoadIDTemp)
                    {
                        decisionData.nextIdList.erase(decisionData.nextIdList.begin());
                        j--;
                    }
                }

                for (int j = 0; j < (int)decisionData.nextIdList.size(); j++)
                {
                    std::cout << "decisionData.nextIdList" << std::get<0>(decisionData.nextIdList[j]) << " " << std::get<1>(decisionData.nextIdList[j]) << ";";
                }
                std::cout << "=====decisionData.nextIdList" << std::endl;
                bool bLetf;
                std::cout << "routingListVector[i+1].leftRightLane" << routingListVector[i + 1].leftRightLane << std::endl;
                if (routingListVector[i + 1].leftRightLane == RoutingList::LEFT)
                {
                    bLetf = true;
                }
                else if (routingListVector[i + 1].leftRightLane == RoutingList::RIGHT)
                {
                    bLetf = false;
                }
                else
                {
                    continue;
                }

                //     //按照这条路走。？？？如果是多换道的话，也先只换一条道过去，换过去再看看情况，
                //     //bool bLetf = decisionData.checkLineVector[i].
                // // for (int i = 0; i <= 1; i++)
                // // // for (int i = 0; i <= 0; i++) // only change left lane
                // // {
                // //     std::cout << "之前的无条件的换道" << i<<std::endl;

                if (changeCurrentPoint(map.roads[roadIndex], map.roads[roadIndex].lanes[laneIndex], tempCurrentLaneId, tempCurrentIndex, currentPoint, bLetf))
                {
                    std::cout << "changeCurrentPoint" << map.roads[roadIndex].id << " " << map.roads[roadIndex].lanes[laneIndex].id << " " << tempCurrentLaneId << " " << tempCurrentIndex << std::endl;
                    if (GetOptimalGlobalTrajectory(imu, map, decisionData, predictionMsg, trafficLight, objectsCmd, tempCurrentLaneId, tempCurrentIndex, stopPoints))
                    {
                        PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);
                        decisionData.optimalGlobalTrajectory = optimalGlobalTrajectory;
                        break;
                    }
                    else
                    {
                        std::cout << "cannot GetOptimalGlobalTrajectory " << std::endl;
                    }
                }
                else
                {
                    std::cout << "cannot find changeCurrentPoint" << std::endl;
                }

                tempCurrentLaneId = currentLaneId;
                tempCurrentIndex = currentIndex;
            }

            // 之前的无条件的换道
            //  for (int i = 0; i <= 1; i++)
            //  // for (int i = 0; i <= 0; i++) // only change left lane
            //  {
            //      std::cout << "之前的无条件的换道" << i<<std::endl;

            //     if( changeCurrentPoint(map.roads[roadIndex], map.roads[roadIndex].lanes[laneIndex], tempCurrentLaneId, tempCurrentIndex, currentPoint, (bool)i))
            //      {
            //          if(GetOptimalGlobalTrajectory(imu,  map, decisionData, predictionMsg, trafficLight, objectsCmd, tempCurrentLaneId, tempCurrentIndex))
            //          {
            //             PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);
            //             decisionData.optimalGlobalTrajectory = optimalGlobalTrajectory;
            //             break;
            //          }

            //      }

            //      std::cout << "之前的无条件的换道------------------------" << i<<std::endl;
            //     tempCurrentLaneId = currentLaneId;
            //     tempCurrentIndex = currentIndex;

            // }//for (int i = 0; i <= 1; i++)左、右换道的一个循环

        } ////    passive lane change
    }     // else // 获取当前位置最近路点成功

    // std::cout <<"decisionData.optimalTrajectoryIndex" << decisionData.optimalTrajectoryIndex<<std::endl;
    //  如果没找到可行轨迹，则停车
    if (decisionData.optimalTrajectoryIndex == -1)
    {
        std::cout << RED << "NO WAY!" << RESET << std::endl;
        decisionData.optimalTrajectoryIndex = 0;
        stop(decisionData, velocity);
    }
    else
    {
        // 20230621 将弯道减速都挪到速度规划里面去了
        //  //std::cout <<"decisionData.optimalTrajectoryIndex--------------" << decisionData.optimalTrajectoryIndex<<std::endl;
        //  // 计算曲率及弯道限速
        //  int turnIndex = -1;
        //  for (int iter = 2; iter < (int) decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() - 2; iter++)
        //  {
        //      // std::cout <<  RED<<" (int) decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() - 2" <<        (int) decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() - 2 <<std::endl;
        //      double curvatureTemp = fabs(calculateCurvature(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter],
        //                                                     decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2],
        //                                                     decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2]));
        //      // std::cout <<  RED<<"?????????????????????????????????????????curvatureTemp = ????????????????????" <<        curvatureTemp <<std::endl;
        //      if (curvatureTemp > CURVATURE_THRESHOLS) // 曲率
        //      {
        //          //  std::cout<<"data"<<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter] .gaussX<< ","
        //          //                           <<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter] .gaussY<< ";"
        //          //                           << decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2].gaussX<< ","
        //          //                           << decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter - 2].gaussY<< ";"
        //          //                          <<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2] .gaussX<< ","
        //          //                          <<decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter + 2] .gaussY<< ","<< std::endl;
        //          turnIndex = iter;
        //          break;
        //      }
        //  }
        //  if (turnIndex >= 0)
        //  {
        //      for (int iter = 0; iter < decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size(); iter++)
        //      {
        //          //弯道的速度限制赋值取消了，这部分可以把整个弯道计算都取消
        //          //decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter].v = std::min(CURVATURE_SPEED, decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints[iter].v);
        //      }
        //  }
    }

    // 如果到了stopPoint,就停车
    // 20230615 取消停车 lry
    // if (stopPointJudge(imu, stopPoints))
    // {
    //     //std::cout << "stopPointJudge(imu, stopPoints))-------------------" << std::endl;
    //     stop(decisionData, velocity);
    // }

    // std::cout << RED << "changeLocalToGlobal---------------------- " << RESET << std::endl;
    PlanningTrajectory optimalGlobalTrajectory = changeLocalToGlobal(decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex], locationPoint);
    decisionData.optimalGlobalTrajectory = optimalGlobalTrajectory;

    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << RED << "local planning eeeeeeeeeeeeeeeeeeeeeeee   1duration proces: " << duration.count() << RESET << std::endl;

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

    int stopIndex = (int)(1.25 * velocity * velocity); // 假设加速度为2，每点的间距是20cm 距离= 速度*速度/加速度/2

    if (stopIndex > CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER / 2)
    {
        stopIndex = CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER / 2;
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

    for (int i = stopIndex; i < CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER; i++)
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
    // std::cout << CYAN << -controlPointDistance * std::sin(controlPoint[3].angle / 180 * M_PI) << " h " << -controlPointDistance * std::cos(controlPoint[3].angle / 180 * M_PI) << " h " << controlPoint[3].angle << std::endl;

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
            resultPoint.frenetAngle = static_cast<int32_t>(360 +
                                                           atan2(curve.planningPoints[i].s - curve.planningPoints[i - 1].s, curve.planningPoints[i].l - curve.planningPoints[i - 1].l) / M_PI * 180) %
                                      360;
        }
        curve.planningPoints.push_back(resultPoint);
        // std::cout << CYAN << "curve.points[i].angle" << curve.points[i].angle << std::endl;
    }
}

void clearPathList(std::vector<PlanningTrajectory> &pathList)
{
    pathList.clear();
}

// 在这个函数中，主要完成了多条轨迹的生成，并作出了轨迹的选择。
int32_t getTrajectoryPlanningResult(double velocity, DecisionData &decisionData, const pc::Imu &imu,
                                    const prediction::ObjectList &predictionMsg, const RoadMap &map,
                                    const infopack::TrafficLight &trafficLight, std::vector<infopack::ObjectsProto> objectsCmd,
                                    const std::vector<GaussRoadPoint> stopPoints) // 20220825 修改函数输入
{
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::vector<PlanningTrajectory> frenetPathList; // frenet坐标系下规划路径

    // std::cout << RED << "decisionData.frenetLocationPoint: " << decisionData.frenetLocationPoint.l << std::endl;
    // 横向规划，生成路径规划曲线
    // generateBezierPathListInFrenet(decisionData.referenceLine, decisionData.frenetLocationPoint, frenetPathList);
    generateSegmentedBezierPathListInFrenet(map, decisionData.referenceLine, decisionData.frenetLocationPoint, frenetPathList);
    // std::cout << RED << "44444444444444444444444!" << RESET << std::endl;

    //  from sl to xy
    PlanningTrajectory tempPath;
    int prevIndex = 0, lastIndex = 0;
    // std::cout << RED << "rrrrrrrrrrrrrrrrrrr"
    //           << "frenetPathList.size: " << frenetPathList.size() << std::endl;
    // std::cout << RED << "rrrrrrrrrrrrrrrrrrr"
    //           << "frenetPathList.planningpoint.size: " << frenetPathList[0].planningPoints.size() << std::endl;

    // 20230518 处理换道还保留原道路规划线
    decisionData.finalPathList.clear();
    decisionData.finalPathList.reserve(frenetPathList.size());
    // 对所有线段转换坐标
    // for (int i = 0; i < CURVE_NUM; i++)
    for (int i = 0; i < (int)frenetPathList.size(); i++)
    {
        // for (int j = 0; j < CURVE_POINT_NUM; j++)
        for (int j = 0; j < (int)frenetPathList[i].planningPoints.size(); j++)
        {
            // std::cout << RED <<"tttttttttttttttttt"<< std::endl;
            frenet2Cartesian(frenetPathList[i].planningPoints[j].s, frenetPathList[i].planningPoints[j].l, frenetPathList[i].planningPoints[j].x, frenetPathList[i].planningPoints[j].y, decisionData.referenceLine, lastIndex, prevIndex);
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
        tempPath.planningPoints.reserve(0);
    } // for (int i = 0; i < CURVE_NUM; i++)
    frenetPathList.clear();

    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << RED << "getTrajectoryPlanningResult  路径规划  ---------------------------- " << duration.count() << RESET << std::endl;
    //  std::cout << "decisionData.finalPathList.size() = " <<decisionData.finalPathList.size() << std::endl;
    //  for(int i=0; i<(int) decisionData.finalPathList.size();i++)
    //  {
    //      std::cout << "decisionData.finalPathList[i].planningPoints.size() = " <<decisionData.finalPathList[i].planningPoints.size() << std::endl;
    //  }

    //  std::cout << "decisionData.controlTrajectoryList.size() = " <<decisionData.controlTrajectoryList.size() << std::endl;

    // std::cout << RED << "5555555555555555555555!" << RESET << std::endl;
    // 纵向规划，生成速度信息
    // generateSpeedList(velocity, decisionData, objectsCmd, imu);

    // lry0620
    generateSpeedList(velocity, decisionData, objectsCmd, predictionMsg, imu, stopPoints, trafficLight, map);
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][0] << RESET << std::endl;
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][10] << RESET << std::endl;
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][20] << RESET << std::endl;
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][30] << RESET << std::endl;
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][40] << RESET << std::endl;
    // std::cout << RED << " hh speedlist: "<<decisionData.speedList[5][49] << RESET << std::endl;

    // std::cout << RED << "666666666666666666666666!" << RESET << std::endl;
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << RED << "getTrajectoryPlanningResult  速度规划  ---------------------------- " << duration.count() << RESET << std::endl;
    //   合并生成轨迹
    // 在generatespeedlist函数里面完成了对每条路径速度的赋值，这里不用再进行合并了
    // generateTrajectoryList(decisionData);

    // std::cout << RED << "777777777777777777777!" << RESET << std::endl;

    // 在规划的路径中选择一条最优路径
    // 这个函数不用在做碰撞检测了，就是选择最后一点速度最大的点
    int32_t optimalTrajectoryIndex = 0;
    double dMaxEndSpeedTemp; // 用于存放最大速度的临时变量
    // if(decisionData.controlTrajectoryList.size() > 0)
    // {
    //     dMaxEndSpeedTemp = decisionData.controlTrajectoryList[optimalTrajectoryIndex].planningPoints.back().v;

    //      for(int i=1; i< (int)decisionData.controlTrajectoryList.size();i++ )
    //      {
    //         if( decisionData.controlTrajectoryList[i].planningPoints.back().v >dMaxEndSpeedTemp )
    //         {
    //             optimalTrajectoryIndex = i;
    //             dMaxEndSpeedTemp = decisionData.controlTrajectoryList[i].planningPoints.back().v;
    //         }
    //      }

    // }

    if (decisionData.controlTrajectoryList.size() > 0)
    {
        dMaxEndSpeedTemp = decisionData.controlTrajectoryList[optimalTrajectoryIndex].planningPoints.back().v;

        for (int i = 1; i < (int)decisionData.controlTrajectoryList.size(); i++)
        {
            if (decisionData.controlTrajectoryList[i].planningPoints.back().v > dMaxEndSpeedTemp)
            {
                optimalTrajectoryIndex = i;
                dMaxEndSpeedTemp = decisionData.controlTrajectoryList[i].planningPoints.back().v;
            }
        }
    }
    // 20230716
    // 改为第一个速度点,现在第一个点是速度最大点，且后面的速度比当前的大于1，才选择后面的.
    // 这里不对，如果所有道路都会停车，应该返回 -1 。实现换道，现在是总会有一条道可行，导致无法实现换道了
    // if (decisionData.controlTrajectoryList.size() > 0)
    // {
    //     dMaxEndSpeedTemp = decisionData.controlTrajectoryList[optimalTrajectoryIndex].planningPoints[0].v;

    //     for (int i = 1; i < (int)decisionData.controlTrajectoryList.size(); i++)
    //     {
    //         // if( decisionData.controlTrajectoryList[i].planningPoints.back().v >dMaxEndSpeedTemp )
    //         if (decisionData.controlTrajectoryList[i].planningPoints[0].v - dMaxEndSpeedTemp > 1)
    //         {
    //             optimalTrajectoryIndex = i;
    //             dMaxEndSpeedTemp = decisionData.controlTrajectoryList[i].planningPoints[0].v;
    //         }
    //     }
    // }

    // std::cout << "optimalTrajectoryIndex after getOptimalTrajectoryIndex: " << optimalTrajectoryIndex << std::endl;

    // int32_t optimalTrajectoryIndex = getOptimalTrajectoryIndex(decisionData, predictionMsg, trafficLight, objectsCmd, imu);
    // std::cout << "optimalTrajectoryIndex after getOptimalTrajectoryIndex: " << optimalTrajectoryIndex << std::endl;
    return optimalTrajectoryIndex;
}

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

        // ？？？这里有个问题，需要增加一个判断，如果最小点是本线段的最后一点，应该与后继线段的第一点做一个比较，避免达到阈值才到下一条线段
        if (targetIndex != -1 && minDis <= POINT_DIS_THRESHOLD_ON_CURRENT_LANE) // 如果找到了，就直接退出
        {
            decisionData.currentId = lastRoadId;
            decisionData.currentLaneId = lastLaneId;
            decisionData.currentIndex = targetIndex;
            // std::cout << " *** hhhhhhhhhhhhhh Flag 1 ***" << std::endl;
            return true;
        }

        // 再找这条车道上的其他点，即上次点之前的点。？？？为什么要往前找，走过的路再回头似乎不合适
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
    if (decisionData.nextIdList.size() > 0)
    {
        int nextRoadId = std::get<0>(decisionData.nextIdList[0]);
        int nexLaneId = std::get<1>(decisionData.nextIdList[0]);
        // std::cout << BOLDBLUE << "20221011 (3): " << nextRoadId << "; " << nexLaneId << std::endl;

        int32_t nextRoadIndex = 0, nextLaneIndex = 0; // 后继road 和 lane 的 index
        for (int i = 0; i < map.roads.size(); i++)
        {
            if (nextRoadId == map.roads[i].id)
            {
                nextRoadIndex = i;
                break;
            }
        }
        for (int i = 0; i < map.roads[nextRoadIndex].lanes.size(); i++)
        {
            if (nexLaneId == map.roads[nextRoadIndex].lanes[i].id)
            {
                nextLaneIndex = i;
                break;
            }
        }

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
            float dis = pointToLineDistance(map.roads[i].lanes[0].gaussRoadPoints[int((j - 1) * (pointsNum - 1) / 5)], map.roads[i].lanes[0].gaussRoadPoints[int(j * (pointsNum - 1) / 5)], currentPoint);
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

// void generateSpeedList(double velocity, DecisionData &decisionData, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu)
// {
//     double currentSpeed = velocity;
//     if (currentSpeed < 0.1)
//         currentSpeed = 1.2;

//     double dDesireSpeed = DESIRED_SPEED;
//     //std::cout << "decisionData.controlTrajectoryList.size()" <<decisionData.controlTrajectoryList.size()<<std::endl;
//     // 处理ACC跟车
//     for (int i = 0; i < (int) decisionData.controlTrajectoryList.size(); i++)
//     {
//         if (hasPrecedingVehicle(imu, objectsCmd))
//         {
//             // 计算acc速度
//             dDesireSpeed = std::min(cruiseController(imu, objectsCmd), DESIRED_SPEED);
//         }
//     }
//     // std::cout << "dDesireSpeed = " << dDesireSpeed << std::endl;
//     //  清空速度列表
//     decisionData.speedList.clear(); // 20220825
//     decisionData.speedList.reserve(END_SPEED_NUM);

//     std::vector<double> endPointSpeedList;
//     endPointSpeedList.clear();
//     endPointSpeedList.reserve(END_SPEED_NUM);

//     ///////20230213 简化速度规划为1条
//     double endPointSpeed = dDesireSpeed;
//     endPointSpeedList.push_back(dDesireSpeed);

//     // 使用定加速法生成平滑曲线
//     std::vector<double> curvePointSpeed = interp::constant_acceleration(currentSpeed, dDesireSpeed, CURVE_POINT_NUM*SEGMENTED_FRENET_NUMBER, 2.0);
//     decisionData.speedList.push_back(curvePointSpeed);

//     // 20230212 简化速度规划，直接只留一条 begin
//     // for (int i = 0; i < END_SPEED_NUM; i++)
//     // {
//     //   double endPointSpeed = 0.0;
//     //   if (i % 2 == 0)
//     //   {
//     //     endPointSpeed = DESIRED_SPEED + (double)i * SPEED_RANGE / (END_SPEED_NUM - 1);
//     //   }
//     //   else
//     //   {
//     //     endPointSpeed = DESIRED_SPEED - (double)(i + 1) * SPEED_RANGE / (END_SPEED_NUM - 1);
//     //   }

//     //   endPointSpeedList.push_back(endPointSpeed);

//     //   // std::cout << "222end point speed: " << endPointSpeed << std::endl;
//     // }
//     // endPointSpeedList.push_back(DESIRED_SPEED);

//     // 20230212 简化速度规划，直接只留一条 end

//     // for (int i = 0; i < END_SPEED_NUM; i++)
//     // {
//     //   // std::vector<double> curvePointSpeed = interp::linear(currentSpeed, endPointSpeedList[i], CURVE_POINT_NUM);
//     //   std::vector<double> curvePointSpeed = interp::constant_acceleration(currentSpeed, endPointSpeedList[i], CURVE_POINT_NUM, 2.0);

//     //   // double endPointSpeedFirst = endPointSpeedList[i];
//     //   // double endPointSpeedSecond = endPointSpeedList[i];
//     //   // std::vector<double> curvePointSpeed;
//     //   // curvePointSpeed.clear();
//     //   // curvePointSpeed.reserve(CURVE_POINT_NUM);
//     //   // for (int k = 0; k < CURVE_POINT_NUM * 2; k++)
//     //   // {
//     //   //   if (k < CURVE_POINT_NUM)
//     //   //   {
//     //   //     curvePointSpeed.push_back(currentSpeed + (endPointSpeedFirst - currentSpeed) * ((double)k / (CURVE_POINT_NUM - 1.0)));
//     //   //   }
//     //   //   else
//     //   //   {
//     //   //     curvePointSpeed.push_back(endPointSpeedFirst);
//     //   //   }
//     //   // }
//     //   decisionData.speedList.push_back(curvePointSpeed);
//     // }
// }

// 生成轨迹采样结果,将之前生成的轨迹和速度合并在controlTrajectoryList中
void generateTrajectoryList(DecisionData &decisionData)
{
    std::vector<PlanningTrajectory>().swap(decisionData.controlTrajectoryList);
    // decisionData.controlTrajectoryList.clear();
    // decisionData.controlTrajectoryList.reserve(CURVE_NUM * END_SPEED_NUM);
    decisionData.controlTrajectoryList.reserve(decisionData.finalPathList.size() * END_SPEED_NUM);

    // for (int i = 0; i < CURVE_NUM; i++)
    for (int i = 0; i < (int)decisionData.finalPathList.size(); i++)
    {
        for (int j = 0; j < END_SPEED_NUM; j++)
        {
            PlanningTrajectory trajectoryPointList;
            trajectoryPointList.planningPoints.clear();
            trajectoryPointList.planningPoints.reserve(0);
            // for (int k = 0; k < CURVE_POINT_NUM; k++)
            for (int k = 0; k < (int)decisionData.finalPathList[i].planningPoints.size(); k++)
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

    // std::cout << "generateTrajectoryList  decisionData.controlTrajectoryList = " << decisionData.controlTrajectoryList.size() << std::endl;
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
        if (traffic_light_state == infopack::TrafficLight::RED_LIGHT ||
            traffic_light_state == infopack::TrafficLight::YELLOW_LIGHT) // 原来算法计算了一下黄灯时，以当前速度可以通过的可行性
        {
            return 0; // 红灯或黄灯，就没有可行走的轨迹了
        }
        else
        {
            for (int i = 0; i < (int)decisionData.controlTrajectoryList.size(); i++)
            {
                decisionData.feasibleTrajectoryIndexList.push_back(i);
            }

            return 2; // 绿灯所有轨迹都可选
        }
    }    // if (traffic_light_active)
    else // 红绿灯无效下，所有轨迹都可选
    {
        for (int i = 0; i < (int)decisionData.controlTrajectoryList.size(); i++)
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

int32_t getOptimalTrajectoryIndex(DecisionData &decisionData, const prediction::ObjectList &prediction, const infopack::TrafficLight &trafficLight, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu)
{
    decisionData.feasibleTrajectoryIndexList.clear();
    decisionData.feasibleTrajectoryIndexList.reserve(0);

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
    for (int i = 0; i < (int)decisionData.feasibleTrajectoryIndexList.size(); i++)
    {
        tempDistance = assessTrajectory(decisionData.controlTrajectoryList[decisionData.feasibleTrajectoryIndexList[i]], prediction, objectsCmd, imu);
        // std::cout << YELLOW << i << "    TEMPDISTANCE:    " << tempDistance << RESET << std::endl;
        //  20230215  不要最安全的，尽量靠近
        if (tempDistance > SAFE_DISTANCE) // 认为不会碰撞的安全距离
        {

            index = i;
            break; // 找到第一条可行路线就退出
        }
    }

    // return 0; /// no!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // std::cout << "index " <<index << "  decisionData.feasibleTrajectoryIndexList.size()  " <<  decisionData.feasibleTrajectoryIndexList.size() <<std::endl;
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
    for (int i = 0; i < (int)trajectory.planningPoints.size() - 1; i++)
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

// 通过当前点，计算相邻的换道后的最近点 ？？？是预瞄点还是当前点都可以用这个函数
// 找另一条车道上，与当前点最近的点
// 这里只能换一条道，如果要求换多条道路，会导致换道比较缓慢，自作主张了，应该还是根据需求生成换道曲线，然后再对曲线进行评估
bool changeCurrentPoint(const Road &road, const Lane &curLane, int &laneId, int &pointIndex, GaussRoadPoint &currentPoint, bool isLeft)
{
    // 计算换道后的laneID
    int changeId; // 换道后的laneID

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
    for (int i = 0; i < (int)trajectory.planningPoints.size(); i++)
    {
        // len = sqrt(pow(trajectory.planningPoints[i].x - trajectory.planningPoints[0].x, 2) + pow(trajectory.planningPoints[i].y - trajectory.planningPoints[0].y, 2));
        len = sqrt((trajectory.planningPoints[i].x - trajectory.planningPoints[0].x) * (trajectory.planningPoints[i].x - trajectory.planningPoints[0].x) + (trajectory.planningPoints[i].y - trajectory.planningPoints[0].y) * (trajectory.planningPoints[i].y - trajectory.planningPoints[0].y));
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
            globalTrajectoryPoint.curvature = calculateCurvature(trajectory.planningPoints[i], trajectory.planningPoints[i - 1], trajectory.planningPoints[i + 1]);
        }
        globalTrajectoryPoint.s = cumul_s;
        if (i < (int)trajectory.planningPoints.size() - 1)
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
    // std::cout << RED << "333333333333333333333333333333333333333332 " << RESET << std::endl;

    // 轨迹初始化
    // std::cout << decisionData.controlTrajectoryList.size() << std::endl;
    std::vector<PlanningPoint>().swap(decisionData.referenceLine.referenceLinePoints);
    decisionData.referenceLine.referenceLinePoints.reserve(0);

    std::vector<PlanningTrajectory>().swap(decisionData.controlTrajectoryList);
    // decisionData.controlTrajectoryList.clear();
    //  decisionData.controlTrajectoryList.resize(0);
    decisionData.controlTrajectoryList.reserve(0);
    // std::cout << decisionData.controlTrajectoryList.size() << std::endl;
    //  std::cout << RED << "4444444444444444444444444444444444444444 " << RESET << std::endl;

    //???? i < 1
    for (int i = 0; i < 1; i++)
    {
        PlanningTrajectory tempTrajectory;
        for (int j = 0; j < CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER; j++)
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
    decisionData.feasibleTrajectoryIndexList.clear();
    decisionData.feasibleTrajectoryIndexList.reserve(0);

    decisionData.checkLineVector.clear();
    decisionData.checkLineVector.reserve(0);
}

// 返回decisionDate中roadID 和laneID的索引
std::tuple<int32_t, int32_t> fixRoadLaneIndex(const DecisionData &decisionData, const RoadMap &map)
{
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
        // return 2 / sqrt(pow(x, 2) + pow(y, 2));
        return 2 / sqrt(x * x + y * y);
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

// 这是在车辆坐标系下进行的距离计算
double getMinDistanceOfPoint(const PlanningPoint &point, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu)
{
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
        //     (object.predictpoint(0).y() - object.l() / 2) < 2 && (object.predictpoint(0).y() + object.l() / 2) > -2)
        if ((object.predictpoint(0).x() + object.w() / 2 + LASER_OFFSET_FRONT > -30) && (object.predictpoint(0).x() - object.w() / 2 + LASER_OFFSET_FRONT < MAX_FRENET_S(imu.velocity())) &&
            (object.predictpoint(0).y() - object.l() / 2) < 30 && (object.predictpoint(0).y() + object.l() / 2) > -30)
        {
            predictTempPoint.x = -object.predictpoint(index).y();
            predictTempPoint.y = object.predictpoint(index).x() + LASER_OFFSET_FRONT;

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
    for (int i = 0; i < (int)objectsCmd.size(); i++)
    {
        const infopack::ObjectsProto objProto = objectsCmd[i];
        if (objProto.type() == myselfVehicleeType && objProto.objectid() == myselfVehicleID) // 不自车
        {
            continue;
        }

        // 转换障碍物的xy经纬度为平面坐标，
        // 20230801 现在假设路测输入的障碍物都有平面坐标，不用再作转换了，节省点时间,后续待测试
        // double latitudeTemp = objProto.lat();
        // double longitudeTemp = objProto.lon();
        // double gaussNorthTemp, gaussEastTemp;
        // gaussConvert(longitudeTemp, latitudeTemp, gaussNorthTemp, gaussEastTemp);

        double gaussNorthTemp, gaussEastTemp;
        gaussNorthTemp = objProto.x();
        gaussEastTemp = objProto.y();

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
            CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow); //  前左
            // std::cout << "distanceFromObject:" << j << " " << point.x << " " << point.y << " " << -pointTemp[j][1] << " " << pointTemp[j][0] << " " << getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]) << std::endl; // you qian
            // 这里计算的是与障碍物四个角点的距离，这个有点不对，如果障碍物很大，而点在障碍物之内，这个计算就不能处理这种问题
            distanceFromObject.push_back(getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]));
        }

        // 这部分为 采用点到多边形的距离公式计算，待验证
        //  //计算点到多边形的距离,gauss坐标系是北东天，即为上右
        //  GaussRoadPoint checkPoint;
        //  checkPoint.GaussX =  point.y; //车辆的坐标系为右前
        //  checkPoint.GaussY =  point.x;

        // GaussRoadPoint gaussPointTemp;
        // std::vector<GaussRoadPoint> pointPolygon;

        // for (int j = 0; j < 4; j++)
        // {
        //     CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow); //  前左
        //     // std::cout << "distanceFromObject:" << j << " " << point.x << " " << point.y << " " << -pointTemp[j][1] << " " << pointTemp[j][0] << " " << getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]) << std::endl; // you qian
        //     //这里计算的是与障碍物四个角点的距离，这个有点不对，如果障碍物很大，而点在障碍物之内，这个计算就不能处理这种问题
        //     gaussPointTemp.GaussX =   pointTemp[j][0];
        //     gaussPointTemp.GaussY =   -pointTemp[j][1];
        //     pointPolygon.push_back(gaussPointTemp);
        // }

        // double distanceTemp =  pointToPolygon( pointPolygon, checkPoint);
        // std::cout << "distanceTemp" << distanceTemp <<std::endl;
        // distanceFromObject.push_back(distanceTemp);
    }

    return findMin(distanceFromObject);
}

// 这是在车辆坐标系下进行的距离平方的计算，减少开根号的计算
// 之前是把把所有的距离计算出来，然后这个再提取最小点，这个完全没必要，直接在循环中比较即可
// 这个计算的结果，主要用于安全距离的判断，完全可以发现不满足安全距离即刻中断返回
double getMinDistanceOfPointCheckSafety(const PlanningPoint &point, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu, double safeDistance)
{
    int index = 0; // 因为预测中包含了很多位置，现在没有预测，只用第一点
    // double tRemainder;
    PlanningPoint predictTempPoint;       // 预测障碍物的临时变量
    double minDistanceSquare = 100 * 100; // 障碍物与车辆的最小距离平方，初始化为100米。100米是一个完全安全的距离了
    double distanceSquareTemp;            // 计算出来的障碍物与车辆的距离平方临时变量
    double safeDistanceSquare = safeDistance * safeDistance;
    std::vector<double> distanceFromObject;
    // index = (int)t / PREDICT_FREQUENCY;
    // tRemainder = t - index * PREDICT_FREQUENCY;

    // std::cout << "处理激光感知障碍物信息"  << std::endl;
    // 处理激光感知障碍物信息
    for (auto object : prediction.object()) // 激光感知障碍物，现在是小方框集合，只计算到该点的距离
    {
        // 20230214   减少碰撞检测物体数量，目前是后30米，前在最大规划轨迹之后（？？？这个可能不对，对于远处无法检测到了），左右30米
        // if ((object.predictpoint(0).x() + object.w() / 2 + LASER_OFFSET_FRONT > -2) && (object.predictpoint(0).x() - object.w() / 2 + LASER_OFFSET_FRONT < MAX_FRENET_S(imu.velocity())) &&
        //     (object.predictpoint(0).y() - object.l() / 2) < 2 && (object.predictpoint(0).y() + object.l() / 2) > -2)

        // if ((object.predictpoint(0).x() + object.w() / 2 + LASER_OFFSET_FRONT > -30) &&
        //     (object.predictpoint(0).x() - object.w() / 2 + LASER_OFFSET_FRONT < MAX_FRENET_S(imu.velocity())) &&
        //     (object.predictpoint(0).y() - object.l() / 2) < 30 &&
        //     (object.predictpoint(0).y() + object.l() / 2) > -30)
        {
            // 将预测障碍无的前左坐标系转为右前笛卡尔坐标系
            predictTempPoint.x = -object.predictpoint(index).y();
            predictTempPoint.y = object.predictpoint(index).x() + LASER_OFFSET_FRONT;

            // std::cout << "处理激光感知障碍物信息"  << predictTempPoint.x << ","<<predictTempPoint.y<<  std::endl;

            distanceSquareTemp = (point.x - predictTempPoint.x) * (point.x - predictTempPoint.x) + (point.y - predictTempPoint.y) * (point.y - predictTempPoint.y);
            if (minDistanceSquare > distanceSquareTemp) // 取较小的距离值
            {
                minDistanceSquare = distanceSquareTemp;

                if (minDistanceSquare < safeDistanceSquare) // 已经都不安全了，就直接退出了
                    return std::sqrt(minDistanceSquare);
            }
        }
    }
    ////////////////////////////////////////////////////////////////////
    // std::cout << "处理路测感知障碍物信息"  << std::endl;
    // 处理路测感知障碍物信息
    for (int i = 0; i < (int)objectsCmd.size(); i++)
    {
        const infopack::ObjectsProto objProto = objectsCmd[i];
        if (objProto.type() == myselfVehicleeType && objProto.objectid() == myselfVehicleID) // 不自车
        {
            continue;
        }

        // 转换障碍物的xy经纬度为平面坐标，
        // 20230801 现在假设路测输入的障碍物都有平面坐标，不用再作转换了，节省点时间,后续待测试
        // double latitudeTemp = objProto.lat();
        // double longitudeTemp = objProto.lon();
        // double gaussNorthTemp, gaussEastTemp;
        // gaussConvert(longitudeTemp, latitudeTemp, gaussNorthTemp, gaussEastTemp);

        double gaussNorthTemp, gaussEastTemp;
        gaussNorthTemp = objProto.x();
        gaussEastTemp = objProto.y();

        // 计算4个顶点平面坐标 xy  shi cheng liang you qian
        // 其实，这部分可以在接收到或者处理开始之前数据初始化的时候进行计算，没必要每个点都要计算一次障碍物的位置尺寸，运算量大大的呀
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
            CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow);
            // std::cout << "distanceFromObject:" << j << " " << point.x << " " << point.y << " " << -pointTemp[j][1] << " " << pointTemp[j][0] << " " << getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]) << std::endl; // you qian
            // 这里计算的是与障碍物四个角点的距离，这个有点不对，如果障碍物很大，而点在障碍物之内，这个计算就不能处理这种问题
            // distanceFromObject.push_back(getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]));
            distanceSquareTemp = (point.x + pointTemp[j][1]) * (point.x + pointTemp[j][1]) + (point.y - pointTemp[j][0]) * (point.y - pointTemp[j][0]);
            if (minDistanceSquare > distanceSquareTemp) // 取较小的距离值
            {
                minDistanceSquare = distanceSquareTemp;

                if (minDistanceSquare < safeDistanceSquare) // 已经都不安全了，就直接退出了
                    return std::sqrt(minDistanceSquare);
            }
        }

        // 这部分为 采用点到多边形的距离公式计算，待验证
        //  //计算点到多边形的距离,gauss坐标系是北东天，即为上右
        //  GaussRoadPoint checkPoint;
        //  checkPoint.GaussX =  point.y; //车辆的坐标系为右前
        //  checkPoint.GaussY =  point.x;

        // GaussRoadPoint gaussPointTemp;
        // std::vector<GaussRoadPoint> pointPolygon;

        // for (int j = 0; j < 4; j++)
        // {
        //     CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow); //  前左
        //     // std::cout << "distanceFromObject:" << j << " " << point.x << " " << point.y << " " << -pointTemp[j][1] << " " << pointTemp[j][0] << " " << getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]) << std::endl; // you qian
        //     //这里计算的是与障碍物四个角点的距离，这个有点不对，如果障碍物很大，而点在障碍物之内，这个计算就不能处理这种问题
        //     gaussPointTemp.GaussX =   pointTemp[j][0];
        //     gaussPointTemp.GaussY =   -pointTemp[j][1];
        //     pointPolygon.push_back(gaussPointTemp);
        // }

        // double distanceTemp =  pointToPolygon( pointPolygon, checkPoint);
        // std::cout << "distanceTemp" << distanceTemp <<std::endl;
        // distanceFromObject.push_back(distanceTemp);
    }

    return std::sqrt(minDistanceSquare);
}

double findMin(const std::vector<double> &array)
{
    // 这个地方会有一个问题，就是本身没有障碍物，结果返回一个-1，直接作距离安全判断的时候容易出现误判，需要注意对返回值进行检查
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
ReferenceLine getReferenceLine(const RoadMap &map, const DecisionData &decisionData, GaussRoadPoint locationPoint, const pc::Imu &imu)
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

    // 补充一下这些变量的赋值
    frenetCurrentPoint.roadID = map.roads[roadIndex].id;
    frenetCurrentPoint.laneID = map.roads[roadIndex].lanes[laneIndex].id;
    frenetCurrentPoint.pointID = decisionData.currentIndex;

    double deltaX = currentPoint.GaussX - locationPoint.GaussX;
    double deltaY = currentPoint.GaussY - locationPoint.GaussY;
    frenetCurrentPoint.x = (deltaY)*cos(locationPoint.yaw / 180.0 * M_PI) - (deltaX)*sin(locationPoint.yaw / 180.0 * M_PI);
    frenetCurrentPoint.y = (deltaX)*cos(locationPoint.yaw / 180.0 * M_PI) + (deltaY)*sin(locationPoint.yaw / 180.0 * M_PI);
    frenetCurrentPoint.angle = currentPoint.yaw - locationPoint.yaw;
    frenetCurrentPoint.s = 0;
    frenetCurrentPoint.l = 0;
    frenetCurrentPoint.gaussX = currentPoint.GaussX;
    frenetCurrentPoint.gaussY = currentPoint.GaussY;

    frenetCurrentPoint.roadIDIndex = roadIndex;
    frenetCurrentPoint.laneIDIndex = laneIndex;

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
        if (deltaS <= MAX_FRENET_S(imu.velocity()))
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
            frenetReferencePoint.roadID = map.roads[roadIndex].id;
            frenetReferencePoint.laneID = map.roads[roadIndex].lanes[laneIndex].id;
            frenetReferencePoint.pointID = index;
            frenetReferencePoint.gaussX = referencePoint.GaussX;
            frenetReferencePoint.gaussY = referencePoint.GaussY;

            frenetReferencePoint.roadIDIndex = roadIndex;
            frenetReferencePoint.laneIDIndex = laneIndex;

            // std::cout << "xxxxxxxxxxxxxxxxxx: roadIndex = "<< roadIndex <<"; "<<laneIndex << "," <<index << std::endl;

            referenceLine.referenceLinePoints.push_back(frenetReferencePoint);
        }
        else
        {
            return referenceLine;
        }
    }
    // if the remaining distance less than MAX_FRENET_S(imu.velocity()), then check !!!the successor road lane
    // get next road id and lane id
    //??????这里出现的问题就是，只能找一条后继道路，如果后继道路很短，如十字路口的连接道路，reference line就很短
    int nextRoadIndex, nextLaneIndex;
    int32_t lastRoadIndex, lastLaneIndex; // 刚才处理的道路索引 和 lane索引
    // nextRoadIndex = map.roads[roadIndex].successorId;
    lastRoadIndex = roadIndex;
    lastLaneIndex = laneIndex;

    double accumS1 = map.roads[roadIndex].lanes[laneIndex].gaussRoadPoints.back().s - currentPoint.s; // 之前段路的累计S
    double accumS2;                                                                                   // 之前 两段路首尾之间的距离

    for (int j = 0; j < decisionData.nextIdList.size(); j++) // 后续所有路的循环
    {
        nextRoadIndex = -1;
        nextLaneIndex = -1;
        // 查找下一段的road\lane index
        for (int m = 0; m < map.roads.size(); m++)
        {
            if (std::get<0>(decisionData.nextIdList[j]) == map.roads[m].id)
            {
                nextRoadIndex = m; // 找到路
                break;
            }
        }
        // 20230922如果这一条路与上一条路是同一条路，只是换道，就不能在这个lane上寻找参考线，
        if (lastRoadIndex == nextRoadIndex)
            continue;

        if (nextRoadIndex != -1)
        {
            for (int n = 0; n < map.roads[nextRoadIndex].lanes.size(); n++)
            {

                if (std::get<1>(decisionData.nextIdList[j]) == map.roads[nextRoadIndex].lanes[n].id)
                {
                    nextLaneIndex = n;
                    break;
                }
            }
        }

        if (nextRoadIndex == -1 || nextLaneIndex == -1)
        {
            break;
        }

        // std::cout << "reference next  road lane ID: " << map.roads[nextRoadIndex].id << "," << map.roads[nextRoadIndex].lanes[nextLaneIndex].id << std::endl;

        accumS2 = getDistance(map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints.back().GaussX, map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints.back().GaussY,
                              map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.front().GaussX, map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.front().GaussY); // 之前 两段路首尾之间的距离

        // std::cout << "accumS2 " << std::setprecision(10) << map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints.back().GaussX << ","
        //           << map.roads[lastRoadIndex].lanes[lastLaneIndex].gaussRoadPoints.back().GaussY << ";"
        //           << map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.front().GaussX << ","
        //           << map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.front().GaussY << std::endl;

        for (int index = 0; index < map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.size(); index++)
        {

            // 计算如果增加这一点后的累计长度

            deltaS = map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints[index].s + accumS1 + accumS2; // 累计S
                                                                                                                 // std::cout << "xxxxxxdeltaS "<<deltaS << " " <<  accumS1 << " "<< accumS2<<std::endl;
            if (deltaS <= MAX_FRENET_S(imu.velocity()))
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
                frenetReferencePoint.roadID = map.roads[nextRoadIndex].id;
                frenetReferencePoint.laneID = map.roads[nextRoadIndex].lanes[nextLaneIndex].id; // 20230920修改
                frenetReferencePoint.pointID = index;
                frenetReferencePoint.gaussX = referencePoint.GaussX;
                frenetReferencePoint.gaussY = referencePoint.GaussY;

                frenetReferencePoint.roadIDIndex = nextRoadIndex;
                frenetReferencePoint.laneIDIndex = nextLaneIndex;

                // std::cout << "yyyyyyyyyyyyyyyyyyyy: "<< referencePoint.GaussX <<"; "<<referencePoint.GaussY << std::endl;
                referenceLine.referenceLinePoints.push_back(frenetReferencePoint);
            }
            else
            {
                return referenceLine;
            }

        } // 处理这一段路的每一个路点

        accumS1 += map.roads[nextRoadIndex].lanes[nextLaneIndex].gaussRoadPoints.back().s; // 之前段路的累计S
        lastRoadIndex = nextRoadIndex;
        lastLaneIndex = nextLaneIndex;

    } // for(int j=0;j<decisionData.nextIdList.size();j++)//后续所有路的循环

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

// 计算frenet坐标系下的bezier曲线,该函数只能生成链接首尾的曲线
// void generateBezierPathListInFrenet(const ReferenceLine &referenceLine, const PlanningPoint &frenetLocationPoint, std::vector<PlanningTrajectory> &pathList)
// {
//     double s = referenceLine.referenceLinePoints.back().s;

//     PlanningPoint startPoint, endPoint; // frenet  坐标系下起点和终点的坐标???
//     // startPoint.s = frenetLocationPoint.s;
//     // startPoint.l = frenetLocationPoint.l;
//     // std:: cout << RED << "frenetLocationPoint.l: " << startPoint.l << std::endl;
//     // std:: cout << RED << "referenceLine.referenceLinePoints.front().yaw: " << referenceLine.referenceLinePoints.front().yaw << std::endl;
//     // startPoint.frenetAngle = 90 - referenceLine.referenceLinePoints.front().yaw;
//     startPoint = frenetLocationPoint; // 车辆位置在frenet坐标系下坐标
//     // std::cout << RED << "startPoint " << startPoint.s << "startPoint frenet angle " << startPoint.frenetAngle << std::endl;
//     // std::cout << RED << "startPoint " << startPoint.l << "startPoint frenet angle " << startPoint.frenetAngle << std::endl;

//     // std::cout << RED << "angleeeeeeeeeeeeeeeeeeeeeeeeeeeeeee: " << startPoint.frenetAngle << std::endl;
//     endPoint.s = s;
//     endPoint.frenetAngle = 0;
//     // std::cout << RED << "endPoint " << endPoint.s << std::endl;

//     // std::cout << RED << "referenceLine.referenceLinePoints.back().yaw: " << referenceLine.referenceLinePoints.back().frenetAngle << std::endl;

//     PlanningTrajectory curve;
//     //根据下一条路的设定，获取贝塞尔曲线的数量和宽度，如果后续没路，则用当前路点的

//     for (int i = 0; i < CURVE_NUM; i++)
//     {
//         // 终点位置l，分别向两侧递增
//         if (i % 2 == 0)
//         {
//             endPoint.l = -(i + 1) / 2 * CURVE_DISTANCE;
//         }
//         else
//         {
//             endPoint.l = (i + 1) / 2 * CURVE_DISTANCE;
//         }
//         //  std:: cout << RED << "endPoint.l: " << endPoint.l << RESET <<std::endl;

//         generateBezierPathInFrenet(startPoint, endPoint, curve);

//         // std::cout << RED << "endPoint.l: " << curve.planningPoints.back().l << RESET <<std::endl;
//         // std:: cout << RED << "curve.size: " << curve.planningPoints.size() << RESET <<std::endl;
//         pathList.push_back(curve);
//         curve.planningPoints.clear();
//     }

//     // for (int i = 0; i < CURVE_NUM; i++)
//     // {
//     //   if (i % 2 == 0)
//     //   {
//     //     firstEndPointList[i].x = firstEndPoint.x + (i + 1) / 2 * CURVE_DISTANCE * sin(firstTheta / 180 * M_PI);
//     //     firstEndPointList[i].y = firstEndPoint.y + (i + 1) / 2 * CURVE_DISTANCE * cos(firstTheta / 180 * M_PI);
//     //     firstEndPointList[i].angle = firstEndPoint.angle;
//     //   }
//     //   else
//     //   {
//     //     firstEndPointList[i].x = firstEndPoint.x - (i + 1) / 2 * CURVE_DISTANCE * sin(firstTheta / 180 * M_PI);
//     //     firstEndPointList[i].y = firstEndPoint.y - (i + 1) / 2 * CURVE_DISTANCE * cos(firstTheta / 180 * M_PI);
//     //     firstEndPointList[i].angle = firstEndPoint.angle;
//     //   }
//     //   firstEndPointList.push_back(firstEndPointList[i]);
//     // }
// }

// 计算frenet坐标系下的bezier曲线,将reference分成多段，生成多段bezier拼接的轨迹
// 目前只是简单拼接，后续应保留每个分段的属性，以优化碰撞检测等
void generateSegmentedBezierPathListInFrenet(const RoadMap &map, const ReferenceLine &referenceLine, const PlanningPoint &frenetLocationPoint, std::vector<PlanningTrajectory> &pathList)
{
    double s = referenceLine.referenceLinePoints.back().s;

    // 确定分成几段，目前简单设定值，后续要考虑停止位置附近参考线短了的问题
    // SEGMENTED_FRENET_NUMBER
    int nSegmentNum = floor(s / 5);

    std::vector<std::vector<PlanningPoint>> segmentedPointList; // frenet  坐标系下各个分段的起点
    std::vector<PlanningPoint> pointList;                       // frenet  坐标系下某个分段的多个起点
    PlanningPoint endPoint;                                     // frenet  坐标系下某个分段的起点

    // 第一段的起点就是车辆当前位置
    pointList.clear();
    pointList.reserve(0);
    pointList.push_back(frenetLocationPoint);
    segmentedPointList.push_back(pointList);

    // 获取贝塞尔曲线的条数和距离

    int endPointRoadIndexTemp = referenceLine.referenceLinePoints.back().roadIDIndex;
    int endPointLaneIndexTemp = referenceLine.referenceLinePoints.back().laneIDIndex;

    // std::cout << "endPointRoadIndexTemp = " << endPointRoadIndexTemp << "endPointLaneIndexTemp = " << endPointLaneIndexTemp
    //           << std::endl;

    int curve_num_temp = map.roads[endPointRoadIndexTemp].lanes[endPointLaneIndexTemp].BazierCurNUM;
    curve_num_temp = std::max(1, curve_num_temp);
    double curve_distance_temp = map.roads[endPointRoadIndexTemp].lanes[endPointLaneIndexTemp].BazierCurDIS;
    curve_distance_temp = std::max(0.0, curve_distance_temp);
    // std::cout << "endPointRoadIndexTemp = " << endPointRoadIndexTemp << "endPointLaneIndexTemp = " << endPointLaneIndexTemp
    //           << "curve_num_temp=" << curve_num_temp << "curve_distance_temp =" << curve_distance_temp << std::endl;

    // 生成从第一段到最后一段的起点，注意，如果是停止位的话，应该最后一段的终端只有一个，目前未处理这个情况
    for (int i = 1; i <= SEGMENTED_FRENET_NUMBER; i++)
    {
        pointList.clear();
        pointList.reserve(0);

        // for (int j = 0; j < CURVE_NUM; j++) // 每个分段
        for (int j = 0; j < curve_num_temp; j++) // 每个分段
        {
            endPoint.s = s * i / SEGMENTED_FRENET_NUMBER;
            endPoint.frenetAngle = 0;
            // 终点位置l，分别向两侧递增
            if (j % 2 == 0)
            {
                // endPoint.l = -(j + 1) / 2 * CURVE_DISTANCE;
                endPoint.l = -(j + 1) / 2 * curve_distance_temp;
            }
            else
            {
                // endPoint.l = (j + 1) / 2 * CURVE_DISTANCE;
                endPoint.l = (j + 1) / 2 * curve_distance_temp;
            }

            pointList.push_back(endPoint);
            // std::cout<<"endPoint.s = " << endPoint.s <<" endPoint.l=" <<endPoint.l<<",";
        } // for(int j=0;j < CURVE_NUM; j++)//每个分段
        segmentedPointList.push_back(pointList);
        // std::cout<<"segmentedPointList.push_back(pointList) = " << segmentedPointList.size()<<std::endl;
    } // for(int i=0;i<SEGMENTED_FRENET_NUMBER;i++)

    // 生成多分段的曲线,
    std::vector<std::vector<PlanningTrajectory>> segmentedCurveList;
    std::vector<PlanningTrajectory> curveList;
    PlanningTrajectory curve;

    for (int i = 0; i < SEGMENTED_FRENET_NUMBER; i++)
    {
        curveList.clear();
        curveList.reserve(0);
        for (int j = 0; j < segmentedPointList[i].size(); j++)
        {
            for (int k = 0; k < segmentedPointList[i + 1].size(); k++)
            {
                PlanningTrajectory curve;
                generateBezierPathInFrenet(segmentedPointList[i][j], segmentedPointList[i + 1][k], curve);
                // std::cout<<"start/end " << i <<" ," <<j<<" /" << i+1<<","<<k<<";";
                curveList.push_back(curve);
            }
        }
        segmentedCurveList.push_back(curveList);
        // std::cout<<"segmentedCurveList" << i <<"curveList.size ," <<curveList.size() <<std::endl;
    }

    // //拼接分段曲线,应该是个递归
    pathList.clear();
    int pathListSize = 1; // 总共的规划曲线数量
    for (int i = 0; i <= SEGMENTED_FRENET_NUMBER; i++)
    {
        pathListSize = pathListSize * segmentedPointList[i].size();
    }
    pathList.reserve(pathListSize);
    // std::cout<<"pathListSize = " << pathListSize <<std::endl;
    // 第一段

    pathList.insert(pathList.end(), segmentedCurveList[0].begin(), segmentedCurveList[0].end());
    // std::cout<<"pathList.insert = " << pathList.size() <<std::endl;

    // 第二段之后
    PlanningTrajectory curveResult;
    for (int i = 1; i < SEGMENTED_FRENET_NUMBER; i++)
    {
        // 自身复制n倍
        std::vector<PlanningTrajectory> TempPathList(pathList);
        for (int j = 0; j < TempPathList.size(); j++)
        {
            pathList.insert(pathList.begin() + j * segmentedPointList[i + 1].size(), segmentedPointList[i + 1].size() - 1, TempPathList[j]);
        }

        // std::cout << "pathList.size() = " <<pathList.size() << ":"<<std::endl;

        // 下一段补充在当前段
        for (int j = 0; j < pathList.size(); j++)
        {
            pathList[j].planningPoints.insert(pathList[j].planningPoints.end(),
                                              segmentedCurveList[i][(j % segmentedCurveList[i].size())].planningPoints.begin(),
                                              segmentedCurveList[i][(j % segmentedCurveList[i].size())].planningPoints.end());

            // std::cout << "segmentedCurveList " <<i<< "," << j%segmentedCurveList[i].size()<< ";" ;
        }
    }

    // std::cout << "pathList------------------- " <<pathList.size()<<std::endl;
    // for(int i=0;i < (int) pathList.size(); i++)
    // {
    //     std::cout << "pathList[i].planningPoints.size() = " <<pathList[i].planningPoints.size()<<std::endl;
    // }
    // <<" pathList[0].planningPoints.size()" << pathList[0].planningPoints.size()
    // <<" pathList[0].planningPoints[n].s=" << pathList[0].planningPoints[pathList[0].planningPoints.size()-1].s
    // <<" pathList[0].planningPoints[n].l=" << pathList[0].planningPoints[pathList[0].planningPoints.size()-1].l
    // <<" pathList[n].planningPoints[0].s=" << pathList[pathList.size()-1].planningPoints[0].s
    // <<" pathList[n].planningPoints[0].l=" << pathList[pathList.size()-1].planningPoints[0].l
    // <<" pathList[n].planningPoints[n].s=" << pathList[pathList.size()-1].planningPoints[pathList[pathList.size()-1].planningPoints.size()-1].s
    // <<" pathList[n].planningPoints[n].l=" << pathList[pathList.size()-1].planningPoints[pathList[pathList.size()-1].planningPoints.size()-1].l<<std::endl;
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

// // 车辆前方是否有需要跟随的车辆,目前这个逻辑有点简单
// bool hasPrecedingVehicle(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> objectsCmd)
// {

//     for (int i = 0; i < (int) objectsCmd.size(); i++)
//     {
//         if (objectsCmd[i].type() == 0 && objectsCmd[i].objectid() == ACC_FOLLOW_OBJECT_ID) // 这个待修改，应该是锁定车辆ID
//         {
//             double dist = getDistance(imu.gaussx(), imu.gaussy(), objectsCmd[i].x(), objectsCmd[i].y()); // 两车距离
//             double angle = atan2(objectsCmd[i].x() - imu.gaussx(), objectsCmd[i].y() - imu.gaussy());    // 两车连线夹角
//             angle = int(angle * 180 / 3.14 + 360) % 360;                                                 // 转角度，范围为0-360
//             double yaw = int(90 - imu.yaw() + 360) % 360;
//             std::cout << "+++++++++++++++++ACC begin distance " << dist << " angle " << angle << " yaw " << yaw << std::endl;
//             // if (dist < ACC_BEGIN_DIST && abs(angle - yaw) < 90) // 距离较近，且前后关系正确
//             if (dist < ACC_BEGIN_DIST) // 距离较近，且前后关系正确
//             {
//                 return true;
//             }
//         }
//     }

//     // std::cout << "-------------------------ACC  NONE  " << std::endl;
//     return false;
// }
// 车辆前方是否有需要跟随的车辆,目前这个逻辑有点简单
// 20230803优化
bool hasPrecedingVehicle(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> &objectsCmd)
{
    for (auto &objProto : objectsCmd)
    // for (int i = 0; i < (int) objectsCmd.size(); i++)
    {
        if (objProto.type() == 0 && objProto.objectid() == ACC_FOLLOW_OBJECT_ID) // 这个待修改，应该是锁定车辆ID
        {
            double dist = getDistance(imu.gaussx(), imu.gaussy(), objProto.x(), objProto.y()); // 两车距离
            double angle = atan2(objProto.x() - imu.gaussx(), objProto.y() - imu.gaussy());    // 两车连线夹角
            angle = int(angle * 180 / 3.14 + 360) % 360;                                       // 转角度，范围为0-360
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
// double cruiseController(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> objectsCmd)
// {

//     int objectIndex = -1;
//     for (int i = 0; i < (int)objectsCmd.size(); i++)
//     {
//         if (objectsCmd[i].type() == 0 && objectsCmd[i].objectid() == ACC_FOLLOW_OBJECT_ID) // 这个待修改，应该是锁定车辆ID
//         {
//             objectIndex = i;
//             break;
//         }
//     }

//     if (objectIndex == -1)
//         return -1;

//     double distance = getDistance(imu.gaussx(), imu.gaussy(), objectsCmd[objectIndex].x(), objectsCmd[objectIndex].y()); // 两车距离
//     double distanceDiff = distance - ACC_RELA_DIST;
//     double speedDiff = objectsCmd[objectIndex].velocity() - imu.velocity();

//     // double kDistance = 0.5, kSpeed = 0.5; // old 1 0.5
//     double kDistance = 0.5, kSpeed = 0.5;
//     //static bool b_hasStopped = false;

//     double targetSpeed = kDistance * distanceDiff + kSpeed * speedDiff + objectsCmd[objectIndex].velocity();
//     std::cout << "distanceDiff " << distanceDiff << "speedDiff " << speedDiff << " objectsCmd[objectIndex].velocity()" << objectsCmd[objectIndex].velocity() << "targetSpeed" << targetSpeed << std::endl;

//     // 从decision-functon.cpp中拷贝过来，暂时不知道啥用处，注释掉了
//     if (objectsCmd[objectIndex].velocity() < 0.1)
//     {
//        if (distanceDiff > 0.7)
//          targetSpeed = 0.7;
//        else
//          targetSpeed = 0;
//      }

//     // std::cout << "[ACC]   PV  speed: " << iovData.vehicleSpeed << std::endl;
//     // std::cout << "[ACC] inter dist.: " << distance << std::endl;
//     // std::cout << GREEN << "[ACC] dist. error: " << RESET << distanceDiff << std::endl;
//     // std::cout << GREEN << "[ACC] speed error: " << RESET << speedDiff << std::endl;
//     // std::cout << "[ACC] 1st  target: " << targetSpeed << std::endl;
//     // if (targetSpeed < 0.5)
//     // {
//     //   targetSpeed = 0;
//     //   b_hasStopped = true;
//     // }
//     // else if (targetSpeed >= 0.5 && targetSpeed < 1.8)
//     // {
//     //   if (true == b_hasStopped)
//     //   {
//     //     targetSpeed = 0.0;
//     //   }
//     //   else
//     //   {
//     //     //  	  targetSpeed = 1.0;
//     //   }
//     // }
//     // else
//     // {
//     //   b_hasStopped = false;
//     // }
//     // if (targetSpeed > 8)
//     // {
//     //   targetSpeed = 8;
//     //   std::cout << "! Top Cruise Speed Limited" << std::endl;
//     // }
//     // if (distance < ACC_MINIMAL_DISTANCE)
//     // {
//     //   targetSpeed = 0;
//     //   b_hasStopped = true;
//     // }
//     // else
//     //   ;
//     // // std::cout << "[CV] targetSpeed = " << targetSpeed << " m/s\n";
//     // return targetSpeed;

//     if (targetSpeed <= 0)
//         targetSpeed = 0;
//     return targetSpeed;
// }
// double cruiseController(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> objectsCmd, double objectIndex, double carDistance)
// {

//     double distance = carDistance; // 两车距离
//     double distanceDiff = distance - ACC_RELA_DIST;
//     double speedDiff = objectsCmd[objectIndex].velocity() - imu.velocity();

//     // double kDistance = 0.5, kSpeed = 0.5; // old 1 0.5
//     double kDistance = 0.5, kSpeed = 0.5;
//     // static bool b_hasStopped = false;

//     double targetSpeed = kDistance * distanceDiff + kSpeed * speedDiff + objectsCmd[objectIndex].velocity();
//     std::cout << "distanceDiff " << distanceDiff << "speedDiff " << speedDiff << " objectsCmd[objectIndex].velocity()" << objectsCmd[objectIndex].velocity() << "targetSpeed" << targetSpeed << std::endl;

//     // 从decision-functon.cpp中拷贝过来，暂时不知道啥用处，注释掉了
//     // if (objectsCmd[objectIndex].velocity() < 0.1)
//     // {
//     //     if (distanceDiff > 0.7)
//     //         targetSpeed = 0.7;
//     //     else
//     //         targetSpeed = 0;
//     // }
//     // if (targetSpeed <= 0)
//     //     targetSpeed = 0;
//     return targetSpeed;
// }
double cruiseController(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> &objectsCmd, double objectIndex, double carDistance)
{

    double distance = carDistance; // 两车距离
    double distanceDiff = distance - (ACC_RELA_DIST+ imu.velocity()*2.0);
    double speedDiff = objectsCmd[objectIndex].velocity() - imu.velocity();

    // double kDistance = 0.5, kSpeed = 0.5; // old 1 0.5
    double kDistance = 0.5, kSpeed = 0.5;
    // static bool b_hasStopped = false;

    double targetSpeed = kDistance * distanceDiff + kSpeed * speedDiff + objectsCmd[objectIndex].velocity();
    std::cout << "distanceDiff " << distanceDiff << "speedDiff " << speedDiff << " objectsCmd[objectIndex].velocity()" << objectsCmd[objectIndex].velocity() << "targetSpeed" << targetSpeed << std::endl;

    // 从decision-functon.cpp中拷贝过来，暂时不知道啥用处，注释掉了
    if (objectsCmd[objectIndex].velocity() < 0.5)
    {
        if (distanceDiff > 0.7)
            targetSpeed = 0.7;
        else
            targetSpeed = 0;
    }
    if (targetSpeed <= 0)
        targetSpeed = 0;
    return targetSpeed;
}
// 在可选的换道路线中，选择一条可用的路线，作为换道的路线
bool SelectBestRoutingList(RoadMap map, std::vector<RoutingList> routingListVector, std::vector<std::tuple<int32_t, int32_t>> &bestRoutingList)
{
    if (routingListVector.size() <= 1) // 就这一条，没得可选，因为中间道路已经被检测过，所以无需检测，直接从后续的规划roadlane中找
        return false;

    // 构造可用的referenceline
    // bool GetCheckLine(map, std::vector<std::tuple<int32_t, int32_t>>  routingList, int roadID, int laneID, int pointID,double dRefLineLength,ReferenceLine &checkLine)

    // 对每条线路前方安全距离的检查
    for (int i = 1; i < (int)routingListVector.size(); i++)
    {
        // 构造check lane
    }

    // 对车辆后方安全距离的检查

    // 设置安全距离，先把参数放在这里，安全距离 = 基本安全距离+ 加速距离 + 换道惩罚距离
    // 基本安全距离，先设置5米
    // 加速距离 按照初速度为0，恒加速度计算达到道路限速一半的行驶距离，这个公式不知对不对 s = v*v/8
    // 换道惩罚距离，是为了防止来回换道，我们现在的速度很慢，所以 没跨一条道增加10米
    double dBaseSafeDistance = 5;                                   // 基本安全距离
    double dAccelerateDistance = DESIRED_SPEED * DESIRED_SPEED / 8; // 现在速度很低。这个值很小呢
    double dChangenDistance = 10;
    double dSafeDistance = dBaseSafeDistance + dAccelerateDistance;

    // 计算每条路的障碍物距离，目前只处理路测障碍物，激光感知的障碍物数据量有点大，
}

bool GetOptimalGlobalTrajectory(const pc::Imu &imu, const RoadMap &map, DecisionData &decisionData, const prediction::ObjectList &predictionMsg, const infopack::TrafficLight &trafficLights, const std::vector<infopack::ObjectsProto> &objectsCmd, int tempCurrentLaneId, int tempCurrentIndex, const std::vector<GaussRoadPoint> stopPoints)
{

    int currentLaneId = decisionData.currentLaneId;
    int currentIndex = decisionData.currentIndex;

    decisionData.currentLaneId = tempCurrentLaneId;
    decisionData.currentIndex = tempCurrentIndex;

    // std::cout << BOLDCYAN << "GetOptimalGlobalTrajectory " << decisionData.currentLaneId << "; " << decisionData.currentIndex<<RESET << std::endl;
    //  imu定位确定的当前全局坐标
    GaussRoadPoint locationPoint;
    locationPoint.GaussX = imu.gaussx();
    locationPoint.GaussY = imu.gaussy();
    locationPoint.yaw = imu.yaw();

    // 生成参考线 referenceLine.referenceLinePoints

    std::vector<PlanningPoint>().swap(decisionData.referenceLine.referenceLinePoints);
    decisionData.referenceLine.referenceLinePoints.reserve(0);
    decisionData.referenceLine = getReferenceLine(map, decisionData, locationPoint, imu);

    // std::cout << "getReferenceLine  " << decisionData.referenceLine.referenceLinePoints.size() << std::endl;
    int nSizeTemp = 1; // decisionData.referenceLine.referenceLinePoints.size() -1;
    // std::cout << "endPointRoadIndex = " << decisionData.referenceLine.referenceLinePoints[nSizeTemp].roadIDIndex
    //           << "endPointLaneIndex = " << decisionData.referenceLine.referenceLinePoints[nSizeTemp].laneIDIndex << std::endl;

    // 计算当前位置在frenet坐标系中坐标
    getFrenetLocationPoint(decisionData, decisionData.referenceLine, locationPoint);

    decisionData.optimalTrajectoryIndex = getTrajectoryPlanningResult(imu.velocity(), decisionData, imu, predictionMsg, map, trafficLights, objectsCmd, stopPoints); // 20220825 修改函数输入
                                                                                                                                                                     // std::cout << "GetOptimalGlobalTrajectory : decisionData.optimalTrajectoryIndex "<< decisionData.optimalTrajectoryIndex<<std::endl;
    // std::cout << RED << "optimalTrajectoryIndex in active lane change:" << decisionData.optimalTrajectoryIndex << RESET << std::endl;

    // 20230526这里是又改回来了，不然会出现3条lane的时候，规划轨迹在来回跳
    decisionData.currentLaneId = currentLaneId;
    decisionData.currentIndex = currentIndex;

    if (decisionData.optimalTrajectoryIndex != -1)
    {
        return true;
    }

    // std::cout << BOLDCYAN << "GetOptimalGlobalTrajectory  end " << decisionData.currentLaneId << "; " << decisionData.currentIndex <<RESET<< std::endl;

    return false;
}

// 根据全局规划的roadlanelist和当前的位置生成referencelane,用于换道的检查，这里先不考虑到达停车点的情况，后续可以再删除
// 这里的routingList里面规划的road可能会比较长，包含了roadID之前的road，如果有的话，就删除了
// 当前位置是int roadID, int laneID, int pointID，但是reference必须开始与同一road，但是可能不是不同lane，对应同样位置的point
bool GetCheckLine(RoadMap map, RoutingList rl, int roadID, int laneID, int pointID, double dRefLineLength, ReferenceLine &checkLine)
{
    std::vector<std::tuple<int32_t, int32_t>> routingList = rl.roadlanelist;

    // rl.

    // 获取当前点的对象
    Road road;                // 当前点所在road
    Lane lane;                // 当前点所在lane
    int point;                // 当前点的索引，也是计数
    PlanningPoint pPointTemp; // 当前的点临时变量

    // 为了防止有换道，需要找到同road最后一个lane,对routingList进行简化
    // 只对本road这样处理，对于后续road不做这样的处理，？？？先这样考虑把
    for (int i = 0; i < (int)routingList.size() - 1; i++)
    {
        if (std::get<0>(routingList[i]) == std::get<0>(routingList[0]) && std::get<0>(routingList[i]) == std::get<0>(routingList[i + 1])) // 跟后面的是同一个road
        {
            routingList.erase(routingList.begin() + i);
            i--; // 删了一个，当前计数还得减去一个
        }
    }

    // 删除roadID之前的road
    for (int i = 0; i < (int)routingList.size(); i++)
    {
        if (std::get<0>(routingList[i]) != roadID) // 跟后面的是同一个road
        {
            routingList.erase(routingList.begin());
            i--;
        }
        else
        {
            break;
        }
    }

    // 输入道路必须有内容,别前面处理把道路都删空了
    if (routingList.size() == 0)
        return false;

    if (!map.GetRoadByRoadID(std::get<0>(routingList[0]), road))
        return false;

    if (!map.GetLaneByLaneID(std::get<1>(routingList[0]), road, lane))
        return false;

    point = pointID;
    if (point < 0)
        return false;

    if (point >= (int)lane.gaussRoadPoints.size()) // 防止不同lane总点数不同
        point = std::max((int)lane.gaussRoadPoints.size() - 1, 0);

    // 把第一点放入checkline
    pPointTemp.gaussX = lane.gaussRoadPoints[point].GaussX;
    pPointTemp.gaussY = lane.gaussRoadPoints[point].GaussY;
    pPointTemp.accumS = 0;
    checkLine.referenceLinePoints.push_back(pPointTemp);

    // 如果累计的距离不够长，就循环添加
    while (pPointTemp.accumS < dRefLineLength)
    {
        if (point < (int)lane.gaussRoadPoints.size() - 1) // 本lane后续还有点,
        {
            point++;
            pPointTemp.gaussX = lane.gaussRoadPoints[point].GaussX;
            pPointTemp.gaussY = lane.gaussRoadPoints[point].GaussY;
            pPointTemp.accumS += lane.gaussRoadPoints[point].s - lane.gaussRoadPoints[point - 1].s;
            checkLine.referenceLinePoints.push_back(pPointTemp);
            continue;
        }

        // 本road已经是最后一条road，无后继road
        routingList.erase(routingList.begin());
        if (routingList.size() == 0) // 这种情况下，总长不满足要求的长度
            return false;

        // 如果还有后继道路，更新道路等信息
        if (!map.GetRoadByRoadID(std::get<0>(routingList[0]), road))
            return false;

        if (!map.GetLaneByLaneID(std::get<1>(routingList[0]), road, lane))
            return false;

        point = 0;
        pPointTemp.gaussX = lane.gaussRoadPoints[point].GaussX;
        pPointTemp.gaussY = lane.gaussRoadPoints[point].GaussY;
        // 因为是道路的第一点，两点距离是上一段lane最后一点到本lane第一点之间的距离
        pPointTemp.accumS += getDistance(pPointTemp.gaussY, pPointTemp.gaussX, checkLine.referenceLinePoints.back().gaussY, checkLine.referenceLinePoints.back().gaussX);
        checkLine.referenceLinePoints.push_back(pPointTemp);
    }

    return true;
}

// lry
// lry
// lry
// lry
void generateSpeedList(double velocity, DecisionData &decisionData, std::vector<infopack::ObjectsProto> &objectsCmd, const prediction::ObjectList &prediction, const pc::Imu &imu, const std::vector<GaussRoadPoint> stopPoints, const infopack::TrafficLight &trafficLight, const RoadMap &map)
{
    auto startstart = std::chrono::steady_clock::now();
    decisionData.speedList.clear(); // 20220825
    decisionData.speedList.reserve(END_SPEED_NUM);

    std::vector<PlanningTrajectory>().swap(decisionData.controlTrajectoryList);
    decisionData.controlTrajectoryList.clear();
    decisionData.controlTrajectoryList.reserve(decisionData.finalPathList.size() * END_SPEED_NUM);
    decisionData.obstacleDistance.clear();
    // std::cout <<"decisionData.finalPathList.size()=="  <<decisionData.finalPathList.size()<<std::endl;
    double currentSpeed = velocity;
    double tempDesireSpeed = DESIRED_SPEED; // 中间速度变量
    double speedThreshold = 0.5;            // 速度阈值,如果最后一个点的速度大于该数*期望速度，则后面的不需计算
    double acc = 1.5;                       // 加速度 3 is better than 2
    if (currentSpeed < 3.6)
    {
        acc = 1.2;
    }
    else
    {
        acc = 1.8;
    }
    double deceleration = 4;               // 障碍物停车减速度
    double decelerationCompensation = 0.5; // 减速度系数
    double manhtDistanceWeight = 0.5;
    // double distanceToStop = std::min(25.0,MAX_FRENET_S(imu.velocity())); // 10; // 到障碍物停车距离
    // double distanceToStop = std::max(DESIRED_SPEED*DESIRED_SPEED/2.0,25.0); // 10; // 到障碍物停车距离
    double distanceToStop = MAX_FRENET_S(velocity) - 5; // 10;
    // double distanceToPark = std::max(DESIRED_SPEED*DESIRED_SPEED/2.0,30.0); // 6;  // 到终点停车距离
    double distanceToPark = MAX_FRENET_S(velocity) - 14; // 10;

    // 20230728增加 换道和道口限速
    // double junctionSpeed = 10;    // 路口
    // double laneChangingSpeed = 8; // 换道

    double leastDistance = 100;   // 与前方车、障碍物、红绿灯、终点的距离初始化
    double stationDistance = 100; // getDistance(imu.gaussx(), imu.gaussy(), 3478597.896,560522.475);站台停车点坐标
    double carDistance = 100;     // getDistance(imu.gaussx(), imu.gaussy(), 3478597.896,560522.475);

    double trafficDistance = 100;

    std::tuple<int32_t, int32_t> fixedRoadLaneIndex = fixRoadLaneIndex(decisionData, map);
    int32_t roadIndex = std::get<0>(fixedRoadLaneIndex);
    int32_t laneIndex = std::get<1>(fixedRoadLaneIndex);

    int32_t currentRoadIndex = roadIndex;
    // for (int i = 0; i < map.roads.size(); i++)
    // {
    //     if (decisionData.currentId == map.roads[i].id)
    //     {
    //         currentRoadIndex = i;
    //         break;
    //     }
    // }
    // std::cout << "当前路索引 " << decisionData.currentLaneId <<"////"<<decisionData.finalPathList[0].planningPoints[0].roadIDIndex<< "当前lane "<< decisionData.currentLaneId<<std::endl;
    // map.roads[roadIndex].lanes[laneIndex].id;
    double currentRoadMaxSpeed = map.roads[currentRoadIndex].speedMax / 3.6;
    // std::cout << "下段路" << std::get<0>(decisionData.nextIdList[0]) << "map.roads.speedMax" << map.roads[currentRoadIndex].speedMax << std::endl;
    int nextRoadIndex = -1;
    if (decisionData.nextIdList.size() > 0)
    {
        for (int i = 0; i < map.roads.size(); i++)
        {
            if (std::get<0>(decisionData.nextIdList[0]) == map.roads[i].id)
            {
                nextRoadIndex = i;
                break;
            }
        }
        // std::cout << "下段路索引" << nextRoadIndex << std::endl;
    }

    // 增加红灯停车点，map.roads[roadindex]，这里的roadindex是容器的索引，不是路的索引，比如map.roads[0]：roadID=38。
    if (trafficLight.state() == 0 && trafficLight.stoproadid() == decisionData.currentId && trafficLight.stoplaneid() == laneIndex)
    {
        // int roadindexx = -1;
        // for (int i = 0; i < map.roads.size(); i++)
        // {
        //     if (decisionData.currentId == map.roads[i].id)
        //     {
        //         roadindexx = i;
        //         break;
        //     }
        // }
        // std::cout<<"trafficLight.stoppointid() "<<trafficLight.stoppointid()<<"(int)map.roads[currentRoadIndex].lanes[decisionData.currentLaneId].gaussRoadPoints.size() - 1"<<(int)map.roads[currentRoadIndex].lanes[decisionData.currentLaneId].gaussRoadPoints.size() - 1<<std::endl;
        // trafficDistance = abs(map.roads[roadindexx].lanes[0].gaussRoadPoints[trafficLight.stoppointid()].s - map.roads[roadindexx].lanes[0].gaussRoadPoints[decisionData.currentIndex].s);
        int trafficLightStoppointid = std::min(trafficLight.stoppointid(), (int)map.roads[currentRoadIndex].lanes[laneIndex].gaussRoadPoints.size() - 1);
        // std::cout << "trafficDistance roadindexx = " << roadindexx << " " << trafficLight.stoppointid() << " " << decisionData.currentIndex << std::endl;
        trafficDistance = abs(map.roads[currentRoadIndex].lanes[laneIndex].gaussRoadPoints[trafficLightStoppointid].s - map.roads[currentRoadIndex].lanes[laneIndex].gaussRoadPoints[decisionData.currentIndex].s);
    }

    double nextRoadMaxSpeed = map.roads[nextRoadIndex].speedMax / 3.6;
    tempDesireSpeed = std::min(currentRoadMaxSpeed, tempDesireSpeed);
    double nextDis = (map.roads[currentRoadIndex].lanes[laneIndex].gaussRoadPoints.back().s - map.roads[currentRoadIndex].lanes[laneIndex].gaussRoadPoints[decisionData.currentIndex].s);
    // std::cout << "下段路距离 " << GREEN << nextDis << std::endl;
    if (nextRoadMaxSpeed < currentRoadMaxSpeed &&
        nextDis < (currentRoadMaxSpeed * currentRoadMaxSpeed - nextRoadMaxSpeed * nextRoadMaxSpeed) / 2 / deceleration)
    {
        tempDesireSpeed = std::min(nextRoadMaxSpeed, tempDesireSpeed);
    }
    double parkingDistance = getDistance(imu.gaussx(), imu.gaussy(), stopPoints[1].GaussX, stopPoints[1].GaussY);             // 终点坐标
    double naiveManhtDistance = 100;                                                                                          // 障碍物到规划起点的frenet距离
    double objToCurveDistance = 100;                                                                                          // 障碍物到规划曲线的距离
    double curvePointsGap = (double)MAX_FRENET_S(imu.velocity()) / (double)CURVE_POINT_NUM / (double)SEGMENTED_FRENET_NUMBER; // 轨迹点之间的距离，要转为浮点数
    // std::cout << "curvePointsGap" << curvePointsGap << std::endl;
    // 计算曲率及弯道限速
    int turnIndex = -1;
    // 速度大于2才判断
    // if (currentSpeed > 2)取消，过路口的时候，低速也需要扩大探测范围
    // {
    for (int iter = 2; iter < (int)decisionData.referenceLine.referenceLinePoints.size() - 2; iter++)
    {
        // std::cout <<  RED<<" (int) decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() - 2" <<        (int) decisionData.controlTrajectoryList[decisionData.optimalTrajectoryIndex].planningPoints.size() - 2 <<std::endl;
        double curvatureTemp = fabs(calculateCurvature(decisionData.referenceLine.referenceLinePoints[iter],
                                                       decisionData.referenceLine.referenceLinePoints[iter - 2],
                                                       decisionData.referenceLine.referenceLinePoints[iter + 2]));
        // std::cout <<  RED<<"?????????????????????????????????????????curvatureTemp = ????????????????????" <<        curvatureTemp <<std::endl;
        if (curvatureTemp > CURVATURE_THRESHOLS) // 曲率
        {
            // turnIndex = iter;
            // std::cout << RED << "进入弯道,限速" << CURVATURE_SPEED << "m/s " << RESET << std::endl;
            // tempDesireSpeed = std::min(CURVATURE_SPEED, tempDesireSpeed);
            // break;
            turnIndex++;
        }
        if (turnIndex > 4)
        {
            std::cout << RED << "进入弯道,限速" << CURVATURE_SPEED << "m/s " << RESET << std::endl;
            tempDesireSpeed = std::min(CURVATURE_SPEED, tempDesireSpeed);
            acc = 1;
            break;
        }
    }

    //    if (turnIndex >= 0)
    //     {
    //         // for (int iter = 0; iter < decisionData.referenceLine.referenceLinePoints.size(); iter++)
    //         // {
    //         std::cout << RED << "进入弯道,限速" << CURVATURE_SPEED << "m/s " << RESET << std::endl;

    //         tempDesireSpeed = std::min(CURVATURE_SPEED, tempDesireSpeed);
    //         // }
    //     }
    // }

    // ACC
    // int objectIndex = -1;
    double followingCarPlanningSpeed = 100;
    int followCarId = -1;
    double dist1 = 100;
    int followCarIndex = -1;
    // for (auto &objProto : objectsCmd)
    for (int i = 0; i < (int)objectsCmd.size(); i++)
    {
        double temp2x, temp2y;
        temp2y = objectsCmd[i].lat();
        temp2x = objectsCmd[i].lon();
        double yaw = M_PI / 2 - objectsCmd[i].yaw() * M_PI / 180.;

        double gauss2x, gauss2y;
        gaussConvert(temp2x, temp2y, gauss2x, gauss2y); // 经纬度转高斯坐标
        // cout<<i<<std::setprecision(10)<<" gauss2.xx: "<< gauss2.x<<"  gauss2.yy: "<< gauss2.y<<endl;

        double veX, veY, veYaw;
        veX = imu.gaussy(); // 自车的位置
        veY = imu.gaussx();
        veYaw = M_PI / 2 - imu.yaw() * M_PI / 180.;
        CoordTran2DForNew0INOld(gauss2y, gauss2x, yaw, veX, veY, veYaw); //  高斯坐标转换车当前坐标
                                                                         // cout<<i<<" gauss2.y: "<< gauss2.y<<" objects[i]: "<< objects[i].y()<<endl;

        if ((gauss2x < 1.5 && gauss2x > -1.5 && gauss2y > 0 && gauss2y < 60 && abs(yaw-veYaw)<60))
        {
            //   std::cout<<i<<std::setprecision(10)<<" gauss2.xx: "<< objectsCmd[i].width()<<"  gauss2.yy: "<< objectsCmd[i].len()<<std::endl;

            double distTemp=getDistance(imu.gaussx(), imu.gaussy(), objectsCmd[i].x(), objectsCmd[i].y())-0.5*std::max( objectsCmd[i].width()/100.0, objectsCmd[i].len()/100.0)-2.0;//多减2米，保证路测信息比栅格距离更近，则会进入跟车状态；
            if (distTemp < dist1)
            {
                dist1 = distTemp;
                followCarId = objectsCmd[i].objectid();
                followCarIndex = i;
                std::cout<<"followCarId"<<followCarId<<std::endl;
            }
        }
    }
    std::cout<<"followCarId*****"<<followCarId<<"dist: "<<dist1<<std::endl;
    if (followCarIndex != -1)
        {
            carDistance = dist1; // 两车距离 (后面可能需要修改到frenet坐标下)
            //  std::cout << YELLOW << "距离第" <<std::setprecision(10)<< imu.gaussx() << "条曲线  " << imu.gaussy() <<objectsCmd[objectIndex].x()<<"条曲线  " << objectsCmd[objectIndex].y()<< RESET << std::endl;
            // if (carDistance <= 30 && carDistance >= 20)
            if (carDistance < 60) // 20到30米的车才跟随,参考线附近的才算障碍物
            {
                leastDistance = std::min(leastDistance, carDistance);
                followingCarPlanningSpeed = cruiseController(imu, objectsCmd, followCarIndex, carDistance);
                tempDesireSpeed = std::min(followingCarPlanningSpeed, tempDesireSpeed);
                // if (turnIndex >= 0)
                // {
                //     // for (int iter = 0; iter < decisionData.referenceLine.referenceLinePoints.size(); iter++)
                //     // {
                //     tempDesireSpeed = std::min(CURVATURE_SPEED, tempDesireSpeed);
                //     // }
                // }
                double followAcc=std::max(2.0,abs((followingCarPlanningSpeed*followingCarPlanningSpeed-imu.velocity()*imu.velocity())/(2*carDistance)));
                if (followAcc>4)
                followAcc=4;
                acc=followAcc;
                deceleration=followAcc;
                std::cout << YELLOW << "跟车中acc::" <<acc<<RESET<< std::endl;
            }
            std::cout << YELLOW << "跟车中tempDesireSpeed::" <<tempDesireSpeed<<RESET<< std::endl;
        }

    // // ACC
    // int objectIndex = -1;
    // double followingCarPlanningSpeed = 100;
    // if (hasPrecedingVehicle(imu, objectsCmd))
    // {
    //     // 前车索引点
    //     for (int i = 0; i < objectsCmd.size(); i++)
    //     {
    //         if (objectsCmd[i].type() == 0 && objectsCmd[i].objectid() == ACC_FOLLOW_OBJECT_ID) // 这个待修改，应该是锁定车辆ID
    //         // if (objectsCmd[i].type() == 60 && objectsCmd[i].objectid() == ACC_FOLLOW_OBJECT_ID) // 仿真使用
    //         {
    //             objectIndex = i;
    //             break;
    //         }
    //     }
    //     if (objectIndex != -1)
    //     {
    //         carDistance = getDistance(imu.gaussx(), imu.gaussy(), objectsCmd[objectIndex].x(), objectsCmd[objectIndex].y()); // 两车距离 (后面可能需要修改到frenet坐标下)
    //         //  std::cout << YELLOW << "距离第" <<std::setprecision(10)<< imu.gaussx() << "条曲线  " << imu.gaussy() <<objectsCmd[objectIndex].x()<<"条曲线  " << objectsCmd[objectIndex].y()<< RESET << std::endl;
    //         // if (carDistance <= 30 && carDistance >= 20)
    //         if (carDistance <= 30) // 20到30米的车才跟随,参考线附近的才算障碍物
    //         {
    //             leastDistance = std::min(leastDistance, carDistance);
    //             followingCarPlanningSpeed = cruiseController(imu, objectsCmd, objectIndex, carDistance);
    //             tempDesireSpeed = std::min(followingCarPlanningSpeed, tempDesireSpeed);
    //             if (turnIndex >= 0)
    //             {
    //                 // for (int iter = 0; iter < decisionData.referenceLine.referenceLinePoints.size(); iter++)
    //                 // {
    //                 tempDesireSpeed = std::min(CURVATURE_SPEED, tempDesireSpeed);
    //                 // }
    //             }
    //         }
    //     }
    // }

    // n条贝塞尔曲线

    // int bezierNum = pow(CURVE_NUM, SEGMENTED_FRENET_NUMBER) - 1; // 找到能走的轨迹的编号
    int bezierNum = (int)decisionData.finalPathList.size() - 1;
    // std::cout << GREEN << "bezierNum11111=   " << bezierNum << RESET << std::endl;
    for (int i = 0; i < (int)decisionData.finalPathList.size(); i++)
    {
        // double listNumber; // 给每条曲线设定权重
        // if (i % 2 == 0)
        // {
        //     listNumber = i / 2;
        // }
        // else
        // {
        //     listNumber = i / 2 + 1;
        // }
        // std::vector<double> distanceFromPointToObject;
        naiveManhtDistance = 100;
        // leastDistance = 100;
        leastDistance = std::min(leastDistance, trafficDistance); // S设置为最小
        leastDistance = std::min(leastDistance, parkingDistance);
        leastDistance = std::min(leastDistance, stationDistance);

        std::vector<double> curvePointSpeed;
        curvePointSpeed.clear();
        std::vector<double>().swap(curvePointSpeed);
        curvePointSpeed.reserve(CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER);
        // std::vector<double> distanceFromPointToObject; // 存储障碍物到路径距离
        // distanceFromPointToObject.clear();
        // std::vector<double>().swap(distanceFromPointToObject);
        std::priority_queue<double, std::vector<double>, std::greater<double>> distanceFromPointToObjectHeap; // 存储障碍物到路径距离
        std::priority_queue<double, std::vector<double>, std::greater<double>> newHeap;
        distanceFromPointToObjectHeap = newHeap;
        // std::vector<double>().swap(distanceFromPointToObjectHeap);
        int hazard = 0; // 危险次数，有2次小于安全距离就break
        // 每条曲线的m个点,主要用来计算障碍物距离
        for (int j = 0; j < decisionData.finalPathList[i].planningPoints.size(); j++)
        {

            // distanceFromPointToObject.push_back(getMinDistanceOfPoint_lry(decisionData.finalPathList[i].planningPoints[j], prediction, objectsCmd, imu));

            // // std::cout << YELLOW << i << "    getMinDistanceOfPoint:    " << getMinDistanceOfPoint(trajectory.planningPoints[i], prediction) << RESET << std::endl;
            // // std::cout << "hh1 " << trajectory.trajectoryPoints[trajectory.trajectoryPoints.size()-1].v << std::endl;
            // objToCurveDistance = findMin(distanceFromPointToObject); // 障碍物到路径的最小距离
            // 小根堆加快效率
            // 弯道增大探测范围
            distanceFromPointToObjectHeap.push(getMinDistanceOfPoint_lry1(decisionData.finalPathList[i].planningPoints[j], prediction, objectsCmd, imu, decisionData, turnIndex,followCarIndex));
            // distanceFromPointToObjectHeap.push(getMinDistanceOfPoint(decisionData.finalPathList[i].planningPoints[j], prediction, objectsCmd, imu));

            objToCurveDistance = distanceFromPointToObjectHeap.top();

            // 如果没有障碍物会输出-1，则赋值距离100
            // if (objToCurveDistance == -1)
            // {
            //     objToCurveDistance = 100;
            // }

            // std::cout << YELLOW << "距离曲线的最近障碍物距离" << objToCurveDistance << RESET << std::endl;
            //  curvePointsGap = getDistance(decisionData.finalPathList[i].planningPoints[0].x, decisionData.finalPathList[i].planningPoints[0].y,
            //  decisionData.finalPathList[i].planningPoints[1].x, decisionData.finalPathList[i].planningPoints[1].y); // 规划轨迹上两个点 间的距离

            //  std::cout << RED << "?????????????????????????????????????????SSSSS = ????????????????????" << decisionData.finalPathList[0].planningPoints[j].s << std::endl;

            // 如果距离小于安全距离1.8，则找到离起点最近的障碍物

            if (objToCurveDistance < SAFE_DISTANCE * SAFE_DISTANCE)
            {
                naiveManhtDistance = std::min(decisionData.finalPathList[0].planningPoints[j].s + manhtDistanceWeight * sqrt(objToCurveDistance), naiveManhtDistance); // 起点到障碍物的距离+障碍物到曲线的距离*权重，避免来回跳
                std::cout << YELLOW << "距离第" << i << "条曲线起点的障碍物距离  " << naiveManhtDistance << RESET << std::endl;
                leastDistance = std::min(leastDistance, naiveManhtDistance);
                hazard++;
                if (hazard > 1)
                    break;
            }
        }

        // std::cout << RED << "?????????????????????????????????????????dDesireSpeed = ????????????????????" << tempDesireSpeed << std::endl;

        //     // 23030728增加：到达路口和换道时减速
        // if( trave_lane && currentSpeed>2.7){
        //         tempDesireSpeed = std::min( laneChangingSpeed, tempDesireSpeed);
        // }

        //  if( trave_lane && currentSpeed>2.7){
        //         tempDesireSpeed = std::min( junctionSpeed, tempDesireSpeed);
        // }
        decisionData.obstacleDistance.push_back(leastDistance); // 加入障碍物距离
        std::vector<double> endPointSpeedList;
        endPointSpeedList.clear();
        endPointSpeedList.reserve(END_SPEED_NUM);

        double endPointSpeed = tempDesireSpeed;
        endPointSpeedList.push_back(tempDesireSpeed);
        double planningVelocity;

        if (i == 0)
        {
            // std::cout << BOLDGREEN << "?????????????????????????????????????????leastDistance = ????????????????????" << leastDistance << std::endl;
            // std::cout << BOLDBLUE << "?????????????????????????????????????????dDesireSpeed = ????????????????????" << tempDesireSpeed << std::endl;
            std::cout << GREEN << "trafficD=" << trafficDistance << "  parkingD= " << parkingDistance << "  stationD= " << stationDistance << "  naiveManhtD= " << naiveManhtDistance << "  carD=" << carDistance << " leastD= " << leastDistance << std::endl;
        }
        // 处理障碍物、行人-停车--------------------------------------------------------------------------------------&& currentSpeed != 0
        if (fabs(leastDistance - naiveManhtDistance) < 0.01 && fabs(leastDistance - carDistance) > 0.01 && leastDistance <= distanceToStop + manhtDistanceWeight * SAFE_DISTANCE + (DESIRED_SPEED * DESIRED_SPEED / (2 * deceleration))) // 前方有障碍物
        {
            // std::cout << BOLDBLUE << "?????????????????????????????????????????处理障碍物 = ????????????????????" << RESET << std::endl;
            if (i == 0)
                std::cout << RED << "?????????????????????????????????????????有障碍物准备停车 = ????????????????????" << distanceToStop + manhtDistanceWeight * SAFE_DISTANCE + (DESIRED_SPEED * DESIRED_SPEED / (2 * deceleration)) << RESET << std::endl;
            // tempDesireSpeed=0;
            // tempDesireSpeed = std::min(0.0, tempDesireSpeed);
            if (leastDistance >= distanceToStop + manhtDistanceWeight * SAFE_DISTANCE)
            {
                // planningVelocity = std::min(sqrt(2 * deceleration * (leastDistance - (distanceToStop + manhtDistanceWeight * SAFE_DISTANCE))), DESIRED_SPEED);
                planningVelocity = std::min((2 * deceleration * (leastDistance - (distanceToStop + manhtDistanceWeight * SAFE_DISTANCE))), DESIRED_SPEED * DESIRED_SPEED); // 速度的平方
                //  std::cout << RED << "第三条" <<  std::endl;
                // if (currentSpeed * currentSpeed - 2 * deceleration * curvePointsGap > 0 && leastDistance >= curvePointsGap+distanceToStop)
                //     planningVelocity = sqrt(2 * deceleration * (leastDistance-distanceToStop - curvePointsGap / (0.5 * (currentSpeed + (sqrt(currentSpeed * currentSpeed - 2 * deceleration * curvePointsGap))))));
                // // planningVelocity=sqrt(2 * deceleration * (leastDistance-curvePointsGap));
                // else
                //     planningVelocity = 0;
                // 注释掉
                // // 处理起步时
                // if (currentSpeed * currentSpeed <= (2 * acc * (leastDistance - (distanceToStop + manhtDistanceWeight * SAFE_DISTANCE))))
                // {
                //     // std::cout << RED << "第三条1" <<  std::endl;
                //     // planningVelocity = std::min(sqrt(currentSpeed * currentSpeed + 2 * acc * curvePointsGap), sqrt(2 * acc * (leastDistance - (distanceToStop + manhtDistanceWeight * SAFE_DISTANCE))));
                //     planningVelocity = std::min((currentSpeed * currentSpeed + 2 * acc * curvePointsGap), (2 * acc * (leastDistance - (distanceToStop + manhtDistanceWeight * SAFE_DISTANCE))));
                //     //     if (planningVelocity >= sqrt(2 * acc * (leastDistance - (distanceToStop + manhtDistanceWeight * SAFE_DISTANCE))))
                //     //     {
                //     //         // 规划中后部分速度可能会超出期望速度
                //     //         planningVelocity = sqrt(2 * acc * (leastDistance - (distanceToStop + manhtDistanceWeight * SAFE_DISTANCE)));
                //     //     }
                // }

                // if (objectIndex != -1 && planningVelocity > followingCarPlanningSpeed * followingCarPlanningSpeed)
                // {
                //     // 规划中后部分速度可能会超出期望速度
                //     planningVelocity = followingCarPlanningSpeed * followingCarPlanningSpeed;
                //     if (planningVelocity < 0)
                //     {
                //         // 规划中后部分速度可能会超出期望速度
                //         planningVelocity = 0;
                //     }
                // }

                // if (turnIndex >= 0 && planningVelocity >= CURVATURE_SPEED * CURVATURE_SPEED)
                // {
                //     planningVelocity = CURVATURE_SPEED * CURVATURE_SPEED;
                // }
                // decisionData.speedList.push_back(curvePointSpeed);
                if (planningVelocity >= DESIRED_SPEED * DESIRED_SPEED)
                {
                    // 规划中后部分速度可能会超出期望速度
                    planningVelocity = DESIRED_SPEED * DESIRED_SPEED;
                }
                // std::cout << RED << "到达行人点附近?????开始减速????????????????规划速度的平方=" << planningVelocity << RESET << std::endl;
                // curvePointSpeed.assign(CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER, planningVelocity);
                curvePointSpeed.push_back(velocity); // 第一个速度点为当前车速
                double vv = planningVelocity;
                for (int i = 1; i < CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER; i++)
                {
                    // if (planningVelocity - 2 * deceleration *decelerationCompensation* decisionData.finalPathList[0].planningPoints[i].s > 0)
                    //     vv = (planningVelocity - 2 * deceleration * decelerationCompensation*decisionData.finalPathList[0].planningPoints[i].s);
                    // else
                    // {
                    //     vv = 0;
                    // }
                    // vv = currentSpeed * currentSpeed + 2 * acc * decelerationCompensation * decisionData.finalPathList[0].planningPoints[i].s;
                    // if (vv <= 0)
                    // {
                    //     // 规划中后部分速度可能会超出期望速度
                    //     vv = 0;
                    // }
                    // 20230905add
                    if (vv > currentSpeed * currentSpeed + 2 * acc * decisionData.finalPathList[0].planningPoints[2].s)
                    {
                        vv = (currentSpeed * currentSpeed + 2 * acc * decisionData.finalPathList[0].planningPoints[2].s);
                    }
                    if (vv >= DESIRED_SPEED * DESIRED_SPEED)
                    {
                        // 规划中后部分速度可能会超出期望速度
                        vv = DESIRED_SPEED * DESIRED_SPEED;
                    }
                    if (vv >= tempDesireSpeed * tempDesireSpeed)
                    {
                        // 规划中后部分速度可能会超出期望速度
                        vv = tempDesireSpeed * tempDesireSpeed;
                    }
                    curvePointSpeed.push_back(sqrt(vv));
                }
            }
            // 处理怠速问题
            //  else if(leastDistance<=distanceToStop && tempDesireSpeed==0 && currentSpeed<=9/3.10)
            else if (leastDistance < (distanceToStop + manhtDistanceWeight * SAFE_DISTANCE))
            {
                curvePointSpeed.push_back(velocity);
                // planningVelocity = 0;
                if (i == 0)
                    std::cout << RED << "到达行人点附近！！！！！！！！！！！！！！刹车！！！！！！！规划速度=" << 0 << RESET << std::endl;
                for (int i = 1; i < CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER; i++)
                {
                    curvePointSpeed.push_back(0);
                }
            }

            //   tempDesireSpeed=0;
            // for (auto planningVelocity : curvePointSpeed)
            // {
            //     std::cout << GREEN << "??下发车速= " << planningVelocity << std::endl;
            // }
            // std::cout << BOLDBLUE << "?????????????????????????????????????????dDesireSpeed1 = ????????????????????" << tempDesireSpeed << std::endl;
            // decisionData.speedList.push_back(curvePointSpeed);
        }

        // 处理站点、红绿灯停车------------------------------------------------------------------
        else if (fabs(leastDistance - naiveManhtDistance) > 0.01 && fabs(leastDistance - carDistance) > 0.01 && leastDistance <= distanceToPark + (DESIRED_SPEED * DESIRED_SPEED / (2 * deceleration))) // 前方不是车,要停车+3车身长度
        {
            // std::cout << BOLDBLUE << "?????????????????????????????????????????处理站点= ????????????????????" << RESET << std::endl;
            if (i == 0)
            {
                std::cout << RED << "?????????????????????????????????????????有站点或红灯准备停车 = ????????????????????" << distanceToPark + (DESIRED_SPEED * DESIRED_SPEED / (2 * deceleration)) << RESET << std::endl;
            }
            // tempDesireSpeed=0;
            // tempDesireSpeed = std::min(0.0, tempDesireSpeed);
            if (leastDistance >= distanceToPark)
            {
                planningVelocity = (2 * deceleration * (leastDistance - distanceToPark));

                // 注释掉
                //  if (currentSpeed * currentSpeed <= (2 * deceleration * (leastDistance - distanceToPark)))
                //  {
                //      planningVelocity = std::min((currentSpeed * currentSpeed + 2 * acc * curvePointsGap), (2 * acc * (leastDistance - distanceToPark)));
                //  }
                //    if (planningVelocity >= currentSpeed)
                //  {
                //      planningVelocity = currentSpeed;
                //  }
                if (planningVelocity >= DESIRED_SPEED * DESIRED_SPEED)
                {
                    // 规划中后部分速度可能会超出期望速度
                    planningVelocity = DESIRED_SPEED * DESIRED_SPEED;
                }
                // if (objectIndex != -1 && planningVelocity > followingCarPlanningSpeed * followingCarPlanningSpeed)
                // {
                //     // 规划中后部分速度可能会超出期望速度
                //     planningVelocity = followingCarPlanningSpeed * followingCarPlanningSpeed;

                //     if (planningVelocity < 0)
                //     {
                //         // 规划中后部分速度可能会超出期望速度
                //         planningVelocity = 0;
                //     }
                // }

                // if (turnIndex >= 0 && planningVelocity >= CURVATURE_SPEED * CURVATURE_SPEED)
                // {
                //     planningVelocity = CURVATURE_SPEED * CURVATURE_SPEED;
                // }
                // std::cout << RED << "到达停车点附近开始减速，规划速度的平方=" << planningVelocity << RESET << std::endl;
                // curvePointSpeed.assign(CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER, planningVelocity);
                curvePointSpeed.push_back(velocity); // 第一个速度点为当前车速
                double vv = planningVelocity;
                for (int i = 1; i < CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER; i++)
                {
                    // if (planningVelocity - 2 * deceleration * decelerationCompensation * decisionData.finalPathList[0].planningPoints[i].s > 0)
                    //     vv = (planningVelocity - 2 * deceleration * decelerationCompensation * decisionData.finalPathList[0].planningPoints[i].s);
                    // else
                    // {
                    //     vv = 0;
                    // }
                    // if (vv <= 0)
                    // {
                    //     // 规划中后部分速度可能会超出期望速度
                    //     vv = 0;
                    // }
                    // if (leastDistance <= distanceToPark) // 处理怠速
                    // {
                    //     vv = 0;
                    // }
                    // 20230905add
                    if (vv > currentSpeed * currentSpeed + 2 * acc * decisionData.finalPathList[0].planningPoints[2].s)
                    {
                        vv = (currentSpeed * currentSpeed + 2 * acc * decisionData.finalPathList[0].planningPoints[2].s);
                    }
                    curvePointSpeed.push_back(sqrt(vv));
                }
            }
            // 处理怠速问题
            // else if (leastDistance <= 3 && tempDesireSpeed == 0 && currentSpeed<=9/3.6)
            else if (leastDistance < distanceToPark)
            {
                // planningVelocity = 0;
                if (i == 0)
                    std::cout << RED << "??????处理怠速,到达停车点附近！！！！！！！！刹车！！！！！！！！，规划速度=" << 0 << RESET << std::endl;

                curvePointSpeed.push_back(velocity);
                // planningVelocity = 0;
                for (int i = 1; i < CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER; i++)
                {
                    curvePointSpeed.push_back(0);
                }
            }
            // curvePointSpeed.assign(CURVE_POINT_NUM, planningVelocity);
        }

        // 正常行驶+处理跟车----------------------------------------------------------------------------------
        else
        {
            if (tempDesireSpeed < 0)
            {
                tempDesireSpeed = 0;
            }
            if (i == 0)
                std::cout << BOLDBLUE << "?????????????????????????????????????????正常行驶+处理跟车 = ????????????????????tempDesireSpeed=" << tempDesireSpeed << RESET << std::endl;
            curvePointSpeed.push_back(velocity);
            if (currentSpeed < tempDesireSpeed)
            {
                for (int i = 0; i < CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER - 1; i++)
                {
                    // planningVelocity = std::min((currentSpeed * currentSpeed + 2 * acc * decisionData.finalPathList[0].planningPoints[i + 1].s), tempDesireSpeed * tempDesireSpeed);
                    planningVelocity = std::min((currentSpeed * currentSpeed + 2 * acc * decisionData.finalPathList[0].planningPoints[2].s), tempDesireSpeed * tempDesireSpeed);
                    // if (planningVelocity >= tempDesireSpeed)
                    // {
                    //     // 规划中后部分速度可能会超出期望速度
                    //     planningVelocity = tempDesireSpeed;
                    // }
                    if (planningVelocity < 0)
                        planningVelocity = 0;
                    curvePointSpeed.push_back(sqrt(planningVelocity));
                    // decisionData.speedList.push_back(curvePointSpeed);
                    // std::cout << BOLDRED << s0 << "当前车速 " << currentSpeed << " 加速度" << acc << RESET << std::endl;
                }
                // decisionData.speedList.push_back(curvePointSpeed);
            }
            else
            {
                for (int i = 0; i < CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER - 1; i++)
                {
                    if (currentSpeed * currentSpeed - 2 * deceleration * decisionData.finalPathList[0].planningPoints[i + 1].s > 0)
                        planningVelocity = (currentSpeed * currentSpeed - 2 * deceleration * decisionData.finalPathList[0].planningPoints[2].s);
                        // planningVelocity = (currentSpeed * currentSpeed - 2 * deceleration * decisionData.finalPathList[0].planningPoints[i + 1].s);
                    else
                    {
                        planningVelocity = 0;
                    }
                    if (planningVelocity <= tempDesireSpeed * tempDesireSpeed)
                    {
                        // 规划中后部分速度可能会超出期望速度
                        planningVelocity = tempDesireSpeed * tempDesireSpeed;
                    }
                    // if (leastDistance <= distanceToPark) // 处理怠速
                    // {
                    //     planningVelocity = 0;
                    // }
                       if (leastDistance < 15) // 处理怠速
                    {
                        planningVelocity = 0;
                    }
                    // if (planningVelocity < 0)
                    //     planningVelocity = 0;
                    curvePointSpeed.push_back(sqrt(planningVelocity));
                }
                // decisionData.speedList.push_back(curvePointSpeed);
            }
        }
        // decisionData.speedList.push_back(curvePointSpeed);

        PlanningTrajectory trajectoryPointList;
        trajectoryPointList.planningPoints.clear();
        trajectoryPointList.planningPoints.reserve(0);
        // for (int k = 0; k < CURVE_POINT_NUM; k++)
        // 速度信息压入最终controlTrajectoryList曲线
        for (int k = 0; k < (int)decisionData.finalPathList[i].planningPoints.size(); k++)
        {
            PlanningPoint trajectoryPoint;
            PlanningPoint Temp;
            trajectoryPoint.x = decisionData.finalPathList[i].planningPoints[k].x;
            trajectoryPoint.y = decisionData.finalPathList[i].planningPoints[k].y;
            trajectoryPoint.angle = decisionData.finalPathList[i].planningPoints[k].angle;
            // trajectoryPoint.v = decisionData.speedList[i][k];//原来是j
            // Temp.v = curvePointSpeed[k];
            // decisionData.controlTrajectoryList[i].planningPoints.push_back(Temp);
            // trajectoryPoint.v = curvePointSpeed[k] * (100 - listNumber * 2) / 100; // 20230803给速度一个权重，越靠边的速度越小；
            trajectoryPoint.v = curvePointSpeed[k];
            trajectoryPointList.planningPoints.push_back(trajectoryPoint);
        }
        decisionData.controlTrajectoryList.push_back(trajectoryPointList);
        // 如果找到能走的轨迹，后面的轨迹速度都设为0
        if (decisionData.controlTrajectoryList[i].planningPoints.back().v > speedThreshold * DESIRED_SPEED)
        {
            bezierNum = i;
            // std::cout << GREEN << "bezierNum=   " << bezierNum << RESET << std::endl;
            break;
        }
    }

    // PlanningTrajectory trajectoryPointList;
    // trajectoryPointList.planningPoints.clear();
    // trajectoryPointList.planningPoints.reserve(0);
    //  for (int i = 0; i < (int)decisionData.finalPathList.size(); i++)
    if (bezierNum < (int)decisionData.finalPathList.size() - 1)
    {
        for (int iii = bezierNum + 1; iii < (int)decisionData.finalPathList.size(); iii++)
        {
            PlanningTrajectory trajectoryPointList;
            trajectoryPointList.planningPoints.clear();
            trajectoryPointList.planningPoints.reserve(0);
            for (int k = 0; k < (int)decisionData.finalPathList[iii].planningPoints.size(); k++)
            {
                PlanningPoint trajectoryPoint;
                PlanningPoint Temp;
                trajectoryPoint.x = decisionData.finalPathList[iii].planningPoints[k].x;
                trajectoryPoint.y = decisionData.finalPathList[iii].planningPoints[k].y;
                trajectoryPoint.angle = decisionData.finalPathList[iii].planningPoints[k].angle;
                trajectoryPoint.v = 0; // 速度都设为0
                trajectoryPointList.planningPoints.push_back(trajectoryPoint);
            }
            decisionData.controlTrajectoryList.push_back(trajectoryPointList);
            decisionData.obstacleDistance.push_back(100.0); // 加入障碍物距离
        }
    }
    // std::cout<<"decisionData.obstacleDistance.size  "<<decisionData.obstacleDistance.size()<<std::endl;
    // //打印每条轨迹的规划速度信息
    // for (auto ii = 0; ii < decisionData.controlTrajectoryList.size(); ii++)
    for (auto ii = 0; ii <= bezierNum; ii++)
    {
        std::cout << GREEN << "第" << ii << "条曲线下发车速= " << RESET << std::endl;
        for (int jj = 0; jj < CURVE_POINT_NUM * SEGMENTED_FRENET_NUMBER; jj++)
            std::cout << std::setprecision(3) << decisionData.controlTrajectoryList[ii].planningPoints[jj].v << " " << RESET;

        std::cout << std::endl;
        //  std::cout << "controlTrajectoryList.size=" << decisionData.controlTrajectoryList.size() <<
        //         " planningPoints.size() 222="<<decisionData.controlTrajectoryList[ii].planningPoints.size()<<std::endl;
    }

    auto endend = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = endend - startstart; // std::micro 表示以微秒为时间单位, std::milli 表示以毫秒为时间单位。
                                                                             // std::cout << "ttime: " << elapsed.count() << "ms" << std::endl;
}

// lry20230703弯道加大探测范围
// 这是在车辆坐标系下进行的距离计算
// 20230801优化计算速度
double getMinDistanceOfPoint_lry1(const PlanningPoint &point, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> &objectsCmd, const pc::Imu &imu, DecisionData &decisionData, int turnIndex, int followCarId)
{
    // auto startstart = std::chrono::steady_clock::now();
    double predictionPointToObject = 100;
    double protoPointToObject = 100;
    int index = 0; //????
    int obssize = 0;
    double tRemainder;
    PlanningPoint predictTempPoint;
    std::vector<double> distanceFromObject;
    distanceFromObject.push_back(100);
    double Detect = (turnIndex < 0) ? 2 : MAX_FRENET_S(imu.velocity()); // 探测距离
    // index = (int)t / PREDICT_FREQUENCY;
    // tRemainder = t - index * PREDICT_FREQUENCY;

    // if( prediction.object(). size()==0)
    // return 100;
    int hazard1 = 0;
    int hazard2 = 0;
    for (const auto &object : prediction.object()) // 激光感知障碍物，现在是小方框集合，只计算到该点的距离
    {
        // std::cout <<"point.x"<< point.y<< std::endl;

        // double Detect=(turnIndex < 0) ? 2:MAX_FRENET_S(imu.velocity());//探测距离
        //  std::cout <<"Detect0"<< Detect<<std::endl;
        // std::cout <<"Detect0"<< object.predictpoint_size()<<std::endl;
        //    if ((object.predictpoint(0).x() + object.w() / 2 + LASER_OFFSET_FRONT > -2) &&
        //     (object.predictpoint(0).x() - object.w() / 2 + LASER_OFFSET_FRONT < MAX_FRENET_S(imu.velocity())) &&
        //        (object.predictpoint(0).y() - object.l() / 2) < Detect &&
        //         (object.predictpoint(0).y() + object.l() / 2) > -Detect )
        // 20230731修改为只判断轨迹上每个点附近的障碍物，减少计算     (MAX_FRENET_S(imu.velocity()) / CURVE_POINT_NUM)相邻两个点之间的距离乘以0.8；
        if ((object.predictpoint(obssize).x() + object.w() / 2 + LASER_OFFSET_FRONT > -((double)MAX_FRENET_S(imu.velocity()) / (double)CURVE_POINT_NUM / (double)SEGMENTED_FRENET_NUMBER) * 2 + point.y) &&
            (object.predictpoint(obssize).x() - object.w() / 2 + LASER_OFFSET_FRONT < ((double)MAX_FRENET_S(imu.velocity()) / (double)CURVE_POINT_NUM / (double)SEGMENTED_FRENET_NUMBER) * 2 + point.y) &&
            (object.predictpoint(obssize).x() + object.w() / 2 + LASER_OFFSET_FRONT > -2) && (object.predictpoint(obssize).x() - object.w() / 2 + LASER_OFFSET_FRONT < MAX_FRENET_S(imu.velocity())) &&
            (object.predictpoint(obssize).y() - object.l() / 2) < Detect && (object.predictpoint(obssize).y() + object.l() / 2) > -Detect)
        {
            // std::cout <<"障碍物数量为"<< obssize<< " "<<prediction.object_size()<<std::endl;
            //  std::cout <<  RED<<"?????????????????????????????????????????探测范围横向 = ????????????????????" << Detect<<std::endl;
            //    std::cout <<  RED<<"?????????????????????????????????????????探测范围纵向 = ????????????????????" <<  -2+point.x<<" "<< 2+point.x<<std::endl;

            predictTempPoint.x = -object.predictpoint(0).y();
            predictTempPoint.y = object.predictpoint(0).x() + LASER_OFFSET_FRONT;

            // std::cout << "predictTempPoint:" << point.x << " " << point.y << " " << predictTempPoint.x << " " << predictTempPoint.y << std::endl;
            // distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x, predictTempPoint.y));//到四个点的距离有问题
            // double curvePointsGap = (double)MAX_FRENET_S(imu.velocity()) /(double) CURVE_POINT_NUM/(double)SEGMENTED_FRENET_NUMBER;
            //  if(predictTempPoint.y >point.y-0.8*((double)MAX_FRENET_S(imu.velocity()) /(double) CURVE_POINT_NUM/(double)SEGMENTED_FRENET_NUMBER) &&
            //      predictTempPoint.y <point.y+0.8*((double)MAX_FRENET_S(imu.velocity()) /(double) CURVE_POINT_NUM/(double)SEGMENTED_FRENET_NUMBER))
            //    {
            predictionPointToObject = getDistance_2(point.x, point.y, predictTempPoint.x, predictTempPoint.y);
            // distanceFromObject.push_back(predictionPointToObject); // 距离的平方
            // 增加判断，如果有两次不安全的障碍物，后面的就不需要计算了
            if (predictionPointToObject < SAFE_DISTANCE * SAFE_DISTANCE)
            {
                hazard1++;
                distanceFromObject.push_back(predictionPointToObject); // 距离的平方
                if (hazard1 > 1)
                    break;
            }
            //    }
            //     distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x + object.l() / 2, predictTempPoint.y + object.w() / 2));
            //     distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x + object.l() / 2, predictTempPoint.y - object.w() / 2));
            //     distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x - object.l() / 2, predictTempPoint.y + object.w() / 2));
            //     distanceFromObject.push_back(getDistance(point.x, point.y, predictTempPoint.x - object.l() / 2, predictTempPoint.y - object.w() / 2));
            //
            // luce ganzhi cheliang
        }
        // else
        // {
        //     distanceFromObject.push_back(1000);
        // }
        // index++;
        //    obssize++;
    }

    ////////////////////////////////////////////////////////////////////路测
    // for (int i = 0; i < objectsCmd.size(); i++)
    for (const auto &objProto : objectsCmd)
    {
        // 20230904  no  路测障碍物 ********************************
        break;

        // std::cout <<"障碍物2数量为objectsCmd.size()"<< " "<<objectsCmd.size()<<std::endl;
        // const infopack::ObjectsProto objProto = objectsCmd[i];
        if (objProto.type() == myselfVehicleeType && objProto.objectid() == myselfVehicleID) // 不自车
        {
            continue;
        }

            if(objProto.objectid() ==followCarId)//跟的车要滤掉
        {
            continue;
        }
        // 仿真使用
        // if (objProto.type() == 60 && objProto.objectid() == 2) // 不自车
        // {
        //     continue;
        // }
        // 转换障碍物的xy经纬度为平面坐标
        double latitudeTemp = objProto.lat();
        double longitudeTemp = objProto.lon();
        double gaussNorthTemp, gaussEastTemp;
        gaussConvert(longitudeTemp, latitudeTemp, gaussNorthTemp, gaussEastTemp);
        // objProto.velocity
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
        dYawVehicleForShow = M_PI / 2 - imu.yaw() * M_PI / 180;

        for (int j = 0; j < 4; j++)
        {
            CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow); //  前左
            // std::cout << "distanceFromObject:"  << -pointTemp[j][1] << " " << pointTemp[j][0] << " " << std::endl; // you qian
            // 这里计算的是与障碍物四个角点的距离，这个有点不对，如果障碍物很大，而点在障碍物之内，这个计算就不能处理这种问题
            if (pointTemp[j][0] > point.y - 0.8 * ((double)MAX_FRENET_S(imu.velocity()) / (double)CURVE_POINT_NUM / (double)SEGMENTED_FRENET_NUMBER) &&
                pointTemp[j][0] < point.y + 0.8 * ((double)MAX_FRENET_S(imu.velocity()) / (double)CURVE_POINT_NUM / (double)SEGMENTED_FRENET_NUMBER))
            {
                protoPointToObject = getDistance_2(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]);
                if (protoPointToObject < SAFE_DISTANCE * SAFE_DISTANCE)
                    hazard2++;
                distanceFromObject.push_back(protoPointToObject); // 距离的平方
            }
        }
        if (hazard2 > 1)
            break;
        // // 改为点到直线的距离
        // // std::priority_queue<double, std::vector<double>, std::greater<double>> dis;
        // CoordTran2DForNew0INOld(pointTemp[3][0], pointTemp[3][1], pointTemp[3][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow); //  前左

        // distanceFromObject.push_back(point2Line(point.x, point.y, -pointTemp[3][1], pointTemp[3][0], -pointTemp[0][1], pointTemp[0][0]));

        // for (int j = 0; j < 3; j++)
        // {
        //     CoordTran2DForNew0INOld(pointTemp[j][0], pointTemp[j][1], pointTemp[j][2], dXVehicleForShow, dYVehicleForShow, dYawVehicleForShow); //  前左
        //     // std::cout << "distanceFromObject:" << j << " " << point.x << " " << point.y << " " << -pointTemp[j][1] << " " << pointTemp[j][0] << " " << getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]) << std::endl; // you qian
        //     // 这里计算的是与障碍物四个角点的距离，这个有点不对，如果障碍物很大，而点在障碍物之内，这个计算就不能处理这种问题
        //     // distanceFromObject.push_back(getDistance(point.x, point.y, -pointTemp[j][1], pointTemp[j][0]));
        //     distanceFromObject.push_back(point2Line(point.x, point.y, -pointTemp[j][1], pointTemp[j][0], -pointTemp[j + 1][1], pointTemp[j + 1][0]));
        // }
    }
    //   std::cout << "distanceFromObject:"  <<std::endl;
    return findMin(distanceFromObject);
}
// 改为距离的平方
double point2Line(double x, double y, double x1, double y1, double x2, double y2)
{
    double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
    if (cross <= 0)
        return ((x - x1) * (x - x1) + (y - y1) * (y - y1));
    double d2 = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    if (cross >= d2)
        return ((x - x2) * (x - x2) + (y - y2) * (y - y2));
    double r = cross / d2;
    double px = x1 + (x2 - x1) * r;
    double py = y1 + (y2 - y1) * r;
    // return (std::pow(x-px, 2) + std::pow(y-py, 2));
    return ((x - px) * (x - px) + (y - py) * (y - py));
}

double getDistance_2(double x1, double y1, double x2, double y2)
{
    return ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}