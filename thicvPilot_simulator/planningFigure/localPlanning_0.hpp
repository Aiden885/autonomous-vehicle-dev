#ifndef __LOCALPLANNING_HPP__
#define __LOCALPLANNING_HPP__

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "controlMsg/control.pb.h"
#include "defineColor.h"
#include "imuMsg/imu.pb.h"
#include "mapMsg/localization_MapAnalysis.h"
#include "predictionMsg/prediction.pb.h"
#include "serverMsg/objects.pb.h"
#include "serverMsg/traffic_light.pb.h"
#include "baseData.h"
#include "geometry.h"
#include "perception/trafficLightFromPerc.pb.h"

#define MINIMAL_ID 0

#define OFFSET_X -0.3
#define OFFSET_Y 0

// curve related.
#define CP_DISTANCE 1      // Bezier曲线中间两点位置的比例值
#define CURVE_POINT_NUM 20 // 20230215  reduce  to 15 from 75
// #define CURVE_NUM 5        // 20230215 reduce  9 // init is 9 //规划曲线的数量
// #define CURVE_DISTANCE 0.5 // 20230215   0.4 /两条规划曲线终点间距
#define CP_DISTANCE_SECOND 1
#define LASER_OFFSET_FRONT 0.96 // lase before rear wheel 原来是1.5

#define myselfVehicleID 1    // 本车ID，这个后续应该挪到配置文件里面
#define myselfVehicleeType 0 // type
#define myselfVCType 6       //

#define PLANNING_VELOCITY_MULTIPLIER 2.6
#define DEFAULT_PLANNING_DISTANCE 5
#define MAX_PLANNING_DISTANCE 6
#define MIN_PLANNING_DISTANCE 5
#define DISTANCE_RANGE 3

#define DEFAULT_SECOND_PLANNING_DISTANCE_LONGER 5 // distance from control planning to lidar planning

// 20220826
#define POINT_DIS_THRESHOLD_ON_CURRENT_LANE 2.0 // 3    //认为车在当前路点的距离阈值（m）这个数值很尴尬，太小了车在两个路中间会找不到路，太大了换道之后不能换到新的路上
#define POINT_DIS_THRESHOLD_ON_OTHER_LANE 2
#define POINT_ANGLE_THRESHOLD 120 // 认为车在当前路点的朝向角阈值（degree）

#define END_SPEED_NUM 1          // 末速度采样数量 原始数值是1 20230212
#define DESIRED_SPEED 40.0 / 3.6 // m/s
#define SPEED_RANGE 3 / 3.6      // m/s
#define CURVATURE_SPEED 13.0 / 3.6
#define CURVATURE_THRESHOLS 0.12 // 曲率门限值

#define VEHICLE_DIAGONAL 2.7951
#define VEHICLE_DIAGONAL_DEGREE 26.5651
#define PREDICT_FREQUENCY 0.1

#define STOP_JUDGE_DISTANCE 3 // 20230214  减小 8

// 规划预瞄距离= 基本安全距离+velocity*velocity/2(从0达到道路规划速度需要形式的距离，加速度为1)
// 时速10km/h 距离19米； 15km/h 距离24米；20km/h 距离31米，看着还算合理
// #define MAX_FRENET_S  (15.0 + (DESIRED_SPEED * DESIRED_SPEED) / 2.0)
#define MAX_FRENET_S(velocity) (25.0 + (velocity * velocity) / 2)

// #define MAX_FRENET_S 30.0
#define ACC_RELA_DIST 20.0                  // ACC跟车保持距离，目前这个数值不合适，会被认为是障碍物
#define ACC_BEGIN_DIST (ACC_RELA_DIST + 30) // ACC进入跟车的距离 +5
#define ACC_FOLLOW_OBJECT_ID 2              // ACC 跟车前车ID
#define doNothing()

#define SEGMENTED_FRENET_NUMBER 1 // 规划预瞄分段数

#define SAFE_DISTANCE 1.2

#define CHANGE_LANE_BACK_SAFE_LENGTH 20 // 换道后方安全距离，待换车道本车后方安全距离 linshixiugai   10
#define CHANGE_LANE_SAFE_WIDTH 0.2      // 换道

struct PlanningPoint
{
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

struct PlanningTrajectory
{
    std::vector<PlanningPoint> planningPoints;
};

// struct Point
// {
//   double x;
//   double y;
//   double angle;
// };

// struct Curve
// {
//   int32_t index = 0;
//   Point points[CURVE_POINT_NUM];
//   std::vector<Point> pointList;
// };

// struct TrajectoryPoint
// {
//   double x = 0.0;
//   double y = 0.0;
//   double theta = 0.0;
//   double v = 0.0;
// }; // 20220825

// struct Trajectory
// {
//   std::vector<TrajectoryPoint> trajectoryPoints;
// };

// struct GlobalTrajectoryPoint
// {
//   double GaussX;
//   double GaussY;
//   double yaw;
//   double v;
//   double curvature;
//   double s;
// };

// struct GlobalTrajectory
// {
//   std::vector<GlobalTrajectoryPoint> globalTrajectoryPoints;
// };

// struct FrenetTrajectoryPoint
// {
//   double s;
//   double l;
//   double angle;
// };

// struct FrenetTrajectory
// {
//   std::vector<FrenetTrajectoryPoint> FrenetTrajectoryPoints;
// };

struct ReferenceLine
{
    std::vector<PlanningPoint> referenceLinePoints;
    double dFrontObsDis; // 检测出来的最近的前方障碍物的距离
    double dBackObsDis;  // 检测出来的最近的后方障碍物的距离
    bool bOK;            // 可用的参考线
};

struct DecisionData
{
    std::vector<PlanningTrajectory> finalPathList; // 横向规划轨迹
    ReferenceLine referenceLine;
    PlanningPoint frenetLocationPoint;
    std::vector<std::vector<double>> speedList;            // 20220821 每种速度采样上每个点的速度
    std::vector<PlanningTrajectory> controlTrajectoryList; // 20220826 轨迹
    std::vector<int32_t> feasibleTrajectoryIndexList;      // 20220904
    PlanningTrajectory optimalGlobalTrajectory;            // 最优轨迹转化成全局坐标
    int32_t optimalTrajectoryIndex;                        // 最优轨迹序号
    int32_t currentId = 0;                                 // 当前RoadID
    int32_t currentLaneId = 0;                             // 当前LaneID          20220825添加
    int32_t currentIndex = 0;                              // 当前路点index
    // std::tuple<int32_t, int32_t> nextId = std::tuple<int32_t, int32_t>(0, 0); // 后继路的roadindex 和laneIndex
    std::vector<std::tuple<int32_t, int32_t>> nextIdList; // 后继所有路的roadID 和laneID
    std::vector<ReferenceLine> checkLineVector;           // 被动换道过程中，生成的参考线，用于线路比较用的
    std::vector<double> obstacleDistance;                 // save obstacle distances
};
// TODO:delete useless structs

double courseAngleRevise(double ang);
double speedSmoothing(double lastTargetSpeed, DecisionData &decisionData);

void loadStopPoints(const std::string fileName, std::vector<GaussRoadPoint> &rawStopPoints, std::tuple<int32_t, int32_t, int32_t> &stopPointRoadLaneId,
                    double offsetX, double offsetY);

bool loadMapFilePathName(const std::string yamlFileName, std::string &mapFilePathName); // 从yaml文件加载地图文件全路径文件名
void loadStopPointsFromYaml(const RoadMap &map_, const std::string &fileName, std::vector<GaussRoadPoint> &rawStopPoints, std::tuple<int32_t, int32_t, int32_t> &stopPointRoadLanePointId,
                            double offsetX, double offsetY);

std::tuple<int32_t, int32_t> getNextRoadLaneId(std::tuple<int32_t, int32_t> currentId, const std::vector<std::tuple<int32_t, int32_t>> &routingList);

std::vector<std::tuple<int32_t, int32_t>> getNextRoadLaneIdList(std::tuple<int32_t, int32_t> currentId, const std::vector<std::tuple<int32_t, int32_t>> &routingList);
double pointDistance(double x1, double x2, double y1, double y2);

double getPlanningDistance(double velocity, double curvature);
GaussRoadPoint getPlanningPoint(double distance, const GaussRoadPoint &currentPoint, const DecisionData &decisionData,
                                double yaw, std::tuple<int32_t, int32_t> nextId, const RoadMap &map);
double limitPlanningDistance(double targetDistance, double lastTargetDistance);

double getAngle(double x0, double y0, double x1, double y1);
double getAngleDiff(double ang0, double ang1);
double getYawDiff(double yawVehicle, double yawRoad);
// void localPlanning(const pc::Imu &imu, double velocity, const RoadMap &map, DecisionData &decisionData,
//                    const prediction::ObjectList &predictionMsg, const std::vector<std::tuple<int32_t, int32_t>> &routingList,
//                    const std::vector<GaussRoadPoint> stopPoints, const infopack::TrafficLight &trafficLights, std::vector<infopack::ObjectsProto> objectsCmd);
void localPlanning(const pc::Imu &imu, double velocity, RoadMap &map, DecisionData &decisionData,
                   const prediction::ObjectList &predictionMsg, std::vector<RoutingList> &routingListVector,
                   const std::vector<GaussRoadPoint> stopPoints, infopack::TrafficLight &trafficLights,
                   const std::vector<infopack::ObjectsProto> &objectsCmd,
                   std::map<std::string, infopack::IntersectionState> &spatTrafficLightMap, infopack::TrafficLightFromPerc &trafficLightFromPerc);

bool inArea(double longitudeX, double latitudeY, double targetX, double targetY);
bool stopPointJudge(const pc::Imu &imu, const std::vector<GaussRoadPoint> &stopPoints);

PlanningPoint pointOnCubicBezier(std::vector<PlanningPoint> cp, double t);
void generateBezierPathInFrenet(const PlanningPoint &startPoint, const PlanningPoint &endPoint, PlanningTrajectory &curve);
void clearPathList(std::vector<PlanningTrajectory> &pathList);

// 在生成的多条轨迹中选择最优的一条
// 20230623 增加了停车点输入变量
int32_t getTrajectoryPlanningResult(double velocity, DecisionData &decisionData, const pc::Imu &imu,
                                    const prediction::ObjectList &predictionMsg, const RoadMap &map,
                                    const infopack::TrafficLight &trafficLight, std::vector<infopack::ObjectsProto> objectsCmd,
                                    const std::vector<GaussRoadPoint> stopPoints);

// 寻找当前位置：RoadID、LaneID、当前点ID
bool getCurrentPosition(DecisionData &decisionData, const pc::Imu &imu, const RoadMap &map);

bool getCurrentPositionDuringLaneChanging(DecisionData &decisionData, const pc::Imu &imu, const RoadMap &map);
// 生成速度采样后，多种  轨迹上每个点的速度
// 20230623 增加了停车点输入变量
// void generateSpeedList(double velocity, DecisionData &decisionData, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu,const std::vector<GaussRoadPoint> stopPoints);
// 生成轨迹采样结果
void generateTrajectoryList(DecisionData &decisionData);

// 在一组轨迹中选择最好的
int32_t getOptimalTrajectoryIndex(DecisionData &decisionData, const prediction::ObjectList &prediction, const infopack::TrafficLight &trafficLight, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu);

bool trajectoryCollisionCheck(PlanningTrajectory &trajectory, const prediction::ObjectList &prediction);

bool ableToPassYellowLight(PlanningTrajectory &trajectory, double remaining_time, double lane_length_before_intersection);

bool pointCollisionCheck(const PlanningPoint &trajectoryPoint, PlanningPoint &predictPoint, double w, double l);

bool boundaryCollisionCheck(const PlanningPoint &p0, const PlanningPoint &p1, const PlanningPoint &p2, const PlanningPoint &p3);

bool innerCollisionCheck(const PlanningPoint &p0, const PlanningPoint &p1, const PlanningPoint &p2, const PlanningPoint &p3, const PlanningPoint &obj);

// 通过当前预瞄点，计算换道后的预瞄点
bool changeCurrentPoint(const Road &road, const Lane &curLane, int &laneId, int &pointIndex, GaussRoadPoint &currentPoint, bool isLeft);

PlanningTrajectory changeLocalToGlobal(PlanningTrajectory &trajectory, GaussRoadPoint gaussRoadPoint);

// 停车
void stop(DecisionData &decisionData, double velocity);

std::tuple<int32_t, int32_t> fixRoadLaneIndex(const DecisionData &decisionData, const RoadMap &map);

double calculateCurvature(const PlanningPoint &p0, const PlanningPoint &p1, const PlanningPoint &p2);

// 局部规划初始化
void initLocalPlanning(DecisionData &decisionData);

double assessTrajectory(PlanningTrajectory &trajectory, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu);

// 20130214
// double getMinDistanceOfPoint(const PlanningPoint &point, const prediction::ObjectList &prediction, const double &t = 0);
double getMinDistanceOfPoint(const PlanningPoint &point, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu);
// 一个计算点到障碍物距离平方的函数，避免开根号的运算，后续用于与安全距离的平方作比较
double getMinDistanceOfPointCheckSafety(const PlanningPoint &point, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu, double safeDistance);

ReferenceLine getReferenceLine(const RoadMap &map, const DecisionData &decisionData, GaussRoadPoint locationPoint, const pc::Imu &imu);

void getFrenetLocationPoint(DecisionData &decisionData, const ReferenceLine &referenceLine, GaussRoadPoint locationPoint);

double findMin(const std::vector<double> &array);

double findMax(const std::vector<double> &array);

void frenet2Cartesian(const double &s, const double &l, double &x, double &y, const ReferenceLine &referenceLine, int &lastIndex, const int &prevIndex);

// void generateBezierPathListInFrenet(const ReferenceLine &referenceLine, const PlanningPoint &frenetLocationPoint, std::vector<PlanningTrajectory> &pathList);

void generateSegmentedBezierPathListInFrenet(const RoadMap &map, const ReferenceLine &referenceLine, const PlanningPoint &frenetLocationPoint, std::vector<PlanningTrajectory> &pathList);

bool restart(const prediction::ObjectList &prediction, double velocity);
// bool calculateTrajectoryCost(const std::vector<Trajectory> &trajectoryList, const prediction::ObjectList &prediction, Trajectory &selectTrajectory);

// double speedModel(const double &distance, const double &maxspeed = 10, const double &minspeed = 0, const double &d1 = 4.5, const double &d2 = 0.5);

// void initSpeedForTrajectory(Trajectory &trajectory, const prediction::ObjectList &prediction);
// bool hasPrecedingVehicle(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> objectsCmd); // 车辆前方是否有需要跟随的车辆
// double cruiseController(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> objectsCmd);  // 计算ACC跟车速度
// 优化后的
bool hasPrecedingVehicle(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> &objectsCmd);                                       // 车辆前方是否有需要跟随的车辆
double cruiseController(const pc::Imu &imu, const std::vector<infopack::ObjectsProto> &objectsCmd, double objectIndex, double carDistance); // 计算ACC跟车速度

void gaussConvert(double longitude1, double latitude1, double &dNorth_X, double &dEast_Y);
void CoordTran2DForNew0INOld(double &dX, double &dY, double &dPhi, double dX0, double dY0, double dPhi0);

// 在全局规划的多条road -lane list中选择最好的一条,不包含当前道路
bool SelectBestRoutingList(std::vector<RoutingList> routingListVector, std::vector<std::tuple<int32_t, int32_t>> &bestRoutingList);

// 根据road lane 生成优化后的轨迹
// 20230623 增加了停车点输入变量
bool GetOptimalGlobalTrajectory(const pc::Imu &imu, const RoadMap &map, DecisionData &decisionData, const prediction::ObjectList &predictionMsg, const infopack::TrafficLight &trafficLights, const std::vector<infopack::ObjectsProto> &objectsCmd, int tempCurrentLaneId, int tempCurrentIndex, const std::vector<GaussRoadPoint> stopPoints);
// 根据全局规划的roadlanelist和当前的位置生成referencelane,用于换道的检查，这里先不考虑到达停车点的情况，后续可以再删除
bool GetCheckLine(RoadMap map, RoutingList rl, int roadID, int laneID, int pointID, double dRefLineLength, ReferenceLine &checkLine);

// lry0620 重新定于生成速度采样后，多种  轨迹上每个点的速度及相关函数
void generateSpeedList(double velocity, DecisionData &decisionData, std::vector<infopack::ObjectsProto> &objectsCmd, const prediction::ObjectList &prediction, const pc::Imu &imu, const std::vector<GaussRoadPoint> stopPoints, const infopack::TrafficLight &trafficLight, const RoadMap &map);
double getMinDistanceOfPoint_lry(const PlanningPoint &point, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> &objectsCmd, const pc::Imu &imu);
double getMinDistanceOfPoint_lry1(const PlanningPoint &point, const prediction::ObjectList &prediction, std::vector<infopack::ObjectsProto> &objectsCmd, const pc::Imu &imu, DecisionData &decisionData, int turnIndex, int followCarId);
double getPoint2Rec(double x, double y, double x1, double y1, double x2, double y2);
double point2Line(double x, double y, double x1, double y1, double x2, double y2);
double getDistance_2(double x1, double y1, double x2, double y2);
#endif
