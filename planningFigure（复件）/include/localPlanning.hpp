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

#include "controlMsg/control.pb.h"
#include "defineColor.h"
#include "imuMsg/imu.pb.h"
#include "mapMsg/localization_MapAnalysis.h"
#include "predictionMsg/prediction.pb.h"
#include "serverMsg/objects.pb.h"
#include "serverMsg/traffic_light.pb.h"
#include <yaml-cpp/yaml.h>

#define MINIMAL_ID 0

#define OFFSET_X -0.3
#define OFFSET_Y 0

// curve related.
#define CP_DISTANCE 3
#define CURVE_POINT_NUM 15 // 20230215  reduce  to 15 from 75
#define CURVE_NUM 1        // 20230215 reduce  9 // init is 9 //规划曲线的数量
#define CURVE_DISTANCE 1.5 // 20230215   0.4 //两条规划曲线终点间距
#define CP_DISTANCE_SECOND 1

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
};

struct DecisionData
{
    std::vector<PlanningTrajectory> finalPathList;
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
    std::tuple<int32_t, int32_t> nextId = std::tuple<int32_t, int32_t>(0, 0); // 后继路的roadindex 和laneIndex
};
// TODO:delete useless structs

double courseAngleRevise(double ang);
double speedSmoothing(double lastTargetSpeed, DecisionData &decisionData);

void loadStopPoints(const std::string fileName, std::vector<GaussRoadPoint> &rawStopPoints,
                    std::tuple<int32_t, int32_t, int32_t> &stopPointRoadLaneId, double offsetX,
                    double offsetY);

void loadStopPointsFromYaml(const RoadMap &map_, const std::string &fileName,
                            std::vector<GaussRoadPoint> &rawStopPoints,
                            std::tuple<int32_t, int32_t, int32_t> &stopPointRoadLanePointId, double offsetX,
                            double offsetY);

std::tuple<int32_t, int32_t> getNextRoadLaneId(std::tuple<int32_t, int32_t> currentId,
                                               const std::vector<std::tuple<int32_t, int32_t>> &routingList);

double pointDistance(double x1, double x2, double y1, double y2);

double getPlanningDistance(double velocity, double curvature);
GaussRoadPoint getPlanningPoint(double distance, const GaussRoadPoint &currentPoint,
                                const DecisionData &decisionData, double yaw,
                                std::tuple<int32_t, int32_t> nextId, const RoadMap &map);
double limitPlanningDistance(double targetDistance, double lastTargetDistance);

double getAngle(double x0, double y0, double x1, double y1);
double getAngleDiff(double ang0, double ang1);
double getYawDiff(double yawVehicle, double yawRoad);
void localPlanning(const pc::Imu &imu, double velocity, const RoadMap &map, DecisionData &decisionData,
                   const prediction::ObjectList &predictionMsg,
                   const std::vector<std::tuple<int32_t, int32_t>> &routingList,
                   const std::vector<GaussRoadPoint> stopPoints, const infopack::TrafficLight &trafficLights,
                   std::vector<infopack::ObjectsProto> objectsCmd);

bool inArea(double longitudeX, double latitudeY, double targetX, double targetY);
bool stopPointJudge(const pc::Imu &imu, const std::vector<GaussRoadPoint> &stopPoints);

PlanningPoint pointOnCubicBezier(std::vector<PlanningPoint> cp, double t);
void generateBezierPathInFrenet(const PlanningPoint &startPoint, const PlanningPoint &endPoint,
                                PlanningTrajectory &curve);
void clearPathList(std::vector<PlanningTrajectory> &pathList);

int32_t getTrajectoryPlanningResult(double velocity, DecisionData &decisionData, const pc::Imu &imu,
                                    const prediction::ObjectList &predictionMsg, const RoadMap &map,
                                    const ReferenceLine &ReferenceLine,
                                    const infopack::TrafficLight &trafficLight,
                                    std::vector<infopack::ObjectsProto> objectsCmd);
// 20220826
// 计算欧式距离
double getDistance(double x1, double y1, double x2, double y2);
// 计算点到线段距离
double pointToLineDistance(const GaussRoadPoint &startPoint, const GaussRoadPoint &stopPoint,
                           const GaussRoadPoint &checkPoint);

// 寻找当前位置：RoadID、LaneID、当前点ID
bool getCurrentPosition(DecisionData &decisionData, const pc::Imu &imu, const RoadMap &map);

bool getCurrentPositionDuringLaneChanging(DecisionData &decisionData, const pc::Imu &imu, const RoadMap &map);
// 生成速度采样后，多种  轨迹上每个点的速度
void generateSpeedList(double velocity, DecisionData &decisionData,
                       std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu);

// 生成轨迹采样结果
void generateTrajectoryList(DecisionData &decisionData);

// 在一组轨迹中选择最好的
int32_t getOptimalTrajectoryIndex(DecisionData &decisionData, const prediction::ObjectList &prediction,
                                  const infopack::TrafficLight &trafficLight,
                                  std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu);

bool trajectoryCollisionCheck(PlanningTrajectory &trajectory, const prediction::ObjectList &prediction);

bool ableToPassYellowLight(PlanningTrajectory &trajectory, double remaining_time,
                           double lane_length_before_intersection);

bool pointCollisionCheck(const PlanningPoint &trajectoryPoint, PlanningPoint &predictPoint, double w,
                         double l);

bool boundaryCollisionCheck(const PlanningPoint &p0, const PlanningPoint &p1, const PlanningPoint &p2,
                            const PlanningPoint &p3);

bool innerCollisionCheck(const PlanningPoint &p0, const PlanningPoint &p1, const PlanningPoint &p2,
                         const PlanningPoint &p3, const PlanningPoint &obj);

// 通过当前预瞄点，计算换道后的预瞄点
bool changeCurrentPoint(const Road &road, const Lane &curLane, int &laneId, int &pointIndex,
                        GaussRoadPoint &currentPoint, bool isLeft);

PlanningTrajectory changeLocalToGlobal(PlanningTrajectory &trajectory, GaussRoadPoint gaussRoadPoint);

// 停车
void stop(DecisionData &decisionData, double velocity);

std::tuple<int32_t, int32_t> fixRoadLaneIndex(const DecisionData &decisionData, const RoadMap &map);

double calculateCurvature(const PlanningPoint &p0, const PlanningPoint &p1, const PlanningPoint &p2);

// 局部规划初始化
void initLocalPlanning(DecisionData &decisionData);

double assessTrajectory(PlanningTrajectory &trajectory, const prediction::ObjectList &prediction,
                        std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu);

// 20130214
// double getMinDistanceOfPoint(const PlanningPoint &point, const prediction::ObjectList &prediction, const
// double &t = 0);
double getMinDistanceOfPoint(const PlanningPoint &point, const prediction::ObjectList &prediction,
                             std::vector<infopack::ObjectsProto> objectsCmd, const pc::Imu &imu);

ReferenceLine getReferenceLine(const RoadMap &map, const DecisionData &decisionData,
                               GaussRoadPoint locationPoint);

void getFrenetLocationPoint(DecisionData &decisionData, const ReferenceLine &referenceLine,
                            GaussRoadPoint locationPoint);

double findMin(const std::vector<double> &array);

double findMax(const std::vector<double> &array);

void frenet2Cartesian(const double &s, const double &l, double &x, double &y,
                      const ReferenceLine &referenceLine, int &lastIndex, const int &prevIndex);

void generateBezierPathListInFrenet(const ReferenceLine &referenceLine,
                                    const PlanningPoint &frenetLocationPoint,
                                    std::vector<PlanningTrajectory> &pathList);

bool restart(const prediction::ObjectList &prediction, double velocity);
// bool calculateTrajectoryCost(const std::vector<Trajectory> &trajectoryList, const prediction::ObjectList
// &prediction, Trajectory &selectTrajectory);

// double speedModel(const double &distance, const double &maxspeed = 10, const double &minspeed = 0, const
// double &d1 = 4.5, const double &d2 = 0.5);

// void initSpeedForTrajectory(Trajectory &trajectory, const prediction::ObjectList &prediction);
bool hasPrecedingVehicle(
    const pc::Imu &imu, const std::vector<infopack::ObjectsProto> objectsCmd); // 车辆前方是否有需要跟随的车辆
double cruiseController(const pc::Imu &imu,
                        const std::vector<infopack::ObjectsProto> objectsCmd); // 计算ACC跟车速度

void gaussConvert(double longitude1, double latitude1, double &dNorth_X, double &dEast_Y);
void CoordTran2DForNew0INOld(double &dX, double &dY, double &dPhi, double dX0, double dY0, double dPhi0);
#endif
