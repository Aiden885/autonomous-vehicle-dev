#ifndef __LOCALPLANNING_M_HPP__
#define __LOCALPLANNING_M_HPP__

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <unistd.h>
#include <chrono>
#include "interpolate_m.h"
#include <algorithm>

#include "controlMsg/control.pb.h"
#include "defineColor.h"
#include "imuMsg/imu.pb.h"
#include "mapMsg/localization_MapAnalysis.h"
#include "predictionMsg/prediction.pb.h"
#include "serverMsg/objects.pb.h"
#include "serverMsg/traffic_light.pb.h"
#include <yaml-cpp/yaml.h>
#include "localPlanning.hpp"

using namespace std;

/**
 * \namespace LocalPlanning
 * \brief 这是一个名为LocalPlanning的namespace。
 */
// namespace LocalPlanning{
#define MINIMAL_ID 0

#define OFFSET_X -0.3
#define OFFSET_Y 0

// curve related.
#define CP_DISTANCE 3
#define CURVE_POINT_NUM 15 // 20230215  reduce  to 15 from 75
#define CURVE_NUM 9        // 20230215 reduce  9 // init is 9 //规划曲线的数量
#define CURVE_DISTANCE 1   // 20230215   0.4 //两条规划曲线终点间距
#define CP_DISTANCE_SECOND 1

// struct PlanningPoint
// {
//     // local path
//     double x;
//     double y;
//     double angle;
//     // global path
//     double gaussX;
//     double gaussY;
//     double gaussAngle;
//     // frenet path
//     double s;
//     double l;
//     double frenetAngle;
//     // speed
//     double v;
//     // curvature
//     double curvature;
//     // accumS
//     double accumS;
// };

// struct PlanningTrajectory
// {
//     std::vector<PlanningPoint> planningPoints;
// };

// struct ReferenceLine
// {
//     std::vector<PlanningPoint> referenceLinePoints;
// };

// struct DecisionData
// {
//     std::vector<PlanningTrajectory> finalPathList;
//     ReferenceLine referenceLine;
//     PlanningPoint frenetLocationPoint;
//     std::vector<std::vector<double>> speedList;                               // 20220821 每种速度采样上每个点的速度
//     std::vector<PlanningTrajectory> controlTrajectoryList;                    // 20220826 轨迹
//     std::vector<int32_t> feasibleTrajectoryIndexList;                         // 20220904
//     PlanningTrajectory optimalGlobalTrajectory;                               // 最优轨迹转化成全局坐标
//     int32_t optimalTrajectoryIndex;                                           // 最优轨迹序号
//     int32_t currentId = 0;                                                    // 当前RoadID
//     int32_t currentLaneId = 0;                                                // 当前LaneID          20220825添加
//     int32_t currentIndex = 0;                                                 // 当前路点index
//     std::tuple<int32_t, int32_t> nextId = std::tuple<int32_t, int32_t>(0, 0); // 后继路的roadindex 和laneIndex
// };
// // TODO:delete useless structs

struct courseAngleReviseParam
{
};

struct courseAngleReviseInput
{
    double ang;
};

struct courseAngleReviseOutput
{
    double ang;
};

void courseAngleRevise(const courseAngleReviseParam &param, const courseAngleReviseInput &input, courseAngleReviseOutput &output);  //done

struct loadStopPointsParam
{
    std::string fileName;
};

struct loadStopPointsInput
{
    std::vector<GaussRoadPoint> rawStopPoints;
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId;
    double offsetX;
    double offsetY;
};

struct loadStopPointsOutput
{
    std::vector<GaussRoadPoint> rawStopPoints;
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId;
};

void loadStopPoints(const loadStopPointsParam &param, const loadStopPointsInput &input, loadStopPointsOutput &output);  //done

struct loadStopPointsFromYamlParam
{
    RoadMap map_;
    std::string fileName;
};

struct loadStopPointsFromYamlInput
{
    std::vector<GaussRoadPoint> rawStopPoints;
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId;
    double offsetX;
    double offsetY;
};

struct loadStopPointsFromYamlOutput
{
    std::vector<GaussRoadPoint> rawStopPoints;
    std::tuple<int32_t, int32_t, int32_t> stopPointRoadLanePointId;
};

void loadStopPointsFromYaml(const loadStopPointsFromYamlParam &param, const loadStopPointsFromYamlInput &input, loadStopPointsFromYamlOutput &output);  //done

struct getNextRoadLaneIdParam
{
    std::vector<std::tuple<int32_t, int32_t>> routingList;
};

struct getNextRoadLaneIdInput
{
    std::tuple<int32_t, int32_t> currentId;
};

struct getNextRoadLaneIdOutput
{
    std::tuple<int32_t, int32_t> nextRoadLaneId;
};

void getNextRoadLaneId(const getNextRoadLaneIdParam &param, const getNextRoadLaneIdInput &input, getNextRoadLaneIdOutput &output);  //done

struct pointDistanceParam
{
};

struct pointDistanceInput
{
    double x1;
    double x2;
    double y1;
    double y2;
};

struct pointDistanceOutput
{
    double distance;
};

void pointDistance(const pointDistanceParam &param, const pointDistanceInput &input, pointDistanceOutput &output);  //done

struct getPlanningDistanceParam
{
};

struct getPlanningDistanceInput
{
    double velocity;
    double curvature;
};

struct getPlanningDistanceOutput
{
    double distance;
};

void getPlanningDistance(const getPlanningDistanceParam &param, const getPlanningDistanceInput &input, getPlanningDistanceOutput &output);  //done

struct getPlanningPointParam
{
    RoadMap map;
};

struct getPlanningPointInput
{
    GaussRoadPoint currentPoint;
    DecisionData decisionData;
    double distance;
    double yaw;
    std::tuple<int32_t, int32_t> nextId;
};

struct getPlanningPointOutput
{
    GaussRoadPoint ret;
};

void getPlanningPoint(const getPlanningPointParam &param, const getPlanningPointInput &input, getPlanningPointOutput &output);  //done

struct limitPlanningDistanceParam
{
};

struct limitPlanningDistanceInput
{
    double targetDistance;
    double lastTargetDistance;
};

struct limitPlanningDistanceOutput
{
    double targetDistance;
};
  //done
void limitPlanningDistance(const limitPlanningDistanceParam &param, const limitPlanningDistanceInput &input, limitPlanningDistanceOutput &output);

struct getAngleParam
{
};

struct getAngleInput
{
    double x0;
    double y0;
    double x1;
    double y1;
};

struct getAngleOutput
{
    double ang;
};
  //done
void getAngle(const getAngleParam &param, const getAngleInput &input, getAngleOutput &output);

struct getAngleDiffParam
{
};

struct getAngleDiffInput
{
    double ang0;
    double ang1;
};

struct getAngleDiffOutput
{
    double ret;
};
  //done
void getAngleDiff(const getAngleDiffParam &param, const getAngleDiffInput &input, getAngleDiffOutput &output);
  //processing
void localPlanning(const pc::Imu &imu, double velocity, const RoadMap &map, DecisionData &decisionData, const prediction::ObjectList &predictionMsg,
                   const std::vector<std::tuple<int32_t, int32_t>> &routingList, const std::vector<GaussRoadPoint> stopPoints, const infopack::TrafficLight &trafficLights,
                   std::vector<infopack::ObjectsProto> objectsCmd);

struct inAreaParam
{
};

struct inAreaInput
{
    double longitudeX;
    double latitudeY;
    double targetX;
    double targetY;
};

struct inAreaOutput
{
    bool flag;
};
  //done
void inArea(const inAreaParam &param, const inAreaInput &input, inAreaOutput &output);

struct stopPointJudgeParam
{
    pc::Imu imu;
    std::vector<GaussRoadPoint> stopPoints;
};

struct stopPointJudgeInput
{
};

struct stopPointJudgeOutput
{
    bool flag;
};
  //done
void stopPointJudge(const stopPointJudgeParam &param, const stopPointJudgeInput &input, stopPointJudgeOutput &output);

struct pointOnCubicBezierParam
{
};

struct pointOnCubicBezierInput
{
    std::vector<PlanningPoint> cp;
    double t;
};

struct pointOnCubicBezierOutput
{
    PlanningPoint result;
};
  //done
void pointOnCubicBezier(const pointOnCubicBezierParam &param, const pointOnCubicBezierInput &input, pointOnCubicBezierOutput &output);

struct generateBezierPathInFrenetParam
{
};

struct generateBezierPathInFrenetInput
{
    PlanningPoint startPoint;
    PlanningPoint endPoint;
    PlanningTrajectory curve;
};

struct generateBezierPathInFrenetOutput
{
    PlanningTrajectory curve;
};
  //done
void generateBezierPathInFrenet(const generateBezierPathInFrenetParam &param, const generateBezierPathInFrenetInput &input, generateBezierPathInFrenetOutput &output);

struct clearPathListParam
{
};

struct clearPathListInput
{
    std::vector<PlanningTrajectory> pathList;
};

struct clearPathListOutput
{
    std::vector<PlanningTrajectory> pathList;
};
  //done
void clearPathList(const clearPathListParam &param, const clearPathListInput &input, clearPathListOutput &output);

struct getTrajectoryPlanningResultParam
{
    pc::Imu imu;
    prediction::ObjectList predictionMsg;
    RoadMap map;
    infopack::TrafficLight trafficLight;
    std::vector<infopack::ObjectsProto> objectsCmd;
};

struct getTrajectoryPlanningResultInput
{
    double velocity;
    DecisionData decisionData;
    ReferenceLine referenceLine;
};

struct getTrajectoryPlanningResultOutput
{
    DecisionData decisionData;
    int32_t index;
};
  //done
void getTrajectoryPlanningResult(const getTrajectoryPlanningResultParam &param, const getTrajectoryPlanningResultInput &input, getTrajectoryPlanningResultOutput &output);

struct getDistanceParam
{
};

struct getDistanceInput
{
    double x1;
    double y1;
    double x2;
    double y2;
};

struct getDistanceOutput
{
    double dis;
};
  //done
void getDistance(const getDistanceParam &param, const getDistanceInput &input, getDistanceOutput &output);

struct pointToLineDistanceParam
{
};

struct pointToLineDistanceInput
{
    GaussRoadPoint startPoint;
    GaussRoadPoint stopPoint;
    GaussRoadPoint checkPoint;
};

struct pointToLineDistanceOutput
{
    double dis;
};
// 计算点到线段距离  //done
void pointToLineDistance(const pointToLineDistanceParam &param, const pointToLineDistanceInput &input, pointToLineDistanceOutput &output);

struct getCurrentPositionParam
{
    pc::Imu imu;
    RoadMap map;
};

struct getCurrentPositionInput
{
    DecisionData decisionData;
};

struct getCurrentPositionOutput
{
    DecisionData decisionData;
    bool flag;
};
// 寻找当前位置：RoadID、LaneID、当前点ID  //done
void getCurrentPosition(const getCurrentPositionParam &param, const getCurrentPositionInput &input, getCurrentPositionOutput &output);

struct generateTrajectoryListParam
{
};

struct generateTrajectoryListInput
{
    DecisionData decisionData;
};

struct generateTrajectoryListOutput
{
    DecisionData decisionData;
};
// 生成速度采样后，多种  轨迹上每个点的速度
// 20230623 增加了停车点输入变量
// 生成轨迹采样结果  //done
void generateTrajectoryList(const generateTrajectoryListParam &param, const generateTrajectoryListInput &input, generateTrajectoryListOutput &output);

struct getOptimalTrajectoryIndexParam
{
    prediction::ObjectList prediction;
    infopack::TrafficLight trafficLight;
    pc::Imu imu;
    std::vector<infopack::ObjectsProto> objectsCmd;
};

struct getOptimalTrajectoryIndexInput
{
    DecisionData decisionData;
};

struct getOptimalTrajectoryIndexOutput
{
    DecisionData decisionData;
    int32_t index;
};
// 在一组轨迹中选择最好的  //done
void getOptimalTrajectoryIndex(const getOptimalTrajectoryIndexParam &param, const getOptimalTrajectoryIndexInput &input, getOptimalTrajectoryIndexOutput &output);

struct trajectoryCollisionCheckParam
{
    prediction::ObjectList prediction;
};

struct trajectoryCollisionCheckInput
{
    PlanningTrajectory trajectory;
};

struct trajectoryCollisionCheckOutput
{
    PlanningTrajectory trajectory;
    bool flag;
};
  //done
void trajectoryCollisionCheck(const trajectoryCollisionCheckParam &param, const trajectoryCollisionCheckInput &input, trajectoryCollisionCheckOutput &output);

struct ableToPassYellowLightParam
{
};

struct ableToPassYellowLightInput
{
    PlanningTrajectory trajectory;
    double remaining_time;
    double lane_length_before_intersection;
};

struct ableToPassYellowLightOutput
{
    PlanningTrajectory trajectory;
    bool flag;
};
  //done
void ableToPassYellowLight(const ableToPassYellowLightParam &param, const ableToPassYellowLightInput &input, ableToPassYellowLightOutput &output);

struct pointCollisionCheckParam
{
};

struct pointCollisionCheckInput
{
    PlanningPoint trajectoryPoint;
    PlanningPoint predictPoint;
    double w;
    double l;
};

struct pointCollisionCheckOutput
{
    bool flag;
};
  //done
void pointCollisionCheck(const pointCollisionCheckParam &param, const pointCollisionCheckInput &input, pointCollisionCheckOutput &output);

struct boundaryCollisionCheckParam
{
};

struct boundaryCollisionCheckInput
{
    PlanningPoint p0;
    PlanningPoint p1;
    PlanningPoint p2;
    PlanningPoint p3;
};

struct boundaryCollisionCheckOutput
{
    bool flag;
};
  //done
void boundaryCollisionCheck(const boundaryCollisionCheckParam &param, const boundaryCollisionCheckInput &input, boundaryCollisionCheckOutput &output);

struct innerCollisionCheckParam
{
};

struct innerCollisionCheckInput
{
    PlanningPoint p0;
    PlanningPoint p1;
    PlanningPoint p2;
    PlanningPoint p3;
    PlanningPoint obj;
};

struct innerCollisionCheckOutput
{
    bool flag;
};
  //done
void innerCollisionCheck(const innerCollisionCheckParam &param, const innerCollisionCheckInput &input, innerCollisionCheckOutput &output);

struct changeCurrentPointParam
{
    Road road;
    Lane curLane;
};

struct changeCurrentPointInput
{
    int laneId;
    int pointIndex;
    GaussRoadPoint currentPoint;
    bool isLeft;
};

struct changeCurrentPointOutput
{
    int laneId;
    int pointIndex;
    GaussRoadPoint currentPoint;
    bool flag;
};
  //done
// 通过当前预瞄点，计算换道后的预瞄点
void changeCurrentPoint(const changeCurrentPointParam &param, const changeCurrentPointInput &input, changeCurrentPointOutput &output);

struct changeLocalToGlobalParam
{
};

struct changeLocalToGlobalInput
{
    PlanningTrajectory trajectory;
    GaussRoadPoint gaussRoadPoint;
};

struct changeLocalToGlobalOutput
{
    PlanningTrajectory globalTrajectory;
};
  //done
// 通过当前预瞄点，计算换道后的预瞄点
void changeLocalToGlobal(const changeLocalToGlobalParam &param, const changeLocalToGlobalInput &input, changeLocalToGlobalOutput &output);

struct stopParam
{
};

struct stopInput
{
    DecisionData decisionData;
    double velocity;
};

struct stopOutput
{
    DecisionData decisionData;
};
// 停车  //done
void stop(const stopParam &param, const stopInput &input, stopOutput &output);

struct fixRoadLaneIndexParam
{
    RoadMap map;
};

struct fixRoadLaneIndexInput
{
    DecisionData decisionData;
};

struct fixRoadLaneIndexOutput
{
    std::tuple<int32_t, int32_t> indexTuple;
};
// 停车  //done
void fixRoadLaneIndex(const fixRoadLaneIndexParam &param, const fixRoadLaneIndexInput &input, fixRoadLaneIndexOutput &output);

struct calculateCurvatureParam
{
};

struct calculateCurvatureInput
{
    PlanningPoint p0;
    PlanningPoint p1;
    PlanningPoint p2;
};

struct calculateCurvatureOutput
{
    double curvature;
};
// 停车  //done
void calculateCurvature(const calculateCurvatureParam &param, const calculateCurvatureInput &input, calculateCurvatureOutput &output);

struct initLocalPlanningParam
{
};

struct initLocalPlanningInput
{
    DecisionData decisionData;
};

struct initLocalPlanningOutput
{
    DecisionData decisionData;
};
// 局部规划初始化  //done
void initLocalPlanning(const initLocalPlanningParam &param, const initLocalPlanningInput &input, initLocalPlanningOutput &output);

struct assessTrajectoryParam
{
    prediction::ObjectList prediction;
    pc::Imu imu;
    std::vector<infopack::ObjectsProto> objectsCmd;
};

struct assessTrajectoryInput
{
    PlanningTrajectory trajectory;
};

struct assessTrajectoryOutput
{
    double minDistance;
};
  //done
void assessTrajectory(const assessTrajectoryParam &param, const assessTrajectoryInput &input, assessTrajectoryOutput &output);

struct getMinDistanceOfPointParam
{
    prediction::ObjectList prediction;
    pc::Imu imu;
    std::vector<infopack::ObjectsProto> objectsCmd;
};

struct getMinDistanceOfPointInput
{
    PlanningPoint point;
};

struct getMinDistanceOfPointOutput
{
    double minDistance;
};
  //done
void getMinDistanceOfPoint(const getMinDistanceOfPointParam &param, const getMinDistanceOfPointInput &input, getMinDistanceOfPointOutput &output);

struct getReferenceLineParam
{
    RoadMap map;
};

struct getReferenceLineInput
{
    DecisionData decisionData;
    GaussRoadPoint locationPoint;
};

struct getReferenceLineOutput
{
    ReferenceLine referenceLine;
};
  //done
void getReferenceLine(const getReferenceLineParam &param, const getReferenceLineInput &input, getReferenceLineOutput &output);

struct getFrenetLocationPointParam
{
};

struct getFrenetLocationPointInput
{
    DecisionData decisionData;
    ReferenceLine referenceLine;
    GaussRoadPoint locationPoint;
};

struct getFrenetLocationPointOutput
{
    DecisionData decisionData;
};
  //done
void getFrenetLocationPoint(const getFrenetLocationPointParam &param, const getFrenetLocationPointInput &input, getFrenetLocationPointOutput &output);

struct findMinParam
{
};

struct findMinInput
{
    std::vector<double> array;
};

struct findMinOutput
{
    double value;
};
  //done
void findMin(const findMinParam &param, const findMinInput &input, findMinOutput &output);

struct findMaxParam
{
};

struct findMaxInput
{
    std::vector<double> array;
};

struct findMaxOutput
{
    double value;
};
  //done
void findMax(const findMaxParam &param, const findMaxInput &input, findMaxOutput &output);

struct frenet2CartesianParam
{
};

struct frenet2CartesianInput
{
    double s;
    double l;
    double x;
    double y;
    ReferenceLine referenceLine;
    int lastIndex;
    int prevIndex;
};

struct frenet2CartesianOutput
{
    double x;
    double y;
    int lastIndex;
};
  //done
void frenet2Cartesian(const frenet2CartesianParam &param, const frenet2CartesianInput &input, frenet2CartesianOutput &output);

struct generateSegmentedBezierPathListInFrenetParam
{
    RoadMap map;
};

struct generateSegmentedBezierPathListInFrenetInput
{
    ReferenceLine referenceLine;
    PlanningPoint frenetLocationPoint;
    std::vector<PlanningTrajectory> pathList;
};

struct generateSegmentedBezierPathListInFrenetOutput
{
    std::vector<PlanningTrajectory> pathList;
};
  //done
void generateSegmentedBezierPathListInFrenet(const generateSegmentedBezierPathListInFrenetParam &param, const generateSegmentedBezierPathListInFrenetInput &input,
                                             generateSegmentedBezierPathListInFrenetOutput &output);

struct restartParam
{
    prediction::ObjectList prediction;
};

struct restartInput
{
    double velocity;
};

struct restartOutput
{
    bool flag;
};
  //done
void restart(const restartParam &param, const restartInput &input, restartOutput &output);

// 优化后的
struct hasPrecedingVehicleParam
{
    pc::Imu imu;
    std::vector<infopack::ObjectsProto> objectsCmd;
};

struct hasPrecedingVehicleInput
{
};

struct hasPrecedingVehicleOutput
{
    bool flag;
};
  //done
void hasPrecedingVehicle(const hasPrecedingVehicleParam &param, const hasPrecedingVehicleInput &input, hasPrecedingVehicleOutput &output); // 车辆前方是否有需要跟随的车辆

struct cruiseControllerParam
{
    pc::Imu imu;
    std::vector<infopack::ObjectsProto> objectsCmd;
};

struct cruiseControllerInput
{
};

struct cruiseControllerOutput
{
    double velocity;
};
  //done
void cruiseController(const cruiseControllerParam &param, const cruiseControllerInput &input, cruiseControllerOutput &output); // 计算ACC跟车速度

struct gaussConvertParam
{
};

struct gaussConvertInput
{
    double longitude1;
    double latitude1;
    double dNorth_X;
    double dEast_Y;
};

struct gaussConvertOutput
{
    double dNorth_X;
    double dEast_Y;
};
  //done
void gaussConvert(const gaussConvertParam &param, const gaussConvertInput &input, gaussConvertOutput &output);

struct coordTran2DParam
{
};

struct coordTran2DInput
{
    double dX;
    double dY;
    double dPhi;
    double dX0;
    double dY0;
    double dPhi0;
};

struct coordTran2DOutput
{
    double dX;
    double dY;
    double dPhi;
};

void coordTran2D(const coordTran2DParam &param, const coordTran2DInput &input, coordTran2DOutput &output);

struct generateBezierPathListInFrenetParam
{
};

struct generateBezierPathListInFrenetInput
{
    ReferenceLine referenceLine;
    PlanningPoint frenetLocationPoint;
    std::vector<PlanningTrajectory> pathList;
};

struct generateBezierPathListInFrenetOutput
{
    std::vector<PlanningTrajectory> pathList;
};
  //done
void generateBezierPathListInFrenet(const generateBezierPathListInFrenetParam &param, const generateBezierPathListInFrenetInput &input, generateBezierPathListInFrenetOutput &output);

struct generateSpeedListParam
{
    std::vector<infopack::ObjectsProto> objectsCmd;
    pc::Imu imu;
};

struct generateSpeedListInput
{
    double velocity;
    DecisionData decisionData;
};

struct generateSpeedListOutput
{
    DecisionData decisionData;
};

void generateSpeedList(const generateSpeedListParam &param, const generateSpeedListInput &input, generateSpeedListOutput &output);

struct angleRegulateParam
{
};

struct angleRegulateInput
{
    double angleInput;
};

struct angleRegulateOutput
{
    double angleOutput;
};

void angleRegulate(const angleRegulateParam &param, const angleRegulateInput &input, angleRegulateOutput &output);

struct processTrafficLightParam
{
    prediction::ObjectList prediction;
    infopack::TrafficLight trafficLight;
};

struct processTrafficLightInput
{
    DecisionData decisionData;
};

struct processTrafficLightOutput
{
    DecisionData decisionData;
    int flag;
};

void processTrafficLight(const processTrafficLightParam &param, const processTrafficLightInput &input, processTrafficLightOutput &output);
// }
#endif
