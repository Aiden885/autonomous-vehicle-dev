/* Auto-generated header */
#ifndef ACCPRO1_H
#define ACCPRO1_H


// INCLUDES_START

#include "math.h"
// #include "protobuf-c/protobuf-c.h"
typedef struct { int dummy; } ProtobufCMessage;
typedef int Infopack__TrafficLight__State;
#define PROTOBUF_C_MESSAGE_INIT(x) {0}
#include "stdbool.h"
#include "stddef.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// INCLUDES_END


// TYPEDEFS_START

typedef float Real;
typedef double Real64;
typedef int Integer;
typedef unsigned int UInteger;
typedef short Integer16;
typedef unsigned short UInteger16;
typedef signed char Integer8;
typedef unsigned char UInteger8;
typedef _Bool Boolean;

// TYPEDEFS_END


// MACROS_START

#define CURVE_NUM (9)// class=param,type=int,name=曲线数量（CURVE_NUM）,group=Planning,range=[1,9],desc="贝塞尔曲线数量"
#define CURVE_DISTANCE (1.0)// class=param,type=double,name=曲线间距（CURVE_DISTANCE）,group=Planning,range=[0.5，1],desc="贝塞尔曲线间距"
#define CP_DISTANCE (3)// class=param,type=int,name=控制点间距（CP_DISTANCE）,group=Planning,range=[1，3],desc="贝塞尔曲线控制点间距"
#define CURVE_POINT_NUM (15)// class=param,type=int,name=曲线点数量（CURVE_POINT_NUM）,group=Planning,range=[15,20],desc="贝塞尔曲线上点数量"
#define BEZIER_POINT_NUM (100)// class=param,type=int,name=参考线点数量（BEZIER_POINT_NUM）,group=Planning,range=[10,100],desc="参考线上最大点数量"
#define BEZIER_CONTROL_POINT_NUM (4)// class=param,type=int,name=贝塞尔曲线控制点数量（BEZIER_CONTROL_POINT_NUM）,group=Planning,range=[3,4],desc="贝塞尔曲线控制点数量"
#define DESIRED_SPEED (12 / 3.6)// class=param,type=double,name=期望速度（DESIRED_SPEED）,group=Planning,range=[5,12],desc="期望速度"
#define ACC_RELA_DIST (15.0)// class=param,type=double,name=ACC跟车距离（ACC_RELA_DIST）,group=Planning,range=[10,15],desc="ACC跟车距离"
#define ACC_BEGIN_DIST (45)// class=param,type=double,name=ACC进入跟车距离（ACC_BEGIN_DIST）,group=Planning,range=[40,45],desc="ACC进入跟车距离"
#define ACC_FOLLOW_OBJECT_ID (2)// class=param,type=int,name=ACC跟车目标ID（ACC_FOLLOW_OBJECT_ID）,group=Planning,range=[1,4],desc="ACC跟车目标ID"
#define SAFE_DISTANCE (1.0)// class=param,type=double,name=安全距离（SAFE_DISTANCE）,group=Planning,range=[0.5,1],desc="安全距离"
#define STOP_JUDGE_DISTANCE (6.0)// class=param,type=double,name=停止判断距离（STOP_JUDGE_DISTANCE）,group=Planning,range=[5,6],desc="停止判断距离"
#define MAX_GRAPH_NODES 64// 图的最大节点数
#define INFINITY_DIST 1.0e30f// 表示不可达的极大距离
#define INVALID_NODE_IDX (-1)// 无效节点索引哨兵值
#define RESET "\033[0m"
#define BLACK "\033[30m"      /* Black */
#define RED "\033[31m"      /* Red */
#define GREEN "\033[32m"      /* Green */
#define YELLOW "\033[33m"      /* Yellow */
#define BLUE "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN "\033[36m"      /* Cyan */
#define WHITE "\033[37m"      /* White */
#define BOLDBLACK "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"      /* Bold White */
#define PLANNING__TRAJECTORY_POINT__INIT { PROTOBUF_C_MESSAGE_INIT (&planning__trajectory_point__descriptor)     , 0, 0, 0, 0, 0, 0 }
#define PLANNING__TRAJECTORY_POINT_VEC__INIT { PROTOBUF_C_MESSAGE_INIT (&planning__trajectory_point_vec__descriptor)     , 0,NULL }
#define CONTROL_DATA__CHASSIS_INFO__INIT { PROTOBUF_C_MESSAGE_INIT (&control_data__chassis_info__descriptor)     , 0, 0, 0 }
#define INFOPACK__OBJECTS_VEC__INIT { PROTOBUF_C_MESSAGE_INIT (&infopack__objects_vec__descriptor)     , 0,NULL, 0 }
#define INFOPACK__TRAFFIC_LIGHT__INIT { PROTOBUF_C_MESSAGE_INIT (&infopack__traffic_light__descriptor)     , 0, INFOPACK__TRAFFIC_LIGHT__STATE__RED_LIGHT, 0, 0 }
#define INFOPACK__OBJECTS_PROTO__INIT { PROTOBUF_C_MESSAGE_INIT (&infopack__objects_proto__descriptor)     , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define INFOPACK__DISPATCH_PROTO__INIT { PROTOBUF_C_MESSAGE_INIT (&infopack__dispatch_proto__descriptor)     , 0, 0, 0, 0,NULL, 0,NULL, 0,NULL, 0, 0 }
#define INFOPACK__DECISIONS_LIST_PROTO__INIT { PROTOBUF_C_MESSAGE_INIT (&infopack__decisions_list_proto__descriptor)     , 0,NULL, 0,NULL }
#define PREDICTION__PREDICT_POINT__INIT { PROTOBUF_C_MESSAGE_INIT (&prediction__predict_point__descriptor)     , 0, 0, 0, 0 }
#define PREDICTION__OBJECT__INIT { PROTOBUF_C_MESSAGE_INIT (&prediction__object__descriptor)     , 0,NULL, 0, 0, 0, 0, 0, 0 }
#define PREDICTION__OBJECT_LIST__INIT { PROTOBUF_C_MESSAGE_INIT (&prediction__object_list__descriptor)     , 0,NULL }
#define UI__UI_PAD2_VDATA__INIT { PROTOBUF_C_MESSAGE_INIT (&ui__ui_pad2_vdata__descriptor)     , 0, 0, 0, 0, 0, 0, 0 }
#define UI__IMU__INIT { PROTOBUF_C_MESSAGE_INIT (&ui__imu__descriptor)     , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define UI__GNSS__INIT { PROTOBUF_C_MESSAGE_INIT (&ui__gnss__descriptor)     , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define UI__POSTURE__INIT { PROTOBUF_C_MESSAGE_INIT (&ui__posture__descriptor)     , 0,NULL, 0,NULL, 0 }
#define UI__IOV__INIT { PROTOBUF_C_MESSAGE_INIT (&ui__iov__descriptor)     , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define UI__CV__INIT { PROTOBUF_C_MESSAGE_INIT (&ui__cv__descriptor)     , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define UI__UI_V2_PAD_DATA__INIT { PROTOBUF_C_MESSAGE_INIT (&ui__ui_v2_pad_data__descriptor)     , 0,NULL, 0,NULL, 0,NULL }
#define PC__IMU__INIT { PROTOBUF_C_MESSAGE_INIT (&pc__imu__descriptor)     , 0, 0, 0, 0, 0, 0, 0, 0 }

// MACROS_END


// ENUMS_START

// ENUMS_END


// STRUCTS_START

typedef struct {
    ProtobufCMessage base;
    int type;
    int objectid;
    double lat;
    double lon;
    double x;
    double y;
    double yaw;
    double velocity;
    double power;
    int finish;
    int len;
    int width;
    int height;
    long long timestamp;
    int status;
    int vctype;
} Infopack__ObjectsProto;
 typedef struct {
    ProtobufCMessage base;
    double longitude;
    double latitude;
    double gaussx;
    double gaussy;
    int gpsvalid;
    double time;
    double velocity;
    double yaw;
} Pc__Imu;
 typedef struct {
    ProtobufCMessage base;
    unsigned long n_objmsg;
    Infopack__ObjectsProto ** objmsg;
    int sensorid;
} Infopack__ObjectsVec;
 typedef struct {
    ProtobufCMessage base;
    double x;
    double y;
    double vx;
    double vy;
} Prediction__PredictPoint;
 typedef struct {
    ProtobufCMessage base;
    unsigned long n_predictpoint;
    Prediction__PredictPoint ** predictpoint;
    double z;
    double w;
    double h;
    double l;
    int type;
    int trackid;
} Prediction__Object;
 typedef struct {
    ProtobufCMessage base;
    unsigned long n_object;
    Prediction__Object ** object;
} Prediction__ObjectList;
 typedef struct {
    ProtobufCMessage base;
    int active;
    Infopack__TrafficLight__State state;
    double remaining_time;
    double lane_length_before_intersection;
} Infopack__TrafficLight;
 typedef struct {
    double x;
    double y;
    double angle;
    double gaussX;
    double gaussY;
    double gaussAngle;
    double s;
    double l;
    double frenetAngle;
    double v;
    double curvature;
    double accumS;
} PlanningPoint;
 typedef struct {
    double GaussX;
    double GaussY;
    double yaw;
    double curvature;
    double s;
    double speedMax;
} GaussRoadPoint;
 typedef struct {
    PlanningPoint planningPoints;
} PlanningTrajectory;
 typedef struct {
    PlanningPoint referenceLinePoints;
    int referenceLinePointsNum;
} ReferenceLine;
 typedef struct {
    int roadIndex;
    int laneIndex;
} RoadLaneIdPair;
 typedef struct {
    PlanningTrajectory finalPathList;
    ReferenceLine referenceLine;
    PlanningPoint frenetLocationPoint;
    double speedList;
    PlanningTrajectory controlTrajectoryList;
    int feasibleTrajectoryIndexList;
    PlanningTrajectory optimalGlobalTrajectory;
    int optimalTrajectoryIndex;
    int currentId;
    int currentLaneId;
    int currentIndex;
    RoadLaneIdPair nextId;
} DecisionData;
 typedef struct {
    ReferenceLine referenceLine;
    PlanningPoint frenetLocationPoint;
    PlanningTrajectory pathList;
} generateBezierPathListInFrenetInput;
 typedef struct {
    PlanningTrajectory pathList;
} generateBezierPathListInFrenetOutput;
 typedef struct {
    PlanningPoint startPoint;
    PlanningPoint endPoint;
    PlanningTrajectory curve;
} generateBezierPathInFrenetInput;
 typedef struct {
    PlanningTrajectory curve;
} generateBezierPathInFrenetOutput;
 typedef struct {
    PlanningPoint cp;
    double t;
} pointOnCubicBezierInput;
 typedef struct {
    PlanningPoint result;
} pointOnCubicBezierOutput;
 typedef struct {
    Infopack__ObjectsVec* objectsCmd;
    Pc__Imu* imu;
} generateSpeedListParam;
 typedef struct {
    double velocity;
    DecisionData decisionData;
} generateSpeedListInput;
 typedef struct {
    DecisionData decisionData;
} generateSpeedListOutput;
 typedef struct {
    Pc__Imu* imu;
    Infopack__ObjectsVec* objectsCmd;
} hasPrecedingVehicleParam;
 typedef struct {
    bool flag;
} hasPrecedingVehicleOutput;
 typedef struct {
    double x1;
    double y1;
    double x2;
    double y2;
} getDistanceInput;
 typedef struct {
    double dis;
} getDistanceOutput;
 typedef struct {
    Pc__Imu* imu;
    Infopack__ObjectsVec* objectsCmd;
} cruiseControllerParam;
 typedef struct {
    Pc__Imu* imu;
    Infopack__ObjectsVec* objectsCmd;
    double velocity;
} cruiseControllerOutput;
 typedef struct {
    double startSpeed;
    double endSpeed;
    size_t num;
    double acceleration;
} interpConstAccInput;
 typedef struct {
    double result;
} interpConstAccOutput;
 typedef struct {
    Prediction__ObjectList* prediction;
} restartParam;
 typedef struct {
    double velocity;
} restartInput;
 typedef struct {
    bool flag;
} restartOutput;
 typedef struct {
    double longitudeX;
    double latitudeY;
    double targetX;
    double targetY;
} inAreaInput;
 typedef struct {
    bool flag;
} inAreaOutput;
 typedef struct {
    Pc__Imu* imu;
    GaussRoadPoint stopPoints;
} stopPointJudgeParam;
 typedef struct {
    bool flag;
} stopPointJudgeOutput;
 typedef struct {
    DecisionData decisionData;
    double velocity;
} stopInput;
 typedef struct {
    DecisionData decisionData;
} stopOutput;
 typedef struct {
    double startSpeed;
    double endSpeed;
    size_t num;
} interpLinearInput;
 typedef struct {
    double result;
} interpLinearOutput;
 typedef struct {
    Prediction__ObjectList* prediction;
    Infopack__TrafficLight* trafficLight;
    Pc__Imu* imu;
    Infopack__ObjectsVec* objectsCmd;
} getOptimalTrajectoryIndexParam;
 typedef struct {
    DecisionData decisionData;
} getOptimalTrajectoryIndexInput;
 typedef struct {
    DecisionData decisionData;
    int32_t index;
} getOptimalTrajectoryIndexOutput;
 typedef struct {
    Prediction__ObjectList* prediction;
    Infopack__TrafficLight* trafficLight;
} processTrafficLightParam;
 typedef struct {
    DecisionData decisionData;
} processTrafficLightInput;
 typedef struct {
    DecisionData decisionData;
    int32_t flag;
} processTrafficLightOutput;
 typedef struct {
    Prediction__ObjectList* prediction;
    Pc__Imu* imu;
    Infopack__ObjectsVec* objectsCmd;
} assessTrajectoryParam;
 typedef struct {
    PlanningTrajectory trajectory;
} assessTrajectoryInput;
 typedef struct {
    double minDistance;
} assessTrajectoryOutput;
 typedef struct {
    double x;
    double y;
    double yaw;
    double v;
    double k;
    double s;
    double acc;
} TrajPoint;
 typedef struct {
    TrajPoint* points;
    size_t size;
} Traj;
 typedef struct {
    double x;
    double v;
    double a;
} VehicleState;
 typedef struct {
    double integral;
    double lastError;
} PIDState;
 typedef struct {
    double x;
    double y;
    double yaw;
    double v;
    double accX;
    double rtkMode;
} State;
 typedef struct {
    Real AdjMatrix[64][64];
    Integer NodeCount;
    Integer SourceNode;
    Integer TargetNode;
} DijkstraStepInput_t;
 typedef struct {
    Integer MaxNodes;
} DijkstraStepParam_t;
 typedef struct {
    Real Dist[64];
    Boolean Visited[64];
    Integer PrevNode[64];
} DijkstraStepState_t;
 typedef struct {
    Real ShortestDist;
    Integer Path[64];
    Integer PathLength;
    Boolean PathFound;
} DijkstraStepOutput_t;
 typedef struct {
    Integer MinNode;
    Real MinDist;
} FindMinNodeOutput_t;
 typedef struct {
    Integer Path[64];
    Integer PathLength;
} BacktrackPathOutput_t;

// STRUCTS_END


// FUNCTIONS_START

double accComputeTargetSpeed( double egoV, double leadV, double distance );
 /* 更新PID控制器的输出值 */
double pid_db9453ce_cfb7_4712_9c92_507dced3adf5(
    double kp,  // [in]
    double ki,  // [in]
    double kd,  // [in]
    double dt,  // [in]
    double integral_max,  // [in]
    double derivative_max,  // [in]
    double error,  // [in] 误差值
    PIDState * state  // [inout] PID状态参数结构体
);
 double limitSymmetrical(double value, double limit);
 double computeDistance1D(double leadX, double egoX);
 double computeSpeedError(double targetSpeed, double egoV);
 /* for循环体内部逻辑 */
void execute_dd594018(
    VehicleState * vehiclePoint,  // [in] 外部变量 vehiclePoint
    double kp,  // [in] 外部变量 kp
    double ki,  // [in] 外部变量 ki
    double kd,  // [in] 外部变量 kd
    double dt,  // [in] 外部变量 dt
    PIDState pidState  // [in] 外部变量 pidState
);
 void vehicleModelUpdate( double accel, double dt, VehicleState* state );
 /* 基于简单纵向运动学模型完成PID模块测试 */
void pidTestMain_16044152_4d25_400b_a90c_555a157c2bda(
);

// FUNCTIONS_END


#endif