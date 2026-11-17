/**
 * @file apaPlannerConfig.h
 * @brief APA 全局规划参数配置 — 对应 get_planner_global.m
 *
 * 将 MATLAB persistent 全局结构体改写为 C99 层次化 struct + 默认初始化函数。
 * 分为四个子结构体：MapConfig / VehConfig / ConfConfig / SlotConfig
 *
 * @date 2026-04-13
 */
#ifndef APA_PLANNER_CONFIG_H
#define APA_PLANNER_CONFIG_H

#include "baseTypes.h"
#include "apaEnums.h"
#include "apaVehicleDefaults.h"
#include "apaTrackingPolicy.h"
#include <math.h>

#ifndef APA_PI
#define APA_PI 3.14159265f
#endif

typedef struct {
  Real MotionResolution;           /**< 路径插值分辨率 [m] */
  Real NSteer;                     /**< 转向离散数 */
  Real ExtendArea;                 /**< 地图扩展区域 [m] */
  Real XyGridResolution;           /**< 栅格分辨率 [m] */
  Real YawGridResolution;          /**< 航向分辨率 [rad] */
  Real MotionStepLength;           /**< 运动步长 [m] */
  Real CollisionDetectStepLength;  /**< 碰撞检测步长 [m] */
  Real MinYaw;                     /**< 航向最小值 [rad] */
  Real MaxYaw;                     /**< 航向最大值 [rad] */
  Real SbCost;                     /**< 换挡惩罚 */
  Real BackCost;                   /**< 倒车惩罚 */
  Real SteerChangeCost;            /**< 转角变化惩罚 */
  Real SteerCost;                  /**< 转角惩罚 */
  Real HCost;                      /**< 启发式代价权重 */
  Integer MapEdgePixNum;           /**< 地图边像素数 */
  Real MapEdgeLen;                 /**< 地图边长 [m] */
  Integer MapYawPixNum;            /**< 航向像素数 */
  Real BackCostInSlot;             /**< 槽内倒车惩罚 */
  Boolean EnableFirstSegDeltaCost; /**< 第一段cost权重 */
  Real SteerCost1stSeg;            /**< 第一段转角惩罚 */
} ApaPlannerMapConfig_t;

typedef struct {
  Real W;         /**< 车宽 [m] */
  Real LL;        /**< 车长 [m] */
  Real WB;        /**< 轴距 [m] */
  Real LB;        /**< 后悬 [m] */
  Real LF;        /**< 前悬+轴距 [m] */
  Real Lf;        /**< 前悬 [m] */
  Real MinCircle; /**< 最小转弯半径 [m] */
  Real MaxSteer;  /**< 最大转角 [rad] */
  /* 安全裕量（通用） */
  Real FrontSafeMargin;
  Real RearSafeMargin;
  Real SideSafeMargin;
  /* 安全裕量（近车位 — 垂直车位） */
  Real FrontSafeMarginNearSlotVt;
  Real RearSafeMarginNearSlotVt;
  Real SideSafeMarginNearSlotVt;
  /* 安全裕量（近车位 — 平行车位） */
  Real FrontSafeMarginNearSlotHt;
  Real RearSafeMarginNearSlotHt;
  Real SideSafeMarginNearSlotHt;
  /* 安全裕量（远车位 — 垂直车位） */
  Real FrontSafeMarginAwaySlotVt;
  Real RearSafeMarginAwaySlotVt;
  Real SideSafeMarginAwaySlotVt;
  /* 安全裕量（远车位 — 平行车位） */
  Real FrontSafeMarginAwaySlotHt;
  Real RearSafeMarginAwaySlotHt;
  Real SideSafeMarginAwaySlotHt;
  /* 轴基裕量 */
  Real WhlBseLongMarginVt;
  Real WhlBseLongMarginHt;
  Real WhlBseLatMargin;
  /* 八边形碰撞模型参数 */
  Real FrontTriLat;
  Real FrontTriLon;
  Real BackTriLat;
  Real BackTriLon;
  /* 碰撞深度阈值 */
  Real CollisionDepthThresh;
} ApaPlannerVehConfig_t;

typedef struct {
  /* 搜索算法配置 */
  Integer SearchStyle;                /**< 1=4方向/L1范数, 2=8方向/L2范数 */
  Integer BoundaryType;               /**< 1=k-means映射, 2=直接赋值 */
  Integer CollisionCheckMethod;       /**< 1=polyxpoly, 2=SAT */
  Integer PathSmoothMethod;           /**< 1=梯度下降, 2=最速下降, 3=梯度法 */
  Real GradDescentStep;               /**< 梯度下降步长 */
  Integer SteepestLinearSearchMethod; /**< 线搜索方法 */
  Real OmegaO;                        /**< 障碍物权重 */
  Real OmegaS;                        /**< 平滑权重 */
  Real OmegaKappa;                    /**< 曲率权重 */
  Real OmegaVoronoi;                  /**< Voronoi权重 */
  Real DMax;                          /**< 最大距离 [m] */
  Integer CurvatureTermCalcMethod;    /**< 曲率计算方法 */
  Integer UseEmptyParkingSpace;       /**< 是否使用空车位 */

  /* 列表容量限制 */
  Integer HybridOpenListLimit;
  Integer HybridCloseListLimit;
  Integer GridOpenListLimit;
  Integer GridCloseListLimit;
  Integer PathMaxLenLimit;
  Integer PathValidLenLimit;

  /* 算力配置 */
  Integer PlanningCalPowerMaxPerFrame;
  Integer PlanningCalPowerMaxPerSlot;
  Integer PlanningCalPowerMinPerSlot;
  Integer GridMapSearchAlgorithm; /**< 1=贪心, 2=A* */
  Integer EnableYawGainPenalty;
  Boolean UseMoreCalUnits;
  Boolean ParkingSearch; /* Consistent short-motion search for parallel parking. */
  Real AddPtPathThresh;
  Integer UseGCost; /**< 0=f=h(贪心), 1=f=h+g */

  /* 重规划阈值 */
  Real LongErrMaxThresh;
  Real LatErrMaxThresh;
  Real YawErrMaxThresh;
  Real LongErrMinThresh;
  Real LatErrMinThresh;
  Real YawErrMinThresh;
  Real LongErrAdjustThresh;
  Real LatErrAdjustThresh;
  Real YawErrAdjustThresh;

  /* 目标位置相关 */
  Real TransitionPosLen;
  Real EnterPosLen;
  Real PolyfitPathLenMin;

  /* 功能开关 */
  Integer SixthReplanType;
  Integer IgnoreCurvatureLimitInOpenSpace;
  Integer HeuristicValueType;
  Integer NeedHybridAlgFusion;
  Integer BoundaryComposeType;
  Integer DetectSearchingPathStopperForParSlot;
  Integer DetectParkingPathStopperForParSlot;
  Integer CollisionModel;
  Integer ExpectStartDirection;
  Real ExpectDirectionOdoThresh;
  Integer AllowTarPosErrExist;

  /* 误差允许范围 */
  Real LongAllowableErrMin;
  Real LongAllowableErrMax;
  Real LatAllowableErrMin;
  Real LatAllowableErrMax;
  Real YawAllowableErrMin;
  Real YawAllowableErrMax;

  /* 路径长度范围 */
  Real PathLenMin;
  Real PathLenMax;

  /* 高级特性开关 */
  Integer UsePriorityQueue;
  Boolean EnableNodesMemory;
  Integer UseSatCollisionCheck;
  Boolean ReplayMode;
  Boolean EnableFirstSegVirtualBound;
  Real MaxFirstSegLatDis;
  Boolean EnableGenerateReversePoint;
  Real ReverseDist;
  Integer StillVehNewObsCollisionDetectCnt;
  Real BrakeDistToAvoidCollision;
  Boolean EnableParallelPlanInSlot;
  Boolean EnableGeoPlannerForPostProcess;
  Boolean EnableSimplifyStraitPath;
  Boolean EnableJumpPoints;
  Boolean EnableThirdReplanForDynamicObs;
  Boolean EnableThirdReplanForStopper;
  Boolean EnableErrorCheck;
} ApaPlannerConfConfig_t;

typedef struct {
  Integer ReverseFlag;
  Real ReverseTheta;
} ApaPlannerSlotConfig_t;

typedef struct {
  ApaPlannerMapConfig_t Map;
  ApaPlannerVehConfig_t Veh;
  ApaPlannerConfConfig_t Conf;
  ApaPlannerSlotConfig_t Slot;
} ApaPlannerConfig_t;

static inline void apaPlannerConfigInitDefault(ApaPlannerConfig_t *cfg) {
  cfg->Map.MotionResolution = 0.1f;
  cfg->Map.NSteer = 20.0f;
  cfg->Map.ExtendArea = 0.0f;
  cfg->Map.XyGridResolution = 0.5f;
  cfg->Map.YawGridResolution = 0.087266f;
  cfg->Map.MotionStepLength = 6.0f;
  cfg->Map.CollisionDetectStepLength = 0.5f;
  cfg->Map.MinYaw = -APA_PI;
  cfg->Map.MaxYaw = APA_PI;
  cfg->Map.SbCost = 1.0f;
  cfg->Map.BackCost = 1.0f;
  cfg->Map.SteerChangeCost = 0.0f;
  cfg->Map.SteerCost = 0.0f;
  cfg->Map.HCost = 5.0f;
  cfg->Map.MapEdgePixNum = 480;
  cfg->Map.MapEdgeLen = 24.0f;
  cfg->Map.MapYawPixNum = 72;
  cfg->Map.BackCostInSlot = 1.1f;
  cfg->Map.EnableFirstSegDeltaCost = false;
  cfg->Map.SteerCost1stSeg = 8.0f;

  cfg->Veh.W = APA_VEHICLE_WIDTH_M;
  cfg->Veh.LL = APA_VEHICLE_LENGTH_M;
  cfg->Veh.WB = APA_VEHICLE_WHEELBASE_M;
  cfg->Veh.LB = APA_VEHICLE_REAR_TO_BACK_M;
  cfg->Veh.LF = APA_VEHICLE_REAR_TO_FRONT_M;
  cfg->Veh.Lf = APA_VEHICLE_FRONT_OVERHANG_M;
  cfg->Veh.MinCircle =
      fmaxf(APA_VEHICLE_MIN_TURN_RADIUS_M, cfg->Veh.WB / tanf(APA_VEHICLE_MAX_STEER_RAD));
  cfg->Veh.MaxSteer = atanf(cfg->Veh.WB / cfg->Veh.MinCircle);

  cfg->Veh.FrontSafeMargin = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.RearSafeMargin = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.SideSafeMargin = APA_PARKIN_PHYSICAL_MARGIN_M;

  cfg->Veh.FrontSafeMarginNearSlotVt = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.RearSafeMarginNearSlotVt = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.SideSafeMarginNearSlotVt = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.FrontSafeMarginNearSlotHt = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.RearSafeMarginNearSlotHt = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.SideSafeMarginNearSlotHt = APA_PARKIN_PHYSICAL_MARGIN_M;

  cfg->Veh.FrontSafeMarginAwaySlotVt = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.RearSafeMarginAwaySlotVt = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.SideSafeMarginAwaySlotVt = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.FrontSafeMarginAwaySlotHt = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.RearSafeMarginAwaySlotHt = APA_PARKIN_PHYSICAL_MARGIN_M;
  cfg->Veh.SideSafeMarginAwaySlotHt = APA_PARKIN_PHYSICAL_MARGIN_M;

  cfg->Veh.WhlBseLongMarginVt = 0.40f;
  cfg->Veh.WhlBseLongMarginHt = 0.37f;
  cfg->Veh.WhlBseLatMargin = 0.4f;

  cfg->Veh.FrontTriLat = APA_VEHICLE_FRONT_CHAMFER_LAT_M;
  cfg->Veh.FrontTriLon = APA_VEHICLE_FRONT_CHAMFER_LON_M;
  cfg->Veh.BackTriLat = APA_VEHICLE_REAR_CHAMFER_LAT_M;
  cfg->Veh.BackTriLon = APA_VEHICLE_REAR_CHAMFER_LON_M;
  cfg->Veh.CollisionDepthThresh = 0.4f;

  cfg->Conf.SearchStyle = 1;
  cfg->Conf.BoundaryType = 2;
  cfg->Conf.CollisionCheckMethod = 2;
  cfg->Conf.PathSmoothMethod = 2;
  cfg->Conf.GradDescentStep = 0.00005f;
  cfg->Conf.SteepestLinearSearchMethod = 4;
  cfg->Conf.OmegaO = 1.0f;
  cfg->Conf.OmegaS = 2.0f;
  cfg->Conf.OmegaKappa = 0.8f;
  cfg->Conf.OmegaVoronoi = 0.0f;
  cfg->Conf.DMax = 2.0f;
  cfg->Conf.CurvatureTermCalcMethod = 2;
  cfg->Conf.UseEmptyParkingSpace = 1;
  cfg->Conf.HybridOpenListLimit = 1000;
  cfg->Conf.HybridCloseListLimit = 1000;
  cfg->Conf.GridOpenListLimit = 200;
  cfg->Conf.GridCloseListLimit = 800;
  cfg->Conf.PathMaxLenLimit = 100;
  cfg->Conf.PathValidLenLimit = 70;
  cfg->Conf.PlanningCalPowerMaxPerFrame = 180;
  cfg->Conf.PlanningCalPowerMaxPerSlot = 150;
  cfg->Conf.PlanningCalPowerMinPerSlot = 50;
  cfg->Conf.GridMapSearchAlgorithm = 2;
  cfg->Conf.EnableYawGainPenalty = 1;
  cfg->Conf.UseMoreCalUnits = true;
  cfg->Conf.ParkingSearch = false;
  cfg->Conf.AddPtPathThresh = 0.3f;
  cfg->Conf.UseGCost = 1;

  cfg->Conf.LongErrMaxThresh = 1.25f;
  cfg->Conf.LatErrMaxThresh = 0.2f;
  cfg->Conf.YawErrMaxThresh = 0.087266f;
  cfg->Conf.LongErrMinThresh = 0.03f;
  cfg->Conf.LatErrMinThresh = 0.05f;
  cfg->Conf.YawErrMinThresh = 0.008727f;
  cfg->Conf.LongErrAdjustThresh = 0.10f;
  cfg->Conf.LatErrAdjustThresh = 0.06f;
  cfg->Conf.YawErrAdjustThresh = 0.008727f;

  cfg->Conf.TransitionPosLen = 1.0f;
  cfg->Conf.EnterPosLen = 2.5f;
  cfg->Conf.PolyfitPathLenMin = 1.0f;

  cfg->Conf.SixthReplanType = 1;
  cfg->Conf.IgnoreCurvatureLimitInOpenSpace = 0;
  cfg->Conf.HeuristicValueType = 0;
  cfg->Conf.NeedHybridAlgFusion = 1;
  cfg->Conf.BoundaryComposeType = 3;
  cfg->Conf.DetectSearchingPathStopperForParSlot = 0;
  cfg->Conf.DetectParkingPathStopperForParSlot = 1;
  cfg->Conf.CollisionModel = 0;
  cfg->Conf.ExpectStartDirection = 0;
  cfg->Conf.ExpectDirectionOdoThresh = 0.15f;
  cfg->Conf.AllowTarPosErrExist = 1;

  cfg->Conf.LongAllowableErrMin = -0.05f;
  cfg->Conf.LongAllowableErrMax = 0.05f;
  cfg->Conf.LatAllowableErrMin = -0.05f;
  cfg->Conf.LatAllowableErrMax = 0.05f;
  cfg->Conf.YawAllowableErrMin = -0.008727f;
  cfg->Conf.YawAllowableErrMax = 0.008727f;

  cfg->Conf.PathLenMin = 0.12f;
  cfg->Conf.PathLenMax = 100.0f;

  cfg->Conf.UsePriorityQueue = 1;
  cfg->Conf.EnableNodesMemory = true;
  cfg->Conf.UseSatCollisionCheck = 1;
  cfg->Conf.ReplayMode = false;
  cfg->Conf.EnableFirstSegVirtualBound = true;
  cfg->Conf.MaxFirstSegLatDis = 1.8f;
  cfg->Conf.EnableGenerateReversePoint = false;
  cfg->Conf.ReverseDist = 1.0f;
  cfg->Conf.StillVehNewObsCollisionDetectCnt = 20;
  cfg->Conf.BrakeDistToAvoidCollision = 0.6f;
  cfg->Conf.EnableParallelPlanInSlot = true;
  cfg->Conf.EnableGeoPlannerForPostProcess = true;
  cfg->Conf.EnableSimplifyStraitPath = false;
  cfg->Conf.EnableJumpPoints = true;
  cfg->Conf.EnableThirdReplanForDynamicObs = false;
  cfg->Conf.EnableThirdReplanForStopper = false;
  cfg->Conf.EnableErrorCheck = false;

  cfg->Slot.ReverseFlag = 0;
  cfg->Slot.ReverseTheta = 0.0f;
}

#endif
