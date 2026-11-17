#ifndef VEHICLE_COLLISION_CHECK_TYPES_H
#define VEHICLE_COLLISION_CHECK_TYPES_H

#include "baseTypes.h"
#include "calcVehCornerPointTypes.h"
#include "collisionCheckSatTypes.h"

/** @brief 边界线段最大数量 */
#ifndef VCC_MAX_BOUNDARY_LINES
#define VCC_MAX_BOUNDARY_LINES 64
#endif
/** @brief 离散障碍物点最大数量 */
#ifndef VCC_MAX_SEPARATE_PTS
#define VCC_MAX_SEPARATE_PTS 128
#endif
/** @brief slot_track_stopper 最大数量 */
#ifndef VCC_MAX_TRACK_STOPPER
#define VCC_MAX_TRACK_STOPPER 64
#endif
/** @brief virtual_boundary_line 最大数量 */
#ifndef VCC_MAX_VIRTUAL_LINES
#define VCC_MAX_VIRTUAL_LINES 64
#endif
/** @brief 碰撞区域数量 */
#ifndef VCC_COLLISION_AREA_COUNT
#define VCC_COLLISION_AREA_COUNT 9
#endif

/** @brief 碰撞检测类型 (对齐 EnumCollisionCheckType) */
#define VCC_CHECK_TYPE_PLANNING 0
#define VCC_CHECK_TYPE_GENERAL 1
#define VCC_CHECK_TYPE_DYNAMIC 2
#define VCC_CHECK_TYPE_STATIC 3

/** @brief 碰撞模型 (对齐 EnumCollisionModel) */
#define VCC_COLLISION_MODEL_FOUR_EDGES 0
#define VCC_COLLISION_MODEL_EIGHT_EDGES 1

/** @brief 碰撞需求 (对齐 EnumCollisionDemand) */
#define VCC_COLLISION_DEMAND_SIMPLIFY 0
#define VCC_COLLISION_DEMAND_ALL 1

/** @brief 碰撞类型输出值 */
#define VCC_COLLISION_NONE 0
#define VCC_COLLISION_BOUNDARY 1
#define VCC_COLLISION_STOPPER 2
#define VCC_COLLISION_VIRTUAL 3

/** @brief Native collision slot kind; diagonal retains its polygon identity. */
#define VCC_SLOT_CLASS_VERTICAL 0
#define VCC_SLOT_CLASS_PARALLEL 1
#define VCC_SLOT_CLASS_DIAGONAL 2

/* Margin family is independent of rectangular-template eligibility. */
static inline Boolean vehicleCollisionCheckUsesCrossSlotMargins(Integer slotClass) {
  return slotClass == VCC_SLOT_CLASS_VERTICAL || slotClass == VCC_SLOT_CLASS_DIAGONAL;
}

/**
 * @brief 车辆碰撞检测 SIMPLIFY 模式输入
 * @en_name VehicleCollisionCheckInput
 * @cn_name 车辆碰撞检测输入
 * @tag STRUCT
 * @field name=CheckType, type=Integer, unit=1, desc=碰撞检测类型
 * @field name=CollisionModel, type=Integer, unit=1, desc=碰撞模型
 * @field name=CollisionDemand, type=Integer, unit=1, desc=碰撞需求
 * @field name=ParkingOutFlag, type=Boolean, unit=1, desc=泊出标志
 * @field name=VerticalLastStep, type=Boolean, unit=1, desc=垂直车位最后一步标志
 * @field name=SlotClass, type=Integer, unit=1, desc=车位类型
 * @field name=TarPosValid, type=Boolean, unit=1, desc=目标位姿有效
 * @field name=TarPosX, type=Real, unit=m, desc=目标位姿X
 * @field name=TarPosY, type=Real, unit=m, desc=目标位姿Y
 * @field name=TarPosTheta, type=Real, unit=rad, desc=目标位姿航向
 * @field name=VehPosX, type=Real, unit=m, desc=车辆后轴X
 * @field name=VehPosY, type=Real, unit=m, desc=车辆后轴Y
 * @field name=VehPosTheta, type=Real, unit=rad, desc=车辆航向角
 * @field name=VehLF, type=Real, unit=m, desc=前悬+轴距
 * @field name=VehLB, type=Real, unit=m, desc=后悬
 * @field name=VehLL, type=Real, unit=m, desc=车长
 * @field name=VehW, type=Real, unit=m, desc=车宽
 * @field name=VehWB, type=Real, unit=m, desc=轴距
 * @field name=UseAutoMargin, type=Boolean, unit=1, desc=自动按MATLAB语义计算裕量
 * @field name=VehNearSlotFlag, type=Boolean, unit=1, desc=车辆贴近车位标志
 * @field name=VehInSlotFlag, type=Boolean, unit=1, desc=车辆在车位内标志
 * @field name=FrontSafeMargin, type=Real, unit=m, desc=前部安全裕量
 * @field name=RearSafeMargin, type=Real, unit=m, desc=后部安全裕量
 * @field name=SideSafeMargin, type=Real, unit=m, desc=侧向安全裕量
 * @field name=WhlBseLongMargin, type=Real, unit=m, desc=轴基纵向裕量
 * @field name=WhlBseLatMargin, type=Real, unit=m, desc=轴基横向裕量
 * @field name=FrontSafeMarginNearSlotVt, type=Real, unit=m, desc=垂直近车位前裕量
 * @field name=RearSafeMarginNearSlotVt, type=Real, unit=m, desc=垂直近车位后裕量
 * @field name=SideSafeMarginNearSlotVt, type=Real, unit=m, desc=垂直近车位侧裕量
 * @field name=FrontSafeMarginAwaySlotVt, type=Real, unit=m, desc=垂直远车位前裕量
 * @field name=RearSafeMarginAwaySlotVt, type=Real, unit=m, desc=垂直远车位后裕量
 * @field name=SideSafeMarginAwaySlotVt, type=Real, unit=m, desc=垂直远车位侧裕量
 * @field name=FrontSafeMarginNearSlotHt, type=Real, unit=m, desc=平行近车位前裕量
 * @field name=RearSafeMarginNearSlotHt, type=Real, unit=m, desc=平行近车位后裕量
 * @field name=SideSafeMarginNearSlotHt, type=Real, unit=m, desc=平行近车位侧裕量
 * @field name=FrontSafeMarginAwaySlotHt, type=Real, unit=m, desc=平行远车位前裕量
 * @field name=RearSafeMarginAwaySlotHt, type=Real, unit=m, desc=平行远车位后裕量
 * @field name=SideSafeMarginAwaySlotHt, type=Real, unit=m, desc=平行远车位侧裕量
 * @field name=FrontTriLat, type=Real, unit=m, desc=前三角横向边长
 * @field name=FrontTriLon, type=Real, unit=m, desc=前三角纵向边长
 * @field name=BackTriLat, type=Real, unit=m, desc=后三角横向边长
 * @field name=BackTriLon, type=Real, unit=m, desc=后三角纵向边长
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=第一边界线段数
 * @field name=BoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=边界线段起点X
 * @field name=BoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=边界线段起点Y
 * @field name=BoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=边界线段终点X
 * @field name=BoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=边界线段终点Y
 * @field name=BoundarySepPtsNum, type=Integer, unit=1, desc=边界离散点数
 * @field name=BoundarySepPtsX, type=Array<Real, [128]>, unit=m, desc=边界离散点X
 * @field name=BoundarySepPtsY, type=Array<Real, [128]>, unit=m, desc=边界离散点Y
 * @field name=SlotTrackStopperNum, type=Integer, unit=1, desc=slot stopper数
 * @field name=SlotTrackStopperX1, type=Array<Real, [64]>, unit=m, desc=stopper端点1 X
 * @field name=SlotTrackStopperY1, type=Array<Real, [64]>, unit=m, desc=stopper端点1 Y
 * @field name=SlotTrackStopperX2, type=Array<Real, [64]>, unit=m, desc=stopper端点2 X
 * @field name=SlotTrackStopperY2, type=Array<Real, [64]>, unit=m, desc=stopper端点2 Y
 * @field name=SeparateStopperNum, type=Integer, unit=1, desc=离散stopper数
 * @field name=SeparateStopperX, type=Array<Real, [128]>, unit=m, desc=离散stopper X
 * @field name=SeparateStopperY, type=Array<Real, [128]>, unit=m, desc=离散stopper Y
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线段数
 * @field name=VirtualBoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=虚拟边界起点X
 * @field name=VirtualBoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=虚拟边界起点Y
 * @field name=VirtualBoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=虚拟边界终点X
 * @field name=VirtualBoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=虚拟边界终点Y
 * @field name=SlotPtsValid, type=Boolean, unit=1, desc=车位角点有效
 * @field name=SlotPts, type=Array<Real, [8]>, unit=m, desc=入口顺序四角点[P1x,P1y,...,P4x,P4y]，P1-P2入口，P3-P4库底
 * @version 1.2
 * @date 2026-05-02
 * @author aide
 */
typedef struct {
  Integer CheckType;
  Integer CollisionModel;
  Integer CollisionDemand;
  Boolean ParkingOutFlag;
  Boolean VerticalLastStep;
  Integer SlotClass;
  Boolean TarPosValid;
  Real TarPosX;
  Real TarPosY;
  Real TarPosTheta;
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Real VehLF;
  Real VehLB;
  Real VehLL; // 车长(LF+LB+wheelbase等价；负三角形扩展使用)
  Real VehW;
  Real VehWB;
  /* 自动裕量解析模式（true=按MATLAB语义计算；false=直接使用Front/Rear/Side） */
  Boolean UseAutoMargin;
  Boolean VehNearSlotFlag;
  Boolean VehInSlotFlag;
  Real FrontSafeMargin;
  Real RearSafeMargin;
  Real SideSafeMargin;
  Real WhlBseLongMargin;
  Real WhlBseLatMargin;
  /* 自动裕量配置表（仅当 UseAutoMargin=true 时使用） */
  Real FrontSafeMarginNearSlotVt;
  Real RearSafeMarginNearSlotVt;
  Real SideSafeMarginNearSlotVt;
  Real FrontSafeMarginAwaySlotVt;
  Real RearSafeMarginAwaySlotVt;
  Real SideSafeMarginAwaySlotVt;
  Real FrontSafeMarginNearSlotHt;
  Real RearSafeMarginNearSlotHt;
  Real SideSafeMarginNearSlotHt;
  Real FrontSafeMarginAwaySlotHt;
  Real RearSafeMarginAwaySlotHt;
  Real SideSafeMarginAwaySlotHt;
  Real FrontTriLat;
  Real FrontTriLon;
  Real BackTriLat;
  Real BackTriLon;
  /* 第一边界：boundary_line */
  Integer BoundaryLineNum;
  Real BoundaryLineX1[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineY1[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineX2[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineY2[VCC_MAX_BOUNDARY_LINES];
  Integer BoundarySepPtsNum;
  Real BoundarySepPtsX[VCC_MAX_SEPARATE_PTS];
  Real BoundarySepPtsY[VCC_MAX_SEPARATE_PTS];
  /* 第二边界：stopper */
  Integer SlotTrackStopperNum;
  Real SlotTrackStopperX1[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperY1[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperX2[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperY2[VCC_MAX_TRACK_STOPPER];
  Integer SeparateStopperNum;
  Real SeparateStopperX[VCC_MAX_SEPARATE_PTS];
  Real SeparateStopperY[VCC_MAX_SEPARATE_PTS];
  /* 第三边界：virtual_boundary_line (仅 PLANNING 模式) */
  Integer VirtualBoundaryLineNum;
  Real VirtualBoundaryLineX1[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineY1[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineX2[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineY2[VCC_MAX_VIRTUAL_LINES];
  /* MATLAB tar_pos 省略时 angle_to_slot(slot_pts, veh_pos) 的回退输入。 */
  Boolean SlotPtsValid;
  Real SlotPts[8]; // Entrance-first: P1-P2 entrance, P3-P4 bottom.
} VehicleCollisionCheckInput_t;

/**
 * @brief 车辆碰撞检测 SIMPLIFY 模式输出
 * @en_name VehicleCollisionCheckOutput
 * @cn_name 车辆碰撞检测输出
 * @tag STRUCT
 * @field name=IsCollision, type=Boolean, unit=1, desc=是否碰撞
 * @field name=CollisionType, type=Integer, unit=1, desc=碰撞类型
 * @field name=CollisionIndex, type=Integer, unit=1, desc=碰撞对象索引
 * @field name=CollisionArea, type=Array<UInteger8, [9]>, unit=1, desc=碰撞区域
 * @field name=CollisionDepth, type=Real, unit=m, desc=碰撞深度
 * @field name=CrossLine, type=UInteger8, unit=1, desc=穿越边界编号
 * @field name=SafeMargin1stBound, type=Array<Real, [3]>, unit=m, desc=第一边界安全裕量
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Boolean IsCollision;    // 是否碰撞
  Integer CollisionType;  // 碰撞类型(0-无 1-边界 2-stopper 3-虚拟边界)
  Integer CollisionIndex; // 碰撞对象索引(1-based)
  UInteger8 CollisionArea[VCC_COLLISION_AREA_COUNT]; // 碰撞区域
  Real CollisionDepth;                               // 碰撞深度
  UInteger8 CrossLine;                               // 穿越边界编号
  Real SafeMargin1stBound[3];                        // 第一边界安全裕量
} VehicleCollisionCheckOutput_t;

/** @brief Load the planner's canonical automatic collision margins. */
void vehicleCollisionCheckLoadDefaultMargins(VehicleCollisionCheckInput_t *input);

/** @brief 解析车位虚拟线安全裕量，单位 m。 */
void vehicleCollisionCheckResolveVirtualMargins(const VehicleCollisionCheckInput_t *input,
                                                Real margins[3]);

/** @brief 解析车身安全裕量，单位 m。 */
void vehicleCollisionCheckResolveBodyMargins(const VehicleCollisionCheckInput_t *input,
                                             Real margins[3]);

void vehicleCollisionCheck(const VehicleCollisionCheckInput_t *input,
                           VehicleCollisionCheckOutput_t *output);

#endif
