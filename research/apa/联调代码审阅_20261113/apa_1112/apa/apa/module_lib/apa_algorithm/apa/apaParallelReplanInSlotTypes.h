#ifndef APA_PARALLEL_REPLAN_IN_SLOT_TYPES_H
#define APA_PARALLEL_REPLAN_IN_SLOT_TYPES_H

#include "baseTypes.h"
#include "vehicleCollisionCheckTypes.h"

#ifndef APA_PARALLEL_REPLAN_PATH_POINT_MAX
#define APA_PARALLEL_REPLAN_PATH_POINT_MAX 100
#endif

/**
 * @brief 槽内重规划输入
 * @en_name ApaParallelReplanInSlotInput
 * @cn_name 槽内重规划输入
 * @tag STRUCT
 * @field name=ReplanReqType, type=Integer, unit=1, desc=重规划请求类型
 * @field name=StartX, type=Real, unit=m, desc=当前车辆X
 * @field name=StartY, type=Real, unit=m, desc=当前车辆Y
 * @field name=StartTheta, type=Real, unit=rad, desc=当前车辆航向
 * @field name=GoalX, type=Real, unit=m, desc=目标X
 * @field name=GoalY, type=Real, unit=m, desc=目标Y
 * @field name=GoalTheta, type=Real, unit=rad, desc=目标航向
 * @field name=Rmin, type=Real, unit=m, desc=最小转弯半径
 * @field name=ExpectDirFlag, type=Integer, unit=1, desc=期望起步方向标志
 * @field name=IsHandleSlot, type=Integer, unit=1, desc=是否为自定义车位
 * @field name=SlotSource, type=Integer, unit=1, desc=车位来源(0=视觉 1=空间)
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=障碍边界线数量
 * @field name=BoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=障碍边界起点X
 * @field name=BoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=障碍边界起点Y
 * @field name=BoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=障碍边界终点X
 * @field name=BoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=障碍边界终点Y
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界线数量
 * @field name=VirtualBoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=虚拟边界起点X
 * @field name=VirtualBoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=虚拟边界起点Y
 * @field name=VirtualBoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=虚拟边界终点X
 * @field name=VirtualBoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=虚拟边界终点Y
 * @field name=BoundarySepPtNum, type=Integer, unit=1, desc=真实边界离散点数量
 * @field name=BoundarySepPtX, type=Array<Real, [128]>, unit=m, desc=真实边界离散点X
 * @field name=BoundarySepPtY, type=Array<Real, [128]>, unit=m, desc=真实边界离散点Y
 * @field name=StopperPtNum, type=Integer, unit=1, desc=挡停点数量
 * @field name=StopperPtX, type=Array<Real, [128]>, unit=m, desc=挡停点X
 * @field name=StopperPtY, type=Array<Real, [128]>, unit=m, desc=挡停点Y
 * @version 1.0
 * @date 2026-04-19
 * @author aide
 */
typedef struct {
  Integer ReplanReqType;
  Real StartX;
  Real StartY;
  Real StartTheta;
  Real GoalX;
  Real GoalY;
  Real GoalTheta;
  Real Rmin;
  Integer ExpectDirFlag;
  Integer IsHandleSlot;
  Integer SlotSource;
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer BoundaryLineNum;
  Real BoundaryLineX1[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineY1[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineX2[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineY2[VCC_MAX_BOUNDARY_LINES];
  Integer VirtualBoundaryLineNum;
  Real VirtualBoundaryLineX1[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineY1[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineX2[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineY2[VCC_MAX_VIRTUAL_LINES];
  Integer BoundarySepPtNum;
  Real BoundarySepPtX[VCC_MAX_SEPARATE_PTS];
  Real BoundarySepPtY[VCC_MAX_SEPARATE_PTS];
  Integer StopperPtNum;
  Real StopperPtX[VCC_MAX_SEPARATE_PTS];
  Real StopperPtY[VCC_MAX_SEPARATE_PTS];
} ApaParallelReplanInSlotInput_t;

/**
 * @brief 槽内重规划参数
 * @en_name ApaParallelReplanInSlotParam
 * @cn_name 槽内重规划参数
 * @tag STRUCT
 * @field name=ShortCutSw, type=Integer, unit=1, desc=是否启用shortcut
 * @field name=ShortCutThd, type=Real, unit=m, desc=shortcut阈值
 * @version 1.0
 * @date 2026-04-19
 * @author aide
 */
typedef struct {
  Integer ShortCutSw;
  Real ShortCutThd;
} ApaParallelReplanInSlotParam_t;

/**
 * @brief 槽内重规划输出
 * @en_name ApaParallelReplanInSlotOutput
 * @cn_name 槽内重规划输出
 * @tag STRUCT
 * @field name=Success, type=Integer, unit=1, desc=是否规划成功
 * @field name=VehicleInSlot, type=Integer, unit=1, desc=车辆是否已在车位内
 * @field name=ExistStopperInSlot, type=Integer, unit=1, desc=车位内是否存在挡停点
 * @field name=SurroundingType, type=Integer, unit=1, desc=周边环境类型
 * @field name=AppliedReqType, type=Integer, unit=1, desc=实际执行的请求类型
 * @field name=PathLen, type=Real, unit=m, desc=路径长度
 * @field name=PtCount, type=Integer, unit=1, desc=路径点数量
 * @field name=PthPtX, type=Array<Real, [100]>, unit=m, desc=路径点X
 * @field name=PthPtY, type=Array<Real, [100]>, unit=m, desc=路径点Y
 * @field name=PthPtTht, type=Array<Real, [100]>, unit=rad, desc=路径点航向
 * @field name=PthPtOdo, type=Array<Real, [100]>, unit=m, desc=路径点里程步长
 * @version 1.0
 * @date 2026-04-19
 * @author aide
 */
typedef struct {
  Integer Success;
  Integer VehicleInSlot;
  Integer ExistStopperInSlot;
  Integer SurroundingType;
  Integer AppliedReqType;
  Real PathLen;
  Integer PtCount;
  Real PthPtX[APA_PARALLEL_REPLAN_PATH_POINT_MAX];
  Real PthPtY[APA_PARALLEL_REPLAN_PATH_POINT_MAX];
  Real PthPtTht[APA_PARALLEL_REPLAN_PATH_POINT_MAX];
  Real PthPtOdo[APA_PARALLEL_REPLAN_PATH_POINT_MAX];
} ApaParallelReplanInSlotOutput_t;

void apaParallelReplanInSlot(const ApaParallelReplanInSlotInput_t *input,
                             const ApaParallelReplanInSlotParam_t *param,
                             ApaParallelReplanInSlotOutput_t *output);

#endif
