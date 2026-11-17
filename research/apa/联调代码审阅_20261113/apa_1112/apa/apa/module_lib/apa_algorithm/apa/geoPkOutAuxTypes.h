/**
 * @file geoPkOutAuxTypes.h
 * @brief 泊出几何附属算法输入输出类型定义
 * @date 2026-04-13
 */
#ifndef GEO_PK_OUT_AUX_TYPES_H
#define GEO_PK_OUT_AUX_TYPES_H

#include "baseTypes.h"
#include "apaPlannerConfig.h"
#include "vehicleCollisionCheckTypes.h"

/**
 * @brief 泊出槽位长度计算输入
 * @en_name GeoPkOutSltLengthInput
 * @cn_name 泊出槽位长度输入
 * @tag STRUCT
 * @field name=VehPosX, type=Real, unit=m, desc=车辆X
 * @field name=VehPosY, type=Real, unit=m, desc=车辆Y
 * @field name=VehPosTheta, type=Real, unit=rad, desc=车辆航向
 * @field name=SlotNum, type=Integer, unit=1, desc=车位数量
 * @field name=SlotPtsX, type=Array<Real, [10, 4]>, unit=m, desc=车位角点X矩阵(最多10个车位,每个4角)
 * @field name=SlotPtsY, type=Array<Real, [10, 4]>, unit=m, desc=车位角点Y矩阵
 * @field name=PkOutDir, type=Integer, unit=1, desc=泊出方向
 * @field name=InSlot, type=Boolean, unit=1, desc=是否在车位内
 */
typedef struct {
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Integer SlotNum;
  Real SlotPtsX[10][4];
  Real SlotPtsY[10][4];
  Integer PkOutDir;
  Boolean InSlot;
} GeoPkOutSltLengthInput_t;

/**
 * @brief 泊出槽位长度计算输出
 * @en_name GeoPkOutSltLengthOutput
 * @cn_name 泊出槽位长度输出
 * @tag STRUCT
 * @field name=PkOutSltLength, type=Real, unit=m, desc=有效泊出预留长度
 */
typedef struct {
  Real PkOutSltLength;
} GeoPkOutSltLengthOutput_t;

void geoPkOutSltLength(const GeoPkOutSltLengthInput_t *input, GeoPkOutSltLengthOutput_t *output);

#ifndef MAX_LRAS_PTS
#define MAX_LRAS_PTS 500
#endif

#ifndef MAX_STPBP_PTS
#define MAX_STPBP_PTS 32
#endif

/**
 * @brief 泊出方向容许度输入
 * @en_name GeoPrkOutDirAllowInput
 * @cn_name 泊出方向容许度输入
 * @tag STRUCT
 * @field name=LrasPtN, type=Integer, unit=1, desc=LRAS点数
 * @field name=LrasPtsX, type=Array<Real, [500]>, unit=m, desc=LRAS点X数组
 * @field name=LrasPtsY, type=Array<Real, [500]>, unit=m, desc=LRAS点Y数组
 * @field name=LrasValid, type=Boolean, unit=1, desc=LRAS有效标志
 * @field name=LrasVehPosX, type=Real, unit=m, desc=LRAS参考车辆X
 * @field name=LrasVehPosY, type=Real, unit=m, desc=LRAS参考车辆Y
 * @field name=LrasVehPosTheta, type=Real, unit=rad, desc=LRAS参考车辆航向
 * @field name=StpbpN, type=Integer, unit=1, desc=障碍物点数
 * @field name=StpbpX, type=Array<Real, [32]>, unit=m, desc=障碍物点X
 * @field name=StpbpY, type=Array<Real, [32]>, unit=m, desc=障碍物点Y
 * @field name=VehPosX, type=Real, unit=m, desc=当前车辆X
 * @field name=VehPosY, type=Real, unit=m, desc=当前车辆Y
 * @field name=VehPosTheta, type=Real, unit=rad, desc=当前车辆航向
 * @field name=Para, type=Array<Real, [30]>, unit=1, desc=规划参数
 * @field name=LrasCfSw, type=Boolean, unit=1, desc=LRAS验证开关
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=边界线段数量
 * @field name=BoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=边界线起点X
 * @field name=BoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=边界线起点Y
 * @field name=BoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=边界线终点X
 * @field name=BoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=边界线终点Y
 * @field name=BoundarySepPtsNum, type=Integer, unit=1, desc=边界离散点数
 * @field name=BoundarySepPtsX, type=Array<Real, [128]>, unit=m, desc=边界离散点X
 * @field name=BoundarySepPtsY, type=Array<Real, [128]>, unit=m, desc=边界离散点Y
 */
typedef struct {
  Integer LrasPtN;
  Real LrasPtsX[MAX_LRAS_PTS];
  Real LrasPtsY[MAX_LRAS_PTS];
  Boolean LrasValid;
  Real LrasVehPosX;
  Real LrasVehPosY;
  Real LrasVehPosTheta;
  Integer StpbpN;
  Real StpbpX[MAX_STPBP_PTS];
  Real StpbpY[MAX_STPBP_PTS];
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Real Para[30];
  Boolean LrasCfSw;
  Integer BoundaryLineNum;
  Real BoundaryLineX1[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineY1[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineX2[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineY2[VCC_MAX_BOUNDARY_LINES];
  Integer BoundarySepPtsNum;
  Real BoundarySepPtsX[VCC_MAX_SEPARATE_PTS];
  Real BoundarySepPtsY[VCC_MAX_SEPARATE_PTS];
} GeoPrkOutDirAllowInput_t;

/**
 * @brief 泊出方向容许度输出
 * @en_name GeoPrkOutDirAllowOutput
 * @cn_name 泊出方向容许度输出
 * @tag STRUCT
 * @field name=PrkOutDir, type=Array<Boolean, [4]>, unit=1, desc=允许方向(1:左,2:右,3:前,4:后)的布尔标识数组
 */
typedef struct {
  Boolean PrkOutDir[4];
} GeoPrkOutDirAllowOutput_t;

void geoPrkOutDirAllow(const GeoPrkOutDirAllowInput_t *input, GeoPrkOutDirAllowOutput_t *output);

/**
 * @brief 泊出危险点计算输入
 * @en_name GeoPkOutDangerPtInput
 * @cn_name 泊出危险点计算输入
 * @tag STRUCT
 * @field name=VehX, type=Real, unit=m, desc=车辆起步X
 * @field name=VehY, type=Real, unit=m, desc=车辆起步Y
 * @field name=VehTheta, type=Real, unit=rad, desc=车辆起步航向
 * @field name=TgtX, type=Real, unit=m, desc=目标X
 * @field name=TgtY, type=Real, unit=m, desc=目标Y
 * @field name=TgtTheta, type=Real, unit=rad, desc=目标航向
 * @field name=Pt1sX, type=Real, unit=m, desc=前探定点X
 * @field name=Pt1sY, type=Real, unit=m, desc=前探定点Y
 * @field name=Pt1sTheta, type=Real, unit=rad, desc=前探定点航向
 * @field name=Rds, type=Real, unit=m, desc=回转半径
 * @field name=Dir, type=Integer, unit=1, desc=行驶方向
 * @field name=BoundaryLineNum, type=Integer, unit=1, desc=边界线数量
 * @field name=BoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=边界起点X
 * @field name=BoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=边界起点Y
 * @field name=BoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=边界终点X
 * @field name=BoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=边界终点Y
 * @field name=BoundarySepPtsNum, type=Integer, unit=1, desc=离散障碍点数
 * @field name=BoundarySepPtsX, type=Array<Real, [128]>, unit=m, desc=离散障碍点X
 * @field name=BoundarySepPtsY, type=Array<Real, [128]>, unit=m, desc=离散障碍点Y
 * @field name=SeparateStopperNum, type=Integer, unit=1, desc=独立轮挡点数
 * @field name=SeparateStopperX, type=Array<Real, [128]>, unit=m, desc=独立轮挡点X
 * @field name=SeparateStopperY, type=Array<Real, [128]>, unit=m, desc=独立轮挡点Y
 * @field name=VirtualBoundaryLineNum, type=Integer, unit=1, desc=虚拟边界数量
 * @field name=VirtualBoundaryLineX1, type=Array<Real, [64]>, unit=m, desc=虚拟边界起点X
 * @field name=VirtualBoundaryLineY1, type=Array<Real, [64]>, unit=m, desc=虚拟边界起点Y
 * @field name=VirtualBoundaryLineX2, type=Array<Real, [64]>, unit=m, desc=虚拟边界终点X
 * @field name=VirtualBoundaryLineY2, type=Array<Real, [64]>, unit=m, desc=虚拟边界终点Y
 */
typedef struct {
  Real VehX;
  Real VehY;
  Real VehTheta;
  Real TgtX;
  Real TgtY;
  Real TgtTheta;
  Real Pt1sX;
  Real Pt1sY;
  Real Pt1sTheta;
  Real Rds;
  Integer Dir;
  Integer BoundaryLineNum;
  Real BoundaryLineX1[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineY1[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineX2[VCC_MAX_BOUNDARY_LINES];
  Real BoundaryLineY2[VCC_MAX_BOUNDARY_LINES];
  Integer BoundarySepPtsNum;
  Real BoundarySepPtsX[VCC_MAX_SEPARATE_PTS];
  Real BoundarySepPtsY[VCC_MAX_SEPARATE_PTS];
  Integer SeparateStopperNum;
  Real SeparateStopperX[VCC_MAX_SEPARATE_PTS];
  Real SeparateStopperY[VCC_MAX_SEPARATE_PTS];
  Integer VirtualBoundaryLineNum;
  Real VirtualBoundaryLineX1[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineY1[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineX2[VCC_MAX_VIRTUAL_LINES];
  Real VirtualBoundaryLineY2[VCC_MAX_VIRTUAL_LINES];
} GeoPkOutDangerPtInput_t;

/**
 * @brief 泊出危险点计算输出
 * @en_name GeoPkOutDangerPtOutput
 * @cn_name 泊出危险点计算输出
 * @tag STRUCT
 * @field name=DangerPtX, type=Real, unit=m, desc=计算所得危险点X
 * @field name=DangerPtY, type=Real, unit=m, desc=计算所得危险点Y
 * @field name=ChCnt, type=Integer, unit=1, desc=检测计数
 * @field name=PtEx, type=Integer, unit=1, desc=0无危险点,1真实边界/点/轮挡,2虚拟边界
 */
typedef struct {
  Real DangerPtX;
  Real DangerPtY;
  Integer ChCnt;
  Integer PtEx;
} GeoPkOutDangerPtOutput_t;

void geoPkOutDangerPt(const GeoPkOutDangerPtInput_t *input, GeoPkOutDangerPtOutput_t *output);

/**
 * @brief 泊出局部区域危险点搜索输入
 * @en_name GeoPkOutDangerAreaSearchInput
 * @cn_name 泊出区域危险点搜索输入
 * @tag STRUCT
 * @field name=PosX, type=Real, unit=m, desc=待搜索位姿X
 * @field name=PosY, type=Real, unit=m, desc=待搜索位姿Y
 * @field name=PosTheta, type=Real, unit=rad, desc=待搜索位姿航向
 * @field name=FrontExtend, type=Real, unit=m, desc=前向搜索扩展距离
 * @field name=RearExtend, type=Real, unit=m, desc=后向搜索扩展距离
 * @field name=LatExtend, type=Real, unit=m, desc=侧向搜索扩展距离
 * @field name=SideSign, type=Integer, unit=1, desc=搜索侧向符号(1左侧,-1右侧)
 * @field name=VehConfig, type=ApaPlannerVehConfig_t, unit=1, desc=车辆几何参数
 * @field name=Obstacles, type=VehicleCollisionCheckInput_t, unit=1, desc=边界/离散点/虚拟边界输入
 */
typedef struct {
  Real PosX;
  Real PosY;
  Real PosTheta;
  Real FrontExtend;
  Real RearExtend;
  Real LatExtend;
  Integer SideSign;
  ApaPlannerVehConfig_t VehConfig;
  VehicleCollisionCheckInput_t Obstacles;
} GeoPkOutDangerAreaSearchInput_t;

/**
 * @brief 泊出局部区域危险点搜索输出
 * @en_name GeoPkOutDangerAreaSearchOutput
 * @cn_name 泊出区域危险点搜索输出
 * @tag STRUCT
 * @field name=DangerPtX, type=Real, unit=m, desc=危险点X
 * @field name=DangerPtY, type=Real, unit=m, desc=危险点Y
 * @field name=CheckCount, type=Integer, unit=1, desc=检查计数
 * @field name=PtEx, type=Integer, unit=1, desc=0无危险点,1真实边界/点,2虚拟边界
 */
typedef struct {
  Real DangerPtX;
  Real DangerPtY;
  Integer CheckCount;
  Integer PtEx;
} GeoPkOutDangerAreaSearchOutput_t;

void geoPkOutDangerAreaSearch(const GeoPkOutDangerAreaSearchInput_t *input,
                              GeoPkOutDangerAreaSearchOutput_t *output);

/**
 * @brief 泊出角度安全空间危险点搜索输入
 * @en_name GeoPkOutDangerAngleSearchInput
 * @cn_name 泊出角度危险点搜索输入
 * @tag STRUCT
 * @field name=VehX, type=Real, unit=m, desc=车辆X
 * @field name=VehY, type=Real, unit=m, desc=车辆Y
 * @field name=VehTheta, type=Real, unit=rad, desc=车辆航向
 * @field name=Dir, type=Integer, unit=1, desc=泊出方向(1左,2右,3前)
 * @field name=CheckStep, type=Real, unit=m, desc=前向检查步长
 * @field name=FrontCheckLen, type=Real, unit=m, desc=前向检查长度
 * @field name=FrontUs, type=Array<Real, [4]>, unit=m, desc=前向四路超声距离
 * @field name=VehConfig, type=ApaPlannerVehConfig_t, unit=1, desc=车辆几何参数
 * @field name=Obstacles, type=VehicleCollisionCheckInput_t, unit=1, desc=边界/离散点/虚拟边界输入
 */
typedef struct {
  Real VehX;
  Real VehY;
  Real VehTheta;
  Integer Dir;
  Real CheckStep;
  Real FrontCheckLen;
  Real FrontUs[4];
  ApaPlannerVehConfig_t VehConfig;
  VehicleCollisionCheckInput_t Obstacles;
} GeoPkOutDangerAngleSearchInput_t;

/**
 * @brief 泊出角度安全空间危险点搜索输出
 * @en_name GeoPkOutDangerAngleSearchOutput
 * @cn_name 泊出角度危险点搜索输出
 * @tag STRUCT
 * @field name=DangerPtX, type=Real, unit=m, desc=危险点X
 * @field name=DangerPtY, type=Real, unit=m, desc=危险点Y
 * @field name=CheckCount, type=Integer, unit=1, desc=检查计数
 * @field name=PtEx, type=Integer, unit=1, desc=0无危险点,1真实边界/碰撞点,2虚拟边界
 * @field name=SafeSpaceFlag, type=Integer, unit=1, desc=是否存在可用安全空间
 * @field name=SelectedPoseX, type=Real, unit=m, desc=选中安全位姿X
 * @field name=SelectedPoseY, type=Real, unit=m, desc=选中安全位姿Y
 * @field name=SelectedPoseTheta, type=Real, unit=rad, desc=选中安全位姿航向
 * @field name=ForwardSafeLen, type=Real, unit=m, desc=前向安全长度
 */
typedef struct {
  Real DangerPtX;
  Real DangerPtY;
  Integer CheckCount;
  Integer PtEx;
  Integer SafeSpaceFlag;
  Real SelectedPoseX;
  Real SelectedPoseY;
  Real SelectedPoseTheta;
  Real ForwardSafeLen;
} GeoPkOutDangerAngleSearchOutput_t;

void geoPkOutDangerAngleSearch(const GeoPkOutDangerAngleSearchInput_t *input,
                               GeoPkOutDangerAngleSearchOutput_t *output);

#endif
