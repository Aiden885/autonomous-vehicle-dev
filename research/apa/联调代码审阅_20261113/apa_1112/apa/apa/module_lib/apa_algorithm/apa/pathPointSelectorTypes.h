#ifndef PATH_POINT_SELECTOR_TYPES_H
#define PATH_POINT_SELECTOR_TYPES_H

#include "baseTypes.h"

#ifndef PATH_PT_MAX
#define PATH_PT_MAX 100
#endif

/** @brief 点里程零阈值 */
#define PATH_POINT_SELECTOR_ODO_EPSILON 1e-4f

/**
 * @brief 路径参考点选择输入
 * @en_name PathPointSelectorInput
 * @cn_name 路径参考点选择输入
 * @tag STRUCT
 * @field name=VehicleX, type=Real, unit=m, desc=当前车辆X位置
 * @field name=VehicleY, type=Real, unit=m, desc=当前车辆Y位置
 * @field name=VehicleYaw, type=Real, unit=rad, desc=当前车辆航向角
 * @field name=PtCount, type=Integer, unit=1, desc=有效离散点数量(范围0到100，越界按无路径处理)
 * @field name=PthPtX, type=Array<Real, [100]>, unit=m, desc=路径点X数组
 * @field name=PthPtY, type=Array<Real, [100]>, unit=m, desc=路径点Y数组
 * @field name=PthPtTht, type=Array<Real, [100]>, unit=rad, desc=路径点航向数组
 * @field name=PthPtOdo, type=Array<Real, [100]>, unit=m, desc=路径点里程数组
 * @field name=SegSearchStart, type=Integer, unit=1, desc=段搜索起始索引(由refHandoff提供,-1表示无约束)
 * @field name=SegSearchEnd, type=Integer, unit=1, desc=段搜索结束索引(由refHandoff提供,-1表示无约束)
 * @version 2.0
 * @date 2026-03-14
 * @author aide
 */
typedef struct {
  Real VehicleX;
  Real VehicleY;
  Real VehicleYaw;
  Integer PtCount;
  Real PthPtX[PATH_PT_MAX];
  Real PthPtY[PATH_PT_MAX];
  Real PthPtTht[PATH_PT_MAX];
  Real PthPtOdo[PATH_PT_MAX];
  Integer SegSearchStart;
  Integer SegSearchEnd;
} PathPointSelectorInput_t;

/**
 * @brief 路径参考点选择参数
 * @en_name PathPointSelectorParam
 * @cn_name 路径参考点选择参数
 * @tag STRUCT
 * @field name=LookAheadSteps, type=Integer, unit=1, desc=参考点前视步数
 * @field name=ForwardRefSpeed, type=Real, unit=m/s, desc=前进段参考速度
 * @field name=ReverseRefSpeed, type=Real, unit=m/s, desc=倒车段参考速度
 * @field name=StopRefSpeed, type=Real, unit=m/s, desc=末端或无路径时参考速度
 * @version 1.0
 * @date 2026-03-06
 * @author aide
 */
typedef struct {
  Integer LookAheadSteps;
  Real ForwardRefSpeed;
  Real ReverseRefSpeed;
  Real StopRefSpeed;
} PathPointSelectorParam_t;

/**
 * @brief 路径参考点选择状态
 * @en_name PathPointSelectorState
 * @cn_name 路径参考点选择状态
 * @tag STRUCT
 * @field name=LastIndex, type=Integer, unit=1, desc=上一周期最近点索引
 * @version 2.0
 * @date 2026-03-10
 * @author aide
 */
typedef struct {
  Integer LastIndex;
} PathPointSelectorState_t;

/**
 * @brief 路径参考点选择输出
 * @en_name PathPointSelectorOutput
 * @cn_name 路径参考点选择输出
 * @tag STRUCT
 * @field name=RefX, type=Real, unit=m, desc=参考点X位置
 * @field name=RefY, type=Real, unit=m, desc=参考点Y位置
 * @field name=RefYaw, type=Real, unit=rad, desc=参考点航向角
 * @field name=RefSpeed, type=Real, unit=m/s, desc=参考目标速度
 * @field name=RefIndex, type=Integer, unit=1, desc=当前输出参考点索引
 * @version 1.0
 * @date 2026-03-06
 * @author aide
 */
typedef struct {
  Real RefX;
  Real RefY;
  Real RefYaw;
  Real RefSpeed;
  Integer RefIndex;
} PathPointSelectorOutput_t;

void pathPointSelector(const PathPointSelectorInput_t *input,
                       const PathPointSelectorParam_t *param,
                       PathPointSelectorState_t *state,
                       PathPointSelectorOutput_t *output);

#endif
