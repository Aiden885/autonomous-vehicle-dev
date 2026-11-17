#ifndef APA_PARKING_OUT_TARGET_CONNECTOR_TYPES_H
#define APA_PARKING_OUT_TARGET_CONNECTOR_TYPES_H

#include "apaParkingPathPlanningTypes.h"

/**
 * @brief 泊出道路目标连接输入
 * @en_name ApaParkingOutTargetConnectorInput
 * @cn_name 泊出目标连接输入
 * @tag STRUCT
 * @field name=PlannerInput, type=ApaParkingPathPlanningInput_t, unit=1, desc=统一规划输入
 * @field name=PlannerParam, type=ApaParkingPathPlanningParam_t, unit=1, desc=统一规划参数
 * @field name=LocalPath, type=ApaParkingPathPlanningOutput_t, unit=1, desc=底层泊出规划器生成的局部路径
 * @version 1.0
 * @date 2026-07-29
 * @author aide
 */
typedef struct {
  ApaParkingPathPlanningInput_t PlannerInput;
  ApaParkingPathPlanningParam_t PlannerParam;
  ApaParkingPathPlanningOutput_t LocalPath;
} ApaParkingOutTargetConnectorInput_t;

/**
 * @brief 泊出道路目标连接输出
 * @en_name ApaParkingOutTargetConnectorOutput
 * @cn_name 泊出目标连接输出
 * @tag STRUCT
 * @field name=Connected, type=Boolean, unit=1, desc=局部路径是否成功连接到道路目标
 * @field name=ConnectedPath, type=ApaParkingPathPlanningOutput_t, unit=1, desc=连接后的完整泊出路径
 * @version 1.0
 * @date 2026-07-29
 * @author aide
 */
typedef struct {
  Boolean Connected;
  ApaParkingPathPlanningOutput_t ConnectedPath;
} ApaParkingOutTargetConnectorOutput_t;

#endif
