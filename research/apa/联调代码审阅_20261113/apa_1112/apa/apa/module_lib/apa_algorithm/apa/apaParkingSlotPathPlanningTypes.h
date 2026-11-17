#ifndef APA_PARKING_SLOT_PATH_PLANNING_TYPES_H
#define APA_PARKING_SLOT_PATH_PLANNING_TYPES_H

#include "apaParkingPathPlanningTypes.h"
#include "apaParkingSlotTargetResolveTypes.h"

#define APA_SLOT_REQUEST_ANGLE_RAD 0
#define APA_SLOT_REQUEST_ANGLE_DEG 1

/**
 * @brief Explicit slot request; angles have an explicit unit.
 * @en_name ApaParkingPathPlanningFromSlotInput
 * @cn_name 车位目标调度输入
 * @tag STRUCT
 * @field name=PlannerInput, type=ApaParkingPathPlanningInput_t, unit=1, desc=约定车位车辆障碍数据
 * @field name=AngleUnit, type=Integer, unit=1, desc=原请求VehicleYaw及TarTheta单位0弧度1度
 */
typedef struct {
  ApaParkingPathPlanningInput_t PlannerInput;
  Integer AngleUnit;
} ApaParkingPathPlanningFromSlotInput_t;

/**
 * @brief Vehicle/planner calibration and input reference conversion.
 * @en_name ApaParkingPathPlanningFromSlotParam
 * @cn_name 车位目标调度参数
 * @tag STRUCT
 * @field name=PlannerParam, type=ApaParkingPathPlanningParam_t, unit=1, desc=现有规划及实际车型参数
 * @field name=VehicleReferenceOffset, type=Real, unit=m, desc=后轴至输入车辆参考点向前偏移默认0要求已为后轴
 */
typedef struct {
  ApaParkingPathPlanningParam_t PlannerParam;
  Real VehicleReferenceOffset;
} ApaParkingPathPlanningFromSlotParam_t;

/**
 * @brief Path plus the exact resolved input used for planning and safety.
 * @en_name ApaParkingPathPlanningFromSlotOutput
 * @cn_name 车位目标调度输出
 * @tag STRUCT
 * @field name=Path, type=ApaParkingPathPlanningOutput_t, unit=1, desc=完整验收后的原路径输出
 * @field name=ResolvedInput, type=ApaParkingPathPlanningInput_t, unit=1, desc=本次规划验收采用的后轴输入角度均为弧度
 * @field name=Target, type=ApaParkingSlotTargetResolveOutput_t, unit=1, desc=解析几何与输入目标诊断
 */
typedef struct {
  ApaParkingPathPlanningOutput_t Path;
  ApaParkingPathPlanningInput_t ResolvedInput;
  ApaParkingSlotTargetResolveOutput_t Target;
} ApaParkingPathPlanningFromSlotOutput_t;

#endif
