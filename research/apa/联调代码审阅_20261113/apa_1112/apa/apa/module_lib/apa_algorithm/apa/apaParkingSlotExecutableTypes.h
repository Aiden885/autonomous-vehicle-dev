#ifndef APA_PARKING_SLOT_EXECUTABLE_TYPES_H
#define APA_PARKING_SLOT_EXECUTABLE_TYPES_H
#include "apaParkingSlotPathPlanningTypes.h"
#include "apaParkingPathExecutabilityTypes.h"
/**
 * @brief 可执行车位规划输入。
 * @en_name ApaParkingPathPlanningFromSlotExecutableInput
 * @cn_name 可执行车位规划输入
 * @tag STRUCT
 * @field name=Slot, type=ApaParkingPathPlanningFromSlotInput_t, unit=1, desc=原车位请求
 */
typedef struct {
  ApaParkingPathPlanningFromSlotInput_t Slot;
} ApaParkingPathPlanningFromSlotExecutableInput_t;
/**
 * @brief 车位规划及执行参数。
 * @en_name ApaParkingPathPlanningFromSlotExecutableParam
 * @cn_name 可执行车位规划参数
 * @tag STRUCT
 * @field name=Slot, type=ApaParkingPathPlanningFromSlotParam_t, unit=1, desc=实际车型与原目标策略
 * @field name=Execution, type=ApaParkingPathExecutabilityParam_t, unit=1, desc=误差包络与共享闭环标定
 */
typedef struct {
  ApaParkingPathPlanningFromSlotParam_t Slot;
  ApaParkingPathExecutabilityParam_t Execution;
} ApaParkingPathPlanningFromSlotExecutableParam_t;

/**
 * @brief Full old payload, execution evidence and actual planning envelope.
 * @en_name ApaParkingPathPlanningFromSlotExecutableOutput
 * @cn_name 可执行车位规划输出
 * @tag STRUCT
 * @field name=Slot, type=ApaParkingPathPlanningFromSlotOutput_t, unit=1, desc=原路径与目标诊断
 * @field name=Execution, type=ApaParkingPathExecutabilityOutput_t, unit=1, desc=选中或最后候选的完整执行判定
 * @field name=Envelope, type=ApaParkingPathPlanningParam_t, unit=1, desc=实际用于生成的规划包络
 * @field name=ExecutionChecks, type=Integer, unit=1, desc=执行验证候选数
 * @field name=ExecutionRejects, type=Integer, unit=1, desc=闭环拒绝候选数
 */
typedef struct {
  ApaParkingPathPlanningFromSlotOutput_t Slot;
  ApaParkingPathExecutabilityOutput_t Execution;
  ApaParkingPathPlanningParam_t Envelope;
  Integer ExecutionChecks;
  Integer ExecutionRejects;
} ApaParkingPathPlanningFromSlotExecutableOutput_t;
#endif
