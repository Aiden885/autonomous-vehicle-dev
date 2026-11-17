#ifndef VEHICLE_COLLISION_CHECK_CORE_H
#define VEHICLE_COLLISION_CHECK_CORE_H

#include "apaPlannerConfig.h"
#include "vehicleCollisionCheckTypes.h"

/**
 * @brief 车辆碰撞结果及几何诊断。
 * @field name=Summary, type=VehicleCollisionCheckOutput_t, unit=1, desc=Public collision result
 * @field name=CollisionPoint, type=Array<Real, [2]>, unit=m, desc=World collision point; [99,99] when unknown
 * @field name=DeepPointArea, type=UInteger8, unit=1, desc=Virtual collision point area; 0 when unknown
 */
typedef struct {
  VehicleCollisionCheckOutput_t Summary;
  Real CollisionPoint[2];
  UInteger8 DeepPointArea;
} VehicleCollisionCheckDetailedOutput_t;

/** @brief Load automatic margins from the caller's vehicle configuration. */
void vehicleCollisionCheckLoadMargins(VehicleCollisionCheckInput_t *input,
                                      const ApaPlannerVehConfig_t *vehicle);

/** @brief 按检查模式计算碰撞结果。 */
void vehicleCollisionCheckDetailed(const VehicleCollisionCheckInput_t *input,
                                   VehicleCollisionCheckDetailedOutput_t *output);

Boolean vehicleCollisionCheckRectContainsPoint(const Real rect[8], Real x, Real y);
Boolean vehicleCollisionCheckLineHitsRect(Real x1, Real y1, Real x2, Real y2, const Real rect[8]);

#endif
