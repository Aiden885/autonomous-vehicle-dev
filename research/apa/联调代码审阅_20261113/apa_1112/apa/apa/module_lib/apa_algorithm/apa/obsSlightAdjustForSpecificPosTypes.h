#ifndef OBS_SLIGHT_ADJUST_FOR_SPECIFIC_POS_TYPES_H
#define OBS_SLIGHT_ADJUST_FOR_SPECIFIC_POS_TYPES_H

#include "baseTypes.h"
#include "vehicleCollisionCheckTypes.h"

/** @brief Vehicle area summary: front, rear, left, right, center. */
#ifndef OBS_SLIGHT_VEH_AREA_COUNT
#define OBS_SLIGHT_VEH_AREA_COUNT 5
#endif

/**
 * @brief Obstacle slight adjustment input.
 * @en_name ObsSlightAdjustForSpecificPosInput
 * @cn_name 特定位姿障碍物微调输入
 * @tag STRUCT
 * @field name=CheckType, type=Integer, unit=1, desc=collision check type
 * @field name=PosX, type=Real, unit=m, desc=vehicle/reference pose X
 * @field name=PosY, type=Real, unit=m, desc=vehicle/reference pose Y
 * @field name=PosTheta, type=Real, unit=rad, desc=vehicle/reference pose heading
 * @field name=SlotPtsValid, type=Boolean, unit=1, desc=explicit slot points valid
 * @field name=SlotPts, type=Array<Real, [8]>, unit=m, desc=slot corner points
 * @field name=SlotClass, type=Integer, unit=1, desc=slot class
 * @field name=ConsiderCollisionDepthFlag, type=Boolean, unit=1, desc=reject deep collision
 * @field name=ParkingOutFlag, type=Boolean, unit=1, desc=parking-out collision mode
 * @field name=StObs, type=VehicleCollisionCheckInput_t, unit=1, desc=source obstacle context
 * @version 1.0
 * @date 2026-05-06
 * @author aide
 */
typedef struct {
  Integer CheckType;
  Real PosX;
  Real PosY;
  Real PosTheta;
  Boolean SlotPtsValid;
  Real SlotPts[8];
  Integer SlotClass;
  Boolean ConsiderCollisionDepthFlag;
  Boolean ParkingOutFlag;
  VehicleCollisionCheckInput_t StObs;
} ObsSlightAdjustForSpecificPosInput_t;

/**
 * @brief Obstacle slight adjustment output.
 * @en_name ObsSlightAdjustForSpecificPosOutput
 * @cn_name 特定位姿障碍物微调输出
 * @tag STRUCT
 * @field name=ObsValid, type=Boolean, unit=1, desc=adjustment success flag
 * @field name=StObs, type=VehicleCollisionCheckInput_t, unit=1, desc=adjusted obstacle context
 * @field name=VehCollisionArea, type=Array<Boolean, [5]>, unit=1, desc=front/rear/left/right/center flags
 * @field name=LineProcessCount, type=Integer, unit=1, desc=boundary line loop count
 * @field name=PointProcessCount, type=Integer, unit=1, desc=boundary point loop count
 * @version 1.0
 * @date 2026-05-06
 * @author aide
 */
typedef struct {
  Boolean ObsValid;
  VehicleCollisionCheckInput_t StObs;
  Boolean VehCollisionArea[OBS_SLIGHT_VEH_AREA_COUNT];
  Integer LineProcessCount;
  Integer PointProcessCount;
} ObsSlightAdjustForSpecificPosOutput_t;

void obsSlightAdjustForSpecificPos(const ObsSlightAdjustForSpecificPosInput_t *input,
                                   ObsSlightAdjustForSpecificPosOutput_t *output);

#endif
