#ifndef CHECK_PARKING_TYPE_TYPES_H
#define CHECK_PARKING_TYPE_TYPES_H

#include "baseTypes.h"

/**
 * @brief 泊车方向推理输入
 * @en_name CheckParkingTypeInput
 * @cn_name 泊车方向推理输入
 * @tag STRUCT
 * @field name=VehX, type=Real, unit=m, desc=车辆X
 * @field name=VehY, type=Real, unit=m, desc=车辆Y
 * @field name=VehTheta, type=Real, unit=rad, desc=车辆航向
 * @field name=TarX, type=Real, unit=m, desc=目标X
 * @field name=TarY, type=Real, unit=m, desc=目标Y
 * @field name=TarTheta, type=Real, unit=rad, desc=目标航向
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=EnterEdge, type=Integer, unit=1, desc=入口边编号(1-4)
 * @version 1.0
 * @date 2026-04-12
 * @author aide
 */
typedef struct {
  Real VehX;
  Real VehY;
  Real VehTheta;
  Real TarX;
  Real TarY;
  Real TarTheta;
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer EnterEdge;
} CheckParkingTypeInput_t;

/**
 * @brief 泊车方向推理输出
 * @en_name CheckParkingTypeOutput
 * @cn_name 泊车方向推理输出
 * @tag STRUCT
 * @field name=ParkingType, type=Integer, unit=1, desc=泊车方向(0=无意义 1=左 2=右 3=未知)
 * @field name=SlotClass, type=Integer, unit=1, desc=车位类型(0=垂直 1=平行)
 * @version 1.0
 * @date 2026-04-12
 * @author aide
 */
typedef struct {
  Integer ParkingType;
  Integer SlotClass;
} CheckParkingTypeOutput_t;

void checkParkingType(const CheckParkingTypeInput_t *input, CheckParkingTypeOutput_t *output);

#endif
