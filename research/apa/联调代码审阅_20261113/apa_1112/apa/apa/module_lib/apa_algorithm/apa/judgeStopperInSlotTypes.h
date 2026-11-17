#ifndef JUDGE_STOPPER_IN_SLOT_TYPES_H
#define JUDGE_STOPPER_IN_SLOT_TYPES_H

#include "baseTypes.h"
#include "vehicleCollisionCheckTypes.h"

/**
 * @brief 车位内挡停点判断输入
 * @en_name JudgeStopperInSlotInput
 * @cn_name 车位内挡停点判断输入
 * @tag STRUCT
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SeparateStopperNum, type=Integer, unit=1, desc=离散挡停点数量
 * @field name=SeparateStopperX, type=Array<Real, [128]>, unit=m, desc=离散挡停点X
 * @field name=SeparateStopperY, type=Array<Real, [128]>, unit=m, desc=离散挡停点Y
 * @field name=SlotTrackStopperNum, type=Integer, unit=1, desc=车位跟踪挡停点数量
 * @field name=SlotTrackStopperX1, type=Array<Real, [64]>, unit=m, desc=跟踪挡停点端点1X
 * @field name=SlotTrackStopperY1, type=Array<Real, [64]>, unit=m, desc=跟踪挡停点端点1Y
 * @field name=SlotTrackStopperX2, type=Array<Real, [64]>, unit=m, desc=跟踪挡停点端点2X
 * @field name=SlotTrackStopperY2, type=Array<Real, [64]>, unit=m, desc=跟踪挡停点端点2Y
 * @version 1.0
 * @date 2026-04-19
 * @author aide
 */
typedef struct {
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer SeparateStopperNum;
  Real SeparateStopperX[VCC_MAX_SEPARATE_PTS];
  Real SeparateStopperY[VCC_MAX_SEPARATE_PTS];
  Integer SlotTrackStopperNum;
  Real SlotTrackStopperX1[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperY1[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperX2[VCC_MAX_TRACK_STOPPER];
  Real SlotTrackStopperY2[VCC_MAX_TRACK_STOPPER];
} JudgeStopperInSlotInput_t;

/**
 * @brief 车位内挡停点判断输出
 * @en_name JudgeStopperInSlotOutput
 * @cn_name 车位内挡停点判断输出
 * @tag STRUCT
 * @field name=ExistStopperInSlot, type=Integer, unit=1, desc=车位内是否存在挡停点
 * @version 1.0
 * @date 2026-04-19
 * @author aide
 */
typedef struct {
  Integer ExistStopperInSlot;
} JudgeStopperInSlotOutput_t;

void judgeStopperInSlot(const JudgeStopperInSlotInput_t *input, JudgeStopperInSlotOutput_t *output);

#endif
