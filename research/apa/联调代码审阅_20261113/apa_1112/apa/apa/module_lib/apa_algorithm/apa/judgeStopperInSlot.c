#include "judgeStopperInSlotTypes.h"

#include "apaMathUtils.h"

static Boolean judgeStopperInSlotCheckPoint(const Real slotPoints[8], Real pointX, Real pointY) {
  return apaMathIsPtInRect(pointX, pointY, slotPoints);
}

/**
 * @brief 车位内挡停点判断
 * @en_name judgeStopperInSlot
 * @cn_name 车位内挡停点判断
 * @type module
 * @param[IN] JudgeStopperInSlotInput_t Input 车位挡停点输入
 * @param[OUT] JudgeStopperInSlotOutput_t Output 车位挡停点输出
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 车位环境
 * @source matlab_rewrite judge_stopper_in_slot.m
 * @version 1.0
 * @date 2026-04-19
 * @author aide
 */
void judgeStopperInSlot(const JudgeStopperInSlotInput_t *input,
                        JudgeStopperInSlotOutput_t *output) {
  Real slotPoints[8];
  Integer stopperIndex;

  *output = (JudgeStopperInSlotOutput_t){0};
  slotPoints[0] = input->SltPt1X;
  slotPoints[1] = input->SltPt1Y;
  slotPoints[2] = input->SltPt2X;
  slotPoints[3] = input->SltPt2Y;
  slotPoints[4] = input->SltPt3X;
  slotPoints[5] = input->SltPt3Y;
  slotPoints[6] = input->SltPt4X;
  slotPoints[7] = input->SltPt4Y;

  stopperIndex = 0;
  while ((stopperIndex < input->SeparateStopperNum) && (stopperIndex < VCC_MAX_SEPARATE_PTS) &&
         (output->ExistStopperInSlot == 0)) {
    if (judgeStopperInSlotCheckPoint(slotPoints,
                                     input->SeparateStopperX[stopperIndex],
                                     input->SeparateStopperY[stopperIndex])) {
      output->ExistStopperInSlot = 1;
    }
    stopperIndex = stopperIndex + 1;
  }

  stopperIndex = 0;
  while ((stopperIndex < input->SlotTrackStopperNum) && (stopperIndex < VCC_MAX_TRACK_STOPPER) &&
         (output->ExistStopperInSlot == 0)) {
    if (judgeStopperInSlotCheckPoint(slotPoints,
                                     input->SlotTrackStopperX1[stopperIndex],
                                     input->SlotTrackStopperY1[stopperIndex])) {
      output->ExistStopperInSlot = 1;
    }
    if ((output->ExistStopperInSlot == 0) &&
        judgeStopperInSlotCheckPoint(slotPoints,
                                     input->SlotTrackStopperX2[stopperIndex],
                                     input->SlotTrackStopperY2[stopperIndex])) {
      output->ExistStopperInSlot = 1;
    }
    stopperIndex = stopperIndex + 1;
  }
}
