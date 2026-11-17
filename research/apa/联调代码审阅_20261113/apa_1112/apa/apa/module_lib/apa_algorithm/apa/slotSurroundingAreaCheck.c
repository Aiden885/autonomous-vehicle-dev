#include "slotSurroundingAreaCheckTypes.h"

#include "geoSurroundingCheckTypes.h"
#include "judgeStopperInSlotTypes.h"

static void slotSurroundingAreaCheckCopyCommonInput(const SlotSurroundingAreaCheckInput_t *input,
                                                    GeoSurroundingCheckInput_t *geoInput) {
  Integer index;

  *geoInput = (GeoSurroundingCheckInput_t){0};
  geoInput->TarPtX = input->TarPtX;
  geoInput->TarPtY = input->TarPtY;
  geoInput->TarPtTheta = input->TarPtTheta;
  geoInput->SltPt1X = input->SltPt1X;
  geoInput->SltPt1Y = input->SltPt1Y;
  geoInput->SltPt2X = input->SltPt2X;
  geoInput->SltPt2Y = input->SltPt2Y;
  geoInput->SltPt3X = input->SltPt3X;
  geoInput->SltPt3Y = input->SltPt3Y;
  geoInput->SltPt4X = input->SltPt4X;
  geoInput->SltPt4Y = input->SltPt4Y;
  geoInput->SltEntr = input->SltEntr;
  geoInput->BoundaryLineNum = input->BoundaryLineNum;
  geoInput->BoundarySepPtNum = input->BoundarySepPtNum;
  geoInput->VirtualBoundaryLineNum = input->VirtualBoundaryLineNum;

  index = 0;
  while (index < input->BoundaryLineNum && index < SURD_LINE_MAX) {
    geoInput->BoundaryP1X[index] = input->BoundaryP1X[index];
    geoInput->BoundaryP1Y[index] = input->BoundaryP1Y[index];
    geoInput->BoundaryP2X[index] = input->BoundaryP2X[index];
    geoInput->BoundaryP2Y[index] = input->BoundaryP2Y[index];
    index = index + 1;
  }

  index = 0;
  while (index < input->BoundarySepPtNum && index < SURD_PT_MAX) {
    geoInput->BoundarySepPtX[index] = input->BoundarySepPtX[index];
    geoInput->BoundarySepPtY[index] = input->BoundarySepPtY[index];
    index = index + 1;
  }

  index = 0;
  while (index < input->VirtualBoundaryLineNum && index < VCC_MAX_VIRTUAL_LINES) {
    geoInput->VirtualBoundaryLineX1[index] = input->VirtualBoundaryLineX1[index];
    geoInput->VirtualBoundaryLineY1[index] = input->VirtualBoundaryLineY1[index];
    geoInput->VirtualBoundaryLineX2[index] = input->VirtualBoundaryLineX2[index];
    geoInput->VirtualBoundaryLineY2[index] = input->VirtualBoundaryLineY2[index];
    index = index + 1;
  }
}

static void slotSurroundingAreaCheckFlattenStoppers(const SlotSurroundingAreaCheckInput_t *input,
                                                    GeoSurroundingCheckInput_t *geoInput) {
  Integer outIndex;
  Integer stopperIndex;

  outIndex = 0;
  stopperIndex = 0;
  while (stopperIndex < input->SeparateStopperNum && stopperIndex < VCC_MAX_SEPARATE_PTS &&
         outIndex < SURD_PT_MAX) {
    geoInput->StopperPtX[outIndex] = input->SeparateStopperX[stopperIndex];
    geoInput->StopperPtY[outIndex] = input->SeparateStopperY[stopperIndex];
    outIndex = outIndex + 1;
    stopperIndex = stopperIndex + 1;
  }

  geoInput->StopperPtNum = outIndex;
  geoInput->SlotTrackStopperNum = input->SlotTrackStopperNum;
  stopperIndex = 0;
  while (stopperIndex < input->SlotTrackStopperNum && stopperIndex < VCC_MAX_TRACK_STOPPER) {
    geoInput->SlotTrackStopperX1[stopperIndex] = input->SlotTrackStopperX1[stopperIndex];
    geoInput->SlotTrackStopperY1[stopperIndex] = input->SlotTrackStopperY1[stopperIndex];
    geoInput->SlotTrackStopperX2[stopperIndex] = input->SlotTrackStopperX2[stopperIndex];
    geoInput->SlotTrackStopperY2[stopperIndex] = input->SlotTrackStopperY2[stopperIndex];
    stopperIndex = stopperIndex + 1;
  }
}

static void slotSurroundingAreaCheckBuildJudgeInput(const SlotSurroundingAreaCheckInput_t *input,
                                                    JudgeStopperInSlotInput_t *judgeInput) {
  Integer stopperIndex;

  *judgeInput = (JudgeStopperInSlotInput_t){0};
  judgeInput->SltPt1X = input->SltPt1X;
  judgeInput->SltPt1Y = input->SltPt1Y;
  judgeInput->SltPt2X = input->SltPt2X;
  judgeInput->SltPt2Y = input->SltPt2Y;
  judgeInput->SltPt3X = input->SltPt3X;
  judgeInput->SltPt3Y = input->SltPt3Y;
  judgeInput->SltPt4X = input->SltPt4X;
  judgeInput->SltPt4Y = input->SltPt4Y;
  judgeInput->SeparateStopperNum = input->SeparateStopperNum;
  judgeInput->SlotTrackStopperNum = input->SlotTrackStopperNum;

  stopperIndex = 0;
  while (stopperIndex < input->SeparateStopperNum && stopperIndex < VCC_MAX_SEPARATE_PTS) {
    judgeInput->SeparateStopperX[stopperIndex] = input->SeparateStopperX[stopperIndex];
    judgeInput->SeparateStopperY[stopperIndex] = input->SeparateStopperY[stopperIndex];
    stopperIndex = stopperIndex + 1;
  }

  stopperIndex = 0;
  while (stopperIndex < input->SlotTrackStopperNum && stopperIndex < VCC_MAX_TRACK_STOPPER) {
    judgeInput->SlotTrackStopperX1[stopperIndex] = input->SlotTrackStopperX1[stopperIndex];
    judgeInput->SlotTrackStopperY1[stopperIndex] = input->SlotTrackStopperY1[stopperIndex];
    judgeInput->SlotTrackStopperX2[stopperIndex] = input->SlotTrackStopperX2[stopperIndex];
    judgeInput->SlotTrackStopperY2[stopperIndex] = input->SlotTrackStopperY2[stopperIndex];
    stopperIndex = stopperIndex + 1;
  }
}

/**
 * @brief 车位周围区域检查
 * @en_name slotSurroundingAreaCheck
 * @cn_name 车位周围区域检查
 * @type module
 * @param[IN] SlotSurroundingAreaCheckInput_t Input 车位周围区域输入
 * @param[OUT] SlotSurroundingAreaCheckOutput_t Output 车位周围区域输出
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 车位环境
 * @source matlab_rewrite slot_surrounding_area_check.m
 * @version 1.0
 * @date 2026-04-19
 * @author aide
 */
void slotSurroundingAreaCheck(const SlotSurroundingAreaCheckInput_t *input,
                              SlotSurroundingAreaCheckOutput_t *output) {
  GeoSurroundingCheckInput_t geoInput;
  GeoSurroundingCheckOutput_t geoOutput;
  JudgeStopperInSlotInput_t judgeInput;
  JudgeStopperInSlotOutput_t judgeOutput;

  *output = (SlotSurroundingAreaCheckOutput_t){0};

  slotSurroundingAreaCheckBuildJudgeInput(input, &judgeInput);
  judgeOutput = (JudgeStopperInSlotOutput_t){0};
  judgeStopperInSlot(&judgeInput, &judgeOutput);
  output->ExistStopperInSlot = judgeOutput.ExistStopperInSlot;

  slotSurroundingAreaCheckCopyCommonInput(input, &geoInput);
  slotSurroundingAreaCheckFlattenStoppers(input, &geoInput);

  geoOutput = (GeoSurroundingCheckOutput_t){0};
  geoSurroundingCheck(&geoInput, &geoOutput);
  output->SurroundingType = geoOutput.SurroundingType;
}
