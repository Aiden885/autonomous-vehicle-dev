#include "geoPthDiscretVhTypes.h"
#include "geoLnDirectDisctetTypes.h"
#include "geoPthDiscretCommon.h"
#include "geoTrnDirectDisctetTypes.h"
#include <math.h>

#ifndef GEO_PTH_DISCRET_VH_DEFAULT_SEGMENTS
#define GEO_PTH_DISCRET_VH_DEFAULT_SEGMENTS 80.0f
#endif
#ifndef GEO_PTH_DISCRET_VH_MIN_STEP
#define GEO_PTH_DISCRET_VH_MIN_STEP 0.12f
#endif

static void geoPthDiscretVhInitWork(const GeoPthDiscretVhInput_t *input,
                                    GeoPthDiscretCommonWork_t *work) {
  Integer index0;
  Real pathStep0;
  Real startDx0;
  Real startDy0;
  Real startDistanceSquared0;
  Boolean useSegmentStart0;

  *work = (GeoPthDiscretCommonWork_t){0};
  pathStep0 = input->PathLen / GEO_PTH_DISCRET_VH_DEFAULT_SEGMENTS;
  if (pathStep0 > GEO_PTH_DISCRET_VH_MIN_STEP) {
    work->StepLen = pathStep0;
  } else {
    work->StepLen = GEO_PTH_DISCRET_VH_MIN_STEP;
  }
  startDx0 = input->VehPosX - input->SegStX[0];
  startDy0 = input->VehPosY - input->SegStY[0];
  startDistanceSquared0 = startDx0 * startDx0 + startDy0 * startDy0;
  useSegmentStart0 = (startDistanceSquared0 < 0.04f);
  if (useSegmentStart0) {
    work->LastX = input->SegStX[0];
    work->LastY = input->SegStY[0];
    work->LastTheta = input->SegStTh[0];
  } else {
    work->LastX = input->VehPosX;
    work->LastY = input->VehPosY;
    work->LastTheta = input->VehPosTheta;
  }
  work->PtCount = 1;
  work->StepCount = 1;
  index0 = 0;
  while (index0 < PATH_PT_MAX) {
    work->X[index0] = 0.0f;
    work->Y[index0] = 0.0f;
    work->Theta[index0] = 0.0f;
    work->Odo[index0] = 0.0f;
    index0 = index0 + 1;
  }
  work->X[0] = work->LastX;
  work->Y[0] = work->LastY;
  work->Theta[0] = work->LastTheta;
}

static void geoPthDiscretVhCopyPathToLineInput(const GeoPthDiscretVhInput_t *input,
                                               const GeoPthDiscretCommonWork_t *work,
                                               Integer segmentIndex,
                                               GeoLnDirectDisctetInput_t *lineInput) {
  Integer index0;
  Integer endIndex0;

  *lineInput = (GeoLnDirectDisctetInput_t){0};
  endIndex0 = segmentIndex + 1;
  lineInput->LineLen = fabsf(input->SegLen[segmentIndex]);
  lineInput->GoDir = input->SegGoDir[segmentIndex];
  lineInput->StepLen = work->StepLen;
  lineInput->EdPtX = endIndex0 < input->SegNum ? input->SegStX[endIndex0] : input->TarPtX;
  lineInput->EdPtY = endIndex0 < input->SegNum ? input->SegStY[endIndex0] : input->TarPtY;
  lineInput->EdPtTheta = endIndex0 < input->SegNum ? input->SegStTh[endIndex0] : input->TarPtTheta;
  lineInput->LstPtX = work->LastX;
  lineInput->LstPtY = work->LastY;
  lineInput->LstPtTheta = work->LastTheta;
  lineInput->VehPosX = input->VehPosX;
  lineInput->VehPosY = input->VehPosY;
  lineInput->VehPosTheta = input->VehPosTheta;
  lineInput->SltTyp = 0;
  lineInput->InPtCount = work->PtCount;
  index0 = 0;
  while (index0 < PATH_PT_MAX) {
    lineInput->InPtX[index0] = work->X[index0];
    lineInput->InPtY[index0] = work->Y[index0];
    lineInput->InPtTht[index0] = work->Theta[index0];
    lineInput->InPtOdo[index0] = work->Odo[index0];
    index0 = index0 + 1;
  }
}

static void geoPthDiscretVhCopyPathToTurnInput(const GeoPthDiscretVhInput_t *input,
                                               const GeoPthDiscretCommonWork_t *work,
                                               Integer segmentIndex,
                                               GeoTrnDirectDisctetInput_t *turnInput) {
  Integer index0;
  Integer endIndex0;

  *turnInput = (GeoTrnDirectDisctetInput_t){0};
  endIndex0 = segmentIndex + 1;
  turnInput->Crv = input->SegCrv[segmentIndex];
  turnInput->CcX = input->SegCcX[segmentIndex];
  turnInput->CcY = input->SegCcY[segmentIndex];
  turnInput->ArcLen = fabsf(input->SegLen[segmentIndex]);
  turnInput->GoDir = input->SegGoDir[segmentIndex];
  turnInput->StepLen = work->StepLen;
  turnInput->ArcAngle = input->SegTht[segmentIndex];
  turnInput->StPtX = input->SegStX[segmentIndex];
  turnInput->StPtY = input->SegStY[segmentIndex];
  turnInput->StPtTheta = input->SegStTh[segmentIndex];
  turnInput->EdPtX = endIndex0 < input->SegNum ? input->SegStX[endIndex0] : input->TarPtX;
  turnInput->EdPtY = endIndex0 < input->SegNum ? input->SegStY[endIndex0] : input->TarPtY;
  turnInput->EdPtTheta = endIndex0 < input->SegNum ? input->SegStTh[endIndex0] : input->TarPtTheta;
  turnInput->LstPtX = work->LastX;
  turnInput->LstPtY = work->LastY;
  turnInput->LstPtTheta = work->LastTheta;
  turnInput->VehPosX = input->VehPosX;
  turnInput->VehPosY = input->VehPosY;
  turnInput->VehPosTheta = input->VehPosTheta;
  turnInput->SltTyp = 0;
  turnInput->InPtCount = work->PtCount;
  index0 = 0;
  while (index0 < PATH_PT_MAX) {
    turnInput->InPtX[index0] = work->X[index0];
    turnInput->InPtY[index0] = work->Y[index0];
    turnInput->InPtTht[index0] = work->Theta[index0];
    turnInput->InPtOdo[index0] = work->Odo[index0];
    index0 = index0 + 1;
  }
}

static void geoPthDiscretVhRunSegment(const GeoPthDiscretVhInput_t *input,
                                      Integer segmentIndex,
                                      GeoPthDiscretCommonWork_t *work) {
  if (input->SegTrnDir[segmentIndex] != 0) {
    GeoTrnDirectDisctetInput_t turnInput0;
    GeoTrnDirectDisctetOutput_t turnOutput0;

    geoPthDiscretVhCopyPathToTurnInput(input, work, segmentIndex, &turnInput0);
    turnOutput0 = (GeoTrnDirectDisctetOutput_t){0};
    geoTrnDirectDisctet(&turnInput0, &turnOutput0);
    geoPthDiscretCommonCommitTurn(&turnOutput0, work);
  } else {
    GeoLnDirectDisctetInput_t lineInput0;
    GeoLnDirectDisctetOutput_t lineOutput0;

    geoPthDiscretVhCopyPathToLineInput(input, work, segmentIndex, &lineInput0);
    lineOutput0 = (GeoLnDirectDisctetOutput_t){0};
    geoLnDirectDisctet(&lineInput0, &lineOutput0);
    geoPthDiscretCommonCommitLine(&lineOutput0, work);
  }
}

static void geoPthDiscretVhRunSegments(const GeoPthDiscretVhInput_t *input,
                                       GeoPthDiscretCommonWork_t *work) {
  Integer segmentIndex0;
  Integer processedSegmentCount0;

  processedSegmentCount0 = input->SegNum;
  segmentIndex0 = 0;
  while (segmentIndex0 < processedSegmentCount0) {
    Real currentOdo0;
    Integer currentDirection0;

    currentOdo0 = work->Odo[work->PtCount - 1];
    currentDirection0 = input->SegGoDir[segmentIndex0];
    if ((Real)currentDirection0 * currentOdo0 < 0.0f) {
      work->StepCount = work->StepCount + 1;
    }
    geoPthDiscretVhRunSegment(input, segmentIndex0, work);
    segmentIndex0 = segmentIndex0 + 1;
  }
}

static void geoPthDiscretVhWriteOutput(const GeoPthDiscretCommonWork_t *work,
                                       GeoPthDiscretVhOutput_t *output) {
  Integer index0;

  output->PtCount = work->PtCount;
  output->StepCount = work->StepCount;
  index0 = 0;
  while (index0 < PATH_PT_MAX) {
    output->PthPtX[index0] = work->X[index0];
    output->PthPtY[index0] = work->Y[index0];
    output->PthPtTht[index0] = work->Theta[index0];
    output->PthPtOdo[index0] = work->Odo[index0];
    index0 = index0 + 1;
  }
}

static void geoPthDiscretVhRun(const GeoPthDiscretVhInput_t *input,
                               GeoPthDiscretVhOutput_t *output) {
  Boolean segmentCountSupported0;

  *output = (GeoPthDiscretVhOutput_t){0};
  segmentCountSupported0 = (input->SegNum >= 0);
  if (segmentCountSupported0) {
    segmentCountSupported0 = (input->SegNum <= GEO_PTH_DISCRET_VH_SEG_MAX);
  }
  if (segmentCountSupported0) {
    output->StepCount = 1;
    if (input->SegNum > 0) {
      GeoPthDiscretCommonWork_t work0;

      geoPthDiscretVhInitWork(input, &work0);
      geoPthDiscretVhRunSegments(input, &work0);

      geoPthDiscretVhWriteOutput(&work0, output);
    }
  }
}

/**
 * @brief 将VH路径段发布为控制轨迹点列和换向元数据
 * @en_name geoPthDiscretVh
 * @cn_name VH路径离散化
 * @type module
 * @param[IN] GeoPthDiscretVhInput_t input VH路径和车辆位姿
 * @param[OUT] GeoPthDiscretVhOutput_t output 完整路径点列和元数据
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 VH垂直头入
 * @source matlab_rewrite geo_PthDiscret_Vh.m
 * @version 1.2
 * @date 2026-07-31
 * @author aide
 */
void geoPthDiscretVh(const GeoPthDiscretVhInput_t *input, GeoPthDiscretVhOutput_t *output) {
  geoPthDiscretVhRun(input, output);
}
