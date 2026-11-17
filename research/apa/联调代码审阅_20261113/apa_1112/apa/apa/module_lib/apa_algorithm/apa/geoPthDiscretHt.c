#include "geoPthDiscretHtTypes.h"
#include "geoLnDirectDisctetTypes.h"
#include "geoPthDiscretCommon.h"
#include "geoTrnDirectDisctetTypes.h"
#include <math.h>

#ifndef GEO_PTH_DISCRET_HT_DEFAULT_SEGMENTS
#define GEO_PTH_DISCRET_HT_DEFAULT_SEGMENTS 80.0f
#endif
#ifndef GEO_PTH_DISCRET_HT_MIN_STEP
#define GEO_PTH_DISCRET_HT_MIN_STEP 0.15f
#endif
#ifndef GEO_PTH_DISCRET_HT_PK_IN_MIN_STEP
#define GEO_PTH_DISCRET_HT_PK_IN_MIN_STEP 0.12f
#endif

static Real geoPthDiscretHtSumPathLength(const GeoPthDiscretHtInput_t *input) {
  Integer index0;
  Real pathLength0;

  pathLength0 = input->SegLen[0];
  index0 = 1;
  while (index0 < input->SegNum) {
    pathLength0 = pathLength0 + input->SegLen[index0];
    index0 = index0 + 1;
  }
  return pathLength0;
}

static void geoPthDiscretHtInitWork(const GeoPthDiscretHtInput_t *input,
                                    Real minimumStep,
                                    GeoPthDiscretCommonWork_t *work) {
  Integer index0;
  Real pathLength0;
  Real pathStep0;
  Real startDx0;
  Real startDy0;
  Real startDistanceSquared0;
  Boolean useSegmentStart0;

  *work = (GeoPthDiscretCommonWork_t){0};
  pathLength0 = geoPthDiscretHtSumPathLength(input);
  pathStep0 = pathLength0 / GEO_PTH_DISCRET_HT_DEFAULT_SEGMENTS;
  if (pathStep0 > minimumStep) {
    work->StepLen = pathStep0;
  } else {
    work->StepLen = minimumStep;
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

static void geoPthDiscretHtSetEndPoint(const GeoPthDiscretHtInput_t *input,
                                       Integer segmentIndex,
                                       Real *endX,
                                       Real *endY,
                                       Real *endTheta) {
  Integer nextIndex0;

  nextIndex0 = segmentIndex + 1;
  if (nextIndex0 < input->SegNum) {
    *endX = input->SegStX[nextIndex0];
    *endY = input->SegStY[nextIndex0];
    *endTheta = input->SegStTh[nextIndex0];
  } else {
    *endX = input->TarPtX;
    *endY = input->TarPtY;
    *endTheta = input->TarPtTheta;
  }
}

static void geoPthDiscretHtCopyPathToLineInput(const GeoPthDiscretHtInput_t *input,
                                               const GeoPthDiscretCommonWork_t *work,
                                               Integer segmentIndex,
                                               GeoLnDirectDisctetInput_t *lineInput) {
  Integer index0;

  *lineInput = (GeoLnDirectDisctetInput_t){0};
  lineInput->LineLen = fabsf(input->SegLen[segmentIndex]);
  lineInput->GoDir = input->SegGoDir[segmentIndex];
  lineInput->StepLen = work->StepLen;
  geoPthDiscretHtSetEndPoint(
      input, segmentIndex, &lineInput->EdPtX, &lineInput->EdPtY, &lineInput->EdPtTheta);
  lineInput->LstPtX = work->LastX;
  lineInput->LstPtY = work->LastY;
  lineInput->LstPtTheta = work->LastTheta;
  lineInput->VehPosX = input->VehPosX;
  lineInput->VehPosY = input->VehPosY;
  lineInput->VehPosTheta = input->VehPosTheta;
  lineInput->SltTyp = 1;
  lineInput->LstStp = (segmentIndex == (input->SegNum - 1));
  lineInput->TarPtX = input->TarPtX;
  lineInput->TarPtY = input->TarPtY;
  lineInput->TarPtTheta = input->TarPtTheta;
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

static void geoPthDiscretHtCopyPathToTurnInput(const GeoPthDiscretHtInput_t *input,
                                               const GeoPthDiscretCommonWork_t *work,
                                               Integer segmentIndex,
                                               GeoTrnDirectDisctetInput_t *turnInput) {
  Integer index0;
  Real curvature0;

  *turnInput = (GeoTrnDirectDisctetInput_t){0};
  curvature0 = input->SegCrv[segmentIndex];
  turnInput->Crv = curvature0;
  turnInput->CcX = input->SegStX[segmentIndex] - sinf(input->SegStTh[segmentIndex]) / curvature0;
  turnInput->CcY = input->SegStY[segmentIndex] + cosf(input->SegStTh[segmentIndex]) / curvature0;
  turnInput->ArcLen = fabsf(input->SegLen[segmentIndex]);
  turnInput->GoDir = input->SegGoDir[segmentIndex];
  turnInput->StepLen = work->StepLen;
  turnInput->ArcAngle = input->SegTht[segmentIndex];
  turnInput->StPtX = input->SegStX[segmentIndex];
  turnInput->StPtY = input->SegStY[segmentIndex];
  turnInput->StPtTheta = input->SegStTh[segmentIndex];
  geoPthDiscretHtSetEndPoint(
      input, segmentIndex, &turnInput->EdPtX, &turnInput->EdPtY, &turnInput->EdPtTheta);
  turnInput->LstPtX = work->LastX;
  turnInput->LstPtY = work->LastY;
  turnInput->LstPtTheta = work->LastTheta;
  turnInput->VehPosX = input->VehPosX;
  turnInput->VehPosY = input->VehPosY;
  turnInput->VehPosTheta = input->VehPosTheta;
  turnInput->SltTyp = 1;
  turnInput->LstStp = (segmentIndex == (input->SegNum - 1));
  turnInput->TarPtX = input->TarPtX;
  turnInput->TarPtY = input->TarPtY;
  turnInput->TarPtTheta = input->TarPtTheta;
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

static void geoPthDiscretHtRunSegment(const GeoPthDiscretHtInput_t *input,
                                      Integer segmentIndex,
                                      GeoPthDiscretCommonWork_t *work) {
  if (input->SegTrnDir[segmentIndex] != 0) {
    GeoTrnDirectDisctetInput_t turnInput0;
    GeoTrnDirectDisctetOutput_t turnOutput0;

    geoPthDiscretHtCopyPathToTurnInput(input, work, segmentIndex, &turnInput0);
    turnOutput0 = (GeoTrnDirectDisctetOutput_t){0};
    geoTrnDirectDisctet(&turnInput0, &turnOutput0);
    geoPthDiscretCommonCommitTurn(&turnOutput0, work);
  } else {
    GeoLnDirectDisctetInput_t lineInput0;
    GeoLnDirectDisctetOutput_t lineOutput0;

    geoPthDiscretHtCopyPathToLineInput(input, work, segmentIndex, &lineInput0);
    lineOutput0 = (GeoLnDirectDisctetOutput_t){0};
    geoLnDirectDisctet(&lineInput0, &lineOutput0);
    geoPthDiscretCommonCommitLine(&lineOutput0, work);
  }
}

static Boolean geoPthDiscretHtShouldRunSegment(const GeoPthDiscretHtInput_t *input,
                                               Integer segmentIndex,
                                               GeoPthDiscretCommonWork_t *work) {
  Boolean shouldRun0;
  Real currentOdo0;
  Integer currentDirection0;
  Boolean hasNext0;

  shouldRun0 = true;
  currentOdo0 = work->Odo[work->PtCount - 1];
  currentDirection0 = input->SegGoDir[segmentIndex];
  hasNext0 = ((segmentIndex + 1) < input->SegNum);
  if (((Real)currentDirection0 * currentOdo0 < 0.0f) && hasNext0 &&
      ((currentDirection0 * input->SegGoDir[segmentIndex + 1]) < 0) &&
      (fabsf(input->SegLen[segmentIndex]) < 0.1f)) {
    shouldRun0 = false;
  } else if ((Real)currentDirection0 * currentOdo0 < 0.0f) {
    work->StepCount = work->StepCount + 1;
  }
  return shouldRun0;
}

static void geoPthDiscretHtRunSegments(const GeoPthDiscretHtInput_t *input,
                                       GeoPthDiscretCommonWork_t *work) {
  Integer segmentIndex0;

  segmentIndex0 = 0;
  while (segmentIndex0 < input->SegNum) {
    Boolean shouldRun0;

    shouldRun0 = geoPthDiscretHtShouldRunSegment(input, segmentIndex0, work);
    if (shouldRun0) {
      geoPthDiscretHtRunSegment(input, segmentIndex0, work);
      if (segmentIndex0 == 0) {
        work->St2Idx = work->PtCount;
      }
    }
    segmentIndex0 = segmentIndex0 + 1;
  }
}

static void geoPthDiscretHtWriteOutput(const GeoPthDiscretCommonWork_t *work,
                                       GeoPthDiscretHtOutput_t *output) {
  Integer index0;

  output->PtCount = work->PtCount;
  output->StepCount = work->StepCount;
  output->St2Idx = work->St2Idx;
  index0 = 0;
  while (index0 < PATH_PT_MAX) {
    output->PthPtX[index0] = work->X[index0];
    output->PthPtY[index0] = work->Y[index0];
    output->PthPtTht[index0] = work->Theta[index0];
    output->PthPtOdo[index0] = work->Odo[index0];
    index0 = index0 + 1;
  }
}

static void geoPthDiscretHtRun(const GeoPthDiscretHtInput_t *input,
                               Real minimumStep,
                               GeoPthDiscretHtOutput_t *output) {
  Boolean segmentCountSupported0;

  *output = (GeoPthDiscretHtOutput_t){0};
  segmentCountSupported0 = (input->SegNum >= 0);
  if (segmentCountSupported0) {
    segmentCountSupported0 = (input->SegNum <= HT_SEG_OUT_MAX);
  }
  if (segmentCountSupported0) {
    output->StepCount = 1;
    if (input->SegNum > 0) {
      GeoPthDiscretCommonWork_t work0;
      GeoPthDiscretShortcutConfig_t shortcutConfig0;

      geoPthDiscretHtInitWork(input, minimumStep, &work0);
      geoPthDiscretHtRunSegments(input, &work0);
      shortcutConfig0 = (GeoPthDiscretShortcutConfig_t){0};
      shortcutConfig0.ShortCutSw = input->ShortCutSw;
      shortcutConfig0.SegNum = input->SegNum;
      shortcutConfig0.Seg1TrnDir = input->SegTrnDir[0];
      shortcutConfig0.Seg1GoDir = input->SegGoDir[0];
      shortcutConfig0.Seg1Len = input->SegLen[0];
      if (input->SegNum > 1) {
        shortcutConfig0.Seg2TrnDir = input->SegTrnDir[1];
        shortcutConfig0.Seg2GoDir = input->SegGoDir[1];
        shortcutConfig0.Seg2Len = input->SegLen[1];
      }
      shortcutConfig0.VehPosX = input->VehPosX;
      shortcutConfig0.VehPosY = input->VehPosY;
      shortcutConfig0.VehPosTheta = input->VehPosTheta;
      shortcutConfig0.ShortCutThd = input->ShortCutThd;
      geoPthDiscretCommonApplyShortcut(&shortcutConfig0, &work0);
      geoPthDiscretHtWriteOutput(&work0, output);
    }
  }
}

/**
 * @brief 将HT路径段发布为控制轨迹点列和换向元数据
 * @en_name geoPthDiscretHt
 * @cn_name HT路径离散化
 * @type module
 * @param[IN] GeoPthDiscretHtInput_t input HT路径和车辆位姿
 * @param[OUT] GeoPthDiscretHtOutput_t output 完整路径点列和元数据
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 路径离散化
 * @source matlab_rewrite geo_PthDiscret_Ht.m
 * @version 1.3
 * @date 2026-07-31
 * @author aide
 */
void geoPthDiscretHt(const GeoPthDiscretHtInput_t *input, GeoPthDiscretHtOutput_t *output) {
  geoPthDiscretHtRun(input, GEO_PTH_DISCRET_HT_MIN_STEP, output);
}

/**
 * @brief 使用0.12米最小步长发布HT泊入路径
 * @en_name geoPthDiscretHtPkIn
 * @cn_name HT泊入路径离散化
 * @type module
 * @param[IN] GeoPthDiscretHtInput_t input HT泊入路径和车辆位姿
 * @param[OUT] GeoPthDiscretHtOutput_t output 完整路径点列和元数据
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 路径离散化
 * @source matlab_rewrite geo_PthDiscret_HtPkIn.m
 * @version 1.1
 * @date 2026-07-31
 * @author aide
 */
void geoPthDiscretHtPkIn(const GeoPthDiscretHtInput_t *input, GeoPthDiscretHtOutput_t *output) {
  geoPthDiscretHtRun(input, GEO_PTH_DISCRET_HT_PK_IN_MIN_STEP, output);
}
