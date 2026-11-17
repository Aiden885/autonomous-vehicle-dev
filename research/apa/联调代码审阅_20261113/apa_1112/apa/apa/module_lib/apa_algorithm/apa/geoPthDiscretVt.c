#include "geoPthDiscretVtTypes.h"
#include "geoLnDirectDisctetTypes.h"
#include "geoPthDiscretCommon.h"
#include "geoTrnDirectDisctetTypes.h"
#include "apaCoordUtils.h"
#include <math.h>

#ifndef GEO_PTH_DISCRET_VT_DEFAULT_SEGMENTS
#define GEO_PTH_DISCRET_VT_DEFAULT_SEGMENTS 80.0f
#endif
#ifndef GEO_PTH_DISCRET_VT_MIN_STEP
#define GEO_PTH_DISCRET_VT_MIN_STEP 0.12f
#endif

static void geoPthDiscretVtInitWork(const GeoPthDiscretVtInput_t *input,
                                    GeoPthDiscretCommonWork_t *work) {
  Integer index0;
  Real pathStep0;
  Real startDx0;
  Real startDy0;
  Real startDistanceSquared0;
  Boolean useSegmentStart0;

  *work = (GeoPthDiscretCommonWork_t){0};
  pathStep0 = input->PathLen / GEO_PTH_DISCRET_VT_DEFAULT_SEGMENTS;
  if (pathStep0 > GEO_PTH_DISCRET_VT_MIN_STEP) {
    work->StepLen = pathStep0;
  } else {
    work->StepLen = GEO_PTH_DISCRET_VT_MIN_STEP;
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

static void geoPthDiscretVtCopyPathToLineInput(const GeoPthDiscretVtInput_t *input,
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
  lineInput->LstStp = (endIndex0 == input->SegNum);
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

static void geoPthDiscretVtCopyPathToTurnInput(const GeoPthDiscretVtInput_t *input,
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
  turnInput->LstStp = (endIndex0 == input->SegNum);
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

static void geoPthDiscretVtRunSegment(const GeoPthDiscretVtInput_t *input,
                                      Integer segmentIndex,
                                      GeoPthDiscretCommonWork_t *work) {
  if (input->SegTrnDir[segmentIndex] != 0) {
    GeoTrnDirectDisctetInput_t turnInput0;
    GeoTrnDirectDisctetOutput_t turnOutput0;

    geoPthDiscretVtCopyPathToTurnInput(input, work, segmentIndex, &turnInput0);
    turnOutput0 = (GeoTrnDirectDisctetOutput_t){0};
    geoTrnDirectDisctet(&turnInput0, &turnOutput0);
    geoPthDiscretCommonCommitTurn(&turnOutput0, work);
  } else {
    GeoLnDirectDisctetInput_t lineInput0;
    GeoLnDirectDisctetOutput_t lineOutput0;

    geoPthDiscretVtCopyPathToLineInput(input, work, segmentIndex, &lineInput0);
    lineOutput0 = (GeoLnDirectDisctetOutput_t){0};
    geoLnDirectDisctet(&lineInput0, &lineOutput0);
    geoPthDiscretCommonCommitLine(&lineOutput0, work);
  }
}

static void geoPthDiscretVtTrimBacktrack(const GeoPthDiscretVtInput_t *input,
                                         Integer segmentIndex,
                                         GeoPthDiscretCommonWork_t *work) {
  Real originX0;
  Real originY0;
  Real originTheta0;
  Real cosOrigin0;
  Real sinOrigin0;
  Real nextDx0;
  Real nextDy0;
  Real nextRelativeX0;
  Boolean trimDone0;

  originX0 = input->SegStX[segmentIndex];
  originY0 = input->SegStY[segmentIndex];
  originTheta0 = input->SegStTh[segmentIndex];
  cosOrigin0 = cosf(originTheta0);
  sinOrigin0 = sinf(originTheta0);
  nextDx0 = input->SegStX[segmentIndex + 1] - originX0;
  nextDy0 = input->SegStY[segmentIndex + 1] - originY0;
  nextRelativeX0 = cosOrigin0 * nextDx0 + sinOrigin0 * nextDy0;
  trimDone0 = false;
  while ((work->PtCount > 0) && (!trimDone0)) {
    Integer lastIndex0;
    Real pointDx0;
    Real pointDy0;
    Real pointRelativeX0;

    lastIndex0 = work->PtCount - 1;
    pointDx0 = work->X[lastIndex0] - originX0;
    pointDy0 = work->Y[lastIndex0] - originY0;
    pointRelativeX0 = cosOrigin0 * pointDx0 + sinOrigin0 * pointDy0;
    if (pointRelativeX0 * (pointRelativeX0 - nextRelativeX0) > 0.0f) {
      trimDone0 = true;
    } else {
      work->X[lastIndex0] = 0.0f;
      work->Y[lastIndex0] = 0.0f;
      work->Theta[lastIndex0] = 0.0f;
      work->Odo[lastIndex0] = 0.0f;
      work->PtCount = work->PtCount - 1;
    }
  }
}

static Boolean geoPthDiscretVtShouldRunSegment(const GeoPthDiscretVtInput_t *input,
                                               Integer segmentIndex,
                                               GeoPthDiscretCommonWork_t *work) {
  Boolean shouldRun0;
  Real currentOdo0;
  Integer currentDirection0;

  shouldRun0 = true;
  currentOdo0 = work->Odo[work->PtCount - 1];
  currentDirection0 = input->SegGoDir[segmentIndex];
  if ((work->PtCount > 1) && ((Real)currentDirection0 * currentOdo0 < 0.0f)) {
    Integer nextDirectionProduct0;
    Real segmentLength0;

    nextDirectionProduct0 = segmentIndex + 1 < input->SegNum
                                ? currentDirection0 * input->SegGoDir[segmentIndex + 1]
                                : 0;
    segmentLength0 = fabsf(input->SegLen[segmentIndex]);
    if ((nextDirectionProduct0 < 0) && (segmentLength0 < 0.1f)) {
      shouldRun0 = false;
    } else if ((nextDirectionProduct0 < 0) && (segmentIndex > 0) &&
               (segmentLength0 < work->StepLen) && (input->SegLen[segmentIndex + 1] >= 2.0f)) {
      geoPthDiscretVtTrimBacktrack(input, segmentIndex, work);
      shouldRun0 = false;
    } else {
      work->StepCount = work->StepCount + 1;
    }
  }
  return shouldRun0;
}

static void geoPthDiscretVtRunSegments(const GeoPthDiscretVtInput_t *input,
                                       GeoPthDiscretCommonWork_t *work) {
  Integer segmentIndex0;
  Integer processedSegmentCount0;

  processedSegmentCount0 = input->SegNum;
  segmentIndex0 = 0;
  while (segmentIndex0 < processedSegmentCount0) {
    Boolean shouldRun0;

    shouldRun0 = geoPthDiscretVtShouldRunSegment(input, segmentIndex0, work);
    if (shouldRun0) {
      geoPthDiscretVtRunSegment(input, segmentIndex0, work);
    }
    if (segmentIndex0 == 0) {
      work->St2Idx = work->PtCount;
    }
    segmentIndex0 = segmentIndex0 + 1;
  }
}

static void geoPthDiscretVtFindForwardPoint(const GeoPthDiscretVtInput_t *input,
                                            const GeoPthDiscretCommonWork_t *work,
                                            Real pointRelative[3]) {
  Integer index0;
  Boolean found0;
  Real pointX0;
  Real pointY0;
  Real pointTheta0;
  Real deltaX0;
  Real deltaY0;
  Real cosVehicle0;
  Real sinVehicle0;

  index0 = 0;
  found0 = false;
  pointX0 = 0.0f;
  pointY0 = 0.0f;
  pointTheta0 = 0.0f;
  while ((index0 < work->PtCount) && (!found0)) {
    if (work->Odo[index0] > 0.01f) {
      pointX0 = work->X[index0];
      pointY0 = work->Y[index0];
      pointTheta0 = work->Theta[index0];
      found0 = true;
    } else {
      index0 = index0 + 1;
    }
  }
  deltaX0 = pointX0 - input->VehPosX;
  deltaY0 = pointY0 - input->VehPosY;
  cosVehicle0 = cosf(input->VehPosTheta);
  sinVehicle0 = sinf(input->VehPosTheta);
  pointRelative[0] = cosVehicle0 * deltaX0 + sinVehicle0 * deltaY0;
  pointRelative[1] = cosVehicle0 * deltaY0 - sinVehicle0 * deltaX0;
  pointRelative[2] = apaCoordWrapToPi(pointTheta0 - input->VehPosTheta);
}

static void geoPthDiscretVtWriteOutput(const GeoPthDiscretCommonWork_t *work,
                                       const Real forwardPointRelative[3],
                                       GeoPthDiscretVtOutput_t *output) {
  Integer index0;

  output->PtCount = work->PtCount;
  output->StepCount = work->StepCount;
  output->St2Idx = work->St2Idx;
  output->Stp1PtRX = forwardPointRelative[0];
  output->Stp1PtRY = forwardPointRelative[1];
  output->Stp1PtRTheta = forwardPointRelative[2];
  index0 = 0;
  while (index0 < PATH_PT_MAX) {
    output->PthPtX[index0] = work->X[index0];
    output->PthPtY[index0] = work->Y[index0];
    output->PthPtTht[index0] = work->Theta[index0];
    output->PthPtOdo[index0] = work->Odo[index0];
    index0 = index0 + 1;
  }
}

static void geoPthDiscretVtRun(const GeoPthDiscretVtInput_t *input,
                               GeoPthDiscretVtOutput_t *output) {
  Boolean segmentCountSupported0;

  *output = (GeoPthDiscretVtOutput_t){0};
  segmentCountSupported0 = (input->SegNum >= 0);
  if (segmentCountSupported0) {
    segmentCountSupported0 = (input->SegNum <= GEO_PTH_DISCRET_VT_SEG_MAX);
  }
  if (segmentCountSupported0) {
    output->StepCount = 1;
    if (input->SegNum > 0) {
      GeoPthDiscretCommonWork_t work0;
      GeoPthDiscretShortcutConfig_t shortcutConfig0;
      Real forwardPointRelative0[3];

      geoPthDiscretVtInitWork(input, &work0);
      geoPthDiscretVtRunSegments(input, &work0);
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
      geoPthDiscretVtFindForwardPoint(input, &work0, forwardPointRelative0);
      geoPthDiscretVtWriteOutput(&work0, forwardPointRelative0, output);
    }
  }
}

/**
 * @brief 将VT路径段发布为控制轨迹点列和换向元数据
 * @en_name geoPthDiscretVt
 * @cn_name VT路径离散化
 * @type module
 * @param[IN] GeoPthDiscretVtInput_t input VT路径和车辆位姿
 * @param[OUT] GeoPthDiscretVtOutput_t output 完整路径点列和元数据
 * @granularity composite
 * @tag_level1 泊车规划
 * @tag_level2 路径离散化
 * @source matlab_rewrite geo_PthDiscret_Vt.m
 * @version 1.3
 * @date 2026-07-31
 * @author aide
 */
void geoPthDiscretVt(const GeoPthDiscretVtInput_t *input, GeoPthDiscretVtOutput_t *output) {
  geoPthDiscretVtRun(input, output);
}
