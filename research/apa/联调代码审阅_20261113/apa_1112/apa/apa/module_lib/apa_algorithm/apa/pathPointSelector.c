#include "apaCountUtils.h"
#include "apaMathUtils.h"
#include "pathPointSelectorTypes.h"

#ifndef PATH_POINT_SELECTOR_SEARCH_AHEAD_MAX
#define PATH_POINT_SELECTOR_SEARCH_AHEAD_MAX 24
#endif

#ifndef PATH_POINT_SELECTOR_SEARCH_BACK_MAX
#define PATH_POINT_SELECTOR_SEARCH_BACK_MAX 2
#endif

#ifndef PATH_POINT_SELECTOR_MAX_PROGRESS_STEP
#define PATH_POINT_SELECTOR_MAX_PROGRESS_STEP 2
#endif

#ifndef PATH_POINT_SELECTOR_REACQUIRE_DIST_SQ
#define PATH_POINT_SELECTOR_REACQUIRE_DIST_SQ 4.0f
#endif

#ifndef PATH_POINT_SELECTOR_REACQUIRE_BACK_MAX
#define PATH_POINT_SELECTOR_REACQUIRE_BACK_MAX 8
#endif

#ifndef PATH_POINT_SELECTOR_ODO_EPSILON
#define PATH_POINT_SELECTOR_ODO_EPSILON 1.0e-4f
#endif

static void pathPointSelectorSearchWindow(const PathPointSelectorInput_t *input,
                                          Integer searchStartIndex,
                                          Integer searchLimit,
                                          Integer stableStopIndex,
                                          Integer lastIndex,
                                          Integer *bestIndex,
                                          Real *bestDistSq,
                                          Integer *stableIndex,
                                          Real *stableDistSq) {
  Integer searchIndex0;
  Real distSq0;

  *bestIndex = lastIndex;
  *bestDistSq = apaMathDistanceSquared2D(
      input->PthPtX[lastIndex], input->PthPtY[lastIndex], input->VehicleX, input->VehicleY);
  *stableIndex = lastIndex;
  *stableDistSq = *bestDistSq;

  searchIndex0 = searchStartIndex;
  while (searchIndex0 <= searchLimit) {
    Boolean condStableIndex0;

    distSq0 = apaMathDistanceSquared2D(
        input->PthPtX[searchIndex0], input->PthPtY[searchIndex0], input->VehicleX, input->VehicleY);
    if (distSq0 < *bestDistSq) {
      *bestDistSq = distSq0;
      *bestIndex = searchIndex0;
    }
    condStableIndex0 = (searchIndex0 <= stableStopIndex);
    if (condStableIndex0) {
      Boolean condStableDist0;

      condStableDist0 = (distSq0 < *stableDistSq);
      if (condStableDist0) {
        *stableDistSq = distSq0;
        *stableIndex = searchIndex0;
      }
    }
    searchIndex0 = searchIndex0 + 1;
  }
}

static void pathPointSelectorReacquire(const PathPointSelectorInput_t *input,
                                       Integer reacquireStartIndex,
                                       Integer searchLimit,
                                       Integer *bestIndex,
                                       Real *bestDistSq) {
  Integer searchIndex0;
  Real distSq0;

  searchIndex0 = reacquireStartIndex;
  while (searchIndex0 <= searchLimit) {
    distSq0 = apaMathDistanceSquared2D(
        input->PthPtX[searchIndex0], input->PthPtY[searchIndex0], input->VehicleX, input->VehicleY);
    if (distSq0 < *bestDistSq) {
      *bestDistSq = distSq0;
      *bestIndex = searchIndex0;
    }
    searchIndex0 = searchIndex0 + 1;
  }
}

static Integer pathPointSelectorFindBestIndex(const PathPointSelectorInput_t *input,
                                              Integer lastIndex,
                                              Integer maxIndex,
                                              Integer segStart,
                                              Integer segEnd) {
  Integer searchStartIndex0;
  Integer searchStartIndex1;
  Integer searchLimitRaw0;
  Integer searchLimit0;
  Integer stableStopIndexRaw0;
  Integer stableStopIndex0;
  Integer stableIndex0;
  Integer bestIndex0;
  Integer bestIndex1;
  Integer bestIndex2;
  Integer reacquireBestIndex0;
  Integer reacquireStartIndexRaw0;
  Integer reacquireStartIndex0;
  Integer floorBound0;
  Integer ceilBound0;
  Integer lastIndexBounded0;
  Real bestDistSq0;
  Real bestDistSq1;
  Real bestDistSq2;
  Real reacquireBestDistSq0;
  Real stableDistSq0;
  Boolean condSearchStartLow0;
  Boolean condSearchLimitHigh0;
  Boolean condStableStopHigh0;
  Boolean condPreferStableBest0;
  Boolean condPreferStableDist0;
  Boolean condNeedReacquire0;
  Boolean condReacquireStartLow0;
  Boolean condHasSegBound0;

  condHasSegBound0 = (segStart >= 0) && (segEnd >= segStart);
  if (condHasSegBound0) {
    floorBound0 = segStart;
    ceilBound0 = segEnd;
  } else {
    floorBound0 = 0;
    ceilBound0 = maxIndex;
  }

  if (ceilBound0 > maxIndex) {
    ceilBound0 = maxIndex;
  }
  if (floorBound0 > maxIndex) {
    floorBound0 = maxIndex;
  }
  if (floorBound0 > ceilBound0) {
    floorBound0 = ceilBound0;
  }

  lastIndexBounded0 = apaClampCount(lastIndex, maxIndex);
  if (lastIndexBounded0 < floorBound0) {
    lastIndexBounded0 = floorBound0;
  }
  if (lastIndexBounded0 > ceilBound0) {
    lastIndexBounded0 = ceilBound0;
  }

  searchStartIndex0 = lastIndexBounded0 - PATH_POINT_SELECTOR_SEARCH_BACK_MAX;
  condSearchStartLow0 = (searchStartIndex0 < floorBound0);
  if (condSearchStartLow0) {
    searchStartIndex1 = floorBound0;
  } else {
    searchStartIndex1 = searchStartIndex0;
  }

  searchLimitRaw0 = lastIndexBounded0 + PATH_POINT_SELECTOR_SEARCH_AHEAD_MAX;
  condSearchLimitHigh0 = (searchLimitRaw0 > ceilBound0);
  if (condSearchLimitHigh0) {
    searchLimit0 = ceilBound0;
  } else {
    searchLimit0 = searchLimitRaw0;
  }

  stableStopIndexRaw0 = lastIndexBounded0 + PATH_POINT_SELECTOR_MAX_PROGRESS_STEP;
  condStableStopHigh0 = (stableStopIndexRaw0 > searchLimit0);
  if (condStableStopHigh0) {
    stableStopIndex0 = searchLimit0;
  } else {
    stableStopIndex0 = stableStopIndexRaw0;
  }

  pathPointSelectorSearchWindow(input,
                                searchStartIndex1,
                                searchLimit0,
                                stableStopIndex0,
                                lastIndexBounded0,
                                &bestIndex0,
                                &bestDistSq0,
                                &stableIndex0,
                                &stableDistSq0);

  condPreferStableBest0 = (bestIndex0 > stableStopIndex0);
  if (condPreferStableBest0) {
    condPreferStableDist0 = (stableDistSq0 <= PATH_POINT_SELECTOR_REACQUIRE_DIST_SQ);
    if (condPreferStableDist0) {
      bestIndex1 = stableIndex0;
      bestDistSq1 = stableDistSq0;
    } else {
      bestIndex1 = bestIndex0;
      bestDistSq1 = bestDistSq0;
    }
  } else {
    bestIndex1 = bestIndex0;
    bestDistSq1 = bestDistSq0;
  }

  condNeedReacquire0 = (bestDistSq1 > PATH_POINT_SELECTOR_REACQUIRE_DIST_SQ);
  if (condNeedReacquire0) {
    reacquireStartIndexRaw0 = lastIndexBounded0 - PATH_POINT_SELECTOR_REACQUIRE_BACK_MAX;
    condReacquireStartLow0 = (reacquireStartIndexRaw0 < floorBound0);
    if (condReacquireStartLow0) {
      reacquireStartIndex0 = floorBound0;
    } else {
      reacquireStartIndex0 = reacquireStartIndexRaw0;
    }
    reacquireBestIndex0 = bestIndex1;
    reacquireBestDistSq0 = bestDistSq1;

    pathPointSelectorReacquire(
        input, reacquireStartIndex0, searchLimit0, &reacquireBestIndex0, &reacquireBestDistSq0);
    bestIndex2 = reacquireBestIndex0;
    bestDistSq2 = reacquireBestDistSq0;
  } else {
    bestIndex2 = bestIndex1;
    bestDistSq2 = bestDistSq1;
  }

  (void)bestDistSq2;
  return bestIndex2;
}

static Integer pathPointSelectorResolveRefIndex(const PathPointSelectorParam_t *param,
                                                Integer bestIndex,
                                                Integer maxIndex,
                                                Integer segEnd) {
  Integer lookAhead0;
  Integer lookAhead1;
  Integer refIndexRaw0;
  Integer refIndex0;
  Integer refLimit0;
  Boolean condLookAheadLow0;
  Boolean condRefIndexHigh0;

  refLimit0 = maxIndex;
  if ((segEnd >= 0) && (segEnd < refLimit0)) {
    refLimit0 = segEnd;
  }

  lookAhead0 = param->LookAheadSteps;
  condLookAheadLow0 = (lookAhead0 < 0);
  if (condLookAheadLow0) {
    lookAhead1 = 0;
  } else {
    lookAhead1 = lookAhead0;
  }

  refIndexRaw0 = bestIndex + lookAhead1;
  condRefIndexHigh0 = (refIndexRaw0 > refLimit0);
  if (condRefIndexHigh0) {
    refIndex0 = refLimit0;
  } else {
    refIndex0 = refIndexRaw0;
  }

  return refIndex0;
}

static Real pathPointSelectorResolveRefSpeed(const PathPointSelectorInput_t *input,
                                             const PathPointSelectorParam_t *param,
                                             Integer refIndex,
                                             Integer maxIndex) {
  Integer refOdoIndex0;
  Integer refOdoIndex1;
  Real refSpeed0;
  Real refOdo0;
  Boolean condHasForwardRef0;
  Boolean condNeedShiftBack0;
  Boolean condForwardRef0;

  condHasForwardRef0 = (refIndex < maxIndex);
  if (condHasForwardRef0) {
    refOdoIndex0 = refIndex;
    condNeedShiftBack0 = (refOdoIndex0 > 0);
    if (condNeedShiftBack0) {
      refOdoIndex1 = refOdoIndex0 - 1;
    } else {
      refOdoIndex1 = refOdoIndex0;
    }
    refOdo0 = input->PthPtOdo[refOdoIndex1];
    condForwardRef0 = (refOdo0 > 0.0f);
    if (condForwardRef0) {
      refSpeed0 = param->ForwardRefSpeed;
    } else {
      refSpeed0 = param->ReverseRefSpeed;
    }
  } else {
    refSpeed0 = param->StopRefSpeed;
  }

  return refSpeed0;
}

static void pathPointSelectorWriteNoPath(const PathPointSelectorInput_t *input,
                                         const PathPointSelectorParam_t *param,
                                         PathPointSelectorState_t *state,
                                         PathPointSelectorOutput_t *output) {
  state->LastIndex = 0;
  output->RefX = input->VehicleX;
  output->RefY = input->VehicleY;
  output->RefYaw = input->VehicleYaw;
  output->RefSpeed = param->StopRefSpeed;
  output->RefIndex = 0;
}

/**
 * @brief
 * 路径参考点选择模块，基于当前车辆位置在离散轨迹上搜索最近点，并按前视步数输出给控制器使用的参考位姿与参考速度
 * @en_name pathPointSelector
 * @cn_name 路径参考点选择
 * @type widget_module
 * @config cn_name=前视步数, en_name=LookAheadSteps,
 * desc=最近点基础上的前视偏移索引, unit=1, type=Integer, range=[0,20],
 * default=3
 * @config cn_name=前进参考速度, en_name=ForwardRefSpeed,
 * desc=路径点里程为正时输出的参考速度, unit=m/s, type=Real, range=[0,5],
 * default=1.2
 * @config cn_name=倒车参考速度, en_name=ReverseRefSpeed,
 * desc=路径点里程为负时输出的参考速度, unit=m/s, type=Real, range=[-5,0],
 * default=-0.8
 * @config cn_name=停止参考速度, en_name=StopRefSpeed,
 * desc=无路径或到达末端时输出的参考速度, unit=m/s, type=Real, range=[-1,1],
 * default=0.0
 * @param[IN] Real VehicleX 当前车辆X位置
 * @param[IN] Real VehicleY 当前车辆Y位置
 * @param[IN] Real VehicleYaw 当前车辆航向角
 * @param[IN] Integer PtCount 有效离散点数量
 * @param[IN] Real[100] PthPtX 路径点X数组
 * @param[IN] Real[100] PthPtY 路径点Y数组
 * @param[IN] Real[100] PthPtTht 路径点航向数组
 * @param[IN] Real[100] PthPtOdo 路径点里程数组
 * @param[IN] Integer SegSearchStart 段搜索起始索引(由refHandoff提供,-1=无约束)
 * @param[IN] Integer SegSearchEnd 段搜索结束索引(由refHandoff提供,-1=无约束)
 * @param[OUT] Real RefX 参考点X位置
 * @param[OUT] Real RefY 参考点Y位置
 * @param[OUT] Real RefYaw 参考点航向角
 * @param[OUT] Real RefSpeed 参考目标速度
 * @param[OUT] Integer RefIndex 当前输出参考点索引
 * @var Integer LastIndex 上一周期最近点索引
 * @granularity atomic
 * @tag_level1 泊车控制
 * @tag_level2 参考轨迹
 * @source demo_new phase-1 Native reference selector
 * @version 2.0
 * @date 2026-03-10
 * @author aide
 */
void pathPointSelector(const PathPointSelectorInput_t *input,
                       const PathPointSelectorParam_t *param,
                       PathPointSelectorState_t *state,
                       PathPointSelectorOutput_t *output) {
  Integer ptCount0;
  Boolean condHasPath0;

  ptCount0 = input->PtCount;
  condHasPath0 = (ptCount0 > 0) && (ptCount0 <= PATH_PT_MAX);

  if (condHasPath0) {
    Integer maxIndex0;
    Integer lastIndex0;
    Integer bestIndex0;
    Integer refIndex0;

    maxIndex0 = ptCount0 - 1;
    lastIndex0 = apaClampCount(state->LastIndex, maxIndex0);
    bestIndex0 = pathPointSelectorFindBestIndex(
        input, lastIndex0, maxIndex0, input->SegSearchStart, input->SegSearchEnd);
    state->LastIndex = bestIndex0;
    refIndex0 = pathPointSelectorResolveRefIndex(param, bestIndex0, maxIndex0, input->SegSearchEnd);
    output->RefX = input->PthPtX[refIndex0];
    output->RefY = input->PthPtY[refIndex0];
    output->RefYaw = input->PthPtTht[refIndex0];
    output->RefSpeed = pathPointSelectorResolveRefSpeed(input, param, refIndex0, maxIndex0);
    output->RefIndex = refIndex0;
  } else {
    pathPointSelectorWriteNoPath(input, param, state, output);
  }
}
