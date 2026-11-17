#include "apaParkingPathPlanning.h"

#include "apaDthEdVtParkingPathPlanning.h"
#include "apaDebugTrace.h"
#include "apaHtParkingPathPlanning.h"
#include "apaParkingOutPathPlanning.h"
#include "apaParkingPathSafetyGate.h"
#include "apaParkingPathPlanningCommon.h"
#include "apaParkingPathPlanningInternal.h"
#include "apaParkingSearchFallback.h"
#include "apaVhParkingPathPlanning.h"
#include "apaVtParkingPathPlanning.h"
#include <stdio.h>

static void apaParkingPathPlanningTracePath(UInteger frameId,
                                            const ApaParkingPathPlanningOutput_t *output) {
  Integer pointCount0;
  Integer lastIndex0;
  Integer pointIndex0;
  Integer lastDirection0;
  Integer direction0;

  pointCount0 = output->PtCount;
  if (pointCount0 > APA_PARKING_PATH_PT_MAX) {
    pointCount0 = APA_PARKING_PATH_PT_MAX;
  }
  if (pointCount0 > 0) {
    lastIndex0 = pointCount0 - 1;
    (void)fprintf(stderr,
                  "[APA_DIAG] frame_id=%lu stage=PATH_POINT role=first "
                  "idx=0 x=%.9g y=%.9g theta=%.9g odo=%.9g\n",
                  (unsigned long)frameId,
                  (double)output->PthPtX[0],
                  (double)output->PthPtY[0],
                  (double)output->PthPtTht[0],
                  (double)output->PthPtOdo[0]);
    (void)fprintf(stderr,
                  "[APA_DIAG] frame_id=%lu stage=PATH_POINT role=last "
                  "idx=%ld x=%.9g y=%.9g theta=%.9g odo=%.9g\n",
                  (unsigned long)frameId,
                  (long)lastIndex0,
                  (double)output->PthPtX[lastIndex0],
                  (double)output->PthPtY[lastIndex0],
                  (double)output->PthPtTht[lastIndex0],
                  (double)output->PthPtOdo[lastIndex0]);

    lastDirection0 = 0;
    pointIndex0 = 0;
    while (pointIndex0 < pointCount0) {
      direction0 = 0;
      if (output->PthPtOdo[pointIndex0] > 1.0e-4f) {
        direction0 = 1;
      } else if (output->PthPtOdo[pointIndex0] < -1.0e-4f) {
        direction0 = -1;
      }
      if ((direction0 != 0) && (direction0 != lastDirection0)) {
        (void)fprintf(stderr,
                      "[APA_DIAG] frame_id=%lu stage=PATH_DIRECTION "
                      "idx=%ld x=%.9g y=%.9g theta=%.9g odo=%.9g dir=%ld\n",
                      (unsigned long)frameId,
                      (long)pointIndex0,
                      (double)output->PthPtX[pointIndex0],
                      (double)output->PthPtY[pointIndex0],
                      (double)output->PthPtTht[pointIndex0],
                      (double)output->PthPtOdo[pointIndex0],
                      (long)direction0);
        lastDirection0 = direction0;
      }
      pointIndex0 = pointIndex0 + 1;
    }
  }
}

/**
 * @brief Plan one APA parking path from vehicle, target-slot, and environment input.
 * @en_name apaParkingPathPlanning
 * @cn_name APA泊车路径规划
 * @type widget_module
 * @config cn_name=车位状态, en_name=SltState, desc=车位状态含6库内重规划, unit=1, type=Integer, range=[0,6], default=1
 * @config cn_name=车宽, en_name=VehW, desc=车宽, unit=m, type=Real, range=[1,3], default=2.16345
 * @config cn_name=后轴到前包络长度, en_name=VehLlF, desc=后轴参考点到前保险杠长度, unit=m, type=Real, range=[0,5], default=3.80932
 * @config cn_name=最小前向安全长度, en_name=VehLfMin, desc=最小前向安全长度, unit=m, type=Real, range=[0,5], default=0.4
 * @config cn_name=后悬长度, en_name=VehLr, desc=后悬到参考点长度, unit=m, type=Real, range=[0,3], default=0.98246
 * @config cn_name=VT回退空间, en_name=VtFallbackSpace, desc=VT边界探测失败回退空间, unit=m, type=Real, range=[0,10], default=4.1
 * @config cn_name=DTH回退空间, en_name=DthFallbackSpace, desc=DTH边界探测失败回退空间, unit=m, type=Real, range=[0,10], default=6.0
 * @config cn_name=HT外侧前角半径, en_name=HtRfoutMin, desc=外侧前角扫掠半径为0按APA_Cal计算, unit=m, type=Real, range=[0,10], default=0
 * @config cn_name=Hybrid融合, en_name=NeedHybridAlgFusion, desc=是否允许HybridA融合回退, unit=1, type=Integer, range=[0,1], default=1
 * @param[IN] Integer ParkingAction 泊车动作1泊入2泊出
 * @param[IN] Integer TargetSlotKind 目标车位类型
 * @param[IN] Real VehicleX 车辆X
 * @param[IN] Real VehicleY 车辆Y
 * @param[IN] Real VehicleYaw 车辆航向
 * @param[IN] Real TarX 目标X
 * @param[IN] Real TarY 目标Y
 * @param[IN] Real TarTheta 目标航向
 * @param[IN] Real Rmin 最小转弯半径
 * @param[IN] Real EntrPtX 入口点X
 * @param[IN] Real EntrPtY 入口点Y
 * @param[IN] Real SltPt1X 车位角点1X
 * @param[IN] Real SltPt1Y 车位角点1Y
 * @param[IN] Real SltPt2X 车位角点2X
 * @param[IN] Real SltPt2Y 车位角点2Y
 * @param[IN] Real SltPt3X 车位角点3X
 * @param[IN] Real SltPt3Y 车位角点3Y
 * @param[IN] Real SltPt4X 车位角点4X
 * @param[IN] Real SltPt4Y 车位角点4Y
 * @param[IN] Integer SltEntr 车位入口边编号
 * @param[IN] Integer PrkType 泊车方向
 * @param[IN] Integer SurdType 周边类型
 * @param[IN] Real AgrOfst 可用空间修正量
 * @param[IN] Integer BoundaryLineNum 边界线数量
 * @param[IN] Real[64] BoundaryLineX1 边界线起点X数组
 * @param[IN] Real[64] BoundaryLineY1 边界线起点Y数组
 * @param[IN] Real[64] BoundaryLineX2 边界线终点X数组
 * @param[IN] Real[64] BoundaryLineY2 边界线终点Y数组
 * @param[IN] Integer VirtualBoundaryLineNum 虚拟边界线数量
 * @param[IN] Real[64] VirtualBoundaryP1X 虚拟边界点1X
 * @param[IN] Real[64] VirtualBoundaryP1Y 虚拟边界点1Y
 * @param[IN] Real[64] VirtualBoundaryP2X 虚拟边界点2X
 * @param[IN] Real[64] VirtualBoundaryP2Y 虚拟边界点2Y
 * @param[IN] Integer SlotTrackStopperNum 车位跟踪挡停线数量
 * @param[IN] Real[64] SlotTrackStopperX1 车位跟踪挡停线端点1X
 * @param[IN] Real[64] SlotTrackStopperY1 车位跟踪挡停线端点1Y
 * @param[IN] Real[64] SlotTrackStopperX2 车位跟踪挡停线端点2X
 * @param[IN] Real[64] SlotTrackStopperY2 车位跟踪挡停线端点2Y
 * @param[IN] Integer BoundarySepPtNum 真实离散点数量
 * @param[IN] Real[128] BoundarySepPtX 真实离散点X
 * @param[IN] Real[128] BoundarySepPtY 真实离散点Y
 * @param[IN] Integer StopperPtNum 挡停点数量
 * @param[IN] Real[128] StopperPtX 挡停点X
 * @param[IN] Real[128] StopperPtY 挡停点Y
 * @param[IN] Integer ShortCutSw 是否启用轨迹shortcut
 * @param[IN] Real ShortCutThd shortcut横向阈值
 * @param[OUT] Integer SelectedSlotKind 已选择车位类型
 * @param[OUT] Integer PlanFlg 规划成功标志
 * @param[OUT] Integer PlannerCase 规划分支编号
 * @param[OUT] Integer SegNum 路径段数
 * @param[OUT] Integer StpN 换向次数
 * @param[OUT] Real PathLen 路径总长
 * @param[OUT] Integer PtCount 离散点数
 * @param[OUT] Integer StepCount 离散步数
 * @param[OUT] Integer St2Idx 第二段起始索引
 * @param[OUT] Real Stp1PtRX 首个前进点车体坐标X
 * @param[OUT] Real Stp1PtRY 首个前进点车体坐标Y
 * @param[OUT] Real Stp1PtRTheta 首个前进点车体航向
 * @param[OUT] Real[100] PthPtX 离散点X数组
 * @param[OUT] Real[100] PthPtY 离散点Y数组
 * @param[OUT] Real[100] PthPtTht 离散点航向数组
 * @param[OUT] Real[100] PthPtOdo 离散点里程数组
 * @granularity composite
 * @tag_level1 规划
 * @tag_level2 泊车路径规划
 * @source demo_new
 * @version 1.0
 * @date 2026-07-28
 * @author aide
 */
void apaParkingPathPlanning(const ApaParkingPathPlanningInput_t *input,
                            const ApaParkingPathPlanningParam_t *param,
                            ApaParkingPathPlanningOutput_t *output) {
  ApaParkingPathSafetyGateInput_t safetyInput;
  ApaParkingPathSafetyGateOutput_t safetyOutput;
  Integer slotKind0;
  Boolean parkinRequested0;
  Boolean parkoutRequested0;
  Boolean parkingActionValid0;
  Boolean slotKindIsVt0;
  Boolean slotKindIsHt0;
  Boolean slotKindIsDth0;
  Boolean slotKindIsVh0;
  Boolean slotKindValid0 = false;
  UInteger frameId0;

  ApaPlanningDiagnostics_t diagnostic = {0};
  ApaParkingPathPlanningParam_t runtimeParam = *param;
  runtimeParam.Diagnostics = &diagnostic;
  ApaParkingHybridTrial_t trial = {input, &runtimeParam};
  if (input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKIN) {
    runtimeParam.HybridPolicy.AcceptCandidate = apaParkingPathPlanningAcceptHybridSegments;
    runtimeParam.HybridPolicy.CompareCandidates = apaParkingPathPlanningCompareHybridSegments;
    runtimeParam.HybridPolicy.UserData = &trial;
    runtimeParam.HybridPolicy.GoalPositionTolerance = input->GoalPositionTolerance;
  }
  param = &runtimeParam;

  const Boolean commonSearch = input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKIN &&
                               param->SceneQuery.CheckPose != 0;
  ApaParkingPathPlanningParam_t geometricParam = *param;
  if (commonSearch)
    geometricParam.NeedHybridAlgFusion = 0;
  frameId0 = apaDebugTraceBeginFrame();
  apaDebugTracePlanningInput(frameId0, input, param);
  apaDebugTraceParkoutInput("APA_INPUT", frameId0, input);
  apaParkingPathPlanningZeroOutput(output);
  parkinRequested0 = (input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKIN);
  parkoutRequested0 = (input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKOUT);
  parkingActionValid0 = parkinRequested0 || parkoutRequested0;
  if (parkingActionValid0) {
    if (parkoutRequested0) {
      apaParkingOutPathPlanning(input, param, output);
    } else {
      slotKind0 = input->TargetSlotKind;
      slotKindIsVt0 = (slotKind0 == APA_SCENARIO_SLOT_KIND_VT);
      slotKindIsHt0 = (slotKind0 == APA_SCENARIO_SLOT_KIND_HT);
      slotKindIsDth0 = (slotKind0 == APA_SCENARIO_SLOT_KIND_DTH_ED_VT);
      slotKindIsVh0 = (slotKind0 == APA_SCENARIO_SLOT_KIND_VH);
      slotKindValid0 = slotKindIsVt0 || slotKindIsHt0 || slotKindIsDth0 || slotKindIsVh0;
      if (slotKindValid0) {
        switch (slotKind0) {
        case APA_SCENARIO_SLOT_KIND_VT:
          apaVtParkingPathPlanning(input, &geometricParam, output);
          break;
        case APA_SCENARIO_SLOT_KIND_HT:
          apaHtParkingPathPlanning(input, &geometricParam, output);
          break;
        case APA_SCENARIO_SLOT_KIND_DTH_ED_VT:
          apaDthEdVtParkingPathPlanning(input, &geometricParam, output);
          break;
        case APA_SCENARIO_SLOT_KIND_VH:
          apaVhParkingPathPlanning(input, &geometricParam, output);
          break;
        default:
          break;
        }
      } else {
        output->ErrorType = APA_FAULT_PLANNER_NOT_ACCEPT_THE_SLOT;
      }
    }

    if (output->PlanFlg != 0 && !parkoutRequested0) {
      safetyInput.PlannerInput = *input;
      safetyInput.PlannerParam = *param;
      safetyInput.CandidateOutput = *output;
      safetyOutput = (ApaParkingPathSafetyGateOutput_t){0};
      apaParkingPathSafetyGate(&safetyInput, &safetyOutput);
      *output = safetyOutput.ValidatedOutput;
    }
    if (commonSearch && slotKindValid0 && (output->PlanFlg || param->NeedHybridAlgFusion)) {

      diagnostic = (ApaPlanningDiagnostics_t){0};
      (void)apaParkingSelectCandidate(input, param, output);
    }
  }

  if (!parkingActionValid0)
    output->ErrorType = APA_FAULT_INVALID_APA_STATE;
  if (output->PlanFlg == 0 && output->ErrorType == APA_FAULT_UNKNOWN_ERR && !diagnostic.Stage)
    apaPlanningRecordFailure(
        &diagnostic, APA_FAULT_SLOT_AVALIABLE_BUT_PATH_INVALID, "geometric_no_solution", 0);
  if (output->PlanFlg == 0 && diagnostic.Stage &&
      (output->ErrorType == APA_FAULT_UNKNOWN_ERR || output->ErrorType == APA_FAULT_NO_ERR))
    output->ErrorType = diagnostic.ErrorType;
  fprintf(stderr,
          "[APA_PLANNING_DIAGNOSTIC] action=%ld slot=%ld ok=%ld error=%ld "
          "stage=%s search_fault=%ld accepted=%ld rejected=%ld truncated=%ld\n",
          (long)input->ParkingAction,
          (long)input->TargetSlotKind,
          (long)output->PlanFlg,
          (long)output->ErrorType,
          output->PlanFlg ? "accepted"
                          : diagnostic.Stage ? diagnostic.Stage : "geometric_no_solution",
          (long)diagnostic.SearchFault,
          (long)diagnostic.AcceptedCandidates,
          (long)diagnostic.RejectedCandidates,
          (long)diagnostic.TruncatedPrimitives);
  apaDebugTracePlanningOutput("APA_OUTPUT", frameId0, output);
  apaParkingPathPlanningTracePath(frameId0, output);
  apaDebugTraceParkoutPath("APA_FINAL", frameId0, input, output);
  if ((output->PlanFlg == 0) || (output->PtCount <= 1) || (output->SegNum <= 0) ||
      (output->PathLen <= 0.0f)) {
    (void)fprintf(stderr,
                  "[APA_DIAG_SUMMARY] frame_id=%lu ParkingAction=%ld "
                  "PlanFlg=%ld ErrorType=%ld PlannerCase=%ld PtCount=%ld "
                  "SegNum=%ld PathLen=%.9g path_valid=0\n",
                  (unsigned long)frameId0,
                  (long)input->ParkingAction,
                  (long)output->PlanFlg,
                  (long)output->ErrorType,
                  (long)output->PlannerCase,
                  (long)output->PtCount,
                  (long)output->SegNum,
                  (double)output->PathLen);
  }
}
