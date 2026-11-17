#include "apaVtParkingPathPlanning.h"
#include "apaParkingPathPlanningInternal.h"
#include "apaParkingPathSafetyGate.h"
#include "apaVtParkingReverseFallback.h"
#include "geoVtPathPlanningInternal.h"
#include <math.h>
#include <stdio.h>

typedef struct {
  const ApaVtParkingPathPlanningInput_t *Input;
  const ApaVtParkingPathPlanningParam_t *Param;
  const GeoVtPathPlanningInput_t *GeometricInput;
  Integer RejectedGeometry;
} ApaVtParkingTrialContext_t;

/**
 * @brief Validate the vehicle geometry used by this VT call.
 * @en_name apaVtParkingModelIsValid
 * @cn_name VT车型配置检查
 * @type module
 * @param[IN] ApaVtParkingPathPlanningInput_t Input 最小半径输入
 * @param[IN] ApaVtParkingPathPlanningParam_t Param 车型配置
 * @param[OUT] Boolean Valid 返回有效标志
 * @retval Boolean 配置是否有限且合法
 * @granularity atomic
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static Boolean apaVtParkingModelIsValid(const ApaVtParkingPathPlanningInput_t *input,
                                        const ApaVtParkingPathPlanningParam_t *param) {
  Boolean valid0;
  valid0 = isfinite(param->VehW) && isfinite(param->VehLlF) && isfinite(param->VehLr) &&
           isfinite(input->Rmin);
  if (valid0) {
    valid0 = (param->VehW > 0.0f) && (param->VehLlF > 0.0f) && (param->VehLr >= 0.0f) &&
             (input->Rmin > 0.0f);
  }
  return valid0;
}

/**
 * @brief Discretize one VT geometry into a candidate without publishing it.
 * @en_name apaVtParkingBuildCandidate
 * @cn_name VT构造待验收候选
 * @type module
 * @param[IN] ApaVtParkingTrialContext_t Trial 本次输入与参数
 * @param[IN] GeoVtPathPlanningOutput_t Geometry 几何段
 * @param[OUT] ApaParkingPathPlanningOutput_t Candidate 离散候选
 * @granularity composite
 * @tag_level1 规划
 * @source demo_new
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static void apaVtParkingBuildCandidate(const ApaVtParkingTrialContext_t *trial,
                                       const GeoVtPathPlanningOutput_t *geometry,
                                       ApaParkingPathPlanningOutput_t *candidate) {
  GeoPthDiscretVtInput_t discretInput;
  GeoPthDiscretVtOutput_t discretOutput;
  apaParkingPathPlanningZeroOutput(candidate);

  discretOutput = (GeoPthDiscretVtOutput_t){0};
  apaParkingPathPlanningFillVtDiscretInput(trial->Input, geometry, &discretInput);
  geoPthDiscretVt(&discretInput, &discretOutput);
  candidate->SelectedSlotKind = APA_SCENARIO_SLOT_KIND_VT;
  candidate->PlanFlg = geometry->PlanFlg;
  candidate->PlannerCase = geometry->PlannerCase;
  candidate->SegNum = geometry->SegNum;
  candidate->StpN = geometry->StpN;
  candidate->PathLen = geometry->PathLen;
  apaParkingPathPlanningCopyVtPath(&discretOutput, candidate);
  apaParkingPathPlanningFinalizeL1Output(candidate, VT_SEG_OUT_MAX);
}

/**
 * @brief Accept only fully validated VT candidates before geometric selection.
 * @en_name apaVtParkingAcceptCandidate
 * @cn_name VT完整候选验收
 * @type module
 * @param[IN] GeoVtPathPlanningOutput_t Geometry 几何候选
 * @param[IN] ApaVtParkingTrialContext_t UserData 本次调用上下文
 * @param[OUT] Boolean Accepted 返回可发布标志
 * @retval Boolean 是否可发布
 * @granularity composite
 * @tag_level1 规划
 * @source safety_only
 * @version 1.0
 * @date 2026-09-30
 * @author aide
 */
static Boolean apaVtParkingAcceptCandidate(const GeoVtPathPlanningOutput_t *geometry,
                                           void *userData) {
  ApaVtParkingTrialContext_t *trial;
  ApaParkingPathSafetyGateInput_t safetyInput;
  ApaParkingPathSafetyGateOutput_t safetyOutput;
  Integer rejectedSegment;
  Boolean accepted0;
  trial = (ApaVtParkingTrialContext_t *)userData;
  accepted0 =
      geoVtPathPlanningGeometryIsContinuous(trial->GeometricInput, geometry, &rejectedSegment);
  if (!accepted0 && ++trial->RejectedGeometry <= 8) {
    fprintf(stderr,
            "[APA_VT_CANDIDATE] accepted=0 reason=segment_continuity "
            "segment=%ld segments=%ld\n",
            (long)rejectedSegment,
            (long)geometry->SegNum);
  }
  if (accepted0) {
    safetyInput.PlannerInput = *trial->Input;
    safetyInput.PlannerParam = *trial->Param;
    apaVtParkingBuildCandidate(trial, geometry, &safetyInput.CandidateOutput);
    safetyOutput = (ApaParkingPathSafetyGateOutput_t){0};
    apaParkingPathSafetyGate(&safetyInput, &safetyOutput);
    accepted0 = (safetyOutput.ValidatedOutput.PlanFlg != 0);
  }
  return accepted0;
}

/**
 * @brief VT business branch for APA parking path planning.
 * @en_name apaVtParkingPathPlanning
 * @cn_name APA泊车路径规划VT分支
 * @type module
 * @param[IN] ApaVtParkingPathPlanningInput_t Input 泊车规划输入
 * @param[IN] ApaVtParkingPathPlanningParam_t Param 泊车规划参数
 * @param[OUT] ApaVtParkingPathPlanningOutput_t Output 泊车规划输出
 * @granularity composite
 * @tag_level1 规划
 * @tag_level2 泊车路径规划分支
 * @source demo_new
 * @version 1.1
 * @date 2026-07-28
 * @author aide
 */
void apaVtParkingPathPlanning(const ApaVtParkingPathPlanningInput_t *input,
                              const ApaVtParkingPathPlanningParam_t *param,
                              ApaVtParkingPathPlanningOutput_t *output) {
  GeoVtPathPlanningInput_t plannerInput;
  GeoVtPathPlanningParam_t plannerParam;
  GeoVtPathPlanningOutput_t plannerOutput;
  GeoPthDiscretVtInput_t discretInput;
  GeoPthDiscretVtOutput_t discretOutput;
  ApaVtParkingReverseFallbackOutput_t fallbackOutput;
  Boolean reversePathNeeded0;
  ApaTargetSlotConstraint_t constraint;
  GeoVtPathPlanningPolicy_t policy;
  ApaVtParkingTrialContext_t trial;
  Boolean modelValid0;
  Boolean policyAllowed0;

  apaParkingPathPlanningZeroOutput(output);
  output->SelectedSlotKind = APA_SCENARIO_SLOT_KIND_VT;
  modelValid0 = true;
  if (input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKIN) {
    modelValid0 = apaVtParkingModelIsValid(input, param);
  }
  if (modelValid0) {
    plannerOutput = (GeoVtPathPlanningOutput_t){0};
    discretOutput = (GeoPthDiscretVtOutput_t){0};
    apaParkingPathPlanningFillVtInput(input, &plannerInput);
    apaParkingPathPlanningFillVtParam(param, &plannerParam);
    plannerParam.RminClassifiedSlot = plannerInput.Rmin;
    policyAllowed0 = (input->ParkingAction == APA_SCENARIO_PARKING_ACTION_PARKIN);
    if (policyAllowed0) {
      trial.Input = input;
      trial.Param = param;

      trial.GeometricInput = &plannerInput;
      trial.RejectedGeometry = 0;
      apaParkingPathPlanningBuildTargetSlotConstraint(input, param, &constraint);
      policy.SlotConstraint = &constraint;
      policy.AcceptCandidate = apaVtParkingAcceptCandidate;
      policy.UserData = &trial;
      geoVtPathPlanningWithPolicy(&plannerInput, &plannerParam, &policy, &plannerOutput);
    } else {
      geoVtPathPlanning(&plannerInput, &plannerParam, &plannerOutput);
    }
    if ((plannerOutput.PlanFlg != 0) && (plannerOutput.SegNum > 0)) {
      apaParkingPathPlanningFillVtDiscretInput(input, &plannerOutput, &discretInput);
      geoPthDiscretVt(&discretInput, &discretOutput);
    }
    output->SelectedSlotKind = APA_SCENARIO_SLOT_KIND_VT;
    output->PlanFlg = plannerOutput.PlanFlg;
    output->PlannerCase = plannerOutput.PlannerCase;
    output->SegNum = plannerOutput.SegNum;
    output->StpN = plannerOutput.StpN;
    output->PathLen = plannerOutput.PathLen;
    apaParkingPathPlanningCopyVtPath(&discretOutput, output);
    reversePathNeeded0 = apaParkingPathPlanningShouldTryReverseVtPath(input);

    if (reversePathNeeded0 &&
        (output->PlanFlg == 0 ||
         !apaParkingPathPlanningCandidateIsStructurallyValid(input, output) ||
         !apaParkingPathPlanningOutputPathIsSafe(input, param, output, true))) {
      fallbackOutput = (ApaVtParkingReverseFallbackOutput_t){0};
      apaVtParkingReverseFallback(input, param, &fallbackOutput);
      if (fallbackOutput.PlanFlg != 0 &&
          apaParkingPathPlanningCandidateIsStructurallyValid(input, &fallbackOutput) &&
          apaParkingPathPlanningOutputPathIsSafe(input, param, &fallbackOutput, true)) {
        *output = fallbackOutput;
      }
    }
    apaParkingPathPlanningFinalizeL1Output(output, VT_SEG_OUT_MAX);
  }
}
