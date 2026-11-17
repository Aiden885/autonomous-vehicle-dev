#include "apaParkingPathSafetyGate.h"
#include "apaDebugTrace.h"
#include "apaParkingPathPlanningInternal.h"
#include <stdio.h>

/**
 * @brief Validate a unified candidate path and reject unsafe output.
 * @en_name apaParkingPathSafetyGate
 * @cn_name 泊车路径安全门
 * @type module
 * @param[IN] ApaParkingPathSafetyGateInput_t Input 待验收泊车规划结果
 * @param[OUT] ApaParkingPathSafetyGateOutput_t Output 安全验收结果
 * @granularity composite
 * @tag_level1 规划
 * @tag_level2 路径验收
 * @source demo_new
 * @version 1.0
 * @date 2026-07-28
 * @author aide
 */
void apaParkingPathSafetyGate(const ApaParkingPathSafetyGateInput_t *input,
                              ApaParkingPathSafetyGateOutput_t *output) {
  const ApaParkingPathPlanningInput_t *plannerInput;
  const ApaParkingPathPlanningParam_t *plannerParam;
  Boolean candidateValid0;
  Boolean candidateStructuralValid0;
  Boolean candidateSafe0;
  Boolean checkEndPose0;

  plannerInput = &input->PlannerInput;
  plannerParam = &input->PlannerParam;
  output->ValidatedOutput = input->CandidateOutput;
  apaDebugTracePlanningOutput(
      "SAFETY_GATE_INPUT", apaDebugTraceCurrentFrame(), &input->CandidateOutput);
  candidateValid0 = (output->ValidatedOutput.PlanFlg != 0);
  candidateStructuralValid0 = false;
  candidateSafe0 = false;
  checkEndPose0 = true;
  output->Rejected = false;
  if (candidateValid0) {
    candidateStructuralValid0 =
        apaParkingPathPlanningCandidateIsStructurallyValid(plannerInput, &output->ValidatedOutput);
    candidateSafe0 = candidateStructuralValid0;
    if (candidateSafe0) {
      candidateSafe0 = apaParkingPathPlanningOutputPathIsSafe(
          plannerInput, plannerParam, &output->ValidatedOutput, checkEndPose0);
    }
    if (!candidateSafe0) {
      if (!candidateStructuralValid0) {
        const Integer maximum =
            plannerInput->TargetSlotKind == APA_SCENARIO_SLOT_KIND_HT
                ? HT_SEG_OUT_MAX
                : plannerInput->TargetSlotKind == APA_SCENARIO_SLOT_KIND_DTH_ED_VT
                      ? GEO_DTH_ED_VT_SEG_OUT_MAX
                      : APA_PARKING_PATH_SEG_OUT_MAX;
        const Integer error = input->CandidateOutput.PtCount > APA_PARKING_PATH_PT_MAX
                                  ? APA_FAULT_PATH_N_EXCEED_TO_ADD
                                  : input->CandidateOutput.SegNum > maximum
                                        ? APA_FAULT_GEO_SEG_NUM_OVERMANY
                                        : APA_FAULT_PATH_STATUS_ERR;
        apaPlanningRecordFailure(plannerParam->Diagnostics, error, "candidate_structure", 4);
      }
      apaParkingPathPlanningRejectUnsafeOutput(&output->ValidatedOutput);
      if (plannerParam->Diagnostics && plannerParam->Diagnostics->Stage)
        output->ValidatedOutput.ErrorType = plannerParam->Diagnostics->ErrorType;
    }
  }
  if (candidateValid0) {
    output->Rejected = (output->ValidatedOutput.PlanFlg == 0);
  }
  (void)fprintf(stderr,
                "[APA_DIAG] frame_id=%lu stage=SAFETY_GATE_RESULT "
                "candidate_PlanFlg=%ld candidate_PtCount=%ld "
                "candidate_PathLen=%.9g validated_PlanFlg=%ld "
                "validated_PtCount=%ld validated_PathLen=%.9g accepted=%ld "
                "structural_valid=%ld safety_valid=%ld\n",
                (unsigned long)apaDebugTraceCurrentFrame(),
                (long)input->CandidateOutput.PlanFlg,
                (long)input->CandidateOutput.PtCount,
                (double)input->CandidateOutput.PathLen,
                (long)output->ValidatedOutput.PlanFlg,
                (long)output->ValidatedOutput.PtCount,
                (double)output->ValidatedOutput.PathLen,
                (long)(candidateValid0 && (!output->Rejected)),
                (long)candidateStructuralValid0,
                (long)candidateSafe0);
  apaDebugTracePlanningOutput(
      "SAFETY_GATE_OUTPUT", apaDebugTraceCurrentFrame(), &output->ValidatedOutput);
  apaDebugTraceParkoutPath(
      "PARKOUT_SAFETY_GATE", apaDebugTraceCurrentFrame(), plannerInput, &output->ValidatedOutput);
}
