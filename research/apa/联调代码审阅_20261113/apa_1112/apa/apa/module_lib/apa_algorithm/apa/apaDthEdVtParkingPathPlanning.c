#include "apaDthEdVtParkingPathPlanning.h"
#include "apaDebugTrace.h"
#include "apaParkingPathPlanningInternal.h"
#include <stdio.h>

/**
 * @brief DthEdVT business branch for APA parking path planning.
 * @en_name apaDthEdVtParkingPathPlanning
 * @cn_name APA泊车路径规划DthEdVT分支
 * @type module
 * @param[IN] ApaDthEdVtParkingPathPlanningInput_t Input 泊车规划输入
 * @param[IN] ApaDthEdVtParkingPathPlanningParam_t Param 泊车规划参数
 * @param[OUT] ApaDthEdVtParkingPathPlanningOutput_t Output 泊车规划输出
 * @granularity composite
 * @tag_level1 规划
 * @tag_level2 泊车路径规划分支
 * @source demo_new
 * @version 1.1
 * @date 2026-07-28
 * @author aide
 */
void apaDthEdVtParkingPathPlanning(const ApaDthEdVtParkingPathPlanningInput_t *input,
                                   const ApaDthEdVtParkingPathPlanningParam_t *param,
                                   ApaDthEdVtParkingPathPlanningOutput_t *output) {
  GeoDthEdVtPathPlanningInput_t plannerInput;
  GeoDthEdVtPathPlanningParam_t plannerParam;
  GeoDthEdVtPathPlanningOutput_t plannerOutput;
  GeoPthDiscretVtInput_t discretInput;
  GeoPthDiscretVtOutput_t discretOutput;
  Boolean planFlagValid0;
  Boolean segmentValid0;
  Boolean discretizeNeeded0;
  Integer failReasonIndex0;

  apaParkingPathPlanningZeroOutput(output);
  plannerOutput = (GeoDthEdVtPathPlanningOutput_t){0};
  discretOutput = (GeoPthDiscretVtOutput_t){0};
  apaParkingPathPlanningFillDthInput(input, &plannerInput);
  apaParkingPathPlanningFillDthParam(param, &plannerParam);
  geoDthEdVtPathPlanning(&plannerInput, &plannerParam, &plannerOutput);
  (void)fprintf(stderr,
                "[APA_DIAG] frame_id=%lu stage=DTH_ED_VT_DIAG "
                "PlanFlg=%ld PlannerCase=%ld SegNum=%ld PathLen=%.9g "
                "FailRs0=%ld FailRs1=%ld FallbackGeomCandidateFlg=%ld "
                "FallbackHybridCandidateFlg=%ld FallbackWinnerKind=%ld "
                "FallbackDecisionReason=%ld\n",
                (unsigned long)apaDebugTraceCurrentFrame(),
                (long)plannerOutput.PlanFlg,
                (long)plannerOutput.PlannerCase,
                (long)plannerOutput.SegNum,
                (double)plannerOutput.PathLen,
                (long)plannerOutput.FailRs0,
                (long)plannerOutput.FailRs1,
                (long)plannerOutput.FallbackGeomCandidateFlg,
                (long)plannerOutput.FallbackHybridCandidateFlg,
                (long)plannerOutput.FallbackWinnerKind,
                (long)plannerOutput.FallbackDecisionReason);
  failReasonIndex0 = 0;
  while (failReasonIndex0 < GEO_DTH_ED_VT_PATH_LIST_MAX) {
    if (plannerOutput.FailRsOut[failReasonIndex0] != 0) {
      (void)fprintf(stderr,
                    "[APA_DIAG] frame_id=%lu stage=DTH_ED_VT_DIAG "
                    "FailRsOut[%ld]=%ld\n",
                    (unsigned long)apaDebugTraceCurrentFrame(),
                    (long)failReasonIndex0,
                    (long)plannerOutput.FailRsOut[failReasonIndex0]);
    }
    failReasonIndex0 = failReasonIndex0 + 1;
  }

  output->SelectedSlotKind = APA_SCENARIO_SLOT_KIND_DTH_ED_VT;
  output->PlanFlg = plannerOutput.PlanFlg;
  output->PlannerCase = plannerOutput.PlannerCase;
  output->SegNum = plannerOutput.SegNum;
  output->StpN = plannerOutput.StpN;
  output->PathLen = plannerOutput.PathLen;
  planFlagValid0 = (plannerOutput.PlanFlg != 0);
  segmentValid0 = (plannerOutput.SegNum > 0);
  discretizeNeeded0 = planFlagValid0 && segmentValid0;
  if (discretizeNeeded0) {
    apaParkingPathPlanningFillDthDiscretInput(input, &plannerOutput, &discretInput);
    geoPthDiscretVt(&discretInput, &discretOutput);
    apaParkingPathPlanningCopyVtPath(&discretOutput, output);
  }
  apaParkingPathPlanningFinalizeL1Output(output, GEO_DTH_ED_VT_SEG_OUT_MAX);
  apaDebugTracePlanningOutput("DTH_ED_VT_PATH_GENERATION", apaDebugTraceCurrentFrame(), output);
}
