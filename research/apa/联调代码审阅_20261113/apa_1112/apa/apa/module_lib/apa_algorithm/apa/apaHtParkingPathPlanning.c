#include "apaHtParkingPathPlanning.h"
#include "apaDebugTrace.h"
#include "apaParkingPathPlanningInternal.h"
#include <stdio.h>

/**
 * @brief HT business branch for APA parking path planning.
 * @en_name apaHtParkingPathPlanning
 * @cn_name APA泊车路径规划HT分支
 * @type module
 * @param[IN] ApaHtParkingPathPlanningInput_t Input 泊车规划输入
 * @param[IN] ApaHtParkingPathPlanningParam_t Param 泊车规划参数
 * @param[OUT] ApaHtParkingPathPlanningOutput_t Output 泊车规划输出
 * @granularity composite
 * @tag_level1 规划
 * @tag_level2 泊车路径规划分支
 * @source demo_new
 * @version 1.1
 * @date 2026-07-28
 * @author aide
 */
void apaHtParkingPathPlanning(const ApaHtParkingPathPlanningInput_t *input,
                              const ApaHtParkingPathPlanningParam_t *param,
                              ApaHtParkingPathPlanningOutput_t *output) {
  GeoHtPathPlanningInput_t plannerInput;
  GeoHtPathPlanningParam_t plannerParam;
  GeoHtPathPlanningOutput_t plannerOutput;
  GeoPthDiscretHtInput_t discretInput;
  GeoPthDiscretHtOutput_t discretOutput;
  Boolean planFlagValid;
  Boolean segmentValid;
  Boolean discretizeNeeded;

  apaParkingPathPlanningZeroOutput(output);
  plannerOutput = (GeoHtPathPlanningOutput_t){0};
  discretOutput = (GeoPthDiscretHtOutput_t){0};
  apaParkingPathPlanningFillHtInput(input, &plannerInput);
  apaParkingPathPlanningFillHtParam(param, &plannerParam);
  geoHtPathPlanning(&plannerInput, &plannerParam, &plannerOutput);
  (void)fprintf(stderr,
                "[APA_DIAG] frame_id=%lu stage=HT_DIAG "
                "PlanFlg=%ld PlannerCase=%ld SegNum=%ld PathLen=%.9g "
                "FailRs0=%ld "
                "FallbackGeomCandidateFlg=%ld "
                "FallbackHybridCandidateFlg=%ld FallbackWinnerKind=%ld "
                "FallbackDecisionReason=%ld\n",
                (unsigned long)apaDebugTraceCurrentFrame(),
                (long)plannerOutput.PlanFlg,
                (long)plannerOutput.PlannerCase,
                (long)plannerOutput.SegNum,
                (double)plannerOutput.PathLen,
                (long)plannerOutput.FailRs0,
                (long)plannerOutput.FallbackGeomCandidateFlg,
                (long)plannerOutput.FallbackHybridCandidateFlg,
                (long)plannerOutput.FallbackWinnerKind,
                (long)plannerOutput.FallbackDecisionReason);

  output->SelectedSlotKind = APA_SCENARIO_SLOT_KIND_HT;
  output->PlanFlg = plannerOutput.PlanFlg;
  output->PlannerCase = plannerOutput.PlannerCase;
  output->SegNum = plannerOutput.SegNum;
  output->StpN = plannerOutput.StpN;
  output->PathLen = plannerOutput.PathLen;
  planFlagValid = (plannerOutput.PlanFlg != 0);
  segmentValid = (plannerOutput.SegNum > 0);
  discretizeNeeded = false;
  if (planFlagValid) {
    if (segmentValid) {
      discretizeNeeded = true;
    }
  }
  if (discretizeNeeded) {
    apaParkingPathPlanningFillHtDiscretInput(input, &plannerOutput, &discretInput);

    geoPthDiscretHtPkIn(&discretInput, &discretOutput);
    apaParkingPathPlanningCopyHtPath(&discretOutput, output);
  }
  apaParkingPathPlanningFinalizeL1Output(output, HT_SEG_OUT_MAX);
  apaDebugTracePlanningOutput("HT_PATH_GENERATION", apaDebugTraceCurrentFrame(), output);
}
