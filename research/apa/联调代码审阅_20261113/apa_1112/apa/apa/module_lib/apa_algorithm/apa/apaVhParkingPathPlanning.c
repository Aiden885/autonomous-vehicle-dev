#include "apaVhParkingPathPlanning.h"
#include "apaDebugTrace.h"
#include "apaParkingPathPlanningInternal.h"
#include <stdio.h>

/**
 * @brief VH business branch for APA parking path planning.
 * @en_name apaVhParkingPathPlanning
 * @cn_name APA泊车路径规划VH分支
 * @type module
 * @param[IN] ApaVhParkingPathPlanningInput_t Input 泊车规划输入
 * @param[IN] ApaVhParkingPathPlanningParam_t Param 泊车规划参数
 * @param[OUT] ApaVhParkingPathPlanningOutput_t Output 泊车规划输出
 * @granularity composite
 * @tag_level1 规划
 * @tag_level2 泊车路径规划分支
 * @source demo_new
 * @version 1.1
 * @date 2026-07-28
 * @author aide
 */
void apaVhParkingPathPlanning(const ApaVhParkingPathPlanningInput_t *input,
                              const ApaVhParkingPathPlanningParam_t *param,
                              ApaVhParkingPathPlanningOutput_t *output) {
  GeoVhPathPlanningInput_t plannerInput;
  GeoVhPathPlanningParam_t plannerParam;
  GeoVhPathPlanningOutput_t plannerOutput;
  GeoPthDiscretVhInput_t discretInput;
  GeoPthDiscretVhOutput_t discretOutput;
  Boolean planFlagValid0;
  Boolean segmentValid0;
  Boolean discretizeNeeded0;

  apaParkingPathPlanningZeroOutput(output);
  plannerOutput = (GeoVhPathPlanningOutput_t){0};
  discretOutput = (GeoPthDiscretVhOutput_t){0};
  apaParkingPathPlanningFillVhInput(input, &plannerInput);
  apaParkingPathPlanningFillVhParam(param, &plannerParam);
  geoVhPathPlanning(&plannerInput, &plannerParam, &plannerOutput);
  (void)fprintf(stderr,
                "[APA_DIAG] frame_id=%lu stage=VH_DIAG "
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

  output->SelectedSlotKind = APA_SCENARIO_SLOT_KIND_VH;
  output->PlanFlg = plannerOutput.PlanFlg;
  output->PlannerCase = plannerOutput.PlannerCase;
  output->SegNum = plannerOutput.SegNum;
  output->StpN = plannerOutput.StpN;
  output->PathLen = plannerOutput.PathLen;
  planFlagValid0 = (plannerOutput.PlanFlg != 0);
  segmentValid0 = (plannerOutput.SegNum > 0);
  discretizeNeeded0 = planFlagValid0 && segmentValid0;
  if (discretizeNeeded0) {
    apaParkingPathPlanningFillVhDiscretInput(input, &plannerOutput, &discretInput);
    geoPthDiscretVh(&discretInput, &discretOutput);
    apaParkingPathPlanningCopyVhPath(&discretOutput, output);
  }
  apaParkingPathPlanningFinalizeL1Output(output, VH_SEG_OUT_MAX);
  apaDebugTracePlanningOutput("VH_PATH_GENERATION", apaDebugTraceCurrentFrame(), output);
}
