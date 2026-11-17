#include "apaVtParkingReverseFallback.h"
#include "apaParkingPathPlanningInternal.h"

/**
 * @brief Run the reverse-start VT fallback and restore forward output order.
 * @en_name apaVtParkingReverseFallback
 * @cn_name VT反向规划回退
 * @type module
 * @param[IN] ApaVtParkingReverseFallbackInput_t Input VT泊车规划输入
 * @param[IN] ApaVtParkingReverseFallbackParam_t Param VT泊车规划参数
 * @param[OUT] ApaVtParkingReverseFallbackOutput_t Output VT泊车规划输出
 * @granularity composite
 * @tag_level1 规划
 * @tag_level2 VT回退
 * @source demo_new
 * @version 1.1
 * @date 2026-07-28
 * @author aide
 */
void apaVtParkingReverseFallback(const ApaVtParkingReverseFallbackInput_t *input,
                                 const ApaVtParkingReverseFallbackParam_t *param,
                                 ApaVtParkingReverseFallbackOutput_t *output) {
  ApaParkingPathPlanningInput_t reverseInput;
  GeoVtPathPlanningInput_t plannerInput;
  GeoVtPathPlanningParam_t plannerParam;
  GeoVtPathPlanningOutput_t plannerOutput;
  GeoPthDiscretVtInput_t discretInput;
  GeoPthDiscretVtOutput_t discretOutput;
  Boolean shouldTry0;
  Boolean planFound0;

  shouldTry0 = apaParkingPathPlanningShouldTryReverseVtPath(input);
  if (shouldTry0) {
    reverseInput = *input;
    apaParkingPathPlanningSwapInputStartTargetPose(&reverseInput);
    plannerOutput = (GeoVtPathPlanningOutput_t){0};
    discretOutput = (GeoPthDiscretVtOutput_t){0};
    apaParkingPathPlanningFillVtInput(&reverseInput, &plannerInput);
    apaParkingPathPlanningFillVtParam(param, &plannerParam);
    plannerParam.RminClassifiedSlot = plannerInput.Rmin;
    geoVtPathPlanning(&plannerInput, &plannerParam, &plannerOutput);
    planFound0 = (plannerOutput.PlanFlg != 0);
    if (planFound0) {
      apaParkingPathPlanningFillVtDiscretInput(&reverseInput, &plannerOutput, &discretInput);
      geoPthDiscretVt(&discretInput, &discretOutput);
      output->SelectedSlotKind = APA_SCENARIO_SLOT_KIND_VT;
      output->PlanFlg = plannerOutput.PlanFlg;
      output->PlannerCase = plannerOutput.PlannerCase;
      output->SegNum = plannerOutput.SegNum;
      output->StpN = plannerOutput.StpN;
      output->PathLen = plannerOutput.PathLen;
      apaParkingPathPlanningCopyVtPath(&discretOutput, output);
      apaParkingPathPlanningReverseOutputPath(output);
      apaParkingPathPlanningRefreshVtStp1Metadata(input, output);
    }
  }
}
