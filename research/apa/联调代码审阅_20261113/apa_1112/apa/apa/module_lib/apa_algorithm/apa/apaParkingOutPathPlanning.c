#include "apaParkingOutPathPlanning.h"
#include "apaParkingPathPlanningCommon.h"
#include "apaParkingSearchFallback.h"

/**
 * @brief Run the APA parking path planning park-out business branch.
 * @en_name apaParkingOutPathPlanning
 * @cn_name APA泊车路径规划泊出分支
 * @type module
 * @param[IN] ApaParkingOutPathPlanningInput_t Input 泊车规划输入
 * @param[IN] ApaParkingOutPathPlanningParam_t Param 泊车规划参数
 * @param[OUT] ApaParkingOutPathPlanningOutput_t Output 泊车规划输出
 * @granularity composite
 * @tag_level1 规划
 * @tag_level2 泊车路径规划分支
 * @source demo_new
 * @version 1.1
 * @date 2026-07-28
 * @author aide
 */
void apaParkingOutPathPlanning(const ApaParkingOutPathPlanningInput_t *input,
                               const ApaParkingOutPathPlanningParam_t *param,
                               ApaParkingOutPathPlanningOutput_t *output) {
  ApaParkingOutPathPlanningParam_t searchParam = *param;
  searchParam.NeedHybridAlgFusion = 1;
  apaParkingPathPlanningZeroOutput(output);
  apaParkingSelectCandidate(input, &searchParam, output);
  output->SelectedSlotKind = APA_SCENARIO_SLOT_KIND_PARKOUT;
}
