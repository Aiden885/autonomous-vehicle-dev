#include "ApaParkingSlotPathPlanning.hpp"

namespace apa {

/**
 * @brief 保留请求目标，经统一 C 入口解析车位并验收候选。
 * @cn_name 统一车位目标规划
 * @type atomic
 * @tag_level0 自动泊车
 * @tag_level1 规划
 * @tag_level2 统一车位目标规划
 * @version 1.1
 * @date 2026-11-03
 * @author wang
 */
void ApaParkingSlotPathPlanning::run(const Input& input, Output& output) {
  native::PlanSlot(input.plannerInput, global::params.planning,
      output.candidatePath, output.resolvedInput);
}

}
