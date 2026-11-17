#include "ApaParkingOutPathPlanning.hpp"

namespace apa {

/**
 * @brief 映射数值端口并通过原子适配调用原有 C 规划入口。
 * @cn_name 泊出路径规划
 * @type atomic
 * @tag_level0 自动泊车
 * @tag_level1 规划
 * @tag_level2 泊出路径规划
 * @version 1.1
 * @date 2026-11-03
 * @author wang
 */
void ApaParkingOutPathPlanning::run(const Input& input, Output& output) {
  native::PlanOut(input.plannerInput, global::params.planning, output.candidatePath);
}

}
