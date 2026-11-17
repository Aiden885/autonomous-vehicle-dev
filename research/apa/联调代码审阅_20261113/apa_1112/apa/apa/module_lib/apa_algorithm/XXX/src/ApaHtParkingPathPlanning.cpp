#include "ApaHtParkingPathPlanning.hpp"

namespace apa {

/**
 * @brief 映射数值端口并通过原子适配调用原有 C 规划入口。
 * @cn_name 平行车位泊入规划
 * @type atomic
 * @tag_level0 自动泊车
 * @tag_level1 规划
 * @tag_level2 平行车位泊入规划
 * @version 1.1
 * @date 2026-11-03
 * @author wang
 */
void ApaHtParkingPathPlanning::run(const Input& input, Output& output) {
  native::PlanHt(input.plannerInput, global::params.planning, output.candidatePath);
}

}
