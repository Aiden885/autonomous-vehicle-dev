#include "ApaParkingPathSafetyGate.hpp"

namespace apa {

/**
 * @brief 在完整场景与原生约束下验收候选路径。
 * @cn_name 泊车路径安全门
 * @type atomic
 * @tag_level0 自动泊车
 * @tag_level1 规划
 * @tag_level2 泊车路径安全门
 * @version 1.1
 * @date 2026-11-03
 * @author wang
 */
void ApaParkingPathSafetyGate::run(const Input& input, Output& output) {
  native::ValidatePath(input.plannerInput, global::params.planning, input.candidatePath,
      output.validatedPath, output.rejected);
}

}
