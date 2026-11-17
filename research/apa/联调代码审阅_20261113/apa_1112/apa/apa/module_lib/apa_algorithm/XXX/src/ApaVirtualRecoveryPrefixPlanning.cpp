#include "ApaVirtualRecoveryPrefixPlanning.hpp"

namespace apa {

/**
 * @brief 生成受物理障碍、初始虚拟约束和路程边界共同限制的恢复前缀。
 * @cn_name APA虚拟约束恢复前缀规划
 * @type atomic
 * @tag_level0 自动泊车
 * @tag_level1 恢复
 * @tag_level2 前缀规划
 * @version 1.0
 * @date 2026-11-03
 * @author wang
 */
void ApaVirtualRecoveryPrefixPlanning::run(const Input& input, Output& output) {
  native::PlanRecovery(input.environment, input.context, input.primitive,
      input.minimumRadius, input.sceneQueryId, output.prefix);
}

}
