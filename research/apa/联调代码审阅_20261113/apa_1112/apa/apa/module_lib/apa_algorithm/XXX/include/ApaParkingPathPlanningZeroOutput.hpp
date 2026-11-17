#pragma once

#include "ApaParkingPathPlanningTypes.hpp"

namespace apa {

/** @brief ApaParkingPathPlanningZeroOutputTraits 是模块接口结构集合。 */
struct ApaParkingPathPlanningZeroOutputTraits {
  /** @brief 输入端口集合。 */
  struct Input {};
  /** @brief 输出端口集合。 */
  struct Output {
    ApaParkingPathPlanningOutputData candidatePath{}; ///< 本周期泊车规划结果。
  };
  /** @brief 模块局部参数集合。 */
  struct Param {};
  /** @brief 模块局部跨周期状态集合。 */
  struct State {};
  /** @brief 子模块集合。 */
  struct Sub {};
};

/** @brief 按 V5 语义初始化统一泊车规划输出。 */
class ApaParkingPathPlanningZeroOutput
    : public FuncModule<ApaParkingPathPlanningZeroOutputTraits> {
 public:
  using FuncModule::FuncModule;

  /** @brief 生成完整的失败初始输出并清零路径数组。 */
  void run(const Input& input, Output& output) override;
};

}
