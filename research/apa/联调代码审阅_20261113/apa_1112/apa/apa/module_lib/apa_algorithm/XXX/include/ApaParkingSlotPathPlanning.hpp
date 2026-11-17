#pragma once

#include "ApaParkingPathPlanningTypes.hpp"
#include "ApaNativePlanning.hpp"

namespace apa {

/** @brief ApaParkingSlotPathPlanningTraits 是模块接口结构集合。 */
struct ApaParkingSlotPathPlanningTraits {
  /** @brief 输入端口集合。 */
  struct Input {
    ApaParkingPathPlanningInputData plannerInput{}; ///< 本周期泊车规划请求。
  };
  /** @brief 输出端口集合。 */
  struct Output {
    ApaParkingPathPlanningOutputData candidatePath{}; ///< 本周期泊车规划结果。
    ApaParkingPathPlanningInputData resolvedInput{}; ///< C 入口解析后的业务请求。
  };
  /** @brief 模块局部参数集合。 */
  struct Param {};
  /** @brief 模块局部跨周期状态集合。 */
  struct State {};
  /** @brief 子模块集合。 */
  struct Sub {};
};

/** @brief VT/HT/DTH/VH/泊出共用的 C 目标解析和候选验收包装。 */
class ApaParkingSlotPathPlanning
    : public FuncModule<ApaParkingSlotPathPlanningTraits> {
 public:
  using FuncModule::FuncModule;
  void run(const Input& input, Output& output) override;
};

}
