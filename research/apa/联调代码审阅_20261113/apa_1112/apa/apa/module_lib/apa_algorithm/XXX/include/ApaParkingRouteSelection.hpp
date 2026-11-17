#pragma once

#include "FuncModule.hpp"
#include "ApaParkingPathPlanningTypes.hpp"
#include "ApaParkingSlotPathPlanning.hpp"

namespace apa {

/** @brief ApaParkingRouteSelectionTraits 是模块接口结构集合。 */
struct ApaParkingRouteSelectionTraits {
  /** @brief 输入端口集合。 */
  struct Input {
    ApaParkingPathPlanningInputData plannerInput{}; ///< 本周期统一规划请求。
  };
  /** @brief 输出端口集合。 */
  struct Output {
    ApaParkingPathPlanningOutputData candidatePath{}; ///< 统一入口已经验收的候选路径。
    ApaParkingPathPlanningInputData resolvedInput{}; ///< C 入口解析后的业务请求。
  };
  /** @brief 模块局部参数集合。 */
  struct Param {};
  /** @brief 模块局部跨周期状态集合。 */
  struct State {};
  /** @brief 子模块集合。 */
  struct Sub {
    ApaParkingSlotPathPlanning slotPathPlanning; ///< 完成车位目标解析和候选验收的原件。
  };
};

/** @brief 共用目标解析和几何安全验收的泊车业务路由。 */
class ApaParkingRouteSelection : public FuncModule<ApaParkingRouteSelectionTraits> {
 public:
  ApaParkingRouteSelection(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{ApaParkingSlotPathPlanning(
                ApaParkingSlotPathPlanningTraits::Param{},
                ApaParkingSlotPathPlanningTraits::State{})},
            std::move(param), std::move(state)} {}
  void run(const Input& input, Output& output) override;
};

}
