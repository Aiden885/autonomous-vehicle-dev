#pragma once

#include "FuncModule.hpp"
#include "ApaParkingPathPlanningTypes.hpp"
#include "ApaParkingRouteSelection.hpp"

namespace apa {

/** @brief ApaParkingPathPlanningTraits 是模块接口结构集合。 */
struct ApaParkingPathPlanningTraits {
  /** @brief 输入端口集合。 */
  struct Input {
    ApaParkingPathPlanningInputData plannerInput{}; ///< 本周期泊车规划请求。
  };
  /** @brief 输出端口集合。 */
  struct Output {
    ApaParkingPathPlanningOutputData candidatePath{}; ///< 本周期泊车规划结果。
  };
  /** @brief 模块局部参数集合。 */
  struct Param {};
  /** @brief 模块局部跨周期状态集合。 */
  struct State {};
  /** @brief 子模块集合。 */
  struct Sub {
    ApaParkingRouteSelection routeSelection; ///< 统一 C 入口，返回已经验收的路径。
  };
};

/** @brief 调度 V5 各泊车规划分支并统一发布安全路径。 */
class ApaParkingPathPlanning : public FuncModule<ApaParkingPathPlanningTraits> {
 public:
  ApaParkingPathPlanning(Param param = Param{}, State state = State{})
      : FuncModule{
            Sub{
                ApaParkingRouteSelection(
                    ApaParkingRouteSelectionTraits::Param{},
                    ApaParkingRouteSelectionTraits::State{})},
            std::move(param),
            std::move(state)} {}

  /** @brief 发布统一 C 规划入口已完成连接及安全验收的路径。 */
  void run(const Input& input, Output& output) override;
};

}
