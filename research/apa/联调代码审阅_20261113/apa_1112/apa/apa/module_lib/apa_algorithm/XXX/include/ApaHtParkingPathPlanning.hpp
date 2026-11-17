#pragma once

#include "ApaParkingPathPlanningTypes.hpp"
#include "ApaNativePlanning.hpp"

namespace apa {

/** @brief ApaHtParkingPathPlanningTraits 是模块接口结构集合。 */
struct ApaHtParkingPathPlanningTraits {
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
  struct Sub {};
};

/** @brief 保持 V5 业务行为的 HT 泊入规划 C++ FuncModule 包装。 */
class ApaHtParkingPathPlanning
    : public FuncModule<ApaHtParkingPathPlanningTraits> {
 public:
  using FuncModule::FuncModule;

  /** @brief 显式映射 C++ 端口并调用现有 V5 HT C 规划入口。 */
  void run(const Input& input, Output& output) override;
};

}
