#pragma once

#include "ApaParkingPathPlanningTypes.hpp"
#include "ApaNativePlanning.hpp"

namespace apa {

/** @brief ApaParkingPathSafetyGateTraits 是模块接口结构集合。 */
struct ApaParkingPathSafetyGateTraits {
  /** @brief 输入端口集合。 */
  struct Input {
    ApaParkingPathPlanningInputData plannerInput{};  ///< 已由 Adapter 转换的统一规划请求；内部字段保留各 CARLA 映射。
    ApaParkingPathPlanningOutputData candidatePath{};  ///< 等待结构、运动学和碰撞验收的候选路径。
  };
  /** @brief 输出端口集合。 */
  struct Output {
    ApaParkingPathPlanningOutputData validatedPath{};  ///< 安全门验收后的路径；失败时保持 V5 拒绝语义。
    Boolean rejected = false;  ///< 候选路径是否被安全门拒绝。
  };
  /** @brief 模块局部参数集合。 */
  struct Param {};
  /** @brief 模块局部跨周期状态集合。 */
  struct State {};
  /** @brief 子模块集合。 */
  struct Sub {};
};

/** @brief 保持 V5 业务行为的统一泊车路径安全门 FuncModule。 */
class ApaParkingPathSafetyGate
    : public FuncModule<ApaParkingPathSafetyGateTraits> {
 public:
  using FuncModule::FuncModule;

  /** @brief 显式映射 C++ 端口并调用现有 V5 安全门 C 入口。 */
  void run(const Input& input, Output& output) override;
};

}
