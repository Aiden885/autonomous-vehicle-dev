#pragma once

#include "FuncModule.hpp"
#include "ApaVirtualRecoveryPrefixPlanningTypes.hpp"
#include "ApaNativePlanning.hpp"

namespace apa {

/** @brief ApaVirtualRecoveryPrefixPlanningTraits 是模块接口结构集合。 */
struct ApaVirtualRecoveryPrefixPlanningTraits {
  /** @brief 输入端口集合。 */
  struct Input {
    ApaRecoveryEnvironment environment{}; ///< 同一任务完整原生约束，复用 C 原子接口。
    ApaVirtualRecoveryContext context{}; ///< 绑定原起点和安全历史一侧的只读恢复上下文。
    UInteger sceneQueryId = 0; ///< 当前线程完整物理场景的有效资源编号，不跨进程序列化。
    Integer primitive = 0; ///< 有限常曲率运动原语编号，范围 0 到 9。
    Real minimumRadius = 0.0; ///< 当前执行器可实现的最小转弯半径，单位 m。
  };
  /** @brief 输出端口集合。 */
  struct Output {
    ApaVirtualRecoveryPrefix prefix{}; ///< C 原子接口的定容候选，保留几何/里程单一来源。
  };
  /** @brief 模块局部参数集合。 */
  struct Param {};
  /** @brief 模块局部跨周期状态集合。 */
  struct State {};
  /** @brief 子模块集合。 */
  struct Sub {};
};

/** @brief 通过 C 原子入口生成可回到正常约束的有限运动前缀。 */
class ApaVirtualRecoveryPrefixPlanning : public FuncModule<ApaVirtualRecoveryPrefixPlanningTraits> {
 public:
  using FuncModule::FuncModule;
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
  void run(const Input& input, Output& output) override;
};

}
