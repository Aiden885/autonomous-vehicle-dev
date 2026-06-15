#pragma once

#include "FuncModule.hpp"

namespace control {

/**
 * @brief CARLAACCLeadDistance 模块接口结构集合。
 */
struct CARLAACCLeadDistanceTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {
    Real distance = 1000000.0; ///< 运行时边界层提供的前车净距离 (m)
    int valid = 0;             ///< 前车状态有效标志，1 表示有效
  };

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    Real distance = 1000000.0; ///< 自车到前车净距离 (m)
    int valid = 0;             ///< 前车距离有效标志，1 表示有效
  };

  /**
   * @brief 参数集合。
   */
  struct Param {};

  /**
   * @brief 状态集合。
   */
  struct State {};

  /**
   * @brief 子模块集合。
   */
  struct Sub {};
};

/**
 * @brief 校验并输出运行时边界层提供的 CARLA 前车净距离。
 */
class CARLAACCLeadDistance : public FuncModule<CARLAACCLeadDistanceTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
