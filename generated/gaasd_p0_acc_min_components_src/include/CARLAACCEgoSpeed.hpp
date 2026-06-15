#pragma once

#include "FuncModule.hpp"

namespace control {

/**
 * @brief CARLAACCEgoSpeed 模块接口结构集合。
 */
struct CARLAACCEgoSpeedTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {
    Real egoV = 0.0; ///< 运行时边界层提供的自车纵向速度 (m/s)
    int valid = 0;   ///< 自车状态有效标志，1 表示有效
  };

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    Real egoV = 0.0; ///< 自车纵向速度 (m/s)
    int valid = 0;   ///< 自车速度有效标志，1 表示有效
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
 * @brief 校验并输出运行时边界层提供的 CARLA 自车纵向速度。
 */
class CARLAACCEgoSpeed : public FuncModule<CARLAACCEgoSpeedTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
