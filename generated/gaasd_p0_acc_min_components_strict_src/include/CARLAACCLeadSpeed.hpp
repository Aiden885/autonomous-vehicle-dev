#pragma once

#include "FuncModule.hpp"

namespace control {

/**
 * @brief CARLAACCLeadSpeed 模块接口结构集合。
 */
struct CARLAACCLeadSpeedTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {
    Real leadV = 0.0;  ///< 运行时边界层提供的前车纵向速度 (m/s)
    int valid = 0;     ///< 输入数据有效标志，1 表示有效
  };

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    Real leadV = 0.0;  ///< 前车纵向速度 (m/s)
    int valid = 0;     ///< 输出数据有效标志，1 表示有效
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
 * @brief 透传运行时边界层提供的 CARLA 前车纵向速度。
 */
class CARLAACCLeadSpeed : public FuncModule<CARLAACCLeadSpeedTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
