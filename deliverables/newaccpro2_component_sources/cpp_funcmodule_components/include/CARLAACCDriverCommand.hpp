#pragma once

#include "FuncModule.hpp"

namespace control {

/**
 * @brief CARLAACCDriverCommand 模块接口结构集合。
 */
struct CARLAACCDriverCommandTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {
    int commandType = 0; ///< 运行时边界层提供的驾驶指令类型，0 表示无指令
    int valid = 0;       ///< 驾驶指令有效标志，1 表示有效
  };

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    int commandType = 0; ///< 驾驶指令类型，0 表示无指令
    int valid = 0;       ///< 驾驶指令输出有效标志，1 表示有效
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
 * @brief 校验并输出运行时边界层提供的 CARLA 驾驶指令。
 */
class CARLAACCDriverCommand : public FuncModule<CARLAACCDriverCommandTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
