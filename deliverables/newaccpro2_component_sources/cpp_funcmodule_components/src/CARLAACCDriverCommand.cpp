#include "CARLAACCDriverCommand.hpp"

namespace control {

/**
 * @brief 校验并输出运行时边界层提供的驾驶指令。
 * @cn_name CARLA驾驶指令
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.1
 * @date 2026-06-12
 * @author ZYK
 */
void CARLAACCDriverCommand::run(const Input& input, Output& output) {
  /**
   * @brief 锁存运行时边界层提供的驾驶指令和有效标志。
   */
  const int commandType0 = input.commandType;
  const int valid0 = input.valid;

  /**
   * @brief 判断驾驶指令数据是否有效。
   */
  const bool dataValid0 = valid0 != 0;

  /**
   * @brief 根据有效标志选择驾驶指令或无指令默认值。
   */
  int commandTypeOut0;
  int validOut0;
  if (dataValid0) {
    commandTypeOut0 = commandType0;
    validOut0 = 1;
  } else {
    commandTypeOut0 = 0;
    validOut0 = 0;
  }

  /**
   * @brief 输出校验后的驾驶指令类型。
   */
  output.commandType = commandTypeOut0;

  /**
   * @brief 输出驾驶指令有效标志。
   */
  output.valid = validOut0;
}

} // namespace control
