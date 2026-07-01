#include "CARLAACCDriverCommand.hpp"

extern "C" int carla_adapter_read_driver_command(int *commandType, int *valid);

namespace control {

/**
 * @brief 通过 CARLA 适配器读取并输出驾驶指令。
 * @cn_name CARLA驾驶指令
 * @type atomic
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.2
 * @date 2026-06-16
 * @author ZYK
 */
void CARLAACCDriverCommand::run(const Input& input, Output& output) {
  /**
   * @brief 忽略空输入端口，保持统一 run 签名。
   */
  (void)input;

  /**
   * @brief 调用 CARLA 适配器读取驾驶指令。
   */
  int commandType0 = 0;
  int valid0 = 0;
  const int adapterRc0 = carla_adapter_read_driver_command(&commandType0, &valid0);

  /**
   * @brief 判断适配器调用是否成功。
   */
  const bool adapterOk0 = adapterRc0 == 0;

  /**
   * @brief 判断驾驶指令数据是否有效。
   */
  const bool dataValid0 = valid0 != 0;

  /**
   * @brief 判断驾驶指令是否可用。
   */
  const bool commandUsable0 = adapterOk0 && dataValid0;

  /**
   * @brief 根据适配状态选择驾驶指令或无指令默认值。
   */
  int commandTypeOut0;
  int validOut0;
  if (commandUsable0) {
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
