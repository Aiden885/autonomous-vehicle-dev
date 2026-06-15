#include "CARLAACCLeadSpeed.hpp"

namespace control {

/**
 * @brief 校验并输出运行时边界层提供的前车纵向速度。
 * @cn_name CARLA前车速度
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.1
 * @date 2026-06-12
 * @author liuruyu
 */
void CARLAACCLeadSpeed::run(const Input& input, Output& output) {
  /**
   * @brief 锁存运行时边界层提供的前车速度和有效标志。
   */
  const Real leadV0 = input.leadV;
  const int valid0 = input.valid;

  /**
   * @brief 判断前车速度数据是否有效。
   */
  const bool dataValid0 = valid0 != 0;

  /**
   * @brief 根据有效标志选择真实速度或安全默认值。
   */
  Real leadVOut0;
  int validOut0;
  if (dataValid0) {
    leadVOut0 = leadV0;
    validOut0 = 1;
  } else {
    leadVOut0 = Real{0.0};
    validOut0 = 0;
  }

  /**
   * @brief 输出校验后的前车纵向速度。
   */
  output.leadV = leadVOut0;

  /**
   * @brief 输出前车速度有效标志。
   */
  output.valid = validOut0;
}

} // namespace control
