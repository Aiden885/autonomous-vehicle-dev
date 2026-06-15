#include "CARLAACCLeadSpeed.hpp"

namespace control {

/**
 * @brief 透传运行时边界层提供的前车纵向速度。
 * @cn_name CARLA前车速度
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.1
 * @date 2026-06-04
 * @author liuruyu
 */
void CARLAACCLeadSpeed::run(const Input& input, Output& output) {
  // 读取结构化输入中的前车速度和有效标志
  const Real leadV0 = input.leadV;
  const int valid0 = input.valid;
  const bool dataValid0 = (valid0 != 0);

  // 无有效数据时输出安全默认前车速度
  Real leadVOut0;
  int validOut0;
  if (dataValid0) {
    leadVOut0 = leadV0;
    validOut0 = 1;
  } else {
    leadVOut0 = 0.0;
    validOut0 = 0;
  }

  // 顶层输出：写出前车纵向速度和有效标志
  output.leadV = leadVOut0;
  output.valid = validOut0;
}

} // namespace control
