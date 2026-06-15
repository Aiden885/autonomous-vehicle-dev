#include "CARLAACCEgoSpeed.hpp"

namespace control {

/**
 * @brief 透传运行时边界层提供的自车纵向速度。
 * @cn_name CARLA自车速度
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 ACC最小闭环
 * @version 2.1
 * @date 2026-06-04
 * @author liuruyu
 */
void CARLAACCEgoSpeed::run(const Input& input, Output& output) {
  // 读取结构化输入中的自车速度和有效标志
  const Real egoV0 = input.egoV;
  const int valid0 = input.valid;
  const bool dataValid0 = (valid0 != 0);

  // 无有效数据时输出安全默认速度
  Real egoVOut0;
  int validOut0;
  if (dataValid0) {
    egoVOut0 = egoV0;
    validOut0 = 1;
  } else {
    egoVOut0 = 0.0;
    validOut0 = 0;
  }

  // 顶层输出：写出自车纵向速度和有效标志
  output.egoV = egoVOut0;
  output.valid = validOut0;
}

} // namespace control
