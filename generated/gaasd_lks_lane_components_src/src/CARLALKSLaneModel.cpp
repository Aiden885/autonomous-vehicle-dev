#include "CARLALKSLaneModel.hpp"

namespace control {

/**
 * @brief 根据边界适配状态选择 CARLA 车道多项式模型输出。
 * @cn_name CARLA车道模型
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 LKS最小闭环
 * @version 2.1
 * @date 2026-06-17
 * @author liuruyu
 */
void CARLALKSLaneModel::run(const Input& input, Output& output) {
  /**
   * @brief 锁存边界适配传入的车道多项式系数、曲率与有效性标志。
   */
  const Real c0In0 = input.c0;
  const Real c1In0 = input.c1;
  const Real c2In0 = input.c2;
  const Real c3In0 = input.c3;
  const Real curvatureIn0 = input.curvature;
  const int adapterRc0 = input.adapterRc;
  const int valid0 = input.valid;

  /**
   * @brief 锁存感知失效或离线测试时使用的回退默认值。
   */
  const Real defaultC00 = param_.defaultC0;
  const Real defaultC10 = param_.defaultC1;
  const Real defaultC20 = param_.defaultC2;
  const Real defaultC30 = param_.defaultC3;
  const Real defaultCurvature0 = param_.defaultCurvature;

  /**
   * @brief 判断车道检测数据是否可用：边界返回码成功且有效标志非零。
   */
  const bool laneValid0 = (adapterRc0 == 0) && (valid0 != 0);

  /**
   * @brief 数据有效则输出适配器实测车道模型，否则输出回退默认值。
   */
  Real c0Out0;
  Real c1Out0;
  Real c2Out0;
  Real c3Out0;
  Real curvatureOut0;
  if (laneValid0) {
    c0Out0 = c0In0;
    c1Out0 = c1In0;
    c2Out0 = c2In0;
    c3Out0 = c3In0;
    curvatureOut0 = curvatureIn0;
  } else {
    c0Out0 = defaultC00;
    c1Out0 = defaultC10;
    c2Out0 = defaultC20;
    c3Out0 = defaultC30;
    curvatureOut0 = defaultCurvature0;
  }

  /**
   * @brief 顶层输出有效保护后的车道多项式系数与道路曲率。
   */
  output.c0 = c0Out0;
  output.c1 = c1Out0;
  output.c2 = c2Out0;
  output.c3 = c3Out0;
  output.curvature = curvatureOut0;
}

} // namespace control
