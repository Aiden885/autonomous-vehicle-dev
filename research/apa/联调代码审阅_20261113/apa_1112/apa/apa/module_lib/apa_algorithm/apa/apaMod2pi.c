#include "apaCoordUtils.h"
#include "apaMod2piTypes.h"

/**
 * @brief 将任意弧度角按 MATLAB single/double 混合精度语义归一化到负π到π范围内
 * @en_name apaMod2pi
 * @cn_name 角度归一化
 * @type module
 * @param[IN] Real Angle 输入角度值（弧度）
 * @param[OUT] Real Result 归一化后的角度值（弧度）
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 基础工具
 * @formula \f[
 * v = \text{rem}(x, 2\pi), \quad v \in [-\pi, \pi]
 * \f]
 * @source matlab_rewrite mod2pi.m
 * @version 1.0
 * @date 2026-03-04
 * @author aide
 */
void apaMod2pi(const ApaMod2piInput_t *input, ApaMod2piOutput_t *output) {
  output->Result = apaCoordWrapToPi(input->Angle);
}
