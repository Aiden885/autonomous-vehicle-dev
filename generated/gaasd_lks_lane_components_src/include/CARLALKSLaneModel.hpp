#pragma once

#include "FuncModule.hpp"

namespace control {

/**
 * @brief CARLALKSLaneModel 模块接口结构集合。
 */
struct CARLALKSLaneModelTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {
    Real c0 = 0.0;         ///< 适配器读取的车道中心线常数项（当前横向偏差，m）
    Real c1 = 0.0;         ///< 适配器读取的车道中心线一阶项
    Real c2 = 0.0;         ///< 适配器读取的车道中心线二阶项 (1/m)
    Real c3 = 0.0;         ///< 适配器读取的车道中心线三阶项 (1/m^2)
    Real curvature = 0.0;  ///< 适配器读取的道路曲率 (1/m)
    int adapterRc = 0;     ///< 边界适配返回码，0 表示读取成功
    int valid = 0;         ///< 车道检测有效标志，非零表示有效
  };

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    Real c0 = 0.0;         ///< 有效保护后的车道常数项
    Real c1 = 0.0;         ///< 有效保护后的车道一阶项
    Real c2 = 0.0;         ///< 有效保护后的车道二阶项
    Real c3 = 0.0;         ///< 有效保护后的车道三阶项
    Real curvature = 0.0;  ///< 有效保护后的道路曲率
  };

  /**
   * @brief 参数集合。
   */
  struct Param {
    Real defaultC0 = 0.8;         ///< 感知失效/离线回退：车道常数项默认值
    Real defaultC1 = 0.0;         ///< 感知失效/离线回退：一阶项默认值
    Real defaultC2 = 0.0;         ///< 感知失效/离线回退：二阶项默认值
    Real defaultC3 = 0.0;         ///< 感知失效/离线回退：三阶项默认值
    Real defaultCurvature = 0.0;  ///< 感知失效/离线回退：曲率默认值
  };

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
 * @brief 选择 CARLA 车道多项式模型有效值的函数模块。
 */
class CARLALKSLaneModel : public FuncModule<CARLALKSLaneModelTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
