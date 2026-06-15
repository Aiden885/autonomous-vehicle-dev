#pragma once

#include "FuncModule.hpp"
#include "CARLAObjectListTypes.hpp"

namespace control {

// CARLA 适配器边界接口（项目自有适配库，非第三方开源依赖），C ABI，数组出参。
extern "C" int carla_adapter_read_object_list(
    int maxObjects,
    int* objectCount,
    int* objectId,
    int* objectType,
    double* objectX,
    double* objectY,
    double* objectYawRad,
    double* objectV,
    double* objectVx,
    double* objectVy,
    double* objectLength,
    double* objectWidth,
    double* objectHeight,
    int* valid);

/**
 * @brief CARLAObjectList 模块接口结构集合。
 */
struct CARLAObjectListTraits {
  /**
   * @brief 输入端口集合。
   */
  struct Input {
    int maxObjects = kMaxObjects;  ///< 最大输出目标数量
  };

  /**
   * @brief 输出端口集合。
   */
  struct Output {
    ObjectArray objects{};  ///< 障碍物目标定长数组（前 objectCount 个有效）
    int objectCount = 0;    ///< 实际输出目标数量
    int valid = 0;          ///< 数据有效标志，1 表示有效
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
 * @brief 读取 CARLA 障碍物列表的边界适配函数模块。
 */
class CARLAObjectList : public FuncModule<CARLAObjectListTraits> {
public:
  using FuncModule::FuncModule;

  void run(const Input& input, Output& output) override;
};

} // namespace control
