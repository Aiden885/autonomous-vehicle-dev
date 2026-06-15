#include "CARLAObjectList.hpp"

namespace control {

/**
 * @brief 通过 CARLA 适配器读取障碍物列表并打包为定长数组输出。
 * @cn_name CARLA障碍物列表
 * @type block
 * @tag_level0 功能模块库
 * @tag_level1 CARLA联合仿真
 * @tag_level2 仿真输入
 * @version 2.0
 * @date 2026-06-03
 * @author liuruyu
 *
 * 注意：本模块为边界适配，读取的是数组型 C ABI 接口。受 C ABI 约束，
 * run() 内需使用定长缓冲数组与传出指针，并循环打包到 ObjectArray，
 * 与 BODY-006/BODY-022 等纯建模约束存在已知例外（详见校验报告）。
 */
void CARLAObjectList::run(const Input& input, Output& output) {
  // 限制请求数量不超过容器容量
  int maxObjects0;
  if (input.maxObjects > kMaxObjects) {
    maxObjects0 = kMaxObjects;
  } else {
    maxObjects0 = input.maxObjects;
  }

  // 边界适配缓冲：接收适配器写入的各字段数组（C ABI 要求连续存储）
  std::array<int, kMaxObjects> objectId{};
  std::array<int, kMaxObjects> objectType{};
  std::array<double, kMaxObjects> objectX{};
  std::array<double, kMaxObjects> objectY{};
  std::array<double, kMaxObjects> objectYawRad{};
  std::array<double, kMaxObjects> objectV{};
  std::array<double, kMaxObjects> objectVx{};
  std::array<double, kMaxObjects> objectVy{};
  std::array<double, kMaxObjects> objectLength{};
  std::array<double, kMaxObjects> objectWidth{};
  std::array<double, kMaxObjects> objectHeight{};
  int objectCount0 = 0;
  int valid0 = 0;

  // 边界适配调用：读取 CARLA 障碍物列表（C ABI，数组传出指针）
  int rc0 = carla_adapter_read_object_list(
      maxObjects0,
      &objectCount0,
      objectId.data(),
      objectType.data(),
      objectX.data(),
      objectY.data(),
      objectYawRad.data(),
      objectV.data(),
      objectVx.data(),
      objectVy.data(),
      objectLength.data(),
      objectWidth.data(),
      objectHeight.data(),
      &valid0);

  // 读取失败时输出空列表
  int objectCountOut0 = 0;
  int validOut0 = 0;
  if (rc0 == 0) {
    objectCountOut0 = objectCount0;
    validOut0 = valid0;
  } else {
    objectCountOut0 = 0;
    validOut0 = 0;
  }

  // 限制有效数量不超过容器容量
  int countClamped0;
  if (objectCountOut0 > kMaxObjects) {
    countClamped0 = kMaxObjects;
  } else {
    countClamped0 = objectCountOut0;
  }

  // 打包：将各字段数组按下标合并到障碍物目标数组
  for (int i = 0; i < countClamped0; ++i) {
    output.objects[i].id = objectId[i];
    output.objects[i].type = objectType[i];
    output.objects[i].x = objectX[i];
    output.objects[i].y = objectY[i];
    output.objects[i].yawRad = objectYawRad[i];
    output.objects[i].v = objectV[i];
    output.objects[i].vx = objectVx[i];
    output.objects[i].vy = objectVy[i];
    output.objects[i].length = objectLength[i];
    output.objects[i].width = objectWidth[i];
    output.objects[i].height = objectHeight[i];
  }

  // 顶层输出：写出有效目标数量与有效标志
  output.objectCount = countClamped0;
  output.valid = validOut0;
}

} // namespace control
