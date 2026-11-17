#include "apaMathUtils.h"
#include "geoSlotSelfAdjustTypes.h"
#include <math.h>

/**
 * @brief 车位入口自选调整
 *
 * 对应 MATLAB geo_self_select_slot_adjust.m：
 * - 当 slot_num > 1 时直接返回并保持原值（仅处理单车位场景）
 * - 根据入口编号重新计算车位方向（垂直/平行）和泊车方向（车头/车尾）
 * - 若规划已完成则重置为失败状态以触发重规划
 *
 * MATLAB 原版使用 randi 随机选择入口作为 fallback；
 * C99 版本改为由调用方通过 EntryIdIn 显式指定入口（0 默认为入口 1）。
 *
 * @en_name geoSlotSelfAdjust
 * @cn_name 车位入口自选调整
 * @type module
 * @param[IN] GeoSlotSelfAdjustInput_t Input 车位入口调整输入
 * @param[OUT] GeoSlotSelfAdjustOutput_t Output 车位入口调整输出
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 车位选择
 * @source matlab_rewrite geo_self_select_slot_adjust.m
 * @version 1.0
 * @date 2026-04-28
 * @author aide
 */
void geoSlotSelfAdjust(const GeoSlotSelfAdjustInput_t *input, GeoSlotSelfAdjustOutput_t *output) {
  Integer entryId;
  Integer p1Idx;
  Integer p2Idx;
  Integer p3Idx;
  Real p1x;
  Real p1y;
  Real p2x;
  Real p2y;
  Real p3x;
  Real p3y;
  Real shortSideLen;
  Real longSideLen;
  Real headingCos;
  Real headingSin;
  Real bx;
  Real by;
  Real dotProd;

  if (input->SlotNum > 1) {
    output->SlotStsOut = input->SlotStsIn;
    output->SlotEntr = input->SlotEntrIn;
    output->SlotTrackDir = input->SlotTrackDirIn;
    output->AuaDir = input->AuaDirIn;
  } else {

    entryId = input->EntryIdIn;
    if ((entryId < 1) || (entryId > 4)) {
      entryId = 1;
    }

    p1Idx = entryId;
    p2Idx = (entryId % 4) + 1;
    p3Idx = ((entryId + 1) % 4) + 1;

    p1x = input->SlotPts[2 * (p1Idx - 1)];
    p1y = input->SlotPts[2 * (p1Idx - 1) + 1];
    p2x = input->SlotPts[2 * (p2Idx - 1)];
    p2y = input->SlotPts[2 * (p2Idx - 1) + 1];
    p3x = input->SlotPts[2 * (p3Idx - 1)];
    p3y = input->SlotPts[2 * (p3Idx - 1) + 1];

    shortSideLen = apaMathNorm2D(p1x - p2x, p1y - p2y);
    longSideLen = apaMathNorm2D(p2x - p3x, p2y - p3y);

    if (shortSideLen < longSideLen + 1.0f) {

      output->SlotTrackDir = 1;

      headingCos = cosf(input->TarPosTheta);
      headingSin = sinf(input->TarPosTheta);
      bx = p1x - input->TarPosX;
      by = p1y - input->TarPosY;
      dotProd = headingCos * bx + headingSin * by;

      if (dotProd < 0.0f) {

        output->AuaDir = 1;
      } else {

        output->AuaDir = 2;
      }
    } else {

      output->SlotTrackDir = 2;

      output->AuaDir = input->AuaDirIn;
    }

    output->SlotEntr = entryId;

    if (input->SlotStsIn == 3) {
      output->SlotStsOut = 4;
    } else {
      output->SlotStsOut = input->SlotStsIn;
    }
  }
}
