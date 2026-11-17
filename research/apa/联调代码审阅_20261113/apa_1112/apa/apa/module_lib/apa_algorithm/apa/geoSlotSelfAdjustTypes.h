#ifndef GEO_SLOT_SELF_ADJUST_TYPES_H
#define GEO_SLOT_SELF_ADJUST_TYPES_H

#include "baseTypes.h"

/**
 * @brief 车位入口自选调整输入
 * @en_name GeoSlotSelfAdjustInput
 * @cn_name 车位入口自选调整输入
 * @tag STRUCT
 * @field name=SlotNum, type=Integer, unit=1, desc=车位总数
 * @field name=TargetSlotIdx, type=Integer, unit=1, desc=目标车位索引(0-based)
 * @field name=SlotPts, type=Array<Real, [8]>, unit=m, desc=车位四角点[P1x,P1y,P2x,P2y,P3x,P3y,P4x,P4y]
 * @field name=TarPosX, type=Real, unit=m, desc=目标位置X
 * @field name=TarPosY, type=Real, unit=m, desc=目标位置Y
 * @field name=TarPosTheta, type=Real, unit=rad, desc=目标航向角
 * @field name=SlotStsIn, type=Integer, unit=1, desc=当前车位状态
 * @field name=EntryIdIn, type=Integer, unit=1, desc=指定入口编号(1-4)，0表示默认入口1
 * @field name=SlotEntrIn, type=Integer, unit=1, desc=当前入口编号，多车位早退时原值透传
 * @field name=SlotTrackDirIn, type=Integer, unit=1, desc=当前车位方向，多车位早退时原值透传
 * @field name=AuaDirIn, type=Integer, unit=1, desc=当前泊车方向，多车位早退时原值透传
 * @version 1.0
 * @date 2026-04-28
 * @author aide
 */
typedef struct {
  Integer SlotNum;
  Integer TargetSlotIdx;
  Real SlotPts[8];
  Real TarPosX;
  Real TarPosY;
  Real TarPosTheta;
  Integer SlotStsIn;
  Integer EntryIdIn;
  Integer SlotEntrIn;
  Integer SlotTrackDirIn;
  Integer AuaDirIn;
} GeoSlotSelfAdjustInput_t;

/**
 * @brief 车位入口自选调整输出
 * @en_name GeoSlotSelfAdjustOutput
 * @cn_name 车位入口自选调整输出
 * @tag STRUCT
 * @field name=SlotStsOut, type=Integer, unit=1, desc=调整后车位状态
 * @field name=SlotEntr, type=Integer, unit=1, desc=选定的入口编号(1-4)
 * @field name=SlotTrackDir, type=Integer, unit=1, desc=车位方向: 1=垂直, 2=平行
 * @field name=AuaDir, type=Integer, unit=1, desc=泊车方向: 1=车头泊入, 2=车尾泊入
 * @version 1.0
 * @date 2026-04-28
 * @author aide
 */
typedef struct {
  Integer SlotStsOut;
  Integer SlotEntr;
  Integer SlotTrackDir;
  Integer AuaDir;
} GeoSlotSelfAdjustOutput_t;

void geoSlotSelfAdjust(const GeoSlotSelfAdjustInput_t *input, GeoSlotSelfAdjustOutput_t *output);

#endif
