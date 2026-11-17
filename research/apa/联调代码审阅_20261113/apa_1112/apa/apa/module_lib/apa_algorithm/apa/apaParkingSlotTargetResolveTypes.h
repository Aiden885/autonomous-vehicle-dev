#ifndef APA_PARKING_SLOT_TARGET_RESOLVE_TYPES_H
#define APA_PARKING_SLOT_TARGET_RESOLVE_TYPES_H

#include "baseTypes.h"
#include "apaEnums.h"

/* Parser status is local to the new slot-request API, not an existing fault enum. */
typedef enum {
  APA_SLOT_TARGET_OK = 0,
  APA_SLOT_TARGET_UNSUPPORTED = 1,
  APA_SLOT_TARGET_INVALID_ENTRY = 2,
  APA_SLOT_TARGET_INVALID_GEOMETRY = 3,
  APA_SLOT_TARGET_INVALID_MODEL_POLICY = 4,
  APA_SLOT_TARGET_NO_LEGAL_POSE = 5,
  APA_SLOT_TARGET_INVALID_REQUEST = 6
} ApaParkingSlotTargetStatus_e;

/**
 * @brief Shared geometry and explicit rear-axle goal inputs for all parking families.
 * @en_name ApaParkingSlotTargetResolveInput
 * @cn_name 车位目标解析输入
 * @tag STRUCT
 * @field name=SlotKind, type=Integer, unit=1, desc=规划族1VT2HT3DTH4VH或泊出5
 * @field name=SlotPts, type=Array<Real, [8]>, unit=m, desc=有序四角点
 * @field name=EnterEdge, type=Integer, unit=1, desc=入口边1至4
 * @field name=VehFront, type=Real, unit=m, desc=后轴到前包络
 * @field name=VehRear, type=Real, unit=m, desc=后轴到后包络
 * @field name=VehWidth, type=Real, unit=m, desc=车辆宽度
 * @field name=ParkingAction, type=Integer, unit=1, desc=1泊入2泊出零保留旧解析调用的泊入语义
 * @field name=RequestedTargetX, type=Real, unit=m, desc=输入的后轴目标X
 * @field name=RequestedTargetY, type=Real, unit=m, desc=输入的后轴目标Y
 * @field name=RequestedTargetTheta, type=Real, unit=rad, desc=输入的目标航向
 */
typedef struct {
  Integer SlotKind;
  Real SlotPts[8];
  Integer EnterEdge;
  Real VehFront;
  Real VehRear;
  Real VehWidth;
  Integer ParkingAction;
  Real RequestedTargetX;
  Real RequestedTargetY;
  Real RequestedTargetTheta;
} ApaParkingSlotTargetResolveInput_t;

/**
 * @brief Slot geometry and diagnostics for the requested rear-axle target.
 * @en_name ApaParkingSlotTargetResolveOutput
 * @cn_name 车位目标解析输出
 * @tag STRUCT
 * @field name=Valid, type=Boolean, unit=1, desc=输入与车位几何校验通过
 * @field name=Status, type=Integer, unit=1, desc=本模块解析状态
 * @field name=CenterX, type=Real, unit=m, desc=四角平均中心X
 * @field name=CenterY, type=Real, unit=m, desc=四角平均中心Y
 * @field name=EntryX, type=Real, unit=m, desc=入口中点X
 * @field name=EntryY, type=Real, unit=m, desc=入口中点Y
 * @field name=BackX, type=Real, unit=m, desc=库底中点X
 * @field name=BackY, type=Real, unit=m, desc=库底中点Y
 * @field name=Width, type=Real, unit=m, desc=入口与库底边长度较小值
 * @field name=Depth, type=Real, unit=m, desc=入口到库底中点距离
 * @field name=HeadingToEntry, type=Real, unit=rad, desc=库底朝入口方向
 * @field name=BackClearance, type=Real, unit=m, desc=泊入车身至库底的轴向留空仅供诊断
 * @field name=EntryOverhang, type=Real, unit=m, desc=泊入车身的入口外伸量仅供诊断
 */
typedef struct {
  Boolean Valid;
  Integer Status;
  Real CenterX;
  Real CenterY;
  Real EntryX;
  Real EntryY;
  Real BackX;
  Real BackY;
  Real Width;
  Real Depth;
  Real HeadingToEntry;
  Real BackClearance;
  Real EntryOverhang;
} ApaParkingSlotTargetResolveOutput_t;

#endif
