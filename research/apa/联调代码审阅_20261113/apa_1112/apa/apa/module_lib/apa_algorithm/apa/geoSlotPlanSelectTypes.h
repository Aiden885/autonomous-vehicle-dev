#ifndef GEO_SLOT_PLAN_SELECT_TYPES_H
#define GEO_SLOT_PLAN_SELECT_TYPES_H

#include "baseTypes.h"
#include "geoAxisTransTypes.h"

/** @brief 算法支持的最大槽位数量 */
#ifndef MAX_SLOT_NUM
#define MAX_SLOT_NUM 20
#endif

/**
 * @brief 车位及规划策略分配选择器输入
 * @en_name GeoSlotPlanSelectInput
 * @cn_name 车位及规划策略分配选择器输入
 * @tag STRUCT
 * @field name=VehPosX, type=Real, unit=m, desc=车辆X坐标
 * @field name=VehPosY, type=Real, unit=m, desc=车辆Y坐标
 * @field name=VehPosTheta, type=Real, unit=rad, desc=车辆航向角
 * @field name=VehSpd, type=Real, unit=m/s, desc=车速
 * @field name=SlotSts, type=Array<Integer, [20]>, unit=1, desc=车位状态数组
 * @field name=SlotSubSts, type=Array<Integer, [20]>, unit=1, desc=车位子状态数组
 * @field name=SlotID, type=Array<Integer, [20]>, unit=1, desc=车位ID数组
 * @field name=SlotNm, type=Integer, unit=1, desc=当前车位总数
 * @field name=SlotTarPtX, type=Array<Real, [20]>, unit=m, desc=各车位目标位置X
 * @field name=SlotTarPtY, type=Array<Real, [20]>, unit=m, desc=各车位目标位置Y
 * @field name=SlotTarPtTheta, type=Array<Real, [20]>, unit=rad, desc=各车位目标位置Theta
 * @field name=SlotDir, type=Array<Integer, [20]>, unit=1, desc=各车位方向
 * @field name=FiltSw, type=Integer, unit=1, desc=过滤开关标志
 * @field name=FiltN, type=Integer, unit=1, desc=过滤限制数量
 * @field name=ApaSrchSts, type=Integer, unit=1, desc=APA系统主状态
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Real VehSpd;
  Integer SlotSts[MAX_SLOT_NUM];
  Integer SlotSubSts[MAX_SLOT_NUM];
  Integer SlotID[MAX_SLOT_NUM];
  Integer SlotNm;
  Real SlotTarPtX[MAX_SLOT_NUM];
  Real SlotTarPtY[MAX_SLOT_NUM];
  Real SlotTarPtTheta[MAX_SLOT_NUM];
  Integer SlotDir[MAX_SLOT_NUM];
  Integer FiltSw;
  Integer FiltN;
  Integer ApaSrchSts;
} GeoSlotPlanSelectInput_t;

/**
 * @brief 车位及规划策略分配选择器输出
 * @en_name GeoSlotPlanSelectOutput
 * @cn_name 车位及规划策略分配选择器输出
 * @tag STRUCT
 * @field name=IdList, type=Array<Integer, [20]>, unit=1, desc=筛选出的车位ID列表
 * @field name=IdNum, type=Integer, unit=1, desc=筛选出的ID数量
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Integer IdList[MAX_SLOT_NUM];
  Integer IdNum;
} GeoSlotPlanSelectOutput_t;

void geoSlotPlanSelect(const GeoSlotPlanSelectInput_t *input, GeoSlotPlanSelectOutput_t *output);

#endif
