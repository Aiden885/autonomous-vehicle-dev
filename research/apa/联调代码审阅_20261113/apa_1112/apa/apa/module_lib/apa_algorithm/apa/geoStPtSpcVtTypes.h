#ifndef GEO_ST_PT_SPC_VT_TYPES_H
#define GEO_ST_PT_SPC_VT_TYPES_H

#include "baseTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include "apaPlanningScene.h"

/**
 * @brief VT起点横向空间检测输入
 * @en_name GeoStPtSpcVtInput
 * @cn_name VT起点横向空间检测输入
 * @tag STRUCT
 * @field name=VehPosX, type=Real, unit=m, desc=车辆位置X
 * @field name=VehPosY, type=Real, unit=m, desc=车辆位置Y
 * @field name=VehPosTheta, type=Real, unit=rad, desc=车辆航向角
 * @field name=TarPtX, type=Real, unit=m, desc=目标点X
 * @field name=TarPtY, type=Real, unit=m, desc=目标点Y
 * @field name=TarPtTheta, type=Real, unit=rad, desc=目标点航向角
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SltEntr, type=Integer, unit=1, desc=车位入口边编号
 * @field name=CheckLen, type=Real, unit=m, desc=前向检查距离
 * @field name=SltType, type=Integer, unit=1, desc=车位类型
 * @field name=FallbackSpace, type=Real, unit=m, desc=最大横向空间回退值
 * @field name=UseCollisionCheck, type=Boolean, unit=1, desc=是否走逐点碰撞扫描
 * @field name=VccTemplate, type=VehicleCollisionCheckInput_t, unit=1, desc=碰撞检测模板含障碍输入
 * @version 2.0
 * @date 2026-05-01
 * @author aide
 */
typedef struct {
  Real VehPosX;
  Real VehPosY;
  Real VehPosTheta;
  Real TarPtX;
  Real TarPtY;
  Real TarPtTheta;
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer SltEntr;
  Real CheckLen;
  Integer SltType;
  Real FallbackSpace;
  Boolean UseCollisionCheck;
  VehicleCollisionCheckInput_t VccTemplate;
} GeoStPtSpcVtInput_t;

/**
 * @brief VT起点横向空间检测输出
 * @en_name GeoStPtSpcVtOutput
 * @cn_name VT起点横向空间检测输出
 * @tag STRUCT
 * @field name=LeftSpace, type=Real, unit=m, desc=左侧可用空间
 * @field name=RightSpace, type=Real, unit=m, desc=右侧可用空间
 * @field name=CheckCount, type=Integer, unit=1, desc=检查次数
 * @field name=IntrvRs, type=Integer, unit=1, desc=干涉原因
 * @field name=CrashPosX, type=Real, unit=m, desc=碰撞位置X
 * @field name=CrashPosY, type=Real, unit=m, desc=碰撞位置Y
 * @field name=CrashPosTheta, type=Real, unit=rad, desc=碰撞位置航向
 * @field name=CorridorValid, type=Integer, unit=1, desc=是否进入槽位走廊或扫描成功
 * @version 2.0
 * @date 2026-05-01
 * @author aide
 */
typedef struct {
  Real LeftSpace;
  Real RightSpace;
  Integer CheckCount;
  Integer IntrvRs;
  Real CrashPosX;
  Real CrashPosY;
  Real CrashPosTheta;
  Integer CorridorValid;
} GeoStPtSpcVtOutput_t;

void geoStPtSpcVt(const GeoStPtSpcVtInput_t *input, GeoStPtSpcVtOutput_t *output);

void geoStPtSpcVtWithScene(const GeoStPtSpcVtInput_t *input,
                           const ApaPlanningSceneQuery_t *scene,
                           GeoStPtSpcVtOutput_t *output);

#endif
