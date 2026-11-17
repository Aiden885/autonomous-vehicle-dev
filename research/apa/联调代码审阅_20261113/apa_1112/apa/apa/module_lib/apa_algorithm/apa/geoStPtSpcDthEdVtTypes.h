/**
 * @file geoStPtSpcDthEdVtTypes.h
 * @brief 斜边垂直泊车预留空间分析输入输出类型定义
 * @date 2026-04-13
 */
#ifndef GEO_ST_PT_SPC_DTH_ED_VT_TYPES_H
#define GEO_ST_PT_SPC_DTH_ED_VT_TYPES_H

#include "baseTypes.h"
#include "vehicleCollisionCheckTypes.h"
#include "apaPlanningScene.h"

/**
 * @brief 斜边垂直空间分析输入
 * @en_name GeoStPtSpcDthEdVtInput
 * @cn_name 斜边垂直空间分析输入
 * @tag STRUCT
 * @field name=StPtX, type=Real, unit=m, desc=起点X
 * @field name=StPtY, type=Real, unit=m, desc=起点Y
 * @field name=StPtTheta, type=Real, unit=rad, desc=起点航向
 * @field name=TarPtX, type=Real, unit=m, desc=目标点X
 * @field name=TarPtY, type=Real, unit=m, desc=目标点Y
 * @field name=TarPtTheta, type=Real, unit=rad, desc=目标点航向
 * @field name=SeparateStopperNum, type=Integer, unit=1, desc=离散障碍物数量
 * @field name=SeparateStopperX, type=Array<Real, [128]>, unit=m, desc=离散障碍物X
 * @field name=SeparateStopperY, type=Array<Real, [128]>, unit=m, desc=离散障碍物Y
 * @field name=UseVccTemplate, type=Boolean, unit=1, desc=是否复用完整碰撞输入模板
 * @field name=VccTemplate, type=VehicleCollisionCheckInput_t, unit=1, desc=完整障碍物碰撞输入模板
 * @field name=SlotPointsX, type=Array<Real, [4]>, unit=m, desc=车位四个角点X
 * @field name=SlotPointsY, type=Array<Real, [4]>, unit=m, desc=车位四个角点Y
 * @field name=VehLF, type=Real, unit=m, desc=前悬+轴距
 * @field name=VehLB, type=Real, unit=m, desc=后悬
 * @field name=VehW, type=Real, unit=m, desc=车宽
 */
typedef struct {
  Real StPtX;
  Real StPtY;
  Real StPtTheta;
  Real TarPtX;
  Real TarPtY;
  Real TarPtTheta;
  Integer SeparateStopperNum;
  Real SeparateStopperX[VCC_MAX_SEPARATE_PTS];
  Real SeparateStopperY[VCC_MAX_SEPARATE_PTS];
  Boolean UseVccTemplate;
  VehicleCollisionCheckInput_t VccTemplate;
  Real SlotPointsX[4];
  Real SlotPointsY[4];
  Real VehLF;
  Real VehLB;
  Real VehW;
} GeoStPtSpcDthEdVtInput_t;

/**
 * @brief 斜边垂直空间分析输出
 * @en_name GeoStPtSpcDthEdVtOutput
 * @cn_name 斜边垂直空间分析输出
 * @tag STRUCT
 * @field name=FtSpc, type=Real, unit=m, desc=前方安全距离
 * @field name=LtSpc, type=Real, unit=m, desc=左侧安全距离
 * @field name=RtSpc, type=Real, unit=m, desc=右侧安全距离
 * @field name=ChkCnt, type=Integer, unit=1, desc=碰撞检测总次数
 * @field name=IntrvRS, type=Integer, unit=1, desc=最后一次碰撞类型
 * @field name=CrshPosX, type=Real, unit=m, desc=碰撞点X
 * @field name=CrshPosY, type=Real, unit=m, desc=碰撞点Y
 * @field name=CrshPosTheta, type=Real, unit=rad, desc=碰撞点航向
 * @field name=ChckPosCX, type=Real, unit=m, desc=前向推算终点X
 * @field name=ChckPosCY, type=Real, unit=m, desc=前向推算终点Y
 * @field name=ChckPosCTheta, type=Real, unit=rad, desc=前向推算终点航向
 */
typedef struct {
  Real FtSpc;
  Real LtSpc;
  Real RtSpc;
  Integer ChkCnt;
  Integer IntrvRS;
  Real CrshPosX;
  Real CrshPosY;
  Real CrshPosTheta;
  Real ChckPosCX;
  Real ChckPosCY;
  Real ChckPosCTheta;
} GeoStPtSpcDthEdVtOutput_t;

void geoStPtSpcDthEdVt(const GeoStPtSpcDthEdVtInput_t *input, GeoStPtSpcDthEdVtOutput_t *output);

void geoStPtSpcDthEdVtWithScene(const GeoStPtSpcDthEdVtInput_t *input,
                                const ApaPlanningSceneQuery_t *scene,
                                GeoStPtSpcDthEdVtOutput_t *output);

#endif
