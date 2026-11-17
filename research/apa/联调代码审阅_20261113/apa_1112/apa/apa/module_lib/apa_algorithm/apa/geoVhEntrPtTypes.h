#ifndef GEO_VH_ENTR_PT_TYPES_H
#define GEO_VH_ENTR_PT_TYPES_H

#include "baseTypes.h"
#include "geoLineCrossPtTypes.h"

/**
 * @brief VH入口点计算输入
 * @en_name GeoVhEntrPtInput
 * @cn_name VH入口点计算输入
 * @tag STRUCT
 * @field name=SltPt1X, type=Real, unit=m, desc=车位角点1X
 * @field name=SltPt1Y, type=Real, unit=m, desc=车位角点1Y
 * @field name=SltPt2X, type=Real, unit=m, desc=车位角点2X
 * @field name=SltPt2Y, type=Real, unit=m, desc=车位角点2Y
 * @field name=SltPt3X, type=Real, unit=m, desc=车位角点3X
 * @field name=SltPt3Y, type=Real, unit=m, desc=车位角点3Y
 * @field name=SltPt4X, type=Real, unit=m, desc=车位角点4X
 * @field name=SltPt4Y, type=Real, unit=m, desc=车位角点4Y
 * @field name=SltEntr, type=Integer, unit=1, desc=车位入口边编号
 * @field name=TgtX, type=Real, unit=m, desc=目标X
 * @field name=TgtY, type=Real, unit=m, desc=目标Y
 * @field name=TgtTheta, type=Real, unit=rad, desc=目标航向
 * @field name=PrkType, type=Integer, unit=1, desc=泊车方向1左2右
 * @field name=SurdType, type=Integer, unit=1, desc=周边类型0无障碍1左开2右开3双侧受限
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real SltPt1X;
  Real SltPt1Y;
  Real SltPt2X;
  Real SltPt2Y;
  Real SltPt3X;
  Real SltPt3Y;
  Real SltPt4X;
  Real SltPt4Y;
  Integer SltEntr;
  Real TgtX;
  Real TgtY;
  Real TgtTheta;
  Integer PrkType;
  Integer SurdType;
} GeoVhEntrPtInput_t;

/**
 * @brief VH入口点计算参数
 * @en_name GeoVhEntrPtParam
 * @cn_name VH入口点计算参数
 * @tag STRUCT
 * @field name=WheelBase, type=Real, unit=m, desc=轴距
 * @field name=LLf, type=Real, unit=m, desc=前保险杠到参考点长度
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real WheelBase;
  Real LLf;
} GeoVhEntrPtParam_t;

/**
 * @brief VH入口点计算输出
 * @en_name GeoVhEntrPtOutput
 * @cn_name VH入口点计算输出
 * @tag STRUCT
 * @field name=EntrX, type=Real, unit=m, desc=入口点X
 * @field name=EntrY, type=Real, unit=m, desc=入口点Y
 * @version 1.0
 * @date 2026-04-13
 * @author aide
 */
typedef struct {
  Real EntrX;
  Real EntrY;
} GeoVhEntrPtOutput_t;

void geoVhEntrPt(const GeoVhEntrPtInput_t *input,
                 const GeoVhEntrPtParam_t *param,
                 GeoVhEntrPtOutput_t *output);

#endif
